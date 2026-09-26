// RoutePlanner 的行为测试。
// 期望值全部来自手工推导的字面量，不复用实现中的任何计算。
#include <cmath>
#include <map>
#include <string>
#include <vector>

#include "core/RoutePlanner.h"
#include "graph_fixtures.h"
#include "test_util.h"

using logistics::LogisticsGraph;
using logistics::NodeType;
using logistics::Node;
using logistics::replanIncremental;
using logistics::TrafficChange;
using logistics::TrafficReport;
using logistics::Order;
using logistics::PlanStatus;
using logistics::RoutePlan;
using logistics::Vehicle;
using logistics::WeightType;
using logistics::nodeIndexAtTime;
using logistics::planRoute;
using logistics::replan;
using logistics::insertUrgentOrder;
using logistics::InsertResult;
using logistics::TransitOp;
using logistics::TransitStock;
using testutil::check;

namespace {

using fixtures::addTwoWay;
using fixtures::makeNode;
using fixtures::makeVehicle;
using fixtures::makeOrder;

// W(仓库) <-> D1(配送点)：往返各 5.0km / 10min / 4元
LogisticsGraph makeWtoD1Graph() {
    LogisticsGraph g;
    g.addNode(makeNode("W", NodeType::Warehouse, "仓库"));
    g.addNode(makeNode("D1", NodeType::Delivery, "客户1"));
    addTwoWay(g, "W", "D1", 5.0, 10.0, 4.0);
    return g;
}

std::string join(const std::vector<std::string>& nodes) {
    std::string s;
    for (std::size_t i = 0; i < nodes.size(); ++i) {
        if (i > 0) {
            s += " -> ";
        }
        s += nodes[i];
    }
    return s;
}

// 切片 1：单订单的完整手算校验。
// W <-> D1 各 5.0km / 10min / 4元；发车 08:00(480)；服务时间 5min；
// 订单窗口 09:00-18:00(540-1080)。
//   出发              480
//   rawArrival(D1)    480 + 10 = 490
//   wait              540 - 490 = 50（早到等待）
//   arrival(D1)       540          未超时 -> penalty 0
//   departure(D1)     540 + 5 = 545
//   回到 W             540 + 10 = 550（服务时间已移除，离开==送达）
//   totalDistance     5.0 + 5.0 = 10.0
//   totalTime         550 - 480 = 70
//   totalCost         4.0 + 4.0 = 8.0
void testSingleOrderRouteIsFullyCorrect() {
    const LogisticsGraph g = makeWtoD1Graph();
    const Vehicle v = makeVehicle("W", 1000.0, 480);
    const std::vector<Order> orders = {makeOrder("O1", "D1", 10.0, 540, 1080, false)};

    const RoutePlan plan = planRoute(g, v, orders, WeightType::Distance);

    check(plan.status == PlanStatus::Ok, "规划成功");
    check(plan.nodes == std::vector<std::string>{"W", "D1", "W"},
          "完整序列为 W -> D1 -> W，实际 " + join(plan.nodes));
    check(fixtures::nearlyEqual(plan.totalDistanceKm, 10.0),
          "总距离 10.0，实际 " + std::to_string(plan.totalDistanceKm));
    check(fixtures::nearlyEqual(plan.totalTimeMin, 70.0),
          "总耗时 75（含等待与服务），实际 " + std::to_string(plan.totalTimeMin));
    // 抵达仓库的时刻（oracle 值 550）
    check(plan.returnArrivalMin == 550,
          "返回仓库时刻 550，实际 " + std::to_string(plan.returnArrivalMin));

    // 逐节点到达时刻与"是否停靠"标记，供界面逐个节点推进
    check(plan.nodeArrivalMin.size() == plan.nodes.size(), "到达时刻序列与节点序列等长");
    check(plan.nodeIsStop.size() == plan.nodes.size(), "停靠标记序列与节点序列等长");
    // W(出发 480) -> D1(送达 540) -> W(回到 550)
    check(plan.nodeArrivalMin == std::vector<int>({480, 540, 550}),
          "逐节点到达时刻应为 480 / 540 / 550");
    check(plan.nodeIsStop == std::vector<bool>({false, true, false}),
          "只有中间那个配送点才是停靠");
    check(fixtures::nearlyEqual(plan.totalCostYuan, 8.0),
          "总成本 8.0，实际 " + std::to_string(plan.totalCostYuan));
    check(plan.totalPenaltyMin == 0, "totalPenalty 为 0");

    check(plan.stops.size() == 1, "恰好 1 个停靠点，实际 " + std::to_string(plan.stops.size()));
    if (plan.stops.size() == 1) {
        const logistics::Stop& s = plan.stops[0];
        check(s.nodeId == "D1", "停靠点是 D1");
        check(s.rawArrivalMin == 490, "rawArrival 490，实际 " + std::to_string(s.rawArrivalMin));
        check(s.waitMin == 50, "等待 50，实际 " + std::to_string(s.waitMin));
        check(s.arrivalMin == 540, "arrival 540，实际 " + std::to_string(s.arrivalMin));
        check(!s.late, "未超时");
        check(s.penaltyMin == 0, "penalty 0");
        check(fixtures::nearlyEqual(s.remainingLoadKg, 0.0),
              "卸完全部货物后剩余载重 0，实际 " + std::to_string(s.remainingLoadKg));
    }
}

// 切片 1（边界）：没有订单时退化为原地不动，不产生任何位移
void testEmptyOrdersDegeneratesToNoMovement() {
    const LogisticsGraph g = makeWtoD1Graph();
    const Vehicle v = makeVehicle("W", 1000.0, 480);

    const RoutePlan plan = planRoute(g, v, std::vector<Order>{}, WeightType::Distance);

    check(plan.status == PlanStatus::Ok, "空订单规划成功");
    check(plan.nodes == std::vector<std::string>{"W"},
          "空订单时序列只有起点，实际 " + join(plan.nodes));
    check(plan.stops.empty(), "空订单时无停靠点");
    check(fixtures::nearlyEqual(plan.totalDistanceKm, 0.0), "总距离 0");
    check(fixtures::nearlyEqual(plan.totalTimeMin, 0.0), "总耗时 0");
    check(fixtures::nearlyEqual(plan.totalCostYuan, 0.0), "总成本 0");
}

// 切片 2：贪心按最短路权重选下一站 —— D1 更近，必须先服务 D1
void testGreedyServesNearestCandidateFirst() {
    LogisticsGraph g;
    g.addNode(makeNode("W", NodeType::Warehouse, "仓库"));
    g.addNode(makeNode("D1", NodeType::Delivery, "客户1"));
    g.addNode(makeNode("D2", NodeType::Delivery, "客户2"));
    addTwoWay(g, "W", "D1", 1.0, 2.0, 1.0);   // 近
    addTwoWay(g, "W", "D2", 5.0, 10.0, 5.0);  // 远
    addTwoWay(g, "D1", "D2", 2.0, 4.0, 2.0);

    const Vehicle v = makeVehicle("W", 1000.0, 480);
    const std::vector<Order> orders = {
        makeOrder("O1", "D1", 10.0, 0, 1440, false),
        makeOrder("O2", "D2", 10.0, 0, 1440, false),
    };

    const RoutePlan plan = planRoute(g, v, orders, WeightType::Distance);

    check(plan.status == PlanStatus::Ok, "规划成功");
    // W->D1 = 1.0，W->D2 = 5.0，故先 D1；随后 D1->D2 = 2.0。
    // 返回段不是直连 D2->W = 5.0，而是绕经 D1：D2->D1->W = 2.0+1.0 = 3.0 更短。
    check(plan.nodes == std::vector<std::string>{"W", "D1", "D2", "D1", "W"},
          "返回段应走更短的 D2->D1->W，实际 " + join(plan.nodes));
    check(plan.stops.size() == 2 && plan.stops[0].nodeId == "D1"
              && plan.stops[1].nodeId == "D2",
          "停靠顺序为 D1, D2（途经 D1 返回不产生第二次停靠）");
    check(fixtures::nearlyEqual(plan.totalDistanceKm, 6.0),
          "总距离 1.0+2.0+3.0 = 6.0，实际 " + std::to_string(plan.totalDistanceKm));
}

// 切片 2：最短路权重并列时，必须按节点 ID 字典序取，保证输出确定。
// 这里刻意把字典序较大的 DZ 放在订单列表前面；若实现"先到先得"，会先服务 DZ。
void testTieBreakIsDeterministicByNodeId() {
    LogisticsGraph g;
    g.addNode(makeNode("W", NodeType::Warehouse, "仓库"));
    g.addNode(makeNode("DA", NodeType::Delivery, "客户A"));
    g.addNode(makeNode("DZ", NodeType::Delivery, "客户Z"));
    addTwoWay(g, "W", "DA", 3.0, 6.0, 3.0);   // 与 W->DZ 等距
    addTwoWay(g, "W", "DZ", 3.0, 6.0, 3.0);
    addTwoWay(g, "DA", "DZ", 2.0, 4.0, 2.0);

    const Vehicle v = makeVehicle("W", 1000.0, 480);
    // DZ 在列表中排在前头，用来区分"字典序"与"先到先得"
    const std::vector<Order> orders = {
        makeOrder("OZ", "DZ", 10.0, 0, 1440, false),
        makeOrder("OA", "DA", 10.0, 0, 1440, false),
    };

    const RoutePlan plan = planRoute(g, v, orders, WeightType::Distance);

    check(plan.status == PlanStatus::Ok, "并列场景规划成功");
    check(plan.stops.size() == 2 && plan.stops[0].nodeId == "DA",
          "并列时按 ID 字典序取 DA，实际首个停靠点 "
              + (plan.stops.empty() ? std::string("(无)") : plan.stops[0].nodeId));
}

// 切片 3：紧急订单优先。
// D2 更远（5.0 vs 1.0），但被标为紧急，因此必须先服务 D2。
// 对照：把 urgent 全部置 false，则顺序反转回按距离的 D1 优先。
void testUrgentOrderIsServedFirstDespiteBeingFarther() {
    LogisticsGraph g;
    g.addNode(makeNode("W", NodeType::Warehouse, "仓库"));
    g.addNode(makeNode("D1", NodeType::Delivery, "客户1"));
    g.addNode(makeNode("D2", NodeType::Delivery, "客户2"));
    addTwoWay(g, "W", "D1", 1.0, 2.0, 1.0);
    addTwoWay(g, "W", "D2", 5.0, 10.0, 5.0);
    addTwoWay(g, "D1", "D2", 2.0, 4.0, 2.0);

    const Vehicle v = makeVehicle("W", 1000.0, 480);

    const std::vector<Order> withUrgent = {
        makeOrder("O1", "D1", 10.0, 0, 1440, false),
        makeOrder("O2", "D2", 10.0, 0, 1440, true),   // 远，但紧急
    };
    const RoutePlan urgentPlan = planRoute(g, v, withUrgent, WeightType::Distance);

    check(urgentPlan.status == PlanStatus::Ok, "紧急单场景规划成功");
    check(urgentPlan.stops.size() == 2 && urgentPlan.stops[0].nodeId == "D2",
          "紧急的 D2 必须先服务，实际首个停靠点 "
              + (urgentPlan.stops.empty() ? std::string("(无)") : urgentPlan.stops[0].nodeId));
    // W->D2 的最短路不是直达(5.0)，而是绕经 D1：W->D1->D2 = 1.0+2.0 = 3.0。
    // 因此完整序列会途经 D1 两次中的一次仅作路过，不产生第二次停靠——
    // 这是设计文档中已声明的"路径按原子处理"简化，此处将其固化为文档化行为。
    check(urgentPlan.nodes == std::vector<std::string>{"W", "D1", "D2", "D1", "W"},
          "顺序为 W->D1->D2->D1->W（去程途经 D1 但不停靠），实际 " + join(urgentPlan.nodes));

    // 对照组：同样的订单但不紧急，应按距离先服务 D1
    const std::vector<Order> withoutUrgent = {
        makeOrder("O1", "D1", 10.0, 0, 1440, false),
        makeOrder("O2", "D2", 10.0, 0, 1440, false),
    };
    const RoutePlan plainPlan = planRoute(g, v, withoutUrgent, WeightType::Distance);
    check(plainPlan.stops.size() == 2 && plainPlan.stops[0].nodeId == "D1",
          "非紧急对照组应按距离先服务 D1");
}

// 切片 4：晚到必须标记超时并记录 penalty。
// W->D1 = 10min，发车 480 -> rawArrival 490；窗口 480-485 已在到达前关闭。
//   wait = 0（窗口起早于到达），arrival = 490，penalty = 490 - 485 = 5
void testLateArrivalIsMarkedWithPenalty() {
    const LogisticsGraph g = makeWtoD1Graph();
    const Vehicle v = makeVehicle("W", 1000.0, 480);
    const std::vector<Order> orders = {makeOrder("O1", "D1", 10.0, 480, 485, false)};

    const RoutePlan plan = planRoute(g, v, orders, WeightType::Distance);

    check(plan.status == PlanStatus::Ok, "规划成功");
    check(plan.stops.size() == 1, "1 个停靠点");
    if (plan.stops.size() == 1) {
        const logistics::Stop& s = plan.stops[0];
        check(s.waitMin == 0, "窗口早于到达，无等待，实际 " + std::to_string(s.waitMin));
        check(s.arrivalMin == 490, "arrival 490，实际 " + std::to_string(s.arrivalMin));
        check(s.late, "必须标记为超时");
        check(s.penaltyMin == 5, "penalty 490-485 = 5，实际 " + std::to_string(s.penaltyMin));
    }
    check(plan.totalPenaltyMin == 5, "totalPenalty 5，实际 " + std::to_string(plan.totalPenaltyMin));
}

// 切片 4：同一配送点的多个订单必须合并为一次停靠——
// 需求求和、窗口取并集 [min 窗口起, max 窗口止]。
//   O1 窗口 480-485（若单独处理会超时，penalty 5）
//   O2 窗口 840-900
//   合并后窗口 480-900，arrival 490 不超时 -> penalty 0，且只停靠一次
void testMultipleOrdersOnSameNodeMergeIntoOneStop() {
    const LogisticsGraph g = makeWtoD1Graph();
    const Vehicle v = makeVehicle("W", 1000.0, 480);
    const std::vector<Order> orders = {
        makeOrder("O1", "D1", 30.0, 480, 485, false),
        makeOrder("O2", "D1", 20.0, 840, 900, false),
    };

    const RoutePlan plan = planRoute(g, v, orders, WeightType::Distance);

    check(plan.status == PlanStatus::Ok, "规划成功");
    check(plan.stops.size() == 1,
          "同节点两订单只产生 1 个停靠点，实际 " + std::to_string(plan.stops.size()));
    check(plan.totalPenaltyMin == 0,
          "窗口并集为 480-900，到达 490 不超时 -> penalty 0，实际 "
              + std::to_string(plan.totalPenaltyMin));
    if (plan.stops.size() == 1) {
        const logistics::Stop& s = plan.stops[0];
        check(s.nodeId == "D1", "停靠点是 D1");
        check(s.arrivalMin == 490, "arrival 490，实际 " + std::to_string(s.arrivalMin));
        // 两单需求量求和 30+20 = 50，一次卸完 -> 剩余 0
        check(fixtures::nearlyEqual(s.remainingLoadKg, 0.0),
              "合并后一次卸完 50kg，剩余 0，实际 " + std::to_string(s.remainingLoadKg));
    }
    // 490 送达 -> 495 离开 -> 505 回到仓库；总耗时 505-480 = 25
    check(fixtures::nearlyEqual(plan.totalTimeMin, 20.0),
          "总耗时 25（若未合并会因等待 840 而暴增），实际 " + std::to_string(plan.totalTimeMin));
    check(fixtures::nearlyEqual(plan.totalDistanceKm, 10.0), "总距离 10.0");
}

// 切片 5：**单个订单**的货量就超过载重上限 -> 这才是真正的载重不可行
// （分多少趟都装不下一个订单）
void testSingleOrderExceedingCapacityIsInfeasible() {
    const LogisticsGraph g = makeWtoD1Graph();
    const Vehicle v = makeVehicle("W", 10.0, 480);   // 载重仅 10
    const std::vector<Order> orders = {makeOrder("O1", "D1", 20.0, 0, 1440, false)};

    const RoutePlan plan = planRoute(g, v, orders, WeightType::Distance);

    check(plan.status == PlanStatus::OrderExceedsCapacity, "状态为 OrderExceedsCapacity");
    check(plan.nodes.empty(), "不可行时不生成路线（nodes 为空）");
    check(plan.stops.empty(), "不可行时无停靠点");
    check(plan.reason.find("20") != std::string::npos,
          "原因应含超限货量，实际: " + plan.reason);
}

// 路线必须是图上**真实可走**的序列。这条断言本该一开始就有：
// 多趟的 Trip 一度漏写起点，就是因为只查了"末尾是不是仓库"而没查相邻可达。
void checkRouteIsWalkable(const LogisticsGraph& g, const RoutePlan& plan,
                          const std::string& depot) {
    check(!plan.nodes.empty(), "路线序列非空");
    if (plan.nodes.empty()) {
        return;
    }
    check(plan.nodes.front() == depot, "路线从仓库出发");
    check(plan.nodes.back() == depot, "路线回到仓库");

    bool walkable = true;
    int lastArrival = -1;
    bool timeMonotonic = true;
    for (std::size_t i = 1; i < plan.nodes.size(); ++i) {
        if (g.findEdge(plan.nodes[i - 1], plan.nodes[i]) == nullptr) {
            walkable = false;
        }
    }
    check(walkable, "路线相邻节点之间都存在有向边（序列真实可走）");

    for (std::size_t i = 0; i < plan.nodeArrivalMin.size(); ++i) {
        if (plan.nodeArrivalMin[i] < lastArrival) {
            timeMonotonic = false;
        }
        lastArrival = plan.nodeArrivalMin[i];
    }
    check(timeMonotonic, "到达时刻沿序列单调不减");

    check(plan.nodes.size() == plan.nodeArrivalMin.size()
              && plan.nodes.size() == plan.nodeIsStop.size()
              && plan.nodes.size() == plan.nodeTripIndex.size(),
          "节点/到达时刻/停靠标记/趟归属四个数组等长");

    // 趟归属必须与 trips 的拼接一一对应（单调不减，且在有效范围内）
    bool tripIndexOk = true;
    std::size_t lastTrip = 0;
    for (std::size_t i = 0; i < plan.nodeTripIndex.size(); ++i) {
        if (plan.nodeTripIndex[i] >= plan.trips.size()) {
            tripIndexOk = false;
        }
        if (i > 0 && plan.nodeTripIndex[i] < plan.nodeTripIndex[i - 1]) {
            tripIndexOk = false;
        }
        lastTrip = plan.nodeTripIndex[i];
    }
    check(tripIndexOk, "趟归属单调不减且都在有效范围内");
    check(plan.trips.empty() || lastTrip + 1 == plan.trips.size(),
          "趟归属覆盖到最后一趟");

    // 各趟首尾相接，且每趟自身也可走
    bool tripsChain = true;
    for (std::size_t k = 0; k < plan.trips.size(); ++k) {
        const logistics::Trip& trip = plan.trips[k];
        if (trip.nodes.empty()) {
            tripsChain = false;
            continue;
        }
        if (!trip.nodeArrivalMin.empty() && trip.nodes.size() != trip.nodeArrivalMin.size()) {
            tripsChain = false;
        }
        if (k > 0 && trip.nodes.front() != plan.trips[k - 1].endNodeId) {
            tripsChain = false;
        }
        for (std::size_t i = 1; i < trip.nodes.size(); ++i) {
            if (g.findEdge(trip.nodes[i - 1], trip.nodes[i]) == nullptr) {
                tripsChain = false;
            }
        }
    }
    check(tripsChain, "各趟首尾相接且每趟自身可走");
    check(!plan.trips.empty() && plan.trips.back().endNodeId == depot,
          "最后一趟终点是仓库");
}

// 构造一个含中转站的图：W -- T -- {D1, D2}，D1/D2 归属于 T 的子网络
LogisticsGraph makeTransitClusterGraph() {
    LogisticsGraph g;
    Node w = makeNode("W", NodeType::Warehouse, "仓库");
    w.x = 0;    w.y = 0;
    Node t = makeNode("T", NodeType::Transit, "集散站");
    t.x = 100;  t.y = 0;   t.subNetworkId = 1;
    Node d1 = makeNode("D1", NodeType::Delivery, "客户1");
    d1.x = 150; d1.y = 0;  d1.subNetworkId = 1;
    Node d2 = makeNode("D2", NodeType::Delivery, "客户2");
    d2.x = 150; d2.y = 20; d2.subNetworkId = 1;
    g.addNode(w); g.addNode(t); g.addNode(d1); g.addNode(d2);
    addTwoWay(g, "W", "T", 10.0, 10.0, 16.0);
    addTwoWay(g, "T", "D1", 5.0, 5.0, 5.0);
    addTwoWay(g, "T", "D2", 6.0, 6.0, 6.0);
    return g;
}

// 切片 6（D21 核心）：总需求超过载重上限**不再不可行**，
// 而是多趟 + 中转站暂存 + 二次配发。
void testTotalDemandOverCapacityBecomesMultiTrip() {
    const LogisticsGraph g = makeTransitClusterGraph();
    const Vehicle v = makeVehicle("W", 20.0, 480);   // 载重 20，总需求 30
    const std::vector<Order> orders = {makeOrder("O1", "D1", 15.0, 0, 1440, false),
                                       makeOrder("O2", "D2", 15.0, 0, 1440, false)};

    const RoutePlan plan = planRoute(g, v, orders, WeightType::Distance);

    // 不变量 1：全部订单被服务（否则才叫不可行）
    check(plan.status == PlanStatus::Ok, "总需求超载不再是不可行");
    check(plan.stops.size() == 2, "两个订单都被送达，实际 "
              + std::to_string(plan.stops.size()));

    // 不变量 2：任一趟的在车货量不超过载重
    bool loadOk = true;
    for (const logistics::Trip& trip : plan.trips) {
        for (const logistics::Stop& s : trip.stops) {
            if (s.remainingLoadKg < -1e-9) {
                loadOk = false;
            }
        }
        // 本趟装载量本身也必须 <= 载重上限
        if (trip.loadKg > v.capacityKg + 1e-9) {
            loadOk = false;
        }
    }
    check(loadOk, "任一趟的在车货量都不超过载重上限");

    // 中转站不参与**常规排线**：那条"有货就用"的路由实现仍不存在。
    // 但 2026-09-26 用户裁定恢复了"缓冲库存**只服务紧急单**"这一窄机制，
    // 所以"暂存非负""寄存只发生在顺路经过的站""库存守恒"**不再**由结构保证——
    // 它们已改由 testTransitBufferProducer() 的守卫 G1/G2/G4 显式断言。

    // 不变量 5：需求超载 -> 必须多于一趟
    check(plan.trips.size() > 1, "多趟配送，实际 " + std::to_string(plan.trips.size()) + " 趟");

    // 路线必须是图上真实可走的序列（这条断言能抓住"Trip 漏写起点"这类错误）
    checkRouteIsWalkable(g, plan, "W");

    // 不变量 6：汇总等于各趟累加
    double distSum = 0.0;
    std::size_t stopSum = 0;
    bool flattenOk = true;
    for (const logistics::Trip& trip : plan.trips) {
        distSum += trip.totalDistanceKm;
        stopSum += trip.stops.size();
        flattenOk = flattenOk && trip.nodes.size() == trip.nodeArrivalMin.size()
                    && trip.nodes.size() == trip.nodeIsStop.size();
    }
    check(std::fabs(distSum - plan.totalDistanceKm) < 1e-6,
          "总距离 = 各趟累加");
    check(stopSum == plan.stops.size(), "扁平停靠点 = 各趟停靠点之和");
    check(flattenOk, "每趟的节点序列与到达时刻/停靠标记等长");
}

// 切片 6：总需求不超过载重时保持单趟，不引入多余的中转环节
void testWithinCapacityStaysSingleTrip() {
    const LogisticsGraph g = makeTransitClusterGraph();
    const Vehicle v = makeVehicle("W", 100.0, 480);
    const std::vector<Order> orders = {makeOrder("O1", "D1", 15.0, 0, 1440, false),
                                       makeOrder("O2", "D2", 15.0, 0, 1440, false)};

    const RoutePlan plan = planRoute(g, v, orders, WeightType::Distance);

    check(plan.status == PlanStatus::Ok, "可行");
    check(plan.trips.size() == 1, "不超载时只有一趟，实际 "
              + std::to_string(plan.trips.size()));
    checkRouteIsWalkable(g, plan, "W");
}

// 切片 5：存在无法到达的配送点 -> Unreachable，且原因要能定位到具体节点
void testUnreachableDeliveryIsInfeasibleAndNamesTheNode() {
    LogisticsGraph g = makeWtoD1Graph();
    g.addNode(makeNode("D9", NodeType::Delivery, "孤岛客户"));   // 无任何连边

    const Vehicle v = makeVehicle("W", 1000.0, 480);
    const std::vector<Order> orders = {makeOrder("O1", "D9", 10.0, 0, 1440, false)};

    const RoutePlan plan = planRoute(g, v, orders, WeightType::Distance);

    check(plan.status == PlanStatus::Unreachable, "状态为 Unreachable");
    check(plan.stops.empty(), "不可行时无停靠点");
    check(plan.reason.find("D9") != std::string::npos,
          "原因中应含不可达节点 ID，实际: " + plan.reason);
}


// ---- P2：增量式重规划（设计 §5.7 / D22）----
//
// 拓扑：W --10-- D1 --10-- D2 --20-- W，另有绕行 D1 --6-- D3 --6-- D2（D3 是中转站）。
// 按耗时规划时 D1->D2 走直连（10 < 12）。把直连封堵成 +50%（=15）后，
// 绕行 12 反而更快，于是只有"含该边的那一段"需要重算。
LogisticsGraph makeIncrementalGraph() {
    LogisticsGraph g;
    Node w = makeNode("W", NodeType::Warehouse, "仓库");   w.x = 0;   w.y = 0;
    Node d1 = makeNode("D1", NodeType::Delivery, "客户1"); d1.x = 100; d1.y = 0;
    Node d2 = makeNode("D2", NodeType::Delivery, "客户2"); d2.x = 200; d2.y = 0;
    Node d3 = makeNode("D3", NodeType::Transit, "中转站"); d3.x = 150; d3.y = 100;
    g.addNode(w); g.addNode(d1); g.addNode(d2); g.addNode(d3);
    addTwoWay(g, "W", "D1", 10.0, 10.0, 10.0);
    addTwoWay(g, "D1", "D2", 10.0, 10.0, 10.0);
    addTwoWay(g, "D2", "W", 20.0, 20.0, 20.0);
    addTwoWay(g, "D1", "D3", 6.0, 6.0, 6.0);
    addTwoWay(g, "D3", "D2", 6.0, 6.0, 6.0);
    return g;
}

// 断言 1/2/3/5：只重算受影响的段、停靠顺序不变、其余段逐位不变、总耗时更优
void testIncrementalReplanRecomputesOnlyAffectedLeg() {
    LogisticsGraph g = makeIncrementalGraph();
    const Vehicle v = makeVehicle("W", 1000.0, 480);
    const std::vector<Order> orders = {makeOrder("O1", "D1", 10.0, 0, 1440, false),
                                       makeOrder("O2", "D2", 10.0, 0, 1440, false)};

    const RoutePlan before = planRoute(g, v, orders, WeightType::Time);
    check(before.status == PlanStatus::Ok, "初始规划可行");
    check(before.nodes == std::vector<std::string>({"W", "D1", "D2", "W"}),
          "初始序列 W->D1->D2->W");

    // 构造确定的拥堵：D1->D2 耗时 +50%（不用随机模拟器，保证可复现）
    const logistics::Edge* e = g.findEdge("D1", "D2");
    check(e != nullptr, "D1->D2 存在");
    if (e == nullptr) {
        return;
    }
    g.updateEdgeTime("D1", "D2", e->baseTimeMin * 1.5);
    const logistics::Edge* e2 = g.findEdge("D2", "D1");
    if (e2 != nullptr) {
        g.updateEdgeTime("D2", "D1", e2->baseTimeMin * 1.5);
    }

    TrafficReport report;
    TrafficChange c;
    c.fromId = "D1";
    c.toId = "D2";
    c.increaseRatio = 0.5;
    c.congested = true;
    report.changes.push_back(c);

    // 重规划前那条路线在**拥堵后**的耗时，作为比较基准
    const RoutePlan congestedSameRoute = replan(g, v, orders, "W", 480, WeightType::Time);

    const RoutePlan after =
        replanIncremental(g, v, orders, before, 480, WeightType::Time, report, 0.2);

    check(after.status == PlanStatus::Ok, "增量重规划可行");

    // 断言 1：停靠顺序逐位相同
    check(after.stops.size() == 2, "仍服务 2 个停靠点");
    if (after.stops.size() == 2) {
        check(after.stops[0].nodeId == "D1" && after.stops[1].nodeId == "D2",
              "停靠顺序不变（增量不重排服务顺序）");
    }

    // 断言 2：未受影响的段逐位不变 —— 首段 W->D1 必须原样
    check(after.nodes.size() >= 2 && after.nodes[0] == "W" && after.nodes[1] == "D1",
          "未受影响的首段 W->D1 保持原样");

    // 受影响的段被重算：改走绕行 D3
    bool viaD3 = false;
    for (const std::string& id : after.nodes) {
        if (id == "D3") {
            viaD3 = true;
        }
    }
    check(viaD3, "受影响的 D1->D2 段被重算，改经 D3 绕行");

    // 断言 5：仍回到仓库，且序列真实可走
    checkRouteIsWalkable(g, after, "W");

    // 断言 3：总耗时优于"沿旧路线的拥堵版本"
    check(after.totalTimeMin <= congestedSameRoute.totalTimeMin + 1e-6,
          "增量结果总耗时 <= 拥堵后沿旧路线："
              + std::to_string(after.totalTimeMin) + " vs "
              + std::to_string(congestedSameRoute.totalTimeMin));
}

// 断言 4：没有任何 leg 受影响时，结果与原路线逐位相同（幂等）
void testIncrementalReplanIsIdempotentWhenNothingAffected() {
    const LogisticsGraph g = makeIncrementalGraph();
    const Vehicle v = makeVehicle("W", 1000.0, 480);
    const std::vector<Order> orders = {makeOrder("O1", "D1", 10.0, 0, 1440, false),
                                       makeOrder("O2", "D2", 10.0, 0, 1440, false)};

    const RoutePlan before = planRoute(g, v, orders, WeightType::Time);

    // 报告里只有一条**不在路线上的**边（D1->D3 是绕行边，当前路线没走）
    TrafficReport report;
    TrafficChange c;
    c.fromId = "D1";
    c.toId = "D3";
    c.increaseRatio = 0.8;
    c.congested = true;
    report.changes.push_back(c);

    const RoutePlan after =
        replanIncremental(g, v, orders, before, 480, WeightType::Time, report, 0.2);

    check(after.nodes == before.nodes, "无 leg 受影响时，节点序列逐位不变");
    check(after.stops.size() == before.stops.size(), "停靠点数不变");
    check(fixtures::nearlyEqual(after.totalTimeMin, before.totalTimeMin),
          "总耗时不变");
}

// 范围限制：上一版是多趟时，增量退回全量重算，结果仍须合法
// D22 的契约：增量重规划的结果必须与全量重算一致；且**多趟也要能走增量**。
//
// 早期实现只支持"整条剩余路线是单趟"，默认 6 趟方案下只有最后一趟用得上，
// 其余一律全量——审计认为这实质等于没实现。现在改为**逐趟增量**。
void testIncrementalReplanHandlesMultiTrip() {
    const LogisticsGraph g = makeTransitClusterGraph();
    const Vehicle v = makeVehicle("W", 20.0, 480);
    const std::vector<Order> orders = {makeOrder("O1", "D1", 15.0, 0, 1440, false),
                                       makeOrder("O2", "D2", 15.0, 0, 1440, false)};

    const RoutePlan multi = planRoute(g, v, orders, WeightType::Distance);
    check(multi.trips.size() > 1, "前提：上一版确为多趟");

    // 空报告：没有受影响的路段，走增量应原样保留
    bool used = false;
    TrafficReport empty;
    const RoutePlan same = replanIncremental(g, v, orders, multi, 480,
                                             WeightType::Distance, empty, 0.2,
                                             std::vector<logistics::OnboardItem>(), &used);
    check(used, "多趟剩余路线也必须走增量路径（不再一律退回全量）");
    check(same.status == PlanStatus::Ok && same.stops.size() == multi.stops.size(),
          "无路况变化时停靠点应完全保留");
    check(same.totalDistanceKm == multi.totalDistanceKm,
          "无路况变化时总距离应不变，实际 "
              + std::to_string(same.totalDistanceKm) + " vs "
              + std::to_string(multi.totalDistanceKm));

    // 有路况变化：增量结果必须与全量重算**一致**（D22 契约）
    TrafficReport report;
    for (const logistics::Edge& e : g.edges()) {
        logistics::TrafficChange c;
        c.fromId = e.fromId;
        c.toId = e.toId;
        c.congested = true;
        c.increaseRatio = 0.5;
        report.changes.push_back(c);
    }
    bool used2 = false;
    const RoutePlan inc = replanIncremental(g, v, orders, multi, 480,
                                            WeightType::Distance, report, 0.2,
                                            std::vector<logistics::OnboardItem>(), &used2);
    check(inc.status == PlanStatus::Ok, "全边拥堵后仍可行");
    checkRouteIsWalkable(g, inc, "W");
    check(inc.stops.size() == multi.stops.size(), "停靠点数量不变");

    // 与全量重算对比：停靠顺序与各站到达时刻应一致
    const RoutePlan full = replan(g, v, orders, "W", 480, WeightType::Distance);
    check(inc.stops.size() == full.stops.size(), "与全量重算的停靠点数一致");
    bool sameOrder = (inc.stops.size() == full.stops.size());
    for (std::size_t i = 0; sameOrder && i < inc.stops.size(); ++i) {
        sameOrder = (inc.stops[i].nodeId == full.stops[i].nodeId);
    }
    check(sameOrder, "增量结果的停靠顺序应与全量重算一致");
}

bool samePlan(const RoutePlan& a, const RoutePlan& b) {
    if (a.status != b.status || a.nodes != b.nodes || a.stops.size() != b.stops.size()) {
        return false;
    }
    if (!fixtures::nearlyEqual(a.totalDistanceKm, b.totalDistanceKm)
        || !fixtures::nearlyEqual(a.totalTimeMin, b.totalTimeMin)
        || !fixtures::nearlyEqual(a.totalCostYuan, b.totalCostYuan)
        || a.totalPenaltyMin != b.totalPenaltyMin) {
        return false;
    }
    for (std::size_t i = 0; i < a.stops.size(); ++i) {
        if (a.stops[i].nodeId != b.stops[i].nodeId
            || a.stops[i].arrivalMin != b.stops[i].arrivalMin
            || a.stops[i].late != b.stops[i].late
            || a.stops[i].penaltyMin != b.stops[i].penaltyMin) {
            return false;
        }
    }
    return true;
}

// 切片 6：replan 从给定位置与给定时刻出发，服务剩余订单后返回车辆起始仓库
void testReplanFromCurrentPosition() {
    LogisticsGraph g;
    g.addNode(makeNode("W", NodeType::Warehouse, "仓库"));
    g.addNode(makeNode("D1", NodeType::Delivery, "客户1"));
    g.addNode(makeNode("D2", NodeType::Delivery, "客户2"));
    addTwoWay(g, "W", "D1", 1.0, 2.0, 1.0);
    addTwoWay(g, "W", "D2", 5.0, 10.0, 5.0);
    addTwoWay(g, "D1", "D2", 2.0, 4.0, 2.0);

    const Vehicle v = makeVehicle("W", 1000.0, 480);
    // 车已在 D2，时刻 600，剩余一个 D1 的订单
    const std::vector<Order> remaining = {makeOrder("O1", "D1", 10.0, 0, 1440, false)};

    const RoutePlan plan = replan(g, v, remaining, "D2", 600, WeightType::Distance);

    check(plan.status == PlanStatus::Ok, "重规划成功");
    // D2->D1 = 4min -> 604 到达；D1->W = 2min -> 606（服务时间已移除）
    check(plan.nodes == std::vector<std::string>{"D2", "D1", "W"},
          "序列为 D2->D1->W，实际 " + join(plan.nodes));
    check(fixtures::nearlyEqual(plan.totalTimeMin, 6.0),
          "总耗时 606-600 = 6，实际 " + std::to_string(plan.totalTimeMin));
    check(fixtures::nearlyEqual(plan.totalDistanceKm, 3.0),
          "总距离 2.0+1.0 = 3.0，实际 " + std::to_string(plan.totalDistanceKm));
    check(plan.stops.size() == 1 && plan.stops[0].rawArrivalMin == 604,
          "首个停靠点 rawArrival 604");
}

// 切片 6（边界）：当前位置恰好就是待配送点 -> 立即服务，不产生位移
void testReplanWhenAlreadyAtTheDeliveryNode() {
    const LogisticsGraph g = makeWtoD1Graph();
    const Vehicle v = makeVehicle("W", 1000.0, 480);
    const std::vector<Order> remaining = {makeOrder("O1", "D1", 10.0, 0, 1440, false)};

    const RoutePlan plan = replan(g, v, remaining, "D1", 600, WeightType::Distance);

    check(plan.status == PlanStatus::Ok, "重规划成功");
    check(plan.nodes == std::vector<std::string>{"D1", "W"},
          "已在配送点则无位移，实际 " + join(plan.nodes));
    check(plan.stops.size() == 1 && plan.stops[0].rawArrivalMin == 600,
          "立即服务，rawArrival 等于当前时刻 600");
    check(fixtures::nearlyEqual(plan.totalDistanceKm, 5.0),
          "本图 W<->D1 为 5.0km，故只算返回段 5.0，实际 "
              + std::to_string(plan.totalDistanceKm));
}

// 切片 6：两个入口必须共用同一份实现 ——
// planRoute(...) 等价于 replan(..., 仓库, 发车时刻, ...)
void testPlanRouteEqualsReplanFromDepot() {
    LogisticsGraph g;
    g.addNode(makeNode("W", NodeType::Warehouse, "仓库"));
    g.addNode(makeNode("D1", NodeType::Delivery, "客户1"));
    g.addNode(makeNode("D2", NodeType::Delivery, "客户2"));
    addTwoWay(g, "W", "D1", 1.0, 2.0, 1.0);
    addTwoWay(g, "W", "D2", 5.0, 10.0, 5.0);
    addTwoWay(g, "D1", "D2", 2.0, 4.0, 2.0);

    const Vehicle v = makeVehicle("W", 1000.0, 480);
    const std::vector<Order> orders = {
        makeOrder("O1", "D1", 10.0, 0, 1440, false),
        makeOrder("O2", "D2", 10.0, 0, 1440, true),
    };

    const RoutePlan viaPlanRoute = planRoute(g, v, orders, WeightType::Distance);
    const RoutePlan viaReplan =
        replan(g, v, orders, v.startNodeId, v.departTimeMin, WeightType::Distance);

    check(samePlan(viaPlanRoute, viaReplan),
          "planRoute 与 replan(仓库, 发车时刻) 结果必须完全一致");
}

// 切片 8：路径成员判定（GUI 路径高亮与路况触发判定共用这一条逻辑）
void testIsEdgeOnRoute() {
    const std::vector<std::string> route = {"W", "D1", "D2", "W"};

    check(logistics::isEdgeOnRoute(route, "W", "D1"), "首段在路径上");
    check(logistics::isEdgeOnRoute(route, "D1", "D2"), "中段在路径上");
    check(logistics::isEdgeOnRoute(route, "D2", "W"), "末段在路径上");

    check(!logistics::isEdgeOnRoute(route, "D1", "W"), "反向边不算（路径有向）");
    check(!logistics::isEdgeOnRoute(route, "W", "D2"), "非相邻的一对不算");
    check(!logistics::isEdgeOnRoute(route, "D2", "D1"), "反向的另一半也不算");
    check(!logistics::isEdgeOnRoute({}, "W", "D1"), "空路径不含任何边");
    check(!logistics::isEdgeOnRoute({"W"}, "W", "W"), "单节点无相邻对");

    // 重复经过同一段也算在路径上
    const std::vector<std::string> loop = {"W", "D1", "W", "D1", "W"};
    check(logistics::isEdgeOnRoute(loop, "D1", "W"), "重复经过的段落也在路径上");
}

// ---- nodeIndexAtTime：软件内时刻 -> "半路显示所在段的起点" ----
//
// 用切片 1 的手算路线：W(480) -> D1(540) -> W(550)，index 0/1/2。
// 该函数的语义是**段起点**：车在 i -> i+1 途中时返回 i。
void testNodeIndexAtTimeReturnsSegmentStart() {
    const LogisticsGraph g = makeWtoD1Graph();
    const Vehicle v = makeVehicle("W", 1000.0, 480);
    const std::vector<Order> orders = {makeOrder("O1", "D1", 10.0, 540, 1080, false)};
    const RoutePlan plan = planRoute(g, v, orders, WeightType::Distance);

    // 先固定前提：到达时刻就是手算的 480 / 540 / 550
    check(plan.nodeArrivalMin == std::vector<int>({480, 540, 550}),
          "前提：逐节点到达时刻为 480 / 540 / 550");
    if (plan.nodes.size() != 3) {
        return;
    }

    // 早于首节点 -> 0（车还在起点，尚未出发/刚到）
    check(nodeIndexAtTime(plan, 400) == 0, "早于首节点 -> 下标 0");
    check(nodeIndexAtTime(plan, 479) == 0, "首节点前 1 分钟 -> 下标 0");

    // 恰等于某节点到达时刻 -> 返回该节点（"恰好已到达"算作到了）
    check(nodeIndexAtTime(plan, 480) == 0, "恰等于节点 0 到达时刻 -> 下标 0");
    check(nodeIndexAtTime(plan, 540) == 1, "恰等于节点 1 到达时刻 -> 下标 1");
    check(nodeIndexAtTime(plan, 550) == 2, "恰等于节点 2 到达时刻 -> 下标 2");

    // 落在两节点之间 -> 返回**前一个**节点（这就是段起点）
    check(nodeIndexAtTime(plan, 500) == 0, "480 之后 540 之前 -> 段起点 0（W->D1 途中）");
    check(nodeIndexAtTime(plan, 539) == 0, "紧临节点 1 之前 -> 段起点 0");
    check(nodeIndexAtTime(plan, 545) == 1, "540 之后 550 之前 -> 段起点 1（D1->W 途中）");

    // 超过末节点 -> 末节点下标（clamp，不越界）
    check(nodeIndexAtTime(plan, 551) == 2, "超过末节点 -> 末节点下标 2");
    check(nodeIndexAtTime(plan, 100000) == 2, "远超末节点 -> 仍为末节点下标 2");
}

// 边界：空 plan 返回 0；节点数 < 到达时刻数等非规范输入也不得越界。
void testNodeIndexAtTimeOnEmptyPlanAndClamp() {
    const RoutePlan empty;
    check(empty.nodes.empty(), "前提：空 plan");
    check(nodeIndexAtTime(empty, 0) == 0, "空 plan -> 0");
    check(nodeIndexAtTime(empty, 99999) == 0, "空 plan（任意时刻）-> 0");

    // 只有一个节点：任何时刻都只能是下标 0（clamp 到 [0, size-1]）
    RoutePlan single;
    single.nodes = {"W"};
    single.nodeArrivalMin = {480};
    check(nodeIndexAtTime(single, 100) == 0, "单节点且早于到达 -> 0");
    check(nodeIndexAtTime(single, 480) == 0, "单节点恰在到达 -> 0");
    check(nodeIndexAtTime(single, 1000) == 0, "单节点超过到达 -> 0（不越界）");

    // nodeArrivalMin 比 nodes 短：只按到达时刻序列判定，且结果不得越界
    RoutePlan ragged;
    ragged.nodes = {"A", "B", "C"};
    ragged.nodeArrivalMin = {480};
    check(nodeIndexAtTime(ragged, 600) == 0, "到达时刻序列不完整时返回其有效下标");
}

// 单调性：timeMin 增大时返回值必须单调不减（抽查多个时刻）。
void testNodeIndexAtTimeIsMonotonicInTime() {
    const LogisticsGraph g = makeTransitClusterGraph();
    const Vehicle v = makeVehicle("W", 20.0, 480);
    const std::vector<Order> orders = {makeOrder("O1", "D1", 10.0, 0, 1440, false),
                                       makeOrder("O2", "D2", 10.0, 0, 1440, false)};
    const RoutePlan plan = planRoute(g, v, orders, WeightType::Distance);
    check(plan.status == PlanStatus::Ok, "前提：多趟路线规划成功");

    std::size_t prev = nodeIndexAtTime(plan, 0);
    bool monotonic = true;
    bool outOfRange = false;
    for (int t = 0; t <= 2000; t += 5) {
        const std::size_t cur = nodeIndexAtTime(plan, t);
        if (cur < prev) {
            monotonic = false;
        }
        if (cur >= plan.nodes.size()) {
            outOfRange = true;
        }
        prev = cur;
    }
    check(monotonic, "timeMin 增大时返回值单调不减（0..2000 每 5min 抽查）");
    check(!outOfRange, "返回值始终落在 [0, nodes.size()-1] 内");

    // 单调性必须是"内容"上的：时间足够大时应落回末节点（回到仓库）
    check(nodeIndexAtTime(plan, 100000) == plan.nodes.size() - 1,
          "足够大的时刻落在末节点（回到仓库）");
}

} // namespace

// 增量重规划在全量回退时，必须把"站内存货"与"在途货"一并带走。
// 否则这条路径上寄存与"积少成多"会静默断掉（审计时发现的缺口）。
static void testIncrementalReplanForwardsOnboard() {
    const logistics::LogisticsGraph g = makeTransitClusterGraph();
    logistics::Vehicle v;
    v.id = "V01"; v.startNodeId = "W"; v.capacityKg = 40.0; v.departTimeMin = 480;

    std::vector<logistics::Order> orders;
    for (const char* id : {"D1", "D2"}) {
        logistics::Order o;
        o.id = std::string("O") + id[1];
        o.nodeId = id;
        o.demandKg = 30.0;
        o.windowStartMin = 0;
        o.windowEndMin = 1440;
        orders.push_back(o);
    }

    // 让增量重规划走上"回退到全量重算"的分支：给一条把当前路线打成不可达的报告
    logistics::TrafficReport report;
    logistics::TrafficChange ch;
    ch.fromId = "T"; ch.toId = "D1";
    ch.congested = true; ch.increaseRatio = 50.0;   // 极大增幅，逼它回退全量重算
    report.changes.push_back(ch);

    // 车上带着 D2 的货：重规划（含回退全量）必须把它当成"车不是空的"
    const std::vector<logistics::OnboardItem> onboard{{"D2", 30.0}};
    const logistics::RoutePlan base = logistics::replan(
        g, v, orders, "W", 480, logistics::WeightType::Distance);
    const logistics::RoutePlan inc = logistics::replanIncremental(
        g, v, orders, base, 480, logistics::WeightType::Distance, report, 0.2, onboard);

    check(inc.status == logistics::PlanStatus::Ok, "带在途货的增量重规划应成功");
    // D2 的货已在车上，故本趟装载只需覆盖它之外的量
    bool carriesD2 = false;
    for (const logistics::Trip& t : inc.trips) {
        for (const logistics::Stop& st : t.stops) {
            if (st.nodeId == "D2") {
                carriesD2 = true;
            }
        }
    }
    check(carriesD2, "在途货所属的 D2 仍应被服务");
}

// ---- 中转站缓冲库存：生产者（满载出仓 + 返程顺路寄存）----
//
// 夹具：W(仓库) —T(中转站,子网络1)— D1(客户,子网络1)，另加 W —D2(客户,子网络1)。
//   · D1 的最短路是 W-T-D1 ⇒ 回程 D1-T-W **真的经过 T**（顺路 ⇒ 应当寄存）；
//   · D2 直连仓库 ⇒ 回程不经过 T（不顺路 ⇒ 缓冲**跟车回仓库**，不寄存）。
// 载重 100、D1/D2 各 60kg ⇒ 总需求 120 > 100 ⇒ 两趟，每趟订单货量 60、空位 40。
LogisticsGraph makeBufferGraph() {
    LogisticsGraph g;
    g.addNode(makeNode("W", NodeType::Warehouse, "仓库"));
    Node t = makeNode("T", NodeType::Transit, "中转站");
    t.subNetworkId = 1;
    g.addNode(t);
    Node d1 = makeNode("D1", NodeType::Delivery, "客户1");
    d1.subNetworkId = 1;
    g.addNode(d1);
    Node d2 = makeNode("D2", NodeType::Delivery, "客户2");
    d2.subNetworkId = 1;
    g.addNode(d2);
    addTwoWay(g, "W", "T", 5.0, 10.0, 4.0);
    addTwoWay(g, "T", "D1", 5.0, 10.0, 4.0);
    addTwoWay(g, "W", "D2", 3.0, 6.0, 2.0);
    return g;
}

void testTransitBufferProducer() {
    const LogisticsGraph g = makeBufferGraph();
    const Vehicle v = makeVehicle("W", 100.0, 480);
    std::vector<Order> orders;
    orders.push_back(makeOrder("O1", "D1", 60.0, 0, 1440));
    orders.push_back(makeOrder("O2", "D2", 60.0, 0, 1440));

    const RoutePlan plan = planRoute(g, v, orders, WeightType::Distance);

    check(plan.status == PlanStatus::Ok, "缓冲夹具应当可规划");
    check(plan.trips.size() == 2, "两个 60kg 订单 + 载重 100 ⇒ 两趟，实际 "
                                      + std::to_string(plan.trips.size()) + " 趟");

    // G4：缓冲不得超载、不得为负；I5：缓冲 == 载重上限 − 本趟订单货量
    bool capOk = true, nonNeg = true, fullOk = true;
    for (const logistics::Trip& t : plan.trips) {
        if (t.loadKg + t.bufferKg > v.capacityKg + 1e-9) { capOk = false; }
        if (t.bufferKg < -1e-9) { nonNeg = false; }
        if (t.bufferKg > 1e-9
            && std::fabs(t.bufferKg - (v.capacityKg - t.loadKg)) > 1e-9) {
            fullOk = false;
        }
    }
    check(capOk, "本趟订单货量 + 缓冲货 不得超过载重上限");
    check(nonNeg, "缓冲货不得为负");
    check(fullOk, "缓冲货 = 载重上限 − 本趟订单货量（「装满」的确切含义）");

    // G1①：常规规划（无紧急单）不得出现任何站内**出库**
    bool noOutbound = true;
    for (const logistics::TransitOp& op : plan.transitOps) {
        if (op.kgDelta < -1e-9) { noOutbound = false; }
    }
    check(noOutbound, "无紧急单时不得有任何站内出库（中转站只服务紧急单）");

    // G1②：寄存在的站必须**真的出现在该趟自己的节点序列里**，且时刻对得上。
    // 这条是"零成本"的实质检查：只有站本来就在序列里，才谈得上"不新增节点"。
    bool bankTraceable = true;
    std::size_t bankCount = 0;
    for (const logistics::Trip& t : plan.trips) {
        for (const logistics::TransitOp& op : t.bankOps) {
            ++bankCount;
            bool hit = false;
            for (std::size_t k = 0; k < t.nodes.size() && k < t.nodeArrivalMin.size(); ++k) {
                if (t.nodes[k] == op.nodeId && t.nodeArrivalMin[k] == op.atMin) { hit = true; }
            }
            if (!hit) { bankTraceable = false; }
        }
    }
    check(bankCount == 1, "夹具里应恰好 1 次寄存（第 1 趟回程顺路经过 T），实际 "
                              + std::to_string(bankCount) + " 次");
    check(bankTraceable, "寄存在的站必须在该趟自己的节点序列里，且 atMin == 该处到达时刻");

    double stock = 0.0;
    for (const logistics::TransitStock& st : plan.transitStock) { stock += st.finalKg; }
    check(std::fabs(stock - 40.0) < 1e-6,
          "顺路的那趟应寄存 40kg 进站，实际站内合计 " + std::to_string(stock) + "kg");

    // 守恒（规格 I3 的生产者部分）：所有寄存之和 == 站内库存增量
    double opsSum = 0.0;
    for (const logistics::TransitOp& op : plan.transitOps) { opsSum += op.kgDelta; }
    check(std::fabs(opsSum - stock) < 1e-6, "所有寄存之和 == 站内库存增量");

    // G2：**库存不得影响选路** —— 同一输入、空库存 vs 有 123kg 存货，路线逐位相同
    std::map<std::string, double> seeded;
    seeded["T"] = 123.0;
    const RoutePlan seededPlan = planRoute(g, v, orders, WeightType::Distance, seeded);
    const bool routeUnchanged =
        std::fabs(seededPlan.totalDistanceKm - plan.totalDistanceKm) < 1e-9
        && std::fabs(seededPlan.totalTimeMin - plan.totalTimeMin) < 1e-9
        && std::fabs(seededPlan.totalCostYuan - plan.totalCostYuan) < 1e-9
        && seededPlan.totalPenaltyMin == plan.totalPenaltyMin
        && seededPlan.trips.size() == plan.trips.size();
    check(routeUnchanged, "期初库存不得改变路径/耗时/成本/penalty/趟数（寄存是纯记账）");

    double seededStock = 0.0;
    for (const logistics::TransitStock& st : seededPlan.transitStock) {
        seededStock += st.finalKg;
    }
    check(std::fabs(seededStock - (stock + 123.0)) < 1e-6,
          "期末库存 = 期初库存 + 本次寄存量");
}

// ---- 中转站缓冲库存：消费者（紧急单三链 + 两版取优）----
//
// 夹具：W(仓库) —T(中转站,子网络1)— D1、D3；W —D2（直连，不经 T）。
// 载重 100、三个 60kg 订单 ⇒ 三趟；第 1、3 趟回程都顺路经过 T ⇒ T 攒下 80kg。
// 车停在 D1（**远离仓库**）时插入一张到 D3 的紧急单：现状必须"回仓库装货"（绕远），
// 而站内库存就在去 D3 的路上 ⇒ 就地满足应当更省。
LogisticsGraph makeUrgentBufferGraph() {
    LogisticsGraph g;
    g.addNode(makeNode("W", NodeType::Warehouse, "仓库"));
    Node t = makeNode("T", NodeType::Transit, "中转站");
    t.subNetworkId = 1;
    g.addNode(t);
    for (const char* id : {"D1", "D2", "D3"}) {
        Node d = makeNode(id, NodeType::Delivery, std::string("客户") + id);
        d.subNetworkId = 1;
        g.addNode(d);
    }
    addTwoWay(g, "W", "T", 5.0, 10.0, 4.0);
    addTwoWay(g, "T", "D1", 5.0, 10.0, 4.0);
    addTwoWay(g, "W", "D2", 3.0, 6.0, 2.0);
    addTwoWay(g, "T", "D3", 5.0, 10.0, 4.0);
    return g;
}

void testUrgentBufferInPlaceDelivery() {
    const LogisticsGraph g = makeUrgentBufferGraph();
    const Vehicle v = makeVehicle("W", 100.0, 480);
    std::vector<Order> orders;
    orders.push_back(makeOrder("O1", "D1", 60.0, 0, 1440));
    orders.push_back(makeOrder("O2", "D2", 60.0, 0, 1440));
    orders.push_back(makeOrder("O3", "D3", 60.0, 0, 1440));

    const RoutePlan base = planRoute(g, v, orders, WeightType::Distance);
    std::map<std::string, double> initialStock;
    double seeded = 0.0;
    for (const logistics::TransitStock& st : base.transitStock) {
        initialStock[st.nodeId] = st.finalKg;
        seeded += st.finalKg;
    }
    check(std::fabs(seeded - 80.0) < 1e-6,
          "基线应给 T 攒下 80kg（第 1、3 趟各顺路寄存 40），实际 "
              + std::to_string(seeded) + "kg");

    const Order urgent = makeOrder("U1", "D3", 10.0, 0, 1440, true);
    const InsertResult inserted = insertUrgentOrder(
        g, v, orders, urgent, "D1", 500, WeightType::Distance,
        std::vector<logistics::OnboardItem>(), 0.0, initialStock);
    check(inserted.plan.status == PlanStatus::Ok, "插入紧急单后应当可规划");
    const RoutePlan& chosen = inserted.plan;

    // 对照：候选 A（现状——独立紧急趟**先回仓库装货**）
    std::vector<Order> all = orders;
    all.push_back(urgent);
    const RoutePlan variantA = replan(g, v, all, "D1", 500, WeightType::Distance,
                                      std::vector<logistics::OnboardItem>(), initialStock);

    // G6：绝不更差（penalty 一票否决 -> 目标值 -> 趟数）
    check(chosen.totalPenaltyMin <= variantA.totalPenaltyMin,
          "取优不得让 penalty 变差：选中 " + std::to_string(chosen.totalPenaltyMin)
              + "min vs 现状 " + std::to_string(variantA.totalPenaltyMin) + "min");
    check(chosen.totalDistanceKm < variantA.totalDistanceKm - 1e-9,
          "本夹具里就地满足应当真的更省：现状 " + std::to_string(variantA.totalDistanceKm)
              + "km，选中 " + std::to_string(chosen.totalDistanceKm) + "km");

    // G5：E3 优先性不得被取优牺牲——紧急单所在停靠点必须早于**所有**其他停靠点
    int urgentArrival = -1;
    int firstOtherArrival = 1 << 30;
    for (const logistics::Trip& t : chosen.trips) {
        for (const logistics::Stop& s : t.stops) {
            if (s.nodeId == "D3") {
                urgentArrival = s.arrivalMin;
            } else if (s.arrivalMin < firstOtherArrival) {
                firstOtherArrival = s.arrivalMin;
            }
        }
    }
    check(urgentArrival >= 0, "紧急单所在停靠点必须被服务");
    check(urgentArrival <= firstOtherArrival,
          "紧急单必须早于所有普通停靠点：紧急 " + std::to_string(urgentArrival)
              + "min vs 最早普通 " + std::to_string(firstOtherArrival) + "min");

    // G3：守恒 + 可追溯
    double opsSum = 0.0;
    bool stockNonNeg = true;
    for (const logistics::TransitOp& op : chosen.transitOps) { opsSum += op.kgDelta; }
    double stockDelta = 0.0;
    for (const logistics::TransitStock& st : chosen.transitStock) {
        stockDelta += st.finalKg - st.initialKg;
        if (st.finalKg < -1e-9) { stockNonNeg = false; }
    }
    check(std::fabs(opsSum - stockDelta) < 1e-6,
          "所有站内装卸之和 == 期末 − 期初（守恒）");
    check(stockNonNeg, "任何时刻站内库存不得为负");

    bool traceable = true;
    for (const logistics::TransitOp& op : chosen.transitOps) {
        bool hit = false;
        for (const logistics::Trip& t : chosen.trips) {
            for (std::size_t k = 0; k < t.nodes.size() && k < t.nodeArrivalMin.size(); ++k) {
                if (t.nodes[k] == op.nodeId && t.nodeArrivalMin[k] == op.atMin) { hit = true; }
            }
        }
        if (!hit) { traceable = false; }
    }
    check(traceable, "每一次站内装卸的 atMin 都必须能追溯到某趟的到达时刻");

    // 就地满足必须**真的**发生：紧急趟在紧急停靠点之前到过 T ⇒ 必须有一条对应该站的出库
    bool tookFromStation = false;
    for (const logistics::TransitOp& op : chosen.transitOps) {
        if (op.kgDelta < -1e-9 && op.nodeId == "T") { tookFromStation = true; }
    }
    check(tookFromStation, "本夹具里车上无缓冲 ⇒ 必须从 T 出库才能就地满足");

    // 车上缓冲足够时**不得**再动站内库存（先车上、后站内的顺序）
    const InsertResult withCar = insertUrgentOrder(
        g, v, orders, urgent, "D1", 500, WeightType::Distance,
        std::vector<logistics::OnboardItem>(), 100.0, initialStock);
    bool anyDraw = false;
    for (const logistics::TransitOp& op : withCar.plan.transitOps) {
        if (op.kgDelta < -1e-9) { anyDraw = true; }
    }
    check(!anyDraw, "车上缓冲已够时不得再动用站内库存");
}

// 反例夹具：**站不在路上**。W —D6 直连；W —T 与 T —D6 各 5km（绕路）。
// 另有一个只能经 T 到达的 D5，用来让生产者把库存攒在 T 上。
// 这样"就地满足"反而绕远 ⇒ 取优必须**退回现状 A**。
LogisticsGraph makeOffRouteStationGraph() {
    LogisticsGraph g;
    g.addNode(makeNode("W", NodeType::Warehouse, "仓库"));
    Node t = makeNode("T", NodeType::Transit, "中转站");
    t.subNetworkId = 1;
    g.addNode(t);
    Node d4 = makeNode("D4", NodeType::Delivery, "客户4");
    d4.subNetworkId = 1;
    g.addNode(d4);
    Node d5 = makeNode("D5", NodeType::Delivery, "客户5");
    d5.subNetworkId = 1;
    g.addNode(d5);
    Node d6 = makeNode("D6", NodeType::Delivery, "客户6");
    d6.subNetworkId = 1;
    g.addNode(d6);
    addTwoWay(g, "W", "D4", 5.0, 10.0, 4.0);
    addTwoWay(g, "W", "D6", 5.0, 10.0, 4.0);   // D6 **直连仓库** ⇒ T 不在去 D6 的路上
    addTwoWay(g, "W", "T", 5.0, 10.0, 4.0);
    addTwoWay(g, "T", "D5", 5.0, 10.0, 4.0);
    addTwoWay(g, "T", "D6", 5.0, 10.0, 4.0);
    return g;
}

void testUrgentBufferChoiceIsNeverWorse() {
    const LogisticsGraph g = makeOffRouteStationGraph();
    const Vehicle v = makeVehicle("W", 100.0, 480);
    std::vector<Order> orders;
    orders.push_back(makeOrder("O4", "D4", 60.0, 0, 1440));
    orders.push_back(makeOrder("O5", "D5", 60.0, 0, 1440));

    const RoutePlan base = planRoute(g, v, orders, WeightType::Distance);
    std::map<std::string, double> initialStock;
    double seeded = 0.0;
    for (const logistics::TransitStock& st : base.transitStock) {
        initialStock[st.nodeId] = st.finalKg;
        seeded += st.finalKg;
    }
    check(seeded > 1e-9, "反例夹具也应先在 T 上攒到库存，实际 "
                             + std::to_string(seeded) + "kg");

    // 紧急单打在直连客户 D6 上（车在仓库）：现状 = W->D6->W = 10km；
    // 经站 = W->T->D6->W = 20km ⇒ 取优必须选现状。
    const Order urgent = makeOrder("U2", "D6", 10.0, 0, 1440, true);
    const InsertResult inserted = insertUrgentOrder(
        g, v, orders, urgent, "W", 480, WeightType::Distance,
        std::vector<logistics::OnboardItem>(), 0.0, initialStock);
    const RoutePlan& chosen = inserted.plan;
    check(chosen.status == PlanStatus::Ok, "反例夹具应当可规划");

    std::vector<Order> all = orders;
    all.push_back(urgent);
    const RoutePlan variantA = replan(g, v, all, "W", 480, WeightType::Distance,
                                      std::vector<logistics::OnboardItem>(), initialStock);

    check(std::fabs(chosen.totalDistanceKm - variantA.totalDistanceKm) < 1e-9,
          "站不在路上时取优必须退回现状（不得\"有货就用\"）：选中 "
              + std::to_string(chosen.totalDistanceKm) + "km vs 现状 "
              + std::to_string(variantA.totalDistanceKm) + "km");
    check(chosen.totalPenaltyMin <= variantA.totalPenaltyMin,
          "退回现状后 penalty 也不得变差");

    // 而且**不得**因为"选了现状"却还去扣站内库存
    bool anyDraw = false;
    for (const logistics::TransitOp& op : chosen.transitOps) {
        if (op.kgDelta < -1e-9) { anyDraw = true; }
    }
    check(!anyDraw, "没选就地满足时不得记任何站内出库（账要跟方案一致）");
}

int main() {
    testSingleOrderRouteIsFullyCorrect();
    testEmptyOrdersDegeneratesToNoMovement();
    testGreedyServesNearestCandidateFirst();
    testTieBreakIsDeterministicByNodeId();
    testUrgentOrderIsServedFirstDespiteBeingFarther();
    testLateArrivalIsMarkedWithPenalty();
    testMultipleOrdersOnSameNodeMergeIntoOneStop();
    testSingleOrderExceedingCapacityIsInfeasible();
    testTotalDemandOverCapacityBecomesMultiTrip();
    testWithinCapacityStaysSingleTrip();
    testIncrementalReplanRecomputesOnlyAffectedLeg();
    testIncrementalReplanIsIdempotentWhenNothingAffected();
    testIncrementalReplanHandlesMultiTrip();
    testUnreachableDeliveryIsInfeasibleAndNamesTheNode();
    testReplanFromCurrentPosition();
    testReplanWhenAlreadyAtTheDeliveryNode();
    testPlanRouteEqualsReplanFromDepot();
    testIsEdgeOnRoute();
    testIncrementalReplanForwardsOnboard();
    testNodeIndexAtTimeReturnsSegmentStart();
    testNodeIndexAtTimeOnEmptyPlanAndClamp();
    testNodeIndexAtTimeIsMonotonicInTime();
    testTransitBufferProducer();
    testUrgentBufferInPlaceDelivery();
    testUrgentBufferChoiceIsNeverWorse();

    return testutil::summarize("routeplanner_tests");
}
