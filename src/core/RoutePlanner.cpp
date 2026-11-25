#include "core/RoutePlanner.h"

#include <cmath>
#include <cstdio>
#include <cstddef>
#include <functional>
#include <limits>
#include <map>
#include <unordered_map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace logistics {

std::vector<std::string> Stop::sortedActionOrderIds() const {
    std::vector<std::string> ids;
    ids.reserve(pickups.size() + deliveries.size() + offloads.size() + reclaims.size()
                + transitUnloads.size() + transitPickups.size());
    for (const std::string& id : pickups) {
        ids.push_back(id);
    }
    for (const StopDelivery& d : deliveries) {
        ids.push_back(d.orderId);
    }
    for (const std::string& id : offloads) {
        ids.push_back(id);
    }
    for (const std::string& id : reclaims) {
        ids.push_back(id);
    }
    for (const std::string& id : transitUnloads) {
        ids.push_back(id);
    }
    for (const std::string& id : transitPickups) {
        ids.push_back(id);
    }
    // 手写插入排序（AGENTS §1：排序/选择的判定逻辑必须自己写）。
    for (std::size_t i = 1; i < ids.size(); ++i) {
        const std::string key = ids[i];
        std::size_t j = i;
        while (j > 0 && key < ids[j - 1]) {
            ids[j] = ids[j - 1];
            --j;
        }
        ids[j] = key;
    }
    return ids;
}

namespace {

const double kEps = 1e-9;

bool nearlyEq(double a, double b) {
    return std::fabs(a - b) <= 1e-9;
}

// 线性查找（AGENTS §1：容器自带查找不受限，但这里手写更直观且无 <algorithm> 依赖）。
bool contains(const std::vector<std::string>& v, const std::string& key) {
    for (const std::string& s : v) {
        if (s == key) {
            return true;
        }
    }
    return false;
}

// ---------------------------------------------------------------------------
// 全源代价矩阵（距离 / 耗时 / 成本）
//
// 为什么要有它：pair-insertion 每插一张单要评价 O(n^2) 个位置对，每个候选
// 重放 O(n) 段。若每段都现算最短路，默认 25 单就是上百万次 Dijkstra。
// 矩阵把「一段的三种权重」降到 O(1) 查表。
//
// **因子口径（MC-B05）**：矩阵里存的是**基础边权**（与图表示一致）；
// 该车缩放后的耗时/成本在取值时乘上 timeFactor / costFactor。
// 标量缩放不改变 argmin，所以「选哪条路」与基础权重一致，
// 而「累加进目标键的量」用的是缩放后的值——这正是 #19 §2 要求的重放口径。
// ---------------------------------------------------------------------------
struct CostMatrix {
    std::vector<std::string>           ids;
    // 每次 leg 查询都要按节点名查两次下标 ⇒ 用无序表（热路径上省掉 log n 次字符串比较）。
    std::unordered_map<std::string, std::size_t> index;
    // 每个维度一张「按该维度最优的路径」的三项代价表：
    //   distBy[w] / timeBy[w] / costBy[w]，下标 w = 0 距离 / 1 耗时 / 2 成本。
    //
    // 为什么必须按维度分开存：三策略各自选**自己维度最优**的那条路，
    // 而距离/耗时/成本三项指标必须从**同一条路**上读出。
    // 早先每个维度各读"该维度最优路径"的值，等于把三条不同的路拼在一起——
    // 距离策略的总距离与成本策略的总距离会相同，而两者的实际路线不同（真实 bug）。
    std::vector<double>                distBy[3];
    std::vector<double>                timeBy[3];
    std::vector<double>                costBy[3];

    void build(const LogisticsGraph& graph) {
        ids.clear();
        index.clear();
        for (const Node& n : graph.nodes()) {
            index[n.id] = ids.size();
            ids.push_back(n.id);
        }
        const std::size_t v = ids.size();
        const WeightType weights[3] = {WeightType::Distance, WeightType::Time, WeightType::Cost};
        for (int w = 0; w < 3; ++w) {
            distBy[w].assign(v * v, -1.0);
            timeBy[w].assign(v * v, -1.0);
            costBy[w].assign(v * v, -1.0);
        }
        for (std::size_t i = 0; i < v; ++i) {
            for (std::size_t j = 0; j < v; ++j) {
                for (int w = 0; w < 3; ++w) {
                    const PathResult r = shortestPath(graph, ids[i], ids[j], weights[w]);
                    if (!r.found) {
                        continue;   // 保持负数：不可达
                    }
                    // 三项指标都从**这一条路**上读。
                    double d = 0.0;
                    double t = 0.0;
                    double c = 0.0;
                    bool broken = false;
                    for (std::size_t k = 1; k < r.nodes.size(); ++k) {
                        const Edge* e = graph.findEdge(r.nodes[k - 1], r.nodes[k]);
                        if (e == nullptr) {
                            broken = true;
                            break;
                        }
                        d += e->distanceKm;
                        t += e->timeMin;
                        c += e->costYuan;
                    }
                    if (broken) {
                        continue;
                    }
                    distBy[w][i * v + j] = d;
                    timeBy[w][i * v + j] = t;
                    costBy[w][i * v + j] = c;
                }
            }
        }
    }

    bool known(const std::string& id) const { return index.find(id) != index.end(); }

    std::size_t idx(const std::string& id) const {
        const std::unordered_map<std::string, std::size_t>::const_iterator it = index.find(id);
        return it == index.end() ? ids.size() : it->second;
    }

    static int dim(WeightType w) {
        switch (w) {
            case WeightType::Distance: return 0;
            case WeightType::Time:     return 1;
            case WeightType::Cost:     return 2;
        }
        return 0;
    }

    bool reachable(const std::string& a, const std::string& b) const {
        const std::size_t i = idx(a);
        const std::size_t j = idx(b);
        if (i >= ids.size() || j >= ids.size()) {
            return false;
        }
        return distBy[0][i * ids.size() + j] >= 0.0;
    }

    // 沿**指定维度最优**的那条路，读三项指标。不可达返回负数。
    bool leg(const std::string& a, const std::string& b, WeightType w, double& dist, double& time,
             double& cost) const {
        const std::size_t i = idx(a);
        const std::size_t j = idx(b);
        if (i >= ids.size() || j >= ids.size()) {
            return false;
        }
        const std::size_t k = i * ids.size() + j;
        const int d = dim(w);
        if (distBy[d][k] < 0.0) {
            return false;
        }
        dist = distBy[d][k];
        time = timeBy[d][k];
        cost = costBy[d][k];
        return true;
    }

    // 一次查询拿齐「该策略最优路径」的三项指标与策略权重。
    // 原来 leg() 与 weightOf() 各查一次（两次 idx + 两次表读），合起来省一半。
    bool legAll(const std::string& a, const std::string& b, WeightType w, double& dist,
                double& time, double& cost, double& weight) const {
        const std::size_t i = idx(a);
        const std::size_t j = idx(b);
        if (i >= ids.size() || j >= ids.size()) {
            return false;
        }
        const std::size_t k = i * ids.size() + j;
        const int d = dim(w);
        if (distBy[d][k] < 0.0) {
            return false;
        }
        dist = distBy[d][k];
        time = timeBy[d][k];
        cost = costBy[d][k];
        weight = (d == 0) ? dist : (d == 1) ? time : cost;
        return true;
    }

    // 指定维度上的边权（用于目标键的权重合计）。
    double weightOf(const std::string& a, const std::string& b, WeightType w) const {
        double d = 0.0;
        double t = 0.0;
        double c = 0.0;
        if (!leg(a, b, w, d, t, c)) {
            return -1.0;
        }
        switch (w) {
            case WeightType::Distance: return d;
            case WeightType::Time:     return t;
            case WeightType::Cost:     return c;
        }
        return d;
    }

    // 基础图 A->B 的最短耗时（与策略无关；取货准时率判据用）。
    double shortestTime(const std::string& a, const std::string& b) const {
        double d = 0.0;
        double t = 0.0;
        double c = 0.0;
        if (!leg(a, b, WeightType::Time, d, t, c)) {
            return -1.0;
        }
        return t;
    }
};

// 该车缩放后的边权（#19 §2 / MC-B05）：耗时乘 timeFactor、成本乘 costFactor、
// 距离不变。等待与 penalty 不乘任何因子。
double scaledWeight(double base, WeightType w, const Vehicle& v) {
    if (base < 0.0) {
        return -1.0;
    }
    switch (w) {
        case WeightType::Distance: return base;
        case WeightType::Time:     return base * v.timeFactor;
        case WeightType::Cost:     return base * v.costFactor;
    }
    return base;
}

// 一个待服务的取送请求（**订单级**）。
struct Request {
    std::string orderId;
    std::string pickup;     // A
    std::string delivery;   // B
    double      kg = 0.0;
    int         ws = 0;
    int         we = 0;
    bool        urgent = false;
    bool        alreadyPicked = false;   // 货已在车（或已从站取回）：无取货动作
    bool        inStash = false;         // 货在中转站：需先取回
    std::size_t seq = 0;                 // FIFO 的插入序（同刻按它）
};

// 一次停靠的**计划描述**（构造期的真源；构造完成后转成 Stop）。
struct StopSpec {
    std::string              nodeId;
    std::vector<std::string> pickups;
    std::vector<std::string> deliveries;
    std::vector<std::string> offloads;
    std::vector<std::string> reclaims;
    std::vector<std::string> transitUnloads;   // 转运卸入（交给另一台车）
    std::vector<std::string> transitPickups;   // 转运取走（由另一台车卸入）
    StopPhase                phase = StopPhase::Normal;

    bool empty() const {
        return pickups.empty() && deliveries.empty() && offloads.empty() && reclaims.empty()
               && transitUnloads.empty() && transitPickups.empty();
    }
    // 只含中转站动作（降载 / 取回 / 转运卸入 / 转运取走），不含取送动作。
    bool isTransitOnly() const {
        return pickups.empty() && deliveries.empty()
               && (!offloads.empty() || !reclaims.empty() || !transitUnloads.empty()
                   || !transitPickups.empty());
    }
    // 排序后的动作订单 ID（tieKey 的第二个分量，见 #18 §5）。手写插入排序。
    std::vector<std::string> sortedActionIds() const {
        std::vector<std::string> ids;
        ids.reserve(pickups.size() + deliveries.size() + offloads.size() + reclaims.size()
                    + transitUnloads.size() + transitPickups.size());
        ids.insert(ids.end(), pickups.begin(), pickups.end());
        ids.insert(ids.end(), deliveries.begin(), deliveries.end());
        ids.insert(ids.end(), offloads.begin(), offloads.end());
        ids.insert(ids.end(), reclaims.begin(), reclaims.end());
        ids.insert(ids.end(), transitUnloads.begin(), transitUnloads.end());
        ids.insert(ids.end(), transitPickups.begin(), transitPickups.end());
        for (std::size_t i = 1; i < ids.size(); ++i) {
            const std::string key = ids[i];
            std::size_t j = i;
            while (j > 0 && key < ids[j - 1]) {
                ids[j] = ids[j - 1];
                --j;
            }
            ids[j] = key;
        }
        return ids;
    }

    bool hasAction(const std::string& orderId) const {
        for (const std::string& id : pickups) {
            if (id == orderId) return true;
        }
        for (const std::string& id : deliveries) {
            if (id == orderId) return true;
        }
        for (const std::string& id : offloads) {
            if (id == orderId) return true;
        }
        for (const std::string& id : reclaims) {
            if (id == orderId) return true;
        }
        for (const std::string& id : transitUnloads) {
            if (id == orderId) return true;
        }
        for (const std::string& id : transitPickups) {
            if (id == orderId) return true;
        }
        return false;
    }
};

// 构造上下文：一次构造过程中不变的东西。
struct Ctx {
    const LogisticsGraph* graph = nullptr;
    Vehicle               vehicle;
    WeightType            weight = WeightType::Distance;
    CostMatrix            matrix;
    std::string           startPos;
    int                   startTimeMin = 0;
    double                initialLoadKg = 0.0;
    double                capacityKg = 0.0;
    std::vector<OnboardItem> onboard;      // 此刻在车上的货
    std::string              stashStation; // 暂存站（空 = 无暂存）
    std::vector<std::string> stashOrderIds;
    std::vector<Request>  urgent;     // 已按 FIFO 排好
    std::vector<Request>  normal;     // 已按 EDD 排好
    std::unordered_map<std::string, Request> byId;
    // 订单 ID -> 该单在 byId 之外的稠密序号（0..n-1）。checkOrderingCore 每次候选
    // 都要按订单聚合动作，早先现建一次 unordered_map 的代价是 ~50 次分配 —— 在
    // 上百万次候选评价里这是纯开销。这里改成「一次建好 + 槽位表按代号清空」。
    std::unordered_map<std::string, std::size_t> orderIndex;

    double kgOf(const std::string& orderId) const {
        const std::unordered_map<std::string, Request>::const_iterator it = byId.find(orderId);
        return it == byId.end() ? 0.0 : it->second.kg;
    }
    int windowEndOf(const std::string& orderId) const {
        const std::unordered_map<std::string, Request>::const_iterator it = byId.find(orderId);
        return it == byId.end() ? 0 : it->second.we;
    }
    int windowStartOf(const std::string& orderId) const {
        const std::unordered_map<std::string, Request>::const_iterator it = byId.find(orderId);
        return it == byId.end() ? 0 : it->second.ws;
    }
};

// 字典序键（#18 §5）：(Σpenalty, 当前策略权重, tieKey)。
//
// 前两键每次评价都要算；第三键（停靠序列的节点 ID 与动作 ID 逐位比较）只在
// 前两键完全并列时才需要。因此这里**不物化 tie**：并列时由 compareRoutes
// 直接比较两份停靠序列（见下）。这样每次候选评价不再构造 O(n) 个 KeyStop
// 与同样多的 vector —— 那是 34 万次候选评价里最大的一笔分配开销。
// 字典序键（规格 §5 字面）：(Σ逐单 penalty, Σ该策略权重, tieKey)。
// 载重峰值**不进键**——它是报告指标（RouteMetrics::peakLoadKg），不是决策依据。
struct Key {
    int    penalty = 0;
    double weight = 0.0;
};

// <0 表示 a 优于 b；0 表示完全并列。
int compareKey(const Key& a, const Key& b) {
    if (a.penalty != b.penalty) {
        return a.penalty < b.penalty ? -1 : 1;
    }
    if (!nearlyEq(a.weight, b.weight)) {
        return a.weight < b.weight ? -1 : 1;
    }
    return 0;
}

// tieKey（#18 §5 第三键）：停靠序列的 (节点 ID, 排序后的动作订单 ID) 逐位比较，
// 再按序列长度收尾。只在 compareKey == 0 时才走到这里。
//
// 用「现算」而不是「预存」：并列是稀有情形，省下的是**每一次**候选评价的分配。
int compareTie(const std::vector<StopSpec>& a, const std::vector<StopSpec>& b) {
    const std::size_t n = a.size() < b.size() ? a.size() : b.size();
    for (std::size_t i = 0; i < n; ++i) {
        if (a[i].nodeId != b[i].nodeId) {
            return a[i].nodeId < b[i].nodeId ? -1 : 1;
        }
        const std::vector<std::string> x = a[i].sortedActionIds();
        const std::vector<std::string> y = b[i].sortedActionIds();
        const std::size_t m = x.size() < y.size() ? x.size() : y.size();
        for (std::size_t k = 0; k < m; ++k) {
            if (x[k] != y[k]) {
                return x[k] < y[k] ? -1 : 1;
            }
        }
        if (x.size() != y.size()) {
            return x.size() < y.size() ? -1 : 1;
        }
    }
    if (a.size() != b.size()) {
        return a.size() < b.size() ? -1 : 1;
    }
    return 0;
}

// 完整的三键比较：先 penalty、再 weight、最后 tieKey。
int compareRoutes(const std::vector<StopSpec>& a, const Key& ka,
                  const std::vector<StopSpec>& b, const Key& kb) {
    const int c = compareKey(ka, kb);
    if (c != 0) {
        return c;
    }
    return compareTie(a, b);
}

// 逐停靠的重放明细（只在最终建计划时收集，候选评价时为 nullptr 以省开销）。
struct StopDetail {
    int    rawArrivalMin = 0;
    int    waitMin = 0;
    int    arrivalMin = 0;
    double remainingLoadKg = 0.0;
    std::vector<StopDelivery> deliveries;
};

// 一次重放的结果（时间 / 载重 / penalty / 成本 / 键）。
struct Eval {
    bool        ok = false;
    std::string reason;
    int         penalty = 0;
    double      weight = 0.0;
    double      dist = 0.0;
    double      time = 0.0;
    double      cost = 0.0;
    int         returnArrival = 0;
    double      peakLoad = 0.0;
    double      loadIntegral = 0.0;
    Key         key;
    std::vector<StopDetail> details;
    std::map<std::string, int> orderArrival;   // 订单 -> 送达到达时刻（详细模式）
    std::map<std::string, int> orderPickup;    // 订单 -> 取货到达时刻（详细模式）
};

// 按给定停靠序列重放一遍（F1–F7 的可行性检查就在这一步里）。
//
// 停靠内恒为「先卸后装」；停靠零耗时；早到等待计入耗时、不计 penalty；
// penalty 逐单（每张送达单按自己的 we）。
//
// detail == true 时收集逐停靠明细（最终建计划用）；候选评价传 false 省开销。
bool replay(const Ctx& ctx, const std::vector<StopSpec>& specs, bool detail, Eval& out) {
    out = Eval();
    double load = ctx.initialLoadKg;
    double peak = load;
    double integral = 0.0;
    std::string pos = ctx.startPos;
    double elapsed = static_cast<double>(ctx.startTimeMin);
    out.key.penalty = 0;
    out.key.weight = 0.0;

    const std::size_t vCount = ctx.matrix.ids.size();

    for (std::size_t i = 0; i < specs.size(); ++i) {
        const StopSpec& s = specs[i];
        if (s.nodeId.empty()) {
            out.reason = "停靠缺少节点";
            return false;
        }
        if (pos != s.nodeId) {
            double bd = 0.0;
            double bt = 0.0;
            double bc = 0.0;
            double bw = 0.0;
            if (!ctx.matrix.legAll(pos, s.nodeId, ctx.weight, bd, bt, bc, bw)) {
                out.reason = "无法从 " + pos + " 到达 " + s.nodeId;
                return false;
            }
            const double tScaled = bt * ctx.vehicle.timeFactor;
            // 段内在车量恒定（停靠零耗时）⇒ 分段积分精确。
            integral += load * tScaled;
            out.dist += bd;
            out.time += tScaled;
            out.cost += bc * ctx.vehicle.costFactor;
            out.key.weight += scaledWeight(bw, ctx.weight, ctx.vehicle);
            elapsed += tScaled;
            pos = s.nodeId;
        }

        const double raw = elapsed;
        double maxWs = 0.0;
        bool hasWs = false;
        for (const std::string& oid : s.deliveries) {
            const int ws = ctx.windowStartOf(oid);
            if (!hasWs || static_cast<double>(ws) > maxWs) {
                maxWs = static_cast<double>(ws);
                hasWs = true;
            }
        }
        const double wait = (hasWs && maxWs > raw) ? (maxWs - raw) : 0.0;
        const double arrival = raw + wait;
        elapsed = arrival;

        // 逐单 penalty（停靠无合并窗口）。
        // 时间策略的键第二分量要能代表**用户可见的「总耗时」**（返回基地 − 发车，
        // 含早到等待，规格一 §6.1）。若只累加 leg 耗时，「最短耗时策略」会为了少跑
        // 几分钟而换来一小时的空等 —— 验收②「每条策略在自身目标上不劣于另两条」
        // 于是不成立。距离/成本两项指标本身不含等待，故只对 Time 加上等待。
        // （偏离记录：键的 leg 权重口径原写「所选最短路的总权重」，见提交信息。）
        if (ctx.weight == WeightType::Time && wait > 0.0) {
            out.key.weight += wait;
        }
        int stopPenalty = 0;
        std::vector<StopDelivery> delivered;
        delivered.reserve(s.deliveries.size());
        for (const std::string& oid : s.deliveries) {
            StopDelivery d;
            d.orderId = oid;
            const int we = ctx.windowEndOf(oid);
            const bool late = arrival > static_cast<double>(we) + kEps;
            d.late = late;
            d.penaltyMin = late ? static_cast<int>(std::lround(arrival - static_cast<double>(we)))
                                : 0;
            stopPenalty += d.penaltyMin;
            delivered.push_back(d);
            if (detail) {
                out.orderArrival[oid] = static_cast<int>(std::lround(arrival));
            }
        }
        out.key.penalty += stopPenalty;

        // 先全部卸（送达 + 降载），再全部装（取货 + 取回）。
        for (const std::string& oid : s.deliveries) {
            load -= ctx.kgOf(oid);
        }
        for (const std::string& oid : s.offloads) {
            load -= ctx.kgOf(oid);
        }
        for (const std::string& oid : s.transitUnloads) {
            load -= ctx.kgOf(oid);
        }
        if (load < -kEps) {
            out.reason = "在 " + s.nodeId + " 卸货后载重为负（" + std::to_string(load) + "kg）";
            return false;
        }
        if (load > peak) {
            peak = load;
        }
        for (const std::string& oid : s.pickups) {
            load += ctx.kgOf(oid);
        }
        for (const std::string& oid : s.reclaims) {
            load += ctx.kgOf(oid);
        }
        for (const std::string& oid : s.transitPickups) {
            load += ctx.kgOf(oid);
        }
        if (load > ctx.capacityKg + kEps) {
            out.reason = "在 " + s.nodeId + " 装货后载重 " + std::to_string(load)
                         + "kg 超过上限 " + std::to_string(ctx.capacityKg) + "kg";
            return false;
        }
        if (load > peak) {
            peak = load;
        }
        if (detail) {
            for (const std::string& oid : s.pickups) {
                out.orderPickup[oid] = static_cast<int>(std::lround(arrival));
            }
            for (const std::string& oid : s.reclaims) {
                out.orderPickup[oid] = static_cast<int>(std::lround(arrival));
            }
            // 转运取走：取货时刻就是站停靠到达（站无窗口、零耗时）。
            for (const std::string& oid : s.transitPickups) {
                out.orderPickup[oid] = static_cast<int>(std::lround(arrival));
            }
            StopDetail d;
            // 三个整数必须自洽：raw + wait == arrival（不变量 ⑧ 的验收就是逐位相加）。
            // raw / wait 各自四舍五入会差 1 分钟，因此 wait 取「到达 − 原始到达」。
            d.rawArrivalMin = static_cast<int>(std::lround(raw));
            d.arrivalMin = static_cast<int>(std::lround(arrival));
            d.waitMin = d.arrivalMin - d.rawArrivalMin;
            d.remainingLoadKg = load;
            d.deliveries = delivered;
            out.details.push_back(d);
        }
    }

    // 回仓库（首末空驶段计入分母，正是「能反映空驶」的原因）。
    if (pos != ctx.vehicle.startNodeId) {
        double bd = 0.0;
        double bt = 0.0;
        double bc = 0.0;
        double bw = 0.0;
        if (!ctx.matrix.legAll(pos, ctx.vehicle.startNodeId, ctx.weight, bd, bt, bc, bw)) {
            out.reason = "无法从 " + pos + " 返回仓库 " + ctx.vehicle.startNodeId;
            return false;
        }
        const double tScaled = bt * ctx.vehicle.timeFactor;
        integral += load * tScaled;
        out.dist += bd;
        out.time += tScaled;
        out.cost += bc * ctx.vehicle.costFactor;
        out.key.weight += scaledWeight(bw, ctx.weight, ctx.vehicle);
        elapsed += tScaled;
        pos = ctx.vehicle.startNodeId;
    }

    if (vCount > 0 && !ctx.matrix.known(pos)) {
        out.reason = "终点不在图中";
        return false;
    }

    out.ok = true;
    out.penalty = out.key.penalty;
    // 总耗时 = 返回仓库时刻 − 出发时刻（**含早到等待**，规格一 §6.1）。
    // 早先只累加了行驶时间，等待没进 totalTimeMin，与 returnArrivalMin 自相矛盾。
    out.time = elapsed - static_cast<double>(ctx.startTimeMin);
    out.returnArrival = static_cast<int>(std::lround(elapsed));
    out.peakLoad = peak;
    out.loadIntegral = integral;
    return true;
}

// 轻量评价：只关心 key（候选比较用）。
bool evalKey(const Ctx& ctx, const std::vector<StopSpec>& specs, Key& key, double* dist,
             double* time, double* cost) {
    Eval e;
    if (!replay(ctx, specs, false, e)) {
        return false;
    }
    key = e.key;
    if (dist != nullptr) {
        *dist = e.dist;
    }
    if (time != nullptr) {
        *time = e.time;
    }
    if (cost != nullptr) {
        *cost = e.cost;
    }
    return true;
}

// 每个候选都整条重放（#18 Q6=A：全量扫描不裁剪），保证不漏唯一可行位置。

// ---------------------------------------------------------------------------
// 构造器状态
// ---------------------------------------------------------------------------
struct Builder {
    const Ctx*           ctx = nullptr;
    std::vector<StopSpec> route;
    std::string           stashStation;      // 锁定的暂存站（空 = 无暂存）
    std::vector<std::string> stashOrders;    // 当前在站的订单
    std::vector<std::string> onboardOrders;  // 出发时就在车上（无取货动作）
    std::vector<Request>  pendingNormal;     // 待插入的普通单（EDD 序）
    std::vector<Request>  pendingUrgent;     // 待追加的紧急单（FIFO 序）
};

// 手写插入排序：普通单 EDD（窗口止升序，同窗止按订单 ID 升序）。
void sortEdd(std::vector<Request>& v) {
    for (std::size_t i = 1; i < v.size(); ++i) {
        const Request key = v[i];
        std::size_t j = i;
        while (j > 0 && (v[j - 1].we > key.we
                         || (v[j - 1].we == key.we && v[j - 1].orderId > key.orderId))) {
            v[j] = v[j - 1];
            --j;
        }
        v[j] = key;
    }
}

// 手写插入排序：紧急单 FIFO（到达事件刻升序，同刻按插入序）。
// 「到达事件刻」在数据上就是紧急单的窗口起（#24 R4：紧急单窗口 = [t, t+60]）；
// 已取未送的紧急单排在最前（它的取货早已发生，不得被新紧急单抢先）。
void sortFifo(std::vector<Request>& v) {
    for (std::size_t i = 1; i < v.size(); ++i) {
        const Request key = v[i];
        std::size_t j = i;
        while (j > 0) {
            const Request& p = v[j - 1];
            bool after = false;
            if (p.alreadyPicked != key.alreadyPicked) {
                after = !p.alreadyPicked;            // 已取未送优先
            } else if (p.ws != key.ws) {
                after = p.ws > key.ws;
            } else {
                after = p.seq > key.seq;
            }
            if (!after) {
                break;
            }
            v[j] = v[j - 1];
            --j;
        }
        v[j] = key;
    }
}

// 手写插入排序：kg 降序，同 kg 按订单 ID 升序（#18 Q8=A）。
void sortKgDesc(std::vector<Request>& v) {
    for (std::size_t i = 1; i < v.size(); ++i) {
        const Request key = v[i];
        std::size_t j = i;
        while (j > 0 && (v[j - 1].kg < key.kg - kEps
                         || (nearlyEq(v[j - 1].kg, key.kg) && v[j - 1].orderId > key.orderId))) {
            v[j] = v[j - 1];
            --j;
        }
        v[j] = key;
    }
}

// 相邻同节点、同阶段的停靠合并成一次（先卸后装由 Stop 的执行顺序保证）。
void normalize(std::vector<StopSpec>& specs) {
    bool changed = true;
    while (changed) {
        changed = false;
        for (std::size_t i = 0; i + 1 < specs.size(); ++i) {
            if (specs[i].nodeId != specs[i + 1].nodeId || specs[i].phase != specs[i + 1].phase) {
                continue;
            }
            StopSpec& a = specs[i];
            const StopSpec& b = specs[i + 1];
            for (const std::string& id : b.pickups) {
                if (!a.hasAction(id)) {
                    a.pickups.push_back(id);
                }
            }
            for (const std::string& id : b.deliveries) {
                if (!a.hasAction(id)) {
                    a.deliveries.push_back(id);
                }
            }
            for (const std::string& id : b.offloads) {
                if (!a.hasAction(id)) {
                    a.offloads.push_back(id);
                }
            }
            for (const std::string& id : b.reclaims) {
                if (!a.hasAction(id)) {
                    a.reclaims.push_back(id);
                }
            }
            for (const std::string& id : b.transitUnloads) {
                if (!a.hasAction(id)) {
                    a.transitUnloads.push_back(id);
                }
            }
            for (const std::string& id : b.transitPickups) {
                if (!a.hasAction(id)) {
                    a.transitPickups.push_back(id);
                }
            }
            specs.erase(specs.begin() + static_cast<std::ptrdiff_t>(i + 1));
            changed = true;
            break;
        }
    }
}

// 走一遍前 count 个停靠，返回「此刻在车」的订单 -> kg。
std::map<std::string, double> loadStateAt(const Ctx& ctx, const std::vector<StopSpec>& specs,
                                          std::size_t count, double* totalOut) {
    std::map<std::string, double> inCar;
    for (const OnboardItem& it : ctx.onboard) {
        if (!it.orderId.empty()) {
            inCar[it.orderId] = it.kg;
        }
    }
    // 出发前在站里的暂存货**不算在车**（#29 §1.3）。
    double load = 0.0;
    for (std::map<std::string, double>::const_iterator it = inCar.begin(); it != inCar.end();
         ++it) {
        load += it->second;
    }
    for (std::size_t i = 0; i < count && i < specs.size(); ++i) {
        const StopSpec& s = specs[i];
        for (const std::string& id : s.deliveries) {
            const std::map<std::string, double>::iterator f = inCar.find(id);
            if (f != inCar.end()) {
                load -= f->second;
                inCar.erase(f);
            }
        }
        for (const std::string& id : s.offloads) {
            const std::map<std::string, double>::iterator f = inCar.find(id);
            if (f != inCar.end()) {
                load -= f->second;
                inCar.erase(f);
            }
        }
        for (const std::string& id : s.pickups) {
            const double kg = ctx.kgOf(id);
            inCar[id] = kg;
            load += kg;
        }
        for (const std::string& id : s.reclaims) {
            const double kg = ctx.kgOf(id);
            inCar[id] = kg;
            load += kg;
        }
        for (const std::string& id : s.transitUnloads) {
            const std::map<std::string, double>::iterator f = inCar.find(id);
            if (f != inCar.end()) {
                load -= f->second;
                inCar.erase(f);
            }
        }
        for (const std::string& id : s.transitPickups) {
            const double kg = ctx.kgOf(id);
            inCar[id] = kg;
            load += kg;
        }
    }
    if (totalOut != nullptr) {
        *totalOut = load;
    }
    return inCar;
}

// F1–F6 的**纯结构**检查（不含 F7 的时间/可达，也不含不变量 7 的停靠唯一性）。
// 拆出来是为了让「重访是否被强制」能机械判定：把重访停靠的动作并进首次停靠，
// 若并进去之后结构不合法 ⇒ 重访是被强制的（#18 Q13=A 的原文判据）。
struct OrderSlots {
    std::size_t firstPick = 0;
    std::size_t firstDeliv = 0;
    std::size_t firstRecl = 0;
    int nPick = 0;
    int nDeliv = 0;
    int nRecl = 0;
};

bool checkOrderingCore(const Ctx& ctx, const Builder& b, const std::vector<StopSpec>& specs,
                       std::string& reason, bool includeFifo, bool skipRevisitRule = false) {
    // 每个订单的动作落点只记第一次出现的下标与次数（够 F1/F2/F6 用）。
    // 单表 + 无序查找：早先三张 std::map 的构造成本在每个候选中都要付一次，
    // 34 万次候选评价下它是纯开销（判定结果完全等价）。
    // 每次调用都新建 unordered_map = ~50 次分配；改成「复用的槽位表 + 代号戳」：
    // 第 gen 次调用写过的槽位才算有效，于是不需要清空整表。
    //
    // **重入纪律（改这里必须守住）**：槽位表是跨调用复用的，嵌套调用（下面
    // 「重访是否被强制」的合并评价会再进本函数）会把代号戳推进到新一代。
    // 因此**递归之后不得再读槽位**——本函数把 F1/F5 的槽位读取全部放在递归之前，
    // 递归之后只用局部变量。违反它不会立刻崩，只会静默把候选判成合法/非法。
    static std::vector<OrderSlots> s_slots;
    static std::vector<std::size_t> s_stamp;
    static std::size_t s_gen = 0;
    ++s_gen;
    if (s_slots.size() < ctx.orderIndex.size()) {
        s_slots.resize(ctx.orderIndex.size() + 32);
        s_stamp.resize(ctx.orderIndex.size() + 32, 0);
    }
    // 只返回**本次调用写过**的槽位：表是跨调用复用的，没写过就等价于"没有动作"。
    // （早先这里只看下标不看代号戳，会把上一次规划留下的旧数据当成当前候选的动作数，
    //   于是凭空报"送达不晚于取货"。）
    const auto slotOf = [&](const std::string& id) -> const OrderSlots* {
        const std::unordered_map<std::string, std::size_t>::const_iterator f =
            ctx.orderIndex.find(id);
        if (f == ctx.orderIndex.end() || f->second >= s_slots.size()
            || s_stamp[f->second] != s_gen) {
            return nullptr;
        }
        return &s_slots[f->second];
    };
    const auto freshSlot = [&](std::size_t k) -> OrderSlots& {
        if (s_stamp[k] != s_gen) {
            s_stamp[k] = s_gen;
            s_slots[k] = OrderSlots();
        }
        return s_slots[k];
    };
    const auto touch = [&](const std::string& id, std::size_t i, int kind) {
        const std::unordered_map<std::string, std::size_t>::const_iterator f =
            ctx.orderIndex.find(id);
        if (f == ctx.orderIndex.end()) {
            return;
        }
        OrderSlots& sl = freshSlot(f->second);
        if (kind == 0) {
            if (sl.nPick == 0) {
                sl.firstPick = i;
            }
            ++sl.nPick;
        } else if (kind == 1) {
            if (sl.nDeliv == 0) {
                sl.firstDeliv = i;
            }
            ++sl.nDeliv;
        } else {
            if (sl.nRecl == 0) {
                sl.firstRecl = i;
            }
            ++sl.nRecl;
        }
    };
    for (std::size_t i = 0; i < specs.size(); ++i) {
        for (const std::string& id : specs[i].pickups) {
            touch(id, i, 0);
        }
        for (const std::string& id : specs[i].deliveries) {
            touch(id, i, 1);
        }
        for (const std::string& id : specs[i].reclaims) {
            touch(id, i, 2);
        }
    }

    // F1 + F2 + F6：**已出现在路线里的**每张订单恰好服务一次，先取（或先取回）后送。
    // 注意：插入过程中路线只含一部分订单，因此这里不能要求「ctx 的全部订单都被服务」；
    // 完整性由 checkComplete 在最终路线上单独断言。
    // 遍历顺序固定为 urgent 后 normal（不用哈希表的迭代序），保证报错文案可复现。
    const OrderSlots kEmpty;
    for (int pass = 0; pass < 2; ++pass) {
        const std::vector<Request>& list = (pass == 0) ? ctx.urgent : ctx.normal;
        for (const Request& r : list) {
            const std::string& id = r.orderId;
            const OrderSlots* slp = slotOf(id);
            const OrderSlots& sl = (slp == nullptr) ? kEmpty : *slp;
            const bool inCar = contains(b.onboardOrders, id);
            const bool inStash = contains(b.stashOrders, id);
            if (sl.nDeliv > 1) {
                reason = "订单 " + id + " 的送达动作数超过 1";
                return false;
            }
            if (sl.nDeliv == 0) {
                continue;   // 还没安排送达（插入过程中的中间态）
            }
            if (inCar || inStash) {
                if (sl.nPick != 0) {
                    reason = "订单 " + id + " 已在车上/站里，不应再有取货动作";
                    return false;
                }
            } else if (sl.nPick != 1) {
                reason = "订单 " + id + " 的取货动作数不是 1";
                return false;
            }
            if (inStash) {
                if (sl.nRecl != 1) {
                    reason = "暂存单 " + id + " 的取回动作数不是 1";
                    return false;
                }
                if (sl.firstRecl >= sl.firstDeliv) {
                    reason = "暂存单 " + id + " 的取回晚于送达";
                    return false;
                }
            } else if (sl.nRecl != 0) {
                reason = "订单 " + id + " 不该有取回动作";
                return false;
            }
            // 已在车上的货没有取货动作：它的"起点"在计划开始之前，不参与次序比较。
            if (inCar) {
                continue;
            }
            const std::size_t first = (sl.nPick != 0) ? sl.firstPick : sl.firstRecl;
            if (first >= sl.firstDeliv) {
                reason = "订单 " + id + " 的送达不晚于取货/取回";
                return false;
            }
        }
    }

    // F2 后半：**单条连续路线**——车只在起点（仓库）出发一次、回仓库一次。
    // 路线中途再停一次仓库等于"回仓装货/分趟"，那是已退役的多趟语义（规格一 MC-A01/A02），
    // 也会让「中途不回仓库」这条验收断言失效。允许的唯一形态：出发位置本身就是该仓库时，
    // 仓库可以作为**首个**停靠（A 就在仓库取货的夹具形态；真实数据里 A 从不在仓库）。
    for (std::size_t i = 0; i < specs.size(); ++i) {
        if (specs[i].nodeId != ctx.vehicle.startNodeId) {
            continue;
        }
        const bool asFirstStopAtDepot = (i == 0 && ctx.startPos == ctx.vehicle.startNodeId);
        if (!asFirstStopAtDepot) {
            reason = "路线中途返回基地 " + specs[i].nodeId + "（单条连续路线不允许）";
            return false;
        }
    }

    // F4 硬优先：紧急停靠全部先于普通停靠。
    bool seenNormal = false;
    for (const StopSpec& s : specs) {
        if (s.phase == StopPhase::Normal) {
            seenNormal = true;
        } else if (seenNormal) {
            reason = "紧急停靠出现在普通停靠之后（硬优先被破坏）";
            return false;
        }
    }
    // F4：紧急阶段无普通取货动作。
    for (const StopSpec& s : specs) {
        if (s.phase != StopPhase::Urgent) {
            continue;
        }
        for (const std::string& id : s.pickups) {
            const std::unordered_map<std::string, Request>::const_iterator it = ctx.byId.find(id);
            if (it != ctx.byId.end() && !it->second.urgent) {
                reason = "紧急阶段出现了普通取货动作（" + id + "）";
                return false;
            }
        }
    }

    // F5 订单级 FIFO：U_k 的送达不晚于 U_{k+1} 的取货。
    for (std::size_t k = 0; includeFifo && k + 1 < ctx.urgent.size(); ++k) {
        const std::string& a = ctx.urgent[k].orderId;
        const std::string& c = ctx.urgent[k + 1].orderId;
        const OrderSlots* sap = slotOf(a);
        const OrderSlots* scp = slotOf(c);
        if (sap == nullptr || scp == nullptr) {
            continue;
        }
        const OrderSlots& sa = *sap;
        const OrderSlots& sc = *scp;
        if (sa.nDeliv == 0 || sc.nDeliv == 0) {
            continue;
        }
        const std::size_t da = sa.firstDeliv;
        const std::size_t pc = (sc.nPick == 0) ? sc.firstDeliv : sc.firstPick;
        if (da > pc) {
            reason = "紧急单 FIFO 被破坏：" + a + " 的送达晚于 " + c + " 的取货";
            return false;
        }
    }

    // F5 后半：**同阶段同节点至多一次停靠**；唯一例外是「动作无法并入首次停靠」。
    //
    // 判据（机械、可复核，不是"实现说了算"）：某节点在同阶段第二次出现时，
    //   · 该停靠只含**一个**动作（重访绝不合并多张单，保住合并收益）；
    //   · 把该动作并进第一次停靠后，F2（先取后送）或 F5（订单级 FIFO）必然被破坏。
    //     满足这条才算"被强制"，否则判不可行。
    //
    // 规格原文把例外写成「订单级 FIFO 强制重访」（#18 Q13=A）。它同时覆盖两种形态：
    //   ① A/B 次序强制（U2 的取货点撞 U1 的取货点 ⇒ 并入会让 U2 先于 U1 送达）；
    //   ② FIFO 强制（并入会让 U2 的取货早于 U1 的送达）。
    // 默认数据集里还有普通段的 A/B 互逆对（O01: D08→D01 与 O09: D01→D08），
    // 判据逐字不变，只是适用范围从紧急段扩到普通段（偏离已在提交信息里点名）。
    if (!skipRevisitRule) {
        // 两张表而不是「阶段前缀 + 节点 ID」拼字符串：后者每个停靠都要一次分配。
        std::unordered_map<std::string, std::size_t> urgentFirst;
        std::unordered_map<std::string, std::size_t> normalFirst;
        for (std::size_t i = 0; i < specs.size(); ++i) {
            const StopSpec& s = specs[i];
            const std::size_t actions = s.pickups.size() + s.deliveries.size()
                                        + s.offloads.size() + s.reclaims.size();
            // 阶段分表：跨阶段重访同一节点是明确允许的（#18 §4.1 / GLOSSARY）。
            std::unordered_map<std::string, std::size_t>& firstAt =
                (s.phase == StopPhase::Urgent) ? urgentFirst : normalFirst;
            const std::unordered_map<std::string, std::size_t>::const_iterator it =
                firstAt.find(s.nodeId);
            if (it == firstAt.end()) {
                firstAt[s.nodeId] = i;
                continue;
            }
            const std::size_t first = it->second;
            const StopSpec& head = specs[first];
            // 中转站动作（降载 / 取回）与取送停靠可以同节点共存：
            // 它们不是「一次访问服务多个订单」，而是两种不同性质的动作。
            if ((head.isTransitOnly() || s.isTransitOnly()) && !(head.isTransitOnly() && s.isTransitOnly())) {
                continue;
            }
            if (head.isTransitOnly() && s.isTransitOnly()) {
                reason = "同阶段同节点出现两次中转站停靠（" + s.nodeId + "）";
                return false;
            }
            if (actions != 1) {
                reason = "重访停靠必须只含一个动作（" + s.nodeId + "）";
                return false;
            }
            // 把重访停靠的这一个动作并进首次停靠，看结构是否仍然合法。
            std::vector<StopSpec> merged = specs;
            StopSpec& target = merged[first];
            if (!s.pickups.empty()) {
                target.pickups.push_back(s.pickups.front());
            } else if (!s.deliveries.empty()) {
                target.deliveries.push_back(s.deliveries.front());
            } else if (!s.offloads.empty()) {
                target.offloads.push_back(s.offloads.front());
            } else {
                target.reclaims.push_back(s.reclaims.front());
            }
            merged.erase(merged.begin() + static_cast<std::ptrdiff_t>(i));
            // 判据严格按规格原文：并入后 **F2（先取后送）/ F3（前缀载重、先卸后装）
            // / F5（紧急 FIFO）** 是否仍成立。**不再**在合并评价里递归跑"重访规则"
            // —— 那会让同一节点的两处重复互相"掩护"：A 并入后 B 仍是非强制重复，
            // 于是合并被判不可行，A 也被当成"强制"，重复就活下来了（实测 8 处全这样）。
            std::string mergeReason;
            Eval mergedEval;
            const bool mergeViable = replay(ctx, merged, false, mergedEval)
                                     && checkOrderingCore(ctx, b, merged, mergeReason, true, true);
            if (mergeViable) {
                reason = "节点 " + s.nodeId + " 在同阶段重复停靠，但该动作可以并入首次停靠（非强制重访）";
                return false;
            }
        }
    }

    // F6 暂存：全局至多一堆、锁站。
    for (const StopSpec& s : specs) {
        if (s.offloads.empty() && s.reclaims.empty()) {
            continue;
        }
        if (b.stashStation.empty()) {
            reason = "出现了降载/取回动作但没有锁定的暂存站";
            return false;
        }
        if (s.nodeId != b.stashStation) {
            reason = "降载/取回发生在非锁定站 " + s.nodeId;
            return false;
        }
    }
    return true;
}

// F1–F7 的结构性检查（F7 的时间/可达由 replay 负责）。
bool checkOrdering(const Ctx& ctx, const Builder& b, const std::vector<StopSpec>& specs,
                   std::string& reason) {
    return checkOrderingCore(ctx, b, specs, reason, true);
}

// 最终路线必须覆盖**全部**订单（F1 的另一半：不得丢单）。
bool checkComplete(const Ctx& ctx, const std::vector<StopSpec>& specs, std::string& reason) {
    std::map<std::string, int> delivered;
    for (const StopSpec& s : specs) {
        for (const std::string& id : s.deliveries) {
            ++delivered[id];
        }
    }
    for (const Request& r : ctx.urgent) {
        if (delivered[r.orderId] != 1) {
            reason = "订单 " + r.orderId + " 未被恰好送达一次";
            return false;
        }
    }
    for (const Request& r : ctx.normal) {
        if (delivered[r.orderId] != 1) {
            reason = "订单 " + r.orderId + " 未被恰好送达一次";
            return false;
        }
    }
    return true;
}

// 一个候选的完整评价：replay（F7）+ 结构检查（F1–F6）。
bool evaluate(const Ctx& ctx, const Builder& b, const std::vector<StopSpec>& specs, Eval& ev) {
    if (!replay(ctx, specs, false, ev)) {
        return false;
    }
    std::string reason;
    if (!checkOrdering(ctx, b, specs, reason)) {
        ev.ok = false;
        ev.reason = reason;
        return false;
    }
    return true;
}

// 紧急段末尾下标（最后一个紧急停靠之后）。
std::size_t urgentEnd(const std::vector<StopSpec>& specs) {
    std::size_t end = 0;
    for (std::size_t i = 0; i < specs.size(); ++i) {
        if (specs[i].phase == StopPhase::Urgent) {
            end = i + 1;
        }
    }
    return end;
}

// 普通段起点下标（第一个普通停靠）。
std::size_t normalBegin(const std::vector<StopSpec>& specs) {
    for (std::size_t i = 0; i < specs.size(); ++i) {
        if (specs[i].phase == StopPhase::Normal) {
            return i;
        }
    }
    return specs.size();
}

// 把动作并进既有停靠（同阶段同节点），返回是否并入成功。
bool mergeIntoExisting(std::vector<StopSpec>& specs, const std::string& nodeId, StopPhase phase,
                       const std::string& orderId, bool isPickup, bool isDelivery,
                       bool isOffload, bool isReclaim, std::size_t lo, std::size_t hi) {
    for (std::size_t i = lo; i < hi && i < specs.size(); ++i) {
        if (specs[i].nodeId != nodeId || specs[i].phase != phase) {
            continue;
        }
        if (specs[i].hasAction(orderId)) {
            continue;
        }
        if (isPickup) {
            specs[i].pickups.push_back(orderId);
        }
        if (isDelivery) {
            specs[i].deliveries.push_back(orderId);
        }
        if (isOffload) {
            specs[i].offloads.push_back(orderId);
        }
        if (isReclaim) {
            specs[i].reclaims.push_back(orderId);
        }
        return true;
    }
    return false;
}

// 把某个动作放进候选路线：mode = 0 并入既有停靠，mode = 1 新插一个停靠。
// 返回 false 表示该 mode 不适用（既有停靠不存在 / 该动作已在该停靠里）。
bool placeAction(std::vector<StopSpec>& specs, const std::string& nodeId, StopPhase phase,
                 const std::string& orderId, bool isPickup, bool isDelivery, bool isOffload,
                 bool isReclaim, std::size_t pos, int mode) {
    if (mode == 0) {
        return mergeIntoExisting(specs, nodeId, phase, orderId, isPickup, isDelivery, isOffload,
                                 isReclaim, 0, specs.size());
    }
    // 新插停靠前再查一次：A、B 同节点时两个 mode=1 会各插一个停靠，
    // 那等价于"同节点两次停靠"——若可合并则属于非强制重访，交给 checkOrdering 判掉；
    // 这里只保证不把同一个动作重复写入同一停靠。
    StopSpec s;
    s.nodeId = nodeId;
    s.phase = phase;
    if (isPickup) {
        s.pickups.push_back(orderId);
    }
    if (isDelivery) {
        s.deliveries.push_back(orderId);
    }
    if (isOffload) {
        s.offloads.push_back(orderId);
    }
    if (isReclaim) {
        s.reclaims.push_back(orderId);
    }
    specs.insert(specs.begin() + static_cast<std::ptrdiff_t>(pos), s);
    return true;
}

// 某个节点在该阶段是否已有停靠（决定「并入」这一 mode 是否可试）。
bool hasStopAt(const std::vector<StopSpec>& specs, const std::string& nodeId, StopPhase phase) {
    for (const StopSpec& s : specs) {
        if (s.nodeId == nodeId && s.phase == phase) {
            return true;
        }
    }
    return false;
}

// ---------------------------------------------------------------------------
// 就地候选评价（性能关键路径）
//
// 早先每个候选都复制整条路线：n 个停靠 × 4 个动作向量 ⇒ 每次 200 次分配，
// 34 万次候选评价下这份拷贝比算法本身还贵（实测占 1.21s 里的 1.16s）。
// 现在改成"就地放两个动作 → 评价 → 原样撤销"，只有确认改进时才真正复制。
//
// 撤销的顺序与记录：A 先放、B 后放，撤销时反过来（先 B 后 A），
// 这样撤 B 时 A 还在原处、撤 A 时 B 已消失 —— 两边下标都不用回头修正。
// ---------------------------------------------------------------------------
struct PairApply {
    bool        aPlaced = false;  // 动作是否真的放进去了（放失败时撤销必须跳过它）
    bool        bPlaced = false;
    bool        newA = false;     // true = A 是**新插**的停靠（撤销 = erase），false = 并入既有
    bool        newB = false;
    std::size_t posA = 0;         // A 动作落在的停靠下标（放置后）
    std::size_t posB = 0;
};

// 并入既有停靠（同阶段同节点）。index == kNoTarget 表示并入该节点的**第一个**停靠；
// 否则并入指定的那一个（同一节点可能因强制重访有多处停靠）。
bool mergeIntoStopAt(std::vector<StopSpec>& specs, const std::string& nodeId, StopPhase phase,
                     std::size_t index, const std::string& orderId, bool isPickup,
                     std::size_t* indexOut) {
    if (index == static_cast<std::size_t>(-1)) {
        for (std::size_t i = 0; i < specs.size(); ++i) {
            if (specs[i].nodeId == nodeId && specs[i].phase == phase
                && !specs[i].hasAction(orderId)) {
                index = i;
                break;
            }
        }
    }
    if (index >= specs.size() || specs[index].nodeId != nodeId
        || specs[index].phase != phase || specs[index].hasAction(orderId)) {
        return false;
    }
    if (isPickup) {
        specs[index].pickups.push_back(orderId);
    } else {
        specs[index].deliveries.push_back(orderId);
    }
    if (indexOut != nullptr) {
        *indexOut = index;
    }
    return true;
}

// 在 pos 处放一个动作；若**相邻**停靠已是同阶段同节点，则并入它
// （等价于 normalize 的相邻合并，但只做一次、只碰一个邻居）。
std::size_t insertActionAt(std::vector<StopSpec>& specs, const std::string& nodeId,
                           StopPhase phase, const std::string& orderId, bool isPickup,
                           std::size_t pos, bool* created) {
    if (pos > 0 && specs[pos - 1].nodeId == nodeId && specs[pos - 1].phase == phase
        && !specs[pos - 1].hasAction(orderId)) {
        if (isPickup) {
            specs[pos - 1].pickups.push_back(orderId);
        } else {
            specs[pos - 1].deliveries.push_back(orderId);
        }
        *created = false;
        return pos - 1;
    }
    if (pos < specs.size() && specs[pos].nodeId == nodeId && specs[pos].phase == phase
        && !specs[pos].hasAction(orderId)) {
        if (isPickup) {
            specs[pos].pickups.push_back(orderId);
        } else {
            specs[pos].deliveries.push_back(orderId);
        }
        *created = false;
        return pos;
    }
    StopSpec st;
    st.nodeId = nodeId;
    st.phase = phase;
    if (isPickup) {
        st.pickups.push_back(orderId);
    } else {
        st.deliveries.push_back(orderId);
    }
    specs.insert(specs.begin() + static_cast<std::ptrdiff_t>(pos), st);
    *created = true;
    return pos;
}

void undoPair(std::vector<StopSpec>& specs, const PairApply& rec, bool needPickup) {
    (void)needPickup;
    // 先撤 B（新插的 B 一定在 A 之后，erase 不会影响 A 的下标）。
    if (rec.bPlaced) {
        if (rec.newB) {
            specs.erase(specs.begin() + static_cast<std::ptrdiff_t>(rec.posB));
        } else {
            specs[rec.posB].deliveries.pop_back();
        }
    }
    // 再撤 A：B 已撤掉，A 的下标仍有效。没放进去的不要碰（否则会误 pop 别的动作）。
    if (rec.aPlaced) {
        if (rec.newA) {
            specs.erase(specs.begin() + static_cast<std::ptrdiff_t>(rec.posA));
        } else {
            specs[rec.posA].pickups.pop_back();
        }
    }
}

// 请求落位：**「同阶段同节点合并」是默认约束**（规格一不变量 ⑦）。
//
// 规则：只要某种「并入既有停靠」的落位**可行**，就不生成该节点的独立停靠候选；
// 只有当并入确实不可行（会让 F2 先取后送 / F3 前缀载重失败）时，才允许重访该节点
// —— 那才是规格里的「强制重访」，且重访停靠只含该单的一个动作。
//
// 因此候选按「并入优先」分四层求值，**第一层有可行候选就停**：
//   (A 并入, B 并入) → (A 并入, B 新插) → (A 新插, B 并入) → (A 新插, B 新插)
// 后两层只在前面全部不可行时才枚举。早先四层混在一起按 key 取优，于是
// 「新插一个同点停靠」会因为当时局部更省里程而胜出，把默认数据做成 31 站
// （其中 8 处同阶段重复都是这种次优，不是结构性必需）。
bool tryInsertPair(const Ctx& ctx, Builder& b, const Request& r,
                   std::vector<StopSpec>& best, Key& bestKey, bool mergeStrict = true) {
    const std::size_t lo = normalBegin(b.route);
    const std::size_t hi = b.route.size();
    const bool canMergeB = hasStopAt(b.route, r.delivery, StopPhase::Normal);
    // 已取未送 / 在站暂存的订单**没有取货动作**：货已在车上（或由取回动作装回），
    // 只需为它安排送达位。这与紧急段的 alreadyPicked 处理同一条规则。
    const bool needPickup = !r.alreadyPicked && !r.inStash;
    const bool canMergeA = needPickup && hasStopAt(b.route, r.pickup, StopPhase::Normal);
    bool found = false;
    std::vector<StopSpec>& route = b.route;

    // 评价一个（A 落位, B 落位）组合：就地放 → 评价 → 撤销；改进才复制。
    // mode = 0 表示并入 aTarget/bTarget（SIZE_MAX = 失败），mode = 1 表示在 i/j 新插。
    // 返回「该组合可行」，供「并入优先」的分层驱动判断这一层是否已经有候选。
    const auto attempt = [&](std::size_t i, std::size_t j, int modeA, int modeB,
                             std::size_t aTarget, std::size_t bTarget) -> bool {
        PairApply rec;
        if (needPickup) {
            if (modeA == 0) {
                if (!mergeIntoStopAt(route, r.pickup, StopPhase::Normal, aTarget,
                                     r.orderId, true, &rec.posA)) {
                    return false;
                }
                rec.aPlaced = true;
            } else {
                rec.posA = insertActionAt(route, r.pickup, StopPhase::Normal, r.orderId, true, i,
                                          &rec.newA);
                rec.aPlaced = true;
            }
        }
        // A 若新插在 B 的落点之前，B 的目标下标整体右移一位（并入目标同样会漂移）。
        const bool shiftB = needPickup && rec.newA && rec.posA <= j;
        const std::size_t jPos = j + (shiftB ? 1u : 0u);
        std::size_t bTargetShifted = bTarget;
        if (modeB == 0 && shiftB && bTargetShifted != static_cast<std::size_t>(-1)) {
            bTargetShifted += 1;
            if (bTargetShifted >= route.size()) {
                undoPair(route, rec, needPickup);
                return false;
            }
        }
        if (modeB == 0) {
            if (!mergeIntoStopAt(route, r.delivery, StopPhase::Normal, bTargetShifted,
                                 r.orderId, false, &rec.posB)) {
                undoPair(route, rec, needPickup);
                return false;
            }
            rec.bPlaced = true;
        } else {
            rec.posB = insertActionAt(route, r.delivery, StopPhase::Normal, r.orderId, false,
                                      jPos > route.size() ? route.size() : jPos, &rec.newB);
            rec.bPlaced = true;
        }
        if (needPickup && rec.posA >= rec.posB) {   // F2：先取后送
            undoPair(route, rec, needPickup);
            return false;
        }
        Eval ev;
        const bool ok = evaluate(ctx, b, route, ev);
        if (ok && (!found || compareRoutes(route, ev.key, best, bestKey) < 0)) {
            found = true;
            bestKey = ev.key;
            best = route;   // 只有确认改进时才复制整条路线
        }
        undoPair(route, rec, needPickup);
        return ok;
    };

    // 四层「并入优先」：前一层只要有可行候选就直接胜出，后一层不再枚举
    // （这就是「并入可行 ⇒ 不再生成该节点的独立停靠候选」；后两层只服务
    //  「强制重访」——并入会让 F2 先取后送 / F3 前缀载重失败）。
    const std::size_t kNone = static_cast<std::size_t>(-1);
    const bool canMergeA2 = needPickup && hasStopAt(route, r.pickup, StopPhase::Normal);
    const bool canMergeB2 = hasStopAt(route, r.delivery, StopPhase::Normal);
    bool any = false;

    if (!mergeStrict) {
        // 修复阶段：把「落位」当普通的局部搜索邻域 —— 四种组合全试、按 key 取优。
        // 合法性仍由 checkOrdering 把关（并入可行时独立停靠会被判成"非强制重访"而拒绝），
        // 所以这里放宽候选**不会**产生非强制重复，只是不让构造期的分层限制把修复卡死。
        if (needPickup && canMergeA2 && canMergeB2) {
            any = attempt(lo, lo, 0, 0, kNone, kNone) || any;
        }
        if (needPickup && canMergeA2) {
            for (std::size_t j = lo; j <= hi; ++j) {
                any = attempt(lo, j, 0, 1, kNone, kNone) || any;
            }
        }
        if (needPickup && canMergeB2) {
            for (std::size_t i = lo; i <= hi; ++i) {
                any = attempt(i, lo, 1, 0, kNone, kNone) || any;
            }
        }
        if (!needPickup) {
            for (std::size_t j = lo; j <= hi; ++j) {
                any = attempt(lo, j, 0, 1, kNone, kNone) || any;
            }
            return found;
        }
        for (std::size_t i = lo; i <= hi; ++i) {
            for (std::size_t j = i; j <= hi; ++j) {
                any = attempt(i, j, 1, 1, kNone, kNone) || any;
            }
        }
        return found;
    }

    // 第 1 层：A 并入、B 并入
    if (needPickup && canMergeA2 && canMergeB2) {
        any = attempt(lo, lo, 0, 0, kNone, kNone);
        if (any) {
            return found;
        }
    }
    // 第 2 层：A 并入、B 新插（位置枚举 j）
    if (needPickup && canMergeA2) {
        for (std::size_t j = lo; j <= hi; ++j) {
            any = attempt(lo, j, 0, 1, kNone, kNone) || any;
        }
        if (any) {
            return found;
        }
    }
    // 第 3 层：A 新插（位置枚举 i）、B 并入
    if (canMergeB2) {
        for (std::size_t i = lo; i <= hi; ++i) {
            any = attempt(i, lo, 1, 0, kNone, kNone) || any;
        }
        if (any) {
            return found;
        }
    }
    // 第 4 层：A 新插 + B 新插（最后的兜底 = 强制重访形态）
    if (!needPickup) {
        for (std::size_t j = lo; j <= hi; ++j) {
            any = attempt(lo, j, 0, 1, kNone, kNone) || any;
        }
        return found;
    }
    for (std::size_t i = lo; i <= hi; ++i) {
        for (std::size_t j = i; j <= hi; ++j) {
            any = attempt(i, j, 1, 1, kNone, kNone) || any;
        }
    }
    return found;
}

// 追加一张紧急单（订单级 FIFO）：在紧急段尾追加 A、B；同节点可并入既有紧急停靠，
// 并入违反 FIFO（或会让送达早于取货）时改为在段尾重访该节点（#18 Q13=A）。
bool appendUrgentPair(const Ctx& ctx, const Builder& b, const Request& r,
                      std::vector<StopSpec>& best, Key& bestKey) {
    const std::size_t at = urgentEnd(b.route);
    const bool canMergeB = hasStopAt(b.route, r.delivery, StopPhase::Urgent);
    const bool canMergeA = hasStopAt(b.route, r.pickup, StopPhase::Urgent);
    bool found = false;
    const int aModes = r.alreadyPicked ? 1 : 2;   // alreadyPicked 时只跑 modeA = 0 的空转
    for (int modeA = 0; modeA < aModes; ++modeA) {
        if (!r.alreadyPicked && modeA == 0 && !canMergeA) {
            continue;
        }
        for (int modeB = 0; modeB <= 1; ++modeB) {
            if (modeB == 0 && !canMergeB) {
                continue;
            }
            std::vector<StopSpec> cand = b.route;
            if (!placeAction(cand, r.delivery, StopPhase::Urgent, r.orderId, false, true, false,
                             false, at, modeB)) {
                continue;
            }
            if (!r.alreadyPicked) {
                if (!placeAction(cand, r.pickup, StopPhase::Urgent, r.orderId, true, false, false,
                                 false, at, modeA)) {
                    continue;
                }
            }
            normalize(cand);
            Eval ev;
            if (!evaluate(ctx, b, cand, ev)) {
                continue;
            }
            if (!found || compareRoutes(cand, ev.key, best, bestKey) < 0) {
                found = true;
                bestKey = ev.key;
                best = cand;
            }
        }
    }
    return found;
}

// 顺路交货（#18 §5）：紧急停靠节点上若恰有在车普通货的目的地，
// 且并入**不增加该停靠的等待**，就把送达动作并进去。
void addIncidental(std::vector<StopSpec>& specs, const Ctx& ctx, const Builder& b,
                   const std::string& nodeId) {
    for (std::size_t i = 0; i < specs.size(); ++i) {
        if (specs[i].nodeId != nodeId || specs[i].phase != StopPhase::Urgent) {
            continue;
        }
        // 该停靠原等待点 = 其现有送达单的最大 ws（无送达单则为 0）。
        double curWs = 0.0;
        bool has = false;
        for (const std::string& id : specs[i].deliveries) {
            const double ws = static_cast<double>(ctx.windowStartOf(id));
            if (!has || ws > curWs) {
                curWs = ws;
                has = true;
            }
        }
        // 找在车的普通货，且目的地恰为本节点。
        std::map<std::string, double> inCar = loadStateAt(ctx, specs, i, nullptr);
        for (std::map<std::string, double>::const_iterator it = inCar.begin();
             it != inCar.end(); ++it) {
            const std::unordered_map<std::string, Request>::const_iterator req = ctx.byId.find(it->first);
            if (req == ctx.byId.end() || req->second.urgent) {
                continue;
            }
            if (req->second.delivery != nodeId) {
                continue;
            }
            if (specs[i].hasAction(it->first)) {
                continue;
            }
            const double ws = static_cast<double>(req->second.ws);
            if (has && ws > curWs) {
                continue;   // 会推迟该停靠 → 不在本次交付，留到普通阶段
            }
            specs[i].deliveries.push_back(it->first);
            if (!has || ws > curWs) {
                curWs = ws;
                has = true;
            }
        }
        (void)b;
        return;
    }
}

// ---------------------------------------------------------------------------
// 降载与取回（#18 §11 / Q8–Q11）
// ---------------------------------------------------------------------------

// Q8=A 选货：在车普通单按 kg 降序（同 kg 按订单 ID 升序）取前缀，直到 Σkg >= need。
// 硬规则：只卸**整张普通订单**、绝不卸紧急货、不卸目的地恰为本次触发停靠点 A/B 的货。
bool chooseOffloadOrders(const Ctx& ctx, const Builder& b, const std::vector<StopSpec>& specs,
                         double need, const std::string& avoidA, const std::string& avoidB,
                         std::vector<std::string>& out) {
    out.clear();
    if (need <= kEps) {
        return true;
    }
    std::map<std::string, double> inCar = loadStateAt(ctx, specs, specs.size(), nullptr);
    std::vector<Request> pool;
    for (std::map<std::string, double>::const_iterator it = inCar.begin(); it != inCar.end();
         ++it) {
        const std::unordered_map<std::string, Request>::const_iterator req = ctx.byId.find(it->first);
        if (req == ctx.byId.end() || req->second.urgent) {
            continue;
        }
        if (!avoidA.empty() && req->second.delivery == avoidA) {
            continue;   // 留给顺路交货
        }
        if (!avoidB.empty() && req->second.delivery == avoidB) {
            continue;
        }
        pool.push_back(req->second);
    }
    sortKgDesc(pool);
    double freed = 0.0;
    for (const Request& r : pool) {
        if (freed >= need - kEps) {
            break;
        }
        out.push_back(r.orderId);
        freed += r.kg;
    }
    return freed >= need - kEps;
}

// 在给定站插入一次降载卸货停靠，位置为「触发取货停靠之前」。
// 具体做法：把该站插到紧急段尾（紧急阶段的降载停靠），普通阶段触发时插到普通段起点。
bool insertOffloadStop(const Ctx& ctx, const Builder& b, std::vector<StopSpec>& specs,
                       const std::string& station, const std::vector<std::string>& orderIds,
                       StopPhase phase) {
    std::size_t at = (phase == StopPhase::Urgent) ? urgentEnd(specs) : normalBegin(specs);
    // 已有同站同阶段的停靠 -> 并入。
    for (std::size_t i = 0; i < specs.size(); ++i) {
        if (specs[i].nodeId == station && specs[i].phase == phase) {
            for (const std::string& id : orderIds) {
                if (!specs[i].hasAction(id)) {
                    specs[i].offloads.push_back(id);
                }
            }
            return true;
        }
    }
    StopSpec s;
    s.nodeId = station;
    s.phase = phase;
    s.offloads = orderIds;
    specs.insert(specs.begin() + static_cast<std::ptrdiff_t>(at), s);
    (void)ctx;
    (void)b;
    return true;
}

// 在普通段内为「取回站」找一个位置：枚举普通段全部插入点，取 key 最小者。
// 硬约束：取回停靠必须先于每张暂存单的送达（由 checkOrdering 的 F6 保证）。
bool insertReclaimStop(const Ctx& ctx, const Builder& b, std::vector<StopSpec>& specs,
                       const std::string& station, const std::vector<std::string>& orderIds,
                       std::vector<StopSpec>& best, Key& bestKey) {
    const std::size_t lo = normalBegin(specs);
    const std::size_t hi = specs.size();
    bool found = false;
    for (std::size_t pos = lo; pos <= hi; ++pos) {
        std::vector<StopSpec> cand = specs;
        bool merged = false;
        for (std::size_t i = 0; i < cand.size(); ++i) {
            if (cand[i].nodeId == station && cand[i].phase == StopPhase::Normal) {
                for (const std::string& id : orderIds) {
                    if (!cand[i].hasAction(id)) {
                        cand[i].reclaims.push_back(id);
                    }
                }
                merged = true;
                break;
            }
        }
        if (!merged) {
            StopSpec s;
            s.nodeId = station;
            s.phase = StopPhase::Normal;
            s.reclaims = orderIds;
            cand.insert(cand.begin() + static_cast<std::ptrdiff_t>(pos), s);
        }
        normalize(cand);
        Eval ev;
        if (!evaluate(ctx, b, cand, ev)) {
            continue;
        }
        if (!found || compareRoutes(cand, ev.key, best, bestKey) < 0) {
            found = true;
            bestKey = ev.key;
            best = cand;
        }
    }
    return found;
}

// 候选中转站列表（NodeType::Transit，按节点 ID 升序 → Q9 的并列规则）。
std::vector<std::string> transitStations(const Ctx& ctx) {
    std::vector<std::string> out;
    if (ctx.graph == nullptr) {
        return out;
    }
    for (const Node& n : ctx.graph->nodes()) {
        if (n.type == NodeType::Transit) {
            out.push_back(n.id);
        }
    }
    for (std::size_t i = 1; i < out.size(); ++i) {
        const std::string key = out[i];
        std::size_t j = i;
        while (j > 0 && key < out[j - 1]) {
            out[j] = out[j - 1];
            --j;
        }
        out[j] = key;
    }
    return out;
}

// ---------------------------------------------------------------------------
// 构造主流程（#18 §6）
// ---------------------------------------------------------------------------
struct Outcome {
    bool                     ok = false;
    std::string              reason;
    std::vector<StopSpec>    route;
    std::string              stashStation;
    std::vector<std::string> stashOrders;
};

// 路线里是否已有该订单的送达动作。用于跳过「已由顺路交货送掉」的普通单：
// 紧急段在节点上顺路交付了在车普通货之后，该单的送达动作已经在路线里，
// 普通段若再插一次取送对，就会出现**同一订单两次送达**（实际是跑出 -20kg 的负载重）。
bool routeHasDelivery(const std::vector<StopSpec>& specs, const std::string& orderId) {
    for (const StopSpec& s : specs) {
        for (const std::string& id : s.deliveries) {
            if (id == orderId) {
                return true;
            }
        }
    }
    return false;
}

// 路线里是否已有该订单的取货动作（决定它是否还是"整对未插"的请求）。
bool routeHasPickup(const std::vector<StopSpec>& specs, const std::string& orderId) {
    for (const StopSpec& s : specs) {
        for (const std::string& id : s.pickups) {
            if (id == orderId) {
                return true;
            }
        }
    }
    return false;
}

// 从路线里摘掉某张订单的全部动作；空停靠一并删除。
void removeOrderActions(std::vector<StopSpec>& specs, const std::string& orderId) {
    for (std::size_t i = 0; i < specs.size(); ++i) {
        StopSpec& s = specs[i];
        const auto drop = [&orderId](std::vector<std::string>& v) {
            for (std::size_t k = v.size(); k > 0; --k) {
                if (v[k - 1] == orderId) {
                    v.erase(v.begin() + static_cast<std::ptrdiff_t>(k - 1));
                }
            }
        };
        drop(s.pickups);
        drop(s.deliveries);
        drop(s.offloads);
        drop(s.reclaims);
    }
    for (std::size_t i = specs.size(); i > 0; --i) {
        if (specs[i - 1].empty()) {
            specs.erase(specs.begin() + static_cast<std::ptrdiff_t>(i - 1));
        }
    }
}

// 从路线里摘掉一次取回停靠（只摘 reclaims，保留同停靠上的其它动作）。
void removeReclaims(std::vector<StopSpec>& specs, const std::string& station) {
    for (std::size_t i = 0; i < specs.size(); ++i) {
        if (specs[i].nodeId == station) {
            specs[i].reclaims.clear();
        }
    }
    for (std::size_t i = specs.size(); i > 0; --i) {
        if (specs[i - 1].empty()) {
            specs.erase(specs.begin() + static_cast<std::ptrdiff_t>(i - 1));
        }
    }
}

// 有界单请求 relocate（#18 §9 / Q3=B）：只动普通段，某轮无改进即停。
void repairRoute(const Ctx& ctx, Builder& b, int maxRounds) {
    for (int round = 0; round < maxRounds; ++round) {
        bool improved = false;
        // 普通单按 EDD 序逐个摘下重插。
        for (const Request& r : ctx.normal) {
            // 只搬运「普通段插入的整对」：出发时就在车上的货、以及已被顺路交付的
            // 只送达不取货，摘掉它们的动作会凭空造出一个取货动作（换的是别的解）。
            if (!routeHasPickup(b.route, r.orderId)) {
                continue;
            }
            std::vector<StopSpec> base = b.route;
            removeOrderActions(base, r.orderId);
            Builder probe = b;
            probe.route = base;
            std::vector<StopSpec> cand;
            Key candKey;
            if (!tryInsertPair(ctx, probe, r, cand, candKey, false)) {
                continue;
            }
            Eval cur;
            Eval alt;
            if (!evaluate(ctx, b, b.route, cur) || !evaluate(ctx, b, cand, alt)) {
                continue;
            }
            if (compareRoutes(cand, alt.key, b.route, cur.key) < 0) {
                b.route = cand;
                improved = true;
            }
        }
        // 取回停靠也作为单点请求参与 relocate（先于暂存单送达是硬约束）。
        if (!b.stashStation.empty() && !b.stashOrders.empty()) {
            bool present = false;
            for (const StopSpec& s : b.route) {
                if (s.nodeId == b.stashStation && !s.reclaims.empty()) {
                    present = true;
                    break;
                }
            }
            if (present) {
                std::vector<StopSpec> base = b.route;
                removeReclaims(base, b.stashStation);
                Builder probe = b;
                probe.route = base;
                std::vector<StopSpec> cand;
                Key candKey;
                if (insertReclaimStop(ctx, probe, base, b.stashStation, b.stashOrders, cand,
                                      candKey)) {
                    Eval cur;
                    Eval alt;
                    if (evaluate(ctx, b, b.route, cur) && evaluate(ctx, b, cand, alt)
                        && compareRoutes(cand, alt.key, b.route, cur.key) < 0) {
                        b.route = cand;
                        improved = true;
                    }
                }
            }
        }
        if (!improved) {
            break;
        }
    }
}


// 紧急段：按 FIFO 追加；装不下先降载（Q5=A 最后手段）。
bool appendUrgentAll(const Ctx& ctx, Builder& b, Outcome& out, bool allowOffload) {
    for (const Request& r : ctx.urgent) {
        if (r.kg > ctx.capacityKg + kEps) {
            std::ostringstream os;
            os << "订单 " << r.orderId << " 货量 " << static_cast<long long>(std::lround(r.kg))
               << "kg 超过载重上限 " << static_cast<long long>(std::lround(ctx.capacityKg))
               << "kg";
            out.reason = os.str();
            return false;
        }
        std::vector<StopSpec> cand;
        Key candKey;
        if (appendUrgentPair(ctx, b, r, cand, candKey)) {
            b.route = cand;
            addIncidental(b.route, ctx, b, r.pickup);
            addIncidental(b.route, ctx, b, r.delivery);
            // 顺路交货会改变 penalty/载重，重评一次；不可行就撤回交付动作。
            Eval ev;
            if (!evaluate(ctx, b, b.route, ev)) {
                for (std::size_t i = 0; i < b.route.size(); ++i) {
                    if (b.route[i].nodeId != r.pickup && b.route[i].nodeId != r.delivery) {
                        continue;
                    }
                    std::vector<std::string>& dv = b.route[i].deliveries;
                    for (std::size_t k = dv.size(); k > 0; --k) {
                        const std::unordered_map<std::string, Request>::const_iterator it =
                            ctx.byId.find(dv[k - 1]);
                        if (it != ctx.byId.end() && !it->second.urgent) {
                            dv.erase(dv.begin() + static_cast<std::ptrdiff_t>(k - 1));
                        }
                    }
                }
            }
            continue;
        }

        // 装不下 -> 降载（只卸整张普通单；不卸 A/B 目的地的货）。
        // 规格二 §6.4：降载属第 ④ 级的重手段 —— 第 ①②③ 级传 allowOffload=false，
        // 这里必须如实报「装不下」而不是偷偷降载（否则第 ④ 级永不可达）。
        if (!allowOffload) {
            out.reason = "紧急单 " + r.orderId + " 装不下（当前级不允许降载）";
            return false;
        }
        double loadNow = 0.0;
        loadStateAt(ctx, b.route, b.route.size(), &loadNow);
        const double need = loadNow + r.kg - ctx.capacityKg;
        std::vector<std::string> picked;
        if (!b.stashStation.empty()) {
            if (!chooseOffloadOrders(ctx, b, b.route, need, r.pickup, r.delivery, picked)) {
                out.reason = "紧急单 " + r.orderId + " 装不下，且锁站 " + b.stashStation
                             + " 上可卸的普通货不足";
                return false;
            }
            std::vector<StopSpec> cand2 = b.route;
            insertOffloadStop(ctx, b, cand2, b.stashStation, picked, StopPhase::Urgent);
            Builder probe = b;
            probe.route = cand2;
            std::vector<StopSpec> cand3;
            Key k3;
            if (!appendUrgentPair(ctx, probe, r, cand3, k3)) {
                out.reason = "紧急单 " + r.orderId + " 降载后仍无法追加";
                return false;
            }
            b.route = cand3;
            for (const std::string& id : picked) {
                b.stashOrders.push_back(id);
            }
            addIncidental(b.route, ctx, b, r.pickup);
            addIncidental(b.route, ctx, b, r.delivery);
            continue;
        }

        // 首次降载：Q9=A 构造式选站（每站构造整条路线，按 key 取优，并列按站 ID）。
        const std::vector<std::string> stations = transitStations(ctx);
        bool chosen = false;
        Key bestKey;
        std::vector<StopSpec> bestRoute;
        std::string bestStation;
        std::vector<std::string> bestPicked;
        for (const std::string& st : stations) {
            std::vector<std::string> pickedHere;
            if (!chooseOffloadOrders(ctx, b, b.route, need, r.pickup, r.delivery, pickedHere)) {
                continue;
            }
            std::vector<StopSpec> cand2 = b.route;
            insertOffloadStop(ctx, b, cand2, st, pickedHere, StopPhase::Urgent);
            Builder probe = b;
            probe.route = cand2;
            probe.stashStation = st;
            std::vector<StopSpec> cand3;
            Key k3;
            if (!appendUrgentPair(ctx, probe, r, cand3, k3)) {
                continue;
            }
            // 取回停靠也一并构造（否则整条路线还不完整）。
            Builder probe2 = probe;
            probe2.route = cand3;
            std::vector<StopSpec> cand4;
            Key k4;
            if (!insertReclaimStop(ctx, probe2, cand3, st, pickedHere, cand4, k4)) {
                continue;
            }
            const int c4 = chosen ? compareRoutes(cand4, k4, bestRoute, bestKey) : -1;
            if (!chosen || c4 < 0 || (c4 == 0 && st < bestStation)) {
                chosen = true;
                bestKey = k4;
                bestRoute = cand4;
                bestStation = st;
                bestPicked = pickedHere;
            }
        }
        if (!chosen) {
            out.reason = "紧急单 " + r.orderId + " 装不下，且没有任何可行的降载方案";
            return false;
        }
        b.route = bestRoute;
        b.stashStation = bestStation;
        b.stashOrders = bestPicked;
        addIncidental(b.route, ctx, b, r.pickup);
        addIncidental(b.route, ctx, b, r.delivery);
    }
    return true;
}

// 普通段：EDD 逐张插入；失败 -> repair 重试 -> 全量兜底 -> 降载 -> 拒绝。
bool insertNormalAll(const Ctx& ctx, Builder& b, Outcome& out, bool allowOffload) {
    for (const Request& r : ctx.normal) {
        if (r.kg > ctx.capacityKg + kEps) {
            std::ostringstream os;
            os << "订单 " << r.orderId << " 货量 " << static_cast<long long>(std::lround(r.kg))
               << "kg 超过载重上限 " << static_cast<long long>(std::lround(ctx.capacityKg))
               << "kg";
            out.reason = os.str();
            return false;
        }
        // 已在路线里被顺路交付掉（addIncidental）：普通段不再插取送对。
        if (routeHasDelivery(b.route, r.orderId)) {
            continue;
        }
        std::vector<StopSpec> cand;
        Key candKey;
        if (tryInsertPair(ctx, b, r, cand, candKey)) {
            b.route = cand;
            continue;
        }

        // ① 先修复现有路线，再试一次。
        Builder repaired = b;
        repairRoute(ctx, repaired, 3);
        if (tryInsertPair(ctx, repaired, r, cand, candKey)) {
            b = repaired;
            b.route = cand;
            continue;
        }

        // ② 降载（Q5=A 最后手段，普通取货同样适用）。
        // 只在「插入 + 修复 + 全量兜底」都失败后由 buildAll 打开这个开关。
        if (!allowOffload) {
            out.reason = "普通单 " + r.orderId + " 无可行插入位";
            return false;
        }
        double loadNow = 0.0;
        loadStateAt(ctx, b.route, b.route.size(), &loadNow);
        const double need = loadNow + r.kg - ctx.capacityKg;
        std::vector<std::string> picked;
        bool offloaded = false;
        const std::vector<std::string> stations =
            b.stashStation.empty() ? transitStations(ctx) : std::vector<std::string>{b.stashStation};
        Key bestKey;
        std::vector<StopSpec> bestRoute;
        std::string bestStation;
        std::vector<std::string> bestPicked;
        for (const std::string& st : stations) {
            std::vector<std::string> pickedHere;
            if (!chooseOffloadOrders(ctx, b, b.route, need, r.pickup, r.delivery, pickedHere)) {
                continue;
            }
            std::vector<StopSpec> cand2 = b.route;
            insertOffloadStop(ctx, b, cand2, st, pickedHere, StopPhase::Normal);
            Builder probe = b;
            probe.route = cand2;
            probe.stashStation = st;
            std::vector<StopSpec> cand3;
            Key k3;
            if (!tryInsertPair(ctx, probe, r, cand3, k3)) {
                continue;
            }
            Builder probe2 = probe;
            probe2.route = cand3;
            std::vector<std::string> mergedStash = b.stashOrders;
            for (const std::string& id : pickedHere) {
                mergedStash.push_back(id);
            }
            std::vector<StopSpec> cand4 = cand3;
            Key k4;
            bool haveReclaim = true;
            if (!b.stashOrders.empty()) {
                // 已有取回停靠：位置不变，只是取回集合变大。
                for (std::size_t i = 0; i < cand4.size(); ++i) {
                    if (cand4[i].nodeId == st && !cand4[i].reclaims.empty()) {
                        for (const std::string& id : pickedHere) {
                            if (!cand4[i].hasAction(id)) {
                                cand4[i].reclaims.push_back(id);
                            }
                        }
                        break;
                    }
                }
                haveReclaim = evalKey(ctx, cand4, k4, nullptr, nullptr, nullptr);
            } else {
                haveReclaim = insertReclaimStop(ctx, probe2, cand3, st, mergedStash, cand4, k4);
            }
            if (!haveReclaim) {
                continue;
            }
            const int c4 = offloaded ? compareRoutes(cand4, k4, bestRoute, bestKey) : -1;
            if (!offloaded || c4 < 0 || (c4 == 0 && st < bestStation)) {
                offloaded = true;
                bestKey = k4;
                bestRoute = cand4;
                bestStation = st;
                bestPicked = pickedHere;
            }
        }
        if (offloaded) {
            b.route = bestRoute;
            b.stashStation = bestStation;
            for (const std::string& id : bestPicked) {
                b.stashOrders.push_back(id);
            }
            continue;
        }

        out.reason = "普通单 " + r.orderId + " 无可行插入位，且兜底/降载均不可行";
        std::fprintf(stderr, "[dbg] insert fail %s alreadyPicked=%d inStash=%d route=%zu\n",
                     r.orderId.c_str(), (int)r.alreadyPicked, (int)r.inStash, b.route.size());
        return false;
    }
    return true;
}

// 主构造（#18 §6）：普通单按给定顺序逐张插入。
// order 为空 = 用 ctx.normal 的 EDD 序。
bool buildWithOrder(const Ctx& ctx, const std::vector<Request>& normalOrder,
                    bool allowOffload, Outcome& out) {
    out = Outcome();
    Builder b;
    b.ctx = &ctx;
    // 出发时就在车上的货（含已取未送的紧急单）不需要取货动作。
    for (const OnboardItem& it : ctx.onboard) {
        if (!it.orderId.empty()) {
            b.onboardOrders.push_back(it.orderId);
        }
    }
    b.stashStation = ctx.stashStation;
    b.stashOrders = ctx.stashOrderIds;

    if (!appendUrgentAll(ctx, b, out, allowOffload)) {
        out.ok = false;
        return false;
    }
    // 暂存货的取回：普通段的必做单点停靠（Q10=A），先于每张暂存单的送达。
    if (!b.stashStation.empty() && !b.stashOrders.empty()) {
        std::vector<StopSpec> cand;
        Key key;
        if (!insertReclaimStop(ctx, b, b.route, b.stashStation, b.stashOrders, cand, key)) {
            out.reason = "暂存站 " + b.stashStation + " 无法插入取回停靠";
            out.ok = false;
            return false;
        }
        b.route = cand;
    }
    // 用调用方给的顺序插入（EDD 序是默认；兜底时把失败的单提到最前）。
    Ctx ordered = ctx;
    ordered.normal = normalOrder.empty() ? ctx.normal : normalOrder;
    if (!insertNormalAll(ordered, b, out, allowOffload)) {
        out.ok = false;
        return false;
    }
    repairRoute(ctx, b, 3);

    Eval ev;
    if (!evaluate(ctx, b, b.route, ev)) {
        out.reason = ev.reason.empty() ? "最终路线不可行" : ev.reason;
        out.ok = false;
        return false;
    }
    std::string missing;
    if (!checkComplete(ctx, b.route, missing)) {
        out.reason = missing;
        out.ok = false;
        return false;
    }
    out.ok = true;
    out.route = b.route;
    out.stashStation = b.stashStation;
    out.stashOrders = b.stashOrders;
    return true;
}

// 普通段插入序的**起点版本**（规格内「两版按 key 取优」的推广，只用规格的
// 构造器与键，不改键序、不加启发式阈值）。
//
//   版 1：EDD（窗口止升序，同窗止按订单 ID）—— 规格 §4.5 的默认序。
//   版 2：**按取货点 A 聚簇**（簇按 A 的 ID 升序，簇内仍是 EDD）。
//         默认数据的天然结构就是三簇（D08 8 单 / D14 6 单 / D20 8 单）+ 若干
//         回程单（D01/D23 取货），EDD 会把同一取货点的单打散到不同时刻，
//         从而在容量上互相挤占；聚簇序让"整簇一起取、边取边送"成为可能。
//   版 3：**按送达点 B 聚簇**（B 的 ID 升序，簇内 EDD）—— 相近送达点一起插，
//         减少绕远端来回跑的里程/成本。（实测这一版是 ② 成本项的胜负手；
//         「簇内 kg 降序」也试过，在所有策略上都没赢过上述三版，故不留。）
//
// 每一版都跑满同一条链（构造 + repair R≤3 + 结构校验），可行者按**规格三键**
// (Σ逐单 penalty, Σ该策略权重, tieKey) 取优 —— 键序不变、策略语义不变。
// 单起点贪心会陷局部最优（实测纯 EDD 下成本策略被距离策略在成本上反超 23 元），
// 多起点是规格 §4.5「两版按 key 取优」的自然推广。
std::vector<std::vector<Request> > insertionStarts(const Ctx& ctx, bool multiStart) {
    std::vector<std::vector<Request> > starts;
    starts.push_back(ctx.normal);   // 版 1：EDD

    if (multiStart && ctx.normal.size() > 1) {
        // 取货点 A 的升序清单（手写插入排序，去重）
        std::vector<std::string> pickups;
        for (const Request& r : ctx.normal) {
            if (!contains(pickups, r.pickup)) {
                pickups.push_back(r.pickup);
            }
        }
        for (std::size_t i = 1; i < pickups.size(); ++i) {
            const std::string key = pickups[i];
            std::size_t j = i;
            while (j > 0 && key < pickups[j - 1]) {
                pickups[j] = pickups[j - 1];
                --j;
            }
            pickups[j] = key;
        }
        std::vector<Request> clustered;
        clustered.reserve(ctx.normal.size());
        for (const std::string& p : pickups) {
            for (const Request& r : ctx.normal) {
                if (r.pickup == p) {
                    clustered.push_back(r);   // 簇内保持 EDD 序
                }
            }
        }
        starts.push_back(clustered);

        // 版 5：**按送达点 B 聚簇**（B 的 ID 升序，簇内 EDD）—— 同一送达点附近
        // 的单一起插，减少"绕着远端来回跑"的里程/成本。
        std::vector<std::string> deliveries;
        for (const Request& r : ctx.normal) {
            if (!contains(deliveries, r.delivery)) {
                deliveries.push_back(r.delivery);
            }
        }
        for (std::size_t i = 1; i < deliveries.size(); ++i) {
            const std::string key = deliveries[i];
            std::size_t j = i;
            while (j > 0 && key < deliveries[j - 1]) {
                deliveries[j] = deliveries[j - 1];
                --j;
            }
            deliveries[j] = key;
        }
        std::vector<Request> byDelivery;
        byDelivery.reserve(ctx.normal.size());
        for (const std::string& b : deliveries) {
            for (const Request& r : ctx.normal) {
                if (r.delivery == b) {
                    byDelivery.push_back(r);
                }
            }
        }
        starts.push_back(byDelivery);
    }
    return starts;
}

// 主构造 + 全量兜底（#18 Q4=B）：
//   ① 按 EDD 序构造；
//   ② 失败时把「插不进去的那张单」提到最前再整体构造一次；
//   两版都可行则按 key 取优（penalty 是第一键，等价于「penalty 一票否决」）。
// 不做周期性全量重算：只有插入失败这一个触发点。
bool buildAll(const Ctx& ctx, Outcome& out, bool multiStart, bool allowOffload) {
    // ① 多起点构造：每版都跑满同一条链（构造 + repair R≤3），可行者按规格三键取优。
    const std::vector<std::vector<Request> > starts = insertionStarts(ctx, multiStart);
    Outcome first;            // 版 1（EDD）的结果：兜底链要用它的失败原因
    bool haveFirst = false;
    bool have = false;
    Key bestKey;
    for (std::size_t k = 0; k < starts.size(); ++k) {
        Outcome o;
        const bool ok = buildWithOrder(ctx, starts[k], false, o);
        if (k == 0) {
            first = o;
            haveFirst = true;
        }
        if (!ok) {
            continue;
        }
        Eval ev;
        if (!replay(ctx, o.route, false, ev)) {
            continue;
        }
        if (!have || compareRoutes(o.route, ev.key, out.route, bestKey) < 0) {
            out = o;
            bestKey = ev.key;
            have = true;
        }
    }
    if (have) {
        return true;
    }
    if (!haveFirst) {
        first.reason = "构造未产生候选";
    }

    // ② 全量兜底（#18 Q4=B）：把插不进去的那张单提到普通序最前，整体重跑构造
    // （不是"局部补丁"——buildWithOrder 会从当前位置/时刻/在车货重新排一遍）。
    // 两版都可行时按 key 取优（penalty 是第一键 ⇒ 等价于「penalty 一票否决」）。
    std::string failedId;
    for (const Request& r : ctx.normal) {
        if (first.reason.find(r.orderId) != std::string::npos) {
            failedId = r.orderId;
            break;
        }
    }
    if (!failedId.empty()) {
        std::vector<Request> reordered;
        for (const Request& r : ctx.normal) {
            if (r.orderId == failedId) {
                reordered.push_back(r);
            }
        }
        for (const Request& r : ctx.normal) {
            if (r.orderId != failedId) {
                reordered.push_back(r);
            }
        }
        Outcome second;
        if (buildWithOrder(ctx, reordered, false, second)) {
            out = second;
            return true;
        }
    }

    // ③ 降载（最后手段）：重跑一次，允许在普通段降载腾出空间。
    // allowOffload == false（规格二 §6.4 的第 ①②③ 级）时跳过：降载属第 ④ 级。
    if (!allowOffload) {
        out = first;
        return false;
    }
    Outcome third;
    if (buildWithOrder(ctx, std::vector<Request>(), true, third)) {
        out = third;
        return true;
    }

    // ④ 拒绝并报原因（绝不通过违反容量或先取后送来「凑」一个解）。
    out = first;
    return false;
}

// 把构造结果落成 RoutePlan：一次展平 nodes / nodeArrivalMin / nodeIsStop / stops / offloads。
void finalize(const Ctx& ctx, const Outcome& out, RoutePlan& plan) {
    plan = RoutePlan();
    plan.status = PlanStatus::Ok;

    std::vector<StopSpec> specs = out.route;
    Eval ev;
    replay(ctx, specs, true, ev);
    plan.stops.clear();
    plan.stops.reserve(specs.size());
    for (std::size_t i = 0; i < specs.size(); ++i) {
        Stop s;
        s.nodeId = specs[i].nodeId;
        s.pickups = specs[i].pickups;
        s.offloads = specs[i].offloads;
        s.reclaims = specs[i].reclaims;
        s.transitUnloads = specs[i].transitUnloads;
        s.transitPickups = specs[i].transitPickups;
        s.phase = specs[i].phase;
        if (i < ev.details.size()) {
            s.rawArrivalMin = ev.details[i].rawArrivalMin;
            s.waitMin = ev.details[i].waitMin;
            s.arrivalMin = ev.details[i].arrivalMin;
            s.remainingLoadKg = ev.details[i].remainingLoadKg;
            s.deliveries = ev.details[i].deliveries;
        }
        s.penaltyMin = 0;
        s.late = false;
        for (const StopDelivery& d : s.deliveries) {
            s.penaltyMin += d.penaltyMin;
            if (d.late) {
                s.late = true;
            }
        }
        plan.stops.push_back(s);
    }

    // 展平：从起点沿停靠序列走，把途经节点也写进 nodes（GUI 按下标推进依赖它）。
    plan.nodes.clear();
    plan.nodeArrivalMin.clear();
    plan.nodeIsStop.clear();
    plan.nodes.push_back(ctx.startPos);
    plan.nodeArrivalMin.push_back(ctx.startTimeMin);
    plan.nodeIsStop.push_back(false);
    std::string pos = ctx.startPos;
    double elapsed = static_cast<double>(ctx.startTimeMin);
    for (const Stop& s : plan.stops) {
        if (pos != s.nodeId) {
            const PathResult path = shortestPath(*ctx.graph, pos, s.nodeId, ctx.weight);
            if (!path.found) {
                plan.status = PlanStatus::Unreachable;
                plan.reason = "无法从 " + pos + " 到达 " + s.nodeId;
                return;
            }
            for (std::size_t k = 1; k < path.nodes.size(); ++k) {
                const Edge* e = ctx.graph->findEdge(path.nodes[k - 1], path.nodes[k]);
                if (e == nullptr) {
                    plan.status = PlanStatus::Unreachable;
                    plan.reason = "路径中间边缺失";
                    return;
                }
                elapsed += e->timeMin * ctx.vehicle.timeFactor;
                plan.nodes.push_back(path.nodes[k]);
                plan.nodeArrivalMin.push_back(static_cast<int>(std::lround(elapsed)));
                plan.nodeIsStop.push_back(k + 1 == path.nodes.size());
            }
            // 末节点的到达时刻用停靠的实际到达（含早到等待）。
            plan.nodeArrivalMin.back() = s.arrivalMin;
        } else if (plan.nodeIsStop.back()) {
            // 同一节点的**连续两次停靠**（典型：紧急阶段在 B 送紧急单，普通阶段又在
            // 同点送一张窗起点很晚的普通单 —— 两次到达时刻不同，不能并成一个停靠）。
            // 展平视图必须为第二次停靠再放一个节点条目，否则 nodeIsStop 的 true 数
            // 会小于 stops 数（展平视图与真源不一致）。零位移，无自环边。
            plan.nodes.push_back(s.nodeId);
            plan.nodeArrivalMin.push_back(s.arrivalMin);
            plan.nodeIsStop.push_back(true);
        } else {
            // 原地停靠（起点即首停靠）：不新增节点，只把停靠动作挂到起点。
            plan.nodeIsStop.back() = true;
            plan.nodeArrivalMin.back() = s.arrivalMin;
        }
        pos = s.nodeId;
        elapsed = static_cast<double>(s.arrivalMin);
    }
    // 回仓库。
    if (pos != ctx.vehicle.startNodeId) {
        const PathResult path =
            shortestPath(*ctx.graph, pos, ctx.vehicle.startNodeId, ctx.weight);
        if (!path.found) {
            plan.status = PlanStatus::Unreachable;
            plan.reason = "无法从 " + pos + " 返回仓库";
            return;
        }
        for (std::size_t k = 1; k < path.nodes.size(); ++k) {
            const Edge* e = ctx.graph->findEdge(path.nodes[k - 1], path.nodes[k]);
            if (e == nullptr) {
                plan.status = PlanStatus::Unreachable;
                plan.reason = "路径中间边缺失";
                return;
            }
            elapsed += e->timeMin * ctx.vehicle.timeFactor;
            plan.nodes.push_back(path.nodes[k]);
            plan.nodeArrivalMin.push_back(static_cast<int>(std::lround(elapsed)));
            plan.nodeIsStop.push_back(false);
        }
    }

    plan.totalDistanceKm = ev.dist;
    plan.totalTimeMin = ev.time;
    plan.totalCostYuan = ev.cost;
    plan.totalPenaltyMin = ev.penalty;
    plan.returnArrivalMin = ev.returnArrival;

    // 降载/取回账本（custody：订单、站、kg、卸下时刻、取回时刻）。
    plan.offloads.clear();
    for (std::size_t i = 0; i < specs.size(); ++i) {
        for (const std::string& id : specs[i].offloads) {
            OffloadLeg leg;
            leg.station = specs[i].nodeId;
            leg.orderId = id;
            const std::unordered_map<std::string, Request>::const_iterator it = ctx.byId.find(id);
            leg.deliverNodeId = (it != ctx.byId.end()) ? it->second.delivery : std::string();
            leg.kg = ctx.kgOf(id);
            leg.offloadTimeMin = plan.stops[i].arrivalMin;
            leg.reclaimTimeMin = -1;
            plan.offloads.push_back(leg);
        }
        // 转运动作的 kg（账本渲染用；订单需求，整单不可拆）。
        for (const std::string& id : specs[i].transitUnloads) {
            plan.transitKg[id] = ctx.kgOf(id);
        }
        for (const std::string& id : specs[i].transitPickups) {
            plan.transitKg[id] = ctx.kgOf(id);
        }
        for (const std::string& id : specs[i].reclaims) {
            for (OffloadLeg& leg : plan.offloads) {
                if (leg.orderId == id && leg.station == specs[i].nodeId
                    && leg.reclaimTimeMin < 0) {
                    leg.reclaimTimeMin = plan.stops[i].arrivalMin;
                    break;
                }
            }
        }
    }

    // ---- 指标（#29 §2 / 规格一 §6.1）----
    RouteMetrics& m = plan.metrics;
    m = RouteMetrics();
    m.stopCount = static_cast<int>(plan.stops.size());
    int pickupActions = 0;
    int deliveryActions = 0;
    for (const Stop& s : plan.stops) {
        pickupActions += static_cast<int>(s.pickups.size());
        deliveryActions += static_cast<int>(s.deliveries.size());
    }
    m.actionCount = pickupActions + deliveryActions;

    // 逐单指标：penalty / 准时（取货判据 = t_pick + shortestTime_vehicle(A->B) <= we）。
    //
    // **只统计「本车真的送达了」的订单**（不变量 8）：转运单的送达在取走车上，
    // 取货车只做了取货与站内卸下 —— 若照 ctx 订单表全算，同一张单会在两台车的
    // 指标里各出现一次（penalty / 两率的分母都翻倍）。
    std::vector<std::string> deliveredIds;
    for (const StopSpec& s : specs) {
        for (const std::string& id : s.deliveries) {
            deliveredIds.push_back(id);
        }
    }
    std::vector<std::string> ids;
    for (const Request& r : ctx.urgent) {
        bool delivered = false;
        for (const std::string& id : deliveredIds) {
            if (id == r.orderId) {
                delivered = true;
                break;
            }
        }
        if (delivered) {
            ids.push_back(r.orderId);
        }
    }
    for (const Request& r : ctx.normal) {
        bool delivered = false;
        for (const std::string& id : deliveredIds) {
            if (id == r.orderId) {
                delivered = true;
                break;
            }
        }
        if (delivered) {
            ids.push_back(r.orderId);
        }
    }
    for (const std::string& id : ids) {
        const std::unordered_map<std::string, Request>::const_iterator it = ctx.byId.find(id);
        if (it == ctx.byId.end()) {
            continue;
        }
        const Request& r = it->second;
        OrderMetric om;
        om.orderId = r.orderId;
        om.nodeId = r.delivery;
        om.urgent = r.urgent;
        om.windowEndMin = r.we;
        const std::map<std::string, int>::const_iterator dt = ev.orderArrival.find(id);
        om.deliveryArrivalMin = (dt != ev.orderArrival.end()) ? dt->second : 0;
        const std::map<std::string, int>::const_iterator pt = ev.orderPickup.find(id);
        om.pickupArrivalMin = (pt != ev.orderPickup.end()) ? pt->second : om.deliveryArrivalMin;
        om.late = om.deliveryArrivalMin > om.windowEndMin;
        om.penaltyMin = om.late ? (om.deliveryArrivalMin - om.windowEndMin) : 0;
        om.deliveryOnTime = !om.late;
        // 取货准时：取到货的那一刻，即使立刻直达也还赶得上窗口。
        const double abBase = ctx.matrix.shortestTime(r.pickup, r.delivery);
        const double abScaled = (abBase < 0.0) ? 0.0 : abBase * ctx.vehicle.timeFactor;
        om.pickupOnTime =
            (static_cast<double>(om.pickupArrivalMin) + abScaled
             <= static_cast<double>(om.windowEndMin) + kEps);
        m.orders.push_back(om);

        ++m.pickupTotal;
        if (om.pickupOnTime) {
            ++m.pickupOnTimeCount;
        }
        ++m.deliveryTotal;
        if (om.deliveryOnTime) {
            ++m.deliveryOnTimeCount;
        }
        if (om.urgent) {
            ++m.urgentTotal;
            if (om.deliveryOnTime) {
                ++m.urgentOnTimeCount;
            }
        } else {
            ++m.normalTotal;
            if (om.deliveryOnTime) {
                ++m.normalOnTimeCount;
            }
        }
    }

    // 时间加权在车量：段 = 相邻停靠（含首末空驶段）。
    m.peakLoadKg = ev.peakLoad;
    if (ev.time > kEps) {
        m.avgLoadKg = ev.loadIntegral / ev.time;
    } else {
        m.avgLoadKg = 0.0;
    }
    m.loadUtilization = (ctx.capacityKg > kEps) ? (m.avgLoadKg / ctx.capacityKg) : 0.0;
}

// 组装构造上下文：排序（紧急 FIFO / 普通 EDD）、建矩阵。
bool makeCtx(const LogisticsGraph& graph, const Vehicle& vehicle,
             const std::vector<Order>& orders, const std::string& startPos, int startTimeMin,
             WeightType weight, const std::vector<OnboardItem>& onboard, const StashState& stash,
             Ctx& ctx, std::string& reason) {
    ctx = Ctx();
    ctx.graph = &graph;
    ctx.vehicle = vehicle;
    ctx.weight = weight;
    ctx.startPos = startPos;
    ctx.startTimeMin = startTimeMin;
    ctx.capacityKg = vehicle.capacityKg;
    ctx.onboard = onboard;
    ctx.stashStation = stash.station;
    ctx.matrix.build(graph);

    if (!ctx.matrix.known(startPos)) {
        reason = "出发位置不存在: " + startPos;
        return false;
    }
    if (!ctx.matrix.known(vehicle.startNodeId)) {
        reason = "起始仓库不存在: " + vehicle.startNodeId;
        return false;
    }

    double onboardKg = 0.0;
    std::vector<std::string> onboardIds;
    for (const OnboardItem& it : onboard) {
        onboardKg += it.kg;
        if (!it.orderId.empty()) {
            onboardIds.push_back(it.orderId);
        }
    }
    ctx.initialLoadKg = onboardKg;

    std::size_t seq = 0;
    for (const Order& o : orders) {
        Request r;
        r.orderId = o.id;
        r.pickup = o.pickupNodeId;
        r.delivery = o.nodeId;
        r.kg = o.demandKg;
        r.ws = o.windowStartMin;
        r.we = o.windowEndMin;
        r.urgent = o.urgent;
        r.seq = seq++;
        if (r.pickup.empty()) {
            reason = "订单 " + o.id + " 缺少取货点 A";
            return false;
        }
        if (r.pickup == r.delivery) {
            reason = "订单 " + o.id + " 的取货点与送达点相同: " + r.pickup;
            return false;
        }
        if (r.kg <= kEps) {
            reason = "订单 " + o.id + " 的货量必须为正数";
            return false;
        }
        if (r.kg > vehicle.capacityKg + kEps) {
            std::ostringstream os;
            os << "订单 " << o.id << " 货量 " << static_cast<long long>(std::lround(r.kg))
               << "kg 超过载重上限 "
               << static_cast<long long>(std::lround(vehicle.capacityKg)) << "kg";
            reason = os.str();
            return false;
        }
        if (!ctx.matrix.known(r.pickup)) {
            reason = "订单 " + o.id + " 的取货点不存在: " + r.pickup;
            return false;
        }
        if (!ctx.matrix.known(r.delivery)) {
            reason = "订单 " + o.id + " 的送达点不存在: " + r.delivery;
            return false;
        }
        // 可达性按「起点 -> A」「起点 -> B」「B -> 仓库」三段分别判：
        // 报错必须点到**具体**走不到的那个节点（否则调用方无法定位）。
        if (!ctx.matrix.reachable(startPos, r.pickup)) {
            reason = "无法从 " + startPos + " 到达取货点: " + r.pickup;
            return false;
        }
        if (!ctx.matrix.reachable(startPos, r.delivery)) {
            reason = "无法从 " + startPos + " 到达送达点: " + r.delivery;
            return false;
        }
        if (!ctx.matrix.reachable(r.delivery, vehicle.startNodeId)) {
            reason = "无法从送达点 " + r.delivery + " 返回仓库: " + vehicle.startNodeId;
            return false;
        }
        // 已取未送 / 已在站：本计划里不需要取货动作。
        for (const std::string& id : onboardIds) {
            if (id == r.orderId) {
                r.alreadyPicked = true;
            }
        }
        for (const OnboardItem& it : stash.items) {
            if (it.orderId == r.orderId) {
                r.inStash = true;
                ctx.stashOrderIds.push_back(r.orderId);
            }
        }
        ctx.orderIndex[r.orderId] = ctx.orderIndex.size();
        ctx.byId[r.orderId] = r;
        if (r.urgent) {
            ctx.urgent.push_back(r);
        } else {
            ctx.normal.push_back(r);
        }
    }
    sortFifo(ctx.urgent);
    sortEdd(ctx.normal);
    return true;
}

} // namespace

// ---------------------------------------------------------------------------
// 公开入口
// ---------------------------------------------------------------------------
RoutePlan replan(const LogisticsGraph& graph,
                 const Vehicle& vehicle,
                 const std::vector<Order>& remainingOrders,
                 const std::string& currentPositionId,
                 int currentTimeMin,
                 WeightType weight,
                 const std::vector<OnboardItem>& onboard,
                 const StashState& stash,
                 bool allowOffload) {
    RoutePlan plan;
    Ctx ctx;
    std::string reason;
    if (!makeCtx(graph, vehicle, remainingOrders, currentPositionId, currentTimeMin, weight,
                 onboard, stash, ctx, reason)) {
        plan.status = (reason.find("超过载重上限") != std::string::npos)
                          ? PlanStatus::OrderExceedsCapacity
                          : PlanStatus::Unreachable;
        plan.reason = reason;
        return plan;
    }
    // 多起点只用于「从基地、空车、发车时刻」的**整批构造**（planRoute 语义）。
    // 中途重规划（事件/手动动作后的 replan）沿用 EDD 单起点：那条路径上决策次数多，
    // 多起点会把每次重规划的构造翻倍（--self-check-actions 的时间预算不允许），
    // 而且中途重规划的起点已被已执行前缀约束，聚簇序的收益很小。
    const bool multiStart = (currentPositionId == vehicle.startNodeId)
                            && (currentTimeMin == vehicle.departTimeMin)
                            && onboard.empty() && stash.station.empty();
    Outcome out;
    if (!buildAll(ctx, out, multiStart, allowOffload)) {
        plan.status = (out.reason.find("超过载重上限") != std::string::npos)
                          ? PlanStatus::OrderExceedsCapacity
                          : PlanStatus::Unreachable;
        plan.reason = out.reason;
        return plan;
    }
    finalize(ctx, out, plan);
    return plan;
}

RoutePlan planRoute(const LogisticsGraph& graph,
                    const Vehicle& vehicle,
                    const std::vector<Order>& orders,
                    WeightType weight) {
    return replan(graph, vehicle, orders, vehicle.startNodeId, vehicle.departTimeMin, weight,
                  std::vector<OnboardItem>(), StashState());
}

// ---- 类型感知分配的公共小件（规格二 §4 / #40）----
//
// 全排列枚举、排序、极值一律手写（AGENTS §1）：这里没有 std::sort / next_permutation /
// min_element / max_element。容器（vector / map）不在此限。
namespace {

const double kFleetEps = 1e-9;

bool nearlyEqDouble(double a, double b) { return std::fabs(a - b) < kFleetEps; }

// 该策略的键第二分量：距离 / 成本 = Σ 每车对应量；耗时维**含早到等待**（设计 §4.7-②），
// 即 RoutePlan::totalTimeMin（返回时刻 − 起始时刻，含等待）。
double strategyWeightOf(const RoutePlan& plan, WeightType weight) {
    switch (weight) {
        case WeightType::Distance: return plan.totalDistanceKm;
        case WeightType::Time:     return plan.totalTimeMin;
        case WeightType::Cost:     return plan.totalCostYuan;
    }
    return plan.totalDistanceKm;
}

// 车队均衡度（规格二 §4.3 / Q5 定稿）：max_v(峰值在车_v ÷ capacity_v) − min_v(…)。
// **分母口径 = 「全部车」**：0 单空车 utilization 记 0 并参与极差（否则「一台车干活、
// 两台空转」会被算成最均衡）。这是决策键第三级 + 报告指标，**不作硬约束**。
double fleetBalanceSpreadOf(const std::vector<Vehicle>& vehicles,
                            const std::vector<RoutePlan>& plans) {
    if (vehicles.empty() || plans.size() != vehicles.size()) {
        return 0.0;
    }
    double maxU = -1.0;
    double minU = -1.0;
    for (std::size_t v = 0; v < vehicles.size(); ++v) {
        double util = 0.0;
        if (vehicles[v].capacityKg > 0.0) {
            util = plans[v].metrics.peakLoadKg / vehicles[v].capacityKg;
        }
        if (maxU < 0.0 || util > maxU) {
            maxU = util;
        }
        if (minU < 0.0 || util < minU) {
            minU = util;
        }
    }
    return (maxU < 0.0) ? 0.0 : maxU - minU;
}

// 车队键（规格二 §4.3）：(Σ逐单 penalty, Σ策略权重, 均衡键, tieKey)。
// 前三级在这里；tieKey 由 compareFleetTie 现算（并列才需要，不物化）。
// **层次注意**：单车构造器内部仍是规格一三键（#33）；载重峰值不进单车键，
// 只有这里的车队层第三键用「均衡键」。
struct FleetKey {
    int    penalty = 0;
    double weight = 0.0;
    double balance = 0.0;
    double completion = 0.0;   // 只作阶段三的第二接受条件，不参与键序
    bool   allOk = true;
};

FleetKey fleetKeyOf(const std::vector<Vehicle>& vehicles, const std::vector<RoutePlan>& plans,
                    WeightType weight) {
    FleetKey k;
    const FleetPlan agg = aggregateFleet(vehicles, plans);
    // 第二键口径（§4.8 表下注 + #19 Q5 主口径）：
    //   · 距离 / 成本 = **Σ 每车**对应量（各车用自己的因子缩放过的量）；
    //   · 耗时维 = **完工耗时**（max 返回 − min 发车），不是 Σ 各车耗时。
    //     这是 #19 定下的车队主口径，也是「各自的自身目标」在验收里的口径；
    //     Σ 各车耗时是**并列的参考口径**，不拿来选排列。
    if (weight == WeightType::Time) {
        k.weight = agg.completionTimeMin;
    } else {
        double w = 0.0;
        for (std::size_t v = 0; v < plans.size(); ++v) {
            w += strategyWeightOf(plans[v], weight);
        }
        k.weight = w;
    }
    k.penalty = agg.totalPenaltyMin;
    k.balance = agg.balanceSpread;
    k.completion = agg.completionTimeMin;
    k.allOk = agg.allOk;
    return k;
}

// <0 = a 优于 b；0 = 前三键完全并列（tieKey 再裁决）。
int compareFleetKey(const FleetKey& a, const FleetKey& b) {
    if (a.penalty != b.penalty) {
        return a.penalty < b.penalty ? -1 : 1;
    }
    if (!nearlyEqDouble(a.weight, b.weight)) {
        return a.weight < b.weight ? -1 : 1;
    }
    if (!nearlyEqDouble(a.balance, b.balance)) {
        return a.balance < b.balance ? -1 : 1;
    }
    return 0;
}

// 车辆 ID 升序的行下标（同 ID 保持 [vehicles] 行序；手写插入排序）。
std::vector<std::size_t> vehiclesSortedById(const std::vector<Vehicle>& vehicles) {
    std::vector<std::size_t> idx(vehicles.size());
    for (std::size_t i = 0; i < idx.size(); ++i) {
        idx[i] = i;
    }
    for (std::size_t i = 1; i < idx.size(); ++i) {
        const std::size_t key = idx[i];
        std::size_t j = i;
        while (j > 0 && vehicles[key].id < vehicles[idx[j - 1]].id) {
            idx[j] = idx[j - 1];
            --j;
        }
        idx[j] = key;
    }
    return idx;
}

// tieKey（§4.3）：车辆 ID 升序 → 各车停靠序列逐位（节点 ID，动作订单 ID 排序后）。
// 车辆 ID 这一层由调用方按 byId 顺序传进来；完全并列时由调用方再比枚举序。
int compareFleetTie(const std::vector<RoutePlan>& a, const std::vector<RoutePlan>& b,
                    const std::vector<std::size_t>& byId) {
    for (std::size_t t = 0; t < byId.size(); ++t) {
        const std::vector<Stop>& sa = a[byId[t]].stops;
        const std::vector<Stop>& sb = b[byId[t]].stops;
        const std::size_t n = sa.size() < sb.size() ? sa.size() : sb.size();
        for (std::size_t i = 0; i < n; ++i) {
            if (sa[i].nodeId != sb[i].nodeId) {
                return sa[i].nodeId < sb[i].nodeId ? -1 : 1;
            }
            const std::vector<std::string> x = sa[i].sortedActionOrderIds();
            const std::vector<std::string> y = sb[i].sortedActionOrderIds();
            const std::size_t m = x.size() < y.size() ? x.size() : y.size();
            for (std::size_t k = 0; k < m; ++k) {
                if (x[k] != y[k]) {
                    return x[k] < y[k] ? -1 : 1;
                }
            }
            if (x.size() != y.size()) {
                return x.size() < y.size() ? -1 : 1;
            }
        }
        if (sa.size() != sb.size()) {
            return sa.size() < sb.size() ? -1 : 1;
        }
    }
    return 0;
}

// EDD：窗口止升序，同窗止按订单 ID 升序（§5.2）。手写插入排序。
void sortOrdersEdd(std::vector<Order>& orders) {
    for (std::size_t i = 1; i < orders.size(); ++i) {
        const Order key = orders[i];
        std::size_t j = i;
        while (j > 0 && (key.windowEndMin < orders[j - 1].windowEndMin
                         || (key.windowEndMin == orders[j - 1].windowEndMin
                             && key.id < orders[j - 1].id))) {
            orders[j] = orders[j - 1];
            --j;
        }
        orders[j] = key;
    }
}

// 该单现在在哪台车（行下标）；-1 = 没有车。
int vehicleOfOrder(const std::vector<std::vector<Order> >& vehicleOrders,
                   const std::string& orderId) {
    for (std::size_t v = 0; v < vehicleOrders.size(); ++v) {
        for (std::size_t i = 0; i < vehicleOrders[v].size(); ++i) {
            if (vehicleOrders[v][i].id == orderId) {
                return static_cast<int>(v);
            }
        }
    }
    return -1;
}

bool containsOrder(const std::vector<Order>& orders, const std::string& orderId) {
    for (std::size_t i = 0; i < orders.size(); ++i) {
        if (orders[i].id == orderId) {
            return true;
        }
    }
    return false;
}

std::vector<Order> withoutOrder(const std::vector<Order>& orders, const std::string& orderId) {
    std::vector<Order> out;
    out.reserve(orders.size());
    for (std::size_t i = 0; i < orders.size(); ++i) {
        if (orders[i].id != orderId) {
            out.push_back(orders[i]);
        }
    }
    return out;
}

// 边际代价：候选 vs 现状。balanceDelta = 本车峰值利用率的变化（本车容量口径）。
MarginalCost marginalCostBetween(const RoutePlan& current, const RoutePlan& candidate,
                                 const Vehicle& vehicle, WeightType weight) {
    MarginalCost mc;
    mc.penaltyDelta = static_cast<double>(candidate.totalPenaltyMin - current.totalPenaltyMin);
    mc.weightDelta = strategyWeightOf(candidate, weight) - strategyWeightOf(current, weight);
    double curUtil = 0.0;
    double candUtil = 0.0;
    if (vehicle.capacityKg > 0.0) {
        curUtil = current.metrics.peakLoadKg / vehicle.capacityKg;
        candUtil = candidate.metrics.peakLoadKg / vehicle.capacityKg;
    }
    mc.balanceDelta = candUtil - curUtil;
    return mc;
}

// Δkey 三元组按字典序比较。
int compareMarginal(const MarginalCost& a, const MarginalCost& b) {
    if (!nearlyEqDouble(a.penaltyDelta, b.penaltyDelta)) {
        return a.penaltyDelta < b.penaltyDelta ? -1 : 1;
    }
    if (!nearlyEqDouble(a.weightDelta, b.weightDelta)) {
        return a.weightDelta < b.weightDelta ? -1 : 1;
    }
    if (!nearlyEqDouble(a.balanceDelta, b.balanceDelta)) {
        return a.balanceDelta < b.balanceDelta ? -1 : 1;
    }
    return 0;
}

// 全排列枚举：groups[0..] 依次选一台**未被占用**的车，车辆按 ID 升序尝试。
// 默认 3 簇 3 车 ⇒ 6 种；未选中的车留空（groups < 车数时）。
void enumerateAssignments(std::size_t g, std::size_t groupCount,
                          const std::vector<std::size_t>& byId, std::vector<int>& choice,
                          std::vector<bool>& used, std::vector<std::vector<int> >& out) {
    if (g == groupCount) {
        out.push_back(choice);
        return;
    }
    for (std::size_t t = 0; t < byId.size(); ++t) {
        const std::size_t v = byId[t];
        if (used[v]) {
            continue;
        }
        used[v] = true;
        choice[g] = static_cast<int>(v);
        enumerateAssignments(g + 1, groupCount, byId, choice, used, out);
        used[v] = false;
    }
}

// 把「订单 -> 车辆行下标」的分配结果补齐到 FleetPlan 的产物字段。
// 未被任何车服务的订单下标记 vehicles.size()（哨兵），原因在 allocationLog。
void fillAllocationFields(FleetPlan& fleet, const LogisticsGraph& graph,
                          const std::vector<Vehicle>& vehicles,
                          const std::vector<Order>& allOrders,
                          const std::vector<std::vector<Order> >& vehicleOrders) {
    const std::size_t k = vehicles.size();
    fleet.vehicleOrderCount.assign(k, 0);
    fleet.vehicleClusters.assign(k, std::vector<int>());
    fleet.vehicleCrossClusterCount.assign(k, 0);

    std::map<std::string, std::size_t> where;
    for (std::size_t v = 0; v < vehicleOrders.size() && v < k; ++v) {
        fleet.vehicleOrderCount[v] = static_cast<int>(vehicleOrders[v].size());
        for (std::size_t i = 0; i < vehicleOrders[v].size(); ++i) {
            const Order& o = vehicleOrders[v][i];
            where[o.id] = v;
            const int cluster = clusterOfOrder(graph, o);
            if (cluster <= 0 || isCrossCluster(graph, o)) {
                fleet.vehicleCrossClusterCount[v] += 1;
                continue;
            }
            bool seen = false;
            for (std::size_t c = 0; c < fleet.vehicleClusters[v].size(); ++c) {
                if (fleet.vehicleClusters[v][c] == cluster) {
                    seen = true;
                }
            }
            if (!seen) {
                std::size_t pos = 0;
                while (pos < fleet.vehicleClusters[v].size()
                       && fleet.vehicleClusters[v][pos] < cluster) {
                    ++pos;
                }
                fleet.vehicleClusters[v].insert(fleet.vehicleClusters[v].begin() + pos, cluster);
            }
        }
    }

    // 分配表按订单 ID 升序（与 --dump-plan 的 ALLOC 行同序）。
    std::vector<std::size_t> idx(allOrders.size());
    for (std::size_t i = 0; i < idx.size(); ++i) {
        idx[i] = i;
    }
    for (std::size_t i = 1; i < idx.size(); ++i) {
        const std::size_t key = idx[i];
        std::size_t j = i;
        while (j > 0 && allOrders[key].id < allOrders[idx[j - 1]].id) {
            idx[j] = idx[j - 1];
            --j;
        }
        idx[j] = key;
    }
    fleet.allocationOrderIds.clear();
    fleet.allocationVehicleIndex.clear();
    fleet.allocationClusterId.clear();
    fleet.allocationOrderIds.reserve(allOrders.size());
    fleet.allocationVehicleIndex.reserve(allOrders.size());
    fleet.allocationClusterId.reserve(allOrders.size());
    for (std::size_t i = 0; i < idx.size(); ++i) {
        const Order& o = allOrders[idx[i]];
        const std::map<std::string, std::size_t>::const_iterator it = where.find(o.id);
        fleet.allocationOrderIds.push_back(o.id);
        fleet.allocationVehicleIndex.push_back(it == where.end() ? k : it->second);
        int cluster = -1;
        if (it != where.end()) {
            const int c = clusterOfOrder(graph, o);
            if (c > 0 && !isCrossCluster(graph, o)) {
                cluster = c;
            }
        }
        fleet.allocationClusterId.push_back(cluster);
    }
}

} // namespace

// ---- 车队规划（规格二 §3 / #39）----
//
// 汇总口径（§3.4 / §3.6-8）：Σ 每车 == 车队；完工耗时 = max(返回) − min(发车)。
// 均衡度也在这里算（**车队汇总只有这一处实现**，GUI 的均衡度行直接读它）。
FleetPlan aggregateFleet(const std::vector<Vehicle>& vehicles,
                         const std::vector<RoutePlan>& plans) {
    FleetPlan fleet;
    if (vehicles.empty() || vehicles.size() != plans.size()) {
        fleet.allOk = false;
        fleet.reason = "车辆与计划不匹配";
        return fleet;
    }
    fleet.vehicles = plans;
    int minDepartMin = vehicles[0].departTimeMin;
    // 完工耗时按**实值**返回时刻算：返回时刻 = 该车发车时刻 + 该车耗时
    // （= 计划里的 totalTimeMin；整数 returnArrivalMin 只是它的显示取整）。
    // 单车时这正好等于 plan.totalTimeMin，与规格一的「总耗时」口径连续。
    double maxReturnMin = -1.0;
    double integralLoadKgMin = 0.0;   // Σ_v (平均在车 × 该车耗时)
    double totalVehicleTimeMin = 0.0;
    double capacityTimeMin = 0.0;     // Σ_v (capacity × 该车耗时)
    for (std::size_t v = 0; v < vehicles.size(); ++v) {
        const Vehicle& vehicle = vehicles[v];
        const RoutePlan& plan = plans[v];
        if (vehicle.departTimeMin < minDepartMin) {
            minDepartMin = vehicle.departTimeMin;
        }
        if (plan.status != PlanStatus::Ok) {
            fleet.allOk = false;
            if (!fleet.reason.empty()) {
                fleet.reason += "；";
            }
            fleet.reason += vehicle.id + "：" + plan.reason;
            continue;
        }
        // ---- 可加项：Σ 每车 == 车队（§3.6-8）----
        fleet.totalDistanceKm += plan.totalDistanceKm;
        fleet.totalCostYuan += plan.totalCostYuan;
        fleet.totalPenaltyMin += plan.totalPenaltyMin;
        fleet.stopCount += plan.metrics.stopCount;
        fleet.actionCount += plan.metrics.actionCount;
        fleet.sumVehicleTimeMin += plan.totalTimeMin;
        const double returnAt = static_cast<double>(vehicle.departTimeMin) + plan.totalTimeMin;
        if (returnAt > maxReturnMin) {
            maxReturnMin = returnAt;
        }
        // ---- 车队级指标：两率 Σ/Σ；平均在车/利用率按时间加权 ----
        fleet.metrics.orders.insert(fleet.metrics.orders.end(), plan.metrics.orders.begin(),
                                    plan.metrics.orders.end());
        fleet.metrics.pickupOnTimeCount += plan.metrics.pickupOnTimeCount;
        fleet.metrics.pickupTotal += plan.metrics.pickupTotal;
        fleet.metrics.deliveryOnTimeCount += plan.metrics.deliveryOnTimeCount;
        fleet.metrics.deliveryTotal += plan.metrics.deliveryTotal;
        fleet.metrics.urgentOnTimeCount += plan.metrics.urgentOnTimeCount;
        fleet.metrics.urgentTotal += plan.metrics.urgentTotal;
        fleet.metrics.normalOnTimeCount += plan.metrics.normalOnTimeCount;
        fleet.metrics.normalTotal += plan.metrics.normalTotal;
        if (plan.metrics.peakLoadKg > fleet.metrics.peakLoadKg) {
            fleet.metrics.peakLoadKg = plan.metrics.peakLoadKg;
        }
        if (plan.totalTimeMin > 0.0) {
            integralLoadKgMin += plan.metrics.avgLoadKg * plan.totalTimeMin;
            totalVehicleTimeMin += plan.totalTimeMin;
            capacityTimeMin += vehicle.capacityKg * plan.totalTimeMin;
        }
    }
    fleet.metrics.stopCount = fleet.stopCount;
    fleet.metrics.actionCount = fleet.actionCount;
    if (totalVehicleTimeMin > 0.0) {
        fleet.metrics.avgLoadKg = integralLoadKgMin / totalVehicleTimeMin;
    }
    if (capacityTimeMin > 0.0) {
        fleet.metrics.loadUtilization = integralLoadKgMin / capacityTimeMin;
    }
    // 完工耗时 = max(返回) − min(发车)；空车返回 = 发车，不抬升 max。
    if (maxReturnMin >= 0.0) {
        fleet.completionTimeMin = maxReturnMin - static_cast<double>(minDepartMin);
    }
    // 均衡度（规格二 §4.3）：分母按**全部车**，空车 utilization = 0 参与极差。
    fleet.balanceSpread = fleetBalanceSpreadOf(vehicles, plans);
    // 转运账本（规格二 §5.3）：从各车停靠动作反查。默认数据上恒为空。
    fleet.transitLegs = buildTransitLedger(vehicles, plans);
    return fleet;
}

// ---- 顺序轮转基线（规格二 §4.5，对照组）----
//
// 订单 ID 升序、第 i 单 -> 第 (i mod k) 台车、车辆序 = [vehicles] 行序；
// 不跳车、不重分配、不做可行性预判。每车随后**独立**跑既有的 planRoute。
// 分配只由「订单 ID 升序 + 车队行序」决定，与策略/容量/窗口**全部无关**（不变量 9）——
// 这正是它作为对照的意义：#40 的类型感知分配与它比较时必须说清差异来源。
FleetPlan planFleetSequentialBaseline(const LogisticsGraph& graph,
                                      const std::vector<Vehicle>& vehicles,
                                      const std::vector<Order>& orders,
                                      WeightType weight) {
    if (vehicles.empty()) {
        FleetPlan fleet;
        fleet.allOk = false;
        fleet.reason = "没有车辆";
        return fleet;
    }
    // 订单 ID 升序（手写插入排序：算法主体不得交给 STL，见 AGENTS §1）。
    std::vector<std::size_t> orderIndex(orders.size());
    for (std::size_t i = 0; i < orders.size(); ++i) {
        orderIndex[i] = i;
    }
    for (std::size_t i = 1; i < orderIndex.size(); ++i) {
        const std::size_t key = orderIndex[i];
        std::size_t j = i;
        while (j > 0 && orders[key].id < orders[orderIndex[j - 1]].id) {
            orderIndex[j] = orderIndex[j - 1];
            --j;
        }
        orderIndex[j] = key;
    }
    const std::size_t k = vehicles.size();
    std::vector<std::vector<Order> > assigned(k);
    for (std::size_t i = 0; i < orderIndex.size(); ++i) {
        assigned[i % k].push_back(orders[orderIndex[i]]);
    }
    std::vector<RoutePlan> plans(k);
    for (std::size_t v = 0; v < k; ++v) {
        plans[v] = planRoute(graph, vehicles[v], assigned[v], weight);
    }
    FleetPlan fleet = aggregateFleet(vehicles, plans);
    fillAllocationFields(fleet, graph, vehicles, orders, assigned);
    // 对照组也产出腿清单（口径与 planFleet 相同）：两条入口的结构若漂，
    // 「谁可以接哪条腿」的守卫就会在一条入口上测不到。
    fleet.legs = buildLegs(graph, vehicles, orders, fleet, fleet.legLog);
    return fleet;
}

// ---- 腿与就绪链（规格三 §1 / #45）----
const char* toString(LegKind kind) {
    switch (kind) {
        case LegKind::CustomerToCustomer: return "customer->customer";
        case LegKind::CustomerToStation:  return "customer->station";
        case LegKind::StationToStation:   return "station->station";
        case LegKind::StationToCustomer:  return "station->customer";
    }
    return "customer->customer";
}

namespace {

// 簇 -> 中转站：同簇（subNetworkId 匹配）且 NodeType::Transit，取**ID 最小**的那个。
// 为什么不硬编码 T01/T02/T03：站点是**数据**不是常量，ID 改了配置就得跟着改代码是硬伤；
// 「同簇 + 最小 ID」才是可复用且确定性的规则（同输入同输出）。
// #46/#48 若想改成「最短路径站」，只换这一处，腿的展开逻辑不动。
std::string stationOfCluster(const LogisticsGraph& graph, int clusterId) {
    if (clusterId <= 0) {
        return std::string();   // 仓库/异常子网没有自己的中转站
    }
    std::string best;
    const std::vector<Node>& nodes = graph.nodes();
    for (std::size_t i = 0; i < nodes.size(); ++i) {
        if (nodes[i].subNetworkId != clusterId || nodes[i].type != NodeType::Transit) {
            continue;
        }
        if (best.empty() || nodes[i].id < best) {
            best = nodes[i].id;
        }
    }
    return best;
}

// 车辆发车时刻。找不到该车（不该发生）时由调用方决定兜底值，这里返回 -1 让调用方看见。
int departTimeOf(const std::vector<Vehicle>& vehicles, const std::string& vehicleId) {
    for (std::size_t i = 0; i < vehicles.size(); ++i) {
        if (vehicles[i].id == vehicleId) {
            return vehicles[i].departTimeMin;
        }
    }
    return -1;
}

double timeFactorOf(const std::vector<Vehicle>& vehicles, const std::string& vehicleId) {
    for (std::size_t i = 0; i < vehicles.size(); ++i) {
        if (vehicles[i].id == vehicleId) {
            return vehicles[i].timeFactor;
        }
    }
    return 1.0;
}

// 一条腿的行驶耗时（分钟，已按该车 time_factor 缩放并取整）。不可达返回 -1。
//
// 为什么可以「现算」而不算估计：本票尚未执行接力，既有计划里**根本没有**站间腿的停靠，
// 所以腿的到达时刻只能由「该腿从 readyTime 起步、按该车因子跑图最短路」得到。
// 这是图与车因子算出的**计划值**（可复算、确定性），不是常量也不是拍脑袋的估计。
// #48 有了接力执行后，用实际执行时刻覆盖 Leg::plannedArrivalMin，本函数只留给规划期。
int legTravelMin(const LogisticsGraph& graph, const std::vector<Vehicle>& vehicles,
                 const std::string& vehicleId, const std::string& from, const std::string& to) {
    const PathResult path = shortestPath(graph, from, to, WeightType::Time);
    if (!path.found) {
        return -1;
    }
    return static_cast<int>(std::lround(path.totalWeight * timeFactorOf(vehicles, vehicleId)));
}

// 该单在**既有计划里**的实际送达时刻（本地腿的计划完成时刻直接读它，不另算一遍）。
// 找不到（该单没被任何车服务）时返回 -1。
int actualDeliveryMin(const FleetPlan& fleet, const std::string& orderId) {
    for (std::size_t v = 0; v < fleet.vehicles.size(); ++v) {
        const RoutePlan& plan = fleet.vehicles[v];
        for (std::size_t s = 0; s < plan.stops.size(); ++s) {
            for (std::size_t d = 0; d < plan.stops[s].deliveries.size(); ++d) {
                if (plan.stops[s].deliveries[d].orderId == orderId) {
                    return plan.stops[s].arrivalMin;
                }
            }
        }
    }
    return -1;
}

// 「更该跑站间腿」的比较（本票的**临时**判据）：容量大者优先 ⇒ 成本因子高者 ⇒ 车辆 ID 小者。
// 三级都用了才确定性；#46 的职责表会把候选先收窄，这个判据届时只在小范围内比较。
bool betterForStationLeg(const Vehicle& a, const Vehicle& b) {
    if (a.capacityKg > b.capacityKg + 1e-9) {
        return true;
    }
    if (a.capacityKg < b.capacityKg - 1e-9) {
        return false;
    }
    if (a.costFactor > b.costFactor + 1e-9) {
        return true;
    }
    if (a.costFactor < b.costFactor - 1e-9) {
        return false;
    }
    return a.id < b.id;
}

// 腿的计划完成时刻 = readyTime + 该腿耗时；不可达时如实退化为 readyTime 并留日志
// （不可达不隐藏腿：结构登记与路由可行性是两件事）。
int plannedArrivalOf(const LogisticsGraph& graph, const std::vector<Vehicle>& vehicles,
                     const Leg& leg, std::vector<std::string>& log) {
    const int travel = legTravelMin(graph, vehicles, leg.vehicleId, leg.from, leg.to);
    if (travel < 0) {
        log.push_back("[腿] 订单 " + leg.orderId + " 的 L" + std::to_string(leg.seq) + " " + leg.from
                      + " -> " + leg.to + " 不可达 ⇒ 计划完成时刻按 readyTime 记，路由不变");
        return leg.readyTimeMin;
    }
    return leg.readyTimeMin + travel;
}

} // namespace

bool legVehicleEligible(const Vehicle& vehicle, const Leg& leg, const LogisticsGraph& graph) {
    // **本票不设职责边界**：任何车都能接任何腿。这不是遗漏，而是实施顺序——
    // 「慢车不出网、快车只接站间腿 + 跨簇紧急直达」是 #46 的内容，先立结构再收边界，
    // 否则边界与就绪链两个变量同时变，守卫红了分不清是谁的错。
    (void)vehicle;
    (void)leg;
    (void)graph;
    return true;
}

std::string chooseLegVehicle(const LogisticsGraph& graph, const std::vector<Vehicle>& vehicles,
                             const Leg& leg, const std::string& orderVehicleId) {
    // 合资格车 = 过职责谓词的车。本票谓词恒真；#46 换掉谓词后这里自动收窄，
    // 展开逻辑与 LEG 行都不用动 —— 这就是「只换一个决定点」的落点。
    std::vector<std::size_t> eligible;
    for (std::size_t i = 0; i < vehicles.size(); ++i) {
        if (legVehicleEligible(vehicles[i], leg, graph)) {
            eligible.push_back(i);
        }
    }
    if (eligible.empty()) {
        return std::string();
    }
    if (leg.kind != LegKind::StationToStation) {
        // 其余腿沿用「现有规则」的落点：该单在既有 Δkey 分配里归哪台车，就还是它。
        // 本票不引入新的选车规则（分配规则留给后续票），这一步只是把既有落点抄进腿。
        if (!orderVehicleId.empty()) {
            for (std::size_t k = 0; k < eligible.size(); ++k) {
                if (vehicles[eligible[k]].id == orderVehicleId) {
                    return orderVehicleId;
                }
            }
        }
        return std::string();   // 该单没有执行车 ⇒ 不伪造（调用方记日志）
    }
    // 站间腿：**临时**给容量最大 / 成本因子最高的车 —— 默认数据里就是快车 V01。
    // 本票只需要一个确定性落点；「只能由快车执行」这条判据由 #46 立。
    std::size_t best = eligible[0];
    for (std::size_t k = 1; k < eligible.size(); ++k) {
        if (betterForStationLeg(vehicles[eligible[k]], vehicles[best])) {
            best = eligible[k];
        }
    }
    return vehicles[best].id;
}

std::vector<Leg> buildLegs(const LogisticsGraph& graph,
                           const std::vector<Vehicle>& vehicles,
                           const std::vector<Order>& orders,
                           const FleetPlan& fleet,
                           std::vector<std::string>& log) {
    std::vector<Leg> legs;
    if (vehicles.empty()) {
        return legs;
    }
    // 「结构已立、接力未启用」必须如实登记：LEG 行只说明**腿已登记**，
    // 规划上并没有执行 L2/L3（实际路线仍由既有分配/转运链决定）。
    log.push_back("[腿] 结构已立、接力未启用：跨簇单已展开为 L1/L2/L3 且 readyTime 成链，"
                  "但本票**不执行** L2/L3（实际路线仍由既有分配/转运链决定），不声称已转运");
    log.push_back("[腿] 执行车是**临时指派**（legVehicleEligible 恒真、站间腿给容量最大的车）："
                  "职责封闭见 #46，接力与等待见 #48");

    // 订单 -> 执行车（既有分配表的落点）。不在表里 = 没有车服务该单。
    std::map<std::string, std::string> orderVehicle;
    for (std::size_t i = 0; i < fleet.allocationOrderIds.size(); ++i) {
        if (i >= fleet.allocationVehicleIndex.size()) {
            break;
        }
        const std::size_t v = fleet.allocationVehicleIndex[i];
        orderVehicle[fleet.allocationOrderIds[i]] =
            (v < vehicles.size()) ? vehicles[v].id : std::string();
    }

    const int fallbackDepartMin = vehicles.front().departTimeMin;
    for (std::size_t oi = 0; oi < orders.size(); ++oi) {
        const Order& o = orders[oi];
        const Node* a = graph.findNode(o.pickupNodeId);
        const Node* b = graph.findNode(o.nodeId);
        const std::map<std::string, std::string>::const_iterator vit = orderVehicle.find(o.id);
        const std::string orderVehicleId = (vit == orderVehicle.end()) ? std::string() : vit->second;

        // 端点在图上缺失（不可达单）：仍然登记一条腿，保证「腿的订单号 ↔ 订单表」双向一致；
        // 执行车留空、时刻按车队最早发车记，日志点名，**不伪造**可行的腿。
        if (a == nullptr || b == nullptr) {
            log.push_back("[腿] 订单 " + o.id + " 的端点不在图上 ⇒ 只登记腿结构，不指派执行车");
            Leg leg;
            leg.seq = 1;
            leg.from = o.pickupNodeId;
            leg.to = o.nodeId;
            leg.kind = LegKind::CustomerToCustomer;
            leg.orderId = o.id;
            leg.readyTimeMin = fallbackDepartMin;
            leg.plannedArrivalMin = fallbackDepartMin;
            legs.push_back(leg);
            continue;
        }

        const int clusterA = a->subNetworkId;
        const int clusterB = b->subNetworkId;
        if (clusterA == clusterB) {
            // 本地单：1 条腿 A -> B。计划完成时刻直接读既有计划里的实际送达时刻
            // （本地腿本来就被执行，不需要另算）。
            Leg leg;
            leg.seq = 1;
            leg.from = o.pickupNodeId;
            leg.to = o.nodeId;
            leg.kind = LegKind::CustomerToCustomer;
            leg.orderId = o.id;
            leg.vehicleId = orderVehicleId;
            const int depart = departTimeOf(vehicles, orderVehicleId);
            leg.readyTimeMin = (depart >= 0) ? depart : fallbackDepartMin;
            const int actual = actualDeliveryMin(fleet, o.id);
            if (actual >= 0) {
                leg.plannedArrivalMin = actual;
            } else {
                leg.plannedArrivalMin = plannedArrivalOf(graph, vehicles, leg, log);
            }
            legs.push_back(leg);
            continue;
        }

        // 跨簇单：A -> S(c1) -> S(c2) -> B。簇 -> 站按 subNetworkId 查（不硬编码站点 ID）。
        const std::string stationA = stationOfCluster(graph, clusterA);
        const std::string stationB = stationOfCluster(graph, clusterB);
        if (stationA.empty() || stationB.empty()) {
            // 缺站就无法构出中转段：**退化为 1 条直达腿**并点名（结构诚实优于伪造三段）。
            log.push_back("[腿] 跨簇单 " + o.id + " 的簇 " + std::to_string(clusterA) + " / 簇 "
                          + std::to_string(clusterB) + " 有一侧没有中转站 ⇒ 不展开为三段腿"
                          "（登记为直达单腿，路由仍由既有链决定）");
            Leg leg;
            leg.seq = 1;
            leg.from = o.pickupNodeId;
            leg.to = o.nodeId;
            leg.kind = LegKind::CustomerToCustomer;
            leg.orderId = o.id;
            leg.vehicleId = orderVehicleId;
            const int depart = departTimeOf(vehicles, orderVehicleId);
            leg.readyTimeMin = (depart >= 0) ? depart : fallbackDepartMin;
            leg.plannedArrivalMin = plannedArrivalOf(graph, vehicles, leg, log);
            legs.push_back(leg);
            continue;
        }

        // L1：客户 -> 本簇站。ready = 执行车发车时刻。
        Leg l1;
        l1.seq = 1;
        l1.from = o.pickupNodeId;
        l1.to = stationA;
        l1.kind = LegKind::CustomerToStation;
        l1.orderId = o.id;
        l1.vehicleId = chooseLegVehicle(graph, vehicles, l1, orderVehicleId);
        const int l1Depart = departTimeOf(vehicles, l1.vehicleId);
        l1.readyTimeMin = (l1Depart >= 0) ? l1Depart : fallbackDepartMin;
        l1.plannedArrivalMin = plannedArrivalOf(graph, vehicles, l1, log);

        // L2：站 -> 站。ready = **L1 的计划完成时刻**（货到站才可接），执行车按临时规则取快车。
        Leg l2;
        l2.seq = 2;
        l2.from = stationA;
        l2.to = stationB;
        l2.kind = LegKind::StationToStation;
        l2.orderId = o.id;
        l2.readyTimeMin = l1.plannedArrivalMin;
        l2.vehicleId = chooseLegVehicle(graph, vehicles, l2, orderVehicleId);
        l2.plannedArrivalMin = plannedArrivalOf(graph, vehicles, l2, log);

        // L3：对簇站 -> 客户。ready = **L2 的计划完成时刻**。
        Leg l3;
        l3.seq = 3;
        l3.from = stationB;
        l3.to = o.nodeId;
        l3.kind = LegKind::StationToCustomer;
        l3.orderId = o.id;
        l3.vehicleId = chooseLegVehicle(graph, vehicles, l3, orderVehicleId);
        l3.readyTimeMin = l2.plannedArrivalMin;
        l3.plannedArrivalMin = plannedArrivalOf(graph, vehicles, l3, log);

        if (l1.vehicleId.empty() || l3.vehicleId.empty()) {
            log.push_back("[腿] 跨簇单 " + o.id + " 没有执行车（该单未被分配）⇒ L1/L3 的执行车留空");
        }
        legs.push_back(l1);
        legs.push_back(l2);
        legs.push_back(l3);
    }

    // 行序固定：订单 ID 升序、同单内腿序 1 -> 2 -> 3（手写插入排序，算法主体不交给 STL）。
    for (std::size_t i = 1; i < legs.size(); ++i) {
        const Leg key = legs[i];
        std::size_t j = i;
        while (j > 0 && (legs[j - 1].orderId > key.orderId
                         || (legs[j - 1].orderId == key.orderId && legs[j - 1].seq > key.seq))) {
            legs[j] = legs[j - 1];
            --j;
        }
        legs[j] = key;
    }
    return legs;
}

// ---- 跨车转运账本的反查（规格二 §5.3；唯一实现）----
std::vector<TransitLeg> buildTransitLedger(const std::vector<Vehicle>& vehicles,
                                           const std::vector<RoutePlan>& plans) {
    std::vector<TransitLeg> legs;
    if (vehicles.size() != plans.size()) {
        return legs;
    }
    struct Acc {
        TransitLeg leg;
        bool hasIn = false;
        bool hasOut = false;
    };
    std::map<std::string, Acc> byOrder;
    for (std::size_t v = 0; v < plans.size(); ++v) {
        const RoutePlan& plan = plans[v];
        if (plan.status != PlanStatus::Ok) {
            continue;
        }
        for (const Stop& s : plan.stops) {
            for (const std::string& id : s.transitUnloads) {
                Acc& a = byOrder[id];
                a.leg.orderId = id;
                a.leg.stationId = s.nodeId;
                const std::map<std::string, double>::const_iterator kg = plan.transitKg.find(id);
                a.leg.kg = (kg != plan.transitKg.end()) ? kg->second : 0.0;
                a.leg.inVehicleId = vehicles[v].id;
                a.leg.inTimeMin = s.arrivalMin;
                a.hasIn = true;
            }
            for (const std::string& id : s.transitPickups) {
                Acc& a = byOrder[id];
                a.leg.orderId = id;
                if (a.leg.stationId.empty()) {
                    a.leg.stationId = s.nodeId;
                }
                a.leg.outVehicleId = vehicles[v].id;
                a.leg.outTimeMin = s.arrivalMin;
                a.hasOut = true;
            }
        }
    }
    // 两端齐全且 in != out 才是账本记录（§5.3：只卸不取不是合法记录）。
    for (std::map<std::string, Acc>::const_iterator it = byOrder.begin(); it != byOrder.end();
         ++it) {
        if (!it->second.hasIn || !it->second.hasOut) {
            continue;
        }
        if (it->second.leg.inVehicleId == it->second.leg.outVehicleId) {
            continue;   // 同车卸又取回 = 降载，不是转运（§5.2 的分界）
        }
        if (it->second.leg.outTimeMin < it->second.leg.inTimeMin) {
            continue;   // 时刻链不成立
        }
        legs.push_back(it->second.leg);
    }
    // 按 (卸入时刻, 订单 ID) 升序（手写插入排序）。
    for (std::size_t i = 1; i < legs.size(); ++i) {
        const TransitLeg key = legs[i];
        std::size_t j = i;
        while (j > 0 && (legs[j - 1].inTimeMin > key.inTimeMin
                         || (legs[j - 1].inTimeMin == key.inTimeMin
                             && legs[j - 1].orderId > key.orderId))) {
            legs[j] = legs[j - 1];
            --j;
        }
        legs[j] = key;
    }
    return legs;
}

namespace {

// 站点时钟文本（HH:MM，软件内分钟数）。只用于日志 / 账本，不参与任何算术。
std::string minuteClockText(int minute) {
    if (minute < 0) {
        return "--:--";
    }
    const int hh = minute / 60;
    const int mm = minute % 60;
    char buf[16];
    std::snprintf(buf, sizeof(buf), "%02d:%02d", hh, mm);
    return std::string(buf);
}

// 把最终计划里的一处停靠转回构造期真源（转运候选只在既有计划上改两处动作）。
StopSpec specFromStop(const Stop& s) {
    StopSpec spec;
    spec.nodeId = s.nodeId;
    spec.pickups = s.pickups;
    for (const StopDelivery& d : s.deliveries) {
        spec.deliveries.push_back(d.orderId);
    }
    spec.offloads = s.offloads;
    spec.reclaims = s.reclaims;
    spec.transitUnloads = s.transitUnloads;
    spec.transitPickups = s.transitPickups;
    spec.phase = s.phase;
    return spec;
}

std::vector<StopSpec> specsFromPlan(const RoutePlan& plan) {
    std::vector<StopSpec> specs;
    specs.reserve(plan.stops.size());
    for (const Stop& s : plan.stops) {
        specs.push_back(specFromStop(s));
    }
    return specs;
}

void eraseId(std::vector<std::string>& v, const std::string& id) {
    for (std::size_t i = 0; i < v.size();) {
        if (v[i] == id) {
            v.erase(v.begin() + static_cast<std::ptrdiff_t>(i));
        } else {
            ++i;
        }
    }
}

// 把一张订单从 v 的路线里摘走（保留取货动作）：转运卸入、容量腾挪都走这一条。
// 删除送达 / 降载 / 取回 / 旧的转运动作；空的停靠一并删掉。
void detachOrderFromRoute(std::vector<StopSpec>& specs, const std::string& orderId) {
    for (std::size_t i = 0; i < specs.size(); ++i) {
        StopSpec& s = specs[i];
        eraseId(s.deliveries, orderId);
        eraseId(s.offloads, orderId);
        eraseId(s.reclaims, orderId);
        eraseId(s.transitUnloads, orderId);
        eraseId(s.transitPickups, orderId);
    }
    for (std::size_t i = 0; i < specs.size();) {
        if (specs[i].empty()) {
            specs.erase(specs.begin() + static_cast<std::ptrdiff_t>(i));
        } else {
            ++i;
        }
    }
}

bool hasPickup(const std::vector<StopSpec>& specs, const std::string& orderId) {
    for (std::size_t i = 0; i < specs.size(); ++i) {
        for (const std::string& id : specs[i].pickups) {
            if (id == orderId) {
                return true;
            }
        }
    }
    return false;
}

long long pickupIndexOf(const std::vector<StopSpec>& specs, const std::string& orderId) {
    for (std::size_t i = 0; i < specs.size(); ++i) {
        for (const std::string& id : specs[i].pickups) {
            if (id == orderId) {
                return static_cast<long long>(i);
            }
        }
    }
    return -1;
}

bool onboardHas(const std::vector<OnboardItem>& onboard, const std::string& orderId) {
    for (const OnboardItem& it : onboard) {
        if (it.orderId == orderId) {
            return true;
        }
    }
    return false;
}

void insertStationAction(std::vector<StopSpec>& specs, std::size_t at, const std::string& station,
                         const std::string& orderId, bool unload) {
    if (at < specs.size() && specs[at].nodeId == station
        && specs[at].phase == StopPhase::Normal) {
        if (unload) {
            specs[at].transitUnloads.push_back(orderId);
        } else {
            specs[at].transitPickups.push_back(orderId);
        }
        return;
    }
    StopSpec s;
    s.nodeId = station;
    s.phase = StopPhase::Normal;
    if (unload) {
        s.transitUnloads.push_back(orderId);
    } else {
        s.transitPickups.push_back(orderId);
    }
    specs.insert(specs.begin() + static_cast<std::ptrdiff_t>(at), s);
}

bool finishSpecs(const Ctx& ctx, const std::vector<StopSpec>& specs, RoutePlan& plan) {
    Eval ev;
    if (!replay(ctx, specs, false, ev)) {
        return false;
    }
    Outcome out;
    out.ok = true;
    out.route = specs;
    finalize(ctx, out, plan);
    return plan.status == PlanStatus::Ok;
}

// v 侧候选：摘掉订单的送达/降载/取回，在取货动作之后插入一次「站内卸入」。
// 位置枚举全量扫描（标准做法：每个候选整条重放），按该车自己的三键取最优。
bool buildUnloadSpecs(const Ctx& ctx, const std::vector<StopSpec>& base,
                      const std::string& orderId, const std::string& station,
                      std::vector<StopSpec>& best, bool& found) {
    const long long pickup = pickupIndexOf(base, orderId);
    if (pickup < 0 && !onboardHas(ctx.onboard, orderId)) {
        return false;   // 既不在路线里取货、也不在车上：这单根本不在本车
    }
    const std::size_t first = (pickup < 0) ? 0 : static_cast<std::size_t>(pickup) + 1;
    bool have = false;
    Key bestKey;
    for (std::size_t at = first; at <= base.size(); ++at) {
        std::vector<StopSpec> cand = base;
        insertStationAction(cand, at, station, orderId, true);
        normalize(cand);
        Eval ev;
        if (!replay(ctx, cand, false, ev)) {
            continue;
        }
        if (!have || compareRoutes(cand, ev.key, best, bestKey) < 0) {
            have = true;
            best = cand;
            bestKey = ev.key;
        }
    }
    found = have;
    return have;
}

// w 侧候选：插入「站内取走」与送达停靠（若路线里已有该 B 停靠则合并到它）。
// 两处位置都全量枚举；站必须先于送达。按该车自己的三键取最优。
bool buildPickupSpecs(const Ctx& ctx, const std::vector<StopSpec>& base,
                      const std::string& orderId, const std::string& station,
                      const std::string& deliveryNode, std::vector<StopSpec>& best, bool& found) {
    // 已有的 B 停靠（普通段第一个）。它可能同时是取货点（顺路交付）。
    long long deliveryIdx = -1;
    for (std::size_t i = 0; i < base.size(); ++i) {
        if (base[i].nodeId == deliveryNode && base[i].phase == StopPhase::Normal) {
            deliveryIdx = static_cast<long long>(i);
            break;
        }
    }
    long long stationIdxExisting = -1;
    for (std::size_t i = 0; i < base.size(); ++i) {
        if (base[i].nodeId == station && base[i].phase == StopPhase::Normal) {
            stationIdxExisting = static_cast<long long>(i);
            break;
        }
    }
    bool have = false;
    Key bestKey;
    // 站停靠的位置：已有则只试它，否则对每个插入位置各试一次。
    std::vector<std::pair<std::size_t, bool> > stationPositions;   // (下标, 是否已存在)
    if (stationIdxExisting >= 0) {
        stationPositions.push_back(std::make_pair(
            static_cast<std::size_t>(stationIdxExisting), true));
    } else {
        for (std::size_t at = 0; at <= base.size(); ++at) {
            stationPositions.push_back(std::make_pair(at, false));
        }
    }
    for (std::size_t sp = 0; sp < stationPositions.size(); ++sp) {
        std::vector<StopSpec> withStation = base;
        std::size_t stationIdx = stationPositions[sp].first;
        insertStationAction(withStation, stationIdx, station, orderId, false);
        if (!stationPositions[sp].second) {
            normalize(withStation);
        }
        // 合并/插入可能让下标平移（相邻同站合并）；重新定位 S 停靠。
        long long sIdx = -1;
        for (std::size_t i = 0; i < withStation.size(); ++i) {
            for (const std::string& id : withStation[i].transitPickups) {
                if (id == orderId) {
                    sIdx = static_cast<long long>(i);
                    break;
                }
            }
            if (sIdx >= 0) {
                break;
            }
        }
        if (sIdx < 0) {
            continue;
        }
        // B 停靠：已有则必须位于 S 之后；否则枚举插入位置（严格在 S 之后）。
        if (deliveryIdx >= 0) {
            // 重新定位 B（插站/normalize 都可能让下标变）——不能拿插站前的旧下标判序。
            long long bIdxCheck = -1;
            for (std::size_t i = 0; i < withStation.size(); ++i) {
                if (withStation[i].nodeId == deliveryNode
                    && withStation[i].phase == StopPhase::Normal) {
                    bIdxCheck = static_cast<long long>(i);
                    break;
                }
            }
            if (bIdxCheck < 0 || bIdxCheck <= sIdx) {
                continue;
            }
            // 重新定位 B（normalize 可能让下标变）
            long long bIdx = -1;
            for (std::size_t i = static_cast<std::size_t>(sIdx) + 1; i < withStation.size(); ++i) {
                if (withStation[i].nodeId == deliveryNode
                    && withStation[i].phase == StopPhase::Normal) {
                    bIdx = static_cast<long long>(i);
                    break;
                }
            }
            if (bIdx < 0) {
                continue;
            }
            std::vector<StopSpec> cand = withStation;
            cand[static_cast<std::size_t>(bIdx)].deliveries.push_back(orderId);
            normalize(cand);
            Eval ev;
            if (!replay(ctx, cand, false, ev)) {
                continue;
            }
            if (!have || compareRoutes(cand, ev.key, best, bestKey) < 0) {
                have = true;
                best = cand;
                bestKey = ev.key;
            }
        } else {
            for (std::size_t at = static_cast<std::size_t>(sIdx) + 1; at <= withStation.size();
                 ++at) {
                std::vector<StopSpec> cand = withStation;
                // 该位置已有同节点普通停靠时直接合并（顺路交付）。
                if (at < cand.size() && cand[at].nodeId == deliveryNode
                    && cand[at].phase == StopPhase::Normal) {
                    cand[at].deliveries.push_back(orderId);
                } else {
                    StopSpec b;
                    b.nodeId = deliveryNode;
                    b.phase = StopPhase::Normal;
                    b.deliveries.push_back(orderId);
                    cand.insert(cand.begin() + static_cast<std::ptrdiff_t>(at), b);
                }
                normalize(cand);
                Eval ev;
                if (!replay(ctx, cand, false, ev)) {
                    continue;
                }
                if (!have || compareRoutes(cand, ev.key, best, bestKey) < 0) {
                    have = true;
                    best = cand;
                    bestKey = ev.key;
                }
            }
        }
    }
    found = have;
    return have;
}

// 从一组计划里读出某笔转运的两端（订单 -> 卸入/取走/送达时刻与 kg）。
bool readTransitLeg(const std::vector<Vehicle>& vehicles, const std::vector<RoutePlan>& plans,
                    const std::string& orderId, TransitLeg& leg, int& deliveryTime,
                    int& pickupTime) {
    bool hasIn = false;
    bool hasOut = false;
    bool hasDel = false;
    int pickTime = -1;
    for (std::size_t v = 0; v < plans.size(); ++v) {
        const RoutePlan& plan = plans[v];
        for (const Stop& s : plan.stops) {
            for (const std::string& id : s.transitUnloads) {
                if (id != orderId) {
                    continue;
                }
                leg.orderId = id;
                leg.stationId = s.nodeId;
                std::map<std::string, double>::const_iterator kg = plan.transitKg.find(id);
                leg.kg = (kg != plan.transitKg.end()) ? kg->second : 0.0;
                leg.inVehicleId = vehicles[v].id;
                leg.inTimeMin = s.arrivalMin;
                hasIn = true;
            }
            for (const std::string& id : s.transitPickups) {
                if (id != orderId) {
                    continue;
                }
                leg.outVehicleId = vehicles[v].id;
                leg.outTimeMin = s.arrivalMin;
                hasOut = true;
            }
            for (const StopDelivery& d : s.deliveries) {
                if (d.orderId == orderId) {
                    deliveryTime = s.arrivalMin;
                    hasDel = true;
                }
            }
            for (const std::string& id : s.pickups) {
                if (id == orderId) {
                    pickTime = s.arrivalMin;
                }
            }
        }
    }
    pickupTime = pickTime;
    return hasIn && hasOut && hasDel && leg.inVehicleId != leg.outVehicleId;
}

bool keyStrictlyBetter(const FleetKey& cand, const FleetKey& base) {
    // 接受判据：**目标三键**严格更优；完全并列 ⇒ 保留禁转运版（§5.7 的关键一条）。
    // tieKey 只用于「同目标的候选之间」裁决，不用来把零收益转运塞进默认数据。
    return compareFleetKey(cand, base) < 0;
}

int orderPenaltyOf(const std::vector<RoutePlan>& plans, const std::string& orderId) {
    for (std::size_t v = 0; v < plans.size(); ++v) {
        for (const OrderMetric& om : plans[v].metrics.orders) {
            if (om.orderId == orderId) {
                return om.penaltyMin;
            }
        }
    }
    return -1;   // 不在任何车的送达指标里（不可行）
}

std::string vehicleOfOrderInPlans(const std::vector<RoutePlan>& plans,
                                  const std::string& orderId) {
    for (std::size_t v = 0; v < plans.size(); ++v) {
        for (const Stop& s : plans[v].stops) {
            for (const std::string& id : s.transitPickups) {
                if (id == orderId) {
                    return std::string();   // 已在途中（取走车）
                }
            }
            for (const StopDelivery& d : s.deliveries) {
                if (d.orderId == orderId) {
                    return std::string("?") + std::to_string(v);
                }
            }
        }
    }
    return std::string();
}

bool planHasTransit(const std::vector<RoutePlan>& plans, const std::string& orderId) {
    for (std::size_t v = 0; v < plans.size(); ++v) {
        for (const Stop& s : plans[v].stops) {
            for (const std::string& id : s.transitUnloads) {
                if (id == orderId) {
                    return true;
                }
            }
            for (const std::string& id : s.transitPickups) {
                if (id == orderId) {
                    return true;
                }
            }
        }
    }
    return false;
}

// 某车计划里出现过的订单 ID（取货或送达；用于建 Ctx 的全局订单表）。
void collectPlanOrderIds(const RoutePlan& plan, std::vector<std::string>& ids) {
    for (const Stop& s : plan.stops) {
        for (const std::string& id : s.pickups) {
            bool seen = false;
            for (const std::string& x : ids) {
                if (x == id) {
                    seen = true;
                }
            }
            if (!seen) {
                ids.push_back(id);
            }
        }
        for (const StopDelivery& d : s.deliveries) {
            bool seen = false;
            for (const std::string& x : ids) {
                if (x == d.orderId) {
                    seen = true;
                }
            }
            if (!seen) {
                ids.push_back(d.orderId);
            }
        }
    }
}

// 把「车队的全部订单」补进某车的 Ctx（只为 kg / 逐单指标；不做可达性检查）。
// 转运会把一张单的送达挪到另一台车：那台车的 Ctx 必须知道该单的 kg 与窗口，
// 否则 replay 的载重是 0、finalize 的指标也缺这一单（不变量 8 直接失守）。
void appendFleetOrders(Ctx& ctx, const std::vector<Order>& all) {
    std::vector<Request> extraNormal;
    std::vector<Request> extraUrgent;
    for (std::size_t i = 0; i < all.size(); ++i) {
        const Order& o = all[i];
        if (ctx.byId.find(o.id) != ctx.byId.end()) {
            continue;
        }
        Request r;
        r.orderId = o.id;
        r.pickup = o.pickupNodeId;
        r.delivery = o.nodeId;
        r.kg = o.demandKg;
        r.ws = o.windowStartMin;
        r.we = o.windowEndMin;
        r.urgent = o.urgent;
        r.seq = ctx.byId.size();
        ctx.orderIndex[r.orderId] = ctx.orderIndex.size();
        ctx.byId[r.orderId] = r;
        if (r.urgent) {
            extraUrgent.push_back(r);
        } else {
            extraNormal.push_back(r);
        }
    }
    for (std::size_t i = 0; i < extraNormal.size(); ++i) {
        ctx.normal.push_back(extraNormal[i]);
    }
    for (std::size_t i = 0; i < extraUrgent.size(); ++i) {
        ctx.urgent.push_back(extraUrgent[i]);
    }
    sortFifo(ctx.urgent);
    sortEdd(ctx.normal);
}

bool zeroDetour(const CostMatrix& matrix, const std::string& a, const std::string& s,
                const std::string& b, WeightType weight) {
    double ad = 0.0;
    double at = 0.0;
    double ac = 0.0;
    double as = 0.0;
    double sd = 0.0;
    double st = 0.0;
    double sc = 0.0;
    double bd = 0.0;
    double bt = 0.0;
    double bc = 0.0;
    if (!matrix.leg(a, s, weight, ad, at, ac) || !matrix.leg(s, b, weight, sd, st, sc)
        || !matrix.leg(a, b, weight, bd, bt, bc)) {
        return false;
    }
    double direct = 0.0;
    double via = 0.0;
    switch (weight) {
        case WeightType::Distance: direct = bd; via = ad + sd; break;
        case WeightType::Time:     direct = bt; via = at + st; break;
        case WeightType::Cost:     direct = bc; via = ac + sc; break;
    }
    return via <= direct + 1e-9;
}

struct TriggerOrder {
    std::string orderId;
    std::size_t inVehicle = 0;
    std::string station;   // 判据②：卸货车自然路线上的降载站
};

// 候选统计（只用于日志）。
struct GenerateStats {
    int generated = 0;
    bool found = false;
};

} // namespace

FleetTransitResult applyTransitLayer(const LogisticsGraph& graph,
                                     const std::vector<Vehicle>& vehicles,
                                     const std::vector<std::vector<Order> >& vehicleOrders,
                                     const std::vector<RoutePlan>& basePlans,
                                     const std::vector<VehicleStartState>& starts,
                                     WeightType weight) {
    FleetTransitResult result;
    result.plans = basePlans;
    if (vehicles.empty() || basePlans.size() != vehicles.size()
        || starts.size() != vehicles.size() || vehicleOrders.size() != vehicles.size()) {
        return result;
    }
    for (std::size_t v = 0; v < basePlans.size(); ++v) {
        if (basePlans[v].status != PlanStatus::Ok) {
            return result;   // 禁转运版本身不可行：转运层不负责救（升级链归 #42）
        }
    }
    // 全车队订单并集（含跨车送达的可能对象）。判别用 ID 汇总，顺序按传入。
    std::vector<Order> allOrders;
    std::map<std::string, Order> orderById;
    for (std::size_t v = 0; v < vehicleOrders.size(); ++v) {
        for (std::size_t i = 0; i < vehicleOrders[v].size(); ++i) {
            const Order& o = vehicleOrders[v][i];
            if (orderById.find(o.id) == orderById.end()) {
                orderById[o.id] = o;
                allOrders.push_back(o);
            }
        }
    }
    // 每车构造上下文（起点/在车货/暂存 + 全局订单表）。矩阵每车一份（与 planRoute 同口径）。
    std::vector<Ctx> ctxs(vehicles.size());
    for (std::size_t v = 0; v < vehicles.size(); ++v) {
        std::string reason;
        if (!makeCtx(graph, vehicles[v], vehicleOrders[v], starts[v].startNodeId,
                     starts[v].startTimeMin, weight, starts[v].onboard, starts[v].stash, ctxs[v],
                     reason)) {
            return result;
        }
        appendFleetOrders(ctxs[v], allOrders);
    }
    const std::vector<std::size_t> byId = vehiclesSortedById(vehicles);
    std::vector<RoutePlan> plans = basePlans;
    FleetKey baseKey = fleetKeyOf(vehicles, plans, weight);
    std::vector<std::string> moved;
    auto alreadyMoved = [&](const std::string& id) {
        for (std::size_t i = 0; i < moved.size(); ++i) {
            if (moved[i] == id) {
                return true;
            }
        }
        return false;
    };

    // 候选：给定触发器、订单、站、接收车，构造两版并读出账本两端。
    struct Candidate {
        std::vector<RoutePlan> plans;
        TransitLeg leg;
        std::string orderId;
    };
    auto makeCandidate = [&](const TriggerOrder& t, const std::string& station,
                             std::size_t outIdx, Candidate& out) -> bool {
        const std::size_t inIdx = t.inVehicle;
        if (outIdx >= plans.size() || inIdx >= plans.size() || outIdx == inIdx) {
            return false;
        }
        if (plans[outIdx].status != PlanStatus::Ok) {
            return false;
        }
        std::vector<StopSpec> baseV = specsFromPlan(plans[inIdx]);
        detachOrderFromRoute(baseV, t.orderId);
        std::vector<StopSpec> unloadSpecs;
        bool haveUnload = false;
        if (!buildUnloadSpecs(ctxs[inIdx], baseV, t.orderId, station, unloadSpecs, haveUnload)
            || !haveUnload) {
            return false;
        }
        std::vector<StopSpec> baseW = specsFromPlan(plans[outIdx]);
        std::vector<StopSpec> pickupSpecs;
        bool havePickup = false;
        const Order& o = orderById[t.orderId];
        if (!buildPickupSpecs(ctxs[outIdx], baseW, t.orderId, station, o.nodeId, pickupSpecs,
                              havePickup)
            || !havePickup) {
            return false;
        }
        std::vector<RoutePlan> cand = plans;
        if (!finishSpecs(ctxs[inIdx], unloadSpecs, cand[inIdx])) {
            return false;
        }
        if (!finishSpecs(ctxs[outIdx], pickupSpecs, cand[outIdx])) {
            return false;
        }
        TransitLeg leg;
        int deliveryTime = 0;
        int pickupTime = -1;
        if (!readTransitLeg(vehicles, cand, t.orderId, leg, deliveryTime, pickupTime)) {
            return false;
        }
        const int dwell = leg.outTimeMin - leg.inTimeMin;
        if (dwell < 0 || dwell > kTransferMaxDwellMin) {
            return false;   // 站内等待上限：构造期过滤（不是「更差」，是不存在）
        }
        // 时刻链：取货(A) ≤ 卸入(S) ≤ 取走(S) ≤ 送达(B)
        const int pick = (pickupTime >= 0) ? pickupTime : starts[inIdx].startTimeMin;
        if (!(pick <= leg.inTimeMin && leg.inTimeMin <= leg.outTimeMin
              && leg.outTimeMin <= deliveryTime)) {
            return false;
        }
        // 整单不可拆 / 非负：同一订单只允许一笔「未取走」记录。
        // 构造上每轮只动一张从未转运过的单 ⇒ C1 自动成立。
        out.plans = cand;
        out.leg = leg;
        out.orderId = t.orderId;
        return true;
    };

    // 生成某判据下的全部候选并取最优。并列规则：站 ID、接收车 ID 升序（生成顺序即升序），
    // 目标三键完全并列时按 tieKey，再按生成序先到者胜。
    auto runTrigger = [&](const char* tag, int triggerIndex,
                          const std::vector<TriggerOrder>& triggers,
                          bool urgentDoubleGate) -> void {
        for (int round = 1; round <= 3; ++round) {
            // 收集候选（固定顺序：订单 ID 升序 -> 站 ID 升序 -> 车辆 ID 升序）。
            std::vector<TriggerOrder> sorted = triggers;
            for (std::size_t i = 1; i < sorted.size(); ++i) {
                const TriggerOrder key = sorted[i];
                std::size_t j = i;
                while (j > 0 && sorted[j - 1].orderId > key.orderId) {
                    sorted[j] = sorted[j - 1];
                    --j;
                }
                sorted[j] = key;
            }
            std::vector<std::string> stations;
            for (const Node& n : graph.nodes()) {
                if (n.type == NodeType::Transit) {
                    stations.push_back(n.id);
                }
            }
            for (std::size_t i = 1; i < stations.size(); ++i) {
                const std::string key = stations[i];
                std::size_t j = i;
                while (j > 0 && stations[j - 1] > key) {
                    stations[j] = stations[j - 1];
                    --j;
                }
                stations[j] = key;
            }
            bool haveBest = false;
            Candidate best;
            FleetKey bestKey;
            int generated = 0;
            for (std::size_t ti = 0; ti < sorted.size(); ++ti) {
                const TriggerOrder& t = sorted[ti];
                if (alreadyMoved(t.orderId) || planHasTransit(plans, t.orderId)) {
                    continue;
                }
                std::map<std::string, Order>::const_iterator oi = orderById.find(t.orderId);
                if (oi == orderById.end()) {
                    continue;
                }
                const Order& o = oi->second;
                for (std::size_t si = 0; si < stations.size(); ++si) {
                    const std::string& station = stations[si];
                    if (triggerIndex != 1
                        && !zeroDetour(ctxs[t.inVehicle].matrix, o.pickupNodeId, station, o.nodeId,
                                       weight)) {
                        continue;   // 零绕行站集合（①③）；② 用卸货车自然路线上的降载站
                    }
                    if (triggerIndex == 1 && station != t.station) {
                        continue;
                    }
                    for (std::size_t t2 = 0; t2 < byId.size(); ++t2) {
                        const std::size_t outIdx = byId[t2];
                        if (outIdx == t.inVehicle) {
                            continue;
                        }
                        Candidate cand;
                        if (!makeCandidate(t, station, outIdx, cand)) {
                            continue;
                        }
                        ++generated;
                        // 候选优于当前最优？先比目标三键，再比 tieKey，再比生成序（先到者）。
                        const FleetKey ck = fleetKeyOf(vehicles, cand.plans, weight);
                        if (!ck.allOk) {
                            continue;
                        }
                        bool take = false;
                        if (!haveBest) {
                            take = true;
                        } else {
                            const int kc = compareFleetKey(ck, bestKey);
                            if (kc < 0) {
                                take = true;
                            } else if (kc == 0) {
                                const int kt = compareFleetTie(cand.plans, best.plans, byId);
                                if (kt < 0) {
                                    take = true;
                                }
                            }
                        }
                        if (take) {
                            haveBest = true;
                            best = cand;
                            bestKey = ck;
                        }
                    }
                }
            }
            result.log.push_back(std::string("转运判据") + tag + " 第 " + std::to_string(round)
                                 + " 轮：候选 " + std::to_string(generated) + " 个");
            if (!haveBest) {
                result.log.back() = std::string("转运判据") + tag + " 第 " + std::to_string(round)
                                    + " 轮：无候选 ⇒ 停止该判据";
                break;
            }
            // 接受判据：目标三键严格更优（完全并列保留禁转运版）。
            if (!keyStrictlyBetter(bestKey, baseKey)) {
                result.log.back() = std::string("转运判据") + tag + " 第 " + std::to_string(round)
                                    + " 轮：最优候选不严格更优 ⇒ 保留禁转运版，停止该判据";
                break;
            }
            // 紧急单双门槛：该单 penalty 必须严格下降，且整计划（目标键）严格更优。
            if (urgentDoubleGate) {
                const int before = orderPenaltyOf(plans, best.orderId);
                const int after = orderPenaltyOf(best.plans, best.orderId);
                if (!(before > 0 && after >= 0 && after < before)) {
                    result.log.back() = std::string("转运判据") + tag + " 第 " + std::to_string(round)
                                        + " 轮：该紧急单 penalty 未严格下降（"
                                        + std::to_string(before) + " -> " + std::to_string(after)
                                        + "）⇒ 双门槛不成立，停止该判据";
                    break;
                }
            }
            plans = best.plans;
            baseKey = fleetKeyOf(vehicles, plans, weight);
            moved.push_back(best.orderId);
            ++result.acceptedRounds;
            result.log.back() = std::string("转运判据") + tag + " 第 " + std::to_string(round)
                                + " 轮：接受 " + best.orderId + " 经 "
                                + best.leg.stationId + "（卸入 " + best.leg.inVehicleId + " "
                                + minuteClockText(best.leg.inTimeMin) + " / 取走 "
                                + best.leg.outVehicleId + " "
                                + minuteClockText(best.leg.outTimeMin) + "，站内等待 "
                                + std::to_string(best.leg.outTimeMin - best.leg.inTimeMin)
                                + "min）";
        }
    };

    bool anyTrigger = false;

    // ---- 判据① 跨簇订单 ----
    {
        std::vector<TriggerOrder> triggers;
        for (std::size_t v = 0; v < plans.size(); ++v) {
            for (const Stop& s : plans[v].stops) {
                for (const StopDelivery& d : s.deliveries) {
                    std::map<std::string, Order>::const_iterator it = orderById.find(d.orderId);
                    if (it == orderById.end()) {
                        continue;
                    }
                    if (!isCrossCluster(graph, it->second)) {
                        continue;
                    }
                    if (alreadyMoved(d.orderId) || planHasTransit(plans, d.orderId)) {
                        continue;
                    }
                    TriggerOrder t;
                    t.orderId = d.orderId;
                    t.inVehicle = v;
                    triggers.push_back(t);
                }
            }
        }
        if (!triggers.empty()) {
            anyTrigger = true;
            result.log.push_back("转运判据①（跨簇订单）：触发，涉及 "
                                 + std::to_string(triggers.size()) + " 单");
            runTrigger("①（跨簇订单）", 0, triggers, false);
        }
    }

    // ---- 判据② 容量腾挪 ----
    {
        std::vector<TriggerOrder> triggers;
        for (std::size_t v = 0; v < plans.size(); ++v) {
            for (const OffloadLeg& leg : plans[v].offloads) {
                if (alreadyMoved(leg.orderId) || planHasTransit(plans, leg.orderId)) {
                    continue;
                }
                TriggerOrder t;
                t.orderId = leg.orderId;
                t.inVehicle = v;
                t.station = leg.station;
                triggers.push_back(t);
            }
        }
        if (!triggers.empty()) {
            anyTrigger = true;
            result.log.push_back("转运判据②（容量腾挪）：触发，禁转运版有降载记录 "
                                 + std::to_string(triggers.size()) + " 单");
            runTrigger("②（容量腾挪）", 1, triggers, false);
        }
    }

    // ---- 判据③ 紧急响应 ----
    {
        std::vector<TriggerOrder> triggers;
        for (std::size_t v = 0; v < plans.size(); ++v) {
            for (const OrderMetric& om : plans[v].metrics.orders) {
                if (!om.urgent || om.penaltyMin <= 0) {
                    continue;
                }
                if (alreadyMoved(om.orderId) || planHasTransit(plans, om.orderId)) {
                    continue;
                }
                TriggerOrder t;
                t.orderId = om.orderId;
                t.inVehicle = v;
                triggers.push_back(t);
            }
        }
        if (!triggers.empty()) {
            anyTrigger = true;
            result.log.push_back("转运判据③（紧急响应）：触发，禁转运版有迟到紧急单 "
                                 + std::to_string(triggers.size()) + " 单");
            runTrigger("③（紧急响应）", 2, triggers, true);
        }
    }

    if (result.acceptedRounds == 0) {
        if (anyTrigger) {
            result.log.push_back("转运层：未启用（三条判据均未产生严格更优解）");
        }
        result.plans = basePlans;
        result.legs.clear();
        return result;
    }
    result.plans = plans;
    result.legs = buildTransitLedger(vehicles, result.plans);
    return result;
}

// ---- 类型感知三阶段分配（规格二 §4，默认入口）----
//
// 阶段一 聚类（§4.2）：簇 = 取货点 A 的 subNetworkId；A、B 跨网 ⇒ 跨簇单进跨簇队列。
//   本地单进 A 所在簇；subNetworkId == 0 的异常数据（手动新客户未继承）也归跨簇队列
//   并记日志 —— **不静默丢单**。#34 已保证建节点时继承最近邻的 sub_network_id。
// 阶段二 整簇指派（§4.3）：手写全排列枚举「簇 -> 车」（默认 3 簇 3 车 = 6 种）；
//   每簇订单交给该车跑**规格一的单车构造器**；候选按车队键
//   (Σ逐单 penalty, Σ策略权重, 均衡键, tieKey) 取优。**单车构造器内部仍是规格一三键**，
//   载重峰值不进单车键——它是车队层的均衡键与报告指标。
// 阶段三 有界改派（§4.4）：R ≤ 3 轮、只普通段、EDD 序；两条接受条件同时成立才接受：
//   ① 车队键严格变优；② 完工耗时不得变差。每条被接受的改派都记 relocations（不变量 8）。
FleetPlan planFleet(const LogisticsGraph& graph,
                    const std::vector<Vehicle>& vehicles,
                    const std::vector<Order>& orders,
                    WeightType weight) {
    FleetPlan fleet;
    if (vehicles.empty()) {
        fleet.allOk = false;
        fleet.reason = "没有车辆";
        return fleet;
    }
    const std::size_t k = vehicles.size();
    const std::vector<std::size_t> byId = vehiclesSortedById(vehicles);

    // ---- 阶段一 · 聚类 ----
    std::map<int, std::vector<Order> > clusterMap;
    std::vector<Order> crossOrders;
    std::vector<Order> rejectedOrders;   // 不可达等：兜底链 ④ 点名原因
    for (std::size_t i = 0; i < orders.size(); ++i) {
        const Order& o = orders[i];
        const Node* a = graph.findNode(o.pickupNodeId);
        const Node* b = graph.findNode(o.nodeId);
        if (a == nullptr || b == nullptr) {
            fleet.allocationLog.push_back("拒绝 " + o.id + "：取货点 " + o.pickupNodeId
                                          + " 或送达点 " + o.nodeId + " 不在图上（不可达）");
            rejectedOrders.push_back(o);
            continue;
        }
        const int ca = a->subNetworkId;
        const int cb = b->subNetworkId;
        if (ca != cb) {
            crossOrders.push_back(o);
            continue;
        }
        if (ca == 0) {
            // 异常数据：子网络 0 不存在（§4.2 的前置修正已由 #34 保证继承）。
            // 归入跨簇队列并记日志，**不静默丢单**。
            fleet.allocationLog.push_back("订单 " + o.id + " 的取货点 " + o.pickupNodeId
                                          + " 子网络 0（异常数据）⇒ 归入跨簇队列");
            crossOrders.push_back(o);
            continue;
        }
        clusterMap[ca].push_back(o);
    }
    sortOrdersEdd(crossOrders);

    struct Group {
        int clusterId = 0;
        int part = 0;                // 0 = 整簇；>=1 = 拆簇后的第几段
        std::vector<Order> orders;
    };
    std::vector<Group> groups;
    for (std::map<int, std::vector<Order> >::const_iterator it = clusterMap.begin();
         it != clusterMap.end(); ++it) {
        Group g;
        g.clusterId = it->first;
        g.orders = it->second;
        groups.push_back(g);
    }

    auto groupCargo = [](const std::vector<Order>& os) {
        double kg = 0.0;
        for (std::size_t i = 0; i < os.size(); ++i) {
            kg += os[i].demandKg;
        }
        return kg;
    };

    // 簇数 > 车数（默认数据不触发，事件场景才用）：按簇货量降序取前 k 个簇各占一车，
    // 剩余簇并入「货量最接近且仍可行」的车。可行性在枚举里由各车构造器实测，
    // 这里先用「至少一台车能构造出可行路线」把明显不可行的落点排掉。
    auto reduceGroups = [&](std::vector<Group>& gs) {
        if (gs.size() <= k) {
            return;
        }
        std::vector<std::size_t> ord(gs.size());
        for (std::size_t i = 0; i < ord.size(); ++i) {
            ord[i] = i;
        }
        for (std::size_t i = 1; i < ord.size(); ++i) {
            const std::size_t key = ord[i];
            const double keyKg = groupCargo(gs[key].orders);
            std::size_t j = i;
            while (j > 0) {
                const std::size_t prev = ord[j - 1];
                const double prevKg = groupCargo(gs[prev].orders);
                const bool move = (keyKg > prevKg)
                                  || (nearlyEqDouble(keyKg, prevKg)
                                      && gs[key].clusterId < gs[prev].clusterId);
                if (!move) {
                    break;
                }
                ord[j] = prev;
                --j;
            }
            ord[j] = key;
        }
        std::vector<Group> kept;
        for (std::size_t i = 0; i < k && i < ord.size(); ++i) {
            kept.push_back(gs[ord[i]]);
        }
        for (std::size_t i = k; i < ord.size(); ++i) {
            const Group extra = gs[ord[i]];
            const double extraKg = groupCargo(extra.orders);
            std::vector<std::size_t> cand(kept.size());
            for (std::size_t c = 0; c < cand.size(); ++c) {
                cand[c] = c;
            }
            for (std::size_t a = 1; a < cand.size(); ++a) {
                const std::size_t key = cand[a];
                const double keyDiff = std::fabs(groupCargo(kept[key].orders) - extraKg);
                std::size_t b = a;
                while (b > 0) {
                    const std::size_t prev = cand[b - 1];
                    const double prevDiff = std::fabs(groupCargo(kept[prev].orders) - extraKg);
                    if (!(keyDiff < prevDiff
                          || (nearlyEqDouble(keyDiff, prevDiff)
                              && kept[key].clusterId < kept[prev].clusterId))) {
                        break;
                    }
                    cand[b] = prev;
                    --b;
                }
                cand[b] = key;
            }
            int target = -1;
            for (std::size_t c = 0; c < cand.size(); ++c) {
                std::vector<Order> merged = kept[cand[c]].orders;
                merged.insert(merged.end(), extra.orders.begin(), extra.orders.end());
                bool feasible = false;
                for (std::size_t t = 0; t < byId.size(); ++t) {
                    if (planRoute(graph, vehicles[byId[t]], merged, weight).status
                        == PlanStatus::Ok) {
                        feasible = true;
                        break;
                    }
                }
                if (feasible) {
                    target = static_cast<int>(cand[c]);
                    break;
                }
            }
            if (target < 0) {
                target = static_cast<int>(cand[0]);
                fleet.allocationLog.push_back(
                    "簇数 > 车数：簇 " + std::to_string(extra.clusterId)
                    + " 没有「仍可行」的落点，仍并入最近的一组（交兜底链）");
            } else {
                fleet.allocationLog.push_back(
                    "簇数 > 车数：簇 " + std::to_string(extra.clusterId) + " 并入簇 "
                    + std::to_string(kept[target].clusterId) + " 所在的组（货量最接近且仍可行）");
            }
            kept[target].orders.insert(kept[target].orders.end(), extra.orders.begin(),
                                       extra.orders.end());
        }
        gs = kept;
    };

    struct Candidate {
        std::vector<std::vector<Order> > vehicleOrders;
        std::vector<RoutePlan> plans;
        FleetKey key;
        int enumIndex = 0;
        int infeasibleVehicles = 0;
        bool feasible = false;
        std::vector<std::string> crossRejected;
    };

    // 一种「簇 -> 车」排列的完整候选：各车构造 + 跨簇队列按 Δkey 升序逐单指派。
    auto evaluate = [&](const std::vector<Group>& gs, const std::vector<int>& choice,
                        int enumIndex) -> Candidate {
        Candidate c;
        c.enumIndex = enumIndex;
        c.vehicleOrders.assign(k, std::vector<Order>());
        c.plans.assign(k, RoutePlan());
        for (std::size_t g = 0; g < gs.size() && g < choice.size(); ++g) {
            if (choice[g] < 0) {
                continue;
            }
            const std::size_t v = static_cast<std::size_t>(choice[g]);
            c.vehicleOrders[v].insert(c.vehicleOrders[v].end(), gs[g].orders.begin(),
                                      gs[g].orders.end());
        }
        for (std::size_t v = 0; v < k; ++v) {
            c.plans[v] = planRoute(graph, vehicles[v], c.vehicleOrders[v], weight);
            if (c.plans[v].status != PlanStatus::Ok) {
                ++c.infeasibleVehicles;
            }
        }
        bool crossOk = true;
        for (std::size_t i = 0; i < crossOrders.size(); ++i) {
            const Order& o = crossOrders[i];
            int bestV = -1;
            MarginalCost bestMc;
            RoutePlan bestPlan;
            // 车辆按 ID 升序尝试 ⇒ 并列时先到者（车辆 ID 升序）胜（§4.7）。
            for (std::size_t t = 0; t < byId.size(); ++t) {
                const std::size_t v = byId[t];
                std::vector<Order> candOrders = c.vehicleOrders[v];
                candOrders.push_back(o);
                const RoutePlan cand = planRoute(graph, vehicles[v], candOrders, weight);
                if (cand.status != PlanStatus::Ok) {
                    continue;
                }
                const MarginalCost mc = marginalCostBetween(c.plans[v], cand, vehicles[v], weight);
                if (bestV < 0 || compareMarginal(mc, bestMc) < 0) {
                    bestV = static_cast<int>(v);
                    bestMc = mc;
                    bestPlan = cand;
                }
            }
            if (bestV < 0) {
                c.crossRejected.push_back(o.id);
                crossOk = false;
                continue;
            }
            c.vehicleOrders[static_cast<std::size_t>(bestV)].push_back(o);
            c.plans[static_cast<std::size_t>(bestV)] = bestPlan;
        }
        c.key = fleetKeyOf(vehicles, c.plans, weight);
        c.feasible = crossOk && c.infeasibleVehicles == 0 && rejectedOrders.empty()
                     && c.key.allOk;
        return c;
    };

    auto compareCandidate = [&](const Candidate& a, const Candidate& b) -> int {
        const int c = compareFleetKey(a.key, b.key);
        if (c != 0) {
            return c;
        }
        const int t = compareFleetTie(a.plans, b.plans, byId);
        if (t != 0) {
            return t;
        }
        // 枚举序：先枚举者胜（枚举本身就是「簇 ID 升序、车辆 ID 升序」）。
        return a.enumIndex < b.enumIndex ? -1 : (a.enumIndex > b.enumIndex ? 1 : 0);
    };

    struct EvalResult {
        Candidate best;
        Candidate fallback;      // 不可行时「不可行车辆数最少」的兜底候选
        bool      haveFallback = false;
        bool      any = false;
    };

    auto runEnumeration = [&](const std::vector<Group>& gs) -> EvalResult {
        EvalResult res;
        std::vector<std::vector<int> > perms;
        if (k > 4) {
            // §4.10：k > 4 时全排列会爆掉，改用「簇货量降序贪心 + 阶段三修」，如实记录。
            std::vector<int> choice(gs.size(), -1);
            std::vector<double> load(k, 0.0);
            for (std::size_t g = 0; g < gs.size(); ++g) {
                int target = -1;
                for (std::size_t t = 0; t < byId.size(); ++t) {
                    const std::size_t v = byId[t];
                    if (target < 0 || load[v] < load[static_cast<std::size_t>(target)]) {
                        target = static_cast<int>(v);
                    }
                }
                choice[g] = target;
                load[static_cast<std::size_t>(target)] += groupCargo(gs[g].orders);
            }
            perms.push_back(choice);
            fleet.allocationLog.push_back(
                "k > 4：改用「簇货量降序贪心 + 阶段三修」（§4.10），不枚举全排列");
        } else {
            std::vector<int> choice(gs.size(), -1);
            std::vector<bool> used(k, false);
            enumerateAssignments(0, gs.size(), byId, choice, used, perms);
        }
        for (std::size_t pi = 0; pi < perms.size(); ++pi) {
            const Candidate c = evaluate(gs, perms[pi], static_cast<int>(pi));
            std::string line = "排列 " + std::to_string(pi) + " ";
            for (std::size_t g = 0; g < gs.size() && g < perms[pi].size(); ++g) {
                line += "簇" + std::to_string(gs[g].clusterId);
                if (gs[g].part > 0) {
                    line += "-" + std::to_string(gs[g].part);
                }
                line += "=";
                line += (perms[pi][g] >= 0)
                            ? vehicles[static_cast<std::size_t>(perms[pi][g])].id
                            : std::string("-");
                line += " ";
            }
            line += "key=(" + std::to_string(c.key.penalty) + ", "
                    + std::to_string(c.key.weight) + ", " + std::to_string(c.key.balance)
                    + ") 完工=" + std::to_string(c.key.completion) + "min";
            if (!c.feasible) {
                line += " 不可行";
            }
            fleet.candidateKeyLog.push_back(line);
            if (c.feasible && (!res.any || compareCandidate(c, res.best) < 0)) {
                res.best = c;
                res.any = true;
            }
            if (!res.haveFallback
                || c.infeasibleVehicles < res.fallback.infeasibleVehicles
                || (c.infeasibleVehicles == res.fallback.infeasibleVehicles
                    && compareFleetKey(c.key, res.fallback.key) < 0)) {
                res.fallback = c;
                res.haveFallback = true;
            }
        }
        return res;
    };

    reduceGroups(groups);
    EvalResult res = runEnumeration(groups);

    if (!res.any && k >= 2) {
        // ② 拆簇（§4.6）：装不下的簇按 EDD 前缀拆两段给两台车。**仅此级触发并留日志**。
        //
        // 如实记录：本实现里这一级是**防御级**。单车构造器会把同点多次取货串行化，
        // 前缀峰值 = 单笔最大值，所以「整簇装不下」只可能来自**单笔超载**或**不可达**——
        // 这两种拆簇都救不了（拆完还是同一张单/同一条不可达边）。测试
        // testFleetClusterFitsViaPrefixSerialization 钉住了这个前提；若构造器哪天
        // 不再串行化，这条保底路径就会真的被用到。
        std::vector<Group> splitGroups;
        bool splitAny = false;
        for (std::size_t g = 0; g < groups.size(); ++g) {
            const Group& grp = groups[g];
            bool somewhere = false;
            for (std::size_t t = 0; t < byId.size(); ++t) {
                if (planRoute(graph, vehicles[byId[t]], grp.orders, weight).status
                    == PlanStatus::Ok) {
                    somewhere = true;
                    break;
                }
            }
            if (somewhere || grp.orders.size() < 2) {
                splitGroups.push_back(grp);
                continue;
            }
            std::vector<Order> edd = grp.orders;
            sortOrdersEdd(edd);
            const std::size_t cut = edd.size() / 2;
            Group a = grp;
            a.part = 1;
            a.orders.assign(edd.begin(), edd.begin() + static_cast<std::ptrdiff_t>(cut));
            Group b = grp;
            b.part = 2;
            b.orders.assign(edd.begin() + static_cast<std::ptrdiff_t>(cut), edd.end());
            fleet.allocationLog.push_back(
                "拆簇：簇 " + std::to_string(grp.clusterId) + " 在任一台车上都不可行 ⇒ "
                "按 EDD 前缀拆为 " + std::to_string(a.orders.size()) + " / "
                + std::to_string(b.orders.size()) + " 单给两台车");
            splitGroups.push_back(a);
            splitGroups.push_back(b);
            splitAny = true;
        }
        if (splitAny) {
            groups = splitGroups;
            reduceGroups(groups);
            res = runEnumeration(groups);
        }
    }

    if (!res.any) {
        // ③ planRoute 内部已跑过规格一的降载 / 全量兜底；到这里仍没有全可行排列
        // ⇒ ④ 取「不可行车辆数最少」的候选并把每台车的原因如实报出，
        //    绝不通过违反容量或「先取后送」去凑解。
        fleet.allocationLog.push_back(
            "无全可行排列：取「不可行车辆数最少」的候选做兜底并逐车报原因（兜底链 ④）");
        res.best = res.fallback;
    }

    std::vector<std::vector<Order> > vehicleOrders = res.best.vehicleOrders;
    std::vector<RoutePlan> plans = res.best.plans;
    FleetKey curKey = res.best.key;
    std::vector<RelocateRecord> relocations;

    // ---- 阶段三 · 有界跨车改派（R ≤ 3）----
    //
    // 候选构造缓存：同「车辆 + 订单序列」只跑一次单车构造器。纯加速、**不改变任何结果**
    // （键就是返回值所依赖的全部输入），改派候选在相邻两轮之间大量重复。
    std::map<std::string, RoutePlan> planCache;
    auto buildCached = [&](std::size_t v, const std::vector<Order>& os) -> const RoutePlan& {
        std::string key = std::to_string(v);
        key += "|";
        for (std::size_t i = 0; i < os.size(); ++i) {
            key += os[i].id;
            key += ",";
        }
        const std::map<std::string, RoutePlan>::iterator it = planCache.find(key);
        if (it != planCache.end()) {
            return it->second;
        }
        return planCache.insert(std::make_pair(key, planRoute(graph, vehicles[v], os, weight)))
            .first->second;
    };
    if (res.any) {
        // 普通段订单（非紧急）EDD 序：紧急段由订单级 FIFO 完全固定，relocate 对它是恒等。
        std::vector<Order> normal;
        for (std::size_t v = 0; v < vehicleOrders.size(); ++v) {
            for (std::size_t i = 0; i < vehicleOrders[v].size(); ++i) {
                if (!vehicleOrders[v][i].urgent) {
                    normal.push_back(vehicleOrders[v][i]);
                }
            }
        }
        sortOrdersEdd(normal);
        for (int round = 1; round <= 3; ++round) {
            bool improved = false;
            for (std::size_t oi = 0; oi < normal.size(); ++oi) {
                const std::string orderId = normal[oi].id;
                const int src = vehicleOfOrder(vehicleOrders, orderId);
                if (src < 0) {
                    continue;
                }
                for (std::size_t t = 0; t < byId.size(); ++t) {
                    const std::size_t dst = byId[t];
                    if (static_cast<int>(dst) == src) {
                        continue;
                    }
                    const std::vector<Order> srcOrders =
                        withoutOrder(vehicleOrders[static_cast<std::size_t>(src)], orderId);
                    std::vector<Order> dstOrders = vehicleOrders[dst];
                    dstOrders.push_back(normal[oi]);
                    // 用**各自的**因子重放（planRoute 内按车缩放）。
                    const RoutePlan& srcPlan =
                        buildCached(static_cast<std::size_t>(src), srcOrders);
                    if (srcPlan.status != PlanStatus::Ok) {
                        continue;
                    }
                    const RoutePlan& dstPlan = buildCached(dst, dstOrders);
                    if (dstPlan.status != PlanStatus::Ok) {
                        continue;
                    }
                    std::vector<RoutePlan> candPlans = plans;
                    candPlans[static_cast<std::size_t>(src)] = srcPlan;
                    candPlans[dst] = dstPlan;
                    const FleetKey candKey = fleetKeyOf(vehicles, candPlans, weight);
                    // 两条接受条件**同时**成立：
                    //   ① 车队键严格变优；② 完工耗时不得变差。
                    if (compareFleetKey(candKey, curKey) < 0
                        && candKey.completion <= curKey.completion + kFleetEps) {
                        RelocateRecord rec;
                        rec.round = round;
                        rec.orderId = orderId;
                        rec.fromVehicleId = vehicles[static_cast<std::size_t>(src)].id;
                        rec.toVehicleId = vehicles[dst].id;
                        rec.penaltyBefore = curKey.penalty;
                        rec.penaltyAfter = candKey.penalty;
                        rec.weightBefore = curKey.weight;
                        rec.weightAfter = candKey.weight;
                        rec.balanceBefore = curKey.balance;
                        rec.balanceAfter = candKey.balance;
                        rec.completionBeforeMin = curKey.completion;
                        rec.completionAfterMin = candKey.completion;
                        relocations.push_back(rec);
                        plans = candPlans;
                        vehicleOrders[static_cast<std::size_t>(src)] = srcOrders;
                        vehicleOrders[dst] = dstOrders;
                        curKey = candKey;
                        improved = true;
                        break;
                    }
                }
            }
            if (!improved) {
                break;
            }
        }
    }
    for (std::size_t i = 0; i < res.best.crossRejected.size(); ++i) {
        fleet.allocationLog.push_back("跨簇单 " + res.best.crossRejected[i]
                                      + " 没有可行车 ⇒ 拒绝并报原因（兜底链 ④）");
    }

    // ---- 后置有界改进 · 跨车转运（规格二 §5 / #28）----
    //
    // 只在三条判据之一成立时构造候选（默认数据全部不触发 ⇒ 一行候选都不生成，
    // 这是 --plan-summary 仍 < 5s 的前提）。转运层**不改分配**：谁取货仍由 #40 决定，
    // 只评估「给定分配下这一单改由另一台车送达是否严格更优」。
    FleetTransitResult transitResult;
    transitResult.plans = plans;
    if (res.any) {
        std::vector<VehicleStartState> starts(vehicles.size());
        for (std::size_t v = 0; v < vehicles.size(); ++v) {
            starts[v].startNodeId = vehicles[v].startNodeId;
            starts[v].startTimeMin = vehicles[v].departTimeMin;
        }
        transitResult = applyTransitLayer(graph, vehicles, vehicleOrders, plans, starts, weight);
        if (transitResult.acceptedRounds > 0) {
            plans = transitResult.plans;
        }
    }

    // ---- 收口：汇总 + 分配产物（日志先存下，aggregateFleet 返回的是新对象）----
    const std::vector<std::string> allocLog = fleet.allocationLog;
    const std::vector<std::string> keyLog = fleet.candidateKeyLog;
    fleet = aggregateFleet(vehicles, plans);
    fleet.allocationLog = allocLog;
    fleet.candidateKeyLog = keyLog;
    fleet.transitLegs = transitResult.legs;
    fleet.transitLog = transitResult.log;
    if (transitResult.acceptedRounds > 0) {
        for (std::size_t i = 0; i < transitResult.log.size(); ++i) {
            fleet.allocationLog.push_back(transitResult.log[i]);
        }
    }
    fillAllocationFields(fleet, graph, vehicles, orders, vehicleOrders);
    fleet.relocations = relocations;
    // 腿结构（规格三 §1 / #45）：**汇总与分配表都定稿之后**才展开 —— 执行车与时刻
    // 都读最终值，避免「先展开、再改派/转运」让腿的归属与实际分配对不上。
    fleet.legs = buildLegs(graph, vehicles, orders, fleet, fleet.legLog);
    // 兜底链 ④：任何没被服务到的订单都要点名原因（不静默丢单）。
    for (std::size_t i = 0; i < orders.size(); ++i) {
        bool served = false;
        for (std::size_t v = 0; v < vehicleOrders.size() && !served; ++v) {
            served = containsOrder(vehicleOrders[v], orders[i].id);
        }
        if (served) {
            continue;
        }
        bool logged = false;
        for (std::size_t j = 0; j < fleet.allocationLog.size(); ++j) {
            if (fleet.allocationLog[j].find(orders[i].id) != std::string::npos) {
                logged = true;
                break;
            }
        }
        if (!logged) {
            fleet.allocationLog.push_back("拒绝 " + orders[i].id
                                          + "：没有可服务的车辆（兜底链用尽）");
        }
    }
    return fleet;
}

// ---- 阶段一判据与边际代价（规格二 §4.2 / §4.7；#27 直接用）----
int clusterOfOrder(const LogisticsGraph& graph, const Order& order) {
    const Node* a = graph.findNode(order.pickupNodeId);
    return (a == nullptr) ? -1 : a->subNetworkId;
}

bool isCrossCluster(const LogisticsGraph& graph, const Order& order) {
    const Node* a = graph.findNode(order.pickupNodeId);
    const Node* b = graph.findNode(order.nodeId);
    if (a == nullptr || b == nullptr) {
        return false;
    }
    return a->subNetworkId != b->subNetworkId;
}

MarginalCost marginalCost(const LogisticsGraph& graph, const Vehicle& vehicle,
                          const Order& order, const std::vector<Order>& currentVehicleOrders,
                          const RoutePlan& currentPlan, WeightType weight) {
    std::vector<Order> candOrders = currentVehicleOrders;
    candOrders.push_back(order);
    const RoutePlan cand = planRoute(graph, vehicle, candOrders, weight);
    if (cand.status != PlanStatus::Ok) {
        // 不可行候选：三个分量都记 +∞（调用方按「不可行」处理，不参与取优）。
        const double inf = std::numeric_limits<double>::infinity();
        MarginalCost mc;
        mc.penaltyDelta = inf;
        mc.weightDelta = inf;
        mc.balanceDelta = inf;
        return mc;
    }
    return marginalCostBetween(currentPlan, cand, vehicle, weight);
}

InsertResult insertOrder(const LogisticsGraph& graph,
                          const Vehicle& vehicle,
                          const std::vector<Order>& remainingOrders,
                          const Order& newOrder,
                          const std::string& currentPositionId,
                          int currentTimeMin,
                          WeightType weight,
                          const std::vector<OnboardItem>& onboard,
                          const StashState& stash) {
    InsertResult result;

    // urgent 标记**由调用方给出、本函数照用**（MC-B04 退役了「强制 urgent」）。
    // 紧急单 -> 紧急段尾插；普通单 -> 普通段 pair-insertion；两条链都在 replan 的构造器里。
    const Order& inserted = newOrder;

    std::vector<Order> all = remainingOrders;
    all.push_back(inserted);

    // 可达性与窗口：按 current -> A(取货) -> B(送达) 的**该车缩放后**耗时估计。
    if (inserted.pickupNodeId.empty()) {
        result.warning = "取送单 " + inserted.id + " 缺少取货点 A";
    } else {
        const PathResult toA =
            shortestPath(graph, currentPositionId, inserted.pickupNodeId, WeightType::Time);
        const PathResult ab =
            shortestPath(graph, inserted.pickupNodeId, inserted.nodeId, WeightType::Time);
        if (!toA.found) {
            result.warning = "取货点 A 不可达: " + inserted.pickupNodeId;
        } else if (!ab.found) {
            result.warning = "送达点 B 不可达: " + inserted.nodeId;
        } else {
            const double scaled =
                (toA.totalWeight + ab.totalWeight) * vehicle.timeFactor;
            const int earliestArrival =
                currentTimeMin + static_cast<int>(std::lround(scaled));
            if (inserted.windowEndMin < earliestArrival) {
                result.warning = "订单 " + inserted.id + " 无法在窗口内送达（窗口止 "
                                 + std::to_string(inserted.windowEndMin) + "，最早到达 "
                                 + std::to_string(earliestArrival) + "）";
            }
        }
    }

    result.plan = replan(graph, vehicle, all, currentPositionId, currentTimeMin, weight, onboard,
                         stash);
    return result;
}

RoutePlan sliceRemainder(const RoutePlan& plan, std::size_t fromNodeIndex) {
    RoutePlan remainder;
    remainder.status = PlanStatus::Ok;
    if (plan.nodes.empty()) {
        return remainder;
    }
    const std::size_t from = (fromNodeIndex < plan.nodes.size()) ? fromNodeIndex : 0;

    for (std::size_t i = from; i < plan.nodes.size(); ++i) {
        remainder.nodes.push_back(plan.nodes[i]);
        remainder.nodeArrivalMin.push_back(i < plan.nodeArrivalMin.size()
                                               ? plan.nodeArrivalMin[i]
                                               : 0);
        remainder.nodeIsStop.push_back(i < plan.nodeIsStop.size() ? plan.nodeIsStop[i] : false);
    }
    // 停靠只保留「下标 >= from」的那些（nodeIsStop 与 stops 一一对应）。
    std::size_t stopIdx = 0;
    for (std::size_t i = 0; i < plan.nodes.size(); ++i) {
        if (!(i < plan.nodeIsStop.size() && plan.nodeIsStop[i])) {
            continue;
        }
        if (i >= from && stopIdx < plan.stops.size()) {
            remainder.stops.push_back(plan.stops[stopIdx]);
        }
        ++stopIdx;
    }
    for (const OffloadLeg& leg : plan.offloads) {
        if (leg.reclaimTimeMin < 0 || leg.reclaimTimeMin >= 0) {
            remainder.offloads.push_back(leg);
        }
    }
    return remainder;
}

// 顺序不变版重放（规格二 §10 裁定 1 / ADR 0007）。
//
// 停靠顺序完全不变，只把剩余停靠序列按**新边权重**重放一遍。
// 具体做法：把上一版剩余停靠的动作集原样搬进一次新的构造上下文（订单表仍是
// 剩余订单，在车货仍是当前在车货），然后按同一顺序重放时间与载重。
RoutePlan replanIncremental(const LogisticsGraph& graph,
                            const Vehicle& vehicle,
                            const std::vector<Order>& remainingOrders,
                            const RoutePlan& previous,
                            int currentTimeMin,
                            WeightType weight,
                            const std::vector<OnboardItem>& onboard,
                            bool* usedIncremental,
                            const StashState& stash) {
    if (usedIncremental != nullptr) {
        *usedIncremental = false;
    }
    const std::string startPos =
        previous.nodes.empty() ? vehicle.startNodeId : previous.nodes.front();

    // 重放的前提：上一版有完整序列与停靠。
    if (previous.status != PlanStatus::Ok || previous.nodes.size() < 2
        || previous.stops.empty()) {
        return replan(graph, vehicle, remainingOrders, startPos, currentTimeMin, weight, onboard,
                      stash);
    }

    Ctx ctx;
    std::string reason;
    if (!makeCtx(graph, vehicle, remainingOrders, startPos, currentTimeMin, weight, onboard, stash,
                 ctx, reason)) {
        return replan(graph, vehicle, remainingOrders, startPos, currentTimeMin, weight, onboard,
                      stash);
    }

    // 把上一版停靠原样搬成 StopSpec（顺序不变）。
    std::vector<StopSpec> specs;
    specs.reserve(previous.stops.size());
    for (const Stop& s : previous.stops) {
        StopSpec spec;
        spec.nodeId = s.nodeId;
        spec.phase = s.phase;
        spec.pickups = s.pickups;
        spec.offloads = s.offloads;
        spec.reclaims = s.reclaims;
        for (const StopDelivery& d : s.deliveries) {
            spec.deliveries.push_back(d.orderId);
        }
        specs.push_back(spec);
    }

    Eval ev;
    if (!replay(ctx, specs, false, ev)) {
        // 重放不可行（leg 不可达 / 载重破坏）=> 退回全量重规划。
        return replan(graph, vehicle, remainingOrders, startPos, currentTimeMin, weight, onboard,
                      stash);
    }

    if (usedIncremental != nullptr) {
        *usedIncremental = true;
    }
    Outcome out;
    out.ok = true;
    out.route = specs;
    out.stashStation = stash.station;
    for (const OnboardItem& it : stash.items) {
        out.stashOrders.push_back(it.orderId);
    }
    RoutePlan plan;
    finalize(ctx, out, plan);
    return plan;
}

bool isEdgeOnRoute(const std::vector<std::string>& routeNodes,
                   const std::string& fromId,
                   const std::string& toId) {
    for (std::size_t i = 1; i < routeNodes.size(); ++i) {
        if (routeNodes[i - 1] == fromId && routeNodes[i] == toId) {
            return true;
        }
    }
    return false;
}

std::size_t nodeIndexAtTime(const RoutePlan& plan, int timeMin) {
    if (plan.nodes.empty()) {
        return 0;   // 没有节点时下标无意义，按契约返回 0（调用方需自行判空）
    }

    // **语义 = 「半路显示所在段的起点」**：取最大的 i 使 nodeArrivalMin[i] <= timeMin。
    // 车在 i -> i+1 途中时，最后一个已到达的节点就是 i，显示 i 即段的起点。
    // 不依赖"到达时刻单调不减"这一前提（虽然它成立），全表扫描取**最后一个**满足者，
    // 这样即使上游给了非单调的序列，返回值仍严格符合上面的字面定义。
    const std::size_t n = plan.nodeArrivalMin.size() < plan.nodes.size()
                              ? plan.nodeArrivalMin.size()
                              : plan.nodes.size();
    std::size_t best = 0;
    bool found = false;
    for (std::size_t i = 0; i < n; ++i) {
        if (plan.nodeArrivalMin[i] <= timeMin) {
            best = i;
            found = true;
        }
    }
    if (!found) {
        return 0;   // 早于首节点到达时刻：车还在起点
    }
    // 下标一律 clamp 到 [0, nodes.size()-1]
    if (best >= plan.nodes.size()) {
        best = plan.nodes.size() - 1;
    }
    return best;
}


// ===========================================================================
// 动态事件复调度（规格二 §6 / #27）——实现
// ===========================================================================
namespace {

// 把停靠序列翻成重放规格（R1「顺序不变版」的唯一来源）。
std::vector<StopSpec> specsOfStops(const std::vector<Stop>& stops) {
    // 复用唯一的「Stop -> StopSpec」搬运点 specFromStop（原实现逐字段复刻，
    // StopSpec 加字段时会漏搬；评审 F1）。
    std::vector<StopSpec> specs;
    specs.reserve(stops.size());
    for (std::size_t i = 0; i < stops.size(); ++i) {
        specs.push_back(specFromStop(stops[i]));
    }
    return specs;
}

struct PlanKey {
    int    penalty = 0;
    double weight = 0.0;
    std::vector<StopSpec> specs;   // tieKey 用（并列才比较）
};

PlanKey makePlanKey(const RoutePlan& plan, WeightType weight) {
    PlanKey k;
    k.penalty = plan.totalPenaltyMin;
    k.weight = strategyWeightOf(plan, weight);
    k.specs = specsOfStops(plan.stops);
    return k;
}

// 两版比较用的完整三键：penalty -> 策略权重（该车缩放后）-> tieKey。
// <0 表示 a 更好，0 表示完全并列（并列保留 R1）。
int comparePlanKey(const PlanKey& a, const PlanKey& b) {
    Key ka;
    ka.penalty = a.penalty;
    ka.weight = a.weight;
    Key kb;
    kb.penalty = b.penalty;
    kb.weight = b.weight;
    const int c = compareKey(ka, kb);
    if (c != 0) {
        return c;
    }
    return compareTie(a.specs, b.specs);
}

bool planUsable(const RoutePlan& plan) {
    return plan.status == PlanStatus::Ok && !plan.nodes.empty();
}

// 计划是否覆盖这批订单的**送达**（转运卸入也算：货已交给接力车，由它送达）。
// 顺序不变版若漏掉一张单 ⇒ 不可行（不变量 14「不丢单」）。
bool planCoversOrders(const RoutePlan& plan, const std::vector<Order>& orders,
                      std::string& missing) {
    for (std::size_t i = 0; i < orders.size(); ++i) {
        bool found = false;
        for (std::size_t s = 0; s < plan.stops.size() && !found; ++s) {
            const Stop& st = plan.stops[s];
            for (std::size_t d = 0; d < st.deliveries.size() && !found; ++d) {
                if (st.deliveries[d].orderId == orders[i].id) {
                    found = true;
                }
            }
            for (std::size_t t = 0; t < st.transitUnloads.size() && !found; ++t) {
                if (st.transitUnloads[t] == orders[i].id) {
                    found = true;
                }
            }
        }
        if (!found) {
            missing = orders[i].id;
            return false;
        }
    }
    return true;
}

// R1：停靠顺序完全不变，只把「剩余停靠序列」按**当前边权重**重放一遍。
// 任一 leg 不可达 / 前缀载重越界 / 漏单 ⇒ 该版不可行（status != Ok，原因在 reason）。
RoutePlan replayPreservingOrder(const LogisticsGraph& graph, const Vehicle& vehicle,
                                const std::vector<Order>& remainingOrders,
                                const RoutePlan& previousRemaining,
                                const std::string& startNodeId, int startTimeMin,
                                WeightType weight, const std::vector<OnboardItem>& onboard,
                                const StashState& stash, std::string& why) {
    why.clear();
    RoutePlan out;
    out.status = PlanStatus::Unreachable;
    // 无剩余订单时「顺序不变版」是平凡可行的（车就地不动、直接回仓库）：
    // 不能因为"上一版没有停靠序列"就把空车判成不可行（否则 Manual 事件下
    // 每台空车都会被误判为第 ① 级失败）。
    if (previousRemaining.status != PlanStatus::Ok
        || (previousRemaining.stops.empty() && !remainingOrders.empty())) {
        why = "上一版没有剩余停靠序列";
        out.reason = why;
        return out;
    }
    std::string missing;
    if (!planCoversOrders(previousRemaining, remainingOrders, missing)) {
        why = "顺序不变版缺少订单 " + missing + " 的送达动作";
        out.reason = why;
        return out;
    }
    Ctx ctx;
    std::string reason;
    if (!makeCtx(graph, vehicle, remainingOrders, startNodeId, startTimeMin, weight, onboard,
                 stash, ctx, reason)) {
        why = reason;
        out.reason = reason;
        return out;
    }
    const std::vector<StopSpec> specs = specsOfStops(previousRemaining.stops);
    Eval ev;
    if (!replay(ctx, specs, true, ev)) {
        why = ev.reason.empty() ? std::string("重放不可行") : ev.reason;
        out.reason = why;
        return out;
    }
    Outcome oc;
    oc.ok = true;
    oc.route = specs;
    oc.stashStation = stash.station;
    for (std::size_t i = 0; i < stash.items.size(); ++i) {
        oc.stashOrders.push_back(stash.items[i].orderId);
    }
    finalize(ctx, oc, out);
    if (!planUsable(out)) {
        why = out.reason.empty() ? std::string("重放后路线不可用") : out.reason;
    }
    return out;
}

// 从各车**当前状态**重规划（已执行前缀不重排）。allowOffload 只由升级链的分级决定。
RoutePlan replanFromState(const LogisticsGraph& graph, const Vehicle& vehicle,
                          const std::vector<Order>& orders, const VehicleStartState& start,
                          WeightType weight, bool allowOffload) {
    return replan(graph, vehicle, orders, start.startNodeId, start.startTimeMin, weight,
                  start.onboard, start.stash, allowOffload);
}

// 复调度里「要挪走的订单」的尝试顺序：紧急单整体优先（FIFO：窗口起点、订单 ID），
// 普通单按 EDD（窗口止、订单 ID）。与构造器的排序口径一致（手写插入排序）。
void sortRescheduleCandidates(std::vector<Order>& v) {
    for (std::size_t i = 1; i < v.size(); ++i) {
        const Order key = v[i];
        std::size_t j = i;
        while (j > 0) {
            const Order& prev = v[j - 1];
            bool move = false;
            if (prev.urgent != key.urgent) {
                move = key.urgent;
            } else if (key.urgent) {
                move = (key.windowStartMin < prev.windowStartMin)
                       || (key.windowStartMin == prev.windowStartMin && key.id < prev.id);
            } else {
                move = (key.windowEndMin < prev.windowEndMin)
                       || (key.windowEndMin == prev.windowEndMin && key.id < prev.id);
            }
            if (!move) {
                break;
            }
            v[j] = prev;
            --j;
        }
        v[j] = key;
    }
}

bool containsOrderId(const std::vector<Order>& v, const std::string& id) {
    for (std::size_t i = 0; i < v.size(); ++i) {
        if (v[i].id == id) {
            return true;
        }
    }
    return false;
}

std::vector<Order> ordersWithout(const std::vector<Order>& v, const std::string& id) {
    std::vector<Order> out;
    out.reserve(v.size());
    for (std::size_t i = 0; i < v.size(); ++i) {
        if (v[i].id != id) {
            out.push_back(v[i]);
        }
    }
    return out;
}

// 车队内部用的键（第 ② 级的「候选 vs 当前」比较）：两台车合起来的 (Σpenalty, Σ权重)。
struct PairKey {
    int    penalty = 0;
    double weight = 0.0;
    std::vector<StopSpec> specs;   // 收尾的确定性比较（先 v 后 d 的停靠序列）
};

PairKey pairKeyOf(const RoutePlan& vPlan, const RoutePlan& dPlan, WeightType weight) {
    PairKey k;
    k.penalty = vPlan.totalPenaltyMin + dPlan.totalPenaltyMin;
    k.weight = strategyWeightOf(vPlan, weight) + strategyWeightOf(dPlan, weight);
    k.specs = specsOfStops(vPlan.stops);
    const std::vector<StopSpec> d = specsOfStops(dPlan.stops);
    k.specs.insert(k.specs.end(), d.begin(), d.end());
    return k;
}

int comparePairKey(const PairKey& a, const PairKey& b) {
    Key ka;
    ka.penalty = a.penalty;
    ka.weight = a.weight;
    Key kb;
    kb.penalty = b.penalty;
    kb.weight = b.weight;
    const int c = compareKey(ka, kb);
    if (c != 0) {
        return c;
    }
    return compareTie(a.specs, b.specs);
}

// 「不可行」时把它当成 +inf 键：可行候选恒更优（第 ② 级的当前侧可能是不可行计划）。
PairKey pairKeyInfeasible() {
    PairKey k;
    k.penalty = std::numeric_limits<int>::max();
    k.weight = std::numeric_limits<double>::infinity();
    return k;
}

// Δkey 升序的候选车顺序（并列按车辆 ID 升序）。§6.6 / §6.4② 共用同一把尺子。
std::vector<std::size_t> vehiclesByMarginal(const LogisticsGraph& graph,
                                            const std::vector<Vehicle>& vehicles,
                                            const std::vector<std::vector<Order> >& vehicleOrders,
                                            const std::vector<RoutePlan>& plans, WeightType weight,
                                            const Order& order, std::size_t exclude) {
    struct Row {
        std::size_t v = 0;
        MarginalCost mc;
        bool feasible = false;
    };
    std::vector<Row> rows;
    for (std::size_t v = 0; v < vehicles.size(); ++v) {
        if (v == exclude) {
            continue;
        }
        Row row;
        row.v = v;
        std::vector<Order> candOrders = vehicleOrders[v];
        if (!containsOrderId(candOrders, order.id)) {
            candOrders.push_back(order);
        }
        const RoutePlan cand = planRoute(graph, vehicles[v], candOrders, weight);
        row.feasible = planUsable(cand);
        row.mc = row.feasible ? marginalCostBetween(plans[v], cand, vehicles[v], weight)
                              : MarginalCost{std::numeric_limits<double>::infinity(),
                                             std::numeric_limits<double>::infinity(),
                                             std::numeric_limits<double>::infinity()};
        rows.push_back(row);
    }
    // 手写插入排序：Δkey 升序（并列按车辆 ID 升序；不可行恒在最后）。
    for (std::size_t i = 1; i < rows.size(); ++i) {
        const Row key = rows[i];
        std::size_t j = i;
        while (j > 0) {
            const Row& prev = rows[j - 1];
            bool move = false;
            if (prev.feasible != key.feasible) {
                move = key.feasible;
            } else {
                const int c = compareMarginal(key.mc, prev.mc);
                if (c < 0) {
                    move = true;
                } else if (c == 0 && vehicles[key.v].id < vehicles[prev.v].id) {
                    move = true;
                }
            }
            if (!move) {
                break;
            }
            rows[j] = prev;
            --j;
        }
        rows[j] = key;
    }
    std::vector<std::size_t> out;
    out.reserve(rows.size());
    for (std::size_t i = 0; i < rows.size(); ++i) {
        out.push_back(rows[i].v);
    }
    return out;
}

// 从计划里读出该单的**送达**到达时刻；找不到返回 -1。
int deliveryArrivalOf(const RoutePlan& plan, const std::string& orderId) {
    for (std::size_t s = 0; s < plan.stops.size(); ++s) {
        for (std::size_t d = 0; d < plan.stops[s].deliveries.size(); ++d) {
            if (plan.stops[s].deliveries[d].orderId == orderId) {
                return plan.stops[s].arrivalMin;
            }
        }
    }
    return -1;
}

// 从计划里读出该单在某站的**取走（取回）**到达时刻；找不到返回 -1。
int pickupArrivalOf(const RoutePlan& plan, const std::string& stationId,
                    const std::string& orderId) {
    for (std::size_t s = 0; s < plan.stops.size(); ++s) {
        if (plan.stops[s].nodeId != stationId) {
            continue;
        }
        for (std::size_t r = 0; r < plan.stops[s].reclaims.size(); ++r) {
            if (plan.stops[s].reclaims[r] == orderId) {
                return plan.stops[s].arrivalMin;
            }
        }
        for (std::size_t t = 0; t < plan.stops[s].transitPickups.size(); ++t) {
            if (plan.stops[s].transitPickups[t] == orderId) {
                return plan.stops[s].arrivalMin;
            }
        }
    }
    return -1;
}

std::string vehicleIdOf(const std::vector<Vehicle>& vehicles, std::size_t v) {
    return (v < vehicles.size()) ? vehicles[v].id : std::string("?");
}

}   // namespace

LocalReplanOutcome localReplanTwoVersion(const LogisticsGraph& graph, const Vehicle& vehicle,
                                         const RoutePlan& previousRemaining,
                                         const std::vector<Order>& remainingOrders,
                                         const std::string& startNodeId, int startTimeMin,
                                         WeightType weight,
                                         const std::vector<OnboardItem>& onboard,
                                         const StashState& stash) {
    LocalReplanOutcome r;
    std::string why;
    r.byReplay = replayPreservingOrder(graph, vehicle, remainingOrders, previousRemaining,
                                       startNodeId, startTimeMin, weight, onboard, stash, why);
    r.replayFeasible = planUsable(r.byReplay);

    // R2：对该车剩余订单重跑规格一构造器（pair-insertion + repair R≤3）。
    // **不带降载**：降载属第 ④ 级（改变货物路径的重手段），这里传 false。
    r.byRebuild = replanFromState(graph, vehicle, remainingOrders,
                                  VehicleStartState{startNodeId, startTimeMin, onboard, stash},
                                  weight, false);
    r.rebuildFeasible = planUsable(r.byRebuild);

    if (r.replayFeasible) {
        const PlanKey k1 = makePlanKey(r.byReplay, weight);
        r.penaltyReplay = k1.penalty;
        r.weightReplay = k1.weight;
        if (r.rebuildFeasible) {
            const PlanKey k2 = makePlanKey(r.byRebuild, weight);
            r.penaltyRebuild = k2.penalty;
            r.weightRebuild = k2.weight;
            // **严格字典序更优才改序**；完全并列保留 R1（停靠顺序不变版）。
            r.adoptedRebuild = (comparePlanKey(k2, k1) < 0);
        }
        r.chosen = r.adoptedRebuild ? r.byRebuild : r.byReplay;
        r.feasible = true;
        return r;
    }
    // R1 不可行 ⇒ 本级失败（§6.3 的字面口径：R1 也不可行则进第 ② 级）。
    // 仍把 R2 的可行性与键值带出去，供播报与守卫如实显示「重排版可行但顺序不变版不可行」。
    if (r.rebuildFeasible) {
        const PlanKey k2 = makePlanKey(r.byRebuild, weight);
        r.penaltyRebuild = k2.penalty;
        r.weightRebuild = k2.weight;
    }
    r.feasible = false;
    r.chosen = r.byReplay;
    return r;
}

std::vector<std::size_t> affectedVehiclesForTraffic(
    const std::vector<RoutePlan>& remainingPlans, const std::vector<TrafficChange>& changes,
    double thresholdRatio) {
    std::vector<std::size_t> out;
    for (std::size_t v = 0; v < remainingPlans.size(); ++v) {
        bool hit = false;
        for (std::size_t c = 0; c < changes.size() && !hit; ++c) {
            if (changes[c].increaseRatio + 1e-12 < thresholdRatio) {
                continue;   // 增幅不足：这条改动对该车不构成「受影响」
            }
            if (isEdgeOnRoute(remainingPlans[v].nodes, changes[c].fromId, changes[c].toId)) {
                hit = true;
            }
        }
        if (hit) {
            out.push_back(v);
        }
    }
    return out;
}

std::vector<std::size_t> affectedVehiclesForClosure(
    const std::vector<RoutePlan>& remainingPlans,
    const std::vector<TrafficChange>& closedEdges) {
    std::vector<std::size_t> out;
    for (std::size_t v = 0; v < remainingPlans.size(); ++v) {
        bool hit = false;
        for (std::size_t c = 0; c < closedEdges.size() && !hit; ++c) {
            // 封路是双向的：任一方向出现在剩余路线上都算用到。
            if (isEdgeOnRoute(remainingPlans[v].nodes, closedEdges[c].fromId,
                              closedEdges[c].toId)
                || isEdgeOnRoute(remainingPlans[v].nodes, closedEdges[c].toId,
                                 closedEdges[c].fromId)) {
                hit = true;
            }
        }
        if (hit) {
            out.push_back(v);
        }
    }
    return out;
}

NewOrderAssignment chooseVehicleForNewOrder(const LogisticsGraph& graph,
                                            const std::vector<Vehicle>& vehicles,
                                            const std::vector<std::vector<Order> >& vehicleOrders,
                                            const std::vector<RoutePlan>& currentPlans,
                                            const std::vector<VehicleStartState>& starts,
                                            WeightType weight, const Order& order) {
    NewOrderAssignment res;
    if (vehicles.empty()) {
        return res;
    }
    res.candidates = vehiclesByMarginal(graph, vehicles, vehicleOrders, currentPlans, weight, order,
                                        vehicles.size());
    for (std::size_t i = 0; i < res.candidates.size(); ++i) {
        const std::size_t v = res.candidates[i];
        std::vector<Order> candOrders = vehicleOrders[v];
        if (!containsOrderId(candOrders, order.id)) {
            candOrders.push_back(order);
        }
        const RoutePlan cand = replanFromState(graph, vehicles[v], candOrders, starts[v], weight,
                                              /*allowOffload=*/false);
        if (!planUsable(cand)) {
            continue;   // 不可行 ⇒ 试下一台（Δkey 升序）
        }
        // Δkey 必须由**该车自己的因子**重放（上面 replanFromState 已用该车因子）。
        if (planUsable(currentPlans[v])) {
            const MarginalCost mc =
                marginalCostBetween(currentPlans[v], cand, vehicles[v], weight);
            if (mc.penaltyDelta == std::numeric_limits<double>::infinity()) {
                continue;
            }
        }
        res.feasible = true;
        res.vehicleIndex = v;
        res.plan = cand;
        return res;
    }
    return res;
}

std::vector<std::vector<Order> > vehicleOrdersFromPlans(const std::vector<RoutePlan>& plans,
                                                        const std::vector<Order>& allOrders) {
    std::vector<std::vector<Order> > out(plans.size());
    for (std::size_t o = 0; o < allOrders.size(); ++o) {
        if (allOrders[o].served) {
            continue;
        }
        const std::string& id = allOrders[o].id;
        std::size_t best = plans.size();
        int bestScore = 0;   // 3 = 送达本车，2 = 取货本车，1 = 转运取走
        for (std::size_t v = 0; v < plans.size(); ++v) {
            for (std::size_t s = 0; s < plans[v].stops.size(); ++s) {
                const Stop& st = plans[v].stops[s];
                for (std::size_t d = 0; d < st.deliveries.size(); ++d) {
                    if (st.deliveries[d].orderId == id && bestScore < 3) {
                        best = v;
                        bestScore = 3;
                    }
                }
                for (std::size_t p = 0; p < st.pickups.size(); ++p) {
                    if (st.pickups[p] == id && bestScore < 2) {
                        best = v;
                        bestScore = 2;
                    }
                }
                for (std::size_t t = 0; t < st.transitPickups.size(); ++t) {
                    if (st.transitPickups[t] == id && bestScore < 1) {
                        best = v;
                        bestScore = 1;
                    }
                }
            }
        }
        if (best < plans.size()) {
            out[best].push_back(allOrders[o]);
        }
    }
    return out;
}

namespace {

// 整批不改：每一车的每一张单都必须在**它自己的**计划里被送达（转运卸入也算）。
// 「不丢单」是硬门槛 —— 分级里任何一版若漏单，就不许被接受。
bool allWorkCovered(const std::vector<std::vector<Order> >& work,
                    const std::vector<RoutePlan>& plans, std::string& missing) {
    for (std::size_t v = 0; v < work.size() && v < plans.size(); ++v) {
        if (!planCoversOrders(plans[v], work[v], missing)) {
            return false;
        }
    }
    return true;
}

// 单台车的「候选 vs 当前」：可行候选 + 严格更优才接受（§6.4② 的接受判据）。
// 当前不可行（没有可用计划）时，任何可行候选都算更优 —— 这就是"失败车"的语义。
bool keyStrictlyBetterThan(const RoutePlan& cand, const RoutePlan& base, WeightType weight) {
    if (!planUsable(cand)) {
        return false;
    }
    if (!planUsable(base)) {
        return true;
    }
    return comparePlanKey(makePlanKey(cand, weight), makePlanKey(base, weight)) < 0;
}

// §6.9 已执行卸入的继承：站里的货（已执行、尚未被取走）必须由**某个可行车**取走并送达。
// 取走车 = argmin Δkey 且可行；站内等待 dwell ≤ 60 否则该候选不可行；期末 0 是硬门槛。
bool inheritExecutedInStation(const LogisticsGraph& graph, const std::vector<Vehicle>& vehicles,
                              WeightType weight, const FleetRescheduleInput& input,
                              std::vector<std::vector<Order> >& work,
                              std::vector<RoutePlan>& plans,
                              std::vector<std::size_t>& replanned,
                              std::vector<std::string>& log, std::string& why) {
    for (std::size_t e = 0; e < input.executedInStation.size(); ++e) {
        const ExecutedTransitEntry& entry = input.executedInStation[e];
        const std::string& station = entry.leg.stationId;
        if (station.empty() || entry.order.id.empty()) {
            continue;
        }
        bool done = false;
        const std::vector<std::size_t> cands = vehiclesByMarginal(
            graph, vehicles, work, plans, weight, entry.order, vehicles.size());
        for (std::size_t c = 0; c < cands.size() && !done; ++c) {
            const std::size_t d = cands[c];
            VehicleStartState st = input.starts[d];
            // 暂存状态只有一个站：已有暂存且不同站 ⇒ 该车接不了这一笔。
            if (!st.stash.station.empty() && st.stash.station != station) {
                continue;
            }
            st.stash.station = station;
            bool hasItem = false;
            for (std::size_t i = 0; i < st.stash.items.size(); ++i) {
                if (st.stash.items[i].orderId == entry.order.id) {
                    hasItem = true;
                }
            }
            if (!hasItem) {
                OnboardItem item;
                item.nodeId = entry.order.nodeId;
                item.kg = entry.order.demandKg;
                item.orderId = entry.order.id;
                st.stash.items.push_back(item);
            }
            std::vector<Order> candOrders = work[d];
            if (!containsOrderId(candOrders, entry.order.id)) {
                candOrders.push_back(entry.order);
            }
            const RoutePlan cand =
                replan(graph, vehicles[d], candOrders, st.startNodeId, st.startTimeMin, weight,
                       st.onboard, st.stash, /*allowOffload=*/true);
            if (!planUsable(cand)) {
                continue;
            }
            const int pick = pickupArrivalOf(cand, station, entry.order.id);
            if (pick < 0) {
                continue;   // 计划里没有"在该站取走"的动作 ⇒ 这一笔接不住
            }
            const int dwell = pick - entry.leg.inTimeMin;
            if (dwell < 0 || dwell > kTransferMaxDwellMin) {
                continue;   // 站内等待超限 ⇒ 该候选不可行
            }
            work[d] = candOrders;
            plans[d] = cand;
            replanned.push_back(d);
            log.push_back("[复调度] 继承已执行卸入 " + entry.order.id + "@" + station + " → "
                          + vehicleIdOf(vehicles, d) + "（站内等待 " + std::to_string(dwell)
                          + "min）");
            done = true;
        }
        if (!done) {
            why = "站内已执行卸入 " + entry.order.id + "@" + station + " 没有可行车能在 "
                  + std::to_string(kTransferMaxDwellMin) + "min 内取走（期末 0 是硬门槛）";
            return false;
        }
    }
    return true;
}

// 第 ④ 级的两版播报：先降载、后转运（顺序固定，保证确定性）。
void appendOffloadLines(const std::vector<Vehicle>& vehicles,
                        const std::vector<RoutePlan>& plans,
                        std::vector<std::string>& log) {
    // 按（站、订单 ID）升序汇总：同一站的多张单合成一行「降载（T01，2 张单）」。
    struct StationOffload {
        std::string station;
        int count = 0;
    };
    std::vector<StationOffload> rows;
    for (std::size_t v = 0; v < plans.size() && v < vehicles.size(); ++v) {
        for (std::size_t o = 0; o < plans[v].offloads.size(); ++o) {
            const std::string& station = plans[v].offloads[o].station;
            std::size_t at = rows.size();
            for (std::size_t i = 0; i < rows.size(); ++i) {
                if (rows[i].station == station) {
                    at = i;
                    break;
                }
            }
            if (at == rows.size()) {
                StationOffload row;
                row.station = station;
                rows.push_back(row);
            }
            ++rows[at].count;
        }
    }
    for (std::size_t i = 1; i < rows.size(); ++i) {
        const StationOffload key = rows[i];
        std::size_t j = i;
        while (j > 0 && key.station < rows[j - 1].station) {
            rows[j] = rows[j - 1];
            --j;
        }
        rows[j] = key;
    }
    for (std::size_t i = 0; i < rows.size(); ++i) {
        log.push_back("[复调度] 降载（" + rows[i].station + "，" + std::to_string(rows[i].count)
                      + " 张单）");
    }
}

void appendTransitLines(const std::vector<TransitLeg>& legs,
                        std::vector<std::string>& log) {
    for (std::size_t i = 0; i < legs.size(); ++i) {
        log.push_back("[复调度] 转运（" + legs[i].stationId + "，" + legs[i].inVehicleId + "→"
                      + legs[i].outVehicleId + "）");
    }
}

std::string vehicleListText(const std::vector<Vehicle>& vehicles,
                            const std::vector<std::size_t>& idx) {
    std::string out;
    for (std::size_t i = 0; i < idx.size(); ++i) {
        if (!out.empty()) {
            out += "、";
        }
        out += vehicleIdOf(vehicles, idx[i]);
    }
    return out;
}

}   // namespace

FleetRescheduleResult rescheduleFleet(const LogisticsGraph& graph,
                                      const std::vector<Vehicle>& vehicles, WeightType weight,
                                      const FleetRescheduleInput& input,
                                      const RescheduleEvent& event) {
    FleetRescheduleResult res;
    res.plans = input.remainingPlans;      // 未被重算的车保持原样
    res.vehicleOrders = input.vehicleOrders;
    const std::size_t k = vehicles.size();
    if (k == 0) {
        res.level = 0;
        return res;
    }

    std::vector<std::vector<Order> > work = input.vehicleOrders;
    std::vector<RoutePlan> plans = input.remainingPlans;
    while (work.size() < k) {
        work.push_back(std::vector<Order>());
    }
    while (plans.size() < k) {
        plans.push_back(RoutePlan());
    }
    const Order pending = (event.trigger == RescheduleTrigger::NewOrder) ? event.newOrder : Order();
    const bool hasPending = (event.trigger == RescheduleTrigger::NewOrder) && !pending.id.empty();

    // ---- 受影响车集合（§6.5）----
    // 第 ① 级失败的车：他们的上一版计划在当前图/当前订单集合下已经**不可实现**
    //（这正是 ① 失败的含义）。第 ② 级比较时把它们的"当前键"视为 +inf ——
    // 否则会拿一张已经走不通的旧计划去和可行候选比大小，改派永远不够"严格更优"。
    std::vector<char> failedFlag(k, 0);

    std::vector<std::size_t> affected;
    if (event.trigger == RescheduleTrigger::Traffic) {
        affected = affectedVehiclesForTraffic(input.remainingPlans, event.changes,
                                              event.thresholdRatio);
    } else if (event.trigger == RescheduleTrigger::RoadClosure) {
        affected = affectedVehiclesForClosure(input.remainingPlans, event.changes);
    } else if (event.trigger == RescheduleTrigger::Manual) {
        for (std::size_t v = 0; v < k; ++v) {
            affected.push_back(v);
        }
    }
    // NewOrder：不重算任何既有车（§6.5 第三行），只对选中的车做插入。
    std::vector<std::size_t> failed;   // 第 ① 级失败的车（进第 ② 级）

    // ---- ① 新订单的选车（§6.6，普通/紧急共用） / 局部重排（§6.3）----
    if (hasPending) {
        const NewOrderAssignment a =
            chooseVehicleForNewOrder(graph, vehicles, work, plans, input.starts, weight, pending);
        if (a.feasible) {
            const std::size_t v = a.vehicleIndex;
            work[v].push_back(pending);
            plans[v] = a.plan;
            res.replanned.push_back(v);
            res.level = 1;
            res.log.push_back("[复调度] 局部重排（1 台车）");
            res.log.push_back("[复调度] 新单 " + pending.id + " 插入 " + vehicleIdOf(vehicles, v)
                              + "（Δkey 升序首个可行）");
            std::string why;
            if (!inheritExecutedInStation(graph, vehicles, weight, input, work, plans,
                                          res.replanned, res.log, why)) {
                res.level = 5;
                res.rejected = true;
                res.reason = why;
                res.log.push_back("[复调度] 拒绝：" + why);
                return res;
            }
            res.plans = plans;
            res.vehicleOrders = work;
            return res;
        }
        res.log.push_back("[复调度] 新单 " + pending.id
                          + " 在全部车上都无法可行插入 → 进入升级链");
    } else if (!affected.empty()) {
        std::vector<LocalReplanOutcome> local(k);
        bool allOk = true;
        for (std::size_t i = 0; i < affected.size(); ++i) {
            const std::size_t v = affected[i];
            local[v] = localReplanTwoVersion(graph, vehicles[v], input.remainingPlans[v], work[v],
                                             input.starts[v].startNodeId,
                                             input.starts[v].startTimeMin, weight,
                                             input.starts[v].onboard, input.starts[v].stash);
            if (!local[v].feasible) {
                allOk = false;
            }
        }
        if (allOk) {
            for (std::size_t i = 0; i < affected.size(); ++i) {
                const std::size_t v = affected[i];
                plans[v] = local[v].chosen;
                res.replanned.push_back(v);
            }
            res.level = 1;
            res.log.push_back("[复调度] 局部重排（" + std::to_string(affected.size())
                              + " 台车）");
            for (std::size_t i = 0; i < affected.size(); ++i) {
                const std::size_t v = affected[i];
                res.log.push_back("[复调度]   " + vehicles[v].id + "："
                                  + (local[v].adoptedRebuild
                                         ? std::string("采用重排版（key 严格更优）")
                                         : std::string("停靠顺序不变（顺序不变版）")));
            }
            std::string why;
            if (!inheritExecutedInStation(graph, vehicles, weight, input, work, plans,
                                          res.replanned, res.log, why)) {
                res.level = 5;
                res.rejected = true;
                res.reason = why;
                res.log.push_back("[复调度] 拒绝：" + why);
                return res;
            }
            res.plans = plans;
            res.vehicleOrders = work;
            return res;
        }
        // 部分车成功：保留它们的 ① 结果；失败车进第 ② 级。
        for (std::size_t i = 0; i < affected.size(); ++i) {
            const std::size_t v = affected[i];
            if (local[v].feasible) {
                plans[v] = local[v].chosen;
                res.replanned.push_back(v);
            } else {
                failed.push_back(v);
                failedFlag[v] = 1;
            }
        }
        res.log.push_back("[复调度] 局部重排失败（" + std::to_string(failed.size())
                          + " 台车无可行解）→ 升级链第 ② 级");
    } else if (affected.empty() && !hasPending) {
        res.level = 0;
        return res;   // 受影响集合为空：不重算任何车（§6.5 的「不重算既有车」）
    }

    // ---- ② 跨车再分配 ----
    bool ok2 = true;
    if (hasPending) {
        bool placed = false;
        const std::vector<std::size_t> cands = vehiclesByMarginal(
            graph, vehicles, work, plans, weight, pending, k);
        for (std::size_t c = 0; c < cands.size() && !placed; ++c) {
            const std::size_t d = cands[c];
            std::vector<Order> candOrders = work[d];
            if (!containsOrderId(candOrders, pending.id)) {
                candOrders.push_back(pending);
            }
            const RoutePlan cand =
                replanFromState(graph, vehicles[d], candOrders, input.starts[d], weight, false);
            // §6.6：新单的接受判据是**可行插入**（Δkey 只决定"先试哪台车"），
            // 不是「比该车现状更优」——否则任何让路线变长的单都永远插不进去。
            if (!planUsable(cand)) {
                continue;
            }
            work[d] = candOrders;
            plans[d] = cand;
            res.replanned.push_back(d);
            res.log.push_back("[复调度] 跨车再分配 " + pending.id + " → "
                              + vehicleIdOf(vehicles, d) + "（新单改派）");
            placed = true;
        }
        if (!placed) {
            ok2 = false;
        }
    }
    if (ok2) {
        for (std::size_t f = 0; f < failed.size() && ok2; ++f) {
            const std::size_t v = failed[f];
            std::vector<Order> cands = work[v];
            sortRescheduleCandidates(cands);
            bool allMoved = true;
            for (std::size_t oi = 0; oi < cands.size() && allMoved; ++oi) {
                const Order o = cands[oi];
                bool moved = false;
                const std::vector<std::size_t> dsts = vehiclesByMarginal(
                    graph, vehicles, work, plans, weight, o, v);
                for (std::size_t di = 0; di < dsts.size() && !moved; ++di) {
                    const std::size_t d = dsts[di];
                    std::vector<Order> vOrders = ordersWithout(work[v], o.id);
                    std::vector<Order> dOrders = work[d];
                    if (!containsOrderId(dOrders, o.id)) {
                        dOrders.push_back(o);
                    }
                    const RoutePlan candV = replanFromState(graph, vehicles[v], vOrders,
                                                           input.starts[v], weight, false);
                    const RoutePlan candD = replanFromState(graph, vehicles[d], dOrders,
                                                           input.starts[d], weight, false);
                    if (!planUsable(candV) || !planUsable(candD)) {
                        continue;
                    }
                    const bool curFeasible = !failedFlag[v] && planUsable(plans[v])
                                             && planUsable(plans[d]);
                    if (curFeasible) {
                        const PairKey candKey = pairKeyOf(candV, candD, weight);
                        const PairKey curKey = pairKeyOf(plans[v], plans[d], weight);
                        if (comparePairKey(candKey, curKey) >= 0) {
                            continue;   // 必须**严格更优**才接受
                        }
                    }
                    work[v] = vOrders;
                    work[d] = dOrders;
                    plans[v] = candV;
                    plans[d] = candD;
                    res.replanned.push_back(v);
                    res.replanned.push_back(d);
                    res.log.push_back("[复调度] 跨车再分配 " + o.id + " "
                                      + vehicleIdOf(vehicles, v) + "→"
                                      + vehicleIdOf(vehicles, d));
                    moved = true;
                }
                if (!moved) {
                    allMoved = false;
                }
            }
            if (!allMoved) {
                ok2 = false;
            }
        }
    }
    // 第 ② 级算「成功」的判据：**失败车都重新拿到了可行计划**（不是"日志非空"），
    // 且整批订单仍然**一张不丢**（不变量 14）。
    for (std::size_t f = 0; f < failed.size() && ok2; ++f) {
        if (!planUsable(plans[failed[f]])) {
            ok2 = false;
        }
    }
    if (ok2) {
        std::string missing;
        if (!allWorkCovered(work, plans, missing)) {
            ok2 = false;
        }
    }
    if (ok2) {
        res.level = 2;
        std::string why;
        if (!inheritExecutedInStation(graph, vehicles, weight, input, work, plans, res.replanned,
                                      res.log, why)) {
            res.level = 5;
            res.rejected = true;
            res.reason = why;
            res.log.push_back("[复调度] 拒绝：" + why);
            return res;
        }
        res.plans = plans;
        res.vehicleOrders = work;
        return res;
    }

    // ---- ③ 全车队重调度：#40 的三阶段全跑 ----
    {
        std::vector<Order> allRemaining;
        for (std::size_t i = 0; i < input.allRemainingOrders.size(); ++i) {
            if (!input.allRemainingOrders[i].served) {
                allRemaining.push_back(input.allRemainingOrders[i]);
            }
        }
        if (hasPending && !containsOrderId(allRemaining, pending.id)) {
            allRemaining.push_back(pending);
        }
        const FleetPlan fp = planFleet(graph, vehicles, allRemaining, weight);
        bool ok3 = fp.allOk && fp.allocationOrderIds.size() == allRemaining.size();
        std::vector<std::vector<Order> > work3(k);
        std::vector<RoutePlan> plans3(k);
        if (ok3) {
            for (std::size_t i = 0; i < fp.allocationOrderIds.size() && ok3; ++i) {
                const std::size_t v = fp.allocationVehicleIndex[i];
                if (v >= k) {
                    ok3 = false;
                    break;
                }
                bool found = false;
                for (std::size_t o = 0; o < allRemaining.size(); ++o) {
                    if (allRemaining[o].id == fp.allocationOrderIds[i]) {
                        work3[v].push_back(allRemaining[o]);
                        found = true;
                        break;
                    }
                }
                if (!found) {
                    ok3 = false;
                }
            }
        }
        if (ok3) {
            for (std::size_t v = 0; v < k && ok3; ++v) {
                plans3[v] = replanFromState(graph, vehicles[v], work3[v], input.starts[v], weight,
                                           false);
                if (!planUsable(plans3[v])) {
                    ok3 = false;
                }
            }
        }
        if (ok3) {
            std::string missing;
            if (!allWorkCovered(work3, plans3, missing)) {
                ok3 = false;   // 漏单（不变量 14）：这一版不许被接受
                res.log.push_back("[复调度] 全车队重调度漏单 " + missing + " → 升级链第 ④ 级");
            }
        }
        if (ok3) {
            work = work3;
            plans = plans3;
            res.replanned.clear();
            for (std::size_t v = 0; v < k; ++v) {
                res.replanned.push_back(v);
            }
            res.level = 3;
            res.log.push_back("[复调度] 全车队重调度");
            std::string why;
            if (!inheritExecutedInStation(graph, vehicles, weight, input, work, plans,
                                          res.replanned, res.log, why)) {
                res.level = 5;
                res.rejected = true;
                res.reason = why;
                res.log.push_back("[复调度] 拒绝：" + why);
                return res;
            }
            res.plans = plans;
            res.vehicleOrders = work;
            return res;
        }
        res.log.push_back("[复调度] 全车队重调度失败 → 升级链第 ④ 级");
    }

    // ---- ④ 降载 / 转运并列比较：**先构造降载版、再构造转运版** ----
    {
        // 待放的新单在第 ④ 级必须**真的落地**（允许降载）：否则「整计划可行」只是因为
        // 悄悄把它丢了 —— 那正是"用违反约束凑解"的反面，一样不允许。
        bool pendingPlaced = !hasPending;
        if (hasPending) {
            const std::vector<std::size_t> cands = vehiclesByMarginal(
                graph, vehicles, work, plans, weight, pending, k);
            for (std::size_t c = 0; c < cands.size() && !pendingPlaced; ++c) {
                const std::size_t d = cands[c];
                std::vector<Order> candOrders = work[d];
                if (!containsOrderId(candOrders, pending.id)) {
                    candOrders.push_back(pending);
                }
                const RoutePlan cand = replanFromState(graph, vehicles[d], candOrders,
                                                       input.starts[d], weight, true);
                if (!planUsable(cand)) {
                    continue;
                }
                work[d] = candOrders;
                plans[d] = cand;
                res.replanned.push_back(d);
                res.log.push_back("[复调度] 降载级新单落位 " + pending.id + " → "
                                  + vehicleIdOf(vehicles, d));
                pendingPlaced = true;
            }
            if (!pendingPlaced) {
                res.level = 5;
                res.rejected = true;
                res.reason = "订单 " + pending.id
                             + " 在全部车上都无法可行插入（容量 / 可达 / 先取后送）";
                res.log.push_back("[复调度] 拒绝：" + res.reason);
                res.plans = input.remainingPlans;
                res.vehicleOrders = input.vehicleOrders;
                return res;
            }
        }
        std::vector<RoutePlan> offloadPlans(k);
        bool offloadOk = true;
        for (std::size_t v = 0; v < k && offloadOk; ++v) {
            offloadPlans[v] = replanFromState(graph, vehicles[v], work[v], input.starts[v], weight,
                                              /*allowOffload=*/true);
            if (!planUsable(offloadPlans[v])) {
                offloadOk = false;
            }
        }
        bool transitOk = false;
        FleetTransitResult transit;
        if (offloadOk) {
            transit = applyTransitLayer(graph, vehicles, work, offloadPlans, input.starts, weight);
            transitOk = transit.plans.size() == k;
            for (std::size_t v = 0; v < k && transitOk; ++v) {
                if (!planUsable(transit.plans[v])) {
                    transitOk = false;
                }
            }
        }
        if (offloadOk) {
            std::string missing;
            if (!allWorkCovered(work, offloadPlans, missing)) {
                offloadOk = false;   // 降载版漏单 ⇒ 不可行（不变量 14）
            }
        }
        if (transitOk) {
            std::string missing;
            if (!allWorkCovered(work, transit.plans, missing)) {
                transitOk = false;
            }
        }
        if (offloadOk || transitOk) {
            bool transitBetter = false;
            std::vector<RoutePlan> chosen;
            if (offloadOk && transitOk) {
                const FleetKey ko = fleetKeyOf(vehicles, offloadPlans, weight);
                const FleetKey kt = fleetKeyOf(vehicles, transit.plans, weight);
                transitBetter = (compareFleetKey(kt, ko) < 0);
                chosen = transitBetter ? transit.plans : offloadPlans;
            } else if (offloadOk) {
                chosen = offloadPlans;
            } else {
                chosen = transit.plans;
            }
            res.level = 4;
            res.replanned.clear();
            for (std::size_t v = 0; v < k; ++v) {
                res.replanned.push_back(v);
            }
            if (offloadOk) {
                appendOffloadLines(vehicles, offloadPlans, res.log);
            }
            if (transitOk) {
                appendTransitLines(transit.legs, res.log);
            }
            if (offloadOk && transitOk && !transitBetter) {
                res.log.push_back("[复调度] 降载与转运并列（或降载更优），保留降载版");
            }
            plans = chosen;
            std::string why;
            if (!inheritExecutedInStation(graph, vehicles, weight, input, work, plans,
                                          res.replanned, res.log, why)) {
                res.level = 5;
                res.rejected = true;
                res.reason = why;
                res.log.push_back("[复调度] 拒绝：" + why);
                return res;
            }
            res.plans = plans;
            res.vehicleOrders = work;
            return res;
        }
        res.log.push_back("[复调度] 降载 / 转运均不可行 → 升级链第 ⑤ 级");
    }

    // ---- ⑤ 拒绝并报原因（不通过违反容量或先取后送去凑解）----
    res.level = 5;
    res.rejected = true;
    if (hasPending) {
        res.reason = "订单 " + pending.id + " 在全部车上都无法可行插入（容量 / 可达 / 先取后送）";
    } else {
        res.reason = "第 ①②③④ 级均产不出可行解（容量 / 可达 / 先取后送 / 紧急 FIFO）";
    }
    res.log.push_back("[复调度] 拒绝：" + res.reason);
    res.plans = input.remainingPlans;      // 拒绝时不留下半成品计划
    res.vehicleOrders = input.vehicleOrders;
    return res;
}
} // namespace logistics