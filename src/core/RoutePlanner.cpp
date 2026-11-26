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

    // 订单 -> (节点 -> 可接时刻)：**车等货的唯一来源**（规格三 §3 / #48）。
    //
    // 为什么不塞进 StopSpec：停靠的「动作集」是构造期真源，而「货什么时候到站」是
    // **前驱腿的执行结果**，与停靠长什么样无关；同一条路线用不同的就绪表重放，得到的
    // 是不同时刻的同一组停靠。把它留在 Ctx（一次构造内不变）才能让「候选枚举 + 整条
    // 重放」这套评价机制原样复用：换时刻只需换表，停靠序列与键的比较一行都不用改。
    //
    // 键是 (订单, 节点) 而不是只用订单：跨簇单在 S_A 与 S_B 各有一个可接时刻。
    // 规划管道按 ①慢车 ②快车 ③收尾 的顺序**只填已规划前驱腿的实际完成时刻**，
    // 所以天然「只前瞻一层」（不做多级连锁预测）。
    std::map<std::string, std::map<std::string, int> > readyAt;

    // 该单在该节点的可接时刻；-1 = 无约束（等价于「货已在站/不走接力」）。
    int readyTimeOf(const std::string& orderId, const std::string& nodeId) const {
        const std::map<std::string, std::map<std::string, int> >::const_iterator o =
            readyAt.find(orderId);
        if (o == readyAt.end()) {
            return -1;
        }
        const std::map<std::string, int>::const_iterator n = o->second.find(nodeId);
        return (n == o->second.end()) ? -1 : n->second;
    }

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
        double wait = (hasWs && maxWs > raw) ? (maxWs - raw) : 0.0;
        // ---- 车等货（规格三 §3 / #48）----
        //
        // 与「早到等窗口」同构：旧的等待只有「货等车」的反面（车早到、等客户开门），
        // 转运则要求「货先到站、车再到」⇒ 时刻链为负 ⇒ 转运恒不可行。这条给车辆补上
        // **原地等待**的能力：含转运取走动作的停靠要等到该 (订单, 节点) 的就绪时刻。
        //
        // 两个常数在这里各管一个方向，都由**构造**保证（候选不可行即被枚举淘汰）：
        //   · 货在**未来**（ready > raw）：最多前瞻 kLookaheadWindowMin，超窗不纳入；
        //   · 货在**过去**（raw > ready）：货物在站停留最多 kTransferMaxDwellMin。
        // 等待只进 elapsed（⇒ 计入该车耗时、完工耗时），**不进 penalty**：penalty 逐单
        // 只由送达窗口决定（下面那段一行不动），否则等待会冤枉客户。
        for (const std::string& oid : s.transitPickups) {
            const int ready = ctx.readyTimeOf(oid, s.nodeId);
            if (ready < 0) {
                continue;
            }
            const double need = static_cast<double>(ready) - raw;
            if (need > static_cast<double>(kLookaheadWindowMin) + kEps) {
                out.reason = "在 " + s.nodeId + " 等订单 " + oid + " 就绪需 "
                             + std::to_string(static_cast<int>(std::lround(need)))
                             + "min，超出前瞻窗口 " + std::to_string(kLookaheadWindowMin)
                             + "min";
                return false;
            }
            if (-need > static_cast<double>(kTransferMaxDwellMin) + kEps) {
                out.reason = "订单 " + oid + " 在 " + s.nodeId + " 已停留 "
                             + std::to_string(static_cast<int>(std::lround(-need)))
                             + "min，超过站内停留上限 " + std::to_string(kTransferMaxDwellMin)
                             + "min";
                return false;
            }
            if (need > wait) {
                wait = need;
            }
        }
        const double arrival = raw + wait;
        elapsed = arrival;

        // 逐单 penalty（停靠无合并窗口）。
        // 时间策略的键第二分量要能代表**用户可见的「总耗时」**（返回基地 − 发车，
        // 含早到等待与车等货，规格一 §6.1 / 规格三 §3）。若只累加 leg 耗时，
        // 「最短耗时策略」会为了少跑几分钟而换来一小时的空等 —— 验收②「每条策略在自身
        // 目标上不劣于另两条」于是不成立。距离/成本两项指标本身不含等待，故只对 Time
        // 加上等待（**等待的来源不区分**：窗口等待与就绪等待都进 key.weight）。
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

bool containsOrder(const std::vector<Order>& orders, const std::string& orderId) {
    for (std::size_t i = 0; i < orders.size(); ++i) {
        if (orders[i].id == orderId) {
            return true;
        }
    }
    return false;
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

// ---- 职责表的两侧（规格三 §1 / #46）：快车 / 慢车 / 簇 ----
//
// 三样东西全部**从数据推出来**，不硬编码车号或簇号。理由：职责封闭是**规则**，
// 换车队、换簇划分、换站 ID 之后规则应当照样成立；写死「V01 是快车、簇1 归 V02」
// 会让一条业务规则退化成一份数据快照，数据一动就悄悄失效。

// 快车 = 站间腿最合适的那台车。判据与接力层（applyRelayLayer）和站间腿落点
// （chooseLegVehicle）**同一个** betterForStationLeg：容量最大 ⇒ 成本因子更高 ⇒ ID 小。
// 三处口径一漂，「LEG 行说的执行车」与「真跑站间腿的车」就会不是同一台。
std::size_t relayVehicleIndex(const std::vector<Vehicle>& vehicles) {
    std::size_t best = 0;
    for (std::size_t v = 1; v < vehicles.size(); ++v) {
        if (betterForStationLeg(vehicles[v], vehicles[best])) {
            best = v;
        }
    }
    return best;
}

// 慢车 = 除快车外的全部车辆，按**车辆 ID 升序**（同 ID 保持 [vehicles] 行序）。
// 为什么按 ID 升序：配对的另一半是「簇按簇 ID 升序」，两边都按 ID 排，
// 「簇 -> 慢车」就是唯一确定的配对（排列数 = 1）——这正是整簇全排列枚举退役的根因。
std::vector<std::size_t> slowVehiclesSortedById(const std::vector<Vehicle>& vehicles,
                                                std::size_t relayIdx) {
    std::vector<std::size_t> slow;
    for (std::size_t v = 0; v < vehicles.size(); ++v) {
        if (v != relayIdx) {
            slow.push_back(v);
        }
    }
    for (std::size_t i = 1; i < slow.size(); ++i) {
        const std::size_t key = slow[i];
        std::size_t j = i;
        while (j > 0 && vehicles[key].id < vehicles[slow[j - 1]].id) {
            slow[j] = slow[j - 1];
            --j;
        }
        slow[j] = key;
    }
    return slow;
}

// 有配送点的非 0 子网，按簇 ID 升序。判据用「有配送点」而不是「有中转站」：
// 一个簇的存在意义是**有客户要服务**；只有站没有客户的子网不该占一台慢车。
std::vector<int> clusterIdsSorted(const LogisticsGraph& graph) {
    std::vector<int> ids;
    const std::vector<Node>& nodes = graph.nodes();
    for (std::size_t i = 0; i < nodes.size(); ++i) {
        if (nodes[i].subNetworkId <= 0 || nodes[i].type != NodeType::Delivery) {
            continue;
        }
        bool seen = false;
        for (std::size_t j = 0; j < ids.size(); ++j) {
            if (ids[j] == nodes[i].subNetworkId) {
                seen = true;
            }
        }
        if (!seen) {
            ids.push_back(nodes[i].subNetworkId);
        }
    }
    for (std::size_t i = 1; i < ids.size(); ++i) {
        const int key = ids[i];
        std::size_t j = i;
        while (j > 0 && ids[j - 1] > key) {
            ids[j] = ids[j - 1];
            --j;
        }
        ids[j] = key;
    }
    return ids;
}

// 簇 -> 本簇慢车（车辆**行下标**）。规则（唯一确定，不枚举）：
//   簇按簇 ID 升序、慢车按车辆 ID 升序，**逐位配对**；簇多于慢车时循环
//   （一台慢车守多簇，并在 allocationLog 点名），慢车多于簇时多出来的慢车空转。
// 返回 vehicles.size() 表示「该簇没有慢车」（调用方记日志，不伪造）。
std::size_t slowVehicleOfCluster(const LogisticsGraph& graph,
                                 const std::vector<Vehicle>& vehicles, int clusterId) {
    if (clusterId <= 0) {
        return vehicles.size();
    }
    const std::vector<std::size_t> slow =
        slowVehiclesSortedById(vehicles, relayVehicleIndex(vehicles));
    if (slow.empty()) {
        return vehicles.size();
    }
    const std::vector<int> clusters = clusterIdsSorted(graph);
    for (std::size_t i = 0; i < clusters.size(); ++i) {
        if (clusters[i] == clusterId) {
            return slow[i % slow.size()];
        }
    }
    return vehicles.size();
}

// 节点所属簇（subNetworkId）；节点不在图上返回 -1。
int clusterOfNode(const LogisticsGraph& graph, const std::string& nodeId) {
    const Node* n = graph.findNode(nodeId);
    return (n == nullptr) ? -1 : n->subNetworkId;
}

// 跨簇单的两个**估计** penalty —— 职责表里「跨簇紧急单直达」判据的输入（#46）。
//
// 为什么是估计：职责表要在规划**之前**回答「快车可不可以接这条客户↔客户腿」，
// 那时三段腿还没跑，只有图与各车因子。口径 = 「按该车因子缩放的该腿最短路耗时 +
// 前驱腿的计划完成时刻」，与 Leg::plannedArrivalMin 的规划期口径同源（可复算、确定性）。
// 估计只用于**比较**（直达 vs relay），不参与任何指标：指标永远读实际计划。
struct CrossPenaltyEstimate {
    int directMin = 0;    // 快车 A -> B 直达的估计 penalty
    int relayMin = 0;     // 三段接力的估计 penalty（按 L3 的计划完成时刻算）
    bool ok = false;      // 两条腿都可达才给出估计
};

CrossPenaltyEstimate estimateCrossPenalties(const LogisticsGraph& graph,
                                            const std::vector<Vehicle>& vehicles,
                                            const std::string& from, const std::string& to,
                                            const std::string& stationA,
                                            const std::string& stationB, int windowEndMin) {
    CrossPenaltyEstimate est;
    const std::size_t relayIdx = relayVehicleIndex(vehicles);
    const std::vector<std::size_t> slow = slowVehiclesSortedById(vehicles, relayIdx);
    if (slow.empty()) {
        return est;
    }
    // 直达：快车从 A 出发、跑到 B。
    const int fastTravel =
        legTravelMin(graph, vehicles, vehicles[relayIdx].id, from, to);
    if (fastTravel < 0) {
        return est;
    }
    const int fastArrive = vehicles[relayIdx].departTimeMin + fastTravel;
    est.directMin = (fastArrive > windowEndMin) ? (fastArrive - windowEndMin) : 0;

    // relay：L1 本簇慢车到 S_A、L2 快车到 S_B、L3 对簇慢车到 B。
    // 三段的执行车由职责表定（簇 -> 慢车），**不看** orderVehicleId：这条估计要在
    // 「谁接这条腿」还没定的时候就能算出来，否则判据与谓词互为前提。
    const int ca = clusterOfNode(graph, from);
    const int cb = clusterOfNode(graph, to);
    // relay 估计**不可得**（缺中转站 / 没有对簇慢车 / 不可达）时，把 relay penalty 记成
    // 「无界」而不是「没有估计」：职责表的比较语义是「直达是否比 relay 更差」，
    // 而 relay 根本不可行时它等价于 +inf ⇒ 紧急单的直达通道自然成立，普通单也不会
    // 因为一个不存在的替代方案被误判。留着 ok=false 会让「不可行」被当成「不许直达」，
    // 把本来只能直达的单判成无解。
    const int kUnboundedPenalty = 1000000;
    const std::size_t l1Idx = slowVehicleOfCluster(graph, vehicles, ca);
    const std::size_t l3Idx = slowVehicleOfCluster(graph, vehicles, cb);
    if (stationA.empty() || stationB.empty() || stationA == stationB
        || l1Idx >= vehicles.size() || l3Idx >= vehicles.size() || l1Idx == l3Idx) {
        est.relayMin = kUnboundedPenalty;
        est.ok = true;
        return est;
    }
    const int t1 = legTravelMin(graph, vehicles, vehicles[l1Idx].id, from, stationA);
    const int t2 = legTravelMin(graph, vehicles, vehicles[relayIdx].id, stationA, stationB);
    const int t3 = legTravelMin(graph, vehicles, vehicles[l3Idx].id, stationB, to);
    if (t1 < 0 || t2 < 0 || t3 < 0) {
        est.relayMin = kUnboundedPenalty;
        est.ok = true;
        return est;
    }
    const int relayArrive =
        vehicles[l1Idx].departTimeMin + t1 + t2 + t3;
    est.relayMin = (relayArrive > windowEndMin) ? (relayArrive - windowEndMin) : 0;
    est.ok = true;
    return est;
}

} // namespace

// ---- 职责表：**「谁可以接哪条腿」的唯一决定点**（规格三 §1 / #46）----
//
// 判据逐字（验收就是这几句，反向验证 C/D 也打在这里）：
//   · **慢车**只能接「两端都在本簇」且**非** station→station 的腿；
//   · **快车**只能接 station→station 腿，或「跨簇**紧急**单且该单 直达 penalty
//     **严格小于** relay penalty」的 customer→customer 直达腿。
//
// 为什么这条规则是「结构性」而不是「择优」：规格二按 Δkey 择优，结果快车一人吃掉
// 16/26 单（总距离 237.1km / 总成本 358.1 元，见规格三 §Problem 3）——高成本倍率的车
// 跑得最多，因为「更快的车先到、penalty 更低」。职责封闭把「谁接」从优化问题变成
// 一条划分规则：**每个子配送网由本簇慢车自治，快车只做簇间搬运**。
//
// 慢车这一侧为什么要求「两端都在本簇」：慢车越出本簇边界，就既不是本簇自治、
// 也不是簇间快运，职责表就退化成一句空话。站间腿也一并禁掉：站↔站是快车的活。
//
// 快车这一侧为什么留「紧急直达」这条缝：紧急单的语义是「越快越好」，三段接力要等
// 两次货（L1 卸站、L2 卸站），可能反而更晚。判据用**严格小于**（不是 ≤）：
// 打平时走 relay 才是「转运成为跨簇单的唯一路径」的本意，只有直达**真的更优**才直达。
// 公开的「哪台是快车」判定：唯一来源是 relayVehicleIndex（与职责表/接力层同判据），
// GUI 按角色取色与画环都读它，避免第三个地方各推一遍「谁是快车」。
std::vector<bool> fastVehicleFlags(const std::vector<Vehicle>& vehicles) {
    std::vector<bool> fast(vehicles.size(), false);
    if (vehicles.empty()) {
        return fast;
    }
    fast[relayVehicleIndex(vehicles)] = true;
    return fast;
}
std::string homeNodeOf(const LogisticsGraph& graph, const std::vector<Vehicle>& vehicles,
                       std::size_t vehicleIndex) {
    if (vehicles.empty() || vehicleIndex >= vehicles.size()) {
        return std::string();
    }
    // 快车回**起始仓库**：与职责表/接力层同一个 relayVehicleIndex（容量最大者）。
    if (vehicleIndex == relayVehicleIndex(vehicles)) {
        return vehicles[vehicleIndex].startNodeId;
    }
    // 慢车回**本簇中转站**：簇 -> 慢车是 slowVehicleOfCluster 的固定配对，这里反查。
    const std::vector<int> clusters = clusterIdsSorted(graph);
    for (std::size_t i = 0; i < clusters.size(); ++i) {
        if (slowVehicleOfCluster(graph, vehicles, clusters[i]) != vehicleIndex) {
            continue;
        }
        const std::string station = stationOfCluster(graph, clusters[i]);
        if (!station.empty()) {
            return station;
        }
        break;
    }
    // 没有簇 / 该簇没有中转站（夹具图常见）=> 退化为起始仓库，不伪造主场。
    return vehicles[vehicleIndex].startNodeId;
}

namespace {

// 一条最短路的三项（距离不缩放、耗时 x timeFactor、成本 x costFactor，与 replay 同口径）。
bool sumPath(const LogisticsGraph& graph, const Vehicle& vehicle, const PathResult& path,
             double& dist, double& time, double& cost) {
    dist = 0.0;
    time = 0.0;
    cost = 0.0;
    if (!path.found) {
        return false;
    }
    for (std::size_t k = 1; k < path.nodes.size(); ++k) {
        const Edge* e = graph.findEdge(path.nodes[k - 1], path.nodes[k]);
        if (e == nullptr) {
            return false;
        }
        dist += e->distanceKm;
        time += e->timeMin * vehicle.timeFactor;
        cost += e->costYuan * vehicle.costFactor;
    }
    return true;
}

// 计划「末段」= 最后一个停靠之后的节点（归位/回仓的那一段）。
// 末段起点下标：最后一个 nodeIsStop 为 true 的节点；没有停靠（空车）=> 起点就是首节点。
std::size_t lastStopNodeIndex(const RoutePlan& plan) {
    std::size_t last = 0;
    for (std::size_t i = 0; i < plan.nodes.size(); ++i) {
        if (i < plan.nodeIsStop.size() && plan.nodeIsStop[i]) {
            last = i;
        }
    }
    return last;
}

// 沿**实际节点序列**累加末段三项。相邻同名节点（同点连续两次停靠）没有边，按 0 计。
bool sumTail(const LogisticsGraph& graph, const Vehicle& vehicle, const RoutePlan& plan,
             std::size_t fromIdx, double& dist, double& time, double& cost) {
    dist = 0.0;
    time = 0.0;
    cost = 0.0;
    for (std::size_t i = fromIdx + 1; i < plan.nodes.size(); ++i) {
        if (plan.nodes[i - 1] == plan.nodes[i]) {
            continue;
        }
        const Edge* e = graph.findEdge(plan.nodes[i - 1], plan.nodes[i]);
        if (e == nullptr) {
            return false;
        }
        dist += e->distanceKm;
        time += e->timeMin * vehicle.timeFactor;
        cost += e->costYuan * vehicle.costFactor;
    }
    return true;
}

} // namespace

void applyIdleHoming(const LogisticsGraph& graph, const std::vector<Vehicle>& vehicles,
                     WeightType weight, std::vector<RoutePlan>& plans,
                     std::vector<std::string>* log) {
    for (std::size_t v = 0; v < vehicles.size() && v < plans.size(); ++v) {
        RoutePlan& plan = plans[v];
        const Vehicle& vehicle = vehicles[v];
        const std::string home = homeNodeOf(graph, vehicles, v);
        plan.homeNodeId = home;
        plan.homeArrivalMin = -1;
        plan.homed = false;
        plan.homingFromNodeId.clear();
        plan.homingDistanceKm = 0.0;
        plan.homingTimeMin = 0.0;
        plan.homingCostYuan = 0.0;
        plan.warehouseReturnKm = -1.0;

        if (plan.status != PlanStatus::Ok || plan.nodes.empty() || home.empty()) {
            continue;
        }
        const std::size_t lastStop = lastStopNodeIndex(plan);
        const std::string from = plan.nodes[lastStop];
        plan.homingFromNodeId = (from == home) ? std::string() : from;

        // ---- 新末段：from -> 主场（from == home 时为空，**不产生 0 距离边**）----
        std::vector<std::string> newTail;
        double newDist = 0.0;
        double newTime = 0.0;
        double newCost = 0.0;
        if (from != home) {
            const PathResult path = shortestPath(graph, from, home, weight);
            if (!sumPath(graph, vehicle, path, newDist, newTime, newCost)) {
                if (log != nullptr) {
                    log->push_back("[归位] 车辆 " + vehicle.id + " 无法从 " + from + " 归位到 "
                                   + home + "（不可达）=> 保留原末段，归位未执行");
                }
                continue;
            }
            for (std::size_t k = 1; k < path.nodes.size(); ++k) {
                newTail.push_back(path.nodes[k]);
            }
        }

        // 对照口径：若仍回起始仓库，末段多少公里（报告「归位省/多跑」用）。
        {
            const PathResult back = shortestPath(graph, from, vehicle.startNodeId, weight);
            double d = 0.0;
            double t = 0.0;
            double c = 0.0;
            if (sumPath(graph, vehicle, back, d, t, c)) {
                plan.warehouseReturnKm = d;
            }
        }

        plan.homed = (from != home);
        plan.homingDistanceKm = newDist;
        plan.homingTimeMin = newTime;
        plan.homingCostYuan = newCost;

        // 末段已与归位段**逐字相同** => 幂等：三项一概不动（复调度反复收口也逐字不变）。
        bool sameTail = (plan.nodes.size() - 1 - lastStop) == newTail.size();
        for (std::size_t k = 0; sameTail && k < newTail.size(); ++k) {
            sameTail = (plan.nodes[lastStop + 1 + k] == newTail[k]);
        }
        if (sameTail) {
            plan.homeArrivalMin = plan.returnArrivalMin;
            continue;
        }

        // ---- 旧末段（当前写法） ----
        double oldDist = 0.0;
        double oldTime = 0.0;
        double oldCost = 0.0;
        if (!sumTail(graph, vehicle, plan, lastStop, oldDist, oldTime, oldCost)) {
            if (log != nullptr) {
                log->push_back("[归位] 车辆 " + vehicle.id + " 的原末段边缺失 => 保留原计划");
            }
            continue;
        }

        // ---- 重写末段：节点序列 + 到达时刻 + 三项指标（penalty / 停靠 / 账本一概不动）----
        const int lastStopArrival =
            (lastStop < plan.nodeArrivalMin.size())
                ? plan.nodeArrivalMin[lastStop]
                : ((plan.nodeArrivalMin.empty())
                       ? vehicle.departTimeMin
                       : plan.nodeArrivalMin.front());
        plan.nodes.resize(lastStop + 1);
        plan.nodeArrivalMin.resize(lastStop + 1);
        plan.nodeIsStop.resize(lastStop + 1);
        double elapsed = static_cast<double>(lastStopArrival);
        std::string pos = from;
        for (std::size_t k = 0; k < newTail.size(); ++k) {
            const Edge* e = graph.findEdge(pos, newTail[k]);
            if (e == nullptr) {
                break;   // 上面的 sumPath 已核过路径边，这里只是防御
            }
            elapsed += e->timeMin * vehicle.timeFactor;
            pos = newTail[k];
            plan.nodes.push_back(newTail[k]);
            plan.nodeArrivalMin.push_back(static_cast<int>(std::lround(elapsed)));
            plan.nodeIsStop.push_back(false);
        }
        // 在车量的时间加权积分也要随末段改写（否则 avgLoad/utilization 会停在旧末段上，
        // oracle 一复算就对不上）：末段的在车量恒为**最后一个停靠的离车量**（归位不装不卸）。
        const double tailLoad = plan.stops.empty() ? 0.0 : plan.stops.back().remainingLoadKg;
        const double oldTotalTime = plan.totalTimeMin;
        const double oldIntegral = plan.metrics.avgLoadKg * oldTotalTime;
        plan.totalDistanceKm = plan.totalDistanceKm - oldDist + newDist;
        plan.totalCostYuan = plan.totalCostYuan - oldCost + newCost;
        plan.totalTimeMin = plan.totalTimeMin - oldTime + newTime;
        if (plan.totalTimeMin > 0.0) {
            const double integral = oldIntegral - tailLoad * oldTime + tailLoad * newTime;
            plan.metrics.avgLoadKg = integral / plan.totalTimeMin;
            plan.metrics.loadUtilization = (vehicle.capacityKg > 1e-9)
                                               ? (plan.metrics.avgLoadKg / vehicle.capacityKg)
                                               : 0.0;
        }
        plan.returnArrivalMin = static_cast<int>(std::lround(elapsed));
        plan.homeArrivalMin = plan.returnArrivalMin;
    }
}


bool legVehicleEligible(const LogisticsGraph& graph, const std::vector<Vehicle>& vehicles,
                        std::size_t vehicleIndex, const Leg& leg) {
    if (vehicleIndex >= vehicles.size()) {
        return false;
    }
    const std::size_t relayIdx = relayVehicleIndex(vehicles);

    // ---- 快车：站间腿 + 跨簇紧急单的直达腿 ----
    if (vehicleIndex == relayIdx) {
        if (leg.kind == LegKind::StationToStation) {
            return true;
        }
        if (leg.kind != LegKind::CustomerToCustomer) {
            return false;   // L1 / L3 是慢车的活
        }
        // 客户↔客户直达：只对**跨簇紧急单**开这条通道，且直达必须严格更优。
        const int ca = clusterOfNode(graph, leg.from);
        const int cb = clusterOfNode(graph, leg.to);
        const bool crossCluster = (ca > 0 && cb > 0 && ca != cb);
        if (!crossCluster || !leg.urgent) {
            return false;
        }
        if (leg.directPenaltyMin < 0 || leg.relayPenaltyMin < 0) {
            return false;   // 估计不成立 ⇒ 不敢声称「直达更优」，按 relay 处理
        }
        return leg.directPenaltyMin < leg.relayPenaltyMin;
    }

    // ---- 慢车：只接本簇、非站间的腿 ----
    if (leg.kind == LegKind::StationToStation) {
        return false;
    }
    const int ca = clusterOfNode(graph, leg.from);
    const int cb = clusterOfNode(graph, leg.to);
    if (ca <= 0 || ca != cb) {
        return false;   // 跨簇 / 端点不在图上 ⇒ 越界
    }
    // 本簇的慢车必须是**这一台**（簇 -> 慢车是固定指派，不是「任一台慢车都行」）。
    return slowVehicleOfCluster(graph, vehicles, ca) == vehicleIndex;
}

std::string chooseLegVehicle(const LogisticsGraph& graph, const std::vector<Vehicle>& vehicles,
                             const Leg& leg) {
    // 合资格车 = 过职责谓词的车。谓词是唯一决定点，这里只做「在收窄后的集合里取一个
    // 确定性落点」—— 职责封闭后这个集合通常只有一个元素（本簇慢车 / 快车）。
    std::vector<std::size_t> eligible;
    for (std::size_t i = 0; i < vehicles.size(); ++i) {
        if (legVehicleEligible(graph, vehicles, i, leg)) {
            eligible.push_back(i);
        }
    }
    if (eligible.empty()) {
        return std::string();
    }
    if (eligible.size() == 1) {
        return vehicles[eligible[0]].id;
    }
    // 多台合资格**只可能**在职责表被放宽之后出现（正常口径下每类腿的合资格集合就是
    // 单元素：慢车腿 = 本簇慢车、站间腿 = 快车）。所以这里取「车辆行序第一台」——
    // 越界会立刻出现在 LEG 行里，而不是被一条「看起来更合理」的启发式掩盖。
    // 站间腿例外：沿用 betterForStationLeg（与接力层同判据），保证「LEG 说的车」
    // 与「真跑站间腿的车」是同一台。
    if (leg.kind != LegKind::StationToStation) {
        return vehicles[eligible[0]].id;
    }
    std::size_t best = eligible[0];
    for (std::size_t k = 1; k < eligible.size(); ++k) {
        if (betterForStationLeg(vehicles[eligible[k]], vehicles[best])) {
            best = eligible[k];
        }
    }
    return vehicles[best].id;
}

namespace {

// 某个 ID 列表里有没有这个 ID（腿结构反查用；手写线性扫描，容器算法不交给 STL）。
bool idListHas(const std::vector<std::string>& v, const std::string& id) {
    for (std::size_t i = 0; i < v.size(); ++i) {
        if (v[i] == id) {
            return true;
        }
    }
    return false;
}

// 往路线里插一个动作（「取」端）：同节点已有普通停靠就合并，否则新建一次停靠。
// 为什么不能直接用 insertStationAction：那是**转运**专用（只会写 transitPickups），
// 而 L1 的取货是**真实取货**（pickups）—— 写错列会让「货在车」根本不成立。
void insertLegActionAt(std::vector<StopSpec>& specs, std::size_t at, const std::string& node,
                       const std::string& orderId, bool transit, bool unload) {
    if (at < specs.size() && specs[at].nodeId == node && specs[at].phase == StopPhase::Normal) {
        StopSpec& s = specs[at];
        if (transit) {
            (unload ? s.transitUnloads : s.transitPickups).push_back(orderId);
        } else {
            (unload ? s.deliveries : s.pickups).push_back(orderId);
        }
        return;
    }
    StopSpec s;
    s.nodeId = node;
    s.phase = StopPhase::Normal;
    if (transit) {
        (unload ? s.transitUnloads : s.transitPickups).push_back(orderId);
    } else {
        (unload ? s.deliveries : s.pickups).push_back(orderId);
    }
    specs.insert(specs.begin() + static_cast<std::ptrdiff_t>(at), s);
}

} // namespace

// 从**最终计划**里读出三段接力的实际执行车与时刻（#48）。
// 只有接力真的跑了才返回 true；否则退回 #45 的规划期估计（图 + 车因子的计划值）。
// 为什么不从 TransitLeg 反推：账本逐单只留最后一笔易手（见 buildTransitLedger），
// 而腿结构要的是三段各自的执行车与完成时刻 —— 停靠里的动作集才是这两件事的真源。
bool readRelayExecution(const FleetPlan& fleet, const std::vector<Vehicle>& vehicles,
                        const std::string& stationA, const std::string& stationB,
                        const std::string& orderId, Leg& l1, Leg& l2, Leg& l3) {
    std::string l1Vehicle;
    std::string l2Vehicle;
    std::string l3Vehicle;
    int t1 = -1;      // L1 在 S_A 的卸入时刻
    int t2 = -1;      // L2 在 S_B 的卸入时刻
    int t3 = -1;      // L3 在 B 的送达时刻
    for (std::size_t v = 0; v < fleet.vehicles.size() && v < vehicles.size(); ++v) {
        const RoutePlan& plan = fleet.vehicles[v];
        if (plan.status != PlanStatus::Ok) {
            continue;
        }
        for (std::size_t s = 0; s < plan.stops.size(); ++s) {
            const Stop& st = plan.stops[s];
            if (idListHas(st.pickups, orderId)) {
                l1Vehicle = vehicles[v].id;
            }
            if (st.nodeId == stationA && idListHas(st.transitUnloads, orderId)) {
                t1 = st.arrivalMin;
            }
            if (st.nodeId == stationB && idListHas(st.transitUnloads, orderId)) {
                t2 = st.arrivalMin;
                l2Vehicle = vehicles[v].id;
            }
            if (st.nodeId == stationB && idListHas(st.transitPickups, orderId)) {
                t3 = st.arrivalMin;
                l3Vehicle = vehicles[v].id;
            }
            for (std::size_t d = 0; d < st.deliveries.size(); ++d) {
                if (st.deliveries[d].orderId == orderId) {
                    t3 = st.arrivalMin;
                    l3Vehicle = vehicles[v].id;
                }
            }
        }
    }
    if (l1Vehicle.empty() || l2Vehicle.empty() || l3Vehicle.empty() || t1 < 0 || t2 < 0
        || t3 < 0) {
        return false;
    }
    const int l1Depart = departTimeOf(vehicles, l1Vehicle);
    l1.vehicleId = l1Vehicle;
    l1.readyTimeMin = (l1Depart >= 0) ? l1Depart : l1.readyTimeMin;
    l1.plannedArrivalMin = t1;
    l2.vehicleId = l2Vehicle;
    l2.readyTimeMin = t1;
    l2.plannedArrivalMin = t2;
    l3.vehicleId = l3Vehicle;
    l3.readyTimeMin = t2;
    l3.plannedArrivalMin = t3;
    return true;
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
    log.push_back("[腿] 执行车由职责表定（#46）：慢车腿 -> 本簇慢车，站间腿 -> 快车；"
                  "跨簇普通单的三段腿另由接力层真执行（#48），腿的时刻优先读实际执行值");
    log.push_back("[腿] 腿结构是**规划产物**：接力没跑起来时 L1/L2/L3 的时刻是规划期估计值"
                  "（日志会逐单点名），不声称已转运");

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
            leg.urgent = o.urgent;
            // 本地单的执行车也**过职责表**（不是直接抄分配表）：两条口径必须同源，
            // 否则「分配表说 A、LEG 说 B」会让守卫读到一个假一致。
            leg.vehicleId = chooseLegVehicle(graph, vehicles, leg);
            if (leg.vehicleId.empty()) {
                leg.vehicleId = orderVehicleId;
                log.push_back("[腿] 本地单 " + o.id + " 没有合职责的执行车（职责表给不出）"
                              " ⇒ 登记分配表的车 " + orderVehicleId + "，记为越界");
            } else if (!orderVehicleId.empty() && leg.vehicleId != orderVehicleId) {
                log.push_back("[腿] 本地单 " + o.id + " 的分配车 " + orderVehicleId
                              + " 与职责表的 " + leg.vehicleId + " 不同（对照入口不看职责，"
                                "如实记录差异）");
            }
            const int depart = departTimeOf(vehicles, leg.vehicleId);
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
            leg.urgent = o.urgent;
            const CrossPenaltyEstimate est = estimateCrossPenalties(
                graph, vehicles, o.pickupNodeId, o.nodeId, stationA, stationB, o.windowEndMin);
            leg.directPenaltyMin = est.ok ? est.directMin : -1;
            leg.relayPenaltyMin = est.ok ? est.relayMin : -1;
            leg.vehicleId = chooseLegVehicle(graph, vehicles, leg);
            if (leg.vehicleId.empty()) {
                leg.vehicleId = orderVehicleId;
                log.push_back("[腿] 跨簇单 " + o.id + " 缺中转站且职责表给不出执行车 ⇒ "
                              "登记分配表的车 " + orderVehicleId + "，记为越界（异常数据兜底）");
            }
            const int depart = departTimeOf(vehicles, leg.vehicleId);
            leg.readyTimeMin = (depart >= 0) ? depart : fallbackDepartMin;
            leg.plannedArrivalMin = plannedArrivalOf(graph, vehicles, leg, log);
            legs.push_back(leg);
            continue;
        }

        // ---- P2：实际执行形态优先于职责表估计 ----
        // 跨簇单被「不更差兜底 / 职责降级」按直达执行时（同一台车取货、并在自己的路线上
        // 送达，全程没有任何站内易手），LEG 必须如实登记 **1 条 customer->customer 直达腿**，
        // 而不是照职责表报 3 条腿 —— 否则「每张跨簇单恰好 3 条腿」的结构断言在降级场景
        // **假绿**（评审实测：ALLOC 是 O26=V01 直达，LEG 却报 V02/V01/V04 三条）。
        {
            std::string pickVehicle, delivVehicle;
            int pickTime = -1, delivTime = -1;
            bool anyTransit = false;
            for (std::size_t v = 0; v < fleet.vehicles.size() && v < vehicles.size(); ++v) {
                const RoutePlan& plan = fleet.vehicles[v];
                if (plan.status != PlanStatus::Ok) {
                    continue;
                }
                for (std::size_t s = 0; s < plan.stops.size(); ++s) {
                    const Stop& st = plan.stops[s];
                    if (idListHas(st.pickups, o.id)) {
                        pickVehicle = vehicles[v].id;
                        pickTime = st.arrivalMin;
                    }
                    for (std::size_t d = 0; d < st.deliveries.size(); ++d) {
                        if (st.deliveries[d].orderId == o.id) {
                            delivVehicle = vehicles[v].id;
                            delivTime = st.arrivalMin;
                        }
                    }
                    if (idListHas(st.transitUnloads, o.id) || idListHas(st.transitPickups, o.id)) {
                        anyTransit = true;
                    }
                }
            }
            if (!anyTransit && !pickVehicle.empty() && pickVehicle == delivVehicle) {
                Leg leg;
                leg.seq = 1;
                leg.from = o.pickupNodeId;
                leg.to = o.nodeId;
                leg.kind = LegKind::CustomerToCustomer;
                leg.orderId = o.id;
                leg.urgent = o.urgent;
                const CrossPenaltyEstimate est2 = estimateCrossPenalties(
                    graph, vehicles, o.pickupNodeId, o.nodeId, stationA, stationB, o.windowEndMin);
                leg.directPenaltyMin = est2.ok ? est2.directMin : -1;
                leg.relayPenaltyMin = est2.ok ? est2.relayMin : -1;
                leg.vehicleId = pickVehicle;
                leg.readyTimeMin = (pickTime >= 0) ? pickTime : fallbackDepartMin;
                leg.plannedArrivalMin = (delivTime >= 0) ? delivTime : leg.readyTimeMin;
                leg.temporaryAssignment = false;
                std::size_t vIdx = vehicles.size();
                for (std::size_t v = 0; v < vehicles.size(); ++v) {
                    if (vehicles[v].id == pickVehicle) {
                        vIdx = v;
                    }
                }
                const bool legal =
                    (vIdx < vehicles.size()) && legVehicleEligible(graph, vehicles, vIdx, leg);
                if (legal) {
                    log.push_back("[腿] 跨簇单 " + o.id + " 实际由 " + pickVehicle
                                  + " 直达执行且合职责表（跨簇紧急单直达例外）⇒ 按实际形态登记"
                                    " 1 条 customer->customer 腿");
                } else {
                    // 降级原因从分配/复调度日志里的 [职责降级] 行提取（两处口径同一句）。
                    std::string why = "三段腿未成立，按直达执行（职责降级）";
                    const std::string key = "[职责降级] 订单 " + o.id + " ";
                    for (std::size_t i = 0; i < fleet.allocationLog.size(); ++i) {
                        if (fleet.allocationLog[i].compare(0, key.size(), key) != 0) {
                            continue;
                        }
                        const std::size_t lp = fleet.allocationLog[i].find("三段腿未成立（");
                        if (lp != std::string::npos) {
                            const std::string head = "三段腿未成立（";
                            const std::size_t rp = fleet.allocationLog[i].find("）", lp);
                            if (rp != std::string::npos) {
                                why = fleet.allocationLog[i].substr(lp + head.size(),
                                                                    rp - (lp + head.size()));
                            }
                        }
                        break;
                    }
                    log.push_back("[腿] 跨簇单 " + o.id + " 实际由 " + pickVehicle
                                  + " 直达执行（职责降级）⇒ 按实际形态登记 1 条 customer->customer"
                                    " 腿，原因：" + why);
                    // 降级必须在 CLI seam 可见（规格三 Testing Decisions：唯一 seam = CLI 可见输出）。
                    log.push_back("LEGDEGRADE " + o.id + " " + why);
                }
                legs.push_back(leg);
                continue;
            }
        }

        // L1：客户 -> 本簇站；L2：站 -> 站；L3：对簇站 -> 客户。
        // 先按结构把三条腿立起来（seq/from/to/kind），时刻与执行车**先按 #45 的规划期
        // 估计**填，再由 readRelayExecution 用**实际执行时刻**覆盖（#48 接力跑通后二者
        // 会不同：真跑了接力，L2.ready 是 L1 真的卸到站的时刻，不是「发车 + 最短路」）。
        // 三段腿的职责输入（urgent / 两个 penalty 估计）只对 L1 有意义：
        // 职责表的快车侧比较的是**整单**的直达 vs relay，不是某条腿的；把它挂在 L1 上
        // 是最小改动（腿结构 / LEG 行格式不变），也让「哪张单是紧急跨簇单」在腿里可查。
        const CrossPenaltyEstimate est = estimateCrossPenalties(
            graph, vehicles, o.pickupNodeId, o.nodeId, stationA, stationB, o.windowEndMin);

        Leg l1;
        l1.seq = 1;
        l1.from = o.pickupNodeId;
        l1.to = stationA;
        l1.kind = LegKind::CustomerToStation;
        l1.orderId = o.id;
        l1.urgent = o.urgent;
        l1.directPenaltyMin = est.ok ? est.directMin : -1;
        l1.relayPenaltyMin = est.ok ? est.relayMin : -1;
        l1.vehicleId = chooseLegVehicle(graph, vehicles, l1);
        const int l1Depart = departTimeOf(vehicles, l1.vehicleId);
        l1.readyTimeMin = (l1Depart >= 0) ? l1Depart : fallbackDepartMin;
        l1.plannedArrivalMin = plannedArrivalOf(graph, vehicles, l1, log);

        Leg l2;
        l2.seq = 2;
        l2.from = stationA;
        l2.to = stationB;
        l2.kind = LegKind::StationToStation;
        l2.orderId = o.id;
        l2.readyTimeMin = l1.plannedArrivalMin;
        l2.vehicleId = chooseLegVehicle(graph, vehicles, l2);
        l2.plannedArrivalMin = plannedArrivalOf(graph, vehicles, l2, log);

        Leg l3;
        l3.seq = 3;
        l3.from = stationB;
        l3.to = o.nodeId;
        l3.kind = LegKind::StationToCustomer;
        l3.orderId = o.id;
        l3.vehicleId = chooseLegVehicle(graph, vehicles, l3);
        l3.readyTimeMin = l2.plannedArrivalMin;
        l3.plannedArrivalMin = plannedArrivalOf(graph, vehicles, l3, log);

        // 接力真的跑了 ⇒ 用**实际**卸入/送达时刻与**实际**执行车覆盖上面的估计。
        // 这是「腿结构反映真实执行」的唯一入口；失败（没跑接力）就保留估计值，不伪造。
        if (!readRelayExecution(fleet, vehicles, stationA, stationB, o.id, l1, l2, l3)) {
            log.push_back("[腿] 跨簇单 " + o.id
                          + " 本票未跑三段接力（不更差兜底或不可行）⇒ 腿的时刻为规划期估计值");
        } else {
            l1.temporaryAssignment = false;
            l2.temporaryAssignment = false;
            l3.temporaryAssignment = false;
        }

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
    // 逐单收集两端的**全部**易手事件：三段接力有两个易手点（S_A 卸入/取走、S_B 卸入/取走），
    // 旧的「一个订单一个累积槽」会被后一次卸入整段覆盖，覆盖后的 (in, out) 可能是同一台车
    // ⇒ 整笔被当成降载丢掉。改成先收集、再按**时刻顺序**配对。
    struct Event {
        std::string station;
        std::string vehicleId;
        int         timeMin = 0;
        double      kg = 0.0;
        bool        used = false;
    };
    std::map<std::string, std::vector<Event> > unloads;   // 订单 -> 卸入（进站）事件
    std::map<std::string, std::vector<Event> > pickups;   // 订单 -> 取走（离站）事件
    std::map<std::string, double> kgOfOrder;
    for (std::size_t v = 0; v < plans.size(); ++v) {
        const RoutePlan& plan = plans[v];
        if (plan.status != PlanStatus::Ok) {
            continue;
        }
        for (const Stop& s : plan.stops) {
            for (const std::string& id : s.transitUnloads) {
                Event e;
                e.station = s.nodeId;
                e.vehicleId = vehicles[v].id;
                e.timeMin = s.arrivalMin;
                const std::map<std::string, double>::const_iterator kg = plan.transitKg.find(id);
                e.kg = (kg != plan.transitKg.end()) ? kg->second : 0.0;
                unloads[id].push_back(e);
                kgOfOrder[id] = e.kg;
            }
            for (const std::string& id : s.transitPickups) {
                Event e;
                e.station = s.nodeId;
                e.vehicleId = vehicles[v].id;
                e.timeMin = s.arrivalMin;
                pickups[id].push_back(e);
            }
        }
    }
    // 逐单配对：每个取走事件吃掉「同站、不同车、时刻不晚于它」的最后一个未用卸入事件。
    // 记录粒度仍是**逐单一条**（§5.2「整单不可拆」），所以只保留最后一次成功的易手——
    // 它才是「谁把货交给最终送货人」。中间那次易手由 LEG 行（L1/L2 的 readyTime）
    // 与 STOP 行提供，账本不重复记，也不另写第二份账本。
    for (std::map<std::string, std::vector<Event> >::iterator it = pickups.begin();
         it != pickups.end(); ++it) {
        std::vector<Event>& outs = it->second;
        // 手写插入排序：时刻升序，同刻按车辆 ID 升序（确定性）。
        for (std::size_t i = 1; i < outs.size(); ++i) {
            const Event key = outs[i];
            std::size_t j = i;
            while (j > 0 && (outs[j - 1].timeMin > key.timeMin
                             || (outs[j - 1].timeMin == key.timeMin
                                 && outs[j - 1].vehicleId > key.vehicleId))) {
                outs[j] = outs[j - 1];
                --j;
            }
            outs[j] = key;
        }
        std::map<std::string, std::vector<Event> >::iterator ins = unloads.find(it->first);
        if (ins == unloads.end()) {
            continue;   // 只取不卸（不该发生）：不伪造另一端
        }
        std::vector<Event>& infos = ins->second;
        TransitLeg last;
        bool haveLast = false;
        for (std::size_t oi = 0; oi < outs.size(); ++oi) {
            long long bestIdx = -1;
            for (std::size_t ii = 0; ii < infos.size(); ++ii) {
                if (infos[ii].used || infos[ii].station != outs[oi].station
                    || infos[ii].vehicleId == outs[oi].vehicleId
                    || infos[ii].timeMin > outs[oi].timeMin) {
                    continue;
                }
                if (bestIdx < 0 || infos[ii].timeMin > infos[static_cast<std::size_t>(bestIdx)].timeMin
                    || (infos[ii].timeMin
                            == infos[static_cast<std::size_t>(bestIdx)].timeMin
                        && infos[ii].vehicleId
                               < infos[static_cast<std::size_t>(bestIdx)].vehicleId)) {
                    bestIdx = static_cast<long long>(ii);
                }
            }
            if (bestIdx < 0) {
                continue;   // 这一端没有配对的卸入：不伪造（只卸不取不是合法记录）
            }
            const Event& in = infos[static_cast<std::size_t>(bestIdx)];
            infos[static_cast<std::size_t>(bestIdx)].used = true;
            last.orderId = it->first;
            last.stationId = in.station;
            last.kg = (in.kg > 0.0) ? in.kg : kgOfOrder[it->first];
            last.inVehicleId = in.vehicleId;
            last.inTimeMin = in.timeMin;
            last.outVehicleId = outs[oi].vehicleId;
            last.outTimeMin = outs[oi].timeMin;
            haveLast = true;
        }
        if (haveLast) {
            legs.push_back(last);
        }
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

// ---- 规格三 #49：custody 守恒审计（读计划产物，不重算公式）----
//
// 「期末 0」过去是**空过**的：账本配对要求同站，封路把 T01->T03 的快车腿吃掉后，
// V02 在 T01 卸入、V04 在 T03 取走，两端不同站 => 账本 0 笔 => 看着「期末 0」其实是
// 货物跨站无承运（假绿）。本函数不看账本，直接读**计划里的每一次易手**逐单核：
//   · 卸入必须有同站、另一台车、时刻不早于它的取走（否则 unload-no-pickup）；
//   · 取走必须有同站、另一台车、时刻不晚于它的卸入（否则 pickup-no-unload）；
//   · 整单不可拆（C1）：每次易手都是该单全部需求 => Σ卸入 kg == Σ取走 kg；
//   · 期末：每单要么送达、要么被显式拒绝。
CustodyReport auditCustody(const std::vector<Vehicle>& vehicles,
                           const std::vector<RoutePlan>& plans,
                           const std::vector<Order>& orders,
                           const std::vector<std::string>& rejectedOrderIds) {
    CustodyReport report;
    struct Ev {
        std::string station;
        std::string vehicleId;
        int         timeMin = 0;
        double      kg = 0.0;
    };
    std::map<std::string, std::vector<Ev> > unloads;   // 订单 -> 卸入事件
    std::map<std::string, std::vector<Ev> > pickups;   // 订单 -> 取走事件
    std::map<std::string, bool> delivered;             // 订单 -> 计划里是否送达
    for (std::size_t v = 0; v < plans.size() && v < vehicles.size(); ++v) {
        if (plans[v].status != PlanStatus::Ok) {
            continue;
        }
        for (std::size_t s = 0; s < plans[v].stops.size(); ++s) {
            const Stop& st = plans[v].stops[s];
            for (std::size_t i = 0; i < st.transitUnloads.size(); ++i) {
                Ev e;
                e.station = st.nodeId;
                e.vehicleId = vehicles[v].id;
                e.timeMin = st.arrivalMin;
                const std::map<std::string, double>::const_iterator kg =
                    plans[v].transitKg.find(st.transitUnloads[i]);
                e.kg = (kg != plans[v].transitKg.end()) ? kg->second : 0.0;
                unloads[st.transitUnloads[i]].push_back(e);
            }
            for (std::size_t i = 0; i < st.transitPickups.size(); ++i) {
                Ev e;
                e.station = st.nodeId;
                e.vehicleId = vehicles[v].id;
                e.timeMin = st.arrivalMin;
                pickups[st.transitPickups[i]].push_back(e);
            }
            for (std::size_t d = 0; d < st.deliveries.size(); ++d) {
                delivered[st.deliveries[d].orderId] = true;
            }
        }
    }
    const std::vector<Ev> emptyEvents;
    for (std::size_t oi = 0; oi < orders.size(); ++oi) {
        const Order& o = orders[oi];
        const double demand = o.demandKg;
        const std::vector<Ev>& ins = (unloads.count(o.id) > 0) ? unloads[o.id] : emptyEvents;
        const std::vector<Ev>& outs = (pickups.count(o.id) > 0) ? pickups[o.id] : emptyEvents;
        const bool del = delivered.count(o.id) > 0;
        bool rejected = false;
        for (std::size_t i = 0; i < rejectedOrderIds.size(); ++i) {
            if (rejectedOrderIds[i] == o.id) {
                rejected = true;
            }
        }
        if (ins.empty() && outs.empty()) {
            if (!del && !rejected) {
                CustodyGap g;
                g.orderId = o.id;
                g.kind = "undelivered-not-rejected";
                g.detail = "订单 " + o.id + " 计划里没有送达，也不在显式拒绝名单里";
                report.gaps.push_back(g);
                report.ok = false;
            }
            continue;
        }
        // kg：整单不可拆 => 每次易手 = 该单全部需求；Σ卸入 必须 == Σ取走。
        double sumIn = 0.0;
        double sumOut = 0.0;
        for (std::size_t i = 0; i < ins.size(); ++i) {
            sumIn += ins[i].kg;
            if (std::fabs(ins[i].kg - demand) > 1e-6) {
                CustodyGap g;
                g.orderId = o.id;
                g.stationId = ins[i].station;
                g.kind = "kg-mismatch";
                g.detail = "订单 " + o.id + " 在 " + ins[i].station + " 卸入 "
                           + std::to_string(ins[i].kg) + "kg != 需求 " + std::to_string(demand)
                           + "kg";
                report.gaps.push_back(g);
                report.ok = false;
            }
        }
        for (std::size_t i = 0; i < outs.size(); ++i) {
            sumOut += demand;
        }
        if (std::fabs(sumIn - sumOut) > 1e-6) {
            CustodyGap g;
            g.orderId = o.id;
            g.kind = "kg-mismatch";
            g.detail = "订单 " + o.id + " Σ卸入 " + std::to_string(sumIn) + "kg != Σ取走 "
                       + std::to_string(sumOut) + "kg";
            report.gaps.push_back(g);
            report.ok = false;
        }
        // 卸入 -> 同站、不同车、时刻不早于它的取走（每一笔取走最多吃一笔卸入）。
        std::vector<char> outUsed(outs.size(), 0);
        for (std::size_t i = 0; i < ins.size(); ++i) {
            long long pick = -1;
            for (std::size_t j = 0; j < outs.size(); ++j) {
                if (outUsed[j]) {
                    continue;
                }
                if (outs[j].station != ins[i].station) {
                    continue;
                }
                if (outs[j].vehicleId == ins[i].vehicleId) {
                    continue;
                }
                if (outs[j].timeMin < ins[i].timeMin) {
                    continue;
                }
                if (pick < 0
                    || outs[j].timeMin < outs[static_cast<std::size_t>(pick)].timeMin) {
                    pick = static_cast<long long>(j);
                }
            }
            if (pick < 0) {
                CustodyGap g;
                g.orderId = o.id;
                g.stationId = ins[i].station;
                g.kind = "unload-no-pickup";
                g.detail = "订单 " + o.id + " 在 " + ins[i].station + " 由 " + ins[i].vehicleId
                           + " 卸入，但无同站、另一台车的取走（货物跨站无承运）";
                report.gaps.push_back(g);
                report.ok = false;
            } else {
                outUsed[static_cast<std::size_t>(pick)] = 1;
            }
        }
        // 取走 -> 同站、不同车、时刻不晚于它的卸入（不允许凭空的「只取不卸」）。
        for (std::size_t j = 0; j < outs.size(); ++j) {
            bool matched = false;
            for (std::size_t i = 0; i < ins.size(); ++i) {
                if (ins[i].station == outs[j].station && ins[i].vehicleId != outs[j].vehicleId
                    && ins[i].timeMin <= outs[j].timeMin) {
                    matched = true;
                }
            }
            if (!matched) {
                CustodyGap g;
                g.orderId = o.id;
                g.stationId = outs[j].station;
                g.kind = "pickup-no-unload";
                g.detail = "订单 " + o.id + " 在 " + outs[j].station + " 由 " + outs[j].vehicleId
                           + " 取走，但无同站、另一台车的卸入（货物凭空出现在站上）";
                report.gaps.push_back(g);
                report.ok = false;
            }
        }
        // 期末：有易手的单也必须最终送达或被显式拒绝。
        if (!del && !rejected) {
            CustodyGap g;
            g.orderId = o.id;
            g.kind = "undelivered-not-rejected";
            g.detail = "订单 " + o.id + " 有站内易手，但计划里没有送达，也不在显式拒绝名单里";
            report.gaps.push_back(g);
            report.ok = false;
        }
    }
    return report;
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

    // ---- 判据① 跨簇订单：**退役**（规格三 §6 / #48）----
    //
    // 跨簇普通单现在**只能**走三段腿（接力层 applyRelayLayer），不再由「三判据门控 +
    // 严格字典序更优才启用 + 并列保留禁转运版」这套后置改进决定。判据②③（容量腾挪 /
    // 紧急响应）仍走同一套候选机件，因此这里只把①的候选收集清空并**如实记一行**，
    // 而不是删掉整段代码：想恢复旧语义只需把收集改回来，但那样跨簇单就有两种并行语义。
    {
        int crossDelivered = 0;
        for (std::size_t v = 0; v < plans.size(); ++v) {
            for (const Stop& s : plans[v].stops) {
                for (const StopDelivery& d : s.deliveries) {
                    std::map<std::string, Order>::const_iterator it = orderById.find(d.orderId);
                    if (it != orderById.end() && isCrossCluster(graph, it->second)) {
                        ++crossDelivered;
                    }
                }
            }
        }
        if (crossDelivered > 0) {
            result.log.push_back("转运判据①（跨簇订单）：已退役 —— 跨簇普通单改由三段接力"
                                 "（规格三 §3 / #48）承载，这里不再生成候选");
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

// ===========================================================================
// 三段接力（规格三 §3 / #48）：跨簇普通单**只能**走三段腿
// ===========================================================================
//
// 为什么是「后置管道」而不是改写构造器：接力是**跨车的先后依赖**（L2 等 L1 到站、
// L3 等 L2 到站），而构造器是**单车**的。把顺序写成三段串行的构造调用，依赖关系就是
// 代码结构本身：① 规划源簇慢车的 L1（卸入时刻已知）→ ② 快车以它为 readyTime 规划 L2
// → ③ 对簇慢车以 L2 卸入为 readyTime 规划 L3。这条顺序同时就是「只前瞻一层」的实现：
// 每条腿只拿**直接前驱已规划的实际完成时刻**，不做多级连锁预测、也不做概率预测。
namespace {

// 把一张订单从一组停靠里**彻底**摘走（含取货动作）。
// 三段腿会把同一张单的取/送拆到不同车上，旧的「取货 + 送达」必须整段消失，
// 否则货会被取两次、送两次（不变量「每单恰一台车送达」直接失守）。
void removeOrderFromRoute(std::vector<StopSpec>& specs, const std::string& orderId) {
    for (std::size_t i = 0; i < specs.size(); ++i) {
        StopSpec& s = specs[i];
        eraseId(s.pickups, orderId);
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

// 往停靠上挂一个动作：from 端是「取」（真实取货 / 站内取走），to 端是「卸」
// （站内卸入 / 送达）。四个组合正好覆盖 L1/L2/L3 与「直达对照」。
void appendLegAction(StopSpec& s, const std::string& orderId, bool transit, bool unload) {
    if (transit) {
        if (unload) {
            s.transitUnloads.push_back(orderId);
        } else {
            s.transitPickups.push_back(orderId);
        }
        return;
    }
    if (unload) {
        s.deliveries.push_back(orderId);
    } else {
        s.pickups.push_back(orderId);
    }
}

// 在给定路线上插入一条腿的**两个动作**：fromNode 处「取」、toNode 处「卸」。
// 两处位置全量枚举、每个候选整条重放，按该车自己的三键取最优 —— 与 buildPickupSpecs
// 完全同一种做法（不裁剪候选，保证不漏唯一可行位置）。
//
// from 必须先于 to；两处都能合并到同节点的已有普通停靠（顺路取、顺路送）。
// 某条腿在计划里的落点时刻（transit 视角：unload=true 读 transitUnloads，否则读送达）。
// 前向声明：本函数在 buildRelayOnwardSpecs 之后定义，但插入器要用它做「尽早交付」的比较。
int transitStopArrival(const RoutePlan& plan, const std::string& nodeId, bool unload,
                       const std::string& orderId);

// 在**既有路线**上给一条腿找落点：把「取」动作插在某处、「卸/送」动作插在它之后某处。
// 两处都枚举**所有位置**（不是只认该节点第一次出现的地方）。
//
// 为什么必须枚举所有位置（#46 实测的坑）：既有路线里某个节点常常只有**一次**停靠，
// 而那一刻正好是载重峰值。例如默认数据里 V02 在 D08 一次取满 160kg（= 容量），
// 只认「既有 D08 停靠」的话，任何新单的取货都只能塞在峰值那一刻 ⇒ 一律超载、
// 接力被判成不可行。而单车构造器本身完全能把它排成「先按老单取满、送掉几单、
// 再回 D08 取这单」（实测峰值 148kg 可行）。插入器必须能表达同样的解，
// 否则接力层的能力就被人为压低，跨簇单只能退回直达。
//
// preferEarliestAtTo：接力链上的**前两段**要「把货尽早交到下一段手上」。
// 车等货的上限是前瞻窗口 W = 60min，一条腿排得再好，只要交付晚了整条链就不成立；
// 所以这两段先比「在本腿终点站的交付时刻」，再比路线键。最后一段没有下游，只比路线键。
bool buildRelayOnwardSpecs(const Ctx& ctx, const std::vector<StopSpec>& base,
                          const std::string& orderId, const std::string& fromNode,
                          const std::string& toNode, bool fromIsTransit, bool toIsTransit,
                          std::vector<StopSpec>& best, bool& found,
                          bool preferEarliestAtTo = false) {
    bool have = false;
    int bestArrival = 0;
    int bestPenalty = 0;
    Key bestKey;
    for (std::size_t fromAt = 0; fromAt <= base.size(); ++fromAt) {
        std::vector<StopSpec> withFrom = base;
        insertLegActionAt(withFrom, fromAt, fromNode, orderId, fromIsTransit, false);
        normalize(withFrom);
        // 合并/插站都可能让下标平移：重新定位「取」动作所在的停靠。
        long long fIdx = -1;
        for (std::size_t i = 0; i < withFrom.size(); ++i) {
            const bool hasIt = fromIsTransit ? idListHas(withFrom[i].transitPickups, orderId)
                                             : idListHas(withFrom[i].pickups, orderId);
            if (hasIt) {
                fIdx = static_cast<long long>(i);
                break;
            }
        }
        if (fIdx < 0) {
            continue;
        }
        for (std::size_t at = static_cast<std::size_t>(fIdx) + 1; at <= withFrom.size(); ++at) {
            std::vector<StopSpec> cand = withFrom;
            insertLegActionAt(cand, at, toNode, orderId, toIsTransit, true);
            normalize(cand);
            Eval ev;
            if (!replay(ctx, cand, false, ev)) {
                continue;
            }
            int arrivalTo = -1;
            if (preferEarliestAtTo) {
                RoutePlan probe;
                if (!finishSpecs(ctx, cand, probe)) {
                    continue;
                }
                arrivalTo = transitStopArrival(probe, toNode, toIsTransit, orderId);
                if (arrivalTo < 0) {
                    continue;
                }
            }
            // 取优次序（从句到细）：penalty（客户口径，不可让步）-> 交付时刻（接力可行性）
            // -> 路线键（本策略自己的权重与 tieKey）。
            bool better = false;
            if (!have) {
                better = true;
            } else if (preferEarliestAtTo && ev.penalty != bestPenalty) {
                better = ev.penalty < bestPenalty;
            } else if (preferEarliestAtTo && arrivalTo != bestArrival) {
                better = arrivalTo < bestArrival;
            } else {
                better = compareRoutes(cand, ev.key, best, bestKey) < 0;
            }
            if (better) {
                have = true;
                best = cand;
                bestKey = ev.key;
                bestArrival = arrivalTo;
                bestPenalty = ev.penalty;
            }
        }
    }
    found = have;
    return have;
}

// 某车里「在 nodeId 上卸入（unload=true）/取走（false）该单」的停靠到达时刻；
// 找不到返回 -1。读的是**重放后的真实到达时刻**，不是估计值。
int transitStopArrival(const RoutePlan& plan, const std::string& nodeId, bool unload,
                       const std::string& orderId) {
    for (std::size_t i = 0; i < plan.stops.size(); ++i) {
        const Stop& s = plan.stops[i];
        if (s.nodeId != nodeId) {
            continue;
        }
        const std::vector<std::string>& v = unload ? s.transitUnloads : s.transitPickups;
        if (idListHas(v, orderId)) {
            return s.arrivalMin;
        }
    }
    return -1;
}

// 该车既有计划里服务过这个簇吗（任一停靠点落在该簇即算）。
// 「本簇慢车」就按这条判据认：谁已经在跑这个簇的单，谁就是这个簇的慢车 ——
// 不硬编码「簇 N -> 第 k 台车」的对应（那是 #46 的职责封闭要立的事）。
bool planServesCluster(const RoutePlan& plan, const LogisticsGraph& graph, int clusterId) {
    if (clusterId <= 0) {
        return false;
    }
    for (std::size_t i = 0; i < plan.stops.size(); ++i) {
        const Node* n = graph.findNode(plan.stops[i].nodeId);
        if (n != nullptr && n->subNetworkId == clusterId) {
            return true;
        }
    }
    return false;
}

std::vector<Order> withoutOrderCopy(const std::vector<Order>& in, const std::string& orderId) {
    std::vector<Order> out;
    for (std::size_t i = 0; i < in.size(); ++i) {
        if (in[i].id != orderId) {
            out.push_back(in[i]);
        }
    }
    return out;
}

// 一次接力的结果：三台车各自的计划 + 两个卸入时刻（供日志与腿结构核对）。
struct RelayBuild {
    bool ok = false;
    std::vector<RoutePlan> plans;      // 接力版（已摘单、已插三腿）
    std::size_t srcIdx = 0;            // L1 执行车
    std::size_t destIdx = 0;           // L3 执行车
    int t1 = -1;                       // L1 在 S_A 的卸入时刻（= L2 的 readyTime）
    int t2 = -1;                       // L2 在 S_B 的卸入时刻（= L3 的 readyTime）
    std::string stationA;
    std::string stationB;
    std::string reason;                // 不可行的原因（写日志）
};

// 接力层产物。
// 一条已成立的三段接力链的引用（RelayOutcome::chains 的元素）。
struct RelayChainRef {
    std::string orderId;
    std::string stationA;
    std::string stationB;
    std::size_t srcIdx = 0;    // L1 执行车（本簇慢车）
    std::size_t destIdx = 0;   // L3 执行车（对簇慢车）
};

struct RelayOutcome {
    std::vector<RoutePlan> plans;
    std::vector<std::vector<Order> > vehicleOrders;
    int relayedCount = 0;
    int fallbackCount = 0;
    int skippedCount = 0;
    std::vector<std::string> log;
    // 计划**真的被改过**的车（行下标）。事件路径只写回这些车，未参与接力的车路线逐字不变
    // （不变量 5）。planFleet 路径不看它（整队计划一次性收口），事件路径靠它圈定写回范围。
    std::vector<std::size_t> touched;
    // 本轮走成三段腿的单（收口复核与 hint 迭代用）：srcIdx/destIdx + 两个站。
    std::vector<RelayChainRef> chains;
};

// 对一张跨簇单构造三段接力（①慢车 L1 -> ②快车 L2 -> ③对簇慢车 L3）。
// detached = 已摘掉该单的逐车计划；ctxs 的 readyAt 按**实际卸入时刻**填表。
RelayBuild buildRelayChain(const LogisticsGraph& graph, const std::vector<Vehicle>& vehicles,
                           const Order& o, const std::vector<RoutePlan>& detached,
                           WeightType weight, std::size_t relayIdx,
                           const std::vector<std::size_t>& slow, std::vector<Ctx>& ctxs,
                           const std::map<std::string, std::map<std::string, int> >* readyHints) {
    RelayBuild out;
    out.plans = detached;
    const Node* pa = graph.findNode(o.pickupNodeId);
    const Node* pb = graph.findNode(o.nodeId);
    if (pa == nullptr || pb == nullptr) {
        out.reason = "端点不在图上";
        return out;
    }
    out.stationA = stationOfCluster(graph, pa->subNetworkId);
    out.stationB = stationOfCluster(graph, pb->subNetworkId);
    if (out.stationA.empty() || out.stationB.empty() || out.stationA == out.stationB) {
        out.reason = "缺对侧中转站";
        return out;
    }
    // 候选车 = **职责表**给的本簇慢车（簇 ID 升序 × 慢车 ID 升序逐位配对），唯一确定。
    //
    // 为什么不再按「谁已经在跑该簇」认（#48 报告偏差 3）：那个口径在「该簇一张本地单
    // 都没有」时会退化成全部慢车，把 L1/L3 落到别的簇的车上，接力随即被「不更差」兜底
    // 退回直达 —— 固定指派之后这个落点是**结构给定**的，不需要猜。
    auto pickCandidates = [&](int clusterId) {
        const std::size_t duty = slowVehicleOfCluster(graph, vehicles, clusterId);
        std::vector<std::size_t> out1;
        if (duty < vehicles.size()) {
            out1.push_back(duty);
            return out1;
        }
        // 职责表给不出（簇不存在 / 车队没有慢车）：退化为全部慢车，并如实记日志。
        out1 = slow;
        return out1;
    };

    // hint（规格五 #57）：上一轮收口复核读出的**实际**站时刻。接力是逐单插入的，
    // 后续插入会把站停靠挪后，按当前值等的链在最终计划里会倒挂；hint 把「等多久」
    // 直接钉到上一轮的终值，让等待先到位（时间只会后移，收敛由外层循环保证）。
    auto hintOf = [&](const std::string& station) -> int {
        if (readyHints == nullptr) {
            return -1;
        }
        const std::map<std::string, std::map<std::string, int> >::const_iterator it =
            readyHints->find(o.id);
        if (it == readyHints->end()) {
            return -1;
        }
        const std::map<std::string, int>::const_iterator s = it->second.find(station);
        return (s == it->second.end()) ? -1 : s->second;
    };

    // ---- ① L1：客户 A -> 本簇站 S_A（真实取货 + 站内卸入）----
    const std::vector<std::size_t> srcCands = pickCandidates(pa->subNetworkId);
    bool haveL1 = false;
    int l1Unload = 0;          // 该候选在 S_A 的卸入时刻（越小越好）
    Key l1Key;
    std::vector<StopSpec> l1Specs;
    RoutePlan l1Plan;
    std::size_t srcIdx = srcCands[0];
    for (std::size_t i = 0; i < srcCands.size(); ++i) {
        const std::size_t v = srcCands[i];
        std::vector<StopSpec> cand;
        bool found = false;
        if (!buildRelayOnwardSpecs(ctxs[v], specsFromPlan(detached[v]), o.id, o.pickupNodeId,
                                   out.stationA, false, true, cand, found, true)
            || !found) {
            continue;
        }
        RoutePlan plan;
        if (!finishSpecs(ctxs[v], cand, plan)) {
            continue;
        }
        Key k;
        if (!evalKey(ctxs[v], cand, k, nullptr, nullptr, nullptr)) {
            continue;
        }
        // L1 的**目的**是把货尽早交到站上：L2 只能等 L1 卸完，且「车等货」的上限是前瞻
        // 窗口 W = 60min。所以先比「站内卸入时刻」、再比路线键。
        //
        // 为什么不能只比路线键（#46 实测）：本簇慢车的第一个取货点常常就是载重峰值
        // （默认数据 V02 在 D08 一次取满 160kg = 容量），新单只能排到「送掉几单之后再回
        // 取货」的第二次停靠；那种排法路线键不差，但卸入时刻可能晚到让 L2 等超 60min，
        // 于是整条接力被判不可行。按卸入时刻取优才是这条腿的本意。
        const int unload = transitStopArrival(plan, out.stationA, true, o.id);
        if (unload < 0) {
            continue;
        }
        if (!haveL1 || unload < l1Unload
            || (unload == l1Unload && compareRoutes(cand, k, l1Specs, l1Key) < 0)) {
            haveL1 = true;
            l1Unload = unload;
            l1Key = k;
            l1Specs = cand;
            l1Plan = plan;
            srcIdx = v;
        }
    }
    if (!haveL1) {
        out.reason = "L1 慢车接不下（取货 + 卸站不可行）";
        return out;
    }
    out.plans[srcIdx] = l1Plan;
    const int t1 = transitStopArrival(l1Plan, out.stationA, true, o.id);
    if (t1 < 0) {
        out.reason = "L1 卸入时刻读不出来";
        return out;
    }

    // ---- ② L2：S_A -> S_B（快车；readyTime = L1 的卸入时刻，W 在这里生效）----
    // readyAt 落到**快车**的 Ctx：它的每个候选位置都会在 replay 里被判「要等多久」，
    // 超窗的候选直接不可行 ⇒ 前瞻窗口参与候选取舍，不靠事后过滤。
    {
        int l2Ready = t1;
        const int hint = hintOf(out.stationA);
        if (hint > l2Ready) {
            l2Ready = hint;
        }
        ctxs[relayIdx].readyAt[o.id][out.stationA] = l2Ready;
    }

    std::vector<StopSpec> l2Specs;
    bool haveL2 = false;
    if (!buildRelayOnwardSpecs(ctxs[relayIdx], specsFromPlan(detached[relayIdx]), o.id,
                               out.stationA, out.stationB, true, true, l2Specs, haveL2, true)
        || !haveL2) {
        out.reason = "L2 站间腿不可行（就绪差超出前瞻窗口 "
                     + std::to_string(kLookaheadWindowMin) + "min 或站内停留超过 "
                     + std::to_string(kTransferMaxDwellMin) + "min）";
        return out;
    }
    RoutePlan l2Plan;
    if (!finishSpecs(ctxs[relayIdx], l2Specs, l2Plan)) {
        out.reason = "L2 重放不可行";
        return out;
    }
    out.plans[relayIdx] = l2Plan;
    const int t2 = transitStopArrival(l2Plan, out.stationB, true, o.id);
    if (t2 < 0) {
        out.reason = "L2 卸入时刻读不出来";
        return out;
    }

    // ---- ③ L3：S_B -> 客户 B（对簇慢车；readyTime = L2 的卸入时刻）----
    //
    // **L3 车不得与 L1 车相同**：同一台车同时做「取货→卸站」和「站内取走→送达」时，
    // 两次插入各自重排它的路线，最终计划里可能出现「站内取走早于站内卸入」的
    // **运单倒挂**（replay 只校验载重，不校验货物的时空连续性）。排除它之后三段腿
    // 必然分给三台车 ⇒ 「期末无在站」与 custody 链由构造保证，而不是事后抽查。
    // 车不够（例如两车车队）时这里自然收窄为空 ⇒ 该单退回直达，这是诚实的退化。
    std::vector<std::size_t> dstCands = pickCandidates(pb->subNetworkId);
    {
        std::vector<std::size_t> filtered;
        for (std::size_t i = 0; i < dstCands.size(); ++i) {
            if (dstCands[i] != srcIdx) {
                filtered.push_back(dstCands[i]);
            }
        }
        dstCands = filtered;
    }
    if (dstCands.empty()) {
        out.reason = "没有第二台可用的对簇慢车（L1/L3 必须由不同车执行）";
        return out;
    }
    const std::vector<std::size_t>& dstFinal = dstCands;
    bool haveL3 = false;
    Key l3Key;
    std::vector<StopSpec> l3Specs;
    RoutePlan l3Plan;
    std::size_t destIdx = dstFinal[0];
    for (std::size_t i = 0; i < dstFinal.size(); ++i) {
        const std::size_t v = dstFinal[i];
        {
            int l3Ready = t2;
            const int hint = hintOf(out.stationB);
            if (hint > l3Ready) {
                l3Ready = hint;
            }
            ctxs[v].readyAt[o.id][out.stationB] = l3Ready;
        }
        std::vector<StopSpec> cand;
        bool found = false;
        if (!buildRelayOnwardSpecs(ctxs[v], specsFromPlan(out.plans[v]), o.id, out.stationB,
                                   o.nodeId, true, false, cand, found)
            || !found) {
            continue;
        }
        RoutePlan plan;
        if (!finishSpecs(ctxs[v], cand, plan)) {
            continue;
        }
        Key k;
        if (!evalKey(ctxs[v], cand, k, nullptr, nullptr, nullptr)) {
            continue;
        }
        if (!haveL3 || compareRoutes(cand, k, l3Specs, l3Key) < 0) {
            haveL3 = true;
            l3Key = k;
            l3Specs = cand;
            l3Plan = plan;
            destIdx = v;
        }
    }
    if (!haveL3) {
        out.reason = "L3 对簇慢车接不下（站内取走 + 送达不可行）";
        return out;
    }
    out.plans[destIdx] = l3Plan;
    out.ok = true;
    out.srcIdx = srcIdx;
    out.destIdx = destIdx;
    out.t1 = t1;
    out.t2 = t2;
    return out;
}

// 三段接力的 custody 链自洽性（在**最终计划**上核）：
//   A 取货 ≤ S_A 卸入 ≤ S_A 取走 ≤ S_B 卸入 ≤ S_B 取走 ≤ B 送达，
// 相邻两段不同车，两个「车等货」的等待都落在 W 内。
//
// 为什么必须核：同一台车可能同时承载**另一张**跨簇单的腿，后续插入会重排它的停靠，
// 从而把先前那条腿的卸入时刻挪后 —— 那样先前单的 readyTime 就是过期值，计划表面可行
// 但运单时空不连续（replay 只校验载重）。核不过就诚实地退回直达，不冒充成功。
bool relayChainConsistent(const std::vector<Vehicle>& vehicles,
                          const std::vector<RoutePlan>& plans, const std::string& orderId,
                          const std::string& stationA, const std::string& stationB) {
    struct Hit {
        bool found = false;
        std::string vehicleId;
        int arrival = -1;
    };
    Hit pickA, unloadA, pickA2, unloadB, pickB2, deliv;
    for (std::size_t v = 0; v < plans.size() && v < vehicles.size(); ++v) {
        for (std::size_t s = 0; s < plans[v].stops.size(); ++s) {
            const Stop& st = plans[v].stops[s];
            if (idListHas(st.pickups, orderId)) {
                pickA.found = true;
                pickA.vehicleId = vehicles[v].id;
                pickA.arrival = st.arrivalMin;
            }
            for (std::size_t d = 0; d < st.deliveries.size(); ++d) {
                if (st.deliveries[d].orderId == orderId) {
                    deliv.found = true;
                    deliv.vehicleId = vehicles[v].id;
                    deliv.arrival = st.arrivalMin;
                }
            }
            if (st.nodeId == stationA && idListHas(st.transitUnloads, orderId)) {
                unloadA.found = true;
                unloadA.vehicleId = vehicles[v].id;
                unloadA.arrival = st.arrivalMin;
            }
            if (st.nodeId == stationA && idListHas(st.transitPickups, orderId)) {
                pickA2.found = true;
                pickA2.vehicleId = vehicles[v].id;
                pickA2.arrival = st.arrivalMin;
            }
            if (st.nodeId == stationB && idListHas(st.transitUnloads, orderId)) {
                unloadB.found = true;
                unloadB.vehicleId = vehicles[v].id;
                unloadB.arrival = st.arrivalMin;
            }
            if (st.nodeId == stationB && idListHas(st.transitPickups, orderId)) {
                pickB2.found = true;
                pickB2.vehicleId = vehicles[v].id;
                pickB2.arrival = st.arrivalMin;
            }
        }
    }
    if (!pickA.found || !unloadA.found || !pickA2.found || !unloadB.found || !pickB2.found
        || !deliv.found) {
        return false;
    }

    if (pickA.arrival > unloadA.arrival || pickA2.arrival > unloadB.arrival
        || pickB2.arrival > deliv.arrival) {
        return false;   // 段内顺序
    }
    if (unloadA.vehicleId == pickA2.vehicleId || unloadB.vehicleId == pickB2.vehicleId
        || unloadA.vehicleId == pickB2.vehicleId) {
        return false;   // 相邻两段必须不同车（inVehicle ≠ outVehicle 的根）
    }
    const int wait1 = pickA2.arrival - unloadA.arrival;
    const int wait2 = pickB2.arrival - unloadB.arrival;
    if (wait1 < 0 || wait2 < 0 || wait1 > kLookaheadWindowMin || wait2 > kLookaheadWindowMin) {
        return false;
    }
    return true;
}

} // namespace

// 接力层入口：把每张跨簇普通单换成三段腿。
//   · 默认数据（25 单全簇内）⇒ 一张跨簇单都没有 ⇒ **一行候选都不生成**，
//     计划与黄金值逐字不变（这是本票最强的「没破东西」证据）。
//   · 跨簇紧急单**不进来**：紧急语义保留快车直达通道（直达/接力的 penalty 判据归 #46）。
//   · 「不更差」兜底：三段腿方案若劣于「把该单直接给快车」的对照方案，就按对照方案
//     执行并记日志（防止形式僵化出荒谬解）。
// starts（规格三 §2④ / US21）：可选「已执行前缀」。非空时按每车当前位置/时刻/在车货/暂存
// 建 Ctx（事件路径），为空时退回「仓库 + 发车时刻」（planFleet 的静态规划，行为逐字不变）。
// 事件刷出的跨簇单必须走同一条接力机件，才谈得上「动态场景与静态演示语义一致」。
// 单轮接力层：把 forcedDirect 里的单**先**按「职责降级直达」落位，再逐单试三段腿。
// 为什么要分先后：直达插入会挪动快车的站停靠，一旦发生在站间腿之后，先前那张单登记的
// readyTime 就过期了（见 applyRelayLayer 的说明）。staleOut 返回**复核不过**的单号。
static void relayLayerOnce(const LogisticsGraph& graph, const std::vector<Vehicle>& vehicles,
                           const std::vector<Order>& orders, WeightType weight,
                           const std::vector<std::vector<Order> >& vehicleOrders,
                           const std::vector<RoutePlan>& basePlans,
                           const std::vector<Order>& relayOrders,
                           const std::vector<std::string>& forcedDirect,
                           const std::map<std::string, std::map<std::string, int> >* readyHints,
                           std::vector<std::string>* staleOut, RelayOutcome& out,
                           const std::vector<VehicleStartState>* starts = nullptr) {
    if (staleOut != nullptr) {
        staleOut->clear();
    }
    out.plans = basePlans;
    out.vehicleOrders = vehicleOrders;
    // 起点表不完整 ⇒ 退回静态口径（防御：宁可当空车也不静默错位到别的车上）。
    if (starts != nullptr && starts->size() < vehicles.size()) {
        starts = nullptr;
    }
    if (vehicles.size() < 2 || basePlans.size() != vehicles.size()
        || vehicleOrders.size() != vehicles.size()) {
        return;
    }
    // 接力车（站间腿执行车）= 容量最大者（并列取成本因子更高、再并列取 ID 小）。
    // 与 chooseLegVehicle 对 StationToStation 的临时规则**同判据**：两处口径一漂，
    // LEG 行说的执行车和真跑的车就会不是同一台。
    std::size_t relayIdx = 0;
    for (std::size_t v = 1; v < vehicles.size(); ++v) {
        if (betterForStationLeg(vehicles[v], vehicles[relayIdx])) {
            relayIdx = v;
        }
    }
    // 慢车列表按**车辆 ID 升序**（P5 / 口径统一）：直接调 planFleet 用的同一个 helper，
    // 避免这里自己按行序重算、两处口径悄悄漂开（职责表配对只认 ID 序）。
    const std::vector<std::size_t> slow = slowVehiclesSortedById(vehicles, relayIdx);
    if (slow.empty()) {
        return;
    }

    // 待接力的单由**调用方**给定（planFleet 的分流结果）：普通跨簇单一律进来，紧急
    // 跨簇单只有「直达不严格更优」时才进来 —— 分流判据与职责表同一句（legVehicleEligible
    // 的快车侧），不在本层重新解释一遍，否则两处判据会漂。
    std::vector<Order> crossOrders = relayOrders;
    // 订单 ID 升序（手写插入排序）：传入序不可依赖，确定性要自己立。
    for (std::size_t i = 1; i < crossOrders.size(); ++i) {
        const Order key = crossOrders[i];
        std::size_t j = i;
        while (j > 0 && crossOrders[j - 1].id > key.id) {
            crossOrders[j] = crossOrders[j - 1];
            --j;
        }
        crossOrders[j] = key;
    }
    if (crossOrders.empty()) {
        return;
    }
    // 处理顺序：**强制降级的单在前**（它们的直达插入要发生在所有站间腿落位之前），
    // 其余仍按订单 ID 升序（确定性不变）。
    {
        std::vector<Order> ordered;
        ordered.reserve(crossOrders.size());
        for (std::size_t i = 0; i < crossOrders.size(); ++i) {
            if (idListHas(forcedDirect, crossOrders[i].id)) {
                ordered.push_back(crossOrders[i]);
            }
        }
        for (std::size_t i = 0; i < crossOrders.size(); ++i) {
            if (!idListHas(forcedDirect, crossOrders[i].id)) {
                ordered.push_back(crossOrders[i]);
            }
        }
        crossOrders = ordered;
    }

    // 每车一份 Ctx（与 applyTransitLayer 同口径：仓库起点、发车时刻、车队全部订单入表——
    // 接送力的 kg/窗口查询要看得见别人的单）。
    std::vector<Ctx> ctxs(vehicles.size());
    for (std::size_t v = 0; v < vehicles.size(); ++v) {
        std::string reason;
        // 有已执行前缀就从当前位置/时刻起算（readyTime 链因此锚在实际执行点，而不是 08:00）。
        const std::string startPos = (starts != nullptr) ? (*starts)[v].startNodeId
                                                         : vehicles[v].startNodeId;
        const int startTime = (starts != nullptr) ? (*starts)[v].startTimeMin
                                                  : vehicles[v].departTimeMin;
        const std::vector<OnboardItem> onboard =
            (starts != nullptr) ? (*starts)[v].onboard : std::vector<OnboardItem>();
        const StashState stash = (starts != nullptr) ? (*starts)[v].stash : StashState();
        if (!makeCtx(graph, vehicles[v], vehicleOrders[v], startPos, startTime, weight,
                     onboard, stash, ctxs[v], reason)) {
            out.log.push_back("[接力] 跳过整层：车辆 " + vehicles[v].id + " 的上下文建不起来（"
                              + reason + "）");
            return;
        }
        appendFleetOrders(ctxs[v], orders);
    }

    // 已被接受的三段接力链（收口复核 + hint 迭代用）。
    std::vector<RelayChainRef> relayedChains;
    // 本轮被降级为直达的单（轮末统一追加到快车路线末尾；见「降级直达的落地」）。
    std::vector<Order> directDeferred;
    out.chains.clear();

    for (std::size_t oi = 0; oi < crossOrders.size(); ++oi) {
        const Order& o = crossOrders[oi];
        // ---- 摘单：从它当前所在车的路线里整段拿走 ----
        std::vector<std::vector<StopSpec> > specs(vehicles.size());
        std::vector<RoutePlan> detached = out.plans;
        bool detachOk = true;
        for (std::size_t v = 0; v < vehicles.size(); ++v) {
            specs[v] = specsFromPlan(out.plans[v]);
            removeOrderFromRoute(specs[v], o.id);
            if (!finishSpecs(ctxs[v], specs[v], detached[v])) {
                detachOk = false;
                break;
            }
        }
        if (!detachOk) {
            ++out.skippedCount;
            out.log.push_back("[接力] 订单 " + o.id + " 摘单后路线不可行 ⇒ 保持原分配");
            continue;
        }

        // ---- 三段接力 ----
        // forcedDirect 里的单不试三段腿：上一轮复核已判定它在最终计划里必不自洽（或由调用方
        // 判定结构上不可能），本轮直接走「职责降级直达」那一条，并如实点名原因。
        const bool forced = idListHas(forcedDirect, o.id);
        RelayBuild relay;
        if (!forced) {
            relay = buildRelayChain(graph, vehicles, o, detached, weight, relayIdx, slow, ctxs,
                                    readyHints);
        } else {
            relay.reason = "上一轮收口复核：三段腿在最终计划里不自洽（后续插入挪动了站停靠）";
        }

        // ---- 对照方案：该单**直接给快车**（在快车既有路线上插入取货 + 送达）----
        std::vector<RoutePlan> control = detached;
        bool haveControl = false;
        {
            std::vector<StopSpec> cand;
            bool found = false;
            if (buildRelayOnwardSpecs(ctxs[relayIdx], specsFromPlan(detached[relayIdx]), o.id,
                                      o.pickupNodeId, o.nodeId, false, false, cand, found)
                && found) {
                RoutePlan plan;
                if (finishSpecs(ctxs[relayIdx], cand, plan)) {
                    control[relayIdx] = plan;
                    haveControl = true;
                }
            }
        }

        // ---- 三段腿 vs「直接给快车」：判据按职责封闭重定（#46）----
        //
        // 对照方案「该单直接给快车」**只有在对紧急单的要求成立时才合法**
        // （职责表：跨簇紧急单且直达 penalty 严格小于 relay penalty）。于是：
        //   · 对照合法 ⇒ 两键（penalty / 该策略权重）不更差才选接力 —— 保留兜底，
        //     防止形式僵化出荒谬解（例如接力把一张单拖到几小时后）；
        //   · 对照**不合法**（普通跨簇单 / 紧急但直达不更优）⇒ 拿违规解去否决合法解，
        //     等于用破规则压住结构必需的方案。此时只在 **penalty** 上要求不更差
        //     （客户口径不能变坏），该策略权重只记日志 —— 这就是「转运成为跨簇单的
        //     唯一路径」在代码里的落点；分治带来的全局权重变差是**代价不是缺陷**。
        //
        // 均衡键照旧不参与（理由同 #48：接力必然把峰值利用率极差推大，拿它否决会让
        // 结构必需的接力永远退回直达）。
        Leg directLeg;
        directLeg.seq = 1;
        directLeg.from = o.pickupNodeId;
        directLeg.to = o.nodeId;
        directLeg.kind = LegKind::CustomerToCustomer;
        directLeg.orderId = o.id;
        directLeg.urgent = o.urgent;
        {
            const CrossPenaltyEstimate est =
                estimateCrossPenalties(graph, vehicles, o.pickupNodeId, o.nodeId,
                                       relay.stationA, relay.stationB, o.windowEndMin);
            directLeg.directPenaltyMin = est.ok ? est.directMin : -1;
            directLeg.relayPenaltyMin = est.ok ? est.relayMin : -1;
        }
        const bool directLegal = legVehicleEligible(graph, vehicles, relayIdx, directLeg);

        const FleetKey relayKey = fleetKeyOf(vehicles, relay.plans, weight);
        const FleetKey controlKey = fleetKeyOf(vehicles, control, weight);
        const bool relayNotWorse = relay.ok && relayKey.allOk
                                   && relayKey.penalty <= controlKey.penalty
                                   && (!directLegal
                                       || relayKey.weight <= controlKey.weight + kFleetEps);
        // custody 链自洽（在**最终计划**上核）：后续插入别单会重排同一台车的停靠，
        // 可能把先前那条腿的卸入时刻挪后 ⇒ 先前单的 readyTime 过期。核不过就退回直达。
        bool chainOk = true;
        if (relayNotWorse) {
            chainOk = relayChainConsistent(vehicles, relay.plans, o.id, relay.stationA,
                                           relay.stationB);
        }
        if (relayNotWorse && chainOk) {
            out.plans = relay.plans;
            for (std::size_t v = 0; v < out.vehicleOrders.size(); ++v) {
                out.vehicleOrders[v] = withoutOrderCopy(out.vehicleOrders[v], o.id);
            }
            out.vehicleOrders[relay.destIdx].push_back(o);
            ++out.relayedCount;
            out.touched.push_back(relay.srcIdx);
            out.touched.push_back(relayIdx);
            out.touched.push_back(relay.destIdx);
            {
                RelayChainRef ref;
                ref.orderId = o.id;
                ref.stationA = relay.stationA;
                ref.stationB = relay.stationB;
                ref.srcIdx = relay.srcIdx;
                ref.destIdx = relay.destIdx;
                relayedChains.push_back(ref);
            }
            std::ostringstream os;
            os << "[接力] " << o.id << " 三段腿成立：" << vehicles[relay.srcIdx].id << " 取 "
               << o.pickupNodeId << " -> 卸 " << relay.stationA << " "
               << minuteClockText(relay.t1) << "；" << vehicles[relayIdx].id << " "
               << relay.stationA << " -> " << relay.stationB << " 卸 "
               << minuteClockText(relay.t2) << "；" << vehicles[relay.destIdx].id << " 取 "
               << relay.stationB << " -> 送 " << o.nodeId << "（键 penalty " << relayKey.penalty
               << "/" << controlKey.penalty << "，权重 " << relayKey.weight << "/"
               << controlKey.weight << "，均衡 " << relayKey.balance << "/" << controlKey.balance
               << " 仅记日志"
               << (directLegal ? "" : "，直达对照不合职责 ⇒ 接力为结构性必需") << "）";
            out.log.push_back(os.str());
        } else if (haveControl && directLegal) {
            // 合法兜底：直达真的不更差，按职责表允许的「快车直达」执行。
            out.plans = control;
            for (std::size_t v = 0; v < out.vehicleOrders.size(); ++v) {
                out.vehicleOrders[v] = withoutOrderCopy(out.vehicleOrders[v], o.id);
            }
            out.vehicleOrders[relayIdx].push_back(o);
            ++out.fallbackCount;
            out.touched.push_back(relayIdx);
            out.log.push_back("[接力] 订单 " + o.id + " 不更差兜底："
                              + (!relay.ok ? std::string("三段腿不可行（" + relay.reason + "）")
                                 : !chainOk
                                       ? std::string("三段腿的 custody 链在最终计划里不自洽")
                                       : std::string("三段腿方案劣于直达对照（penalty ")
                                             + std::to_string(relayKey.penalty) + "/"
                                             + std::to_string(controlKey.penalty)
                                             + "，权重 " + std::to_string(relayKey.weight)
                                             + "/" + std::to_string(controlKey.weight) + "）")
                              + " ⇒ 按「直接给快车 " + vehicles[relayIdx].id + "」执行");
            for (std::size_t v = 0; v < ctxs.size(); ++v) {
                ctxs[v].readyAt.erase(o.id);
            }
        } else if (haveControl) {
            // 三段腿没成立，而「直接给快车」又不合职责表 —— 两难。取舍是
            // **不丢单**（既有不变量）优先：按职责表挑一台执行（表给不出就退化为快车），
            // 并**大声标记职责降级**，绝不冒充合规。反向验证 C 打的就是这里：
            // 把慢车对「跨簇客户腿」的禁令放宽之后，兜底会落到一台越界慢车上。
            std::size_t target = relayIdx;
            const std::string pick = chooseLegVehicle(graph, vehicles, directLeg);
            if (!pick.empty()) {
                for (std::size_t v = 0; v < vehicles.size(); ++v) {
                    if (vehicles[v].id == pick) {
                        target = v;
                    }
                }
            }
            std::vector<RoutePlan> degraded = detached;
            bool haveDegraded = false;
            if (target == relayIdx) {
                // 快车执行降级直达 ⇒ **交给轮末批量追加**（见下面「降级直达的落地」一段）。
                // 为什么不在这一单上立刻插：降级直达会把**客户停靠**插进快车的站间腿之间，
                // 把站停靠整体挪后 ⇒ 同轮已成立的其它单的三段腿 readyTime 过期（运单倒挂）。
                // 批量追加把「取货块 + 送达块」放在路线**末尾**（站停靠之后），不动任何既有
                // 停靠，因此与三段腿结构上不冲突；逐单追加还会把客户停靠交错摆放
                // （实测 12 单逐单追加 = 4624km / penalty 15545min）。
                directDeferred.push_back(o);
                for (std::size_t v = 0; v < out.vehicleOrders.size(); ++v) {
                    out.vehicleOrders[v] = withoutOrderCopy(out.vehicleOrders[v], o.id);
                }
                out.vehicleOrders[relayIdx].push_back(o);
                ++out.fallbackCount;
                out.log.push_back(
                    "[职责降级] 订单 " + o.id + " 三段腿未成立（"
                    + (!relay.ok ? std::string("不可行：" + relay.reason)
                       : !chainOk ? std::string("custody 链在最终计划里不自洽")
                                  : std::string("方案劣于直达对照"))
                    + "），且职责表不允许快车直达 ⇒ 为不丢单按「直达给 "
                    + vehicles[relayIdx].id + "」执行，**记为越界**（职责边界需人工关注）");
                for (std::size_t v = 0; v < ctxs.size(); ++v) {
                    ctxs[v].readyAt.erase(o.id);
                }
                continue;
            } else {
                std::vector<StopSpec> cand;
                bool found = false;
                if (buildRelayOnwardSpecs(ctxs[target], specsFromPlan(detached[target]), o.id,
                                          o.pickupNodeId, o.nodeId, false, false, cand, found)
                    && found) {
                    RoutePlan plan;
                    if (finishSpecs(ctxs[target], cand, plan)) {
                        degraded[target] = plan;
                        haveDegraded = true;
                    }
                }
            }
            if (haveDegraded) {
                out.plans = degraded;
                for (std::size_t v = 0; v < out.vehicleOrders.size(); ++v) {
                    out.vehicleOrders[v] = withoutOrderCopy(out.vehicleOrders[v], o.id);
                }
                out.vehicleOrders[target].push_back(o);
                ++out.fallbackCount;
                out.touched.push_back(target);
                out.log.push_back(
                    "[职责降级] 订单 " + o.id + " 三段腿未成立（"
                    + (!relay.ok ? std::string("不可行：" + relay.reason)
                       : !chainOk ? std::string("custody 链在最终计划里不自洽")
                                  : std::string("方案劣于直达对照"))
                    + "），且职责表不允许快车直达 ⇒ 为不丢单按「直达给 " + vehicles[target].id
                    + "」执行，**记为越界**（职责边界需人工关注）");
            } else {
                ++out.skippedCount;
                out.log.push_back("[接力] 订单 " + o.id
                                  + " 三段腿未成立且直达也构造不出来 ⇒ 该单没有执行方案，"
                                    "拒绝并点名（兜底链 ④）");
                out.plans = detached;
                for (std::size_t v = 0; v < out.vehicleOrders.size(); ++v) {
                    out.vehicleOrders[v] = withoutOrderCopy(out.vehicleOrders[v], o.id);
                }
            }
            for (std::size_t v = 0; v < ctxs.size(); ++v) {
                ctxs[v].readyAt.erase(o.id);
            }
        } else {
            ++out.skippedCount;
            out.log.push_back("[接力] 订单 " + o.id + " 三段腿不可行（" + relay.reason
                              + "）且直达对照也构造不出来 ⇒ 该单没有执行方案，"
                                "拒绝并点名（兜底链 ④）");
            out.plans = detached;
            for (std::size_t v = 0; v < out.vehicleOrders.size(); ++v) {
                out.vehicleOrders[v] = withoutOrderCopy(out.vehicleOrders[v], o.id);
            }
            for (std::size_t v = 0; v < ctxs.size(); ++v) {
                ctxs[v].readyAt.erase(o.id);
            }
        }
    }

    // ---- 站时刻定点收敛（规格五 #57）：把「车等货」的时刻钉到最终值上 ----
    //
    // 病灶：接力是逐单插入的，后面插入的单会把前面单的站停靠挪后 ⇒ 之前登记的 readyAt
    // 过期，重放出来的链在时空上倒挂（取走早于卸入）。站与站之间还有**循环依赖**：
    // V03 的 T02 停靠既是 2->3 的 L1 卸入、又是 1->2 的 L3 取走 —— V01 与 V03 在同一站上
    // 互为「等对方」，合法解只有两者取 max(raw) 的定点。
    //
    // 修法：反复「读当前实际卸站时刻 -> 回填 readyAt -> 只重放时刻（动作集不动）」直到
    // 不再变化（每轮时刻单调不减，收敛点 = 各站 raw 到达的最大值；实测 2 轮收敛）。
    // 重放失败（某个 ready 超出前瞻窗口/停留上限）⇒ 保留上一版，交给收口复核降级 ——
    // 不放宽任何常数。
    for (int it = 0; it < 8 && !relayedChains.empty(); ++it) {
        bool changed = false;
        for (std::size_t i = 0; i < relayedChains.size(); ++i) {
            const RelayChainRef& ref = relayedChains[i];
            if (ref.srcIdx >= vehicles.size() || ref.destIdx >= vehicles.size()) {
                continue;
            }
            const int t1 = transitStopArrival(out.plans[ref.srcIdx], ref.stationA, true,
                                              ref.orderId);
            if (t1 >= 0) {
                int& slot = ctxs[relayIdx].readyAt[ref.orderId][ref.stationA];
                if (slot != t1) {
                    slot = t1;
                    changed = true;
                }
            }
            const int t2 = transitStopArrival(out.plans[relayIdx], ref.stationB, true,
                                              ref.orderId);
            if (t2 >= 0) {
                int& slot = ctxs[ref.destIdx].readyAt[ref.orderId][ref.stationB];
                if (slot != t2) {
                    slot = t2;
                    changed = true;
                }
            }
        }
        if (!changed && it > 0) {
            break;
        }
        std::vector<RoutePlan> rebuilt = out.plans;
        bool rebuildOk = true;
        for (std::size_t v = 0; v < vehicles.size() && rebuildOk; ++v) {
            bool touchedVehicle = (v == relayIdx);
            for (std::size_t i = 0; i < relayedChains.size() && !touchedVehicle; ++i) {
                touchedVehicle = (relayedChains[i].srcIdx == v
                                  || relayedChains[i].destIdx == v);
            }
            if (!touchedVehicle) {
                continue;
            }
            if (!finishSpecs(ctxs[v], specsFromPlan(out.plans[v]), rebuilt[v])) {
                rebuildOk = false;
            }
        }
        if (!rebuildOk) {
            break;   // 保留上一版；哪条链坏了由收口复核点名并降级
        }
        out.plans = rebuilt;
    }

    // ---- 降级直达的落地：把「取货块 + 送达块」一次性追加到快车路线末尾 ----
    //
    // 位置在**全部跨簇单处理完之后**：批量追加发生在所有站间腿落位之后，因此不会挪动
    // 任何站停靠（这正是它与逐单全量枚举的差别 —— 后者会把客户停靠插进站间腿之间，
    // 让已成立的三段腿 readyTime 过期）。块的内部顺序 = 订单 ID 升序（与处理序一致）。
    // 若批量追加不可行（容量把所有降级单都同时装上车装不下等），退回逐单全量枚举并记日志。
    if (!directDeferred.empty()) {
        std::vector<StopSpec> specs = specsFromPlan(out.plans[relayIdx]);
        const std::size_t frozenStops = specs.size();
        // 落位顺序 = 地理扫掠（规格五 #57 实测教训）：按**取货点簇降序**（3 -> 2 -> 1）
        // 分组、组内按订单 ID，每组先铺全部取货、再铺全部送达。快车跑完最后一站后从
        // 簇 3 一路扫回 W01：3->1 的单（T03 附近取、簇 1 送）只穿一次干线；1->2 的单
        // 接在簇 1 送完之后取、再穿一次去簇 2 —— 总干线穿越次数 = 走廊数，而不是单数
        // （实测逐单交错摆放 = 4624km / penalty 15545min，扫掠布局把它压回一个量级）。
        std::vector<Order> sweep = directDeferred;
        for (std::size_t i = 1; i < sweep.size(); ++i) {
            const Order key = sweep[i];
            std::size_t j = i;
            while (j > 0) {
                const Node* pa = graph.findNode(sweep[j - 1].pickupNodeId);
                const Node* pb = graph.findNode(key.pickupNodeId);
                const int ca = (pa == nullptr) ? 0 : pa->subNetworkId;
                const int cb = (pb == nullptr) ? 0 : pb->subNetworkId;
                if (ca < cb || (ca == cb && sweep[j - 1].id > key.id)) {
                    sweep[j] = sweep[j - 1];
                    --j;
                } else {
                    break;
                }
            }
            sweep[j] = key;
        }
        int lastPickupCluster = -1;
        for (std::size_t i = 0; i < sweep.size(); ++i) {
            const Node* pn = graph.findNode(sweep[i].pickupNodeId);
            const int cluster = (pn == nullptr) ? 0 : pn->subNetworkId;
            if (cluster != lastPickupCluster) {
                // 新组：先铺这一组的全部取货。
                for (std::size_t m = i; m < sweep.size(); ++m) {
                    const Node* qn = graph.findNode(sweep[m].pickupNodeId);
                    if (qn != nullptr && qn->subNetworkId == cluster) {
                        insertLegActionAt(specs, specs.size(), sweep[m].pickupNodeId,
                                          sweep[m].id, false, false);
                    }
                }
                lastPickupCluster = cluster;
            }
            insertLegActionAt(specs, specs.size(), sweep[i].nodeId, sweep[i].id, false, true);
        }
        normalize(specs);
        RoutePlan batchPlan;
        if (finishSpecs(ctxs[relayIdx], specs, batchPlan)) {
            out.plans[relayIdx] = batchPlan;
            out.touched.push_back(relayIdx);
        } else {
            out.log.push_back("[职责降级] 降级直达的批量追加不可行（"
                              + batchPlan.reason + "）⇒ 退回逐单全量位置枚举");
            for (std::size_t i = 0; i < directDeferred.size(); ++i) {
                const Order& o = directDeferred[i];
                std::vector<RoutePlan> stripped = out.plans;
                bool stripOk = true;
                for (std::size_t v = 0; v < vehicles.size() && stripOk; ++v) {
                    std::vector<StopSpec> sp = specsFromPlan(out.plans[v]);
                    removeOrderFromRoute(sp, o.id);
                    if (!finishSpecs(ctxs[v], sp, stripped[v])) {
                        stripOk = false;
                    }
                }
                if (!stripOk) {
                    ++out.skippedCount;
                    out.log.push_back("[接力] 订单 " + o.id + " 降级直达也构造不出来 ⇒ "
                                      "拒绝并点名（兜底链 ④）");
                    continue;
                }
                std::vector<StopSpec> cand;
                bool found = false;
                if (buildRelayOnwardSpecs(ctxs[relayIdx], specsFromPlan(stripped[relayIdx]), o.id,
                                          o.pickupNodeId, o.nodeId, false, false, cand, found)
                    && found) {
                    RoutePlan plan;
                    if (finishSpecs(ctxs[relayIdx], cand, plan)) {
                        out.plans = stripped;
                        out.plans[relayIdx] = plan;
                    }
                }
            }
        }
        (void)frozenStops;
    }

    out.chains = relayedChains;

    // ---- 收口复核：本轮的接力链在**最终计划**上是否仍然自洽 ----
    //
    // 病灶（默认数据换成「簇内/跨簇参半」后必然触发）：接力是**逐单插入**的，后续插入的
    // 别单会重排同一台车（快车）的停靠 —— 一旦把某个站停靠挪后，先前那张单登记的
    // readyTime 就过期了：货还没到站，接力的取走车已经在站里把它「取走」了（运单倒挂）。
    // replay 只校验载重、不校验货物的时空连续性，所以这种倒挂能一路走到最终计划里，
    // 表现为 custody 缺口（有取走、其时刻早于卸入）。
    //
    // 这里只**报告**不自洽的单号；修法在 applyRelayLayer：把它们记进 forcedDirect 重跑整层。
    if (staleOut != nullptr) {
        for (std::size_t i = 0; i < relayedChains.size(); ++i) {
            if (!relayChainConsistent(vehicles, out.plans, relayedChains[i].orderId,
                                      relayedChains[i].stationA, relayedChains[i].stationB)) {
                staleOut->push_back(relayedChains[i].orderId);
            }
        }
    }
}

// 接力层入口（对外行为与签名不变）：**迭代到自洽**。
//
// 为什么要迭代（规格五 #57 实测出来的缺陷修法）：三段接力是**逐单插入**快车路线上的，
// 后续插入的别单（尤其是「接力不可行 ⇒ 职责降级直达」的客户停靠）可能把某个站的停靠
// 挪后 ⇒ 先前那张单登记的 readyTime 过期，接力的取走车在货到站之前就把它「取走」了
// （运单倒挂）。replay 只校验载重、不校验货物的时空连续性，所以倒挂能活到最终计划里，
// 表现为 custody 缺口（有取走、其时刻早于卸入）。
//
// 修法（**不改插入算法、不放宽任何常数**）：一轮跑完后逐单复核 relayChainConsistent；
// 不自洽的单记进 forcedDirect，**重跑整层**，这些单在下一轮**先**按「职责降级直达」落位 ——
// 它们的插入因此发生在所有站间腿之前，不会再把站停靠挪后。轮数上界 = 跨簇单数 + 1
// （每轮至少多一单被强制降级 ⇒ 单调收敛），默认数据实测 2 轮收敛。
void applyRelayLayer(const LogisticsGraph& graph, const std::vector<Vehicle>& vehicles,
                     const std::vector<Order>& orders, WeightType weight,
                     const std::vector<std::vector<Order> >& vehicleOrders,
                     const std::vector<RoutePlan>& basePlans,
                     const std::vector<Order>& relayOrders, RelayOutcome& out,
                     const std::vector<VehicleStartState>* starts = nullptr) {
    std::vector<std::string> forcedDirect;
    const int maxRounds = static_cast<int>(relayOrders.size()) + 2;
    for (int round = 0; round < maxRounds; ++round) {
        std::vector<std::string> stale;
        relayLayerOnce(graph, vehicles, orders, weight, vehicleOrders, basePlans, relayOrders,
                       forcedDirect, nullptr, &stale, out, starts);
        if (stale.empty()) {
            return;
        }
        bool grew = false;
        for (std::size_t i = 0; i < stale.size(); ++i) {
            if (!idListHas(forcedDirect, stale[i])) {
                forcedDirect.push_back(stale[i]);
                grew = true;
            }
        }
        if (!grew) {
            // 已经全部强制过仍不自洽：如实保留并点名（不伪造「链成立」）。
            out.log.push_back("[接力] 收口复核仍有 " + std::to_string(stale.size())
                              + " 张单的三段腿不自洽，且已全部按降级直达处理 ⇒ 保留现状并点名");
            return;
        }
    }
}

// ---- 职责封闭的车队分配（规格三 §4，默认入口）----
//
// 阶段一 聚类（§4.2）：簇 = 取货点 A 的 subNetworkId；A、B 跨网 ⇒ 跨簇单进跨簇队列。
//   本地单进 A 所在簇；subNetworkId == 0 的异常数据（手动新客户未继承）也归跨簇队列
//   并记日志 —— **不静默丢单**。#34 已保证建节点时继承最近邻的 sub_network_id。
// 阶段二 **固定指派**（#46）：慢车按 ID 升序、簇按簇 ID 升序**逐位配对**（规则只写在
//   slowVehicleOfCluster 一处），每簇订单交给本簇慢车跑**规格一的单车构造器**。
//   **没有枚举**：整簇全排列（规格二阶段二，6 种）已退役 —— 职责表把「簇 -> 车」的
//   可行集合收缩成单点，枚举只会构造越界解。跨簇单另走分流（普通单只走三段接力）。
// 阶段三 **退役**（#46）：有界跨车改派（R ≤ 3 轮、RelocateRecord）在职责封闭后没有
//   改派空间（一张簇内单只能由本簇慢车接）⇒ relocations 恒空，字段保留供报告/对照读。
// 兜底链：planRoute 内部的降载/全量兜底 -> 拒绝并点名；跨簇单另有「接力不可行 ⇒
//   职责降级直达（大声记日志）或拒绝」两条明确出路。
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

    // ---- 阶段二 · 固定指派：簇 -> 本簇慢车（**排列数 1，不枚举**）----
    //
    // 退役 ①：**整簇全排列枚举**（enumerateAssignments / 规格二 §4.3 的「3 簇 3 车 =
    //   6 种排列 + 车队键取优」）。
    //   为什么退役：职责封闭之后「一张簇内单只能由本簇慢车接」（见 legVehicleEligible
    //   的慢车侧判据），把一簇交给别的车是**一执行就越界**。于是「簇 -> 车」的可行集合
    //   被规则收缩成**单点**：排列数 = 1。继续枚举只会先构造出 5 个越界解再丢掉 ——
    //   白跑构造器，还会把越界解写进 candidateKeyLog，让人误以为它「可以被选中」。
    //   规则不写在分配器里，只写在 slowVehicleOfCluster（簇 ID 升序 × 慢车 ID 升序逐位
    //   配对）；分配器只调它 ⇒ 分配表与职责表不可能漂开。
    // 退役 ②：**有界跨车改派**（R <= 3 轮 + RelocateRecord / 规格二 §4.4）。
    //   为什么退役：改派的前提是「同一张单挪给另一台车仍然合职责」；职责封闭后这个前提
    //   恒假（簇内单只能由本簇慢车接）⇒ 候选集合恒空 ⇒ relocations 恒空。
    //   字段保留（报告与对照入口仍读它），只是不再产生记录。
    // 保留：planFleetSequentialBaseline（顺序轮转基线）。它不参与默认入口，是「不看
    //   职责」的对照物 —— 用来证明职责封闭不是靠分配器碰巧选对。
    const std::size_t relayIdx = relayVehicleIndex(vehicles);
    const std::vector<std::size_t> slow = slowVehiclesSortedById(vehicles, relayIdx);

    // 簇 -> 车辆行下标。慢车为空（车队只有 1 台车）时职责封闭不成立：由唯一那台车
    // 兜底服务全部本簇单，并**如实点名**，不声称已分治。
    std::vector<std::size_t> clusterOwner(groups.size(), k);
    for (std::size_t g = 0; g < groups.size(); ++g) {
        clusterOwner[g] = slow.empty()
                              ? relayIdx
                              : slowVehicleOfCluster(graph, vehicles, groups[g].clusterId);
    }

    std::vector<std::vector<Order> > vehicleOrders(k);
    if (slow.empty()) {
        fleet.allocationLog.push_back("车队没有慢车（只有 1 台车）⇒ 职责封闭无法成立："
                                      "本簇单由它兜底服务，不声称已分治");
    }
    for (std::size_t g = 0; g < groups.size(); ++g) {
        const std::size_t owner = clusterOwner[g];
        if (owner >= k) {
            fleet.allocationLog.push_back("簇 " + std::to_string(groups[g].clusterId)
                                          + " 没有本簇慢车 ⇒ 该簇订单交兜底链（逐单点名）");
            continue;
        }
        vehicleOrders[owner].insert(vehicleOrders[owner].end(), groups[g].orders.begin(),
                                    groups[g].orders.end());
    }
    // 慢车数 < 簇数 ⇒ 一台慢车守多簇（按簇 ID 升序循环配对）。这是**降级**，要留痕。
    if (!slow.empty()) {
        std::vector<int> ownerCount(k, 0);
        for (std::size_t g = 0; g < groups.size(); ++g) {
            if (clusterOwner[g] < k) {
                ++ownerCount[clusterOwner[g]];
            }
        }
        for (std::size_t v = 0; v < k; ++v) {
            if (ownerCount[v] > 1) {
                fleet.allocationLog.push_back(
                    "簇数 " + std::to_string(groups.size()) + " > 慢车数 "
                    + std::to_string(slow.size()) + " ⇒ 慢车 " + vehicles[v].id
                    + " 按「簇 ID 升序 × 慢车 ID 升序」循环配对，守 "
                    + std::to_string(ownerCount[v]) + " 个簇（职责封闭降级，如实记录）");
            }
        }
    }

    // ---- 跨簇单的分流（判据与职责表**同一句**）----
    //
    // · 普通跨簇单：**只能**走三段腿 ⇒ 一律交接力层，不由任何车直达；
    // · 紧急跨簇单：只有「直达 penalty **严格小于** relay penalty」才允许快车直接跑
    //   （职责表唯一的 customer->customer 例外），否则同样交接力层；
    // · 端点不属于任何簇（子网络 0 的历史数据 / 缺站）：职责表无从分类 ⇒ 交快车兜底
    //   并点名（异常数据不静默丢单，也不冒充已分治）。
    //
    // penalty 用规划期估计（estimateCrossPenalties），与交接力层的判定读同一函数；
    // 实际执行后 key 若与估计不符，「不更差」兜底会在接力层接手。
    std::vector<Order> relayOrders;
    for (std::size_t i = 0; i < crossOrders.size(); ++i) {
        const Order& o = crossOrders[i];
        const int ca = clusterOfNode(graph, o.pickupNodeId);
        const int cb = clusterOfNode(graph, o.nodeId);
        if (ca <= 0 || cb <= 0) {
            vehicleOrders[relayIdx].push_back(o);
            fleet.allocationLog.push_back(
                "订单 " + o.id + " 的端点不属于任何子网（" + std::to_string(ca) + " / "
                + std::to_string(cb) + "）⇒ 构不出三段腿，由快车 " + vehicles[relayIdx].id
                + " 兜底直达（异常数据，职责表无解，如实记录）");
            continue;
        }
        const CrossPenaltyEstimate est =
            estimateCrossPenalties(graph, vehicles, o.pickupNodeId, o.nodeId,
                                   stationOfCluster(graph, ca), stationOfCluster(graph, cb),
                                   o.windowEndMin);
        if (o.urgent && est.ok && est.directMin < est.relayMin) {
            vehicleOrders[relayIdx].push_back(o);
            fleet.allocationLog.push_back(
                "[职责] 跨簇紧急单 " + o.id + " 由快车 " + vehicles[relayIdx].id
                + " 直达：直达估计 penalty " + std::to_string(est.directMin) + " < relay "
                + std::to_string(est.relayMin) + "（职责表唯一的 customer->customer 例外）");
            continue;
        }
        relayOrders.push_back(o);
        if (o.urgent) {
            fleet.allocationLog.push_back(
                "[职责] 跨簇紧急单 " + o.id + " 的直达估计 penalty 不严格优于 relay（"
                + std::to_string(est.directMin) + " / " + std::to_string(est.relayMin)
                + "）⇒ 交接力层走三段腿");
        }
    }

    std::vector<RoutePlan> plans(k);
    for (std::size_t v = 0; v < k; ++v) {
        plans[v] = planRoute(graph, vehicles[v], vehicleOrders[v], weight);
    }
    int infeasibleVehicles = 0;
    for (std::size_t v = 0; v < k; ++v) {
        if (plans[v].status != PlanStatus::Ok) {
            ++infeasibleVehicles;
            fleet.allocationLog.push_back("车辆 " + vehicles[v].id + " 的本职路线不可行（"
                                          + plans[v].reason + "）");
        }
    }
    if (infeasibleVehicles > 0) {
        fleet.allocationLog.push_back(
            "兜底链：换车（随整簇枚举退役）与「把半簇挪给别的车」（随职责封闭退役）都已"
            "不可用，只剩「planRoute 内部的降载/全量兜底 -> 拒绝该单并点名」两级");
    }

    // 阶段二留痕：固定指派只有**一个**候选 ⇒ 恒 1 行（枚举时代的「6 行排列」已退役）。
    // 为什么还留这一行：candidateKeyLog 是「阶段二到底选了什么」的唯一留痕；
    // 退役的是「6 种排列的择优过程」，不是「记录结果」。
    {
        std::string line = "固定指派（职责封闭，排列数 1）";
        for (std::size_t g = 0; g < groups.size(); ++g) {
            line += " 簇" + std::to_string(groups[g].clusterId) + "=";
            line += (clusterOwner[g] < k) ? vehicles[clusterOwner[g]].id : std::string("-");
        }
        const FleetKey fixedKey = fleetKeyOf(vehicles, plans, weight);
        line += " key=(" + std::to_string(fixedKey.penalty) + ", "
                + std::to_string(fixedKey.weight) + ", " + std::to_string(fixedKey.balance)
                + ") 完工=" + std::to_string(fixedKey.completion) + "min";
        if (infeasibleVehicles > 0) {
            line += " 有 " + std::to_string(infeasibleVehicles) + " 台车不可行";
        }
        fleet.candidateKeyLog.push_back(line);
    }

    // 阶段三已退役：relocations 恒空（口径变更，守卫按新语义断言；见提交信息）。
    const std::vector<RelocateRecord> relocations;
    const bool anyFeasible = (infeasibleVehicles == 0 && rejectedOrders.empty());
    (void)relocations;

    // ---- 三段接力（规格三 §3 / #48）：跨簇普通单**只能**走三段腿 ----
    //
    // 位置：分配（含跨车改派）定稿之后、可选的转运改进之前。理由：接力会**改分配**
    // （同一张单的取/送拆给两台慢车），必须在所有「谁服务哪些单」的判断都做完之后再动；
    // 而它之后的一段（转运 ②③ 容量腾挪/紧急响应）只看计划形状，跟着接力版走即可。
    // 默认数据没有跨簇单 ⇒ 本调用立刻返回，计划与黄金值逐字不变。
    {
        RelayOutcome relayOutcome;
        applyRelayLayer(graph, vehicles, orders, weight, vehicleOrders, plans, relayOrders,
                        relayOutcome);
        plans = relayOutcome.plans;
        vehicleOrders = relayOutcome.vehicleOrders;
        for (std::size_t i = 0; i < relayOutcome.log.size(); ++i) {
            fleet.allocationLog.push_back(relayOutcome.log[i]);
        }
        if (relayOutcome.relayedCount > 0) {
            fleet.allocationLog.push_back("[接力] 合计 "
                                          + std::to_string(relayOutcome.relayedCount)
                                          + " 张跨簇普通单走完三段腿（不更差兜底 "
                                          + std::to_string(relayOutcome.fallbackCount)
                                          + " 张，跳过 " + std::to_string(relayOutcome.skippedCount)
                                          + " 张）");
        }
    }

    // ---- 后置有界改进 · 跨车转运（规格二 §5 / #28）----
    //
    // 只在三条判据之一成立时构造候选（默认数据全部不触发 ⇒ 一行候选都不生成，
    // 这是 --plan-summary 仍 < 5s 的前提）。转运层**不改分配**：谁取货仍由 #40 决定，
    // 只评估「给定分配下这一单改由另一台车送达是否严格更优」。
    FleetTransitResult transitResult;
    transitResult.plans = plans;
    if (anyFeasible) {
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

    // ---- 空闲归位（规格五 §5.13 / #57）：**所有「谁服务哪些单」的判断定稿之后**才收尾 ----
    //
    // 位置：接力层与转运层之后、aggregateFleet 之前。理由：
    //   · 归位只改「跑完之后的去向」（末段终点），不改任何停靠顺序/时刻/账本，所以放在
    //     所有分配与转运判定之后收口，前面那些层的键比较不受影响；
    //   · 汇总由 aggregateFleet 唯一实现，归位必须**先**写进逐车计划，车队口径才含归位那段。
    applyIdleHoming(graph, vehicles, weight, plans, &fleet.allocationLog);

    // ---- 收口：汇总 + 分配产物（日志先存下，aggregateFleet 返回的是新对象）----
    const std::vector<std::string> allocLog = fleet.allocationLog;
    const std::vector<std::string> keyLog = fleet.candidateKeyLog;
    fleet = aggregateFleet(vehicles, plans);
    fleet.allocationLog = allocLog;
    fleet.candidateKeyLog = keyLog;
    // 账本从**最终**计划反查（接力/转运都可能动过路线）：唯一实现仍是 buildTransitLedger。
    fleet.transitLegs = buildTransitLedger(vehicles, plans);
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

// ---- 事件路径的跨簇接力与腿重建（规格三 §2④ / US21 / P1）----

// 跨簇单能否走三段接力（结构前提）：两端都在某个非 0 簇、两簇各有中转站且不同。
// 不满足（缺站 / 子网 0 的异常数据）就不是「跨簇三段腿」能表达的单，交既有升级链兜底。
bool relayChainPossible(const LogisticsGraph& graph, const Order& o) {
    const int ca = clusterOfNode(graph, o.pickupNodeId);
    const int cb = clusterOfNode(graph, o.nodeId);
    if (ca <= 0 || cb <= 0 || ca == cb) {
        return false;
    }
    const std::string sa = stationOfCluster(graph, ca);
    const std::string sb = stationOfCluster(graph, cb);
    return !sa.empty() && !sb.empty() && sa != sb;
}

// 事件后重建腿结构（P1-③）+ 把「期末 0 空过」点名（§4.8 登记项）。
//
// 为什么单独重建：rescheduleFleet 干的是逐车重规划，FleetPlan::legs 只在 planFleet 收口
// 时 build 一次；事件不动它 ⇒ 呈现/守卫读到的腿仍是旧形态。这里用**同一套** buildLegs
// 读事件后的真实计划，保证「LEG 说的形态」= 「STOP 里的实际执行」。
//
// readyTime 锚定「已执行前缀」：事件从中途发起时，腿的可接时刻是**该车当前时刻**，
// 不是 08:00 发车时刻（规格三 Implementation Decisions §2④）。
void finalizeRescheduleLegs(const LogisticsGraph& graph, const std::vector<Vehicle>& vehicles,
                            const std::vector<Order>& legOrders,
                            const std::vector<VehicleStartState>& starts,
                            FleetRescheduleResult& res) {
    FleetPlan shell = aggregateFleet(vehicles, res.plans);
    // 让 buildLegs 提取「职责降级」原因时看得到接力层的点名行（与 planFleet 同口径）。
    shell.allocationLog = res.log;
    shell.legs = buildLegs(graph, vehicles, legOrders, shell, shell.legLog);
    bool anchored = false;
    for (std::size_t i = 0; i < shell.legs.size(); ++i) {
        Leg& leg = shell.legs[i];
        for (std::size_t v = 0; v < vehicles.size() && v < starts.size(); ++v) {
            if (leg.vehicleId != vehicles[v].id) {
                continue;
            }
            if (starts[v].startTimeMin > leg.readyTimeMin) {
                leg.readyTimeMin = starts[v].startTimeMin;
                anchored = true;
            }
            break;
        }
    }
    if (anchored) {
        shell.legLog.push_back("[腿] 事件路径的 readyTime 锚定已执行前缀（车辆当前位置/时刻），"
                               "不再用发车时刻");
    }
    // 兜底告警（#49 起正常情况下**不会**触发）：有卸入必须有**同站、另一台车**的取走。
    // 「封路/重排后 V02 在 T01 卸入、V04 在 T03 取走」的断链已由 repairCustodyBreaks 在
    // 收口前修好（重跑接力或显式拒绝），所以走到这里时不应再有未配对卸入。保留此逐笔点名
    // 作为最后的防线：万一将来某个新路径绕过了修复，仍能在 CLI/GUI 播报上看见，不静默。
    for (std::size_t v = 0; v < res.plans.size(); ++v) {
        for (std::size_t s = 0; s < res.plans[v].stops.size(); ++s) {
            const Stop& st = res.plans[v].stops[s];
            for (std::size_t i = 0; i < st.transitUnloads.size(); ++i) {
                const std::string& id = st.transitUnloads[i];
                bool matched = false;
                for (std::size_t v2 = 0; v2 < res.plans.size() && !matched; ++v2) {
                    if (v2 == v) {
                        continue;
                    }
                    for (std::size_t s2 = 0; s2 < res.plans[v2].stops.size() && !matched; ++s2) {
                        const Stop& st2 = res.plans[v2].stops[s2];
                        if (st2.nodeId != st.nodeId) {
                            continue;
                        }
                        for (std::size_t j = 0; j < st2.transitPickups.size(); ++j) {
                            if (st2.transitPickups[j] == id) {
                                matched = true;
                            }
                        }
                    }
                }
                if (!matched) {
                    res.log.push_back("[复调度] ⚠ 账本空过：订单 " + id + " 在 " + st.nodeId
                                      + " 由 " + vehicles[v].id + " 卸入，但没有同站取走"
                                        "（接力链断裂，货物跨站无承运）⇒ 期末 0 不能作数"
                                        "（§4.8 登记项）");
                }
            }
        }
    }
    res.legs = shell.legs;
    res.legLog = shell.legLog;
}

// ---- 规格三 #49：封路/路况打断接力链后的 custody 断链修复 ----
//
// 判据只读**计划产物**：某笔 transitUnload 找不到「**同站、另一台车**」的 transitPickup
// ⇒ 货物在站里没有承接人（跨站无承运，正是封路把 T01->T03 快车腿吃掉后的病灶）。
// 返回这些订单 ID（去重、按出现序）。
//
// 为什么这里**不**把「取走时刻 >= 卸入时刻」也放进判据：时刻倒挂（取走比卸入早几分钟）
// 是既有「逐易手账本 + 局部重排」的另一类保真问题（US19，已登记为不修），任何一次与本单
// 无关的逐车重排都可能把它挪动几分钟；把它当断链会误伤**没有跨站**的正常接力、并触发没
// 必要的重跑。跨站无承运才是本票要修的封路病灶；时刻单调由 repairCustodyBreaks 末尾的
// relayChainConsistent 在**重跑后**严格复核（那里要求取走 >= 卸入且等待 <= W）。
std::vector<std::string> unpairedUnloadOrders(const std::vector<Vehicle>& vehicles,
                                              const std::vector<RoutePlan>& plans) {
    std::vector<std::string> broken;
    for (std::size_t v = 0; v < plans.size() && v < vehicles.size(); ++v) {
        if (plans[v].status != PlanStatus::Ok) {
            continue;
        }
        for (std::size_t s = 0; s < plans[v].stops.size(); ++s) {
            const Stop& st = plans[v].stops[s];
            for (std::size_t i = 0; i < st.transitUnloads.size(); ++i) {
                const std::string& id = st.transitUnloads[i];
                bool matched = false;
                for (std::size_t v2 = 0; v2 < plans.size() && !matched; ++v2) {
                    if (v2 == v) {
                        continue;
                    }
                    for (std::size_t s2 = 0; s2 < plans[v2].stops.size() && !matched; ++s2) {
                        const Stop& st2 = plans[v2].stops[s2];
                        if (st2.nodeId != st.nodeId) {
                            continue;
                        }
                        for (std::size_t j = 0; j < st2.transitPickups.size(); ++j) {
                            if (st2.transitPickups[j] == id) {
                                matched = true;
                            }
                        }
                    }
                }
                bool known = false;
                for (std::size_t b = 0; b < broken.size(); ++b) {
                    if (broken[b] == id) {
                        known = true;
                    }
                }
                if (!matched && !known) {
                    broken.push_back(id);
                }
            }
        }
    }
    return broken;
}

// 收口前的 custody 断链修复（US22 / US29）。
//
// 为什么首选「用新图重跑该单的三段接力」而不是直接拒绝：封路只堵 T01->T03 的**直达**，
// 经 W01 的绕行（T01->W01->T03）仍在，接力往往仍可行 —— 一刀切拒绝会把能走通的单也
// 丢掉。只有新图下确实接不上（或该单根本不是可接力的跨簇单）时才显式拒绝并记原因，
// 这就是五级链第 ⑤ 级的落点；绝不允许「无车承运即跨站」通过。
//
// starts = 已执行前缀：事件从中途发起时，接力的可接时刻锚在该车当前位置/时刻。
bool repairCustodyBreaks(const LogisticsGraph& graph, const std::vector<Vehicle>& vehicles,
                         WeightType weight, const FleetRescheduleInput& input,
                         const std::vector<Order>& legOrders,
                         std::vector<std::vector<Order> >& work, std::vector<RoutePlan>& plans,
                         std::vector<std::size_t>& replanned, std::vector<std::string>& log,
                         std::string& why, bool repairEnabled) {
    // 只在**封路/路况**这类「把某条腿打断」的事件上修 custody：跨簇新单另有 ①-a 的接力
    // 分流（新单本来就重新构造三段接力，不存在旧链被打断的问题），而 Manual/NewOrder 在
    // 已执行前缀上的保真属 US19（逐易手账本，已登记不修），把它们一并「重跑接力」会把
    // 中途已执行的卸入当未执行重排，反而丢单 —— 那不是本票范围，也不该由这里兜。
    if (!repairEnabled) {
        return true;
    }
    const std::vector<std::string> broken = unpairedUnloadOrders(vehicles, plans);
    if (broken.empty()) {
        return true;   // 没有断链：原样通过（默认数据/无跨簇事件逐字不变）
    }
    std::vector<Order> targets;
    for (std::size_t i = 0; i < broken.size(); ++i) {
        Order found;
        bool have = false;
        for (std::size_t j = 0; j < legOrders.size(); ++j) {
            if (legOrders[j].id == broken[i]) {
                found = legOrders[j];
                have = true;
            }
        }
        if (!have) {
            why = "订单 " + broken[i] + " 有卸入、无同站取走，但订单表里找不到它（无法重跑接力）";
            return false;
        }
        if (!(isCrossCluster(graph, found) && relayChainPossible(graph, found))) {
            why = "订单 " + found.id + " 有卸入、无同站取走，且不是可接力的跨簇单 "
                  "（无法用三段接力修复）";
            return false;
        }
        targets.push_back(found);
    }
    // 用**新图**重跑这些单的三段接力（starts = 已执行前缀）。
    const std::vector<Order> fleetOrders = legOrders;
    RelayOutcome ro;
    applyRelayLayer(graph, vehicles, fleetOrders, weight, work, plans, targets, ro, &input.starts);
    for (std::size_t i = 0; i < ro.log.size(); ++i) {
        log.push_back("[复调度] " + ro.log[i]);
    }
    for (std::size_t i = 0; i < ro.touched.size(); ++i) {
        const std::size_t t = ro.touched[i];
        if (t < ro.plans.size() && t < plans.size()) {
            plans[t] = ro.plans[t];
            replanned.push_back(t);
        }
    }
    work = ro.vehicleOrders;
    // 复核：每张目标单都必须真的走完**合法接力**（6 个 custody 锚点齐全、相邻不同车、
    // 等待落在 W 内）。relayChainConsistent 读的是最终计划的停靠，不是重算公式。
    for (std::size_t i = 0; i < targets.size(); ++i) {
        const Order& o = targets[i];
        const std::string sa = stationOfCluster(graph, clusterOfNode(graph, o.pickupNodeId));
        const std::string sb = stationOfCluster(graph, clusterOfNode(graph, o.nodeId));
        if (!relayChainConsistent(vehicles, plans, o.id, sa, sb)) {
            why = "封路后订单 " + o.id + " 无法重跑成合法三段接力（"
                  + (ro.relayedCount > 0 ? std::string("新图下接力链仍不自洽")
                                         : std::string("新图下接力不可行"))
                  + "）";
            return false;
        }
    }
    // 再核：不能留下别的未配对卸入。
    const std::vector<std::string> still = unpairedUnloadOrders(vehicles, plans);
    if (!still.empty()) {
        why = "重跑接力后仍有「有卸入、无同站取走」的订单 " + still[0];
        return false;
    }
    log.push_back("[复调度] custody 断链已修复：用新图重跑 " + std::to_string(targets.size())
                  + " 张跨簇单的三段接力（绕行 / 账本两端齐全 / 期末 0）");
    return true;
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

    // 事件后重建腿结构要用「未送达订单全集 + 新单」（buildLegs 按订单展开腿）。
    std::vector<Order> legOrders;
    for (std::size_t i = 0; i < input.allRemainingOrders.size(); ++i) {
        if (!input.allRemainingOrders[i].served) {
            legOrders.push_back(input.allRemainingOrders[i]);
        }
    }
    if (hasPending && !containsOrderId(legOrders, pending.id)) {
        legOrders.push_back(pending);
    }

    // 收口（每条被接受的升级级都走这里）：先修「封路/路况打断接力链」造成的 custody
    // 断链（#49 / US22 / US29），再重建腿结构。修不好就按五级链第 ⑤ 级拒绝并点名，
    // 绝不留下「无车承运即跨站」。
    auto accept = [&]() -> FleetRescheduleResult {
        std::string custodyWhy;
        const bool repairEnabled = (event.trigger == RescheduleTrigger::RoadClosure
                                    || event.trigger == RescheduleTrigger::Traffic);
        if (!repairCustodyBreaks(graph, vehicles, weight, input, legOrders, work, plans,
                                 res.replanned, res.log, custodyWhy, repairEnabled)) {
            res.level = 5;
            res.rejected = true;
            res.reason = custodyWhy;
            res.log.push_back("[复调度] 拒绝：" + custodyWhy);
            res.plans = input.remainingPlans;
            res.vehicleOrders = input.vehicleOrders;
            return res;
        }
        // 归位收口（规格五 §5.13）：复调度同样以「回主场待命」结束（与 planFleet 同一机件）。
        // 幂等：未受影响的车的末段本就已是归位段 ⇒ 三项逐字不动（不变量 5 不受影响）。
        applyIdleHoming(graph, vehicles, weight, plans, &res.log);
        res.plans = plans;
        res.vehicleOrders = work;
        finalizeRescheduleLegs(graph, vehicles, legOrders, input.starts, res);
        return res;
    };

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
        // ---- ①-a 跨簇单：走与 planFleet **同一套**三段接力（P1 / US21）----
        //
        // 为什么放在第 ① 级之前分叉：跨簇单的**合法执行形态只有三段腿**（职责表：
        // 慢车不接跨簇客户腿、快车不接非紧急客户腿），「单台车插入」必然越界
        // （评审实测：事件后 V04 把 O26 直达取送、账本 0 笔）。所以它一进升级链就
        // 走接力机件，不与「单台车插入」竞争；分流判据与 planFleet 同一句
        // （紧急且直达估计严格更优才允许快车直达，其余一律接力 —— 由 applyRelayLayer
        // 内部按职责表判定，不在本层重写一遍）。
        if (isCrossCluster(graph, pending) && relayChainPossible(graph, pending)) {
            std::vector<Order> fleetOrders = legOrders;   // 未送达全集（已含 pending）
            std::vector<Order> relayOrders;
            relayOrders.push_back(pending);
            RelayOutcome relayOutcome;
            applyRelayLayer(graph, vehicles, fleetOrders, weight, work, plans, relayOrders,
                            relayOutcome, &input.starts);
            for (std::size_t i = 0; i < relayOutcome.log.size(); ++i) {
                res.log.push_back("[复调度] " + relayOutcome.log[i]);
            }
            const bool placed = (relayOutcome.relayedCount > 0 || relayOutcome.fallbackCount > 0);
            if (placed) {
                for (std::size_t i = 0; i < relayOutcome.touched.size(); ++i) {
                    const std::size_t t = relayOutcome.touched[i];
                    if (t < plans.size() && t < relayOutcome.plans.size()) {
                        plans[t] = relayOutcome.plans[t];
                        res.replanned.push_back(t);
                    }
                }
                work = relayOutcome.vehicleOrders;
                res.level = 1;
                res.log.push_back("[复调度] 跨簇单 " + pending.id
                                  + " 走三段接力（与静态 planFleet 同机件，readyTime 锚已执行前缀）");
                std::string why;
                if (!inheritExecutedInStation(graph, vehicles, weight, input, work, plans,
                                              res.replanned, res.log, why)) {
                    res.level = 5;
                    res.rejected = true;
                    res.reason = why;
                    res.log.push_back("[复调度] 拒绝：" + why);
                    return res;
                }
                return accept();
            }
            res.log.push_back("[复调度] 跨簇单 " + pending.id
                              + " 三段接力未成立（各腿不可行 / 不更差兜底与职责降级均未落位）"
                                " → 退回既有升级链（降级结果由腿结构的 LEGDEGRADE 点名）");
        }
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
            return accept();
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
            return accept();
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
        return accept();
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
            return accept();
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
            return accept();
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