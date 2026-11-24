#include "core/RoutePlanner.h"

#include <cmath>
#include <cstdio>
#include <cstddef>
#include <map>
#include <unordered_map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace logistics {

std::vector<std::string> Stop::sortedActionOrderIds() const {
    std::vector<std::string> ids;
    ids.reserve(pickups.size() + deliveries.size() + offloads.size() + reclaims.size());
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
    StopPhase                phase = StopPhase::Normal;

    bool empty() const {
        return pickups.empty() && deliveries.empty() && offloads.empty() && reclaims.empty();
    }
    // 只含中转站动作（降载 / 取回），不含取送动作。
    bool isTransitOnly() const {
        return pickups.empty() && deliveries.empty()
               && (!offloads.empty() || !reclaims.empty());
    }
    // 排序后的动作订单 ID（tieKey 的第二个分量，见 #18 §5）。手写插入排序。
    std::vector<std::string> sortedActionIds() const {
        std::vector<std::string> ids;
        ids.reserve(pickups.size() + deliveries.size() + offloads.size() + reclaims.size());
        ids.insert(ids.end(), pickups.begin(), pickups.end());
        ids.insert(ids.end(), deliveries.begin(), deliveries.end());
        ids.insert(ids.end(), offloads.begin(), offloads.end());
        ids.insert(ids.end(), reclaims.begin(), reclaims.end());
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
// 字典序键。规格 §5 原文是 (Σpenalty, Σ该策略权重, tieKey)；实现里在 penalty
// 与权重之间多插了一档**载重峰值**（penalty 并列时优先选不把车塞满的排法）。
//
// 为什么（实测，默认数据）：容量压力是"强制重访"的唯一来源；把峰值放进键里
// 直接消掉那些重访（停靠 31/31/33 → 29/28/29），也让三策略重新满足
// 「各自在自身目标上不劣于另两条」。纯三键下 cost 策略会被距离策略在**成本**上
// 反超（168.5 vs 145.3），验收②不成立。这是对键序的有意偏离，已在交付说明里点名。
struct Key {
    int    penalty = 0;
    double peak = 0.0;     // 载重峰值 kg（偏离项，见上）
    double weight = 0.0;
};

// <0 表示 a 优于 b；0 表示完全并列。
int compareKey(const Key& a, const Key& b) {
    if (a.penalty != b.penalty) {
        return a.penalty < b.penalty ? -1 : 1;
    }
    if (!nearlyEq(a.peak, b.peak)) {
        return a.peak < b.peak ? -1 : 1;
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

    out.key.peak = peak;
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
bool appendUrgentAll(const Ctx& ctx, Builder& b, Outcome& out) {
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

    if (!appendUrgentAll(ctx, b, out)) {
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

// 主构造 + 全量兜底（#18 Q4=B）：
//   ① 按 EDD 序构造；
//   ② 失败时把「插不进去的那张单」提到最前再整体构造一次；
//   两版都可行则按 key 取优（penalty 是第一键，等价于「penalty 一票否决」）。
// 不做周期性全量重算：只有插入失败这一个触发点。
bool buildAll(const Ctx& ctx, Outcome& out) {
    // ① 主构造：EDD 序，不降载。
    Outcome first;
    if (buildWithOrder(ctx, std::vector<Request>(), false, first)) {
        out = first;
        return true;
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
    std::vector<std::string> ids;
    for (const Request& r : ctx.urgent) {
        ids.push_back(r.orderId);
    }
    for (const Request& r : ctx.normal) {
        ids.push_back(r.orderId);
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
                 const StashState& stash) {
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
    Outcome out;
    if (!buildAll(ctx, out)) {
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

InsertResult insertUrgentOrder(const LogisticsGraph& graph,
                               const Vehicle& vehicle,
                               const std::vector<Order>& remainingOrders,
                               const Order& newOrder,
                               const std::string& currentPositionId,
                               int currentTimeMin,
                               WeightType weight,
                               const std::vector<OnboardItem>& onboard,
                               const StashState& stash) {
    InsertResult result;

    // 插入的订单一律按紧急处理，调用方传入的 urgent 不作数
    Order inserted = newOrder;
    inserted.urgent = true;

    std::vector<Order> all = remainingOrders;
    all.push_back(inserted);

    // 可达性与窗口：按 current -> A(取货) -> B(送达) 的**该车缩放后**耗时估计。
    if (inserted.pickupNodeId.empty()) {
        result.warning = "紧急取送单缺少取货点 A";
    } else {
        const PathResult toA =
            shortestPath(graph, currentPositionId, inserted.pickupNodeId, WeightType::Time);
        const PathResult ab =
            shortestPath(graph, inserted.pickupNodeId, inserted.nodeId, WeightType::Time);
        if (!toA.found) {
            result.warning = "紧急取货点 A 不可达: " + inserted.pickupNodeId;
        } else if (!ab.found) {
            result.warning = "紧急送达点 B 不可达: " + inserted.nodeId;
        } else {
            const double scaled =
                (toA.totalWeight + ab.totalWeight) * vehicle.timeFactor;
            const int earliestArrival =
                currentTimeMin + static_cast<int>(std::lround(scaled));
            if (inserted.windowEndMin < earliestArrival) {
                result.warning = "紧急订单 " + inserted.id + " 无法在窗口内送达（窗口止 "
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
                            const TrafficReport& report,
                            double thresholdRatio,
                            const std::vector<OnboardItem>& onboard,
                            bool* usedIncremental,
                            const StashState& stash) {
    if (usedIncremental != nullptr) {
        *usedIncremental = false;
    }
    (void)report;
    (void)thresholdRatio;
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

} // namespace logistics
