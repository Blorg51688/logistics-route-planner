#!/usr/bin/env python3
"""独立期望值计算器（测试预言机 / test oracle）。

背景：TDD 要求期望值来自"独立的真值来源"。此前由人手工推算，已连续出错三次
（路径绕行、尺度、单位），因此改为脚本计算。

**独立性保证（避免同义反复）**——本脚本刻意不复用被测代码的算法：
  1. 最短路用「穷举所有简单路径取最小」的暴力法，与 C++ 侧的 Dijkstra + 手写堆
     是两种不同方法；两者同时错成一样的概率极低。
  2. 时间模型直接按 docs/设计.md §4.3 的定义转写，不复用 RoutePlanner 的实现。

用法：
    python3 tools/oracle.py            # 打印全部测试夹具的期望值
    python3 tools/oracle.py --check     # 校验内置自检用例，非零退出码表示 oracle 自身有问题

本脚本只用于离线生成测试字面量：不参与构建，不被交付程序引用。
"""

import sys

INF = float("inf")
EPS = 1e-9


class Graph:
    def __init__(self):
        self.nodes = []
        self.edges = {}  # (u, v) -> {"dist": .., "time": .., "cost": ..}

    def add_node(self, name):
        if name not in self.nodes:
            self.nodes.append(name)

    def add_edge(self, u, v, dist, t, cost):
        self.nodes_add(u)
        self.nodes_add(v)
        self.edges[(u, v)] = {"dist": dist, "time": t, "cost": cost}

    def nodes_add(self, n):
        if n not in self.nodes:
            self.nodes.append(n)

    def two_way(self, u, v, dist, t, cost):
        self.add_edge(u, v, dist, t, cost)
        self.add_edge(v, u, dist, t, cost)

    def out_edges(self, u):
        return sorted((v, w) for (a, v), w in self.edges.items() if a == u)

    def weight(self, edge, kind):
        return edge["dist"] if kind == "distance" else edge["time"] if kind == "time" else edge["cost"]


def all_simple_paths(g, src, dst, _seen=None):
    """穷举 src->dst 的所有简单路径（暴力法，与 Dijkstra 完全不同的方法）。"""
    if _seen is None:
        _seen = []
    _seen = _seen + [src]
    if src == dst:
        return [[src]]
    paths = []
    for nxt, _w in g.out_edges(src):
        if nxt in _seen:
            continue
        for tail in all_simple_paths(g, nxt, dst, _seen):
            paths.append([src] + tail)
    return paths


def shortest(g, src, dst, kind):
    """返回 (路径, 总权重)；不可达返回 (None, inf)。并列时按路径字典序取。"""
    if src == dst:
        return [src], 0.0
    best, best_w = None, INF
    for path in all_simple_paths(g, src, dst):
        w = 0.0
        for i in range(1, len(path)):
            w += g.weight(g.edges[(path[i - 1], path[i])], kind)
        if w < best_w - EPS or (abs(w - best_w) < EPS and best is not None and path < best):
            best, best_w = path, w
    if best is None:
        return None, INF
    return best, best_w


def path_weight(g, path, kind):
    return sum(g.weight(g.edges[(path[i - 1], path[i])], kind) for i in range(1, len(path)))


def route_metrics(g, start_pos, return_to, start_time, stops, windows, service_min, kind):
    """按 设计.md §4.3 的时间模型推进。

    stops: 依次服务的节点；windows: {node: (start, end)} 已合并后的窗口。
    返回完整节点序列、逐站记录与汇总。distance/cost 始终按实际维度累加，
    时间推进始终用耗时维度（与所选策略无关）。
    """
    seq = [start_pos]
    cur = start_pos
    elapsed = float(start_time)
    total_dist = 0.0
    total_cost = 0.0
    records = []
    total_penalty = 0

    for node in stops:
        path, _w = shortest(g, cur, node, kind)
        if path is None:
            return None
        for i in range(1, len(path)):
            total_dist += g.edges[(path[i - 1], path[i])]["dist"]
            total_cost += g.edges[(path[i - 1], path[i])]["cost"]
            elapsed += g.edges[(path[i - 1], path[i])]["time"]
            seq.append(path[i])

        ws, we = windows[node]
        raw = elapsed
        wait = max(0.0, ws - raw)
        arrival = raw + wait
        late = arrival > we + EPS
        penalty = round(arrival - we) if late else 0
        elapsed = arrival + service_min
        total_penalty += penalty
        records.append({
            "node": node, "raw": round(raw), "wait": round(wait),
            "arrival": round(arrival), "departure": round(elapsed),
            "late": late, "penalty": penalty,
        })
        cur = node

    back, _w = shortest(g, cur, return_to, kind)
    if back is None:
        return None
    for i in range(1, len(back)):
        total_dist += g.edges[(back[i - 1], back[i])]["dist"]
        total_cost += g.edges[(back[i - 1], back[i])]["cost"]
        elapsed += g.edges[(back[i - 1], back[i])]["time"]
        seq.append(back[i])

    return {
        "nodes": seq,
        "stops": records,
        "total_distance": round(total_dist, 6),
        "total_time": round(elapsed - start_time, 6),
        "total_cost": round(total_cost, 6),
        "total_penalty": total_penalty,
    }


# ---------------------------------------------------------------- 测试夹具
# 以下图与 tests/*.cpp 中构造的图一一对应，便于交叉核对。

def fixture_dijkstra():
    g = Graph()
    g.add_edge("A", "B", 1.0, 10.0, 5.0)
    g.add_edge("A", "C", 4.0, 3.0, 2.0)
    g.add_edge("B", "C", 1.0, 10.0, 5.0)
    g.add_edge("B", "D", 1.0, 1.0, 5.0)
    g.add_edge("C", "D", 1.0, 1.0, 2.0)
    g.add_node("E")            # 孤岛
    return g


def fixture_w_d1():
    g = Graph()
    g.two_way("W", "D1", 5.0, 10.0, 4.0)
    return g


def fixture_greedy():
    g = Graph()
    g.two_way("W", "D1", 1.0, 2.0, 1.0)
    g.two_way("W", "D2", 5.0, 10.0, 5.0)
    g.two_way("D1", "D2", 2.0, 4.0, 2.0)
    return g


def fixture_tie():
    g = Graph()
    g.two_way("W", "DA", 3.0, 6.0, 3.0)
    g.two_way("W", "DZ", 3.0, 6.0, 3.0)
    g.two_way("DA", "DZ", 2.0, 4.0, 2.0)
    return g


def show(title, result):
    print(f"\n=== {title} ===")
    if result is None:
        print("  <不可达>")
        return
    print(f"  nodes        = {result['nodes']}")
    for r in result["stops"]:
        print(f"  stop {r['node']}: raw={r['raw']} wait={r['wait']} "
              f"arrival={r['arrival']} departure={r['departure']} "
              f"late={r['late']} penalty={r['penalty']}")
    print(f"  totalDistance = {result['total_distance']}")
    print(f"  totalTime     = {result['total_time']}")
    print(f"  totalCost     = {result['total_cost']}")
    print(f"  totalPenalty  = {result['total_penalty']}")


def self_check():
    """自检：用可由人工复核的极小用例验证 oracle 自身没写错。"""
    ok = True
    g = fixture_dijkstra()

    path, w = shortest(g, "A", "D", "distance")
    if path != ["A", "B", "D"] or abs(w - 2.0) > EPS:
        print(f"[FAIL] 自检 A->D 距离: {path} {w}")
        ok = False

    path, w = shortest(g, "A", "D", "time")
    if path != ["A", "C", "D"] or abs(w - 4.0) > EPS:
        print(f"[FAIL] 自检 A->D 耗时: {path} {w}")
        ok = False

    path, w = shortest(g, "A", "C", "distance")
    if path != ["A", "B", "C"] or abs(w - 2.0) > EPS:
        print(f"[FAIL] 自检 A->C 距离: {path} {w}")
        ok = False

    path, w = shortest(g, "A", "E", "distance")
    if path is not None:
        print(f"[FAIL] 自检 A->E 应不可达: {path}")
        ok = False

    # 三节点链上的时间模型：(W->D1 10min) 到达 490，窗口 540 -> 等待 50
    g2 = fixture_w_d1()
    r = route_metrics(g2, "W", "W", 480, ["D1"], {"D1": (540, 1080)}, 5.0, "distance")
    if r["stops"][0]["raw"] != 490 or r["stops"][0]["wait"] != 50 or r["stops"][0]["arrival"] != 540:
        print(f"[FAIL] 自检时间模型: {r['stops'][0]}")
        ok = False
    if abs(r["total_time"] - 75.0) > EPS:
        print(f"[FAIL] 自检总耗时: {r['total_time']}")
        ok = False

    if ok:
        print("[oracle] 自检通过")
    return 0 if ok else 1


def main():
    if "--check" in sys.argv:
        return self_check()

    print("=" * 72)
    print("Dijkstra 夹具（tests/dijkstra_tests.cpp）")
    print("=" * 72)
    g = fixture_dijkstra()
    for kind in ("distance", "time", "cost"):
        for dst in ("D", "C"):
            path, w = shortest(g, "A", dst, kind)
            print(f"  A->{dst} {kind:8s} path={path} weight={w}")
    path, w = shortest(g, "A", "E", "distance")
    print(f"  A->E distance 不可达: {path is None}")

    print("\n" + "=" * 72)
    print("RoutePlanner 夹具（tests/routeplanner_tests.cpp）")
    print("=" * 72)

    show("切片1 单订单 W<->D1，窗口 540-1080，发车 480，服务 5",
         route_metrics(fixture_w_d1(), "W", "W", 480, ["D1"], {"D1": (540, 1080)}, 5.0, "distance"))

    show("切片1 空订单",
         route_metrics(fixture_w_d1(), "W", "W", 480, [], {}, 5.0, "distance"))

    show("切片2 贪心 D1 优先（W-D1=1.0 近于 W-D2=5.0）",
         route_metrics(fixture_greedy(), "W", "W", 480, ["D1", "D2"],
                       {"D1": (0, 1440), "D2": (0, 1440)}, 5.0, "distance"))

    show("切片2 并列：应取字典序 DA",
         route_metrics(fixture_tie(), "W", "W", 480, ["DA", "DZ"],
                       {"DA": (0, 1440), "DZ": (0, 1440)}, 5.0, "distance"))

    show("切片3 紧急优先：先 D2 再 D1",
         route_metrics(fixture_greedy(), "W", "W", 480, ["D2", "D1"],
                       {"D1": (0, 1440), "D2": (0, 1440)}, 5.0, "distance"))

    show("切片4 晚到：窗口 480-485",
         route_metrics(fixture_w_d1(), "W", "W", 480, ["D1"], {"D1": (480, 485)}, 5.0, "distance"))

    show("切片4 合并：窗口并集 480-900",
         route_metrics(fixture_w_d1(), "W", "W", 480, ["D1"], {"D1": (480, 900)}, 5.0, "distance"))

    show("切片6 replan：已到 D2，时刻 600，剩余 D1",
         route_metrics(fixture_greedy(), "D2", "W", 600, ["D1"],
                       {"D1": (0, 1440)}, 5.0, "distance"))

    show("切片6 replan：已在 D1，时刻 600",
         route_metrics(fixture_w_d1(), "D1", "W", 600, ["D1"], {"D1": (0, 1440)}, 5.0, "distance"))

    return 0


if __name__ == "__main__":
    sys.exit(main())
