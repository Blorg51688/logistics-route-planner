#!/usr/bin/env python3
"""独立期望值计算器（测试预言机 / test oracle）。

背景：TDD 要求期望值来自"独立的真值来源"。此前由人手工推算，已连续出错三次
（路径绕行、尺度、单位），因此改为脚本计算。

**独立性保证（避免同义反复）**——本脚本刻意不复用被测代码的算法：
  1. 最短路用「穷举所有简单路径取最小」的暴力法，与 C++ 侧的 Dijkstra + 手写堆
     是两种不同方法；两者同时错成一样的概率极低。
  2. 时间模型直接按规格一（#18 §4.4 / #23 §5.4）的定义转写，不复用 RoutePlanner 的实现。

**A→B 取送时间模型（规格一 #23 §5.4）**：
  - 取货停靠（A）无窗口、不记 penalty、零服务时间；
  - 只在送达点 B 做早到等待与 penalty；等待 = max(0, max{ws_i} - raw)（一个停靠只有一个到达时刻）；
  - 停靠零耗时（departure == arrival）；
  - penalty 逐单累加：每张送达单按自己的 we 计，arrival == windowEnd **不算迟到**（严格 >）；
  - **oracle 不实现选路**——节点序列由被测方给出，oracle 只对给定序列复算算术。

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


def _normalize_stops(stops, windows):
    """把简写（str 节点或 {'node': ...}）统一成 dict；str 视为纯送达停靠。"""
    result = []
    for stop in stops:
        if isinstance(stop, str):
            if windows is None or stop not in windows:
                raise ValueError("纯字符串停靠 %s 缺少窗口（windows 参数）" % stop)
            result.append({"node": stop, "pickups": [],
                           "deliveries": [(stop, windows[stop][0], windows[stop][1])]})
            continue
        node = stop["node"]
        pickups = list(stop.get("pickups", []))
        deliveries = [(d[0], d[1], d[2]) for d in stop.get("deliveries", [])]
        result.append({"node": node, "pickups": pickups, "deliveries": deliveries})
    return result


def route_metrics(g, start_pos, return_to, start_time, stops, kind, windows=None):
    """A→B 取送时间模型（规格一 #18 §4.4 / #23 §5.4）。

    stops: 依次停靠的节点描述，每项可为
      - dict: {"node": 节点,
               "pickups": [订单 id, ...]                      # A：无窗口、不记 penalty、零耗时
               "deliveries": [(订单 id, 窗口起, 窗口止), ...]}  # B：早到等待 + 逐单 penalty
      - str : 纯送达停靠的简写，窗口由 windows 提供。
    windows: {节点: (窗口起, 窗口止)}，仅供 str 简写使用。
    返回 dict: nodes / stops / total_distance / total_time / total_cost / total_penalty。
    距离 / 成本始终按实际边权累加；时间推进始终用耗时维度（与所选策略无关）。
    """
    normalized = _normalize_stops(stops, windows)
    seq = [start_pos]
    cur = start_pos
    elapsed = float(start_time)
    total_dist = 0.0
    total_cost = 0.0
    records = []
    total_penalty = 0

    def travel(to):
        nonlocal cur, elapsed, total_dist, total_cost
        path, _w = shortest(g, cur, to, kind)
        if path is None:
            return False
        for i in range(1, len(path)):
            edge = g.edges[(path[i - 1], path[i])]
            total_dist += edge["dist"]
            total_cost += edge["cost"]
            elapsed += edge["time"]
            seq.append(path[i])
        cur = to
        return True

    for stop in normalized:
        if not travel(stop["node"]):
            return None

        raw = elapsed
        # 只在 B 做早到等待：一个停靠只有一个到达时刻，等到**全部**送达单窗口都开启
        if stop["deliveries"]:
            wait = max(0.0, max(ws for _id, ws, _we in stop["deliveries"]) - raw)
        else:
            wait = 0.0
        arrival = raw + wait

        deliveries = []
        stop_penalty = 0
        stop_late = False
        for oid, ws, we in stop["deliveries"]:
            late = arrival > we + EPS          # arrival == windowEnd 不算迟到（严格 >）
            penalty = round(arrival - we) if late else 0
            stop_penalty += penalty
            stop_late = stop_late or late
            deliveries.append({"id": oid, "arrival": round(arrival), "late": late,
                              "penalty": penalty})
        total_penalty += stop_penalty
        elapsed = arrival                       # 停靠零耗时：departure == arrival
        records.append({
            "node": stop["node"], "raw": round(raw), "wait": round(wait),
            "arrival": round(arrival), "departure": round(elapsed),
            "late": stop_late, "penalty": stop_penalty,
            "pickups": list(stop["pickups"]), "deliveries": deliveries,
        })

    if not travel(return_to):
        return None

    return {
        "nodes": seq,
        "stops": records,
        "total_distance": round(total_dist, 6),
        "total_time": round(elapsed - start_time, 6),
        "total_cost": round(total_cost, 6),
        "total_penalty": total_penalty,
    }


def stops_from_route_nodes(route_nodes, orders):
    """由「完整节点序列 + 订单表」推出每站的取货/送达动作（oracle 不实现选路）。

    route_nodes: 被测方给出的完整节点序列（含首尾仓库）。
    orders: [{"id": .., "pickup": A, "delivery": B, "ws": .., "we": ..}, ...]
    规则：送达先于取货（合并停靠内先卸后装）；同一节点既是某单取货点又是另一单
    送达点时并成一次停靠；重访节点只承接尚未完成的动作——
    某次访问只能送达**已经取到**的货，否则该动作留给后续访问（规格一 #18 Q13 的
    「先取后送强制重访」：D01 第一次是取货，第二次才是送达）。
    """
    by_pickup, by_delivery = {}, {}
    for o in orders:
        by_pickup.setdefault(o["pickup"], []).append(o)
        by_delivery.setdefault(o["delivery"], []).append(o)

    picked, delivered = set(), set()
    stops = []
    for node in route_nodes:
        pickups, deliveries = [], []
        for o in by_delivery.get(node, []):
            if o["id"] in delivered:
                continue
            if o["id"] not in picked:
                continue   # 还没取到：这次访问不送达它（留给后续重访）
            deliveries.append((o["id"], o["ws"], o["we"]))
            delivered.add(o["id"])
        for o in by_pickup.get(node, []):
            if o["id"] in picked or o["id"] in delivered:
                continue
            pickups.append(o["id"])
            picked.add(o["id"])
        if pickups or deliveries:
            stops.append({"node": node, "pickups": pickups, "deliveries": deliveries})

    unserved = [o["id"] for o in orders if o["id"] not in delivered]
    if unserved:
        raise ValueError("节点序列未覆盖全部订单: %s" % unserved)
    return stops


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


def fixture_pdp():
    """取送链：W - A - B，B 回 W 的直达边故意更长（避免与绕行并列）。"""
    g = Graph()
    g.two_way("W", "A", 1.0, 10.0, 4.0)
    g.two_way("A", "B", 1.0, 10.0, 4.0)
    g.two_way("W", "B", 5.0, 50.0, 20.0)
    return g


def fixture_merged():
    """合并停靠：M 同时是 X1 的送达点与 X2 的取货点。"""
    g = Graph()
    g.two_way("W", "M", 1.0, 10.0, 4.0)
    g.two_way("M", "N", 1.0, 10.0, 4.0)
    g.two_way("W", "N", 5.0, 50.0, 20.0)
    return g


def show(title, result):
    print(f"\n=== {title} ===")
    if result is None:
        print("  <不可达>")
        return
    print(f"  nodes        = {result['nodes']}")
    for r in result["stops"]:
        picks = ",".join(r["pickups"]) or "-"
        dels = " ".join("%s(arr %d%s p%d)" % (d["id"], d["arrival"],
                                              "+" if d["late"] else "",
                                              d["penalty"]) for d in r["deliveries"]) or "-"
        print(f"  stop {r['node']}: raw={r['raw']} wait={r['wait']} "
              f"arrival={r['arrival']} departure={r['departure']} "
              f"late={r['late']} penalty={r['penalty']} 取[{picks}] 送[{dels}]")
    print(f"  totalDistance = {result['total_distance']}")
    print(f"  totalTime     = {result['total_time']}")
    print(f"  totalCost     = {result['total_cost']}")
    print(f"  totalPenalty  = {result['total_penalty']}")


def _clock_minutes(text):
    hh, mm = text.split(":")
    return int(hh) * 60 + int(mm)


def _read_ini_sections(path):
    """按节读 INI（手写：default.ini 的数据行本身就是逗号分隔的裸行，
    configparser 会把它们当成语法错误）。"""
    sections = {}
    current = None
    with open(path, encoding="utf-8") as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith("#") or line.startswith(";"):
                continue
            if line.startswith("[") and line.endswith("]"):
                current = line[1:-1].strip()
                sections[current] = ""
                continue
            if current is not None:
                sections[current] += line + "\n"
    return sections


def _cli_route(argv):
    """oracle 的命令行入口（oracle 算术对拍）：

        python3 tools/oracle.py --route <kind> <start> <return> <startTime> <节点序列...>

    节点序列由 app 的 --plan-summary 第 4 行给出；订单表与边表从 config/default.ini 读。
    oracle 只对**给定序列**复算算术，不实现选路。
    """
    import os
    kind = argv[0]
    start = argv[1]
    ret = argv[2]
    start_time = int(argv[3])
    route_nodes = argv[4:]
    ini = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                       "config", "default.ini")
    sections = _read_ini_sections(ini)
    orders = []
    for raw in sections.get("orders", "").splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        parts = [x.strip() for x in line.split(",")]
        if len(parts) < 7:
            continue
        orders.append({"id": parts[0], "pickup": parts[1], "delivery": parts[2],
                       "kg": float(parts[3]), "ws": _clock_minutes(parts[4]),
                       "we": _clock_minutes(parts[5])})
    g = Graph()
    for raw in sections.get("edges", "").splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        parts = [x.strip() for x in line.split(",")]
        if len(parts) < 5:
            continue
        g.add_edge(parts[0], parts[1], float(parts[2]), float(parts[3]), float(parts[4]))
    stops = stops_from_route_nodes(route_nodes, orders)
    res = route_metrics(g, start, ret, start_time, stops, kind)
    if res is None:
        print("不可达")
        return 1
    print("total_distance = %.3f" % res["total_distance"])
    print("total_time     = %.3f" % res["total_time"])
    print("total_cost     = %.3f" % res["total_cost"])
    print("total_penalty  = %d" % res["total_penalty"])
    print("stops          = %d" % len(res["stops"]))
    actions = sum(len(r["pickups"]) + len(r["deliveries"]) for r in res["stops"])
    print("actions        = %d" % actions)
    return 0


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

    # ---- 时间模型：等待 + 停靠零耗时（旧服务时间已退役）----
    # W->D1 10min 到达 490，窗口 540 开 -> 等待 50，arrival 540，departure == 540
    g2 = fixture_w_d1()
    r = route_metrics(g2, "W", "W", 480, ["D1"], "distance", windows={"D1": (540, 1080)})
    s = r["stops"][0]
    if s["raw"] != 490 or s["wait"] != 50 or s["arrival"] != 540:
        print(f"[FAIL] 自检时间模型: {s}")
        ok = False
    if s["departure"] != s["arrival"]:
        print(f"[FAIL] 自检停靠零耗时: departure={s['departure']} arrival={s['arrival']}")
        ok = False
    if abs(r["total_time"] - 70.0) > EPS:
        print(f"[FAIL] 自检总耗时: {r['total_time']}（应为 70，服务时间已退役）")
        ok = False

    # ---- 边界：arrival == windowEnd 不算迟到（严格 >）；略晚 1 分钟记 1 ----
    r = route_metrics(g2, "W", "W", 480, ["D1"], "distance", windows={"D1": (480, 490)})
    if r["stops"][0]["late"] or r["stops"][0]["penalty"] != 0 or r["total_penalty"] != 0:
        print(f"[FAIL] 自检 arrival==windowEnd 不算迟到: {r['stops'][0]}")
        ok = False
    r = route_metrics(g2, "W", "W", 480, ["D1"], "distance", windows={"D1": (480, 489)})
    if not r["stops"][0]["late"] or r["total_penalty"] != 1:
        print(f"[FAIL] 自检略晚 1 分钟记 penalty 1: {r['stops'][0]}")
        ok = False

    # ---- A→B 取送：取货停靠无窗口、不等待、不记 penalty ----
    r = route_metrics(fixture_pdp(), "W", "W", 480,
                      [{"node": "A", "pickups": ["X1"], "deliveries": []},
                       {"node": "B", "pickups": [], "deliveries": [("X1", 0, 1440)]}],
                      "distance")
    a, b = r["stops"][0], r["stops"][1]
    if a["wait"] != 0 or a["penalty"] != 0 or a["arrival"] != 490:
        print(f"[FAIL] 自检取货停靠无窗口: {a}")
        ok = False
    if a["departure"] != a["arrival"]:
        print(f"[FAIL] 自检取货零耗时: {a}")
        ok = False
    if b["arrival"] != 500 or b["penalty"] != 0:
        print(f"[FAIL] 自检送达停靠: {b}")
        ok = False
    if abs(r["total_time"] - 40.0) > EPS:
        print(f"[FAIL] 自检取送总耗时 40: {r['total_time']}")
        ok = False
    if abs(r["total_distance"] - 4.0) > EPS or abs(r["total_cost"] - 16.0) > EPS:
        print(f"[FAIL] 自检取送距离/成本: {r}")
        ok = False

    # ---- 合并停靠：M 先送 X1 再取 X2，等待只由送达单决定 ----
    r = route_metrics(fixture_merged(), "W", "W", 480,
                      [{"node": "M", "pickups": ["X2"], "deliveries": [("X1", 480, 1440)]},
                       {"node": "N", "pickups": [], "deliveries": [("X2", 0, 1440)]}],
                      "distance")
    m = r["stops"][0]
    if m["wait"] != 0 or m["arrival"] != 490 or m["departure"] != 490:
        print(f"[FAIL] 自检合并停靠零耗时: {m}")
        ok = False
    if r["stops"][1]["arrival"] != 500:
        print(f"[FAIL] 自检合并停靠后续: {r['stops'][1]}")
        ok = False
    if abs(r["total_time"] - 40.0) > EPS:
        print(f"[FAIL] 自检合并停靠总耗时 40: {r['total_time']}")
        ok = False

    # ---- 逐单 penalty：同一停靠两张单各按自己的 we 记 ----
    r = route_metrics(g2, "W", "W", 480,
                      [{"node": "D1", "pickups": [],
                        "deliveries": [("P1", 480, 485), ("P2", 480, 600)]}],
                      "distance")
    d = r["stops"][0]
    if d["arrival"] != 490 or d["wait"] != 0:
        print(f"[FAIL] 自检同停靠双单到达: {d}")
        ok = False
    if d["deliveries"][0]["penalty"] != 5 or not d["deliveries"][0]["late"]:
        print(f"[FAIL] 自检逐单 penalty（紧窗单）: {d['deliveries'][0]}")
        ok = False
    if d["deliveries"][1]["penalty"] != 0 or d["deliveries"][1]["late"]:
        print(f"[FAIL] 自检逐单 penalty（宽窗单）: {d['deliveries'][1]}")
        ok = False
    if r["total_penalty"] != 5:
        print(f"[FAIL] 自检逐单 penalty 合计: {r['total_penalty']}")
        ok = False
    # 等待取所有送达单窗口起的最大值：一张单 480 已开、另一张 540 才开 -> 等 50
    r = route_metrics(g2, "W", "W", 480,
                      [{"node": "D1", "pickups": [],
                        "deliveries": [("P1", 480, 600), ("P2", 540, 600)]}],
                      "distance")
    if r["stops"][0]["wait"] != 50 or r["stops"][0]["arrival"] != 540:
        print(f"[FAIL] 自检等待取全部窗口开启: {r['stops'][0]}")
        ok = False

    # ---- stops_from_route_nodes：由节点序列 + 订单表推出动作（合并停靠）----
    orders = [
        {"id": "X1", "pickup": "W", "delivery": "M", "ws": 480, "we": 1080},
        {"id": "X2", "pickup": "M", "delivery": "N", "ws": 0, "we": 1440},
    ]
    stops = stops_from_route_nodes(["W", "M", "N", "W"], orders)
    if len(stops) != 3 or [s["node"] for s in stops] != ["W", "M", "N"]:
        print(f"[FAIL] 自检序列推动作: {stops}")
        ok = False
    else:
        if stops[1]["pickups"] != ["X2"] or [d[0] for d in stops[1]["deliveries"]] != ["X1"]:
            print(f"[FAIL] 自检合并停靠动作集（先卸后装）: {stops[1]}")
            ok = False
        r = route_metrics(fixture_merged(), "W", "W", 480, stops, "distance")
        if abs(r["total_time"] - 40.0) > EPS or r["total_penalty"] != 0:
            print(f"[FAIL] 自检序列推动作复算: {r}")
            ok = False
    # 序列不合法（送达先于取货）必须报错，而不是给出"看起来合理"的数
    try:
        stops_from_route_nodes(["W", "N", "W"], orders)
        print("[FAIL] 自检非法序列应报错")
        ok = False
    except ValueError:
        pass
    # 序列未覆盖全部订单必须报错
    try:
        stops_from_route_nodes(["W", "M", "W"], orders)
        print("[FAIL] 自检未覆盖订单应报错")
        ok = False
    except ValueError:
        pass

    # 贪心夹具：W->D2 的最短路是绕经 D1 的 3.0，而非直达 5.0
    path, w = shortest(fixture_greedy(), "W", "D2", "distance")
    if path != ["W", "D1", "D2"] or abs(w - 3.0) > EPS:
        print(f"[FAIL] 自检 W->D2 距离: {path} {w}")
        ok = False

    # 并列夹具：W->DA 与 W->DZ 等距
    _, wa = shortest(fixture_tie(), "W", "DA", "distance")
    _, wz = shortest(fixture_tie(), "W", "DZ", "distance")
    if abs(wa - wz) > EPS:
        print(f"[FAIL] 自检并列夹具等距: {wa} vs {wz}")
        ok = False

    # 路况改动条数口径
    for e_count, ratio, expect in ((6, 0.5, 3), (6, 0.1, 1), (6, 1.0, 6), (72, 0.1, 7)):
        got = traffic_count(e_count, ratio)
        if got != expect:
            print(f"[FAIL] 自检 traffic_count({e_count},{ratio}) = {got}，应为 {expect}")
            ok = False

    # 全部边 +50% 后的重规划：总耗时 75（服务时间退役后在旧 80 上减 5）
    r = route_metrics(fixture_w_d1_congested(0.5), "W", "W", 480, ["D1"], "distance",
                      windows={"D1": (540, 1080)})
    if abs(r["total_time"] - 75.0) > EPS or r["stops"][0]["raw"] != 495:
        print(f"[FAIL] 自检拥堵后重规划: {r}")
        ok = False

    # 紧急单场景：先 D2 后 D1，总耗时 12.0（旧值 22 = 12 + 2×5 服务时间，已退役）
    r = route_metrics(fixture_greedy(), "W", "W", 480, ["D2", "D1"], "distance",
                      windows={"D1": (0, 1440), "D2": (0, 1440)})
    if abs(r["total_time"] - 12.0) > EPS or r["stops"][0]["node"] != "D2":
        print(f"[FAIL] 自检紧急优先: {r}")
        ok = False

    # 停靠级转储对拍（用真实 app 二进制；找不到 app 时跳过）
    if not _dump_selfcheck():
        ok = False

    if ok:
        print("[oracle] 自检通过")
    return 0 if ok else 1


def traffic_count(edge_count, ratio):
    """路况模拟改动边数的口径：round(ratio*E)；ratio>0 且算得 0 时至少改 1 条。"""
    if edge_count <= 0 or ratio <= 0:
        return 0
    n = int(ratio * edge_count + 0.5)
    return max(1, min(n, edge_count))


def fixture_w_d1_congested(increase):
    """W<->D1，两条边的耗时统一乘 (1+increase)。"""
    g = Graph()
    g.two_way("W", "D1", 5.0, round(10.0 * (1.0 + increase), 6), 4.0)
    return g



# ---------------------------------------------------------------- 停靠级转储对拍
# 背景：--plan-summary 第 4 行是**节点序列（含途经节点）**。含重访时该序列无法唯一
# 还原逐单动作归属（同一节点会被合并/重访），因此 oracle **不猜动作**。
# app 的 --dump-plan 直接吐**停靠级真源**（RoutePlan::stops）：每停靠一行，含
# phase / 动作集 / 逐单 penalty / 离站在车量。oracle 只对给定停靠序列独立复算算术。
#
# 转储格式（--dump-plan <kind>）：
#   PLAN <startNode> <returnNode> <departMin>
#   STOP <idx> <nodeId> <phase:U|N> <rawMin> <waitMin> <arrMin> \
#        <pickups:id,id..> <deliveries:id:ws:we:penalty,..> <remainingLoadKg>
# 空动作集写作 "-"。


def _parse_dump_text(text):
    """把 --dump-plan 的文本解析成车队结构（每台车一段）。

      PLAN <vehicleId> <start> <return> <depart>
      STOP <idx> <nodeId> <phase:U|N> <rawMin> <waitMin> <arrMin> \
           <pickups:id,id..> <deliveries:id:ws:we:penalty,..> <remainingLoadKg>
      METRICS <vehicleId> <17 个数>     # app 自报：逐车 13 字段（两率用命中/分母）
      FLEET <17 个数>                  # app 自报：车队 13 字段（总耗时 = 完工耗时）
    空动作集写作 "-"；空车只有 PLAN + METRICS（没有 STOP）。
    """
    fleet = {"plans": [], "fleet_metrics": None, "legs": []}
    cur = None
    for raw in text.splitlines():
        line = raw.strip()
        if not line:
            continue
        tok = line.split()
        if tok[0] == "PLAN":
            cur = {"id": tok[1], "start": tok[2], "ret": tok[3],
                   "depart": int(tok[4]), "stops": [], "metrics": None,
                   "infeasible": None}
            fleet["plans"].append(cur)
        elif tok[0] == "STOP" and cur is not None:
            # STOP idx node phase raw wait arr pickups deliveries remaining
            pickups = [] if tok[7] == "-" else tok[7].split(",")
            deliveries = []
            if tok[8] != "-":
                for spec in tok[8].split(","):
                    oid, ws, we, pen = spec.split(":")
                    deliveries.append({"id": oid, "ws": int(ws), "we": int(we),
                                       "penalty": int(pen)})
            cur["stops"].append({
                "idx": int(tok[1]), "node": tok[2], "phase": tok[3],
                "raw": int(tok[4]), "wait": int(tok[5]), "arrival": int(tok[6]),
                "pickups": pickups, "deliveries": deliveries,
                "remaining": float(tok[9]),
            })
        elif tok[0] == "XACT" and cur is not None and len(tok) >= 4:
            # XACT <停靠下标> <U|P> <订单>：站内易手的**精确落点**（#57）。
            # 之前只能按 (车, 站) 猜，同一站多次经过时会把取走认到错的停靠上。
            idx = int(tok[1])
            cur["has_xact"] = True
            if 0 <= idx < len(cur["stops"]):
                st = cur["stops"][idx]
                st.setdefault("relay_pickups", [])
                st.setdefault("relay_unloads", [])
                if tok[2] == "P":
                    st["relay_pickups"].append(tok[3])
                else:
                    st["relay_unloads"].append(tok[3])
        elif tok[0] == "METRICS" and cur is not None:
            cur["metrics"] = [float(x) for x in tok[2:]]
        elif tok[0] == "INFEASIBLE" and cur is not None:
            cur["infeasible"] = " ".join(tok[2:])
        elif tok[0] == "FLEET":
            fleet["fleet_metrics"] = [float(x) for x in tok[1:]]
        elif tok[0] == "LEG" and len(tok) >= 8:
            # LEG <腿序> <from> <to> <订单> <kind> <HH:MM readyTime> <执行车>
            # 只有 #48 的三段接力会让「含转运取走的停靠」真的需要 readyTime；
            # 别的行这里一律不碰（只加行、不改旧行的约定）。
            fleet["legs"].append({
                "seq": int(tok[1]), "from": tok[2], "to": tok[3], "order": tok[4],
                "kind": tok[5], "ready": _clock_minutes(tok[6]), "vehicle": tok[7],
            })
    return fleet


def _relay_ready_tables(legs):
    """从 LEG 行建两张表（**只读 app 的腿结构，不回读 STOP 的 wait/arrival**）。

    ready[(订单, 节点)] = 该腿的可接时刻 —— 用于复算「车等货」的等待；
    pickups[(车, 节点)] = 该车在该节点站内取走的订单清单 —— 用于认出哪些停靠是
    接力取货停靠（STOP 行只打真实取货/送达，转运动作不在里面）。

    只取 station->station / station->customer 两类：L1（customer->station）的 ready
    是执行车发车时刻，不是「货未到」的等待来源；customer->customer 是本地单，恒无约束。
    """
    ready = {}
    pickups = {}
    for leg in legs:
        if leg["kind"] not in ("station->station", "station->customer"):
            continue
        ready[(leg["order"], leg["from"])] = leg["ready"]
        pickups.setdefault((leg["vehicle"], leg["from"]), []).append(leg["order"])
    return ready, pickups


def _parse_dump_file(path):
    with open(path, encoding="utf-8") as f:
        return _parse_dump_text(f.read())


def aggregate_fleet(results):
    """把逐车复算结果聚合成车队 13 字段（与 C++ planFleet 同口径）。

    距离/成本/penalty/停靠/动作 = Σ；总耗时（主口径 = 完工耗时）=
    max(各车返回时刻) − min(各车发车时刻)，返回时刻按 app 的**整数到达**取整；
    平均在车 / 利用率按**车时**时间加权；峰值取 max；两率 = Σ命中 / Σ分母。
    """
    if not results:
        return None
    total_dist = sum(m["distance"] for m in results)
    total_cost = sum(m["cost"] for m in results)
    total_penalty = sum(m["penalty"] for m in results)
    stops = sum(m["stops"] for m in results)
    actions = sum(m["actions"] for m in results)
    depart_min = min(m["depart"] for m in results)
    ret_max = max(m["return_arrival"] for m in results)
    weighted_time = sum(m["time"] for m in results if m["time"] > EPS)
    integral = sum(m["avg_load"] * m["time"] for m in results if m["time"] > EPS)
    cap_time = 0.0
    for m in results:
        if m["time"] > EPS:
            cap_time += m["capacity"] * m["time"]
    pickup_hit = sum(m["pickup_hit"] for m in results)
    pickup_total = sum(m["pickup_total"] for m in results)
    deliv_hit = sum(m["deliv_hit"] for m in results)
    deliv_total = sum(m["deliv_total"] for m in results)
    urg_hit = sum(m["urgent_hit"] for m in results)
    urg_total = sum(m["urgent_total"] for m in results)
    norm_hit = sum(m["normal_hit"] for m in results)
    norm_total = sum(m["normal_total"] for m in results)

    def rate(hit, total):
        return None if total <= 0 else 100.0 * hit / total

    return {
        "distance": total_dist,
        "time": float(ret_max - depart_min),
        "cost": total_cost,
        "penalty": total_penalty,
        "stops": stops,
        "actions": actions,
        "pickup_rate": rate(pickup_hit, pickup_total),
        "delivery_rate": rate(deliv_hit, deliv_total),
        "urgent_rate": rate(urg_hit, urg_total),
        "normal_rate": rate(norm_hit, norm_total),
        "avg_load": (integral / weighted_time) if weighted_time > EPS else 0.0,
        "utilization": (integral / cap_time * 100.0) if cap_time > EPS else 0.0,
        "peak": max(m["peak"] for m in results),
        "pickup_hit": pickup_hit, "pickup_total": pickup_total,
        "deliv_hit": deliv_hit, "deliv_total": deliv_total,
        "urgent_hit": urg_hit, "urgent_total": urg_total,
        "normal_hit": norm_hit, "normal_total": norm_total,
        "completion": float(ret_max - depart_min),
        "sum_time": sum(m["time"] for m in results),
    }


def recompute_plan(g, plan, kind, time_factor=1.0, cost_factor=1.0, capacity=0.0,
                   urgent_ids=None, ready=None, relay_pickups=None, order_a=None):
    """从**停靠级转储**独立复算 13 字段（不实现选路，只按给定停靠序列算算术）。

    - 相邻停靠之间用 default.ini 边表的最短路（按 kind 维度），距离/耗时/成本从
      **同一条最短路**上读；耗时乘 time_factor、成本乘 cost_factor、距离不乘。
    - 逐单 penalty：arrival > we 才迟到（严格 >，arrival == we 不算）；早到等待只在
      有送达单的停靠发生，等全部送达单窗口起的最大值；停靠零耗时。
    - **车等货**（规格三 §3 / #48）：含站内取走的停靠还要等到该 (订单, 节点) 的
      readyTime（来自 LEG 行）。wait = max(窗口等待, readyTime − raw) —— 这是**独立复算**：
      readyTime 读的是腿的可接时刻、raw 由边表最短路算出，不回读 STOP 行里的 wait/arrival。
    - 平均在车用**分段积分**（首末空驶段计入分母）：段内在车量 = 上一停靠离站在车量。
    - 峰值 = max(各停靠离站在车量)（停靠内「先卸后装」，卸后 <= 卸前，故峰值必在装后）。
    """
    if urgent_ids is None:
        urgent_ids = set()
    elapsed = float(plan["depart"])
    pos = plan["start"]
    total_dist = 0.0
    total_cost = 0.0
    penalty = 0
    integral = 0.0
    cur_load = 0.0          # 上一停靠离站在车量（首段空驶 = 0）
    peak = 0.0
    order_pickup = {}       # oid -> (arrival, pickupNode)
    order_deliv = {}        # oid -> (arrival, we, late, penalty, deliveryNode)
    if ready is None:
        ready = {}
    if relay_pickups is None:
        relay_pickups = {}
    pending_relay = {k: list(v) for k, v in relay_pickups.items()}

    def travel(to):
        nonlocal pos, elapsed, total_dist, total_cost, integral
        path, _w = shortest(g, pos, to, kind)
        if path is None:
            return None
        leg_t = 0.0
        for i in range(1, len(path)):
            e = g.edges[(path[i - 1], path[i])]
            total_dist += e["dist"]
            total_cost += e["cost"]
            leg_t += e["time"]
        leg_t *= time_factor
        integral += cur_load * leg_t     # 该段在车量恒定（停靠零耗时）
        elapsed += leg_t
        pos = to
        return True

    for st in plan["stops"]:
        if pos != st["node"]:
            if not travel(st["node"]):
                return None
        raw = elapsed
        if st["deliveries"]:
            max_ws = max(d["ws"] for d in st["deliveries"])
            wait = max(0.0, max_ws - raw)
        else:
            wait = 0.0
        # 车等货：该车在这个节点的站内取走，要等货到站（readyTime 由 LEG 行给出）。
        # 一个停靠可以同时取走**多张单**（同站合并停靠），所以这里把该 (车, 站) 的待取
        # 全部认领：等待取它们的 readyTime 最大值（与 C++ replay 的口径一致）。
        stop_relay = list(st.get("relay_pickups") or [])
        if not stop_relay and not plan.get("has_xact"):
            # 兜底（**旧 dump** 没有 XACT 行）：按 (车, 站) 猜第一个待取的。
            # 新 dump 一律走 XACT 的精确落点 —— (车, 站) 猜法在同一站多次经过时会认错停靠。
            key_relay = (plan["id"], st["node"])
            if key_relay in pending_relay and pending_relay[key_relay]:
                oid = pending_relay[key_relay].pop(0)
                stop_relay.append(oid)
        for oid in stop_relay:
            rdy = ready.get((oid, st["node"]))
            if rdy is not None and rdy - raw > wait:
                wait = float(rdy) - raw
        arrival = raw + wait
        elapsed = arrival
        for d in st["deliveries"]:
            late = arrival > d["we"] + EPS
            p = round(arrival - d["we"]) if late else 0
            penalty += p
            order_deliv[d["id"]] = (arrival, d["we"], late, p, st["node"])
        for oid in st["pickups"]:
            order_pickup[oid] = (arrival, st["node"])
        for oid in stop_relay:
            order_pickup[oid] = (arrival, st["node"])
        cur_load = st["remaining"]
        if cur_load > peak:
            peak = cur_load

    if pos != plan["ret"]:
        if not travel(plan["ret"]):
            return None

    # 成本乘该车的 cost_factor（规格二 §3.3：成本缩放、距离不变）。
    # 旧单车数据两因子都是 1.00，所以这行在规格一阶段是恒等变换。
    total_cost *= cost_factor
    total_time = elapsed - float(plan["depart"])

    pickup_total = pickup_ok = 0
    deliv_total = deliv_ok = 0
    urg_total = urg_ok = 0
    norm_total = norm_ok = 0
    # 订单表：A（取货点）按订单 ID 查（#57）。取货准时的判据是
    #   t_pick + shortestTime_vehicle(A -> B) <= we
    # A 是**订单的取货点**，不是「取货停靠所在的节点」—— 接力单的取货发生在中转站，
    # 拿站当 A 会把跨簇单的 A->B 算短（实测把 2 张单错算成准时）。
    if order_a is None:
        order_a = {}
    for oid, (parr, pnode) in order_pickup.items():
        rec = order_deliv.get(oid)
        if rec is None:
            # 只统计**本车真的送达了**的订单（与 C++ finalize 同口径）：转运中间那台车
            # 取了货却不送达，它的两率分母不能把这张单算进去，否则同一张单分母翻倍。
            continue
        pickup_total += 1
        _darr, we, late, _p, dnode = rec
        _path, stime = shortest(g, order_a.get(oid, pnode), dnode, "time")
        scaled = (0.0 if stime == INF else stime * time_factor)
        if parr + scaled <= we + EPS:
            pickup_ok += 1
        deliv_total += 1
        if not late:
            deliv_ok += 1
        if oid in urgent_ids:
            urg_total += 1
            if not late:
                urg_ok += 1
        else:
            norm_total += 1
            if not late:
                norm_ok += 1

    def rate(hit, total):
        return None if total <= 0 else 100.0 * hit / total

    return {
        "distance": total_dist,
        "time": total_time,
        "cost": total_cost,
        "penalty": penalty,
        "stops": len(plan["stops"]),
        "actions": sum(len(st["pickups"]) + len(st["deliveries"]) for st in plan["stops"]),
        "pickup_rate": rate(pickup_ok, pickup_total),
        "delivery_rate": rate(deliv_ok, deliv_total),
        "urgent_rate": rate(urg_ok, urg_total),
        "normal_rate": rate(norm_ok, norm_total),
        "avg_load": (integral / total_time) if total_time > EPS else 0.0,
        "utilization": ((integral / total_time) / capacity * 100.0)
                       if (total_time > EPS and capacity > EPS) else 0.0,
        "peak": peak,
        # 逐车/车队比对用的计数（两率的分母 0 只能靠计数表达，不能靠「—」）；
        # return_arrival / depart / capacity 供车队汇总（完工耗时、利用率加权）。
        "pickup_hit": pickup_ok, "pickup_total": pickup_total,
        "deliv_hit": deliv_ok, "deliv_total": deliv_total,
        "urgent_hit": urg_ok, "urgent_total": urg_total,
        "normal_hit": norm_ok, "normal_total": norm_total,
        "return_arrival": elapsed,
        "depart": float(plan["depart"]),
        "capacity": capacity,
    }


def _load_dataset(path=None):
    """从仓库 config/default.ini 读 orders / edges / vehicles（独立于被测代码）。

    返回 (graph, orders, vehicles[])：vehicles 按 [vehicles] 行序，供逐车对拍。
    path 非空时读那份配置（夹具对拍用；--plan 的 --config 参数传进来）。
    """
    import os
    ini = path if path else os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config", "default.ini")
    sections = _read_ini_sections(ini)
    orders = []
    for raw in sections.get("orders", "").splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        parts = [x.strip() for x in line.split(",")]
        if len(parts) < 7:
            continue
        orders.append({"id": parts[0], "pickup": parts[1], "delivery": parts[2],
                       "kg": float(parts[3]), "ws": _clock_minutes(parts[4]),
                       "we": _clock_minutes(parts[5]), "urgent": int(parts[6]) != 0})
    g = Graph()
    for raw in sections.get("edges", "").splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        parts = [x.strip() for x in line.split(",")]
        if len(parts) < 5:
            continue
        # [edges] 每行就是**一条有向边**（双向路发两条、单向路只发一条），
        # 不能 here 用 two_way——那会给 6 条单向路凭空补上反向边。
        g.add_edge(parts[0], parts[1], float(parts[2]), float(parts[3]), float(parts[4]))
    vehicles = []
    for raw in sections.get("vehicles", "").splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        parts = [x.strip() for x in line.split(",")]
        if len(parts) < 6:
            continue
        vehicles.append({"id": parts[0], "start": parts[1], "capacity": float(parts[2]),
                         "depart": _clock_minutes(parts[3]), "time_factor": float(parts[4]),
                         "cost_factor": float(parts[5])})
    return g, orders, vehicles


def _print_plan_metrics(m):
    def rat(v):
        return "—" if v is None else "%.1f" % v
    print("distance      = %.3f" % m["distance"])
    print("time          = %.3f" % m["time"])
    print("cost          = %.3f" % m["cost"])
    print("penalty       = %d" % m["penalty"])
    print("stops         = %d" % m["stops"])
    print("actions       = %d" % m["actions"])
    print("pickup_rate   = %s" % rat(m["pickup_rate"]))
    print("delivery_rate = %s" % rat(m["delivery_rate"]))
    print("urgent_rate   = %s" % rat(m["urgent_rate"]))
    print("normal_rate   = %s" % rat(m["normal_rate"]))
    print("avg_load      = %.1f" % m["avg_load"])
    print("utilization   = %.1f" % m["utilization"])
    print("peak          = %.1f" % m["peak"])


_METRIC_FIELD_ORDER = ["distance", "time", "cost", "penalty", "stops", "actions",
                       "pickup_hit", "pickup_total", "deliv_hit", "deliv_total",
                       "urgent_hit", "urgent_total", "normal_hit", "normal_total",
                       "avg_load", "utilization", "peak"]


def _rat(v):
    return "—" if v is None else "%.1f" % v


def _compare_fields(values, m, tag):
    """METRICS / FLEET 的 17 个数与复算值逐项比对（容差 §6.4）。"""
    ok = True
    for idx, key in enumerate(_METRIC_FIELD_ORDER):
        if idx >= len(values):
            print("[FAIL] %s: 缺字段 %s" % (tag, key))
            return False
        a = float(values[idx])
        b = m[key]
        if key in ("penalty", "stops", "actions", "pickup_hit", "pickup_total",
                   "deliv_hit", "deliv_total", "urgent_hit", "urgent_total",
                   "normal_hit", "normal_total"):
            good = int(round(a)) == int(round(b))
        elif key in ("distance", "time", "cost", "avg_load"):
            good = abs(a - b) <= 1e-3 + 1e-9
        else:
            good = abs(a - b) <= 0.1 + 1e-9
        if not good:
            print("[FAIL] %s: %s app=%.6f oracle=%.6f" % (tag, key, a, b))
            ok = False
    return ok


def _compare_vehicle(values, m, vehicle_id):
    return _compare_fields(values, m, "dump-%s" % vehicle_id)


def _compare_fleet(values, fleet):
    return _compare_fields(values, fleet, "dump-FLEET")


def _cli_plan(argv):
    """oracle 的停靠级对拍入口：

        python3 tools/oracle.py --plan <dumpfile> <kind>

    读 app 的 --dump-plan 输出（每车一段），**逐车**独立复算 13 字段再汇总；
    与 app 自报的 METRICS / FLEET 逐项比对，最后打印车队 13 字段
    （供 cli_plan_summary 与 --plan-summary 对拍）。oracle 不实现选路。
    """
    cfg_path = None
    if "--config" in argv:
        i = argv.index("--config")
        if i + 1 < len(argv):
            cfg_path = argv[i + 1]
        del argv[i:i + 2]
    if len(argv) < 2:
        print("用法: oracle.py --plan <dumpfile> <kind> [--config <ini>]")
        return 2
    dumpfile, kind = argv[0], argv[1]
    dump = _parse_dump_file(dumpfile)
    g, orders, vehicles = _load_dataset(cfg_path)
    ready, relay_pickups = _relay_ready_tables(dump.get("legs", []))
    urgent_ids = {o["id"] for o in orders if o["urgent"]}
    by_id = {v["id"]: v for v in vehicles}
    results = []
    ok = True
    for plan in dump["plans"]:
        if plan.get("infeasible"):
            print("[FAIL] %s: app 报不可行（%s）" % (plan["id"], plan["infeasible"]))
            ok = False
            continue
        veh = by_id.get(plan["id"])
        if veh is None:
            print("[FAIL] %s: 配置里没有这台车" % plan["id"])
            ok = False
            continue
        m = recompute_plan(g, plan, kind, time_factor=veh["time_factor"],
                           cost_factor=veh["cost_factor"], capacity=veh["capacity"],
                           urgent_ids=urgent_ids, ready=ready, relay_pickups=relay_pickups,
                           order_a={o["id"]: o["pickup"] for o in orders})
        if m is None:
            print("[FAIL] %s: dump 复算不可达" % plan["id"])
            ok = False
            continue
        results.append(m)
        if plan.get("metrics") is not None and not _compare_vehicle(plan["metrics"], m,
                                                                   plan["id"]):
            ok = False
        # 逐车指标行（#43 的 7 字段来源；本票先打出来供人工核对）
        print("%s.distance      = %.3f" % (plan["id"], m["distance"]))
        print("%s.time          = %.3f" % (plan["id"], m["time"]))
        print("%s.cost          = %.3f" % (plan["id"], m["cost"]))
        print("%s.penalty       = %d" % (plan["id"], m["penalty"]))
        print("%s.stops         = %d" % (plan["id"], m["stops"]))
        print("%s.actions       = %d" % (plan["id"], m["actions"]))
        print("%s.utilization   = %.1f" % (plan["id"], m["utilization"]))
        print("%s.pickup_rate   = %s" % (plan["id"], _rat(m["pickup_rate"])))
        print("%s.delivery_rate = %s" % (plan["id"], _rat(m["delivery_rate"])))
    fleet = aggregate_fleet(results)
    if fleet is None:
        print("[FAIL] 没有任何可复算的车辆段")
        return 1
    if dump.get("fleet_metrics") is not None and not _compare_fleet(dump["fleet_metrics"],
                                                                    fleet):
        ok = False
    _print_plan_metrics(fleet)
    print("completion    = %.3f" % fleet["completion"])
    print("sum_time      = %.3f" % fleet["sum_time"])
    return 0 if ok else 1


def _parse_summary(text):
    """解析 --plan-summary 的 stdout（四行），返回 13 字段（与 recompute_plan 同键名）。"""
    import re
    lines = [ln for ln in text.splitlines() if ln.strip()]
    m = {}
    l1 = lines[0]
    mm = re.search(r"总距离 ([\d.]+) km .* 总耗时 ([\d.]+) min .* 总成本 ([\d.]+) 元 .* "
                   r"总 penalty (\d+) min .* 停靠 (\d+) 站 .* 动作 (\d+) 次", l1)
    m["distance"] = float(mm.group(1))
    m["time"] = float(mm.group(2))
    m["cost"] = float(mm.group(3))
    m["penalty"] = int(mm.group(4))
    m["stops"] = int(mm.group(5))
    m["actions"] = int(mm.group(6))
    l2 = lines[1]
    # 取货准时率 <v>%（x/y） | 送达准时率 <v>%（x/y；紧急 <u> · 普通 <n>） | 平均在车 ...
    mp = re.search(r"取货准时率 ([\d.—]+)%", l2)
    md = re.search(r"送达准时率 ([\d.—]+)%", l2)
    mu = re.search(r"紧急 ([\d.—]+)", l2)
    mn = re.search(r"普通 ([\d.—]+)", l2)
    ml = re.search(r"平均在车 ([\d.]+) kg", l2)
    mut = re.search(r"利用率 ([\d.]+)%", l2)
    mpk = re.search(r"峰值 ([\d.]+) kg", l2)

    def gv(x):
        return None if (x is None or x.group(1) == "—") else float(x.group(1))
    m["pickup_rate"] = gv(mp)
    m["delivery_rate"] = gv(md)
    m["urgent_rate"] = gv(mu)
    m["normal_rate"] = gv(mn)
    m["avg_load"] = float(ml.group(1))
    m["utilization"] = float(mut.group(1))
    m["peak"] = float(mpk.group(1))
    return m


def _compare_13(appvals, oraclevals, tag):
    """把 app 自报的 13 字段与 oracle 复算值比对（容差按 §6.4）。"""
    ok = True
    tol3 = ["distance", "time", "cost"]
    tolint = ["penalty", "stops", "actions"]
    tol1 = ["pickup_rate", "delivery_rate", "urgent_rate", "normal_rate",
            "avg_load", "utilization", "peak"]
    for k in tol3 + tolint + tol1:
        a, b = appvals[k], oraclevals[k]
        if a is None or b is None:
            good = (a is None and b is None)
        elif k in tolint:
            good = int(a) == int(b)
        elif k in tol3:
            good = abs(float(a) - float(b)) <= 1e-3 + 1e-9
        else:
            good = abs(float(a) - float(b)) <= 0.1 + 1e-9
        if not good:
            print("[FAIL] %s: %s app=%s oracle=%s" % (tag, k, a, b))
            ok = False
    return ok


def _dump_selfcheck():
    """用真实 app 二进制生成的 --dump-plan 跑一遍复算，与 app 自报值比对。

    三层都要一致（容差 §6.4）：逐车 METRICS、车队 FLEET、--plan-summary 的 13 字段。
    纪律：不一致先查实现、不改数据（若这里红，是实现/口径分叉，不是允许改数据）。
    找不到 app 二进制时跳过（Qt 缺失的环境里 --dump-plan 无法运行）。
    """
    import os
    import subprocess
    repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    app = os.path.join(repo, "build", "app")
    if not os.path.exists(app):
        print("[oracle] 未找到 app 二进制，跳过 --dump-plan 对拍")
        return True
    env = dict(os.environ)
    env.setdefault("QT_QPA_PLATFORM", "offscreen")
    ok = True
    for kind in ("distance", "cost", "time"):
        dump = subprocess.run([app, "--dump-plan", kind], capture_output=True, text=True,
                              env=env).stdout
        summary = subprocess.run([app, "--plan-summary", kind], capture_output=True,
                                 text=True, env=env).stdout
        data = _parse_dump_text(dump)
        ready, relay_pickups = _relay_ready_tables(data.get("legs", []))
        g, orders, vehicles = _load_dataset()
        urgent_ids = {o["id"] for o in orders if o["urgent"]}
        by_id = {v["id"]: v for v in vehicles}
        results = []
        for plan in data["plans"]:
            veh = by_id.get(plan["id"])
            if plan.get("infeasible") or veh is None:
                print("[FAIL] %s: 车辆段不可行或不在配置里" % plan.get("id"))
                ok = False
                continue
            m = recompute_plan(g, plan, kind, time_factor=veh["time_factor"],
                               cost_factor=veh["cost_factor"], capacity=veh["capacity"],
                               urgent_ids=urgent_ids, ready=ready,
                               relay_pickups=relay_pickups,
                               order_a={o["id"]: o["pickup"] for o in orders})
            if m is None:
                print("[FAIL] %s: dump 复算不可达" % plan["id"])
                ok = False
                continue
            results.append(m)
            if plan.get("metrics") is not None and not _compare_vehicle(
                    plan["metrics"], m, plan["id"]):
                ok = False
        fleet = aggregate_fleet(results)
        if fleet is None:
            print("[FAIL] %s: 没有可复算的车辆段" % kind)
            ok = False
            continue
        if data.get("fleet_metrics") is not None and not _compare_fleet(
                data["fleet_metrics"], fleet):
            ok = False
        appvals = _parse_summary(summary)
        if not _compare_13(appvals, fleet, "summary-%s" % kind):
            ok = False
        else:
            print("[oracle] dump 对拍通过（%s）：逐车 %d 台 + 车队 13 字段"
                  % (kind, len(results)))
    return ok

def main():
    if "--check" in sys.argv:
        return self_check()
    if "--route" in sys.argv:
        i = sys.argv.index("--route")
        return _cli_route(sys.argv[i + 1:])
    if "--plan" in sys.argv:
        i = sys.argv.index("--plan")
        return _cli_plan(sys.argv[i + 1:])

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
    print("RoutePlanner 夹具（tests/routeplanner_tests.cpp，A→B 时间模型）")
    print("=" * 72)

    show("切片1 单订单 W<->D1，窗口 540-1080，发车 480（停靠零耗时）",
         route_metrics(fixture_w_d1(), "W", "W", 480, ["D1"], "distance",
                       windows={"D1": (540, 1080)}))

    show("切片1 空订单",
         route_metrics(fixture_w_d1(), "W", "W", 480, [], "distance"))

    show("切片2 贪心 D1 优先（W-D1=1.0 近于 W-D2=5.0）",
         route_metrics(fixture_greedy(), "W", "W", 480, ["D1", "D2"], "distance",
                       windows={"D1": (0, 1440), "D2": (0, 1440)}))

    show("切片2 并列：应取字典序 DA",
         route_metrics(fixture_tie(), "W", "W", 480, ["DA", "DZ"], "distance",
                       windows={"DA": (0, 1440), "DZ": (0, 1440)}))

    show("切片3 紧急优先：先 D2 再 D1",
         route_metrics(fixture_greedy(), "W", "W", 480, ["D2", "D1"], "distance",
                       windows={"D1": (0, 1440), "D2": (0, 1440)}))

    show("切片4 晚到：窗口 480-485",
         route_metrics(fixture_w_d1(), "W", "W", 480, ["D1"], "distance",
                       windows={"D1": (480, 485)}))

    show("切片4 同停靠两单各按自己的窗口（P1 485 止 / P2 600 止）",
         route_metrics(fixture_w_d1(), "W", "W", 480,
                       [{"node": "D1", "pickups": [],
                         "deliveries": [("P1", 480, 485), ("P2", 480, 600)]}], "distance"))

    show("切片6 replan：已到 D2，时刻 600，剩余 D1",
         route_metrics(fixture_greedy(), "D2", "W", 600, ["D1"], "distance",
                       windows={"D1": (0, 1440)}))

    show("切片6 replan：已在 D1，时刻 600",
         route_metrics(fixture_w_d1(), "D1", "W", 600, ["D1"], "distance",
                       windows={"D1": (0, 1440)}))

    print("\n" + "=" * 72)
    print("A→B 取送时间模型（规格一 #23 §5.4 / #18 §4.4）")
    print("=" * 72)

    show("取送链：W->A 取 X1（无窗口）-> B 送 X1 -> 回 W",
         route_metrics(fixture_pdp(), "W", "W", 480,
                       [{"node": "A", "pickups": ["X1"], "deliveries": []},
                        {"node": "B", "pickups": [], "deliveries": [("X1", 0, 1440)]}],
                       "distance"))

    show("合并停靠：M 先送 X1 再取 X2 -> N 送 X2 -> 回 W",
         route_metrics(fixture_merged(), "W", "W", 480,
                       [{"node": "M", "pickups": ["X2"], "deliveries": [("X1", 480, 1440)]},
                        {"node": "N", "pickups": [], "deliveries": [("X2", 0, 1440)]}],
                       "distance"))

    show("边界：arrival == windowEnd（490 到、窗口止 490）不算迟到",
         route_metrics(fixture_w_d1(), "W", "W", 480, ["D1"], "distance",
                       windows={"D1": (480, 490)}))

    orders = [
        {"id": "X1", "pickup": "W", "delivery": "M", "ws": 480, "we": 1080},
        {"id": "X2", "pickup": "M", "delivery": "N", "ws": 0, "we": 1440},
    ]
    show("由节点序列 W M N W 推动作后复算（合并停靠：M 卸 X1 + 取 X2）",
         route_metrics(fixture_merged(), "W", "W", 480,
                       stops_from_route_nodes(["W", "M", "N", "W"], orders), "distance"))

    # ------------------------------------------------------------ E1 路况
    print("\n" + "=" * 72)
    print("E1 路况模拟与重规划（tests/traffic_tests.cpp）")
    print("=" * 72)

    greedy_edges = len(fixture_greedy().edges)
    print(f"  fixture_greedy 有向边数 = {greedy_edges}")
    for ratio in (0.1, 0.5, 1.0):
        print(f"  traffic_count(E={greedy_edges}, ratio={ratio}) = "
              f"{traffic_count(greedy_edges, ratio)}")

    show("切片C 全部边 +50% 后重规划（W 480 出发，服务 D1，窗口 540-1080）",
         route_metrics(fixture_w_d1_congested(0.5), "W", "W", 480, ["D1"], "distance",
                       windows={"D1": (540, 1080)}))
    print("  对照（未拥堵）: totalDistance 10.0  totalTime 70.0")

    # ------------------------------------------------------------ E3 插单
    print("\n" + "=" * 72)
    print("E3 动态订单插入（tests/traffic_tests.cpp）")
    print("=" * 72)

    show("切片D 插入紧急单：从 W 480 出发服务 [D2(紧急), D1]",
         route_metrics(fixture_greedy(), "W", "W", 480, ["D2", "D1"], "distance",
                       windows={"D1": (0, 1440), "D2": (0, 1440)}))

    path, w = shortest(fixture_greedy(), "W", "D2", "time")
    print(f"\n  切片E 冲突判定：W->D2 最短路 path={path} travel={w}min")
    print(f"    时刻 600 出发的最早到达 = {600 + w}")
    print("    新单窗口止 500  < 606 -> 必然超时 -> warning 非空")
    print("    新单窗口止 1440 >= 606 -> 无冲突 -> warning 为空")

    show("切片E 紧张窗口 [400,500] 的实际结果（超时不弃，仍生成路线）",
         route_metrics(fixture_greedy(), "W", "W", 600, ["D2"], "distance",
                       windows={"D2": (400, 500)}))

    show("切片E 对照：窗口 [0,1440]",
         route_metrics(fixture_greedy(), "W", "W", 600, ["D2"], "distance",
                       windows={"D2": (0, 1440)}))

    return 0


if __name__ == "__main__":
    sys.exit(main())
