#!/usr/bin/env python3
"""默认数据集生成器：可复现、带自检、与已提交文件比对。

背景：config/default.ini 原本由一次性脚本（随手写在 /tmp）产出，脚本未入库。
后果是这份数据**无法复现、无法审计、无法在调整参数后重生成**，
也正是两个真实数据缺陷（巡游 28 小时、成本与距离线性相关）得以产生的土壤。

本脚本解决三个问题：
  1. 生成可复现：所有影响权重的系数都是具名参数，拓扑为显式定义。
  2. 生成即自检：生成后立刻校验数据有效性，不通过则非零退出。
  3. 产物可证明：默认模式下**重生成并与已提交的 default.ini 逐字节比对**，
     从而保证提交的数据确实出自本脚本，而非被手工改动过的孤儿文件。

用法：
    python3 tools/generate_dataset.py            # 校验：重生成并与已提交文件比对 + 自检
    python3 tools/generate_dataset.py --write    # 自检通过后写回 config/default.ini
    python3 tools/generate_dataset.py --check    # 只对已提交文件做自检

已接入 ctest（dataset_selfcheck）。

注：本脚本的自检是**生成期的必要预警**，不是充分证明。例如它能发现
"cost/dist 比值恒定"这一"双策略必然等价"的必要条件，但真正"两种策略必须
给出不同路线"的充分验证在 tests/integration_tests.cpp 中（需运行规划器）。
"""

import argparse
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_INI = os.path.join(ROOT, "config", "default.ini")

# ---------------------------------------------------------------- 可调参数
# 坐标单位为 1/SCALE 公里。放大 SCALE 会缩短里程与耗时；
# 若过小会导致单车辆一天跑不完、全部订单超时（这正是当初的缺陷之一）。
SCALE = 64.0

# 道路等级：主干道快但贵（含通行费），支线慢但便宜。
# 若两类系数让 cost 与 dist 的比值**处处相同**，最低成本策略就必然等价于
# 最短距离策略，双策略功能形同虚设（这正是当初的另一个缺陷）。
TRUNK_TIME_FACTOR = 1.0    # 主干道 min/km
TRUNK_COST_FACTOR = 1.6    # 主干道 元/km
BRANCH_TIME_FACTOR = 2.4
BRANCH_COST_FACTOR = 1.2

TRAFFIC_CHANGE_RATIO = 0.1
TRAFFIC_TIME_INCREASE_MIN = 0.2
TRAFFIC_TIME_INCREASE_MAX = 0.5
# 软件内时间的事件模型（见 docs/records/specs/sim-time-events.md §2/§3.2 与 docs/设计.md D24）：
# 每 EVENT_INTERVAL_MIN 分钟软件内时间抽 1 个事件。概率由四类事件的**等级**决定
# （规格一 #23 §3.3）：extra_low / low / medium / high -> 代码内固定表 1/3/9/27。
EVENT_INTERVAL_MIN = 15
EVENT_LEVELS = [
    ("event_level_traffic", "high"),
    ("event_level_customer", "medium"),
    ("event_level_urgent", "low"),
    ("event_level_closure", "extra_low"),
]

# 默认车队（规格二 §3.2 数据集事实，**权威**）：
#   快车 V01 400kg / 1.00 / 1.50；慢车 V02、V03 160kg / 1.25 / 0.80；全部 W01、08:00 同刻发车。
# 三车 / 三枢纽批次（160 / 144 / 152kg）天然一一对应；"慢车 160kg 零余量"是例外非常态
# （规格二 §3.8），只有分配把两批压到同一台慢车时才会走降载/兜底。
VEHICLES = [
    ("V01", "W01", 400, "08:00", 1.00, 1.50),
    ("V02", "W01", 160, "08:00", 1.25, 0.80),
    ("V03", "W01", 160, "08:00", 1.25, 0.80),
]

# 初始数据集（规格一 #23「四、订单表（权威版，25 行）」**逐字照抄**）。
# 纪律：kg 与窗口一律照抄，不在生成器里做任何算术或"顺手调整"；
# 结构事实（A 集合 / B 集合 / 总重 / 窗口分布）由下方 self_check 独立断言。
ORDER_ROWS = [
    "O01, D08, D01, 20, 08:00, 13:00, 0",
    "O02, D08, D02, 20, 08:15, 16:00, 0",
    "O03, D08, D03, 20, 08:15, 15:00, 0",
    "O04, D08, D04, 20, 09:00, 20:00, 0",
    "O05, D08, D07, 20, 08:30, 17:00, 0",
    "O06, D08, D10, 20, 08:30, 18:00, 0",
    "O07, D08, D11, 20, 08:00, 08:30, 0",
    "O08, D08, D12, 20, 08:45, 19:00, 0",
    "O09, D01, D08, 20, 08:15, 14:00, 0",
    "O10, D14, D05, 24, 09:15, 21:00, 0",
    "O11, D14, D06, 24, 10:00, 10:30, 0",
    "O12, D14, D09, 24, 09:00, 11:00, 0",
    "O13, D14, D13, 24, 09:00, 09:30, 0",
    "O14, D14, D15, 24, 09:45, 15:30, 0",
    "O15, D14, D16, 24, 10:00, 13:00, 0",
    "O16, D05, D14, 20, 09:30, 12:30, 0",
    "O17, D20, D17, 19, 13:30, 20:30, 0",
    "O18, D20, D18, 19, 10:30, 20:00, 0",
    "O19, D20, D19, 19, 10:30, 19:00, 0",
    "O20, D20, D21, 19, 13:00, 20:00, 0",
    "O21, D20, D22, 19, 10:30, 15:30, 0",
    "O22, D20, D23, 19, 10:30, 17:00, 0",
    "O23, D20, D24, 19, 10:30, 21:00, 0",
    "O24, D20, D25, 19, 14:00, 21:00, 0",
    "O25, D23, D20, 20, 10:30, 18:00, 0",
]

# 规格一 §3.5 的数据集事实（self_check 的独立期望值）
EXPECTED_PICKUP_NODES = {"D01", "D05", "D08", "D14", "D20", "D23"}
EXPECTED_TOTAL_KG = 516.0
EXPECTED_MAX_SINGLE_KG = 24.0
EXPECTED_ORDER_COUNT = 25
TIGHT_WINDOWS = {
    "D06": ("10:00", "10:30"),
    "D11": ("08:00", "08:30"),
    "D13": ("09:00", "09:30"),
}
LATE_OPEN_WINDOWS = {
    "D17": ("13:30", "20:30"),
    "D21": ("13:00", "20:00"),
    "D25": ("14:00", "21:00"),
}
EXAMPLE_WINDOW = ("D09", ("09:00", "11:00"))  # 需求原文「9:00-11:00 送达」示例窗

# ---------------------------------------------------------------- 拓扑定义
NODES = [
    ("W01", "warehouse", 80, 160, "中央仓库A", 0),
    ("W02", "warehouse", 80, 500, "中央仓库B", 0),
    ("T01", "transit", 380, 200, "中转站甲", 1),
    ("T02", "transit", 380, 460, "中转站乙", 2),
    ("T03", "transit", 700, 330, "中转站丙", 3),
]

DELIVERY_COORDS = {
    "D01": (180, 90), "D02": (200, 180), "D03": (250, 280), "D04": (160, 300),
    "D05": (230, 380), "D06": (170, 480), "D07": (270, 120), "D08": (300, 220),
    "D09": (300, 400), "D10": (480, 120), "D11": (520, 200), "D12": (460, 260),
    "D13": (500, 340), "D14": (470, 460), "D15": (520, 540), "D16": (450, 600),
    "D17": (560, 420), "D18": (620, 120), "D19": (660, 220), "D20": (600, 300),
    "D21": (760, 180), "D22": (820, 260), "D23": (700, 440), "D24": (800, 400),
    "D25": (760, 540),
}

# 配送点之间的横向连接：把原来的"星形"支路补成网状。
# 加这 14 对之后，无向道路对数达到 50，满足"路径总数 >= 50"的无向口径（D19）。
EXTRA_LINKS = [
    ("D01", "D02"), ("D02", "D03"), ("D03", "D05"), ("D05", "D06"),
    ("D07", "D08"), ("D08", "D03"),
    ("D10", "D11"), ("D11", "D12"), ("D12", "D13"), ("D13", "D14"), ("D14", "D15"),
    ("D18", "D19"), ("D19", "D20"), ("D21", "D22"),
]

# 单行道（D18）：每条都是"捷径"，其反向可由既有路网绕行到达，
# 因此不破坏"每个配送点都能往返仓库"的可达性要求。
# 它们使"有向图"在行为上可与"无向图"区分开。
ONE_WAY_LINKS = [
    ("D07", "D10"),   # 反向：D10 -> T01 -> D07
    ("D16", "D17"),   # 反向：D17 -> T03 -> T02 -> D16
    ("D22", "D25"),   # 反向：D25 -> T03 -> D22
    ("D04", "D07"),   # 反向：D07 -> T01 -> D04
    ("D09", "D14"),   # 反向：D14 -> T02 -> D09
    ("D13", "D17"),   # 反向：D17 -> T03 -> T02 -> D13
]


def build_links():
    links = [("W01", "T01"), ("W01", "T03"), ("W02", "T02"), ("W02", "T03"),
             ("T01", "T02"), ("T02", "T03"),
             ("W01", "D01"), ("W02", "D25"),
             ("T01", "D10"), ("T02", "D09"), ("T03", "D17")]
    for i in range(1, 10):
        links.append(("T01", "D%02d" % i))
    for i in range(10, 18):
        links.append(("T02", "D%02d" % i))
    for i in range(18, 26):
        links.append(("T03", "D%02d" % i))
    links.extend(EXTRA_LINKS)
    return links


def build_one_way_links():
    return list(ONE_WAY_LINKS)


def nearest_transit(x, y):
    """最近的集散站编号。配送点据此归属某个中转站的子网络
    （需求原文：中转站可嵌套子配送网络，如"中转站A下属3个社区配送点"）。"""
    best, best_d = 0, None
    for nid, ntype, nx, ny, _name, sub in NODES:
        if ntype != "transit":
            continue
        d = math.hypot(x - nx, y - ny)
        if best_d is None or d < best_d:
            best, best_d = sub, d
    return best


def all_nodes():
    nodes = list(NODES)
    for i in range(1, 26):
        nid = "D%02d" % i
        x, y = DELIVERY_COORDS[nid]
        # 配送点归属于最近集散站的子网络；这是数据里的**行政归属**。
        nodes.append((nid, "delivery", x, y, "客户%02d" % i, nearest_transit(x, y)))
    return nodes


def is_trunk(a, b):
    return a[0] in "WT" and b[0] in "WT"


def edge_weights(positions, a, b):
    dx = positions[a][0] - positions[b][0]
    dy = positions[a][1] - positions[b][1]
    dist = round(math.hypot(dx, dy) / SCALE, 1)
    if is_trunk(a, b):
        return dist, round(dist * TRUNK_TIME_FACTOR, 1), round(dist * TRUNK_COST_FACTOR, 1)
    return dist, round(dist * BRANCH_TIME_FACTOR, 1), round(dist * BRANCH_COST_FACTOR, 1)


# ---------------------------------------------------------------- 生成
def build_text():
    nodes = all_nodes()
    positions = {n[0]: (n[2], n[3]) for n in nodes}
    links = build_links()

    out = ["[general]",
           "traffic_change_ratio = %s" % TRAFFIC_CHANGE_RATIO,
           "traffic_time_increase_min = %s" % TRAFFIC_TIME_INCREASE_MIN,
           "traffic_time_increase_max = %s" % TRAFFIC_TIME_INCREASE_MAX,
           "event_interval_min = %d" % EVENT_INTERVAL_MIN]
    for key, level in EVENT_LEVELS:
        out.append("%s = %s" % (key, level))
    out += ["", "[nodes]", "# ID, type, x, y, name, sub_network_id"]
    for n in nodes:
        out.append("%s, %s, %d, %d, %s, %d" % n)

    out += ["", "[edges]",
            "# from, to, distance_km, time_min, cost_yuan",
            "# 双向路发两条；单向路（单行道）只发一条"]
    for a, b in links:
        d, t, c = edge_weights(positions, a, b)
        out.append("%s, %s, %s, %s, %s" % (a, b, d, t, c))
        out.append("%s, %s, %s, %s, %s" % (b, a, d, t, c))
    for a, b in build_one_way_links():
        d, t, c = edge_weights(positions, a, b)
        out.append("%s, %s, %s, %s, %s" % (a, b, d, t, c))

    out += ["", "[vehicles]",
            "# ID, start_node, capacity_kg, depart_time, time_factor, cost_factor"]
    for vid, start, cap, depart, tf, cf in VEHICLES:
        out.append("%s, %s, %d, %s, %.2f, %.2f" % (vid, start, cap, depart, tf, cf))
    out += ["", "[orders]",
            "# ID, pickup, delivery, demand_kg, window_start, window_end, is_urgent"]
    out += ORDER_ROWS

    return "\n".join(out) + "\n"


# ---------------------------------------------------------------- 解析 + 自检
def parse(text):
    sections, cur = {}, None
    for raw in text.splitlines():
        line = raw.split("#")[0].strip()
        if not line:
            continue
        if line.startswith("["):
            cur = line[1:-1]
            sections[cur] = []
            continue
        if cur:
            sections[cur].append(line)
    return sections


def to_minutes(hhmm):
    h, m = hhmm.split(":")
    return int(h) * 60 + int(m)


def reachable(edges, start, goal):
    adj = {}
    for (a, b), _w in edges.items():
        adj.setdefault(a, []).append(b)
    seen, queue = {start}, [start]
    for u in queue:
        if u == goal:
            return True
        for v in adj.get(u, []):
            if v not in seen:
                seen.add(v)
                queue.append(v)
    return goal in seen


def self_check(text):
    """返回问题列表；空列表表示通过。"""
    problems = []
    sec = parse(text)
    for required in ("general", "nodes", "edges", "vehicles", "orders"):
        if required not in sec:
            problems.append("缺少 [%s] 节" % required)
    if problems:
        return problems

    # ---- [general]：等级键，且退役键一个都不能出现 ----
    general = {}
    for row in sec["general"]:
        key, _, value = row.partition("=")
        general[key.strip()] = value.strip()
    allowed_levels = {"extra_low", "low", "medium", "high"}
    for key, _default in EVENT_LEVELS:
        if key not in general:
            problems.append("[general] 缺少 %s" % key)
        elif general[key] not in allowed_levels:
            problems.append("%s 取值非法: %s" % (key, general[key]))
    for retired in ("service_time_min", "event_weight_traffic", "event_weight_urgent",
                    "event_weight_customer", "event_weight_closure",
                    "traffic_change_interval_sec", "urgent_order_interval_sec"):
        if retired in general:
            problems.append("[general] 仍含退役键 %s" % retired)

    # ---- [nodes] ----
    nodes = {}
    sub_of = {}
    for row in sec["nodes"]:
        f = [x.strip() for x in row.split(",")]
        if len(f) != 6:
            problems.append("[nodes] 字段数不是 6: %s" % row)
            continue
        if f[0] in nodes:
            problems.append("节点 ID 重复: %s" % f[0])
        nodes[f[0]] = f[1]
        sub_of[f[0]] = f[5]

    counts = {"warehouse": 0, "delivery": 0, "transit": 0}
    for t in nodes.values():
        if t in counts:
            counts[t] += 1
    if len(nodes) < 30:
        problems.append("节点总数 %d < 30" % len(nodes))
    if counts["warehouse"] < 2:
        problems.append("仓库 %d < 2" % counts["warehouse"])
    if counts["delivery"] < 15:
        problems.append("配送点 %d < 15" % counts["delivery"])
    if counts["transit"] < 3:
        problems.append("中转站 %d < 3" % counts["transit"])

    # ---- [edges] ----
    edges = {}
    ratios = set()
    for row in sec["edges"]:
        f = [x.strip() for x in row.split(",")]
        if len(f) != 5:
            problems.append("[edges] 字段数不是 5: %s" % row)
            continue
        a, b = f[0], f[1]
        try:
            d, t, c = float(f[2]), float(f[3]), float(f[4])
        except ValueError:
            problems.append("[edges] 权重非数值: %s" % row)
            continue
        if a not in nodes or b not in nodes:
            problems.append("边端点不存在: %s -> %s" % (a, b))
        if (a, b) in edges:
            problems.append("重复边: %s -> %s" % (a, b))
        if min(d, t, c) <= 0:
            problems.append("非正权重: %s -> %s" % (a, b))
        edges[(a, b)] = (d, t, c)
        ratios.add(round(c / d, 6) if d else 0.0)

    # 规模口径（D19）：有向边数与无向道路对数都要达标，避免验收时口径不同而不合格
    undirected = set()
    one_way = 0
    for (a, b) in edges:
        undirected.add((a, b) if a < b else (b, a))
    for (a, b) in edges:
        if (b, a) not in edges:
            one_way += 1

    if len(edges) < 50:
        problems.append("有向边数 %d < 50" % len(edges))
    if len(undirected) < 50:
        problems.append("无向道路对数 %d < 50" % len(undirected))
    if one_way == 0:
        problems.append("没有任何单向边——有向图与无向图无从区分（D18）")

    # 中转站语义（D16/D17）：每个中转站的子网络必须至少含 1 个配送点，
    # 否则该站的"集散/暂存"没有服务对象，子网络划分也失去意义
    subs = {}
    for row in sec["nodes"]:
        f = [x.strip() for x in row.split(",")]
        if len(f) < 6:
            continue
        subs.setdefault(f[5], []).append((f[0], f[1]))
    for sub, members in subs.items():
        if sub == "0":
            continue
        deliveries = [m for m in members if m[1] == "delivery"]
        transits = [m for m in members if m[1] == "transit"]
        if not transits:
            problems.append("子网络 %s 没有中转站" % sub)
        if not deliveries:
            problems.append("子网络 %s 没有归属的配送点" % sub)
    for nid, ntype in nodes.items():
        if ntype != "transit":
            continue
        found = any(nid in [m[0] for m in members] for sub, members in subs.items() if sub != "0")
        if not found:
            problems.append("中转站 %s 未归入任何子网络" % nid)

    # 关键预警：cost/dist 比值若处处相同，则最低成本策略必然等价于最短距离策略
    if len(ratios) < 2:
        problems.append("cost/dist 比值恒定（%s），双策略将恒等价" % sorted(ratios))
    elif max(ratios) / min(ratios) < 1.2:
        problems.append("cost/dist 比值变化过小（%.3f~%.3f），双策略可能无法分化"
                        % (min(ratios), max(ratios)))

    # ---- [vehicles]：六列（规格一阶段单车 V01，两因子 1.00/1.00） ----
    vehicles = []
    vehicle_ids = set()
    for row in sec["vehicles"]:
        f = [x.strip() for x in row.split(",")]
        if len(f) != 6:
            problems.append("[vehicles] 字段数不是 6: %s" % row)
            continue
        if not f[0]:
            problems.append("[vehicles] 车辆 ID 为空")
        if f[0] in vehicle_ids:
            problems.append("[vehicles] 车辆 ID 重复: %s" % f[0])
        vehicle_ids.add(f[0])
        if f[1] not in nodes or nodes.get(f[1]) != "warehouse":
            problems.append("起始节点不是仓库: %s" % f[1])
        try:
            cap = float(f[2])
            to_minutes(f[3])
            time_factor = float(f[4])
            cost_factor = float(f[5])
        except ValueError:
            problems.append("[vehicles] 数值非法: %s" % row)
            continue
        if cap <= 0:
            problems.append("载重必须为正: %s" % f[2])
        if time_factor <= 0 or cost_factor <= 0:
            problems.append("车辆因子必须为正: %s" % row)
        vehicles.append((f[0], f[1], cap, f[3], time_factor, cost_factor))

    # 生成器自检另加（规格二 §3.1，比解析层严，只约束**默认数据集**）：
    #   · 因子 ∈ [0.5, 2]
    #   · 至少一台 capacity_kg ≥ 2 × min(capacity_kg)
    #   · 存在 time_factor > 1 与 cost_factor < 1 的车（两型车真的分化）
    # 注意（规格自相矛盾，已按权威事实处理）：§3.1 还要求「至少 1 台
    # time_factor = 1 且 cost_factor = 1 的基准车」，但 §3.2 的默认车队
    # V01 是 1.00 / 1.50 —— **没有这种车**。§3.2 是数据集事实（权威），
    # 故该条**跳过不检**，也不为了凑它改数据。
    for vid, _start, cap, _depart, tf, cf in vehicles:
        if tf < 0.5 or tf > 2.0 or cf < 0.5 or cf > 2.0:
            problems.append("[vehicles] 因子不在 [0.5, 2]: %s (%.2f/%.2f)" % (vid, tf, cf))
    if vehicles:
        caps = [v[2] for v in vehicles]
        min_cap = min(caps)
        if not any(c >= 2.0 * min_cap for c in caps):
            problems.append("[vehicles] 没有一台车的容量 >= 2 × 最小容量（快车大容量不成立）")
        if not any(v[4] > 1.0 for v in vehicles):
            problems.append("[vehicles] 不存在 time_factor > 1 的慢车")
        if not any(v[5] < 1.0 for v in vehicles):
            problems.append("[vehicles] 不存在 cost_factor < 1 的车")

    # ---- [orders]：七列 + 规格一 §3.5/§3.8 的全部结构事实 ----
    delivery_ids = {nid for nid, t in nodes.items() if t == "delivery"}
    parsed_orders = []
    seen_order_ids = set()
    total_demand = 0.0
    max_single_demand = 0.0
    pickup_nodes = set()
    delivery_nodes = set()
    window_of = {}
    for row in sec["orders"]:
        f = [x.strip() for x in row.split(",")]
        if len(f) != 7:
            problems.append("[orders] 字段数不是 7: %s" % row)
            continue
        oid, a, b = f[0], f[1], f[2]
        if not oid:
            problems.append("[orders] 订单 ID 为空: %s" % row)
        if oid in seen_order_ids:
            problems.append("[orders] 订单 ID 重复: %s" % oid)
        seen_order_ids.add(oid)
        if a == b:
            problems.append("[orders] 取货点与送达点相同: %s" % oid)
        if nodes.get(a) != "delivery":
            problems.append("取货点不是配送点: %s" % a)
        if nodes.get(b) != "delivery":
            problems.append("送达点不是配送点: %s" % b)
        if a in sub_of and b in sub_of and sub_of[a] != sub_of[b]:
            problems.append("[orders] A/B 不在同一子网络: %s (%s->%s)" % (oid, a, b))
        try:
            demand = float(f[3])
            ws, we = to_minutes(f[4]), to_minutes(f[5])
            urgent = int(f[6])
        except ValueError:
            problems.append("[orders] 数值非法: %s" % row)
            continue
        if demand <= 0:
            problems.append("货物量必须为正: %s" % f[3])
        if ws >= we:
            problems.append("窗口起必须早于止: %s" % row)
        if urgent not in (0, 1):
            problems.append("is_urgent 只能为 0 或 1: %s" % f[6])
        if urgent != 0:
            problems.append("初始数据集必须全普通（is_urgent = 0）: %s" % oid)
        pickup_nodes.add(a)
        delivery_nodes.add(b)
        window_of[b] = (f[4], f[5])
        total_demand += demand
        if demand > max_single_demand:
            max_single_demand = demand
        parsed_orders.append((oid, a, b, demand))

    if len(parsed_orders) != EXPECTED_ORDER_COUNT:
        problems.append("订单数 %d != %d" % (len(parsed_orders), EXPECTED_ORDER_COUNT))
    if delivery_nodes != delivery_ids:
        problems.append("B 集合 != 全部配送点（缺 %s / 多 %s）"
                        % (sorted(delivery_ids - delivery_nodes),
                           sorted(delivery_nodes - delivery_ids)))
    if pickup_nodes != EXPECTED_PICKUP_NODES:
        problems.append("A 集合 != %s（实际 %s）"
                        % (sorted(EXPECTED_PICKUP_NODES), sorted(pickup_nodes)))
    if abs(total_demand - EXPECTED_TOTAL_KG) > 1e-9:
        problems.append("总重 %.1fkg != %.1fkg" % (total_demand, EXPECTED_TOTAL_KG))
    if abs(max_single_demand - EXPECTED_MAX_SINGLE_KG) > 1e-9:
        problems.append("最大单笔 %.1fkg != %.1fkg"
                        % (max_single_demand, EXPECTED_MAX_SINGLE_KG))

    # 窗口分布：3 紧 + 3 晚开 + D09 示例窗仍在
    for node, expect in TIGHT_WINDOWS.items():
        if window_of.get(node) != expect:
            problems.append("紧窗丢失或改动: %s 应为 %s，实际 %s"
                            % (node, expect, window_of.get(node)))
    for node, expect in LATE_OPEN_WINDOWS.items():
        if window_of.get(node) != expect:
            problems.append("晚开窗丢失或改动: %s 应为 %s，实际 %s"
                            % (node, expect, window_of.get(node)))
    if window_of.get(EXAMPLE_WINDOW[0]) != EXAMPLE_WINDOW[1]:
        problems.append("示例窗丢失: %s 应为 %s" % (EXAMPLE_WINDOW[0], EXAMPLE_WINDOW[1]))

    if vehicles:
        min_capacity = min(v[2] for v in vehicles)
        if not (max_single_demand < min_capacity):
            problems.append("最大单笔 %.1fkg 不小于最小车辆载重 %.1fkg（Q1 严格小于）"
                            % (max_single_demand, min_capacity))
        # 往返可达性：有向图里"能去"不等于"能回"；A 与 B 都要能往返
        start = vehicles[0][1]
        for oid, a, b, _demand in parsed_orders:
            for nid in (a, b):
                if not reachable(edges, start, nid):
                    problems.append("从仓库 %s 不可达节点 %s（订单 %s）" % (start, nid, oid))
                if not reachable(edges, nid, start):
                    problems.append("节点 %s 无法返回仓库 %s（订单 %s）" % (nid, start, oid))

    return problems


# ---------------------------------------------------------------- CLI
def main():
    ap = argparse.ArgumentParser(description="默认数据集生成器")
    ap.add_argument("--write", action="store_true", help="自检通过后写回 config/default.ini")
    ap.add_argument("--check", action="store_true", help="只对已提交文件做自检，不做重生成比对")
    args = ap.parse_args()

    generated = build_text()

    problems = self_check(generated)
    if problems:
        print("[generate_dataset] 生成结果未通过自检：")
        for p in problems:
            print("  - " + p)
        return 1
    print("[generate_dataset] 自检通过（七列订单 / 六列车辆 / 等级键 / 双口径规模 / 单向边 / "
          "中转站归属 / A-B 同子网络 / 总重与窗口分布 / 往返可达 / 双策略可分化）")

    if args.write:
        os.makedirs(os.path.dirname(DEFAULT_INI), exist_ok=True)
        with open(DEFAULT_INI, "w", encoding="utf-8") as fh:
            fh.write(generated)
        print("[generate_dataset] 已写回 " + DEFAULT_INI)
        return 0

    if not os.path.exists(DEFAULT_INI):
        print("[generate_dataset] 找不到 " + DEFAULT_INI)
        return 1

    with open(DEFAULT_INI, encoding="utf-8") as fh:
        committed = fh.read()

    committed_problems = self_check(committed)
    if committed_problems:
        print("[generate_dataset] 已提交文件未通过自检：")
        for p in committed_problems:
            print("  - " + p)
        return 1

    if not args.check and committed != generated:
        print("[generate_dataset] 已提交的 default.ini 与生成器输出**不一致**：")
        print("  说明数据被手工改动过，或生成参数已变更。")
        print("  确认无误后执行 python3 tools/generate_dataset.py --write")
        import difflib
        for line in list(difflib.unified_diff(
                committed.splitlines(), generated.splitlines(),
                "committed", "generated", lineterm=""))[:40]:
            print("  " + line)
        return 1

    print("[generate_dataset] 已提交文件通过自检，且与生成器输出一致")
    return 0


if __name__ == "__main__":
    sys.exit(main())