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
SCALE = 40.0

# 道路等级：主干道快但贵（含通行费），支线慢但便宜。
# 若两类系数让 cost 与 dist 的比值**处处相同**，最低成本策略就必然等价于
# 最短距离策略，双策略功能形同虚设（这正是当初的另一个缺陷）。
TRUNK_TIME_FACTOR = 1.0    # 主干道 min/km
TRUNK_COST_FACTOR = 1.6    # 主干道 元/km
BRANCH_TIME_FACTOR = 1.8   # 支线 min/km
BRANCH_COST_FACTOR = 1.0   # 支线 元/km

SERVICE_TIME_MIN = 5
TRAFFIC_CHANGE_INTERVAL_SEC = 30
TRAFFIC_CHANGE_RATIO = 0.1
TRAFFIC_TIME_INCREASE_MIN = 0.2
TRAFFIC_TIME_INCREASE_MAX = 0.5
URGENT_ORDER_INTERVAL_SEC = 60

CAPACITY_KG = 800
DEPART_TIME = "08:00"
DEFAULT_WINDOW = ("09:00", "18:00")
# 刻意收紧的窗口：用于演示 E2（超时标记 + penalty），且保证超时是"少数"
TIGHT_WINDOWS = {"D05": ("09:00", "10:00"), "D12": ("09:00", "11:00")}
# 初始数据集中**不放任何紧急订单**：E3 的语义是"配送过程中动态插入紧急订单"，
# 预置紧急订单会让该功能看起来一开始就存在，混淆演示。
# 需要演示紧急优先时用界面的「插入紧急订单」或 Debug 模式。
URGENT_NODES = set()

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

DEMANDS = [30, 25, 35, 20, 40, 28, 32, 26, 30, 34, 22, 38, 24, 30, 26,
           42, 28, 20, 36, 30, 24, 26, 32, 28, 34]


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
    return links


def all_nodes():
    nodes = list(NODES)
    for i in range(1, 26):
        nid = "D%02d" % i
        x, y = DELIVERY_COORDS[nid]
        nodes.append((nid, "delivery", x, y, "客户%02d" % i, 0))
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
           "service_time_min = %d" % SERVICE_TIME_MIN,
           "traffic_change_interval_sec = %d" % TRAFFIC_CHANGE_INTERVAL_SEC,
           "traffic_change_ratio = %s" % TRAFFIC_CHANGE_RATIO,
           "traffic_time_increase_min = %s" % TRAFFIC_TIME_INCREASE_MIN,
           "traffic_time_increase_max = %s" % TRAFFIC_TIME_INCREASE_MAX,
           "urgent_order_interval_sec = %d" % URGENT_ORDER_INTERVAL_SEC,
           "", "[nodes]", "# ID, type, x, y, name, sub_network_id"]
    for n in nodes:
        out.append("%s, %s, %d, %d, %s, %d" % n)

    out += ["", "[edges]", "# from, to, distance_km, time_min, cost_yuan"]
    for a, b in links:
        d, t, c = edge_weights(positions, a, b)
        out.append("%s, %s, %s, %s, %s" % (a, b, d, t, c))
        out.append("%s, %s, %s, %s, %s" % (b, a, d, t, c))

    out += ["", "[vehicles]", "# ID, start_node, capacity_kg, depart_time",
            "V01, W01, %d, %s" % (CAPACITY_KG, DEPART_TIME),
            "", "[orders]", "# ID, node_id, demand_kg, window_start, window_end, is_urgent"]
    for i in range(1, 26):
        nid = "D%02d" % i
        ws, we = TIGHT_WINDOWS.get(nid, DEFAULT_WINDOW)
        urgent = 1 if nid in URGENT_NODES else 0
        out.append("O%02d, %s, %d, %s, %s, %d" % (i, nid, DEMANDS[i - 1], ws, we, urgent))

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

    nodes = {}
    for row in sec["nodes"]:
        f = [x.strip() for x in row.split(",")]
        if len(f) != 6:
            problems.append("[nodes] 字段数不是 6: %s" % row)
            continue
        if f[0] in nodes:
            problems.append("节点 ID 重复: %s" % f[0])
        nodes[f[0]] = f[1]

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

    if len(edges) < 50:
        problems.append("边总数 %d < 50" % len(edges))

    # 关键预警：cost/dist 比值若处处相同，则最低成本策略必然等价于最短距离策略
    if len(ratios) < 2:
        problems.append("cost/dist 比值恒定（%s），双策略将恒等价" % sorted(ratios))
    elif max(ratios) / min(ratios) < 1.2:
        problems.append("cost/dist 比值变化过小（%.3f~%.3f），双策略可能无法分化"
                        % (min(ratios), max(ratios)))

    vehicles = []
    for row in sec["vehicles"]:
        f = [x.strip() for x in row.split(",")]
        if len(f) != 4:
            problems.append("[vehicles] 字段数不是 4: %s" % row)
            continue
        if f[1] not in nodes or nodes.get(f[1]) != "warehouse":
            problems.append("起始节点不是仓库: %s" % f[1])
        try:
            cap = float(f[2])
            to_minutes(f[3])
        except ValueError:
            problems.append("[vehicles] 数值非法: %s" % row)
            continue
        if cap <= 0:
            problems.append("载重必须为正: %s" % f[2])
        vehicles.append((f[0], f[1], cap, f[3]))

    total_demand = 0.0
    for row in sec["orders"]:
        f = [x.strip() for x in row.split(",")]
        if len(f) != 6:
            problems.append("[orders] 字段数不是 6: %s" % row)
            continue
        if nodes.get(f[1]) != "delivery":
            problems.append("订单目标不是配送点: %s" % f[1])
        try:
            demand = float(f[2])
            ws, we = to_minutes(f[3]), to_minutes(f[4])
        except ValueError:
            problems.append("[orders] 数值非法: %s" % row)
            continue
        if demand <= 0:
            problems.append("货物量必须为正: %s" % f[2])
        if ws >= we:
            problems.append("窗口起必须早于止: %s" % row)
        total_demand += demand

    if vehicles:
        cap = vehicles[0][2]
        if total_demand > cap:
            problems.append("总需求 %.0f > 载重 %.0f" % (total_demand, cap))
        # 往返可达性：有向图里"能去"不等于"能回"，B6 要求遍历后返回
        start = vehicles[0][1]
        for row in sec["orders"]:
            nid = row.split(",")[1].strip()
            if not reachable(edges, start, nid):
                problems.append("从仓库 %s 不可达配送点 %s" % (start, nid))
            if not reachable(edges, nid, start):
                problems.append("配送点 %s 无法返回仓库 %s" % (nid, start))

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
    print("[generate_dataset] 自检通过（规模 / 端点 / 可达性 / 载重 / 双策略可分化）")

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
