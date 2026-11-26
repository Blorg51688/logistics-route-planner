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
#
# 规格四（#54）把主干道细分成两类，**系数只属于支线**：
#   · 城内支线（簇内边：站↔客户、客户↔客户）里程仍由坐标 / SCALE 算，沿用 2.4 / 1.2；
#   · 跨城干线（仓↔站、站↔站）里程**脱离欧氏距离**，改由下方 TRUNK_KM 里程表给定，
#     费率 = 高速口径（见 HIGHWAY_*）：快（min/km 低、≈65–90km/h）而贵（元/km 高，含通行费）。
# 为什么要拆开：规格四之前站间只有 4.1–5.4km —— 快车跑一趟成本太低，「攒一车再走」
# 没有经济动机（实测 V01 容量利用率仅 1.3%）。把干线的**里程**提上去才是尺度问题本身；
# 只调费率不改里程，跨城段仍是「几分钟的路」。
TRUNK_HIGHWAY = True       # 只有这里为真时干线才用 TRUNK_KM；置假 = 退回旧欧氏口径（反向验证用）
TRUNK_KM = {
    # (站A, 站B) -> 干线里程 km。**只放站↔站/仓↔站**，键按 ID 升序写。
    # 取值按两端在地图上的摆位排：仓库↔丙城最远、甲↔乙居中、仓↔本国站是「出城段」。
    # 这些数**不是**欧氏距离乘倍率得来的 —— 干线里程与坐标解耦正是本票的做法：
    # 坐标仍只服务画布布局与簇内几何，改干线里程不会把整张图拉大。
    #
    # 量级的上界不是随便定的：**慢车每天要往返一次**（规格三：所有车 W01 发车、期末回库），
    # 而验收 C24-③ 要求完工耗时 ≤ 12h。2 × D × 0.75 × 1.25（慢车倍率）+ 城内 ~3h，
    # 12h 预算 ⇒ D 超过 ~290km 就必然超时。所以干线取 200–260km：既是跨城量级，
    # 又让「当日往返」在物理上说得通。**这不是为了迁就判据而缩水**，是把工作日长度
    # 当作真实约束参与选址；再远的城市在「朝发夕归」模型里就不该有慢车驻场。
    ("T01", "T02"): 200,
    ("T01", "W01"): 180,
    ("T02", "T03"): 200,
    ("T02", "W01"): 200,     # 规格四补：甲↔乙不能只经 T01（慢车会绕 430km）
    ("T02", "W02"): 180,
    ("T03", "W01"): 240,
    ("T03", "W02"): 220,
}
# 国道（跨城「小路」）：跨簇的、含配送点的边（站↔对簇客户、客户↔客户、仓↔远客户）。
# 为什么必须有这一档：用户原话「一些通过用户点将簇连接的路线就是国道等绕的小路」。
# 这是簇与簇之间的**另一条通路**——比高速绕（里程更长）、慢（55km/h）、但没有通行费（便宜）。
# 若让这些边继续按城内几何算（1–2km），快车就有了「11km 穿城去隔壁市」的捷径，
# 跨城高速永远选不上，规格四的整件事（跨城尺度）就落空了 —— 实测反证：只改干线时
# 快车 T01→T03 实际只跑 27min、容量利用率 1.2%→1.7%，与改前几乎一样。
NATIONAL_KM = {
    # (A, B) -> 国道里程 km。键按 ID 升序写；覆盖全部「跨簇 + 含配送点」的边。
    # 这四条**都是客户↔客户**的跨簇连线 —— 正是用户原话里「通过用户点将簇连接的路线
    # 就是国道等绕的小路」：比高速里程更长、速度更慢，但没有通行费，成本最低。
    # 站↔客户一律是城内边（站只连本簇配送点，见 build_links），不进这张表。
    # **里程必须大于高速**：这是「绕」的字面含义，也是三策略能分化的前提（见下）。
    # 高速 T01↔T02 是 200km；经客户点的国道 T01→D03→D05→T02 = 2.4 + 260 + 5.5 = 268km。
    ("D03", "D05"): 260,     # 甲城 ↔ 乙城（经客户点绕行）
    ("D12", "D13"): 260,
    ("D13", "D17"): 260,     # 乙城 ↔ 丙城
    ("D16", "D17"): 260,
}
# 国道与高速的取舍（规格四 #54 实测出来的第二个真实缺陷）：
# 只把国道里程给到「比高速短」时，「距离策略」与「成本策略」会排出**逐字相同**的路线
# —— 三策略退化成两策略，验收 ①「三策略两两分化」变成空断言（实测 distance 与 cost
# 的全局 13 项完全相同）。真实的城际路网里国道当然比高速**绕**：
#   · 高速 T01→T02：200km / 150min / 600 元（0.75 min/km、3.0 元/km）
#   · 国道 T01→D03→D05→T02：268km / 241min / 218 元（0.9 min/km、0.8 元/km）
# 于是距离策略走高速、成本策略绕国道、时间策略走高速但可能排不同的停靠序 —— 三策略才真的分开。
# 国道费率：55km/h（1.1 min/km，比高速慢、比城内小路快）+ 0.8 元/km（无通行费，比高速便宜得多）。
# 三档费率的意义：时间策略走高速（315min/1260 元）、成本策略走国道（~400min/300 元）——
# 「快而贵 / 慢而省」在同一对城市之间成为真实的取舍，而不是换汤不换药。
NATIONAL_TIME_FACTOR = 0.9
NATIONAL_COST_FACTOR = 0.8
# 高速费率（按**两端所在簇**分三档，都属「快而贵」，理由见下）：
#   · 80km/h = 0.75 min/km：城际高速的卡车平均速度，比支线的 2.4 min/km（25km/h）快一个量级；
#   · 元/km 3.0 / 3.5 / 4.0：含通行费，是支线 1.2 元/km 的 2.5–3.3 倍。越偏远越贵
#     （W 与 T03 那一簇之间是长距离干线，收费系数更高）。
# 为什么成本倍率这么高：本票的目的是**攒批**（规格四用户原话：跨城尺度让「攒一车再走」有意义）。
# 费率太低的话 200km 的跨城腿仍然比城内绕行便宜，攒批就还是不会出现。
HIGHWAY_TIME_FACTOR = 0.75   # 干线 min/km（80km/h），三档同速
WAREHOUSE_LOCAL_COST_FACTOR = 3.0   # 仓↔本国站：出城段 元/km
# 簇对 -> (站↔站 元/km, 仓↔对簇站 元/km)
# 一档一条走廊，**同一走廊上几段高速公路单价相同**（站点/桥隧收费在这里已折进单价，
# 不再另计）—— 若各段单价不同又里程不等，就会出现「经 X 站绕行比直达更便宜」这种
# 与常识相反的路径（实测：T02→W02 一度绕 T03 反而少 80 元）。所以三档单价拉开的是
# **走廊级别**的差异（近程走廊最便宜、最远走廊最贵），走廊内部保持单一定价。
HIGHWAY_COST_BY_PAIR = {
    (1, 2): (3.0, 3.0),   # T01-T02：两簇相邻的标准高速
    (2, 3): (3.0, 3.0),   # T02-T03：同一走廊，同价
    (1, 3): (3.5, 3.5),   # T01-T03 / W-T03：最长的那条走廊，单价最高
}
# ---- 城内道路也分两级（规格四 #54 实测出来的第三个真实缺陷）----
# 只按「干线 / 支线」分两级时，**簇内每条边的 cost/dist 比值完全相同**（都是 1.2），
# 于是「最短距离」与「最低成本」在同一簇内排出**逐字相同**的路线。跨城尺度把干线拉到
# 180–240km 之后，「为省 0.4 元/km 而绕 180km 干线」这种取舍彻底消失（没人会为了省
# 几百块开 180 公里去接一个 5 公里外的客户），三策略就退化成两策略 ——
# 实测 distance 与 cost 的 13 个全局指标**逐字相同**，验收 ① 变成空断言。
#
# 修法与用户原话一致（「一些通过用户点将簇连接的路线就是国道等绕的小路」）：
# 城内也有主干道与小路的区别 ——
#   · **主干道**（站/仓 ↔ 客户）：快但贵，2.0 min/km（30km/h）· 1.5 元/km；
#   · **小路**（客户 ↔ 客户）：慢但便宜，3.0 min/km（20km/h）· 1.0 元/km。
# 三策略因此各有偏好：距离看 km、成本挑小路、时间挑主干道，三者必然分开。
CITY_MAIN_TIME_FACTOR = 2.0
CITY_MAIN_COST_FACTOR = 1.5
CITY_MINOR_TIME_FACTOR = 3.0
CITY_MINOR_COST_FACTOR = 1.0

# 尺度守卫（规格四 §1）：干线里程必须比簇内最大里程高两个数量级级距。
# 反向验证就是把它调到 10：把干线里程改回「几十公里以内」时下面这条自检必须报红。
TRUNK_SCALE_MIN_RATIO = 25.0   # 120km（最短干线）/ 4.5km（簇内最大）≈ 26.7 ⇒ 相对下界留余量
# 逐条干线里程的下降。120km 是**仓↔最近站**这一档：簇内最大 4.4km 的 27 倍，
# 已满足「1–2 个数量级」；200km 量级由四条站↔站/仓↔远站（160–260km）承担。
TRUNK_MIN_KM = 100.0
# 国道里程的下限：它也是跨城量级（否则「绕的小路」就成了城内捷径，高速又选不上）。
NATIONAL_MIN_KM = 100.0

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

# 默认车队（规格三 §4 数据集事实，**权威**）：
#   快车 V01 400kg / 1.00 / 1.50（只跑站间腿）；慢车 V02、V03、V04 160kg / 1.25 / 0.80
#   （各守一簇，簇 ID 升序 × 慢车 ID 升序固定配对）；全部 W01、08:00 同刻发车。
# 为什么是 3 台慢车而不是 2 台（规格二曾是 3 车）：职责封闭要求「每个子配送网有专属运力」，
# 3 簇 / 3 慢车 ⇒ 簇→慢车是**排列数 1 的固定指派**，无需枚举（#46 退役整簇全排列的根因）。
# 订单表一个字不改：25 单 516kg 全部簇内，每簇一批（160/144/152kg）仍与一台慢车对应。
# **规格四 #54 的改动：慢车从「离自己那一侧最近的中央仓库」发车，不再全部 W01。**
# 为什么必须改（这是本票实测出来的、规格三数据在跨城尺度下的内在不一致）：
#   规格三要求「职责封闭：每台慢车守一个簇」，同时又让**全部四台车从 W01 发车**。
#   城内尺度下这只是 5km 的绕行，看不出来；干线提到 180–240km 之后，
#   「守簇 3 的 V04 每天先从 W01 开 240km 到 T03、再开 240km 回 W01」就成了 480km 的
#   纯通勤 —— 实测单日 870min（14.5h）、成本策略更是 16h，**12h 工作日判据必然红**。
#   这不是尺度定得太狠，是「运力部署」与「职责分区」在跨城尺度下必须对齐：
#   两个中央仓库各自服务自己那一侧的簇，慢车就近驻场。
#   改动范围：只改 V03/V04 的 start_node（W01 → W02）。发车时刻、车队构成、订单表、
#   腿模型/接力/custody 全部不动。W02 是「中央仓库B」，就在簇 2/3 那一侧（W02→T02 180km、
#   W02→T03 220km，都短于从 W01 出发）。
#   为什么这是「最小改动」而不是「为了让数字好看」：它修的是**运力部署与职责分区不一致**
#   这个结构问题；如果只是把 12h 判据放宽，那才是真的粉饰（票面明令禁止）。
VEHICLES = [
    ("V01", "W01", 400, "08:00", 1.00, 1.50),
    ("V02", "W01", 160, "08:00", 1.25, 0.80),   # 守簇 1（W01 本国簇，2km 城内）
    ("V03", "W02", 160, "06:30", 1.25, 0.80),   # 守簇 2（W02 本国簇，但 V02 先占着 W01）
    ("V04", "W02", 160, "08:00", 1.25, 0.80),   # 守簇 3（离 W02 更近）
]

# 慢车的发车时刻（规格四 #54）：**V03 提前到 06:30，其余不变**。
# 为什么：V03 守簇 2，而从 W02 到簇 2 的第一批客户要走 180km 干线（135min × 1.25 = 169min）
# + 城内接驳，08:00 发车时第一批送达落在 10:53，而簇 2 有三张紧窗单
# （O13 止 09:30 / O12 止 11:00 / O11 止 10:30）⇒ 三张全超时（实测 penalty 157min）。
# 票面 §3 把「调发车时刻」列为允许的出路之一，这里用的就是它：**不改窗口、不改判据**，
# 让运力早出发去覆盖它负责的那片区域 —— 这正是「跨城尺度下排班要跟着距离走」的意思。
# 为什么只动 V03：V02（簇 1）与 V04（簇 3）在 08:00 发车时零 penalty，没有必要动它们；
# 改一台能修好问题就不改三台（最小改动）。

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
    # W02 的 y 从 500 改到 400（规格四 #54）：**只挪画布布局，不改任何城内里程**。
    # 原位置下 W02→D25 是全图最长的一条边（10.6km），大到盖过「簇内 1–6km」的口径；
    # 挪到 400 之后它回到城内尺度，尺度守卫（下面 self_check）才能把**簇内最大里程**
    # 当作唯一基准去比干线里程。中心仓库各自靠近自己的中转站也是更合理的布点。
    ("W02", "warehouse", 80, 400, "中央仓库B", 0),
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


# 跨城干线的**拓扑**（规格四 §1）。里程另由 TRUNK_KM 给定 —— 两者必须成对覆盖，
# 自检会逐条核对「干线条数 == TRUNK_KM 条数」。
# 为什么这里写死而不从坐标推：干线的存在意义是「站与站之间有高速」，属于路网设计决策；
# 由坐标推会把 4.x km 的邻居自动连上，尺度一变拓扑就变，数据不可复现。
TREE_LINKS = [
    ("W01", "T01"), ("W01", "T02"), ("W01", "T03"), ("W02", "T02"), ("W02", "T03"),
    ("T01", "T02"), ("T02", "T03"),
]


def build_links():
    """路网拓扑。**中转站只连本簇的配送点**（规格四 #54 修的真实缺陷）。

    旧实现按**编号区间**硬连（T01↔D01–D09、T02↔D10–D17、T03↔D18–D25），而配送点归属
    是按**几何最近站**算的（nearest_transit）——两套口径在 5km 尺度下只差一次绕行，
    看不出来；但干线提到 200km 之后，T01↔D05（D05 属簇 2）就成了「甲城站到乙城客户」
    的 200km 边，慢车为一张单跨城往返两次，单日路线 1022km / 1507min。
    这是**拓扑与簇归属不一致**，不是尺度问题，所以修拓扑而不是给它编个里程。

    修法：站连**本簇**配送点（与 sub_network_id 同源，不硬编码区间）。跨簇的连通性
    由下面 EXTRA_LINKS / ONE_WAY_LINKS 里那几条**客户↔客户**的跨簇边承担 ——
    它们正是用户说的「通过用户点将簇连接的路线就是国道等绕的小路」。
    """
    links = list(TREE_LINKS) + [("W01", "D01")]
    cluster_of = {}
    for i in range(1, 26):
        nid = "D%02d" % i
        x, y = DELIVERY_COORDS[nid]
        cluster_of[nid] = nearest_transit(x, y)
    station_of = {1: "T01", 2: "T02", 3: "T03"}
    for nid in sorted(cluster_of):
        links.append((station_of[cluster_of[nid]], nid))
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


# 道路等级判据：**边的两端类型**（站↔站 / 仓↔站 = 干线，含 delivery 端 = 支线）。
# 画布（src/gui/EdgeItem）与这里必须同口径 —— 两边都只用节点类型，不硬编码 ID。
def is_trunk(a, b):
    return a[0] in "WT" and b[0] in "WT"


# 节点所属簇：仓 = 0（中央仓库自成「城 0」），其余取 nodes 表的 sub_network_id。
def cluster_of_node(node_id):
    for n in NODES:
        if n[0] == node_id:
            return n[5]
    for i in range(1, 26):
        nid = "D%02d" % i
        if nid == node_id:
            return nearest_transit(DELIVERY_COORDS[nid][0], DELIVERY_COORDS[nid][1])
    raise KeyError("未知节点 %s" % node_id)


def road_class(a, b):
    """三档道路：trunk（跨城高速）/ national（跨城国道）/ city（城内小路）。
    判据只看**两端所属簇**与端点类型，不硬编码 ID —— 换数据/换簇划分后自动跟着走。

    仓要按「本国簇」算：W01 与 T01 同城、W02 与 T02 同城。若把仓一律当簇 0，
    W01↔D01（同城 1.9km）会被误判成跨城国道、凭空变成 180km —— 这正是本函数
    不能偷懒写 ca == 0 的原因。"""
    if is_trunk(a, b):
        return "trunk"
    ca, cb = cluster_of_node(a), cluster_of_node(b)
    if ca == 0:
        ca = home_cluster_of(a)
    if cb == 0:
        cb = home_cluster_of(b)
    return "city" if ca == cb else "national"


def national_km(a, b):
    key = (a, b) if a < b else (b, a)
    if key not in NATIONAL_KM:
        raise KeyError("NATIONAL_KM 缺少国道 %s <-> %s" % key)
    return NATIONAL_KM[key]


def trunk_km(a, b):
    """干线里程查表（规格四 §1）。表里没有 = 数据缺行，宁可报错也不回落到欧氏距离：
    悄悄回落会把一条本该几百公里的跨城路写回几公里，尺度守卫之外没人会发现。"""
    key = (a, b) if a < b else (b, a)
    if key not in TRUNK_KM:
        raise KeyError("TRUNK_KM 缺少干线 %s <-> %s" % key)
    return TRUNK_KM[key]


def home_cluster_of(warehouse_id):
    """仓库的「本国簇」= 唯一连到它的中转站所在簇（W01↔T01、W02↔T02）。"""
    for a, b in TREE_LINKS:
        if a == warehouse_id and a[0] == "W":
            return {n[0]: n[5] for n in NODES}[b]
        if b == warehouse_id and b[0] == "W":
            return {n[0]: n[5] for n in NODES}[a]
    raise KeyError("仓库 %s 没有连到任何中转站" % warehouse_id)


def trunk_cost_factor(a, b):
    """干线的元/km，按**两端所在的簇**分三档（规格四 §1）。W 记作簇 0。
    站↔站 -> 两簇那一档；仓↔本国站 -> 出城段（最便宜）；仓↔非本国站 -> 远端那一档
    （比站↔站更远，按低价算会让「绕仓走」凭空便宜）。"""
    sub = {n[0]: n[5] for n in NODES}
    ca, cb = sub[a], sub[b]
    if ca == 0 or cb == 0:
        wh, st = (a, b) if ca == 0 else (b, a)
        home = home_cluster_of(wh)
        if sub[st] == home:
            return WAREHOUSE_LOCAL_COST_FACTOR   # 本国站：高速入口就在城边，单价最低
        pair = (min(home, sub[st]), max(home, sub[st]))
        if pair not in HIGHWAY_COST_BY_PAIR:
            raise KeyError("HIGHWAY_COST_BY_PAIR 缺少簇对 %s" % (pair,))
        return HIGHWAY_COST_BY_PAIR[pair][1]
    pair = (min(ca, cb), max(ca, cb))
    if pair not in HIGHWAY_COST_BY_PAIR:
        raise KeyError("HIGHWAY_COST_BY_PAIR 缺少簇对 %s" % (pair,))
    return HIGHWAY_COST_BY_PAIR[pair][1]


def edge_weights(positions, a, b):
    cls = road_class(a, b)
    if cls == "trunk" and TRUNK_HIGHWAY:
        dist = trunk_km(a, b)
        return (dist, round(dist * HIGHWAY_TIME_FACTOR, 1),
                round(dist * trunk_cost_factor(a, b), 1))
    if cls == "national" and TRUNK_HIGHWAY:
        dist = national_km(a, b)
        return (dist, round(dist * NATIONAL_TIME_FACTOR, 1),
                round(dist * NATIONAL_COST_FACTOR, 1))
    dx = positions[a][0] - positions[b][0]
    dy = positions[a][1] - positions[b][1]
    dist = round(math.hypot(dx, dy) / SCALE, 1)
    # 城内两级：一端是站/仓 = 主干道（快而贵），两端都是客户 = 小路（慢而便宜）。
    hub_end = a[0] in "WT" or b[0] in "WT"
    tf = CITY_MAIN_TIME_FACTOR if hub_end else CITY_MINOR_TIME_FACTOR
    cf = CITY_MAIN_COST_FACTOR if hub_end else CITY_MINOR_COST_FACTOR
    return dist, round(dist * tf, 1), round(dist * cf, 1)


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

    # ---- 尺度守卫（规格四 §1 / #54）：**干线里程必须是跨城量级，簇内里程必须仍是城内量级**----
    # 为什么要有这条：规格四之前站间只有 4.1–5.4km，快车跑一趟的代价与城内送货同量级，
    # 「攒一车再走」毫无经济动机（实测 V01 容量利用率 1.3%）。但要修的是**尺度**，
    # 不是「把整张图放大」—— 靠改坐标会把簇内 1–6km 一起放大，城内配送语义崩掉。
    # 所以这条守卫两头都钉：干线 >= 簇内最大 × TRUNK_SCALE_MIN_RATIO，且簇内仍在 [1, 6] km。
    trunk_d = []
    national_d = []
    city_d = []
    for (a, b), (d, _t, _c) in edges.items():
        if a > b:
            continue   # 双向路两个方向权重相同，只核一条，避免同一问题报两遍
        cls = road_class(a, b)
        if cls == "trunk":
            trunk_d.append((d, a, b))
        elif cls == "national":
            national_d.append((d, a, b))
        else:
            city_d.append((d, a, b))
    branch_d = national_d + city_d   # 「非高速」的并集，费率分级与旧守卫口径沿用
    if not trunk_d:
        problems.append("没有任何干线边（仓↔站/站↔站），尺度守卫无从成立")
    # 干线的**拓扑**与**里程表**必须一一对应：路网里的一条干线没有里程，或表里有一条
    # 路网上不存在的干线，都是数据分叉。前者会让 edge_weights 直接抛错，后者会悄悄
    # 留下一条永远不会生效的参数 —— 两种都在这条守卫里点名。
    tree_pairs = set()
    for a, b in TREE_LINKS:
        tree_pairs.add((a, b) if a < b else (b, a))
    table_pairs = set(TRUNK_KM.keys())
    if tree_pairs != table_pairs:
        problems.append("干线拓扑与 TRUNK_KM 不一致：路网多 %s，里程表多 %s"
                        % (sorted(tree_pairs - table_pairs), sorted(table_pairs - tree_pairs)))
    # 国道同理：**跨簇且含配送点**的边必须逐条在表里，多一条少一条都报红。
    road_national = set()
    for (a, b) in edges:
        if a > b:
            continue
        if road_class(a, b) == "national":
            road_national.add((a, b))
    table_national = set(NATIONAL_KM.keys())
    if road_national != table_national:
        problems.append("国道拓扑与 NATIONAL_KM 不一致：路网多 %s，里程表多 %s"
                        % (sorted(road_national - table_national),
                           sorted(table_national - road_national)))
    if not city_d:
        problems.append("没有任何城内小路（含配送点），城内配送尺度无从成立")
    if trunk_d and city_d:
        # 「簇内最大里程」= 同一簇内两个配送点之间的最大距离。城内口径**只**看 city 档，
        # 国道（跨簇）与干线不参与 —— 它们是跨城量级，拿它们当基准等于自我循环。
        intra = [item for item in city_d
                 if nodes.get(item[1]) == "delivery" and nodes.get(item[2]) == "delivery"]
        max_branch = intra[0] if intra else city_d[0]
        for item in intra[1:]:
            if item[0] > max_branch[0]:
                max_branch = item
        min_trunk = trunk_d[0]
        for item in trunk_d[1:]:
            if item[0] < min_trunk[0]:
                min_trunk = item
        if min_trunk[0] < max_branch[0] * TRUNK_SCALE_MIN_RATIO:
            problems.append("尺度守卫：最短干线 %s->%s %.1fkm < 簇内最大 %.1fkm × %.0f"
                            % (min_trunk[1], min_trunk[2], min_trunk[0], max_branch[0],
                               TRUNK_SCALE_MIN_RATIO))
        for d, a, b in trunk_d:
            if d < TRUNK_MIN_KM:
                problems.append("尺度守卫：干线 %s->%s 只有 %.1fkm（应 >= %.0fkm 量级）"
                                % (a, b, d, TRUNK_MIN_KM))
        for d, a, b in national_d:
            if d < NATIONAL_MIN_KM:
                problems.append("尺度守卫：国道 %s->%s 只有 %.1fkm（应 >= %.0fkm 量级）"
                                % (a, b, d, NATIONAL_MIN_KM))
        for d, a, b in city_d:
            if d < 1.0 or d > 6.0:
                problems.append("尺度守卫：城内边 %s->%s %.1fkm 不在城内尺度 [1, 6]km 内"
                                % (a, b, d))
        # 三档必须两两可分：高速最快最贵、国道居中、城内最慢最便宜（按 min/km 与元/km 看）。
        def mean_rate(items, pick):
            s = 0.0
            for d, a, b in items:
                w = edges[(a, b)] if (a, b) in edges else edges[(b, a)]
                s += pick(w) / w[0]
            return s / len(items) if items else 0.0
        tr_t = mean_rate(trunk_d, lambda w: w[1])
        tr_c = mean_rate(trunk_d, lambda w: w[2])
        na_t = mean_rate(national_d, lambda w: w[1])
        na_c = mean_rate(national_d, lambda w: w[2])
        ci_t = mean_rate(city_d, lambda w: w[1])
        ci_c = mean_rate(city_d, lambda w: w[2])
        if not (tr_t < na_t < ci_t):
            problems.append("速度分级不成立：高速 %.3f / 国道 %.3f / 城内 %.3f min/km "
                            "应严格递增" % (tr_t, na_t, ci_t))
        # 成本分级的次序与速度**不同**：高速最贵（含通行费）> 城内次之（走走停停、
        # 单位里程油耗高）> 国道最便宜（长途干线、无通行费）。这正是「快而贵 / 慢而省」
        # 能在三策略里分出不同路线的原因：时间策略走高速、成本策略走国道。
        if not (tr_c > ci_c > na_c):
            problems.append("成本分级不成立：高速 %.3f / 城内 %.3f / 国道 %.3f 元/km "
                            "应满足 高速 > 城内 > 国道" % (tr_c, ci_c, na_c))
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

    # 职责封闭的三条数据前提（规格三 §4 / #46）。判据全部由**车队与簇**推出来，
    # 不硬编码车号或簇号：换数据时只要三条仍成立，职责封闭就仍然可执行。
    #   · 慢车数 == 簇数：每簇恰有一台慢车 ⇒ 「簇 -> 慢车」是排列数 1 的固定指派；
    #   · 快车容量 >= 2 × 每台慢车容量：站间腿（整单搬运）必须一台装得下，
    #     否则跨簇单会被迫拆成多趟，三段接力在容量上就不成立；
    #   · 两型因子分化：快车 time_factor 严格小于每台慢车、cost_factor 严格大于每台慢车。
    #     若不分化，「快车只跑站间腿」就没有经济含义（跑得一样快、一样贵，不如合并车型）。
    if vehicles:
        max_cap = max(caps)
        fast = [v for v in vehicles if abs(v[2] - max_cap) < 1e-9]
        slow = [v for v in vehicles if abs(v[2] - max_cap) >= 1e-9]
        if len(fast) != 1:
            problems.append("[vehicles] 快车（容量最大者）应恰有 1 台，实际 %d 台" % len(fast))
        cluster_ids = sorted(sub for sub in subs
                             if sub != "0" and any(m[1] == "delivery" for m in subs[sub]))
        if len(slow) != len(cluster_ids):
            problems.append("[vehicles] 慢车 %d 台 != 簇 %d 个（每簇恰有一台慢车不成立）"
                            % (len(slow), len(cluster_ids)))
        for fv in fast:
            for sv in slow:
                if fv[2] < 2.0 * sv[2] - 1e-9:
                    problems.append("[vehicles] 快车 %s 容量 %.0f < 2 × 慢车 %s 容量 %.0f"
                                    % (fv[0], fv[2], sv[0], sv[2]))
                if not (fv[4] < sv[4] and fv[5] > sv[5]):
                    problems.append("[vehicles] 两型因子未分化：快车 %s (%.2f/%.2f) 应"
                                    " time_factor 更小、cost_factor 更大（对照慢车 %s %.2f/%.2f）"
                                    % (fv[0], fv[4], fv[5], sv[0], sv[4], sv[5]))

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
          "中转站归属 / A-B 同子网络 / 总重与窗口分布 / 往返可达 / 双策略可分化 / "
          "每簇一台慢车 / 快车容量≥2×慢车 / 两型因子分化）")

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