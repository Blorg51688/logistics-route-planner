#!/usr/bin/env python3
"""核对人工测试向导里**写死的数据事实**是否与当前 config/default.ini 一致。

背景：向导的"预期"文案多次因数据调整而过期（第 3、6、7 轮都发生过），
每次都要靠人工测试撞出来。能动态取值的（规划汇总）已经改成现场取值；
剩下的是若干写死的规模与结构数字，这里逐个对照，让它们**过期即报错**。
"""
import os, re, sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WIZARD = os.path.join(ROOT, "scripts", "manual_test_wizard.sh")
CONFIG = os.path.join(ROOT, "config", "default.ini")


def read_config():
    sec, nodes, edges, orders, vehicles = None, {}, [], [], []
    for line in open(CONFIG, encoding="utf-8"):
        line = line.split("#")[0].strip()
        if not line:
            continue
        if line.startswith("["):
            sec = line[1:-1]
            continue
        f = [x.strip() for x in line.split(",")]
        if sec == "nodes":
            nodes[f[0]] = {"type": f[1], "sub": int(f[5])}
        elif sec == "edges":
            edges.append((f[0], f[1]))
        elif sec == "orders":
            orders.append({"node": f[1], "demand": float(f[2])})
        elif sec == "vehicles":
            vehicles.append({"cap": float(f[2])})
    return nodes, edges, orders, vehicles


def facts():
    nodes, edges, orders, vehicles = read_config()
    und = {(a, b) if a < b else (b, a) for a, b in edges}
    one_way = sum(1 for a, b in edges if (b, a) not in edges)
    by_type = {}
    for n in nodes.values():
        by_type[n["type"]] = by_type.get(n["type"], 0) + 1
    # 子网络 -> (中转站, 配送点数)
    subs = {}
    for nid, n in nodes.items():
        if n["sub"] == 0:
            continue
        s = subs.setdefault(n["sub"], {"transit": [], "delivery": []})
        s["transit" if n["type"] == "transit" else "delivery"].append(nid)
    demands = {}
    for o in orders:
        demands[o["node"]] = demands.get(o["node"], 0.0) + o["demand"]
    cluster = {}
    for o in orders:
        sub = nodes[o["node"]]["sub"]
        cluster[sub] = cluster.get(sub, 0.0) + o["demand"]
    return {
        "nodes": len(nodes),
        "warehouse": by_type.get("warehouse", 0),
        "delivery": by_type.get("delivery", 0),
        "transit": by_type.get("transit", 0),
        "directed": len(edges),
        "undirected": len(und),
        "oneway": one_way,
        "capacity": vehicles[0]["cap"] if vehicles else 0.0,
        "total_demand": sum(demands.values()),
        "max_order": max(demands.values()) if demands else 0.0,
        "min_cluster": min(cluster.values()) if cluster else 0.0,
        "subs": subs,
    }


def main():
    text = open(WIZARD, encoding="utf-8").read()
    f = facts()
    checks = []

    def claim(pattern, expected, desc):
        m = re.search(pattern, text)
        if not m:
            checks.append((False, "%s：向导里没找到这处声明（模式 %s）" % (desc, pattern)))
            return
        got = m.group(1)
        try:
            ok = abs(float(got) - float(expected)) < 1e-9
            shown = "%.0f" % float(expected)
        except (TypeError, ValueError):
            ok = str(got) == str(expected)
            shown = str(expected)
        checks.append((ok, "%s：向导写的是 %s，数据实际是 %s" % (desc, got, shown)))

    claim(r"(\d+) 个节点：蓝色", f["nodes"], "节点总数")
    claim(r"蓝色=仓库\((\d+)\)", f["warehouse"], "仓库数")
    claim(r"绿色=配送点\((\d+)\)", f["delivery"], "配送点数")
    claim(r"橙色=中转站\((\d+)\)", f["transit"], "中转站数")
    claim(r"至少 (\d+) 条边只有一端有箭头", f["oneway"], "单向边数")
    claim(r"载重上限 (\d+)kg", f["capacity"], "载重上限")
    first_sub = list(f["subs"].values())[0] if f["subs"] else {"delivery": []}
    claim(r"下属配送点 (\d+)/", len(first_sub["delivery"]), "第一个子网络的配送点数")
    # 这条没有捕获组，单独判断：向导写死了「子网络 1/2/3」
    checks.append(("子网络 1/2/3" in text and len(f["subs"]) == 3,
                   "向导写死「子网络 1/2/3」，但数据里有 %d 个子网络" % len(f["subs"])))

    # 载重必须同时满足：小于最小簇货量、大于最大单订单
    # 注：中转站已改为"有存货才考虑"（不再每簇必用），因此这条不再是
    # "否则中转站不参与"的理由，而是保证默认数据确实触发**多趟**。
    checks.append((f["capacity"] < f["min_cluster"],
                   "载重 %.0f 应小于最小簇货量 %.0f（否则不会触发多趟）"
                   % (f["capacity"], f["min_cluster"])))
    checks.append((f["capacity"] > f["max_order"],
                   "载重 %.0f 应大于最大单订单 %.0f" % (f["capacity"], f["max_order"])))

    fails = [c for c in checks if not c[0]]
    if fails:
        print("check_wizard_claims: %d 处向导声明与数据不符" % len(fails))
        for _, msg in fails:
            print("  - " + msg)
        sys.exit(1)
    print("check_wizard_claims: 向导里的 %d 处数据声明均与当前数据一致" % len(checks))
    print("  （节点 %d / 有向边 %d / 无向道路 %d / 单向 %d / 载重 %.0f / "
          "最小簇货量 %.0f / 最大单订单 %.0f）"
          % (f["nodes"], f["directed"], f["undirected"], f["oneway"], f["capacity"],
             f["min_cluster"], f["max_order"]))


if __name__ == "__main__":
    main()
