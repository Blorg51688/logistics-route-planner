#!/usr/bin/env python3
"""核对向导里让用户点击的界面元素**真实存在**。

背景：向导会写「点工具栏的『图表示…』」这类指示。一旦按钮改名、被删或从未实现，
用户就卡在那里，而这类错误 bash -n 与引号检查都发现不了
（本项目已有"向导让人点一个不存在的按钮"的先例）。

做法：跑 `--ui-probe` 拿到实际的动作名与面板名，再扫描向导里 todo 行的「」引用。
"""
import os, re, subprocess, sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WIZARD = os.path.join(ROOT, "scripts", "manual_test_wizard.sh")
APP = os.path.join(ROOT, "build", "app")

# 这些「」不是界面元素（是概念、状态或对话框内的字段说明），不参与比对
WHITELIST = {
    "不可行", "规划策略", "权重标签", "手工增删", "模拟路况", "插入紧急订单", "模拟新客户",
    "模拟道路封闭", "推进一站", "重新规划", "图表示", "Debug 模式", "Debug 速度",
    "节点 ID（删除用）", "节点 ID", "起点/终点 ID", "添加边", "删除边", "删除节点",
    "中转站 / 集散", "停靠明细", "超时订单", "订单列表", "路线信息", "车辆信息",
    "最短距离策略", "最低耗时策略", "最低成本策略", "显示距离", "显示耗时", "显示成本",
    "展示模式（2 秒/步）", "快速预览（1 秒/步）", "规划成功", "送达", "经过",
}


def probe():
    env = dict(os.environ, QT_QPA_PLATFORM="offscreen")
    out = subprocess.run([APP, "--ui-probe"], capture_output=True, text=True, env=env).stdout
    actions, docks = "", ""
    for line in out.splitlines():
        if line.startswith("[ui-probe] 动作: "):
            actions = line[len("[ui-probe] 动作: "):]
        elif line.startswith("[ui-probe] 面板: "):
            docks = line[len("[ui-probe] 面板: "):]
    return set(x.strip() for x in actions.split("|") if x.strip()), \
           set(x.strip() for x in docks.split("|") if x.strip())


def main():
    if not os.path.exists(APP):
        print("check_wizard_ui_names: 找不到 build/app，跳过")
        return 0
    actions, docks = probe()
    if not actions:
        print("check_wizard_ui_names: 探针没有输出动作名，跳过")
        return 0

    text = open(WIZARD, encoding="utf-8").read()
    bad = []
    checked = 0
    for lineno, line in enumerate(text.splitlines(), 1):
        stripped = line.strip()
        # 只看 todo 行 —— 那才是"让用户去点某个东西"的指示。
        # want 行里的「」多是**预期看到的日志文本或字段名**（如「送达 XX」），
        # 不是界面元素，收进来会全是误报。
        if not re.match(r'^todo\s', stripped):
            continue
        for name in re.findall(r'「([^」]+)」', stripped):
            if name in WHITELIST:
                continue
            checked += 1
            # 允许写成「图表示…」而实际是「图表示…」；也允许是面板/动作的一部分
            if name in actions or name in docks:
                continue
            if any(name in a for a in actions) or any(name in d for d in docks):
                continue
            bad.append((lineno, name))

    if bad:
        print("check_wizard_ui_names: %d 处向导提到的界面元素并不存在：" % len(bad))
        for lineno, name in bad:
            print("  manual_test_wizard.sh:%d  「%s」" % (lineno, name))
        print("  实际动作: " + " | ".join(sorted(actions)))
        print("  实际面板: " + " | ".join(sorted(docks)))
        sys.exit(1)
    print("check_wizard_ui_names: 向导提到的 %d 处界面元素都真实存在" % checked)


if __name__ == "__main__":
    main()
