#!/usr/bin/env python3
"""检查人工测试向导里的文案行是否有"字符串被引号截断"的问题。

背景：本文件多次出现把中文引号误写成英文 `"` 的情况，
例如  want "**不显示"不可行"**"  ——  语法合法（bash -n 通过），
但实际传给函数的是两个参数，标题/正文被静默截断。
这类错误语法检查抓不到，只能用"引号计数"来发现。
"""
import re, sys, os

PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                    "scripts", "manual_test_wizard.sh")
# record / ask_result 本来就是多参数调用（"名字" "结果" "备注"），不参与检查
CMDS = ("want", "dim", "say", "todo", "head1", "stage")

bad = []
for lineno, line in enumerate(open(PATH, encoding="utf-8"), 1):
    stripped = line.strip()
    m = re.match(r'^(%s)\s+(.*)$' % "|".join(CMDS), stripped)
    if not m:
        continue
    cmd, rest = m.group(1), m.group(2)
    # 去掉行尾注释与续行反斜杠后再计数
    body = rest.rstrip()
    if body.count('"') != 2:
        # 允许 0 个（无参调用形式），其余一律可疑
        if body.count('"') != 0:
            bad.append((lineno, cmd, stripped))

if bad:
    print("发现 %d 行文案的引号数量异常（字符串会被截断）：" % len(bad))
    for lineno, cmd, text in bad:
        print("  %s:%d  %s" % (os.path.basename(PATH), lineno, text))
    sys.exit(1)
print("check_wizard_text: 未发现引号截断问题")
