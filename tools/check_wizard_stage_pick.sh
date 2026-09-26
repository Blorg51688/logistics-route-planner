#!/usr/bin/env bash
# 守护人工测试向导的"关卡选择"逻辑（输入 0 跑全部 / 关号 / 关名 / 片段）。
# 直接 source 向导脚本，只加载函数、不进入交互。
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=/dev/null
source "$ROOT/scripts/manual_test_wizard.sh"

fail=0
expect() {  # expect <输入> <期望关卡名，空串=全部；! 前缀表示"应无法识别">
    local q="$1" want="$2" got err log
    log="$(mktemp)"
    got="$(resolve_stage_input "$q" 2>"$log")"
    err="$(cat "$log")"
    rm -f "$log"

    # 必须区分"干净地判定为无法识别"与"函数崩溃了"：
    # 后者同样是非零退出，不查错误输出就会把崩溃当成正确行为。
    if [[ -n "$err" ]]; then
        echo "FAIL  「$q」产生了错误输出（多半是越界或崩溃）：${err%%$'\n'*}"
        fail=1
        return
    fi
    if [[ "$want" == "!"* ]]; then
        if [[ -n "$got" ]]; then
            echo "FAIL  「$q」本应无法识别，却解析成了「$got」"
            fail=1
        fi
        return
    fi
    if [[ "$got" != "$want" ]]; then
        echo "FAIL  「$q」解析为「$got」，期望「$want」"
        fail=1
    fi
}

first="${STAGE_NAMES[0]}"
third="${STAGE_NAMES[2]}"
last="${STAGE_NAMES[$((${#STAGE_NAMES[@]} - 1))]}"

expect "0" ""
expect "" ""
expect "1" "$first"
expect "3" "$third"
expect "$((${#STAGE_NAMES[@]}))" "$last"
expect "$((${#STAGE_NAMES[@]} + 1))" "!"
expect "99" "!"
expect "$third" "$third"
expect "不存在的关卡" "!"

# --only 传错关卡名时，必须**直接报"无法识别"**，
# 而不是被"非终端"检查抢先拦截（那会让人以为参数是对的、只是环境不对）。
out="$(bash "$ROOT/scripts/manual_test_wizard.sh" --only 不存在的关卡 </dev/null 2>&1 || true)"
if ! grep -q "无法识别" <<<"$out"; then
    echo "FAIL  --only 传错关卡名时没有报'无法识别'（可能被终端检查抢先了）"
    echo "      实际输出最后两行：$(tail -2 <<<"$out" | tr '\n' ' ')"
    fail=1
fi

# 静态检查：每个 do_stage_N() 内部必须真的有 stage_wanted 守卫。
# 之前"只跑一关"整个失效，就是因为 stage_wanted 函数写好了却没有任何一关调用它——
# 光测 resolve_stage_input 是发现不了的。
wiz="$ROOT/scripts/manual_test_wizard.sh"
n_funcs="$(grep -cE '^do_stage_[0-9]+\(\) \{$' "$wiz")"
n_guards="$(grep -cE '^    stage_wanted "' "$wiz")"
if [[ "$n_funcs" != "$n_guards" ]]; then
    echo "FAIL  关卡函数 $n_funcs 个，但带 stage_wanted 守卫的只有 $n_guards 个"
    echo "      （守卫缺失时'只跑一关'会退化成跑完全程）"
    fail=1
fi
if (( n_funcs != ${#STAGE_NAMES[@]} )); then
    echo "FAIL  注册表 ${#STAGE_NAMES[@]} 关，但只有 $n_funcs 个关卡函数"
    fail=1
fi

if [[ "$TOTAL_STAGES" != "${#STAGE_NAMES[@]}" ]]; then
    echo "FAIL  总关数 TOTAL_STAGES=$TOTAL_STAGES 与注册表 ${#STAGE_NAMES[@]} 关不一致"
    fail=1
fi

# ---------------------------------------------------------------------------
# 记录忠实性：声称覆盖的关数必须等于用户真正看到的表格行数。
#
# 缺陷背景：write_record 曾用硬编码总关数写"共 8 关"，且只要 FAIL==0 且
# SKIP==0 就无条件打印"全部关卡通过"。于是 --only 单关模式实际只跑 1 关，
# 产出的 docs/人工测试记录.md（要交给教师当证据）却写着"共 8 关 / 全部关卡通过"。
#
# 这里直接构造 RESULTS_* 驱动 write_record 落临时文件，再**读回这份用户可见的
# 记录**做断言——不重算脚本内部的公式，防止守卫只验证"我以为的算法"。
# ---------------------------------------------------------------------------

reset_results() {
    RESULTS_NAME=(); RESULTS_STATE=(); RESULTS_NOTE=()
    PASS=0; FAIL=0; SKIP=0; ONLY_STAGE=""
}

# 用户可见表格的数据行数：表头与分隔行不算
record_rows() { grep -cE '^\| [0-9]+ \|' "$1"; }
# 用户可见的"本次执行 N 关"声明值；没有该字样的旧格式返回空
record_declared_executed() {
    sed -n 's/.*本次执行 \([0-9][0-9]*\) 关.*/\1/p' "$1" | head -1
}

live_commit="$(git -C "$ROOT" rev-parse --short HEAD 2>/dev/null || echo '未知')"

check_declared_matches_rows() {  # check_declared_matches_rows <记录文件> <标签>
    local rec="$1" tag="$2" declared rows
    declared="$(record_declared_executed "$rec")"
    rows="$(record_rows "$rec")"
    if [[ -z "$declared" ]]; then
        echo "FAIL  $tag 记录里没有「本次执行 N 关」字样（旧格式？）：$(grep -m1 "^- 结果" "$rec")"
        fail=1
        return
    fi
    if [[ "$declared" != "$rows" ]]; then
        echo "FAIL  $tag 记录声称「本次执行 ${declared} 关」，但表格只有 ${rows} 行"
        fail=1
    fi
}

# 场景 A：单关模式（--only 第 7 关），实际只执行 1 关 -> 绝不得说"全部关卡通过"
reset_results
RECORD="$(mktemp)"
ONLY_STAGE="${STAGE_NAMES[6]}"
RESULTS_NAME+=("$ONLY_STAGE"); RESULTS_STATE+=("通过"); RESULTS_NOTE+=(""); PASS=1
write_record
if grep -q "全部关卡通过" "$RECORD"; then
    echo "FAIL  单关模式（实际执行 1 关）的记录仍声称「全部关卡通过」——字面误导"
    fail=1
fi
if ! grep -q "本次仅执行第 7 关（${ONLY_STAGE}）" "$RECORD"; then
    echo "FAIL  单关模式记录未写明「本次仅执行第 7 关（${ONLY_STAGE}）」"
    echo "      实际：$(sed -n '5p' "$RECORD")"
    fail=1
fi
if ! grep -qF -- "- 提交：$live_commit" "$RECORD"; then
    echo "FAIL  记录里的「提交」不是现场 git rev-parse 的 $live_commit"
    fail=1
fi
check_declared_matches_rows "$RECORD" "单关模式"
rm -f "$RECORD"

# 场景 B：全量模式，实际执行关数 == 总关数 -> 必须仍然说"全部关卡通过"
reset_results
RECORD="$(mktemp)"
for n in "${STAGE_NAMES[@]}"; do
    RESULTS_NAME+=("$n"); RESULTS_STATE+=("通过"); RESULTS_NOTE+=("")
done
PASS=${#STAGE_NAMES[@]}
write_record
if ! grep -q "全部关卡通过" "$RECORD"; then
    echo "FAIL  全量模式（跑满 ${#STAGE_NAMES[@]} 关）的记录不再说「全部关卡通过」——正常路径被改坏"
    fail=1
fi
if [[ "$(record_declared_executed "$RECORD")" != "${#STAGE_NAMES[@]}" ]]; then
    echo "FAIL  全量模式记录未声称「本次执行 ${#STAGE_NAMES[@]} 关」"
    fail=1
fi
check_declared_matches_rows "$RECORD" "全量模式"
rm -f "$RECORD"

# 场景 C：非单关的中途中断（执行 3 关就落盘）也不得说"全部关卡通过"
reset_results
RECORD="$(mktemp)"
for i in 0 1 2; do
    RESULTS_NAME+=("${STAGE_NAMES[$i]}"); RESULTS_STATE+=("通过"); RESULTS_NOTE+=("")
done
PASS=3
write_record
if grep -q "全部关卡通过" "$RECORD"; then
    echo "FAIL  只执行 3/$((${#STAGE_NAMES[@]})) 关的记录仍声称「全部关卡通过」"
    fail=1
fi
check_declared_matches_rows "$RECORD" "部分执行"
rm -f "$RECORD"

# 清空结果数组，避免 EXIT trap 里的 cleanup 又写一份记录
reset_results
RECORD=""

if [[ "$fail" -eq 0 ]]; then
    echo "check_wizard_stage_pick: 关卡选择逻辑与记录覆盖声明正常（共 ${#STAGE_NAMES[@]} 关）"
fi
exit "$fail"
