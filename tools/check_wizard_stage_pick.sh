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

if [[ "$fail" -eq 0 ]]; then
    echo "check_wizard_stage_pick: 关卡选择逻辑正常（共 ${#STAGE_NAMES[@]} 关）"
fi
exit "$fail"
