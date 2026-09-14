#!/usr/bin/env bash
#
# 人工测试向导 —— 电商物流配送路径规划系统
#
# 作用：逐关引导人工完成**只有人能做**的验证（查看图形界面、拖动节点、点击按钮），
#       并在最后一关汇总结果、写入 docs/人工测试记录.md。
#
# 它**不做**的：不采集任何密钥或环境变量（本流程没有这类值），
#              不写任何报告内容，不修改交付代码。
#
# 为什么需要人工：单元/集成测试（624 项）覆盖不了 GUI 的观感与交互——
#   · 箭头是否画对、标签是否遮挡、拖动节点时边是否跟随
#   · 点击按钮后的界面反馈是否符合直觉
#   这些只能靠人看。本项目在开发中已经因此抓到一个真实缺陷
#   （箭头几何被画进节点内部而完全不可见）。
#
# 用法：  bash scripts/manual_test_wizard.sh
# 可重复运行，每次覆盖上一份记录。

set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT" || exit 1

RECORD="docs/人工测试记录.md"
APP="./build/app"
TOTAL_STAGES=18
STAGE=0
APP_PID=""

RESULTS_NAME=()
RESULTS_STATE=()
RESULTS_NOTE=()

PASS=0
FAIL=0
SKIP=0

# ============================ 库部分（保持稳定，勿改） ============================

if [[ -t 1 ]]; then
    BOLD=$'\033[1m'; DIM=$'\033[2m'; RED=$'\033[31m'
    GREEN=$'\033[32m'; YELLOW=$'\033[33m'; CYAN=$'\033[36m'; RESET=$'\033[0m'
else
    BOLD=""; DIM=""; RED=""; GREEN=""; YELLOW=""; CYAN=""; RESET=""
fi

say()  { printf '%s\n' "$*"; }
dim()  { printf '%s%s%s\n' "$DIM" "$*" "$RESET"; }
head1() { printf '%s%s%s\n' "$BOLD$CYAN" "$*" "$RESET"; }

pause() {
    printf '\n%s按回车继续…%s' "$DIM" "$RESET"
    read -r _
}

confirm() {   # confirm "问题" -> 0 表示是
    local answer
    printf '%s [y/N] ' "$1"
    read -r answer
    [[ "$answer" == "y" || "$answer" == "Y" ]]
}

record() {    # record <关卡名> <结果> [备注]
    RESULTS_NAME+=("$1")
    RESULTS_STATE+=("$2")
    RESULTS_NOTE+=("${3:-}")
    case "$2" in
        通过) PASS=$((PASS + 1)) ;;
        失败) FAIL=$((FAIL + 1)) ;;
        跳过) SKIP=$((SKIP + 1)) ;;
    esac
}

stage() {     # stage <标题> —— 清屏并显示"第 N / TOTAL 关"
    STAGE=$((STAGE + 1))
    clear 2>/dev/null || true
    printf '%s════════════════════════════════════════════════════════════%s\n' "$CYAN" "$RESET"
    printf '%s  第 %d / %d 关   %s%s\n' "$BOLD$CYAN" "$STAGE" "$TOTAL_STAGES" "$1" "$RESET"
    printf '%s════════════════════════════════════════════════════════════%s\n\n' "$CYAN" "$RESET"
}

ask_result() {   # ask_result <关卡名> —— 三选一；失败时追问备注
    local name="$1" choice note=""
    while true; do
        printf '\n%s结果：[1] 通过  [2] 失败  [3] 跳过%s\n> ' "$BOLD" "$RESET"
        read -r choice
        case "$choice" in
            1) record "$name" "通过"; return ;;
            2)
                printf '请简述失败现象（回车结束）：\n> '
                read -r note
                record "$name" "失败" "$note"
                return ;;
            3)
                printf '跳过原因（回车结束）：\n> '
                read -r note
                record "$name" "跳过" "$note"
                return ;;
            *) printf '%s请输入 1、2 或 3%s\n' "$YELLOW" "$RESET" ;;
        esac
    done
}

open_path() {   # open_path <文件> —— 跨平台打开，失败不影响流程
    local path="$1"
    [[ -e "$path" ]] || return 0
    if command -v xdg-open >/dev/null 2>&1; then
        xdg-open "$path" >/dev/null 2>&1 &
    elif command -v gio >/dev/null 2>&1; then
        gio open "$path" >/dev/null 2>&1 &
    elif command -v wslview >/dev/null 2>&1; then
        wslview "$path" >/dev/null 2>&1 &
    elif command -v open >/dev/null 2>&1; then
        open "$path" >/dev/null 2>&1 &
    else
        dim "（未能自动打开，请手动查看 $path）"
    fi
}

launch_app() {   # launch_app [额外参数...] —— 后台启动，返回是否成功
    if ! [[ -n "${DISPLAY:-}" || -n "${WAYLAND_DISPLAY:-}" ]]; then
        printf '%s没有检测到图形显示环境（DISPLAY / WAYLAND_DISPLAY 均为空）。%s\n' "$RED" "$RESET"
        say "本向导需要在能显示窗口的桌面上运行。"
        return 1
    fi
    "$APP" "$@" >/dev/null 2>&1 &
    APP_PID=$!
    sleep 1
    if ! kill -0 "$APP_PID" 2>/dev/null; then
        printf '%s程序启动后立即退出，请先在终端运行 %s 查看报错。%s\n' "$RED" "$APP" "$RESET"
        APP_PID=""
        return 1
    fi
    return 0
}

stop_app() {
    if [[ -n "$APP_PID" ]] && kill -0 "$APP_PID" 2>/dev/null; then
        kill "$APP_PID" 2>/dev/null || true
        wait "$APP_PID" 2>/dev/null || true
    fi
    APP_PID=""
}

cleanup() { stop_app; }
trap cleanup EXIT INT TERM

write_record() {
    local stamp commit
    stamp="$(date '+%Y-%m-%d %H:%M')"
    commit="$(git rev-parse --short HEAD 2>/dev/null || echo '未知')"

    mkdir -p "$(dirname "$RECORD")"
    {
        printf '# 人工测试记录\n\n'
        printf -- '- 时间：%s\n' "$stamp"
        printf -- '- 提交：%s\n' "$commit"
        printf -- '- 结果：**通过 %d / 失败 %d / 跳过 %d**（共 %d 关）\n\n' \
               "$PASS" "$FAIL" "$SKIP" "$TOTAL_STAGES"
        if [[ "$FAIL" -eq 0 && "$SKIP" -eq 0 ]]; then
            printf -- '> 全部关卡通过。\n\n'
        elif [[ "$FAIL" -eq 0 ]]; then
            printf -- '> 无失败，但有 %d 关被跳过。\n\n' "$SKIP"
        else
            printf -- '> **存在失败关卡，需修复后重测。**\n\n'
        fi
        printf '| # | 关卡 | 结果 | 备注 |\n|---|---|---|---|\n'
        local i
        for i in "${!RESULTS_NAME[@]}"; do
            printf '| %d | %s | %s | %s |\n' \
                   "$((i + 1))" "${RESULTS_NAME[$i]}" "${RESULTS_STATE[$i]}" "${RESULTS_NOTE[$i]}"
        done
        printf '\n---\n\n*本文件由 `scripts/manual_test_wizard.sh` 生成，是人工测试证据，非课程报告内容。*\n'
    } > "$RECORD"

    say ""
    head1 "结果已写入 $RECORD"
}

summary() {
    clear 2>/dev/null || true
    head1 "════════ 人工测试结束 ════════"
    say ""
    local i
    for i in "${!RESULTS_NAME[@]}"; do
        local color="$GREEN" mark="✓"
        [[ "${RESULTS_STATE[$i]}" == "失败" ]] && { color="$RED"; mark="✗"; }
        [[ "${RESULTS_STATE[$i]}" == "跳过" ]] && { color="$YELLOW"; mark="-"; }
        printf '  %s%s%s %2d. %-34s %s\n' "$color" "$mark" "$RESET" \
               "$((i + 1))" "${RESULTS_NAME[$i]}" "${RESULTS_STATE[$i]}"
    done
    say ""
    printf '  %s通过 %d   %s失败 %d   %s跳过 %d%s\n' \
           "$GREEN" "$PASS" "$RED" "$FAIL" "$YELLOW" "$SKIP" "$RESET"
    say ""
    write_record
    if [[ "$FAIL" -gt 0 ]]; then
        printf '\n%s存在失败关卡。请把失败现象的备注反馈出来，修复后重跑本向导。%s\n' "$RED" "$RESET"
    fi
}

# ============================ 以下是本流程的关卡 ============================

main() {
    clear 2>/dev/null || true
    head1 "人工测试向导 —— 电商物流配送路径规划系统"
    say ""
    say "本向导共 $TOTAL_STAGES 关：第 1 关自动检查，其余需要你亲自看界面、点按钮。"
    say "全程约 10–15 分钟。程序窗口与终端之间来回切换即可。"
    say ""
    dim "提示：每一关都会清屏并只显示当前这一关；切回终端时它仍在屏幕上。"
    say ""
    if [[ ! -t 0 ]]; then
        printf '%s当前 stdin 不是终端，无法交互。请在终端里直接运行：%s\n' "$RED" "$RESET"
        say "  bash scripts/manual_test_wizard.sh"
        exit 2
    fi
    confirm "准备好了吗？" || { say "已取消。"; exit 0; }

    # ---------------------------------------------------------------- 第 1 关
    stage "构建与环境检查（自动）"
    say "正在构建项目…"
    say ""
    local build_ok=1
    if cmake -S . -B build >/tmp/wizard_cmake.log 2>&1 \
       && cmake --build build >/tmp/wizard_build.log 2>&1; then
        say "${GREEN}构建成功${RESET}"
    else
        build_ok=0
        printf '%s构建失败，日志：/tmp/wizard_build.log%s\n' "$RED" "$RESET"
        tail -20 /tmp/wizard_build.log
    fi

    if [[ "$build_ok" -eq 1 ]]; then
        say ""
        say "快速自检（单元 + 集成 + 数据 + 预言机）："
        if (cd build && ctest >/tmp/wizard_ctest.log 2>&1); then
            say "${GREEN}ctest 全部通过${RESET}"
            grep -E "tests passed" /tmp/wizard_ctest.log | tail -1
        else
            build_ok=0
            printf '%sctest 有失败项：%s\n' "$RED" "$RESET"
            grep -E "Failed|failed" /tmp/wizard_ctest.log | head -10
        fi
    fi

    if [[ "$build_ok" -ne 1 ]]; then
        record "构建与环境检查" "失败" "构建或自检未通过"
        summary
        exit 1
    fi
    if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
        record "构建与环境检查" "失败" "无图形显示环境"
        summary
        exit 1
    fi
    record "构建与环境检查" "通过" "构建与 ctest 均通过，图形环境可用"
    say ""
    say "${GREEN}第 1 关通过。${RESET}"
    pause

    # ------------------------------------------------ 第 2–14 关：一次启动
    stage "启动程序（后续第 3–14 关都在这一次会话里做）"
    say "即将启动程序。请保持它开着，做完一关就切回终端按回车。"
    say ""
    if ! confirm "现在启动？"; then
        record "启动程序" "跳过" "用户取消启动"
        summary
        exit 0
    fi
    if ! launch_app; then
        record "启动程序" "失败" "程序未能启动"
        summary
        exit 1
    fi
    sleep 1
    say "${GREEN}程序已启动。${RESET}"
    pause

    # 第 3 关
    stage "初始网络图（B1 / B2 / B4）"
    say "看程序窗口中的配送网络图。"
    say ""
    head1 "预期："
    say "  · 共 30 个节点：蓝色圆点 = 仓库（2 个），绿色 = 配送点（25 个），橙色 = 中转站（3 个）"
    say "  · 每个节点下方有名称（中央仓库A、客户01、中转站甲…）"
    say "  · 每条边都是连线的两端各带一个箭头（有向图，双向边两端都有箭头）"
    say "  · 没有一个箭头被画进节点圆圈内部而看不见"
    say ""
    dim "参考：docs/screenshots/gui-network.png"
    open_path "docs/screenshots/gui-network.png"
    pause
    ask_result "初始网络图（B1/B2/B4）"

    # 第 4 关
    stage "初始规划与路径高亮（B6 / B7）"
    say "看画布上的红色粗线，以及右侧「路线信息」栏。"
    say ""
    head1 "预期："
    say "  · 一条红色粗线串起全部 25 个配送点，从 W01 出发并回到 W01"
    say "  · 红色边上有白底的权重标签；灰色边不显示标签"
    say "  · 路线信息：总距离约 262.2 km、总成本约 290.8 元、停靠 25 站"
    pause
    ask_result "初始规划与路径高亮（B6/B7）"

    # 第 5 关
    stage "双策略分化（B7）"
    say "把工具栏最左边的「规划策略」下拉切到「最低成本策略」。"
    say ""
    head1 "预期："
    say "  · 路由高亮发生变化（不再与最短距离策略相同）"
    say "  · 总距离变大到约 288.2 km，总成本变小到约 288.2 元"
    say "  · 两者互换优劣：距离策略更短、成本策略更便宜"
    say ""
    dim "若两种策略给出完全一样的结果，说明数据或算法退化，属失败。"
    say ""
    dim "看完可以把策略切回「最短距离策略」再做后面的关卡。"
    pause
    ask_result "双策略分化（B7）"

    # 第 6 关
    stage "权重标签切换（B4）"
    say "把工具栏的「权重标签」下拉依次切到「显示耗时」和「显示成本」。"
    say ""
    head1 "预期："
    say "  · 边上标签的数值随之改变：例如同一条边 5.2km → 约 8min → 约 6元"
    say "  · 高亮的路线本身**不重算、不变化**（只换显示文本）"
    say ""
    dim "切回「显示距离」后继续。"
    pause
    ask_result "权重标签切换（B4）"

    # 第 7 关
    stage "拖动节点与边联动（GUI §7）"
    say "用鼠标按住任意一个节点（例如某个绿色的客户点）拖动一段距离，再松手。"
    say ""
    head1 "预期："
    say "  · 与该节点相连的边**实时跟随**移动"
    say "  · 箭头仍然贴在节点边界上，没有飘在空中或被埋进圆点"
    say "  · 权重标签也跟着移动"
    pause
    ask_result "拖动节点与边联动（GUI §7）"

    # 第 8 关
    stage "时间窗与超时 penalty（E2）"
    say "看右侧「超时订单」栏。"
    say ""
    head1 "预期："
    say "  · 只列出 2 个超时站点：D12 与 D05"
    say "  · 每行给出到达时刻与 penalty 分钟数"
    say "  · 其余 23 个站点准时 —— 超时应是**少数**、且只出现在窗口很紧的那两站"
    say ""
    dim "若大量站点都超时，说明默认数据或时间模型有问题，属失败。"
    pause
    ask_result "时间窗与超时 penalty（E2）"

    # 第 9 关
    stage "模拟路况与自动重规划（E1）"
    say "点工具栏的「模拟路况」。"
    say ""
    head1 "预期："
    say "  · 底部日志新增一行「路况变化：改动 N 条边」（N 约为 72 的 10%，即 7 条）"
    say "  · 若受影响的边正好在当前路径上且耗时增幅 ≥ 20%，紧跟一行"
    say "    「受影响边位于当前路径且增幅达标 -> 自动触发重规划」"
    say "  · 触发重规划后，红色高亮与「总耗时」会更新"
    say "  · 若受影响边不在路径上或增幅不足，则提示「不触发重规划」"
    say ""
    dim "两种结果都算通过 —— 重点是判断逻辑正确、日志说清了原因。"
    pause
    ask_result "模拟路况与自动重规划（E1）"

    # 第 10 关
    stage "插入紧急订单（E3）"
    say "点工具栏的「插入紧急订单」。"
    say ""
    head1 "预期："
    say "  · 日志出现「插入紧急订单 <订单号> @ <配送点>」"
    say "  · 该配送点在新的高亮路线里被**优先服务**（排在前面的停靠点）"
    say "  · 若该单窗口必然无法满足，会额外给出一行 ⚠ 警告，但订单**仍被纳入路线**"
    say "    （本项目口径：超时不弃，只记 penalty）"
    pause
    ask_result "插入紧急订单（E3）"

    # 第 11 关
    stage "模拟新客户（B5）"
    say "点工具栏的「模拟新客户」。"
    say ""
    head1 "预期："
    say "  · 画布上多出一个新节点（编号形如 X001、X002…，绿色配送点）"
    say "  · 它连到距离最近的既有节点，**两个方向都有边**"
    say "  · 日志出现「模拟新客户：新增配送点 X###」"
    say "  · 随后自动重新规划成功（日志「规划成功：… 站」）"
    pause
    ask_result "模拟新客户（B5）"

    # 第 12 关
    stage "模拟道路封闭（B5）—— 重点回归关卡"
    say "点工具栏的「模拟道路封闭」。"
    say ""
    head1 "预期："
    say "  · 日志出现「模拟道路封闭：<A>-><B>（含反向）」"
    say "  · 对应的边（含反向）从画布上消失"
    say "  · **仍然「规划成功」** —— 不会出现「无法从 … 到达 …」的不可行"
    say ""
    dim "这一关守着一个真实缺陷：早先封路会把网络切断（把某个配送点变成孤岛），"
    dim "一次点击就把程序锁死在不可行状态。现已改为不封闭「桥」。"
    say ""
    dim "若出现「规划不可行」，属失败，请记下日志原文。"
    pause
    ask_result "模拟道路封闭（B5）"

    # 第 13 关
    stage "手工增删节点 / 边（B5）"
    say "点工具栏的「手工增删…」打开对话框，依次试三件事："
    say ""
    head1 "预期："
    say "  ① 在「起点 ID / 终点 ID」填两个尚未直连的既有节点 → 点「添加边」"
    say "     · 日志「手工添加边 A -> B」，画布出现该边并重新规划"
    say "  ② 用同一对 ID 点「删除边」"
    say "     · 日志「手工删除边 A <-> B」，该边（含反向）消失"
    say "  ③ 在「节点 ID」填一个**不存在的** ID（例如 NOPE）→ 点「删除节点」"
    say "     · 日志报「删除节点失败：NOPE 不存在」"
    say "     · 程序**不得崩溃或卡死**"
    say ""
    dim "做完用对话框的 Close 关闭。"
    pause
    ask_result "手工增删节点 / 边（B5）"

    # 第 14 关
    stage "推进一站与当前位置追踪（B6）—— 重点回归关卡"
    say "点几次工具栏的「推进一站」（4–5 次），然后点一次「重新规划」。"
    say ""
    head1 "预期："
    say "  · 每次推进，日志记录「送达 XX（到达 时:分）」"
    say "  · 右侧「订单列表」中对应订单的状态变成「已送达」"
    say "  · 点「重新规划」后，**已送达的站点不再出现在新路线里**"
    say "  · 「总 penalty」保持在一个合理量级（几百分钟），**不应暴涨到几千**"
    say ""
    dim "这一关守着另一个真实缺陷：重规划会用全新计划覆盖旧计划，而站点计数"
    dim "仍指向旧计划，导致取到错误时刻、penalty 暴涨（实测曾达 5322min）。"
    say ""
    dim "若 penalty 变成四位数或更大，属失败，请记下数值。"
    pause
    ask_result "推进一站与当前位置追踪（B6）"

    # 第 15 关
    stage "Debug 模式（A1）—— 重点回归关卡"
    say "勾选工具栏最右边的「Debug 模式」，让它自动跑一会儿；然后**取消勾选**。"
    say ""
    head1 "预期："
    say "  · 勾选后日志**持续增长**：会自动模拟路况、偶尔插入紧急订单、自动推进送达"
    say "  · 画布上的高亮路线随之变化"
    say "  · **取消勾选后必须立刻停下来** —— 日志不再新增，界面不再变化"
    say ""
    dim "这一关守着 Debug 模式必须「可随时关闭」。若取消勾选后仍在刷日志，属失败。"
    pause
    ask_result "Debug 模式（A1）"

    stop_app
    say ""
    dim "已关闭程序，准备下一关。"
    pause

    # 第 16 关
    stage "载重约束（E4）"
    say "本关用一份「载重被改小」的配置启动程序，验证超载时明确不可行。"
    say ""
    mkdir -p build
    if sed 's/^V01, W01, 800, 08:00$/V01, W01, 100, 08:00/' config/default.ini \
         > build/test_capacity.ini 2>/dev/null; then
        dim "已生成 build/test_capacity.ini（载重上限 800 -> 100 kg）"
    else
        printf '%s生成测试配置失败%s\n' "$RED" "$RESET"
    fi
    say ""
    if confirm "现在用该配置启动程序？"; then
        if launch_app --config build/test_capacity.ini; then
            sleep 1
            say ""
            head1 "预期："
            say "  · 右侧「路线信息」显示「不可行：总需求 740kg 超过载重上限 100kg」"
            say "  · 画布上没有任何红色高亮路线"
            say "  · 程序不崩溃"
            pause
        else
            record "载重约束（E4）" "失败" "未能用测试配置启动程序"
            summary
            exit 1
        fi
        ask_result "载重约束（E4）"
        stop_app
    else
        record "载重约束（E4）" "跳过" "用户取消"
    fi
    pause

    # 第 17 关
    stage "图表示输出：邻接表 / 邻接矩阵（B3）"
    say "本关在命令行完成，向导直接运行给你看。"
    say ""
    if "$APP" --dump-graph both >/tmp/wizard_dump.txt 2>/dev/null; then
        dim "输出已保存到 /tmp/wizard_dump.txt"
        say ""
        head1 "邻接表（前 3 行）："
        sed -n '1,3p' /tmp/wizard_dump.txt
        say ""
        head1 "邻接矩阵（表头 + 第 1 行）："
        grep -n "ID" /tmp/wizard_dump.txt | head -1
        say ""
        head1 "预期："
        say "  · 邻接表：30 行，每行形如 W01(warehouse,中央仓库A): -> T01(7.6km,7.6min,12.2元) …"
        say "  · 邻接矩阵：1 行列标 + 30 行数据，共 31×31 个单元格"
        say "  · 矩阵中缺边位置显示 -，对角线也全是 -（无自环）"
        say "  · 数值与画布上看到的边一致"
        say ""
        printf '%s完整内容可执行：%s less /tmp/wizard_dump.txt\n' "$DIM" "$RESET"
        pause
        ask_result "图表示输出（B3）"
    else
        record "图表示输出（B3）" "失败" "--dump-graph 运行失败"
    fi
    pause

    # 第 18 关
    stage "边界：制造不可行场景并观察提示"
    say "本关故意把网络弄断，验证程序**给出明确原因而不是崩溃或静默**。"
    say ""
    if confirm "现在启动程序做这一关？"; then
        if launch_app; then
            sleep 1
            say ""
            head1 "操作步骤："
            say "  ① 点「手工增删…」"
            say "  ② 在「节点 ID」填一个配送点，例如 D01；也可以先在上面的"
            say "     「起点/终点 ID」里逐个删掉 D01 的所有连边"
            say "  ③ 更简单的做法：把 D01 的邻居逐个删边，直到 D01 成为孤岛"
            say ""
            head1 "预期："
            say "  · 「路线信息」显示「不可行：无法从 … 到达配送点: D01」"
            say "  · 原因里**明确指出是哪个节点**不可达"
            say "  · 程序不崩溃，其他按钮仍可继续点击"
            pause
        else
            record "边界：不可行场景" "失败" "程序未能启动"
            summary
            exit 1
        fi
        ask_result "边界：不可行场景"
        stop_app
    else
        record "边界：不可行场景" "跳过" "用户取消"
    fi
}

main "$@"
summary
