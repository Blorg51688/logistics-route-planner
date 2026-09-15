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
# 为什么需要人工：单元/集成测试覆盖不了 GUI 的观感与交互——
#   · 箭头是否画对、标签是否遮挡、拖动节点时边是否跟随
#   · 点击按钮后的界面反馈是否符合直觉
#   本项目已因此抓到两个真实缺陷：箭头被画进节点内部而完全不可见；
#   以及更严重的——**普通启动路径根本不含工具栏与侧栏**（只有一块画布）。
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

# ============================ 库部分 ============================

if [[ -t 1 ]]; then
    BOLD=$'\033[1m'; DIM=$'\033[2m'; RED=$'\033[31m'
    GREEN=$'\033[32m'; YELLOW=$'\033[33m'; CYAN=$'\033[36m'; RESET=$'\033[0m'
else
    BOLD=""; DIM=""; RED=""; GREEN=""; YELLOW=""; CYAN=""; RESET=""
fi

say()   { printf '%s\n' "$*"; }
dim()   { printf '%s%s%s\n' "$DIM" "$*" "$RESET"; }
head1() { printf '%s%s%s\n' "$BOLD$CYAN" "$*" "$RESET"; }
todo()  { printf '  %s▸%s %s\n' "$YELLOW" "$RESET" "$1"; }
want()  { printf '  %s·%s %s\n' "$GREEN" "$RESET" "$1"; }

pause() {
    printf '\n%s按回车继续…%s' "$DIM" "$RESET"
    read -r _
}

confirm() {
    local answer
    printf '%s [y/N] ' "$1"
    read -r answer
    [[ "$answer" == "y" || "$answer" == "Y" ]]
}

record() {
    RESULTS_NAME+=("$1")
    RESULTS_STATE+=("$2")
    RESULTS_NOTE+=("${3:-}")
    case "$2" in
        通过) PASS=$((PASS + 1)) ;;
        失败) FAIL=$((FAIL + 1)) ;;
        跳过) SKIP=$((SKIP + 1)) ;;
    esac

    # 每答完一关立即落盘。向导此前只在最后才写盘，一旦中途中断
    # （例如终端被全屏分页器打乱后不得不终止），整轮结果就全部丢失。
    write_record
}

stage() {
    STAGE=$((STAGE + 1))
    clear 2>/dev/null || true
    printf '%s════════════════════════════════════════════════════════════%s\n' "$CYAN" "$RESET"
    printf '%s  第 %d / %d 关   %s%s\n' "$BOLD$CYAN" "$STAGE" "$TOTAL_STAGES" "$1" "$RESET"
    printf '%s════════════════════════════════════════════════════════════%s\n\n' "$CYAN" "$RESET"
}

ask_result() {
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

open_path() {
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

launch_app() {
    if ! [[ -n "${DISPLAY:-}" || -n "${WAYLAND_DISPLAY:-}" ]]; then
        printf '%s没有检测到图形显示环境（DISPLAY / WAYLAND_DISPLAY 均为空）。%s\n' "$RED" "$RESET"
        say "本向导需要在能显示窗口的桌面上运行。"
        return 1
    fi
    "$APP" "$@" >/dev/null 2>&1 &
    APP_PID=$!
    sleep 2
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

cleanup() {
    stop_app
    # 即使被中断（Ctrl-C / 终端异常），也把已经记录到的关卡保存下来
    if [[ ${#RESULTS_NAME[@]} -gt 0 ]]; then
        write_record
    fi
}
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
    head1 "结果已写入 $RECORD"
    if [[ "$FAIL" -gt 0 ]]; then
        printf '\n%s存在失败关卡。请把失败现象的备注反馈出来，修复后重跑本向导。%s\n' "$RED" "$RESET"
    fi
}

# ============================ 本流程的关卡 ============================

main() {
    clear 2>/dev/null || true
    head1 "人工测试向导 —— 电商物流配送路径规划系统"
    say ""
    say "共 $TOTAL_STAGES 关：第 1 关自动检查，第 2 关说明界面布局，其余需要你亲自操作。"
    say "全程约 15 分钟。程序窗口与终端之间来回切换即可。"
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
        say "运行自动化检查（单元 / 集成 / 数据 / 预言机）："
        if (cd build && ctest >/tmp/wizard_ctest.log 2>&1); then
            say "${GREEN}ctest 全部通过${RESET}"
            grep -E "tests passed" /tmp/wizard_ctest.log | tail -1
        else
            build_ok=0
            printf '%sctest 有失败项：%s\n' "$RED" "$RESET"
            grep -E "Failed|failed" /tmp/wizard_ctest.log | head -10
        fi
    fi

    # 关键：先确认"界面元件真的被构造出来"，再让人去找它们。
    # 曾经普通启动路径只构造了一块裸画布，工具栏与侧栏根本不存在——
    # 那次是让用户先去找、找不到才暴露的。这里提前挡住。
    if [[ "$build_ok" -eq 1 ]]; then
        say ""
        say "检查界面元件是否完整构造…"
        if QT_QPA_PLATFORM=offscreen "$APP" --ui-probe >/tmp/wizard_ui.log 2>&1; then
            grep "ui-probe" /tmp/wizard_ui.log | sed 's/^/  /'
            say "${GREEN}工具栏与侧栏均已构造${RESET}"
        else
            build_ok=0
            printf '%s界面元件不完整：%s\n' "$RED" "$RESET"
            cat /tmp/wizard_ui.log
            say "请先修复，否则后面的人工关卡无法进行。"
        fi
    fi

    if [[ "$build_ok" -ne 1 ]]; then
        record "构建与环境检查" "失败" "构建/自检/界面元件检查未通过"
        summary
        exit 1
    fi
    if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
        record "构建与环境检查" "失败" "无图形显示环境"
        summary
        exit 1
    fi
    record "构建与环境检查" "通过" "构建、ctest、界面元件检查均通过"
    say ""
    say "${GREEN}第 1 关通过。${RESET}"
    pause

    # ---------------------------------------------------------------- 第 2 关
    stage "启动程序，并认清界面布局"
    say "本关不判断功能，只让你先看清东西都在哪儿。"
    say ""
    if ! confirm "现在启动程序？"; then
        record "启动程序与界面布局" "跳过" "用户取消启动"
        summary
        exit 0
    fi
    if ! launch_app; then
        record "启动程序与界面布局" "失败" "程序未能启动"
        summary
        exit 1
    fi
    sleep 2
    say "${GREEN}程序已启动。${RESET}它应当**自动最大化**。"
    say ""
    head1 "界面分四块，位置如下："
    say ""
    printf '  %s① 工具栏（窗口最顶部一整行）%s\n' "$BOLD" "$RESET"
    say "     从左到右依次是："
    say "       · 下拉框「规划策略」—— 切换 最短距离 / 最低成本"
    say "       · 下拉框「权重标签」—— 切换 显示距离 / 耗时 / 成本"
    say "       · 按钮「模拟路况」"
    say "       · 按钮「插入紧急订单」"
    say "       · 按钮「模拟新客户」"
    say "       · 按钮「模拟道路封闭」"
    say "       · 按钮「推进一站」"
    say "       · 按钮「重新规划」"
    say "       · 按钮「手工增删…」"
    say "       · 开关「Debug 模式」（可勾选/取消）"
    say ""
    printf '  %s② 画布（中间最大的一块）%s\n' "$BOLD" "$RESET"
    say "     配送网络图、红色高亮路线、边上的权重标签都在这里"
    say ""
    printf '  %s③ 右侧栏（窗口右边竖排）%s\n' "$BOLD" "$RESET"
    say "     从上到下："
    say "       · 「路线信息」—— 策略、总距离/耗时/成本、总 penalty、完整序列"
    say "       · 「车辆信息」—— 车辆 ID、起始仓库、载重上限、发车时刻、当前载重"
    say "       · 「订单列表」与「超时订单」—— 这两个是**标签页**："
    say "         看那一块底部的两个标签，点一下就能切换"
    say ""
    printf '  %s④ 日志（窗口最底部）%s\n' "$BOLD" "$RESET"
    say "     每做一次操作都会追加一行说明，包括为什么触发/不触发重规划"
    say ""
    head1 "如果看不到工具栏或侧栏，按下面顺序排查："
    todo "窗口没最大化 → 双击标题栏，或点右上角最大化按钮"
    todo "某个面板被关掉了（点过它右上角的 ×）→ 在工具栏空白处**右键**，"
    todo "  会弹出面板列表，勾选「路线信息」「车辆信息」「订单列表」「超时订单」「日志」即可恢复"
    todo "右侧栏太窄看不清 → 拖动它与画布之间那条竖直分隔条"
    todo "日志栏太矮 → 拖动它与画布之间那条水平分隔条"
    say ""
    dim "确认能看到：顶部工具栏、中间画布、右侧三个面板、底部日志。"
    pause
    ask_result "启动程序与界面布局"

    # ---------------------------------------------------------------- 第 3 关
    stage "初始网络图（B1 / B2 / B4）"
    todo "看向中间的画布。"
    say ""
    head1 "预期："
    want "共 30 个节点：蓝色圆点 = 仓库（2 个），绿色 = 配送点（25 个），橙色 = 中转站（3 个）"
    want "每个节点下方有名称（中央仓库A、客户01、中转站甲…）"
    want "每条边的两端各带一个箭头（有向图；双向边两端都有箭头）"
    want "没有箭头被画进节点圆圈内部而看不见"
    say ""
    dim "参考图：docs/screenshots/gui-network.png"
    open_path "docs/screenshots/gui-network.png"
    pause
    ask_result "初始网络图（B1/B2/B4）"

    # ---------------------------------------------------------------- 第 4 关
    stage "初始规划与路径高亮（B6 / B7）"
    todo "看画布上的红色粗线；再看右侧「路线信息」面板。"
    say ""
    head1 "预期："
    want "一条红色粗线串起全部 25 个配送点，从 W01 出发、最后回到 W01"
    want "红色边上有**白底**的权重标签；灰色边**不显示**标签"
    want "W01 上有一圈**品红色**的环，表示车辆当前停在仓库"
    say ""
    head1 "「路线信息」应与下列参考值一致："
    printf '    %s\n' "$(QT_QPA_PLATFORM=offscreen "$APP" --plan-summary distance 2>/dev/null)"
    say ""
    dim "参考值由程序自身按当前的 config/default.ini 算出，数据集调整后不会过期。"
    dim "（精确数值的回归由 ctest 中 integration_tests 的黄金值断言守住。）"
    pause
    ask_result "初始规划与路径高亮（B6/B7）"

    # ---------------------------------------------------------------- 第 5 关
    stage "双策略分化（B7）"
    todo "点工具栏**最左边**那个下拉框「规划策略」，选「最低成本策略」。"
    say ""
    head1 "预期："
    want "路由高亮发生变化（不再与最短距离策略相同）"
    want "两者互换优劣：距离策略更短、成本策略更便宜"
    say ""
    head1 "两种策略的参考值："
    printf '    最短距离策略  %s\n' "$(QT_QPA_PLATFORM=offscreen "$APP" --plan-summary distance 2>/dev/null)"
    printf '    最低成本策略  %s\n' "$(QT_QPA_PLATFORM=offscreen "$APP" --plan-summary cost 2>/dev/null)"
    say ""
    dim "对照要点：成本策略的总成本应**更小**，但总距离**更大**。"
    dim "若两种策略给出完全一样的结果，说明数据或算法退化，属失败。"
    dim "看完请把它切回「最短距离策略」，后面几关基于它。"
    pause
    ask_result "双策略分化（B7）"

    # ---------------------------------------------------------------- 第 6 关
    stage "权重标签切换（B4）"
    todo "点工具栏左边第二个下拉框「权重标签」，依次选「显示耗时」「显示成本」。"
    say ""
    head1 "预期："
    want "边上标签数值随之改变：同一条边约 5.2km → 约 8min → 约 6元"
    want "高亮路线本身**不重算、不变化**（只换显示文本）"
    want "双向边的两个方向若同时高亮，同一段路上**只显示一个标签**，不重复"
    say ""
    dim "看完请切回「显示距离」。"
    pause
    ask_result "权重标签切换（B4）"

    # ---------------------------------------------------------------- 第 7 关
    stage "拖动节点与边联动（GUI 语义）"
    todo "在画布上按住任意一个绿色客户点，拖动一段距离再松手。"
    say ""
    head1 "预期："
    want "与该节点相连的边**实时跟随**移动"
    want "箭头仍贴在节点边界上，没有飘在空中或被埋进圆点"
    want "权重标签也跟着走"
    say ""
    dim "若拖不动，请确认按在节点圆点上（不是空白处）。"
    pause
    ask_result "拖动节点与边联动（GUI 语义）"

    # ---------------------------------------------------------------- 第 8 关
    stage "时间窗与超时 penalty（E2）"
    todo "看右侧栏最下面那块，点它的「超时订单」标签页。"
    say ""
    head1 "预期："
    want "列头为：订单 / 配送点 / 到达 / 窗口 / penalty"
    want "只列出 2 个超时站点：D12 与 D05"
    want "「订单」列给出该配送点上的订单号（如 O12、O05）"
    want "「窗口」列给出该点的送达窗口（如 09:00-11:00），便于判断为什么超时"
    want "其余 23 个站点准时 —— 超时应是**少数**、且只出现在窗口很紧的那两站"
    say ""
    dim "若大量站点都超时，说明默认数据或时间模型有问题，属失败。"
    dim "看完可点「订单列表」标签切回去。"
    pause
    ask_result "时间窗与超时 penalty（E2）"

    # ---------------------------------------------------------------- 第 9 关
    stage "模拟路况与自动重规划（E1）"
    todo "点工具栏第 3 个按钮「模拟路况」（在「规划策略 / 权重标签」两个下拉框右侧）。"
    say ""
    head1 "预期（看底部日志）："
    want "新增一行「路况变化：改动 N 条边（新增拥堵 X 条，转为畅通 Y 条）」"
    want "  （N 约为 72 的 10%，即 7 条；X + Y = N）"
    want "**多按几次**：应能同时看到「新增拥堵」和「转为畅通」两种情况"
    want "  —— 需求把实时路况定义为「拥堵 / 畅通」两种动态属性，只增不减是漏实现"
    want "若受影响的边正好在当前路径上且耗时增幅 ≥ 20%，紧跟一行"
    want "  「受影响边位于当前路径且增幅达标 -> 自动触发重规划」"
    want "触发重规划后，红色高亮与右侧「总耗时」会更新"
    want "若受影响边不在路径上或增幅不足，则提示「不触发重规划」"
    say ""
    dim "两种结果都算通过 —— 重点是判断逻辑正确、日志说清了原因。"
    pause
    ask_result "模拟路况与自动重规划（E1）"

    # ---------------------------------------------------------------- 第 10 关
    stage "插入紧急订单（E3）"
    todo "点工具栏「插入紧急订单」（紧挨着「模拟路况」右边）。"
    say ""
    head1 "预期："
    want "日志出现「插入紧急订单 U001 @ <配送点>（货量 X kg，要求 时:分 前送达）」"
    want "右侧「订单列表」的**第一行**就是这个 U 开头的订单，状态显示为「紧急」"
    want "  （紧急订单会排到列表最前，不必向下滚动去找）"
    want "它的窗口只有 **1 小时**（形如 10:11-11:11），对应需求举例的「1 小时内送达」"
    want "该配送点在新的高亮路线里被**优先服务**（排在最前面的停靠点）"
    want "若该单窗口必然无法满足，会额外给出一行 ⚠ 警告，但订单**仍被纳入路线**"
    want "  （本项目口径：超时不弃，只记 penalty）"
    say ""
    dim "**请连点 3–4 次**：每次都应新增一条 U 开头的订单，且它们整体排在普通订单前面。"
    dim "（早先的缺陷是只有最后一单被保留，前面的紧急单会被丢掉。）"
    say ""
    dim "另外：默认数据集里**本来没有任何紧急订单**，紧急单只能靠这里动态产生。"
    pause
    ask_result "插入紧急订单（E3）"

    # ---------------------------------------------------------------- 第 11 关
    stage "模拟新客户（B5）"
    todo "点工具栏「模拟新客户」。"
    say ""
    head1 "预期："
    want "画布上多出一个新节点（编号形如 X001、X002…，绿色配送点）"
    want "它连到距离最近的既有节点，**两个方向都有边、都有箭头**"
    want "日志出现「模拟新客户：新增配送点 X###」"
    want "随后自动重新规划成功（日志「规划成功：… 站」）"
    pause
    ask_result "模拟新客户（B5）"

    # ---------------------------------------------------------------- 第 12 关
    stage "模拟道路封闭（B5）—— 重点回归关卡"
    todo "点工具栏「模拟道路封闭」。"
    say ""
    head1 "预期："
    want "日志出现「模拟道路封闭：<A>-><B>（含反向）」"
    want "对应的边（含反向）从画布上消失"
    want "**仍然「规划成功」** —— 不会出现「无法从 … 到达 …」的不可行"
    say ""
    dim "这一关守着一个真实缺陷：早先封路会把网络切断（把某个配送点变成孤岛），"
    dim "一次点击就把程序锁死在不可行状态。现已改为不封闭「桥」。"
    say ""
    dim "若出现「规划不可行」，属失败，请把日志原文记到备注里。"
    pause
    ask_result "模拟道路封闭（B5）"

    # ---------------------------------------------------------------- 第 13 关
    stage "手工增删节点 / 边（B5）"
    todo "点工具栏「手工增删…」，会弹出一个对话框。"
    say ""
    head1 "对话框里有三个输入框（起点 ID、终点 ID、节点 ID）。依次试三件事："
    say ""
    printf '  %s① 添加边%s\n' "$BOLD" "$RESET"
    todo "在「起点 ID」「终点 ID」填两个**尚未直连**的既有节点（例如 D01 与 D25）"
    todo "点「添加边」"
    want "日志「手工添加边 D01 <-> D25（双向）」"
    want "画布上出现**两个方向**各一条边（两端都有箭头）并重新规划"
    say ""
    printf '  %s② 删除边%s\n' "$BOLD" "$RESET"
    todo "用同一对 ID 点「删除边」"
    want "日志「手工删除边 D01 <-> D25」；两个方向的边一并消失"
    say ""
    printf '  %s③ 删除不存在的节点（健壮性）%s\n' "$BOLD" "$RESET"
    todo "在「节点 ID」填一个不存在的 ID（例如 NOPE），点「删除节点」"
    want "日志报「删除节点失败：NOPE 不存在」"
    want "程序**不得崩溃或卡死**"
    say ""
    dim "做完点对话框右下角的 Close 关闭。"
    pause
    ask_result "手工增删节点 / 边（B5）"

    # ---------------------------------------------------------------- 第 14 关
    stage "推进一站与当前位置追踪（B6）—— 重点回归关卡"
    todo "点工具栏「推进一站」。**多点几次**，注意观察车辆标记逐个节点移动，"
    todo "一直推到日志出现「本次配送已完成」为止，然后再点「重新规划」。"
    say ""
    head1 "预期："
    want "推进是**逐节点**的：每点一次就前进到路线序列里的下一个节点，"
    want "  包括途经的**中转站**和**仓库**，不只是配送点"
    want "日志会区分两种：配送点记「送达 XX（到达 时:分）」；"
    want "  其他节点记「经过 XX（到达 时:分）」"
    want "画布上那圈**品红色的车辆标记**会**自动**移动到刚到达的节点上"
    want "  （不需要点「重新规划」，推进本身就会刷新画布）"
    want "右侧「订单列表」标签页里，对应订单状态变成「已送达」"
    want "**一直推到全部送完**：最后一步应是「返回仓库 W01（到达 时:分）」，"
    want "  车辆标记回到 W01 —— 对应 B6「遍历后返回仓库」"
    want "点「重新规划」后，**已送达的站点不再出现在新路线里**"
    want "「路线信息」里的「总 penalty」保持在一个合理量级（几百分钟）"
    want "  **不应暴涨到几千分钟**"
    say ""
    dim "这一关守着另一个真实缺陷：重规划会用全新计划覆盖旧计划，而站点计数"
    dim "仍指向旧计划，导致取到错误时刻、penalty 暴涨（实测曾达 5322min）。"
    say ""
    dim "若 penalty 变成四位数或更大，属失败，请把数值记到备注里。"
    pause
    ask_result "推进一站与当前位置追踪（B6）"

    # ---------------------------------------------------------------- 第 15 关
    stage "Debug 模式（A1）—— 重点回归关卡"
    todo "点工具栏**最右边**的「Debug 模式」，让它变成勾选状态，观察 10–20 秒。"
    todo "然后再点一次取消勾选。"
    say ""
    head1 "预期："
    want "勾选后日志**持续增长**：会自动模拟路况、偶尔插入紧急订单、自动推进送达"
    want "开启时会写明节流规则：紧急订单每 60 秒最多一单，且同时最多 2 单待处理"
    want "  （不会连续冒出多单把整体规划搅乱）"
    want "路况与推进**约每 3 秒**一次，不必久等"
    want "画布上的高亮路线随之变化"
    want "**取消勾选后必须立刻停下来** —— 日志不再新增，界面不再变化"
    want "若让它一直跑到结束：最后车辆应回到仓库 W01，而不是停在最后一个客户处"
    say ""
    dim "这一关守着 Debug 模式必须「可随时关闭」。若取消勾选后日志仍在刷，属失败。"
    pause
    ask_result "Debug 模式（A1）"

    stop_app
    say ""
    dim "已关闭程序，准备下一关。"
    pause

    # ---------------------------------------------------------------- 第 16 关
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
            sleep 2
            say ""
            head1 "预期："
            want "右侧「路线信息」显示「不可行：总需求 740kg 超过载重上限 100kg」"
            want "画布上**没有任何红色高亮路线**"
            want "程序不崩溃"
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

    # ---------------------------------------------------------------- 第 17 关
    stage "图表示输出：邻接表 / 邻接矩阵（B3）"
    say "本关在命令行完成，向导直接运行并把结果落成两个文件。"
    say ""

    local list_file="$ROOT/build/graph_adjacency_list.txt"
    local matrix_file="$ROOT/build/graph_adjacency_matrix.txt"
    mkdir -p "$ROOT/build"

    if "$APP" --dump-graph list >"$list_file" 2>/dev/null \
       && "$APP" --dump-graph matrix >"$matrix_file" 2>/dev/null; then
        head1 "① 邻接表（节选前 4 行，行已截断）"
        sed -n '1,4p' "$list_file" | cut -c1-150
        say ""
        head1 "② 邻接矩阵（列标 + 前 2 行，列已截断）"
        sed -n '1,3p' "$matrix_file" | cut -c1-150
        say ""
        head1 "预期："
        want "邻接表：1 行标题 + 30 行节点，每行形如"
        want "  W01(warehouse,中央仓库A): -> T01(7.6km,7.6min,12.2元) -> …"
        want "邻接矩阵：1 行列标 + 30 行数据，共 31×31 个单元格"
        want "缺边位置显示为 -，对角线也全是 -（无自环）"
        want "数值与画布上看到的边一致"
        say ""
        head1 "查看完整输出："
        dim "正在用系统默认文本查看器打开这两个文件（效果等同双击 txt 预览）…"
        open_path "$list_file"
        open_path "$matrix_file"
        say ""
        dim "若没有自动弹出，可复制下面命令到任意终端执行："
        printf '    less %s\n' "$list_file"
        printf '    less %s\n' "$matrix_file"
        printf '    xdg-open %s\n' "$matrix_file"
        say ""
        dim "两个文件分别保存邻接表与邻接矩阵的完整内容，演示时可随时重新打开。"
        pause
        ask_result "图表示输出（B3）"
    else
        record "图表示输出（B3）" "失败" "--dump-graph 运行失败"
    fi
    pause

    # ---------------------------------------------------------------- 第 18 关
    stage "边界：制造不可行场景并观察提示"
    say "本关故意把网络弄断，验证程序**给出明确原因而不是崩溃或静默**。"
    say ""
    if confirm "现在启动程序做这一关？"; then
        if launch_app; then
            sleep 2
            say ""
            head1 "操作步骤："
            todo "点「手工增删…」"
            todo "在「节点 ID（删除用）」填一个配送点，例如 D01"
            todo "点「删除节点」—— D01 从图上消失"
            todo "关闭对话框"
            say ""
            head1 "预期："
            want "右侧「路线信息」显示不可行，且**原因里明确指出是哪个节点不可达**"
            want "  （形如「无法从 … 到达配送点: D0x」）"
            want "程序不崩溃，工具栏其他按钮仍可继续点击"
            say ""
            dim "提示：删掉 D01 后，与它相关的订单会失去目标，正因如此才会不可行。"
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
