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
TOTAL_STAGES=8
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
    # 落盘日志：程序若启动即退出，把它的输出打出来，便于定位
    # （第 7 轮报过一次"程序未能启动"但无法复现，先把诊断信息留全）
    local log=/tmp/wizard_app.log
    "$APP" "$@" >"$log" 2>&1 &
    APP_PID=$!

    # 轮询最多 10 秒：既避免"启动稍慢被误判"，也能在早期崩溃时立刻发现
    local waited=0
    while [[ $waited -lt 20 ]]; do
        if ! kill -0 "$APP_PID" 2>/dev/null; then
            printf '%s程序启动后立即退出。%s\n' "$RED" "$RESET"
            printf '%s--- 程序输出（%s）---%s\n' "$DIM" "$log" "$RESET"
            cat "$log"
            APP_PID=""
            return 1
        fi
        sleep 0.5
        waited=$((waited + 1))
        # 已稳定存活 2 秒即认为启动成功（启动即崩通常发生在前 2 秒内）
        if [[ $waited -ge 4 ]]; then
            break
        fi
    done
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
#
# 8 关（第 2 次瘦身，用户要求"把只是看看的关卡合并"）：
#   原来"认清布局""看网络图""看规划结果""看超时表""看三策略"是五关纯粹的"看"，
#   熟练测试者每关都要清屏、按回车、再切回程序，成本全在切换而不在验证。
#   合并后：看界面/看网络/看图表示 -> 一关；看路线/看车辆/看中转站/看明细/
#   切三策略/看超时 -> 一关。每个"要不要动手"的环节仍然是独立一关，不打乱节奏。

main() {
    clear 2>/dev/null || true
    head1 "人工测试向导 —— 电商物流配送路径规划系统"
    say ""
    say "共 $TOTAL_STAGES 关；熟练后全程约 5–6 分钟。"
    say "第 1 关自动跑；第 2–3 关主要是「看」，第 4–8 关需要你动手。"
    say ""
    if [[ ! -t 0 ]]; then
        printf '%s当前 stdin 不是终端，无法交互。请在终端里直接运行：%s\n' "$RED" "$RESET"
        say "  bash scripts/manual_test_wizard.sh"
        exit 2
    fi
    confirm "准备好了吗？" || { say "已取消。"; exit 0; }

    # ---------------------------------------------------------------- 1
    stage "构建与自动检查"
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
        if (cd build && ctest >/tmp/wizard_ctest.log 2>&1); then
            say "${GREEN}ctest 全部通过${RESET}"
            grep -E "tests passed" /tmp/wizard_ctest.log | tail -1
        else
            build_ok=0
            printf '%sctest 有失败项：%s\n' "$RED" "$RESET"
            grep -E "Failed|failed" /tmp/wizard_ctest.log | head -10
        fi
    fi
    if [[ "$build_ok" -eq 1 ]]; then
        if QT_QPA_PLATFORM=offscreen "$APP" --ui-probe >/tmp/wizard_ui.log 2>&1; then
            grep "ui-probe" /tmp/wizard_ui.log | sed 's/^/  /'
        else
            build_ok=0
            printf '%s界面元件不完整：%s\n' "$RED" "$RESET"
            cat /tmp/wizard_ui.log
        fi
        if [[ "$build_ok" -eq 1 ]] \
           && ! QT_QPA_PLATFORM=offscreen "$APP" --self-check-actions >/tmp/wizard_act.log 2>&1; then
            build_ok=0
            printf '%s动作自检失败：%s\n' "$RED" "$RESET"
            grep -E "FAIL" /tmp/wizard_act.log
        fi
    fi
    if [[ "$build_ok" -ne 1 ]]; then
        record "构建与自动检查" "失败" "构建/自检未通过"
        summary
        exit 1
    fi
    if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
        record "构建与自动检查" "失败" "无图形显示环境"
        summary
        exit 1
    fi
    record "构建与自动检查" "通过" "构建、ctest、界面元件、动作自检均通过"
    pause

    # ---------------------------------------------------------------- 2
    stage "启动、界面布局、网络图与图表示"
    say "这一关把「认界面」和「看网络图」合在一起——都是看，不必分两次启动。"
    say ""
    if ! confirm "现在启动程序？"; then
        record "启动与网络图" "跳过" "用户取消启动"
        summary
        exit 0
    fi
    if ! launch_app; then
        record "启动与网络图" "失败" "程序未能启动"
        summary
        exit 1
    fi
    sleep 1
    say "${GREEN}程序已启动，应自动最大化。${RESET}"
    say ""
    head1 "① 界面四块："
    say "   工具栏：规划策略▾ 权重标签▾ | 模拟路况 插入紧急订单 模拟新客户 模拟道路封闭"
    say "           | 推进一站 重新规划 手工增删… 图表示… | Debug 速度▾ Debug 模式"
    say "   画布：网络图、红色高亮路线、权重标签、品红环=车辆位置"
    say "   右侧栏：路线信息 / 车辆信息 / 标签页组（订单列表·超时订单·中转站/集散·停靠明细）"
    say "   底部：日志"
    todo "看不到工具栏或侧栏时：双击标题栏最大化；或在工具栏空白处**右键**勾选恢复"
    say ""
    head1 "② 画布上的网络："
    want "30 个节点：蓝色=仓库(2)、绿色=配送点(25)、橙色=中转站(3)，各带名称"
    want "绝大多数边两端各有箭头（双向路）"
    want "**至少 6 条边只有一端有箭头**——单行道，无法逆行"
    say ""
    todo "点工具栏的「图表示…」，打开对话框，切两个标签页看看"
    want "「邻接表」：每个节点一行，列出全部出边（目标(距离/耗时/成本)）"
    want "「邻接矩阵」：真正的 N×N 表格，按当前权重显示，缺边为 -"
    todo "关掉对话框"
    pause
    ask_result "启动、界面布局、网络图与图表示"

    # ---------------------------------------------------------------- 3
    stage "规划结果：路线 / 车辆 / 中转站 / 停靠明细 / 三策略 / 超时"
    say "这一关全是「看」，一口气看完六样。中途不用切回终端。"
    say ""
    head1 "① 路线信息"
    want "一条红色粗线串起全部 25 个配送点，起止都在 W01"
    want "载重上限 200kg **小于**单个子网络簇的货量(220~262kg)，因此规划是**多趟**的"
    want "显示「停靠 25 站，已送达 0 站，共 N 趟」，逐趟列出形如："
    want "    第 1 趟：7 个配送点  D01 → D04  28.2km  装载 190kg"
    want "  **每趟都直接从仓库出发、送完返回仓库**（初始规划不绕中转站）"
    want "  注意「装载」是**本趟**装了多少，不是全部货物"
    dim "为什么绕开中转站：中转站**初始无存货**。让它参与路由要多跑一次"
    dim "「站与配送点之间」的往返，实测比直达差（178.2 vs 198.2km、6 趟 vs 13 趟）。"
    say ""
    head1 "② 车辆信息——三条载重必须互不混淆"
    want "载重上限 200kg"
    want "本趟装载（本趟出发时装了多少）与 **当前载重**（此刻车上还有多少）都 **≤ 200kg**"
    want "剩余待送（全部未送达货量之和，740kg 量级）**可以远超** 200kg —— 这正是要多趟的原因"
    say ""
    todo "切到「中转站 / 集散」标签页"
    want "3 个中转站，子网络 1/2/3，下属配送点 9/7/9"
    want "**「峰值暂存」与「当前暂存」此刻都是 0** —— 初始规划不从簇里带货回来"
    say ""
    todo "切到「停靠明细」标签页"
    want "列头：趟 | 配送点 | 原始到达 | 等待(分) | 送达 | 离开 | 送后余载(kg)"
    want "**「趟」这一列必须存在**——明细是跨趟拉平的，没有它，"
    want "  「送后余载」会从 0 跳回几十公斤，看起来像数据错了，其实是新的一趟开始装货"
    head1 "参考值（由程序现场算出，不写死）："
    QT_QPA_PLATFORM=offscreen "$APP" --ui-probe 2>/dev/null \
        | grep -E "停靠明细 [0-9]+ 行|^\[ui-probe\]   第 " | sed 's/^\[ui-probe\] /    /' | head -4
    want "等待 = 窗口开始 − 原始到达；送后余载 = 本趟装载 − 已送需求之和"
    want "  例：D01 等待 = 窗口 09:00 − 原始到达 08:05；余载 = 首趟装载 − D01 需求"
    say ""
    todo "切到「超时订单」标签页"
    want "超时站点应当**很少**，且只出现在窗口很紧的点上"
    head1 "本次数据下的参考值（程序现场算出）："
    QT_QPA_PLATFORM=offscreen "$APP" --plan-summary distance 2>/dev/null | sed 's/^/    /'
    say ""
    todo "回到工具栏，把「规划策略」依次切三种、「权重标签」依次切三档"
    want "「规划策略」下拉顺序是 最短距离 / 最低耗时 / 最低成本，与「权重标签」一致"
    printf '    最短距离策略  %s\n' "$(QT_QPA_PLATFORM=offscreen "$APP" --plan-summary distance 2>/dev/null | head -1)"
    printf '    最低耗时策略  %s\n' "$(QT_QPA_PLATFORM=offscreen "$APP" --plan-summary time 2>/dev/null | head -1)"
    printf '    最低成本策略  %s\n' "$(QT_QPA_PLATFORM=offscreen "$APP" --plan-summary cost 2>/dev/null | head -1)"
    want "三种策略各自在目标上占优：距离最小 / 耗时最小 / 成本最小，且三者互异"
    want "「权重标签」切换后边上标签在 km / min / 元 之间变化，路线本身不重算"
    dim "某两种策略结果相同不一定是错——主干道同时是最短与最快时会重合。"
    dim "看完把策略切回「最短距离策略」。"
    pause
    ask_result "规划结果：路线/车辆/中转站/明细/三策略/超时"

    # ---------------------------------------------------------------- 4
    stage "画布交互与手工增删健壮性"
    todo "按住任意一个绿色客户点拖动一段距离再松手"
    want "相连的边实时跟随、箭头贴在节点边界上、权重标签跟着走"
    say ""
    todo "点「手工增删…」"
    todo "在「起点/终点 ID」填两个尚未直连的既有节点，点「添加边」"
    want "日志「手工添加边 A <-> B（双向）」，画布上两个方向各一条边"
    todo "用同一对 ID 点「删除边」→ 两个方向的边一并消失"
    todo "在「节点 ID」填一个不存在的 ID（如 NOPE），点「删除节点」"
    want "日志报「删除节点失败：NOPE 不存在」，**程序不得崩溃或卡死**"
    todo "关闭对话框"
    pause
    ask_result "画布交互与手工增删健壮性"

    # ---------------------------------------------------------------- 5
    stage "动态事件：路况 / 增量重规划 / 紧急订单 / 新客户 / 封路"
    say "五个按钮，都在工具栏上，按顺序点。"
    say ""
    printf '  %s① 模拟路况（含增量重规划）%s\n' "$BOLD" "$RESET"
    todo "点几次「模拟路况」"
    want "日志出现「路况变化：改动 N 条边（新增拥堵 X 条，转为畅通 Y 条）」，两种都要出现"
    want "若受影响边在当前路径上且增幅达标，紧跟「自动触发重规划」，再跟一行："
    want "  「→ 增量式重规划：仅重算受影响的路段，其余原样保留」或「→ 上一版为多趟方案，退回全量重算」"
    say ""
    printf '  %s② 插入紧急订单%s\n' "$BOLD" "$RESET"
    todo "**连点 3–4 次**「插入紧急订单」"
    want "日志「插入紧急订单 U001 @ <配送点>（货量 X kg，要求 时:分 前送达）」"
    want "「订单列表」第一行就是这些 U 开头的订单，状态「紧急」，窗口只有 **1 小时**"
    want "路线里它们被**优先服务**；每次点击都应新增一条，**不应只剩最后一单**"
    dim "默认数据集里本来没有任何紧急订单，紧急单只能靠这里动态产生。"
    say ""
    printf '  %s③ 模拟新客户%s\n' "$BOLD" "$RESET"
    todo "点「模拟新客户」"
    want "画布多出一个绿色配送点 X###，双向连接到最近的既有节点"
    want "日志「模拟新客户：新增配送点 X### 与订单 C###」；路线站点数 +1"
    say ""
    printf '  %s④ 模拟道路封闭%s\n' "$BOLD" "$RESET"
    todo "点「模拟道路封闭」"
    want "日志「模拟道路封闭：<A>-><B>（含反向）」，对应边从画布消失"
    want "**仍然「规划成功」**，不出现「无法从 … 到达 …」"
    say ""
    printf '  %s⑤ 回看中转站%s\n' "$BOLD" "$RESET"
    todo "切到「中转站 / 集散」标签页"
    want "「当前暂存」**可能变成非 0** —— 这是「顺路寄存」在起作用："
    want "  车本次要回仓库，而车上还载着**来不及送**的货，且该货所属簇的中转站"
    want "  就在回程路径上，于是顺手卸在站里（**零成本**，不绕路）"
    want "一旦出现存货会被**保留**，供后续规划把它当前置仓库使用（积少成多）"
    dim "看不到非零**不一定是错**：寄存要「来不及送 + 站在回程路上」两个条件同时满足。"
    pause
    ask_result "动态事件：路况/增量重规划/紧急订单/新客户/封路"

    # ---------------------------------------------------------------- 6
    stage "推进一站：车辆位置、当前载重与暂存随动"
    todo "点几次「推进一站」，盯住画布与右侧三处"
    say ""
    head1 "预期："
    want "画布上品红环自动移动到刚到达的节点（不必点「重新规划」）"
    want "已走过的路段**不再标红**，只有尚未走完的路线高亮"
    want "「车辆信息」的「**当前载重**」随送达**递减**，且始终 ≤ 载重上限"
    want "日志区分「送达 XX（到达 时:分）」与「经过 XX」；经过中转站时补注「入库暂存 / 取货配发」"
    say ""
    todo "一直推到全部送完"
    want "最后一步是「返回仓库 W01（到达 时:分）」，品红环回到 W01"
    want "再点「重新规划」：已送达站点不再出现在新路线里，penalty 不暴涨"
    pause
    ask_result "推进一站：车辆位置、当前载重与暂存随动"

    # ---------------------------------------------------------------- 7
    stage "Debug 模式（A1）"
    todo "先用「Debug 速度」选「展示模式（2 秒/步）」，再勾选「Debug 模式」"
    want "日志每 2 秒增长一次：自动模拟路况、偶尔插单、自动推进、偶发道路封闭"
    todo "把速度切到「快速预览（1 秒/步）」"
    want "节奏立刻变快一倍，日志出现「Debug 速度切换为每 1 秒一步」"
    todo "最后**取消勾选「Debug 模式」**"
    want "**必须立刻停下**——日志不再新增，界面不再变化"
    pause
    ask_result "Debug 模式（A1）"

    # ---------------------------------------------------------------- 8
    stage "边界：制造不可行场景"
    todo "点「手工增删…」，在「节点 ID」填一个配送点（如 D01），点「删除节点」，关闭对话框"
    head1 "预期："
    want "「路线信息」显示不可行，且**原因里明确指出哪个节点不可达**"
    want "程序不崩溃，工具栏其他按钮仍可继续点击"
    pause
    ask_result "边界：制造不可行场景"
    stop_app
}

main "$@"
summary
