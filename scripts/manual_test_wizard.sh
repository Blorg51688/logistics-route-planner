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
TOTAL_STAGES=13
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
# 瘦身说明（第 7 轮用户反馈）：
#   原 20 关里，多趟集散、单向路、增量重规划、图表示输出四关，
#   要么已经并入上面的常规流程（集散已是默认行为），要么 GUI 里已有更直观的入口
#   （图表示表格）。另有若干关多次通过且与改动无耦合，合并为复合关卡。
#   20 关 -> 13 关；每一关仍然"只做一件聚焦的事"，但把同类检查并到一关内一起看。

main() {
    clear 2>/dev/null || true
    head1 "人工测试向导 —— 电商物流配送路径规划系统"
    say ""
    say "共 $TOTAL_STAGES 关：第 1 关自动检查，第 2 关说明界面布局，其余 $((TOTAL_STAGES - 2)) 关需要你亲自操作。"
    say "已按「多次证明无问题的内容不再单列」做过瘦身，全程约 8–10 分钟。"
    say ""
    dim "提示：每一关都会清屏并只显示当前这一关；切回终端时它仍在屏幕上。"
    say ""
    if [[ ! -t 0 ]]; then
        printf '%s当前 stdin 不是终端，无法交互。请在终端里直接运行：%s\n' "$RED" "$RESET"
        say "  bash scripts/manual_test_wizard.sh"
        exit 2
    fi
    confirm "准备好了吗？" || { say "已取消。"; exit 0; }

    # ---------------------------------------------------------------- 1
    stage "构建与自动检查"
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
        say "运行自动化检查（单元 / 集成 / 数据 / 预言机 / 界面元件）："
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
        say ""
        say "界面元件与动作自检："
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
    say ""
    say "${GREEN}第 1 关通过。${RESET}"
    pause

    # ---------------------------------------------------------------- 2
    stage "启动程序，认清界面布局"
    say "本关不判断功能，只让你先看清东西都在哪儿。"
    say ""
    if ! confirm "现在启动程序？"; then
        record "启动与界面布局" "跳过" "用户取消启动"
        summary
        exit 0
    fi
    if ! launch_app; then
        record "启动与界面布局" "失败" "程序未能启动"
        summary
        exit 1
    fi
    sleep 1
    say "${GREEN}程序已启动。${RESET}它应当**自动最大化**。"
    say ""
    head1 "界面四块："
    say "  ① 工具栏（最顶一行）：规划策略▾ 权重标签▾ | 模拟路况 插入紧急订单 模拟新客户"
    say "     模拟道路封闭 | 推进一站 重新规划 手工增删… 图表示… | Debug 速度▾ Debug 模式"
    say "  ② 画布（中间最大）：网络图、红色高亮路线、边上权重标签、品红环标记车辆位置"
    say "  ③ 右侧栏：路线信息 / 车辆信息 / 标签页组（订单列表·超时订单·中转站/集散·停靠明细）"
    say "  ④ 日志（最底部）"
    say ""
    head1 "看不到工具栏或侧栏时："
    todo "窗口没最大化 → 双击标题栏"
    todo "面板被 × 关掉 → 在工具栏空白处**右键**勾选恢复"
    todo "栏太窄 → 拖它与画布之间的分隔条"
    pause
    ask_result "启动与界面布局"

    # ---------------------------------------------------------------- 3
    stage "网络图、有向性与图表示（B1/B2/B4/B3）"
    say "看向画布。"
    say ""
    head1 "预期："
    want "30 个节点：蓝色=仓库(2)、绿色=配送点(25)、橙色=中转站(3)，各带名称"
    want "绝大多数边两端各有箭头（双向路）"
    want "**至少 6 条边只有一端有箭头**——这些是单行道，无法逆行（有向图的关键体现）"
    want "参考图（本次现场渲染，不会过期）："
    QT_QPA_PLATFORM=offscreen "$APP" --render build/wizard_network.png --width 1200 --height 800 >/dev/null 2>&1
    open_path "build/wizard_network.png"
    say ""
    todo "点工具栏的「图表示…」，会打开一个对话框，里面有两个标签页"
    want "「邻接表」：每个节点一行，列出它的全部出边（目标(距离/耗时/成本)）"
    want "「邻接矩阵」：真正的 N×N 表格，按当前权重显示数值，缺边为 -，对角线全为 -"
    want "表格能滚动查看，不是文本文件"
    pause
    ask_result "网络图、有向性与图表示（B1/B2/B4/B3）"

    # ---------------------------------------------------------------- 4
    stage "规划结果、多趟与中转站（B6/B7/E4）"
    say "看右侧「路线信息」与「车辆信息」。"
    say ""
    head1 "预期："
    want "画布上一条红色粗线串起全部 25 个配送点，起止都在 W01"
    want "载重上限 200kg **小于**单个子网络簇的货量(220~262kg)，因此规划是**多趟**的"
    want "「路线信息」显示「停靠 25 站，已送达 0 站，共 N 趟」并逐趟列出，形如："
    want "    第 1 趟：W01 → …（若干节点）"
    want "  **各趟都是直接从仓库出发、送完返回仓库**——不绕中转站"
    say ""
    dim "为什么绕开中转站：中转站**初始无存货**。让它参与路由就要多跑一次"
    dim "「站与配送点之间」的往返，实测比直达差（178.2 vs 198.2km、6 趟 vs 13 趟）。"
    dim "本项目定的原则是：不为满足某个机制的前提，去执行一个更差的方案。"
    say ""
    head1 "「车辆信息」三条载重必须互不混淆："
    want "载重上限 200kg"
    want "本趟装载（本趟出发时装了多少）与 **当前载重**（此刻车上还有多少）都 **≤ 200kg**"
    want "剩余待送（全部未送达货量之和，740kg 量级）**可以远超** 200kg —— 这正是要多趟的原因"
    say ""
    todo "切到右侧的「中转站 / 集散」标签页"
    want "3 个中转站，子网络 1/2/3，下属配送点 9/7/9"
    want "**「峰值暂存」与「当前暂存」此刻都是 0** —— 初始无存货，符合上面的原则"
    todo "再切到「停靠明细」标签页"
    want "每个配送点一行：配送点 | 原始到达 | 等待 | 送达 | 离开 | 剩余载重"
    pause
    ask_result "规划结果、多趟与中转站（B6/B7/E4）"

    # ---------------------------------------------------------------- 5
    stage "三种策略与权重标签（B7/B4）"
    say "工具栏最左边的「规划策略」下拉，依次选三种；右边「权重标签」下拉切三档。"
    say ""
    head1 "预期："
    want "「规划策略」的下拉顺序是 最短距离 / 最低耗时 / 最低成本，与「权重标签」一致"
    want "切换策略后路由高亮会变化，三者各自在目标上占优："
    printf '    最短距离策略  %s\n' "$(QT_QPA_PLATFORM=offscreen "$APP" --plan-summary distance 2>/dev/null | head -1)"
    printf '    最低耗时策略  %s\n' "$(QT_QPA_PLATFORM=offscreen "$APP" --plan-summary time 2>/dev/null | head -1)"
    printf '    最低成本策略  %s\n' "$(QT_QPA_PLATFORM=offscreen "$APP" --plan-summary cost 2>/dev/null | head -1)"
    want "「权重标签」切换后：边上标签在 km / min / 元 之间变化，路线本身不重算"
    want "双向边同时高亮时同一段路只显示一个标签，不重复"
    say ""
    dim "对照要点：距离策略距离最小、成本策略成本最小、耗时策略耗时最小；三者应互异。"
    dim "看完把策略切回「最短距离策略」。" 
    pause
    ask_result "三种策略与权重标签（B7/B4）"

    # ---------------------------------------------------------------- 6
    stage "拖动节点与边联动"
    todo "按住任意一个绿色客户点拖动一段距离再松手。"
    say ""
    head1 "预期："
    want "相连的边实时跟随；箭头仍贴在节点边界上；权重标签跟着走"
    pause
    ask_result "拖动节点与边联动"

    # ---------------------------------------------------------------- 7
    stage "时间窗与超时 penalty（E2）"
    todo "切到右侧的「超时订单」标签页。"
    say ""
    head1 "预期："
    want "列头为 订单 / 配送点 / 到达 / 窗口 / penalty"
    want "超时站点应当**很少**，且只出现在窗口很紧的点上"
    head1 "本次数据下的参考值（程序现场算出）："
    QT_QPA_PLATFORM=offscreen "$APP" --plan-summary distance 2>/dev/null | sed 's/^/    /'
    dim "若表里的站点与上面的参考值不一致，属失败。"
    pause
    ask_result "时间窗与超时 penalty（E2）"

    # ---------------------------------------------------------------- 8
    stage "模拟路况与增量重规划（E1/D22）"
    todo "点几次工具栏的「模拟路况」，看底部日志。"
    say ""
    head1 "预期："
    want "新增一行「路况变化：改动 N 条边（新增拥堵 X 条，转为畅通 Y 条）」"
    want "多按几次应能同时看到「新增拥堵」与「转为畅通」两种情况"
    want "若受影响的边在当前路径上且增幅达标，紧跟一行"
    want "  「受影响边位于当前路径且增幅达标 -> 自动触发重规划」"
    want "再紧跟一行，明确说明采用了哪种方式："
    want "  「→ 增量式重规划：仅重算受影响的路段，其余原样保留」（单趟时）"
    want "  「→ 上一版为多趟方案，退回全量重算」（多趟时）"
    want "重规划后红色高亮与「总耗时」更新"
    pause
    ask_result "模拟路况与增量重规划（E1/D22）"

    # ---------------------------------------------------------------- 9
    stage "插入紧急订单（E3）"
    todo "**连点 3–4 次**工具栏的「插入紧急订单」。"
    say ""
    head1 "预期："
    want "日志出现「插入紧急订单 U001 @ <配送点>（货量 X kg，要求 时:分 前送达）」"
    want "「订单列表」的**第一行**就是这些 U 开头的订单，状态显示「紧急」"
    want "它们的窗口只有 **1 小时**（形如 10:11-11:11）"
    want "路线里它们被**优先服务**（排在最前面）"
    want "每次点击都应新增一条——**不应只剩最后一单**"
    dim "默认数据集里本来**没有任何紧急订单**，紧急单只能靠这里动态产生。"
    pause
    ask_result "插入紧急订单（E3）"

    # ---------------------------------------------------------------- 10
    stage "动态增删：新客户 / 道路封闭 / 手工增删（B5）"
    say "依次做三件事，都在同一次会话里。"
    say ""
    printf '  %s① 模拟新客户%s\n' "$BOLD" "$RESET"
    todo "点「模拟新客户」"
    want "画布上多出一个绿色配送点（编号 X###），连到最近的既有节点（双向都有箭头）"
    want "日志「模拟新客户：新增配送点 X### 与订单 C###（货量 X kg）」"
    want "路线站点数 +1（例如 25 → 26），完整序列里出现该节点"
    say ""
    printf '  %s② 模拟道路封闭%s\n' "$BOLD" "$RESET"
    todo "点「模拟道路封闭」"
    want "日志「模拟道路封闭：<A>-><B>（含反向）」，对应边从画布消失"
    want "**仍然「规划成功」** —— 不会出现「无法从 … 到达 …」的不可行"
    say ""
    printf '  %s③ 手工增删（含健壮性）%s\n' "$BOLD" "$RESET"
    todo "点「手工增删…」"
    todo "在「起点/终点 ID」填两个尚未直连的既有节点，点「添加边」"
    want "日志「手工添加边 A <-> B（双向）」，画布上两个方向各一条边"
    todo "用同一对 ID 点「删除边」→ 两个方向的边一并消失"
    todo "在「节点 ID」填一个不存在的 ID（如 NOPE），点「删除节点」"
    want "日志报「删除节点失败：NOPE 不存在」，**程序不得崩溃或卡死**"
    todo "做完关闭对话框"
    pause
    ask_result "动态增删：新客户 / 道路封闭 / 手工增删（B5）"

    # ---------------------------------------------------------------- 11
    stage "推进一站：车辆位置、当前载重与暂存随动（B6）"
    todo "点几次「推进一站」，同时盯住右侧三个地方。"
    say ""
    head1 "预期："
    want "画布上那圈**品红环**自动移动到刚到达的节点（不必点「重新规划」）"
    want "已走过的路段**不再标红**，只有尚未走完的路线高亮"
    want "「车辆信息」的「**当前载重**」随送达**递减**，且始终 ≤ 载重上限"
    want "日志区分「送达 XX（到达 时:分）」与「经过 XX」；若某趟确实经过中转站，还会补注「入库暂存 / 取货配发」"
    say ""
    todo "切到「中转站 / 集散」标签页，继续推进"
    want "本轮默认数据下中转站无存货，两列**应始终为 0**——这正是「不绕站」的直接体现"
    dim "（若将来引入了「顺路寄存」产生库存，这两列会随之实时变化）"
    say ""
    todo "一直推到全部送完"
    want "最后一步是「返回仓库 W01（到达 时:分）」，品红环回到 W01"
    want "再点「重新规划」：已送达站点不再出现在新路线里，penalty 不暴涨"
    pause
    ask_result "推进一站：车辆位置、当前载重与暂存随动（B6）"

    # ---------------------------------------------------------------- 12
    stage "Debug 模式（A1）"
    todo "先用工具栏的「Debug 速度」选「展示模式（2 秒/步）」，再勾选「Debug 模式」"
    say ""
    head1 "预期："
    want "日志每 2 秒增长一次：自动模拟路况、偶尔插单、自动推进、偶发道路封闭"
    want "画布高亮与车辆标记随之变化"
    todo "把速度切到「快速预览（1 秒/步）」"
    want "节奏立刻变快一倍，日志出现「Debug 速度切换为每 1 秒一步」"
    todo "最后**取消勾选「Debug 模式」**"
    want "**必须立刻停下**——日志不再新增，界面不再变化"
    pause
    ask_result "Debug 模式（A1）"

    # ---------------------------------------------------------------- 13
    stage "边界：制造不可行场景（E2 边界）"
    say "本关故意把网络弄断，验证程序给出明确原因而不是崩溃或静默。"
    say ""
    todo "点「手工增删…」，在「节点 ID」填一个配送点（例如 D01），点「删除节点」，关闭对话框"
    say ""
    head1 "预期："
    want "「路线信息」显示不可行，且**原因里明确指出哪个节点不可达**"
    want "程序不崩溃，工具栏其他按钮仍可继续点击"
    pause
    ask_result "边界：制造不可行场景（E2 边界）"
    stop_app
}

main "$@"
summary
