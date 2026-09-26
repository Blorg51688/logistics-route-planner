#!/usr/bin/env bash
#
# 入口：人工测试向导（8 关，逐关引导你验证图形界面）
#
#   终端里：  bash 人工测试向导.sh
#   文件管理器里：双击本文件。没有终端时它会**自动在终端模拟器里重启自己**
#                 （见下方「双击兜底」）——因为向导要读你的按键。
#
# 它是干什么的：单元/集成测试覆盖不了 GUI 的观感与交互（箭头是否画对、标签是否遮挡、
# 拖动节点时边是否跟随、按钮反馈是否符合直觉）。本向导逐关告诉你「现在该点哪里、
# 该看到什么」，你确认后它记一关；跑完把结果写进 docs/人工测试记录.md。
#
# 多久：全程约 5–6 分钟。可以只跑某一关：  bash 人工测试向导.sh --only 3
# 列出全部关卡：                          bash 人工测试向导.sh --list
# 查看本入口的用法：                      bash 人工测试向导.sh --help
#
# 注意：向导需要真实终端（要读你的按键）。管道 / 重定向里运行它会明确拒绝——
# 这是有意设计，避免在没有人的情况下「假装跑过」。

set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ---- 先做参数预扫，再决定是否构建 -------------------------------------------
# 这一点很重要：`--list` 是**只读**用法（只打印关卡名单），既不需要图形界面，
# 也不需要编译。如果先无条件构建、再被「缺 Qt6」的门禁挡住，就会出现
# 「文档说支持 --list，实际却 exit 3」的自相矛盾。
want_list=0
want_help=0
have_only=0
prev=""
norm=()   # 规范化后的参数：把 --only=X 拆成 --only X 再透传
for a in "$@"; do
    case "$a" in
        # 等号写法（--only=3）在这里归一化。必须做：向导本体**不解析** --only=3，
        # 直接透传会被它当未知参数丢掉，于是"只跑一关"静默变成"跑完整 8 关"。
        --only=*) have_only=1; norm+=("--only" "${a#--only=}"); continue ;;
    esac
    [[ "$prev" == "--only" ]] && have_only=1
    case "$a" in
        --list)    want_list=1 ;;
        --help|-h) want_help=1 ;;
    esac
    norm+=("$a")
    prev="$a"
done
set -- ${norm[@]+"${norm[@]}"}

# `--help` 由本入口自己处理：向导本体**没有** --help（传给它只会被忽略并进入交互），
# 所以这里不能假装"向导会处理"。
if (( want_help )); then
    cat <<'USAGE'
用法： bash 人工测试向导.sh [选项]

  无参数            从头到尾跑完 8 关（需要图形界面；约 5–6 分钟）
  --only <关卡>     只跑一关（需要图形界面）。关卡可写序号、关名或唯一片段，例如：
                      --only 3          --only=3   （两种写法等价）
                      --only 停靠明细
                      --only 边界
  --list            只列出全部关卡，不交互、不构建
  -h, --help        显示本帮助

说明：向导需要真实终端（要读你的按键）；在管道 / 重定向中运行会被明确拒绝。
      缺 Qt6（没有图形界面）时，看界面的关卡都跑不了，请改用 `cd build && ctest`。
USAGE
    exit 0
fi

# ============================ 双击兜底 ============================
# 文件管理器「双击」在多数桌面（含本机 KDE：~/.config/kiorc 的
# behaviourOnLaunch=execute）里 = **直接运行、不分配终端**。向导必须读你的按键，
# 于是它会打印一段"请在终端里运行"——可这段输出没有任何终端可显示，
# 用户看到的就是「双击毫无反应」。
#
# 这里主动在终端模拟器里重启自己。判据：stdin 不是终端，**且**没有控制终端
# （`/dev/tty` 打不开）。后半条很关键——「在终端里 `... < file`」是有意拒绝的
# 用法（脚本头部写明"管道/重定向里运行会明确拒绝"），那种情况有控制终端，
# 不能误判成双击而去另开一个终端。
if [[ ! -t 0 ]] && [[ -z "${WIZARD_RELAUNCHED:-}" ]] \
   && ! (exec 3</dev/tty) 2>/dev/null \
   && (( want_list == 0 && want_help == 0 )); then
    # 重启命令：置标记（防止终端也没给 TTY 时无限自我重启）+ 原样带回参数
    printf -v _wiz_cmd 'WIZARD_RELAUNCHED=1 exec bash %q' "$ROOT/人工测试向导.sh"
    for _a in "$@"; do printf -v _wiz_cmd '%s %q' "$_wiz_cmd" "$_a"; done

    _wiz_term=""
    for _t in konsole gnome-terminal xfce4-terminal mate-terminal tilix kgx \
              alacritty kitty x-terminal-emulator xterm; do
        if command -v "$_t" >/dev/null 2>&1; then _wiz_term="$_t"; break; fi
    done

    if [[ -n "$_wiz_term" ]]; then
        case "$_wiz_term" in
            gnome-terminal)
                setsid "$_wiz_term" -- bash -c "$_wiz_cmd" >/dev/null 2>&1 & ;;
            *)
                setsid "$_wiz_term" -e bash -c "$_wiz_cmd" >/dev/null 2>&1 & ;;
        esac
        exit 0
    fi

    # 连终端都没有：至少把"无反应"变成"看得见、可照做"的一句话。
    _wiz_msg="无法自动打开终端来运行人工测试向导。请打开终端后执行：  bash 人工测试向导.sh"
    if [[ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]]; then
        if command -v kdialog >/dev/null 2>&1; then
            setsid kdialog --error "$_wiz_msg" >/dev/null 2>&1 &
            exit 0
        elif command -v zenity >/dev/null 2>&1; then
            setsid zenity --error --text="$_wiz_msg" >/dev/null 2>&1 &
            exit 0
        elif command -v xmessage >/dev/null 2>&1; then
            setsid xmessage -center "$_wiz_msg" >/dev/null 2>&1 &
            exit 0
        fi
    fi
    printf '%s\n' "$_wiz_msg" >&2
    exit 2
fi

# shellcheck source=scripts/ensure_build.sh
source "$ROOT/scripts/ensure_build.sh"

if (( want_list )); then
    # 纯只读用法：直接交给向导，跳过一切构建与门禁。
    exec bash "$ROOT/scripts/manual_test_wizard.sh" --list
fi

rc=0
ensure_build "$ROOT" || rc=$?

case "$rc" in
    0) ;;
    2)
        if (( have_only )); then
            # 指定了单关就不替用户判断"这一关需不需要图形界面"——
            # 关名支持序号/关名/片段三种写法，在这里复刻一套解析只会再次不同步。
            # 交给向导按关卡自行判断（需要界面的关卡会自己报错）。
            eb_warn "本机没有编译出 GUI 程序；你指定了 --only，交由向导按该关卡自行判断。"
        else
            eb_err "本机没有编译出 GUI 程序（多半缺 Qt6 开发包），因此**看界面**的关卡都跑不了。"
            eb_err "  装上后重试：  sudo zypper install qt6-base-devel"
            eb_err "  现在就能跑、不需要图形界面的："
            eb_err "    · 自动化测试：cd build && ctest   （718 项断言）"
            eb_err "    · 关卡名单：  bash 人工测试向导.sh --list"
            exit 3
        fi
        ;;
    *)
        eb_err "无法进入向导：构建未能完成（原因见上方输出）。"
        exit 1
        ;;
esac

# 原样透传参数（--only 等）。
# 这里刻意**不做** TTY 预检：向导自身在解析完 --only/--list 之后才检查终端，
# 提前拦截会破坏「非交互式列出关卡」这种合法用法。
exec bash "$ROOT/scripts/manual_test_wizard.sh" "$@"
