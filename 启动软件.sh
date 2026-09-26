#!/usr/bin/env bash
#
# 入口：启动主软件（图形界面）
#
#   终端里：  bash 启动软件.sh
#   文件管理器里：双击本文件（选「在终端中运行」）
#
# 不需要任何前置准备：若尚未构建，本脚本会自动完成 CMake 配置与编译，然后打开主程序窗口。
# 想先跑测试或看命令行结果？见同目录 README.md。
#
# 透传参数：本脚本把命令行参数原样交给主程序，例如
#   bash 启动软件.sh --plan distance      # 启动即按「最短距离」策略规划并高亮
#   bash 启动软件.sh --help               # 查看主程序全部命令行选项

set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=scripts/ensure_build.sh
source "$ROOT/scripts/ensure_build.sh"

rc=0
ensure_build "$ROOT" || rc=$?

case "$rc" in
    0) ;;
    2)
        eb_err "无法启动图形界面：本机没有编译出 GUI 程序（多半缺 Qt6 开发包，见上方提示）。"
        eb_err "  装上后重试：  sudo zypper install qt6-base-devel"
        eb_err "  在此之前可用：cd build && ctest    （跑一遍 790 项自动化断言，无需图形界面）"
        exit 3
        ;;
    *)
        eb_err "启动中止：构建未能完成（原因见上方输出）。"
        exit 1
        ;;
esac

# 没有显示环境时直接启动会崩在 Qt 初始化上 —— 先给可操作的替代路径。
#
# 只在「不带参数」时拦截：不带参数就是要求打开交互窗口，没有显示必然失败。
# 带了参数则交给主程序自己判断——因为 --help / --plan-summary / --dump-graph /
# --ui-probe / --render 这些模式本来就不需要显示环境，不该被这里误拦。
if (( $# == 0 )) && [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
    eb_err "当前环境没有图形显示（DISPLAY 与 WAYLAND_DISPLAY 都为空），无法弹出窗口。"
    eb_err "如果你在远程 / 无界面环境里，可以这样查看运行结果："
    eb_err "  · 规划汇总（纯文本）：   bash 启动软件.sh --plan-summary distance"
    eb_err "  · 把界面渲染成图片：     QT_QPA_PLATFORM=offscreen ./build/app --render-window /tmp/gui.png"
    eb_err "  · 打印邻接表 / 邻接矩阵：bash 启动软件.sh --dump-graph list"
    eb_err "  · 跑自动化测试：         cd build && ctest"
    exit 4
fi

eb_step "启动主程序…"
# 用 ensure_build 校验过并回传的路径，不要自己拼 "$ROOT/build/app"：
# 设了 BUILD_DIR 时两者会不一致（校验的是一处、启动的是另一处）。
exec "$EB_APP" "$@"
