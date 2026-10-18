#!/usr/bin/env bash
#
# 快捷方式：启动主软件（图形界面）
#
# 这是**薄封装**——真正的实现只有一份，在项目根的 `启动软件.sh`。
# 本脚本只做三件事：定位项目根 → cd 过去 → exec 原脚本（参数原样透传）。
# 因此原脚本的一切行为（自动构建、缺 Qt6 的提示、无显示环境时的替代路径）都照旧生效。
#
# 用法（二选一）：
#   bash start-here/run/启动软件.sh
#   双击本文件后选「在终端中运行」
#
# 透传示例：
#   bash start-here/run/启动软件.sh --plan distance
#   bash start-here/run/启动软件.sh --help

set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

if [[ ! -f "$ROOT/启动软件.sh" ]]; then
    echo "找不到项目根的 启动软件.sh（推算出的项目根是：$ROOT）" >&2
    echo "本脚本必须留在 <项目根>/start-here/run/ 下，不要单独复制出去。" >&2
    exit 1
fi

cd "$ROOT" || exit 1
exec bash "$ROOT/启动软件.sh" "$@"
