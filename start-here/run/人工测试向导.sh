#!/usr/bin/env bash
#
# 快捷方式：人工测试向导（8 关，约 5–6 分钟）
#
# 这是**薄封装**——真正的实现只有一份，在项目根的 `人工测试向导.sh`。
# 本脚本只做三件事：定位项目根 → cd 过去 → exec 原脚本（参数原样透传）。
# 注意：向导**需要真实终端**（要读你的按键），管道 / 重定向里运行会被它明确拒绝——
# 那是原脚本的有意设计，避免"没有人的情况下假装跑过"。
#
# 用法（二选一）：
#   bash start-here/run/人工测试向导.sh
#   双击本文件（没有终端时原脚本会自动在终端模拟器里重启自己）
#
# 透传示例：
#   bash start-here/run/人工测试向导.sh --list        # 列出全部关卡（只读，不构建）
#   bash start-here/run/人工测试向导.sh --only 3      # 只跑第 3 关
#   bash start-here/run/人工测试向导.sh --help

set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

if [[ ! -f "$ROOT/人工测试向导.sh" ]]; then
    echo "找不到项目根的 人工测试向导.sh（推算出的项目根是：$ROOT）" >&2
    echo "本脚本必须留在 <项目根>/start-here/run/ 下，不要单独复制出去。" >&2
    exit 1
fi

cd "$ROOT" || exit 1
exec bash "$ROOT/人工测试向导.sh" "$@"
