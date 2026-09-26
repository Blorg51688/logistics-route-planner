#!/usr/bin/env bash
# 守护「双击人工测试向导.sh 无反应」的修复。
#
# 缺陷背景：本机 KDE 的 KIO 设置 behaviourOnLaunch=execute（~/.config/kiorc），
# 文件管理器双击可执行脚本 = 直接运行、**不分配终端**。向导必须读按键，于是它
# 打印一段"请在终端里运行"后 exit 2——可这段输出没有任何终端可显示，用户看到的
# 就是「双击毫无反应」。修复后：入口在**没有控制终端**时自己在终端模拟器里重启。
#
# 这里用「桩终端」把这件事变成可观察的：把假的 konsole 放到 PATH 最前，它只把
# 收到的参数写进日志。于是"入口是否真的去开终端、开的命令对不对"能被断言，
# 而不需要真的弹出窗口、也不需要人来点。
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LAUNCHER="$ROOT/人工测试向导.sh"
fail=0

work="$(mktemp -d)"
log="$work/stub.log"
stub_dir="$work/bin"
mkdir -p "$stub_dir"
cat > "$stub_dir/konsole" <<'STUB'
#!/usr/bin/env bash
{ printf '%s\n' "$*"; } >> "$STUB_LOG"
STUB
chmod +x "$stub_dir/konsole"
cleanup() { rm -rf "$work"; }
trap cleanup EXIT

# 等桩日志出现（后台 setsid 启动，给一点时间；最多 ~2s，避免用固定 sleep 造成偶发失败）
wait_for_log() {
    local i
    for i in $(seq 1 40); do
        [[ -s "$log" ]] && return 0
        sleep 0.05
    done
    return 1
}

# ---------------------------------------------------------------------------
# 场景 1：没有控制终端（= 双击的处境）+ 图形环境 -> 必须在终端里重启自己。
# 用 </dev/null 且本脚本自身无 /dev/tty，等价于文件管理器直接执行。
# ---------------------------------------------------------------------------
: > "$log"
STUB_LOG="$log" PATH="$stub_dir:$PATH" DISPLAY=:0 \
    bash "$LAUNCHER" --only 3 </dev/null >/dev/null 2>&1
rc1=$?
if ! wait_for_log; then
    echo "FAIL  无终端时入口没有启动终端模拟器——双击仍会「毫无反应」"
    echo "      （期望在 PATH 里调用 konsole；实际桩未被调用，入口 exit=$rc1）"
    fail=1
else
    got="$(cat "$log")"
    # 必须重新运行**入口脚本本身**，而不是别的什么东西
    if ! grep -qF -- "人工测试向导.sh" <<<"$got"; then
        echo "FAIL  终端里重启的命令没有指向入口脚本：$got"
        fail=1
    fi
    # 必须带防重启标记：否则若终端也没给 TTY，会无限自我重启
    if ! grep -qF -- "WIZARD_RELAUNCHED=1" <<<"$got"; then
        echo "FAIL  重启命令缺少 WIZARD_RELAUNCHED=1 防重启标记：$got"
        fail=1
    fi
    # 用户传的参数（--only 3）必须原样带到新终端里
    if ! grep -qF -- "--only 3" <<<"$got"; then
        echo "FAIL  重启命令丢失了用户参数（--only 3）：$got"
        fail=1
    fi
    if [[ "$rc1" -ne 0 ]]; then
        echo "FAIL  重启成功后入口应正常退出 0，实际 exit=$rc1"
        fail=1
    fi
fi

# ---------------------------------------------------------------------------
# 场景 2：--list 是只读用法，**绝不能**被"双击兜底"抢走去开终端。
# 入口注释专门保证过这一点（提前拦截会破坏非交互式列出关卡）。
# ---------------------------------------------------------------------------
: > "$log"
list_out="$work/list.out"
STUB_LOG="$log" PATH="$stub_dir:$PATH" DISPLAY=:0 \
    bash "$LAUNCHER" --list </dev/null >"$list_out" 2>&1
sleep 0.2
if [[ -s "$log" ]]; then
    echo "FAIL  --list 被当成交互运行、去开终端了（它应保持只读、直接打印）"
    fail=1
fi
if ! grep -q "构建与自动检查" "$list_out"; then
    echo "FAIL  --list 没有正常列出关卡；实际输出：$(head -1 "$list_out")"
    fail=1
fi

# ---------------------------------------------------------------------------
# 场景 3：连终端模拟器都没有 + 没有图形环境 -> 必须给一句**看得见、可照做**的话，
# 且非零退出。旧行为是打印向导内部的拒绝信息（用户看不到）——那正是"无反应"。
# PATH 裁到只剩 dirname/bash，模拟"机器上一个终端程序都没有"。
# ---------------------------------------------------------------------------
min_dir="$work/min"
mkdir -p "$min_dir"
ln -s "$(command -v dirname)" "$min_dir/dirname"
ln -s "$(command -v bash)" "$min_dir/bash"
out3="$(env -u DISPLAY -u WAYLAND_DISPLAY PATH="$min_dir" "$min_dir/bash" "$LAUNCHER" </dev/null 2>&1)"
rc3=$?
if [[ "$rc3" -eq 0 ]]; then
    echo "FAIL  没有终端也没有图形环境时入口竟然成功退出——用户仍会一头雾水"
    fail=1
fi
if ! grep -q "无法自动打开终端" <<<"$out3"; then
    echo "FAIL  没有终端时入口没有给出可照做的说明；实际输出：$(head -2 <<<"$out3" | tr '\n' ' ')"
    fail=1
fi

# ---------------------------------------------------------------------------
# 场景 4（可选，需 util-linux 的 script）：**有**控制终端、只是 stdin 被重定向时，
# 不得误判成双击去另开终端——脚本头部写明"管道/重定向里运行会明确拒绝"是有意设计。
# ---------------------------------------------------------------------------
if command -v script >/dev/null 2>&1; then
    : > "$log"
    script -qec "STUB_LOG='$log' PATH='$stub_dir:$PATH' DISPLAY=:0 bash '$LAUNCHER' </dev/null" \
        /dev/null >/dev/null 2>&1
    sleep 0.2
    if [[ -s "$log" ]]; then
        echo "FAIL  在终端里、仅 stdin 被重定向时被误判成双击而新开了终端（破坏了'重定向即拒绝'）"
        fail=1
    fi
fi

if [[ "$fail" -eq 0 ]]; then
    echo "check_wizard_terminal: 双击兜底正常（无终端时自动开终端；--list 只读；无终端时给人话）"
fi
exit "$fail"
