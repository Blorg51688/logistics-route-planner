#!/usr/bin/env bash
#
# 共享的「确保已构建」逻辑 —— 供根目录的入口脚本 source，不要单独双击。
#
# 为什么单独抽出来：根目录的「启动软件.sh」与「人工测试向导.sh」都要先保证
# 构建产物可用。这段逻辑必须只有一份，否则两处会各自漂移。
#
# 用法：
#   source "$ROOT/scripts/ensure_build.sh"
#   ensure_build "$ROOT"
#     返回 0 = build/app 可用（并把可用路径导出到 $EB_APP）
#          2 = 编译成功，但没有生成 GUI 目标（通常是缺 Qt6 开发包）
#          1 = 失败（缺工具链 / 配置失败 / 编译失败 / 拒绝越界删除）
#
# 可覆盖的环境变量：
#   BUILD_DIR  构建目录（默认 <root>/build）
#   JOBS       并行编译任务数（默认 CPU 核数）
#
# 输出约定：本文件（以及它调用的 cmake）**只写 stderr**，
# 好让 stdout 完整留给被启动的程序本身。

# ---- 输出助手：只在终端上色；被重定向到文件时保持纯文本 ----
if [[ -t 2 ]]; then
    _EB_RED=$'\033[31m'; _EB_GRN=$'\033[32m'; _EB_YLW=$'\033[33m'
    _EB_BLD=$'\033[1m'; _EB_RST=$'\033[0m'
else
    _EB_RED=; _EB_GRN=; _EB_YLW=; _EB_BLD=; _EB_RST=
fi

eb_step() { printf '%s==>%s %s\n' "$_EB_BLD" "$_EB_RST" "$*" >&2; }
eb_ok()   { printf '%s  ✓%s %s\n'  "$_EB_GRN" "$_EB_RST" "$*" >&2; }
eb_warn() { printf '%s  ! %s%s\n'  "$_EB_YLW" "$*" "$_EB_RST" >&2; }
eb_err()  { printf '%s  ✗ %s%s\n'  "$_EB_RED" "$*" "$_EB_RST" >&2; }

ensure_build() {
    local root="${1:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
    local build_dir="${BUILD_DIR:-$root/build}"
    local jobs="${JOBS:-}"

    # JOBS 必须是正整数；否则 cmake 会抛一条与用户输入毫无关系的晦涩错误。
    if [[ ! "$jobs" =~ ^[1-9][0-9]*$ ]]; then
        jobs="$( (getconf _NPROCESSORS_ONLN 2>/dev/null) || echo 4 )"
        [[ "$jobs" =~ ^[1-9][0-9]*$ ]] || jobs=4
    fi

    # 1) 工具链
    local missing=()
    command -v cmake >/dev/null 2>&1 || missing+=(cmake)
    if ! command -v g++ >/dev/null 2>&1 && ! command -v c++ >/dev/null 2>&1; then
        missing+=(g++)
    fi
    if [[ "${#missing[@]}" -gt 0 ]]; then
        eb_err "缺少构建工具：${missing[*]}"
        eb_err "请先安装（openSUSE 例：sudo zypper install cmake gcc-c++），然后重试。"
        return 1
    fi

    # 2) 构建目录是否属于「另一个位置」。
    #    真实踩过的坑：工作区被移动/改名后，CMakeCache.txt 里仍记着旧绝对路径，
    #    于是 cmake --build 直接报
    #      CMakeCache.txt directory ... is different than the directory ... was created
    #    这里比对缓存的 CMAKE_HOME_DIRECTORY，不符就重建构建目录。
    if [[ -f "$build_dir/CMakeCache.txt" ]]; then
        local cached
        cached="$(sed -n 's/^CMAKE_HOME_DIRECTORY:INTERNAL=//p' "$build_dir/CMakeCache.txt" | head -1)"
        if [[ -n "$cached" && "$cached" != "$root" ]]; then
            eb_warn "构建目录记录的是另一个位置（工作区被移动过？），需要重建："
            eb_warn "    缓存里：$cached"
            eb_warn "    当前：  $root"

            # ---- 删除前的安全闸门（这一段必须保守，错了就是删别人的目录）----
            # 只做字符串前缀比较是不够的：BUILD_DIR 可以由用户/环境覆盖，
            # `$root/../别处` 能通过前缀检查、`$root/build` 若是指向别处的符号链接
            # 也会让 `rm -rf build/` 穿透链接删掉目标内容。因此先做路径规范化，
            # 再要求结果**恰好等于** <规范 root>/build —— 白名单，其余一律拒绝。
            local real_root real_build
            real_root="$(realpath -m -- "$root" 2>/dev/null || printf '%s' "$root")"
            real_build="$(realpath -m -- "$build_dir" 2>/dev/null || printf '%s' "$build_dir")"

            if [[ -n "$build_dir" && "$build_dir" != "/" && "$real_build" == "$real_root/build" ]]; then
                rm -rf -- "$real_build"
            else
                eb_err "拒绝自动删除：构建目录不是 <工作区>/build。"
                eb_err "    传入：  $build_dir"
                eb_err "    解析为：$real_build（应为 $real_root/build）"
                eb_err "为安全起见本脚本不会删除它。请确认后自行处理，例如："
                eb_err "    rm -rf -- '$real_build'"
                return 1
            fi
        fi
    fi

    # 3) 配置（仅首次或缓存被清后）
    if [[ ! -f "$build_dir/CMakeCache.txt" ]]; then
        eb_step "首次构建：配置 CMake…"
        # 构建过程的所有输出一律进 stderr：stdout 必须留给被启动的程序本身。
        # 否则 `bash 启动软件.sh --plan-summary distance > 结果.txt` 会把
        # 「Built target ...」之类的构建日志一起写进结果文件。
        if ! cmake -S "$root" -B "$build_dir" -DCMAKE_BUILD_TYPE=Release >&2; then
            eb_err "CMake 配置失败（原因见上方 cmake 输出）。"
            return 1
        fi
    fi

    # 4) 编译（增量；无改动时几乎不耗时）
    eb_step "编译（增量）…"
    if ! cmake --build "$build_dir" -j "$jobs" >&2; then
        eb_err "编译失败（原因见上方 cmake 输出）。"
        return 1
    fi

    # 5) GUI 目标是否真的生成了。
    #    CMakeLists.txt 在找不到 Qt6 时会「静默跳过」app 目标（只打印一行 STATUS），
    #    所以这里必须显式检查，否则用户只会看到一句莫名其妙的「启动失败」。
    if [[ ! -x "$build_dir/app" ]]; then
        eb_warn "编译成功，但没有生成图形界面程序 app（$build_dir/app）。"
        eb_warn "最常见原因：本机缺少 Qt6 开发包（CMake 找不到 Qt6 时会自动跳过 GUI 目标）。"
        eb_warn "  · 装上后重试：  sudo zypper install qt6-base-devel"
        eb_warn "  · 不装也能验证核心逻辑：cd build && ctest   （718 项自动化断言，不需要图形界面）"
        return 2
    fi

    # 把**校验过的**可执行文件路径回传给调用方。
    # 调用方必须 exec "$EB_APP"，不能再自己拼 $root/build/app —— 否则设了 BUILD_DIR
    # 时会出现「这里说构建就绪、那里 exec 找不到文件」的分裂。
    EB_APP="$build_dir/app"
    export EB_APP

    eb_ok "构建就绪：$EB_APP"
    return 0
}

# 本文件只定义函数；被直接执行（双击）时给一句人话，而不是静默退出 0。
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    printf '%s本文件只提供函数，请由入口脚本 source，不要直接运行。%s\n' "$_EB_YLW" "$_EB_RST" >&2
    printf '请改用根目录的入口：  bash 启动软件.sh     或     bash 人工测试向导.sh\n' >&2
    exit 1
fi
