#!/usr/bin/env python3
"""检查 markdown 内部的相对链接是否真的指向存在的文件。

为什么需要：项目里已经发生过一次——`文档导航.md` 被放在根目录，但链接是按
`docs/` 位置写的，于是**11 条链接全部失效**。"能点到的文档入口"是本项目的
交付目标之一，死链会让它当场失效，而纯靠人眼很难发现。

检查范围：仓库根与 `docs/` 下的所有 `*.md`（跳过 `.omd/`、`build/`、`.git/`）。
判定规则：
  · 只检查**相对**链接的路径部分；
  · 跳过 http/https/mailto、纯锚点（`#...`）、以及含 `<...>` 占位符的目标；
  · 结尾是 `/` 的目录链接，检查目录是否存在。

退出码：0 = 全部可解析；1 = 存在失效链接（打印 file:line → 目标）。
"""

from __future__ import annotations

import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

SKIP_DIRS = {".git", ".omd", "build", "cmake-build-debug", "cmake-build-release"}

# markdown 行内链接 `](目标)`；目标里不允许出现空格以外的花括号（本项目的链接形态很规整）
LINK_RE = re.compile(r"\]\(\s*([^)\s]+)(?:\s+\"[^\"]*\")?\s*\)")


def iter_markdown_files() -> list[str]:
    found: list[str] = []
    for base, dirs, files in os.walk(ROOT):
        dirs[:] = [d for d in dirs if d not in SKIP_DIRS]
        for name in files:
            if name.endswith(".md"):
                found.append(os.path.join(base, name))
    return sorted(found)


def is_external(target: str) -> bool:
    lowered = target.lower()
    if lowered.startswith(("http://", "https://", "mailto:", "tel:")):
        return True
    if target.startswith("#"):
        return True
    # 允许带锚点：只校验路径部分
    if "<" in target or ">" in target:  # 占位符
        return True
    return False


def check_file(path: str) -> list[str]:
    problems: list[str] = []
    with open(path, encoding="utf-8") as fh:
        for lineno, line in enumerate(fh, 1):
            for raw in LINK_RE.findall(line):
                if is_external(raw):
                    continue
                target = raw.split("#", 1)[0]
                if not target:
                    continue
                resolved = os.path.normpath(os.path.join(os.path.dirname(path), target))
                if not os.path.exists(resolved):
                    rel_src = os.path.relpath(path, ROOT)
                    problems.append(f"{rel_src}:{lineno} → {raw}")
    return problems


def main() -> int:
    files = iter_markdown_files()
    problems: list[str] = []
    for path in files:
        problems.extend(check_file(path))

    if problems:
        print(f"[doc_links_lint] 发现 {len(problems)} 条失效链接（检查了 {len(files)} 个 md 文件）：")
        for item in problems:
            print(f"  ✗ {item}")
        print("提示：链接是相对于**该 md 文件所在目录**解析的；根目录的文档指向 docs/ 时要写全 `docs/…`。")
        return 1

    print(f"[doc_links_lint] 通过：{len(files)} 个 md 文件中的相对链接全部可解析。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
