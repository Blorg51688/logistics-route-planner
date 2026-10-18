#!/usr/bin/env bash
# 渲染 docs/figures/src/*.mmd → docs/figures/*.png
# 依赖：npx（随 Node.js 安装）、chrome-headless-shell（首次运行自动下载到 ~/.cache/puppeteer）
set -euo pipefail
cd "$(dirname "$0")/../../.."

CHROME=$(ls "$HOME"/.cache/puppeteer/chrome-headless-shell/*/chrome-headless-shell-linux64/chrome-headless-shell 2>/dev/null | head -1)
if [ -z "$CHROME" ]; then
  echo "✗ 找不到 chrome-headless-shell，请先运行：npx -y puppeteer browsers install chrome-headless-shell" >&2
  exit 1
fi
PPTR=$(mktemp /tmp/pptr-XXXX.json)
trap 'rm -f "$PPTR"' EXIT
printf '{"executablePath":"%s","args":["--no-sandbox","--disable-gpu"]}\n' "$CHROME" > "$PPTR"

for f in docs/figures/src/*.mmd; do
  name=$(basename "$f" .mmd)
  out="docs/figures/$name.png"
  echo "→ $name"
  npx -y @mermaid-js/mermaid-cli@11 -i "$f" -o "$out" -w 1600 -b white -p "$PPTR" 2>&1 | grep -v '^npm notice' || true
done
echo "✓ 全部渲染完成"
