#!/bin/bash
# docs の HTML（設計書・操作手順書）から PDF を作り直す。HTML を直したら実行して、PDF も一緒にコミットする。
#   tools/build_pdf.sh
# Chrome（Mac）か chromium / google-chrome（Linux）を使う。ページ番号は HTML の CSS で付く。
set -e
cd "$(dirname "$0")/../docs"
for c in "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome" google-chrome chromium chromium-browser; do
  if command -v "$c" >/dev/null 2>&1 || [ -x "$c" ]; then CHROME="$c"; break; fi
done
[ -n "$CHROME" ] || { echo "Chrome が見つかりません"; exit 1; }
for n in 設計書 操作手順書; do
  "$CHROME" --headless=new --disable-gpu --no-pdf-header-footer --print-to-pdf="$PWD/$n.pdf" "file://$PWD/$n.html" >/dev/null 2>&1
  echo "作りました：docs/$n.pdf"
done
