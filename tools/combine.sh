#!/bin/bash
# iPhone で撮ったロボットの動画と、指の検出の動画（bag_to_mp4.py で作ったもの）を、時刻を合わせて左右に並べる（Mac で実行する）
#   tools/combine.sh <iPhone の動画.MOV> <指の検出.mp4> [ずれの補正 秒] [出力.mp4]
#   右の見出しを変えるときは LABEL='ダンスの検出（MediaPipe）' tools/combine.sh … のようにする
#   iPhone の動画が横倒しに写っているときは ROTATE=ccw（左に 90°）、cw（右に 90°）、180 を付ける
# ・時刻は、iPhone の撮影開始時刻（動画の creationdate）と、指の検出の最初の画像の時刻（<指の検出>.json）で合わせる。
#   どちらも Mac・iPhone の時計（ネットの時刻に合っている）なので、ずれは 1 秒程度。
# ・ずれの補正：指の検出の動画が遅れて見えるときは + 、早く見えるときは − の秒数を指定する（例: 0.5、-0.3）
# ・必要なもの：ffmpeg（libfreetype 付き）、ヒラギノ角ゴシック（文字を書くため）
# ・音声は iPhone の1本目（AAC）だけを使う（iPhone の空間オーディオ（APAC）は ffmpeg 5 で読めないため）
# ・どちらの動画も、始まりの時刻を 0 にそろえて 30 fps にしてから並べる（iPhone の HDR 動画は時刻の付け方が特殊で、
#   そのまま並べると同じ画像を何万枚も書き続けてしまうため）
set -e
movie="$1"; finger="$2"; adjust="${3:-0}"
[ -f "$movie" ] && [ -f "$finger" ] || { echo "使い方: tools/combine.sh <iPhone の動画.MOV> <指の検出.mp4> [ずれの補正 秒] [出力.mp4]"; exit 1; }
out="${4:-${movie%.*}_combined.mp4}"
json="${finger%.*}.json"
case "${ROTATE:-}" in
    ccw) rot="transpose=2," ;;
    cw) rot="transpose=1," ;;
    180) rot="transpose=1,transpose=1," ;;
    *) rot="" ;;
esac
font="/System/Library/Fonts/ヒラギノ角ゴシック W6.ttc"
[ -f "$font" ] || font="/System/Library/Fonts/ヒラギノ角ゴシック W3.ttc"

# iPhone の撮影開始時刻（例: 2026-10-26T11:15:17+0900）
created=$(ffprobe -v error -show_entries format_tags=com.apple.quicktime.creationdate -of default=nw=1:nk=1 "$movie")
[ -n "$created" ] || created=$(ffprobe -v error -show_entries format_tags=creation_time -of default=nw=1:nk=1 "$movie")
movie_dur=$(ffprobe -v error -show_entries format=duration -of default=nw=1:nk=1 "$movie")
# 指の検出の動画を、iPhone の動画の始まりに合わせるためのずれ（秒）。+ なら指の検出の方が後から始まっている
delta=$(python3 - "$created" "$json" "$adjust" <<'EOF'
import json, sys
from datetime import datetime
created, path, adjust = sys.argv[1], sys.argv[2], float(sys.argv[3])
s = created.replace('Z', '+00:00')
if len(s) > 5 and s[-5] in '+-' and s[-3] != ':':   # +0900 → +09:00
    s = s[:-2] + ':' + s[-2:]
t_movie = datetime.fromisoformat(s).timestamp()
t_finger = json.load(open(path))['start_epoch']
print(f'{t_finger - t_movie + adjust:.3f}')
EOF
)
echo "iPhone の撮影開始: $created（${movie_dur} 秒）"
echo "指の検出の開始との差: ${delta} 秒（補正 ${adjust} 秒を含む）"

if python3 -c "import sys; sys.exit(0 if float('$delta') >= 0 else 1)"; then
    shift_filter="tpad=start_duration=${delta}:start_mode=add:color=black"   # 始まるまでは黒い画面
else
    shift_filter="trim=start=${delta#-},setpts=PTS-STARTPTS"                # 先に始まっている分を切る
fi

ffmpeg -y -v error -stats -i "$movie" -i "$finger" -filter_complex "\
[0:v]setpts=PTS-STARTPTS,fps=30,${rot}scale=-2:720,setsar=1,drawtext=fontfile='$font':text='ロボット':x=w-tw-20:y=20:fontsize=36:fontcolor=white:box=1:boxcolor=black@0.5:boxborderw=8[a];\
[1:v]${shift_filter},tpad=stop_mode=clone:stop_duration=${movie_dur},fps=30,scale=-2:720,setsar=1,drawtext=fontfile='$font':text='${LABEL:-指の認識（MediaPipe）}':x=w-tw-20:y=20:fontsize=36:fontcolor=white:box=1:boxcolor=black@0.5:boxborderw=8[b];\
[a][b]hstack=inputs=2,format=yuv420p[v]" \
    -map "[v]" -map 0:a:0? -t "$movie_dur" -c:v libx264 -crf 23 -preset medium -c:a aac -b:a 128k -movflags +faststart "$out"
echo "できました: $out"
