#!/bin/bash
# ダンスの物真似で使うダンス動画（Pexels、Pexels License）を ros2_ws/videos/ に取る（SD の大きさ、1 本 1〜5 MB）
#   tools/download_videos.sh
set -e
cd "$(dirname "$0")/../ros2_ws" && mkdir -p videos && cd videos
get() { [ -f "$1" ] && { echo "あり: $1"; return; }; echo "取得: $1"; curl -sSfL -o "$1" "$2"; }
get pexels_7571381_man_studio.mp4          https://videos.pexels.com/video-files/7571381/7571381-sd_960_506_25fps.mp4    # Antoni Shkraba「Video of a Man Dancing」
get pexels_7570258_woman_studio.mp4        https://videos.pexels.com/video-files/7570258/7570258-sd_506_960_25fps.mp4    # Antoni Shkraba「Woman Dancing」
get pexels_6939968_woman_contemporary.mp4  https://videos.pexels.com/video-files/6939968/6939968-sd_540_960_30fps.mp4    # Pavel Danilyuk「A Woman Dancing Gracefully」
get pexels_10264579_woman_hiphop.mp4       https://videos.pexels.com/video-files/10264579/10264579-sd_960_506_25fps.mp4  # cottonbro studio「Woman Dancing in Hip Hop Style」
get pexels_6322570_man_hiphop.mp4          https://videos.pexels.com/video-files/6322570/6322570-sd_960_506_25fps.mp4    # cottonbro studio「Man Dancing Hip Hop」
get pexels_8027276_woman_freestyle.mp4     https://videos.pexels.com/video-files/8027276/8027276-sd_960_540_25fps.mp4    # Yaroslav Shuraev「A Woman Freestyle Dancing」
