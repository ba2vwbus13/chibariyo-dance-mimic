# 実機（Raspberry Pi Mouse）との接続を切り替えるコマンド
# （専攻科実験 2026 の robot.bash を研究用に写したもの。使い方の表示だけ研究用に変えた）
#   robot_connect <PiのIPアドレス>  … 実機モード（rmw_zenoh を使い、PC の Zenoh ルータを Pi の Zenoh ルータにつなぐ）
#   robot_disconnect                … シミュレーションモード（元の通信方式）に戻す
#   set_domain_id <番号>            … ROS_DOMAIN_ID を設定する（研究用は 30）
# 接続先は ~/.robot_ip に保存し、新しく開いた端末にも自動で反映する。
# ROS_DOMAIN_ID は ~/ros2_ws/ros_domain_id.txt（PC 側の研究フォルダ）に保存するので、コンテナを作り直しても消えない。
#
# 仕組み：PC のノードは PC の中の Zenoh ルータ（localhost:7447）に接続し、そのルータが Pi のルータにつながる。
# PC の中だけで使うデータ（カメラの画像など）は Wi-Fi に流れず、Pi のノードとやり取りするデータだけが Wi-Fi を通る。

_robot_router_running() {
    pgrep -f "[r]mw_zenoh_cpp/rmw_zenohd" >/dev/null
}

_robot_start_router() {
    # 端末を開いた直後は ROS 2 の設定がまだ読み込まれていないので、ここで読み込んでから起動する
    ( ZENOH_CONFIG_OVERRIDE="connect/endpoints=[\"tcp/$1:7447\"]" RMW_IMPLEMENTATION=rmw_zenoh_cpp \
        nohup bash -c "source /opt/ros/jazzy/setup.bash && exec /opt/ros/jazzy/lib/rmw_zenoh_cpp/rmw_zenohd" \
        > ~/.robot_router.log 2>&1 & )
    sleep 2
}

_robot_stop_router() {
    pkill -f "[r]mw_zenoh_cpp/rmw_zenohd"
    for _ in 1 2 3 4 5 6; do _robot_router_running || return 0; sleep 0.5; done
    pkill -9 -f "[r]mw_zenoh_cpp/rmw_zenohd"
}

_robot_apply() {
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp
    unset ZENOH_CONFIG_OVERRIDE
}

_ROBOT_DOMAIN_FILE=~/ros2_ws/ros_domain_id.txt

set_domain_id() {
    if ! [[ "$1" =~ ^[0-9]+$ ]] || [ "$1" -gt 101 ]; then
        echo "使い方: set_domain_id <番号>　（例: set_domain_id 30）"
        return 1
    fi
    echo "$((10#$1))" > "$_ROBOT_DOMAIN_FILE"
    export ROS_DOMAIN_ID=$((10#$1))
    ros2 daemon stop >/dev/null 2>&1
    echo "ROS_DOMAIN_ID を $ROS_DOMAIN_ID にしました。新しく開く端末にも反映されます（コンテナを作り直しても消えません）。"
}

robot_connect() {
    if [ -z "$1" ]; then
        echo "使い方: robot_connect <PiのIPアドレス>　（例: robot_connect <ロボットのIP>　← chibamouse1）"
        return 1
    fi
    if [ -z "$ROS_DOMAIN_ID" ]; then
        echo "注意: ROS_DOMAIN_ID が設定されていません。先に set_domain_id 30 を実行してください。"
    fi
    if ! timeout 3 bash -c "</dev/tcp/$1/7447" 2>/dev/null; then
        echo "警告: $1 の Zenoh ルータ（7447番ポート）に接続できません。"
        echo "      IPアドレス、PCとPiが同じWi-Fiにつながっているか、Piの電源が入っているかを確認してください。"
    fi
    _robot_stop_router
    echo "$1" > ~/.robot_ip
    _robot_start_router "$1"
    _robot_apply
    ros2 daemon stop >/dev/null 2>&1
    echo "実機モードにしました（接続先: $1）。新しく開く端末にも反映されます。"
}

robot_disconnect() {
    rm -f ~/.robot_ip
    _robot_stop_router
    unset RMW_IMPLEMENTATION ZENOH_CONFIG_OVERRIDE
    ros2 daemon stop >/dev/null 2>&1
    echo "シミュレーションモードに戻しました。すでに開いている他の端末は、開き直してください。"
}

# 端末を開いたときに、保存してある設定を反映する（コンテナを起動し直したときは、PC のルータも起動する）
if [ -f "$_ROBOT_DOMAIN_FILE" ]; then
    _d=$(tr -dc '0-9' < "$_ROBOT_DOMAIN_FILE")   # Windows で編集して改行コードが変わっても読めるように数字だけ取り出す
    [ -n "$_d" ] && export ROS_DOMAIN_ID=$((10#$_d))
    unset _d
fi
if [ -f ~/.robot_ip ]; then
    _robot_apply
    _robot_router_running || _robot_start_router "$(cat ~/.robot_ip)"
    echo "[実機モード] 接続先: $(cat ~/.robot_ip)　ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-未設定}　（シミュレーションに戻すには robot_disconnect）"
fi
