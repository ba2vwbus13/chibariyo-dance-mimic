"""人混み(通行人あり)での追尾・Re-ID評価デモ

  Gazebo(crowd_test.world) + 車椅子 + 本人(person) + 通行人(person2)
  + 追従ノード + 計測ノード(metrics_logger) を一括起動する。

使い方:
  ros2 launch wheelchair_gazebo crowd_demo.launch.py                  # Re-ID方式(既定)
  ros2 launch wheelchair_gazebo crowd_demo.launch.py method:=yolo     # 比較: 最近傍YOLO
  ros2 launch wheelchair_gazebo crowd_demo.launch.py method:=lidar    # 比較: LiDAR
  ros2 launch wheelchair_gazebo crowd_demo.launch.py world:=crowd_swap.world  # 服装違いの検証

  計測結果は ~/ros2_ws/metrics/ に CSV と summary_*.txt で保存される。
  60〜120秒ほど走らせて Ctrl+C で終了すると summary が確定する。

シナリオ:
  本人は部屋を周回し、通行人(赤い帯)は部屋の中央を南北に往復する。
  車椅子と本人の間を通行人が繰り返し横切るため、
  「別人を追いかけないか(ID維持率)」「見失いから何秒で復帰するか」を測れる。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

FOLLOWER_EXECUTABLES = {
    'lidar': 'follower',
    'yolo': 'yolo_follower',
    'reid': 'yolo_reid_follower',
}

SPAWN_X = -1.0
SPAWN_Y = 0.0

# 本人の周回コース(2026-08-21変更: 45度ずつ曲がる小さな八角形ループ)
# 旧コースは部屋全体を回る長距離・直角カーブで、追従側が同じコーナーで
# 毎回取り逃がし、見失い時間が計測の7割を占めてID性能の差が埋もれた。
# 「通行人横切り時のID維持」だけを測るため、追従が確実に続く穏やかな周回にする。
# 1周約15m(0.4m/sで約38秒) → 180秒で約4.7周
TARGET_WAYPOINTS = [3.5, 0.0,
                    3.0, 2.0,
                    1.5, 2.8,
                    0.0, 2.0,
                    -0.5, 0.0,
                    0.0, -2.0,
                    1.5, -2.8,
                    3.0, -2.0]

# 通行人: ループ中央(x=1.8)を南北に往復し、車椅子と本人の間を繰り返し横切る
# (6mを0.6m/s → 約10秒ごとに1回横切る。180秒で15回以上のID試練)
# 2026-08-23変更: 旧範囲(±4.5)は本人コース(|y|<=2.8)の外まで伸びており、
# 一度つられると場外まで引き出されて復帰できず、計測が座礁で埋まった。
# 横切りの頻度は落とさずに、追従側が場外へ出ない範囲へ収める。
DISTRACTOR_WAYPOINTS = [1.8, -3.0,
                        1.8, 3.0]


def launch_setup(context, *args, **kwargs):
    pkg_gazebo = get_package_share_directory('wheelchair_gazebo')
    method = LaunchConfiguration('method').perform(context)
    if method not in FOLLOWER_EXECUTABLES:
        raise RuntimeError(
            f'method は {list(FOLLOWER_EXECUTABLES)} のいずれかです: {method}')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'sim.launch.py')),
        launch_arguments={'world': LaunchConfiguration('world').perform(context),
                          'spawn_x': str(SPAWN_X),
                          'spawn_y': str(SPAWN_Y)}.items(),
    )

    # person2用の追加ブリッジ(sim.launch.pyはperson1のみブリッジするため)
    bridge2 = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='person2_bridge',
        arguments=[
            '/model/person2/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/model/person2/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            # 車椅子の真値位置(ワールド座標)。計測と人の衝突回避に使う。
            # 車輪odom(/odom)はスリップで向きがドリフトするため評価には使わない
            '/model/wheelchair/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
        ],
        remappings=[
            ('/model/person2/cmd_vel', '/person2/cmd_vel'),
            ('/model/person2/odometry', '/person2/odom'),
            ('/model/wheelchair/odometry', '/wheelchair/ground_truth'),
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    person_mover = Node(
        package='wheelchair_follower',
        executable='person_mover',
        name='person_mover',
        # /odomの代わりに車椅子の真値位置を参照(ドリフトの影響を受けない)
        remappings=[('/odom', '/wheelchair/ground_truth')],
        parameters=[{'use_sim_time': True,
                     'waypoints': TARGET_WAYPOINTS,
                     'speed': 0.4,
                     # 既定0.7mだと追従側が詰めすぎた時点で本人が立ち止まり、
                     # 追従側は後退できないため相互デッドロックになる。
                     # 本人は歩き続けさせ、膠着で数値が水増しされるのを防ぐ。
                     'robot_stop_radius': 0.3,
                     'robot_world_offset': [0.0, 0.0]}],
        output='screen',
    )

    # 通行人: トピックをperson2系に付け替えた2つ目のperson_mover
    person2_mover = Node(
        package='wheelchair_follower',
        executable='person_mover',
        name='person2_mover',
        remappings=[
            ('/person/cmd_vel', '/person2/cmd_vel'),
            ('/person/odom', '/person2/odom'),
            ('/odom', '/wheelchair/ground_truth'),
        ],
        parameters=[{'use_sim_time': True,
                     'waypoints': DISTRACTOR_WAYPOINTS,
                     'speed': 0.6,
                     # 立ち止まらず、車椅子が1m以内に入ったら避けて歩き続ける。
                     # 旧設定(0.35m以内で停止)では車椅子の角に接したまま双方が
                     # 動けなくなり、t=133s付近から計測終了まで見失いが続く回があった
                     'robot_stop_radius': 0.0,
                     'avoid_radius': 1.0,
                     'robot_world_offset': [0.0, 0.0]}],
        output='screen',
    )

    # Re-IDの受理条件はチューニング対象なので launch から振れるようにする。
    # yolo/lidar方式はこれらのパラメータを宣言していないため reid のときだけ渡す。
    follower_params = {'use_sim_time': True}
    if method == 'reid':
        follower_params['hist_min_sim'] = float(
            LaunchConfiguration('hist_min_sim').perform(context))
        follower_params['accept_threshold'] = float(
            LaunchConfiguration('accept_threshold').perform(context))
        follower_params['anchor_update_sim'] = float(
            LaunchConfiguration('anchor_update_sim').perform(context))
        follower_params['hist_blur'] = int(
            LaunchConfiguration('hist_blur').perform(context))
        for key in ('depth_sync', 'hist_relax', 'predict_search', 'torso_roi'):
            follower_params[key] = (
                LaunchConfiguration(key).perform(context).lower() == 'true')

    follower = Node(
        package='wheelchair_follower',
        executable=FOLLOWER_EXECUTABLES[method],
        parameters=[follower_params],
        output='screen',
    )

    metrics = Node(
        package='wheelchair_follower',
        executable='metrics_logger',
        # 真値トピック(/wheelchair/ground_truth)が届いていればそちらを優先し、
        # robot_world_offsetは/odomフォールバック時のみ使われる
        parameters=[{'use_sim_time': True,
                     'label': LaunchConfiguration('label').perform(context) or method,
                     'out_dir': LaunchConfiguration('out_dir').perform(context),
                     'robot_world_offset': [SPAWN_X, SPAWN_Y]}],
        output='screen',
    )

    # Gazebo起動が落ち着いてからノードを開始
    return [sim, bridge2,
            TimerAction(period=8.0,
                        actions=[person_mover, person2_mover,
                                 follower, metrics])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('method', default_value='reid',
                              description='追従方式: reid / yolo / lidar'),
        # 計測結果の出力先。Docker共有マウントが壊れて書き込めない場合は
        # out_dir:=/tmp/metrics などコンテナ内部を指定して退避できる
        DeclareLaunchArgument('world', default_value='crowd_test.world',
                              description='crowd_test.world または服装違いの crowd_<variant>.world '
                                          '(worlds/make_clothing_variants.py で生成)'),
        DeclareLaunchArgument('out_dir', default_value='~/ros2_ws/metrics',
                              description='計測結果(CSV/summary)の出力先'),
        DeclareLaunchArgument('label', default_value='',
                              description='計測ファイル名のラベル(既定: method)'),
        DeclareLaunchArgument('hist_min_sim', default_value='0.55',
                              description='reid: 色類似度の下限(未満は別人)'),
        DeclareLaunchArgument('accept_threshold', default_value='0.45',
                              description='reid: 本人と認める最小score'),
        DeclareLaunchArgument('anchor_update_sim', default_value='0.7',
                              description='reid: 記憶更新に要する原本との類似度(0で無効)'),
        DeclareLaunchArgument('depth_sync', default_value='true',
                              description='reid: RGBと同時刻の深度を使う(falseで旧挙動)'),
        DeclareLaunchArgument('hist_relax', default_value='false',
                              description='reid: 見失い中に色の下限を緩める(trueで旧挙動)'),
        DeclareLaunchArgument('predict_search', default_value='false',
                              description='reid: 予測位置へ向き直って探索(falseで一定旋回)'),
        DeclareLaunchArgument('hist_blur', default_value='2',
                              description='reid: 色ヒストグラムを色相・彩度方向にぼかす幅[ビン](0で旧挙動)'),
        DeclareLaunchArgument('torso_roi', default_value='true',
                              description='reid: 色を床からの高さの帯で取る(falseでボックス比率)'),
        OpaqueFunction(function=launch_setup),
    ])
