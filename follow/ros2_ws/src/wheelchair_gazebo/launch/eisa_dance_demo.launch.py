"""エイサー（ダンス物まね）Gazebo 3D車いす専用・高速シミュレーション:
  人モデルや重いカメラ/LiDARセンサ描画を省いた軽量ワールド (eisa_dance.world) に
  3D車いす (wheelchair) のみをスポーンし、軽快にエイサー動作を確認できる Launch ファイル。

使い方:
  # 1. カメラ・ダンス動画連動＋待機時エイサー自動演舞デモ（既定）
  ros2 launch wheelchair_gazebo eisa_dance_demo.launch.py

  # 2. カメラ入力のみに反応させる場合 (auto_demo:=false)
  ros2 launch wheelchair_gazebo eisa_dance_demo.launch.py auto_demo:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('wheelchair_gazebo')
    pkg_desc = get_package_share_directory('wheelchair_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world = os.path.join(pkg_gazebo, 'worlds', 'eisa_dance.world')
    xacro_file = os.path.join(pkg_desc, 'urdf', 'wheelchair.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    delay_arg = DeclareLaunchArgument(
        'delay', default_value='0.0',
        description='カメラの動きから車いすが反応するまでの遅れ秒数 (既定: 0.0秒＝リアルタイム)')
    dist_scale_arg = DeclareLaunchArgument(
        'dist_scale', default_value='2.25',
        description='Gazebo 3D空間で見やすくするための前後移動倍率')
    angle_scale_arg = DeclareLaunchArgument(
        'angle_scale', default_value='1.0',
        description='Gazebo 3D空間で見やすくするための旋回角度倍率')
    auto_demo_arg = DeclareLaunchArgument(
        'auto_demo', default_value='true',
        description='カメラ未接続時にエイサー4動作デモを自動再生するか')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world}'}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen',
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'wheelchair',
                   '-x', '0.0', '-y', '0.0', '-z', '0.1'],
        output='screen',
    )

    # 車いすの移動・真値オドメトリに必要なトピックをブリッジ（タイヤ滑り誤差のない真値位置を使用）
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/model/wheelchair/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
        ],
        remappings=[
            ('/model/wheelchair/odometry', '/wheelchair/ground_truth'),
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    eisa_dance = Node(
        package='wheelchair_follower',
        executable='eisa_dance',
        parameters=[{
            'use_sim_time': False,
            'delay': LaunchConfiguration('delay'),
            'dist_scale': LaunchConfiguration('dist_scale'),
            'angle_scale': LaunchConfiguration('angle_scale'),
            'auto_demo': LaunchConfiguration('auto_demo'),
            'move_person': False,
        }],
        output='screen',
    )

    delayed = TimerAction(period=3.5, actions=[eisa_dance])

    return LaunchDescription([
        delay_arg,
        dist_scale_arg,
        angle_scale_arg,
        auto_demo_arg,
        gz_sim,
        robot_state_publisher,
        spawn,
        bridge,
        delayed,
    ])
