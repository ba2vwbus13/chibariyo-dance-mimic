# 人のダンスに合わせて踊る（体の検出 → 踊りの指令）をまとめて起動する
#   ros2 launch finger_dance pose_dance.launch.py turtlesim:=true                 … ダンスの動画で、turtlesim の亀を踊らせる
#   ros2 launch finger_dance pose_dance.launch.py                                 … ダンスの動画で、ロボットを踊らせる（robot_connect の後）
#   ros2 launch finger_dance pose_dance.launch.py source:=camera                  … カメラページの映像（目の前の人）で踊らせる
# 引数：source（video または camera）、file（動画ファイル）、turtlesim（false）、delay（0：同時。1.0 なら 1 秒遅れて真似る）、
#       view（true：rqt_image_view で体の検出の画面を表示）、rosbridge_port（9091）

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

DEFAULT_VIDEO = '/home/ubuntu/ros2_ws/videos/pexels_7571381_man_studio.mp4'


def generate_launch_description():
    turtlesim = LaunchConfiguration('turtlesim')
    return LaunchDescription([
        DeclareLaunchArgument('source', default_value='video'),
        DeclareLaunchArgument('file', default_value=DEFAULT_VIDEO),
        DeclareLaunchArgument('turtlesim', default_value='false'),
        DeclareLaunchArgument('delay', default_value='0.0'),
        DeclareLaunchArgument('view', default_value='true'),
        DeclareLaunchArgument('rosbridge_port', default_value='9091'),

        # 映像の入り口：動画ファイル（video）か、ブラウザのカメラページ（camera）
        Node(package='finger_dance', executable='video_source', output='screen',
             parameters=[{'file': LaunchConfiguration('file')}],
             condition=LaunchConfigurationEquals('source', 'video')),
        Node(package='rosbridge_server', executable='rosbridge_websocket', output='screen',
             parameters=[{'port': ParameterValue(LaunchConfiguration('rosbridge_port'), value_type=int)}],
             condition=LaunchConfigurationEquals('source', 'camera')),
        Node(package='finger_dance', executable='pose_detect', output='screen'),
        Node(package='finger_dance', executable='pose_dance', output='screen',
             parameters=[{'turtlesim': ParameterValue(turtlesim, value_type=bool),
                          'delay': ParameterValue(LaunchConfiguration('delay'), value_type=float)}]),
        Node(package='turtlesim', executable='turtlesim_node', condition=IfCondition(turtlesim)),
        Node(package='rqt_image_view', executable='rqt_image_view', arguments=['/pose_image'],
             condition=IfCondition(LaunchConfiguration('view'))),
    ])
