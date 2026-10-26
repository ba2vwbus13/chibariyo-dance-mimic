# 指ダンスをまとめて起動する
#   ros2 launch finger_dance finger_dance.launch.py turtlesim:=true   … turtlesim で確かめる
#   ros2 launch finger_dance finger_dance.launch.py                   … Raspberry Pi Mouse（robot_connect の後）
# 引数：turtlesim（false）、rosbridge_port（9091。camera.html の ?port= と同じにする）、
#       view（true：rqt_image_view で検出結果を表示）、mirror（false：映像が左右反転しているとき true）

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    turtlesim = LaunchConfiguration('turtlesim')
    return LaunchDescription([
        DeclareLaunchArgument('turtlesim', default_value='false'),
        DeclareLaunchArgument('rosbridge_port', default_value='9091'),
        DeclareLaunchArgument('view', default_value='true'),
        DeclareLaunchArgument('mirror', default_value='false'),

        # ブラウザ（camera.html）からカメラの映像を受け取る
        Node(package='rosbridge_server', executable='rosbridge_websocket', output='screen',
             parameters=[{'port': ParameterValue(LaunchConfiguration('rosbridge_port'), value_type=int)}]),
        Node(package='finger_dance', executable='finger_detect', output='screen',
             parameters=[{'mirror': ParameterValue(LaunchConfiguration('mirror'), value_type=bool)}]),
        Node(package='finger_dance', executable='finger_dance', output='screen',
             parameters=[{'turtlesim': ParameterValue(turtlesim, value_type=bool)}]),
        Node(package='turtlesim', executable='turtlesim_node', condition=IfCondition(turtlesim)),
        # 非圧縮の /finger_image を表示する（圧縮画像のトピック名を渡すと、起動した時点で finger_detect が
        # 見つかっていない場合に型を取り違えて何も表示されないため）
        Node(package='rqt_image_view', executable='rqt_image_view', arguments=['/finger_image'],
             condition=IfCondition(LaunchConfiguration('view'))),
    ])
