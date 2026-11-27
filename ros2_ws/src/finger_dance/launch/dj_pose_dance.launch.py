# ======================================================================
# DJダンシングマウス（dj_pose_dance）起動ファイル
#   ros2 launch finger_dance dj_pose_dance.launch.py
#   ros2 launch finger_dance dj_pose_dance.launch.py source:=camera
# ======================================================================

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

DEFAULT_VIDEO = '/home/ubuntu/ros2_ws/videos/pexels_10264579_woman_hiphop.mp4'


def generate_launch_description():
    turtlesim = LaunchConfiguration('turtlesim')
    return LaunchDescription([
        DeclareLaunchArgument('source', default_value='video'),
        DeclareLaunchArgument('file', default_value=DEFAULT_VIDEO),
        DeclareLaunchArgument('turtlesim', default_value='false'),
        DeclareLaunchArgument('delay', default_value='0.0'),
        DeclareLaunchArgument('view', default_value='true'),
        DeclareLaunchArgument('rosbridge_port', default_value='9091'),
        DeclareLaunchArgument('model', default_value='/opt/finger_dance/pose_landmarker_full.task'),

        # 映像の入り口：動画ファイル（video）か、ブラウザのカメラページ（camera）
        Node(package='finger_dance', executable='video_source', output='screen',
             parameters=[{'file': LaunchConfiguration('file')}],
             condition=LaunchConfigurationEquals('source', 'video')),
        Node(package='rosbridge_server', executable='rosbridge_websocket', output='screen',
             parameters=[{'port': ParameterValue(LaunchConfiguration('rosbridge_port'), value_type=int)}],
             condition=LaunchConfigurationEquals('source', 'camera')),
        Node(package='finger_dance', executable='pose_detect', output='screen',
             parameters=[{'model': LaunchConfiguration('model')}]),
        Node(package='finger_dance', executable='dj_pose_dance', output='screen',
             parameters=[{'turtlesim': ParameterValue(turtlesim, value_type=bool),
                          'delay': ParameterValue(LaunchConfiguration('delay'), value_type=float)}]),
        Node(package='turtlesim', executable='turtlesim_node', condition=IfCondition(turtlesim)),
        Node(package='rqt_image_view', executable='rqt_image_view', arguments=['/pose_image'],
             condition=IfCondition(LaunchConfiguration('view'))),
    ])
