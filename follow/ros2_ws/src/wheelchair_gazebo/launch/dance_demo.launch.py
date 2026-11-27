"""踊り物真似デモ (エイサー3D車いす単独・軽量版):
  eisa_dance_demo.launch.py と同じく、人モデルを描画せず3D車いすのみで軽快にエイサー動作を再現します。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('wheelchair_gazebo')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo, 'launch', 'eisa_dance_demo.launch.py')))
    ])
