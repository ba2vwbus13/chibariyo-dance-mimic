from glob import glob

from setuptools import find_packages, setup

package_name = 'finger_dance'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ba2vwbus13',
    maintainer_email='ba2vwbus13@users.noreply.github.com',
    description='MediaPipe で検出した指の形と向きに合わせてロボットを動かす',
    license='MIT',
    entry_points={
        'console_scripts': [
            'finger_detect = finger_dance.finger_detect:main',
            'finger_dance = finger_dance.finger_dance:main',
            'video_source = finger_dance.video_source:main',
            'pose_detect = finger_dance.pose_detect:main',
            'pose_dance = finger_dance.pose_dance:main',
        ],
    },
)
