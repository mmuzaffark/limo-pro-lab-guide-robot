# limo_start_guide.launch.py (EKF version): wheel launch with pub_odom_tf false + robot_localization EKF.
# The EKF publishes odom->base_link from wheel speed (vx, vy) + IMU yaw rate. Wheel-only original: limo_start_wheel.launch.py
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

H = os.path.expanduser('~/guide_ws')


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(H + '/launch/limo_start_wheel.launch.py'),
            launch_arguments={'pub_odom_tf': 'false'}.items()),
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
             output='screen', parameters=[H + '/params/ekf_lab.yaml']),
    ])
