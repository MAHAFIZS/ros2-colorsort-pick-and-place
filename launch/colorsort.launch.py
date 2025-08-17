from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('colorsort_arm')
    urdf = os.path.join(pkg, 'urdf', 'planar_arm.urdf')
    with open(urdf, 'r') as f:
        robot_desc = f.read()

    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_desc}], output='screen'),
        Node(package='colorsort_arm', executable='arm_controller',  output='screen'),
        Node(package='colorsort_arm', executable='color_detector',  output='screen',
             parameters=[{'image_path': os.path.join(pkg, 'sample.jpg')}]),
    ])
