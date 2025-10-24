import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # path to package
    pkg_path = get_package_share_directory('rover_control')

    # path to URDF file
    urdf_file_path = os.path.join(pkg_path, 'urdf', 'rover.urdf')
    
    # path to RViz configuration file
    rviz_config_path = os.path.join(pkg_path, 'rviz', 'robot.rviz')

    return LaunchDescription([
        # 1. Start robot_state_publisher
        # Reads the URDF file and publishes the robot's structure to /robot_description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file_path).read()}]
        ),

        # 2. Start joint_state_publisher_gui
        # Provides a GUI to control the robot's joints
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),

        # 3. Start RViz
        # Loads the configuration file we created
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path] # This argument loads the config file
        ),
    ])