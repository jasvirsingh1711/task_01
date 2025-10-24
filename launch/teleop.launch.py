import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('rover_control')
    urdf_file_path = os.path.join(pkg_path, 'urdf', 'rover.urdf')

    rviz_config_path = os.path.join(pkg_path, 'rviz', 'robot.rviz')

    return LaunchDescription([
        # 1. Robot State Publisher (publishes TF tree)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file_path).read()}]
        ),
        
        # 2. Forward Kinematics Node (publishes /joint_states)
        Node(
            package='rover_control',
            executable='forward_kinematics_node',
            name='forward_kinematics_node'
        ),
        
        # 3. Keyboard Controller (publishes /cmd_vel)
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            output='screen',
            prefix='xterm -e' # Opens in a new terminal
        ),
        
        # 4. RViz (visualizes the robot)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path] # Loads saved config
        ),
    ])
