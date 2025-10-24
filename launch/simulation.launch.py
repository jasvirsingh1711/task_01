import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    pkg_name = 'rover_control'
    urdf_file_name = 'rover.urdf'
    

    # Package paths
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_share = get_package_share_directory(pkg_name)

    # Set use_sim_time parameter to true
    # This is essential for simulation
    use_sim_time = {'use_sim_time': True}

    # 1. Launch Gazebo
    world_file = os.path.join(pkg_share, 'worlds', 'depot.world')
    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    # 2. Get the URDF file
    urdf_file = os.path.join(pkg_share, 'urdf', urdf_file_name)
    with open(urdf_file, 'r') as infp:
        infp.readline() # Read and discard the first line (the <?xml ...?> tag)
        robot_description = infp.read()

    # 3. Launch Robot State Publisher (RSP)
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[use_sim_time, 
                    {'robot_description': robot_description}]
    )

    # 4. Spawn the rover in Gazebo
    start_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'my_rover',
                   '-topic', 'robot_description',
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.5'], # Spawn 0.5m above ground
        output='screen',
        parameters=[use_sim_time]
    )

    # 5. Launch Teleop
    start_teleop_twist_cmd = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e', # Opens in a new terminal
        parameters=[use_sim_time]
    )

    # Create the launch description
    ld = LaunchDescription()

    # Add all actions
    ld.add_action(start_gazebo_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_spawner_cmd)
    ld.add_action(start_teleop_twist_cmd)

    return ld