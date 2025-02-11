from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os
import xacro
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_path = get_package_share_directory('robot_arm')
    urdf_xacro_file = os.path.join(package_path, 'urdf', 'robot_arm.urdf.xacro')

    # Convert Xacro to URDF
    doc = xacro.process_file(urdf_xacro_file)
    urdf_content = doc.toxml()

    urdf_output_file = os.path.join(package_path, 'urdf', 'robot_arm_parsed.urdf')

    # Save the processed URDF for debugging
    with open(urdf_output_file, 'w') as f:
        f.write(urdf_content)

    # Path to the controller YAML file
    controller_config = os.path.join(package_path, 'config', 'robot_arm_controllers.yaml')

    # Ensure controller config exists before launching
    if not os.path.exists(controller_config):
        raise FileNotFoundError(f"Controller config file not found: {controller_config}")

    return LaunchDescription([
        # Start Gazebo with correct ROS2 plugin
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # Spawn the robot using the correctly parsed URDF
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'robot_arm', '-file', urdf_output_file, '-z', '1.0'],
            output='screen'
        ),

        # Publish robot state (for visualization and control)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': urdf_content}]
        ),

        # Publish joint states
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher'
        ),

        # Load ROS2 controllers (position control)
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[controller_config],
            output="screen"
        ),

        # Spawn controllers
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen"
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
            output="screen"
        ),
    ])
