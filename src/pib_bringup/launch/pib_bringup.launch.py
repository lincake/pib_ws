from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("pib", package_name="pib_moveit_config").to_moveit_configs()

    pib_moveit_config_share = get_package_share_directory("pib_moveit_config")
    
    # 1. Publish pib robot topics
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    # 2. controller_manager receiving topics & 3. controller_manager controlling
    ros2_controllers_path = os.path.join(
        pib_moveit_config_share,
        "config",
        "ros2_controllers.yaml"
    )
    
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="screen",
    )

    # Spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    right_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["pib_arm_controller", "--controller-manager", "/controller_manager"],
    )

    # 4. ros2 launch my_robot_moveit_config move_group.launch.py
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pib_moveit_config_share, "launch", "move_group.launch.py")
        )
    )

    # 5. Launch RViz
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pib_moveit_config_share, "launch", "moveit_rviz.launch.py")
        )
    )

    return LaunchDescription([
        rsp_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        right_arm_controller_spawner,
        move_group_launch,
        rviz_launch,
    ])
