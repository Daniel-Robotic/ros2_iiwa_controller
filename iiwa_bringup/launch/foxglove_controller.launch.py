import os

from launch_ros.actions import Node
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction


def generate_launch_description():
    pkg_description = FindPackageShare("iiwa_description")
    robot_model_path = PathJoinSubstitution([pkg_description, "urdf", "iiwa7.urdf.xacro"])
    foxglove_launch_path =  os.path.join(
                                get_package_share_directory("foxglove_bridge"), 
                                "launch", "foxglove_bridge_launch.xml")
    controllers_yaml_path = PathJoinSubstitution([pkg_description, "config", "iiwa_controllers.yaml"])
    
    declare_robot_model = DeclareLaunchArgument(
        "robot_model",
        default_value=robot_model_path,
        description="Robot model xacro file"
    )

    robot_model = LaunchConfiguration("robot_model")

    urdf_content = ParameterValue(
        Command(["xacro ", robot_model]),
        value_type=str
    )

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": urdf_content}]
    )

    foxglove_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(foxglove_launch_path)
    )
    
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": urdf_content}, controllers_yaml_path],
        output="screen"
    )

    load_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    load_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["iiwa_arm_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    return LaunchDescription([
        declare_robot_model,
        rsp_node,
        foxglove_launch,
        controller_manager_node,
        TimerAction(period=2.0, actions=[load_jsb]),
        TimerAction(period=3.0, actions=[load_trajectory_controller])
    ])