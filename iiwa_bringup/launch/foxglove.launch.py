import os

from launch_ros.actions import Node
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    pkg_description = FindPackageShare("iiwa_description")
    robot_model_path = PathJoinSubstitution([pkg_description, "urdf", "iiwa7.urdf.xacro"])
    foxglove_launch_path =  os.path.join(
                                get_package_share_directory("foxglove_bridge"), 
                                "launch", "foxglove_bridge_launch.xml")
    
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

    jsp_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui"
    )

    foxglove_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(foxglove_launch_path)
    )

    return LaunchDescription([
        declare_robot_model,
        rsp_node,
        jsp_node,
        foxglove_launch
    ])