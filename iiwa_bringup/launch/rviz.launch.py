from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    pkg_description = FindPackageShare("iiwa_description")
    robot_model_path = PathJoinSubstitution([pkg_description, "urdf", "iiwa7.urdf.xacro"])
    rviz_config_path = PathJoinSubstitution([pkg_description, "config", "rviz_iiwa7.rviz"])

    declare_robot_model = DeclareLaunchArgument(
        "robot_model",
        default_value=robot_model_path,
        description="Robot model xacro file"
    )

    declare_rviz_config = DeclareLaunchArgument(
        "rviz_config",
        default_value=rviz_config_path,
        description="Rviz config path"
    )

    robot_model = LaunchConfiguration("robot_model")
    rviz_config = LaunchConfiguration("rviz_config")

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

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz_gui",
        arguments=["-d", rviz_config]
    )

    return LaunchDescription([
        declare_robot_model,
        declare_rviz_config,
        rsp_node,
        jsp_node,
        rviz2_node
    ])