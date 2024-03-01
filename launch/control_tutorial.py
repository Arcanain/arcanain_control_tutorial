from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'arcanain_control_tutorial'
    rviz_file_name = "arcanain_control_tutorial.rviz"

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(package_name), "rviz", rviz_file_name]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    single_obstacle_avoidance_car_mpc_control_node = Node(
        package=package_name,
        executable='single_obstacle_avoidance_car_mpc_control',
        output="screen",
    )

    multiple_obstacle_avoidance_car_mpc_control_node = Node(
        package=package_name,
        executable='multiple_obstacle_avoidance_car_mpc_control',
        output="screen",
    )

    single_obstacle_avoidance_mobile_robot_mpc_control_node = Node(
        package=package_name,
        executable='single_obstacle_avoidance_mobile_robot_mpc_control',
        output="screen",
    )

    multiple_obstacle_avoidance_mobile_robot_mpc_control_node = Node(
        package=package_name,
        executable='multiple_obstacle_avoidance_mobile_robot_mpc_control',
        output="screen",
    )

    single_obstacle_avoidance_mobile_robot_mpc_cbf_control_node = Node(
        package=package_name,
        executable='single_obstacle_avoidance_mobile_robot_mpc_cbf_control',
        output="screen",
    )

    multiple_obstacle_avoidance_mobile_robot_mpc_cbf_control_node = Node(
        package=package_name,
        executable='multiple_obstacle_avoidance_mobile_robot_mpc_cbf_control',
        output="screen",
    )

    nodes = [
        rviz_node,
        single_obstacle_avoidance_car_mpc_control_node,
        multiple_obstacle_avoidance_car_mpc_control_node,
        single_obstacle_avoidance_mobile_robot_mpc_control_node,
        multiple_obstacle_avoidance_mobile_robot_mpc_control_node,
        single_obstacle_avoidance_mobile_robot_mpc_cbf_control_node,
        multiple_obstacle_avoidance_mobile_robot_mpc_cbf_control_node,
    ]

    return LaunchDescription(nodes)
