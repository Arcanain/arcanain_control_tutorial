from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'runge_kutta'
    rviz_file_name = "runge_kutta.rviz"

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

    minimal_publisher_node = Node(
        package=package_name,
        executable='talker',
        output="screen",
    )
    
    runge_kutta_node = Node(
        package=package_name,
        executable='RungeKutta',
        output="screen",
    )

    nodes = [
        runge_kutta_node,
    ]

    return LaunchDescription(nodes)
