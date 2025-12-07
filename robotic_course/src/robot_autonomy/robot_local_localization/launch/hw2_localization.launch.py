from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic", default="/cmd_vel")

    prediction = Node(
        package="robot_local_localization",
        executable="prediction_node",
        name="prediction_node",
        parameters=[
            {"use_sim_time": use_sim_time, "cmd_vel_topic": cmd_vel_topic},
        ],
        output="screen",
    )

    measurement = Node(
        package="robot_local_localization",
        executable="measurement_node",
        name="measurement_node",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    ekf = Node(
        package="robot_local_localization",
        executable="ekf_node",
        name="ekf_node",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    tester = Node(
        package="robot_local_localization",
        executable="test_node",
        name="test_node",
        parameters=[{"use_sim_time": use_sim_time, "cmd_vel_topic": cmd_vel_topic}],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            prediction,
            measurement,
            ekf,
            tester,
        ]
    )


