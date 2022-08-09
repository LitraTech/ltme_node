from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():


    # baudrate_arg = DeclareLaunchArgument(
    #     "baudrate", default_value=TextSubstitution(text=default_baudrate)
    # )


    return LaunchDescription([
        Node(
            package="ltme_node",
            executable="ltme_node",
            name="ltme_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"device_model": "LTME-02Axxx"},
                {"device_address": "192.168.10.160"}
            ]
        )
    ])