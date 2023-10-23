from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    drone_name = "mavic_mini_4_pro"

    drone = Node(
        package = "drone",
        executable = "drone",
        name = "drone"
    )
    ld.add_action(drone)

    for i in range(4):
        motor = Node(
            package = "drone",
            executable = "motor",
            name = "motor_" + str(i),
            namespace=drone_name
        )
        ld.add_action(motor)

    return ld