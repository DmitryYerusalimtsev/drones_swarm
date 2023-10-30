from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    drone_name = "mavic_mini_4_pro"
    motors = ""

    for i in range(4):
        name = "motor_" + str(i)
        if not motors:
            motors = name
        else:
            motors = motors + "," + name

        motor = Node(
            package = "drone",
            executable = "motor",
            name = name,
            namespace=drone_name
        )
        ld.add_action(motor)

    drone = Node(
        package = "drone",
        executable = "drone",
        name = drone_name,
        parameters = [
            { "motors": motors }
        ]
    )
    ld.add_action(drone)

    return ld