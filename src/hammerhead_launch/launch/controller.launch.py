#!/usr/bin/python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    print("Starting SubbyBoi Launch...")
    ld = LaunchDescription()
    ld.add_entity(generate_rotational_pid_nodes())
    return ld

def generate_rotational_pid_nodes() -> LaunchDescription:
    yaw_pid = Node(
        package = "pid_control",
        executable = "controller",
        namespace = "/controls",
        name = "YawPIDController",
        remappings=[
            ('state', '/sensors/vectornav/translated/z')
        ]
    )

    pitch_pid = Node(
        package = "pid_control",
        executable = "controller",
        namespace = "/controls",
        name = "PitchPIDController",
        remappings=[
            ('state', '/sensors/vectornav/translated/x')
        ]
    )

    roll_pid = Node(
        package = "pid_control",
        executable = "controller",
        namespace = "/controls",
        name = "RollPIDController",
        remappings=[
            ('state', '/sensors/vectornav/translated/y')
        ]
    )
    return LaunchDescription([yaw_pid, pitch_pid, roll_pid])
