#!/usr/bin/python3
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    print("Starting SubbyBoi Launch...")
    ld = LaunchDescription()
    ld.add_entity(generate_sensor_nodes())
    ld.add_entity(generate_motor_control_nodes())
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
        namespace = "/control",
        name = "PitchPIDController",
        remappings=[
            ('state', '/sensor/vectornav/translated/x')
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

def generate_motor_control_nodes() -> LaunchDescription:
    motors = []
    base_name = "MotorController"
    for motor_id in range(8):
        motor = Node(
            package= "cyphal_motor_py",
            executable= "motor",
            namespace= "/motor",
            name = base_name + str(motor_id+1),
            parameters=[
                {"motor_id":            120+motor_id},
                {"status_id":           1200+motor_id},
                {"rat_setpoint_id":     1100+motor_id},
                {"readiness_id":        0},
                {"deadman_timerout":    0.35}
            ],
            remappings=[
                {"setpoint": f"/output/motor{motor_id+1}"}
            ]
        )
        motors.append(motor)
    return LaunchDescription(motors)

def generate_sensor_nodes() -> LaunchDescription:

    vectornav_path = get_package_share_directory("vectornav")
    vectornav_path = os.path.join(vectornav_path, "config" ,"vectornav.yaml")
    vectornav = Node(
        package= "vectornav",
        executable= "vectornav",
        namespace="/sensor",
        name = "Vectornav",
        parameters= [vectornav_path]
    )

    dvl = Node(
        package= "waterlined_a50_py",
        executable= "dvl",
        namespace= "/sensor",
        name = "WaterlinkedDVLNode",
        parameters=[
            {"ip_address": "10.10.69.XXX"},
            {"port": 12345},
            {"rate": 10},
            {"qu_min_data": True},
            {"qu_dead_reckon": True},
            {"qu_full_dump", False}
        ]
    )

    sonar = Node(
        package="br_ping360+py",
        executable="ping360",
        namespace="/sensor",
        name="Ping360Node",
        parameters=[
            {"ip_address": "10.10.69.102"},
            {"port": 12345},
            {"range": 10}
        ]
    )

    return LaunchDescription([vectornav, dvl])