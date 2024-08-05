#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from rclpy.service import Service

class ServiceTest(Node):
    def __init__(self):
        super().__init__("ServiceTester")
        self.create_subscription()
        
        self.guard