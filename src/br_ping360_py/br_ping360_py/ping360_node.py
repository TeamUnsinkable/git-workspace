import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from br_ping360_py.br_ping360_py.ping360_sonar import SonarReader
from rclpy.service import SrvTypeRequest, SrvTypeResponse
import numpy as np
import threading

class SonarNode(Node):
    def __init__(self):
        super().__init__("Ping360Node")
        self.sonar = SonarReader(self.ip_address, self.port)
        
        # Load driver parameters
        self._load_params()

        # Configure sonar driver
        self.sonar.configure_params()


    # Action Server
    

    def _load_params(self):
        # Connection Parameters
        self.declare_parameter("ip_address", "10.10.69.101")
        self.declare_parameter("port", 12345)

        # Declare Parameters
        self.declare_parameter("gain_setting", 0)
        self.declare_parameter('angle_start', 0)
        self.declare_parameter('angle_stop', 400)
        self.declare_parameter('transmit_frequency', 750)
        self.declare_parameter('number_of_samples', 1200)
        self.declare_parameter('speed_of_sound', 1500)
        self.declare_parameter('range', 10)
        self.declare_parameter('delay', 4)

        # Load Parameters
        self.ip_address     = self.get_parameter("ip_address")
        self.port           = self.get_parameter("port")

        self.gain_setting       = self.get_parameter("gain_setting").get_parameter_value().integer_value
        self.angle_start        = self.get_parameter("angle_start").get_parameter_value().integer_value
        self.angle_stop         = self.get_parameter("angle_stop").get_parameter_value().integer_value
        self.transmit_frequency = self.get_parameter('transmit_frequency').get_parameter_value().integer_value
        self.number_of_samples  = self.get_parameter('number_of_samples').get_parameter_value().integer_value
        self.speed_of_sound     = self.get_parameter('speed_of_sound').get_parameter_value().integer_value
        self.range              = self.get_parameter('range').get_parameter_value().integer_value
        self.delay              = self.get_parameter('delay').get_parameter_value().integer_value
        
    
def main():
    rclpy.init()
    node = SonarNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
