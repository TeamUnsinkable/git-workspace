import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from std_msgs.msg import Float64
from br_ping360_py.ping1D_sonar import SonarReader
from br_ping360_msgs.msg import Ping1D
from rclpy.service import SrvTypeRequest, SrvTypeResponse
import numpy as np
import threading


class SonarNode(Node):
    def __init__(self):
        super().__init__("Ping1DNode")
        self.sonar = SonarReader(self.port, self.baud)
        
        # Load driver parameters
        self._load_params()

        # Configure sonar driver
        self.sonar.configure_params()    
        self.lock = threading.Lock()

    def _load_params(self):
        # Connection Parameters
        self.declare_parameter("port", "/dev/ttyUSB1")
        self.declare_parameter("baud", 115200)

        # Operational Parameters
        self.declare_parameter("gain_setting", 0)
        self.declare_parameter('transmit_frequency', 750)
        self.declare_parameter('speed_of_sound', 1500)
        self.declare_parameter('range', 10)
        self.declare_parameter('rate', 10)
        
        # Load Parameters
        self.port     = self.get_parameter("port")
        self.baud      = self.get_parameter("baud")

        self.gain_setting       = self.get_parameter("gain_setting").get_parameter_value().integer_value
        self.transmit_frequency = self.get_parameter('transmit_frequency').get_parameter_value().integer_value
        self.speed_of_sound     = self.get_parameter('speed_of_sound').get_parameter_value().integer_value
        self.range              = self.get_parameter('range').get_parameter_value().integer_value
        self.rate               = self.get_parameter('rate').get_parameter_value().integer_value
        # Create publishers
        # TODO: Fix Publisher initialization
        self.pub_distance = self.create_publisher(Ping1D, "sonar/one_dim", 5)
        self.timer = self.create_timer(1/rate, self.timer_callback)
        

    def timer_callback(self) -> None:
        value = Ping1D()
        self.lock.acquire(True)
        value.distance, value.confidence = self.sonar.scan()
        self.pub_distance.publish(value)
        self.lock.release()

    
def main():
    rclpy.init()
    node = SonarNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
