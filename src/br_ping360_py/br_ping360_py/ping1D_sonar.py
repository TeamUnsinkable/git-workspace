
from brping import PingDevice, Ping1D, PingMessage
import numpy as np

def SonarReader():
    def __init__(self, port="/dev/ttyUSB0", baud=115200) -> bool:
        self.sonar = Ping1D()
        self.sonar.connect_serial(port, baud)
        return self.sonar.initialize()
    
    def configure_params(self):
        pass

    def scan(self) -> tuple:
        return self.sonar.get_distance_simple()
    

if __name__ == "__main__":
    sonar = SonarReader()
    