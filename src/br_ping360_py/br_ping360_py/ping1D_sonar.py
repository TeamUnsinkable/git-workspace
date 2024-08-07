
from brping import PingDevice, Ping360, PingMessage
import numpy as np

def SonarReader():
    def __init__(self, ip="10.10.69.100", port=12345) -> bool:
        self.sonar = Ping360()
        self.sonar.connect_udp(ip, port)
        return self.sonar.initialize()
    
    def configure_params(self):
        pass

    def scan(self, angle) -> list:
        return self.sonar.transmitAngle(angle)

    ############################
    ## Parameter Calculations ##
    ############################

    def _calculate_sample_period(self):
        return  int((2 * self.range) / (self.number_of_samples * self.speed_of_sound * 25*(10**-9)))
    
    def _calculate_transmit_duration(self):
        """
        Per firmware engineer:
            1. Starting point is TxPulse in usec = ((one-way range in metres) * 8000) / (Velocity of sound in metres per second)
            2. Then check that TxPulse is wide enough for currently selected sample interval in usec, i.e.,
                if TxPulse < (2.5 * sample interval) then TxPulse = (2.5 * sample interval)
            3. Perform limit checking
        """
        inital_transmit = round((self.range * 8000) / self.speed_of_sound)
        duration = max((2.5 * self.sample_period) / 1000, inital_transmit)
        sonar_min_duration = 5
        transmit_max = min(self.sample_period*(64*10**6), 500)
        transmit_duration = max(sonar_min_duration, min(duration, transmit_max))
        return transmit_duration

 
    def _validate_parameters(self):
        #Gain Values
        if self.gain_setting not in [0,1,2]:
            raise ValueError("Gain Values not valid")
        #Angle scan values
        if self.angle_start > self.angle_end:
            raise ValueError("Scan angle start can not be greater than scan angle end")
        elif self.angle_start > 400 or self.angle_start < 0:
            raise ValueError("Scan angle start must be between 0 and 400 inclusive")
        elif self.angle_end > 400 or self.angle_end < 0:
            raise ValueError("Scan andle end can must be between 400 and 0 inclusive")
        #Transmit Frequency
        if self.transmit_frequency < 650 or self.transmit_frequency > 850:
            raise ValueError("Transmit frequency must be between 650 and 850")
        if self.number_of_samples < 200 or self.number_of_samples > 1200:
            raise ValueError("Number of samples should be between 200 and 1200 inclusive")
        