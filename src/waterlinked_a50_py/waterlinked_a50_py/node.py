import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from rclpy.service import SrvTypeRequest, SrvTypeResponse
import numpy as np
import threading

from waterlinked_a50_py.dvl import DVL_Reader
from waterlinked_a50_msgs.msg import DVLBeam, DVLDeadReckon, DVLMin, DVLFullDump
from waterlinked_a50_msgs.srv import DeadReckon, GyroCalibration, GetConfig

class DVLPublisher(Node):
    def __init__(self) -> None:
        super().__init__("WaterlinkedA50")
        self._load_ros_params()
        self._logger.info("Connecting to DVL at {}:{}...".format(self.ip, self.port))
        try:
            self.dvl = DVL_Reader(self.ip, self.port)
        except:
            self._logger.fatal("Failed to connect to DVL")
            rclpy.shutdown()
        else:
            self._logger.info("Connected successfully!")
        self.lock = threading.Lock()


    def timer_callback(self) -> None:
        self.lock.acquire(True)
        data = self.dvl.get_data()
        if data['type'] == 'velocity':
            if self.pub_min:
                self.pub_min_data( self.generate_min_dump(data) )
            if self.pub_full_dump:
                self.pub_full_dump( self.generate_full_dump(data) )
        #Publish Dead Reckon data
        elif data['type'] == 'position_local':
            if self.pub_dead_reckon:
                self.pub_dead_reckon( self.generate_dead_reckon(data) )



    def _load_ros_params(self) -> None:
        self.ip = self.get_parameter("ip", "10.10.69.21")
        self.port = self.get_parameter("port", 16171)
        rate = self.get_parameter("rate", 10) # Unit in frequency
        self.rate = Timer(self.timer_callback, self.default_callback_group, 1/rate * 10e9)

        # Publishing flags
        self.qu_min_data = self.get_parameter("qu_min_data", True)
        self.qu_dead_reckon = self.get_parameter("qu_dead_reckon", True)
        self.qu_full_dump = self.get_parameter("qu_full_dump", True)
        
        # Create Publishers
        if self.qu_min_data:
            self.pub_min_data = self.create_publisher(DVLMin, "min_data", 5)
        if self.qu_dead_reckon:
            self.pub_dead_reckon = self.create_publisher(DVLDeadReckon, "dead_reckon", 5)
        if self.qu_full_dump:
            self.pub_full_dump = self.create_publisher(DVLFullDump, "full_dump", 5)
        
        # Create Services
        self.srv_calibrate_gyro = self.create_service(GyroCalibration, "calibrate_gyro", self.cb_calibrate_gyro )
        self.srv_reset_deadreckon = self.create_service(DeadReckon, "calibrate_dead_reckon", self.cb_dead_reckon )

    ######################
    # Message Population #
    ######################
    def generate_full_dump(self, data) -> DVLFullDump:
        full_dump = DVLFullDump()
        #Insert All Data from dump
        full_dump.time_since_last = data['time']
        full_dump.Vx = data['vx']
        full_dump.Vy = data['vy']
        full_dump.Vz = data['vy']
        full_dump.fom = data['fom']    

        full_dump.altitude = data['altitude']
        #Transducer Loop
        full_dump.transducers = []
        for transducer in data['transducers']:
            beam = DVLBeam()
            beam.id = transducer['id']
            beam.velocity = transducer['velocity']
            beam.distance = transducer['distance']
            beam.rssi = transducer['rssi']
            beam.nsd = transducer['nsd']
            beam.beam_valid = transducer['beam_valid']
            full_dump.transducers.append(beam)
        
        full_dump.velocity_valid = data['velocity_valid']
        full_dump.status = data['status']
        full_dump.time_of_validity = data['time_of_validity']
        full_dump.time_of_transmission = data['time_of_transmission']
        return full_dump
    
    def generate_min_dump(self, data) -> DVLMin:
        min_dump = DVLMin()
        min_dump.Vx = data['vx']
        min_dump.Vy = data['vy']
        min_dump.Vz = data['vy']
        min_dump.fom = data['fom']    
        min_dump.altitude = data['altitude']
        min_dump.velocity_valid = data['velocity_valid']
        min_dump.status = data['status']
        return min_dump
    
    def generate_dead_reckon(self, data) -> DVLDeadReckon:
        dead_reckon = DVLDeadReckon()
        #Publish DeadReckon Report
        dead_reckon.ts = data['ts']
        dead_reckon.x = data['x']
        dead_reckon.y = data['y']
        dead_reckon.z = data['z']
        dead_reckon.std = data['std']
        dead_reckon.roll = data['roll']
        dead_reckon.pitch = data['pitch']
        dead_reckon.yaw = data['yaw']
        dead_reckon.status = data['status']
        return dead_reckon
    
    #####################
    # Service Callbacks #
    #####################
        
    def cb_calibrate_gyro(self, request: SrvTypeRequest, response: SrvTypeResponse):
        self._logger.info("Recieved gyro recalibration request. Locking thread...")
        self.lock.acquire(True)
        self._logger.info("Locked!")
        resp = self.dvl.calibrate_gyro()
        self.lock.release()
        response.success = resp['success']
        response.error_message = resp['error_message']
        
        return response

    def cb_dead_reckon(self, request: SrvTypeRequest, response: SrvTypeResponse):
        self._logger.info("Recieved reset deadreckon request. Locking thread...")
        self.lock.acquire(True)
        self._logger.info("Locked!")
        resp = self.dvl.reset_deadreckon()
        self.lock.release()
        response.status = resp['success']
        response.error_message = resp['error_message']
        return response


def main():
    rclpy.init()
    node = DVLPublisher()
    rclpy.spin(node)
    node.dvl.terminate()
    
if __name__ == "__main__":
    main()