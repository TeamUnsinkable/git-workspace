import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from rclpy.service import SrvTypeRequest, SrvTypeResponse
import numpy as np
import threading

from waterlinked_a50_py.dvl import DVL_Reader
from waterlinked_a50_msgs.msg import DVLBeam, DVLDeadReckon, DVLMin, DVLFullDump
from waterlinked_a50_msgs.srv import DeadReckon, CalibrateGyro

#TODO Add functionality to check connection health and attempt reconnection

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
            if self.qu_min_data:
                self.pub_min_data.publish( self.generate_min_dump(data) )
            if self.qu_full_dump:
                self.pub_full_dump.publish( self.generate_full_dump(data) )
        #Publish Dead Reckon data
        elif data['type'] == 'position_local':
            if self.qu_dead_reckon:
                self.pub_dead_reckon.publish( self.generate_dead_reckon(data) )
        self.lock.release()


    def _load_ros_params(self) -> None:
        self.declare_parameter('ip', "10.10.69.10")
        self.declare_parameter('port', 16171)
        self.declare_parameter('rate', 25) # Unit in frequency domain
        self.declare_parameter('qu_min_data', True)
        self.declare_parameter('qu_dead_reckon', True)
        self.declare_parameter('qu_full_dump', True)

        # Connection Parameters
        self.ip = self.get_parameter("ip").get_parameter_value().string_value
        self.port = self.get_parameter("port").get_parameter_value().integer_value
        rate = self.get_parameter("rate").get_parameter_value().integer_value
        
        # Timer declaration
        self.rate = self.create_timer(1/rate, self.timer_callback)
        
        # Publishing flags
        self.qu_min_data = self.get_parameter("qu_min_data").get_parameter_value().bool_value
        self.qu_dead_reckon = self.get_parameter("qu_dead_reckon").get_parameter_value().bool_value
        self.qu_full_dump = self.get_parameter("qu_full_dump").get_parameter_value().bool_value
        self._logger.info(f"Publishers: [ min:{self.qu_min_data}, dead_reckon:{self.qu_dead_reckon}, full_dump:{self.qu_full_dump} ]")
        # Create Publishers
        if self.qu_min_data:
            self.pub_min_data = self.create_publisher(DVLMin, "min_data", 5)
        if self.qu_dead_reckon:
            self.pub_dead_reckon = self.create_publisher(DVLDeadReckon, "dead_reckon", 5)
        if self.qu_full_dump:
            self.pub_full_dump = self.create_publisher(DVLFullDump, "full_dump", 5)
        
        # Create Services
        self.srv_calibrate_gyro = self.create_service(CalibrateGyro, "calibrate_gyro", self.cb_calibrate_gyro )
        self.srv_reset_deadreckon = self.create_service(DeadReckon, "calibrate_dead_reckon", self.cb_dead_reckon )

    ######################
    # Message Population #
    ######################
    def generate_full_dump(self, data) -> DVLFullDump:
        full_dump = DVLFullDump()
        #Insert All Data from dump
        full_dump.time_since_last = data['time']
        full_dump.vx = data['vx']
        full_dump.vy = data['vy']
        full_dump.vz = data['vy']
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
        full_dump.time_of_validity = float(data['time_of_validity'])
        full_dump.time_of_transmission = float(data['time_of_transmission'])
        return full_dump
    
    def generate_min_dump(self, data) -> DVLMin:
        min_dump = DVLMin()
        min_dump.vx = data['vx']
        min_dump.vy = data['vy']
        min_dump.vz = data['vy']
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