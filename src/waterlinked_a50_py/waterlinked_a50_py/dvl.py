#!/usr/bin/python3
import socket
import json
import logging


# Based on Waterlinked A50 Documentation as of 04/06/2023
# https://waterlinked.github.io/dvl/dvl-protocol/
# By Sirio Jansen-Sanchez

class DVL_Reader():
    def __init__(self, ip, port) -> None:
        self.ip = ip
        self.port = port

    def init_stream(self):
        # Define as a TCP, INET, connection of type STREAM for continous flow of data 
        try:
            #dvl_com = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            dvl_com = socket.create_connection((self.ip, self.port), timeout=5)
            #print("Connected")
        except socket.error as e:
            return e
        
        self.sock = dvl_com        
        try:
            self.get_data()
        except AttributeError as e:
            return("Failed to connect to DVL")

        return True

    def calibrate_gyro(self) -> dict:
        command = {'command': 'calibrate_gyro'}
        command = json.dumps(command)
        self.sock.sendall(bytes(command, encoding="utf-8"))
        return self.get_data_srv(command['command'])
    
    def reset_deadreckon(self) -> dict:
        command = {'command': 'reset_dead_reckoning'}
        command = json.dumps(command)
        self.sock.sendall(bytes(command, encoding="utf-8"))
        return self.get_data_srv(command['command'])

    def trigger_ping(self) -> dict:
        command = {'command': 'trigger_ping'}
        command = json.dumps(command)
        self.sock.sendall(bytes(command, encoding="utf-8"))
        return self.get_data_srv(command['command'])

    def get_data_srv(self, service):
        recv = False
        while not recv:
            raw_data = ''
            #Collect a single report, ended with a \r\n
            while '\r' not in raw_data:
                new =self.sock.recv(1).decode()
                raw_data += new
            parsed = json.loads(raw_data)
            try:
                recv = parsed['response_to'] == service
            except:
                # Wrong message recieved, listen for the next ones
                pass
            return parsed
    
    def get_data(self):
        raw_data = ''
        #Collect a single report, ended with a \r\n
        while '\r' not in raw_data:
            new =self.sock.recv(1).decode()
            raw_data += new
        parsed = json.loads(raw_data)
        return parsed

    def terminate(self):
        self.sock.close()

if __name__ == "__main__":
    DVL = DVL_Reader("10.10.69.20", 16171)
    dvl_status = DVL.init_stream() 

    if dvl_status == True:
        #Parse JSON
        print(DVL.get_data())
        print(DVL.calibrate_gyro())
    else:
        print(dvl_status) 
    