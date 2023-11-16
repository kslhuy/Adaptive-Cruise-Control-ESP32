"""
This module will implement in The car file arduino
"""

"""
Comunicator Server module
"""
import threading
import socket
import sys
from PyQt5.QtCore import QObject, pyqtSignal

class SendData_PC_server(QObject):
    log_signal = pyqtSignal(str)

    """
    Comunicator Server class
    receive the commands sent by user
    store them in a Queue to be processed by CarManager
    """
    def __init__(self, host, port=6666):
        super().__init__()
        self.__socket = None
        self.__server_address = (host, port)
        self.__connection = None


    def send_data_2_following(self, acceleration_CAR_QUEUE_PC,distance_Velocity_CAR_QUEUE_PC):
        """
        provide the PC (car following) with data ( acceleration ,speed , steer) 
        """
        current_thread = threading.currentThread()
        self.__socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.__socket.bind(self.__server_address)
        # We want only one client
        self.__socket.listen(1)
        self.log_signal.emit("Send Data PC SERVER connecting...")

        while getattr(current_thread, 'is_running', True):
            self.__connection, client_address = self.__socket.accept()
            try:
                while getattr(current_thread, 'is_connected', True):
                    acc = acceleration_CAR_QUEUE_PC.get(True, None)
                    dis_vel =  distance_Velocity_CAR_QUEUE_PC.get(True, None)

                    data = "accel:" + acc + ";" + dis_vel + "\n"
                    # data += distance_Velocity_CAR_QUEUE_PC.get(True, None)
                    
                    # self.__connection.send(str(len(states)).ljust(1024))
                    self.__connection.send(data.encode('utf-8'))
                    # print(data) 


                    acceleration_CAR_QUEUE_PC.task_done()
                    distance_Velocity_CAR_QUEUE_PC.task_done()
                current_thread.is_connected = True
            finally:
                self.__connection.close()
        self.__socket.close()
