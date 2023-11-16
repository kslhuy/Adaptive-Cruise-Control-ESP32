"""
This module will implement in The car file arduino
"""

"""
Comunicator Server module
"""
import threading
import socket
import sys
import time
from PyQt5.QtCore import QObject, pyqtSignal

class SendData_PC_client(QObject):
    log_signal = pyqtSignal(str)

    """
    Comunicator Server class
    receive the commands sent by user
    store them in a Queue to be processed by CarManager
    """
    def __init__(self, host, port=5566):
        super().__init__()

        self.__socket = None
        self.__server_address = (host, port)
        self.__connection = None



    def send_commands(self, car_data_following,user_commands_queue):
        """
        provide the PC (car following) with data ( acceleration ,speed , steer) 
        """
        current_thread = threading.currentThread()
        self.__socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.__socket.connect(self.__server_address)

        self.log_signal.emit("SendData_PC_client : Connect to server")

        while getattr(current_thread, 'is_running', True):
            command = car_data_following.get(True, None)
            self.__socket.sendall(str(command))
            time.sleep(100.0 / 1000.0)
        self.__socket.close()
         
