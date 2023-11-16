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

class SendData_Car(QObject):
    log_signal = pyqtSignal(str)

    """
    Comunicator Server class
    receive the commands sent by user
    store them in a Queue to be processed by CarManager
    """
    def __init__(self, host, port=3333):
        super().__init__()

        self.__socket = None
        self.__server_address = (host, port)
        self.__connection = None


    def send_commands(self , user_commands_queue):
        """
        provide the car with data (speed , steer) 
        """
        current_thread = threading.currentThread()
        self.__socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.__socket.bind(self.__server_address)
        # We want only one client
        self.__socket.listen(1)
        # self.log_signal.emit("Waiting for connection...")

        while getattr(current_thread, 'is_running', True):       
            self.__connection, client_address = self.__socket.accept()
            self.log_signal.emit("Etabli send to: " + str(client_address))
            try:
                while getattr(current_thread, 'is_connected', True):
                    states = user_commands_queue.get(True, None)
                    print(states)
                    # self.__connection.send(str(len(states)).ljust(1024))
                    self.__connection.send(states.encode('utf-8'))
                    user_commands_queue.task_done()
                current_thread.is_connected = True
            finally:
                self.__connection.close()
        self.__socket.close()

    def send_direct_commands(self, data):
        if (self.__connection == None) : 
            # print("Not connected , Can not send")
            self.log_signal.emit("Not connected , Can not send ")
            return
        self.__connection.send(data.encode())



