"""
Data Provider Server module
"""
import sys
import threading
import socket
from PyQt5.QtCore import QObject, pyqtSignal

class ReceiveData_PC_server(QObject):
    log_signal = pyqtSignal(str)

    """
    data provider server class
    """
    def __init__(self, host, port=5566):
        super().__init__()

        self.__socket = None
        self.__server_address = (host, port)
        self.__connection = None

    # def provide(self, car_data_queue):
    #     """
    #     provide the remote app with car data if a connection has been established
    #     """
    #     current_thread = threading.currentThread()
    #     self.__socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    #     self.__socket.bind(self.__server_address)
    #     # We want only one client
    #     self.__socket.listen(1)
    #     while getattr(current_thread, 'is_running', True):
    #         self.__connection, client_address = self.__socket.accept()
    #         try:
    #             while getattr(current_thread, 'is_connected', True):
    #                 states = car_data_queue.get(True, None)
    #                 self.__connection.send(str(len(states)).ljust(1024))
    #                 self.__connection.send(states)
    #                 car_data_queue.task_done()
    #             current_thread.is_connected = True
    #         finally:
    #             self.__connection.close()
    #     self.__socket.close()

    def update_car_state(self, car_data_queue):
        """
        update car_state received from CAR via sockets
        """
        current_thread = threading.currentThread()
        self.__socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.__socket.bind(self.__server_address)
        # We want only one client
        self.__socket.listen(1)
        self.log_signal.emit("Waiting for connection...")

        # Listen to infinite connection if Client disconnect
        while getattr(current_thread, 'is_running', True):
            self.__connection, client_address = self.__socket.accept()
            try:
                while getattr(current_thread, 'is_connected', True):
                    data = self.__connection.recv(1024) #valid
                    if data:
                        # save all user commands to a queue
                        car_data_queue.put(str(data), True, None)
                    else:
                        break
            finally:
                # Clean up the connection
                self.__connection.close()
        self.__socket.close()
