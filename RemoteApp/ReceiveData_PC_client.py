"""
Data Provider Server module
"""
import sys
import threading
import socket
import time
import RemoteMain as GLOBAL

from PyQt5.QtCore import QObject, pyqtSignal

class ReceiveData_PC_client(QObject):
    log_signal = pyqtSignal(str)

    """
    ---------- Receive Acceleration , velocity from car preceding --------
    """
    def __init__(self, host, port=6666):
        super().__init__()

        self.__socket = None
        self.__server_address = (host, port)
        self.__connection = None
        self.max_retries = 10
        self.retry_interval = 3



    def update_car_pre_accel_vel_distance(self,distance_velo_accel_car_pre_queue):
        """
        update car_state received from CAR via sockets
        """
        current_thread = threading.currentThread()

        retries = 0
        while retries < self.max_retries:
            try:
                self.__socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.__socket.connect(self.__server_address)
                self.log_signal.emit("Receive Data PC established !!!!")
                break  # Break out of the loop if connection is successful
            except ConnectionRefusedError:
                self.log_signal.emit(f"Connection attempt {retries + 1}/{self.max_retries} failed. Retrying in {self.retry_interval} seconds...")
                time.sleep(self.retry_interval)
                retries += 1
        else:
            print("Max retries reached. Unable to connect to the server.")

        # Listen to infinite connection if Client disconnect
        while getattr(current_thread, 'is_running', True):
            # self.__connection, client_address = self.__socket.accept()
            try:
                while getattr(current_thread, 'is_connected', True):
                    data = self.__socket.recv(100) #valid
                    if data:
                        # parse_data(data.decode())
                        # save all user commands to a queue
                        data = data.decode().split('\n')
                        if len(data) > 1 and len(data[1]) > 15:
                            data_new = data[1]
                        else:
                            data_new = data[0]
                        distance_velo_accel_car_pre_queue.put(data_new, True, None)
                    else:
                        break
            finally:
                # Clean up the connection
                self.__socket.close()
        # self.__socket.close()

def parse_data(data):
    if data:
        data = data.split('\n')
        for ele1 in data:
            if (len(ele1)>15):
                data = ele1.split(';')
                # print(distance_velo_car_ego)
                for elem in data:
                    current_distance_velo = elem.split(':')
                    if len(current_distance_velo) > 1:
                        if GLOBAL.CSQ_DISTANCE in current_distance_velo[0]:
                            print("distance" + str(float(current_distance_velo[1])))
                            # self.__distance_to_car = float(current_distance_velo[1])
                        elif GLOBAL.CSQ_SPEED in current_distance_velo[0]:
                            print("speed" + str(float(current_distance_velo[1])))
                            # self._current_speed_ego = float(current_distance_velo[1])
                        elif GLOBAL.CSQ_ACCEL in current_distance_velo[0]:
                            print("accel" + str(float(current_distance_velo[1])))
                            # self._current_speed_ego = float(current_distance_velo[1])
                    