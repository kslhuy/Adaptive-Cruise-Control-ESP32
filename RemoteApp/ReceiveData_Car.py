"""
Data Provider Server module
"""
import sys
import threading
import socket
import RemoteMain as GLOBAL

from PyQt5.QtCore import QObject, pyqtSignal

class ReceiveData_Car(QObject):
    log_signal = pyqtSignal(str)

    """
    data provider server class
    """
    def __init__(self, host, port=4444):
        super().__init__()

        self.__socket = None
        self.__server_address = (host, port)
        self.__connection = None
        self.list_vel = []

    # def provide(self, car_data_queue):


    def update_car_distance_vel(self, distance_vel ,distance_vel_pc ):
        """
        update distance and velocity received from CAR via sockets
        """
        current_thread = threading.currentThread()
        self.__socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.__socket.bind(self.__server_address)
        # We want only one client
        self.__socket.listen(1)
        # self.log_signal.emit("Waiting for connection...")

        # Listen to infinite connection if Client disconnect
        while getattr(current_thread, 'is_running', True):
            self.__connection, client_address = self.__socket.accept()
            self.log_signal.emit("Etabli recve to: " + str(client_address))

            try:
                while getattr(current_thread, 'is_connected', True):
                    data = self.__connection.recv(500) #valid
                    if data:
                        # self.log_signal.emit("Recvied: " + str(data))
                        # data = data.decode('utf-8')
                        # self.parse_data(data.decode('utf-8'))

                        data = data.decode('utf-8').split('\n')

                        if len(data) > 1 and len(data[1]) > 15:
                            data_new = data[1]
                        else:
                            data_new = data[0]
                        print(data_new)

                        # save all user commands to a queue
                        distance_vel.put(data_new, True, None)
                        distance_vel_pc.put(data_new, True, None)
                    else:
                        break
            finally:
                # Clean up the connection
                self.__connection.close()
        self.__socket.close()


    def update_car_distance_vel_follow(self, distance_vel  ):
        """
        update distance and velocity received from CAR via sockets ()
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
            self.log_signal.emit("Etabli recve to: " + str(client_address))

            try:
                while getattr(current_thread, 'is_connected', True):
                    data = self.__connection.recv(500) #valid
                    if data:
                        # print((data.decode('utf-8')))
                        # self.log_signal.emit("Recvied: " + str(data))

                        # save all user commands to a queue
                        distance_vel.put(data.decode('utf-8'), True, None)
                        # self.parse_data(data.decode('utf-8'))
                    else:
                        break
            finally:
                # Clean up the connection
                self.__connection.close()
        self.__socket.close()


    def parse_data(self,distance_velo_car_ego):
        if distance_velo_car_ego:
            distance_velo_car_ego = distance_velo_car_ego.split(';')
            # print(distance_velo_car_ego)
            for elem in distance_velo_car_ego:
                current_distance_velo = elem.split(':')
                if len(current_distance_velo) > 1:
                    if GLOBAL.CSQ_DISTANCE in current_distance_velo[0]:
                        print("distance" + current_distance_velo[1])
                        # self.__distance_to_car = float(current_distance_velo[1])
                    elif GLOBAL.CSQ_SPEED in current_distance_velo[0]:
                        print("speed" + (current_distance_velo[1]))
                        # self.list_vel.append(str(current_distance_velo[1]))
                        # self._current_speed_ego = float(current_distance_velo[1])


    # def save_list_to_file(self):
    #     with open('velocity_data.txt', 'a') as file:
    #         while True:
    #             velocity = self.queue.get()  # Get the value from the Queue
    #             if velocity is None:  # Assume None indicates the end of data
    #                 break
    #             file.write(velocity + '\n')
                
