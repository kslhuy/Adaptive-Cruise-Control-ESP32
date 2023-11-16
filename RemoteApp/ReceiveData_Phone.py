"""
Receive acceleration from the phone  
"""
import math
import sys
import threading
import socket
from PyQt5.QtCore import QObject, pyqtSignal

class ReceiveData_Phone(QObject):
    log_signal = pyqtSignal(str)

    """
    Act Like server , that recive data from the phone 
    """
    def __init__(self, host, port=5555):
        super().__init__()

        self.__socket = None
        self.__server_address = (host, port)
        self.__connection = None


    # def update_car_acceleration_follow_2(self, car_acceleration_data_queue,car_acceleration_data_queue_pc):
    #     """
    #     update car_state received from CAR via sockets
    #     """
    #     current_thread = threading.currentThread()
    #     self.__socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    #     self.__socket.bind(self.__server_address)
    #     # We want only one client
    #     self.__socket.listen(1)
    #     self.log_signal.emit("Waiting for connection...")

    #     # Listen to infinite connection if Client disconnect
    #     while getattr(current_thread, 'is_running', True):
    #         self.__connection, client_address = self.__socket.accept()
    #         try:
    #             while getattr(current_thread, 'is_connected', True):
    #                 data = self.__connection.recv(1024) #valid
    #                 if data:
    #                     self.log_signal.emit("Recvied: " + str(data))
    #                     data_sets = data.decode().strip().split('\n')
    #                     # data_type, data_values = data_line.split(':', 1)  # Split data type and values
    #                     for data_line in data_sets:
    #                         data_line = data_line.split(':', 1)  # Split data type and values
    #                         if (len(data_line) > 1):
    #                             data_type = data_line[0]
    #                             data_values = data_line[1]
    #                             if data_type == "GYO":
    #                                 # rotation_values = [float(val) for val in data_values.strip().split(',')]
    #                                 print(f"Received gyroscope data from client : {data_values}")
                                
    #                             elif data_type == "ACC":
    #                                 # acc_values = [float(val) for val in data_values.strip().split(',')]
    #                                 print(f"Received acceleration data from client : {data_values}")
    #                                 # save all user commands to a queue
    #                                 car_acceleration_data_queue.put(str(data), True, None)
    #                                 car_acceleration_data_queue_pc.put(str(data), True, None)
    #                 else:
    #                     break
    #         finally:
    #             # Clean up the connection
    #             self.__connection.close()
    #     self.__socket.close()
    def update_car_acceleration_follow_2(self, car_acceleration_data_queue,car_acceleration_data_queue_pc):
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
                        self.log_signal.emit("Recvied: " + str(data))
                        data_sets = data.decode().strip().split('\n')
                        # data_type, data_values = data_line.split(':', 1)  # Split data type and values
                        for data_line in data_sets:
                            data_line = data_line.split(':', 1)  # Split data type and values
                            if (len(data_line) > 3):
                                data_type = data_line[0]
                                data_values = data_line[1]

                                if data_type == "x":
                                    pass
                                    
                                elif data_type == "x":
                                    # acc_values = [float(val) for val in data_values.strip().split(',')]
                                    print(f"Received acceleration data from client : {data_values}")
                                    # save all user commands to a queue
                                    car_acceleration_data_queue.put(str(data), True, None)
                                    car_acceleration_data_queue_pc.put(str(data), True, None)
                    else:
                        break
            finally:
                # Clean up the connection
                self.__connection.close()
        self.__socket.close()


    def update_car_acceleration(self, car_acceleration_data_queue,car_acceleration_data_queue_pc):
        """
        update car_state received from CAR via sockets
        """
        current_thread = threading.currentThread()
        # Create a UDP socket

        self.__socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.__socket.bind(self.__server_address)
        # We want only one client
        # self.__socket.listen(1)
        self.log_signal.emit("Waiting for connection accel...")

        # Listen to infinite connection if Client disconnect
        while getattr(current_thread, 'is_running', True):

            data = self.__socket.recv(800) #valid
            
            # self.log_signal.emit("Recvied: " + str(data))
            # data_sets = data.decode().strip().split('\n')
            data = data.decode().strip().split(',')
            linear_accel_x_data = parse_linear_acceleration(data)
            if (linear_accel_x_data != 999 ):
                # linear_accel_data_magitude =  math.sqrt(linear_accel_data[0]**2 + linear_accel_data[1]**2 + linear_accel_data[2]**2)
                # self.log_signal.emit(f"Recvied linear accel: {linear_accel_x_data}" )
                # print(f"Recvied linear accel: {linear_accel_x_data}" )
                car_acceleration_data_queue.put(linear_accel_x_data, True, None)
                car_acceleration_data_queue_pc.put(linear_accel_x_data, True, None)

        # Clean up the connection
        self.__socket.close()

    def update_car_acceleration_follow(self, car_acceleration_data_queue):
        """
        update car_state received from CAR via sockets
        """
        current_thread = threading.currentThread()
        # Create a UDP socket

        self.__socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.__socket.bind(self.__server_address)
        # We want only one client
        # self.__socket.listen(1)
        self.log_signal.emit("Waiting for connection accel...")

        # Listen to infinite connection if Client disconnect
        while getattr(current_thread, 'is_running', True):
            # self.__connection, client_address = self.__socket.accept()
            try:
                while getattr(current_thread, 'is_connected', True):
                    data = self.__socket.recv(1024) #valid
                    if data:
                        # self.log_signal.emit("Recvied: " + str(data))
                        # data_sets = data.decode().strip().split('\n')
                        data = data.decode().strip().split(',')
                        linear_accel_x_data = parse_linear_acceleration(data)
                        if (linear_accel_x_data != 999 ):
                            # linear_accel_data_magitude =  math.sqrt(linear_accel_data[0]**2 + linear_accel_data[1]**2 + linear_accel_data[2]**2)
                            # self.log_signal.emit(f"Recvied linear accel: {linear_accel_x_data}" )
                            # print(f"Recvied linear accel: {linear_accel_x_data}" )
                            car_acceleration_data_queue.put(linear_accel_x_data, True, None)

                    else:
                        break
            finally:
                # Clean up the connection
                self.__socket.close()
        # self.__socket.close()


def parse_linear_acceleration(data):
    # linear_accel_data = []
    # print(data , len(data))
    # Iterate through the data in groups of 4 values
    for i in range(8, len(data)):
        try:
            sensor_type = int(data[i])
            # print(sensor_type)

            if sensor_type == 82:  # Sensor type 5 corresponds to Linear Acceleration Data
                # print(sensor_type)
                return data[i + 1]
                # linear_accel_data.extend(map(float, data[i + 1:i + 4]))
        except ValueError:
            pass  # Skip non-integer sensor types
    return 999






    
