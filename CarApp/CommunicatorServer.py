"""
This module will implement in The car file arduino
"""

"""
Comunicator Server module
"""
import threading
import socket
import sys

class CommunicatorServer(object):
    """
    Comunicator Server class
    receive the commands sent by user
    store them in a Queue to be processed by CarManager
    """
    def __init__(self, host, port=32656):
        self.__socket = None
        self.__server_address = (host, port)
        self.__connection = None

    def update_user_commands(self, user_commands_queue):
        """
        update user commands received from Remote App via sockets
        """
        current_thread = threading.currentThread()
        self.__socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.__socket.bind(self.__server_address)
        # We want only one client
        self.__socket.listen(1)
        # Listen to infinite connection if Client disconnect
        while getattr(current_thread, 'is_running', True):
            self.__connection, client_address = self.__socket.accept()
            try:
                while getattr(current_thread, 'is_connected', True):
                    data = self.__connection.recv(1024) #valid
                    if data:
                        # save all user commands to a queue
                        user_commands_queue.put(str(data), True, None)
                    else:
                        break
            finally:
                # Clean up the connection
                self.__connection.close()
        self.__socket.close()
