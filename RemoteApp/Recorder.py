"""
// TO DO HUY 
This module need to be in the server side (PC Side)
"""
"""
Recorder module
This module provide a class that get the frame from the camera continously
"""
import threading
import socket
import time
import cv2
import numpy
from PyQt5.QtCore import Qt, QObject, pyqtSignal


class Recorder(QObject):
    log_signal = pyqtSignal(str)
    """
    Recorder class - get frame from camera continously
    Store the captured frame in a Queue
    Encrypt the image for low data transfering
    """
    def __init__(self, video_source , b_high_resolution=False):
        super().__init__()

        self.video_source = video_source
        self.__camera = None
        self.__encode_parameter = [int(cv2.IMWRITE_JPEG_QUALITY), 60]
        self.__high_resolution_enabled = b_high_resolution
        self.event = threading.Event()


    def record(self, frame_queue):
        """
        record frame
        # """

        self.event.wait()

        self.__camera = cv2.VideoCapture(self.video_source)

        if bool(self.__high_resolution_enabled) is False:
            ret = self.__camera.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
            ret = self.__camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

        current_thread = threading.currentThread()
        self.log_signal.emit("Start Camera")
        while getattr(current_thread, 'is_running', True):
            ret, frame = self.__camera.read()
            if ret :
                frame_queue.put(frame)
            else:
                self.log_signal.emit("No connection with Camera")
                break

        self.__camera.release()
