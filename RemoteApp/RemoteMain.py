"""
Main for Remote App
"""
import sys
import threading
import numpy
import cv2
import queue
from PyQt5 import QtGui, QtWidgets, QtCore
from PyQt5.QtWidgets import QPlainTextEdit


# import CommunicatorClient
# import DataProviderClient

import SendData_Car
import SendData_PC_server

import ReceiveData_Car
import ReceiveData_Phone

import ReceiveData_PC_server

import Utils


import Analyser
import Recorder

FRAME_QUEUE = queue.Queue(1)
ANALYSED_FRAME_QUEUE = queue.Queue(1)
COMMANDS_QUEUE = queue.Queue(1)
USER_COMMANDS_QUEUE = queue.Queue(1)
AUTONOMOUS_STATES_QUEUE = queue.Queue(1)
CAR_DATA_QUEUE = queue.Queue(1)
CAR_STATES_QUEUE = queue.Queue(1) # Will send 

Acceleration_CAR_QUEUE = queue.Queue(1)
Distance_Velocity_CAR_QUEUE = queue.Queue(1)

#  some queue but , to send to PC (following)
Acceleration_CAR_QUEUE_PC = queue.Queue(1)
Distance_Velocity_CAR_QUEUE_PC = queue.Queue(1)





from enum import Enum

class CarState(Enum):
    STOP = 0
    DEPART = 1
    FORWARD = 2
    BACKWARD = 3


isforward = True

# string resources
CMD_GO_FORWARD = 'FORWARD'
CMD_INCREASE_SPEED = 'UP:\n'
CMD_DECREASE_SPEED = 'DE:\n'
CMD_BRAKE = 'BRAKE:\n'
CMD_GO_LEFT = 'LEFT:\n'
CMD_GO_RIGHT = 'RIGHT:\n'
CMD_GO_BACKWARD = 'BACKWARD'
CMD_STEER = 'STEER'
CMD_STOP = 'STOP:\n'

# CSQ = car state queue , use for parsing the data , that recevie from car 
CSQ_CRUISE_DISTANCE = 'CRUISE_DISTANCE'
CSQ_CRUISE_SPEED = 'CRUISE_SPEED'
CSQ_CRUISE_PREFFERED_SPEED = 'CRUISE_PREF_SPEED'
CSQ_ACCELERATION = 'ACCE'

CSQ_DISTANCE = 'dist'
CSQ_SPEED= 'vel'
CSQ_ACCEL= 'accel'




# ----------------- Change here -------------------
video_source = 'http://192.168.92.159:4747/video'
# video_source = 'http://192.168.69.88:4747/video'
# video_source = 'http://192.168.69.88:4747/video'  # phone 





try:
    #we want to use the same name form
    FROM_UTF8 = QtCore.QString.fromUtf8
except AttributeError:
    def from_utf8(string_name):
        """
        from utf8 alternative
        """
        return string_name
    FROM_UTF8 = from_utf8

try:
    ENCODING = QtWidgets.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtWidgets.QApplication.translate(context, text, disambig, ENCODING)
except AttributeError:
    def _translate(context, text, disambig):
        return QtWidgets.QApplication.translate(context, text, disambig)

class OwnImageWidget(QtWidgets.QWidget):
    """
    own class for image widget (adapted for numpy arrays)
    """
    def __init__(self, parent=None):
        super(OwnImageWidget, self).__init__(parent)
        self.image = None

    def set_image(self, image):
        """
        set the current image to be showed
        """
        self.image = image
        image_size = image.size()
        self.setMinimumSize(image_size)
        self.update()

    #overwritten function from Qt Library
    def paintEvent(self, event):
        """
        paint event
        """
        qpainter = QtGui.QPainter()
        qpainter.begin(self)
        if self.image:
            qpainter.drawImage(QtCore.QPoint(0, 0), self.image)
        qpainter.end()



class RemoteMain(object):
    """
    Remote Main Class
    """

    def __init__(self, server_host):
        """
        All components needed for UI
        * declared first in __init__ as None
        """
        self.__host = server_host

        self.__recorder = None
        self.__streamer_client = None
        self.__communicator_client = None
        self.__data_provider_client = None
        self.__analyser = None

        self.__streamer_client_thread = None
        self.__communicator_client_thread = None
        self.__data_provider_client_thread = None
        self.__analyser_thread = None

        self.update_frame_timer = None
        self.update_car_data_timer = None
        self.superviser_timer = None

        self.__current_command = ''

        self.__acc_activated = False
        self.__decisions_activated = False
        self.__cruise_watch_area = 4
        self.__cruise_speed = 0
        self.__cruise_preffered_speed = 0

        self.window_width = None
        self.window_height = None
        self.centralwidget = None
        self.grid_layout = None
        self.distance_buttons_layout = None
        self.acc_activate_button = None
        self.decisions_activate_button = None
        self.increase_distance_button = None
        self.decrease_distance_button = None
        self.streamer_image_layout = None
        self.streamer_image_view = None
        self.speed_buttons_layout = None
        self.speed_up_button = None
        self.speed_down_button = None
        self.brake_button = None
        self.states_layout = None
        self.speed_label = None
        self.speed_text = None
        self.preferred_speed_label = None
        self.preferred_speed_text = None
        self.cruise_distance_label = None
        self.cruise_distance_text = None
        self.detection_label = None
        self.detection_text = None

        self.log_label = None
        self.log_text = None
        self.command_label = None
        self.command_text = None
        self.statusbar = None

        # self.log_signal = self.LogSignal()

        self.Acceleration_CAR_data = 0
        self.Distance_Velocity_CAR_data = [3]

        #  some queue but , to send to PC (following)
        self.Acceleration_CAR_data_PC = 0  
        self.Distance_Velocity_CAR_data_PC = [3] 

        self.utils = Utils.Utils()


    def setup_ui(self, main_window):
        """
        Setup UI components
        """


        main_window.setObjectName(FROM_UTF8("main_window"))
        main_window.resize(660, 700)



        main_window.keyPressEvent = self.key_press_dynamic
        # main_window.keyReleaseEvent = self.key_release_event

        # self.key_pressed = set()

        # self.key_timer = QtCore.QTimer(self.centralwidget)
        # self.key_timer.timeout.connect(self.parse_vehicle_keys)
        # self.key_timer.start(100)  # Start the timer for continuous key checking

        
        main_window.closeEvent = self.close_event
        self.centralwidget = QtWidgets.QWidget(main_window)
        self.centralwidget.setObjectName(FROM_UTF8("centralwidget"))
        self.grid_layout = QtWidgets.QGridLayout(self.centralwidget)
        self.grid_layout.setObjectName(FROM_UTF8("grid_layout"))
        self.distance_buttons_layout = QtWidgets.QVBoxLayout()
        self.distance_buttons_layout.setObjectName(FROM_UTF8("distance_buttons_layout"))
        self.acc_activate_button = QtWidgets.QPushButton(self.centralwidget)
        self.acc_activate_button.setObjectName(FROM_UTF8("acc_activate_button"))
        self.distance_buttons_layout.addWidget(self.acc_activate_button)
        self.decisions_activate_button = QtWidgets.QPushButton(self.centralwidget)
        self.decisions_activate_button.setObjectName(FROM_UTF8("decisions_activate_button"))
        self.distance_buttons_layout.addWidget(self.decisions_activate_button)
        self.increase_distance_button = QtWidgets.QPushButton(self.centralwidget)
        self.increase_distance_button.setObjectName(FROM_UTF8("increase_distance_button"))
        self.distance_buttons_layout.addWidget(self.increase_distance_button)
        self.decrease_distance_button = QtWidgets.QPushButton(self.centralwidget)
        self.decrease_distance_button.setObjectName(FROM_UTF8("decrease_distance_button"))
        self.distance_buttons_layout.addWidget(self.decrease_distance_button)
        self.grid_layout.addLayout(self.distance_buttons_layout, 1, 0, 3, 1)
        self.streamer_image_layout = QtWidgets.QVBoxLayout()
        self.streamer_image_layout.setObjectName(FROM_UTF8("streamer_image_layout"))
        self.streamer_image_view = QtWidgets.QLabel(self.centralwidget)
        self.streamer_image_view = OwnImageWidget(self.streamer_image_view)
        self.streamer_image_view.setMinimumSize(QtCore.QSize(640, 480))
        self.streamer_image_view.setObjectName(FROM_UTF8("streamer_image_view"))
        self.streamer_image_layout.addWidget(self.streamer_image_view)
        self.grid_layout.addLayout(self.streamer_image_layout, 0, 0, 1, 6)
        self.speed_buttons_layout = QtWidgets.QVBoxLayout()
        self.speed_buttons_layout.setObjectName(FROM_UTF8("speed_buttons_layout"))
        self.speed_up_button = QtWidgets.QPushButton(self.centralwidget)
        self.speed_up_button.setObjectName(FROM_UTF8("speed_up_button"))
        self.speed_buttons_layout.addWidget(self.speed_up_button)
        self.speed_down_button = QtWidgets.QPushButton(self.centralwidget)
        self.speed_down_button.setObjectName(FROM_UTF8("speed_down_button"))
        self.speed_buttons_layout.addWidget(self.speed_down_button)
        self.brake_button = QtWidgets.QPushButton(self.centralwidget)
        self.brake_button.setObjectName(FROM_UTF8("brake_button"))
        self.speed_buttons_layout.addWidget(self.brake_button)
        self.grid_layout.addLayout(self.speed_buttons_layout, 1, 5, 3, 1)
        self.states_layout = QtWidgets.QFormLayout()
        self.states_layout.setObjectName(FROM_UTF8("states_layout"))
        self.speed_label = QtWidgets.QLabel(self.centralwidget)
        self.speed_label.setObjectName(FROM_UTF8("speed_label"))
        self.states_layout.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.speed_label)
        self.speed_text = QtWidgets.QLineEdit(self.centralwidget)
        self.speed_text.setReadOnly(True)
        self.speed_text.setObjectName(FROM_UTF8("speed_text"))
        self.states_layout.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.speed_text)

        self.preferred_speed_label = QtWidgets.QLabel(self.centralwidget)
        self.preferred_speed_label.setObjectName(FROM_UTF8("preferred_speed_label"))
        self.states_layout.setWidget(1, QtWidgets.QFormLayout.LabelRole, self.preferred_speed_label)

        self.preferred_speed_text = QtWidgets.QLineEdit(self.centralwidget)
        self.preferred_speed_text.setReadOnly(True)
        self.preferred_speed_text.setObjectName(FROM_UTF8("preferred_speed_text"))
        self.states_layout.setWidget(1, QtWidgets.QFormLayout.FieldRole, self.preferred_speed_text)

        self.cruise_distance_label = QtWidgets.QLabel(self.centralwidget)
        self.cruise_distance_label.setObjectName(FROM_UTF8("cruise_distance_label"))
        self.states_layout.setWidget(2, QtWidgets.QFormLayout.LabelRole, self.cruise_distance_label)
        self.cruise_distance_text = QtWidgets.QLineEdit(self.centralwidget)
        self.cruise_distance_text.setReadOnly(True)
        self.cruise_distance_text.setObjectName(FROM_UTF8("cruise_distance_text"))
        self.states_layout.setWidget(2, QtWidgets.QFormLayout.FieldRole, self.cruise_distance_text)
        self.detection_label = QtWidgets.QLabel(self.centralwidget)
        self.detection_label.setObjectName(FROM_UTF8("detection_label"))
        self.states_layout.setWidget(3, QtWidgets.QFormLayout.LabelRole, self.detection_label)
        self.detection_text = QtWidgets.QLineEdit(self.centralwidget)
        self.detection_text.setReadOnly(True)
        self.detection_text.setObjectName(FROM_UTF8("detection_text"))
        self.states_layout.setWidget(3, QtWidgets.QFormLayout.FieldRole, self.detection_text)





        self.command_label = QtWidgets.QLabel(self.centralwidget)
        self.command_label.setObjectName(FROM_UTF8("command_label"))
        self.states_layout.setWidget(4, QtWidgets.QFormLayout.LabelRole, self.command_label)
        self.command_text = QtWidgets.QLineEdit(self.centralwidget)
        self.command_text.setReadOnly(True)
        self.command_text.setObjectName(FROM_UTF8("command_text"))
        self.states_layout.setWidget(4, QtWidgets.QFormLayout.FieldRole, self.command_text)

        self.log_label = QtWidgets.QLabel(self.centralwidget)
        self.log_label.setObjectName(FROM_UTF8("log_label"))
        self.states_layout.setWidget(5, QtWidgets.QFormLayout.LabelRole, self.log_label)
        # self.log_text = QtWidgets.QLineEdit(self.centralwidget)
        # self.log_text.setReadOnly(True)
        # self.log_text.setObjectName(FROM_UTF8("log_text"))
        # self.states_layout.setWidget(5, QtWidgets.QFormLayout.FieldRole, self.log_text)

        self.log_text = QPlainTextEdit(self.centralwidget)
        self.log_text.setReadOnly(True)
        self.log_text.setObjectName(FROM_UTF8("log_text"))
        self.states_layout.setWidget(5, QtWidgets.QFormLayout.FieldRole, self.log_text)
        # self.grid_layout.addWidget(self.log_text, 3, 1, 1, 4)


        self.grid_layout.addLayout(self.states_layout, 1, 1, 3, 4)
        main_window.setCentralWidget(self.centralwidget)
        self.statusbar = QtWidgets.QStatusBar(main_window)
        self.statusbar.setObjectName(FROM_UTF8("statusbar"))
        main_window.setStatusBar(self.statusbar)


        

        """
        Setup each module to start on each thread
        """

        self.car_state = CarState.FORWARD

        self.__recorder = Recorder.Recorder(video_source)
        self.__recorder.log_signal.connect(self.update_log)

        self.__send_client_CAR = SendData_Car.SendData_Car(self.__host)
        self.__send_client_CAR.log_signal.connect(self.update_log)

        self.__send_to_client_PC = SendData_PC_server.SendData_PC_server(self.__host)
        self.__send_to_client_PC.log_signal.connect(self.update_log)

        self.__recive_data_client_CAR = ReceiveData_Car.ReceiveData_Car(self.__host)
        self.__recive_data_client_CAR.log_signal.connect(self.update_log)

        self.__recive_data_client_Phone = ReceiveData_Phone.ReceiveData_Phone(self.__host)
        self.__recive_data_client_Phone.log_signal.connect(self.update_log)

        self.__analyser = Analyser.Analyser()
        
        """ Threads """

        # receive frames thread
        self.__recorder_thread = \
            threading.Thread(target=Recorder.Recorder.record , \
                             args=(self.__recorder , FRAME_QUEUE))
        self.__recorder_thread.daemon = (True)
        self.__recorder_thread.start()

        """ That config for Leader CAR (2 send , 2 receive)"""
        # send commands to car thread
        self.__send_client_CAR_thread = \
            threading.Thread(target=SendData_Car.SendData_Car.send_commands, \
            args=(self.__send_client_CAR , COMMANDS_QUEUE ))
        self.__send_client_CAR_thread.daemon = True
        self.__send_client_CAR_thread.start()

        # send commands  (Lead car) to PC (car following) thread
        self.__send_to_client_PC_thread = \
            threading.Thread(target=SendData_PC_server.SendData_PC_server.send_data_2_following, \
            args=(self.__send_to_client_PC, Acceleration_CAR_QUEUE_PC,Distance_Velocity_CAR_QUEUE_PC ))
        self.__send_to_client_PC_thread.daemon = True
        self.__send_to_client_PC_thread.start()

        #  Receive from car sensor
        self.__recive_data_client_CAR_thread = \
            threading.Thread(target=ReceiveData_Car.ReceiveData_Car.update_car_distance_vel, \
            args=(self.__recive_data_client_CAR, Distance_Velocity_CAR_QUEUE,Distance_Velocity_CAR_QUEUE_PC))
        self.__recive_data_client_CAR_thread.daemon = True
        self.__recive_data_client_CAR_thread.start()

        #  Receive from phone sensor (act like car sensor)
        self.__recive_data_client_Phone_thread = \
            threading.Thread(target=ReceiveData_Phone.ReceiveData_Phone.update_car_acceleration, \
            args=(self.__recive_data_client_Phone,Acceleration_CAR_QUEUE,Acceleration_CAR_QUEUE_PC))
        self.__recive_data_client_Phone_thread.daemon = True
        self.__recive_data_client_Phone_thread.start()


        # analyse every frame and take decisions of the car 
        ########## TODO HUY :,(MAY BE) not need frame_queue now beacause we can get direct here

        self.__analyser_thread = \
            threading.Thread(target=Analyser.Analyser.analyse_pre, \
            args=(self.__analyser, FRAME_QUEUE, ANALYSED_FRAME_QUEUE, \
            COMMANDS_QUEUE, CAR_STATES_QUEUE,Acceleration_CAR_QUEUE , Distance_Velocity_CAR_QUEUE))
        self.__analyser_thread.daemon = True
        self.__analyser_thread.start()
        self.__analyser_thread.is_analysing = self.__acc_activated

        """
        Update each module with its own timer
        """
        # region UPDATE loop of the modules
        
        # update frames thread method every 10 milliseconds
        self.update_frame_timer = QtCore.QTimer(self.streamer_image_layout)
        self.update_frame_timer.timeout.connect(self.__update_frame)
        self.update_frame_timer.start(10)

        # update car data thread method every 200 milliseconds
        self.update_car_data_timer = QtCore.QTimer(self.states_layout)
        self.update_car_data_timer.timeout.connect(self.__update_car_data)
        self.update_car_data_timer.start(200)

        # supervise the user settings and sync entire app thread
        self.superviser_timer = QtCore.QTimer(self.centralwidget)
        self.superviser_timer.timeout.connect(self.__superviser_thread)
        self.superviser_timer.start(100)

        # endregion UPDATE loop of the modules

        self.window_width = self.streamer_image_view.frameSize().width()
        self.window_height = self.streamer_image_view.frameSize().height()

        """
        Register Button event 
        """
        self.speed_up_button.clicked.connect(self.__speed_up_button_clicked)
        self.speed_down_button.clicked.connect(self.__speed_down_button_clicked)
        self.brake_button.clicked.connect(self.__brake_button_clicked)
        self.increase_distance_button.clicked.connect(self.__increase_distance_btn_clicked)
        self.decrease_distance_button.clicked.connect(self.__decrease_distance_btn_clicked)
        self.acc_activate_button.clicked.connect(self.__acc_activate_button_clicked)
        self.decisions_activate_button.clicked.connect(self.__decisions_button_clicked)

        self.retranslate_ui(main_window)
        QtCore.QMetaObject.connectSlotsByName(main_window)

    def retranslate_ui(self, main_window):
        """
        UI retranslations
        """
        main_window.setWindowTitle(\
            _translate("main_window", "Adaptive Cruise Control - Preceding", None))
        self.acc_activate_button.setText(_translate("main_window", "ACC", None))
        self.decisions_activate_button.setText(_translate("main_window", "Decisions", None))
        self.increase_distance_button.setText(_translate("main_window", "Increase Distance", None))
        self.decrease_distance_button.setText(_translate("main_window", "Decrease Distance", None))
        self.speed_up_button.setText(_translate("main_window", "Speed Up", None))
        self.speed_down_button.setText(_translate("main_window", "Speed Down", None))
        self.brake_button.setText(_translate("main_window", "Brake", None))
        self.speed_label.setText(_translate("main_window", "Speed", None))
        self.preferred_speed_label.setText(_translate("main_window", "Preferred Speed", None))
        self.cruise_distance_label.setText(_translate("main_window", "Cruise Distance", None))
        self.detection_label.setText(_translate("main_window", "Detection", None))
        self.command_label.setText(_translate("main_window", "Command", None))
        self.command_label.setText(_translate("main_window", "LOGs", None))

    def update_log(self, message):
        self.log_text.appendPlainText(message)


    

    """
    Button event trigger
    """
    # region Button event trigger

    
    
    def __acc_activate_button_clicked(self):
        self.__acc_activated = not self.__acc_activated
        self.__analyser_thread.is_analysing = self.__acc_activated

    

    def __decisions_button_clicked(self):
        self.__decisions_activated = not self.__decisions_activated
        self.__analyser_thread.is_deciding = self.__decisions_activated

    def __speed_up_button_clicked(self ):
        if self.__cruise_preffered_speed < 250:
            if self.__cruise_preffered_speed < 70:
                self.__cruise_preffered_speed = 70
            else:
                self.__cruise_preffered_speed = self.__cruise_preffered_speed +10

    def __speed_down_button_clicked(self ):
        if self.__cruise_speed == 0:
            self.__cruise_preffered_speed = 0
        if self.__cruise_preffered_speed > 70:
            self.__cruise_preffered_speed = self.__cruise_preffered_speed - 10
        else:
            self.__cruise_preffered_speed = 0

    def __brake_button_clicked(self):
        self.__cruise_preffered_speed = 0
        print ('brake')

    def __increase_distance_btn_clicked(self):
        if self.__cruise_watch_area < 5:
            self.__cruise_watch_area = self.__cruise_watch_area + 1
        print ('increase ' + str(self.__cruise_watch_area))

    def __decrease_distance_btn_clicked(self):
        if self.__cruise_watch_area > 1:
            self.__cruise_watch_area = self.__cruise_watch_area - 1
        print ('decrease ' + str(self.__cruise_watch_area))


    def key_press_event_fixed(self, event):
        """
        key press
        """
        key = event.key()

        if key == QtCore.Qt.Key_Escape:
            sys.exit()
        elif key == QtCore.Qt.Key_W:
            COMMANDS_QUEUE.put(CMD_INCREASE_SPEED, True, None)
        elif key == QtCore.Qt.Key_A:
            COMMANDS_QUEUE.put(CMD_GO_LEFT, True, None)
        elif key == QtCore.Qt.Key_S:
            COMMANDS_QUEUE.put(CMD_DECREASE_SPEED, True, None)
        elif key == QtCore.Qt.Key_D:
            COMMANDS_QUEUE.put(CMD_GO_RIGHT, True, None)
        elif key == QtCore.Qt.Key_M:
            COMMANDS_QUEUE.put(CMD_BRAKE, True, None)
        elif key == QtCore.Qt.Key_R:
            COMMANDS_QUEUE.put(CMD_GO_BACKWARD, True, None)
        elif key == QtCore.Qt.Key_I:
            self.__analyser_thread.is_analysing = True
        elif key == QtCore.Qt.Key_O:
            self.__analyser_thread.is_analysing = False


    def key_press_dynamic(self, event):
        """
        key press
        """
        key = event.key()

        global isforward
        if key == QtCore.Qt.Key_Escape:
            sys.exit()
        elif key == QtCore.Qt.Key_Z:
            # recover when the car in STOP STATE 
            if (self.utils._throttle < self.utils.MIN_SPEED):
                self.utils._throttle = self.utils.MIN_SPEED
            
            # print("press w")
            if (isforward):
                self.utils._throttle = min(self.utils._throttle + self.utils.SPEED_UP, self.utils.MAX_SPEED)
                data = CMD_GO_FORWARD + ":" + str(self.utils._throttle) + "\n"
                COMMANDS_QUEUE.put(data, True, None)
            else:
                self.utils._throttle = max(self.utils._throttle - self.utils.SPEED_DOWN, self.utils.MIN_SPEED)
                data = CMD_GO_FORWARD + ":" + str(self.utils._throttle) + "\n"
                COMMANDS_QUEUE.put(data, True, None)
            # self.__send_client_CAR.send_direct_commands(f'FORWARD:{self.utils._throttle}')
        
        elif key == QtCore.Qt.Key_S:
            if (self.utils._throttle < self.utils.MIN_SPEED):
                self.utils._throttle = self.utils.MIN_SPEED
            if (isforward):
                self.utils._throttle = max(self.utils._throttle - self.utils.SPEED_DOWN, self.utils.MIN_SPEED)
                data = CMD_GO_FORWARD + ":" + str(self.utils._throttle) + "\n"
                COMMANDS_QUEUE.put(data, True, None)
            else:
                self.utils._throttle = min(self.utils._throttle + self.utils.SPEED_UP, self.utils.MAX_SPEED)
                data = CMD_GO_FORWARD + ":" + str(self.utils._throttle) + "\n"
                COMMANDS_QUEUE.put(data, True, None)
            
        
        elif key == QtCore.Qt.Key_Q:
            if self.utils._steer_cache < 0 :
                self.utils._steer_cache = 0
            else:
                self.utils._steer_cache += self.utils.STEER_PULSE

            self.utils._steer_cache = min(0.9, max(-0.9, self.utils._steer_cache))
            self.steer = round(self.utils._steer_cache, 1)

            data = CMD_STEER + ":" + str(self.utils.map_value_Steer_ESP2(self.steer)) + "\n"

            COMMANDS_QUEUE.put(data, True, None)
            # COMMANDS_QUEUE.put(CMD_GO_LEFT, True, None)

            # self.__send_client_CAR.send_direct_commands(f'STEER:{self.utils.map_value_Steer_ESP2(self.steer)}\n')


        elif key == QtCore.Qt.Key_D:
            if self.utils._steer_cache > 0 :
                self.utils._steer_cache = 0
            else:
                self.utils._steer_cache -= self.utils.STEER_PULSE

            self.utils._steer_cache = min(0.9, max(-0.9, self.utils._steer_cache))
            self.steer = round(self.utils._steer_cache, 1)
            data = CMD_STEER + ":" + str(self.utils.map_value_Steer_ESP2(self.steer)) + "\n"
            COMMANDS_QUEUE.put(data, True, None)
            # # self.__send_client_CAR.send_direct_commands(f'STEER:{self.utils.map_value_Steer_ESP2(self.steer)}\n')
            # COMMANDS_QUEUE.put(CMD_GO_RIGHT, True, None)


            
        elif key == QtCore.Qt.Key_T:
            # CMD_BRAKE
            self.utils._throttle = 0
            self.utils._steer_cache = 0
            self.__send_client_CAR.send_direct_commands(f'FORWARD:0\n')
            self.__send_client_CAR.send_direct_commands(f'STEER:{self.utils.map_value_Steer_ESP2(0)}\n')

            # def save_data_to_file(data):
            with open('data.txt', 'a') as file:
                data_string = ' '.join(map(str, self.__recive_data_client_CAR.list_vel))
                file.write(data_string + '\n')

        elif key == QtCore.Qt.Key_F:
            # CMD_BRAKE
            self.utils._throttle = self.utils.START_SPEED
            data = CMD_GO_FORWARD + ":" + str(self.utils._throttle) + "\n"
            COMMANDS_QUEUE.put(data, True, None)

            
            # self.__send_client_CAR.send_direct_commands(f'FORWARD:0\n')
            # self.__send_client_CAR.send_direct_commands(f'STEER:{self.utils.map_value_Steer_ESP2(0)}\n')
            
        elif key == QtCore.Qt.Key_R:
            isforward = (isforward + 1) % 2

            if (isforward == 1): 
                self.update_log( "ForWard mode ")
            else:
                self.update_log( "BackWard mode ")
            
            COMMANDS_QUEUE.put(CMD_GO_BACKWARD +str(isforward)+"\n", True, None)
            # self.__send_client_CAR.send_direct_commands(f'BACKWARD:\n')

        elif key == QtCore.Qt.Key_I:
            COMMANDS_QUEUE.put(CMD_INCREASE_SPEED, True, None)
        elif key == QtCore.Qt.Key_J:
            COMMANDS_QUEUE.put(CMD_GO_LEFT, True, None)
        elif key == QtCore.Qt.Key_K:
            COMMANDS_QUEUE.put(CMD_DECREASE_SPEED, True, None)
        elif key == QtCore.Qt.Key_L:
            COMMANDS_QUEUE.put(CMD_GO_RIGHT, True, None)

        elif key == QtCore.Qt.Key_P:
            if (self.__recorder.event.is_set()):
                self.__recorder.event.clear()
            else:
                self.__recorder.event.set()
        
        # elif key == QtCore.Qt.Key_R:
        #     self.__send_client_CAR.send_direct_commands(f'BACKWARD:{self.utils.map_value_Steer_ESP2(self.steer)}\n')
        elif key == QtCore.Qt.Key_V:
            self.__analyser_thread.is_analysing = True
        elif key == QtCore.Qt.Key_B:
            self.__analyser_thread.is_analysing = False
        else:
            self.utils._steer_cache = 0.0


            

    # endregion Button event trigger
    def __update_frame(self):
        # string_data = ANALYSED_FRAME_QUEUE.get(True, None)
        # data = numpy.fromstring(string_data, dtype='uint8')
        # cv_image = cv2.imdecode(data, 1)
        try : 
            cv_image = ANALYSED_FRAME_QUEUE.get(False, None)
        except queue.Empty:
            cv_image = None
        if cv_image is not None:
            img_height, img_width, img_colors = cv_image.shape
            scale_w = float(self.window_width) / float(img_width)
            scale_h = float(self.window_height) / float(img_height)
            scale = min([scale_w, scale_h])

            if scale == 0:
                scale = 1

            cv_image = \
                cv2.resize(cv_image, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            height, width, bpc = cv_image.shape
            bpl = bpc * width
            image = QtGui.QImage(cv_image.data, width, height, bpl, QtGui.QImage.Format_RGB888)
            self.streamer_image_view.set_image(image)
            ANALYSED_FRAME_QUEUE.task_done()
    def __update_car_data(self):
        if (self.utils._steer_cache > 0 and abs(self.utils._steer_cache) < self.utils.STEER_SEUIL_TO_Center) or (self.utils._steer_cache < 0 and abs(self.utils._steer_cache) < self.utils.STEER_SEUIL_TO_Center)  :
            self.utils._steer_cache = 0
        elif self.utils._steer_cache > 0 and abs(self.utils._steer_cache) > self.utils.STEER_SEUIL_TO_Center:
            self.utils._steer_cache -= self.utils.STEER_decay
        elif self.utils._steer_cache < 0 and abs(self.utils._steer_cache) > self.utils.STEER_SEUIL_TO_Center: 
            self.utils._steer_cache += self.utils.STEER_decay
        else:
            self.utils._steer_cache = 0

        try:
            car_data = CAR_DATA_QUEUE.get(False)
        except queue.Empty:
            return
        car_data = car_data.split(';')
        for elem in car_data:
            current_state = elem.split(',')
            if len(current_state) > 1:
                if 'ACTION' in current_state[0]:
                    self.__current_command = str(current_state[1])
                elif 'SPEED' in current_state[0]:
                    self.__cruise_speed = int(current_state[1])
        CAR_DATA_QUEUE.task_done()

    def __superviser_thread(self):
        if bool(self.__acc_activated) is False:
            self.acc_activate_button.setStyleSheet("background-color: red")
        else:
            self.acc_activate_button.setStyleSheet("background-color: green")

        if bool(self.__decisions_activated) is False:
            self.decisions_activate_button.setStyleSheet("background-color: red")
        else:
            self.decisions_activate_button.setStyleSheet("background-color: green")

        self.speed_text.setText(_translate("main_window", str(self.__cruise_speed), None))
        self.command_text.setText(_translate("main_window", str(self.__current_command), None))
        self.preferred_speed_text.setText(_translate("main_window", \
            str(self.__cruise_preffered_speed), None))

        cruise_distance = self.__cruise_watch_area * 5
        self.cruise_distance_text.setText(_translate("main_window", \
            str(cruise_distance) + ' cm', None))

        preffered_speed = CSQ_CRUISE_PREFFERED_SPEED +','+ str(self.__cruise_preffered_speed) + ';'
        speed = CSQ_CRUISE_SPEED +','+ str(self.__cruise_speed) + ';'
        speed = speed + preffered_speed
        cruise_distance = CSQ_CRUISE_DISTANCE + ',' + str(self.__cruise_watch_area) + ';'
        cruise_distance = cruise_distance + speed





        try:
            CAR_STATES_QUEUE.put(cruise_distance, False)
        except queue.Full:
            pass

    def close_event(self, event):
        """
        close event
        """
        self.update_frame_timer.stop()
        self.update_car_data_timer.stop()
        self.superviser_timer.stop()
        event.accept()


    def usage():
        usageStr = """
        Make sure to keep the pygame window in focus!\r
        
        Use the following keys to drive the robot:\r

        \tW:        Go forward\r
        \tS:        Go backward\r
        \tA:        Turn slightly left (while driving)\r
        \tD:        Turn slightly right (while driving)\r
        \tQ:        Rotate left\r
        \tE:        Rotate right\r

        \tM:        Drive mode\r
        \tN:        Toggle noise\r
        \tLeft:     Left indicator\r
        \tRight:    Right indicator\r
        \tUp:       Cancel indicators\r
        \tDown:     Network mode\r
        \tSPACE:    Toggle logging\r
        \tESC:      Quit\r
        """

        return usageStr


if __name__ == "__main__":
    MAIN_APP = QtWidgets.QApplication(sys.argv)
    MAIN_WINDOW = QtWidgets.QMainWindow()

    # PC IP address and port number
    # pc_ip = "192.168.137.1"  # Replace with your PC's IP address
    pc_ip = "192.168.85.251"
    pc_port = 1234  # Replace with your chosen port number


    if len(sys.argv) > 1:
        USER_INTERFACE = RemoteMain(str(sys.argv[1]))
        USER_INTERFACE.setup_ui(MAIN_WINDOW)
        MAIN_WINDOW.show()
        sys.exit(MAIN_APP.exec_())
    else:
        USER_INTERFACE = RemoteMain(pc_ip)
        USER_INTERFACE.setup_ui(MAIN_WINDOW)
        MAIN_WINDOW.show()
        sys.exit(MAIN_APP.exec_())
        # print ('No ip given')
