"""
Analyser module
"""
from collections import deque
import threading
import queue
import time
import cv2
import numpy as np
import numpy.matlib


import DetectChars
import DetectPlates
import RemoteMain as GLOBAL

SCALAR_BLACK = (0.0, 0.0, 0.0)
SCALAR_WHITE = (255.0, 255.0, 255.0)
SCALAR_YELLOW = (0.0, 255.0, 255.0)
SCALAR_GREEN = (0.0, 255.0, 0.0)
SCALAR_RED = (0.0, 0.0, 255.0)

showSteps = False

# Region-of-interest vertices
# We want a trapezoid shape, with bottom edge at the bottom of the image
TRAPEZOID_BOTTOM_WIDTH = 1.0
TRAPEZOID_TOP_WIDTH = 0.65
TRAPEZOID_HEIGHT = 0.7

# Hough Transform
HOUGH_DIST_RESOLUTION = 1 # distance resolution in pixels of the Hough grid
ANGULAR_RESOLUTION = 1 * np.pi/180 # angular resolution in radians of the Hough grid
HOUGH_THRESHOLD = 50 # minimum number of votes (intersections in Hough grid cell)
MIN_LINE_LENGHT = 70 #minimum number of pixels making up a line
MAX_LINE_GAP = 60	# maximum gap in pixels between connectable line segments

ALPHA = 0.8
BETA = 1.
GAMMA = 0.

AVD_START_AVOIDANCE = 'START_AVOIDANCE'
AVD_PREPARE_AVOIDANCE = "PREPARE_AVOIDANCE"
AVD_AVOID_OBJECT = 'AVOID_OBJECT'
AVD_GO_FORWARD = 'AVOID_GO_FORWARD'
AVD_RETURN_TO_LANE = 'RETURN_TO_LANE'
AVD_PREPARE_TO_FINISH = 'PREPARE_TO_FINISH'
AVD_FINISHED = 'FINISHED_AVD'

class Analyser(object):
    """
    Analyser class
    - responsible to analyse the current frame
    - detect lanes, cars, obstacles, road signs, etc
    - send the commands to SerialManager via Controller queue
    - send analysed frames to StreamServer via queue
    - update user rights about controlling the car
    """
    def __init__(self):
        self.__current_frame = None
        self.__encode_parameter = [int(cv2.IMWRITE_JPEG_QUALITY), 60]
        self.__command_timer = 0
        self.__fps_timer = 0

        self.__go_forward = False
        self.__go_left = False
        self.__go_right = False

        self.__fps_counter = 0
        self.__frame_fps = 0

        self.__lanes_coords_list = []

        self.__cruise_ndf_contor = 0
        self.__cruise_watch_area = 0
        self.__distance_to_car = 0
        self.__plate_coords = []
        self.__cruise_speed = 0
        self.__cruise_preffered_speed = 0

        # ------- Control variable ---------
        self._K_1 = 0.5 
        self._K_2 = 0.5
        self._h = 0.5
        self.L = 5


        self._current_speed_ego = 0
        self._current_speed_pre = 0
        self._current_acceleration_ego = 0 
        self._current_acceleration_pre = 0
        self.__distance_to_car = 0
        self._error_spacing_buffer = deque(maxlen=10)


        self.__car_states_timer = 0
        self.__car_data_updated = False

        self.__car_stopped = False

        self.__cruise_timer = 0
        self.__lane_assist_timer = 0

        #region Unknow input#


        # Ts = 1/100
        # t = np.arange(0, 20 + Ts, Ts)
        # Nt = len(t)
        # I3 = np.eye(3)

        # tau = 0.4
        # li = 5
        # Li = 7.3
        # h = 0.5
        # k1 = -0.8
        # k2 = 2.5
        # k3 = (1 - h * k1 * k2)
        # ai_1 = 3 * np.sin(2 * np.pi / 10 * t)

        # Ai = np.array([[0, 1, 0], [0, 0, 1], [0, 0, -1/tau]])
        # Bi = np.array([[0], [0], [1/tau]])
        # Ci = np.zeros((1, 3))
        # Di = np.zeros((1, 1))
        # Aid = np.eye(len(Ai)) + Ts * Ai
        # Bid = Ts * Bi
        # Cid = Ci
        # Did = Di

        # Ai_1 = np.array([[0, 1, 0], [0, 0, 0], [0, 0, 0]])
        # Bi_1 = np.array([[0], [1], [0]])
        # Ci_1 = np.zeros((1, 3))
        # Di_1 = np.zeros((1, 1))
        # Ai_1d = np.eye(len(Ai_1)) + Ts * Ai_1
        # Bi_1d = Ts * Bi_1
        # Ci_1d = Ci_1
        # Di_1d = Di_1

        # Ac = np.array([[0, 1, 0], [0, 0, 1], [-k2 / (tau * h), -k3 / (tau * h), (k1 - k3) / tau]])
        # Bc = np.array([[0], [0], [k2 / tau]])
        # Fc = np.array([[0], [0], [k3 / tau]])
        # Wc = np.array([[0], [0], [(k3 - k1) / tau]])
        # deltac = np.array([[0], [0], [k2 * (Li - li) / (tau * h)]])
        # Cc = np.array([[1, 0, 0], [0, 1, 0]])

        # Ad = np.eye(len(Ac)) + Ts * Ac
        # Bd = Ts * Bc
        # Fd = Ts * Fc
        # Wd = Ts * Wc
        # deltad = Ts * deltac
        # Cd = Cc



        # w(:,1) = [14;-5;7;10]';
        # w(:,2) = w(:,1);
        # w(:,3) = w(:,1);
        # xp(:,1) = x_p0;
        # xf(:,1) = x_f0;
        # psi(:,1) = psi0;
        # ksi_hat(:,1) = w(:,1)+Qz*Cd*psi(:,1);
        # ksi_hat(:,2) = ksi_hat(:,1);
        # ksi_hat(:,3) = ksi_hat(:,1)


        self._dt = 0.02 

        #endregion Unknow input

        # Not use yet 
        # object avoidance data
        self.__dp = 1
        self.__min_dist = 50
        self.__circle_param_1 = 150
        self.__circle_param_2 = 50
        self.__objects_coords_list = []
        self.__avoiding_activated = False

        self.__avoidance_go_forward = False
        self.__avoidance_go_left = False
        self.__avoidance_go_right = False
        self.__avoidance_brake = False

        self.__avoidance_timer = 0
        self.__returner_timer = 0
        self.__avoidance_state = ''

    def analyse_pre(self, frame_queue, analysed_frame_queue, \
    commands_queue, car_states_queue , acceleration_queue , distance_vel_queue):
        """
        Get the current frame from FRAME_QUEUE and analyse 
        then put it in analysed_frame_queue to draw in the window
        Put the decision to Command_Queue to send to the car 
        """
        current_thread = threading.currentThread()
        self.__command_timer = time.time()
        #--------------- HUY Not use yet

        # bln_knn_training_successful = DetectChars.loadKNNDataAndTrainKNN() # attempt KNN training
        # if bool(bln_knn_training_successful) is False:
        #     return

        self.__fps_timer = time.time()
        while getattr(current_thread, 'is_running', True):


            self.__current_frame = frame_queue.get(True, None)

            # HUY : Uncomment for test first 
            # self.__get_cruise_states_data(car_states_queue,acceleration_queue,distance_vel_queue)
            self.__get_Car_data(True,acceleration_queue,distance_vel_queue)


            # if getattr(current_thread, 'is_analysing', True):
            #     # self.__car_detection(autonomous_states_queue)
            #     # self.__detect_objects()
            #     self.__lane_assist()

            #     if getattr(current_thread, 'is_deciding', True):
            #         self.__take_cruise_decision(commands_queue)
            #         # self.__avoid_detected_objects(commands_queue)

            #     # self.__draw_rect_around_plate(self.__current_frame)
            #     self.__draw_distance_to_car()
            #     # self.__draw_car_cruise_watch_area()
            #     self.__draw_lane_assist_decision()
            #     # self.__draw_detected_objects()

            self.__draw_fps()

            # # result, encrypted_image = \
            # #     cv2.imencode('.jpg', self.__current_frame, self.__encode_parameter)

            # # if bool(result) is False:
            # #     break


            analysed_frame_queue.put(self.__current_frame, True, None)

            frame_queue.task_done()

            self.__fps_counter = self.__fps_counter + 1

            if time.time() - self.__fps_timer > 1:
                self.__frame_fps = self.__fps_counter
                self.__fps_counter = 0
                self.__fps_timer = time.time()


    def analyse_follow(self, frame_queue, analysed_frame_queue, \
    commands_queue, car_states_queue , acceleration_queue ,distance_vel_queue,dist_vel_accel_CAR_Pre_QUEUE):
        """
        Get the current frame from FRAME_QUEUE and analyse 
        then put it in analysed_frame_queue to draw in the window
        Put the decision to Command_Queue to send to the car 
        """
        current_thread = threading.currentThread()
        self.__command_timer = time.time()
        #--------------- HUY Not use yet

        # bln_knn_training_successful = DetectChars.loadKNNDataAndTrainKNN() # attempt KNN training
        # if bool(bln_knn_training_successful) is False:
        #     return

        self.__fps_timer = time.time()
        while getattr(current_thread, 'is_running', True):
            # string_data = frame_queue.get(True, None)
            # frame = numpy.fromstring(string_data, dtype='uint8')
            # self.__current_frame = cv2.imdecode(frame, 1)

            self.__current_frame = frame_queue.get(True, None)

            # HUY : Uncomment for test first 
            self.__get_Car_data(True,acceleration_queue,distance_vel_queue,dist_vel_accel_CAR_Pre_QUEUE)


            if getattr(current_thread, 'is_analysing', True):
                # self.__car_detection(autonomous_states_queue)
                # self.__detect_objects()
                # self.__lane_assist()
                pass

                if getattr(current_thread, 'is_deciding', True):
                    # self.__CO_apdativeCC(self, commands_queue,acceleration_queue,distance_vel_queue,dist_vel_accel_CAR_Pre_QUEUE)
                    # self.__take_cruise_decision(commands_queue)
                    # self.__avoid_detected_objects(commands_queue)
                    pass

                # self.__draw_rect_around_plate(self.__current_frame)
                # self.__draw_distance_to_car()
                # self.__draw_car_cruise_watch_area()
                # self.__draw_lane_assist_decision()
                # self.__draw_detected_objects()

            self.__draw_fps()

            # result, encrypted_image = \
            #     cv2.imencode('.jpg', self.__current_frame, self.__encode_parameter)

            # if bool(result) is False:
            #     break

            # analysed_frame = numpy.array(encrypted_image)
            # analysed_frame_queue.put(str(analysed_frame.tostring()), True, None)
            analysed_frame_queue.put(self.__current_frame, True, None)

            frame_queue.task_done()

            self.__fps_counter = self.__fps_counter + 1

            if time.time() - self.__fps_timer > 1:
                self.__frame_fps = self.__fps_counter
                self.__fps_counter = 0
                self.__fps_timer = time.time()


    def CO_apdativeCC(self, commands_queue, acceleration_queue, distance_vel_queue, dist_vel_accel_CAR_Pre_QUEUE):
        self._K_1 = 0.5
        self._K_2 = 0.5
        self._h = 0.5
        self.L_des = 15 #cm

        current_thread = threading.currentThread()
        while getattr(current_thread, 'is_running', True):

            self.__get_Car_data(True , acceleration_queue,distance_vel_queue,dist_vel_accel_CAR_Pre_QUEUE)

            # dist_padding = 10
            # low_thresh_area = self.__cruise_watch_area * 5
            # high_thresh_area = self.__cruise_watch_area * 5 + dist_padding

            # if self.__distance_to_car < self.L_des:
            #     try:
            #         commands_queue.put(GLOBAL.CMD_BRAKE, False)
            #     except queue.Queue.Full:
            #         return
            # else:
            spacing_error = self.__distance_to_car + self.L_des

            
            self._error_spacing_buffer.append(spacing_error)
            if len(self._error_spacing_buffer) >= 2:
                spacing_error_dot = (self._error_spacing_buffer[-1] - self._error_spacing_buffer[-2]) / self._dt
                if (spacing_error_dot != 0):
                    velocity_error =   self._current_speed_ego - self._current_speed_pre
                    spacing_error_dot = (spacing_error_dot + velocity_error)/2
            else :
                spacing_error_dot = 0
            acceleration = (-self._K_1 * self._current_acceleration_pre) + (self._K_1 + self._h*self._K_1*self._K_2)*self._current_acceleration_ego - \
                            (1/self._h)*(1 - self._h*self._K_1*self._K_2)*spacing_error_dot - (self._K_2/self._h)*spacing_error - self._K_2*self._current_speed_ego
            
            # print("accel : " ,acceleration)

            time.sleep(self._dt)

    def __draw_fps(self):
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(self.__current_frame, str(self.__frame_fps), \
            (0, 21), font, 1, (0, 255, 255), 2, cv2.LINE_AA)

    def __get_Car_data(self, isfollowing , acceleration_queue ,distance_vel_queue,dist_vel_accel_CAR_Pre_QUEUE = 0):
        if time.time() - self.__car_states_timer > self._dt:
            if (isfollowing):
                try:
                    # car_states = car_states_queue.get(False)
                    acceleration_ego = acceleration_queue.get(False)
                    distance_velo_car_ego = distance_vel_queue.get(False)
                    distance_velo_accel_car_pre = dist_vel_accel_CAR_Pre_QUEUE.get(False)
                except queue.Empty:
                    if True:
                        pass
                    return 

                # fowlling
                if distance_velo_accel_car_pre:
                    distance_velo_accel_car_pre = distance_velo_accel_car_pre.split(';')
                    # print(distance_velo_car_ego)
                    for elem in distance_velo_accel_car_pre:
                        current_pre_velo_accel = elem.split(':')
                        if len(current_pre_velo_accel) > 1:
                            if GLOBAL.CSQ_SPEED in current_pre_velo_accel[0]:
                                # print("speed" + str(float(current_pre_velo_accel[1])))
                                self._current_speed_pre = float(current_pre_velo_accel[1])
                            elif GLOBAL.CSQ_ACCEL in current_pre_velo_accel[0]:
                                # print("accel" + str(float(current_pre_velo_accel[1])))
                                self._current_acceleration_pre = float(current_pre_velo_accel[1])
                            # elif GLOBAL.CSQ_DISTANCE in current_pre_velo_accel[0]:
                            #     # print("distance" + str(float(current_pre_velo_accel[1])))
                            #     self.__distance_to_car = float(current_pre_velo_accel[1])
                dist_vel_accel_CAR_Pre_QUEUE.task_done()

            # is leader
            else:
                try:
                    # car_states = car_states_queue.get(False)
                    acceleration_ego = acceleration_queue.get(False)
                    distance_velo_car_ego = distance_vel_queue.get(False)
                except queue.Empty:
                    return 
                

            # Common data can get from sensor car
            if acceleration_ego:
                self._current_acceleration_ego = float(acceleration_ego) # linear acceleration in x aixs ? 
            if distance_velo_car_ego:
                distance_velo_car_ego = distance_velo_car_ego.split(';')
                for elem in distance_velo_car_ego:
                    current_state = elem.split(':')
                    if len(current_state) > 1:
                        if GLOBAL.CSQ_CRUISE_DISTANCE in current_state[0]:
                            self.__distance_to_car = float(current_state[1])
                        elif GLOBAL.CSQ_CRUISE_SPEED in current_state[0]:
                            self._current_speed_ego = float(current_state[1])

            # car_states_queue.task_done()
            acceleration_queue.task_done()
            distance_vel_queue.task_done()

            
            self.__car_states_timer = time.time()

    

    def __draw_rect_around_plate(self, current_scene):
        if len(self.__plate_coords) > 3:
            if self.__distance_to_car < 100:
                cv2.line(current_scene, tuple(self.__plate_coords[0]), \
                    tuple(self.__plate_coords[1]), SCALAR_RED, 2)
                cv2.line(current_scene, tuple(self.__plate_coords[1]), \
                    tuple(self.__plate_coords[2]), SCALAR_RED, 2)
                cv2.line(current_scene, tuple(self.__plate_coords[2]), \
                    tuple(self.__plate_coords[3]), SCALAR_RED, 2)
                cv2.line(current_scene, tuple(self.__plate_coords[3]), \
                    tuple(self.__plate_coords[0]), SCALAR_RED, 2)

    def __draw_distance_to_car(self):
        if self.__distance_to_car < 100:
            frame_shape = self.__current_frame.shape
            distance_position = (2 * frame_shape[1] / 100, 95 * frame_shape[0] / 100)
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(self.__current_frame, 'Distance:' + str(self.__distance_to_car) + 'cm', \
                distance_position, font, 1, (0, 255, 255), 2, cv2.LINE_AA)

    def __take_cruise_decision(self, commands_queue):
        if bool(self.__avoiding_activated) is True:
            return
        
        dist_padding = 10
        low_thresh_area = self.__cruise_watch_area * 5
        high_thresh_area = self.__cruise_watch_area * 5 + dist_padding
        
        if self.__distance_to_car < high_thresh_area:
            try:
                commands_queue.put(GLOBAL.CMD_BRAKE, False)
            except Queue.Full:
                return
        else:
            if self.__cruise_preffered_speed == 0:
                try:
                    commands_queue.put(GLOBAL.CMD_BRAKE, False)
                except Queue.Full:
                    return
            if self.__cruise_speed > self.__cruise_preffered_speed:
                if time.time() - self.__cruise_timer > (200.0 / 1000.0):
                    try:
                        commands_queue.put(GLOBAL.CMD_DECREASE_SPEED, False)
                    except Queue.Full:
                        return
                    self.__cruise_timer = time.time()

            # scenarious when you have to increase the speed
            if self.__distance_to_car > (high_thresh_area + dist_padding):
                if self.__cruise_speed < self.__cruise_preffered_speed:
                    if time.time() - self.__cruise_timer > (200.0 / 1000.0):
                        if bool(self.__car_data_updated) is True:
                            try:
                                commands_queue.put(GLOBAL.CMD_INCREASE_SPEED, False)
                            except Queue.Full:
                                return
                            self.__cruise_timer = time.time()
                            self.__car_data_updated = False


 

    # def __UIO_observer(self, commands_queue,acceleration_queue,distance_vel_queue,dist_vel_accel_CAR_Pre_QUEUE):
    #     for i in range(len(t)):
    #           # acceleration here 
            
    #         if t[i] < 4:
    #             fc[i] = 0
    #         elif 4 <= t[i] < 8:
    #             fc[i] = 10 * np.sin(10 * np.pi / 2 * t[i]) * np.exp(-0.01 * t[i])
    #         elif 9 <= t[i] < 16:
    #             fc[i] = np.exp(-2 * np.sin(10 * np.pi / 2 * t[i]) * 0.01 * t[i])
    #         else:
    #             fc[i] = 0
            
    #         mu[i] = self._current_acceleration_pre + fc[i]

    #     # Vectorized updates
    #     xp[:, 1:] = np.dot(Ai_1d, xp[:, :-1]) + np.outer(Bi_1d.flatten(), ai_1)
    #     psi[:, 1:] = np.dot(Ad, psi[:, :-1]) + np.outer(Bd.flatten(), xf[1, :-1]) + Fd * mu - Wd * fc + deltad
    #     y = np.dot(Cd, psi[:, :-1])

    #     ksi_hat[:, :2] = w[:, :2] + np.dot(Qz, y[:, :2])

            # Unknow input update loop
    #     for i in range(2, len(t)):
    #         w[:, i] = np.dot(N, w[:, i-1]) + np.dot(L, y[:, i-1]) + M * xf[1, i-2] + G * mu[i-2] + np.dot(Pz, deltae)
    #         ksi_hat[:, i] = w[:, i] + np.dot(Qz, y[:, i])

            # u syn 
    #     usync = h * k1 * k2 * mu - k2 * xf[1] - (k1 + h * k1 * k2) * (xp[2] - xf[2]) + \
    #             k3 / h * (xp[1] - xf[1]) + k2 / h * (xp[0] - xf[0] - li) - k2 / h * (Li - li) - \
    #             (k1 + h * k1 * k2) * np.sign(ksi_hat[3]) * np.minimum(np.abs(ksi_hat[3]), 20)

    #     xf[:, 1:] = np.dot(Aid, xf[:, :-1]) + np.outer(Bid.flatten(), usync)
        
        

    # def __apdativeCC(self, commands_queue):
    #     self._K_1 = 
    #     self._K_2 = 
    #     self._h = 


    #     self._current_speed_ego =
    #     self._current_acceleration_ego = 
    #     self._current_acceleration_preceding =
    #     self.__distance_to_car = 
    #     spacing_error = self.__distance_to_car - L

        
    #     self._error_spacing_buffer.append(spacing_error)
    #     if len(self._error_spacing_buffer) >= 2:
    #         spacing_error_dot = (self._error_spacing_buffer[-1] - self._error_spacing_buffer[-2]) / self._dt
    #     else :
    #         spacing_error_dot = 0
        

    #     acceleration = (-self._K_1 * self._current_acceleration_preceding) + (self._K_1 + self._h*self._K_1*self._K_2)*self._current_acceleration_ego - (1/self._h)*(1 - self._h*self._K_1*self._K_2)*spacing_error_dot - (self._K_2/self._h)*spacing_error - self._K_2*self._current_speed_ego
    #     print(-acceleration)



    def __populate_autonomous_states(self, autonomous_states_queue):
        pass

    def __car_detection(self, autonomous_states_queue):
        """
        Detect the possible car in front of our car
        Store useful data states
        """
        list_of_possible_plates = DetectPlates.detectPlatesInScene(self.__current_frame)
        list_of_possible_plates = DetectChars.detectCharsInPlates(list_of_possible_plates)

        list_of_possible_plates.sort(key=lambda possiblePlate: len(possiblePlate.strChars), \
            reverse=True)

        if len(list_of_possible_plates) > 0:
            #at least one car
            lic_plate = list_of_possible_plates[0]
            frame_shape = self.__current_frame.shape
            self.__plate_coords = cv2.boxPoints(lic_plate.rrLocationOfPlateInScene)
            self.__distance_to_car = frame_shape[0] - self.__plate_coords[3][1] # in pixels
            self.__distance_to_car = self.__distance_to_car
            self.__distance_to_car = float("{0:.2f}".format(self.__distance_to_car))
            self.__cruise_ndf_contor = 0
        else:
            # make sure that the algoritm doesn't fail for a specific frame
            self.__cruise_ndf_contor = self.__cruise_ndf_contor + 1
            if self.__cruise_ndf_contor > 5:
                self.__distance_to_car = 1000
                self.__cruise_ndf_contor = 0

    def __gaussian_blur(self, img, kernel_size=3):
        """Applies a Gaussian Noise kernel"""
        return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)

    def __grayscale(self, img):
        return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    def __canny(self, image, sigma=0.33):
        """Applies the Canny transform"""
        median_variable = np.median(image)

        lower = int(max(0, (1.0 - sigma) * median_variable))
        upper = int(min(255, (1.0 + sigma) * median_variable))
        self.__circle_param_1 = upper
        edged = cv2.Canny(image, lower, upper)

        return edged

    def __region_of_interest(self, img, vertices):

        mask = np.zeros_like(img)

        if len(img.shape) > 2:
            channel_count = img.shape[2]
            ignore_mask_color = (255,) * channel_count
        else:
            ignore_mask_color = 255

        cv2.fillPoly(mask, vertices, ignore_mask_color)

        masked_image = cv2.bitwise_and(img, mask)
        return masked_image

    def __draw_detected_lanes(self, img, lines, color=None, thickness=5):
        if color is None:
            color = [255, 0, 0]
        if lines is None:
            return
        if len(lines) == 0:
            return
        draw_right = True
        draw_left = True

        slope_threshold = 0.5
        slopes = []
        new_lines = []
        for line in lines:
            x1, y1, x2, y2 = line[0]  # line = [[x1, y1, x2, y2]]

            if x2 - x1 == 0.:  # corner case, avoiding division by 0
                slope = 999.  # practically infinite slope
            else:
                slope = (y2 - y1) / (x2 - x1)

            if abs(slope) > slope_threshold:
                slopes.append(slope)
                new_lines.append(line)

        lines = new_lines

        right_lines = []
        left_lines = []
        for i, line in enumerate(lines):
            x1, y1, x2, y2 = line[0]
            img_x_center = img.shape[1] / 2  # x coordinate of center of image
            if slopes[i] > 0 and x1 > img_x_center and x2 > img_x_center:
                right_lines.append(line)
            elif slopes[i] < 0 and x1 < img_x_center and x2 < img_x_center:
                left_lines.append(line)

        right_lines_x = []
        right_lines_y = []

        for line in right_lines:
            x1, y1, x2, y2 = line[0]

            right_lines_x.append(x1)
            right_lines_x.append(x2)

            right_lines_y.append(y1)
            right_lines_y.append(y2)

        if len(right_lines_x) > 0:
            right_m, right_b = np.polyfit(right_lines_x, right_lines_y, 1)  # y = m*x + b
        else:
            right_m, right_b = 1, 1
            draw_right = False

        left_lines_x = []
        left_lines_y = []

        for line in left_lines:
            x1, y1, x2, y2 = line[0]

            left_lines_x.append(x1)
            left_lines_x.append(x2)

            left_lines_y.append(y1)
            left_lines_y.append(y2)

        if len(left_lines_x) > 0:
            left_m, left_b = np.polyfit(left_lines_x, left_lines_y, 1)  # y = m*x + b
        else:
            left_m, left_b = 1, 1
            draw_left = False

        y1 = img.shape[0]
        y2 = img.shape[0] * (1 - TRAPEZOID_HEIGHT)

        right_x1 = (y1 - right_b) / right_m
        right_x2 = (y2 - right_b) / right_m

        left_x1 = (y1 - left_b) / left_m
        left_x2 = (y2 - left_b) / left_m

        y1 = int(y1)
        y2 = int(y2)
        right_x1 = int(right_x1)
        right_x2 = int(right_x2)
        left_x1 = int(left_x1)
        left_x2 = int(left_x2)

        self.__lanes_coords_list = []

        if draw_left:
            if left_x1 < img.shape[1] and left_x2 < img.shape[1]:
                if left_x1 > 0  and left_x2 > 0:
                    cv2.line(img, (left_x1, y1), (left_x2, y2), color, thickness)
                    self.__lanes_coords_list.append((left_x1, y1))
                    self.__lanes_coords_list.append((left_x2, y2))

        if draw_right:
            if right_x1 < img.shape[1] and right_x2 < img.shape[1]:
                if right_x1 > 0 and right_x2 > 0:
                    cv2.line(img, (right_x1, y1), (right_x2, y2), color, thickness)
                    self.__lanes_coords_list.append((right_x1, y1))
                    self.__lanes_coords_list.append((right_x2, y2))

    def __hough_lines(self, img):
        lines = cv2.HoughLinesP(img, HOUGH_DIST_RESOLUTION, ANGULAR_RESOLUTION, \
            HOUGH_THRESHOLD, np.array([]), minLineLength=MIN_LINE_LENGHT, maxLineGap=MAX_LINE_GAP)

        (height, width) = img.shape
        datatype = np.dtype(np.uint8)
        line_img = np.zeros((height, width, 3), datatype)
        self.__draw_detected_lanes(line_img, lines)

        return line_img

    def __draw_car_cruise_watch_area(self):
        if self.__cruise_watch_area == 0:
            return

        height, width, channels = self.__current_frame.shape

        tile_bottom_x_coord = width / 2
        tile_bottom_y_coord = height
        tile_width = 25 * width / 100
        tile_height = 25
        tile_padding = 5

        for contor in range(0, self.__cruise_watch_area):
            car_cruise_area_vertex = []

            car_cruise_area_vertex.append([tile_bottom_x_coord + tile_width, tile_bottom_y_coord])
            car_cruise_area_vertex.append([tile_bottom_x_coord - tile_width, tile_bottom_y_coord])

            tile_bottom_y_coord = tile_bottom_y_coord - tile_height
            tile_width = 80 * tile_width / 100

            car_cruise_area_vertex.append([tile_bottom_x_coord - tile_width, tile_bottom_y_coord])
            car_cruise_area_vertex.append([tile_bottom_x_coord + tile_width, tile_bottom_y_coord])

            overlay = self.__current_frame.copy()
            pts = np.array(car_cruise_area_vertex, np.int32)
            pts = pts.reshape((-1, 1, 2))
            cv2.fillConvexPoly(overlay, pts, (0, 200, 0))
            alpha = 0.5
            cv2.addWeighted(overlay, alpha, self.__current_frame, 1-alpha, 0, self.__current_frame)

            tile_bottom_y_coord = tile_bottom_y_coord - tile_padding

    def __lane_assist(self):
        current_frame = self.__grayscale(self.__current_frame)
        current_frame = self.__gaussian_blur(current_frame)

        current_frame = self.__canny(current_frame)

        imshape = self.__current_frame.shape
        vertices = np.array([[\
		((imshape[1] * (1 - TRAPEZOID_BOTTOM_WIDTH)) // 2, imshape[0]),\
		((imshape[1] * (1 - TRAPEZOID_TOP_WIDTH)) // 2, imshape[0] - imshape[0] * TRAPEZOID_HEIGHT),\
		(imshape[1] - (imshape[1] * (1 - TRAPEZOID_TOP_WIDTH)) // 2, \
            imshape[0] - imshape[0] * TRAPEZOID_HEIGHT),\
		(imshape[1] - (imshape[1] * (1 - TRAPEZOID_BOTTOM_WIDTH)) // 2, imshape[0])]], dtype=np.int32)

        current_frame = self.__region_of_interest(current_frame, vertices)

        current_frame = self.__hough_lines(current_frame)

        final_image = self.__current_frame.astype('uint8')

        cv2.addWeighted(self.__current_frame, ALPHA, current_frame, BETA, GAMMA, final_image)

        final_image = final_image.astype('uint8')

        self.__current_frame = final_image

    def __maintain_between_lanes(self, commands_queue):
        if self.__cruise_preffered_speed == 0 or self.__cruise_speed == 0:
            self.__car_stopped = True
        else:
            self.__car_stopped = False

        if bool(self.__car_stopped) is True:
            return

        self.__go_forward = False
        self.__go_left = False
        self.__go_right = False

        frame_shape = self.__current_frame.shape
        car_head = (frame_shape[1] / 2, 0)
        distance_padding = 10 * frame_shape[1] / 100

        if len(self.__lanes_coords_list) > 3:
            # both lanes detected
            if car_head[0] - distance_padding > self.__lanes_coords_list[1][0]:
                if car_head[0] + distance_padding < self.__lanes_coords_list[3][0]:
                    if time.time() - self.__lane_assist_timer > 200.0/1000.0:
                        print ('[Both] GO FORWARD')
                        try:
                            commands_queue.put(GLOBAL.CMD_GO_FORWARD, False)
                        except Queue.Full:
                            pass
                        self.__lane_assist_timer = time.time()
                        self.__go_forward = True

            if car_head[0] - distance_padding < self.__lanes_coords_list[1][0]:
                if time.time() - self.__lane_assist_timer > 200.0/1000.0:
                    print ('[Both] GO RIGHT')
                    try:
                        commands_queue.put(GLOBAL.CMD_GO_RIGHT, False)
                    except Queue.Full:
                        pass
                    self.__lane_assist_timer = time.time()
                    self.__go_right = True

            if car_head[0] + distance_padding > self.__lanes_coords_list[3][0]:
                if time.time() - self.__lane_assist_timer > 200.0/1000.0:
                    print ('[Both] GO LEFT')
                    try:
                        commands_queue.put(GLOBAL.CMD_GO_LEFT, False)
                    except Queue.Full:
                        pass
                    self.__lane_assist_timer = time.time()
                    self.__go_left = True
        else:
            print ('[None] BRAKE')
            try:
                commands_queue.put(GLOBAL.CMD_BRAKE, False)
            except Queue.Full:
                pass

    def __draw_lane_assist_decision(self):
        (height, width, channels) = self.__current_frame.shape
        padding = 10

        arrow_vertex_list = []

        if bool(self.__go_left) is True:
            arrow_vertex_list.append([padding, height/2])
            arrow_vertex_list.append([padding * 3, height/3])
            arrow_vertex_list.append([padding * 3, 3 * height/4])
        elif bool(self.__go_right) is True:
            arrow_vertex_list.append([width - padding, height/2])
            arrow_vertex_list.append([width - padding * 3, height/3])
            arrow_vertex_list.append([width - padding * 3, 3 * height/4])
        elif bool(self.__go_forward) is True:
            arrow_vertex_list.append([width/2, padding])
            arrow_vertex_list.append([width/2 - padding * 3, padding * 2])
            arrow_vertex_list.append([width/2 + padding * 3, padding * 2])

        overlay = self.__current_frame.copy()
        pts = np.array(arrow_vertex_list, np.int32)
        pts = pts.reshape((-1, 1, 2))
        cv2.fillConvexPoly(overlay, pts, (0, 200, 200))
        alpha = 0.5
        cv2.addWeighted(overlay, alpha, self.__current_frame, 1-alpha, 0, self.__current_frame)

    def __detect_objects(self):
        frame = self.__current_frame
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur_grey = self.__gaussian_blur(gray)

        circles = cv2.HoughCircles(blur_grey, cv2.HOUGH_GRADIENT, dp=self.__dp, \
        minDist=self.__min_dist, param1=self.__circle_param_1, param2=self.__circle_param_2, \
        minRadius=0, maxRadius=0)

        self.__objects_coords_list = []

        # ensure at least some circles were found
        if circles is not None:
            # convert the (x, y) coordinates and radius of the circles to integers
            circles = np.round(circles[0, :]).astype("int")
            # loop over the (x, y) coordinates and radius of the circles
            for element in circles:
                if element[1] > (3 * frame_shape[0] / 5):
                    self.__objects_coords_list.append(element)

    def __avoid_detected_objects(self, commands_queue):

        if bool(self.__avoiding_activated) is False:
            frame_shape = self.__current_frame.shape
            go_left = True
            go_right = True

            for elem in self.__objects_coords_list:
                if elem[0] < frame_shape[1] / 2:
                    go_left = False
                else:
                    go_right = False

            if bool(go_left) is True:
                self.__avoiding_activated = True
                self.__avoidance_go_left = True
            elif bool(go_right) is True:
                self.__avoiding_activated = True
                self.__avoidance_go_right = True
            else:
                self.__avoiding_activated = True
                self.__avoidance_brake = True

        if bool(self.__avoiding_activated) is True:
            if bool(self.__avoidance_brake) is True:
                try:
                    commands_queue.put(GLOBAL.CMD_BRAKE, False)
                    self.__avoiding_activated = False
                    self.__avoidance_brake = False
                except Queue.Full:
                    self.__avoiding_activated = True
                    self.__avoidance_brake = True
            elif self.__avoidance_state == '':
                if bool(self.__avoidance_go_left) is True:
                    self.__avoidance_state = AVD_START_AVOIDANCE
                elif bool(self.__avoidance_go_right) is True:
                    self.__avoidance_state = AVD_START_AVOIDANCE

            #P1 - start avoidance (change direction)
            elif self.__avoidance_state == AVD_START_AVOIDANCE:
                if bool(self.__avoidance_go_left) is True:
                    try:
                        commands_queue.put(GLOBAL.CMD_GO_LEFT, False)
                        self.__avoidance_state = AVD_PREPARE_AVOIDANCE
                    except Queue.Full:
                        self.__avoidance_state = AVD_START_AVOIDANCE
                elif bool(self.__avoidance_go_right) is True:
                    try:
                        commands_queue.put(GLOBAL.CMD_GO_RIGHT, False)
                        self.__avoidance_state = AVD_PREPARE_AVOIDANCE
                    except Queue.Full:
                        self.__avoidance_state = AVD_START_AVOIDANCE
                self.__avoidance_timer = time.time()

            #P2 - prepare avoidance
            elif self.__avoidance_state == AVD_PREPARE_AVOIDANCE:
                if time.time() - self.__avoidance_timer > 0.9:
                    if bool(self.__avoidance_go_left) is True:
                        try:
                            commands_queue.put(GLOBAL.CMD_GO_RIGHT, False)
                            self.__avoidance_state = AVD_AVOID_OBJECT
                        except Queue.Full:
                            self.__avoidance_state = AVD_PREPARE_AVOIDANCE
                    elif bool(self.__avoidance_go_right) is True:
                        try:
                            commands_queue.put(GLOBAL.CMD_GO_LEFT, False)
                            self.__avoidance_state = AVD_AVOID_OBJECT
                        except Queue.Full:
                            self.__avoidance_state = AVD_PREPARE_AVOIDANCE
                    self.__avoidance_timer = time.time()

            #P3 - start avoid object
            elif self.__avoidance_state == AVD_AVOID_OBJECT:
                if time.time() - self.__avoidance_timer > 1.1:
                    if bool(self.__avoidance_go_left) is True:
                        try:
                            commands_queue.put(GLOBAL.CMD_GO_FORWARD, False)
                            self.__avoidance_state = AVD_GO_FORWARD
                        except Queue.Full:
                            self.__avoidance_state = AVD_AVOID_OBJECT
                    elif bool(self.__avoidance_go_right) is True:
                        try:
                            commands_queue.put(GLOBAL.CMD_GO_FORWARD, False)
                            self.__avoidance_state = AVD_GO_FORWARD
                        except Queue.Full:
                            self.__avoidance_state = AVD_AVOID_OBJECT
                    self.__avoidance_timer = time.time()

            #P3 BIS - avoiding object
            elif self.__avoidance_state == AVD_GO_FORWARD:
                if time.time() - self.__avoidance_timer > 0.7:
                    if bool(self.__avoidance_go_left) is True:
                        try:
                            commands_queue.put(GLOBAL.CMD_INCREASE_SPEED, False)
                            self.__avoidance_state = AVD_RETURN_TO_LANE
                        except Queue.Full:
                            self.__avoidance_state = AVD_GO_FORWARD
                    elif bool(self.__avoidance_go_right) is True:
                        try:
                            commands_queue.put(GLOBAL.CMD_INCREASE_SPEED, False)
                            self.__avoidance_state = AVD_RETURN_TO_LANE
                        except Queue.Full:
                            self.__avoidance_state = AVD_GO_FORWARD
                    self.__avoidance_timer = time.time()

            #P4 - return to lane
            elif self.__avoidance_state == AVD_RETURN_TO_LANE:
                if time.time() - self.__avoidance_timer > 1:
                    if bool(self.__avoidance_go_left) is True:
                        try:
                            commands_queue.put(GLOBAL.CMD_GO_RIGHT, False)
                            self.__avoidance_state = AVD_PREPARE_TO_FINISH
                        except Queue.Full:
                            self.__avoidance_state = AVD_RETURN_TO_LANE
                    elif bool(self.__avoidance_go_right) is True:
                        if time.time() - self.__avoidance_timer > 1:
                            try:
                                commands_queue.put(GLOBAL.CMD_GO_LEFT, False)
                                self.__avoidance_state = AVD_PREPARE_TO_FINISH
                            except Queue.Full:
                                self.__avoidance_state = AVD_RETURN_TO_LANE
                    self.__avoidance_timer = time.time()

            #P5 - prepare to finish
            elif self.__avoidance_state == AVD_PREPARE_TO_FINISH:
                if time.time() - self.__avoidance_timer > 0.8:
                    if bool(self.__avoidance_go_left) is True:
                        try:
                            commands_queue.put(GLOBAL.CMD_GO_LEFT, False)
                            self.__avoidance_state = AVD_FINISHED
                        except Queue.Full:
                            self.__avoidance_state = AVD_PREPARE_TO_FINISH
                    elif bool(self.__avoidance_go_right) is True:
                        try:
                            commands_queue.put(GLOBAL.CMD_GO_RIGHT, False)
                            self.__avoidance_state = AVD_FINISHED
                        except Queue.Full:
                            self.__avoidance_state = AVD_PREPARE_TO_FINISH
                    self.__avoidance_timer = time.time()

            #P6 - finish
            elif self.__avoidance_state == AVD_FINISHED:
                self.__avoiding_activated = False
                self.__avoidance_go_forward = True
                self.__avoidance_go_left = False
                self.__avoidance_go_right = False
                self.__avoidance_state = ''

        if bool(self.__avoidance_go_forward) is True:
            if time.time() - self.__avoidance_timer > 1:
                try:
                    commands_queue.put(GLOBAL.CMD_GO_FORWARD, False)
                    self.__avoidance_go_forward = False
                except Queue.Full:
                    self.__avoidance_go_forward = True

    def __draw_detected_objects(self):
        for element in self.__objects_coords_list:
            cv2.circle(self.__current_frame, (element[0], element[1]), element[2], (0, 255, 0), 4)
        self.__objects_coords_list = []

