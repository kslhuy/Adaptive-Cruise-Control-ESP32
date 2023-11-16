import numpy as np

class Utils:
    def __init__(self):
        # DEFAUT CAR INFOMARTION
        self.MAX_SPEED = 250
        self.MIN_SPEED = 100
        self.START_SPEED = 150


        self.SPEED_UP = 10

        self.SPEED_DOWN = 10

        self.STEER_PULSE = 0.2
        self.STEER_SEUIL_TO_Center = 0.2
        self.STEER_decay = 0.01

        self._throttle = 100
        self._steer_cache = 0
        self.steer = 0
        # DEFAUT CAR INFOMARTION


        self.steer_min = -1
        self.steer_max = 1
        self.steer_degree_min = 65
        self.steer_degree_max = 115
        # self.steer_degree_calibrated_min = 20
        # self.steer_degree_calibrated_max = 140

    def map_value_Steer_ESP2(self, x):
        pwm_calibrated_min = 92 
        pwm_calibrated_max = 488

        steer_degree_calibrated_min = 0
        steer_degree_calibrated_max = 132

        mapped_value_1 = np.interp(x, (self.steer_min, self.steer_max), (self.steer_degree_min, self.steer_degree_max))
        mapped_value_2 = np.interp(mapped_value_1, (steer_degree_calibrated_min, steer_degree_calibrated_max), (pwm_calibrated_min, pwm_calibrated_max))

        return mapped_value_2

    def map_value_Steer_ESP1(self, x):
        pwm_calibrated_min = 138
        pwm_calibrated_max = 482

        steer_degree_calibrated_min = 20
        steer_degree_calibrated_max = 140

        mapped_value_1 = np.interp(x, (self.steer_min, self.steer_max), (self.steer_degree_min, self.steer_degree_max))
        mapped_value_2 = np.interp(mapped_value_1, (steer_degree_calibrated_min, steer_degree_calibrated_max), (pwm_calibrated_min, pwm_calibrated_max)).round()

        return mapped_value_2
