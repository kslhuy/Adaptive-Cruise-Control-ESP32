#include <Arduino.h>
#include <math.h>      // For math functions
#include "MotorController.h"


#define NOOP 0
#define STOP_VEHICLE 1
#define SPEED_UP 2
#define SPEED_DOWN 3
#define LEFT_TURN 4
#define RIGHT_TURN 5

#define pre 1     
#define follow 2     


#define CAR pre

class VehicleLogic {
public:
    const int KEYBOARD_SPEED_INC = 10;
    const int KEYBOARD_SPEED_DEC = 10;

    const int KEYBOARD_STEERING_PULSE = 10;
    const int KEYBOARD_STEERING_DECAY = 9;

    const int DUTY_MOTOR_MIN = 100;
    const int DUTY_MOTOR_MAX = 250;
    
    #if CAR == follow
    const int PWMSteering_Center = 339;
    // CAR 2
    #else
    const int PWMSteering_Center = 365;
    #endif
    


    VehicleLogic() {
        kbd_speed = 0;
        kbd_steering_mag = 0;
        kbd_steering_sign = 1;
        // inf_speed = 0;
        // inf_steering = 0;
    }



    void _set_noop() {
        // Do nothing
    }

    void _set_speed(int pwmSpeed) {
        kbd_speed = pwmSpeed;

    }

    void _set_Up() {
        kbd_speed = std::min(kbd_speed + KEYBOARD_SPEED_INC, DUTY_MOTOR_MAX);

    }

    void _set_Down() {
        kbd_speed = std::max(kbd_speed - KEYBOARD_SPEED_INC, DUTY_MOTOR_MIN);
    }


    void _set_steering(int pwmSteering) {
        int new_steering = pwmSteering;
        if (new_steering > PWMSteering_Center) {
            kbd_steering_sign = -1;
        } else {
            kbd_steering_sign = +1;
        }
        kbd_steering_mag = new_steering;
    }

    void _set_veer_right() {
        float new_steering = kbd_steering_mag * kbd_steering_sign + KEYBOARD_STEERING_PULSE;
        if (new_steering >= PWMSteering_Center) {
            kbd_steering_mag = new_steering;
            kbd_steering_sign = 1;
        } else {
            kbd_steering_mag = -new_steering;
            kbd_steering_sign = -1;
        }
        kbd_steering_mag = _limit_rang(kbd_steering_mag);
    }

    void _set_veer_left() {
        float new_steering = kbd_steering_mag * kbd_steering_sign - KEYBOARD_STEERING_PULSE;
        if (new_steering >= PWMSteering_Center) {
            kbd_steering_mag = new_steering;
            kbd_steering_sign = 1;
        } else {
            kbd_steering_mag = -new_steering;
            kbd_steering_sign = -1;
        }
        kbd_steering_mag = _limit_rang(kbd_steering_mag);
    }

    void _set_stop() {
        kbd_speed = 0;
        kbd_steering_mag = PWMSteering_Center;
        kbd_steering_sign = 1;
        // inf_speed = 0.0;
        // inf_steering = 0.0;
    }

    void get_next_speed_steering_data(int& speed, int& steering) {

        speed = kbd_speed;
        steering = kbd_steering_mag;
        

        // std::tie(speed, steering) = _get_speed_steering(speed_noise, steering_noise);

        // Decay the steering magnitude for kbd inputs - we are pulsing
        if (kbd_steering_mag != PWMSteering_Center){
            
            kbd_steering_mag += KEYBOARD_STEERING_DECAY*kbd_steering_sign;
            kbd_steering_mag = _limit_rang(kbd_steering_mag);
            // if (kbd_steering_sign == 1){
            //     kbd_steering_mag = std::min(PWMSteering_Center, kbd_steering_mag );
            // }else {
            //     kbd_steering_mag = std::max(PWMSteering_Center , kbd_steering_mag);
            // }
            // kbd_steering_mag = _limit_rang(kbd_steering_mag , 425 , 267);
        }
    }

private:
    int kbd_speed;
    int kbd_steering_mag;
    int kbd_steering_sign;
    // int inf_speed;
    // int inf_steering;

    // std::pair<float, float> _get_speed_steering(float speed_noise = 0.0, float steering_noise = 0.0) {
    //     float total_speed = _limit_rang(std::min(kbd_speed, inf_speed) + speed_noise);
    //     float total_steering = _limit_rang(inf_steering +
    //         (kbd_steering_mag * kbd_steering_sign) + steering_noise);
    //     return std::make_pair(total_speed, total_steering);
    // }

    int _limit_rang(int value ) {
        #if (CAR == follow)
        return std::min(425, std::max(value, 267));
        #else
        return std::min(441, std::max(value, 289));
        #endif
    }


};
