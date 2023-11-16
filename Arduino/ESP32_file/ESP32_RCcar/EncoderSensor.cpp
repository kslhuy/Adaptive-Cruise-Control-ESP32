// #include "EncoderSensor.h"

// EncoderSensor::EncoderSensor(int encoderPinA) {
//   this->encoderPinA = encoderPinA;
//   // this->encoderPinB = encoderPinB;
//   // this->pulsesPerRevolution = pulsesPerRevolution;
//   this->distancePerPulse = this->wheelCircumference / this->pulsesPerRevolution;;
  

//   pinMode(encoderPinA, INPUT);
//   // pinMode(encoderPinB, INPUT);
//   attachInterrupt(digitalPinToInterrupt(encoderPinA), EncoderSensor::countPulse, RISING);
// }

// void EncoderSensor::begin() {
//   pulseCount = 0;
//   lastPulseTime = millis();
// }

// bool EncoderSensor::can_GetSpeed(){
//   unsigned long currentTime = millis();
//   return (currentTime - lastPulseTime >= interval);
// }

// float EncoderSensor::get_Speed() {
//   unsigned long currentTime = millis();

//   float revs = (float)pulseCount / pulsesPerRevolution;
//   float timeInSeconds = (float)(currentTime - lastPulseTime) / 1000.0;
//   float speedRPM = (revs / timeInSeconds) * 60.0;
//   float speedMPS = (speedRPM * distancePerPulse) / 60.0;

//   pulseCount = 0;
//   lastPulseTime = currentTime;

//   return speedMPS;
  
// }

// void EncoderSensor::countPulse() {
//   pulseCount++;
// }



// ------------------- open bot

// unsigned long wheel_interval = 1000;  // Interval for sending wheel odometry

// #if (HAS_SPEED_SENSORS_FRONT)
//   pinMode(PIN_SPEED_LF, INPUT_PULLUP);
//   pinMode(PIN_SPEED_RF, INPUT_PULLUP);
//   attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(PIN_SPEED_LF), update_speed_lf, RISING);
//   attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(PIN_SPEED_RF), update_speed_rf, RISING);
// #endif

// // ISR: Increment speed sensor counter (left front)
// void update_speed_lf() {
//   if (ctrl_left < 0) {
//     counter_lf--;
//   } else if (ctrl_left > 0) {
//     counter_lf++;
//   }
// }
// void send_wheel_reading(long duration) {
//   float rpm_factor = 60.0 * 1000.0 / duration / TICKS_PER_REV;
//   rpm_left = (counter_lf + counter_lb + counter_lm) * rpm_factor;
//   rpm_right = (counter_rf + counter_rb + counter_rm) * rpm_factor;
//   counter_lf = 0;
//   counter_rf = 0;
//   counter_lb = 0;
//   counter_rb = 0;
//   counter_lm = 0;
//   counter_rm = 0;
// #if (HAS_SPEED_SENSORS_FRONT and HAS_SPEED_SENSORS_BACK and HAS_SPEED_SENSORS_MIDDLE)
//   sendData("w" + String(rpm_left / 3) + "," + String(rpm_right / 3));
// #elif ((HAS_SPEED_SENSORS_FRONT and HAS_SPEED_SENSORS_BACK) or (HAS_SPEED_SENSORS_FRONT and HAS_SPEED_SENSORS_MIDDLE) or (HAS_SPEED_SENSORS_MIDDLE and HAS_SPEED_SENSORS_BACK))
//   sendData("w" + String(rpm_left / 2) + "," + String(rpm_right / 2));
// #elif (HAS_SPEED_SENSORS_FRONT or HAS_SPEED_SENSORS_BACK or HAS_SPEED_SENSORS_MIDDLE)
//   sendData("w" + String(rpm_left) + "," + String(rpm_right));
// #endif
// }



// ----------- loop ()
#if (HAS_SPEED_SENSORS_FRONT or HAS_SPEED_SENSORS_BACK or HAS_SPEED_SENSORS_MIDDLE)
  // Send wheel odometry reading via serial
  if ((millis() - wheel_time) >= wheel_interval) {
    send_wheel_reading(millis() - wheel_time);
    wheel_time = millis();
  }
#endif