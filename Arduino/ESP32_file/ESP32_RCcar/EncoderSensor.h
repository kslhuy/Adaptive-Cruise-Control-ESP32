// #ifndef EncoderSensor_h
// #define EncoderSensor_h
// #include <Arduino.h>

// class EncoderSensor {
// public:
//   EncoderSensor(int encoderPinA);
//   void begin();
//   bool can_GetSpeed();
//   float get_Speed();
// private:
//   int encoderPinA;
//   int pulsesPerRevolution = 20;  // encoder have 20 holes
//   float wheelCircumference = 0.5; // Circumference of the wheel in meters
//   float distancePerPulse;
//   volatile unsigned long pulseCount = 0;
//   volatile unsigned long lastPulseTime = 0;
//   const unsigned long interval = 1000; // Interval in milliseconds for calculating speed

//   static void countPulse();
// };

// #endif
