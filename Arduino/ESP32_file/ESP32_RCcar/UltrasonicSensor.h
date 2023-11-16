#ifndef UltrasonicSensor_h
#define UltrasonicSensor_h

class UltrasonicSensor {
public:
  UltrasonicSensor(int trigPin, int echoPin);
  float calculateMedian(float values[], unsigned int size);
  void calculatDistance();
  float getDistance();


private:
  int trigPin;
  int echoPin;
  float median;

  static const unsigned int bufferSize = 10; // Number of values to buffer
  float distanceBuffer[bufferSize]; // Buffer for distance values
  unsigned int bufferIndex; // Buffer index
  unsigned int pingSpeed = 60; // How frequently are we going to send out a ping (in milliseconds). 50ms would be 20 times a second.
  unsigned long pingTimer;     // Holds the next ping time.

  const float temperatureCelsius = 20.0; // Adjust to actual temperature in Celsius
  const float relativeHumidity = 80.0;   // Adjust to actual relative humidity in percentage

  // Convert speed of sound to centimeters per microsecond
  const float speedOfSoundCmPerUs = (331.5 + 0.6 * temperatureCelsius + 0.0124 * relativeHumidity) / 10000.0; // Divide by 2 * 1000000 to convert to cm/us


};

#endif
