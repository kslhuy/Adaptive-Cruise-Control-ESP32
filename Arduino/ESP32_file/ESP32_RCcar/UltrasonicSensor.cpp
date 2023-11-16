#include "UltrasonicSensor.h"
#include <Arduino.h>

// const int trigPin = 18;
// const int echoPin = 5;

//define sound speed in cm/uS
// #define SOUND_SPEED 0.034

// unsigned int pingSpeed = 50; // How frequently are we going to send out a ping (in milliseconds). 50ms would be 20 times a second.
// unsigned long pingTimer;     // Holds the next ping time.

// long duration;
// float distanceCm;
// float distanceInch;

// const unsigned int bufferSize = 10; // Number of values to buffer
// float distanceBuffer[bufferSize];
// unsigned int bufferIndex = 0;


// const float temperatureCelsius = 20.0; // Adjust to actual temperature in Celsius
// const float relativeHumidity = 80.0;   // Adjust to actual relative humidity in percentage

// // Convert speed of sound to centimeters per microsecond
// const float speedOfSoundCmPerUs = (331.5 + 0.6 * temperatureCelsius + 0.0124 * relativeHumidity) / 10000.0; // Divide by 2 * 1000000 to convert to cm/us


UltrasonicSensor::UltrasonicSensor(int trigPin, int echoPin ) {
  this->trigPin = trigPin;
  this->echoPin = echoPin;
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  this->pingTimer = millis();
  bufferIndex = 0; // Initialize buffer index

}

// float calculateMedian(float values[], int size) {
//   float temp[size];
//   memcpy(temp, values, size * sizeof(float));
//   for (int i = 0; i < size; i++) {
//     for (int j = i + 1; j < size; j++) {
//       if (temp[j] < temp[i]) {
//         float tempValue = temp[i];
//         temp[i] = temp[j];
//         temp[j] = tempValue;
//       }
//     }
//   }
//   if (size % 2 == 0) {
//     return (temp[size / 2 - 1] + temp[size / 2]) / 2.0;
//   } else {
//     return temp[size / 2];
//   }
// }

float UltrasonicSensor::calculateMedian(float values[],unsigned int size) {
  float sortedValues[size];
  memcpy(sortedValues, values, size * sizeof(float));

  float *begin = sortedValues;
  float *end = sortedValues + size;
  float *middle = begin + size / 2;

  std::sort(begin, end); // Use the built-in sort function

  if (size % 2 == 0) {
    return (*middle + *(middle - 1)) / 2.0;
  } else {
    return *middle;
  }
}


// void setup() {
//   Serial.begin(115200); // Starts the serial communication
//   pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
//   pinMode(echoPin, INPUT); // Sets the echoPin as an Input
//   pingTimer = millis();

//   for (int i = 0; i < bufferSize; i++) {
//     distanceBuffer[i] = 0.0;
//   }

// }

void UltrasonicSensor::calculatDistance() {
  if (millis() >= pingTimer) {   // pingSpeed milliseconds since last ping, do another ping.

    pingTimer += pingSpeed; 


    // Clears the trigPin
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    
    // Reads the echoPin, returns the sound wave travel time in microseconds
    unsigned long duration = pulseIn(echoPin, HIGH);
    
    // Calculate the distance
    // float distanceCm = duration * speedOfSoundCmPerUs/2.0;

    // Store the value in the buffer
    distanceBuffer[bufferIndex] = duration * speedOfSoundCmPerUs/2.0;
    bufferIndex = (bufferIndex + 1) % bufferSize;

    // Calculate the median of the buffer
    median = calculateMedian(distanceBuffer, bufferSize);
    // Serial.print(median  );


    // Prints the distance in the Serial Monitor
    // Serial.print("Distance (cm): ");
    // Serial.println(median);
    // Serial.print("Distance (inch): ");
    // Serial.println(distanceInch);
  }
}


float UltrasonicSensor::getDistance() {

  return median;
    
}



// //------------------------------------------------------//
// // INTERRUPT SERVICE ROUTINES (ISR)
// //------------------------------------------------------//
// #if HAS_SONAR

// // ISR: Start timer to measure the time it takes for the pulse to return
// void start_timer() {
//   start_time = micros();
//   attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(PIN_ECHO), stop_timer, FALLING);
// }

// // ISR: Stop timer and record the time
// void stop_timer() {
//   echo_time = micros() - start_time;
//   detachPinChangeInterrupt(digitalPinToPinChangeInterrupt(PIN_ECHO));
//   ping_success = true;
// }

// #endif



// #if HAS_SONAR

// void send_sonar_reading() {
//   sendData("s" + String(distance_estimate));
// }

// // Send pulse by toggling trigger pin
// void send_ping() {
//   echo_time = 0;
//   ping_success = false;
//   if (PIN_TRIGGER == PIN_ECHO)
//     pinMode(PIN_TRIGGER, OUTPUT);
//   digitalWrite(PIN_TRIGGER, LOW);
//   delayMicroseconds(5);
//   digitalWrite(PIN_TRIGGER, HIGH);
//   delayMicroseconds(10);
//   digitalWrite(PIN_TRIGGER, LOW);
//   if (PIN_TRIGGER == PIN_ECHO)
//     pinMode(PIN_ECHO, INPUT);
//   attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(PIN_ECHO), start_timer, RISING);
// }

// void update_distance_estimate() {
// #if SONAR_MEDIAN
//   distance_array[distance_counter % distance_array_sz] = distance;
//   distance_counter++;
//   distance_estimate = get_median(distance_array, distance_array_sz);
// #else
//   distance_estimate = distance;
// #endif
// }

// #endif




/// ---------- Code run in loop ------------
// #if HAS_SONAR
//   // Check for successful sonar reading
//   if (!sonar_sent && ping_success) {
//     distance = echo_time * US_TO_CM;
//     update_distance_estimate();
//     send_sonar_reading();
//     sonar_sent = true;
//   }
//   // Measure distance every sonar_interval
//   if ((millis() - sonar_time) >= max(sonar_interval, MAX_SONAR_TIME)) {
//     if (!sonar_sent && !ping_success) {  // Send max val if last ping was not returned
//       distance = MAX_SONAR_DISTANCE;
//       update_distance_estimate();
//       send_sonar_reading();
//       sonar_sent = true;
//     }
//     sonar_time = millis();
//     sonar_sent = false;
//     send_ping();
//   }
// #endif