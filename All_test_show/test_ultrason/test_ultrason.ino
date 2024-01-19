/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-hc-sr04-ultrasonic-arduino/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/

#include <algorithm>


const int trigPin = 18;
const int echoPin = 5;

//define sound speed in cm/uS
#define SOUND_SPEED 0.034

unsigned int pingSpeed = 50; // How frequently are we going to send out a ping (in milliseconds). 50ms would be 20 times a second.
unsigned long pingTimer;     // Holds the next ping time.

long duration;
float distanceCm;
float distanceInch;

const unsigned int bufferSize = 10; // Number of values to buffer
float distanceBuffer[bufferSize];
unsigned int bufferIndex = 0;


const float temperatureCelsius = 20.0; // Adjust to actual temperature in Celsius
const float relativeHumidity = 80.0;   // Adjust to actual relative humidity in percentage

// Convert speed of sound to centimeters per microsecond
const float speedOfSoundCmPerUs = (331.5 + 0.6 * temperatureCelsius + 0.0124 * relativeHumidity) / 10000.0; // Divide by 2 * 1000000 to convert to cm/us


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

float calculateMedian(float values[], int size) {
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


void setup() {
  Serial.begin(115200); // Starts the serial communication
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  pingTimer = millis();

  for (int i = 0; i < bufferSize; i++) {
    distanceBuffer[i] = 0.0;
  }




}

void loop() {

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
    duration = pulseIn(echoPin, HIGH);
    
    // Calculate the distance
    distanceCm = duration * speedOfSoundCmPerUs/2.0;

    // Store the value in the buffer
    distanceBuffer[bufferIndex] = distanceCm;
    bufferIndex = (bufferIndex + 1) % bufferSize;

    // Calculate the median of the buffer
    float median = calculateMedian(distanceBuffer, bufferSize);

    // Convert to inches
    // distanceInch = distanceCm * CM_TO_INCH;
    
    // Prints the distance in the Serial Monitor
    Serial.print("Distance (cm): ");
    Serial.println(median);
    // Serial.print("Distance (inch): ");
    // Serial.println(distanceInch);
  }
    
}





// #define trigPin 18
// #define echoPin 5
// #define SOUND_SPEED 0.034


// unsigned int pingSpeed = 50; // How frequently are we going to send out a ping (in milliseconds). 50ms would be 20 times a second.
// unsigned long pingTimer;     // Holds the next ping time.



// typedef struct ultrasonic_s {
//   uint64_t last_trigger_time_us;
//   uint64_t time_of_flight_us;
//   bool start_new_reading;
//   bool obtained_ranging;
// };

// typedef struct {
//   uint8_t *buffer;
//   size_t capacity;
//   size_t write_index;
//   bool filled;
// } avg_buffer_s;

// static ultrasonic_s front_ultrasonic;
// static avg_buffer_s front_avg_buffer;
// static const uint8_t AVG_BUFFER_CAPACITY = 10;

// void IRAM_ATTR front_ultrasonic_echo_falling_edge_interrupt_handler() {
//   front_ultrasonic.time_of_flight_us = micros() - front_ultrasonic.last_trigger_time_us;
//   front_ultrasonic.obtained_ranging = true;
//   front_ultrasonic.start_new_reading = true;
// }

// void IRAM_ATTR front_ultrasonic_echo_rising_edge_interrupt_handler() {
  
// }

// void ultrasonic__initialize(ultrasonic_s *ultrasonic_i){

//   pinMode(trigPin, OUTPUT);
//   pinMode(echoPin, INPUT);
//   ultrasonic_i->time_of_flight_us = 0U;
//   ultrasonic_i->last_trigger_time_us = 0U;
//   ultrasonic_i->start_new_reading = true;
//   ultrasonic_i->obtained_ranging = false;

//   digitalWrite(trigPin, LOW);
// }

// // Blocking get range function which can stall at maximum (60 ms + 10 us) = 60.01 ms
// void ultrasonic__get_range_blocking(ultrasonic_s *ultrasonic_i){
//   if ((micros() - ultrasonic_i->last_trigger_time_us) > 50U) {
//     const uint64_t timer = micros();
//     digitalWrite(trigPin, HIGH);
//     while ((micros() - timer) < 10U) {
//       ; // Hold trigger line high for 10 microseconds
//     }
//     digitalWrite(trigPin, LOW);
//     ultrasonic_i->last_trigger_time_us = micros();

//     bool echo_rising_edge_found = false;
//     uint64_t echo_high_trigger_time_us = 0U;

//     do {
//       if (digitalRead(echoPin)) { // sample echo until high
//         echo_rising_edge_found = true;
//         echo_high_trigger_time_us = micros() - ultrasonic_i->last_trigger_time_us;
//       } else {
//         if (echo_rising_edge_found) {                                // Echo was sampled high and deasserted
//           ultrasonic_i->time_of_flight_us = echo_high_trigger_time_us; // Divide by 148U for inch conversion
//           break;                                                     // Exit when echo falling edge detected
//         }
//       }

//     } while ((micros() - ultrasonic_i->last_trigger_time_us) < 60000U); // Timeout on 60 milliseconds
//   }
// }

// // Blocks for 10 us for toggling Trigger line
// void ultrasonic__initiate_range(ultrasonic_s *ultrasonic_i){
//   // Send trigger on the start of a new reading through interrupts or when a 60 ms timeout occurs
//   if (ultrasonic_i->start_new_reading || (micros() - ultrasonic_i->last_trigger_time_us) > 6000U) {
//     ultrasonic_i->start_new_reading = false;
//     digitalWrite(trigPin, HIGH);
//     delayMicroseconds(10); // Hold trigger line high for 10 microseconds
//     digitalWrite(trigPin, LOW);
//     ultrasonic_i->last_trigger_time_us = micros();
//     digitalWrite(trigPin, LOW);

//   }
// }


// void ultrasonic_implementation__initialize(void) {
//   // Initialize Ultrasonics
//   ultrasonic__initialize(&front_ultrasonic); //  Trigger, Echo
//   // Attach and Initialize Echo falling edge interrupt handlers
//   attachInterrupt(echoPin, front_ultrasonic_echo_falling_edge_interrupt_handler, FALLING);

//   // attachInterrupt(echoPin, front_ultrasonic_echo_rising_edge_interrupt_handler, RISING);

//   // Initialize averaging buffers
//   static uint8_t front_buffer[AVG_BUFFER_CAPACITY];
//   avg_buffer__initialize(&front_avg_buffer, front_buffer, AVG_BUFFER_CAPACITY);

// }

// static void ultrasonic_implementation__update_avg_buffers(void) {
//   if (front_ultrasonic.obtained_ranging) {
//     const uint8_t front_ultrasonic_range = front_ultrasonic.time_of_flight_us * SOUND_SPEED/2;

    
//     avg_buffer__insert_value(&front_avg_buffer, front_ultrasonic_range > 2 ? front_ultrasonic_range
//                                                                            : avg_buffer__get_value(&front_avg_buffer));
//     front_ultrasonic.obtained_ranging = false;
//   }
// }

// void ultrasonic_implementation__initiate_ultrasonics_range(void) {
//   ultrasonic_implementation__update_avg_buffers();

//   ultrasonic__initiate_range(&front_ultrasonic);
// }

// uint8_t ultrasonic_implementation__get_front_ultrasonic_distance_in(void) {
//   return avg_buffer__get_value(&front_avg_buffer);
// }


// void avg_buffer__initialize(avg_buffer_s *avg_buffer, uint8_t *buffer, size_t buffer_capacity) {
//   avg_buffer->buffer = buffer;
//   avg_buffer->write_index = 0;
//   avg_buffer->filled = false;
//   avg_buffer->capacity = buffer_capacity;
//   memset(avg_buffer->buffer, 0, avg_buffer->capacity);
// }

// void avg_buffer__insert_value(avg_buffer_s *avg_buffer, uint8_t value) {
//   avg_buffer->buffer[avg_buffer->write_index++] = value;
//   if (!avg_buffer->filled && avg_buffer->write_index >= avg_buffer->capacity) {
//     avg_buffer->filled = true;
//   }
//   avg_buffer->write_index %= avg_buffer->capacity;
// }

// uint8_t avg_buffer__get_value(avg_buffer_s *avg_buffer) {
//   size_t sum = 0;
//   uint8_t avg = 0;

//   const size_t capacity = avg_buffer__get_insertions(avg_buffer);

//   if (capacity) {
//     for (size_t i = 0; i < capacity; i++) {
//       sum += avg_buffer->buffer[i];
//     }

//     avg = sum / capacity;
//   }

//   return avg;
// }

// uint8_t avg_buffer__get_insertions(avg_buffer_s *avg_buffer) {
//   return avg_buffer->filled ? avg_buffer->capacity : avg_buffer->write_index;
// }

// void setup(){
//   Serial.begin (115200);

//   pingTimer = millis();
//   ultrasonic_implementation__initialize();
// }
// void loop(){
//   if (millis() >= pingTimer) {   // pingSpeed milliseconds since last ping, do another ping.
//     pingTimer += pingSpeed; 
//     Serial.println(ultrasonic_implementation__get_front_ultrasonic_distance_in()); 
//     // Serial.println (front_ultrasonic.time_of_flight_us);
//   }
//   ultrasonic_implementation__initiate_ultrasonics_range();

//   // Serial.println(ultrasonic_implementation__get_front_ultrasonic_distance_in()); 
//   // delay(1000);
// }

























// #define trigPin 18
// #define echoPin 5
// // #define LED 21
// #define MAX_SENSOR_DISTANCE 200  // cm
// #define US_ROUNDTRIP_CM 57 
// #define ECHO_TIMER_FREQ 24  // 24uS 
// #define MAX_SENSOR_DELAY 6000 // 60 uS
 
// float duration, distance;
// hw_timer_t *Timer0_Cfg = NULL;

// unsigned int pingSpeed = 50; // How frequently are we going to send out a ping (in milliseconds). 50ms would be 20 times a second.
// unsigned long pingTimer;     // Holds the next ping time.

// unsigned int _maxEchoTime;
// unsigned long _max_time;
// unsigned long ping_result;


 
// void setup() {
//   Serial.begin (115200);
//   pinMode(trigPin, OUTPUT);
//   pinMode(echoPin, INPUT);

//   pingTimer = millis(); // Start now.
//   _maxEchoTime = min((unsigned int) 1, (unsigned int) MAX_SENSOR_DISTANCE + 1) * US_ROUNDTRIP_CM; // Calculate the maximum distance in uS (no rounding).
 
//   // Use 1st timer of 4 (counted from zero).
//   // Set 80 divider for prescaler (see ESP32 Technical Reference Manual for more
//   // info).

//   Timer0_Cfg = timerBegin(0, 80, true);
//   // Attach echoCheck function to our timer.

//   timerAttachInterrupt(Timer0_Cfg, &echoCheck, true);
//   // Set alarm to call onTimer function every second (value in microseconds).
//   // Repeat the alarm (third parameter)
//   timerAlarmWrite(Timer0_Cfg, 24, true);

//   timerAlarmEnable(Timer0_Cfg);




// }

// // void timer_us(unsigned int frequency, void (*userFunc)(void)){
  

// //   timerAlarmDisable(Timer0_Cfg);
// //   // Start an alarm
// //   timerAlarmEnable(Timer0_Cfg);
// //   // Serial.println("Timer_us");
// // }

 
// void loop() {
//   if (millis() >= pingTimer) {   // pingSpeed milliseconds since last ping, do another ping.
//     // Serial.println("DO ping: ");

//     pingTimer += pingSpeed;      // Set the next ping time.
//     ping_timer(); // Send out the ping, calls "echoCheck" function every 24uS where you can check the ping status.
//   }

// }

// void ping_timer(){
//   // Serial.print("ping_trigger: "  );
//   // Serial.println(ping_trigger());

//   if (!ping_trigger()) return;         // Trigger a ping, if it returns false, return without starting the echo timer.
//   // timer_us(ECHO_TIMER_FREQ, userFunc); // Set ping echo timer check every ECHO_TIMER_FREQ uS.
// }

// bool ping_trigger(){
//   // digitalWrite(trigPin, LOW);
//   // delayMicroseconds(2);
//   // Write a pulse to the HC-SR04 Trigger Pin

//   digitalWrite(trigPin, HIGH);
//   delayMicroseconds(10);
//   digitalWrite(trigPin, LOW);

//   if (digitalRead(echoPin)) {
//     // Serial.print("echopin: ");
//     // Serial.print (digitalRead(echoPin)); 
//     return false;
//   }                // // sample echo stil high , Previous ping hasn't finished, abort.
//   _max_time = micros() + _maxEchoTime + MAX_SENSOR_DELAY; // Maximum time we'll wait for ping to start (most sensors are <450uS, the SRF06 can take up to 34,300uS!)
//   while (!digitalRead(echoPin))                          // Wait for ping to start.
    
//     if (micros() > _max_time) {
//       // Serial.println("time out");
      
//       return false;             // Took too long to start, abort.
//     }
//   _max_time = micros() + _maxEchoTime; // Ping started, set the time-out.
// 	return true;                         // Ping started successfully.


// } 

// bool check_timer() {
//   if (micros() > _max_time) { // Outside the time-out limit.
//     // timerAlarmDisable(Timer0_Cfg);           // Disable timer interrupt
//     // Serial.println("check_timer false");
//     return false;           // Cancel ping timer.
//   }
//   if (digitalRead(echoPin)) { // Ping echo received.
//     // timerAlarmDisable(Timer0_Cfg);                // Disable timer interrupt
//     ping_result = (micros() - (_max_time - _maxEchoTime) ); // Calculate ping time including overhead.
//     return true;                 // Return ping echo true.
//   }

//   return false; // Return false because there's no ping echo yet.
// }


// void ARDUINO_ISR_ATTR echoCheck() { // Timer2 interrupt calls this function every 24uS where you can check the ping status.
 
//   // Don't do anything here!
//   if (check_timer()) { // This is how you check to see if the ping was received.
//     // Here's where you can add code.
//     Serial.print("Ping: ");
//     Serial.print(ping_result / US_ROUNDTRIP_CM); // Ping returned, uS result in ping_result, convert to cm with US_ROUNDTRIP_CM.
//     Serial.println("cm");
//   }
//   // Don't do anything here!
// }