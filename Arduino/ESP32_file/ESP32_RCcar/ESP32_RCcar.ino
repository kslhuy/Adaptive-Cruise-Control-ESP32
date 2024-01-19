/*
 * This ESP32 code is created by esp32io.com
 *
 * This ESP32 code is released in the public domain
 *
 * For more detail (instruction and wiring diagram), visit https://esp32io.com/tutorials/esp32-servo-motor
 */
  // %%% xe pin cu 
  // degree = map(value , 0 , 4095 , 140 , 480);
  // value = map(degree , 20 , 140 , 137 , 480);
// %%%%%%%%%%%%%%%%  xe pin moi mua
  // degree = map(value , 0 , 4095 , 20 , 140);
  // value = map(degree , 20 , 140 , 138 , 482);

#include <WiFi.h>
#include "MotorController.h"
// #include "UltrasonicSensor.h"
#include "VehicleLogic.cpp"

// #include <ESP32Servo.h>


const char* ssid = "laptophuy";
const char* password = "huy123qwe";

// PC server IP address and port number
const char* pc_ip = "192.168.85.251";  // Replace with your PC's IP address

#define pre 1
#define follow 2


// MODIFIER HERE
#define ESP pre

#if (ESP == pre)
const uint16_t pc_port_send = 4444;       // Replace with the chosen port number on the PC
const uint16_t pc_port_receive = 3333;       // Replace with the chosen port number on the PC
#else
const uint16_t pc_port_send = 4455;       // Replace with the chosen port number on the PC
const uint16_t pc_port_receive = 3344;       // Replace with the chosen port number on the PC
#endif
// Sonnar

#define attachPinChangeInterrupt attachInterrupt
#define detachPinChangeInterrupt detachInterrupt
#define digitalPinToPinChangeInterrupt digitalPinToInterrupt



const int PIN_TRIGGER = 18;
const int PIN_ECHO = 5;
// Sonar sensor

// const float temperatureCelsius = 20.0; // Adjust to actual temperature in Celsius
// const float relativeHumidity = 80.0;   // Adjust to actual relative humidity in percentage

// // Convert speed of sound to centimeters per microsecond
// const float US_TO_CM = ((331.5 + 0.6 * temperatureCelsius + 0.0124 * relativeHumidity) *  100 / 1000000) / 2; //  

const float US_TO_CM = 0.01715;               //cm/uS -> (343 * 100 / 1000000) / 2;
const unsigned int MAX_SONAR_DISTANCE = 250;  //cm
const unsigned long MAX_SONAR_TIME = (long)MAX_SONAR_DISTANCE * 2 * 10 / 343 + 1;
const unsigned int STOP_DISTANCE = 10;  //cm

unsigned long sonar_interval = 200;


unsigned long sonar_time = 0;
boolean sonar_sent = false;
boolean ping_success = false;
float distance = -1;           //cm
float distance_estimate = -1;  //cm
unsigned long start_time;
unsigned long echo_time = 0;

const unsigned int distance_array_sz = 3;
float distance_array[distance_array_sz] = {};
unsigned int distance_counter = 0;



// Servo 
#define SERVO_PIN 2  // ESP32 pin GIOP2 connected to servo motor
const int channel_servo = 10;




TaskHandle_t Task1;  //ULTRASONIC MEASUREMENT
TaskHandle_t Task2;  //WIFI TCP SOCKET
QueueHandle_t q = NULL;

WiFiClient client_send;
WiFiClient client_receive;



static void Task1code(void* parameter);
static void Task2code(void* parameter);

// MOTOR Pin
const int motor1Pin1 = 27; // Pin1 or Pin3 
const int motor1Pin2 = 26; // Pin2 or Pin4 
const int enable1Pin = 14; // enable Pin

// Setting PWM properties
const int freq = 5000;
const int pwmChannel = 0;
const int resolution = 8;

// Encoder

const unsigned int diskslots = 20;  //  disc slot (holes)!! 
// const unsigned long SPEED_TRIGGER_THRESHOLD = 1;  // Triggers within this time will be ignored (ms)

const int encoder_pin = 15;             // The pin the encoder is connected
unsigned int rpm;                 // rpm reading
float speed;                 // rpm reading

volatile int pulses;             // number of pulses
unsigned long timeold;
const float diameter_wheel = 7; // cm
const float wheelCircumference = 22.00; // Circumference of the wheel in cm
float distancePerPulse = wheelCircumference / diskslots;
const int deltaTime_GetData = 500;



// #if (ESP == pre)
// #include <Wire.h>
// #include <Adafruit_Sensor.h>
// #include <Adafruit_ADXL345_U.h>
// // SCL 22
// // SDA 21
// /* Assign a unique ID to this sensor at the same time */
// Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
// #endif




// The number of pulses per revolution
// depends on your index disc!!

// For Ultrasonic
// UltrasonicSensor ultrasonic(PIN_TRIGGER, PIN_ECHO);
// EncoderSensor encoder(encoder_pin);

// QueueList<int> speedQueue;
// QueueList<int> steerQueue;

VehicleLogic vehicleLogic;
bool isforward = true;

unsigned int loopSpeed = 100; // How frequently are we going to send out a ping (in milliseconds). 50ms would be 20 times a second.
unsigned long vehicleLogicTimer;     // Holds the next ping time.

int current_speedPWM = 0;
int current_steeringPWM = 339;

bool servoIsCenter = false;


// ISR: Start timer to measure the time it takes for the pulse to return
void start_timer() {
  start_time = micros();
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(PIN_ECHO), stop_timer, FALLING);
}

// ISR: Stop timer and record the time
void stop_timer() {
  echo_time = micros() - start_time;
  detachPinChangeInterrupt(digitalPinToPinChangeInterrupt(PIN_ECHO));
  ping_success = true;
}

// // Send pulse by toggling trigger pin
void send_ping() {
  echo_time = 0;
  ping_success = false;
  if (PIN_TRIGGER == PIN_ECHO)
    pinMode(PIN_TRIGGER, OUTPUT);
  digitalWrite(PIN_TRIGGER, LOW);
  delayMicroseconds(5);
  digitalWrite(PIN_TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIGGER, LOW);
  if (PIN_TRIGGER == PIN_ECHO)
    pinMode(PIN_ECHO, INPUT);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(PIN_ECHO), start_timer, RISING);
}


void update_distance_estimate() {
  distance_array[distance_counter % distance_array_sz] = distance;
  distance_counter++;
  distance_estimate = calculateMedian(distance_array, distance_array_sz);
  // Serial.print("linera speed = ");
  // Serial.println( distance_estimate );
  

}

float calculateMedian(float values[],unsigned int size) {
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
  Serial.begin(115200);
  // Connection Wifi setup
  setup_Connection_Wifi();

  // // Servo ultrasonic


  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_TRIGGER, OUTPUT);
  // // Servo setup

  setup_Servo();
  setup_Motor();
  setup_Encoder();

  vehicleLogicTimer = millis();

  // Serial.print("SetUp Done");

  // Task Setup 
  xTaskCreatePinnedToCore(
    Task_dataAcquisition,
    "DataAcquisitionTask",
    10000,
    NULL,
    1,
    NULL,
    1
  );

    
  // //   // Setup Task for Send measurement
  xTaskCreatePinnedToCore(
    Task2_SendDataToPC, /* Task function. */
    "Task2",            /* name of task. */
    5000,              /* Stack size of task */
    NULL,               /* parameter of the task */
    1,                  /* priority of the task */
    &Task2,             /* Task handle to keep track of created task */
    1);                 /* pin task to core 0 */
   
}




static void Task2_SendDataToPC(void* parameter) {

  while (1) {
    // xQueueReceive(q, &distance, portMAX_DELAY);  //read measurement value from Queue and run code below, if no value, WAIT....
    if (!client_send.connected()) {
      if (!client_send.connect(pc_ip, pc_port_send)) {
        Serial.println("Connection failed.");
      }
    }

    // Send the data as a string to the server
    // String dataString = "dist:" + String(ultrasonic.getDistance()) + ";"+"vel:" + String(speed)+"\n";
    String dataString = "dist:" + String(distance_estimate) + ";"+"vel:" + String(speed)+"\n";
    
    client_send.print(dataString);

    // Serial.print("Sent data: ");
    Serial.println(dataString);

    vTaskDelay(pdMS_TO_TICKS(100)); // Send data every 20 ms (50 Hz)

  }
}

void Task_dataAcquisition(void *pvParameters) {
  while (1) {
    // Update position and velocity data (Replace this with your actual data acquisition code)
    // position += 0.1;
    // velocity += 1.0;
    if (!client_receive.connected()) {
      if (!client_receive.connect(pc_ip, pc_port_receive)) {
        Serial.println("Connection failed.");
        vTaskDelay(pdMS_TO_TICKS(100)); 

      }
    }

    String receivedData = client_receive.readStringUntil('\n');
    // receivedResponse.trim();
    // Serial.println(receivedData);
  
    int colonPosition = receivedData.indexOf(":");
      if (colonPosition != -1) { // Ensure that a colon was found
        String command = receivedData.substring(0, colonPosition);
        String valueStr = receivedData.substring(colonPosition + 1);
        // Serial.println("command: " + command + ", Value: " + valueStr);
        // Serial.println("command: " + command );

        // int value = valueStr.toInt();

        if (command == "FORWARD") {
          int speedPWM = valueStr.toInt();
          vehicleLogic._set_speed(speedPWM);
          // forward(pwmChannel,motor1Pin1,motor1Pin2,speedPWM);
          // Do something with 'UP' value (upValue)
          // Serial.println("Processing 'UP' value: " + String(upValue));
        } 
        else if (command == "STEER") {
          int steerPWM = valueStr.toInt();
          vehicleLogic._set_steering(steerPWM);

          // ledcWrite(1, steerValue);   
        } 

        else if (command == "UP") {
          // Serial.println("Processing 'UP' : ");

          vehicleLogic._set_Up();

          // ledcWrite(1, steerValue);   
        }else if (command == "DE") {
          vehicleLogic._set_Down();

          // ledcWrite(1, steerValue);   
        } 
        else if (command == "LEFT") {
          vehicleLogic._set_veer_left();

          // ledcWrite(1, steerValue);   
        }else if (command == "RIGHT") {
          vehicleLogic._set_veer_right();
          // ledcWrite(1, steerValue);   
        }

        else if (command == "BACKWARD") {
          int mode = valueStr.toInt();
          if (mode == 1){
            // Serial.print("Forward");
            isforward = true;  
            forward(pwmChannel,motor1Pin1,motor1Pin2,10);
            delay(2);
            forward(pwmChannel,motor1Pin1,motor1Pin2,10);

          }
          else{
            // Serial.print("backward");
            isforward = false;
            backward(pwmChannel,motor1Pin1,motor1Pin2,10);
            delay(2);
            backward(pwmChannel,motor1Pin1,motor1Pin2,10);

          }
          // Serial.print(mode);
        }

        else if (command == "STOP") {
          
          vehicleLogic._set_stop();
          Stop(pwmChannel,motor1Pin1,motor1Pin2);
          forward(pwmChannel,motor1Pin1,motor1Pin2,0);

          // backward(pwmChannel,motor1Pin1,motor1Pin2,speedPWM);
        }
      }
    // Serial.println("Server response: " + response);
    vTaskDelay(pdMS_TO_TICKS(20)); // Run this task every 20 ms (50 Hz)

    
  }
}

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void setup_Connection_Wifi(){
  // Connect to WiFi network
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println(F(""));
  Serial.println(F("Wifi connected with IP:"));
  Serial.println(WiFi.localIP());


  // Connect client_send  to the PC server
  Serial.print("Connecting client_send to PC server...");
  while (!client_send.connect(pc_ip, pc_port_send)) {
    delay(1000);
    Serial.print(".");
  }

  Serial.print("Connecting client_receive to PC server...");
  while (!client_receive.connect(pc_ip, pc_port_receive)) {
    delay(1000);
    Serial.print(".");
  }

}


// void setup_Acceleometer(){
//   if(!accel.begin())
//   {
//     /* There was a problem detecting the ADXL345 ... check your connections */
//     Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
//     client_send.print("NO:Accel");
//     while(1);
//   }

//   /* Set the range to whatever is appropriate for your project */
//   accel.setRange(ADXL345_RANGE_16_G);
// }
void setup_Motor() {

  // sets the pins as outputs:
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);

  // configure LED PWM functionalitites
  ledcSetup(pwmChannel, freq, resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(enable1Pin, pwmChannel);

  //forward
  isforward = true;
  digitalWrite(motor1Pin1,HIGH); //Motor A Rotate Clockwise
  digitalWrite(motor1Pin2,LOW);
  // testing
  // Serial.print("Testing DC Motor...");
}

void setup_Encoder() {

  //Use statusPin to flash along with interrupts
  pinMode(encoder_pin, INPUT);

  //Triggers on RISING (change from HIGH to LOW)
  attachInterrupt(digitalPinToInterrupt(encoder_pin), Encoder_counter, RISING);
  // Initialize
  pulses = 0;
  rpm = 0;
  speed = 0;
  timeold = 0;
}

void Encoder_counter() {
  //Update count
  pulses++;
}

void loop_Encoder(long duration)
{
 
  //Because we use serial.print so we need to detach Interrupt 
  // detachInterrupt(encoder_pin);
  //Note that this would be 60*1000/(millis() - timeold)*pulses if the interrupt
  //happened once per revolution
  rpm = ((60 * 1000 / diskslots )/ duration)* pulses;

  // float linearSpeed = (PI * diameter_wheel * rpm) / 60.0; // Linear speed in cm/s

  speed = (pulses * distancePerPulse) / (duration / 1000.0); // Calculate speed in cm/s


  //Write it out to serial port
  // Serial.print("speed,");
  // Serial.println(speed);

  // Serial.print("linera speed = ");
  // Serial.println(linearSpeed);
  // Serial.print("pulses = ");
  // Serial.println(pulses);
  pulses = 0;

  
}

//
// void SetAngleServoMotor(int angleDegree) {
  
//   if (numberESP == 2){
//     int value = map(angleDegree , 0 , 130 , 92 , 486);
//     ledcWrite(channel_servo, value);   

//   }else{
//     int value = map(angleDegree , 20 , 140 , 138 , 482);
//     ledcWrite(channel_servo, value);   

//   }

// }


void setup_Servo() {
  // freq = 50 ; resolution 12 bits; 
  ledcSetup(channel_servo,50,12);
  // channel_servo = PWM channel
  ledcAttachPin(SERVO_PIN,channel_servo);
  delay(10);
  ledcWrite(channel_servo, vehicleLogic.PWMSteering_Center);

  // SetAngleServoMotor(90);
}

// Track connectivity de wifi , if disconnect try to reconnect
void KeepTrackWifiConnect() {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(ssid, password);  
  }
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  yield();
}



void loop(){


  if (millis() >= vehicleLogicTimer) {   // loopSpeed milliseconds since last ping, do another ping.
    vehicleLogicTimer += loopSpeed; 

    vehicleLogic.get_next_speed_steering_data(current_speedPWM , current_steeringPWM);
    // Serial.print("current_steeringPWM");
    // Serial.println(current_steeringPWM);


    // MOTOR CONTROL
    ledcWrite(pwmChannel, current_speedPWM);   
    // Servo CONTROL
    ledcWrite(channel_servo, current_steeringPWM);

  }

  if (millis() - timeold >= deltaTime_GetData){  /*Uptade every one second, this will be equal to reading frecuency (Hz).*/

    loop_Encoder( millis() - timeold );
    // ultrasonic.calculatDistance();



      // Serial.print("Distance = ");
    // Serial.println(ultrasonic.getDistance());

    timeold = millis();

  }

  if (!sonar_sent && ping_success) {
    distance = echo_time * US_TO_CM;
    update_distance_estimate();
    // send_sonar_reading();
    sonar_sent = true;
  }

  // Measure distance every sonar_interval
  if ((millis() - sonar_time) >= max(sonar_interval, MAX_SONAR_TIME)) {
    if (!sonar_sent && !ping_success) {  // Send max val if last ping was not returned
      distance = MAX_SONAR_DISTANCE;
      update_distance_estimate();
      // send_sonar_reading();
      sonar_sent = true;
    }
    sonar_time = millis();
    sonar_sent = false;
    send_ping();
  }


}
