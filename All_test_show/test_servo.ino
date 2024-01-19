// int pot = 2;
// int channel_servo = 10;

// // MOTOR Pin
// const int motor1Pin1 = 27; // Pin1 or Pin3 
// const int motor1Pin2 = 26; // Pin2 or Pin4 
// const int enable1Pin = 14; // enable Pin

// // Setting PWM properties
// const int freq = 5000;
// const int pwmChannel = 0;
// const int resolution = 8;
// int dutyCycle = 110;
// void setup() {
//   Serial.begin(115200);
//   // put your setup code here, to run once:
//   // freq = 50 ; resolution 12 bits; 
//   ledcSetup(channel_servo,50,12);
//   // 1 = PWM channel
//   ledcAttachPin(2,channel_servo);

//   setup_Motor();
// }

// void setup_Motor() {

//   // sets the pins as outputs:
//   pinMode(motor1Pin1, OUTPUT);
//   pinMode(motor1Pin2, OUTPUT);
//   pinMode(enable1Pin, OUTPUT);

//   // configure LED PWM functionalitites
//   ledcSetup(pwmChannel, freq, resolution);

//   // attach the channel to the GPIO to be controlled
//   ledcAttachPin(enable1Pin, pwmChannel);

//   digitalWrite(motor1Pin1,HIGH); //Motor A Rotate Clockwise
//   digitalWrite(motor1Pin2,LOW);

//   ledcWrite(pwmChannel, dutyCycle); 
//   // testing
//   // Serial.print("Testing DC Motor...");
// }
// // int value = map(angleDegree , 0 , 130 , 92 , 486);

// void loop() {
   
//   // rotates from 0 degrees to 180 degrees
//   for (int pos = 70; pos <= 115; pos += 1) {
//     // in steps of 1 degree
//     int value = map(pos , 0 , 130 , 92 , 486);

//     // int value = map(pos , 20 , 140 , 138 , 482);  
//     Serial.print("current_degree : ");
//     Serial.print(pos);
//     Serial.print("current_degreePWM : ");
//     Serial.println(value);

//     ledcWrite(channel_servo,value);
//     delay(40); // waits 15ms to reach the position
//   }

//   // rotates from 180 degrees to 0 degrees
//   for (int pos = 115; pos >= 70; pos -= 1) {
//     int value = map(pos , 0 , 130 , 92 , 486);

//     // int value = map(pos , 20 , 140 , 138 , 482);  

//     Serial.print("current_degree : ");
//     Serial.print(pos);
//     Serial.print("current_degreePWM : ");
//     Serial.println(value);

//     ledcWrite(channel_servo,value);
//     delay(40); // waits 15ms to reach the position
//   }




//   // int value = analogRead(pot);
//   // int duty = map(value , 0 , 4095 , 138 , 482);
//   // int degre = map(duty , 138 , 482 ,20 , 140 ); 
//   // Serial.println(degre);

//   // Serial.println(degre);
//   // ledcWrite(1,duty);
// }



#include <Servo.h>

Servo myservo;  // create servo object to control a servo

int potpin = 0;  // analog pin used to connect the potentiometer
int val;    // variable to read the value from the analog pin

void setup() {
  Serial.begin(9600);
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
}

void loop() {
  val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023)
  Serial.println(val);
  val = map(val, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
  myservo.write(val);                  // sets the servo position according to the scaled value
  delay(15);                           // waits for the servo to get there
}

