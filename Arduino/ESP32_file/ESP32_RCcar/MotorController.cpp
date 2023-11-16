#include "MotorController.h"

void forward(int pwmChannel,int AIN1,int AIN2,int dutyCycle)
{
  digitalWrite(AIN1,HIGH); //Motor A Rotate Clockwise
  digitalWrite(AIN2,LOW);


  ledcWrite(pwmChannel, dutyCycle);   

  // analogWrite(PWM1,Speed); //Speed control of Motor A
  //Serial.println("Fwd");
}
void backward(int PWM1,int AIN1,int AIN2,int Speed)
{
  digitalWrite(AIN1,LOW); //Motor A Rotate Clockwise
  digitalWrite(AIN2,HIGH);

  ledcWrite(PWM1,Speed); //Speed control of Motor A
  //Serial.println("Back");
}

void Stop(int PWM1,int AIN1,int AIN2)
{
  digitalWrite(AIN1,LOW); //Motor A Rotate Clockwise
  digitalWrite(AIN2,LOW);

  ledcWrite(PWM1,0); //Speed control of Motor A
  //Serial.println("STOP");
}
