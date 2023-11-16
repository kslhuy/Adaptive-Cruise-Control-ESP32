#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H
#include <Arduino.h>
#include <analogWrite.h>

void forward(int PWM1,int AIN1,int AIN2,int Speed);
void backward(int PWM1,int AIN1,int AIN2,int Speed);
void Stop(int PWM1,int AIN1,int AIN2);

#endif
