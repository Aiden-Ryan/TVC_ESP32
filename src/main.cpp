#include <Arduino.h>
#include <Wire.h>
#include <LSM6.h>
#include <LIS3MDL.h>
#include "MadgwickAHRS.h"
#include <String>
#define COUNT_LOW 1638
#define COUNT_HIGH 7864
#define TIMER_WIDTH 16
#define SDA 21
#define SCL 22
int servoPin = 2;

//Constants
float countsPerDegree = 34.6;
// === PID Parameters ===
float Kp = 1.0;
float Ki = 0;
float Kd = 0;

float setpoint = 0;  // Desired pitch angle (e.g., level flight = 0 deg)
float currentAngle = 0;
// === Internal State ===
float error, previousError = 0.0;
float integral = 0.0;
float derivative = 0.0;
float dt = 0.02; // 20 ms
hw_timer_t *My_timer = NULL; 
volatile float pidOutput = 0;
volatile bool controlFlag = false;


LSM6 imu; 
LIS3MDL mag; 
Madgwick filter; 

// === IMU Setup ===
// Change depending on your IMU
void initializeIMU()
{
  Wire.begin(21,22);
  imu.init(); 
  imu.enableDefault(); 
  
  mag.init(); 
  mag.enableDefault();

  filter.begin(50); //50 Hz refresh rate
}
float readIMU() 
{
  imu.read();
  mag.read(); 
  float ax = imu.a.x; 
  float ay = imu.a.y; 
  float az = imu.a.z;
  float gx = imu.g.x * DEG_TO_RAD;
  float gy = imu.g.y * DEG_TO_RAD; 
  float gz = imu.g.z * DEG_TO_RAD; 
  filter.updateIMU(gx, gy, gz, ax, ay, az);
  return filter.getYaw();
}

float computePID(float current)
{
  error = setpoint - current;
  integral += error *dt;

  //Anti-windup clamp
  if (integral > 100.0) integral = 100;
  if (integral < 100.0) integral = -100;

  derivative = (error -previousError) / dt;
  float output = Kp * error + Ki * integral + Kd * derivative; 
  previousError = error; 
  return output; 
}
void setServo(float pidOutput)
{
  float servoAngle = constrain(pidOutput+90, 20, 150);
  ledcWrite(0,servoAngle*countsPerDegree);
  Serial.print("IMU ANGLE: "); Serial.print(currentAngle);
  Serial.print(" Servo angle: "); Serial.print(servoAngle);
  Serial.print(" Error :"); Serial.print(error);
  Serial.print(" PID OUTPUT: "); Serial.println(pidOutput);
}
//read current angle and throw into control loop
void IRAM_ATTR controlLoopISR()
{
  controlFlag  = true;
}

void setup()
{
  Serial.begin(9600);
  initializeIMU();
  ledcSetup(0, 50, 16);        // 50 Hz, 16-bit
  ledcAttachPin(servoPin, 0);  // pin, channel
  ledcWrite(0,45*countsPerDegree);
  //Begin ISR
  My_timer = timerBegin(0,80, true); //arg 1 is the number of timer, arg 2 is the value of the prescaler, last one is a flag fi the c ounter should count up (true) or down (false)
  timerAttachInterrupt(My_timer, &controlLoopISR, true); 
  timerAlarmWrite(My_timer, 20000, true); 
  timerAlarmEnable(My_timer); 
}

void loop()
{

  if (controlFlag)
  {
    unsigned long t0 = micros();
    controlFlag = false;
    currentAngle = readIMU()-180;
    pidOutput = computePID(currentAngle); //get controller output
    setServo(pidOutput);
    unsigned long dt = micros() - t0;

  }
  
  
}
