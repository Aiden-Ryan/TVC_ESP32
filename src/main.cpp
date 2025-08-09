#include <Arduino.h>
#include <Wire.h>
#include <LSM6.h>
#include <LIS3MDL.h>
#include "MadgwickAHRS.h"
#include <HX711_ADC.h>
#include <String>

//calibration flag
bool calibrationFlag = 0;
long scaleFactor = 0;

//HX711 wiring
const int loadCell_DOUT_PIN = 13;
const int loadCell_SCK_PIN = 14;


//servo wiring
#define COUNT_LOW 1638
#define COUNT_HIGH 7864
int servoPin = 2;

//IMU wiring
#define SDA 21
#define SCL 22

//Constants
float countsPerDegree = (COUNT_HIGH-COUNT_LOW)/180;

// === PID Parameters ===
float Kp = 1.0;
float Ki = 0;
float Kd = 0;

float setpoint = 0;  // Desired pitch angle (e.g., level flight = 0 deg)
float currentAngle = 0;
volatile float pidOutput = 0;

// === Internal State ===
float error, previousError = 0.0;
float integral = 0.0;
float derivative = 0.0;
float dt = 0.02; // 20 ms

//ISR Def
volatile bool controlFlag = false;

//objects
hw_timer_t *My_timer = NULL; 
LSM6 imu; 
LIS3MDL mag; 
Madgwick filter; 
HX711_ADC loadCell(loadCell_DOUT_PIN,loadCell_SCK_PIN);

// === IMU Setup ===
// Change depending on your IMU
void initializeIMU()
{
  Wire.begin(21,22);//begins I2C protocol
  imu.init(); 
  imu.enableDefault(); 
  
  mag.init(); 
  mag.enableDefault();

  filter.begin(50); //sets 50 Hz refresh rate for quaternion calcs
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
  //compute PID
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

void IRAM_ATTR controlLoopISR()
{
  controlFlag  = true;
}

float calibrate(HX711_ADC &loadCell, bool &calibrationFlag) {
  Serial.println("Starting calibration...");
  Serial.println("Taring... Remove all weight from the scale.");

  loadCell.setCalFactor(1.0); // temporarily set to 1.0
  loadCell.tareNoDelay();

  // Wait for tare to complete
  while (!loadCell.getTareStatus()) {
    loadCell.update();
  }

  Serial.println("Tare complete. Place known weight and press enter to begin calibration.");
  while (Serial.available() == 0);  // wait for keypress
  Serial.read();  // clear buffer

  Serial.println("Stabilizing reading...");
  for (int i = 0; i < 50; i++) loadCell.update();
  loadCell.refreshDataSet();  // ensures fresh average

  float known_weight = 10.0;  // in kg
  float newCalFactor = loadCell.getNewCalibration(known_weight);

  Serial.print("New calibration factor: ");
  Serial.println(newCalFactor, 5);

  loadCell.setCalFactor(newCalFactor);
  Serial.println("Calibration complete. Press any key to continue.");
  while (Serial.available() == 0);
  Serial.read();

  calibrationFlag = true;
  return newCalFactor;
}

void setup()
{
  Serial.begin(115200);
  loadCell.begin();

  //initialize IMU and Servo
  initializeIMU();
  ledcSetup(0, 50, 16);        // 50 Hz, 16-bit; 50 Hz is servo standard. 16 bit is the PWM resolution
  ledcAttachPin(servoPin, 0);  // pin, channel
  ledcWrite(0,45*countsPerDegree);

  //initialize load cell config
  // while (calibrationFlag == false) //only run once
  // {
  //   calibrate(loadCell, calibrationFlag);
  // }  

  //Begin ISR
  My_timer = timerBegin(0,80, true); //arg 1 is the number of timer, arg 2 is the value of the prescaler, last one is a flag fi the c ounter should count up (true) or down (false)
  timerAttachInterrupt(My_timer, &controlLoopISR, true); 
  timerAlarmWrite(My_timer, 20000, true); //runs ISR every 20 ms or 50 Hz
  timerAlarmEnable(My_timer); 
}

void loop()
{
 
  // if (loadCell.update()){
  //   Serial.print("Weight: "); Serial.println(loadCell.getData());
  // }
  // Serial.read(); 
  // if (Serial.available())
  // {
  //   loadCell.tare();
  // }
  // delay(250);
  if (controlFlag) //is true from ISR
  {

    controlFlag = false;
    currentAngle = readIMU()-180;//set baseline angle to zero
    pidOutput = computePID(currentAngle); //get controller output
    setServo(pidOutput);


  }
  
  
}
