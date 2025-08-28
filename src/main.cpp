#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h> 
#include <iostream>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "MadgwickAHRS.h"
#include <HX711_ADC.h>
#include <String>

//wiring
// #define loadCellCalibrationPin 17
#define COUNT_LOW 1800 //0 deg
#define COUNT_HIGH 8100 //180 deg
#define SDA 21
#define SCL 22
#define servoPin 2

//load cell calibration
bool loadCellCalibrationFlag = false;
long scaleFactor = 0;
float newCalFactor = -42203.69922;

//HX711 wiring
const int loadCell_DOUT_PIN = 13;
const int loadCell_SCK_PIN = 14;

//loadCell Constants
float weight = 0;

//servo calibration
bool servoCal = false;
int duty = 0;
const uint8_t PWMChannel = 0;
const uint8_t PWMResolution = 16;
const uint32_t PWMFrequency = 50; //manufacturing spec


//Constants
float countsPerDegree = (COUNT_HIGH-COUNT_LOW)/180;
float rad2Deg = 180/PI;

// === PID Parameters ===
float Kp = 1.0;
float Ki = 0;
float Kd = 0;
const float setpoint = 90;  // Desired pitch angle (e.g., level flight = 0 deg)
float currentAngle = 0;
volatile float pidOutput = 0;

// === Internal State ===
float error, previousError = 0.0;
float integral = 0.0;
float derivative = 0.0;
float dt = 0.02; // 20 ms

//ISR Def
volatile bool controlFlag = false;

//IMU yaw calcs
uint32_t t0 = 0;
volatile uint32_t ts = 0;
float theta_i = 0;
bool firstRead = true;
float previousTheta = 0;
float bias = 0;
bool biasFlag = false;

//objects
hw_timer_t *My_timer = NULL; 
Adafruit_MPU6050 mpu;
Madgwick filter; 
HX711_ADC loadCell(loadCell_DOUT_PIN,loadCell_SCK_PIN);

// === State Machine ===
enum SystemMode {
  MODE_LOADCELL_CAL,
  MODE_SERVO_CAL,
  MODE_IMU_BIAS,
  MODE_RUN
};

// === IMU Setup ===
void initializeIMU()
{
Wire.begin(21,22);
      while (!Serial);
      delay(10);
    Serial.println("Adafruit MPU6050 test!");
    if (!mpu.begin())
    {
      Serial.println("Failed to find MPU6050 chip");

      while (1) {
        delay(10);
      }
    }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
}
inline float dt_seconds(uint32_t now, uint32_t prev) {
    return (float)((uint32_t)(now - prev)) * 1e-6f;
}
float computePID(float current)
{
  error = setpoint - current;
  integral += error *dt;

  //Anti-windup clamp
  if (integral > 100.0) integral = 100;
  if (integral < -100.0) integral = -100;

  derivative = (error -previousError) / dt;
  //compute PID
  float output = Kp * error + Ki * integral + Kd * derivative; 
  previousError = error; 
  return output; 
}
void setServo(float pidOutput)
{
  float servoAngle = constrain(pidOutput, 10, 170);//fux; 0 is the min, 180 is the max. 
  duty = servoAngle*countsPerDegree;
  ledcWrite(0,duty);
  Serial.print("IMU ANGLE: "); Serial.print(theta_i);
  Serial.print(" Servo angle: "); Serial.print(servoAngle);
  Serial.print(" Error :"); Serial.print(error);
  Serial.print(" PID OUTPUT: "); Serial.println(pidOutput);
  Serial.print("Load Cell: "); Serial.println(weight);
}
void servoRawCount()
{
  bool loopFlag = true;
  while(loopFlag)
  {
    Serial.println("Enter the value you would like to test in microseconds");
    while (Serial.available()  == 0){}
    if (Serial.available() > 0)
    {
      String inputString =Serial.readStringUntil('\n');
      String exit = "exit";
      inputString.trim();  
      if(inputString == exit)
      {
        loopFlag = false; 
        servoCal = true;        
      }
      else
      {
        uint32_t count = constrain(inputString.toInt(), COUNT_LOW, COUNT_HIGH); //count for Servo
        if (count == 1800)
        {
          ledcWrite(PWMChannel, count);
          theta_i = 0;
        }
        Serial.println(count);
        ledcWrite(PWMChannel, count);  
        // sensors_event_t a,g,t; 
        // mpu.getEvent(&a, &g, &t);
        // float dt = dt_seconds(ts, t0);
        // t0 = ts;
        // theta_i += (g.gyro.z - bias)* dt;            // integrate yaw
        // previousTheta = theta_i;
        // theta_i = theta_i;
        // Serial.print("Theta = "); Serial.println(theta_i*180/PI);

      }
       

    }
  }
}
void IRAM_ATTR controlLoopISR()
{
  controlFlag  = true;
  ts = micros();
}
float calibrate(HX711_ADC &loadCell, bool &loadCellCalibrationFlag) {
  float known_weight = 0;  // in kg
  Serial.println("Starting calibration...");
  Serial.println("Taring... Remove all weight from the scale.");
  loadCell.setCalFactor(1.0); // temporarily set to 1.0
  loadCell.tareNoDelay();
  Serial.println("Tare complete.");
  // Wait for tare to complete
  while (!loadCell.getTareStatus()) 
  {
    loadCell.update();
  }
  Serial.println("Enter calibration weight: ");
  if(Serial.available())
  {
    known_weight = Serial.parseFloat();
    Serial.read();
  }
  Serial.println("Press enter key to begin calibration.");
  while (Serial.available() == 0);  // wait for keypress
  Serial.read();  // clear buffer
  Serial.println("Stabilizing reading...");
  for (int i = 0; i < 50; i++) loadCell.update();
  loadCell.refreshDataSet();  // ensures fresh average
  newCalFactor = loadCell.getNewCalibration(known_weight);
  Serial.print("New calibration factor: ");
  Serial.println(newCalFactor, 5);
  loadCell.setCalFactor(newCalFactor);
  Serial.println("Would you like to store this value in EEPROM (y/n)?"); 
  if(Serial.available())
  {
    String test = Serial.readStringUntil('\n');
    Serial.println("if you're reading this the code works"); 
  }
  else
  {
    Serial.println("it didn't work");
  }
  Serial.println("Calibration complete. Press any key to continue.");
  while (Serial.available() == 0);
  Serial.read();

  loadCellCalibrationFlag = true;
  return newCalFactor;
}


SystemMode mode = MODE_LOADCELL_CAL;  // start here

void serialCommand()
{
  if (!Serial.available())
  {
    String cmd = " ";
    Serial.println("Enter the mode to start in.");
    while(Serial.available() == 0)
    {
        cmd = Serial.readStringUntil('\n');
    }
    cmd.trim();

    if(cmd.equalsIgnoreCase("loadcal"))
    {
      mode = MODE_LOADCELL_CAL;
      Serial.println("Switched to Load Cell Calibration"); 
    }
    else if (cmd.equalsIgnoreCase("servocal")) {
      mode = MODE_SERVO_CAL;
      Serial.println("Switched to Servo Calibration");
    }
    else if (cmd.equalsIgnoreCase("bias")) {
      mode = MODE_IMU_BIAS;
      Serial.println("Switched to IMU Bias Calibration");
    }
    else if (cmd.equalsIgnoreCase("run")) {
      mode = MODE_RUN;
      Serial.println("Switched to RUN mode");
    }
    else if (cmd.startsWith("kp ")) {
      Kp = cmd.substring(3).toFloat();
      Serial.print("New Kp = "); Serial.println(Kp);
    }
    else if (cmd.startsWith("kd ")) {
      Kd = cmd.substring(3).toFloat();
      Serial.print("New Kd = "); Serial.println(Kd);
    }
    else if (cmd.startsWith("ki ")) {
      Ki = cmd.substring(3).toFloat();
      Serial.print("New Ki = "); Serial.println(Ki);
    }
    else {
      Serial.print("Unknown command: "); Serial.println(cmd);
    }
  }
}



void setup(void) {

  Serial.begin(115200);
  serialCommand();
  // Init hardware
  loadCell.begin();
  // pinMode(loadCellCalibrationPin, INPUT); 
  ledcSetup(PWMChannel, PWMFrequency, PWMResolution);
  ledcAttachPin(servoPin, PWMChannel);
  ledcWrite(PWMChannel, (COUNT_HIGH-COUNT_LOW)/2);

  // Timer ISR
  My_timer = timerBegin(0,80,true);
  timerAttachInterrupt(My_timer, &controlLoopISR, true);
  timerAlarmWrite(My_timer, 20000, true);
  timerAlarmEnable(My_timer);
}

void loop() {

  switch (mode) {
    case MODE_LOADCELL_CAL: {
      if (!loadCellCalibrationFlag) {
        calibrate(loadCell, loadCellCalibrationFlag);
      } 
      if (loadCellCalibrationFlag) {
        mode = MODE_SERVO_CAL;
      }
      break;
    }

    case MODE_SERVO_CAL: {
      if (!servoCal) {
        servoRawCount();    // blocks until “exit” entered
      } else {
        mode = MODE_IMU_BIAS;
      }
      break;
    }

    case MODE_IMU_BIAS: {
      if (!biasFlag) {
        Serial.println("Calibrating Bias...");
        double biasSum = 0;
        for (int i=0; i<500; i++) {
          sensors_event_t a,g,t;
          mpu.getEvent(&a,&g,&t);
          biasSum += g.gyro.z;
          delay(2);
        }
        bias = biasSum/500;
        Serial.print("Bias: "); Serial.println(bias,6);
        biasFlag = true;
        mode = MODE_RUN;
      }
      break;
    }

    case MODE_RUN: {
      if (loadCell.update()) {
        weight = loadCell.getData();
      }
      if (controlFlag) {
        controlFlag = false;
        sensors_event_t a,g,t;
        mpu.getEvent(&a,&g,&t);

        float dt = dt_seconds(ts,t0);
        t0 = ts;
        theta_i += (g.gyro.z - bias)*dt;

        float u = computePID(theta_i*rad2Deg);
        setServo(u);
      }
      break;
    }
  }
}