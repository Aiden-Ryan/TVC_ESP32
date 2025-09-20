#include <Arduino.h>
#include <Wire.h>
#include <iostream>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "MadgwickAHRS.h"
#include <HX711_ADC.h>
#include <String>
//Logging or Dev
#define LOG
// #define DEV
//wiring
#define loadCellCalibrationPin 17
#define COUNT_LOW 1800 //0 deg
#define COUNT_HIGH 8100 //180 deg
#define SDA 21
#define SCL 22
#define servoPin 2
#define servoPin2 4
//load cell calibration
bool loadCellCalibrationFlag = false;
long scaleFactor = 0;
float newCalFactor = -41711.60156;
//HX711 wiring
const int loadCell_DOUT_PIN = 13;
const int loadCell_SCK_PIN = 14;
//loadCell Constants
float weight = 0;
//servo calibration
bool servoCal = false;
int duty = 0;
const uint8_t PWMChannel1 = 0;
const uint8_t PWMChannel2 = 1;
const uint8_t PWMResolution = 16;
const uint32_t PWMFrequency = 50; //manufacturing spec
//Constants
float countsPerDegree = (COUNT_HIGH-COUNT_LOW)/180;
float rad2Deg = 180/PI;
bool runOnce =  false;
// === PID Parameters ===
float Kp = 2.75;
float Ki = 0;
float Kd = 0;
const float pitchSetpoint = 0;  // Desired pitch angle (e.g., level flight = 0 deg)
const float rollSetpoint = 0;
float currentPitchAngle = 0;
float currentRollAngle  = 0;
volatile float pidOutput = 0;
// === Internal State ===
float error, previousError = 0.0;
float integral = 0.0;
float derivative = 0.0;
float dt = 0.02; // 20 ms
float rollError, rollPreviousError = 0.0; 
float rollIntegral = 0.0; 
float rollDerivative = 0.0; 
//ISR Def
volatile bool controlFlag = false;
//IMU calcs
uint32_t t0 = 0;
volatile uint32_t ts = 0;
float theta_yaw = 0;
float theta_pitch = 0;
float theta_roll = 0;
bool firstRead = true;
float bias = 0;
float biasX = 0;
bool biasFlag = false;
//objects
hw_timer_t *My_timer = NULL; 
Adafruit_MPU6050 mpu;
Madgwick filter; 
HX711_ADC loadCell(loadCell_DOUT_PIN,loadCell_SCK_PIN);
//FUNCTIONS
//dt function that returns a float
inline float dt_seconds(uint32_t now, uint32_t prev) 
{
    return (float)((uint32_t)(now - prev)) * 1e-6f;
}
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
float computePID(float current, float setpoint)
{
  error = setpoint - current;
  integral += error *dt;
  //Anti-windup clamp
  if (integral > 45) integral = 45;
  if (integral < -45) integral = -45;
  derivative = (error - previousError) / dt;
  //compute PID
  float pidOutput = Kp * error + Ki * integral + Kd * derivative; 
  previousError = error; 
  return (pidOutput); 
}
void setServo(float pidOutput, int channel)
{
  float servoAngle = constrain(pidOutput, -35, 60);//fix; 50 is the min, 110 is the max. 
  duty = servoAngle*countsPerDegree;
  ledcWrite(channel,duty+4500);
  // Serial.print("IMU ANGLE: "); Serial.print(theta_yaw*rad2Deg);
  // Serial.print(" SERVO ANGLE: "); Serial.print(servoAngle);
  // Serial.print(" ERROR :"); Serial.print(error);
  // Serial.print(" PID OUTPUT: "); Serial.print(pidOutput);
  // Serial.print(" LOAD CELL: "); Serial.println(weight);
}
//MAIN CODE
// === IMU Setup ===

#ifdef LOG
void setup()
{
  Serial.begin(115200);
  initializeIMU();
  ledcSetup(PWMChannel1, PWMFrequency, PWMResolution);
  ledcSetup(PWMChannel2, PWMFrequency, PWMResolution);
  ledcAttachPin(servoPin, PWMChannel1);
  ledcAttachPin(servoPin2, PWMChannel2);
  ledcWrite(PWMChannel1, 4500); //set servo to 45 deg
  ledcWrite(PWMChannel2, 4500); //set servo to 45 deg (haven't double checked)

}
bool calibrationBias = false;
double biasSum = 0;
double biasSumx = 0;
void loop()
{
  //Sample 500 data points
  if (!calibrationBias)
  {
    Serial.println("Calibrating Bias...");
    for (int i=0; i<500; i++) {
    sensors_event_t a,g,t;
    mpu.getEvent(&a,&g,&t);
    biasSum += g.gyro.y;
    biasSumx += g.gyro.x;
    delay(2);
    }
    calibrationBias = true;
    //calcluate bias for both axes
    bias = biasSum/500;
    biasX = biasSumx/500;
    Serial.print("Bias: "); Serial.println(bias,6);
  }  
  ts = micros();
  sensors_event_t a,g,t;
  mpu.getEvent(&a,&g,&t); //grab IMU data
  float roll = g.gyro.x;
  float pitch = g.gyro.y;
  float dt = dt_seconds(ts,t0);
  t0 = ts;    
  //integrate pitch and roll (angular velocities) to get theta
  theta_pitch += (pitch- bias)*dt;
  theta_roll += (roll- biasX)*dt;
  float PIDpitch = computePID(-theta_pitch*RAD_TO_DEG, pitchSetpoint);
  setServo(PIDpitch,0);
  float b = computePID(theta_roll*RAD_TO_DEG, rollSetpoint);
  setServo(b,1);

  Serial.print(ts); Serial.print(","); Serial.print(theta_roll*RAD_TO_DEG); Serial.print(","); Serial.print(theta_pitch*RAD_TO_DEG); Serial.print(","); 
  Serial.println(b);
}
#endif
#ifdef DEV
// === State Machine ===
enum SystemMode 
{
  MODE_LOADCELL_CAL,
  MODE_SERVO_CAL,
  MODE_IMU_BIAS,
  MODE_RUN,
  MODE_TUNEPID,
  MODE_IMU_READ,
  DEBUG
};
SystemMode mode = MODE_LOADCELL_CAL;  // start here


void servoRawCount()
{
  bool loopFlag = true;
  while(loopFlag)
  {
    Serial.println("Enter the value you would like to test in microseconds");
    while (Serial.available()  == 0){}
    if (Serial.available() > 0)
    {
      String inputString = Serial.readStringUntil('\n');
      String exit = "exit";
      inputString.trim();  
      if(inputString == exit)
      {
        loopFlag = false; 
        servoCal = true;        
      }
      else
      {
        Serial.read();
        String channelString;
        bool channelFlag = false;
        while (!channelFlag)
        {          
          Serial.println("Enter channel to write to: "); 
          while (Serial.available() == 0) {}
          if (Serial.available() > 0)
          {
            channelString = Serial.readStringUntil('\n'); 
            channelString.trim();
            if ((channelString == "1") || (channelString == "0"))
            {
              channelFlag = true;
            }
            else
            {
              Serial.println("Invalid channel selection.");
            }
          }
        } 
        int channel = channelString.toInt();
        // uint32_t count = inputString.toInt();
        uint32_t count = constrain(inputString.toInt(), 3250, 6000); //count for Servo 4500 and 4500 were chosen to be in the range of +- 15 deg
        if (count == 4500)
        {
          ledcWrite(PWMChannel1, count);
          ledcWrite(PWMChannel2, count);
        }
        Serial.print(count); Serial.print(" "); Serial.println(count/countsPerDegree-4500/countsPerDegree);
        ledcWrite(channel, count);  
      }
    }
  }
}
void IRAM_ATTR controlLoopISR()
{
  controlFlag  = true;
  ts = micros();
}
float calibrate(HX711_ADC &loadCell) 
{
  float known_weight = 0;  // in kg
  Serial.println("Starting calibration...");
  Serial.println("Taring... Remove all weight from the scale.");
  loadCell.setCalFactor(1.0); // temporarily set to 1.0
  loadCell.tareNoDelay();
  // Wait for tare to complete
  while (!loadCell.getTareStatus()) 
  {
    loadCell.update();
  }
  Serial.println("Tare complete.");
  Serial.println("Enter calibration weight: ");
  while(Serial.available() == 0){}
  if(Serial.available() > 0)
  {
    known_weight = Serial.parseFloat();
    Serial.read();
  }
  Serial.println("Known weight: " + String(known_weight)); Serial.read();
  Serial.println("Place known weight on load cell and press enter key to begin calibration.");
  while (Serial.available() == 0);  // wait for keypress
  Serial.read();  // clear buffer
  Serial.println("Stabilizing reading...");
  for (int i = 0; i < 50; i++) loadCell.update();
  loadCell.refreshDataSet();  // ensures fresh average
  newCalFactor = loadCell.getNewCalibration(known_weight);
  Serial.print("New calibration factor: ");
  Serial.println(newCalFactor, 5);
  loadCell.setCalFactor(newCalFactor);
  Serial.println("Calibration complete. Set calibration pin to high. Press any key to continue."); Serial.read();
  while (Serial.available() == 0){};
  Serial.read();
  loadCellCalibrationFlag = true;
  return newCalFactor;
}
void serialCommand(String mode = " ")
{
  if (!Serial.available())
  {
    String cmd = " ";
    Serial.println("Enter the mode to start in.");
    while(Serial.available() == 0){}
    if (Serial.available() > 0){
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
      if (!My_timer)
      {
        timerAlarmEnable(My_timer);
      }
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
    else if (cmd.equalsIgnoreCase("debug")){
      mode = DEBUG;
      Serial.println("Switched to debug mode.");
    }
    else if (cmd.equalsIgnoreCase("imuRead")){
      mode = MODE_IMU_READ;
    }
    else {
      Serial.print("Unknown command: "); Serial.println(cmd);
      serialCommand();
    }
  }
  else
  { 
    Serial.println("You passed the peek function");
  }
}
void setup(void) 
{
  
  // Timer ISR
  My_timer = timerBegin(0,80,true); //80 MHz is the prescaler
  timerAttachInterrupt(My_timer, &controlLoopISR, true);
  timerAlarmWrite(My_timer, 20000, true);
  timerAlarmEnable(My_timer);
  Serial.begin(115200);

  pinMode(loadCellCalibrationPin, INPUT_PULLDOWN); 
  Serial.read(); //Clear buffer
  loadCell.begin();
  initializeIMU();
  serialCommand();
  
  // Init hardware
  ledcSetup(PWMChannel1, PWMFrequency, PWMResolution);
  ledcSetup(PWMChannel2, PWMFrequency, PWMResolution);
  ledcAttachPin(servoPin, PWMChannel1);
  ledcAttachPin(servoPin2, PWMChannel2);
  ledcWrite(PWMChannel1, 4500); //set servo to 45 deg
  ledcWrite(PWMChannel2, 4500); //set servo to 45 deg (haven't double checked)

}
void defaultState()
{
  ledcWrite(PWMChannel1, 4500); //set servo to 45 deg
  ledcWrite(PWMChannel2, 4500); //set servo to 45 deg
}
void loop() 
{
  if(!runOnce)
  {
    runOnce= true;
    t0 = micros(); 
  }
  switch (mode) {
    case MODE_LOADCELL_CAL: {
      if ((digitalRead(loadCellCalibrationPin) == 0)) {
        calibrate(loadCell);
      } 
      if ((digitalRead(loadCellCalibrationPin) == 1)) {
        mode = MODE_SERVO_CAL;
        Serial.println("Load Cell has been calibrated. Moving to servo calibration");
        loadCell.tareNoDelay();
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
        double biasSumx = 0;
        for (int i=0; i<500; i++) {
          sensors_event_t a,g,t;
          mpu.getEvent(&a,&g,&t);
          biasSum += g.gyro.y;
          biasSumx += g.gyro.x;
          delay(2);
        }
        bias = biasSum/500;
        biasX = biasSumx/500;
        Serial.print("Bias: "); Serial.println(bias,6);
        biasFlag = true;
        serialCommand();
      }
      break;
    }
    case DEBUG:
    {
      ts = micros();
      timerAlarmDisable(My_timer);
      sensors_event_t a,g,t; 
      mpu.getEvent(&a,&g, &t); 
      dt = dt_seconds(ts, t0);
      t0 = ts;
      theta_roll += (g.gyro.x +.09) *dt;
      Serial.println(theta_roll*rad2Deg);
      delay(250);
      if(Serial.available() > 0)
      {
        char c = Serial.peek(); 
        if (c == '0') 
        {
          Serial.read(); 
          theta_roll = 0;
        }
        else if (c == '3')
        {
          Serial.read();
          serialCommand();
        }
      }
    }
    case MODE_RUN: {
      if(Serial.available())
      {
        char c = Serial.peek(); 
        if (c == 'q' || c == 'Q')
        {
          Serial.read();
          defaultState();
          serialCommand();
        }
      }
      if (loadCell.update()) {
        weight = loadCell.getData();
      }
      if (controlFlag) {
        ts = micros();
        controlFlag = false;
        sensors_event_t a,g,t;
        mpu.getEvent(&a,&g,&t); //grab IMU data
        // float yaw = g.gyro.z; 
        float roll = g.gyro.x;
        float pitch = g.gyro.y;
        float dt = dt_seconds(ts,t0);
        t0 = ts;
        // theta_yaw += (yaw - bias)*dt;
        theta_pitch += (pitch- bias)*dt; 
        theta_roll += (roll- bias)*dt;
        Serial.println(theta_roll);
        // float rollPID = computePID(theta_roll*rad2Deg, rollSetpoint);
        float pitchPID = computePID(theta_pitch*rad2Deg, pitchSetpoint); 
        // setServo(rollPID,PWMChannel1);
        // setServo(pitchPID, PWMChannel2);
      }
      break;
    }
    case MODE_TUNEPID: {
      Serial.println("Connect Putty to COM port to begin writing to file. Press enter to continue.");
      while (Serial.available() == 0); Serial.read(); 
    }
    case MODE_IMU_READ:{ //ONLY TO READ IMU
      if(!biasFlag){
        mode = MODE_IMU_BIAS; 
        Serial.println("bias isn't calculated...");
        serialCommand();
        break;
      }
      ts = micros();
      sensors_event_t a,g,t;
      mpu.getEvent(&a,&g,&t); //grab IMU data
      float roll = g.gyro.x;
      float pitch = g.gyro.y;
      float dt = dt_seconds(ts,t0);
      t0 = ts;    
      //integrate pitch and roll (angular velocities) to get theta
      theta_pitch += (pitch- bias)*dt;
      theta_roll += (roll- biasX)*dt;
      Serial.print("ROLL: "); Serial.print(theta_roll*RAD_TO_DEG); Serial.print(" PITCH: "); Serial.println(theta_pitch*RAD_TO_DEG);
    }
  }
}
#endif 