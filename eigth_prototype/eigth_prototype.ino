/*Control Panel---------------------------------------------------------------*/
/*Libraries*/
//ESP8266 Library
#include <ESP8266WiFi.h>

//Async Server & WebSocket Library
#include "IPAddress.h"
#include "Hash.h"
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSocketsServer.h>

//Mesh Library
#include <list>
#include "painlessMesh.h"

//QMC5883L Library
#include <QMC5883L.h>
#include <Wire.h>

//FUSION/MPU6050 Library
#include "Kalman.h"

//PCF8574 Library
#include "PCF8574.h"

//PID Library
#include<PID_v1.h>

/*Control Variables*/
//Serial Controls
#define SERIAL      1       //for configuring Serial on or off  
#define SERIAL_GPIO 0       //to set Tx, Rx pin as GPIOs        

//I2C Controls
#define I2C_COM     1       //for enabling I2C on SDA (D3), SCL (D1)  

//WIFI Controls
#define WIFISCANNER 1       //for enabling WIFI signal scanning 
#define WIFIAPMODE  0       //for configuring own access point  

//Async Server & WebSocket Controlsrpm
#define ASYNCSERVER 0       //for setup of AsyncWebServer
#define WEBSOCKET   1       //for setup of WebSocket

//Mesh Controls
#define MESHNETWORK 1       //for setup of Mesh network z
#define ROOT "807D3A235911" //specify who is root (MAC)
#define ROOTID 975395089    //specify root ID
bool isROOT = 1;

//Battery Controls
#define BATTERYREADER   1   //for detecting battery voltage & percent z
#define BATTERYPRINT    0   //to print battery data in serial

//QMC5883L Controls
#define QMC5883LMODULE  1   //for setup of compass module
#define QMC5883LPRINT   0   //for printing out values
#define QMC_MANUAL_CAL  0   //specify if manual calibration set
#define QMC_AUTO_CAL    1   //specify if robot calibrates itself by rotating

//MPU6050 Controls
#define MPU6050MODULE   1   //for setup of MPU6050 module
#define MPU6050PRINT    0   //for printing out values

//Sensor Fusion Controls
#define FUSION          1   //for setup of fusion data for MPU6050 and QMC5883L
#define FUSIONPRINT     0   //for printing out values
#define FUSIONRAW       1   //to output raw data
#define FUSIONTILT      1   //to calculate data compensated with tilt
#define FUSIONCOMP      1   //to calculate data using complementary filter
#define FUSIONKALMAN    1   //to calculate data using kalman filter
#define FUSIONTEMP      0   //to use temporary YAW during movement

//PCF8574 Controls
#define PCF8574MODULE   1   //for setup of PCF8574 IO Extender
#define PCF8574LED0     0   //to auto blink led 0
#define PCF8574LED1     0   //to auto blink led 1

//Encoder Controls
#define ENCODER_L 1         //for enabling Left encoder on pin D2   z
#define ENCODER_R 1         //for enabling Right encoder on pin D6  zf

//PID Controls
#define PID_CONTINUOUS  1   //for testing PID algorithm on continuous wheel

//Motor Controls
#define MOTOR_CONTROL_L 1   //for enabling/disabling left motor
#define MOTOR_CONTROL_R 1   //for enabling/disabling right motor
#define STARTUP_NORTH   1   //to auto turn to North on startup

//Bump Button Controls
#define BUMP_L  1           //for enabling bump sensor on D4  z
#define BUMP_R  1           //for enabling bump sensor on TX  z
#define BUMP_B  1           //for enabling bump sensor on D0  z

//Matrix Controls
#define MATRIX  1           //for enabling Matrix movement
/*Global Variables*/
#define M_PI 3.14159265358979323846264338327950288
Scheduler taskScheduler;

volatile bool START = 0;
volatile bool gotoRoot = 0;
volatile bool foundNodes = 0;
volatile bool foundRoot = 0;
float prevVis = 0,currVis = 0;
float prevRootVis = 0, currRootVis = 0;
long int mesh_timer = 0;
volatile float currentDegree = 0;
volatile int movedTss = 0;
volatile float currentHighest = 0;
volatile uint32_t currentTarget = 0;
volatile bool MATRIX_bumped = 0;
volatile bool find_Highest = 0;
int currentNodeCount = 0;

volatile int step_counter = 0;
volatile bool getBattery = 0;
volatile bool getQMC5883L = 0;
volatile bool getMPU6050 = 0;
volatile bool getFusion = 0;
volatile bool startScan = 0;
volatile bool getScan = 0;
/*End of Control Panel--------------------------------------------------------*/
/*Replace TXRX pins Code------------------------------------------------------*/
const uint8_t TX = 1; //GPIO1 (TX)
const uint8_t RX = 3; //GPIO3 (RX)

void TXRX_to_GPIO() {
  pinMode(TX, FUNCTION_3);
  pinMode(RX, FUNCTION_3);
}
void TXRX_to_DEFAULT() {
  pinMode(TX, FUNCTION_0);
  pinMode(RX, FUNCTION_0);
}
/*End of TXRX Replace Code----------------------------------------------------*/
/*Battery Voltage Reader Code-------------------------------------------------*/
/* The battery voltage passes through a voltage divider circuit of:
   V_A0 = V_BAT (R1/R1+R2)
   Where
   R1 = 10k ohm
   R2 = 16k ohm
   V_BAT = 8.4V(max) or 7.4V(nominal)
   V_A0 = 3.231V(max) or 2.846V(nominal)
   Note: Battery voltage input is also connected to Buck converter input
   CRITICAL EDIT:
   Since NodeMCU already has a fixed built-in voltage divider of
   Node_A0 = V_A0 (R3/R3+R4)
   Where
   R3 = 100k ohm
   R4 = 220k ohm
   The addition of another voltage divider from above has caused some changes,
   V_A0 = 0.377(V_BAT)
   Node_A0 = 0.3125(V_A0)
           = 0.1178(V_BAT)
   Thus, the concluded expected voltages are:
   V_BAT = 8.4V(max) or 7.4V(nominal)
   V_A0 = 3.167V(max) or 2.789(nominal)   //0.377 V_BAT
   Node_A0 = 0.989V(max) or 0.872V(nominal) //0.3125*0.377 V_BAT
*/
#define R1 10 //external resistor in k
#define R2 16 //external resistor in k
#define R3 100  //built-in resistor in k
#define R4 220  //built-in resistor in k
#define NODEMCU_VOLT 3.33
#define BATTERY_CONSTANT 0.228

const uint8_t BAT_IN = A0;  //set Battery input pin

float BAT_voltage = 0, BAT_percent = 0;

void BATTERY_read();
Task readBattery(1000, TASK_FOREVER, &BATTERY_read);

void BATTERY_Setup() {
  if (BATTERYREADER) {
    taskScheduler.addTask(readBattery);
    readBattery.enable();
  }
}
void BATTERY_read() {
  int analogInput = analogRead(BAT_IN);
  float A0_voltage = ((float)analogInput / 1023) * NODEMCU_VOLT; //convert into 3.3V range
  BAT_voltage = (A0_voltage / 0.377); //convert into 8.4V range
  if (BAT_voltage > BATTERY_CONSTANT)
    BAT_voltage -= BATTERY_CONSTANT;  //then deduct by 0.228V constant due to error
  BAT_percent = (BAT_voltage / 8.4) * 100; //convert into percentage
  if(BATTERYPRINT)
    Serial.printf("raw:%d,A0:%.2f,BAT:%.2f,Prcnt:%.2f%%\n", analogInput, A0_voltage, BAT_voltage, BAT_percent);
  getBattery = 1; //to signify done
}
/*End of Battery Votlage Reader Code------------------------------------------*/
/*Path Loss Model Code--------------------------------------------------------*/
/* The estimated distance is calculated based on RSSI using a modified version of
   Solah's model, which adds in a constant of -3 based on compiled test results.
   The RSSI calculation assumes signal travels in LOS, indoors.
   Seng's (modified Solah's) = PLo + 10*n*log_10(d) + (-3)
   d = 10^( (RSSI-PLo-(-3)) / (10*n) )
   Note: The input, targetRSSI should be in -dBm
*/
const float PLo = 50.667;
const float PLn = 2.667;
const float PLc = -3;

float PL_calcDistance(float targetRSSI) {
  float distance;
  distance = (targetRSSI - PLo - PLc); //(RSSI-PLo-(-3) part
  distance = distance / (10 * PLn); //divide previous by (10*n)
  distance = pow(10, distance);     //10 power of previous result
  return distance;
}
/*End of Path Loss Model Code-------------------------------------------------*/
/*QMC5883L Codes--------------------------------------------------------------*/
#define MAG_CAL_PERIOD 15000  // 15000ms, 15sec
QMC5883L compass;

int16_t MagX, MagY, MagZ, Xmax, Xmin, Ymax, Ymin, Zmax, Zmin;
float Xoffset, Yoffset, Zoffset, declinationAngle = 0;
float rawHeading = 0;
uint32_t QMC5883L_timer;
bool RightHemis = 1;  //initially turn to north, then move to right hemisphere to check

void QMC5883L_print();
Task readCompass(10, TASK_FOREVER, &QMC5883L_read);
Task printCompass(10, TASK_FOREVER, &QMC5883L_print);

void QMC5883L_Setup() {
  if (QMC5883LMODULE) {
    declinationAngle = (0.0 - (19.0 / 60.0) / (180.0 / M_PI)); //calculate declination
    compass.init();
    if (QMC_MANUAL_CAL) {   //if manual set calibration
      compass.setCalibration(3710, -5240, 6042, -5175);
    }
    else if (QMC_AUTO_CAL) { //robot auto rotate to calibrate
      QMC5883L_autocalibrate();
    }
    else {                  //manually calibrate by moving the robot
      QMC5883L_calibrate();
    }
    if (!FUSION && QMC5883LMODULE && QMC5883LPRINT) {
      if (QMC5883LPRINT) {
        taskScheduler.addTask(printCompass);
        printCompass.enable();
      }
      else {
        taskScheduler.addTask(readCompass);
        readCompass.enable();
      }
    }
  }
}
void QMC5883L_read() {     //retrieve data and store in variables
  rawHeading = compass.readHeading();
  compass.readRaw(&MagX, &MagY, &MagZ);
  compass.readCalFull(&Xmax, &Xmin, &Ymax, &Ymin, &Zmax, &Zmin);
  Xoffset = (float)(Xmax + Xmin) / 2;
  Yoffset = (float)(Ymax + Ymin) / 2;
  Zoffset = (float)(Zmax + Zmin) / 2;
  MagX = MagX - Xoffset;
  MagY = MagY - Yoffset;
  MagZ = MagZ - Zoffset;
  getQMC5883L = 1; //to signify done
}
void QMC5883L_print() { //data not fused with MPU6050
  QMC5883L_read();
  Serial.printf("H:%f,", rawHeading);
  Serial.printf("X:%d,x:%d,", Xmax, Xmin);
  Serial.printf("Y:%d,y:%d,", Ymax, Ymin);
  Serial.printf("Z:%d,z:%d\n", Zmax, Zmin);
  Serial.println();
}
void QMC5883L_calibrate() { //adjust for soft/hard iron
  QMC5883L_timer = millis();
  while ( (millis() - QMC5883L_timer) < MAG_CAL_PERIOD) { //calibrate manually 15 sec
    QMC5883L_read();
    Serial.printf("X:%d,x:%d,Y:%d,y:%d,Z:%d,z:%d\n", Xmax, Xmin, Ymax, Ymin, Zmax, Zmin);
    yield();
  }
}
void PID_Cali_Rotate();
void QMC5883L_autocalibrate(){
  PID_Cali_Rotate();
}
/*End of QMC5883L Codes-------------------------------------------------------*/
/*MPU6050 Code----------------------------------------------------------------*/
// MPU6050 Slave Device Address
const uint8_t MPU6050SlaveAddress = 0x68;

// sensitivity scale factor respective to full scale setting provided in datasheet
const uint16_t AccelScaleFactor = 16384;
const uint16_t GyroScaleFactor = 131;

// MPU6050 few configuration register addresses
const uint8_t MPU6050_REGISTER_SMPLRT_DIV   =  0x19;
const uint8_t MPU6050_REGISTER_USER_CTRL    =  0x6A;
const uint8_t MPU6050_REGISTER_PWR_MGMT_1   =  0x6B;
const uint8_t MPU6050_REGISTER_PWR_MGMT_2   =  0x6C;
const uint8_t MPU6050_REGISTER_CONFIG       =  0x1A;
const uint8_t MPU6050_REGISTER_GYRO_CONFIG  =  0x1B;
const uint8_t MPU6050_REGISTER_ACCEL_CONFIG =  0x1C;
const uint8_t MPU6050_REGISTER_FIFO_EN      =  0x23;
const uint8_t MPU6050_REGISTER_INT_ENABLE   =  0x38;
const uint8_t MPU6050_REGISTER_ACCEL_XOUT_H =  0x3B;
const uint8_t MPU6050_REGISTER_SIGNAL_PATH_RESET  = 0x68;

int16_t AccelX, AccelY, AccelZ, Temperature, GyroX, GyroY, GyroZ;
float Ax, Ay, Az, T, Gx, Gy, Gz;

Task readMPU(10, TASK_FOREVER, &MPU6050_read);
Task printMPU(10, TASK_FOREVER, &MPU6050_print);

void I2C_Write(uint8_t deviceAddress, uint8_t regAddress, uint8_t data) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.write(data);
  Wire.endTransmission();
}
void MPU6050_Setup() {
  if (MPU6050MODULE) {
    delay(150); //let module settle down first
    I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SMPLRT_DIV, 0x07);
    I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_1, 0x01);
    I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_2, 0x00);
    I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_CONFIG, 0x00);
    I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_GYRO_CONFIG, 0x00);//set +/-250 degree/second full scale
    I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_CONFIG, 0x00);// set +/- 2g full scale
    I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_FIFO_EN, 0x00);
    I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_INT_ENABLE, 0x01);
    I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SIGNAL_PATH_RESET, 0x00);
    I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_USER_CTRL, 0x00);
  }
  if (!FUSION && MPU6050MODULE) {
    if (MPU6050PRINT) {
      taskScheduler.addTask(printMPU);
      printMPU.enable();
    }
    else {
      taskScheduler.addTask(readMPU);
      readMPU.enable();
    }
  }
}
void MPU6050_readRaw(uint8_t deviceAddress, uint8_t regAddress) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, (uint8_t)14);
  AccelX = (((int16_t)Wire.read() << 8) | Wire.read());
  AccelY = (((int16_t)Wire.read() << 8) | Wire.read());
  AccelZ = (((int16_t)Wire.read() << 8) | Wire.read());
  Temperature = (((int16_t)Wire.read() << 8) | Wire.read());
  GyroX = (((int16_t)Wire.read() << 8) | Wire.read());
  GyroY = (((int16_t)Wire.read() << 8) | Wire.read());
  GyroZ = (((int16_t)Wire.read() << 8) | Wire.read());
}
void MPU6050_read() {
  MPU6050_readRaw(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_XOUT_H);

  Ax = (float)AccelX / AccelScaleFactor;
  Ay = (float)AccelY / AccelScaleFactor;
  Az = (float)AccelZ / AccelScaleFactor;
  T  = (float)Temperature / 340 + 36.53; //temperature formula
  Gx = (float)GyroX / GyroScaleFactor;
  Gy = (float)GyroY / GyroScaleFactor;
  Gz = (float)GyroZ / GyroScaleFactor;
  getMPU6050 = 1;
  MPU6050_checkbump();
}
void MPU6050_print() {
  MPU6050_read();
//  Serial.printf("Ax:%f,Ay:%f,Az:%f\n", Ax, Ay, Az);
//  Serial.printf("T:%f,", T);
//  Serial.printf("Gx:%f,Gy:%f,Gz:%f\n", Gx, Gy, Gz);
}
void MPU6050_checkbump() {
  float yAxis = (float) Ay * (-1.0);
  float magnitude = sqrt(Ax*Ax + yAxis*yAxis);
  float angle = atan(yAxis/Ax)*57.2958;
  if(angle <0){
    angle = 360.0 + angle;
  }
  if(magnitude >= 2){
    Serial.printf("Mag:% 4.2f, angle: % 4.2f\n",magnitude,angle);
    Serial.println(yAxis);
  }
}
/*End of MPU6050 Code---------------------------------------------------------*/
/*Sensor Fusion Code----------------------------------------------------------*/
Kalman kalmanX, kalmanY, kalmanZ; // Create the Kalman instances

float tiltHeading;
float AngleX, AngleY, AngleZ;
float compAngleX, compAngleY, compAngleZ;
float kalAngleX, kalAngleY, kalAngleZ;
float roll, pitch, yaw;
float estAngleZrate, estHeading, estDrift, maxDrift, posOffset, negOffset;

uint32_t FUSION_timer, TEMP_timer;

Task readFUSION(10, TASK_FOREVER, &FUSION_read);
Task printFUSION(10, TASK_FOREVER, &FUSION_print);

void FUSION_Setup() {
  if (FUSION && QMC5883LMODULE && MPU6050MODULE) {
    QMC5883L_read();
    MPU6050_read();
    updatePitchRoll();
    updateYaw();

    kalmanX.setAngle(roll);
    AngleX = roll;
    compAngleX = roll;

    kalmanY.setAngle(pitch);
    AngleY = pitch;
    compAngleY = pitch;

    kalmanZ.setAngle(yaw);
    AngleZ = yaw;
    compAngleZ = yaw;

    if (FUSIONTEMP) {
      measureDrift();
    }
    FUSION_timer = micros();
    TEMP_timer = micros();
    if (FUSION) {
      if (FUSIONPRINT) {
        taskScheduler.addTask(printFUSION);
        printFUSION.enable();
      }
      else {
        taskScheduler.addTask(readFUSION);
        readFUSION.enable();
      }
    }
  }
}
void FUSION_read() {
  QMC5883L_read();
  MPU6050_read();

  float dt = (float)(micros() - FUSION_timer) / 1000000; // Calculate delta time
  FUSION_timer = micros();

  updatePitchRoll();
  float AngleXrate = Gx / 131.0;
  float AngleYrate = Gy / 131.0;

  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    AngleX = roll;
    compAngleX = roll;
    kalAngleX = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, AngleXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    AngleYrate = -AngleYrate; // Invert rate, so it fits the restricted accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, AngleYrate, dt);

  updateYaw();
  float AngleZrate = AngleZ / 131.0;

  if ((yaw < -90 && kalAngleZ > 90) || (yaw > 90 && kalAngleZ < -90)) {
    kalmanZ.setAngle(yaw);
    AngleZ = yaw;
    compAngleZ = yaw;
    kalAngleZ = yaw;
  } else
    kalAngleZ = kalmanZ.getAngle(yaw, AngleZrate, dt); // Calculate the angle using a Kalman filter

  AngleXrate += AngleXrate * dt;
  AngleYrate += AngleYrate * dt;
  AngleZrate += AngleZrate * dt;

  compAngleX = 0.93 * (compAngleX + AngleXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + AngleYrate * dt) + 0.07 * pitch;
  compAngleZ = 0.93 * (compAngleZ + AngleZrate * dt) + 0.07 * yaw;

  // Reset the gyro angles when they has drifted too much
  if (AngleX < -180 || AngleX > 180)
    AngleX = kalAngleX;
  if (AngleY < -180 || AngleY > 180)
    AngleY = kalAngleY;
  if (AngleZ < -180 || AngleZ > 180)
    AngleZ = kalAngleZ;

  rawHeading = noTiltCompensate();
  tiltHeading = tiltCompensate();

  if (tiltHeading == -1000)
    tiltHeading = rawHeading;

  rawHeading -= declinationAngle;
  tiltHeading -= declinationAngle;

  rawHeading = correctAngle(rawHeading);
  tiltHeading = correctAngle(tiltHeading);

  rawHeading = rawHeading * 180 / M_PI;
  tiltHeading = tiltHeading * 180 / M_PI;

  if (FUSIONTEMP) {
    FUSION_tempyaw();
  }
  getFusion = 1; //to signify done
}
void FUSION_tempyaw() {
  MPU6050_read();
  float dt = (float)(micros() - TEMP_timer) / 1000000; // Calculate delta time
  TEMP_timer = micros();
  estAngleZrate = Gz / 131.0;
  estAngleZrate += estAngleZrate * dt;
  if ((floorf(estAngleZrate * 100) / 100) > 0 && estAngleZrate > maxDrift) { //if maxDrift is positive, estAngleZ>0 & >max, then will be true, if negative, if >0 then true
    estHeading += (((floorf(estAngleZrate * 100) / 100) + posOffset + 0.1) * dt);
  }
  else if ((ceilf(estAngleZrate * 100) / 100) < 0 && estAngleZrate < maxDrift) { //if maxDrift is positive, angleZ<0 then true, if negative, angleZ<0 & <max then true
    estHeading += (((ceilf(estAngleZrate * 100) / 100) + negOffset - 0.1) * dt);
  }
}
void FUSION_print() {
  FUSION_read();
  if (FUSIONRAW) {
    Serial.printf("% d,% d,% d,% 4.2f,", MagX, MagY, MagZ, rawHeading);
  }
  if (FUSIONTILT) {
    Serial.printf("% 4.2f,% 4.2f,% 4.2f,% 4.2f,", Ax, Ay, Az, tiltHeading);
  }
  if (FUSIONCOMP) {
    Serial.printf("% 4.2f,% 4.2f,% 4.2f,", compAngleX, compAngleY, compAngleZ);
  }
  if (FUSIONKALMAN) {
    Serial.printf("% 4.2f,% 4.2f,% 4.2f,", kalAngleX, kalAngleY, kalAngleZ);
  }
  if (FUSIONTEMP) {
    Serial.printf("% 4.2f,% 4.2f,", estAngleZrate, estHeading);
  }
  Serial.println();
}
void updatePitchRoll() {
  roll = atan2(Ax, Az) * RAD_TO_DEG;
  pitch = atan(-Ax / sqrt(Ay * Ay + Az * Az)) * RAD_TO_DEG;
}
void updateYaw() {
  float rollAngle = kalAngleX * DEG_TO_RAD;
  float pitchAngle = kalAngleY * DEG_TO_RAD;

  float fy = MagZ * sin(rollAngle) - MagY * cos(rollAngle);
  float fx = MagX * cos(pitchAngle) + MagY * sin(pitchAngle) * sin(rollAngle) + MagZ * sin(pitchAngle) * cos(rollAngle);
  yaw = atan2(-fy, fx) * RAD_TO_DEG;
  yaw *= -1;
}
float noTiltCompensate() {
  float fx, fy;
  fx = (float)MagX / (Xmax - Xmin);
  fy = (float)MagY / (Ymax - Ymin);
  float heading = atan2(fy, fx);
  return heading;
}
float tiltCompensate() {
  roll = asin(Ay);
  pitch = asin(-Ax);

  if (roll > 0.78 || roll < -0.78 || pitch > 0.78 || pitch < -0.78)
  {
    return -1000;
  }

  float cosRoll = cos(roll);
  float sinRoll = sin(roll);
  float cosPitch = cos(pitch);
  float sinPitch = sin(pitch);

  float Xh = MagX * cosPitch + MagZ * sinPitch;
  float Yh = MagX * sinRoll * sinPitch + MagY * cosRoll - MagZ * sinRoll * cosPitch;

  float heading = atan2(Yh, Xh);

  return heading;
}
float correctAngle(float heading)
{
  if (heading < 0) {
    heading += 2 * PI;
  }
  if (heading > 2 * PI) {
    heading -= 2 * PI;
  }

  return heading;
}
void measureDrift() {
  float cumulatedDrift;
  estDrift = Gz / 131.0;
  maxDrift = 0;
  uint32_t FUSION_stopwatch = millis();
  FUSION_timer = micros();
  int i = 0;
  while (i < 100) { //total required 1 sec
    if (millis() - FUSION_stopwatch > 10) {
      float dt = (float)(micros() - FUSION_timer) / 1000000; // Calculate delta time
      estDrift = Gz / 131.0;
      estDrift += estDrift * dt;
      if (fabs(estDrift) > maxDrift) {
        maxDrift = fabs(estDrift);
      }
      cumulatedDrift += estDrift;
      FUSION_timer = micros();
      i++;
    }
    yield();
  }
  estDrift = (float)cumulatedDrift / i;
  maxDrift *= 1.5;
  if (estDrift < 0) {
    maxDrift *= -1;
    posOffset = fabs(maxDrift);
    negOffset = fabs(maxDrift) / 2;
  }
  else {
    posOffset = fabs(maxDrift) / 2;
    negOffset = fabs(maxDrift);
  }
}
/*End of Sensor Fusion Code---------------------------------------------------*/
/*PCF8574 Code----------------------------------------------------------------*/
PCF8574 PCF(0x20);  //initialize pcf8574 address

bool LED_0_STATUS = 0;
bool LED_1_STATUS = 0;
long int LED_0_Timer = 0;
long int LED_1_Timer = 0;

Task PCF_Blink0(1000,TASK_FOREVER,&blink_LED0);
Task PCF_Blink1(1000,TASK_FOREVER,&blink_LED1);

void PCF8574_Setup(){
  if(PCF8574MODULE){
    PCF.begin(0xFF);  //initialize all pins to HIGH
    LED_0(0); //initially off
    LED_1(0); //initially off
    if(PCF8574LED0){
      taskScheduler.addTask(PCF_Blink0);
      PCF_Blink0.enable();
    }
    if(PCF8574LED1){
      taskScheduler.addTask(PCF_Blink1);
      PCF_Blink1.enable();
    }
  }
}
void LED_0(int value){
  PCF.write(0,value); //0-1023
}
void LED_1(int value){
  PCF.write(1,value); //0-1023
}
void Buzzer(int value){
  PCF.write(2,value); //0 or 1
}
void blink_LED0(){
  if((millis() - LED_0_Timer) >= 1000){
    LED_0_STATUS = !LED_0_STATUS;
    LED_0(LED_0_STATUS); 
    LED_0_Timer = millis();
  }
}
void blink_LED1(){
  if((millis() - LED_1_Timer) >= 1000){
    LED_1_STATUS = !LED_1_STATUS;
    LED_1(LED_1_STATUS); 
    LED_1_Timer = millis();
  }
}
/*End of PCF8574 Code---------------------------------------------------------*/
/*Encoder Code----------------------------------------------------------------*/
/*Tested to have <1% error when measured Tp is compared to actual Tp with lower standard deviation*/
#define debounceVal 15 //ms

const uint8_t L_EN = D2;
const uint8_t R_EN = D6;

volatile unsigned long L_micros = 0, R_micros = 0; //for managing debounce
bool  L_move = 0, R_move = 0;
const int ENC_SAMPLES = 5;
unsigned long L_timer = 0, R_timer = 0;
int   L_tp[ENC_SAMPLES], R_tp[ENC_SAMPLES];
volatile float L_avgtp = 0, R_avgtp = 0;  //volatile because can change in interrupt
volatile long  L_tso = 0, R_tso = 0, L_tss = 0, R_tss = 0;
long  L_tsg = 0, R_tsg = 0;
int   L_counter = 0, R_counter = 0;
float L_rpm = 0, R_rpm = 0;

void ICACHE_RAM_ATTR L_Callback() {
  if ((long)(micros() - L_micros) >= debounceVal * 1000) {
    if (digitalRead(L_EN)) { //check if L_EN pin is high
      if (L_move) { //if Left motor is controlled by NodeMCU to move
        if (L_tss == 0) { //if tick since stop is 0, this is the first tick, not counted because motor stopped for long periods
          //L_avgtp = millis() - L_avgtp;
          L_timer = millis(); //reset timer for tick
          L_tss++;  //1 tick and above means that motor is moving
        }
        else { //1 tick and above
          L_tp[L_counter] = millis() - L_timer; //calculate period between ticks
          L_timer = millis(); //reset timer for tick
          L_avgtp = 0;  //reset previous avg period
          L_counter++; L_tss++; //L_tso++;  //increment, L_counter is for n sample of array, L_tss is to count tick since stop, L_tso is tick since on
          int i;  //local variable, so as to not disrupt other functions
          for (i = 0; i < (L_tss - 1) && i < ENC_SAMPLES; i++) { //L_tss-1 because first tick doesn't count, ENC_SAMPLES is max sample to store
            L_avgtp += L_tp[i]; //add up all existing samples, if there is only 1 sample, L_tss = 2, (L_tss-1) is >= ENC_SAMPLES after 5+1 ticks
          }
          L_avgtp = (float)L_avgtp / i; //take accumulated period and divide by i(number of samples added up) to get average period
          L_rpm = (float)3000 / L_avgtp;; //calculated rpm, 60000ms / L_avgtp ms / 20 ticks => 3000/L_avgtp
          if (L_counter == ENC_SAMPLES) //if L_counter reaches ENC_SAMPLES, array is full, restart from beginning
            L_counter = 0;  //reset L_counter
          //Serial.printf("L_TP:%.2fms,L_RPM:%.2f\n",L_avgtp,L_rpm);
        }
      }
      else {  //Left motor is either moved by external force, or leftover momentum after stopped
        L_tsg++;
      }
      L_tso++;  //accumulate ticks since on for dead reckoning positioning
    }
    L_micros = micros();
  }
}
void ICACHE_RAM_ATTR R_Callback() {
  if ((long)(micros() - R_micros) >= debounceVal * 1000) {
    if (digitalRead(R_EN)) { //check if R_EN pin is high
      if (R_move) { //if Right motor is controlled by NodeMCU to move
        if (R_tss == 0) { //if tick since stop is 0, this is the first tick, not counted because motor stopped for long periods
          R_timer = millis(); //reset timer for tick
          R_tss++;  //1 tick and above means that motor is moving
        }
        else { //1 tick and above
          R_tp[R_counter] = millis() - R_timer; //calculate period between ticks
          R_timer = millis(); //reset timer for tick
          R_avgtp = 0;  //reset previous avg period
          R_counter++; R_tss++; //R_tso++;  //increment, R_counter is for n sample of array, R_tss is to count tick since stop, R_tso is tick since on
          int i;  //local variable, so as to not disrupt other functions
          for (i = 0; i < (R_tss - 1) && i < ENC_SAMPLES; i++) { //L_tss-1 because first tick doesn't count, ENC_SAMPLES is max sample to store
            R_avgtp += R_tp[i]; //add up all existing samples, if there is only 1 sample, R_tss = 2, (R_tss-1) is >= ENC_SAMPLES after 5+1 ticks
          }
          R_avgtp = (float)R_avgtp / i; //take accumulated period and divide by i(number of samples added up) to get average period
          R_rpm = (float)3000 / R_avgtp;; //calculated rpm, 60000ms / R_avgtp ms / 20 ticks => 3000/R_avgtp
          if (R_counter == ENC_SAMPLES) //if R_counter reaches ENC_SAMPLES, array is full, restart from beginning
            R_counter = 0;  //reset R_counter
          //Serial.printf("R_TP:%.2fms,R_RPM:%.2f\n",R_avgtp,R_rpm);
        }
      }
      else {  //Right motor is either moved by external force, or leftover momentum after stopped
        R_tsg++;
      }
      R_tso++;  //accumulate ticks since on for dead reckoning positioning
    }
    R_micros = micros();
  }
}
void ENCODER_Setup() {
  if (ENCODER_L) {
    pinMode(L_EN, INPUT);
    attachInterrupt(digitalPinToInterrupt(L_EN), L_Callback, RISING);
  }
  if (ENCODER_R) {
    pinMode(R_EN, INPUT);
    attachInterrupt(digitalPinToInterrupt(R_EN), R_Callback, RISING);
  }
}
void ENCODER_reset() {
  L_counter = 0;  R_counter = 0;  //Reset counter for array
  L_avgtp = 0; R_avgtp = 0; //Reset average tp, only use avgtp after tss>=2
  L_tss = 0;  R_tss = 0;  //Reset tick since stop
  L_tsg = 0;  R_tsg = 0;  //Reset tick since go
  for (int i = 0; i < ENC_SAMPLES; i++) {
    L_tp[i] = 0;  R_tp[i] = 0;  //Reset whole array
  }
}
/*End of Encoder Code---------------------------------------------------------*/
/*Bump Button Code------------------------------------------------------------*/
const uint8_t L_BUT = D4;
const uint8_t R_BUT = TX;
const uint8_t B_BUT = D0;
volatile unsigned long L_b_micros = 0, R_b_micros = 0, B_b_micros; //for managing debounce
volatile bool  L_bumped = 0, R_bumped = 0, B_bumped = 0, F_bumped = 0;

Task PID_B_left(0, TASK_ONCE, &MATRIX_Bump);
Task PID_B_right(0, TASK_ONCE, &MATRIX_Bump);
Task PID_B_front(0, TASK_ONCE, &MATRIX_Bump);
Task PID_B_back(0, TASK_ONCE, &MATRIX_Bump);
Task PID_B_check(5,TASK_FOREVER, &B_Bump);

void clearLocate();
void ICACHE_RAM_ATTR L_Bump() {
  if ((long)(micros() - L_b_micros) >= debounceVal * 1000) {
    if(digitalRead(L_BUT) && digitalRead(R_BUT)){
      F_bumped = 1; MATRIX_bumped = 1;
      //PID_B_front.restart();
    }
    else{
      L_bumped = 1; MATRIX_bumped = 1;
      //PID_B_left.restart();
    }
    L_b_micros = micros();
  }
}
void ICACHE_RAM_ATTR R_Bump() {
  if ((long)(micros() - R_b_micros) >= debounceVal * 1000) {
    if(digitalRead(L_BUT) && digitalRead(R_BUT)){
      F_bumped = 1; MATRIX_bumped = 1;
      //PID_B_front.restart();
    }
    else{
      R_bumped = 1; MATRIX_bumped = 1;
      //PID_B_right.restart();
    }
    R_b_micros = micros();
  }
}
void B_Bump() {
  if (BUMP_B && !isROOT) {
    if (digitalRead(B_BUT)) {
      if ((long)(micros() - L_b_micros) >= debounceVal * 1000) {
        B_bumped = 1; MATRIX_bumped = 1;
        //PID_B_back.restart();
        B_b_micros = micros();
      }
    }
  }
}
void BUTTON_Setup() {
  taskScheduler.addTask(PID_B_left);
  taskScheduler.addTask(PID_B_right);
  taskScheduler.addTask(PID_B_front);
  taskScheduler.addTask(PID_B_back);
  taskScheduler.addTask(PID_B_check);
  if (BUMP_L) {
    pinMode(L_BUT, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(L_BUT), L_Bump, FALLING);
  }
  if (BUMP_R) {
    pinMode(R_BUT, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(R_BUT), R_Bump, FALLING);
  }
  if (BUMP_B) {
    pinMode(B_BUT, INPUT_PULLDOWN_16);
    PID_B_check.enable(); //check button every 10ms
  }
}
/*End of Bump Button Code-----------------------------------------------------*/
/*Motor PID Code--------------------------------------------------------------*/
float Setpoint_offset = 1.5;
float actualSetpoint = 35;  //85.714 RPM
float Setpoint = 35, L_pidin, L_pidout, R_pidin, R_pidout;    //35
//float Kp=7.2, Ki=0.065, Kd=0.01625; //Kcrit = 12, Pcrit = 130ms
float Kp = 10.2, Ki = 0.03, Kd = 0.0075; //Kcrit = 17, Pcrit = 60ms (improved accuracy)

PID L_PID(&L_pidin, &L_pidout, &Setpoint, Kp, Ki, Kd, REVERSE);
PID R_PID(&R_pidin, &R_pidout, &Setpoint, Kp, Ki, Kd, REVERSE);

void PID_Setup() {
  L_PID.SetSampleTime(10); R_PID.SetSampleTime(10); //ms
  L_PID.SetOutputLimits(0, 1023); R_PID.SetOutputLimits(0, 1023); //min max
  L_PID.SetMode(MANUAL); R_PID.SetMode(MANUAL); //MANUAL to manually reset output, AUTOMATIC to let code auto set
}

const int STEADY_SAMPLES = 4;
const float steady_state_err = 5;  //max allowed steady state err %
const float steady_state_value = (actualSetpoint*steady_state_err) / 100.0;
float prev_max = 0, prev_min = 100;
float maxer[STEADY_SAMPLES], miner[STEADY_SAMPLES];
float avg_max = 0, avg_min = 100;
int max_counter = 0, min_counter = 0, max_samples = 0, min_samples = 0;
bool prev_state = 0, current_state = 0, transition = 0;

bool getSteadyState(float avgtp) {
  if (avgtp > Setpoint) {
    current_state = 1;
    if (prev_max < avgtp)
      prev_max = avgtp;
    if (prev_state != current_state) { //transition
      transition = 1;
      if (min_counter == STEADY_SAMPLES) //if counter reach max
        min_counter = 0;  //reset counter
      miner[min_counter] = prev_min;  //store previous highest point
      min_samples++; min_counter++;
      avg_min = 0;
      int i;  //declare locally to not affect other code
      for (i = 0; i < min_samples && i < STEADY_SAMPLES; i++)
        avg_min += miner[i];  //sum up all highest point
      avg_min = (float)avg_min / (float)i; //determine current high point avg
      prev_min = actualSetpoint;
    }
    prev_state = current_state;
  }
  else if (avgtp < Setpoint) {
    current_state = 0;
    if (prev_min > avgtp)
      prev_min = avgtp;
    if (prev_state != current_state) { //transition
      transition = 1;
      if (max_counter == STEADY_SAMPLES) //if counter reach max
        max_counter = 0;  //reset counter
      maxer[max_counter] = prev_max;  //store previous highest point
      max_samples++; max_counter++;
      avg_max = 0;
      int i;  //declare locally to not affect other code
      for (i = 0; i < max_samples && i < STEADY_SAMPLES; i++)
        avg_max += maxer[i];  //sum up all highest point
      avg_max = (float)avg_max / (float)i; //determine current high point avg
      prev_max = actualSetpoint;
    }
    prev_state = current_state;
  }
  if (transition) {
    //Wait until response signal is within +- Steady state error
    if ( (avg_max - actualSetpoint) < steady_state_value && (actualSetpoint - avg_min) < steady_state_value ) {
      //Serial.printf("Reached Steady State with Error of %f%%\n",steady_state_err);
      return true;
    }
    transition = 0;
  }
  return false;
}
/*End of Motor PID Code-------------------------------------------------------*/
/*Motor Driver Code-----------------------------------------------------------*/
/* Since wheel circumference = 21.049cm, 1 slot movement = 1.052cm
   A 360 rotate around Axis of Rotation with radius 6.5cm is 40.846cm
   1 degree of rotation = 0.113cm, thus minimum rotation is 9.31 degree, using 1 slot,
   but since the robot requires at least 2 slots to accurately measure and adjust the
   speed of the wheel, by the time it sense to stop, 3 slots will have passed by.
   Thus, with this limitation, it is much better to move straight then turn 90 degree
   to reach the desired location instead.
   Even so, that method comes with its complication such as hitting obstacles or walls
   during the movement.
   Another solution is to add 360 degrees if the desired angle of rotation is too small,
   allowing the robot to readjust calculation to reach the desired angle in the end, albeit
   having some errors.
*/
const uint8_t M1A = D7;
const uint8_t M1B = D8;
const uint8_t M2A = RX;
const uint8_t M2B = D5;

#define WHEEL_DIAMETER      6.7     //in cm
#define WHEEL_CIRCUMFERENCE 21.049  //cm
#define AOR_RADIUS          6.5     //distance from center of wheel to axis of rotation in cm
#define DEFAULT_SPEED       550     //start with 550, then adjust to (600/1023)
#define DEFAULT_SETPOINT    35

void sendToRoot(char []);

volatile float distanceToMove = 100;//cm
int   L_pwm = 0,  R_pwm = 0;    //0 - 1023
int   L_prev = 0, R_prev = 0;   //store previous PID speed

Task PID_forward(0, TASK_ONCE, &PID_Forward);
Task PID_rotate(0, TASK_ONCE, &PID_Rotate);

void PID_MOTOR_Setup() {
  PID_Setup();
  if (MOTOR_CONTROL_L) {
    L_pwm = DEFAULT_SPEED;
    pinMode(M1A, OUTPUT);  pinMode(M2A, OUTPUT);
    digitalWrite(M1A, 0);  digitalWrite(M2A, 0);
  }
  if (MOTOR_CONTROL_R) {
    R_pwm = DEFAULT_SPEED;
    pinMode(M1B, OUTPUT);  pinMode(M2B, OUTPUT);
    digitalWrite(M2A, 0);  digitalWrite(M2B, 0);
  }
  if (MOTOR_CONTROL_L || MOTOR_CONTROL_R) {
    taskScheduler.addTask(PID_forward);
    taskScheduler.addTask(PID_rotate);
  }
}
void PID_Forward() {
  Setpoint = DEFAULT_SETPOINT;
  float avgtp, tempL, tempR;  //temporary storage for avgtp, L & R tp
  int donetime; //to measure time taken to complete action
  int ticksRequired = round((distanceToMove / WHEEL_CIRCUMFERENCE) * 20); //distance to travel/circumference * slots
  Serial.printf("Ticks Req:%d\n", ticksRequired);
  if (L_prev != 0 && R_prev != 0) {
    L_pwm = L_prev; R_pwm = R_prev; //use previous PID computed values
  }
  else{
    L_pwm = DEFAULT_SPEED;  R_pwm = DEFAULT_SPEED;
  }
  L_PID.SetMode(MANUAL);    R_PID.SetMode(MANUAL);    //to manually reset output
  L_pidout = (float)L_pwm;  R_pidout = (float)R_pwm;  //reset output value
  L_PID.SetMode(AUTOMATIC); R_PID.SetMode(AUTOMATIC); //auto again
  digitalWrite(M1B, 0); digitalWrite(M2B, 0); //Should set low first before starting motors, outside loop because it doesnt change
  ENCODER_reset();  //reset all encoder values
  donetime = millis();  //measure time for action to stop
  L_move = 1; R_move = 1; //start moving
  //L_avgtp = millis(); R_avgtp = millis(); //to start counting first tick (0 -> 1)
  analogWrite(M1A, L_pwm); analogWrite(M2A, R_pwm); //Start motors, so that first tp can be measured
  while ( ( (L_tss < ticksRequired) || (R_tss < ticksRequired) ) && !L_bumped && !R_bumped && !B_bumped && !F_bumped){
    if (L_tss > 1 && L_tss < ticksRequired) { //once the first tp is measured, then start computing PID
      avgtp = L_avgtp;  //temporarily store current avg tp
      tempL = avgtp;    //to send to app
      L_pidin = avgtp;  //input PID value
      L_PID.Compute();  //Compute the output based on err/delta
      L_pwm = (int)round(L_pidout);  //round off and convert to int
      L_prev = L_pwm;   //store PID-ed value for next time
    }
    else if (L_tss >= ticksRequired) { //if already reached supposed ticks, stop motor
      L_pwm = 0;  L_move = 0;
    }
    if (R_tss > 1 && R_tss < ticksRequired) { //once the first tp is measured, then start computing PID
      avgtp = R_avgtp;  //temporarily store current avg tp
      tempR = avgtp;    //to send to app
      R_pidin = avgtp;  //input PID value
      R_PID.Compute();  //Compute the output based on err/delta
      R_pwm = (int)round(R_pidout);  //round off and convert to int
      R_prev = R_pwm;
    }
    else if (R_tss >= ticksRequired) { //if already reached supposed ticks, stop motor
      R_pwm = 0;  R_move = 0;
    }
    int temp_Lspeed, temp_Rspeed;
    if (abs(R_tss - L_tss) > 1 && R_tss > 1 && R_tss < ticksRequired && L_tss > 1 && L_tss < ticksRequired) {
      int tickDiff = abs(R_tss - L_tss);
      float enhancer = 0.05 * tickDiff;
      float dehancer = 1.0 - enhancer;
      if (R_tss > L_tss) {
        temp_Lspeed = round((enhancer + 1.0) * L_pwm);
        temp_Rspeed = round(dehancer * R_pwm);
      }
      else {
        temp_Rspeed = round((enhancer + 1.0) * R_pwm);
        temp_Lspeed = round(dehancer * L_pwm);
      }
    }
    else {
      temp_Lspeed = L_pwm;
      temp_Rspeed = R_pwm;
    }
    analogWrite(M1A, temp_Lspeed); analogWrite(M2A, temp_Rspeed); //Change motor value
    //Serial.printf("L_pwm:%d,L_val:%f,L_avg:%f,L_tss:%d\n",L_pwm,L_pidout,avgtp,L_tss);
    //char data[100];
    //sprintf(data,"%s:L_TP,%f,R_TP,%f,%d\n","MOVINGGUY",tempL,tempR,L_tss);
    //sendToRoot(data);
    yield();
  }
  if(L_bumped || R_bumped || F_bumped || B_bumped){
    movedTss = (float)(L_tss + R_tss)/2;
  }
  donetime = millis() - donetime;
  digitalWrite(M1B, 0); digitalWrite(M2B, 0);
  digitalWrite(M1A, 0); digitalWrite(M2A, 0); //stop both wheels
  L_move = 0; R_move = 0; //stop recording tp
  Serial.printf("In:%.2f,Out:%d,Delta:%.2f,Set:%.2f\n", L_avgtp, L_pwm, L_avgtp - actualSetpoint, actualSetpoint);
  Serial.printf("Total time taken:%d\n", donetime);
  ENCODER_reset();  //reset values
}

void PID_Backward() {
  Setpoint = DEFAULT_SETPOINT;
  float avgtp, tempL, tempR;  //temporary storage for avgtp, L & R tp
  int donetime; //to measure time taken to complete action
  int ticksRequired = round((distanceToMove / WHEEL_CIRCUMFERENCE) * 20); //distance to travel/circumference * slots
  Serial.printf("Ticks Req:%d\n", ticksRequired);
  if (L_prev != 0 && R_prev != 0) {
    L_pwm = L_prev; R_pwm = R_prev; //use previous PID computed values
  }
  else{
    L_pwm = DEFAULT_SPEED;  R_pwm = DEFAULT_SPEED;
  }
  L_PID.SetMode(MANUAL);    R_PID.SetMode(MANUAL);    //to manually reset output
  L_pidout = (float)L_pwm;  R_pidout = (float)R_pwm;  //reset output value
  L_PID.SetMode(AUTOMATIC); R_PID.SetMode(AUTOMATIC); //auto again
  digitalWrite(M1A, 0); digitalWrite(M2A, 0); //Should set low first before starting motors, outside loop because it doesnt change
  ENCODER_reset();  //reset all encoder values
  donetime = millis();  //measure time for action to stop
  L_move = 1; R_move = 1; //start moving
  //L_avgtp = millis(); R_avgtp = millis(); //to start counting first tick (0 -> 1)
  analogWrite(M1B, L_pwm); analogWrite(M2B, R_pwm); //Start motors, so that first tp can be measured
  while ( ( (L_tss < ticksRequired) || (R_tss < ticksRequired) ) && !L_bumped && !R_bumped && !B_bumped && !F_bumped){
    if (L_tss > 1 && L_tss < ticksRequired) { //once the first tp is measured, then start computing PID
      avgtp = L_avgtp;  //temporarily store current avg tp
      tempL = avgtp;    //to send to app
      L_pidin = avgtp;  //input PID value
      L_PID.Compute();  //Compute the output based on err/delta
      L_pwm = (int)round(L_pidout);  //round off and convert to int
      L_prev = L_pwm;   //store PID-ed value for next time
    }
    else if (L_tss >= ticksRequired) { //if already reached supposed ticks, stop motor
      L_pwm = 0;  L_move = 0;
    }
    if (R_tss > 1 && R_tss < ticksRequired) { //once the first tp is measured, then start computing PID
      avgtp = R_avgtp;  //temporarily store current avg tp
      tempR = avgtp;    //to send to app
      R_pidin = avgtp;  //input PID value
      R_PID.Compute();  //Compute the output based on err/delta
      R_pwm = (int)round(R_pidout);  //round off and convert to int
      R_prev = R_pwm;
    }
    else if (R_tss >= ticksRequired) { //if already reached supposed ticks, stop motor
      R_pwm = 0;  R_move = 0;
    }
    int temp_Lspeed, temp_Rspeed;
    if (abs(R_tss - L_tss) > 1 && R_tss > 1 && R_tss < ticksRequired && L_tss > 1 && L_tss < ticksRequired) {
      int tickDiff = abs(R_tss - L_tss);
      float enhancer = 0.05 * tickDiff;
      float dehancer = 1.0 - enhancer;
      if (R_tss > L_tss) {
        temp_Lspeed = round((enhancer + 1.0) * L_pwm);
        temp_Rspeed = round(dehancer * R_pwm);
      }
      else {
        temp_Rspeed = round((enhancer + 1.0) * R_pwm);
        temp_Lspeed = round(dehancer * L_pwm);
      }
    }
    else {
      temp_Lspeed = L_pwm;
      temp_Rspeed = R_pwm;
    }
    analogWrite(M1B, temp_Lspeed); analogWrite(M2B, temp_Rspeed); //Change motor value
    //Serial.printf("L_pwm:%d,L_val:%f,L_avg:%f,L_tss:%d\n",L_pwm,L_pidout,avgtp,L_tss);
    //char data[100];
    //sprintf(data,"%s:L_TP,%f,R_TP,%f,%d\n","MOVINGGUY",tempL,tempR,L_tss);
    //sendToRoot(data);
    yield();
  }
  if(L_bumped || R_bumped || F_bumped || B_bumped){
    movedTss = (float)(L_tss + R_tss)/2;
  }
  donetime = millis() - donetime;
  digitalWrite(M1B, 0); digitalWrite(M2B, 0);
  digitalWrite(M1A, 0); digitalWrite(M2A, 0); //stop both wheels
  L_move = 0; R_move = 0; //stop recording tp
  Serial.printf("In:%.2f,Out:%d,Delta:%.2f,Set:%.2f\n", L_avgtp, L_pwm, L_avgtp - actualSetpoint, actualSetpoint);
  Serial.printf("Total time taken:%d\n", donetime);
  ENCODER_reset();  //reset values
}

#define ROTATE_SPEED        350
#define ROTATE_SETPOINT     45
volatile bool   rotate_Clockwise = 1; //default clockwise rotation
volatile float  angleToRotate = 90;   //degrees
const float     angleErrorRange = 5;  //+- 5 degree difference
int L_front, L_back, R_front, R_back;
int L_prev_rot = 0, R_prev_rot = 0;

void change_Rotate(){
  if (rotate_Clockwise) {
    L_front = M1A;  L_back = M1B;
    R_front = M2B;  R_back = M2A;
  }
  else {
    L_front = M1B;  L_back = M1A;
    R_front = M2A;  R_back = M2B;
  }
}

void PID_Rotate() {
  Setpoint = ROTATE_SETPOINT; //set target wheel speed
  float avgtp, tempL, tempR;  //temporary storage for avgtp, L & R tp
  int donetime; //to measure time taken to complete action
  if(angleToRotate >=0 && angleToRotate <= 10.0){
    angleToRotate = angleToRotate + 360.0;
  }
  else if(angleToRotate < 0){
    angleToRotate = 360.0;
  }
  float multiplier = (float)angleToRotate/360.0;
  if(multiplier > 720){
    multiplier = 2;
  }
  else if(multiplier < 0){
    multiplier = 1;
  }
  float distanceToRotate = (float)(2 * M_PI * AOR_RADIUS) * multiplier;
  int ticksRequired = round((distanceToRotate / WHEEL_CIRCUMFERENCE) * 20); //distance to travel/circumference * slots
  Serial.printf("Ticks Req:%d\n", ticksRequired);
  rotate_Clockwise = 1; //best to rotate clockwise
  change_Rotate();
  if (L_prev_rot != 0 && R_prev_rot != 0) {
    L_pwm = L_prev_rot; R_pwm = R_prev_rot;     //use previous PID computed values
  }
  else{
    L_pwm = ROTATE_SPEED; R_pwm = ROTATE_SPEED; //default rotation speed, slower than straight movement
  }
  L_PID.SetMode(MANUAL);    R_PID.SetMode(MANUAL);    //to manually reset output
  L_pidout = (float)L_pwm;  R_pidout = (float)R_pwm;  //reset output value
  L_PID.SetMode(AUTOMATIC); R_PID.SetMode(AUTOMATIC); //auto again
  digitalWrite(L_back, 0); digitalWrite(R_back, 0); //Should set low first before starting motors, outside loop because it doesnt change
  ENCODER_reset();  //reset all encoder values
  donetime = millis();  //measure time for action to stop
  L_move = 1; R_move = 1; //start moving
  //L_avgtp = millis(); R_avgtp = millis(); //to start counting first tick (0 -> 1)
  analogWrite(L_front, L_pwm); analogWrite(R_front, R_pwm); //Start motors, so that first tp can be measured
  while ( ( (L_tss < ticksRequired) || (R_tss < ticksRequired) ) && !L_bumped && !R_bumped && !B_bumped && !F_bumped) {
    if (L_tss > 1 && L_tss < ticksRequired) { //once the first tp is measured, then start computing PID
      avgtp = L_avgtp;  //temporarily store current avg tp
      tempL = avgtp;    //to send to app
      L_pidin = avgtp;  //input PID value
      L_PID.Compute();  //Compute the output based on err/delta
      L_pwm = (int)round(L_pidout);  //round off and convert to int
      L_prev_rot = L_pwm;   //store PID-ed value for next time
    }
    else if (L_tss >= ticksRequired) { //if already reached supposed ticks, stop motor
      L_pwm = 0;  L_move = 0;
    }
    if (R_tss > 1 && R_tss < ticksRequired) { //once the first tp is measured, then start computing PID
      avgtp = R_avgtp;  //temporarily store current avg tp
      tempR = avgtp;    //to send to app
      R_pidin = avgtp;  //input PID value
      R_PID.Compute();  //Compute the output based on err/delta
      R_pwm = (int)round(R_pidout);  //round off and convert to int
      R_prev_rot = R_pwm;
    }
    else if (R_tss >= ticksRequired) { //if already reached supposed ticks, stop motor
      R_pwm = 0;  R_move = 0;
    }
    int temp_Lspeed, temp_Rspeed;
    if (abs(R_tss - L_tss) > 1 && R_tss > 1 && R_tss < ticksRequired && L_tss > 1 && L_tss < ticksRequired) {
      int tickDiff = abs(R_tss - L_tss);
      float enhancer = 0.05 * tickDiff;
      float dehancer = 1.0 - enhancer;
      if (R_tss > L_tss) {
        temp_Lspeed = round((enhancer + 1.0) * L_pwm);
        temp_Rspeed = round(dehancer * R_pwm);
      }
      else {
        temp_Rspeed = round((enhancer + 1.0) * R_pwm);
        temp_Lspeed = round(dehancer * L_pwm);
      }
    }
    else {
      temp_Lspeed = L_pwm;
      temp_Rspeed = R_pwm;
    }
    analogWrite(L_front, temp_Lspeed); analogWrite(R_front, temp_Rspeed); //Change motor value
    //Serial.printf("L_pwm:%d,L_val:%f,L_avg:%f,L_tss:%d\n",L_pwm,L_pidout,avgtp,L_tss);
    //char data[100];
    //sprintf(data,"%s:L_TP,%f,R_TP,%f,%d\n","MOVINGGUY",tempL,tempR,L_tss);
    //sendToRoot(data);
    yield();
  }
  donetime = millis() - donetime;
  digitalWrite(L_back, 0);  digitalWrite(R_back, 0);
  digitalWrite(L_front, 0); digitalWrite(R_front, 0); //stop both wheels
  L_move = 0; R_move = 0; //stop recording tp
  Serial.printf("In:%.2f,Out:%d,Delta:%.2f,Set:%.2f\n", L_avgtp, L_pwm, L_avgtp - actualSetpoint, actualSetpoint);
  Serial.printf("Total time taken:%d\n", donetime);
  ENCODER_reset();  //reset values
}
void PID_Cali_Rotate() {
  Setpoint = ROTATE_SETPOINT; //set target wheel speed
  float avgtp, tempL, tempR;  //temporary storage for avgtp, L & R tp
  int donetime; //to measure time taken to complete action
  float distanceToRotate = (2 * M_PI * AOR_RADIUS) * (720 / 360.0);
  int ticksRequired = round((distanceToRotate / WHEEL_CIRCUMFERENCE) * 20); //distance to travel/circumference * slots
  Serial.printf("Ticks Req:%d\n", ticksRequired);
  rotate_Clockwise = 1; //best to rotate clockwise
  change_Rotate();
  if (L_prev_rot != 0 && R_prev_rot != 0) {
    L_pwm = L_prev_rot; R_pwm = R_prev_rot;     //use previous PID computed values
  }
  else{
    L_pwm = ROTATE_SPEED; R_pwm = ROTATE_SPEED; //default rotation speed, slower than straight movement
  }
  L_PID.SetMode(MANUAL);    R_PID.SetMode(MANUAL);    //to manually reset output
  L_pidout = (float)L_pwm;  R_pidout = (float)R_pwm;  //reset output value
  L_PID.SetMode(AUTOMATIC); R_PID.SetMode(AUTOMATIC); //auto again
  digitalWrite(L_back, 0); digitalWrite(R_back, 0); //Should set low first before starting motors, outside loop because it doesnt change
  ENCODER_reset();  //reset all encoder values
  donetime = millis();  //measure time for action to stop
  L_move = 1; R_move = 1; //start moving
  //L_avgtp = millis(); R_avgtp = millis(); //to start counting first tick (0 -> 1)
  analogWrite(L_front, L_pwm); analogWrite(R_front, R_pwm); //Start motors, so that first tp can be measured
  while ( ( (L_tss < ticksRequired) || (R_tss < ticksRequired) ) && !L_bumped && !R_bumped && !B_bumped && !F_bumped) {
    if (L_tss > 1 && L_tss < ticksRequired) { //once the first tp is measured, then start computing PID
      avgtp = L_avgtp;  //temporarily store current avg tp
      tempL = avgtp;    //to send to app
      L_pidin = avgtp;  //input PID value
      L_PID.Compute();  //Compute the output based on err/delta
      L_pwm = (int)round(L_pidout);  //round off and convert to int
      L_prev_rot = L_pwm;   //store PID-ed value for next time
    }
    else if (L_tss >= ticksRequired) { //if already reached supposed ticks, stop motor
      L_pwm = 0;  L_move = 0;
    }
    if (R_tss > 1 && R_tss < ticksRequired) { //once the first tp is measured, then start computing PID
      avgtp = R_avgtp;  //temporarily store current avg tp
      tempR = avgtp;    //to send to app
      R_pidin = avgtp;  //input PID value
      R_PID.Compute();  //Compute the output based on err/delta
      R_pwm = (int)round(R_pidout);  //round off and convert to int
      R_prev_rot = R_pwm;
    }
    else if (R_tss >= ticksRequired) { //if already reached supposed ticks, stop motor
      R_pwm = 0;  R_move = 0;
    }
    int temp_Lspeed, temp_Rspeed;
    if (abs(R_tss - L_tss) > 1 && R_tss > 1 && R_tss < ticksRequired && L_tss > 1 && L_tss < ticksRequired) {
      int tickDiff = abs(R_tss - L_tss);
      float enhancer = 0.05 * tickDiff;
      float dehancer = 1.0 - enhancer;
      if (R_tss > L_tss) {
        temp_Lspeed = round((enhancer + 1.0) * L_pwm);
        temp_Rspeed = round(dehancer * R_pwm);
      }
      else {
        temp_Rspeed = round((enhancer + 1.0) * R_pwm);
        temp_Lspeed = round(dehancer * L_pwm);
      }
    }
    else {
      temp_Lspeed = L_pwm;
      temp_Rspeed = R_pwm;
    }
    analogWrite(L_front, temp_Lspeed); analogWrite(R_front, temp_Rspeed); //Change motor value
    QMC5883L_read();
    //Serial.printf("L_pwm:%d,L_val:%f,L_avg:%f,L_tss:%d\n",L_pwm,L_pidout,avgtp,L_tss);
    //char data[100];
    //sprintf(data,"%s:L_TP,%f,R_TP,%f,%d\n","MOVINGGUY",tempL,tempR,L_tss);
    //sendToRoot(data);
    yield();
  }
  donetime = millis() - donetime;
  digitalWrite(L_back, 0);  digitalWrite(R_back, 0);
  digitalWrite(L_front, 0); digitalWrite(R_front, 0); //stop both wheels
  L_move = 0; R_move = 0; //stop recording tp
  Serial.printf("In:%.2f,Out:%d,Delta:%.2f,Set:%.2f\n", L_avgtp, L_pwm, L_avgtp - actualSetpoint, actualSetpoint);
  Serial.printf("Total time taken:%d\n", donetime);
  ENCODER_reset();  //reset values
}
void PID_RotateTo(float angle){
  if(currentDegree != angle){
    if(currentDegree > angle){
      angle += 360;
    }
    angleToRotate = angle - currentDegree;
    PID_Rotate();
  }
}
void PID_MoveTo(float distance){
  if(distance > 0){
    distanceToMove = distance;
    PID_Forward();
  }
  else if(distance <0){
    PID_Backward();
  }
}
void PID_North(){
  FUSION_read();      //read MPU6050 and magnetometer values
  delay(10);          //to let some time pass
  FUSION_read();      //read again and calculate using latest dt
  float currentAngle = compAngleZ;  //using complementary filter for faster response
  if (currentAngle < 0) { //rescale it to 360 degree instead
    currentAngle = (180.0 + currentAngle) + 180.0;
  }
  angleToRotate = fabs(currentAngle);
  PID_Rotate();
}
void PID_antiNorth(){
  FUSION_read();      //read MPU6050 and magnetometer values
  delay(10);          //to let some time pass
  FUSION_read();      //read again and calculate using latest dt
  float currentAngle = compAngleZ;  //using complementary filter for faster response
  currentAngle += 180.0;
  angleToRotate = fabs(currentAngle);
  PID_Rotate();
}
void PID_B_Left(){
  L_bumped = 0;
  distanceToMove = 5;
  PID_Backward();
  distanceToMove = 100;
  
  rotate_Clockwise = 1;
  change_Rotate();
  angleToRotate = 45.0;
  PID_Rotate();
}
void PID_B_Right(){
  R_bumped = 0;
  distanceToMove = 5;
  PID_Backward();
  distanceToMove = 100;
  
  rotate_Clockwise = 0;
  change_Rotate();
  angleToRotate = 45.0;
  PID_Rotate();
  rotate_Clockwise = 1;
  change_Rotate();
}
void PID_B_Front(){
  F_bumped = 0;
  distanceToMove = 5;
  PID_Backward();
  distanceToMove = 100;
}
void PID_B_Back(){
  B_bumped = 0;
  distanceToMove = 5;
  PID_Forward();
  distanceToMove = 100;
}
/*End of Motor Driver Code----------------------------------------------------*/
/*WiFi Codes------------------------------------------------------------------*/
#define WIFI_MODE     WIFI_AP_STA   //set wifi mode here, AP_STA is access point and station mode together
#define WIFI_SSID     "Swarm_Intel" //same as mesh_wifi
#define WIFI_PASSWORD "password"
#define WIFI_MAX  15
#define WIFI_CHAR 12        //MAC address without any symbols is only 12 characters, + 1 to terminate, so 13
#define STORE_SWARM_ONLY 1  //1 for yes, 0 for no

typedef struct {
  char ssid[WIFI_CHAR];
  char mac[WIFI_CHAR];
  uint32_t intmac;  //only stores the last 8bit of the MAC in int form
  int  rssi;
  float distance;
  bool secured = 0; //0 if unsecured, 1 if secured
} wifiNetwork;      //wifi info structure/map

char ownMAC[WIFI_CHAR];
uint32_t ownintMAC;
bool scanCompleted = 0; //0 if haven't complete scanning, 1 if done
int  samples = 0;
int swarmNetworks;

void scanNetwork();
void saveNetwork();

wifiNetwork availableNetworks[WIFI_MAX];  //declares an array to store network info

Task scanNodes(0, TASK_ONCE, &scanNetwork);
Task checkScan(1000, TASK_FOREVER, &saveNetwork);

void WiFi_Setup() {
  if (WIFISCANNER) {
    taskScheduler.addTask(scanNodes);
    taskScheduler.addTask(checkScan);
    //scanNodes.enable();
  }
}

void saveOwnMac() {
  char rawMAC[18];  //mac string length is 18
  char concatMAC[9];
  WiFi.macAddress().toCharArray(rawMAC, sizeof(rawMAC));
  int tempMACnum = 0; //counter for temporary storage
  for (int k = 0; k < sizeof(rawMAC); k++) {
    if (rawMAC[k] != ':' && rawMAC[k] != '\0') { //find and remove ':' & '\0' characters
      ownMAC[tempMACnum] = rawMAC[k];  //store non-':' char into temp storage
      if(tempMACnum > 3)
        concatMAC[tempMACnum-4]=rawMAC[k];
      tempMACnum++;
    }
  }
  ownMAC[tempMACnum] = '\0'; //terminate
  concatMAC[8] = '\0';
  ownintMAC = strtoul(concatMAC,NULL,16);
  Serial.printf("SAVED:%u\n",ownintMAC);
}

void Locate();
void scanNetwork() {
  scanCompleted = 0;
  memset(availableNetworks, (char)0, WIFI_MAX); //clears all previous wifi info
  WiFi.mode(WIFI_MODE); //station mode = 3, access point + station
  WiFi.scanDelete();    //reset scanned connections
  WiFi.scanNetworks(true, false); //scan asynchronously, but has to check with .scanComplete, (async,showhidden)
  samples++;  //to check how many times scanned
  startScan = 1;
  checkScan.enable();
}

void saveNetwork() {
  int networksFound = WiFi.scanComplete();  //store scan status/networks found
  swarmNetworks = 0;
  if (networksFound < 0) { //scan issues
    if (networksFound == -1) { //scanning in process
      Serial.println("Scanning still in progress");
    }
    else if (networksFound == -2) { //scan not triggered
      Serial.println("Scan not triggered");
      scanNodes.restartDelayed(200);
    }
  }
  else { //scan completed
    if (networksFound == 0) { //no networks found
      Serial.println("No networks found");  //do nothing, or add some codes
    }
    else if (STORE_SWARM_ONLY) { //stores only "Swarm_Intel" SSIDs
      for (int i = 0; i < networksFound; i++) { //store wifi info while sorting
        if (WiFi.SSID(i) == WIFI_SSID) { //compare String(ssid) with "Swarm_Intel"
          /*Store all collected data into wifiNetwork object first*/
          char rawMAC[18];  //mac string length is 18
          WiFi.SSID(i).toCharArray(availableNetworks[swarmNetworks].ssid, sizeof(availableNetworks[swarmNetworks].ssid));   //store ssid
          WiFi.BSSIDstr(i).toCharArray(rawMAC, sizeof(rawMAC)); //store MAC with ':'
          availableNetworks[swarmNetworks].rssi = WiFi.RSSI(i); //store RSSI in dBm
          availableNetworks[swarmNetworks].distance = PL_calcDistance(availableNetworks[swarmNetworks].rssi * (-1)); //convert to -dBm
          WiFi.encryptionType(i) == ENC_TYPE_NONE ? //check encryption type
          availableNetworks[swarmNetworks].secured = 0
              :
              availableNetworks[swarmNetworks].secured = 1;
          /*Filter out ":" character from the stored mac address*/
          char concatMAC[9];
          int tempMACnum = 0; //counter for temporary storage
          for (int k = 0; k < sizeof(rawMAC); k++) {
            if (rawMAC[k] != ':' && rawMAC[k] != '\0') { //find and remove ':' & '\0' characters
              availableNetworks[swarmNetworks].mac[tempMACnum] = rawMAC[k];  //store non-':' char into temp storage
              if(tempMACnum > 3)
                concatMAC[tempMACnum-4]=rawMAC[k];
              tempMACnum++;
            }
          }
          availableNetworks[swarmNetworks].mac[tempMACnum] = '\0';  //add delimiter to make mac into string
          concatMAC[8] = '\0';  //add delimiter to make mac into string (used for int instead)
          availableNetworks[swarmNetworks].intmac = strtoul(concatMAC,NULL,16); //convert into integer
          Serial.printf("CONVERTED:%u\n",availableNetworks[swarmNetworks].intmac);
          /*Sort the collected Swarm macs according to RSSI in descending order*/
          if (swarmNetworks > 0) { //only start sorting after at least 2 data stored, sorts every i increment (on every new network added)
            for (int j = swarmNetworks; (availableNetworks[j].rssi > availableNetworks[j - 1].rssi && j > 0); j--) { //insertion sort, descending order
              wifiNetwork tempNetwork = availableNetworks[j];
              availableNetworks[j] = availableNetworks[j - 1];
              availableNetworks[j - 1] = tempNetwork;
            }
          }
          swarmNetworks++;
        }
      }
      /*Print out data into excel sheet*/
      Serial.printf("DATA,TIME,%d,", samples);
      Serial.print(ownMAC);
      for (int i = 0; i < swarmNetworks; i++) {
        if (strcmp(availableNetworks[i].ssid, WIFI_SSID) == 0) { //check if SSID is "Swarm_Intel" although theoretically should only have that
          Serial.print(",");
          for (int z = 0; z < sizeof(availableNetworks[i].mac); z++)
            Serial.print(availableNetworks[i].mac[z]);
          Serial.printf(",%d", availableNetworks[i].rssi);
          Serial.printf(",%.3f", availableNetworks[i].distance);
        }
      }
      Serial.println(",AUTOSCROLL_20"); //to autoscroll excel sheet and go to next line
      //Locate();
    }
    else { //Stores all known SSIDs
      Serial.printf("%d networks found\n", networksFound);
      for (int i = 0; i < networksFound; i++) { //store wifi info while sorting
        /*Store all collected data into wifiNetwork object first*/
        WiFi.SSID(i).toCharArray(availableNetworks[i].ssid, sizeof(availableNetworks[i].ssid));   //store ssid
        WiFi.BSSIDstr(i).toCharArray(availableNetworks[i].mac, sizeof(availableNetworks[i].mac)); //store MAC with ':'
        availableNetworks[i].rssi = WiFi.RSSI(i); //store RSSI in dbm
        WiFi.encryptionType(i) == ENC_TYPE_NONE ? //check encryption type
        availableNetworks[i].secured = 0
                                       :
        availableNetworks[i].secured = 1;
        /*Filter out ":" character from the stored mac address*/
        char tempMAC[sizeof(availableNetworks[i].mac)];  //temporary storage for MAC without ':'
        int tempMACnum = 0; //counter for temporary storage
        for (int k = 0; k < sizeof(availableNetworks[i].mac); k++) {
          if (availableNetworks[i].mac[k] != ':') { //find and remove ':' characters
            tempMAC[tempMACnum] = availableNetworks[i].mac[k];  //store non-':' char into temp storage
            tempMACnum++;
          }
          availableNetworks[i].mac[k] = 0;  //clear original storage
        }
        for (int L = 0; L < tempMACnum; L++) {
          availableNetworks[i].mac[L] = tempMAC[L]; //copy over non-':' char from temp storage to cleared original storage
        }
        /*Sort the collected Swarm macs according to RSSI in descending order*/
        if (i > 0) { //only start sorting after 2 data stored, sorts every i increment (on every new network added)
          for (int j = swarmNetworks; (availableNetworks[j].rssi > availableNetworks[j - 1].rssi && j > 0); j--) { //insertion sort, descending order
            wifiNetwork tempNetwork = availableNetworks[j];
            availableNetworks[j] = availableNetworks[j - 1];
            availableNetworks[j - 1] = tempNetwork;
          }
        }
      }
      /*Print out all found networks with their RSSI value*/
      for (int i = 0; i < networksFound; i++) {
        Serial.printf("%d:", i + 1);
        Serial.print(availableNetworks[i].ssid);
        Serial.printf("(%d) ", availableNetworks[i].rssi);
        Serial.println(availableNetworks[i].mac);
      }
    }
    scanCompleted = 1;
    getScan = 1;
    checkScan.disable();
    //scanNodes.restartDelayed(200);
  }
}

/*To set as Access point mode*/
String  SELF_SSID = "ESP_";
#define SELF_PASSWORD "password"
#define SELF_CHANNEL 0
#define SELF_HIDDEN false
#define SELF_MAX_CONNECTION 8
void configureAccessPoint() {
  if (WIFIAPMODE) {
    SELF_SSID.concat(String(ESP.getChipId(), HEX)); //SSID becomes "ESP_{chipID}"
    Serial.println(WiFi.softAP(SELF_SSID, SELF_PASSWORD, SELF_CHANNEL, SELF_HIDDEN, SELF_MAX_CONNECTION) ? "Soft AP Success" : "Soft AP Failed");
  }
}
/*End of WiFi Codes-----------------------------------------------------------*/
/*Async Server & WebSocket Codes----------------------------------------------*/
#define STATION_SSID     "Swarm"
#define STATION_PASSWORD "password"
#define HOSTNAME "SWARM_BRIDGE"

IPAddress local_IP(192, 168, 43, 111);
IPAddress gateway(192, 168, 43, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8, 8, 8, 8);
IPAddress secondaryDNS(8, 8, 4, 4);

AsyncWebServer server(80);
WebSocketsServer webSocket(81);

Task sendThings(2000, TASK_FOREVER, &sendTestData);

void AsyncServer_Setup() {
  if (ASYNCSERVER) {
    server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
      request->send(200, "text/plain", "HELLO WORLD");
      if (request->hasArg("BROADCAST")) {
        String msg = request->arg("BROADCAST");
        Serial.println(msg);
      }
    });
    server.begin();
  }
}

void WebSocket_Setup() {
  if (WEBSOCKET) {
    WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS);
    webSocket.onEvent(webSocketEvent);
    webSocket.begin();
    taskScheduler.addTask(sendThings);
//    sendThings.enable();
//    sendThings.restart();
  }
}

void WebSocket_Stop() {
  webSocket.close();
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t lenght) {
  switch (type) {
    case WStype_DISCONNECTED:             // if the websocket is disconnected
      Serial.printf("[%u] Disconnected!\n", num);
      break;
    case WStype_CONNECTED: {              // if a new websocket connection is established
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
      }
      break;
    case WStype_TEXT:                     // if new text data is received
      Serial.printf("[%u] get Text: %s\n", num, payload);
      if(lenght == 1){  //only accepts 1 char, if more then data error
        Serial.printf("%s\n",payload);
        if(payload[0] == 'Y'){
          step_counter = 0;
          mesh_timer = millis();
          START = 1;
          if(currentNodeCount > 2)  //only find target when more than at least 2 node exist
            find_Highest = 1;
          sendStart();
        }
        else if(payload[0] == 'Z'){
          START = 0;
          sendStop();
        }
      }
      break;
  }
}

void sendTestData(){
  char data[100];
  sprintf(data, "%d:Sample,%d\n", ownintMAC, millis());
  webSocket.broadcastTXT(data);
  Serial.printf("Sent Socket,%s", data);
}

void sendNodeData(String data){
  webSocket.broadcastTXT(data);
  Serial.println(data);
}
/*End of Async Server & WebSocket Codes---------------------------------------*/
/*Mesh Codes------------------------------------------------------------------*/
#define MESH_SSID "Swarm_Intel"
#define MESH_PASSWORD "password"
#define MESH_PORT 5555

#define MESH_TIMEOUT 10000  //10sec
uint32_t nodeList[10];
uint32_t selfNodeID;
uint32_t currentMaster;

int current_step = 0;
int head_count = 0;
int root_counter = 0, node_counter = 0;
uint32_t nodeCount[10];
bool node_move = 0;

painlessMesh mesh;

void mesh_Setup() {
  if (MESHNETWORK) {
    mesh.setDebugMsgTypes( ERROR | STARTUP | CONNECTION );  // set before init() so that you can see startup messages
    mesh.init(MESH_SSID, MESH_PASSWORD, &taskScheduler, MESH_PORT, WIFI_AP_STA, 6);
    mesh.onReceive(&onReceiveCallback);
    //mesh.onNewConnection(&newConnectionCallback);
    mesh.onChangedConnections(&onChangedConnectionsCallback);
    mesh.setRoot(false);
    mesh.setContainsRoot(true);

    saveOwnMac();
    if(ownintMAC == ROOTID){
      isROOT = 1;
      mesh.stationManual(STATION_SSID, STATION_PASSWORD); //connect hotspot
      mesh.setHostname(HOSTNAME);
      mesh.setRoot(true);
      Serial.println("I AM ROOT");
    }

    selfNodeID = mesh.getNodeId();
    currentMaster = selfNodeID;
    addSelfToNodeList();
  }
}
void newConnectionCallback(uint32_t nodeID) {
  updateNodeList();
  printNodeList();
}
void onChangedConnectionsCallback() {
  updateNodeList();
  printNodeList();
}
void onReceiveCallback(uint32_t from, String &msg) {
  Serial.println("RECEIVED");
  String json;
  DynamicJsonDocument doc(1024);
  json = msg.c_str();
  deserializeJson(doc, json);
  if(from == ROOTID){ //if message is from root
    if(selfNodeID != ROOTID){ //if self is not root
      bool command = doc["START"];
      if(START != command){   //if root is only sending command from app
        START = command;      //to start or stop
        if(START){
          if(currentNodeCount > 2)  //if more than 2 node exist
            find_Highest = 1; //determine which node closest to root and other nodes
        }
        step_counter = 0;
      }
      else{ //if root is replying nodes
        gotoRoot = doc["gotoRoot"]; //find out if going towards root or not
        currentTarget = doc["currentTarget"]; //find out current target
        if(find_Highest){ //if was finding target, then reset to 0
          find_Highest = 0;
          step_counter = 0;
        }
        else{
          if(gotoRoot){ //going towards root
            if(foundRoot)
              step_counter = 0; //back to scanning, dont move since already found
            else
              step_counter++;
          }
          else{ //going towards ndoes
            if(foundNodes)
              step_counter = 0; //back to scanning, dont move since already found
            else
              step_counter++;
          }
          Serial.printf("gotoRoot:%d,foundRoot:%d,foundNodes:%d\n",gotoRoot,foundRoot,foundNodes); 
        }
      }
    }
  }
  else{ //if message is to root
    if(findNode(from) < 0){
      Serial.printf("New Node:%u\n",from);
      nodeCount[head_count] = from; //store in known list
      head_count++;
      if(find_Highest){
        float vis = doc["visibility"];
        if(head_count == 1){  //first data
          currentHighest = vis;
          currentTarget = from;
        }
        else{
          if(vis > currentHighest){  //compare and find lowest
            currentHighest = vis;
            currentTarget = from;
          }
        }
        char data[100];
        sprintf(data,"%u:Visibility,%4.2f\n",from, vis);
        sendNodeData(data);
      }
      else{
        float batpercent = doc["batpercent"];
        float rootvis = doc["rootvis"];
        float currvis = doc["currvis"];
        bool root_found = doc["foundRoot"];
        bool node_found = doc["foundNodes"];
        if(root_found)
          root_counter++;
        if(node_found)
          node_counter++;
        
        Serial.println(head_count);
        char data[100];
        sprintf(data,"%u:BAT,%4.2fRootVis,%4.2f,CurrVis,%4.2f,foundRoot,%d,foundNodes,%d\n",from, batpercent, rootvis, currvis, root_found, node_found);
        sendNodeData(data);
      }
      mesh_timer = millis();
    }
  }
}
void waitForNodes(){
  if( ( (currentNodeCount > 1) && (head_count == (currentNodeCount - 1) ) ) || ((millis()-mesh_timer)>MESH_TIMEOUT) ){  //currentNodeCount - 1 because master counts as 1
    if(find_Highest){  //if currently finding lowest
      Serial.printf("Target:%u, Visibility: %4.2f\n",currentTarget,currentHighest);
    }
    else{
      if( (node_counter == head_count)){ //if all nodes converged or within range, or if there is only 1 node
        gotoRoot = 1; //start to find root
      }
      else{ //if not converged yet or too far away from each other
        gotoRoot = 0; //continue to converge
      }
      if(gotoRoot){
        if((root_counter == head_count) && (head_count != 0)){
          Serial.println("All nodes converged to root\n");
        }
      }
      for(int i=0;i<10;i++){
        nodeCount[i] = 0;
      }
    }
    Serial.printf("GotoRoot:%d,root_counter:%d,node_counter:%d,head_count:%d\n",gotoRoot,root_counter,node_counter,head_count);
    head_count = 0; node_counter = 0; root_counter = 0; //reset
    step_counter++;
  }
  else if(currentNodeCount == 1){ //if currently only root exist
    step_counter++;
  }
}
void pingNodes(){
  DynamicJsonDocument doc(1024);
  doc["START"] = 1;
  doc["gotoRoot"] = gotoRoot; //tell all nodes what mode after deciding
  doc["currentTarget"] = currentTarget;
  find_Highest = 0;
  String msg;
  serializeJson(doc,msg);
  mesh.sendBroadcast(msg);
  mesh_timer = millis();
}
void waitForRoot(){
  LED_1(1);
}
void pingRoot(){
  if (selfNodeID != ROOTID) {
    DynamicJsonDocument doc(1024);
    if(find_Highest){
      doc["visibility"] = currentHighest;
      Serial.printf("CurrentHigh:%4.2f\n",currentHighest);
    }
    else{
      doc["batpercent"] = BAT_percent;  //report battery percentage
      doc["rootvis"] = currRootVis;     //report current visibility
      doc["currvis"] = currVis;         //report current visibility
      doc["foundRoot"]  = foundRoot;    //report if already found root
      doc["foundNodes"] = foundNodes;   //report if already found node
    }
    String msg;
    serializeJson(doc, msg);
    mesh.sendSingle(ROOTID, msg);
  }
}
void sendToRoot(char data[]) {
  if (selfNodeID != ROOTID) {
    DynamicJsonDocument doc(1024);
    doc["Print"] = data;
    String msg;
    serializeJson(doc, msg);
    mesh.sendSingle(ROOTID, msg);
    Serial.println(data);
  }
}
void addSelfToNodeList() {
  nodeList[0] = selfNodeID;
  currentNodeCount = 1;
}
void updateNodeList() {
  addSelfToNodeList();
  std::list <uint32_t> NodeList = mesh.getNodeList();
  std::list <uint32_t> :: iterator it;
  for (it = NodeList.begin(); it != NodeList.end(); ++it) {
    nodeList[currentNodeCount] = *it;
    currentNodeCount++;
  }
}
void printNodeList() {
  Serial.printf("%d existing nodes:\n", currentNodeCount);
  for (int x = 0; x < currentNodeCount; x++) {
    Serial.println(nodeList[x]);
  }
}
void printMeshTopology() {
  Serial.println(mesh.subConnectionJson());
}
void selectMaster() {
  uint32_t largest = selfNodeID;
  if (selfNodeID == ROOTID) { //if own self is ROOTID
    for (int x = 0; x < currentNodeCount; x++) {
      if (nodeList[x] < largest && nodeList[x] != ROOTID)
        largest = nodeList[x];  //select the smallest NodeID excluding self
    }
  }
  /*If ROOTID is the only one in the list, then ROOTID will be its own master*/
  for (int x = 0; x < currentNodeCount; x++) {
    if (nodeList[x] > largest && nodeList[x] != ROOTID)
      largest = nodeList[x];  //select largest ID as master excluding root
  }
  currentMaster = largest;
  Serial.printf("Current Master:%u\n", currentMaster);
}
int findNode(uint32_t node){
  for(int i=0;i<currentNodeCount;i++){
    if(node == nodeCount[i])
      return i;
  }
  return -1;
}
void sendStart(){
  DynamicJsonDocument doc(1024);
  doc["START"] = 1;
  String msg;
  serializeJson(doc,msg);
  mesh.sendBroadcast(msg);
}
void sendStop(){
  DynamicJsonDocument doc(1024);
  doc["START"] = 0;
  String msg;
  serializeJson(doc,msg);
  mesh.sendBroadcast(msg);
}
/*End of Mesh Codes-----------------------------------------------------------*/
/*Location Finder Code--------------------------------------------------------*/
#define TIMEOUT_PERIOD 30000  //ms
#define MOVE_DISTANCE 1       //meter

bool mode_front = 1;
bool found_Master = 0;

typedef struct {
  uint32_t mac;
  float prevDistance = 0;
  float currDistance = 0;
  uint32_t prevTime = 0;
} swarmDistance;      //wifi into structure/map

swarmDistance availableDistance[WIFI_MAX];
int swarmStored = 0;

Task readLocate(0,TASK_ONCE,&Locate);

int searchForMAC(uint32_t MAC){
  for(int i=0;i<swarmStored;i++){ //iterate through list of stored swarmDistance data
    if (availableDistance[i].mac == MAC){ //if found wanted Mac from list
      return i; //return the index of the mac in the list
    }
  }
  return -1; //if not found, return -1
}

void clearLocate(){
    memset(availableDistance, (char)0, WIFI_MAX); //clears all previous wifi info
}

int FaceTarget(uint32_t target){
  int index = searchForMAC(target);
  if(index >= 0){
    if(availableDistance[index].currDistance < 1.0){
      found_Master = 1;
    }
    else{
      found_Master = 0;
    }
    if(mode_front){
      float angle;
      angle = calcAngle(availableDistance[index].prevDistance,availableDistance[index].currDistance);
      angleToRotate = (float)(180.0 - angle);
      PID_Rotate();
    }
    else{
      float angle;
      angle = calcAngle(availableDistance[index].prevDistance,availableDistance[index].currDistance);
      angleToRotate = angle;
      PID_Rotate();
    }
    return 1;
  }
  else
    return -1;
}

void Locate(){
  for(int i=0;i<swarmNetworks;i++){ //for every swarm network detected
    int index = searchForMAC(availableNetworks[i].intmac);
    if(index >= 0){ //if found, value will be 0 or higher
      if(availableDistance[index].prevTime != 0 && (millis()-availableDistance[index].prevTime) < TIMEOUT_PERIOD){  //if not first time data, or past time out, can calc angle here
        availableDistance[index].prevDistance = availableDistance[index].currDistance;  //store previous distance
        availableDistance[index].currDistance = availableNetworks[i].distance;      //overwrite with current distance
        availableDistance[index].prevTime = millis();  //restart timer
      }
      else{ //if first time data, or if timed out, restore as first time data (even though should be impossible to be first time data here) cnt calc angle
        availableDistance[index].currDistance = availableNetworks[i].distance;
        availableDistance[index].prevTime = millis();
      }
    }
    else{ //if first time data, not found inside list of stored data
      availableDistance[swarmStored].mac = availableNetworks[i].intmac;
      availableDistance[swarmStored].currDistance = availableNetworks[i].distance;  //store current distance
      availableDistance[swarmStored].prevTime = millis(); //start timer
      swarmStored++;  //started list from index of 0, if added new mac, plus 1
    }
  }
  
  Serial.printf("Swarm stored:%d\n",swarmStored);
  for(int i=0;i<swarmStored;i++){
    Serial.printf("MAC:%u,Prv:%4.2f,Cur:%4.2f\n",availableDistance[i].mac,availableDistance[i].prevDistance,availableDistance[i].currDistance);
  }
}

/* Angle calculated using trigonometry rules
 * A,B,C are angles; a,b,c are lengths of triangle sides
 * a: distance between self and target at 2nd position (moved 1 meter)
 * b: distance self moved from original position to new position
 * c: distance between self and target at original position
 * A: angle between b and c (original position)
 * B: angle between a and c (target position)
 * C: angle between a and b (moved position)
 * C = (a^2 + b^2 - c^2)/2(a*b)
 * Triangle rule:
 * any sum of 2 side must be longer than 3rd side
 * a + b > c, a + c > b, b + c > a
 * Note: C is the angle to be found so that target is located
 */
float calcAngle(float &prev, float &curr){  //prev is distance before moved (c), curr is after (a)
  float b = (float)MOVE_DISTANCE;
  if(abs(curr-prev) > b){ //impossible to move more than b, or set MOVE_DISTANCE
    if(curr > prev)
      curr = prev + 1;
    else
      curr = prev - 1;
  }
  //if Triangle rule is broken, then adjust prev n curr value
  while((prev + b) <= curr || (curr + b) <= prev || (prev + curr) <= b){
    if((prev + b) <= curr){
      float diff = curr - (prev + b);
      diff = diff*1.2;
      float adjcurr =curr - diff/2;
      float adjprev = prev + diff/2;
      curr = adjcurr; prev = adjprev;
    }
    if((curr + b) <= prev){
      float diff = prev - (curr + b);
      diff = diff*1.2;
      float adjcurr = curr + diff/2;
      float adjprev = prev - diff/2;
      curr = adjcurr; prev = adjprev;
    }
    if((prev + curr) <= b){
      float diff = b - (prev + curr);
      diff = diff*1.2;
      float adjcurr = curr + diff/2;
      float adjprev = prev + diff/2;
      if(adjcurr = adjprev)
        adjcurr += 0.1;
      curr = adjcurr; prev = adjprev;
    }
//    if((prev + b)< curr){
//      float diff = curr - (prev + b); //curr is longest (too long), or prev too short
//      diff = diff*1.1;
//      float adjcurr = diff*abs(log(curr)/log(10));
//      adjcurr = curr - adjcurr;
//      float adjprev = diff*abs(log(prev)/log(10));
//      adjprev = prev + adjprev;
//      curr = adjcurr; prev = adjprev;
//    }
    yield();
  }
  float ans;
  ans = pow(curr,2) + pow(b,2) - pow(prev,2); //MOVE_DISTANCE is b, so a^2 + b^2 - c^2
  ans = ans/(2*curr*b); //divide by 2(a)(b)
  ans = acos(ans);
  ans = ans*57.2957795; //convert to degree
  Serial.printf("Adjprev:% 4.2f, Adjcurr:% 4.2f\n",prev,curr);
  Serial.printf("Angle is:% 4.2f\n",ans);
  char data[100];
  sprintf(data, "%u:,%u,prev:% 4.2f,curr:% 4.2f,angle:% 4.2f\n\0", ownintMAC,  prev, curr, ans);
  sendToRoot(data);
  return ans;
}
/*End of Location Finder Code-------------------------------------------------*/
/*ACO Movement Code-----------------------------------------------------------*/
/* 
 * Movement Matrix
 * [1][2][3]
 * [4][5][6]
 * [7][8][9]
 * Direction Matrix
 * [-1,-1][-1, 0][-1,+1]
 * [ 0,-1][ 0, 0][ 0,+1]
 * [+1,-1][+1, 0][+1,+1]
 */
#define height 21
#define width 21
#define initialPheromone 1
const float MATRIX_LENGTH = 100; //1 meter
const float MATRIX_DIAGONAL = 141; //1.414 meter
float matrix[width][height];
float rootmatrix[width][height];
int currX, currY, nextX, nextY, prevX, prevY;
int MATRIX_direction = 0;

#define max_delay 2000
#define min_delay 500

Task blink_Visibility(max_delay,TASK_FOREVER,&blink_GreenLED);

bool LED_Green_STATUS = 0;

void blink_GreenLED(){
  LED_Green_STATUS = !LED_Green_STATUS;
  LED_0(LED_Green_STATUS); 
}
void adjustLED(){
  int range = max_delay - min_delay;
  int led_delay = max_delay;
  if(gotoRoot){ //if moving to root
    if(currRootVis > 0) //if not 0
      led_delay = min_delay + round((1.0-currRootVis)*range);
    else  //if 0, (not visible)
      led_delay = max_delay;
  }
  else{ //if moving to node
    if(currVis > 0)
      led_delay = min_delay + round((1.0-currVis)*range);
    else
      led_delay = max_delay;
  }
  if(led_delay > max_delay)
    led_delay = max_delay;
  else if(led_delay <= 0)
    led_delay = min_delay;
  blink_Visibility.setInterval(led_delay);
}

void MATRIX_Setup(){
  if(MATRIX){
    currX = round((float)height/2) - 1; //middle, -1 because array starts from 0
    currY = round((float)width/2) - 1;
    MATRIX_reset(matrix);
    MATRIX_reset(rootmatrix);
    Serial.println("Node Matrix:");
    MATRIX_print(matrix);
    Serial.println("Root Matrix:");
    MATRIX_print(rootmatrix);
    taskScheduler.addTask(blink_Visibility);
    blink_Visibility.enable();
    currentDegree = 0;
  }
}
void MATRIX_move(int directions){
  Serial.printf("Direction:%d\n",directions);
  float angle, distance;
  switch(directions){
    case 1: angle = 315;distance = MATRIX_DIAGONAL; nextX = currX-1; nextY = currY-1;break;
    case 2: angle = 0;  distance = MATRIX_LENGTH;   nextX = currX-1; nextY = currY-0;break;
    case 3: angle = 45; distance = MATRIX_DIAGONAL; nextX = currX-1; nextY = currY+1;break;
    case 4: angle = 270;distance = MATRIX_LENGTH;   nextX = currX-0; nextY = currY-1;break;
    case 6: angle = 90; distance = MATRIX_LENGTH;   nextX = currX-0; nextY = currY+1;break;
    case 7: angle = 225;distance = MATRIX_DIAGONAL; nextX = currX+1; nextY = currY-1;break;
    case 8: angle = 180;distance = MATRIX_LENGTH;   nextX = currX+1; nextY = currY-0;break;
    case 9: angle = 135;distance = MATRIX_DIAGONAL; nextX = currX+1; nextY = currY+1;break;
  }
  Serial.printf("CurrX:%d,CurrY:%d,NextX:%d,NextY:%d\n",currX,currY,nextX,nextY);
  if(directions != 5){
    PID_RotateTo(angle);
    currentDegree = angle;
    PID_MoveTo(distance);
  }
  if(MATRIX_bumped){  //if previously bumpped, then robot move back to prev position
    matrix[nextX][nextY] = -1;  //-1 means there is obstacle
  }
  else{ //update current position
    prevX = currX; prevY = currY;
    currX = nextX; currY = nextY;
  }
}
//pheromone = {1,2,3,4,6,7,8,9};  //data stored in that order
void MATRIX_selectdirection(){
  float pheromone[8];
  float probability[8];
  //shift coordinate to top left, eg. (1,1) -> (0,0), (8,4) -> (7,3), (0,0) -> (-1,-1)
  int x = currX - 1;
  int y = currY - 1;
  int datacount = 0;
  bool allzero = 1;
  float totalPheromone = 0;
  //retrieve the pheromone of the 8 cells surrounding current coordinate, datacount should be 8 at the end
  for(int i=0;i<3;i++){ //row
    for(int j=0;j<3;j++){ //column
      if( !( ((x+i) == currX) && ((y+j) == currY) ) ){ //do not take in pheromone of current coordinate
        if( (x+i)>=0 && (y+j)>=0 && (x+i)<width && (y+j)<height ){  //if cell not out of bounds
          if(gotoRoot)
            pheromone[datacount] = rootmatrix[x+i][y+j];  //store valid data including -1 (root matrix)
          else
            pheromone[datacount] = matrix[x+i][y+j];      //store valid data including -1 (node matrix)
            
          if(pheromone[datacount] >= 0) //if data is not -1
            totalPheromone += pheromone[datacount]; //sum of all pheromone value that is not -1
        }
        else{
          pheromone[datacount] = -1.0;  //temporarily store as -1 for out of bounds cell
        }
        datacount++;
      }
    }
  }
  //calculate probability of each cell
  for(int i=0;i<datacount;i++){ 
    if(pheromone[i] >= 0){  //if data is valid, not obstacle (-1)
      if(pheromone[i] == 0){//if data is 0
        probability[i] = 0; //directly set probability as 0 to avoid dividing 0 by a number
      }
      else{ //if data is not 0 or -1
        probability[i] = (pheromone[i]/totalPheromone)*100.0; //calculate probability in percentage
      }
    }
    else{ //if data is -1, or suggest obstacle/out of bounds
      probability[i] = 0; //directly set probability as 0 as those cells should not be considered
    }
  }
  //checking if all surrounding cells are 0 probability, if it is, then robot is stuck
  for(int i=0;i<datacount;i++){
    if(probability[i] > 0){
      allzero = 0;
      break;  //if found at least 1 cell with non zero probability, exit loop
    }
  }
  //using probability of each cell to decide direction of movement
  if(!allzero){ //if at least 1 cell has non zero probability
    int counter = 0;
    while(1){
      if(probability[counter] != 0){  //probability is not 0
        randomSeed(millis()); //set a random seed using current time
        int randnum = random(1,100);  //generate num between 1 and 100
        if(randnum <= round(probability[counter])){ //round up and convert to int for easy comparison
          MATRIX_direction = counter+1; //set direction to number of the cell
          if(MATRIX_direction >=5)
            MATRIX_direction++; //offset by 1, because 5 doesnt count
          break;  //exit loop once direction found
        }
      }
      counter++;
      if(counter == 8)  //reset if already 8
        counter = 0;
    }
  }
  else{
    randomSeed(millis()); //set a random seed using current time
    MATRIX_direction = random(1,8); //randomly select a number between 1 and 8
    if(MATRIX_direction >=5)
      MATRIX_direction++; //offset by 1, because 5 doesnt count
  }
  //Print pheromones
  Serial.printf("[ % 3.2f ][ % 3.2f ][ % 3.2f ]\n",pheromone[0],pheromone[1],pheromone[2]);
  Serial.printf("[ % 3.2f ][ % 3.2f ][ % 3.2f ]\n",pheromone[3],     0      ,pheromone[4]);
  Serial.printf("[ % 3.2f ][ % 3.2f ][ % 3.2f ]\n",pheromone[5],pheromone[6],pheromone[7]);
  Serial.println("");
  //Print probabilities
  Serial.printf("[ % 3.2f ][ % 3.2f ][ % 3.2f ]\n",probability[0],probability[1],probability[2]);
  Serial.printf("[ % 3.2f ][ % 3.2f ][ % 3.2f ]\n",probability[3],      0      ,probability[4]);
  Serial.printf("[ % 3.2f ][ % 3.2f ][ % 3.2f ]\n",probability[5],probability[6],probability[7]);
}
void MATRIX_update(){
  if(find_Highest){ //if currently deciding target
    currentHighest = calcHighestVisibility();
  }
  else{
    bool updatemat = 1;    //to trigger update
    if(selfNodeID == currentTarget)
      currVis = 1;
    else
      currVis = calcVisibility();       //if no nodes around, will get 0
    currRootVis = calcRootVisibility(); //if root not around, will get 0
    Serial.printf("RootVis:%.2f,NodeVis:%.2f\n",currRootVis,currVis);
    
    //if currRootVis is 0, root is nowhere to be found so robot swarm will just keep moving (impossible not found)
    //if currVis is 0, then considered as converged and moving towards root
    
    //Check if robot converged with other nodes first
    if(currVis == 0){ //if no nodes detected or nearby
      foundNodes = 1; //considered as converged, so that it will go towards root
    }
    else if(currVis>=1){ //if in range or converged
      foundNodes = 1; //converged, wait for other nodes to converge
    }
    else{
      foundNodes = 0; //not converged
    }
    //Check if robot converged with root
    if(currRootVis == 0){ //if no root detected or nearby
      foundRoot = 0;  //considered as not converged, will keep moving
    }
    else if(currRootVis>=1){  //if in range of root
      foundRoot = 1;  //converged
    }
    else{
      foundRoot = 0;  //not converged, keep moving
    }
    //Check if currently going towards root
    if(gotoRoot){     //if currently moving to root or no nodes nearby
      if(foundRoot){  //if already inside range
        updatemat = 0;//dont update matrix (value larger than 1)
      }
      else{           //if not in range yet
        updatemat = 1;//update matrix
      }
    }
    else{ //if currently moving towards nodes
      if(foundNodes){ //if already inside range 
        updatemat = 0;//dont update
      }
      else{           //if not in range yet
        updatemat = 1;//update
      }
    }
    //update matrix
    if(updatemat){
      if(MATRIX_direction == 0){  //first move
        matrix[currX][currY] += currVis;  //only update current square, because surrounding unknown
        rootmatrix[currX][currY] += currRootVis;  //update for root also
      }
      else{ //if moved before, update pheromone according to direction moved
        if(MATRIX_bumped){  //if bumped when moving
          if(currVis < 1 && currRootVis < 1){ //if either other nodes or root are not nearby, then consider obstacle
            MATRIX_Bump();  //move back to previous location
            matrix[nextX][nextY] = -1;  //set target destination as obstacle
          }
          //otherwise, do nothing, assume the robot bumped into root or other bot
        }
        else{ //if not bumped, then update current position, and update pheromones
          prevX = currX; prevY = currY;
          currX = nextX; currY = nextY;
          update_Pheromones();
        }
        MATRIX_bumped = 0;  //reset bump trigger
      }
    }
    adjustLED();
    Serial.printf("Prev Direction:%d\n",MATRIX_direction);
    if(!gotoRoot){
      Serial.println("Node Matrix:");
      MATRIX_print(matrix);
    }
    else{
      Serial.println("Root Matrix:");
      MATRIX_print(rootmatrix); 
    }
  }
}
void update_Pheromones(){
  float pheromone = 0;
  int larger = 0, rootlarger = 0;
  if(currVis == prevVis){ //if there are no nodes, or if same visibility
    matrix[prevX][prevY] = 0.5*matrix[prevX][prevY];  //half the pheromone, so that try not to return to same place
  }
  else{
    if(currVis > prevVis)
      larger = 1;
    switch(MATRIX_direction){
      case 1: MATRIX_1(matrix,currVis,larger);break;
      case 2: MATRIX_2(matrix,currVis,larger);break;
      case 3: MATRIX_3(matrix,currVis,larger);break;
      case 4: MATRIX_4(matrix,currVis,larger);break;
      case 6: MATRIX_6(matrix,currVis,larger);break;
      case 7: MATRIX_7(matrix,currVis,larger);break;
      case 8: MATRIX_8(matrix,currVis,larger);break;
      case 9: MATRIX_9(matrix,currVis,larger);break;
    }
  }
  
  if(currRootVis == prevRootVis){
    rootmatrix[prevX][prevY] = 0.5*rootmatrix[prevX][prevY];  //half the pheromone, so that try not to return to same place
  }
  else{
    if(currRootVis > prevRootVis)
      rootlarger = 1;
    switch(MATRIX_direction){
      case 1: MATRIX_1(rootmatrix,currRootVis,rootlarger);break;
      case 2: MATRIX_2(rootmatrix,currRootVis,rootlarger);break;
      case 3: MATRIX_3(rootmatrix,currRootVis,rootlarger);break;
      case 4: MATRIX_4(rootmatrix,currRootVis,rootlarger);break;
      case 6: MATRIX_6(rootmatrix,currRootVis,rootlarger);break;
      case 7: MATRIX_7(rootmatrix,currRootVis,rootlarger);break;
      case 8: MATRIX_8(rootmatrix,currRootVis,rootlarger);break;
      case 9: MATRIX_9(rootmatrix,currRootVis,rootlarger);break;
    }
  }
}
void MATRIX_1(float matrix[width][height],float pheromone, int larger){
  if(larger){
    int reference[5][5] = {
      { 1, 1, 1, 0, 0},
      { 1, 1, 0,-1, 0},
      { 1, 0,-1,-1, 0}, //m
      { 0,-1,-1,-1, 0},
      { 0, 0, 0, 0, 0},
    };      //m
    MATRIX_updatemat(matrix, reference, pheromone, prevX, prevY);
  }
  else{
    int reference[5][5] = {
      {-1,-1,-1, 0, 0},
      {-1,-1, 0, 1, 0},
      {-1, 0, 1, 1, 0}, //m
      { 0, 1, 1, 1, 0},
      { 0, 0, 0, 0, 0}
    };      //m
    MATRIX_updatemat(matrix, reference, pheromone, prevX, prevY);
  }
}
void MATRIX_2(float matrix[width][height],float pheromone, int larger){
  if(larger){
    int reference[5][5] = {
      { 0, 1, 1, 1, 0},
      { 0, 0, 1, 0, 0},
      { 0,-1,-1,-1, 0}, //m
      { 0,-1,-1,-1, 0},
      { 0, 0, 0, 0, 0},
    };      //m
    MATRIX_updatemat(matrix, reference, pheromone, prevX, prevY);
  }
  else{
    int reference[5][5] = {
      { 0,-1,-1,-1, 0},
      { 0,-1,-1,-1, 0},
      { 0, 0, 1, 0, 0}, //m
      { 0, 1, 1, 1, 0},
      { 0, 0, 0, 0, 0}
    };      //m
    MATRIX_updatemat(matrix, reference, pheromone, prevX, prevY);
  }
}
void MATRIX_3(float matrix[width][height],float pheromone, int larger){
  if(larger){
    int reference[5][5] = {
      { 0, 0, 1, 1, 1},
      { 0,-1, 0, 1, 1},
      { 0,-1,-1, 0, 1}, //m
      { 0,-1,-1,-1, 0},
      { 0, 0, 0, 0, 0},
    };      //m
    MATRIX_updatemat(matrix, reference, pheromone, prevX, prevY);
  }
  else{
    int reference[5][5] = {
      { 0, 0,-1,-1,-1},
      { 0, 1, 0,-1,-1},
      { 0, 1, 1, 0,-1}, //m
      { 0, 1, 1, 1, 0},
      { 0, 0, 0, 0, 0}
    };      //m
    MATRIX_updatemat(matrix, reference, pheromone, prevX, prevY);
  }
}
void MATRIX_4(float matrix[width][height],float pheromone, int larger){
  if(larger){
    int reference[5][5] = {
      { 0, 0, 0, 0, 0},
      { 1, 0,-1,-1, 0},
      { 1, 1,-1,-1, 0}, //m
      { 1, 0,-1,-1, 0},
      { 0, 0, 0, 0, 0},
    };      //m
    MATRIX_updatemat(matrix, reference, pheromone, prevX, prevY);
  }
  else{
    int reference[5][5] = {
      { 0, 0, 0, 0, 0},
      {-1,-1, 0, 1, 0},
      {-1,-1, 1, 1, 0}, //m
      {-1,-1, 0, 1, 0},
      { 0, 0, 0, 0, 0}
    };      //m
    MATRIX_updatemat(matrix, reference, pheromone, prevX, prevY);
  }
}
void MATRIX_6(float matrix[width][height],float pheromone, int larger){
  if(larger){
    int reference[5][5] = {
      { 0, 0, 0, 0, 0},
      { 0,-1,-1, 0, 1},
      { 0,-1,-1, 1, 1}, //m
      { 0,-1,-1, 0, 1},
      { 0, 0, 0, 0, 0},
    };      //m
    MATRIX_updatemat(matrix, reference, pheromone, prevX, prevY);
  }
  else{
    int reference[5][5] = {
      { 0, 0, 0, 0, 0},
      { 0, 1, 0,-1,-1},
      { 0, 1, 1,-1,-1}, //m
      { 0, 1, 0,-1,-1},
      { 0, 0, 0, 0, 0}
    };      //m
    MATRIX_updatemat(matrix, reference, pheromone, prevX, prevY);
  }
}
void MATRIX_7(float matrix[width][height],float pheromone, int larger){
  if(larger){
    int reference[5][5] = {
      { 0, 0, 0, 0, 0},
      { 0,-1,-1,-1, 0},
      { 1, 0,-1,-1, 0}, //m
      { 1, 1, 0,-1, 0},
      { 1, 1, 1, 0, 0},
    };      //m
    MATRIX_updatemat(matrix, reference, pheromone, prevX, prevY);
  }
  else{
    int reference[5][5] = {
      { 0, 0, 0, 0, 0},
      { 0, 1, 1, 1, 0},
      {-1, 0, 1, 1, 0}, //m
      {-1,-1, 0, 1, 0},
      {-1,-1,-1, 0, 0}
    };      //m
    MATRIX_updatemat(matrix, reference, pheromone, prevX, prevY);
  }
}
void MATRIX_8(float matrix[width][height],float pheromone, int larger){
  if(larger){
    int reference[5][5] = {
      { 0, 0, 0, 0, 0},
      { 0,-1,-1,-1, 0},
      { 0,-1,-1,-1, 0}, //m
      { 0, 0, 1, 0, 0},
      { 0, 1, 1, 1, 0},
    };      //m
    MATRIX_updatemat(matrix, reference, pheromone, prevX, prevY);
  }
  else{
    int reference[5][5] = {
      { 0, 0, 0, 0, 0},
      { 0, 1, 1, 1, 0},
      { 0, 0, 1, 0, 0}, //m
      { 0,-1,-1,-1, 0},
      { 0,-1,-1,-1, 0}
    };      //m
    MATRIX_updatemat(matrix, reference, pheromone, prevX, prevY);
  }
}
void MATRIX_9(float matrix[width][height],float pheromone, int larger){
  if(larger){
    int reference[5][5] = {
      { 0, 0, 0, 0, 0},
      { 0,-1,-1,-1, 0},
      { 0,-1,-1, 0, 1}, //m
      { 0,-1, 0, 1, 1},
      { 0, 0, 1, 1, 1},
    };      //m
    MATRIX_updatemat(matrix, reference, pheromone, prevX, prevY);
  }
  else{
    int reference[5][5] = {
      { 0, 0, 0, 0, 0},
      { 0, 1, 1, 1, 0},
      { 0, 1, 1, 0,-1}, //m
      { 0, 1, 0,-1,-1},
      { 0, 0,-1,-1,-1}
    };      //m
    MATRIX_updatemat(matrix, reference, pheromone, prevX, prevY);
  }
}
void MATRIX_updatemat(float matrix[width][height], int reference[5][5], float pheromone, int posx, int posy){
  //shift the center to top left corner, eg (2,2) -> (0,0), (8,5) -> (6,3)
  int x = posx - 2;
  int y = posy - 2;
  float pheromoneval = 0;
  for(int i=0;i<5;i++){
    for(int j=0;j<5;j++){
      pheromoneval = (float)reference[i][j]*pheromone;  //either plus or minus or nothing depending on reference
      MATRIX_updatecell(matrix, pheromoneval, x+i, y+j);
    }
  }
}
void MATRIX_updatecell(float matrix[width][height], float pheromone, int posx, int posy){
  //Check if position within bounds
  if( (posx >= 0) && (posy >= 0) && (posx < width) && (posy < height)){
    //check if position has obstacle, or if after deducting will be less than 0
    if( (matrix[posx][posy] > 0) ){ //if position is not obstacle (-1)
      if( (matrix[posx][posy] + pheromone) < 0) //if after deduct is less than 0
        matrix[posx][posy] = 0; //directly 0
      else
        matrix[posx][posy] += pheromone;
    }
  }
}
void MATRIX_Bump(){
  bool b_bumped = B_bumped;
  L_bumped = 0; R_bumped = 0; F_bumped = 0; B_bumped = 0;
  distanceToMove = (float)WHEEL_CIRCUMFERENCE/20;
  distanceToMove = (float)movedTss*distanceToMove;
  if(!b_bumped)
    PID_Backward();
  else
    PID_Forward();
  distanceToMove = 100;
  movedTss = 0;
}
void MATRIX_reset(float matrix[width][height]){
  for(int i=0;i<height;i++){
    for(int j=0;j<width;j++){
      matrix[i][j] = (float)initialPheromone;
    }
  }
}
void MATRIX_print(float matrix[width][height]){
  for(int i=0;i<height;i++){  //i is row, X
    for(int j=0;j<width;j++){ //j is column, Y
      if(j == currY && i == currX)
        Serial.printf("[%.2f]",matrix[i][j]);
      else
        Serial.printf(" %.2f ",matrix[i][j]);
    }
    Serial.println("");
  }
  Serial.println("");
}
float calcVisibility(){
  float visibility = 0;
  for(int i=0;i<(currentNodeCount-1);i++){  //find distance to target
    if(availableNetworks[i].intmac == currentTarget){
      float temp = (1.0/availableNetworks[i].distance);
      if(temp > 1)  //if temp > 1, means inside range already
        temp = 1;   //cap value at 1, where 1 = in range
      return temp;
    }
  }
  return 0; //nothing visible, or nothing nearby or exist
}
float calcRootVisibility(){
  float visibility = 0;
  for(int i=0;i<(currentNodeCount-1);i++){  //find distance to root
    if(availableNetworks[i].intmac == ROOTID){
      float temp = (1.0/availableNetworks[i].distance);
      if(temp > 1)  //if temp > 1, means inside range already
        temp = 1;   //cap value at 1, where 1 = in range
      return temp;
    }
  }
  return 0; //if root not found, return 0
}
float calcHighestVisibility(){
  int nodeNum = 0;
  float visibility = 0;
  for(int i=0;i<(currentNodeCount-1);i++){
    float temp = (1.0/availableNetworks[i].distance);
    visibility += temp;
  }
  return visibility;
}
/*End of ACO Movement Code----------------------------------------------------*/
/*Master Code-----------------------------------------------------------------*/
Task master(10,TASK_FOREVER,&Master);

long int startup_timer = 0;
long int wait_time = 0;

int Master(){
  if(!START){ //wait for command to start moving
    LED_1(0);
    LED_0(0);
    return 0;
  }
  else{
    if(isROOT){
      if(step_counter == 0){
        waitForNodes();
      }
      if(step_counter == 1){
        if(!startScan){
          scanNodes.restart();
        }
        if(getScan){
          startScan = 0;
          getScan = 0;
          step_counter++;
        }
      }
      if(step_counter == 2){
        pingNodes();
        step_counter++;
        Serial.println("done");
        step_counter = 0;
      }
    }
    else{
      if(step_counter == 0){
        waitForSeconds(3);
      }
      if(step_counter == 1){
        if(!startScan){
          scanNodes.restart();
        }
        if(getScan){
          startScan = 0;
          getScan = 0;
          step_counter++;
        }
      }
      if(step_counter == 2){
        MATRIX_update();  //update pheromone
        pingRoot();       //signal other nodes
        step_counter++;  
      }
      if(step_counter == 3){
        waitForRoot();    //wait for other nodes to be ready
      }
      if(step_counter == 4){
        LED_1(0);         //off LED while moving
        MATRIX_selectdirection();       //decide direction
        MATRIX_move(MATRIX_direction);  //move to direction
        step_counter++;
      }
      if(step_counter == 5){
        Serial.println("done");
        step_counter = 0;
      }
    }
  }
}
bool startwaiting = 0;
long int waitforseconds = 0;
void waitForSeconds(int seconds){
  if(!startwaiting){
    waitforseconds = millis();
    startwaiting = 1;
  }
  else{
    if(millis() - waitforseconds > (seconds*1000)){
      startwaiting = 0;
      step_counter++;
    }
  }
}
/*End of Master Code--------------------=-------------------------------------*/
/*Utility Code----------------------------------------------------------------*/
// Select SDA and SCL pins for I2C communication
const uint8_t scl = D1;
const uint8_t sda = D3;

void Serial_Setup() {
  if (SERIAL)
    Serial.begin(115200);
  if (SERIAL_GPIO)
    TXRX_to_GPIO();
  else
    TXRX_to_DEFAULT();
}
void I2C_Setup() {
  if (I2C_COM)
    Wire.begin(sda, scl);
}
/*End of Utility Code---------------------------------------------------------*/
/*Main Code-------------------------------------------------------------------*/
void setup() {
  delay(1000);
  Serial_Setup();
  if (!isROOT) {
    I2C_Setup();
    BATTERY_Setup();
    BUTTON_Setup();
    PCF8574_Setup();
    ENCODER_Setup();
    PID_MOTOR_Setup();
    QMC5883L_Setup();
    MPU6050_Setup();
    FUSION_Setup();
    if(STARTUP_NORTH){
      PID_North();
    }
    mesh_Setup();
    WiFi_Setup();
    MATRIX_Setup();
  }
  else {
    mesh_Setup();
    WiFi_Setup();
    WebSocket_Setup(); 
  }
  startup_timer = millis();
  taskScheduler.addTask(master);
  master.enable();
}

void loop() {
  taskScheduler.execute();
  if (MESHNETWORK) {
    mesh.update();
  }
  if (WEBSOCKET) {
    webSocket.loop();
  }
}
/*End of Main Code------------------------------------------------------------*/
