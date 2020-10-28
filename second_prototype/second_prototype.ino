/* Note:
 *  -Try to keep medium tasks >=2ms, low tasks >=15ms
 *  -NodeMCU Soft Access Point can handle max 5 connections
 */
/*WiFi Codes------------------------------------------------------------------*/
#include <ESP8266WiFi.h>
#include <list>
#define WIFI_MODE WIFI_AP_STA //set wifi mode here, AP_STA is access point and station mode together
#define WIFI_MAX 15
typedef struct{
  String mac;
  String ssid;
  int rssi;
  bool secured=0; //0 if unsecured, 1 if secured
} wifiNetwork;    //wifi into structure/map
wifiNetwork availableNetworks[WIFI_MAX];  //declares an array to store network info
bool scanCompleted=0; //0 if haven't complete scanning, 1 if done
/* The scanNetwork and saveNetwork functions will delete all previously saved wifi info,
 * scans for available wifi signals, then sort it according to RSSI in descending order,
 * then save all wifi signal info in the struct array availableNetworks. Once scanning
 * and saving is done, it will set the scanCompleted variable, otherwise it will stay 0.
 * Note:Typically takes ~2.184s to scan and store available wifi networks
 */
#include "painlessMesh.h"
void scanNetwork(){
  scanCompleted=0;
  memset(availableNetworks,(char)0,WIFI_MAX);  //clears all previous wifi info
  WiFi.mode(WIFI_MODE);  //station mode = 3, access point + station
  WiFi.scanDelete();
  //Serial.println("Scanning for WiFi networks...");
  WiFi.scanNetworksAsync(saveNetwork,true); //asynchronously scan for networks available(wont wait)
}
Task scanNodes(0,TASK_ONCE,&scanNetwork);
void saveNetwork(int networksFound){
  int n = networksFound;  //retrieve scanned networks number
  if (n == 0)
    //Serial.println("No networks found");
    delay(1);
  else{ 
    //Serial.printf("%d networks found\n",n);
    for(int i=0;i<n;i++){ //store wifi info while sorting
      //availableNetworks[i].ssid=WiFi.SSID(i);
      availableNetworks[i].mac=WiFi.BSSIDstr(i);
      availableNetworks[i].ssid=WiFi.SSID(i);
      availableNetworks[i].rssi=WiFi.RSSI(i);
      WiFi.encryptionType(i) == ENC_TYPE_NONE?  //check encryption type
        availableNetworks[i].secured=0
        :
        availableNetworks[i].secured=1;
      if(i>0){
        for(int j=i;(availableNetworks[j].rssi>availableNetworks[j-1].rssi && j>0);j--){  //insertion sort, descending order
          wifiNetwork tempNetwork=availableNetworks[j];
          availableNetworks[j]=availableNetworks[j-1];
          availableNetworks[j-1]=tempNetwork;
        }
      }
    }
//    for(int i=0;i<n;i++){
//      Serial.printf("%d:",i+1);
//      Serial.print(availableNetworks[i].ssid);
//      Serial.print("(");
//      Serial.print(availableNetworks[i].rssi);
//      Serial.print(")\n");
//    }
    Serial.print("DATA,TIME");
    for(int i=0;i<n;i++){
      //if(availableNetworks[i].ssid == "Swarm_Intel"){
      if(true){
        Serial.print(",");
        Serial.print(availableNetworks[i].ssid);
        Serial.print(availableNetworks[i].mac);
        Serial.printf(",%d",availableNetworks[i].rssi);
      }
    }
    Serial.print(",AUTOSCROLL_20");
  }
  Serial.println("");
  scanCompleted=1;
  //scanNodes.disable();
  scanNodes.restartDelayed(200);
}
String SELF_SSID="ESP_";
#define SELF_PASSWORD "password"
#define SELF_CHANNEL 0
#define SELF_HIDDEN false
#define SELF_MAX_CONNECTION 8
void configureAccessPoint(){
  SELF_SSID.concat(String(ESP.getChipId(),HEX));  //SSID becomes "ESP_{chipID}"
  Serial.println(WiFi.softAP(SELF_SSID,SELF_PASSWORD,SELF_CHANNEL,SELF_HIDDEN,SELF_MAX_CONNECTION) ? "Soft AP Success" : "Soft AP Failed");
}
/*End of WiFi Codes-----------------------------------------------------------*/
/*Mesh Codes------------------------------------------------------------------*/
//#include <list>
//#include "painlessMesh.h"
#define MESH_SSID "Swarm_Intel"
#define MESH_PASSWORD "password"
#define MESH_PORT 5555

uint32_t nodeList[10];
int currentNodeCount = 0;
uint32_t selfNodeID;
uint32_t currentMaster;

Scheduler taskScheduler;
painlessMesh mesh;

//Task taskSendMessage(TASK_SECOND*1,TASK_FOREVER,&sendMessage);
Task communicate(5000,TASK_FOREVER,&communicateWithNodes);
Task self_blink_on(1000,TASK_FOREVER,&LED_ON);
Task self_blink_off(0,TASK_ONCE,&LED_OFF);
//Task scanNodes(0,TASK_ONCE,&scanNetwork);
Task enableTOA(1000,TASK_FOREVER,&TOAenabler);
Task doingTOA(0,TASK_ONCE,&sendTOA2);

void meshSetup(){
  mesh.setDebugMsgTypes(ERROR|STARTUP); 
  mesh.init(MESH_SSID,MESH_PASSWORD,&taskScheduler,MESH_PORT);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&onChangedConnections);
  //mesh.onNodeDelayReceived(&onNodeDelayReceived);
  mesh.onNodeDelayReceived(&receiveTOA);
  //mesh.onReceive(&sync);
  mesh.onReceive(&receiveTOA2);
  taskScheduler.addTask(communicate);
  taskScheduler.addTask(self_blink_on);
  taskScheduler.addTask(self_blink_off);
  taskScheduler.addTask(doingTOA);
  taskScheduler.addTask(enableTOA);
  taskScheduler.addTask(scanNodes);
  selfNodeID = mesh.getNodeId();
  currentMaster = selfNodeID;
  addSelfToNodeList();
  pinMode(BUILTIN_LED,OUTPUT);
  //communicate.enable();
  scanNodes.enable();
  //enableTOA.enable();
}
int TOAstarted = 0;
void TOAenabler(){
  if(currentNodeCount>1 && TOAstarted == 0){
    doingTOA.enable();
    Serial.println("CLEARSHEET");
    Serial.println("LABEL,Start(us),End(us),From,To,Delta(us),Delay(us),Error(us),Topology");
    TOAstarted = 1;
  }
  else if (currentNodeCount == 1){
    TOAstarted = 0;
  }
}
uint32_t sendTime, receiveTime;
void sendTOA2(){
  DynamicJsonDocument doc(1024);
  sendTime = mesh.getNodeTime();
  char sendtext[5];
  sprintf(sendtext,"%lu",sendTime);
  doc["TOA"]=String(sendtext);
  //Serial.println(sendtext);
  String msg;
  serializeJson(doc,msg);
  mesh.sendBroadcast(msg);
}
void receiveTOA2(uint32_t from, String &msg){
  uint32_t firstReceive = mesh.getNodeTime();
  String json;
  DynamicJsonDocument doc(1024);
  json=msg.c_str();
  deserializeJson(doc,json);
  String text=doc["TOA"];
  //Serial.println(text);
  char text2[11];
  text.toCharArray(text2,11);
  receiveTime = strtoul(text2,NULL,10);
  //Serial.println(receiveTime);
  uint32_t delayTime = firstReceive-receiveTime;
  //Serial.printf("%u,%u,%u\n",firstReceive,receiveTime,delayTime);
  if(delayTime<10000){
    Serial.printf("DATA,%u,%u,%u,%u,%u,%d,%u,",receiveTime,firstReceive,selfNodeID,from,0,delayTime,0);
    Serial.print(mesh.subConnectionJson());
    Serial.print(",AUTOSCROLL_20");
    Serial.println();
  }
  doingTOA.restartDelayed(1000);
}
void sendTOA(){
  if(currentNodeCount>1 && TOAstarted == 1){
    mesh.startDelayMeas(nodeList[1]);
    //Serial.printf("Started:%uus\n",mesh.getNodeTime());  //in us
    sendTime = mesh.getNodeTime();
    //Serial.printf("DATA,%u,",sendTime);
  }
}
void receiveTOA(uint32_t nodeID, int32_t nodeDelay){
  //Serial.printf("Ended:%uus\n",mesh.getNodeTime());
  //Serial.printf("From ID:%u Delay:%dus\n",nodeID,nodeDelay);
  if(nodeDelay<10000){
    uint32_t receiveTime = mesh.getNodeTime();
    uint32_t delta = (receiveTime-sendTime)/2;
    uint32_t error;
    if(delta>nodeDelay)
      error = delta-nodeDelay;
    else
      error = nodeDelay-delta;
    Serial.printf("DATA,%u,%u,%u,%u,%u,%d,%u,",sendTime,receiveTime,selfNodeID,nodeID,delta,nodeDelay,error);
    Serial.print(mesh.subConnectionJson());
    Serial.print(",AUTOSCROLL_20");
    Serial.println();
  }
  doingTOA.restartDelayed(1000);
}

void newConnectionCallback(uint32_t nodeID){
  updateNodeList();
  printNodeList();
  selectMaster();
}
void onChangedConnections(){
  updateNodeList();
  printNodeList();
  selectMaster();
}
void onNodeDelayReceived(uint32_t nodeID, int32_t nodeDelay){
  Serial.printf("From ID:%u Delay:%dus\n",nodeID,nodeDelay);  //in us
}
void sync(uint32_t from, String &msg){
  String json;
  DynamicJsonDocument doc(1024);
  json=msg.c_str();
  DeserializationError error = deserializeJson(doc,json);
  if (error)
  {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
  }
  String text=doc["Blink"];
  Serial.println(text);
  self_blink_on.enable();
}
void LED_ON(){
  digitalWrite(BUILTIN_LED,LOW);
  self_blink_off.restartDelayed(100);
}
void LED_OFF(){
  digitalWrite(BUILTIN_LED,HIGH);
}
void communicateWithNodes(){
  Serial.printf("Node ID is:%u\n",selfNodeID);
  Serial.printf("Node Time is:%uus\n",mesh.getNodeTime());  //in us
  if(currentMaster == selfNodeID){
    DynamicJsonDocument doc(1024);
    doc["Blink"]="I am slave";
    String msg;
    serializeJson(doc,msg);
    mesh.sendBroadcast(msg);
    self_blink_on.enable();
    Serial.println("I am master");
  }
  printNodeList();
  printMeshTopology();
  pingNodes();
}
void pingNodes(){
  for(int x=1;x<currentNodeCount;x++){
    mesh.startDelayMeas(nodeList[x]);
  }
}
void printMeshTopology(){
  Serial.println(mesh.subConnectionJson());
}
void addSelfToNodeList(){
  nodeList[0] = selfNodeID;
  currentNodeCount = 1;
}
void updateNodeList(){
  addSelfToNodeList();
  std::list <uint32_t> NodeList = mesh.getNodeList();
  std::list <uint32_t> :: iterator it;
  for(it = NodeList.begin(); it != NodeList.end(); ++it){
    nodeList[currentNodeCount] = *it;
    currentNodeCount++;
  }
}
void printNodeList(){
  Serial.printf("%d existing nodes:\n",currentNodeCount);
  for(int x=0;x<currentNodeCount;x++){
    Serial.println(nodeList[x]);
  }
}
void selectMaster(){
  uint32_t largest = selfNodeID;
  for(int x=0;x<currentNodeCount;x++){
    if(nodeList[x]>largest)
      largest = nodeList[x];
  }
  currentMaster = largest;
  Serial.printf("Current Master:%u\n",currentMaster);
}
void sendMessage(){
  DynamicJsonDocument doc(1024);
  doc["Test"]=String(ESP.getChipId(),HEX);
  String msg;
  serializeJson(doc,msg);
  mesh.sendBroadcast(msg);
  //taskSendMessage.setInterval((TASK_SECOND*1));
}
void receivedCallback(uint32_t from, String &msg){
  String json;
  DynamicJsonDocument doc(1024);
  json=msg.c_str();
  DeserializationError error = deserializeJson(doc,json);
  if (error)
  {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
  }
  String text=doc["Time"];
  Serial.print(String(ESP.getChipId(),HEX));
  Serial.print(":");
  Serial.println(text);
  //blink_led();
}
/*End of Mesh Codes-------------------------------------------------------------*/
/*QMC5883L Codes----------------------------------------------------------------*/
#include <QMC5883L.h>
#include <Wire.h>
QMC5883L compass;
void QMC5883L_setup(){
  compass.init();
  compass.setCalibration(3710,-5240,6042,-5175);
  Serial.println("QMC5883L Compass Demo");
  Serial.println("Turn compass in all directions to calibrate....");
}
void QMC5883L_print(){
  int16_t xmax,xmin,ymax,ymin;
  float heading = compass.readHeading();
  if(heading==0) {
    /* Still calibrating, so measure but don't print */
  } 
  else {
    compass.readCal(&xmax,&xmin,&ymax,&ymin);
    Serial.print("H:");
    Serial.print(heading);
    Serial.print(" X:");
    Serial.print(xmax);
    Serial.print(" x:");
    Serial.print(xmin);
    Serial.print(" Y:");
    Serial.print(ymax);
    Serial.print(" y:");
    Serial.println(ymin);
  }
}
/*End of QMC5883L Codes---------------------------------------------------------*/
/*MPU6050 Code------------------------------------------------------------------*/
#include <Wire.h>

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

void MPU6050_print(){
  double Ax, Ay, Az, T, Gx, Gy, Gz;
  
  Read_RawValue(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_XOUT_H);
  
  //divide each with their sensitivity scale factor
  Ax = (double)AccelX/AccelScaleFactor;
  Ay = (double)AccelY/AccelScaleFactor;
  Az = (double)AccelZ/AccelScaleFactor;
  T = (double)Temperature/340+36.53; //temperature formula
  Gx = (double)GyroX/GyroScaleFactor;
  Gy = (double)GyroY/GyroScaleFactor;
  Gz = (double)GyroZ/GyroScaleFactor;

  Serial.print("Ax: "); Serial.print(Ax);
  Serial.print(" Ay: "); Serial.print(Ay);
  Serial.print(" Az: "); Serial.print(Az);
  Serial.print(" T: "); Serial.print(T);
  Serial.print(" Gx: "); Serial.print(Gx);
  Serial.print(" Gy: "); Serial.print(Gy);
  Serial.print(" Gz: "); Serial.println(Gz);
}

void I2C_Write(uint8_t deviceAddress, uint8_t regAddress, uint8_t data){
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.write(data);
  Wire.endTransmission();
}

// read all 14 register
void Read_RawValue(uint8_t deviceAddress, uint8_t regAddress){
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, (uint8_t)14);
  AccelX = (((int16_t)Wire.read()<<8) | Wire.read());
  AccelY = (((int16_t)Wire.read()<<8) | Wire.read());
  AccelZ = (((int16_t)Wire.read()<<8) | Wire.read());
  Temperature = (((int16_t)Wire.read()<<8) | Wire.read());
  GyroX = (((int16_t)Wire.read()<<8) | Wire.read());
  GyroY = (((int16_t)Wire.read()<<8) | Wire.read());
  GyroZ = (((int16_t)Wire.read()<<8) | Wire.read());
}

//configure MPU6050
void MPU6050_Init(){
  delay(150);
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
/*End of MPU6050 Code-----------------------------------------------------------*/
/*Replace TXRX pins Code--------------------------------------------------------*/
const uint8_t TX = 1; //GPIO1 (TX)
const uint8_t RX = 3; //GPIO3 (RX)

void TXRX_to_GPIO(){
  pinMode(TX, FUNCTION_3);
  pinMode(RX, FUNCTION_3);
}

void TXRX_to_DEFAULT(){
  pinMode(TX, FUNCTION_0);
  pinMode(RX, FUNCTION_0);
}

bool STAT = 0;
int TEST_TIME = 0;
void Test_GPIO(){
  pinMode(TX,OUTPUT);
  pinMode(RX,OUTPUT);
  if(TEST_TIME == 0){
    TEST_TIME = millis();
  }
  else if(millis()-TEST_TIME>2000){
    STAT = ~STAT;
    digitalWrite(TX,STAT);
    digitalWrite(RX,STAT);
    TEST_TIME = 0;
  }
  
}
/*End of TXRX Replace Code------------------------------------------------------*/
/*Motor Driver Code-------------------------------------------------------------*/
const uint8_t M1A = D7;
const uint8_t M1B = D8;
const uint8_t M2A = RX;
const uint8_t M2B = D5;

void Test_Stop(){
  digitalWrite(M2A,0);
  digitalWrite(M2B,0);
  digitalWrite(M1A,0);
  digitalWrite(M1B,0);
}
void Motor_Setup(){
  pinMode(M1A,OUTPUT);
  pinMode(M1B,OUTPUT);
  pinMode(M2A,OUTPUT);
  pinMode(M2B,OUTPUT);
  Test_Stop();
}
void Test_Left(){
  analogWrite(M1A,600);
  digitalWrite(M1B,0);
  digitalWrite(M2A,0);
  digitalWrite(M2B,0);
}
void Test_Right(){
  analogWrite(M2A,600);
  digitalWrite(M2B,0);
  digitalWrite(M1A,0);
  digitalWrite(M1B,0);
}
/*End of Motor Driver Code------------------------------------------------------*/
/*Encoder Code------------------------------------------------------------------*/
const uint8_t L_EN = D2;
const uint8_t R_EN = D6;

void ICACHE_RAM_ATTR L_Callback(){
 //Serial.println("HMM LEFT");
 
}
void ICACHE_RAM_ATTR R_Callback(){
 //Serial.println("HMM RIGHT");
}

void Encoder_setup(){
  pinMode(L_EN,INPUT);
  pinMode(R_EN,INPUT);
  attachInterrupt(digitalPinToInterrupt(L_EN), L_Callback, FALLING);
  attachInterrupt(digitalPinToInterrupt(R_EN), R_Callback, FALLING);
}
/*End of Encoder Code-----------------------------------------------------------*/
/*Bump Button Code--------------------------------------------------------------*/
const uint8_t L_BUT = D4;
const uint8_t R_BUT = TX;
const uint8_t B_BUT = D0;

void ICACHE_RAM_ATTR L_Bump(){
 //Serial.println("LEFTED");
 Test_Left();
}
void ICACHE_RAM_ATTR R_Bump(){
 //Serial.println("RIGHTED");
 Test_Right();
}
void B_Bump(){
 //Serial.println("BACKED");
 Test_Stop();
}

void Button_setup(){
  pinMode(L_BUT,INPUT_PULLUP);
  pinMode(R_BUT,INPUT_PULLUP);
  pinMode(B_BUT,INPUT_PULLDOWN_16);
  attachInterrupt(digitalPinToInterrupt(L_BUT), L_Bump, FALLING);
  attachInterrupt(digitalPinToInterrupt(R_BUT), R_Bump, FALLING);
}
/*End of Bump Button Code-------------------------------------------------------*/
/*PCF8574 Code------------------------------------------------------------------*/
#include "PCF8574.h"
PCF8574 PCF(0x20);  //initialize pcf8574 address
void PCF8574_Setup(){
  PCF.begin(0xFF);  //initialize all pins to HIGH
}
void LED_0(int value){
  PCF.write(0,value); //0-1024
}
void LED_1(int value){
  PCF.write(1,value); //0-1024
}
void Buzzer(int value){
  PCF.write(2,value); //0 or 1
}
/*End of Bump Button Code-------------------------------------------------------*/
long previousMillis = 0;        // will store last time LED was updated
long interval = 500;           // interval at which to blink (milliseconds)
void blink_led(){
  previousMillis=millis();
  digitalWrite(BUILTIN_LED,HIGH);
  while(1){
    unsigned long currentMillis = millis();
    if(currentMillis-previousMillis>interval)
      break;
  }
  previousMillis=millis();
  digitalWrite(BUILTIN_LED,LOW);
  while(1){
    unsigned long currentMillis = millis();
    if(currentMillis-previousMillis>interval)
      break;
  }
}
/*Main Code---------------------------------------------------------------------*/
// Select SDA and SCL pins for I2C communication 
const uint8_t scl = D1;
const uint8_t sda = D3;

void setup() {
  Serial.begin(115200);
  Wire.begin(sda, scl);
  TXRX_to_DEFAULT();
  meshSetup();
}

void loop() {
  mesh.update();
}
/*End of Main Code--------------------------------------------------------------*/
