/* Note:
 *  -Try to keep medium tasks <=2ms, low tasks <=15ms
 *  -NodeMCU Soft Access Point can handle max 5 connections
 */
/*Control Panel---------------------------------------------------------------*/
#include <ESP8266WiFi.h>
#include <list>
#include "IPAddress.h"
#include "painlessMesh.h"
#include "Hash.h"
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSocketsServer.h>

//1 to turn on, 0 to turn off
#define WIFIAPMODE  0 //for configuring own access point
#define ASYNCSERVER 0 //for setup of AsyncWebServer
#define WEBSOCKET   1 //for setup of WebSocket
#define MESHNETWORK 1 //for setup of Mesh network

Scheduler taskScheduler;
 /*End of Control Panel-------------------------------------------------------*/
/*WiFi Codes------------------------------------------------------------------*/
/* The scanNetwork and saveNetwork functions will delete all previously saved wifi info,
 * scans for available wifi signals, then sort it according to RSSI in descending order,
 * then save all wifi signal info in the struct array availableNetworks. Once scanning
 * and saving is done, it will set the scanCompleted variable, otherwise it will stay 0.
 * Note:Typically takes ~2.184s to scan and store available wifi networks
 */
#define WIFI_MODE WIFI_AP_STA //set wifi mode here, AP_STA is access point and station mode together
#define WIFI_SSID "Swarm_Intel" //same as mesh_wifi
#define WIFI_PASSWORD "password"
#define WIFI_MAX 15
#define WIFI_CHAR 15
#define STORE_SWARM_ONLY 1  //1 for yes, 0 for no

typedef struct{
  char ssid[WIFI_CHAR];
  char mac[WIFI_CHAR];
  int rssi;
  bool secured = 0; //0 if unsecured, 1 if secured
} wifiNetwork;    //wifi into structure/map

wifiNetwork availableNetworks[WIFI_MAX];  //declares an array to store network info
char ownMAC[WIFI_CHAR];
bool scanCompleted = 0; //0 if haven't complete scanning, 1 if done
int samples = 0;

void scanNetwork();
void saveNetwork();

Task scanNodes(0,TASK_ONCE,&scanNetwork);
Task checkScan(5000,TASK_FOREVER,&saveNetwork);

void saveOwnMac(){
  WiFi.macAddress().toCharArray(ownMAC,sizeof(ownMAC));
  char tempMAC[sizeof(ownMAC)];  //temporary storage for MAC without ':'
  int tempMACnum = 0; //counter for temporary storage
  for(int k = 0;k<sizeof(ownMAC);k++){
    if(ownMAC[k] != ':'){ //find and remove ':' characters
      tempMAC[tempMACnum] = ownMAC[k];  //store non-':' char into temp storage
      tempMACnum++;
    }
    ownMAC[k] = 0;  //clear original storage
  }
  for(int L = 0;L<tempMACnum;L++){
    ownMAC[L] = tempMAC[L]; //copy over non-':' char from temp storage to cleared original storage
  }
}

void scanNetwork(){
  scanCompleted = 0;
  memset(availableNetworks,(char)0,WIFI_MAX);  //clears all previous wifi info
  WiFi.mode(WIFI_MODE);  //station mode = 3, access point + station
  WiFi.scanDelete();  //reset scanned connections
  WiFi.scanNetworks(true, false); //scan asynchronously, but has to check with .scanComplete, (async,showhidden)
  samples++;  //to check how many times scanned
}

void saveNetwork(){
  int networksFound = WiFi.scanComplete();  //store scan status/networks found
  int swarmNetworks = 0;
  if(networksFound<0){  //scan issues
    if(networksFound == -1){  //scanning in process
      //Serial.println("Scanning still in progress");
    }
    else if(networksFound == -2){ //scan not triggered
      //Serial.println("Scan not triggered");
      scanNodes.restartDelayed(200);
    }
  }
  else{ //scan completed
    if (networksFound == 0){  //no networks found
      //Serial.println("No networks found");  //do nothing, or add some codes
    }
    else if(STORE_SWARM_ONLY) { //stores only "Swarm_Intel" SSIDs
      for(int i = 0;i<networksFound;i++){ //store wifi info while sorting
        if(WiFi.SSID(i) == WIFI_SSID){  //compare String(ssid) with "Swarm_Intel"
          /*Store all collected data into wifiNetwork object first*/
          WiFi.SSID(i).toCharArray(availableNetworks[swarmNetworks].ssid,sizeof(availableNetworks[swarmNetworks].ssid));    //store ssid
          WiFi.BSSIDstr(i).toCharArray(availableNetworks[swarmNetworks].mac,sizeof(availableNetworks[swarmNetworks].mac));  //store MAC with ':'
          availableNetworks[swarmNetworks].rssi = WiFi.RSSI(i); //store RSSI in dbm
          WiFi.encryptionType(i) == ENC_TYPE_NONE?  //check encryption type
            availableNetworks[swarmNetworks].secured = 0
            :
            availableNetworks[swarmNetworks].secured = 1;
          /*Filter out ":" character from the stored mac address*/
          char tempMAC[sizeof(availableNetworks[swarmNetworks].mac)];  //temporary storage for MAC without ':'
          int tempMACnum = 0; //counter for temporary storage
          for(int k = 0;k<sizeof(availableNetworks[swarmNetworks].mac);k++){
            if(availableNetworks[swarmNetworks].mac[k] != ':'){ //find and remove ':' characters
              tempMAC[tempMACnum] = availableNetworks[swarmNetworks].mac[k];  //store non-':' char into temp storage
              tempMACnum++;
            }
            availableNetworks[swarmNetworks].mac[k] = 0;  //clear original storage
          }
          for(int L = 0;L<tempMACnum;L++){
            availableNetworks[swarmNetworks].mac[L] = tempMAC[L]; //copy over non-':' char from temp storage to cleared original storage
          }
          /*Sort the collected Swarm macs according to RSSI in descending order*/
          if(swarmNetworks>0){  //only start sorting after at least 2 data stored, sorts every i increment (on every new network added)
            for(int j=swarmNetworks;(availableNetworks[j].rssi>availableNetworks[j-1].rssi && j>0);j--){  //insertion sort, descending order
              wifiNetwork tempNetwork=availableNetworks[j];
              availableNetworks[j]=availableNetworks[j-1];
              availableNetworks[j-1]=tempNetwork;
            }
          }
          swarmNetworks++;
        }
      }
      /*Print out data into excel sheet*/
      Serial.printf("DATA,TIME,%d,",samples);
      Serial.print(ownMAC);
      for(int i = 0;i<swarmNetworks;i++){
        if(strcmp(availableNetworks[i].ssid,WIFI_SSID)==0){ //check if SSID is "Swarm_Intel" although theoretically should only have that
          Serial.print(",");
          Serial.print(availableNetworks[i].mac);
          Serial.printf(",%d",availableNetworks[i].rssi);
        }
      }
      Serial.println(",AUTOSCROLL_20"); //to autoscroll excel sheet and go to next line
    }
    else{ //Stores all known SSIDs
      Serial.printf("%d networks found\n",networksFound);
      for(int i=0;i<networksFound;i++){ //store wifi info while sorting
        /*Store all collected data into wifiNetwork object first*/
        WiFi.SSID(i).toCharArray(availableNetworks[i].ssid,sizeof(availableNetworks[i].ssid));    //store ssid
        WiFi.BSSIDstr(i).toCharArray(availableNetworks[i].mac,sizeof(availableNetworks[i].mac));  //store MAC with ':'
        availableNetworks[i].rssi = WiFi.RSSI(i); //store RSSI in dbm
        WiFi.encryptionType(i) == ENC_TYPE_NONE?  //check encryption type
          availableNetworks[i].secured = 0
          :
          availableNetworks[i].secured = 1;
        /*Filter out ":" character from the stored mac address*/
        char tempMAC[sizeof(availableNetworks[i].mac)];  //temporary storage for MAC without ':'
        int tempMACnum = 0; //counter for temporary storage
        for(int k = 0;k<sizeof(availableNetworks[i].mac);k++){
          if(availableNetworks[i].mac[k] != ':'){ //find and remove ':' characters
            tempMAC[tempMACnum] = availableNetworks[i].mac[k];  //store non-':' char into temp storage
            tempMACnum++;
          }
          availableNetworks[i].mac[k] = 0;  //clear original storage
        }
        for(int L = 0;L<tempMACnum;L++){
          availableNetworks[i].mac[L] = tempMAC[L]; //copy over non-':' char from temp storage to cleared original storage
        }
        /*Sort the collected Swarm macs according to RSSI in descending order*/
        if(i>0){  //only start sorting after 2 data stored, sorts every i increment (on every new network added)
          for(int j=swarmNetworks;(availableNetworks[j].rssi>availableNetworks[j-1].rssi && j>0);j--){  //insertion sort, descending order
            wifiNetwork tempNetwork=availableNetworks[j];
            availableNetworks[j]=availableNetworks[j-1];
            availableNetworks[j-1]=tempNetwork;
          }
        }
      }
      /*Print out all found networks with their RSSI value*/
      for(int i=0;i<networksFound;i++){
        Serial.printf("%d:",i+1);
        Serial.print(availableNetworks[i].ssid);
        Serial.printf("(%d) ",availableNetworks[i].rssi);
        Serial.println(availableNetworks[i].mac);
      }
    }
  scanCompleted=1;
  scanNodes.restartDelayed(200);
  }
}

/*To set as Access point mode*/
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
/*Async Server & WebSocket Codes----------------------------------------------*/
#define   STATION_SSID     "Swarm"
#define   STATION_PASSWORD "password"
#define HOSTNAME "HTTP_BRIDGE"
IPAddress myIP(0,0,0,0);
IPAddress myAPIP(0,0,0,0);

IPAddress local_IP(192,168,43,111);
IPAddress gateway(192,168,43,1);
IPAddress subnet(255,255,255,0);
IPAddress primaryDNS(8,8,8,8);
IPAddress secondaryDNS(8,8,4,4);

AsyncWebServer server(80);
WebSocketsServer webSocket(81);

void AsyncServer_Setup(){
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", "HELLO WORLD");
    if (request->hasArg("BROADCAST")){
      String msg = request->arg("BROADCAST");
      Serial.println(msg);
    }
  });
  server.begin();
}

void WebSocket_Setup(){
  webSocket.onEvent(webSocketEvent);
  webSocket.begin();
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t lenght){
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
      break;
  }
}

void sendNodeData(const char data[]){
  //sprintf(data,"%s:Sample,%d\n%s:Sample,%d\n",mac,sample,mac2,sample);
  webSocket.broadcastTXT(data);
  Serial.println(data);
}

//Task sendThings(1000,TASK_FOREVER,&sendNodeData);

/*End of Async Server & WebSocket Codes---------------------------------------*/
/*Mesh Codes------------------------------------------------------------------*/
#define MESH_SSID "Swarm_Intel"
#define MESH_PASSWORD "password"
#define MESH_PORT 5555

int currentNodeCount = 0;
uint32_t nodeList[10];
uint32_t selfNodeID;
uint32_t currentMaster;

painlessMesh mesh;

//Task sendThings(1000,TASK_FOREVER,&sendNodeData);
Task self_blink_on(1000,TASK_FOREVER,&LED_ON);
Task self_blink_off(0,TASK_ONCE,&LED_OFF);
Task communicate(5000,TASK_FOREVER,&communicateWithNodes);

void meshSetup(){
  //WiFi.persistent(false);
  WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS);
  
  mesh.setDebugMsgTypes( ERROR | STARTUP | CONNECTION );  // set before init() so that you can see startup messages
  mesh.init(MESH_SSID,MESH_PASSWORD,&taskScheduler,MESH_PORT, WIFI_AP_STA, 6);
  mesh.onReceive(&sync);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&onChangedConnections);

  //mesh.stationManual(STATION_SSID, STATION_PASSWORD);
  //mesh.setHostname(HOSTNAME);
  // Bridge node, should (in most cases) be a root node. See [the wiki](https://gitlab.com/painlessMesh/painlessMesh/wikis/Possible-challenges-in-mesh-formation) for some background
  //mesh.setRoot(true);
  // This node and all other nodes should ideally know the mesh contains a root, so call this on all nodes
  mesh.setContainsRoot(true);

  myAPIP = IPAddress(mesh.getAPIP());
  Serial.println("My AP IP is " + myAPIP.toString());
  
  selfNodeID = mesh.getNodeId();
  currentMaster = selfNodeID;
  addSelfToNodeList();
  saveOwnMac();
  
  Serial.println("CLEARSHEET");
  Serial.println("LABEL,timestamp,sample,ownMAC,MAC_1,RSSI_1,MAC_2,RSSI_2,MAC_3,RSSI_3,MAC_4,RSSI_4,MAC_5,RSSI_5");

  taskScheduler.addTask(communicate);
  taskScheduler.addTask(self_blink_on);
  taskScheduler.addTask(self_blink_off);
  taskScheduler.addTask(scanNodes);
  taskScheduler.addTask(checkScan);
  scanNodes.enable();
  checkScan.enable();
  communicate.enable();
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
int SERVERON = 0;
void communicateWithNodes(){
  Serial.printf("Node ID is:%u\n",selfNodeID);
  Serial.printf("Node Time is:%uus\n",mesh.getNodeTime());  //in us
  if(currentMaster == selfNodeID){
    DynamicJsonDocument doc(1024);
    doc["Blink"]="I am slave";
    String msg;
    serializeJson(doc,msg);
    char text[] = "aabbcc:lalalala\n";
    if(SERVERON){
      sendNodeData(text);
    }
    else if(WiFi.status() == 3){
      Serial.println("Enabling WebSocket");
      WebSocket_Setup();
      SERVERON = 1;
    }
    mesh.sendBroadcast(msg);
    self_blink_on.enable();
    Serial.println("I am master");
  }
  else{
    if(SERVERON){
      Serial.println("Disabling WebSocket");
      webSocket.close();
      SERVERON = 0;
    }
  }
  printNodeList();
  printMeshTopology();
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
void printMeshTopology(){
  Serial.println(mesh.subConnectionJson());
}
void selectMaster(){
  uint32_t largest = selfNodeID;
  for(int x=0;x<currentNodeCount;x++){
    if(nodeList[x]>largest)
      largest = nodeList[x];
  }
  currentMaster = largest;
  if(currentMaster != selfNodeID){
    Serial.println("Root Disabled");
    mesh.setRoot(false);
  }
  else{
    mesh.setRoot(true);
  }
  Serial.printf("Current Master:%u\n",currentMaster);
}
void LED_ON(){
  digitalWrite(BUILTIN_LED,LOW);
  self_blink_off.restartDelayed(100);
}
void LED_OFF(){
  digitalWrite(BUILTIN_LED,HIGH);
}
/*End of Mesh Codes-------------------------------------------------------------*/
/*Main Code---------------------------------------------------------------------*/
// Select SDA and SCL pins for I2C communication 
const uint8_t scl = D1;
const uint8_t sda = D3;

void setup() {
  Serial.begin(115200);
  pinMode(BUILTIN_LED,OUTPUT);
  meshSetup();
}

void loop() {
  mesh.update();
  webSocket.loop();
  taskScheduler.execute();
}
/*End of Main Code--------------------------------------------------------------*/
