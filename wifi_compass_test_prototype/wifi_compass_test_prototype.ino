/* Note:
 *  -Try to keep medium tasks <=2ms, low tasks <=15ms
 *  -NodeMCU Soft Access Point can handle max 5 connections
 */
 
/*WiFi Codes------------------------------------------------------------------*/
/* The scanNetwork and saveNetwork functions will delete all previously saved wifi info,
 * scans for available wifi signals, then sort it according to RSSI in descending order,
 * then save all wifi signal info in the struct array availableNetworks. Once scanning
 * and saving is done, it will set the scanCompleted variable, otherwise it will stay 0.
 * Note:Typically takes ~2.184s to scan and store available wifi networks
 */
#include <ESP8266WiFi.h>
#define WIFI_MODE WIFI_AP_STA //set wifi mode here, AP_STA is access point and station mode together
#define WIFI_SSID "Swarm_Intel" //same as mesh_wifi
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
bool scanCompleted = 0; //0 if haven't complete scanning, 1 if done
int samples = 0;

void scanNetwork(){
  scanCompleted = 0;
  memset(availableNetworks,(char)0,WIFI_MAX);  //clears all previous wifi info
  WiFi.mode(WIFI_MODE);  //station mode = 3, access point + station
  WiFi.scanDelete();
  WiFi.scanNetworksAsync(saveNetwork,true); //asynchronously scan for networks available(wont wait)
}

#include <list>
#include "painlessMesh.h"
Task scanNodes(0,TASK_ONCE,&scanNetwork);

void saveNetwork(int networksFound){
  int swarmNetworks = 0;
  if (networksFound == 0){
    delay(1); //do nothing, or add some codes
  }
  else if(STORE_SWARM_ONLY) { //stores only "Swarm_Intel" SSIDs
    samples++;
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
    Serial.printf("DATA,TIME,%d",samples);
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

void meshSetup(){
  mesh.setDebugMsgTypes(ERROR|STARTUP); 
  mesh.init(MESH_SSID,MESH_PASSWORD,&taskScheduler,MESH_PORT);
  //mesh.onChangedConnections(&onChangedConnections);
  taskScheduler.addTask(scanNodes);
  selfNodeID = mesh.getNodeId();
  currentMaster = selfNodeID;
  addSelfToNodeList();
  Serial.println("CLEARSHEET");
  Serial.println("LABEL,timestamp,sample,MAC_1,RSSI_1,MAC_2,RSSI_2,MAC_3,RSSI_3,MAC_4,RSSI_4,MAC_5,RSSI_5");
  scanNodes.enable();
}
void onChangedConnections(){
  updateNodeList();
  printNodeList();
  selectMaster();
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
/*End of Mesh Codes-------------------------------------------------------------*/
/*Main Code---------------------------------------------------------------------*/
// Select SDA and SCL pins for I2C communication 
const uint8_t scl = D1;
const uint8_t sda = D3;

void setup() {
  Serial.begin(115200);
  meshSetup();
}

void loop() {
  mesh.update();
}
/*End of Main Code--------------------------------------------------------------*/
