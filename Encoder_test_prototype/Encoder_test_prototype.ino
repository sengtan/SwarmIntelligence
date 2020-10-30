/* Note:
 *  -Try to keep medium tasks >=2ms, low tasks >=15ms
 *  -NodeMCU Soft Access Point can handle max 5 connections
 */
#include <ESP8266WiFi.h>
#include <list>
#include <TaskScheduler.h>
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
/*Encoder Code------------------------------------------------------------------*/
const uint8_t L_EN = D2;
const uint8_t R_EN = D6;

bool L_move = 0, R_move = 0;
const int ENC_SAMPLES = 5;
unsigned long L_timer = 0, R_timer = 0;
int L_tp[20], R_tp[20];
float L_avgtp = 0, R_avgtp = 0;
long L_tso = 0, R_tso = 0, L_tss = 0, R_tss = 0;
int L_counter = 0, R_counter = 0;
int L_pwm = 0, R_pwm = 0;
float L_rpm = 0, R_rpm = 0;

void ICACHE_RAM_ATTR L_Callback(){
 if(digitalRead(L_EN)){ //check if L_EN pin is high
  if(L_move){ //if Left motor is controlled by NodeMCU to move
    if(L_tss == 0){ //if tick since stop is 0, this is the first tick, not counted because motor stopped for long periods
      L_timer = millis(); //reset timer for tick
      L_tss++;  //1 tick and above means that motor is moving
    }
    else{ //1 tick and above
      L_tp[L_counter] = millis()-L_timer; //calculate period between ticks
      L_timer = millis(); //reset timer for tick
      L_avgtp = 0;  //reset previous avg period
      L_counter++; L_tss++; L_tso++;  //increment, L_counter is for n sample of array, L_tss is to count tick since stop, L_tso is tick since on
      int i;  //local variable, so as to not disrupt other functions
      for(i=0;i<(L_tss-1) && i<ENC_SAMPLES;i++){  //L_tss-1 because first tick doesn't count, ENC_SAMPLES is max sample to store
        L_avgtp += L_tp[i]; //add up all existing samples, if there is only 1 sample, L_tss = 2, (L_tss-1) is >= ENC_SAMPLES after 5+1 ticks
      }
      L_avgtp = (float)L_avgtp/i; //take accumulated period and divide by i(number of samples added up) to get average period
      L_rpm = (float)3000/L_avgtp;; //calculated rpm, 60000ms / L_avgtp ms / 20 ticks => 3000/L_avgtp 
      if(L_counter == ENC_SAMPLES)  //if L_counter reaches ENC_SAMPLES, array is full, restart from beginning
        L_counter = 0;  //reset L_counter
      Serial.printf("L_TP:%.2fms,L_RPM:%.2f\n",L_avgtp,L_rpm);
    }
  }
  else {  //Left motor is either moved by external forces/bumped
    
  }
 }
}
void ICACHE_RAM_ATTR R_Callback(){
 if(digitalRead(R_EN))
  Serial.println("R_HIGH");
 else
  Serial.println("R_LOW");
}
void Encoder_setup(){
  pinMode(L_EN,INPUT);
  pinMode(R_EN,INPUT);
  attachInterrupt(digitalPinToInterrupt(L_EN), L_Callback, RISING);
  attachInterrupt(digitalPinToInterrupt(R_EN), R_Callback, RISING);
}
void Encoder_Reset(){
  L_counter = 0;  R_counter = 0;  //Reset counter for array
  L_tss = 0;  R_tss = 0;  //Reset tick since stop
  for(int i=0;i<ENC_SAMPLES;i++){
    L_tp[i] = 0;  R_tp[i] = 0;  //Reset whole array
  }
}
/*End of Encoder Code-----------------------------------------------------------*/
/*Motor Driver Code-------------------------------------------------------------*/
const uint8_t M1A = D7;
const uint8_t M1B = D8;
const uint8_t M2A = RX;
const uint8_t M2B = D5;

#define WHEEL_DIAMETER 67 //in mm
#define WHEEL_CIRCUMFERENCE 21.049 //cm

Task forward(0,TASK_ONCE,&Test_Forward);
Task rest(0,TASK_ONCE,&Test_Stop);
Task testing(0,TASK_ONCE,&Forward);

void Test_Stop(){
  digitalWrite(M2A,0);
  digitalWrite(M2B,0);
  digitalWrite(M1A,0);
  digitalWrite(M1B,0);
  L_move = 0; R_move = 0;
  Encoder_Reset();
  Serial.println("STOP");
  forward.restartDelayed(1000);
}
void Motor_Setup(){
  pinMode(M1A,OUTPUT);
  pinMode(M1B,OUTPUT);
  pinMode(M2A,OUTPUT);
  pinMode(M2B,OUTPUT);
  digitalWrite(M2A,0);
  digitalWrite(M2B,0);
  digitalWrite(M1A,0);
  digitalWrite(M1B,0);
  //Test_Stop();
}
void Forward(){ //in cm
  float distance=100;
  int ticksRequired = distance/WHEEL_CIRCUMFERENCE;
  ticksRequired = ticksRequired*20;
  Serial.println(ticksRequired);
  Encoder_Reset();
  while(L_tss < ticksRequired ){  //|| R_tss < ticksRequired
    if(L_tss == 0 && R_tss == 0){
      L_move = 1; R_move = 1;
      digitalWrite(M1B,0); digitalWrite(M2B,0);
      analogWrite(M1A,600); analogWrite(M2A,600);
    }
    Serial.printf("L_TSS:%d, R_TSS:%d\n",L_tss,R_tss);
    yield();
  }
  if(L_tss >= ticksRequired){
    digitalWrite(M2A,0);
    digitalWrite(M2B,0);
    digitalWrite(M1A,0);
    digitalWrite(M1B,0);
  }
  Serial.println("DONE");
}
void Test_Forward(){
  L_move = 1; R_move = 1;
  Serial.println("MOVE");
  analogWrite(M1A,600);
  digitalWrite(M1B,0);
  analogWrite(M2A,600);
  digitalWrite(M2B,0);
  rest.restartDelayed(1000);
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
/*Battery Voltage Reader Code---------------------------------------------------*/
/* The battery voltage passes through a voltage divider circuit of:
 * V_A0 = V_BAT (R1/R1+R2)
 * Where
 * R1 = 10k ohm
 * R2 = 16k ohm
 * V_BAT = 8.4V(max) or 7.4V(nominal)
 * V_A0 = 3.231V(max) or 2.846V(nominal)
 * Note: Battery voltage input is also connected to Buck converter input
 * CRITICAL EDIT:
 * Since NodeMCU already has a fixed built-in voltage divider of 
 * Node_A0 = V_A0 (R3/R3+R4)
 * Where
 * R3 = 100k ohm
 * R4 = 220k ohm
 * The addition of another voltage divider from above has caused some changes,
 * V_A0 = 0.377(V_BAT)
 * Node_A0 = 0.3125(V_A0)
 *         = 0.1178(V_BAT)
 * Thus, the concluded expected voltages are:
 * V_BAT = 8.4V(max) or 7.4V(nominal)
 * V_A0 = 3.167V(max) or 2.789(nominal)   //0.377 V_BAT
 * Node_A0 = 0.989V(max) or 0.872V(nominal) //0.3125*0.377 V_BAT
 */
#define R1 10 //external resistor in k
#define R2 16 //external resistor in k
#define R3 100  //built-in resistor in k
#define R4 220  //built-in resistor in k
#define NODEMCU_VOLT 3.33
const uint8_t BAT_IN = A0;  //set Battery input pin
void Read_Battery(){
  int analogInput = analogRead(BAT_IN);
  float A0_voltage = ((float)analogInput/1023)*NODEMCU_VOLT; //convert into 3.3V range
  float BAT_voltage = A0_voltage/0.377;  //convert into 8.4V range
  float BAT_percent = (BAT_voltage/8.4)*100;
  Serial.printf("raw:%d,A0:%.2f,BAT:%.2f,Prcnt:%.2f%%\n",analogInput,A0_voltage,BAT_voltage,BAT_percent);
}
/*End of Battery Votlage Reader Code--------------------------------------------*/
/*Main Code---------------------------------------------------------------------*/
#include <Wire.h>
// Select SDA and SCL pins for I2C communication 
const uint8_t scl = D1;
const uint8_t sda = D3;

Scheduler taskScheduler;

void setup() {
  Serial.begin(115200);
  Wire.begin(sda, scl);
  taskScheduler.addTask(forward);
  taskScheduler.addTask(rest);
  taskScheduler.addTask(testing);
  Motor_Setup();
  Encoder_setup();
  //forward.enable();
  testing.enable();
}

void loop() {
  taskScheduler.execute();
}
/*End of Main Code--------------------------------------------------------------*/
