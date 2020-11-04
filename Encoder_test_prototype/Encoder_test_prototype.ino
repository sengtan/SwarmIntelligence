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
/*Tested to have <1% error when measured Tp is compared to actual Tp with lower standard deviation*/
const uint8_t L_EN = D2;
const uint8_t R_EN = D6;

bool L_move = 0, R_move = 0;
const int ENC_SAMPLES = 5;
unsigned long L_timer = 0, R_timer = 0;
int L_tp[ENC_SAMPLES], R_tp[ENC_SAMPLES];
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
     //Serial.printf("L_TP:%.2fms,L_RPM:%.2f\n",L_avgtp,L_rpm);
    }
  }
  else {  //Left motor is either moved by external forces/bumped
    delay(1);
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
/*Motor PID Code----------------------------------------------------------------*/
#include<PID_v1.h>
float Setpoint_offset = 1.5;
float actualSetpoint = 35;
float Setpoint = actualSetpoint, L_pidin, L_pidout, R_pidin, R_pidout;
float Kp=7.2, Ki=0.065, Kd=0.01625; //Kcrit = 12, Pcrit = 130ms

PID L_PID(&L_pidin,&L_pidout,&Setpoint,Kp,Ki,Kd,REVERSE);
PID R_PID(&R_pidin,&R_pidout,&Setpoint,Kp,Ki,Kd,REVERSE);

void PID_Setup(){
  L_PID.SetSampleTime(10); R_PID.SetSampleTime(10); //ms
  L_PID.SetOutputLimits(0,1023); R_PID.SetOutputLimits(0,1023); //min max
  L_PID.SetMode(MANUAL); R_PID.SetMode(MANUAL); //MANUAL to manually reset output, AUTOMATIC to let code auto set
}

const int STEADY_SAMPLES = 4;
const float steady_state_err = 5;  //max allowed steady state err %
const float steady_state_value = (actualSetpoint*steady_state_err)/100.0;
float prev_max = 0, prev_min = 100;
float maxer[STEADY_SAMPLES], miner[STEADY_SAMPLES];
float avg_max = 0, avg_min = 100;
int max_counter = 0, min_counter = 0, max_samples = 0, min_samples = 0;
bool prev_state = 0, current_state = 0, transition = 0;

bool getSteadyState(float avgtp){
  if(avgtp>Setpoint){
    current_state = 1;
    if(prev_max<avgtp)
      prev_max = avgtp;
    if(prev_state != current_state){  //transition
      transition = 1;
      if(min_counter == STEADY_SAMPLES) //if counter reach max
        min_counter = 0;  //reset counter
      miner[min_counter] = prev_min;  //store previous highest point
      min_samples++; min_counter++;
      avg_min = 0;
      int i;  //declare locally to not affect other code
      for(i=0;i<min_samples && i<STEADY_SAMPLES;i++)
        avg_min += miner[i];  //sum up all highest point
      avg_min = (float)avg_min/(float)i;  //determine current high point avg
      prev_min = actualSetpoint;
    }
    prev_state = current_state;
  }
  else if(avgtp<Setpoint){
    current_state = 0;
    if(prev_min>avgtp)
      prev_min = avgtp;
    if(prev_state != current_state){  //transition
      transition = 1;
      if(max_counter == STEADY_SAMPLES) //if counter reach max
        max_counter = 0;  //reset counter
      maxer[max_counter] = prev_max;  //store previous highest point
      max_samples++; max_counter++;
      avg_max = 0;
      int i;  //declare locally to not affect other code
      for(i=0;i<max_samples && i<STEADY_SAMPLES;i++)
        avg_max += maxer[i];  //sum up all highest point
      avg_max = (float)avg_max/(float)i;  //determine current high point avg
      prev_max = actualSetpoint;
    }
    prev_state = current_state;
  }
  if(transition){
    //Wait until response signal is within +- Steady state error
    if( (avg_max-actualSetpoint) < steady_state_value && (actualSetpoint-avg_min) < steady_state_value ){
      //Serial.printf("Reached Steady State with Error of %f%%\n",steady_state_err);
      return true;
    }
    transition = 0;
  }
  return false;
}
/*End of Motor PID Code---------------------------------------------------------*/
/*Motor Driver Code-------------------------------------------------------------*/
const uint8_t M1A = D7;
const uint8_t M1B = D8;
const uint8_t M2A = RX;
const uint8_t M2B = D5;

#define WHEEL_DIAMETER 67 //in mm
#define WHEEL_CIRCUMFERENCE 21.049 //cm
#define DEFAULT_SPEED 600 //600/1023

int L_speed, R_speed;

Task PID_forward(0,TASK_ONCE,&PID_Test);

void PID_Test(){  //using left wheel only
  float avgtp; int donetime; bool done = 0;
  L_move = 1; R_move = 1; L_avgtp = 0;
  L_speed = DEFAULT_SPEED; R_speed = DEFAULT_SPEED;
  L_PID.SetMode(MANUAL);  //to manually reset output
  L_pidout = (float)L_speed;  //reset output value
  L_PID.SetMode(AUTOMATIC); //auto again
  digitalWrite(M1B,0); digitalWrite(M2B,0); //Should set low first before starting motors, outside loop because it doesnt change
  donetime = millis();
  do{
    analogWrite(M1A,L_speed); analogWrite(M2A, R_speed); //Start motors, inside loop because speed value changes
    if(L_tss>1){
      avgtp=L_avgtp;
      L_pidin = avgtp;
      L_PID.Compute();  //Compute the output based on err/delta
      L_speed = (int)L_pidout;  //convert float to int
      //To see monitor, use below 1 line
      //Serial.printf("In:%.2f,Out:%d,Delta:%.2f,Set:%.2f,\n",L_avgtp,L_speed,L_avgtp-actualSetpoint,actualSetpoint);
      //To see plot, use below 3 lines
      Serial.printf("40,30,%f,%f,%f,%f,%f,%f,",avg_max,avg_min,actualSetpoint,actualSetpoint+steady_state_value,actualSetpoint-steady_state_value,((float)L_speed/100)+25);
      Serial.print(avgtp);
      Serial.println();
    }
    yield();
  }while(!getSteadyState(avgtp));
  donetime = millis() - donetime;
  digitalWrite(M1A,0); digitalWrite(M2A,0);
  Serial.printf("In:%.2f,Out:%d,Delta:%.2f,Set:%.2f\n",L_avgtp,L_speed,L_avgtp-actualSetpoint,actualSetpoint);
  Serial.printf("Total time taken:%d\n",donetime);
  L_move = 0; L_avgtp = 0;
}

void PID_Forward(){
  float avgtp; int donetime; bool done = 0;
  L_move = 1; R_move = 1; L_avgtp = 0;
  L_speed = DEFAULT_SPEED; R_speed = DEFAULT_SPEED;
  L_PID.SetMode(MANUAL);  //to manually reset output
  L_pidout = (float)L_speed;  //reset output value
  L_PID.SetMode(AUTOMATIC); //auto again
  digitalWrite(M1B,0); digitalWrite(M2B,0); //Should set low first before starting motors, outside loop because it doesnt change
  donetime = millis();
  do{
    analogWrite(M1A,L_speed); analogWrite(M2A, R_speed); //Start motors, inside loop because speed value changes
    if(L_tss>1){
      avgtp=L_avgtp;
      L_pidin = avgtp;
      L_PID.Compute();  //Compute the output based on err/delta
      L_speed = (int)L_pidout;  //convert float to int
      //To see monitor, use below 1 line
      //Serial.printf("In:%.2f,Out:%d,Delta:%.2f,Set:%.2f,\n",L_avgtp,L_speed,L_avgtp-actualSetpoint,actualSetpoint);
      //To see plot, use below 3 lines
      Serial.printf("40,30,%f,%f,%f,%f,%f,%f,",avg_max,avg_min,actualSetpoint,actualSetpoint+steady_state_value,actualSetpoint-steady_state_value,((float)L_speed/100)+27);
      Serial.print(avgtp);
      Serial.println();
    }
    yield();
  }while(!getSteadyState(avgtp) || true);
  donetime = millis() - donetime;
  digitalWrite(M1A,0); digitalWrite(M2A,0);
  Serial.printf("In:%.2f,Out:%d,Delta:%.2f,Set:%.2f\n",L_avgtp,L_speed,L_avgtp-actualSetpoint,actualSetpoint);
  Serial.printf("Total time taken:%d\n",donetime);
  L_move = 0; L_avgtp = 0;
}

void Test_Stop(){
  digitalWrite(M2A,0);
  digitalWrite(M2B,0);
  digitalWrite(M1A,0);
  digitalWrite(M1B,0);
  L_move = 0; R_move = 0;
  Encoder_Reset();
  Serial.println("STOP");
  //forward.restartDelayed(1000);
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
#define BATTERY_CONSTANT 0.228
const uint8_t BAT_IN = A0;  //set Battery input pin
void Read_Battery(){
  int analogInput = analogRead(BAT_IN);
  float A0_voltage = ((float)analogInput/1023)*NODEMCU_VOLT; //convert into 3.3V range
  float BAT_voltage = (A0_voltage/0.377)-BATTERY_CONSTANT;  //convert into 8.4V range, then deduct by 0.228V constant due to error
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
int donetime = 0;
void setup() {
  Serial.begin(115200);
  Wire.begin(sda, scl);
  taskScheduler.addTask(PID_forward);
  Encoder_setup();
  Motor_Setup();
  PID_Setup();
  PID_forward.restartDelayed(1000);
}

void loop() {
  taskScheduler.execute();
}
/*End of Main Code--------------------------------------------------------------*/
