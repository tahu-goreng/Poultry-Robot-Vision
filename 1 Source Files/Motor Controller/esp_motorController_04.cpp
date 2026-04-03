/*
====================================================
ESP32-S3 | Differential Drive
Motor + Encoder + Odometry
Protocol:
  ARM
  DISARM
  CMD v w        (m/s, rad/s)
Output:
  ODOM x y theta v w ticksL ticksR
====================================================
*/

#include <Arduino.h>
#include "driver/ledc.h"
#include <math.h>

/* ===================== PIN DEFINITIONS ===================== */
// PWM
#define FR_PA  14
#define BR_PB  5
#define FL_PA  8
#define BL_PB  11

// Direction
#define FR_A1  3
#define FR_A2  4
#define BR_B1  6
#define BR_B2  7
#define FL_A1  9
#define FL_A2 10
#define BL_B1 12
#define BL_B2 13

// Encoder
#define ENC_R_A  39
#define ENC_R_B  38
#define ENC_L_A  37
#define ENC_L_B  36

/* ===================== ROBOT PARAM ========================= */
static const float WHEEL_DIAMETER = 0.20f;   // meter
static const float WHEEL_BASE     = 0.45f;   // meter
static const float LOOP_DT        = 0.02f;   // 50 Hz
static const int   TICKS_PER_REV  = 220;

static const float WHEEL_CIRC  = M_PI * WHEEL_DIAMETER;
static const float TICKS_PER_M = TICKS_PER_REV / WHEEL_CIRC;

/* ===================== GLOBAL ============================== */
volatile long encRight = 0;
volatile long encLeft  = 0;

bool motorArmed = false;

// Pose
float posX = 0.0f, posY = 0.0f, theta = 0.0f;

// Encoder snapshot
long prevEncL_odom = 0, prevEncR_odom = 0;
long prevEncL_spd  = 0, prevEncR_spd  = 0;

// Command
float targetV = 0.0f;   // m/s
float targetW = 0.0f;   // rad/s

// Speed
float speedL = 0.0f, speedR = 0.0f;

/* ===================== PID ================================ */
float Kp = 140, Ki = 40, Kd = 6;
float iL = 0, iR = 0, lastEL = 0, lastER = 0;

/* ===================== ENCODER ISR ======================== */
void IRAM_ATTR encR_ISR(){
  if (digitalRead(ENC_R_A) == digitalRead(ENC_R_B)) encRight++;
  else encRight--;
}

void IRAM_ATTR encL_ISR(){
  if (digitalRead(ENC_L_A) == digitalRead(ENC_L_B)) encLeft++;
  else encLeft--;
}

/* ===================== PWM ================================ */
#define PWM_FREQ   20000
#define PWM_TIMER  LEDC_TIMER_0

void setupPWM(uint8_t ch, int pin){
  static bool timerInit = false;
  if(!timerInit){
    ledc_timer_config_t t = {
      .speed_mode      = LEDC_LOW_SPEED_MODE,
      .duty_resolution = LEDC_TIMER_10_BIT,
      .timer_num       = PWM_TIMER,
      .freq_hz         = PWM_FREQ,
      .clk_cfg         = LEDC_AUTO_CLK
    };
    ledc_timer_config(&t);
    timerInit = true;
  }

  ledc_channel_config_t c = {
    .gpio_num   = pin,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel    = (ledc_channel_t)ch,
    .intr_type  = LEDC_INTR_DISABLE,
    .timer_sel  = PWM_TIMER,
    .duty       = 0,
    .hpoint     = 0
  };
  ledc_channel_config(&c);
}

void pwmWrite(uint8_t ch, int duty){
  duty = constrain(duty, 0, 1023);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)ch, duty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)ch);
}

void drivePair(int in1, int in2, int ch, int pwm){
  int duty = map(abs(pwm), 0, 255, 0, 1023);

  if(pwm > 0){ digitalWrite(in1,HIGH); digitalWrite(in2,LOW); }
  else if(pwm < 0){ digitalWrite(in1,LOW); digitalWrite(in2,HIGH); }
  else{ digitalWrite(in1,LOW); digitalWrite(in2,LOW); duty = 0; }

  pwmWrite(ch, duty);
}

// Motor mapping (LEFT reversed)
void setMotor(uint8_t id, int pwm){
  switch(id){
    case 1: drivePair(BR_B1, BR_B2, 1,  pwm); break;
    case 2: drivePair(FR_A1, FR_A2, 0,  pwm); break;
    case 3: drivePair(BL_B1, BL_B2, 3, -pwm); break;
    case 4: drivePair(FL_A1, FL_A2, 2, -pwm); break;
  }
}

void stopAll(){
  for(int i=1;i<=4;i++) setMotor(i,0);
}

/* ===================== PID ================================ */
int PIDwheel(float target, float current, float &iTerm, float &lastE){
  float e = target - current;
  iTerm += e * LOOP_DT;
  iTerm = constrain(iTerm, -0.4, 0.4);
  float d = (e - lastE) / LOOP_DT;
  lastE = e;
  int u = (int)(Kp*e + Ki*iTerm + Kd*d);
  return constrain(u, -255, 255);
}

/* ===================== SPEED ============================== */
void updateWheelSpeed(){
  long curL, curR;
  noInterrupts();
  curL = encLeft;
  curR = encRight;
  interrupts();

  long dL = curL - prevEncL_spd;
  long dR = curR - prevEncR_spd;
  prevEncL_spd = curL;
  prevEncR_spd = curR;

  speedL = (dL / TICKS_PER_M) / LOOP_DT;
  speedR = (dR / TICKS_PER_M) / LOOP_DT;
}

/* ===================== ODOMETRY =========================== */
void updateOdometry(){
  long curL, curR;
  noInterrupts();
  curL = encLeft;
  curR = encRight;
  interrupts();

  long dL = curL - prevEncL_odom;
  long dR = curR - prevEncR_odom;
  prevEncL_odom = curL;
  prevEncR_odom = curR;

  float distL = dL / TICKS_PER_M;
  float distR = dR / TICKS_PER_M;
  float dist  = 0.5f * (distL + distR);
  float dTh   = (distR - distL) / WHEEL_BASE;

  float midTheta = theta + dTh * 0.5f;
  posX += dist * cosf(midTheta);
  posY += dist * sinf(midTheta);
  theta += dTh;

  if(theta > M_PI) theta -= 2*M_PI;
  if(theta < -M_PI) theta += 2*M_PI;
}

/* ===================== SERIAL CMD ========================= */
String line;
unsigned long lastCmdMs = 0;

void handleLine(String s){
  s.trim();
  s.replace("\r","");
  if(!s.length()) return;

  if(s == "ARM"){
    motorArmed = true;
    lastCmdMs = millis();
    Serial.println("OK ARM");
    return;
  }

  if(s == "DISARM"){
    motorArmed = false;
    targetV = 0;
    targetW = 0;
    stopAll();
    Serial.println("OK DISARM");
    return;
  }

  float v,w;
  if(sscanf(s.c_str(),"CMD %f %f",&v,&w) == 2){
    targetV = constrain(v, -0.6f, 0.6f);
    targetW = constrain(w, -2.0f, 2.0f);
    lastCmdMs = millis();
    return;
  }
}

/* ===================== SETUP ============================== */
void setup(){
  Serial.begin(115200);
  delay(200);

  pinMode(FR_A1,OUTPUT); pinMode(FR_A2,OUTPUT);
  pinMode(BR_B1,OUTPUT); pinMode(BR_B2,OUTPUT);
  pinMode(FL_A1,OUTPUT); pinMode(FL_A2,OUTPUT);
  pinMode(BL_B1,OUTPUT); pinMode(BL_B2,OUTPUT);

  setupPWM(0,FR_PA);
  setupPWM(1,BR_PB);
  setupPWM(2,FL_PA);
  setupPWM(3,BL_PB);

  pinMode(ENC_R_A,INPUT_PULLUP);
  pinMode(ENC_R_B,INPUT_PULLUP);
  pinMode(ENC_L_A,INPUT_PULLUP);
  pinMode(ENC_L_B,INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_R_A), encR_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), encL_ISR, CHANGE);

  stopAll();

  prevEncL_odom = encLeft;
  prevEncR_odom = encRight;
  prevEncL_spd  = encLeft;
  prevEncR_spd  = encRight;

  Serial.println("READY ESP32 ODOM");
}

/* ===================== LOOP =============================== */
unsigned long lastLoop = 0;

void loop(){
  while(Serial.available()){
    char c = (char)Serial.read();
    if(c == '\n'){
      handleLine(line);
      line = "";
    } else line += c;
  }

  if(millis() - lastCmdMs > 500){
    targetV = 0;
    targetW = 0;
  }

  if(millis() - lastLoop >= (unsigned long)(LOOP_DT*1000)){
    lastLoop = millis();

    float targetVL = targetV - targetW * WHEEL_BASE * 0.5f;
    float targetVR = targetV + targetW * WHEEL_BASE * 0.5f;

    updateWheelSpeed();

    int pwmL = PIDwheel(targetVL, speedL, iL, lastEL);
    int pwmR = PIDwheel(targetVR, speedR, iR, lastER);

    if(!motorArmed) stopAll();
    else{
      setMotor(1, pwmR); setMotor(2, pwmR);
      setMotor(3, pwmL); setMotor(4, pwmL);
    }

    updateOdometry();

    float v_meas = 0.5f * (speedL + speedR);
    float w_meas = (speedR - speedL) / WHEEL_BASE;

    Serial.printf(
      "ODOM %.4f %.4f %.4f %.3f %.3f %ld %ld\n",
      posX, posY, theta, v_meas, w_meas, encLeft, encRight
    );
  }
}