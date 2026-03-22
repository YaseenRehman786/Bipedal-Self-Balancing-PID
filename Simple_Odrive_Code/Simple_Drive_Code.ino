#include <Arduino.h>
#include "ODriveCAN.h"
#include <FlexCAN_T4.h>
#include "ODriveFlexCAN.hpp"
struct ODriveStatus; // Prevent compile error

// Configuration
#define CAN_BAUDRATE 1000000
#define ODRV0_NODE_ID 0
#define ODRV1_NODE_ID 1

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_intf;

void onCanMessage(const CanMsg& msg);

bool setupCan() {
  can_intf.begin();
  can_intf.setBaudRate(CAN_BAUDRATE);
  can_intf.setMaxMB(16);
  can_intf.enableFIFO();
  can_intf.enableFIFOInterrupt();
  can_intf.onReceive(onCanMessage);
  return true;
}

ODriveCAN odrv0(wrap_can_intf(can_intf), ODRV0_NODE_ID);
ODriveCAN odrv1(wrap_can_intf(can_intf), ODRV1_NODE_ID);
ODriveCAN* odrives[] = {&odrv0, &odrv1};

struct ODriveUserData {
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
};

ODriveUserData odrv0_user_data;
ODriveUserData odrv1_user_data;

void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_heartbeat = msg;
  odrv_user_data->received_heartbeat = true;
}

void onCanMessage(const CanMsg& msg) {
  for (auto odrive : odrives) {
    onReceive(msg, *odrive);
  }
}

// ========== RC Input Pins ==========
// Confirmed via serial diagnostic (all 4 pins printed simultaneously):
const int receiver_pin  = 26;  // Right joystick — forward/back (throttle)
const int receiver_pin2 = 24;  // Left joystick  — left/right turn
const int receiver_pin3 = 29;  // Kill switch (HIGH ~2000us = stop)
const int receiver_pin4 = 27;  // Reserved

// ALL channels interrupt-based
volatile unsigned long pulseStartCh1 = 0;
volatile unsigned long pulseWidthCh1 = 1500;

volatile unsigned long pulseStartCh2 = 0;
volatile unsigned long pulseWidthCh2 = 1500;

volatile unsigned long pulseStartCh3 = 0;
volatile unsigned long pulseWidthCh3 = 1500;

volatile unsigned long pulseStartCh4 = 0;
volatile unsigned long pulseWidthCh4 = 1500;

void isrCh1() {
  if (digitalRead(receiver_pin) == HIGH) pulseStartCh1 = micros();
  else pulseWidthCh1 = micros() - pulseStartCh1;
}
void isrCh2() {
  if (digitalRead(receiver_pin2) == HIGH) pulseStartCh2 = micros();
  else pulseWidthCh2 = micros() - pulseStartCh2;
}
void isrCh3() {
  if (digitalRead(receiver_pin3) == HIGH) pulseStartCh3 = micros();
  else pulseWidthCh3 = micros() - pulseStartCh3;
}
void isrCh4() {
  if (digitalRead(receiver_pin4) == HIGH) pulseStartCh4 = micros();
  else pulseWidthCh4 = micros() - pulseStartCh4;
}

// ========== Tuning Constants — edit these only ==========
// EMA smoothing: lower = smoother but laggier (range 0.05–0.5)
#define ALPHA           0.20f  // turn channel smoothing factor
#define ALPHA2          0.20f  // throttle channel smoothing factor

// Top speed for each axis (ODrive units = turns/s)
// Increase MAX_DRIVE_SPEED carefully — robot can be fast
#define MAX_DRIVE_SPEED  0.4f  // turns/s — forward/back full-stick //ThIS SHOULD BE CALLED TURN SPEED
// MAX_TURN_SPEED: each wheel gets ±this value added/subtracted
// If turning is still too fast at 0.1, try 0.05; too slow → try 0.3
#define MAX_TURN_SPEED   0.6f  // turns/s — left/right full-stick  //THIS SHOULD BE CALLED DRIVE SPEED

// Deadband: fraction of full stick to ignore as noise (range 0.05–0.15)
#define DEADBAND         0.08f

static float filteredThrottle = 0.0f;
static float filteredTurn     = 0.0f;
unsigned long lastDebugMs     = 0;

void setup() {
  pinMode(receiver_pin,  INPUT);
  pinMode(receiver_pin2, INPUT);
  pinMode(receiver_pin3, INPUT);
  pinMode(receiver_pin4, INPUT);

  attachInterrupt(digitalPinToInterrupt(receiver_pin),  isrCh1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(receiver_pin2), isrCh2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(receiver_pin3), isrCh3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(receiver_pin4), isrCh4, CHANGE);

  Serial.begin(115200);
  for (int i = 0; i < 30 && !Serial; ++i) delay(100);
  delay(200);

  Serial.println("Starting ODrive velocity driving sketch");

  odrv0.onStatus(onHeartbeat, &odrv0_user_data);
  odrv1.onStatus(onHeartbeat, &odrv1_user_data);

  if (!setupCan()) {
    Serial.println("CAN failed to initialize: reset required");
    while (true);
  }

  Serial.println("Waiting for ODrive heartbeats...");
  while (!odrv0_user_data.received_heartbeat || !odrv1_user_data.received_heartbeat) {
    pumpEvents(can_intf);
    delay(100);
  }
  Serial.println("Found both ODrives");

  // Clear errors and go to idle first
  odrv0.clearErrors();
  odrv1.clearErrors();
  pumpEvents(can_intf);
  delay(100);

  // Explicitly set VELOCITY control mode before enabling closed loop
  // ControlMode 2 = VELOCITY_CONTROL, InputMode 1 = PASSTHROUGH
  odrv0.setControllerMode(2, 1);
  odrv1.setControllerMode(2, 1);
  pumpEvents(can_intf);
  delay(100);

  Serial.println("Enabling closed loop control...");
  while (odrv0_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
    odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    pumpEvents(can_intf);
    delay(10);
  }
  while (odrv1_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
    odrv1.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    pumpEvents(can_intf);
    delay(10);
  }

  Serial.println("Ready (velocity mode):");
  Serial.println("  pin 26 (right joystick): forward/back  — MAX_DRIVE_SPEED");
  Serial.println("  pin 24 (left joystick) : left/right    — MAX_TURN_SPEED");
  Serial.println("  pin 29 (kill switch)   : HIGH=stop");
  Serial.print(  "  MAX_DRIVE_SPEED="); Serial.print(MAX_DRIVE_SPEED);
  Serial.print(  "  MAX_TURN_SPEED=");  Serial.println(MAX_TURN_SPEED);
}

void loop() {
  pumpEvents(can_intf);

  // Safely read ISR values
  unsigned long localCh1, localCh2, localCh3;
  noInterrupts();
    localCh1 = pulseWidthCh1;  // pin 26 = right joystick = THROTTLE (forward/back)
    localCh2 = pulseWidthCh2;  // pin 24 = left joystick  = TURN (left/right)
    localCh3 = pulseWidthCh3;  // pin 29 = kill switch
  interrupts();

  // rawThrottle: joystick forward → positive speed → robot moves forward
  // Negate because typical RC: stick forward → pulse > 1500 → we want positive
  float rawThrottle = -((float)localCh1 - 1500.0f) / 500.0f;
  rawThrottle = constrain(rawThrottle, -1.0f, 1.0f);
  if (fabsf(rawThrottle) < DEADBAND) rawThrottle = 0.0f;

  float rawTurn = -((float)localCh2 - 1500.0f) / 500.0f;
  rawTurn = constrain(rawTurn, -1.0f, 1.0f);
  if (fabsf(rawTurn) < DEADBAND) rawTurn = 0.0f;

  // Smooth both channels
  filteredTurn     = ALPHA  * rawTurn     + (1.0f - ALPHA)  * filteredTurn;
  filteredThrottle = ALPHA2 * rawThrottle + (1.0f - ALPHA2) * filteredThrottle;

  // Clamp filtered values to [-1, 1] before scaling
  filteredTurn     = constrain(filteredTurn,     -1.0f, 1.0f);
  filteredThrottle = constrain(filteredThrottle, -1.0f, 1.0f);

  // Scale to final speeds using independent maximums
  float baseSpeed = MAX_DRIVE_SPEED * filteredThrottle;  // max ±5.0 turns/s
  float turnSpeed = MAX_TURN_SPEED  * filteredTurn;      // max ±0.5 turns/s

  // Kill switch: Ch3 high (~2000us) → full stop
  if (localCh3 >= 1800UL) {
    odrv0.setVelocity(0.0f);
    odrv1.setVelocity(0.0f);
  } else {
    // Differential drive: each wheel gets base ± turn, no further scaling
    float rightSpeed = baseSpeed + turnSpeed;
    float leftSpeed  = baseSpeed - turnSpeed;
    odrv0.setVelocity(rightSpeed);
    odrv1.setVelocity(leftSpeed);
  }

  // Debug print every 200ms — ALL 4 channels so we can identify which pin moves with each joystick
  unsigned long now = millis();
  if (now - lastDebugMs >= 200) {
    lastDebugMs = now;
    unsigned long localCh4;
    noInterrupts();
      localCh4 = pulseWidthCh4;
    interrupts();
    float rightSpeed = baseSpeed + turnSpeed;
    float leftSpeed  = baseSpeed - turnSpeed;
    Serial.print("p24(TRN)=");   Serial.print(localCh1); 
    Serial.print(" p26(FWD)="); Serial.print(localCh2);   
    Serial.print(" p29(kill)="); Serial.print(localCh3);
    Serial.print(" p27=");       Serial.print(localCh4);
    Serial.print(" base="); Serial.print(baseSpeed, 2);
    Serial.print(" turn="); Serial.print(turnSpeed, 2);
    Serial.print(" R="); Serial.print(rightSpeed, 2);
    Serial.print(" L="); Serial.println(leftSpeed, 2);
  }
}
