#include <Arduino.h>
#include "ODriveCAN.h"
#include <FlexCAN_T4.h>
#include "ODriveFlexCAN.hpp"
struct ODriveStatus; // Prevent compile error

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#include <Adafruit_PWMServoDriver.h>

MPU6050 mpu;
#define EARTH_GRAVITY_MS2 9.80665  // m/s^2
#define DEG_TO_RAD        0.017453292519943295769236907684886
#define RAD_TO_DEG        57.295779513082320876798154814105

bool DMPReady = false;  // Set true if DMP init was successful
uint8_t MPUIntStatus;   // Holds actual interrupt status byte from MPU
uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64]; // FIFO storage buffer

/*---MPU6050 Control/Status Variables---*/
Quaternion q;           
VectorInt16 aa;         
VectorInt16 gg;         
VectorInt16 aaWorld;    
VectorInt16 ggWorld;    
VectorFloat gravity;    
float euler[3];         
float ypr[3];           

// Configuration of example sketch
#define CAN_BAUDRATE 1000000
#define ODRV0_NODE_ID 2
#define ODRV1_NODE_ID 1
#define ODRV2_NODE_ID 0  // New ODrive node ID for motor 3
#define ODRV3_NODE_ID 3  // New ODrive node ID for motor 4

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_intf;

// Forward declaration
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

// Instantiate ODrive objects for four nodes
ODriveCAN odrv0(wrap_can_intf(can_intf), ODRV0_NODE_ID);
ODriveCAN odrv1(wrap_can_intf(can_intf), ODRV1_NODE_ID);
ODriveCAN odrv2(wrap_can_intf(can_intf), ODRV2_NODE_ID); // New ODrive
ODriveCAN odrv3(wrap_can_intf(can_intf), ODRV3_NODE_ID); // New ODrive
ODriveCAN* odrives[] = {&odrv0, &odrv1, &odrv2, &odrv3};

struct ODriveUserData {
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
};

ODriveUserData odrv0_user_data;
ODriveUserData odrv1_user_data;
ODriveUserData odrv2_user_data;
ODriveUserData odrv3_user_data;
// No user data callbacks registered for odrv2 and odrv3

void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_heartbeat = msg;
  odrv_user_data->received_heartbeat = true;
}

void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_feedback = msg;
  odrv_user_data->received_feedback = true;
}

void onCanMessage(const CanMsg& msg) {
  for (auto odrive : odrives) {
    onReceive(msg, *odrive);
  }
}

// ========== Smoothing / Filtering Constants for RC Throttles ==========
#define ALPHA  0.19  // Exponential smoothing for throttleLeft
#define ALPHA2 0.10  // Exponential smoothing for throttleRight

static float filteredThrottleLeft = 0.0;
static float filteredThrottleRight = 0.0;
static float filteredThrottleUPDOWN = 0.0;

float desiredAngleADD = 0.0;

// ========== Velocity PID (PID #1) ==========
float vKp = 0.075; //0.075
float vKi = 0.02; //0.005
float vKd = 0.0003; //0.0002

float velocitySetpoint = 0.0;     // Desired average wheel velocity (rev/s)
float velocityMeasured = 0.0;     // Measured average velocity from ODrive feedback
float velocityError = 0.0;
float velocityErrorPrev = 0.0;
float velocityIntegral = 0.0;
float velocityDerivative = 0.0;
float velocityControlOutput = 0.0; // This becomes the desired pitch angle

// ========== Pitch PID (PID #2) ==========
float pKp = 16.8;   // e.g. 4.8 from your Ziegler–Nichols attempt
float pKi = 7.0;  // e.g. 32
float pKd = 0.69;  // e.g. 0.18

float pitchSetpoint = 0.0;       // From the velocity PID
float pitchError = 0.0;
float pitchErrorPrev = 0.0;
float pitchIntegral = 0.0;
float pitchDerivative = 0.0;
float pitchControlOutput = 0.0;  // Final motor velocity command

// ========== Saturation Limit for Pitch PID Output ==========
float MAX_CONTROL_OUTPUT = 10.0;

// ========== IMU smoothing factors ==========
float smoothedPitch = 0.0;
float smoothedYaw   = 0.0;
float smoothedRoll  = 0.0;
float alpha         = 0.8;  // For smoothing IMU readings

// ========== Turn command ==========
float turn_command = 0.0;
float up_down_command = 0.0;

// ========== RC Input Pins ==========
// Right throttle remains with pulseIn(); the rest use the new interrupt-based method.
const int receiver_pin  = 39;  // Throttle right? 
const int receiver_pin2 = 40;  // Throttle left?
const int receiver_pin3 = 41;  // Kill switch or mode? - CH7
const int receiver_pin4 = 38;  // UP DOWN MOTION - CH8
const int receiver_pin5 = 13;  // ANGLE OFFSET - CH5
const int receiver_pin6 = 26;  // LATCH TOGGLE - CH6

// Original pulseIn variables (right throttle remains)
unsigned long pulse_duration;   // THROTTLE RIGHT (FORWARD/BACKWARDS)

// For channels 2, 3, 4, 5, and 6, we now use the new interrupt-based method:
unsigned long pulse_duration2;  // THROTTLE LEFT (LEFT/RIGHT)
unsigned long pulse_duration3;  // FOR KILL SWITCH
unsigned long pulse_duration4;  // FOR UP DOWN MOTION
unsigned long pulse_duration5;  // FOR ANGLE OFFSET 
unsigned long pulse_duration6;  // FOR LATCH TOGGLE

// ============================
// New interrupt-based variables for channels 2-6
// ============================

// For receiver_pin2 (throttle left)
volatile unsigned long pulseStartCh2 = 0;
volatile unsigned long pulseWidthCh2 = 1500;
volatile bool newPulseCh2 = false;

// For receiver_pin3 (kill switch)
volatile unsigned long pulseStartCh3 = 0;
volatile unsigned long pulseWidthCh3 = 1500;
volatile bool newPulseCh3 = false;

// For receiver_pin4 (up/down motion)
volatile unsigned long pulseStartCh4 = 0;
volatile unsigned long pulseWidthCh4 = 1500;
volatile bool newPulseCh4 = false;

// For receiver_pin5 (angle offset)
volatile unsigned long pulseStartCh5 = 0;
volatile unsigned long pulseWidthCh5 = 1500;
volatile bool newPulseCh5 = false;

// For receiver_pin6 (latch toggle)
volatile unsigned long pulseStartCh6 = 0;
volatile unsigned long pulseWidthCh6 = 1500;
volatile bool newPulseCh6 = false;

// ============================
// ISRs for channels 2-6
// ============================
void isrCh2() {
  if (digitalRead(receiver_pin2) == HIGH) {
    pulseStartCh2 = micros();
  } else {
    pulseWidthCh2 = micros() - pulseStartCh2;
    newPulseCh2 = true;
  }
}

void isrCh3() {
  if (digitalRead(receiver_pin3) == HIGH) {
    pulseStartCh3 = micros();
  } else {
    pulseWidthCh3 = micros() - pulseStartCh3;
    newPulseCh3 = true;
  }
}

void isrCh4() {
  if (digitalRead(receiver_pin4) == HIGH) {
    pulseStartCh4 = micros();
  } else {
    pulseWidthCh4 = micros() - pulseStartCh4;
    newPulseCh4 = true;
  }
}

void isrCh5() {
  if (digitalRead(receiver_pin5) == HIGH) {
    pulseStartCh5 = micros();
  } else {
    pulseWidthCh5 = micros() - pulseStartCh5;
    newPulseCh5 = true;
  }
}

void isrCh6() {
  if (digitalRead(receiver_pin6) == HIGH) {
    pulseStartCh6 = micros();
  } else {
    pulseWidthCh6 = micros() - pulseStartCh6;
    newPulseCh6 = true;
  }
}

unsigned long lastTime = 0;
unsigned long currentTime = 0;
float deltaTime = 0.0;

// ============================
// Latching globals (unchanged)
// ============================
const int relayPin = 9; // Relay for Magnetic Latch
const uint16_t SERVO_MIN = 112;    // PWM value for 0° (relay triggered)
const uint16_t SERVO_MAX = 450;    // PWM value for 180° (relay untriggered)

Adafruit_PWMServoDriver pwm(0x40, Wire1);

bool latched = false;        // Current state: false = unlatched, true = latched
bool lastButtonState = false;  // To detect the edge

// ================================================================
//                          SETUP
// ================================================================
void setup() {
  // Setup RC input pins
  pinMode(receiver_pin,  INPUT);
  pinMode(receiver_pin2, INPUT);
  pinMode(receiver_pin3, INPUT);
  pinMode(receiver_pin4, INPUT);
  pinMode(receiver_pin5, INPUT);
  pinMode(receiver_pin6, INPUT);

  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW);  // Relay on

  Wire1.begin();
  pwm.begin();
  pwm.setPWMFreq(50);

  Serial.begin(115200);
  for (int i = 0; i < 30 && !Serial; ++i) {
    delay(100);
  }
  delay(200);

  Serial.println("Starting ODriveCAN");

  // Register the feedback and heartbeat callbacks for odrv0, odrv1, odrv2, and odrv3
  odrv0.onFeedback(onFeedback, &odrv0_user_data);
  odrv0.onStatus(onHeartbeat, &odrv0_user_data);
  odrv1.onFeedback(onFeedback, &odrv1_user_data);
  odrv1.onStatus(onHeartbeat, &odrv1_user_data);
  odrv2.onFeedback(onFeedback, &odrv2_user_data);
  odrv2.onStatus(onHeartbeat, &odrv2_user_data);
  odrv3.onFeedback(onFeedback, &odrv3_user_data);
  odrv3.onStatus(onHeartbeat, &odrv3_user_data);

  if (!setupCan()) {
    Serial.println("CAN failed to initialize: reset required");
    while (true); // spin indefinitely
  }

  Serial.println("Waiting for ODrive...");
  // Wait for all ODrives to send heartbeat
  while (!odrv0_user_data.received_heartbeat || !odrv1_user_data.received_heartbeat ||
         !odrv2_user_data.received_heartbeat || !odrv3_user_data.received_heartbeat) {
    pumpEvents(can_intf);
    delay(100);
  }
  Serial.println("found ODrive");

  // Read bus voltage and current for odrv0, odrv1, odrv2, odrv3
  Get_Bus_Voltage_Current_msg_t vbus;
  if (!odrv0.request(vbus, 1)) {
    Serial.println("vbus request failed for odrv0!");
    while (true);
  }
  Serial.print("ODRV0 DC voltage [V]: ");
  Serial.println(vbus.Bus_Voltage);
  Serial.print("ODRV0 DC current [A]: ");
  Serial.println(vbus.Bus_Current);

  if (!odrv1.request(vbus, 1)) {
    Serial.println("vbus request failed for odrv1!");
    while (true);
  }
  Serial.print("ODRV1 DC voltage [V]: ");
  Serial.println(vbus.Bus_Voltage);
  Serial.print("ODRV1 DC current [A]: ");
  Serial.println(vbus.Bus_Current);

  if (!odrv2.request(vbus, 1)) {
    Serial.println("vbus request failed for odrv2!");
    while (true);
  }
  Serial.print("ODRV2 DC voltage [V]: ");
  Serial.println(vbus.Bus_Voltage);
  Serial.print("ODRV2 DC current [A]: ");
  Serial.println(vbus.Bus_Current);

  if (!odrv3.request(vbus, 1)) {
    Serial.println("vbus request failed for odrv3!");
    while (true);
  }
  Serial.print("ODRV3 DC voltage [V]: ");
  Serial.println(vbus.Bus_Voltage);
  Serial.print("ODRV3 DC current [A]: ");
  Serial.println(vbus.Bus_Current);

  // Enabling closed loop control for all ODrives
  Serial.println("Enabling closed loop control...");
  while (odrv0_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL ||
         odrv1_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL ||
         odrv2_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL ||
         odrv3_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
    odrv0.clearErrors();
    odrv1.clearErrors();
    odrv2.clearErrors();
    odrv3.clearErrors();
    delay(1);
    odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    odrv1.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    odrv2.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    odrv3.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    for (int i = 0; i < 15; ++i) {
      delay(10);
      pumpEvents(can_intf);
    }
  }
  Serial.println("ODrives running!");
  delay(1000);

  // ========== MPU6050 (IMU) Setup ==========
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  Serial.println(F("Testing MPU6050 connection..."));
  if(!mpu.testConnection()){
    Serial.println("MPU6050 connection failed");
    while(true);
  } else {
    Serial.println("MPU6050 connection successful");
  }

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);

  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    Serial.println("These are the Active offsets: ");
    mpu.PrintActiveOffsets();
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    MPUIntStatus = mpu.getIntStatus();
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  delay(1000);

  // Attach interrupts for the new RC channels (all except receiver_pin)
  attachInterrupt(digitalPinToInterrupt(receiver_pin2), isrCh2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(receiver_pin3), isrCh3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(receiver_pin4), isrCh4, CHANGE);
  attachInterrupt(digitalPinToInterrupt(receiver_pin5), isrCh5, CHANGE);
  attachInterrupt(digitalPinToInterrupt(receiver_pin6), isrCh6, CHANGE);
}

// ================================================================
//                          LOOP
// ================================================================
void loop(){
  // ========== Read RC Inputs ==========
  // For the right throttle (receiver_pin) keep the original pulseIn() method.
  pulse_duration = pulseIn(receiver_pin, HIGH);
  
  // For the rest, use the new interrupt-based method:
  noInterrupts();
    unsigned long localPulseCh2 = pulseWidthCh2;
    unsigned long localPulseCh3 = pulseWidthCh3;
    unsigned long localPulseCh4 = pulseWidthCh4;
    unsigned long localPulseCh5 = pulseWidthCh5;
    unsigned long localPulseCh6 = pulseWidthCh6;
  interrupts();
  
  pulse_duration2 = localPulseCh2;
  pulse_duration3 = localPulseCh3;
  pulse_duration4 = localPulseCh4;
  pulse_duration5 = localPulseCh5;
  pulse_duration6 = localPulseCh6;
  
  // Map the right throttle (pulse_duration) to a small range
  float throttleRight = 0.2 - ((float)(pulse_duration - 999) * 0.4 / 1001.0);
  throttleRight = round(throttleRight / 0.01) * 0.01;
  filteredThrottleRight = ALPHA2 * throttleRight + (1 - ALPHA2) * filteredThrottleRight;
  velocitySetpoint = -15.0 * filteredThrottleRight;
  
  // Map the left throttle (pulse_duration2) to some range for turning
  float throttleLeft = 1.0 - ((float)(pulse_duration2 - 999) * (2.0 / 1001.0));
  throttleLeft = round(throttleLeft / 0.1) * 0.1;
  filteredThrottleLeft = ALPHA * throttleLeft + (1 - ALPHA) * filteredThrottleLeft;
  turn_command = filteredThrottleLeft;
  
  // Map the up/down motion (pulse_duration4) to a range from 1.0 to -1.0
  float throttleUPDOWN = 1.0 - ((float)(pulse_duration4 - 999) * (2.0 / 1001.0));
  throttleUPDOWN = round(throttleUPDOWN / 0.1) * 0.1;
  filteredThrottleUPDOWN = ALPHA * throttleUPDOWN + (1 - ALPHA) * filteredThrottleUPDOWN;
  up_down_command = filteredThrottleUPDOWN;
  
  // Process the angle offset (pulse_duration5) using defined ranges:
  static int lastState = 1; // Assume starting in middle state
  int currentState = -1;
  if (pulse_duration5 >= 980 && pulse_duration5 <= 1020) {
      currentState = 0; // bottom
  } else if (pulse_duration5 >= 1490 && pulse_duration5 <= 1510) {
      currentState = 1; // middle
  } else if (pulse_duration5 >= 1980 && pulse_duration5 <= 2020) {
      currentState = 2; // top
  }
  if (currentState != -1 && currentState != lastState) {
      if (lastState == 1) {
          if (currentState == 0) {
              desiredAngleADD -= 0.005;
          } else if (currentState == 2) {
              desiredAngleADD += 0.005;
          }
      }
      lastState = currentState;
  }
  
  // ========== Compute deltaTime ==========
  currentTime = micros();
  deltaTime   = (currentTime - lastTime) / 1000000.0;
  lastTime    = currentTime;
  
  pumpEvents(can_intf);
  
  // ========== Read IMU (Pitch) ==========
  if (!DMPReady) return;
  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetAccel(&aa, FIFOBuffer);
    mpu.dmpConvertToWorldFrame(&aaWorld, &aa, &q);
    mpu.dmpGetGyro(&gg, FIFOBuffer);
    mpu.dmpConvertToWorldFrame(&ggWorld, &gg, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  }
  
  smoothedPitch = alpha * ypr[1] + (1.0 - alpha) * smoothedPitch;
  smoothedYaw   = alpha * ypr[0] + (1.0 - alpha) * smoothedYaw;
  smoothedRoll  = alpha * ypr[2] + (1.0 - alpha) * smoothedRoll;
  
  // ========== Read ODrive feedback (for velocity) ==========
  float v0 = odrv0_user_data.last_feedback.Vel_Estimate;
  float v1 = odrv1_user_data.last_feedback.Vel_Estimate;
  velocityMeasured = (v0 - v1) * 0.5;
  
  // --------------------------------------------------------
  // 1) VELOCITY PID  --> outputs pitchSetpoint
  // --------------------------------------------------------
  velocityError = velocitySetpoint - velocityMeasured;
  velocityIntegral += velocityError * deltaTime;
  velocityDerivative = (velocityError - velocityErrorPrev) / (deltaTime);
  velocityControlOutput = (vKp * velocityError) + (vKi * velocityIntegral) + (vKd * velocityDerivative);
  velocityErrorPrev = velocityError;
  pitchSetpoint = -velocityControlOutput;
  
  // --------------------------------------------------------
  // 2) PITCH PID --> outputs final motor velocity
  // --------------------------------------------------------
  pitchError = pitchSetpoint - smoothedPitch + desiredAngleADD;
  pitchIntegral += pitchError * deltaTime;
  pitchDerivative = (pitchError - pitchErrorPrev) / (deltaTime > 0 ? deltaTime : 1.0e-6);
  float unsat_output = pKp * pitchError + pKi * pitchIntegral + pKd * pitchDerivative;
  if (unsat_output >  MAX_CONTROL_OUTPUT) {
    pitchControlOutput =  MAX_CONTROL_OUTPUT;
  } else if (unsat_output < -MAX_CONTROL_OUTPUT) {
    pitchControlOutput = -MAX_CONTROL_OUTPUT;
  } else {
    pitchControlOutput = unsat_output;
  }
  pitchErrorPrev = pitchError;
  
  // ========== If Kill Switch is active, reset integrals and set velocities to 0 ==========
  if (pulse_duration3 >= 1900 && pulse_duration3 <= 2100) {
    pitchControlOutput = 0;
    velocityControlOutput = 0;
    velocitySetpoint   = 0;
    pitchSetpoint      = 0;
    velocityIntegral   = 0;
    pitchIntegral      = 0;
    pitchDerivative    = 0;
    pitchError         = 0;
    velocityError      = 0;
  }
  
  // ========== Final Wheel Velocities ==========
  float leftVelocity  = pitchControlOutput + turn_command;
  float rightVelocity = -pitchControlOutput + turn_command;
  
  // ========== Send to ODrive ==========
  odrv0.setVelocity(leftVelocity);
  odrv1.setVelocity(rightVelocity);
  odrv2.setPosition(up_down_command);
  odrv3.setPosition(-up_down_command);
  
  // ========== Debug Printouts ==========
  Serial.println("---- RC & IMU Data ----");
  Serial.print("pulse_duration (right throttle): ");  Serial.println(pulse_duration);
  Serial.print("pulse_duration2 (left throttle):  ");  Serial.println(pulse_duration2);
  Serial.print("pulse_duration3 (kill switch): ");  Serial.println(pulse_duration3);
  Serial.print("pulse_duration4 (up/down motion): ");  Serial.println(pulse_duration4);
  Serial.print("pulse_duration5 (angle offset): ");  Serial.println(pulse_duration5);
  Serial.print("pulse_duration6 (latch toggle): ");  Serial.println(pulse_duration6);
  Serial.print("angle offset: "); Serial.println(desiredAngleADD);

  
  Serial.print("Filtered Throttle Right (fwd/bck): ");
  Serial.println(filteredThrottleRight, 4);
  Serial.print("Filtered Throttle Left  (turn):    ");
  Serial.println(filteredThrottleLeft, 4);
  Serial.print("Filtered Throttle Up_Down:    ");
  Serial.println(filteredThrottleUPDOWN, 4);
  
  Serial.println("---- Velocity PID ----");
  Serial.print("velocitySetpoint: ");   Serial.println(velocitySetpoint);
  Serial.print("velocityMeasured: ");   Serial.println(velocityMeasured);
  Serial.print("velocityError: ");      Serial.println(velocityError);
  Serial.print("velocityIntegral: ");   Serial.println(velocityIntegral);
  Serial.print("velocityControlOutput (pitchSetpoint): ");
  Serial.println(velocityControlOutput);
  
  Serial.println("---- Pitch PID ----");
  Serial.print("pitchSetpoint: ");      Serial.println(pitchSetpoint);
  Serial.print("pitchMeasured: ");      Serial.println(smoothedPitch);
  Serial.print("pitchError: ");         Serial.println(pitchError);
  Serial.print("pitchIntegral: ");      Serial.println(pitchIntegral);
  Serial.print("pitchControlOutput: "); Serial.println(pitchControlOutput);
  
  Serial.print("leftVelocity command:  "); Serial.println(leftVelocity);
  Serial.print("rightVelocity command: "); Serial.println(rightVelocity);
  
  Serial.print("odrv0-vel: "); Serial.println(v0);
  Serial.print("odrv1-vel: "); Serial.println(v1);
  
  Serial.println("======================================");

  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    if (input.startsWith("vkp=")) {
      vKp = input.substring(4).toFloat();
      Serial.print("vKp updated to: ");
      Serial.println(vKp);
    }
    if (input.startsWith("vki=")) {
      vKi = input.substring(4).toFloat();
      Serial.print("vKi updated to: ");
      Serial.println(vKi);
    }
    if (input.startsWith("vkd=")) {
      vKd = input.substring(4).toFloat();
      Serial.print("vKd updated to: ");
      Serial.println(vKd);
    }
    if (input.startsWith("kp=")) {
      pKp = input.substring(3).toFloat();
      Serial.print("Kp updated to: ");
      Serial.println(pKp);
    }
    if (input.startsWith("ki=")) {
      pKi = input.substring(3).toFloat();
      Serial.print("Ki updated to: ");
      Serial.println(pKi);
    }
    if (input.startsWith("kd=")) {
      pKd = input.substring(3).toFloat();
      Serial.print("Kd updated to: ");
      Serial.println(pKd);
    }
    if (input.startsWith("angle=")) {
      desiredAngleADD = input.substring(6).toFloat();
      Serial.print("Desired Angle updated to: ");
      Serial.println(desiredAngleADD);
    }
    
  }

  
  Serial.print("vKp command:  "); Serial.println(vKp, 5);
  Serial.print("vKi command:  "); Serial.println(vKi, 5);
  Serial.print("vKd command: "); Serial.println(vKd, 5);
  Serial.print("pKp command:  "); Serial.println(pKp, 5);
  Serial.print("pKi command:  "); Serial.println(pKi, 5);
  Serial.print("pKd command: "); Serial.println(pKd, 5);
  Serial.print("added angle: "); Serial.println(desiredAngleADD);


  
  // Latching button is processed using the original pulseIn() method for receiver_pin6 is replaced by the interrupt method.
  bool buttonPressed = (pulse_duration6 > 1800);  // true when the pulse indicates a press
  
  // Check for a rising edge: button was not pressed last time, but is pressed now
  if (buttonPressed && !lastButtonState) {
    latched = !latched;
    if (latched) {
      Serial.println("Latching");
      digitalWrite(relayPin, LOW);
      pwm.setPWM(0, 0, SERVO_MAX);
      pwm.setPWM(1, 0, SERVO_MAX);
    } else {
      Serial.println("Unlatching");
      digitalWrite(relayPin, HIGH);
      pwm.setPWM(0, 0, SERVO_MIN);
      pwm.setPWM(1, 0, SERVO_MIN);
    }
  }
  
  lastButtonState = buttonPressed;
}
