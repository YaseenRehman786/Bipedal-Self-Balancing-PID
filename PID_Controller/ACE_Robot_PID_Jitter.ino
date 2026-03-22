#include <Arduino.h>
#include "ODriveCAN.h"
#include <FlexCAN_T4.h>
#include "ODriveFlexCAN.hpp"
struct ODriveStatus; // Prevent compile error

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

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
#define ODRV0_NODE_ID 0
#define ODRV1_NODE_ID 1

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

// Instantiate ODrive objects for two nodes
ODriveCAN odrv0(wrap_can_intf(can_intf), ODRV0_NODE_ID);
ODriveCAN odrv1(wrap_can_intf(can_intf), ODRV1_NODE_ID);
ODriveCAN* odrives[] = {&odrv0, &odrv1};

struct ODriveUserData {
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
};

ODriveUserData odrv0_user_data;
ODriveUserData odrv1_user_data;

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

// ========== RC Throttle Filtering Constants ==========
#define ALPHA  0.19  // Exponential smoothing for throttle left
#define ALPHA2 0.40  // Exponential smoothing for throttle right

// Moving average filter parameters for the right throttle:
#define MA_WINDOW_RIGHT 10
float maRight[MA_WINDOW_RIGHT] = {0};
int maIndexRight = 0;
float maSumRight = 0.0;

static float filteredThrottleLeft = 0.0;
static float filteredThrottleRight = 0.0;
float desiredAngleADD = 0.0;

// ========== Velocity PID (PID #1) ==========
float vKp = 0.039; 
float vKi = 0.0012;
float vKd = 0.00018;

float velocitySetpoint = 0.0;     // Desired average wheel velocity (rev/s)
float velocityMeasured = 0.0;     // Measured average velocity from ODrive feedback
float velocityError = 0.0;
float velocityErrorPrev = 0.0;
float velocityIntegral = 0.0;
float velocityDerivative = 0.0;
float velocityControlOutput = 0.0; // This becomes the desired pitch angle

// ========== Pitch PID (PID #2) ==========
float pKp = 5.0;   // e.g. 4.8 from your Ziegler–Nichols attempt
float pKi = 35.0;  // e.g. 32
float pKd = 0.21;  // e.g. 0.18

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

// ========== RC Input Pins ==========
// All channels use the interrupt-based method.
const int receiver_pin  = 24;  // Throttle right (channel 1)
const int receiver_pin2 = 26;  // Throttle left  (channel 2)
const int receiver_pin3 = 29;  // Kill switch or mode (channel 3)
const int receiver_pin4 = 27;  // Desired angle adjustments (channel 4)

// --- Declare volatile variables for all channels ---
volatile unsigned long pulseStartCh1 = 0;
volatile unsigned long pulseWidthCh1 = 1500;
volatile bool newPulseCh1 = false;

volatile unsigned long pulseStartCh2 = 0;
volatile unsigned long pulseWidthCh2 = 1500;
volatile bool newPulseCh2 = false;

volatile unsigned long pulseStartCh3 = 0;
volatile unsigned long pulseWidthCh3 = 1500;
volatile bool newPulseCh3 = false;

volatile unsigned long pulseStartCh4 = 0;
volatile unsigned long pulseWidthCh4 = 1500;
volatile bool newPulseCh4 = false;

// ISRs for all channels
void isrCh1() {
  if (digitalRead(receiver_pin) == HIGH) {
    pulseStartCh1 = micros();
  } else {
    pulseWidthCh1 = micros() - pulseStartCh1;
    newPulseCh1 = true;
  }
}

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

unsigned long lastTime = 0;
unsigned long currentTime = 0;
float deltaTime = 0.0;

void setup() {
  // Setup RC input pins
  pinMode(receiver_pin, INPUT);
  pinMode(receiver_pin2, INPUT);
  pinMode(receiver_pin3, INPUT);
  pinMode(receiver_pin4, INPUT);

  // Attach interrupts for all channels
  attachInterrupt(digitalPinToInterrupt(receiver_pin), isrCh1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(receiver_pin2), isrCh2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(receiver_pin3), isrCh3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(receiver_pin4), isrCh4, CHANGE);

  Serial.begin(115200);
  for (int i = 0; i < 30 && !Serial; ++i) {
    delay(100);
  }
  delay(200);

  Serial.println("Starting ODriveCAN demo");

  // Register the feedback and heartbeat callbacks
  odrv0.onFeedback(onFeedback, &odrv0_user_data);
  odrv0.onStatus(onHeartbeat, &odrv0_user_data);
  odrv1.onFeedback(onFeedback, &odrv1_user_data);
  odrv1.onStatus(onHeartbeat, &odrv1_user_data);

  if (!setupCan()) {
    Serial.println("CAN failed to initialize: reset required");
    while (true); // spin indefinitely
  }

  Serial.println("Waiting for ODrive...");
  // Wait for both ODrives to send heartbeat
  while (!odrv0_user_data.received_heartbeat || !odrv1_user_data.received_heartbeat) {
    pumpEvents(can_intf);
    delay(100);
  }
  Serial.println("found ODrive");

  // Read bus voltage and current for both ODrives
  Get_Bus_Voltage_Current_msg_t vbus;
  if (!odrv0.request(vbus, 1)) {
    Serial.println("vbus request failed for odrv0!");
    while (true); // spin indefinitely
  }
  Serial.print("ODRV0 DC voltage [V]: ");
  Serial.println(vbus.Bus_Voltage);
  Serial.print("ODRV0 DC current [A]: ");
  Serial.println(vbus.Bus_Current);

  if (!odrv1.request(vbus, 1)) {
    Serial.println("vbus request failed for odrv1!");
    while (true); // spin indefinitely
  }
  Serial.print("ODRV1 DC voltage [V]: ");
  Serial.println(vbus.Bus_Voltage);
  Serial.print("ODRV1 DC current [A]: ");
  Serial.println(vbus.Bus_Current);

  // Enabling closed loop control for both ODrives
  Serial.println("Enabling closed loop control...");
  while (odrv0_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL ||
         odrv1_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
    odrv0.clearErrors();
    odrv1.clearErrors();
    delay(1);
    odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    odrv1.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);

    // Pump events to ensure the state is applied
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
  }
  else {
    Serial.println("MPU6050 connection successful");
  }

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // You can supply your gyro offsets here if you want them more precise:
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
}

void loop(){
  // Non-blocking read of RC inputs by copying volatile values
  unsigned long localPulseCh1, localPulseCh2, localPulseCh3, localPulseCh4;
  noInterrupts();
    localPulseCh1 = pulseWidthCh1;
    localPulseCh2 = pulseWidthCh2;
    localPulseCh3 = pulseWidthCh3;
    localPulseCh4 = pulseWidthCh4;
  interrupts();
  
  // --- Process each channel using the interrupt-based method ---
  
  // Channel 1 (Throttle Right) with Moving Average Filter:
  float rawThrottleRight = 0.2 - ((float)(localPulseCh1 - 999) * 0.4 / 1001.0);
  rawThrottleRight = round(rawThrottleRight / 0.001) * 0.001;
  // Update moving average for right throttle:
  maSumRight = maSumRight - maRight[maIndexRight] + rawThrottleRight;
  maRight[maIndexRight] = rawThrottleRight;
  maIndexRight = (maIndexRight + 1) % MA_WINDOW_RIGHT;
  filteredThrottleRight = maSumRight / MA_WINDOW_RIGHT;
  velocitySetpoint = 15.0 * filteredThrottleRight;
  
  // Channel 2 (Throttle Left) using original exponential filtering:
  float throttleLeft = 1.0 - ((float)(localPulseCh2 - 999) * (2.0 / 1001.0));
  throttleLeft = round(throttleLeft / 0.1) * 0.1;
  filteredThrottleLeft = ALPHA * throttleLeft + (1 - ALPHA) * filteredThrottleLeft;
  turn_command = filteredThrottleLeft;
  
  // Channel 3 (Kill Switch) processing:
  // (You can simply use localPulseCh3 directly.)
  
  // Channel 4 (Desired Angle Adjustments)
  // Process channel 4 using ranges:
  // Bottom: ~980–1020, Middle: ~1490–1510, Top: ~1980–2020
  static int lastState = 1; // assume starting in middle state
  int currentState = -1;
  if (localPulseCh4 >= 980 && localPulseCh4 <= 1020) {
      currentState = 0; // bottom
  } else if (localPulseCh4 >= 1490 && localPulseCh4 <= 1510) {
      currentState = 1; // middle
  } else if (localPulseCh4 >= 1980 && localPulseCh4 <= 2020) {
      currentState = 2; // top
  }
  if (currentState != -1 && currentState != lastState) {
      if (lastState == 1) {
          if (currentState == 0) {
              desiredAngleADD -= 0.01;
          } else if (currentState == 2) {
              desiredAngleADD += 0.01;
          }
      }
      lastState = currentState;
  }
  
  // Compute deltaTime
  currentTime = micros(); 
  deltaTime   = (currentTime - lastTime) / 1000000.0; 
  lastTime    = currentTime;

  pumpEvents(can_intf);

  // --- Read IMU (Pitch) ---
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

  // --- Read ODrive feedback (for velocity) ---
  float v0 = odrv0_user_data.last_feedback.Vel_Estimate;
  float v1 = odrv1_user_data.last_feedback.Vel_Estimate;
  velocityMeasured = (v0 - v1) * 0.5;

  // 1) VELOCITY PID --> outputs pitchSetpoint
  velocityError = velocitySetpoint - velocityMeasured;
  velocityIntegral += velocityError * deltaTime;
  velocityDerivative = (velocityError - velocityErrorPrev) / (deltaTime);
  velocityControlOutput = (vKp * velocityError) + (vKi * velocityIntegral) + (vKd * velocityDerivative);
  velocityErrorPrev = velocityError;
  pitchSetpoint = -velocityControlOutput;

  // 2) PITCH PID --> outputs final motor velocity
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

  // If Kill Switch (channel 3) is active, reset integrals and set velocities to 0
  if (pulseWidthCh3 >= 1900 && pulseWidthCh3 <= 2100) {
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

  // --- Final Wheel Velocities ---
  float rightVelocity  = pitchControlOutput + turn_command;
  float leftVelocity = -pitchControlOutput + turn_command;

  odrv0.setVelocity(rightVelocity);
  odrv1.setVelocity(leftVelocity);

  // --- Debug Printouts ---
  Serial.println("---- RC & IMU Data ----");
  Serial.print("Channel 1 (throttle right): "); Serial.println(localPulseCh1);
  Serial.print("Channel 2 (throttle left):  "); Serial.println(localPulseCh2);
  Serial.print("Channel 3 (kill switch):    "); Serial.println(localPulseCh3);
  Serial.print("Channel 4 (desired angle):  "); Serial.println(localPulseCh4);
  Serial.print("Filtered Throttle Right:    "); Serial.println(filteredThrottleRight, 4);
  Serial.print("Filtered Throttle Left:     "); Serial.println(filteredThrottleLeft, 4);
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
      if (input.startsWith("kp=")) {
          vKp = input.substring(3).toFloat();
          Serial.print("vKp updated to: ");
          Serial.println(vKp);
      }
      if (input.startsWith("ki=")) {
          vKi = input.substring(3).toFloat();
          Serial.print("vKi updated to: ");
          Serial.println(vKi);
      }
      if (input.startsWith("kd=")) {
          vKd = input.substring(3).toFloat();
          Serial.print("vKd updated to: ");
          Serial.println(vKd);
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
  Serial.print("added angle: "); Serial.println(desiredAngleADD, 5);
}
