#include <Arduino.h>
#include <Encoder.h> 

const int STEP_PIN = 8;   //To DM332T PULSE
const int DIR_PIN = 9;    //To DM332T DIRECTION

const int EA_PIN = 2;   //Encoder A channel
const int EB_PIN = 3;   //Encoder B channel
const int EZ_PIN = 4;   //Encoder Z channel

// -------------Motor, Gearbox and Encoder settings-------------------- 

const long STEPS_PER_REV = 200; //Steps per revolution of the motor

const long MICROSTEPS = 32L; // 6400 (pulses/rev) / 200 (steps/rev) = 32 microsteps  

const long GEAR_RATIO = 10L; //Gear ratio 

const long ENCODER_CPR_MOTOR = 4000L; // Encoder: 1000 PPR → 4000 CPR on MOTOR shaft (counts per revolution)

const long ENCODER_CPR_OUTPUT   = ENCODER_CPR_MOTOR * GEAR_RATIO; // Effective CPR at OUTPUT shaft (what 1 output rev means in counts) 40000

const long STEPS_PER_MOTOR_REV  = STEPS_PER_REV * MICROSTEPS; // 6400

const long STEPS_PER_OUTPUT_REV = STEPS_PER_MOTOR_REV * GEAR_RATIO; // 64000

const float COUNTS_PER_STEP = (float)ENCODER_CPR_MOTOR / (float)STEPS_PER_MOTOR_REV;  // Encoder counts per microstep 4000 / 6400 = 0.625

const float COUNTS_PER_DEG = (float)ENCODER_CPR_OUTPUT / 360.0f;  // ≈ 111.11

const float MAX_SPEED_COUNTS_PER_SEC = 8000.0f; // Max speed (in encoder counts per second) allowed by PID output  8000 / 0.625 ≈ 12800 microsteps/s

// ----------end of settings-------------

const int PID_PERIOD_MS = 10; //100 Hz

float kp = 0.1; 
float kd = 0.03f; 
float ki = 0.0;

long targetCounts = 0; // desired position in encoder counts
float integrator = 0.0;
float previous_error = 0.0;

Encoder encoder(EA_PIN, EB_PIN); 

unsigned long last_time = 0; 
unsigned long lastStepTimeMs = 0;
unsigned long stepIntervalUs = 0;     // 0 = no stepping
bool stepDirForward = true;

// Helper functions 

void setTargetAngle(float angleDeg)
{ 
  targetCounts = (long)(angleDeg * COUNTS_PER_DEG); 
  Serial.print("New target angle: ");
  Serial.print(angleDeg);
  Serial.print(" deg (counts = ");
  Serial.print(targetCounts);
  Serial.println(")");
}

void setStepRate(long outputSpeedCounts)
{ 
  if(outputSpeedCounts == 0){ 
    stepIntervalUs = 0; 
    return; 
  }

  // setting the direction
  stepDirForward = (outputSpeedCounts > 0); 
  digitalWrite(DIR_PIN, stepDirForward ? HIGH : LOW); 

  float speedAbs = outputSpeedCounts; 
  if (speedAbs < 0) speedAbs = -speedAbs; 

  // Convert encoder-count speed = steps/sec
  float stepsPerSec = speedAbs / COUNTS_PER_STEP; 

  if (stepsPerSec < 1.0f) {
    stepIntervalUs = 0;
    return;
  }

  // Hard safety limit
  if (stepsPerSec > 50000.0f) {
    stepsPerSec = 50000.0f;
  }

  stepIntervalUs = (unsigned long)(1000000.0f / stepsPerSec); //microseconds per step - frequency of stepping 
}

void serviceStepper()
{ 
  if(stepIntervalUs == 0) return;

  unsigned long nowMs = micros(); 
  if(nowMs - lastStepTimeMs >= stepIntervalUs){ 
    lastStepTimeMs = nowMs;
    digitalWrite(STEP_PIN, HIGH); 
    delayMicroseconds(8); 
    digitalWrite(STEP_PIN, LOW); 
  }
}

long pid(long error, float dt)
{ 
  long propotional = error;

  integrator += error * dt;
  if(integrator > 50000.0f) integrator = 50000.0f;
  if(integrator < -50000.0f) integrator = -50000.0f; 

  long derivative = (error - previous_error) / dt; 

  previous_error = error; 

  long output = (kp*propotional) + (ki*integrator) + (kd*derivative); 

  return output; 
}

void setup() 
{ 
  //Stepper
  pinMode(STEP_PIN, OUTPUT); 
  pinMode(DIR_PIN, OUTPUT); 

  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN,  LOW);

  //Encoder
  pinMode(EA_PIN, INPUT_PULLUP);
  pinMode(EB_PIN, INPUT_PULLUP);
  pinMode(EZ_PIN, INPUT_PULLUP);

  Serial.begin(115200);

  setTargetAngle(0.0f); 
}

void loop() 
{
  unsigned long now = millis(); 

  if(Serial.available()){ // get the desired angle from user
    float angle = Serial.parseFloat(); // read a float like "90" or "-180"
    setTargetAngle(angle); 
  }

  if(now - last_time >= PID_PERIOD_MS){
    float dt = (now - last_time)/1000.0f; // in seconds 
    last_time = now;

    long pos = encoder.read(); // Counts at motor shaft
    long error = targetCounts - pos; 

    long outputSpeedCounts = pid(error, dt); // output speed counts per second  

    // Limit max speed 
    if (outputSpeedCounts > MAX_SPEED_COUNTS_PER_SEC)
      outputSpeedCounts = MAX_SPEED_COUNTS_PER_SEC; 
    if (outputSpeedCounts < -MAX_SPEED_COUNTS_PER_SEC)
      outputSpeedCounts = -MAX_SPEED_COUNTS_PER_SEC;

    setStepRate(outputSpeedCounts); 
  }
  
  serviceStepper();
}




