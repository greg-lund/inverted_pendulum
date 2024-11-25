#include <TMC2130Stepper.h>
#include <climits>
#include "consts.h"

TMC2130Stepper driver = TMC2130Stepper(STP_EN, STP_DIR, STP_STEP, STP_CS);

long encoder_pos = 0;
long motor_pos = 0;

// Loop time variables
elapsedMicros sinceControl, sinceStep, sinceSettle;

// Velocity control variables
unsigned int state = 0;
bool hasHomed = false, step_state = HIGH, step_dir = FWD, inc_motor = false;
unsigned long vel_dt = ULONG_MAX;

// Logging variables
float tLog[N], xLog[N], thetaLog[N];
unsigned int logIndex = 0;

// Function defs
void homeControl();
void sinControl(float,float,float);
void encoder_callback();
void logVars(float);
void printVars();

void setup() {
  // Encoder Interrupt
  pinMode(ENC_A,INPUT);
  pinMode(ENC_B,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A),encoder_callback,RISING);

  // Limit Switch
  pinMode(LIM,INPUT);

  // Motor driver
  pinMode(MISO,INPUT_PULLUP);
  driver.begin();
  driver.rms_current(1000);
  driver.microsteps(MICROSTEPS);
  digitalWrite(STP_EN,LOW);

  sinceControl = 0; 
  sinceStep = 0;
}

void loop() {
  
  if(sinceControl >= CONTROL_DT) // Update Velocity
  {
    sinceControl = 0;
    switch(state)
    {
      case 0: // Homing
        homeControl();
        break;
      case 1: // Swingup
        sinControl(0.05, 0.75, 4);
        break;
      case 2: // Balance
        break;
      case 3: // Print Log
        printVars();
        state = 4;
        break;
      case 4: // Done
        digitalWrite(STP_EN,HIGH);
        break;
    }
  }

  if(sinceStep >= vel_dt) // Take steps
  {
    sinceStep = 0;
    digitalWrite(STP_STEP,step_state);
    step_state = !step_state;
    if(inc_motor) motor_pos += (step_dir == FWD ? 1 : -1);
    inc_motor = !inc_motor;
  }
}

void homeControl()
{
  if(!hasHomed)
  {
    digitalWrite(STP_DIR,REV);
    vel_dt = 0.5 * 1e6 * (DIST_PER_STEP / HOME_VEL);
    if(!digitalRead(LIM)){
      hasHomed = true;
      motor_pos = 0;
      vel_dt = ULONG_MAX;
    } 
  } else
  {
    if(motor_pos >= 0.13 / DIST_PER_STEP)
    {
      state = 1; // Move to swingup
      sinceSettle = 0; // Reset so we can measure settle time
      vel_dt = ULONG_MAX;
      hasHomed = false; // Can reuse this for other stuff
      motor_pos = 0;
    } else
    {
      digitalWrite(STP_DIR,FWD);
      step_dir = FWD;
      vel_dt =  0.5 * 1e6 * (DIST_PER_STEP / HOME_VEL);
    }
  }
}

void sinControl(float amplitude, float period, float duration)
{
  if(!hasHomed)
  {
    if(sinceSettle < 1e6 * SETTLE_TIME)
    {
      vel_dt = ULONG_MAX;
      return;
    }
    digitalWrite(STP_DIR,FWD);
    step_dir = FWD;
    vel_dt =  0.5 * 1e6 * (DIST_PER_STEP / HOME_VEL);
    if(DIST_PER_STEP * motor_pos >= amplitude)
    {
      hasHomed = true;
      vel_dt = ULONG_MAX;
      sinceSettle = 0;
    }
  } else if(sinceSettle > 1e6 * SETTLE_TIME)
  {
    float t = sinceSettle * 1e-6 - SETTLE_TIME;
    float v = - (2*M_PI / period) * amplitude * sin(2*M_PI / period * t);
    step_dir = v > 0 ? FWD : REV;
    digitalWrite(STP_DIR,step_dir);
    vel_dt = 0.5 * 1e6 * (DIST_PER_STEP / abs(v));
    logVars(t);
    if(t > duration)
    {
      state = 3; // Go to print logging
      vel_dt = ULONG_MAX;
    }
  }
}

void encoder_callback()
{
  encoder_pos += digitalRead(ENC_B) ? 1 : -1;
}


void logVars(float t)
{
  if(logIndex>=N) return;
  tLog[logIndex] = t;
  xLog[logIndex] = motor_pos * DIST_PER_STEP;
  thetaLog[logIndex] = encoder_pos * (2*M_PI / PPR);
  logIndex++;
}

void printVars()
{
  for(unsigned int k = 0; k < N; k++)
  {
    Serial.print(tLog[k],4); Serial.print(",");
    Serial.print(xLog[k],4); Serial.print(",");
    Serial.println(thetaLog[k],4);
  }
}


