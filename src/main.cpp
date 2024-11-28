#include <TMC2130Stepper.h>
#include <climits>
#include "consts.h"
#include "control_traj.h"

TMC2130Stepper driver = TMC2130Stepper(STP_EN, STP_DIR, STP_STEP, STP_CS);

long encoderPos = 0, motorPos = 0;
long prevEncoderPos = 0, prevMotorPos = 0;

// Loop time variables
elapsedMicros sinceControl, sinceStep, sinceSettle;

// Velocity control variables
unsigned int state = 0, next_state = 0;
bool hasHomed = false, step_state = HIGH, step_dir = FWD, inc_motor = false;
unsigned long vel_dt = ULONG_MAX;

// Logging variables
float tLog[N], xLog[N], thetaLog[N];
unsigned int logIndex = 0, controlIndex = 0;

// Function defs
void homeControl();
void sinControl(float,float,float);
void swingupControl();
void balanceControl();
void justBalanceControl();
void energyPumpControl(float);
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

  state = 0; 
  sinceSettle = 0;
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
        //swingupControl();
        energyPumpControl(20.0);
        break;
      case 2: // Balance
        balanceControl();
        break;
      case 3: // Print Log
        printVars();
        state = 4;
        break;
      case 4: // Done
        digitalWrite(STP_EN,HIGH);
        break;
      case 5: // Record
        float t = sinceSettle * 1e-6;
        logVars(t);
        if(t > LOG_DURATION) state = 3;
    }
  }

  if(sinceStep >= vel_dt) // Take steps
  {
    sinceStep = 0;
    digitalWrite(STP_STEP,step_state);
    step_state = !step_state;
    if(inc_motor) motorPos += (step_dir == FWD ? 1 : -1);
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
      motorPos = 0;
      vel_dt = ULONG_MAX;
    } 
  } else
  {
    if(motorPos >= 0.13 / DIST_PER_STEP)
    {
      state = 1; // Move to swingup
      sinceSettle = 0; // Reset so we can measure settle time
      vel_dt = ULONG_MAX;
      hasHomed = false; // Can reuse this for other stuff
      motorPos = 0;
      prevMotorPos = 0;
      prevEncoderPos = 0;
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
    if(DIST_PER_STEP * motorPos >= amplitude)
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

void swingupControl()
{
  // Settle
  if(sinceSettle < 1e6 * SETTLE_TIME)
  {
    vel_dt = ULONG_MAX;
    return;
  } 

  // Run control trajectory
  if(controlIndex >= sizeof(vel)/sizeof(vel[0]) || abs(M_PI - encoderPos*(2*M_PI/PPR)) < 0.01)
  {
    Serial.print("Finished swingup with theta = ");
    Serial.print(encoderPos*(2*M_PI/PPR),3); Serial.print(".x = ");
    Serial.println(motorPos*DIST_PER_STEP,3);
    prevEncoderPos = encoderPos;
    vel_dt = ULONG_MAX;
    state = 3; // Go to balance
    sinceSettle = 0;
    return;
  }
  step_dir = vel[controlIndex] > 0 ? FWD : REV;
  digitalWrite(STP_DIR,step_dir);
  if(abs(vel[controlIndex]) < 1e-6)
  {
    vel_dt = ULONG_MAX;
  } 
  else 
  {
    vel_dt = 0.5 * 1e6 * (DIST_PER_STEP / abs(vel[controlIndex]));
  }
  controlIndex++;
  logVars(sinceSettle*1e-6 - SETTLE_TIME);
}

void balanceControl(bool justBalance)
{
  if(sinceSettle > 1e6 * ((justBalance ? 1.0 : 0.0)*BALANCE_DURATION+SETTLE_TIME))
  {
    vel_dt = ULONG_MAX;
    state = 4;
    return;
  } else if(justBalance && sinceSettle < 1e6 * SETTLE_TIME)
  {
    prevEncoderPos = encoderPos;
    vel_dt = ULONG_MAX;
    return;
  }

  float x = motorPos*DIST_PER_STEP;
  float theta = encoderPos * (2*M_PI/PPR);
  float xdot = (motorPos-prevMotorPos)/(CONTROL_DT*1e-6)*DIST_PER_STEP;
  float thetadot = (encoderPos-prevEncoderPos)/(CONTROL_DT*1e-6)*(2*M_PI/PPR);

  float u = -(K[0]*x + K[1]*(theta-M_PI) + K[2]*xdot + K[3]*thetadot);
  u = max(min(u,MAX_U),-MAX_U); // Clamp control
  float desVel = xdot+u*(CONTROL_DT*1e-6);
  
  // Catch limits
  if(abs(x) >= XMAX/2)
  {
    Serial.println("X out of bounds, ending!");
    vel_dt = ULONG_MAX;
    state = 4; // Quit
    return;
  }

  step_dir = desVel > 0 ? FWD : REV;
  digitalWrite(STP_DIR,step_dir);
  if(abs(desVel) < 1e-6)
  {
    vel_dt = ULONG_MAX;
  } 
  else 
  {
    vel_dt = 0.5 * 1e6 * (DIST_PER_STEP / abs(desVel));
  }

  prevMotorPos = motorPos;
  prevEncoderPos = encoderPos;
}

void justBalanceControl()
{
  if(sinceSettle > 1e6 * (BALANCE_DURATION+SETTLE_TIME))
  {
    vel_dt = ULONG_MAX;
    state = 4;
    return;
  } else if(sinceSettle < 1e6 * SETTLE_TIME)
  {
    prevEncoderPos = encoderPos;
    vel_dt = ULONG_MAX;
    return;
  }

  float x = motorPos*DIST_PER_STEP;
  float theta = encoderPos * (2*M_PI/PPR);
  float xdot = (motorPos-prevMotorPos)/(CONTROL_DT*1e-6)*DIST_PER_STEP;
  float thetadot = (encoderPos-prevEncoderPos)/(CONTROL_DT*1e-6)*(2*M_PI/PPR);

  float u = -(K[0]*x + K[1]*(theta-M_PI) + K[2]*xdot + K[3]*thetadot);
  float desVel = xdot+u*(CONTROL_DT*1e-6);

  // Catch limits
  if(abs(x) >= XMAX/2)
  {
    vel_dt = ULONG_MAX;
    state = 4; // Quit
    return;
  }

  step_dir = desVel > 0 ? FWD : REV;
  digitalWrite(STP_DIR,step_dir);
  if(abs(desVel) < 1e-6)
  {
    vel_dt = ULONG_MAX;
  } 
  else 
  {
    vel_dt = 0.5 * 1e6 * (DIST_PER_STEP / abs(desVel));
  }

  prevMotorPos = motorPos;
  prevEncoderPos = encoderPos;
}

void energyPumpControl(float duration)
{
  float maxU = 1, ke = 1, kx = 10, kv = 5;
  float a = 54.22, b = 0.075, c = 5.6;
  float x = motorPos*DIST_PER_STEP;
  float theta = encoderPos * (2*M_PI/PPR);
  float xdot = (motorPos-prevMotorPos)/(CONTROL_DT*1e-6)*DIST_PER_STEP;
  float thetadot = (encoderPos-prevEncoderPos)/(CONTROL_DT*1e-6)*(2*M_PI/PPR);

  if(abs(theta) > 3.1)
  {
    if(theta < 0) encoderPos += PPR;
    vel_dt = ULONG_MAX;
    state = 2; // Success, go to balance
    return;
  } else if(sinceSettle > 1e6*duration || abs(x) >= XMAX/2)
  {
    vel_dt = ULONG_MAX;
    state = 4; // Failed, go to done
    return;
  }

  float ue = -b*thetadot / (c*cos(theta)) + cos(theta)*thetadot*(1/(2*a)*thetadot*thetadot-cos(theta)-1.005);
  float u = ke*ue - kx*x - kv*xdot;
  u = max(min(u,maxU),-maxU);
  float desVel = xdot+u*(CONTROL_DT*1e-6);

  step_dir = desVel > 0 ? FWD : REV;
  digitalWrite(STP_DIR,step_dir);
  if(abs(desVel) < 1e-6)
  {
    vel_dt = ULONG_MAX;
  } 
  else 
  {
    vel_dt = 0.5 * 1e6 * (DIST_PER_STEP / abs(desVel));
  }
  prevMotorPos = motorPos;
  prevEncoderPos = encoderPos;
}

void encoder_callback()
{
  encoderPos += digitalRead(ENC_B) ? 1 : -1;
}


void logVars(float t)
{
  if(logIndex>=N) return;
  tLog[logIndex] = t;
  xLog[logIndex] = motorPos * DIST_PER_STEP;
  thetaLog[logIndex] = encoderPos * (2*M_PI / PPR);
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
