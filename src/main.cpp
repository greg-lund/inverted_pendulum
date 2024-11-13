#include "Arduino.h"
#include "TMCStepper.h"
#include "consts.h"
//#include "digitalWriteFast.h"
#include "control_traj.h"

long encoder_pos = 0;
long motor_pos = 0;
TMC2130Stepper driver(STP_CS,0.11f,STP_SDI,STP_SDO,STP_SCK);

// Output stuff
char tbuffer[10],xbuffer[10],thetabuffer[10],outbuffer[40];

void encoder_callback()
{
  encoder_pos += digitalRead(ENC_B) ? 1 : -1;
}

void home()
{
  // Move in reverse (towards limit switch)
  digitalWrite(STP_DIR,REV);
  int t = 16000/MICROSTEPS;
  while(digitalRead(LIM))
  {
    digitalWrite(STP_STEP,HIGH);
    delayMicroseconds(t);
    digitalWrite(STP_STEP,LOW);
    delayMicroseconds(t);
    if(t > 3600/MICROSTEPS)
    {
      t--;
    }
  }

  // Set position to zero
  motor_pos = 0;
}

void move(bool dir, float dist, float vel)
{
  int steps = dist / DIST_PER_STEP;
  int dt_micros = 0.5 * 1e6 * (DIST_PER_STEP / vel);

  digitalWrite(STP_DIR,dir);
  for(unsigned long i = 0; i < steps; i++)
  {
    digitalWrite(STP_STEP,HIGH);
    delayMicroseconds(dt_micros);
    digitalWrite(STP_STEP,LOW);
    delayMicroseconds(dt_micros);
  }
}

void printState(float t, float x, float theta)
{
  dtostrf(t,1,4,tbuffer);
  dtostrf(x,1,4,xbuffer);
  dtostrf(theta,1,4,thetabuffer);
  sprintf(outbuffer,"%s,%s,%s",tbuffer,xbuffer,thetabuffer);
  Serial.println(outbuffer);
}

void testMotion(float amplitude, float cycles_per_second, float duration, float end_time)
{
  // Parameters
  float controller_dt = 1e-2; // How often does the control get updated?
  float omega = 2 * M_PI * cycles_per_second; // Frequency of sin wave

  // Move amplitude distance
  move(FWD, amplitude, 0.1);
  delay(2000);

  unsigned long prev_step_micros = micros();
  unsigned long start_micros;

  float desired_vel,vel_dt;
  bool step_state = HIGH;
  bool dir;

  unsigned long t0_micros = micros();

  char tbuf[10], velbuf[10], thetabuf[10], buf[32];

  while(true)
  {

    // Get desired velocity
    desired_vel = (amplitude * omega * cos( (omega * 1.0e-6) * (micros() - t0_micros) + M_PI/2.0)) / DIST_PER_STEP;

    // Run for the desired time
    if(1e-6*(micros() - t0_micros) > duration)
    {
      if(1e-6*(micros() - t0_micros) > end_time) {
        break;
      }
      desired_vel = 0;
    }


    if(DEBUG)
    {
      dtostrf(1e-6*(micros()-t0_micros),1,4,tbuf);
      dtostrf(desired_vel*DIST_PER_STEP,1,4,velbuf);
      dtostrf(encoder_pos * (2*M_PI/PPR),1,4,thetabuf);
      sprintf(buf,"%s,%s,%s",tbuf,velbuf,thetabuf);
      Serial.println(buf);
    }
    
    // If desired velocity is too low, we can't make any steps
    if(abs(desired_vel) < 1 / controller_dt) 
    {
      delay(1e3 * controller_dt);
      continue;
    }
    // dt should be 1/2 of calculation bc one step is HIGH -> LOW transition
    vel_dt = abs( 0.5 * 1.0e6 / desired_vel); // MAKE SURE TO GET A POSITIVE VALUE FOR DT!!
    dir = (desired_vel < 0) ? REV : FWD;
    
    // Run the controller for controller_dt seconds
    digitalWrite(STP_DIR,dir);
    start_micros = micros();

    while((micros() - start_micros) < (controller_dt * 1e6))
    {
      digitalWrite(STP_STEP,HIGH);
      delayMicroseconds(vel_dt);
      digitalWrite(STP_STEP,LOW);
      delayMicroseconds(vel_dt);
    }
  }
}

void swingup()
{
  unsigned long prev_step_micros = micros();
  unsigned long start_micros;

  float desired_vel,vel_dt;
  bool step_state = HIGH;
  bool dir;

  int inc;

  unsigned long t0_micros = micros();

  for(unsigned int i = 0; i < sizeof(vel)/sizeof(*vel); i++)
  {
     
    // Get desired velocity
    desired_vel = vel[i] / DIST_PER_STEP;

    // dt should be 1/2 of calculation bc one step is HIGH -> LOW transition
    vel_dt = abs( 0.5 * 1.0e6 / desired_vel); // MAKE SURE TO GET A POSITIVE VALUE FOR DT!!
    dir = (desired_vel < 0) ? REV : FWD;

    if(DEBUG)
    {
      printState(1e-6*(micros()-t0_micros),motor_pos * DIST_PER_STEP,encoder_pos*(2*M_PI/PPR));
    }
    
    // If desired velocity is too low, we can't make any steps
    if(abs(desired_vel) < 1 / control_dt) 
    {
      delay(1e3 * control_dt);
      continue;
    }

    // Run the controller for controller_dt seconds
    start_micros = micros();
    digitalWrite(STP_DIR,dir);
    start_micros = micros();
    while((micros() - start_micros) < (control_dt * 1e6))
    {
      digitalWrite(STP_STEP,HIGH);
      delayMicroseconds(vel_dt);
      digitalWrite(STP_STEP,LOW);
      delayMicroseconds(vel_dt);
      motor_pos += (desired_vel > 0) ? 1 : -1;
    }
  }
}

void balance(float duration)
{

  //float K[] = {-2.942587950705084,88.46775301437461,-4.925693802211881,15.278676342978887};
  //float K[] = {-9.285228361131146,104.00732299536902,-7.831774228603373,17.962080281422665};
  // float K[] = {-52.6378, 121.615, -30.9833, 16.4404}; // More control effort but smaller cart deviation
  float K[] = {-19.5027, 63.1984, -12.7427, 8.03225};
  float controller_dt = 1e-2;

  unsigned long prev_step_micros = micros();
  unsigned long start_micros;

  float x, xdot=0, theta, thetadot;
  long prev_encoder_pos = encoder_pos;
  float desired_vel=0,vel_dt;
  bool step_state = HIGH;
  bool dir;
  int inc;

  for(unsigned int i = 0; i < duration / controller_dt; i++)
  {
    // Get state
    x = motor_pos * DIST_PER_STEP;
    xdot = desired_vel * DIST_PER_STEP;
    theta = encoder_pos * (2*M_PI / PPR);
    thetadot = (encoder_pos - prev_encoder_pos) / controller_dt * (2*M_PI/PPR);
    prev_encoder_pos = encoder_pos;

    if(DEBUG) {
      printState(1e-6*(micros()),x,theta);
    }
    if(abs(x) > 0.13) 
    {
      Serial.println("FAILED! Cart hit bounds");
      break;
    }

    // Get desired velocity via LQR
    desired_vel = (xdot - (K[0]*x + K[1]*(theta-M_PI) + K[2]*xdot + K[3]*thetadot)*controller_dt) / DIST_PER_STEP;
    // dt should be 1/2 of calculation bc one step is HIGH -> LOW transition
    vel_dt = abs( 0.5 * 1.0e6 / desired_vel); // MAKE SURE TO GET A POSITIVE VALUE FOR DT!!
    dir = (desired_vel < 0) ? REV : FWD;

    // Run the controller for controller_dt seconds
    start_micros = micros();

    inc = 0;
    while((micros() - start_micros) < (control_dt * 1e6))
    {
      if( (micros() - prev_step_micros) >= vel_dt)
      {
        digitalWrite(STP_DIR,dir);
        digitalWrite(STP_STEP,step_state);
        step_state = !step_state;
        inc += (desired_vel > 0) ? 1 : -1;
        prev_step_micros = micros();
      } else
      {
        delayMicroseconds(5);
      }
    }
    motor_pos += inc/2;
  }
}

void setup()
{
  // Debugging
  if(DEBUG) {
    Serial.begin(BAUDRATE);
  }

  // Encoder Interrupt
  pinMode(ENC_A,INPUT);
  pinMode(ENC_B,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A),encoder_callback,RISING);

  // Limit Switch
  pinMode(LIM,INPUT);

  // Stepper Motor
  pinMode(STP_EN,OUTPUT);
  pinMode(STP_STEP,OUTPUT);
  pinMode(STP_DIR,OUTPUT);
  digitalWrite(STP_DIR,FWD);

  driver.begin();
  driver.rms_current(1000);
  driver.en_pwm_mode(STEALTHCHOP); // Turns on Stealth Chop (quiet mode)
  driver.microsteps(MICROSTEPS);
  digitalWrite(STP_EN,LOW);

  // Home
  home();
  move(FWD, 0.13, 0.06);
  delay(2000);

  //testMotion(0.04,1.5,8.0,10.0);

  swingup(); 
  //balance(10.0);
  digitalWrite(STP_EN,HIGH);

}

void loop()
{

}
