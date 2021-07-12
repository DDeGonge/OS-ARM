#include <OSR.h>
#include "gparse.h"

#define Serial SERIAL_PORT_USBVIRTUAL


void setup() {
  // put your setup code here, to run once:
  Serial.begin(250000);
  while (!Serial);
  Serial.println("BEGINNING!");

  delay(2000);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
  uint8_t clear_faults[4] = {B00000000, B00000000, B00000000, B00000111};  // write this to 0x01 to clear driver faults
  
  /* Stepper drive initialization */
  TMC5160 Driver0(S0EN, S0CS);
  TMCstep Stepper0 = TMCstep(S0STEP, S0DIR, Driver0, false);
  
  TMC5160 Driver1(S1EN, S1CS);
  TMCstep Stepper1 = TMCstep(S1STEP, S1DIR, Driver1, false);
  
  TMC5160 Driver2(S2EN, S2CS);
  TMCstep Stepper2 = TMCstep(S2STEP, S2DIR, Driver2, false);
  
  TMC2041 Driver3(S3S4EN, S3S4CS, true);  // CONFIGURED FOR SINGLE MOT DRIVE ATM
  TMCstep Stepper3 = TMCstep(S3STEP, S3DIR, Driver3, 0, false);
  //TMCstep Stepper4 = TMCstep(S4STEP, S4DIR, Driver3, 1, false);
  
  TMC2041 Driver5(S5S6EN, S5S6CS, true);  // CONFIGURED FOR SINGLE MOT DRIVE ATM
  TMCstep Stepper5 = TMCstep(S5STEP, S5DIR, Driver5, 0, true);
  //TMCstep Stepper6 = TMCstep(S6STEP, S6DIR, Driver5, 1, false);

  /* Stepper drive configuration */
  Serial.println("Configuring motor drivers...");
  Driver0.configure();
  Driver1.configure();
  Driver2.configure();
  Driver3.configure();
  Driver5.configure();
  Serial.println("Configured motor drivers");


  motorDrive mDrive3 = motorDrive(Stepper3, 8149);   // steps per rad, 8:1 reduction
  motorDrive mDrive5 = motorDrive(Stepper5, 103896); // steps per rad, 102:1 reduction
  
  mDrive3.set_default_vel_mmps(0.05);
  mDrive3.set_default_acc_mmps2(20);

  mDrive5.set_default_vel_mmps(0.05);
  mDrive5.set_default_acc_mmps2(20);

  Serial.println("Enabling and zeroing all motors");
  mDrive3.enable();
  mDrive5.enable();

  char serial_data[MAX_MSG_LEN];
  char base_cmd, char_value;
  int32_t base_value, int_value;
  float float_value;

  while (true)
  {
    if (respondToSerial(serial_data)) 
    {
      vector<string> args;
      parse_inputs(serial_data, args);
      parse_int(args[0], base_cmd, base_value);

      switch (tolower(base_cmd)) 
      {
        case 'g': {
          switch (base_value) 
          {
            case 0:
            case 1: {
              // LINEAR MOVE DO NOT WAIT
              float xpos, ypos, feedrate;
              gcode_command_floats gcode(args);
              if(gcode.com_exists('x')) mDrive3.plan_move(gcode.fetch('x'), gcode.fetch('f'));
              if(gcode.com_exists('y')) mDrive5.plan_move(gcode.fetch('y'), gcode.fetch('f'));
              mDrive3.execute_move_async();
              mDrive5.execute_move_async();
              while(true)
              {
                uint32_t tnow = micros();
                bool r3 = mDrive3.async_move_step_check(tnow);
                bool r5 = mDrive5.async_move_step_check(tnow);
                if(r3 && r5)
                  break;
              }
              break;
            }
            case 92: {
              // Overwrite current pos
              gcode_command_floats gcode(args);
              if(gcode.com_exists('x')) mDrive3.set_current_pos_mm(gcode.fetch('x'));
              if(gcode.com_exists('y')) mDrive5.set_current_pos_mm(gcode.fetch('y'));
              if(!gcode.com_exists('x') && !gcode.com_exists('y'))
              {
                mDrive3.zero();
                mDrive5.zero();
              }
              break;
            }
            case 100: {
              // Overwrite default vel
              gcode_command_floats gcode(args);
              if(gcode.com_exists('x')) mDrive3.set_default_vel_mmps(gcode.fetch('x'));
              if(gcode.com_exists('y')) mDrive5.set_default_vel_mmps(gcode.fetch('y'));
              break;
            }
            case 101: {
              // Overwrite default acc
              gcode_command_floats gcode(args);
              if(gcode.com_exists('x')) mDrive3.set_default_acc_mmps2(gcode.fetch('x'));
              if(gcode.com_exists('y')) mDrive5.set_default_acc_mmps2(gcode.fetch('y'));
              break;
            }
          }
        }
      }
    }
  }
  

  delay(200);
  int movetarget = 1;

//  speedtest(mDrive3, 20, 10, 20, 0.5);
//  speedtest(mDrive5, 0.5, 0.5, 2, 0.1);

//  while (true) 
//  {
//    mDrive3.set_pos_target_mm_sync(3, 60*5, false);
//    mDrive3.set_pos_target_mm_sync(0, 60*5, false);
//    mDrive5.set_pos_target_mm_sync(0.1, 60*0.5, false);
//    mDrive5.set_pos_target_mm_sync(0, 60*0.5, false);
//  }

  while (true)
  {
    Serial.println("Moving all motors to 100mm simeltaneously");
    // Plan each motors move
    mDrive3.plan_move(2);
    mDrive5.plan_move(0.5);
  
    // Begin move
    mDrive3.execute_move_async();
    mDrive5.execute_move_async();
  
    // Loop until all are done stepping
    while(true)
    {
      uint32_t tnow = micros();
      bool r3 = mDrive3.async_move_step_check(tnow);
      bool r5 = mDrive5.async_move_step_check(tnow);
      if(r3 && r5)
        break;
    }

    mDrive3.plan_move(0);
    mDrive5.plan_move(0);
  
    // Begin move
    mDrive3.execute_move_async();
    mDrive5.execute_move_async();
  
    // Loop until all are done stepping
    while(true)
    {
      uint32_t tnow = micros();
      bool r3 = mDrive3.async_move_step_check(tnow);
      bool r5 = mDrive5.async_move_step_check(tnow);
      if(r3 && r5)
        break;
    }
  }
}

void speedtest(motorDrive mDrive, float dist, float vstart, float vend, float vstep)
{
  for (float v = vstart; v < vend; v += vstep)
  {
    Serial.print("Setting speed to ");
    Serial.print(v);
    Serial.print(" rads/s\n");

    mDrive.set_pos_target_mm_sync(dist, 60*v, false);
    mDrive.set_pos_target_mm_sync(0, 60*v, false);
  }
}
