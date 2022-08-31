#include "OSR.h"
#include "gparse.h"
#include "pins.h"

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
  Serial.println("Driver 0 initialized...");
  
  TMC5160 Driver1(S1EN, S1CS);
  TMCstep Stepper1 = TMCstep(S1STEP, S1DIR, Driver1, false);
  Serial.println("Driver 1 initialized...");
  
  TMC5160 Driver2(S2EN, S2CS);
  TMCstep Stepper2 = TMCstep(S2STEP, S2DIR, Driver2, false);
  Serial.println("Driver 2 initialized...");
  
  TMC2041 Driver3(S3S4EN, S3S4CS);
  TMCstep Stepper3 = TMCstep(S3STEP, S3DIR, Driver3, 0, false);
  TMCstep Stepper4 = TMCstep(S4STEP, S4DIR, Driver3, 1, false);
  Serial.println("Driver 3/4 initialized...");
  
  TMC2041 Driver5(S5S6EN, S5S6CS);
  TMCstep Stepper5 = TMCstep(S5STEP, S5DIR, Driver5, 0, false);
  TMCstep Stepper6 = TMCstep(S6STEP, S6DIR, Driver5, 1, false);
  Serial.println("Driver 5/6 initialized...");

  /* Stepper drive configuration */
  Serial.println("Configuring motor drivers...");
  Driver0.configure();
  Driver1.configure();
  Driver2.configure();
  Driver3.configure();
  Driver5.configure();
  Serial.println("Configured motor drivers");

  motorDrive mDrive0 = motorDrive(Stepper0, 24400);   // steps per rad
  motorDrive mDrive1 = motorDrive(Stepper1, 103896); // steps per rad, 102:1 reduction
  motorDrive mDrive2 = motorDrive(Stepper2, 103896); // steps per rad, 102:1 reduction
  motorDrive mDrive3 = motorDrive(Stepper3, 10695);  // steps per rad 8:1 reduction
  motorDrive mDrive4 = motorDrive(Stepper4, 6400);   // steps per rad,
  motorDrive mDrive5 = motorDrive(Stepper5, 6400);   // steps per rad
  motorDrive mDrive6 = motorDrive(Stepper6, 6400);   // steps per rad

  Stepper0.set_run_current(24);
  Stepper0.set_hold_current(12);
  mDrive0.set_default_vel_mmps(1);
  mDrive0.set_default_acc_mmps2(40);

  Stepper1.set_run_current(30);
  Stepper1.set_hold_current(12);
  mDrive1.set_default_vel_mmps(0.05);
  mDrive1.set_default_acc_mmps2(20);

  Stepper2.set_run_current(30);
  Stepper2.set_hold_current(12);
  mDrive2.set_default_vel_mmps(0.05);
  mDrive2.set_default_acc_mmps2(20);

  Stepper3.set_run_current(28);
  Stepper3.set_hold_current(20);
  mDrive3.set_default_vel_mmps(1);
  mDrive3.set_default_acc_mmps2(10);

  Stepper4.set_run_current(28);
  Stepper4.set_hold_current(20);
  mDrive4.set_default_vel_mmps(1);
  mDrive4.set_default_acc_mmps2(10);

  Stepper5.set_run_current(28);
  Stepper5.set_hold_current(20);
  mDrive5.set_default_vel_mmps(1);
  mDrive5.set_default_acc_mmps2(10);

  Stepper6.set_run_current(28);
  Stepper6.set_hold_current(20);
  mDrive6.set_default_vel_mmps(1);
  mDrive6.set_default_acc_mmps2(10);

  Serial.println("Enabling and zeroing all motors");
  mDrive0.enable();
  mDrive1.enable();
  mDrive2.enable();
  mDrive3.enable();
  mDrive4.enable();
  mDrive5.enable();
  mDrive6.enable();

  char serial_data[MAX_MSG_LEN];
  char base_cmd, char_value;
  int32_t base_value, int_value;
  float float_value;

  while (true) 
  {
    Serial.println("Drive 0...");
    mDrive0.set_pos_target_mm_sync(0.5, 300*1, false);
    mDrive0.set_pos_target_mm_sync(0, 300*1, false);
//    Serial.println("Drive 1...");
//    mDrive1.set_pos_target_mm_sync(-20, 40*1, true);
//    mDrive1.set_pos_target_mm_sync(0, 20*1, false);
//    Serial.println("Drive 2...");
//    mDrive2.set_pos_target_mm_sync(5, 20*1, false);
//    mDrive2.set_pos_target_mm_sync(0, 20*1, false);
//    Serial.println("Drive 3...");
//    mDrive3.set_pos_target_mm_sync(1, 200*1, false);
//    mDrive3.set_pos_target_mm_sync(0, 200*1, false);
//    Serial.println("Drive 4...");
//    mDrive4.set_pos_target_mm_sync(1, 60*1, false);
//    mDrive4.set_pos_target_mm_sync(0, 60*1, false);
//    Serial.println("Drive 5...");
//    mDrive5.set_pos_target_mm_sync(1, 120*1, false);
//    mDrive5.set_pos_target_mm_sync(0, 120*1, false);
//    Serial.println("Drive 6...");
//    mDrive6.set_pos_target_mm_sync(1, 600*1, false);
//    mDrive6.set_pos_target_mm_sync(0, 600*1, false);
  }

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
              gcode_command_floats gcode(args);
              if(gcode.com_exists('x')) mDrive0.plan_move(gcode.fetch('x'), gcode.fetch('f'));
              if(gcode.com_exists('y')) mDrive1.plan_move(gcode.fetch('y'), gcode.fetch('f'));
              if(gcode.com_exists('z')) mDrive2.plan_move(gcode.fetch('z'), gcode.fetch('f'));
              if(gcode.com_exists('a')) mDrive3.plan_move(gcode.fetch('a'), gcode.fetch('f'));
              if(gcode.com_exists('b')) mDrive4.plan_move(gcode.fetch('b'), gcode.fetch('f'));
              if(gcode.com_exists('c')) mDrive5.plan_move(gcode.fetch('c'), gcode.fetch('f'));
              
              mDrive0.execute_move_async();
              mDrive1.execute_move_async();
              mDrive2.execute_move_async();
              mDrive3.execute_move_async();
              mDrive4.execute_move_async();
              mDrive5.execute_move_async();
              while(true)
              {
                uint32_t tnow = micros();
                bool r0 = mDrive0.async_move_step_check(tnow);
                bool r1 = mDrive1.async_move_step_check(tnow);
                bool r2 = mDrive2.async_move_step_check(tnow);
                bool r3 = mDrive3.async_move_step_check(tnow);
                bool r4 = mDrive4.async_move_step_check(tnow);
                bool r5 = mDrive5.async_move_step_check(tnow);
                if(r0 && r1 && r2 && r3 && r4 && r5)
                  break;
              }
              break;
            }
            case 92: {
              // Overwrite current pos
              gcode_command_floats gcode(args);
              if(gcode.com_exists('x')) mDrive0.set_current_pos_mm(gcode.fetch('x'));
              if(gcode.com_exists('y')) mDrive1.set_current_pos_mm(gcode.fetch('y'));
              if(gcode.com_exists('z')) mDrive2.set_current_pos_mm(gcode.fetch('z'));
              if(gcode.com_exists('a')) mDrive3.set_current_pos_mm(gcode.fetch('a'));
              if(gcode.com_exists('b')) mDrive4.set_current_pos_mm(gcode.fetch('b'));
              if(gcode.com_exists('c')) mDrive5.set_current_pos_mm(gcode.fetch('c'));
              if(!gcode.com_exists('x') && !gcode.com_exists('y') && !gcode.com_exists('z') && !gcode.com_exists('a') && !gcode.com_exists('b') && !gcode.com_exists('c'))
              {
                mDrive0.zero();
                mDrive1.zero();
                mDrive2.zero();
                mDrive3.zero();
                mDrive4.zero();
                mDrive5.zero();
              }
              break;
            }
            case 100: {
              // Overwrite default vel
              gcode_command_floats gcode(args);
              if(gcode.com_exists('x')) mDrive0.set_default_vel_mmps(gcode.fetch('x'));
              if(gcode.com_exists('y')) mDrive1.set_default_vel_mmps(gcode.fetch('y'));
              if(gcode.com_exists('z')) mDrive2.set_default_vel_mmps(gcode.fetch('z'));
              if(gcode.com_exists('a')) mDrive3.set_default_vel_mmps(gcode.fetch('a'));
              if(gcode.com_exists('b')) mDrive4.set_default_vel_mmps(gcode.fetch('b'));
              if(gcode.com_exists('c')) mDrive5.set_default_vel_mmps(gcode.fetch('c'));
              break;
            }
            case 101: {
              // Overwrite default acc
              gcode_command_floats gcode(args);
              if(gcode.com_exists('x')) mDrive0.set_default_acc_mmps2(gcode.fetch('x'));
              if(gcode.com_exists('y')) mDrive1.set_default_acc_mmps2(gcode.fetch('y'));
              if(gcode.com_exists('z')) mDrive2.set_default_acc_mmps2(gcode.fetch('z'));
              if(gcode.com_exists('a')) mDrive3.set_default_acc_mmps2(gcode.fetch('a'));
              if(gcode.com_exists('b')) mDrive4.set_default_acc_mmps2(gcode.fetch('b'));
              if(gcode.com_exists('c')) mDrive5.set_default_acc_mmps2(gcode.fetch('c'));
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
