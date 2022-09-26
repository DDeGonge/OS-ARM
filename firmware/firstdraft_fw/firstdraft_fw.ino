#include "OSR.h"
#include "gparse.h"
#include "pins.h"

#define Serial SERIAL_PORT_USBVIRTUAL

void setup() {
  // put your setup code here, to run once:
  Serial.begin(250000);
  while (!Serial);
  Serial.println("BEGINNING!");

  delay(100);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

//  pinMode(S1CS, OUTPUT);
//  pinMode(14, OUTPUT);
//  pinMode(15, OUTPUT);
//  while(true)
//  {
//    digitalWrite(S1CS, HIGH);
//    digitalWrite(14, HIGH);
//    digitalWrite(15, HIGH);
//    delay(1000);
//    digitalWrite(S1CS, LOW);
//    digitalWrite(14, LOW);
//    digitalWrite(15, LOW);
//    delay(1000);
//  }
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
  TMCstep Stepper2 = TMCstep(S2STEP, S2DIR, Driver2, true);
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

  motorDrive mDrive0 = motorDrive(Stepper0, 3050);   // steps per rad
  motorDrive mDrive1 = motorDrive(Stepper1, 12987); // steps per rad, 102:1 reduction  - 103896
  motorDrive mDrive2 = motorDrive(Stepper2, 12987); // steps per rad, 102:1 reduction
  motorDrive mDrive3 = motorDrive(Stepper3, 10695);  // steps per rad 8:1 reduction - 10695
  motorDrive mDrive4 = motorDrive(Stepper4, 6400);   // steps per rad,
  motorDrive mDrive5 = motorDrive(Stepper5, 6400);   // steps per rad
  motorDrive mDrive6 = motorDrive(Stepper6, 6400);   // steps per rad

  Stepper0.set_run_current(29);
  Stepper0.set_hold_current(12);
  Stepper0.set_microsteps(4); // 1/16 stepping
  mDrive0.set_default_vel_mmps(1);
  mDrive0.set_default_acc_mmps2(20);
  mDrive0.set_move_limits_mm(-1.65, 1.65);

  Stepper1.set_run_current(30);
  Stepper1.set_hold_current(12);
  Stepper1.set_microsteps(6); // 1/4 stepping
  mDrive1.set_default_vel_mmps(0.5);
  mDrive1.set_default_acc_mmps2(5);
  mDrive0.set_move_limits_mm(0.0, 2.25);

  Stepper2.set_run_current(30);
  Stepper2.set_hold_current(12);
  Stepper2.set_microsteps(6); // 1/4 stepping
  mDrive2.set_default_vel_mmps(0.5);
  mDrive2.set_default_acc_mmps2(5);
  mDrive0.set_move_limits_mm(0.0, 2.25);

  Stepper3.set_run_current(30);
  Stepper3.set_hold_current(26);
  Stepper0.set_microsteps(4); // 1/16 stepping
  mDrive3.set_default_vel_mmps(1.0);
  mDrive3.set_default_acc_mmps2(25);
  mDrive3.set_move_limits_mm(-1.7, 1.7);

  Stepper4.set_run_current(28);
  Stepper4.set_hold_current(20);
  mDrive4.set_default_vel_mmps(1);
  mDrive4.set_default_acc_mmps2(50);
  mDrive4.set_move_limits_mm(0.0, 2.25);

  Stepper5.set_run_current(28);
  Stepper5.set_hold_current(20);
  mDrive5.set_default_vel_mmps(1);
  mDrive5.set_default_acc_mmps2(50);
  mDrive5.set_move_limits_mm(0.0, 2.25);

  Stepper6.set_run_current(28);
  Stepper6.set_hold_current(24);
  mDrive6.set_default_vel_mmps(0.5);
  mDrive6.set_default_acc_mmps2(25);
  mDrive6.set_move_limits_mm(-6.5, 6.5);

  Serial.println("Enabling and zeroing all motors");
  mDrive0.enable();
  mDrive1.enable();
  mDrive2.enable();
  mDrive3.enable();
  mDrive4.enable();
  mDrive5.enable();
  mDrive6.enable();

  char serial_data[MAX_MSG_LEN];
  clear_data(serial_data);
  char base_cmd, char_value;
  int32_t base_value, int_value;
  float float_value;
  bool run_dynostep_loop = false;

//  while (true) 
//  {
//    Serial.println("Drive 0...");
//    mDrive0.set_pos_target_mm_sync(0.1, 300*1, false);
//    mDrive0.set_pos_target_mm_sync(0, 300*1, false);
//    Serial.println("Drive 1...");
//    mDrive1.set_pos_target_mm_sync(0.1, 10*1, true);
//    mDrive1.set_pos_target_mm_sync(0, 10*1, false);
//    Serial.println("Drive 2...");
//    mDrive2.set_pos_target_mm_sync(0.1, 10*1, false);
//    mDrive2.set_pos_target_mm_sync(0, 10*1, false);
//    Serial.println("Drive 3...");
//    mDrive3.set_pos_target_mm_sync(0.1, 200*1, false);
//    mDrive3.set_pos_target_mm_sync(0, 200*1, false);
////    Serial.println("Drive 4...");
////    mDrive4.set_pos_target_mm_sync(1, 60*1, false);
////    mDrive4.set_pos_target_mm_sync(0, 60*1, false);
////    Serial.println("Drive 5...");
////    mDrive5.set_pos_target_mm_sync(1, 120*1, false);
////    mDrive5.set_pos_target_mm_sync(0, 120*1, false);
//    Serial.println("Drive 6...");
//    mDrive6.set_pos_target_mm_sync(0.1, 600*1, false);
//    mDrive6.set_pos_target_mm_sync(0, 600*1, false);
//  }

//  speedtest(mDrive1, 0.3, 0.5, 2, 0.1);

  while (true)
  {
    // First handle the async step thing if you wanna go this route
    if (run_dynostep_loop)
    {
      uint32_t tnow = micros();
      bool r0 = mDrive0.step_if_needed(tnow);
      bool r1 = mDrive1.step_if_needed(tnow);
      bool r2 = mDrive2.step_if_needed(tnow);
      bool r3 = mDrive3.step_if_needed(tnow);
      bool r6 = mDrive6.step_if_needed(tnow);

      if (!r0 && !r1 && !r2 && !r3 && !r6)
      {
        Serial.println("STOPPING DYNOSTEPPS");
        run_dynostep_loop = false;
      }
    }
    

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
            case 0: {
              // LINEAR MOVE DO NOT WAIT
              gcode_command_floats gcode(args);
              if(gcode.com_exists('x')) mDrive0.set_pos_target_mm_async(gcode.fetch('x'), gcode.fetch('f'));
              if(gcode.com_exists('y')) mDrive1.set_pos_target_mm_async(gcode.fetch('y'), gcode.fetch('f'));
              if(gcode.com_exists('z')) mDrive2.set_pos_target_mm_async(gcode.fetch('z'), gcode.fetch('f'));
              if(gcode.com_exists('a')) mDrive3.set_pos_target_mm_async(gcode.fetch('a'), gcode.fetch('f'));
              if(gcode.com_exists('b')) mDrive6.set_pos_target_mm_async(gcode.fetch('b'), gcode.fetch('f'));
              run_dynostep_loop = true;
              break;
            }
            case 1: {
              // LINEAR MOVE WAIT
              gcode_command_floats gcode(args);
              if(gcode.com_exists('x')) mDrive0.plan_move(gcode.fetch('x'), gcode.fetch('f'));
              if(gcode.com_exists('y')) mDrive1.plan_move(gcode.fetch('y'), gcode.fetch('f'));
              if(gcode.com_exists('z')) mDrive2.plan_move(gcode.fetch('z'), gcode.fetch('f'));
              if(gcode.com_exists('a')) mDrive3.plan_move(gcode.fetch('a'), gcode.fetch('f'));
              if(gcode.com_exists('b')) mDrive6.plan_move(gcode.fetch('b'), gcode.fetch('f'));
//              if(gcode.com_exists('c')) mDrive5.plan_move(gcode.fetch('c'), gcode.fetch('f'));
              
              mDrive0.execute_move_async();
              mDrive1.execute_move_async();
              mDrive2.execute_move_async();
              mDrive3.execute_move_async();
              mDrive6.execute_move_async();
//              mDrive5.execute_move_async();
              while(true)
              {
                uint32_t tnow = micros();
                bool r0 = mDrive0.async_move_step_check(tnow);
                bool r1 = mDrive1.async_move_step_check(tnow);
                bool r2 = mDrive2.async_move_step_check(tnow);
                bool r3 = mDrive3.async_move_step_check(tnow);
                bool r6 = mDrive6.async_move_step_check(tnow);
//                bool r5 = mDrive5.async_move_step_check(tnow);
                if(r0 && r1 && r2 && r3 && r6)
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
              if(gcode.com_exists('b')) mDrive6.set_current_pos_mm(gcode.fetch('b'));
//              if(gcode.com_exists('c')) mDrive5.set_current_pos_mm(gcode.fetch('c'));
              if(!gcode.com_exists('x') && !gcode.com_exists('y') && !gcode.com_exists('z') && !gcode.com_exists('a') && !gcode.com_exists('b')) // && !gcode.com_exists('c'))
              {
                mDrive0.zero();
                mDrive1.zero();
                mDrive2.zero();
                mDrive3.zero();
                mDrive6.zero();
//                mDrive5.zero();
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
              if(gcode.com_exists('b')) mDrive6.set_default_vel_mmps(gcode.fetch('b'));
//              if(gcode.com_exists('c')) mDrive5.set_default_vel_mmps(gcode.fetch('c'));
              break;
            }
            case 101: {
              // Overwrite default acc
              gcode_command_floats gcode(args);
              if(gcode.com_exists('x')) mDrive0.set_default_acc_mmps2(gcode.fetch('x'));
              if(gcode.com_exists('y')) mDrive1.set_default_acc_mmps2(gcode.fetch('y'));
              if(gcode.com_exists('z')) mDrive2.set_default_acc_mmps2(gcode.fetch('z'));
              if(gcode.com_exists('a')) mDrive3.set_default_acc_mmps2(gcode.fetch('a'));
              if(gcode.com_exists('b')) mDrive6.set_default_acc_mmps2(gcode.fetch('b'));
//              if(gcode.com_exists('c')) mDrive5.set_default_acc_mmps2(gcode.fetch('c'));
              break;
            }
          }
        }
        case 'm': {
          switch (base_value) 
          {
            case 80: {
              mDrive0.enable();
              mDrive1.enable();
              mDrive2.enable();
              mDrive3.enable();
              mDrive6.enable();
              break;
            }
            case 81: {
              mDrive0.disable();
              mDrive1.disable();
              mDrive2.disable();
              mDrive3.disable();
              mDrive6.disable();
              break;
            }
            case 114: {
              Serial.print("X:");
              Serial.print(mDrive0.get_current_pos_mm());
              Serial.print(" Y:");
              Serial.print(mDrive1.get_current_pos_mm());
              Serial.print(" Z:");
              Serial.print(mDrive2.get_current_pos_mm());
              Serial.print(" A:");
              Serial.print(mDrive3.get_current_pos_mm());
              Serial.print(" B:");
              Serial.print(mDrive6.get_current_pos_mm());
//              Serial.print(" C:");
//              Serial.print(mDrive0.get_current_pos_mm());
              Serial.println();
              break;
            }
          }
        }
        Serial.println("ok");
        clear_data(serial_data);
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
