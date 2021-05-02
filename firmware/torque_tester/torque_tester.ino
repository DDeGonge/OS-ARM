#include <HX711_ADC.h>

#define EN 6
#define DIR 9
#define STEP 10

#define HX711_dout 12 //mcu > HX711 dout pin
#define HX711_sck 13 //mcu > HX711 sck pin

static int steps_per_rev = 185600;
static bool reverse = true;
static int step_dwell_slow_us = 10000;
static int step_dwell_fast_us = 500;
static int step_high_us = 5;

HX711_ADC LoadCell(HX711_dout, HX711_sck);
static float calibrationValue = 109209.89;
static float arm_len_m = 0.2;
static float g_accel = 9.81;

static float min_force_kg = 0.03; // Min force that is def not noise, for homing
static int serialPrintInterval = 100;

void setup() {
  // put your setup code here, to run once:
  pinMode(EN, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(STEP, OUTPUT);
  digitalWrite(EN, LOW);
  digitalWrite(DIR, reverse);

  Serial.begin(115200);
  while (!Serial);

  LoadCell.begin();
  LoadCell.start(500, true);
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1);
  }
  else {
    LoadCell.setCalFactor(calibrationValue); // set calibration value (float)
    LoadCell.setSamplesInUse(2);
    Serial.println("HX711 Startup is complete");
  }

//  debug_print(LoadCell);
//  debug_spin();
}

void loop() {
  // put your main code here, to run repeatedly:
  float load_kg = 0, load_nm = 0;
  unsigned long last_print = 0, last_step = 0;

  serialFlush();
  Serial.println("enter 's' to move to start");
  Serial.println("enter 't' for torque test");
  Serial.println("enter 'c' for cycle test");

  while (Serial.available() == 0);
  char inByte = Serial.read();

  switch (inByte)
  {
    case 's':
    {
      move_to_start(LoadCell, false);
      break;
    }
    case 't':
    {
      backlash_test(LoadCell);
      break;
    }
    case 'c':
    {
      lifeTest(LoadCell, 0.2, 1.0, 100);
      break;
    }
  }
}

void debug_print(HX711_ADC LoadCell)
{
  while(true)
  {
    if (LoadCell.update()) 
    {
      Serial.println(LoadCell.getData() * g_accel * arm_len_m);
    }
  }
}

void debug_spin()
{
  digitalWrite(DIR, reverse);
  while(true)
  {
    digitalWrite(STEP, HIGH);
    delayMicroseconds(5);
    digitalWrite(STEP, LOW);
    delayMicroseconds(200);
  }
}

void move_to_start(HX711_ADC LoadCell, bool extra_backoff)
{
//  Serial.println("Starting homing sequence");
  int state = 0; // 0 is back-off, 1 is extra back-off, 2 is fast seek, 3 is back-off, 4 is extra back-off, 5 is slow seek
  while (true)
  {
    long last_step = 0;
    int move_dwell = step_dwell_slow_us;
    float load_kg = 0;
//    Serial.print("State | ");
//    Serial.println(state);
    switch (state)
    {
      case 0: // back-off until zero force
      case 3:
      {
        digitalWrite(DIR, !reverse);
        load_kg = 100;
        while (load_kg > min_force_kg)
        {
          if (LoadCell.update()) 
          {
            load_kg = LoadCell.getData();
          }
        
          // Step motor if it is time
          if (micros() > last_step + step_dwell_fast_us)
          {
            digitalWrite(STEP, HIGH);
            delayMicroseconds(step_high_us);
            digitalWrite(STEP, LOW);
            last_step = micros();
          }
        }
        break;
      }
      case 1: //extra back-off
      case 4:
      {
        digitalWrite(DIR, !reverse);
        manySteps(50, step_dwell_fast_us);
        break;
      }
      case 2: // fast seek
        move_dwell = step_dwell_fast_us;
      case 5: // slow seek
      {
        digitalWrite(DIR, reverse);
        load_kg = 0;
        while (load_kg < min_force_kg)
        {
          if (LoadCell.update()) 
          {
            load_kg = LoadCell.getData();
          }
        
          // Step motor if it is time
          if (micros() > last_step + move_dwell)
          {
            digitalWrite(STEP, HIGH);
            delayMicroseconds(step_high_us);
            digitalWrite(STEP, LOW);
            last_step = micros();
          }
        }
        break;
      }
      case 6:
      {
        if (extra_backoff)
        {
          digitalWrite(DIR, !reverse);
          manySteps(1000, step_dwell_fast_us);
        }
        return;
      }
    }
    state++;
  }
}

void backlash_test(HX711_ADC LoadCell)
{
  move_to_start(LoadCell, true);
  Serial.println("Press down on the arm with ~1kg force");
  Serial.println("Then, enter desired load in mNm...");
  serialFlush();
  while (Serial.available() == 0);
  unsigned int target_mNm = Serial.parseInt();
  Serial.println(target_mNm);
  move_to_start(LoadCell, false);
  
  long steps_taken = 0, last_print = 0, last_step = 0;
  digitalWrite(DIR, reverse);
  float load_mNm = 0;
  while (load_mNm < target_mNm)
  {
    if (LoadCell.update()) 
    {
      load_mNm = LoadCell.getData() * g_accel * arm_len_m * 1000;
    }
  
    // Step motor if it is time
    if (micros() > last_step + step_dwell_slow_us)
    {
      digitalWrite(STEP, HIGH);
      delayMicroseconds(step_high_us);
      digitalWrite(STEP, LOW);
      steps_taken++;
      last_step = micros();
    }

    // periodically print to terminal
    if (millis() > last_print + serialPrintInterval)
    {
      double rads = steps_taken;
      rads *= (2 * 3.14159);
      rads /= steps_per_rev;
      Serial.print(rads);
      Serial.print(" rads\t");
      Serial.print(load_mNm / 1000);
      Serial.println(" NM");
      last_print = millis();
    }
  }
  double arc_mins = steps_taken;
  arc_mins *= 60 * 360;
  arc_mins /= steps_per_rev;
  Serial.print(arc_mins);
  Serial.print(" arc minutes backlash measured at ");
  Serial.print(load_mNm / 1000);
  Serial.println(" NM");
}

void lifeTest(HX711_ADC LoadCell, float dist_rads, float frequency, int cycles_between_test)
{
  long cycles = 0;
  long stepsPerZip = dist_rads * steps_per_rev / 6.283;
  long delay_us = 1000000 / (frequency * 2 * stepsPerZip);
  delay_us -= step_high_us;
  delay_us = delay_us < step_high_us ? step_high_us : delay_us;

  while (true)
  {
    // Test max force
    Serial.println();
    Serial.print(cycles);
    Serial.println(" cycles");
    move_to_start(LoadCell, false);
    testMaxForce(LoadCell);

    // Run cycles
    for (long i = 0; i < cycles_between_test; i++)
    {
      cycles++;
      Serial.print("|");
      digitalWrite(DIR, !reverse);
      manySteps(stepsPerZip, delay_us);
      digitalWrite(DIR, reverse);
      manySteps(stepsPerZip, delay_us);
    }

    // Rinse and repeeet
  }
}

void testMaxForce(HX711_ADC LoadCell)
{
  digitalWrite(DIR, reverse);
  long steps_at_peak = 0, steps_taken = 0, last_print = 0, last_step = 0;
  float load_mNm = 0, peak_load = 0;
  while (true)
  {
    // Update if load cell reading available
    if (LoadCell.update()) 
    {
      load_mNm = LoadCell.getData() * g_accel * arm_len_m * 1000;
      load_mNm = load_mNm < 0 ? 0 : load_mNm;
      if (load_mNm > peak_load)
      {
        peak_load = load_mNm;
        steps_at_peak = steps_taken;
      }

      if ((peak_load - load_mNm) > 500) break;
    }
  
    // Step motor if it is time
    if (micros() > last_step + step_dwell_slow_us)
    {
      digitalWrite(STEP, HIGH);
      delayMicroseconds(step_high_us);
      digitalWrite(STEP, LOW);
      steps_taken++;
      last_step = micros();
    }
  }
  double arc_mins = peak_load;
  arc_mins *= (60 * 360);
  arc_mins /= steps_per_rev;
  Serial.print(arc_mins);
  Serial.print(" arc minutes deflection at ");
  Serial.print(peak_load / 1000);
  Serial.println(" NM");

  digitalWrite(DIR, !reverse);
  manySteps(2*steps_taken, step_dwell_fast_us);
}

void cycleTest(float dist_rads)
{
  long stepsPerDir = dist_rads * steps_per_rev / 6.283;
  while (true)
  {
    digitalWrite(DIR, !reverse);
    manySteps(stepsPerDir, 100);
    digitalWrite(DIR, reverse);
    manySteps(stepsPerDir, 100);
  }
}

void manySteps(int nSteps, int speed_delay)
{
  for(int i = 0; i < nSteps; i++)
  {
    digitalWrite(STEP, HIGH);
    delayMicroseconds(step_high_us);
    digitalWrite(STEP, LOW);
    delayMicroseconds(speed_delay);
  }
}

void serialFlush()
{
  while(Serial.available() > 0) {
    char t = Serial.read();
  }
}
