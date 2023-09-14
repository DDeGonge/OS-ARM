#include <Arduino.h>
#include "OSR.h"

TMC2041::TMC2041(uint8_t en_pin, uint8_t cs_pin)
{
    set_pins(en_pin, cs_pin);

    // Start SPI if it aint running
    SPI.begin();
}

TMC2041::TMC2041(uint8_t en_pin, uint8_t cs_pin, bool single_motor_mode)
{
    set_pins(en_pin, cs_pin);

    // Start SPI if it aint running
    SPI.begin();

    // TODO make this not so janky
    if (single_motor_mode == true)
    {
        uint8_t new_gconf[4]  = {B00000000, B00000000, B00000000, B00000111};
        std::copy(std::begin(new_gconf), std::end(new_gconf), std::begin(gconf));
    }
}

void TMC2041::set_pins(uint8_t en_pin, uint8_t cs_pin)
{
    // Save and set up CS and enable pins
    CS_PIN = cs_pin;
    pinMode(CS_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH);
    EN_PIN = en_pin;
    pinMode(EN_PIN, OUTPUT);
    digitalWrite(EN_PIN, HIGH);
}

void TMC2041::configure()
{
    // Configure driver with defaults and enable driver (not motors)
    write_gconf();
    enable_driver();  // HW pin will be left enabled, disable individual motors with SPI commands
}

// Public - enable stepper driver
void TMC2041::enable_driver()
{
  digitalWrite(EN_PIN, LOW);
}

// Public - disable stepper motor
void TMC2041::disable_driver()
{
  digitalWrite(EN_PIN, HIGH);
}

void TMC2041::write_cmd(uint8_t addr, uint8_t chunk[4])
{
    SPI.beginTransaction(TMCspiSettings);
    digitalWrite(CS_PIN, LOW);

    SPI.transfer(addr + 0x80);
    for (uint8_t i = 0; i < 4; i++)
        SPI.transfer(chunk[i]);

    SPI.endTransaction();
    digitalWrite(CS_PIN, HIGH);

    // Serial.print(addr, HEX);
    // for(uint8_t j = 0; j < 4; j++)
    // {
    //     Serial.print("\t");
    //     Serial.print(chunk[j], BIN);
    // }
    int32_t verifychunk = read_cmd(addr);
     Serial.print("\t");
     Serial.print(verifychunk, BIN);
     Serial.println();
}

int32_t TMC2041::read_cmd(uint8_t addr)
{
    SPI.beginTransaction(TMCspiSettings);
    digitalWrite(CS_PIN, LOW);

    SPI.transfer(addr);
    SPI.transfer16(0x0000);
    SPI.transfer16(0x0000);

    digitalWrite(CS_PIN, HIGH);
    digitalWrite(CS_PIN, LOW);
    
    uint8_t status_resp = SPI.transfer(addr);
    uint32_t out = SPI.transfer(0x00);
    out <<= 8;
    out |= SPI.transfer(0x00);
    out <<= 8;
    out |= SPI.transfer(0x00);
    out <<= 8;
    out |= SPI.transfer(0x00);

    SPI.endTransaction();
    digitalWrite(CS_PIN, HIGH);

    return out;
}

void TMC2041::write_gconf()
{
    write_cmd(a_gconf, gconf);
}

uint8_t TMC2041::get_ihold_address(uint8_t index)
{
    return a_iholdirun[index];
}

uint8_t TMC2041::get_chop_address(uint8_t index)
{
    return a_chop[index];
}

uint8_t TMC2041::get_cool_address(uint8_t index)
{
    return a_cool[index];
}

uint8_t TMC2041::get_status_address(uint8_t index)
{
    return a_status[index];
}


TMC5160::TMC5160(uint8_t en_pin, uint8_t cs_pin)
{
    set_pins(en_pin, cs_pin);

    // Start SPI if it aint running
    SPI.begin();
}

void TMC5160::set_pins(uint8_t en_pin, uint8_t cs_pin)
{
    // Save and set up CS and enable pins
    CS_PIN = cs_pin;
    pinMode(CS_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH);
    EN_PIN = en_pin;
    pinMode(EN_PIN, OUTPUT);
    digitalWrite(EN_PIN, HIGH);
}

void TMC5160::configure()
{
    // Configure driver with defaults and enable driver (not motors)
    write_gconf();
}

// Public - enable stepper driver
void TMC5160::enable_driver()
{
  digitalWrite(EN_PIN, LOW);
}

// Public - disable stepper motor
void TMC5160::disable_driver()
{
  digitalWrite(EN_PIN, HIGH);
}

void TMC5160::write_cmd(uint8_t addr, uint8_t chunk[4])
{
    SPI.beginTransaction(TMCspiSettings);
    digitalWrite(CS_PIN, LOW);

    SPI.transfer(addr + 0x80);
    for (uint8_t i = 0; i < 4; i++)
        SPI.transfer(chunk[i]);

    SPI.endTransaction();
    digitalWrite(CS_PIN, HIGH);
}

int32_t TMC5160::read_cmd(uint8_t addr)
{
    SPI.beginTransaction(TMCspiSettings);
    digitalWrite(CS_PIN, LOW);

    SPI.transfer(addr);
    SPI.transfer16(0x0000);
    SPI.transfer16(0x0000);

    digitalWrite(CS_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(CS_PIN, LOW);
    
    uint8_t status_resp = SPI.transfer(addr);
    uint32_t out = SPI.transfer(0x00);
    out <<= 8;
    out |= SPI.transfer(0x00);
    out <<= 8;
    out |= SPI.transfer(0x00);
    out <<= 8;
    out |= SPI.transfer(0x00);

    SPI.endTransaction();
    digitalWrite(CS_PIN, HIGH);

    return out;
}

void TMC5160::write_gconf()
{
    write_cmd(a_gconf, gconf);
}

uint8_t TMC5160::get_ihold_address()
{
    return a_iholdirun;
}

uint8_t TMC5160::get_chop_address()
{
    return a_chop;
}

uint8_t TMC5160::get_cool_address()
{
    return a_cool;
}

uint8_t TMC5160::get_status_address()
{
    return a_status;
}

uint8_t TMC5160::get_gstat_address()
{
    return a_gstat;
}

uint8_t TMC5160::get_tpowerdown_address()
{
    return a_tpowerdown;
}

uint8_t TMC5160::get_tpwmthrs_address()
{
    return a_tpwmthrs;
}


TMCstep::TMCstep(uint8_t step_pin, uint8_t dir_pin, TMC2041 &my_driver, uint8_t motor_index, bool reverse)
{
    set_pins(step_pin, dir_pin);
    set_index(motor_index);
    driver_2041 = my_driver;
    drv_type = 1;
    reversed = reverse;

    set_2041_default_bitfields();

    // Write initial configuration
    write_iholdirun();
    write_chop();
    write_cool();
}

TMCstep::TMCstep(uint8_t step_pin, uint8_t dir_pin, TMC5160 &my_driver, bool reverse)
{
    set_pins(step_pin, dir_pin);
    driver_5160 = my_driver;
    drv_type = 2;
    reversed = reverse;

    set_5160_default_bitfields();

    // Write initial configuration
    write_iholdirun();
    write_chop();
    write_cool();

    uint8_t tpowerdown[4] = {B00000000, B00000000, B00000000, B00001010};
    driver_5160.write_cmd(driver_5160.get_tpowerdown_address(), tpowerdown);

    uint8_t tpwmthrs[4] = {B00000000, B00000000, B00000001, B11110100};
    driver_5160.write_cmd(driver_5160.get_tpwmthrs_address(), tpwmthrs);
}


void TMCstep::set_2041_default_bitfields()
{
    uint8_t new_ihold[4] = {B00000000, B00000001, B00010000, B00001000};
    uint8_t new_chop[4]  = {B00000010, B00000001, B00000000, B00000011};
    uint8_t new_cool[4]  = {B00000000, B00000110, B00000000, B00000000};

    std::copy(std::begin(new_ihold), std::end(new_ihold), std::begin(ihold));
    std::copy(std::begin(new_chop), std::end(new_chop), std::begin(chop));
    std::copy(std::begin(new_cool), std::end(new_cool), std::begin(cool));
}

void TMCstep::set_5160_default_bitfields()
{
    uint8_t new_ihold[4] = {B00000000, B00000001, B00011110, B00001000};
    uint8_t new_chop[4]  = {B00000010, B00000001, B00000000, B00001001};
    uint8_t new_cool[4]  = {B00000000, B00000110, B00000000, B00000000};

    std::copy(std::begin(new_ihold), std::end(new_ihold), std::begin(ihold));
    std::copy(std::begin(new_chop), std::end(new_chop), std::begin(chop));
    std::copy(std::begin(new_cool), std::end(new_cool), std::begin(cool));
}

void TMCstep::set_pins(uint8_t step_pin, uint8_t dir_pin)
{
    STEP_PIN = step_pin;
    pinMode(STEP_PIN, OUTPUT);
    digitalWrite(STEP_PIN, LOW);
    DIR_PIN = dir_pin;
    pinMode(DIR_PIN, OUTPUT);
    digitalWrite(DIR_PIN, LOW);
}

void TMCstep::set_index(uint8_t index)
{
    INDEX = index;
}

uint8_t TMCstep::get_index()
{
    return INDEX;
}

void TMCstep::step()
{
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(step_pulse_len_us);
    digitalWrite(STEP_PIN, LOW);

    step_count = motor_dir ? step_count + 1 : step_count - 1;
}

void TMCstep::set_step(int32_t steps)
{
    step_count = steps;
}

int32_t TMCstep::get_step()
{
    return step_count;
}

void TMCstep::set_dir(bool dir)
{
    if (motor_dir != dir)
    {
        motor_dir = dir;
        uint8_t setdir = motor_dir ? HIGH : LOW;
        if (reversed)
            digitalWrite(DIR_PIN, !setdir);
        else
            digitalWrite(DIR_PIN, setdir);
    }
}

bool TMCstep::get_dir()
{
    return motor_dir;
}

void TMCstep::enable()
{
    if (drv_type == 1)
    {
        chop[3] = saved_chop;
        write_chop();
    }
    else if (drv_type == 2)
    {
        driver_5160.enable_driver();
    }
    
}

void TMCstep::disable()
{
    if (drv_type == 1)
    {
        saved_chop = chop[3];
        chop[3] = 0x00;
        write_chop();
    }
    else if (drv_type == 2)
    {
        driver_5160.disable_driver();
    }
    
}

void TMCstep::write_iholdirun()
{
    if (drv_type == 1)
    {
        driver_2041.write_cmd(driver_2041.get_ihold_address(INDEX), ihold);
    }
    else if (drv_type == 2)
    {
        driver_5160.write_cmd(driver_5160.get_ihold_address(), ihold);
    }
}

void TMCstep::write_chop()
{
    if (drv_type == 1)
    {
        driver_2041.write_cmd(driver_2041.get_chop_address(INDEX), chop);
    }
    else if (drv_type == 2)
    {
        driver_5160.write_cmd(driver_5160.get_chop_address(), chop);
    }
}

void TMCstep::write_cool()
{
    if (drv_type == 1)
    {
        driver_2041.write_cmd(driver_2041.get_cool_address(INDEX), cool);
    }
    else if (drv_type == 2)
    {
        driver_5160.write_cmd(driver_5160.get_cool_address(), cool);
    }
}

void TMCstep::update_motor_status()
{
    if (drv_type == 1)
    {
        status_bits = driver_2041.read_cmd(driver_2041.get_status_address(INDEX));
    }
    else if (drv_type == 2)
    {
        status_bits = driver_5160.read_cmd(driver_5160.get_status_address());
    }
}

bool TMCstep::get_stallguard()
{
    return status_bits & 0x01000000;
}

uint32_t TMCstep::get_stall_value()
{
    return status_bits & 0x000003FF;
}

uint32_t TMCstep::debug_print_status()
{
    return status_bits;
}

void TMCstep::set_hold_current(uint8_t newval)
{
    newval = newval > 31 ? 31 : newval;
    write_bits(ihold[3], newval, 3, 7);
    write_iholdirun();
}

void TMCstep::set_run_current(uint8_t newval)
{
    newval = newval > 31 ? 31 : newval;
    write_bits(ihold[2], newval, 3, 7);
    write_iholdirun();
}

void TMCstep::set_microsteps(uint8_t newval)
{
    newval = newval > 8 ? 8 : newval;
    write_bits(chop[0], newval, 4, 7);
    write_chop();
}

void TMCstep::clear_bits(uint8_t &chunk, uint8_t i_min, uint8_t i_max)
{
    for(uint8_t i = (7 - i_max); i <= (7 - i_min); i++)
        chunk &= ~(1u << i);
}

void TMCstep::write_bits(uint8_t &chunk, uint8_t new_chunk, uint8_t i_min, uint8_t i_max)
{
    clear_bits(chunk, i_min, i_max);
    chunk |= (new_chunk << (7 - i_max));
}


// Constructor
motorDrive::motorDrive(TMCstep &new_stepper, int32_t steps_per_mm_new)
{
    stepper = new_stepper;
    set_steps_per_mm(steps_per_mm_new);
    zero();
}

// Public - Update stepper default velocity in mm/s if input is not NOVALUE
void motorDrive::set_steps_per_mm(int32_t steps_per_mm_new)
{
    steps_per_mm = steps_per_mm_new;
    step_size_mm = 1 / steps_per_mm;
}

// Public - Update stepper default velocity in mm/s if input is not NOVALUE
void motorDrive::set_default_vel_mmps(float max_vel_new)
{
    if (max_vel_new != NOVALUE)
        max_vel = max_vel_new;
}

// Public - Update stepper default acceleration in mm/s^2 if input is not NOVALUE
void motorDrive::set_default_acc_mmps2(float max_accel_new)
{
    if (max_accel_new != NOVALUE)
        max_accel = max_accel_new;
}

// Public - Overwrite the current position to be any mm value designated
void motorDrive::set_current_pos_mm(double target)
{
    target = target == NOVALUE ? 0 : target;
    double working_count = target * steps_per_mm;
    working_count += 0.4999; // For the rounding
    int32_t current_step_count = (int32_t) working_count;
    stepper.set_step(current_step_count);
}

// Public - Set softstop limit for linear drive
void motorDrive::set_move_limits_mm(float minlim, float maxlim)
{
    min_dist_mm = minlim;
    max_dist_mm = maxlim;
}

// Public - Update target position in mm. Respects present joint momentum.
void motorDrive::set_pos_target_mm_async(double target, float feedrate)
{
    target_mm = check_target(target);
    next_step_us = micros();
    if (feedrate != NOVALUE)
        max_vel = feedrate;
}

// Public - Plan and execute acceleration profile Will stay in loop until move is complete
void motorDrive::set_pos_target_mm_sync(double target, float feedrate, bool ignore_limits)
{
    plan_move(target, feedrate, NOVALUE, ignore_limits);
    execute_move_async();
    while (true)
    {
        if(async_move_step_check(micros()))
            return;
    }
}

// Public - Creates and stores move plan to later be executed asynchronously
void motorDrive::plan_move(double target, float feedrate, float accel, bool ignore_limits)
{
    float maxvel = feedrate == NOVALUE ? max_vel : feedrate / 60;
    float maxaccel = accel == NOVALUE ? max_accel : accel;
    double clean_target = ignore_limits ? target : check_target(target);
    int32_t step_target = clean_target * steps_per_mm;
    // Set stepper turn direction
    if((step_target - stepper.get_step()) < 0)
        stepper.set_dir(false);
    else
        stepper.set_dir(true);

    // Calculate some motion paramters
    plan_nsteps = abs(step_target - stepper.get_step());
    float accel_dist = (pow(maxvel, 2) / (2 * maxaccel));
    float move_dist = abs(clean_target - (stepper.get_step() / steps_per_mm));

    // Determine if move will be all accelerations or if it will have plateau
    if(abs(2 * accel_dist) < move_dist)
    {
        plan_tmin = (1000000 * step_size_mm) / maxvel;
        plan_asteps = steps_per_mm * accel_dist;
    }
    else
    {
        plan_tmin = 0;
        plan_asteps = floor(plan_nsteps / 2);
    }

    // Construct acceleration ramp timings
    plan_accel_timings.clear();
    int32_t next_target_delta_us = 0;
    int32_t totaltime_us = 0;
    float v_now = 0;
    for(int i = 0; i < plan_asteps; i++)
    {
        next_target_delta_us = solve_for_t_us(v_now, maxaccel, step_size_mm);
        plan_accel_timings.push_back(next_target_delta_us);
        totaltime_us += next_target_delta_us;
        v_now = (totaltime_us * maxaccel) / 1000000;
    }
}

// Public - Start asynchronous move, requires motion plan and calling async_move_step_check frequently
void motorDrive::execute_move_async()
{
    if (plan_nsteps > 0)
    {
        plan_stepstaken = 1;
        stopped = false;
        plan_nextstep_us = micros() + plan_accel_timings[0];
        stepper.step();
    }
}

// Public - Take a step if ready. Call this in a loop until it returns true
bool motorDrive::async_move_step_check(uint32_t t_now, bool stall_check)
{
    if (stopped) return true;
    if ((plan_stepstaken >= plan_nsteps) && !stopped)
    {
        plan_nsteps = 0;
        plan_stepstaken = 0;
        plan_accel_timings.clear();
        stopped = true;
        return true;
    }
    else if (t_now > plan_nextstep_us)
    {
        // Accelerating
        if (plan_stepstaken < plan_asteps)
        {
            plan_nextstep_us += plan_accel_timings[plan_stepstaken];
        }
        // Decelerating
        else if (plan_stepstaken >= (plan_nsteps - plan_asteps))
        {
            plan_nextstep_us += plan_accel_timings[plan_nsteps - plan_stepstaken - 1];
        }
        // Constant vel no plateau
        else if (plan_tmin == 0)
        {
            plan_nextstep_us += plan_accel_timings[plan_asteps - 1];
        }
        // Constant vel with plateau
        else
        {
            plan_nextstep_us += plan_tmin; // TODO keep track of tmin error for more accurate velocities
        } 
        stepper.step();
        plan_stepstaken++;

        // If stall_check is requested and motor has moved for long enough, check for stall
        if (stall_check && (plan_stepstaken % 16 == 0) && (plan_stepstaken > 640))
        {
            stepper.update_motor_status();
            if(stepper.get_stallguard())
            {
                plan_nsteps = 0;
                plan_stepstaken = 0;
                return true;
            }
        }
    }
    return false;
}

// Private - Kinematic equation solving for time
int32_t motorDrive::solve_for_t_us(float v, float a, float d)
{
    float t = -v;
    if(v >= 0)
    {
        t += sqrt(pow(v, 2) + 2 * a * d);
        t /= a;
    }
    else
    {
        t -= sqrt(pow(v, 2) - 2 * a * d);
        t /= a;
    }
    t *= 1000000;
    t = (int32_t)(t - 1); // Subtract 1 to account for the time of the step pulse
    if(t > 50000)
    {
        return 50000;
    }
    return t;
}

// Public - The magic sauce. Tracks motor motion and calculates if a step is needed now to stay on track.
bool motorDrive::step_if_needed(uint32_t t_now)
{
    int32_t step_target = (steps_per_mm * target_mm);

    // Check if motor is in right place already
    if((abs(current_velocity) < 0.001) && (step_target == stepper.get_step()))
        return false;
    else if((abs(current_velocity) < 0.001) && (stepper.get_step() > step_target))
        stepper.set_dir(false);
    else if((abs(current_velocity) < 0.001) && (stepper.get_step() < step_target))
        stepper.set_dir(true);
    
    if(micros() > next_step_us)
    {
        stepper.step();

        uint32_t cur_step_us = next_step_us;
        double stop_dist_mm = pow(current_velocity, 2) / (max_accel);
        double stop_pos_mm = stepper.get_step() / steps_per_mm;

        // This mess determines if we need to slow down
        if((stepper.get_dir() && ((stop_pos_mm + stop_dist_mm) > target_mm)) || (!stepper.get_dir() && ((stop_pos_mm - stop_dist_mm) < target_mm)))
        {
            // First check if we are coming to a stop
            if((pow(current_velocity, 2) - 2 * max_accel * step_size_mm) < 0)
            {
                // See if we should turn round
                if(stepper.get_step() > step_target)
                {
                next_step_us = (2 * next_step_us) - last_step_us;
                stepper.set_dir(false);
                }
                else if(stepper.get_step() < step_target)
                {
                next_step_us = (2 * next_step_us) - last_step_us;
                stepper.set_dir(true);
                }
                else
                {
                next_step_us = 4294967294;
                diff_exact_us = 4294967294;
                }
            }

            // Otherwise just decelerate normally
            else
            {
                double t0, t1;
                quad_solve(t0, t1, -max_accel, current_velocity, step_size_mm);
                double next_step_temp = 1000000 * min(t0, t1);
                diff_exact_us = next_step_temp;
                next_step_us = (uint32_t) (next_step_temp + 0.5);
                next_step_us += cur_step_us;
            }
        }

        // Otherwise check if we can speed up
        else if(abs(current_velocity) < max_vel)
        {
            // Quadratic has 2 roots, only use one that results in positive time
            double t0, t1;
            quad_solve(t0, t1, max_accel, current_velocity, step_size_mm);
            double next_step_temp = 1000000 * max(t0, t1);
            diff_exact_us = next_step_temp;
            next_step_us = (uint32_t) (next_step_temp + 0.5);
            next_step_us += cur_step_us;
        }

        // Last resort is maintain max speed
        else
        {
            next_step_us += 1000000 * step_size_mm / max_vel;
        }

        // Update current motor velocity
        current_velocity = step_size_mm * 1000000;
        current_velocity /= diff_exact_us;
        current_velocity = stepper.get_dir() ? current_velocity : -current_velocity;
        current_velocity = abs(current_velocity) < 0.01 ? 0 : current_velocity;
        last_step_us = cur_step_us;
    }
    return true;
}

// Public - Return current position in mm
double motorDrive::get_current_pos_mm()
{
    return stepper.get_step() / steps_per_mm;
}

// Public - Return current velocity in mm/sec
double motorDrive::get_current_vel_mmps()
{
    return current_velocity;
}

// Public - Sensorless homing
void motorDrive::home(bool to_min)
{
    if (to_min)
        plan_move(-300, home_vel * 60, 5000, true);
    else
        plan_move(300, home_vel * 60, 5000, true);

    execute_move_async();
    while (true)
    {
        if(async_move_step_check(micros(), true))
            break;
    }

    if (to_min)
    {
        stepper.set_step(0);
        set_pos_target_mm_sync(home_backoff_mm);
        stepper.set_step(0);
    }
    else
    {
        stepper.set_step(steps_per_mm * max_dist_mm);
        set_pos_target_mm_sync(steps_per_mm * (max_dist_mm - home_backoff_mm));
        stepper.set_step(steps_per_mm * max_dist_mm);
    }
}

// Public - Prints current stallguard value
uint32_t motorDrive::get_stall_value()
{
    stepper.update_motor_status();
    return stepper.get_stall_value();
}

// Private - Quadratic equation yo
void motorDrive::quad_solve(double &t_0, double &t_1, double a, double b, double c)
{
    double temp0 = -abs(b);
    double temp1 = sqrt(pow(b, 2) + 2 * a * c);
    t_0 = (temp0 + temp1) / a;
    t_1 = (temp0 - temp1) / a;
}

// Private - Filter move target to ensure valid and not below min (0) or above max(max_dist_mm)
double motorDrive::check_target(double target)
{
    target = target == NOVALUE ? target_mm : target;
    target = target < min_dist_mm ? min_dist_mm : target;
    target = target > max_dist_mm ? max_dist_mm : target;
    return target;
}

// Public - Set current position to zero
void motorDrive::zero()
{
    set_current_pos_mm(0);
}

void motorDrive::enable()
{
    stepper.enable();
}

void motorDrive::disable()
{
    stepper.disable();
}
