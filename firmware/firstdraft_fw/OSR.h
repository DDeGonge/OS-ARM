#include <Arduino.h>
#include <SPI.h>
#include <vector>
#include <iostream>

#define NOVALUE 999999


/* Stepper driver object, to be used with up to 2 stepper objects */
struct TMC2041
{
    TMC2041() {};
    TMC2041(uint8_t en_pin, uint8_t cs_pin);
    TMC2041(uint8_t en_pin, uint8_t cs_pin, bool single_motor_mode);

    public:
    void set_pins(uint8_t en_pin, uint8_t cs_pin);
    void configure();
    void enable_driver();
    void disable_driver();

    /* Driver com functions */
    void write_gconf();
    uint8_t get_ihold_address(uint8_t index);
    uint8_t get_chop_address(uint8_t index);
    uint8_t get_cool_address(uint8_t index);
    uint8_t get_status_address(uint8_t index);
    void write_cmd(uint8_t addr, uint8_t chunk[4]);
    int32_t read_cmd(uint8_t addr);

    private:
    uint8_t EN_PIN;
    uint8_t CS_PIN;

    /* SPI functions */
    SPISettings TMCspiSettings = SPISettings(1000000, MSBFIRST, SPI_MODE3); 
    
    /* Addresses. Tuple if unique address for each stepper */
    uint8_t a_gconf = 0x00;
    uint8_t a_iholdirun[2] = {0x30, 0x50};
    uint8_t a_chop[2] = {0x6C, 0x7C};
    uint8_t a_cool[2] = {0x6D, 0x7D};
    uint8_t a_status[2] = {0x6F, 0x7F};

    /* Driver level bitfields */
    uint8_t gconf[4] = {B00000000, B00000000, B00000000, B00000110};
};

/* Stepper driver object */
struct TMC5160
{
    TMC5160() {};
    TMC5160(uint8_t en_pin, uint8_t cs_pin);

    public:
    void set_pins(uint8_t en_pin, uint8_t cs_pin);
    void configure();
    void enable_driver();
    void disable_driver();

    /* Driver com functions */
    void write_gconf();
    uint8_t get_ihold_address();
    uint8_t get_chop_address();
    uint8_t get_cool_address();
    uint8_t get_status_address();
    uint8_t get_gstat_address();
    uint8_t get_tpowerdown_address();
    uint8_t get_tpwmthrs_address();
    void write_cmd(uint8_t addr, uint8_t chunk[4]);
    int32_t read_cmd(uint8_t addr);

    private:
    uint8_t EN_PIN;
    uint8_t CS_PIN;

    /* SPI functions */
    SPISettings TMCspiSettings = SPISettings(1000000, MSBFIRST, SPI_MODE3); 
    
    /* SPI Addresses. Tuple if unique address for each stepper */
    uint8_t a_gconf = 0x00;
    uint8_t a_gstat = 0x01;
    uint8_t a_ioin = 0x04;
    uint8_t a_sconf = 0x09;
    uint8_t a_drvconf = 0x0A;
    uint8_t a_drvcur = 0x0B;
    uint8_t a_iholdirun = 0x10;
    uint8_t a_chop = 0x6C;
    uint8_t a_cool = 0x6D;
    uint8_t a_status = 0x6F;
    uint8_t a_tpowerdown = 0x11;
    uint8_t a_tpwmthrs = 0x13;

    /* SPI default bitfields */
    // uint8_t gconf[4] = {B00000000, B00000000, B00000000, B00001100};
    uint8_t gconf[4] = {B00000000, B00000000, B00000000, B00001000};  // take3
};

/* Stepper object, supports 2041 and 5160 driver types */
struct TMCstep
{
    TMCstep() {};
    TMCstep(uint8_t step_pin, uint8_t dir_pin, TMC2041 &my_driver, uint8_t motor_index, bool reverse = false);
    TMCstep(uint8_t step_pin, uint8_t dir_pin, TMC5160 &my_driver, bool reverse = false);

    public:
    uint8_t drv_type = 0;
    TMC2041 driver_2041;  // Type 1
    TMC5160 driver_5160;  // Type 2

    void set_2041_default_bitfields();
    void set_5160_default_bitfields();

    void set_pins(uint8_t step_pin, uint8_t dir_pin);
    void set_index(uint8_t index);
    uint8_t get_index();

    void step();
    void set_step(int32_t steps);
    int32_t get_step();
    void set_dir(bool dir);
    bool get_dir();

    void enable();
    void disable();

    /* Motor configurating funcs */
    void set_hold_current(uint8_t newval); // Values 0-31 acceptable
    void set_run_current(uint8_t newval);  // Values 0-31 acceptable
    void set_microsteps(uint8_t newval);   // Values 0 (256u) to 8 (fullstep) acceptable

    /* Driver com functions */
    void write_iholdirun();
    void write_chop();
    void write_cool();
    void update_motor_status();
    uint32_t debug_print_status();

    /* Sensorless homing stuff */
    bool get_stallguard();
    uint32_t get_stall_value();

    /* Motor level bitfields */
    // TODO these should probably be private
    uint8_t ihold[4] = {B00000000, B00000001, B00010000, B00001000};
    uint8_t chop[4]  = {B00000010, B00000001, B00000000, B00000011};
    uint8_t cool[4]  = {B00000000, B00000110, B00000000, B00000000};
    uint8_t saved_chop = chop[3];


    private:
    uint8_t INDEX;
    uint8_t STEP_PIN;
    uint8_t DIR_PIN;
    uint16_t step_pulse_len_us = 1;
    bool reversed = false;

    int32_t step_count = 0;
    bool motor_dir = false;

    uint32_t status_bits;

    void clear_bits(uint8_t &chunk, uint8_t i_min, uint8_t i_max);
    void write_bits(uint8_t &chunk, uint8_t new_chunk, uint8_t i_min, uint8_t i_max);
};

/* Stepper motor wrapper to handle motion profiles */
// TODO simplify this and merge with TMCstep
struct motorDrive
{
    motorDrive(TMCstep &new_stepper, int32_t steps_per_mm_new);

    public:
    // Configuration functions
    void set_default_vel_mmps(float max_vel_new);
    void set_default_acc_mmps2(float max_accel_new);
    void set_steps_per_mm(int32_t steps_per_mm_new);

    // Zeroing & Other Functions
    void set_current_pos_mm(double target);
    void zero();
    void enable();
    void disable();
    void set_max_move_dist_mm(float new_lim_mm);
    double get_current_pos_mm();
    double get_current_vel_mmps();

    // Homing and sensing related
    void home(bool to_min = true);
    uint32_t get_stall_value();

    // Async move related supporting real time target adjustment. Limit of ~20KHz step speed
    void set_pos_target_mm_async(double target, float feedrate = NOVALUE);
    bool step_if_needed();
    
    // Standard Synchronous move
    void set_pos_target_mm_sync(double target, float feedrate = NOVALUE, bool ignore_limits = false);
    
    // Pre-planned async move. Runs WAY faster than the realtime version
    void plan_move(double target, float feedrate = NOVALUE, float accel = NOVALUE, bool ignore_limits = false);
    void execute_move_async();
    bool async_move_step_check(uint32_t t_now = micros(), bool stall_check = false);


    private:
    TMCstep stepper;

    // Configuration
    float steps_per_mm = 160;
    float max_vel = 100;
    float home_vel = 30;
    float max_accel = 500;
    float max_dist_mm = 10000;
    float home_backoff_mm = 1;
    float step_size_mm;

    // Backend funcs
    void quad_solve(double &t_0, double &t_1, double a, double b, double c);
    int32_t solve_for_t_us(float v, float a, float d);
    double check_target(double target);

    // Control vars
    double target_mm = 0;

    // Async Motion tracking vars
    double current_velocity = 0;
    double diff_exact_us = 0;
    uint32_t last_step_us = 0;
    uint32_t next_step_us = 0;

    // Sync motion tracking vars
    int8_t plan_sign;
    int32_t plan_tmin;
    uint32_t plan_asteps = 0;;
    uint32_t plan_nsteps = 0;;
    std::vector<uint16_t> plan_accel_timings;

    uint32_t plan_stepstaken = 0;
    uint32_t plan_nextstep_us = 0;
};