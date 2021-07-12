/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */

#include "driver_init.h"
#include <peripheral_clk_config.h>
#include <utils.h>
#include <hal_init.h>
#include <hpl_gclk_base.h>
#include <hpl_pm_base.h>

#include <hpl_adc_base.h>

struct adc_sync_descriptor ADC_0;

struct flash_descriptor FLASH_0;

struct i2c_s_sync_descriptor I2C_0;

void ADC_0_PORT_init(void)
{
}

void ADC_0_CLOCK_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBC, ADC);
	_gclk_enable_channel(ADC_GCLK_ID, CONF_GCLK_ADC_SRC);
}

void ADC_0_init(void)
{
	ADC_0_CLOCK_init();
	ADC_0_PORT_init();
	adc_sync_init(&ADC_0, ADC, (void *)NULL);
}

void FLASH_0_CLOCK_init(void)
{

	_pm_enable_bus_clock(PM_BUS_APBB, NVMCTRL);
}

void FLASH_0_init(void)
{
	FLASH_0_CLOCK_init();
	flash_init(&FLASH_0, NVMCTRL);
}

void I2C_0_PORT_init(void)
{

	gpio_set_pin_pull_mode(SCL,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(SCL, PINMUX_PA08C_SERCOM0_PAD0);

	gpio_set_pin_pull_mode(SDA,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(SDA, PINMUX_PA09C_SERCOM0_PAD1);
}

void I2C_0_CLOCK_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBC, SERCOM0);
	_gclk_enable_channel(SERCOM0_GCLK_ID_CORE, CONF_GCLK_SERCOM0_CORE_SRC);
	_gclk_enable_channel(SERCOM0_GCLK_ID_SLOW, CONF_GCLK_SERCOM0_SLOW_SRC);
}

void I2C_0_init(void)
{
	I2C_0_CLOCK_init();
	i2c_s_sync_init(&I2C_0, SERCOM0);
	I2C_0_PORT_init();
}

void system_init(void)
{
	init_mcu();

	// GPIO on PA04

	// Disable digital pin circuitry
	gpio_set_pin_direction(OP1, GPIO_DIRECTION_OFF);

	gpio_set_pin_function(OP1, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA05

	// Disable digital pin circuitry
	gpio_set_pin_direction(OP2, GPIO_DIRECTION_OFF);

	gpio_set_pin_function(OP2, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA06

	// Disable digital pin circuitry
	gpio_set_pin_direction(ON2, GPIO_DIRECTION_OFF);

	gpio_set_pin_function(ON2, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA07

	// Disable digital pin circuitry
	gpio_set_pin_direction(ON1, GPIO_DIRECTION_OFF);

	gpio_set_pin_function(ON1, GPIO_PIN_FUNCTION_OFF);

	ADC_0_init();

	FLASH_0_init();

	I2C_0_init();
}
