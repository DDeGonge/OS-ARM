#include <atmel_start.h>

int main(void)
{
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();
	
	/* I2C Initialization */
	struct io_descriptor *io;
	i2c_s_sync_get_io_descriptor(&I2C_0, &io);
	i2c_s_sync_set_addr(&I2C_0, 1);
	i2c_s_sync_enable(&I2C_0);
	
	/* ADC init */

	/* Replace with your application code */
	uint8_t c;
	while (1) {
		c = read_i2c(io);
	}
}


uint8_t read_i2c(struct io_descriptor io)
{
	uint8_t c;
	io_read(io, &c, 1);
	return c;
}
