/* Auto-generated config file hpl_sercom_config.h */
#ifndef HPL_SERCOM_CONFIG_H
#define HPL_SERCOM_CONFIG_H

// <<< Use Configuration Wizard in Context Menu >>>

#include <peripheral_clk_config.h>

#ifndef SERCOM_I2CM_CTRLA_MODE_I2C_SLAVE
#define SERCOM_I2CM_CTRLA_MODE_I2C_SLAVE (4 << 2)
#endif

#ifndef CONF_SERCOM_0_I2CS_ENABLE
#define CONF_SERCOM_0_I2CS_ENABLE 1
#endif

// <h> Basic Configuration

// <o> Address <0x0-0x3FF>
// <i> In 7-bit addressing mode the range is 0x00 to 0x7F, in 10-bit addressing mode the range is 0x000 to 0x3FF
// <id> i2c_slave_address
#ifndef CONF_SERCOM_0_I2CS_ADDRESS
#define CONF_SERCOM_0_I2CS_ADDRESS 0xa5
#endif

// </h>

// <e> Advanced Configuration
// <id> i2c_slave_advanced
#ifndef CONF_SERCOM_0_I2CS_ADVANCED_CONFIG
#define CONF_SERCOM_0_I2CS_ADVANCED_CONFIG 0
#endif

// <q> Run in stand-by
// <i> Determine if the module shall run in standby sleep mode
// <id> i2c_slave_runstdby
#ifndef CONF_SERCOM_0_I2CS_RUNSTDBY
#define CONF_SERCOM_0_I2CS_RUNSTDBY 0
#endif

// <o> SDA Hold Time (SDAHOLD)
// <0=>Disabled
// <1=>50-100ns hold time
// <2=>300-600ns hold time
// <3=>400-800ns hold time
// <i> Defines the SDA hold time with respect to the negative edge of SCL
// <id> i2c_slave_sdahold
#ifndef CONF_SERCOM_0_I2CS_SDAHOLD
#define CONF_SERCOM_0_I2CS_SDAHOLD 0x2
#endif

// <q> Slave SCL Low Extend Time-Out (SEXTTOEN)
// <i> Enables the slave SCL low extend time-out. If SCL is cumulatively held low for greater than 25ms from the initial START to a STOP, the slave will release its clock hold if enabled and reset the internal state machine
// <id> i2c_slave_sexttoen
#ifndef CONF_SERCOM_0_I2CS_SEXTTOEN
#define CONF_SERCOM_0_I2CS_SEXTTOEN 0
#endif

// <q> SCL Low Time-Out (LOWTOUT)
// <i> Enables SCL low time-out. If SCL is held low for 25ms-35ms, the master will release it's clock hold
// <id> i2c_slave_lowtout
#ifndef CONF_SERCOM_0_I2CS_LOWTOUT
#define CONF_SERCOM_0_I2CS_LOWTOUT 0
#endif

// <q> SCL Clock Stretch Mode (SCLSM)
// <i> Enables SCL stretching.
// <id> i2c_slave_sclsm
#ifndef CONF_SERCOM_0_I2CS_SCLSM
#define CONF_SERCOM_0_I2CS_SCLSM 0
#endif

// <q> General call addressing (GENCEN)
// <i> Enables general call addressing
// <id> i2c_slave_gencen
#ifndef CONF_SERCOM_0_I2CS_GENCEN
#define CONF_SERCOM_0_I2CS_GENCEN 0
#endif

// <o> Address mode (AMODE)
// <0=>Mask
// <1=>Two addresses
// <2=>Range
// <i> Defines the address mode of a slave device
// <id> i2c_slave_amode
#ifndef CONF_SERCOM_0_I2CS_AMODE
#define CONF_SERCOM_0_I2CS_AMODE 0x0
#endif

// <q> Ten bit addressing (TENBITEN)
// <i> Enables 10 bit addressing addressing
// <id> i2c_slave_tenbiten
#ifndef CONF_SERCOM_0_I2CS_TENBITEN
#define CONF_SERCOM_0_I2CS_TENBITEN 0
#endif

// <o> Address mask<0x0-0x3FF>
// <i> This mask acts as second address match register, an address mask register or the lower limit of an address range
// <i> When acts as mask, bit value 1 means that the corresponding address bit is ignored
// <id> i2c_slave_address_mask
#ifndef CONF_SERCOM_0_I2CS_ADDRESS_MASK
#define CONF_SERCOM_0_I2CS_ADDRESS_MASK 0x0
#endif
// </e>

#ifndef CONF_SERCOM_0_I2CS_SPEED
#define CONF_SERCOM_0_I2CS_SPEED 0x02 // Speed: High speed mode
#endif

// <<< end of configuration section >>>

#endif // HPL_SERCOM_CONFIG_H
