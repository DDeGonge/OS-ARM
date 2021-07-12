============================
I2C Slave synchronous driver
============================

I2C (Inter-Integrated Circuit) is a two wire serial interface commonly used
for on-board low-speed bi-directional communication between controllers and
peripherals. There can be several slave devices on I2C bus.
I2C uses only two bidirectional open-drain lines, usually designated SDA
(Serial Data Line) and SCL (Serial Clock Line), with pull up resistors.

I2C Modes (standard mode/fastmode+/highspeed mode) can only be selected in
Atmel Start.

Features
--------

	* I2C Slave support
	* Initialization and de-initialization
	* Enabling and disabling
	* 10- and 7- bit addressing
	* Setting slave address
	* I2C Modes supported
	       +----------------------+-------------------+
	       |* Standard/Fast mode  | (SCL: 1 - 100kHz) |
	       +----------------------+-------------------+
	       |* Fastmode+           | (SCL: 1 - 1000kHz)|
	       +----------------------+-------------------+
	       |* Highspeed mode      | (SCL: 1 - 3400kHz)|
	       +----------------------+-------------------+

Applications
------------

* Sensors, device to device communication, memory.

Dependencies
------------

* I2C Slave capable hardware

Concurrency
-----------

N/A

Limitations
-----------

* System Management Bus (SMBus) not supported.
* Power Management Bus (PMBus) not supported.

Known issues and workarounds
----------------------------

N/A


