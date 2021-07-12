/**
 * \file
 *
 * \brief Sync I2C Slave Hardware Abstraction Layer(HAL) declaration.
 *
 * Copyright (c) 2015-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */

#ifndef _HAL_I2C_S_SYNC_H_INCLUDED
#define _HAL_I2C_S_SYNC_H_INCLUDED

#include <hal_io.h>
#include <hpl_i2c_s_sync.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \addtogroup doc_driver_hal_i2c_slave_sync
 *
 * @{
 */

/**
 * \brief I2C slave descriptor structure
 */
struct i2c_s_sync_descriptor {
	struct _i2c_s_sync_device device;
	struct io_descriptor      io;
};

/**
 * \brief Initialize synchronous I2C slave interface
 *
 * This function initializes the given I2C descriptor to be used as synchronous
 * I2C slave interface descriptor. It checks if the given hardware is not
 * initialized and if it is permitted to be initialized.
 *
 * \param[in] descr An I2C slave descriptor which is used to communicate through
 *                I2C
 * \param[in] hw The pointer to hardware instance
 *
 * \return Initialization status.
 */
int32_t i2c_s_sync_init(struct i2c_s_sync_descriptor *const descr, void *hw);

/**
 * \brief Deinitialize synchronous I2C slave interface
 *
 * This function deinitializes the given synchronous I2C slave descriptor.
 * It checks if the given hardware is initialized and if it is
 * permitted to be deinitialized.
 *
 * \param[in] descr An I2C slave descriptor which is used to communicate through
 *                I2C
 *
 * \return De-initialization status.
 */
int32_t i2c_s_sync_deinit(struct i2c_s_sync_descriptor *const descr);

/**
 * \brief Set the device address
 *
 * This function sets the I2C slave device address.
 *
 * \param[in] descr An I2C slave descriptor which is used to communicate  through
 *                I2C
 * \param[in] address An address
 *
 * \return Status of the address setting.
 */
int32_t i2c_s_sync_set_addr(struct i2c_s_sync_descriptor *const descr, const uint16_t address);

/**
 * \brief Enable I2C slave communication
 *
 * This function enables the I2C slave device
 *
 * \param[in] descr An I2C slave descriptor which is used to communicate through
 *                I2C
 *
 * \return Enabling status.
 */
int32_t i2c_s_sync_enable(struct i2c_s_sync_descriptor *const descr);

/**
 * \brief Disable I2C slave communication
 *
 * This function disables the I2C slave device
 *
 * \param[in] descr An I2C slave descriptor which is used to communicate through
 *                I2C
 *
 * \return Disabling status.
 */
int32_t i2c_s_sync_disable(struct i2c_s_sync_descriptor *const descr);

/**
 * \brief Retrieve I/O descriptor
 *
 * This function returns a I/O instance for the given I2C slave driver instance
 *
 * \param[in] descr An I2C slave descriptor which is used to communicate through
 *                I2C
 * \param[in] io A pointer to an I/O descriptor pointer type
 *
 * \return I/O retrieving status.
 */
int32_t i2c_s_sync_get_io_descriptor(struct i2c_s_sync_descriptor *const descr, struct io_descriptor **io);

/**
 * \brief Retrieve the current interface status
 *
 * \param[in]  descr An i2c descriptor which is used to communicate via USART
 * \param[out] status The state of I2C slave
 *
 * \return The status of I2C status retrieving.
 */
int32_t i2c_s_sync_get_status(const struct i2c_s_sync_descriptor *const descr, i2c_s_status_t *const status);

/**
 * \brief Retrieve the current driver version
 *
 * \return Current driver version.
 */
uint32_t i2c_s_sync_get_version(void);

/**@}*/

#ifdef __cplusplus
}
#endif

#endif /* _HAL_I2C_S_SYNC_H_INCLUDED */
