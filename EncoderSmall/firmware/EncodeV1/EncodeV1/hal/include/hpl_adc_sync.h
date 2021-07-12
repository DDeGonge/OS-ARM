/**
 * \file
 *
 * \brief ADC related functionality declaration.
 *
 * Copyright (c) 2014-2018 Microchip Technology Inc. and its subsidiaries.
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

#ifndef _HPL_ADC_SYNC_H_INCLUDED
#define _HPL_ADC_SYNC_H_INCLUDED

/**
 * \addtogroup HPL ADC
 *
 * \section hpl_adc_sync_rev Revision History
 * - v1.0.0 Initial Release
 *
 *@{
 */

#include "compiler.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief ADC reference source
 */
typedef uint8_t adc_reference_t;

/**
 * \brief ADC resolution
 */
typedef uint8_t adc_resolution_t;

/**
 * \brief ADC positive input for channel
 */
typedef uint8_t adc_pos_input_t;

/**
 * \brief ADC negative input for channel
 */
typedef uint8_t adc_neg_input_t;

/**
 * \brief ADC threshold
 */
typedef uint16_t adc_threshold_t;

/**
 * \brief ADC gain
 */
typedef uint8_t adc_gain_t;

/**
 * \brief ADC conversion mode
 */
enum adc_conversion_mode { ADC_CONVERSION_MODE_SINGLE_CONVERSION = 0, ADC_CONVERSION_MODE_FREERUN };

/**
 * \brief ADC differential mode
 */
enum adc_differential_mode { ADC_DIFFERENTIAL_MODE_SINGLE_ENDED = 0, ADC_DIFFERENTIAL_MODE_DIFFERENTIAL };

/**
 * \brief ADC window mode
 */
typedef uint8_t adc_window_mode_t;

/**
 * \brief ADC threshold status
 */
typedef bool adc_threshold_status_t;

/**
 * \brief ADC sync descriptor device structure
 */
struct _adc_sync_device {
	void *hw;
};

/**
 * \name HPL functions
 */
//@{
/**
 * \brief Initialize synchronous ADC
 *
 * This function does low level ADC configuration.
 *
 * param[in] device The pointer to ADC device instance
 * param[in] hw     The pointer to hardware instance
 *
 * \return Initialization status
 */
int32_t _adc_sync_init(struct _adc_sync_device *const device, void *const hw);

/**
 * \brief Deinitialize ADC
 *
 * \param[in] device The pointer to ADC device instance
 */
void _adc_sync_deinit(struct _adc_sync_device *const device);

/**
 * \brief Enable ADC
 *
 * \param[in] device   The pointer to ADC device instance
 * \param[in] channel  Channel number
 */
void _adc_sync_enable_channel(struct _adc_sync_device *const device, const uint8_t channel);

/**
 * \brief Disable ADC
 *
 * \param[in] device   The pointer to ADC device instance
 * \param[in] channel  Channel number
 */
void _adc_sync_disable_channel(struct _adc_sync_device *const device, const uint8_t channel);

/**
 * \brief Retrieve ADC conversion data size
 *
 * \param[in] device The pointer to ADC device instance
 *
 * \return The data size in bytes
 */
uint8_t _adc_sync_get_data_size(const struct _adc_sync_device *const device);

/**
 * \brief Check if conversion is done
 *
 * \param[in] device   The pointer to ADC device instance
 * \param[in] channel  Channel number
 *
 * \return The status of conversion
 * \retval true The conversion is done
 * \retval false The conversion is not done
 */
bool _adc_sync_is_channel_conversion_done(const struct _adc_sync_device *const device, const uint8_t channel);

/**
 * \brief Make conversion
 *
 * \param[in] device The pointer to ADC device instance
 */
void _adc_sync_convert(struct _adc_sync_device *const device);

/**
 * \brief Retrieve the conversion result
 *
 * \param[in] device   The pointer to ADC device instance
 * \param[in] channel  Channel number
 *
 * \return The result value of channel
 */
uint16_t _adc_sync_read_channel_data(const struct _adc_sync_device *const device, const uint8_t channel);

/**
 * \brief Set reference source
 *
 * \param[in] device The pointer to ADC device instance
 * \param[in] reference A reference source to set
 */
void _adc_sync_set_reference_source(struct _adc_sync_device *const device, const adc_reference_t reference);

/**
 * \brief Set resolution
 *
 * \param[in] device The pointer to ADC device instance
 * \param[in] resolution A resolution to set
 */
void _adc_sync_set_resolution(struct _adc_sync_device *const device, const adc_resolution_t resolution);

/**
 * \brief Set ADC input source of a channel
 *
 * \param[in] device    The pointer to ADC device instance
 * \param[in] pos_input A positive input source to set
 * \param[in] neg_input A negative input source to set
 * \param[in] channel   Channel number
 */
void _adc_sync_set_inputs(struct _adc_sync_device *const device, const adc_pos_input_t pos_input,
                          const adc_neg_input_t neg_input, const uint8_t channel);

/**
 * \brief Set conversion mode
 *
 * \param[in] device The pointer to ADC device instance
 * \param[in] mode A conversion mode to set
 */
void _adc_sync_set_conversion_mode(struct _adc_sync_device *const device, const enum adc_conversion_mode mode);

/**
 * \brief Set differential mode
 *
 * \param[in] device  The pointer to ADC device instance
 * \param[in] channel Channel number
 * \param[in] mode    A differential mode to set
 */
void _adc_sync_set_channel_differential_mode(struct _adc_sync_device *const device, const uint8_t channel,
                                             const enum adc_differential_mode mode);

/**
 * \brief Set gain
 *
 * \param[in] device   The pointer to ADC device instance
 * \param[in] channel  Channel number
 * \param[in] gain     A gain to set
 */
void _adc_sync_set_channel_gain(struct _adc_sync_device *const device, const uint8_t channel, const adc_gain_t gain);

/**
 * \brief Set window mode
 *
 * \param[in] device The pointer to ADC device instance
 * \param[in] mode   A mode to set
 */
void _adc_sync_set_window_mode(struct _adc_sync_device *const device, const adc_window_mode_t mode);

/**
 * \brief Set threshold
 *
 * \param[in] device        The pointer to ADC device instance
 * \param[in] low_threshold  A lower threshold to set
 * \param[in] up_threshold   An upper thresholds to set
 */
void _adc_sync_set_thresholds(struct _adc_sync_device *const device, const adc_threshold_t low_threshold,
                              const adc_threshold_t up_threshold);

/**
 * \brief Retrieve threshold state
 *
 * \param[in] device The pointer to ADC device instance
 * \param[out] state The threshold state
 */
void _adc_sync_get_threshold_state(const struct _adc_sync_device *const device, adc_threshold_status_t *const state);
//@}

#ifdef __cplusplus
}
#endif
/**@}*/
#endif /* _HPL_ADC_SYNC_H_INCLUDED */
