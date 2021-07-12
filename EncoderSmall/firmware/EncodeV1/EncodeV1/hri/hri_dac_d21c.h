/**
 * \file
 *
 * \brief SAM DAC
 *
 * Copyright (c) 2017-2018 Microchip Technology Inc. and its subsidiaries.
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

#ifdef _SAMD21_DAC_COMPONENT_
#ifndef _HRI_DAC_D21C_H_INCLUDED_
#define _HRI_DAC_D21C_H_INCLUDED_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <hal_atomic.h>

#if defined(ENABLE_DAC_CRITICAL_SECTIONS)
#define DAC_CRITICAL_SECTION_ENTER() CRITICAL_SECTION_ENTER()
#define DAC_CRITICAL_SECTION_LEAVE() CRITICAL_SECTION_LEAVE()
#else
#define DAC_CRITICAL_SECTION_ENTER()
#define DAC_CRITICAL_SECTION_LEAVE()
#endif

typedef uint16_t hri_dac_data_reg_t;
typedef uint16_t hri_dac_databuf_reg_t;
typedef uint8_t  hri_dac_ctrla_reg_t;
typedef uint8_t  hri_dac_ctrlb_reg_t;
typedef uint8_t  hri_dac_evctrl_reg_t;
typedef uint8_t  hri_dac_intenset_reg_t;
typedef uint8_t  hri_dac_intflag_reg_t;
typedef uint8_t  hri_dac_status_reg_t;

static inline void hri_dac_wait_for_sync(const void *const hw)
{
	while (((const Dac *)hw)->STATUS.bit.SYNCBUSY)
		;
}

static inline bool hri_dac_is_syncing(const void *const hw)
{
	return ((const Dac *)hw)->STATUS.bit.SYNCBUSY;
}

static inline bool hri_dac_get_INTFLAG_UNDERRUN_bit(const void *const hw)
{
	return (((Dac *)hw)->INTFLAG.reg & DAC_INTFLAG_UNDERRUN) >> DAC_INTFLAG_UNDERRUN_Pos;
}

static inline void hri_dac_clear_INTFLAG_UNDERRUN_bit(const void *const hw)
{
	((Dac *)hw)->INTFLAG.reg = DAC_INTFLAG_UNDERRUN;
}

static inline bool hri_dac_get_INTFLAG_EMPTY_bit(const void *const hw)
{
	return (((Dac *)hw)->INTFLAG.reg & DAC_INTFLAG_EMPTY) >> DAC_INTFLAG_EMPTY_Pos;
}

static inline void hri_dac_clear_INTFLAG_EMPTY_bit(const void *const hw)
{
	((Dac *)hw)->INTFLAG.reg = DAC_INTFLAG_EMPTY;
}

static inline bool hri_dac_get_INTFLAG_SYNCRDY_bit(const void *const hw)
{
	return (((Dac *)hw)->INTFLAG.reg & DAC_INTFLAG_SYNCRDY) >> DAC_INTFLAG_SYNCRDY_Pos;
}

static inline void hri_dac_clear_INTFLAG_SYNCRDY_bit(const void *const hw)
{
	((Dac *)hw)->INTFLAG.reg = DAC_INTFLAG_SYNCRDY;
}

static inline bool hri_dac_get_interrupt_UNDERRUN_bit(const void *const hw)
{
	return (((Dac *)hw)->INTFLAG.reg & DAC_INTFLAG_UNDERRUN) >> DAC_INTFLAG_UNDERRUN_Pos;
}

static inline void hri_dac_clear_interrupt_UNDERRUN_bit(const void *const hw)
{
	((Dac *)hw)->INTFLAG.reg = DAC_INTFLAG_UNDERRUN;
}

static inline bool hri_dac_get_interrupt_EMPTY_bit(const void *const hw)
{
	return (((Dac *)hw)->INTFLAG.reg & DAC_INTFLAG_EMPTY) >> DAC_INTFLAG_EMPTY_Pos;
}

static inline void hri_dac_clear_interrupt_EMPTY_bit(const void *const hw)
{
	((Dac *)hw)->INTFLAG.reg = DAC_INTFLAG_EMPTY;
}

static inline bool hri_dac_get_interrupt_SYNCRDY_bit(const void *const hw)
{
	return (((Dac *)hw)->INTFLAG.reg & DAC_INTFLAG_SYNCRDY) >> DAC_INTFLAG_SYNCRDY_Pos;
}

static inline void hri_dac_clear_interrupt_SYNCRDY_bit(const void *const hw)
{
	((Dac *)hw)->INTFLAG.reg = DAC_INTFLAG_SYNCRDY;
}

static inline hri_dac_intflag_reg_t hri_dac_get_INTFLAG_reg(const void *const hw, hri_dac_intflag_reg_t mask)
{
	uint8_t tmp;
	tmp = ((Dac *)hw)->INTFLAG.reg;
	tmp &= mask;
	return tmp;
}

static inline hri_dac_intflag_reg_t hri_dac_read_INTFLAG_reg(const void *const hw)
{
	return ((Dac *)hw)->INTFLAG.reg;
}

static inline void hri_dac_clear_INTFLAG_reg(const void *const hw, hri_dac_intflag_reg_t mask)
{
	((Dac *)hw)->INTFLAG.reg = mask;
}

static inline void hri_dac_set_INTEN_UNDERRUN_bit(const void *const hw)
{
	((Dac *)hw)->INTENSET.reg = DAC_INTENSET_UNDERRUN;
}

static inline bool hri_dac_get_INTEN_UNDERRUN_bit(const void *const hw)
{
	return (((Dac *)hw)->INTENSET.reg & DAC_INTENSET_UNDERRUN) >> DAC_INTENSET_UNDERRUN_Pos;
}

static inline void hri_dac_write_INTEN_UNDERRUN_bit(const void *const hw, bool value)
{
	if (value == 0x0) {
		((Dac *)hw)->INTENCLR.reg = DAC_INTENSET_UNDERRUN;
	} else {
		((Dac *)hw)->INTENSET.reg = DAC_INTENSET_UNDERRUN;
	}
}

static inline void hri_dac_clear_INTEN_UNDERRUN_bit(const void *const hw)
{
	((Dac *)hw)->INTENCLR.reg = DAC_INTENSET_UNDERRUN;
}

static inline void hri_dac_set_INTEN_EMPTY_bit(const void *const hw)
{
	((Dac *)hw)->INTENSET.reg = DAC_INTENSET_EMPTY;
}

static inline bool hri_dac_get_INTEN_EMPTY_bit(const void *const hw)
{
	return (((Dac *)hw)->INTENSET.reg & DAC_INTENSET_EMPTY) >> DAC_INTENSET_EMPTY_Pos;
}

static inline void hri_dac_write_INTEN_EMPTY_bit(const void *const hw, bool value)
{
	if (value == 0x0) {
		((Dac *)hw)->INTENCLR.reg = DAC_INTENSET_EMPTY;
	} else {
		((Dac *)hw)->INTENSET.reg = DAC_INTENSET_EMPTY;
	}
}

static inline void hri_dac_clear_INTEN_EMPTY_bit(const void *const hw)
{
	((Dac *)hw)->INTENCLR.reg = DAC_INTENSET_EMPTY;
}

static inline void hri_dac_set_INTEN_SYNCRDY_bit(const void *const hw)
{
	((Dac *)hw)->INTENSET.reg = DAC_INTENSET_SYNCRDY;
}

static inline bool hri_dac_get_INTEN_SYNCRDY_bit(const void *const hw)
{
	return (((Dac *)hw)->INTENSET.reg & DAC_INTENSET_SYNCRDY) >> DAC_INTENSET_SYNCRDY_Pos;
}

static inline void hri_dac_write_INTEN_SYNCRDY_bit(const void *const hw, bool value)
{
	if (value == 0x0) {
		((Dac *)hw)->INTENCLR.reg = DAC_INTENSET_SYNCRDY;
	} else {
		((Dac *)hw)->INTENSET.reg = DAC_INTENSET_SYNCRDY;
	}
}

static inline void hri_dac_clear_INTEN_SYNCRDY_bit(const void *const hw)
{
	((Dac *)hw)->INTENCLR.reg = DAC_INTENSET_SYNCRDY;
}

static inline void hri_dac_set_INTEN_reg(const void *const hw, hri_dac_intenset_reg_t mask)
{
	((Dac *)hw)->INTENSET.reg = mask;
}

static inline hri_dac_intenset_reg_t hri_dac_get_INTEN_reg(const void *const hw, hri_dac_intenset_reg_t mask)
{
	uint8_t tmp;
	tmp = ((Dac *)hw)->INTENSET.reg;
	tmp &= mask;
	return tmp;
}

static inline hri_dac_intenset_reg_t hri_dac_read_INTEN_reg(const void *const hw)
{
	return ((Dac *)hw)->INTENSET.reg;
}

static inline void hri_dac_write_INTEN_reg(const void *const hw, hri_dac_intenset_reg_t data)
{
	((Dac *)hw)->INTENSET.reg = data;
	((Dac *)hw)->INTENCLR.reg = ~data;
}

static inline void hri_dac_clear_INTEN_reg(const void *const hw, hri_dac_intenset_reg_t mask)
{
	((Dac *)hw)->INTENCLR.reg = mask;
}

static inline bool hri_dac_get_STATUS_SYNCBUSY_bit(const void *const hw)
{
	hri_dac_wait_for_sync(hw);
	return (((Dac *)hw)->STATUS.reg & DAC_STATUS_SYNCBUSY) >> DAC_STATUS_SYNCBUSY_Pos;
}

static inline hri_dac_status_reg_t hri_dac_get_STATUS_reg(const void *const hw, hri_dac_status_reg_t mask)
{
	uint8_t tmp;
	hri_dac_wait_for_sync(hw);
	tmp = ((Dac *)hw)->STATUS.reg;
	tmp &= mask;
	return tmp;
}

static inline hri_dac_status_reg_t hri_dac_read_STATUS_reg(const void *const hw)
{
	hri_dac_wait_for_sync(hw);
	return ((Dac *)hw)->STATUS.reg;
}

static inline void hri_dac_set_CTRLA_SWRST_bit(const void *const hw)
{
	DAC_CRITICAL_SECTION_ENTER();
	((Dac *)hw)->CTRLA.reg |= DAC_CTRLA_SWRST;
	hri_dac_wait_for_sync(hw);
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_dac_get_CTRLA_SWRST_bit(const void *const hw)
{
	uint8_t tmp;
	hri_dac_wait_for_sync(hw);
	tmp = ((Dac *)hw)->CTRLA.reg;
	tmp = (tmp & DAC_CTRLA_SWRST) >> DAC_CTRLA_SWRST_Pos;
	return (bool)tmp;
}

static inline void hri_dac_set_CTRLA_ENABLE_bit(const void *const hw)
{
	DAC_CRITICAL_SECTION_ENTER();
	((Dac *)hw)->CTRLA.reg |= DAC_CTRLA_ENABLE;
	hri_dac_wait_for_sync(hw);
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_dac_get_CTRLA_ENABLE_bit(const void *const hw)
{
	uint8_t tmp;
	hri_dac_wait_for_sync(hw);
	tmp = ((Dac *)hw)->CTRLA.reg;
	tmp = (tmp & DAC_CTRLA_ENABLE) >> DAC_CTRLA_ENABLE_Pos;
	return (bool)tmp;
}

static inline void hri_dac_write_CTRLA_ENABLE_bit(const void *const hw, bool value)
{
	uint8_t tmp;
	DAC_CRITICAL_SECTION_ENTER();
	tmp = ((Dac *)hw)->CTRLA.reg;
	tmp &= ~DAC_CTRLA_ENABLE;
	tmp |= value << DAC_CTRLA_ENABLE_Pos;
	((Dac *)hw)->CTRLA.reg = tmp;
	hri_dac_wait_for_sync(hw);
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_dac_clear_CTRLA_ENABLE_bit(const void *const hw)
{
	DAC_CRITICAL_SECTION_ENTER();
	((Dac *)hw)->CTRLA.reg &= ~DAC_CTRLA_ENABLE;
	hri_dac_wait_for_sync(hw);
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_dac_toggle_CTRLA_ENABLE_bit(const void *const hw)
{
	DAC_CRITICAL_SECTION_ENTER();
	((Dac *)hw)->CTRLA.reg ^= DAC_CTRLA_ENABLE;
	hri_dac_wait_for_sync(hw);
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_dac_set_CTRLA_RUNSTDBY_bit(const void *const hw)
{
	DAC_CRITICAL_SECTION_ENTER();
	((Dac *)hw)->CTRLA.reg |= DAC_CTRLA_RUNSTDBY;
	hri_dac_wait_for_sync(hw);
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_dac_get_CTRLA_RUNSTDBY_bit(const void *const hw)
{
	uint8_t tmp;
	hri_dac_wait_for_sync(hw);
	tmp = ((Dac *)hw)->CTRLA.reg;
	tmp = (tmp & DAC_CTRLA_RUNSTDBY) >> DAC_CTRLA_RUNSTDBY_Pos;
	return (bool)tmp;
}

static inline void hri_dac_write_CTRLA_RUNSTDBY_bit(const void *const hw, bool value)
{
	uint8_t tmp;
	DAC_CRITICAL_SECTION_ENTER();
	tmp = ((Dac *)hw)->CTRLA.reg;
	tmp &= ~DAC_CTRLA_RUNSTDBY;
	tmp |= value << DAC_CTRLA_RUNSTDBY_Pos;
	((Dac *)hw)->CTRLA.reg = tmp;
	hri_dac_wait_for_sync(hw);
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_dac_clear_CTRLA_RUNSTDBY_bit(const void *const hw)
{
	DAC_CRITICAL_SECTION_ENTER();
	((Dac *)hw)->CTRLA.reg &= ~DAC_CTRLA_RUNSTDBY;
	hri_dac_wait_for_sync(hw);
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_dac_toggle_CTRLA_RUNSTDBY_bit(const void *const hw)
{
	DAC_CRITICAL_SECTION_ENTER();
	((Dac *)hw)->CTRLA.reg ^= DAC_CTRLA_RUNSTDBY;
	hri_dac_wait_for_sync(hw);
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_dac_set_CTRLA_reg(const void *const hw, hri_dac_ctrla_reg_t mask)
{
	DAC_CRITICAL_SECTION_ENTER();
	((Dac *)hw)->CTRLA.reg |= mask;
	hri_dac_wait_for_sync(hw);
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline hri_dac_ctrla_reg_t hri_dac_get_CTRLA_reg(const void *const hw, hri_dac_ctrla_reg_t mask)
{
	uint8_t tmp;
	hri_dac_wait_for_sync(hw);
	tmp = ((Dac *)hw)->CTRLA.reg;
	tmp &= mask;
	return tmp;
}

static inline void hri_dac_write_CTRLA_reg(const void *const hw, hri_dac_ctrla_reg_t data)
{
	DAC_CRITICAL_SECTION_ENTER();
	((Dac *)hw)->CTRLA.reg = data;
	hri_dac_wait_for_sync(hw);
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_dac_clear_CTRLA_reg(const void *const hw, hri_dac_ctrla_reg_t mask)
{
	DAC_CRITICAL_SECTION_ENTER();
	((Dac *)hw)->CTRLA.reg &= ~mask;
	hri_dac_wait_for_sync(hw);
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_dac_toggle_CTRLA_reg(const void *const hw, hri_dac_ctrla_reg_t mask)
{
	DAC_CRITICAL_SECTION_ENTER();
	((Dac *)hw)->CTRLA.reg ^= mask;
	hri_dac_wait_for_sync(hw);
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline hri_dac_ctrla_reg_t hri_dac_read_CTRLA_reg(const void *const hw)
{
	hri_dac_wait_for_sync(hw);
	return ((Dac *)hw)->CTRLA.reg;
}

static inline void hri_dac_set_CTRLB_EOEN_bit(const void *const hw)
{
	DAC_CRITICAL_SECTION_ENTER();
	((Dac *)hw)->CTRLB.reg |= DAC_CTRLB_EOEN;
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_dac_get_CTRLB_EOEN_bit(const void *const hw)
{
	uint8_t tmp;
	tmp = ((Dac *)hw)->CTRLB.reg;
	tmp = (tmp & DAC_CTRLB_EOEN) >> DAC_CTRLB_EOEN_Pos;
	return (bool)tmp;
}

static inline void hri_dac_write_CTRLB_EOEN_bit(const void *const hw, bool value)
{
	uint8_t tmp;
	DAC_CRITICAL_SECTION_ENTER();
	tmp = ((Dac *)hw)->CTRLB.reg;
	tmp &= ~DAC_CTRLB_EOEN;
	tmp |= value << DAC_CTRLB_EOEN_Pos;
	((Dac *)hw)->CTRLB.reg = tmp;
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_dac_clear_CTRLB_EOEN_bit(const void *const hw)
{
	DAC_CRITICAL_SECTION_ENTER();
	((Dac *)hw)->CTRLB.reg &= ~DAC_CTRLB_EOEN;
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_dac_toggle_CTRLB_EOEN_bit(const void *const hw)
{
	DAC_CRITICAL_SECTION_ENTER();
	((Dac *)hw)->CTRLB.reg ^= DAC_CTRLB_EOEN;
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_dac_set_CTRLB_IOEN_bit(const void *const hw)
{
	DAC_CRITICAL_SECTION_ENTER();
	((Dac *)hw)->CTRLB.reg |= DAC_CTRLB_IOEN;
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_dac_get_CTRLB_IOEN_bit(const void *const hw)
{
	uint8_t tmp;
	tmp = ((Dac *)hw)->CTRLB.reg;
	tmp = (tmp & DAC_CTRLB_IOEN) >> DAC_CTRLB_IOEN_Pos;
	return (bool)tmp;
}

static inline void hri_dac_write_CTRLB_IOEN_bit(const void *const hw, bool value)
{
	uint8_t tmp;
	DAC_CRITICAL_SECTION_ENTER();
	tmp = ((Dac *)hw)->CTRLB.reg;
	tmp &= ~DAC_CTRLB_IOEN;
	tmp |= value << DAC_CTRLB_IOEN_Pos;
	((Dac *)hw)->CTRLB.reg = tmp;
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_dac_clear_CTRLB_IOEN_bit(const void *const hw)
{
	DAC_CRITICAL_SECTION_ENTER();
	((Dac *)hw)->CTRLB.reg &= ~DAC_CTRLB_IOEN;
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_dac_toggle_CTRLB_IOEN_bit(const void *const hw)
{
	DAC_CRITICAL_SECTION_ENTER();
	((Dac *)hw)->CTRLB.reg ^= DAC_CTRLB_IOEN;
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_dac_set_CTRLB_LEFTADJ_bit(const void *const hw)
{
	DAC_CRITICAL_SECTION_ENTER();
	((Dac *)hw)->CTRLB.reg |= DAC_CTRLB_LEFTADJ;
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_dac_get_CTRLB_LEFTADJ_bit(const void *const hw)
{
	uint8_t tmp;
	tmp = ((Dac *)hw)->CTRLB.reg;
	tmp = (tmp & DAC_CTRLB_LEFTADJ) >> DAC_CTRLB_LEFTADJ_Pos;
	return (bool)tmp;
}

static inline void hri_dac_write_CTRLB_LEFTADJ_bit(const void *const hw, bool value)
{
	uint8_t tmp;
	DAC_CRITICAL_SECTION_ENTER();
	tmp = ((Dac *)hw)->CTRLB.reg;
	tmp &= ~DAC_CTRLB_LEFTADJ;
	tmp |= value << DAC_CTRLB_LEFTADJ_Pos;
	((Dac *)hw)->CTRLB.reg = tmp;
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_dac_clear_CTRLB_LEFTADJ_bit(const void *const hw)
{
	DAC_CRITICAL_SECTION_ENTER();
	((Dac *)hw)->CTRLB.reg &= ~DAC_CTRLB_LEFTADJ;
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_dac_toggle_CTRLB_LEFTADJ_bit(const void *const hw)
{
	DAC_CRITICAL_SECTION_ENTER();
	((Dac *)hw)->CTRLB.reg ^= DAC_CTRLB_LEFTADJ;
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_dac_set_CTRLB_VPD_bit(const void *const hw)
{
	DAC_CRITICAL_SECTION_ENTER();
	((Dac *)hw)->CTRLB.reg |= DAC_CTRLB_VPD;
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_dac_get_CTRLB_VPD_bit(const void *const hw)
{
	uint8_t tmp;
	tmp = ((Dac *)hw)->CTRLB.reg;
	tmp = (tmp & DAC_CTRLB_VPD) >> DAC_CTRLB_VPD_Pos;
	return (bool)tmp;
}

static inline void hri_dac_write_CTRLB_VPD_bit(const void *const hw, bool value)
{
	uint8_t tmp;
	DAC_CRITICAL_SECTION_ENTER();
	tmp = ((Dac *)hw)->CTRLB.reg;
	tmp &= ~DAC_CTRLB_VPD;
	tmp |= value << DAC_CTRLB_VPD_Pos;
	((Dac *)hw)->CTRLB.reg = tmp;
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_dac_clear_CTRLB_VPD_bit(const void *const hw)
{
	DAC_CRITICAL_SECTION_ENTER();
	((Dac *)hw)->CTRLB.reg &= ~DAC_CTRLB_VPD;
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_dac_toggle_CTRLB_VPD_bit(const void *const hw)
{
	DAC_CRITICAL_SECTION_ENTER();
	((Dac *)hw)->CTRLB.reg ^= DAC_CTRLB_VPD;
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_dac_set_CTRLB_BDWP_bit(const void *const hw)
{
	DAC_CRITICAL_SECTION_ENTER();
	((Dac *)hw)->CTRLB.reg |= DAC_CTRLB_BDWP;
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_dac_get_CTRLB_BDWP_bit(const void *const hw)
{
	uint8_t tmp;
	tmp = ((Dac *)hw)->CTRLB.reg;
	tmp = (tmp & DAC_CTRLB_BDWP) >> DAC_CTRLB_BDWP_Pos;
	return (bool)tmp;
}

static inline void hri_dac_write_CTRLB_BDWP_bit(const void *const hw, bool value)
{
	uint8_t tmp;
	DAC_CRITICAL_SECTION_ENTER();
	tmp = ((Dac *)hw)->CTRLB.reg;
	tmp &= ~DAC_CTRLB_BDWP;
	tmp |= value << DAC_CTRLB_BDWP_Pos;
	((Dac *)hw)->CTRLB.reg = tmp;
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_dac_clear_CTRLB_BDWP_bit(const void *const hw)
{
	DAC_CRITICAL_SECTION_ENTER();
	((Dac *)hw)->CTRLB.reg &= ~DAC_CTRLB_BDWP;
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_dac_toggle_CTRLB_BDWP_bit(const void *const hw)
{
	DAC_CRITICAL_SECTION_ENTER();
	((Dac *)hw)->CTRLB.reg ^= DAC_CTRLB_BDWP;
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_dac_set_CTRLB_REFSEL_bf(const void *const hw, hri_dac_ctrlb_reg_t mask)
{
	DAC_CRITICAL_SECTION_ENTER();
	((Dac *)hw)->CTRLB.reg |= DAC_CTRLB_REFSEL(mask);
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline hri_dac_ctrlb_reg_t hri_dac_get_CTRLB_REFSEL_bf(const void *const hw, hri_dac_ctrlb_reg_t mask)
{
	uint8_t tmp;
	tmp = ((Dac *)hw)->CTRLB.reg;
	tmp = (tmp & DAC_CTRLB_REFSEL(mask)) >> DAC_CTRLB_REFSEL_Pos;
	return tmp;
}

static inline void hri_dac_write_CTRLB_REFSEL_bf(const void *const hw, hri_dac_ctrlb_reg_t data)
{
	uint8_t tmp;
	DAC_CRITICAL_SECTION_ENTER();
	tmp = ((Dac *)hw)->CTRLB.reg;
	tmp &= ~DAC_CTRLB_REFSEL_Msk;
	tmp |= DAC_CTRLB_REFSEL(data);
	((Dac *)hw)->CTRLB.reg = tmp;
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_dac_clear_CTRLB_REFSEL_bf(const void *const hw, hri_dac_ctrlb_reg_t mask)
{
	DAC_CRITICAL_SECTION_ENTER();
	((Dac *)hw)->CTRLB.reg &= ~DAC_CTRLB_REFSEL(mask);
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_dac_toggle_CTRLB_REFSEL_bf(const void *const hw, hri_dac_ctrlb_reg_t mask)
{
	DAC_CRITICAL_SECTION_ENTER();
	((Dac *)hw)->CTRLB.reg ^= DAC_CTRLB_REFSEL(mask);
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline hri_dac_ctrlb_reg_t hri_dac_read_CTRLB_REFSEL_bf(const void *const hw)
{
	uint8_t tmp;
	tmp = ((Dac *)hw)->CTRLB.reg;
	tmp = (tmp & DAC_CTRLB_REFSEL_Msk) >> DAC_CTRLB_REFSEL_Pos;
	return tmp;
}

static inline void hri_dac_set_CTRLB_reg(const void *const hw, hri_dac_ctrlb_reg_t mask)
{
	DAC_CRITICAL_SECTION_ENTER();
	((Dac *)hw)->CTRLB.reg |= mask;
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline hri_dac_ctrlb_reg_t hri_dac_get_CTRLB_reg(const void *const hw, hri_dac_ctrlb_reg_t mask)
{
	uint8_t tmp;
	tmp = ((Dac *)hw)->CTRLB.reg;
	tmp &= mask;
	return tmp;
}

static inline void hri_dac_write_CTRLB_reg(const void *const hw, hri_dac_ctrlb_reg_t data)
{
	DAC_CRITICAL_SECTION_ENTER();
	((Dac *)hw)->CTRLB.reg = data;
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_dac_clear_CTRLB_reg(const void *const hw, hri_dac_ctrlb_reg_t mask)
{
	DAC_CRITICAL_SECTION_ENTER();
	((Dac *)hw)->CTRLB.reg &= ~mask;
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_dac_toggle_CTRLB_reg(const void *const hw, hri_dac_ctrlb_reg_t mask)
{
	DAC_CRITICAL_SECTION_ENTER();
	((Dac *)hw)->CTRLB.reg ^= mask;
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline hri_dac_ctrlb_reg_t hri_dac_read_CTRLB_reg(const void *const hw)
{
	return ((Dac *)hw)->CTRLB.reg;
}

static inline void hri_dac_set_EVCTRL_STARTEI_bit(const void *const hw)
{
	DAC_CRITICAL_SECTION_ENTER();
	((Dac *)hw)->EVCTRL.reg |= DAC_EVCTRL_STARTEI;
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_dac_get_EVCTRL_STARTEI_bit(const void *const hw)
{
	uint8_t tmp;
	tmp = ((Dac *)hw)->EVCTRL.reg;
	tmp = (tmp & DAC_EVCTRL_STARTEI) >> DAC_EVCTRL_STARTEI_Pos;
	return (bool)tmp;
}

static inline void hri_dac_write_EVCTRL_STARTEI_bit(const void *const hw, bool value)
{
	uint8_t tmp;
	DAC_CRITICAL_SECTION_ENTER();
	tmp = ((Dac *)hw)->EVCTRL.reg;
	tmp &= ~DAC_EVCTRL_STARTEI;
	tmp |= value << DAC_EVCTRL_STARTEI_Pos;
	((Dac *)hw)->EVCTRL.reg = tmp;
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_dac_clear_EVCTRL_STARTEI_bit(const void *const hw)
{
	DAC_CRITICAL_SECTION_ENTER();
	((Dac *)hw)->EVCTRL.reg &= ~DAC_EVCTRL_STARTEI;
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_dac_toggle_EVCTRL_STARTEI_bit(const void *const hw)
{
	DAC_CRITICAL_SECTION_ENTER();
	((Dac *)hw)->EVCTRL.reg ^= DAC_EVCTRL_STARTEI;
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_dac_set_EVCTRL_EMPTYEO_bit(const void *const hw)
{
	DAC_CRITICAL_SECTION_ENTER();
	((Dac *)hw)->EVCTRL.reg |= DAC_EVCTRL_EMPTYEO;
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_dac_get_EVCTRL_EMPTYEO_bit(const void *const hw)
{
	uint8_t tmp;
	tmp = ((Dac *)hw)->EVCTRL.reg;
	tmp = (tmp & DAC_EVCTRL_EMPTYEO) >> DAC_EVCTRL_EMPTYEO_Pos;
	return (bool)tmp;
}

static inline void hri_dac_write_EVCTRL_EMPTYEO_bit(const void *const hw, bool value)
{
	uint8_t tmp;
	DAC_CRITICAL_SECTION_ENTER();
	tmp = ((Dac *)hw)->EVCTRL.reg;
	tmp &= ~DAC_EVCTRL_EMPTYEO;
	tmp |= value << DAC_EVCTRL_EMPTYEO_Pos;
	((Dac *)hw)->EVCTRL.reg = tmp;
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_dac_clear_EVCTRL_EMPTYEO_bit(const void *const hw)
{
	DAC_CRITICAL_SECTION_ENTER();
	((Dac *)hw)->EVCTRL.reg &= ~DAC_EVCTRL_EMPTYEO;
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_dac_toggle_EVCTRL_EMPTYEO_bit(const void *const hw)
{
	DAC_CRITICAL_SECTION_ENTER();
	((Dac *)hw)->EVCTRL.reg ^= DAC_EVCTRL_EMPTYEO;
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_dac_set_EVCTRL_reg(const void *const hw, hri_dac_evctrl_reg_t mask)
{
	DAC_CRITICAL_SECTION_ENTER();
	((Dac *)hw)->EVCTRL.reg |= mask;
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline hri_dac_evctrl_reg_t hri_dac_get_EVCTRL_reg(const void *const hw, hri_dac_evctrl_reg_t mask)
{
	uint8_t tmp;
	tmp = ((Dac *)hw)->EVCTRL.reg;
	tmp &= mask;
	return tmp;
}

static inline void hri_dac_write_EVCTRL_reg(const void *const hw, hri_dac_evctrl_reg_t data)
{
	DAC_CRITICAL_SECTION_ENTER();
	((Dac *)hw)->EVCTRL.reg = data;
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_dac_clear_EVCTRL_reg(const void *const hw, hri_dac_evctrl_reg_t mask)
{
	DAC_CRITICAL_SECTION_ENTER();
	((Dac *)hw)->EVCTRL.reg &= ~mask;
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_dac_toggle_EVCTRL_reg(const void *const hw, hri_dac_evctrl_reg_t mask)
{
	DAC_CRITICAL_SECTION_ENTER();
	((Dac *)hw)->EVCTRL.reg ^= mask;
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline hri_dac_evctrl_reg_t hri_dac_read_EVCTRL_reg(const void *const hw)
{
	return ((Dac *)hw)->EVCTRL.reg;
}

static inline void hri_dac_set_DATA_DATA_bf(const void *const hw, hri_dac_data_reg_t mask)
{
	DAC_CRITICAL_SECTION_ENTER();
	((Dac *)hw)->DATA.reg |= DAC_DATA_DATA(mask);
	hri_dac_wait_for_sync(hw);
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline hri_dac_data_reg_t hri_dac_get_DATA_DATA_bf(const void *const hw, hri_dac_data_reg_t mask)
{
	uint16_t tmp;
	hri_dac_wait_for_sync(hw);
	tmp = ((Dac *)hw)->DATA.reg;
	tmp = (tmp & DAC_DATA_DATA(mask)) >> DAC_DATA_DATA_Pos;
	return tmp;
}

static inline void hri_dac_write_DATA_DATA_bf(const void *const hw, hri_dac_data_reg_t data)
{
	uint16_t tmp;
	DAC_CRITICAL_SECTION_ENTER();
	tmp = ((Dac *)hw)->DATA.reg;
	tmp &= ~DAC_DATA_DATA_Msk;
	tmp |= DAC_DATA_DATA(data);
	((Dac *)hw)->DATA.reg = tmp;
	hri_dac_wait_for_sync(hw);
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_dac_clear_DATA_DATA_bf(const void *const hw, hri_dac_data_reg_t mask)
{
	DAC_CRITICAL_SECTION_ENTER();
	((Dac *)hw)->DATA.reg &= ~DAC_DATA_DATA(mask);
	hri_dac_wait_for_sync(hw);
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_dac_toggle_DATA_DATA_bf(const void *const hw, hri_dac_data_reg_t mask)
{
	DAC_CRITICAL_SECTION_ENTER();
	((Dac *)hw)->DATA.reg ^= DAC_DATA_DATA(mask);
	hri_dac_wait_for_sync(hw);
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline hri_dac_data_reg_t hri_dac_read_DATA_DATA_bf(const void *const hw)
{
	uint16_t tmp;
	hri_dac_wait_for_sync(hw);
	tmp = ((Dac *)hw)->DATA.reg;
	tmp = (tmp & DAC_DATA_DATA_Msk) >> DAC_DATA_DATA_Pos;
	return tmp;
}

static inline void hri_dac_set_DATA_reg(const void *const hw, hri_dac_data_reg_t mask)
{
	DAC_CRITICAL_SECTION_ENTER();
	((Dac *)hw)->DATA.reg |= mask;
	hri_dac_wait_for_sync(hw);
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline hri_dac_data_reg_t hri_dac_get_DATA_reg(const void *const hw, hri_dac_data_reg_t mask)
{
	uint16_t tmp;
	hri_dac_wait_for_sync(hw);
	tmp = ((Dac *)hw)->DATA.reg;
	tmp &= mask;
	return tmp;
}

static inline void hri_dac_write_DATA_reg(const void *const hw, hri_dac_data_reg_t data)
{
	DAC_CRITICAL_SECTION_ENTER();
	((Dac *)hw)->DATA.reg = data;
	hri_dac_wait_for_sync(hw);
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_dac_clear_DATA_reg(const void *const hw, hri_dac_data_reg_t mask)
{
	DAC_CRITICAL_SECTION_ENTER();
	((Dac *)hw)->DATA.reg &= ~mask;
	hri_dac_wait_for_sync(hw);
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_dac_toggle_DATA_reg(const void *const hw, hri_dac_data_reg_t mask)
{
	DAC_CRITICAL_SECTION_ENTER();
	((Dac *)hw)->DATA.reg ^= mask;
	hri_dac_wait_for_sync(hw);
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline hri_dac_data_reg_t hri_dac_read_DATA_reg(const void *const hw)
{
	hri_dac_wait_for_sync(hw);
	return ((Dac *)hw)->DATA.reg;
}

static inline void hri_dac_set_DATABUF_DATABUF_bf(const void *const hw, hri_dac_databuf_reg_t mask)
{
	DAC_CRITICAL_SECTION_ENTER();
	((Dac *)hw)->DATABUF.reg |= DAC_DATABUF_DATABUF(mask);
	hri_dac_wait_for_sync(hw);
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline hri_dac_databuf_reg_t hri_dac_get_DATABUF_DATABUF_bf(const void *const hw, hri_dac_databuf_reg_t mask)
{
	uint16_t tmp;
	hri_dac_wait_for_sync(hw);
	tmp = ((Dac *)hw)->DATABUF.reg;
	tmp = (tmp & DAC_DATABUF_DATABUF(mask)) >> DAC_DATABUF_DATABUF_Pos;
	return tmp;
}

static inline void hri_dac_write_DATABUF_DATABUF_bf(const void *const hw, hri_dac_databuf_reg_t data)
{
	uint16_t tmp;
	DAC_CRITICAL_SECTION_ENTER();
	tmp = ((Dac *)hw)->DATABUF.reg;
	tmp &= ~DAC_DATABUF_DATABUF_Msk;
	tmp |= DAC_DATABUF_DATABUF(data);
	((Dac *)hw)->DATABUF.reg = tmp;
	hri_dac_wait_for_sync(hw);
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_dac_clear_DATABUF_DATABUF_bf(const void *const hw, hri_dac_databuf_reg_t mask)
{
	DAC_CRITICAL_SECTION_ENTER();
	((Dac *)hw)->DATABUF.reg &= ~DAC_DATABUF_DATABUF(mask);
	hri_dac_wait_for_sync(hw);
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_dac_toggle_DATABUF_DATABUF_bf(const void *const hw, hri_dac_databuf_reg_t mask)
{
	DAC_CRITICAL_SECTION_ENTER();
	((Dac *)hw)->DATABUF.reg ^= DAC_DATABUF_DATABUF(mask);
	hri_dac_wait_for_sync(hw);
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline hri_dac_databuf_reg_t hri_dac_read_DATABUF_DATABUF_bf(const void *const hw)
{
	uint16_t tmp;
	hri_dac_wait_for_sync(hw);
	tmp = ((Dac *)hw)->DATABUF.reg;
	tmp = (tmp & DAC_DATABUF_DATABUF_Msk) >> DAC_DATABUF_DATABUF_Pos;
	return tmp;
}

static inline void hri_dac_set_DATABUF_reg(const void *const hw, hri_dac_databuf_reg_t mask)
{
	DAC_CRITICAL_SECTION_ENTER();
	((Dac *)hw)->DATABUF.reg |= mask;
	hri_dac_wait_for_sync(hw);
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline hri_dac_databuf_reg_t hri_dac_get_DATABUF_reg(const void *const hw, hri_dac_databuf_reg_t mask)
{
	uint16_t tmp;
	hri_dac_wait_for_sync(hw);
	tmp = ((Dac *)hw)->DATABUF.reg;
	tmp &= mask;
	return tmp;
}

static inline void hri_dac_write_DATABUF_reg(const void *const hw, hri_dac_databuf_reg_t data)
{
	DAC_CRITICAL_SECTION_ENTER();
	((Dac *)hw)->DATABUF.reg = data;
	hri_dac_wait_for_sync(hw);
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_dac_clear_DATABUF_reg(const void *const hw, hri_dac_databuf_reg_t mask)
{
	DAC_CRITICAL_SECTION_ENTER();
	((Dac *)hw)->DATABUF.reg &= ~mask;
	hri_dac_wait_for_sync(hw);
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_dac_toggle_DATABUF_reg(const void *const hw, hri_dac_databuf_reg_t mask)
{
	DAC_CRITICAL_SECTION_ENTER();
	((Dac *)hw)->DATABUF.reg ^= mask;
	hri_dac_wait_for_sync(hw);
	DAC_CRITICAL_SECTION_LEAVE();
}

static inline hri_dac_databuf_reg_t hri_dac_read_DATABUF_reg(const void *const hw)
{
	hri_dac_wait_for_sync(hw);
	return ((Dac *)hw)->DATABUF.reg;
}

#ifdef __cplusplus
}
#endif

#endif /* _HRI_DAC_D21C_H_INCLUDED */
#endif /* _SAMD21_DAC_COMPONENT_ */
