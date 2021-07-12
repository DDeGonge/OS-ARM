/**
 * \file
 *
 * \brief SAM TCC
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

#ifdef _SAMD21_TCC_COMPONENT_
#ifndef _HRI_TCC_D21C_H_INCLUDED_
#define _HRI_TCC_D21C_H_INCLUDED_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <hal_atomic.h>

#if defined(ENABLE_TCC_CRITICAL_SECTIONS)
#define TCC_CRITICAL_SECTION_ENTER() CRITICAL_SECTION_ENTER()
#define TCC_CRITICAL_SECTION_LEAVE() CRITICAL_SECTION_LEAVE()
#else
#define TCC_CRITICAL_SECTION_ENTER()
#define TCC_CRITICAL_SECTION_LEAVE()
#endif

typedef uint16_t hri_tcc_patt_reg_t;
typedef uint16_t hri_tcc_pattb_reg_t;
typedef uint32_t hri_tcc_cc_reg_t;
typedef uint32_t hri_tcc_ccb_reg_t;
typedef uint32_t hri_tcc_count_reg_t;
typedef uint32_t hri_tcc_ctrla_reg_t;
typedef uint32_t hri_tcc_drvctrl_reg_t;
typedef uint32_t hri_tcc_evctrl_reg_t;
typedef uint32_t hri_tcc_fctrla_reg_t;
typedef uint32_t hri_tcc_fctrlb_reg_t;
typedef uint32_t hri_tcc_intenset_reg_t;
typedef uint32_t hri_tcc_intflag_reg_t;
typedef uint32_t hri_tcc_per_reg_t;
typedef uint32_t hri_tcc_perb_reg_t;
typedef uint32_t hri_tcc_status_reg_t;
typedef uint32_t hri_tcc_syncbusy_reg_t;
typedef uint32_t hri_tcc_wave_reg_t;
typedef uint32_t hri_tcc_waveb_reg_t;
typedef uint32_t hri_tcc_wexctrl_reg_t;
typedef uint8_t  hri_tcc_ctrlbset_reg_t;
typedef uint8_t  hri_tcc_dbgctrl_reg_t;

static inline void hri_tcc_wait_for_sync(const void *const hw, hri_tcc_syncbusy_reg_t reg)
{
	while (((Tcc *)hw)->SYNCBUSY.reg & reg) {
	};
}

static inline bool hri_tcc_is_syncing(const void *const hw, hri_tcc_syncbusy_reg_t reg)
{
	return ((Tcc *)hw)->SYNCBUSY.reg & reg;
}

static inline bool hri_tcc_get_INTFLAG_OVF_bit(const void *const hw)
{
	return (((Tcc *)hw)->INTFLAG.reg & TCC_INTFLAG_OVF) >> TCC_INTFLAG_OVF_Pos;
}

static inline void hri_tcc_clear_INTFLAG_OVF_bit(const void *const hw)
{
	((Tcc *)hw)->INTFLAG.reg = TCC_INTFLAG_OVF;
}

static inline bool hri_tcc_get_INTFLAG_TRG_bit(const void *const hw)
{
	return (((Tcc *)hw)->INTFLAG.reg & TCC_INTFLAG_TRG) >> TCC_INTFLAG_TRG_Pos;
}

static inline void hri_tcc_clear_INTFLAG_TRG_bit(const void *const hw)
{
	((Tcc *)hw)->INTFLAG.reg = TCC_INTFLAG_TRG;
}

static inline bool hri_tcc_get_INTFLAG_CNT_bit(const void *const hw)
{
	return (((Tcc *)hw)->INTFLAG.reg & TCC_INTFLAG_CNT) >> TCC_INTFLAG_CNT_Pos;
}

static inline void hri_tcc_clear_INTFLAG_CNT_bit(const void *const hw)
{
	((Tcc *)hw)->INTFLAG.reg = TCC_INTFLAG_CNT;
}

static inline bool hri_tcc_get_INTFLAG_ERR_bit(const void *const hw)
{
	return (((Tcc *)hw)->INTFLAG.reg & TCC_INTFLAG_ERR) >> TCC_INTFLAG_ERR_Pos;
}

static inline void hri_tcc_clear_INTFLAG_ERR_bit(const void *const hw)
{
	((Tcc *)hw)->INTFLAG.reg = TCC_INTFLAG_ERR;
}

static inline bool hri_tcc_get_INTFLAG_DFS_bit(const void *const hw)
{
	return (((Tcc *)hw)->INTFLAG.reg & TCC_INTFLAG_DFS) >> TCC_INTFLAG_DFS_Pos;
}

static inline void hri_tcc_clear_INTFLAG_DFS_bit(const void *const hw)
{
	((Tcc *)hw)->INTFLAG.reg = TCC_INTFLAG_DFS;
}

static inline bool hri_tcc_get_INTFLAG_FAULTA_bit(const void *const hw)
{
	return (((Tcc *)hw)->INTFLAG.reg & TCC_INTFLAG_FAULTA) >> TCC_INTFLAG_FAULTA_Pos;
}

static inline void hri_tcc_clear_INTFLAG_FAULTA_bit(const void *const hw)
{
	((Tcc *)hw)->INTFLAG.reg = TCC_INTFLAG_FAULTA;
}

static inline bool hri_tcc_get_INTFLAG_FAULTB_bit(const void *const hw)
{
	return (((Tcc *)hw)->INTFLAG.reg & TCC_INTFLAG_FAULTB) >> TCC_INTFLAG_FAULTB_Pos;
}

static inline void hri_tcc_clear_INTFLAG_FAULTB_bit(const void *const hw)
{
	((Tcc *)hw)->INTFLAG.reg = TCC_INTFLAG_FAULTB;
}

static inline bool hri_tcc_get_INTFLAG_FAULT0_bit(const void *const hw)
{
	return (((Tcc *)hw)->INTFLAG.reg & TCC_INTFLAG_FAULT0) >> TCC_INTFLAG_FAULT0_Pos;
}

static inline void hri_tcc_clear_INTFLAG_FAULT0_bit(const void *const hw)
{
	((Tcc *)hw)->INTFLAG.reg = TCC_INTFLAG_FAULT0;
}

static inline bool hri_tcc_get_INTFLAG_FAULT1_bit(const void *const hw)
{
	return (((Tcc *)hw)->INTFLAG.reg & TCC_INTFLAG_FAULT1) >> TCC_INTFLAG_FAULT1_Pos;
}

static inline void hri_tcc_clear_INTFLAG_FAULT1_bit(const void *const hw)
{
	((Tcc *)hw)->INTFLAG.reg = TCC_INTFLAG_FAULT1;
}

static inline bool hri_tcc_get_INTFLAG_MC0_bit(const void *const hw)
{
	return (((Tcc *)hw)->INTFLAG.reg & TCC_INTFLAG_MC0) >> TCC_INTFLAG_MC0_Pos;
}

static inline void hri_tcc_clear_INTFLAG_MC0_bit(const void *const hw)
{
	((Tcc *)hw)->INTFLAG.reg = TCC_INTFLAG_MC0;
}

static inline bool hri_tcc_get_INTFLAG_MC1_bit(const void *const hw)
{
	return (((Tcc *)hw)->INTFLAG.reg & TCC_INTFLAG_MC1) >> TCC_INTFLAG_MC1_Pos;
}

static inline void hri_tcc_clear_INTFLAG_MC1_bit(const void *const hw)
{
	((Tcc *)hw)->INTFLAG.reg = TCC_INTFLAG_MC1;
}

static inline bool hri_tcc_get_INTFLAG_MC2_bit(const void *const hw)
{
	return (((Tcc *)hw)->INTFLAG.reg & TCC_INTFLAG_MC2) >> TCC_INTFLAG_MC2_Pos;
}

static inline void hri_tcc_clear_INTFLAG_MC2_bit(const void *const hw)
{
	((Tcc *)hw)->INTFLAG.reg = TCC_INTFLAG_MC2;
}

static inline bool hri_tcc_get_INTFLAG_MC3_bit(const void *const hw)
{
	return (((Tcc *)hw)->INTFLAG.reg & TCC_INTFLAG_MC3) >> TCC_INTFLAG_MC3_Pos;
}

static inline void hri_tcc_clear_INTFLAG_MC3_bit(const void *const hw)
{
	((Tcc *)hw)->INTFLAG.reg = TCC_INTFLAG_MC3;
}

static inline bool hri_tcc_get_interrupt_OVF_bit(const void *const hw)
{
	return (((Tcc *)hw)->INTFLAG.reg & TCC_INTFLAG_OVF) >> TCC_INTFLAG_OVF_Pos;
}

static inline void hri_tcc_clear_interrupt_OVF_bit(const void *const hw)
{
	((Tcc *)hw)->INTFLAG.reg = TCC_INTFLAG_OVF;
}

static inline bool hri_tcc_get_interrupt_TRG_bit(const void *const hw)
{
	return (((Tcc *)hw)->INTFLAG.reg & TCC_INTFLAG_TRG) >> TCC_INTFLAG_TRG_Pos;
}

static inline void hri_tcc_clear_interrupt_TRG_bit(const void *const hw)
{
	((Tcc *)hw)->INTFLAG.reg = TCC_INTFLAG_TRG;
}

static inline bool hri_tcc_get_interrupt_CNT_bit(const void *const hw)
{
	return (((Tcc *)hw)->INTFLAG.reg & TCC_INTFLAG_CNT) >> TCC_INTFLAG_CNT_Pos;
}

static inline void hri_tcc_clear_interrupt_CNT_bit(const void *const hw)
{
	((Tcc *)hw)->INTFLAG.reg = TCC_INTFLAG_CNT;
}

static inline bool hri_tcc_get_interrupt_ERR_bit(const void *const hw)
{
	return (((Tcc *)hw)->INTFLAG.reg & TCC_INTFLAG_ERR) >> TCC_INTFLAG_ERR_Pos;
}

static inline void hri_tcc_clear_interrupt_ERR_bit(const void *const hw)
{
	((Tcc *)hw)->INTFLAG.reg = TCC_INTFLAG_ERR;
}

static inline bool hri_tcc_get_interrupt_DFS_bit(const void *const hw)
{
	return (((Tcc *)hw)->INTFLAG.reg & TCC_INTFLAG_DFS) >> TCC_INTFLAG_DFS_Pos;
}

static inline void hri_tcc_clear_interrupt_DFS_bit(const void *const hw)
{
	((Tcc *)hw)->INTFLAG.reg = TCC_INTFLAG_DFS;
}

static inline bool hri_tcc_get_interrupt_FAULTA_bit(const void *const hw)
{
	return (((Tcc *)hw)->INTFLAG.reg & TCC_INTFLAG_FAULTA) >> TCC_INTFLAG_FAULTA_Pos;
}

static inline void hri_tcc_clear_interrupt_FAULTA_bit(const void *const hw)
{
	((Tcc *)hw)->INTFLAG.reg = TCC_INTFLAG_FAULTA;
}

static inline bool hri_tcc_get_interrupt_FAULTB_bit(const void *const hw)
{
	return (((Tcc *)hw)->INTFLAG.reg & TCC_INTFLAG_FAULTB) >> TCC_INTFLAG_FAULTB_Pos;
}

static inline void hri_tcc_clear_interrupt_FAULTB_bit(const void *const hw)
{
	((Tcc *)hw)->INTFLAG.reg = TCC_INTFLAG_FAULTB;
}

static inline bool hri_tcc_get_interrupt_FAULT0_bit(const void *const hw)
{
	return (((Tcc *)hw)->INTFLAG.reg & TCC_INTFLAG_FAULT0) >> TCC_INTFLAG_FAULT0_Pos;
}

static inline void hri_tcc_clear_interrupt_FAULT0_bit(const void *const hw)
{
	((Tcc *)hw)->INTFLAG.reg = TCC_INTFLAG_FAULT0;
}

static inline bool hri_tcc_get_interrupt_FAULT1_bit(const void *const hw)
{
	return (((Tcc *)hw)->INTFLAG.reg & TCC_INTFLAG_FAULT1) >> TCC_INTFLAG_FAULT1_Pos;
}

static inline void hri_tcc_clear_interrupt_FAULT1_bit(const void *const hw)
{
	((Tcc *)hw)->INTFLAG.reg = TCC_INTFLAG_FAULT1;
}

static inline bool hri_tcc_get_interrupt_MC0_bit(const void *const hw)
{
	return (((Tcc *)hw)->INTFLAG.reg & TCC_INTFLAG_MC0) >> TCC_INTFLAG_MC0_Pos;
}

static inline void hri_tcc_clear_interrupt_MC0_bit(const void *const hw)
{
	((Tcc *)hw)->INTFLAG.reg = TCC_INTFLAG_MC0;
}

static inline bool hri_tcc_get_interrupt_MC1_bit(const void *const hw)
{
	return (((Tcc *)hw)->INTFLAG.reg & TCC_INTFLAG_MC1) >> TCC_INTFLAG_MC1_Pos;
}

static inline void hri_tcc_clear_interrupt_MC1_bit(const void *const hw)
{
	((Tcc *)hw)->INTFLAG.reg = TCC_INTFLAG_MC1;
}

static inline bool hri_tcc_get_interrupt_MC2_bit(const void *const hw)
{
	return (((Tcc *)hw)->INTFLAG.reg & TCC_INTFLAG_MC2) >> TCC_INTFLAG_MC2_Pos;
}

static inline void hri_tcc_clear_interrupt_MC2_bit(const void *const hw)
{
	((Tcc *)hw)->INTFLAG.reg = TCC_INTFLAG_MC2;
}

static inline bool hri_tcc_get_interrupt_MC3_bit(const void *const hw)
{
	return (((Tcc *)hw)->INTFLAG.reg & TCC_INTFLAG_MC3) >> TCC_INTFLAG_MC3_Pos;
}

static inline void hri_tcc_clear_interrupt_MC3_bit(const void *const hw)
{
	((Tcc *)hw)->INTFLAG.reg = TCC_INTFLAG_MC3;
}

static inline hri_tcc_intflag_reg_t hri_tcc_get_INTFLAG_reg(const void *const hw, hri_tcc_intflag_reg_t mask)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->INTFLAG.reg;
	tmp &= mask;
	return tmp;
}

static inline hri_tcc_intflag_reg_t hri_tcc_read_INTFLAG_reg(const void *const hw)
{
	return ((Tcc *)hw)->INTFLAG.reg;
}

static inline void hri_tcc_clear_INTFLAG_reg(const void *const hw, hri_tcc_intflag_reg_t mask)
{
	((Tcc *)hw)->INTFLAG.reg = mask;
}

static inline void hri_tcc_set_CTRLB_DIR_bit(const void *const hw)
{
	((Tcc *)hw)->CTRLBSET.reg = TCC_CTRLBSET_DIR;
}

static inline bool hri_tcc_get_CTRLB_DIR_bit(const void *const hw)
{
	return (((Tcc *)hw)->CTRLBSET.reg & TCC_CTRLBSET_DIR) >> TCC_CTRLBSET_DIR_Pos;
}

static inline void hri_tcc_write_CTRLB_DIR_bit(const void *const hw, bool value)
{
	if (value == 0x0) {
		((Tcc *)hw)->CTRLBCLR.reg = TCC_CTRLBSET_DIR;
	} else {
		((Tcc *)hw)->CTRLBSET.reg = TCC_CTRLBSET_DIR;
	}
}

static inline void hri_tcc_clear_CTRLB_DIR_bit(const void *const hw)
{
	((Tcc *)hw)->CTRLBCLR.reg = TCC_CTRLBSET_DIR;
}

static inline void hri_tcc_set_CTRLB_LUPD_bit(const void *const hw)
{
	((Tcc *)hw)->CTRLBSET.reg = TCC_CTRLBSET_LUPD;
}

static inline bool hri_tcc_get_CTRLB_LUPD_bit(const void *const hw)
{
	return (((Tcc *)hw)->CTRLBSET.reg & TCC_CTRLBSET_LUPD) >> TCC_CTRLBSET_LUPD_Pos;
}

static inline void hri_tcc_write_CTRLB_LUPD_bit(const void *const hw, bool value)
{
	if (value == 0x0) {
		((Tcc *)hw)->CTRLBCLR.reg = TCC_CTRLBSET_LUPD;
	} else {
		((Tcc *)hw)->CTRLBSET.reg = TCC_CTRLBSET_LUPD;
	}
}

static inline void hri_tcc_clear_CTRLB_LUPD_bit(const void *const hw)
{
	((Tcc *)hw)->CTRLBCLR.reg = TCC_CTRLBSET_LUPD;
}

static inline void hri_tcc_set_CTRLB_ONESHOT_bit(const void *const hw)
{
	((Tcc *)hw)->CTRLBSET.reg = TCC_CTRLBSET_ONESHOT;
}

static inline bool hri_tcc_get_CTRLB_ONESHOT_bit(const void *const hw)
{
	return (((Tcc *)hw)->CTRLBSET.reg & TCC_CTRLBSET_ONESHOT) >> TCC_CTRLBSET_ONESHOT_Pos;
}

static inline void hri_tcc_write_CTRLB_ONESHOT_bit(const void *const hw, bool value)
{
	if (value == 0x0) {
		((Tcc *)hw)->CTRLBCLR.reg = TCC_CTRLBSET_ONESHOT;
	} else {
		((Tcc *)hw)->CTRLBSET.reg = TCC_CTRLBSET_ONESHOT;
	}
}

static inline void hri_tcc_clear_CTRLB_ONESHOT_bit(const void *const hw)
{
	((Tcc *)hw)->CTRLBCLR.reg = TCC_CTRLBSET_ONESHOT;
}

static inline void hri_tcc_set_CTRLB_IDXCMD_bf(const void *const hw, hri_tcc_ctrlbset_reg_t mask)
{
	((Tcc *)hw)->CTRLBSET.reg = TCC_CTRLBSET_IDXCMD(mask);
}

static inline hri_tcc_ctrlbset_reg_t hri_tcc_get_CTRLB_IDXCMD_bf(const void *const hw, hri_tcc_ctrlbset_reg_t mask)
{
	uint8_t tmp;
	tmp = ((Tcc *)hw)->CTRLBSET.reg;
	tmp = (tmp & TCC_CTRLBSET_IDXCMD(mask)) >> TCC_CTRLBSET_IDXCMD_Pos;
	return tmp;
}

static inline hri_tcc_ctrlbset_reg_t hri_tcc_read_CTRLB_IDXCMD_bf(const void *const hw)
{
	uint8_t tmp;
	tmp = ((Tcc *)hw)->CTRLBSET.reg;
	tmp = (tmp & TCC_CTRLBSET_IDXCMD_Msk) >> TCC_CTRLBSET_IDXCMD_Pos;
	return tmp;
}

static inline void hri_tcc_write_CTRLB_IDXCMD_bf(const void *const hw, hri_tcc_ctrlbset_reg_t data)
{
	((Tcc *)hw)->CTRLBSET.reg = TCC_CTRLBSET_IDXCMD(data);
	((Tcc *)hw)->CTRLBCLR.reg = ~TCC_CTRLBSET_IDXCMD(data);
}

static inline void hri_tcc_clear_CTRLB_IDXCMD_bf(const void *const hw, hri_tcc_ctrlbset_reg_t mask)
{
	((Tcc *)hw)->CTRLBCLR.reg = TCC_CTRLBSET_IDXCMD(mask);
}

static inline void hri_tcc_set_CTRLB_CMD_bf(const void *const hw, hri_tcc_ctrlbset_reg_t mask)
{
	((Tcc *)hw)->CTRLBSET.reg = TCC_CTRLBSET_CMD(mask);
}

static inline hri_tcc_ctrlbset_reg_t hri_tcc_get_CTRLB_CMD_bf(const void *const hw, hri_tcc_ctrlbset_reg_t mask)
{
	uint8_t tmp;
	tmp = ((Tcc *)hw)->CTRLBSET.reg;
	tmp = (tmp & TCC_CTRLBSET_CMD(mask)) >> TCC_CTRLBSET_CMD_Pos;
	return tmp;
}

static inline hri_tcc_ctrlbset_reg_t hri_tcc_read_CTRLB_CMD_bf(const void *const hw)
{
	uint8_t tmp;
	tmp = ((Tcc *)hw)->CTRLBSET.reg;
	tmp = (tmp & TCC_CTRLBSET_CMD_Msk) >> TCC_CTRLBSET_CMD_Pos;
	return tmp;
}

static inline void hri_tcc_write_CTRLB_CMD_bf(const void *const hw, hri_tcc_ctrlbset_reg_t data)
{
	((Tcc *)hw)->CTRLBSET.reg = TCC_CTRLBSET_CMD(data);
	((Tcc *)hw)->CTRLBCLR.reg = ~TCC_CTRLBSET_CMD(data);
}

static inline void hri_tcc_clear_CTRLB_CMD_bf(const void *const hw, hri_tcc_ctrlbset_reg_t mask)
{
	((Tcc *)hw)->CTRLBCLR.reg = TCC_CTRLBSET_CMD(mask);
}

static inline void hri_tcc_set_CTRLB_reg(const void *const hw, hri_tcc_ctrlbset_reg_t mask)
{
	((Tcc *)hw)->CTRLBSET.reg = mask;
}

static inline hri_tcc_ctrlbset_reg_t hri_tcc_get_CTRLB_reg(const void *const hw, hri_tcc_ctrlbset_reg_t mask)
{
	uint8_t tmp;
	tmp = ((Tcc *)hw)->CTRLBSET.reg;
	tmp &= mask;
	return tmp;
}

static inline hri_tcc_ctrlbset_reg_t hri_tcc_read_CTRLB_reg(const void *const hw)
{
	return ((Tcc *)hw)->CTRLBSET.reg;
}

static inline void hri_tcc_write_CTRLB_reg(const void *const hw, hri_tcc_ctrlbset_reg_t data)
{
	((Tcc *)hw)->CTRLBSET.reg = data;
	((Tcc *)hw)->CTRLBCLR.reg = ~data;
}

static inline void hri_tcc_clear_CTRLB_reg(const void *const hw, hri_tcc_ctrlbset_reg_t mask)
{
	((Tcc *)hw)->CTRLBCLR.reg = mask;
}

static inline void hri_tcc_set_INTEN_OVF_bit(const void *const hw)
{
	((Tcc *)hw)->INTENSET.reg = TCC_INTENSET_OVF;
}

static inline bool hri_tcc_get_INTEN_OVF_bit(const void *const hw)
{
	return (((Tcc *)hw)->INTENSET.reg & TCC_INTENSET_OVF) >> TCC_INTENSET_OVF_Pos;
}

static inline void hri_tcc_write_INTEN_OVF_bit(const void *const hw, bool value)
{
	if (value == 0x0) {
		((Tcc *)hw)->INTENCLR.reg = TCC_INTENSET_OVF;
	} else {
		((Tcc *)hw)->INTENSET.reg = TCC_INTENSET_OVF;
	}
}

static inline void hri_tcc_clear_INTEN_OVF_bit(const void *const hw)
{
	((Tcc *)hw)->INTENCLR.reg = TCC_INTENSET_OVF;
}

static inline void hri_tcc_set_INTEN_TRG_bit(const void *const hw)
{
	((Tcc *)hw)->INTENSET.reg = TCC_INTENSET_TRG;
}

static inline bool hri_tcc_get_INTEN_TRG_bit(const void *const hw)
{
	return (((Tcc *)hw)->INTENSET.reg & TCC_INTENSET_TRG) >> TCC_INTENSET_TRG_Pos;
}

static inline void hri_tcc_write_INTEN_TRG_bit(const void *const hw, bool value)
{
	if (value == 0x0) {
		((Tcc *)hw)->INTENCLR.reg = TCC_INTENSET_TRG;
	} else {
		((Tcc *)hw)->INTENSET.reg = TCC_INTENSET_TRG;
	}
}

static inline void hri_tcc_clear_INTEN_TRG_bit(const void *const hw)
{
	((Tcc *)hw)->INTENCLR.reg = TCC_INTENSET_TRG;
}

static inline void hri_tcc_set_INTEN_CNT_bit(const void *const hw)
{
	((Tcc *)hw)->INTENSET.reg = TCC_INTENSET_CNT;
}

static inline bool hri_tcc_get_INTEN_CNT_bit(const void *const hw)
{
	return (((Tcc *)hw)->INTENSET.reg & TCC_INTENSET_CNT) >> TCC_INTENSET_CNT_Pos;
}

static inline void hri_tcc_write_INTEN_CNT_bit(const void *const hw, bool value)
{
	if (value == 0x0) {
		((Tcc *)hw)->INTENCLR.reg = TCC_INTENSET_CNT;
	} else {
		((Tcc *)hw)->INTENSET.reg = TCC_INTENSET_CNT;
	}
}

static inline void hri_tcc_clear_INTEN_CNT_bit(const void *const hw)
{
	((Tcc *)hw)->INTENCLR.reg = TCC_INTENSET_CNT;
}

static inline void hri_tcc_set_INTEN_ERR_bit(const void *const hw)
{
	((Tcc *)hw)->INTENSET.reg = TCC_INTENSET_ERR;
}

static inline bool hri_tcc_get_INTEN_ERR_bit(const void *const hw)
{
	return (((Tcc *)hw)->INTENSET.reg & TCC_INTENSET_ERR) >> TCC_INTENSET_ERR_Pos;
}

static inline void hri_tcc_write_INTEN_ERR_bit(const void *const hw, bool value)
{
	if (value == 0x0) {
		((Tcc *)hw)->INTENCLR.reg = TCC_INTENSET_ERR;
	} else {
		((Tcc *)hw)->INTENSET.reg = TCC_INTENSET_ERR;
	}
}

static inline void hri_tcc_clear_INTEN_ERR_bit(const void *const hw)
{
	((Tcc *)hw)->INTENCLR.reg = TCC_INTENSET_ERR;
}

static inline void hri_tcc_set_INTEN_DFS_bit(const void *const hw)
{
	((Tcc *)hw)->INTENSET.reg = TCC_INTENSET_DFS;
}

static inline bool hri_tcc_get_INTEN_DFS_bit(const void *const hw)
{
	return (((Tcc *)hw)->INTENSET.reg & TCC_INTENSET_DFS) >> TCC_INTENSET_DFS_Pos;
}

static inline void hri_tcc_write_INTEN_DFS_bit(const void *const hw, bool value)
{
	if (value == 0x0) {
		((Tcc *)hw)->INTENCLR.reg = TCC_INTENSET_DFS;
	} else {
		((Tcc *)hw)->INTENSET.reg = TCC_INTENSET_DFS;
	}
}

static inline void hri_tcc_clear_INTEN_DFS_bit(const void *const hw)
{
	((Tcc *)hw)->INTENCLR.reg = TCC_INTENSET_DFS;
}

static inline void hri_tcc_set_INTEN_FAULTA_bit(const void *const hw)
{
	((Tcc *)hw)->INTENSET.reg = TCC_INTENSET_FAULTA;
}

static inline bool hri_tcc_get_INTEN_FAULTA_bit(const void *const hw)
{
	return (((Tcc *)hw)->INTENSET.reg & TCC_INTENSET_FAULTA) >> TCC_INTENSET_FAULTA_Pos;
}

static inline void hri_tcc_write_INTEN_FAULTA_bit(const void *const hw, bool value)
{
	if (value == 0x0) {
		((Tcc *)hw)->INTENCLR.reg = TCC_INTENSET_FAULTA;
	} else {
		((Tcc *)hw)->INTENSET.reg = TCC_INTENSET_FAULTA;
	}
}

static inline void hri_tcc_clear_INTEN_FAULTA_bit(const void *const hw)
{
	((Tcc *)hw)->INTENCLR.reg = TCC_INTENSET_FAULTA;
}

static inline void hri_tcc_set_INTEN_FAULTB_bit(const void *const hw)
{
	((Tcc *)hw)->INTENSET.reg = TCC_INTENSET_FAULTB;
}

static inline bool hri_tcc_get_INTEN_FAULTB_bit(const void *const hw)
{
	return (((Tcc *)hw)->INTENSET.reg & TCC_INTENSET_FAULTB) >> TCC_INTENSET_FAULTB_Pos;
}

static inline void hri_tcc_write_INTEN_FAULTB_bit(const void *const hw, bool value)
{
	if (value == 0x0) {
		((Tcc *)hw)->INTENCLR.reg = TCC_INTENSET_FAULTB;
	} else {
		((Tcc *)hw)->INTENSET.reg = TCC_INTENSET_FAULTB;
	}
}

static inline void hri_tcc_clear_INTEN_FAULTB_bit(const void *const hw)
{
	((Tcc *)hw)->INTENCLR.reg = TCC_INTENSET_FAULTB;
}

static inline void hri_tcc_set_INTEN_FAULT0_bit(const void *const hw)
{
	((Tcc *)hw)->INTENSET.reg = TCC_INTENSET_FAULT0;
}

static inline bool hri_tcc_get_INTEN_FAULT0_bit(const void *const hw)
{
	return (((Tcc *)hw)->INTENSET.reg & TCC_INTENSET_FAULT0) >> TCC_INTENSET_FAULT0_Pos;
}

static inline void hri_tcc_write_INTEN_FAULT0_bit(const void *const hw, bool value)
{
	if (value == 0x0) {
		((Tcc *)hw)->INTENCLR.reg = TCC_INTENSET_FAULT0;
	} else {
		((Tcc *)hw)->INTENSET.reg = TCC_INTENSET_FAULT0;
	}
}

static inline void hri_tcc_clear_INTEN_FAULT0_bit(const void *const hw)
{
	((Tcc *)hw)->INTENCLR.reg = TCC_INTENSET_FAULT0;
}

static inline void hri_tcc_set_INTEN_FAULT1_bit(const void *const hw)
{
	((Tcc *)hw)->INTENSET.reg = TCC_INTENSET_FAULT1;
}

static inline bool hri_tcc_get_INTEN_FAULT1_bit(const void *const hw)
{
	return (((Tcc *)hw)->INTENSET.reg & TCC_INTENSET_FAULT1) >> TCC_INTENSET_FAULT1_Pos;
}

static inline void hri_tcc_write_INTEN_FAULT1_bit(const void *const hw, bool value)
{
	if (value == 0x0) {
		((Tcc *)hw)->INTENCLR.reg = TCC_INTENSET_FAULT1;
	} else {
		((Tcc *)hw)->INTENSET.reg = TCC_INTENSET_FAULT1;
	}
}

static inline void hri_tcc_clear_INTEN_FAULT1_bit(const void *const hw)
{
	((Tcc *)hw)->INTENCLR.reg = TCC_INTENSET_FAULT1;
}

static inline void hri_tcc_set_INTEN_MC0_bit(const void *const hw)
{
	((Tcc *)hw)->INTENSET.reg = TCC_INTENSET_MC0;
}

static inline bool hri_tcc_get_INTEN_MC0_bit(const void *const hw)
{
	return (((Tcc *)hw)->INTENSET.reg & TCC_INTENSET_MC0) >> TCC_INTENSET_MC0_Pos;
}

static inline void hri_tcc_write_INTEN_MC0_bit(const void *const hw, bool value)
{
	if (value == 0x0) {
		((Tcc *)hw)->INTENCLR.reg = TCC_INTENSET_MC0;
	} else {
		((Tcc *)hw)->INTENSET.reg = TCC_INTENSET_MC0;
	}
}

static inline void hri_tcc_clear_INTEN_MC0_bit(const void *const hw)
{
	((Tcc *)hw)->INTENCLR.reg = TCC_INTENSET_MC0;
}

static inline void hri_tcc_set_INTEN_MC1_bit(const void *const hw)
{
	((Tcc *)hw)->INTENSET.reg = TCC_INTENSET_MC1;
}

static inline bool hri_tcc_get_INTEN_MC1_bit(const void *const hw)
{
	return (((Tcc *)hw)->INTENSET.reg & TCC_INTENSET_MC1) >> TCC_INTENSET_MC1_Pos;
}

static inline void hri_tcc_write_INTEN_MC1_bit(const void *const hw, bool value)
{
	if (value == 0x0) {
		((Tcc *)hw)->INTENCLR.reg = TCC_INTENSET_MC1;
	} else {
		((Tcc *)hw)->INTENSET.reg = TCC_INTENSET_MC1;
	}
}

static inline void hri_tcc_clear_INTEN_MC1_bit(const void *const hw)
{
	((Tcc *)hw)->INTENCLR.reg = TCC_INTENSET_MC1;
}

static inline void hri_tcc_set_INTEN_MC2_bit(const void *const hw)
{
	((Tcc *)hw)->INTENSET.reg = TCC_INTENSET_MC2;
}

static inline bool hri_tcc_get_INTEN_MC2_bit(const void *const hw)
{
	return (((Tcc *)hw)->INTENSET.reg & TCC_INTENSET_MC2) >> TCC_INTENSET_MC2_Pos;
}

static inline void hri_tcc_write_INTEN_MC2_bit(const void *const hw, bool value)
{
	if (value == 0x0) {
		((Tcc *)hw)->INTENCLR.reg = TCC_INTENSET_MC2;
	} else {
		((Tcc *)hw)->INTENSET.reg = TCC_INTENSET_MC2;
	}
}

static inline void hri_tcc_clear_INTEN_MC2_bit(const void *const hw)
{
	((Tcc *)hw)->INTENCLR.reg = TCC_INTENSET_MC2;
}

static inline void hri_tcc_set_INTEN_MC3_bit(const void *const hw)
{
	((Tcc *)hw)->INTENSET.reg = TCC_INTENSET_MC3;
}

static inline bool hri_tcc_get_INTEN_MC3_bit(const void *const hw)
{
	return (((Tcc *)hw)->INTENSET.reg & TCC_INTENSET_MC3) >> TCC_INTENSET_MC3_Pos;
}

static inline void hri_tcc_write_INTEN_MC3_bit(const void *const hw, bool value)
{
	if (value == 0x0) {
		((Tcc *)hw)->INTENCLR.reg = TCC_INTENSET_MC3;
	} else {
		((Tcc *)hw)->INTENSET.reg = TCC_INTENSET_MC3;
	}
}

static inline void hri_tcc_clear_INTEN_MC3_bit(const void *const hw)
{
	((Tcc *)hw)->INTENCLR.reg = TCC_INTENSET_MC3;
}

static inline void hri_tcc_set_INTEN_reg(const void *const hw, hri_tcc_intenset_reg_t mask)
{
	((Tcc *)hw)->INTENSET.reg = mask;
}

static inline hri_tcc_intenset_reg_t hri_tcc_get_INTEN_reg(const void *const hw, hri_tcc_intenset_reg_t mask)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->INTENSET.reg;
	tmp &= mask;
	return tmp;
}

static inline hri_tcc_intenset_reg_t hri_tcc_read_INTEN_reg(const void *const hw)
{
	return ((Tcc *)hw)->INTENSET.reg;
}

static inline void hri_tcc_write_INTEN_reg(const void *const hw, hri_tcc_intenset_reg_t data)
{
	((Tcc *)hw)->INTENSET.reg = data;
	((Tcc *)hw)->INTENCLR.reg = ~data;
}

static inline void hri_tcc_clear_INTEN_reg(const void *const hw, hri_tcc_intenset_reg_t mask)
{
	((Tcc *)hw)->INTENCLR.reg = mask;
}

static inline bool hri_tcc_get_SYNCBUSY_SWRST_bit(const void *const hw)
{
	return (((Tcc *)hw)->SYNCBUSY.reg & TCC_SYNCBUSY_SWRST) >> TCC_SYNCBUSY_SWRST_Pos;
}

static inline bool hri_tcc_get_SYNCBUSY_ENABLE_bit(const void *const hw)
{
	return (((Tcc *)hw)->SYNCBUSY.reg & TCC_SYNCBUSY_ENABLE) >> TCC_SYNCBUSY_ENABLE_Pos;
}

static inline bool hri_tcc_get_SYNCBUSY_CTRLB_bit(const void *const hw)
{
	return (((Tcc *)hw)->SYNCBUSY.reg & TCC_SYNCBUSY_CTRLB) >> TCC_SYNCBUSY_CTRLB_Pos;
}

static inline bool hri_tcc_get_SYNCBUSY_STATUS_bit(const void *const hw)
{
	return (((Tcc *)hw)->SYNCBUSY.reg & TCC_SYNCBUSY_STATUS) >> TCC_SYNCBUSY_STATUS_Pos;
}

static inline bool hri_tcc_get_SYNCBUSY_COUNT_bit(const void *const hw)
{
	return (((Tcc *)hw)->SYNCBUSY.reg & TCC_SYNCBUSY_COUNT) >> TCC_SYNCBUSY_COUNT_Pos;
}

static inline bool hri_tcc_get_SYNCBUSY_PATT_bit(const void *const hw)
{
	return (((Tcc *)hw)->SYNCBUSY.reg & TCC_SYNCBUSY_PATT) >> TCC_SYNCBUSY_PATT_Pos;
}

static inline bool hri_tcc_get_SYNCBUSY_WAVE_bit(const void *const hw)
{
	return (((Tcc *)hw)->SYNCBUSY.reg & TCC_SYNCBUSY_WAVE) >> TCC_SYNCBUSY_WAVE_Pos;
}

static inline bool hri_tcc_get_SYNCBUSY_PER_bit(const void *const hw)
{
	return (((Tcc *)hw)->SYNCBUSY.reg & TCC_SYNCBUSY_PER) >> TCC_SYNCBUSY_PER_Pos;
}

static inline bool hri_tcc_get_SYNCBUSY_CC0_bit(const void *const hw)
{
	return (((Tcc *)hw)->SYNCBUSY.reg & TCC_SYNCBUSY_CC0) >> TCC_SYNCBUSY_CC0_Pos;
}

static inline bool hri_tcc_get_SYNCBUSY_CC1_bit(const void *const hw)
{
	return (((Tcc *)hw)->SYNCBUSY.reg & TCC_SYNCBUSY_CC1) >> TCC_SYNCBUSY_CC1_Pos;
}

static inline bool hri_tcc_get_SYNCBUSY_CC2_bit(const void *const hw)
{
	return (((Tcc *)hw)->SYNCBUSY.reg & TCC_SYNCBUSY_CC2) >> TCC_SYNCBUSY_CC2_Pos;
}

static inline bool hri_tcc_get_SYNCBUSY_CC3_bit(const void *const hw)
{
	return (((Tcc *)hw)->SYNCBUSY.reg & TCC_SYNCBUSY_CC3) >> TCC_SYNCBUSY_CC3_Pos;
}

static inline bool hri_tcc_get_SYNCBUSY_PATTB_bit(const void *const hw)
{
	return (((Tcc *)hw)->SYNCBUSY.reg & TCC_SYNCBUSY_PATTB) >> TCC_SYNCBUSY_PATTB_Pos;
}

static inline bool hri_tcc_get_SYNCBUSY_WAVEB_bit(const void *const hw)
{
	return (((Tcc *)hw)->SYNCBUSY.reg & TCC_SYNCBUSY_WAVEB) >> TCC_SYNCBUSY_WAVEB_Pos;
}

static inline bool hri_tcc_get_SYNCBUSY_PERB_bit(const void *const hw)
{
	return (((Tcc *)hw)->SYNCBUSY.reg & TCC_SYNCBUSY_PERB) >> TCC_SYNCBUSY_PERB_Pos;
}

static inline bool hri_tcc_get_SYNCBUSY_CCB0_bit(const void *const hw)
{
	return (((Tcc *)hw)->SYNCBUSY.reg & TCC_SYNCBUSY_CCB0) >> TCC_SYNCBUSY_CCB0_Pos;
}

static inline bool hri_tcc_get_SYNCBUSY_CCB1_bit(const void *const hw)
{
	return (((Tcc *)hw)->SYNCBUSY.reg & TCC_SYNCBUSY_CCB1) >> TCC_SYNCBUSY_CCB1_Pos;
}

static inline bool hri_tcc_get_SYNCBUSY_CCB2_bit(const void *const hw)
{
	return (((Tcc *)hw)->SYNCBUSY.reg & TCC_SYNCBUSY_CCB2) >> TCC_SYNCBUSY_CCB2_Pos;
}

static inline bool hri_tcc_get_SYNCBUSY_CCB3_bit(const void *const hw)
{
	return (((Tcc *)hw)->SYNCBUSY.reg & TCC_SYNCBUSY_CCB3) >> TCC_SYNCBUSY_CCB3_Pos;
}

static inline hri_tcc_syncbusy_reg_t hri_tcc_get_SYNCBUSY_reg(const void *const hw, hri_tcc_syncbusy_reg_t mask)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->SYNCBUSY.reg;
	tmp &= mask;
	return tmp;
}

static inline hri_tcc_syncbusy_reg_t hri_tcc_read_SYNCBUSY_reg(const void *const hw)
{
	return ((Tcc *)hw)->SYNCBUSY.reg;
}

static inline void hri_tcc_set_CTRLA_SWRST_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CTRLA.reg |= TCC_CTRLA_SWRST;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_SWRST);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_CTRLA_SWRST_bit(const void *const hw)
{
	uint32_t tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_SWRST);
	tmp = ((Tcc *)hw)->CTRLA.reg;
	tmp = (tmp & TCC_CTRLA_SWRST) >> TCC_CTRLA_SWRST_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_set_CTRLA_ENABLE_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CTRLA.reg |= TCC_CTRLA_ENABLE;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_SWRST | TCC_SYNCBUSY_ENABLE);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_CTRLA_ENABLE_bit(const void *const hw)
{
	uint32_t tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_SWRST | TCC_SYNCBUSY_ENABLE);
	tmp = ((Tcc *)hw)->CTRLA.reg;
	tmp = (tmp & TCC_CTRLA_ENABLE) >> TCC_CTRLA_ENABLE_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_CTRLA_ENABLE_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->CTRLA.reg;
	tmp &= ~TCC_CTRLA_ENABLE;
	tmp |= value << TCC_CTRLA_ENABLE_Pos;
	((Tcc *)hw)->CTRLA.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_SWRST | TCC_SYNCBUSY_ENABLE);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_CTRLA_ENABLE_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CTRLA.reg &= ~TCC_CTRLA_ENABLE;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_SWRST | TCC_SYNCBUSY_ENABLE);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_CTRLA_ENABLE_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CTRLA.reg ^= TCC_CTRLA_ENABLE;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_SWRST | TCC_SYNCBUSY_ENABLE);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_CTRLA_RUNSTDBY_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CTRLA.reg |= TCC_CTRLA_RUNSTDBY;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_CTRLA_RUNSTDBY_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->CTRLA.reg;
	tmp = (tmp & TCC_CTRLA_RUNSTDBY) >> TCC_CTRLA_RUNSTDBY_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_CTRLA_RUNSTDBY_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->CTRLA.reg;
	tmp &= ~TCC_CTRLA_RUNSTDBY;
	tmp |= value << TCC_CTRLA_RUNSTDBY_Pos;
	((Tcc *)hw)->CTRLA.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_CTRLA_RUNSTDBY_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CTRLA.reg &= ~TCC_CTRLA_RUNSTDBY;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_CTRLA_RUNSTDBY_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CTRLA.reg ^= TCC_CTRLA_RUNSTDBY;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_CTRLA_ALOCK_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CTRLA.reg |= TCC_CTRLA_ALOCK;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_CTRLA_ALOCK_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->CTRLA.reg;
	tmp = (tmp & TCC_CTRLA_ALOCK) >> TCC_CTRLA_ALOCK_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_CTRLA_ALOCK_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->CTRLA.reg;
	tmp &= ~TCC_CTRLA_ALOCK;
	tmp |= value << TCC_CTRLA_ALOCK_Pos;
	((Tcc *)hw)->CTRLA.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_CTRLA_ALOCK_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CTRLA.reg &= ~TCC_CTRLA_ALOCK;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_CTRLA_ALOCK_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CTRLA.reg ^= TCC_CTRLA_ALOCK;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_CTRLA_CPTEN0_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CTRLA.reg |= TCC_CTRLA_CPTEN0;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_CTRLA_CPTEN0_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->CTRLA.reg;
	tmp = (tmp & TCC_CTRLA_CPTEN0) >> TCC_CTRLA_CPTEN0_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_CTRLA_CPTEN0_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->CTRLA.reg;
	tmp &= ~TCC_CTRLA_CPTEN0;
	tmp |= value << TCC_CTRLA_CPTEN0_Pos;
	((Tcc *)hw)->CTRLA.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_CTRLA_CPTEN0_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CTRLA.reg &= ~TCC_CTRLA_CPTEN0;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_CTRLA_CPTEN0_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CTRLA.reg ^= TCC_CTRLA_CPTEN0;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_CTRLA_CPTEN1_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CTRLA.reg |= TCC_CTRLA_CPTEN1;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_CTRLA_CPTEN1_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->CTRLA.reg;
	tmp = (tmp & TCC_CTRLA_CPTEN1) >> TCC_CTRLA_CPTEN1_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_CTRLA_CPTEN1_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->CTRLA.reg;
	tmp &= ~TCC_CTRLA_CPTEN1;
	tmp |= value << TCC_CTRLA_CPTEN1_Pos;
	((Tcc *)hw)->CTRLA.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_CTRLA_CPTEN1_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CTRLA.reg &= ~TCC_CTRLA_CPTEN1;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_CTRLA_CPTEN1_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CTRLA.reg ^= TCC_CTRLA_CPTEN1;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_CTRLA_CPTEN2_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CTRLA.reg |= TCC_CTRLA_CPTEN2;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_CTRLA_CPTEN2_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->CTRLA.reg;
	tmp = (tmp & TCC_CTRLA_CPTEN2) >> TCC_CTRLA_CPTEN2_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_CTRLA_CPTEN2_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->CTRLA.reg;
	tmp &= ~TCC_CTRLA_CPTEN2;
	tmp |= value << TCC_CTRLA_CPTEN2_Pos;
	((Tcc *)hw)->CTRLA.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_CTRLA_CPTEN2_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CTRLA.reg &= ~TCC_CTRLA_CPTEN2;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_CTRLA_CPTEN2_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CTRLA.reg ^= TCC_CTRLA_CPTEN2;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_CTRLA_CPTEN3_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CTRLA.reg |= TCC_CTRLA_CPTEN3;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_CTRLA_CPTEN3_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->CTRLA.reg;
	tmp = (tmp & TCC_CTRLA_CPTEN3) >> TCC_CTRLA_CPTEN3_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_CTRLA_CPTEN3_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->CTRLA.reg;
	tmp &= ~TCC_CTRLA_CPTEN3;
	tmp |= value << TCC_CTRLA_CPTEN3_Pos;
	((Tcc *)hw)->CTRLA.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_CTRLA_CPTEN3_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CTRLA.reg &= ~TCC_CTRLA_CPTEN3;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_CTRLA_CPTEN3_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CTRLA.reg ^= TCC_CTRLA_CPTEN3;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_CTRLA_RESOLUTION_bf(const void *const hw, hri_tcc_ctrla_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CTRLA.reg |= TCC_CTRLA_RESOLUTION(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_ctrla_reg_t hri_tcc_get_CTRLA_RESOLUTION_bf(const void *const hw, hri_tcc_ctrla_reg_t mask)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->CTRLA.reg;
	tmp = (tmp & TCC_CTRLA_RESOLUTION(mask)) >> TCC_CTRLA_RESOLUTION_Pos;
	return tmp;
}

static inline void hri_tcc_write_CTRLA_RESOLUTION_bf(const void *const hw, hri_tcc_ctrla_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->CTRLA.reg;
	tmp &= ~TCC_CTRLA_RESOLUTION_Msk;
	tmp |= TCC_CTRLA_RESOLUTION(data);
	((Tcc *)hw)->CTRLA.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_CTRLA_RESOLUTION_bf(const void *const hw, hri_tcc_ctrla_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CTRLA.reg &= ~TCC_CTRLA_RESOLUTION(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_CTRLA_RESOLUTION_bf(const void *const hw, hri_tcc_ctrla_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CTRLA.reg ^= TCC_CTRLA_RESOLUTION(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_ctrla_reg_t hri_tcc_read_CTRLA_RESOLUTION_bf(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->CTRLA.reg;
	tmp = (tmp & TCC_CTRLA_RESOLUTION_Msk) >> TCC_CTRLA_RESOLUTION_Pos;
	return tmp;
}

static inline void hri_tcc_set_CTRLA_PRESCALER_bf(const void *const hw, hri_tcc_ctrla_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CTRLA.reg |= TCC_CTRLA_PRESCALER(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_ctrla_reg_t hri_tcc_get_CTRLA_PRESCALER_bf(const void *const hw, hri_tcc_ctrla_reg_t mask)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->CTRLA.reg;
	tmp = (tmp & TCC_CTRLA_PRESCALER(mask)) >> TCC_CTRLA_PRESCALER_Pos;
	return tmp;
}

static inline void hri_tcc_write_CTRLA_PRESCALER_bf(const void *const hw, hri_tcc_ctrla_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->CTRLA.reg;
	tmp &= ~TCC_CTRLA_PRESCALER_Msk;
	tmp |= TCC_CTRLA_PRESCALER(data);
	((Tcc *)hw)->CTRLA.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_CTRLA_PRESCALER_bf(const void *const hw, hri_tcc_ctrla_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CTRLA.reg &= ~TCC_CTRLA_PRESCALER(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_CTRLA_PRESCALER_bf(const void *const hw, hri_tcc_ctrla_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CTRLA.reg ^= TCC_CTRLA_PRESCALER(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_ctrla_reg_t hri_tcc_read_CTRLA_PRESCALER_bf(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->CTRLA.reg;
	tmp = (tmp & TCC_CTRLA_PRESCALER_Msk) >> TCC_CTRLA_PRESCALER_Pos;
	return tmp;
}

static inline void hri_tcc_set_CTRLA_PRESCSYNC_bf(const void *const hw, hri_tcc_ctrla_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CTRLA.reg |= TCC_CTRLA_PRESCSYNC(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_ctrla_reg_t hri_tcc_get_CTRLA_PRESCSYNC_bf(const void *const hw, hri_tcc_ctrla_reg_t mask)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->CTRLA.reg;
	tmp = (tmp & TCC_CTRLA_PRESCSYNC(mask)) >> TCC_CTRLA_PRESCSYNC_Pos;
	return tmp;
}

static inline void hri_tcc_write_CTRLA_PRESCSYNC_bf(const void *const hw, hri_tcc_ctrla_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->CTRLA.reg;
	tmp &= ~TCC_CTRLA_PRESCSYNC_Msk;
	tmp |= TCC_CTRLA_PRESCSYNC(data);
	((Tcc *)hw)->CTRLA.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_CTRLA_PRESCSYNC_bf(const void *const hw, hri_tcc_ctrla_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CTRLA.reg &= ~TCC_CTRLA_PRESCSYNC(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_CTRLA_PRESCSYNC_bf(const void *const hw, hri_tcc_ctrla_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CTRLA.reg ^= TCC_CTRLA_PRESCSYNC(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_ctrla_reg_t hri_tcc_read_CTRLA_PRESCSYNC_bf(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->CTRLA.reg;
	tmp = (tmp & TCC_CTRLA_PRESCSYNC_Msk) >> TCC_CTRLA_PRESCSYNC_Pos;
	return tmp;
}

static inline void hri_tcc_set_CTRLA_reg(const void *const hw, hri_tcc_ctrla_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CTRLA.reg |= mask;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_SWRST | TCC_SYNCBUSY_ENABLE);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_ctrla_reg_t hri_tcc_get_CTRLA_reg(const void *const hw, hri_tcc_ctrla_reg_t mask)
{
	uint32_t tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_SWRST | TCC_SYNCBUSY_ENABLE);
	tmp = ((Tcc *)hw)->CTRLA.reg;
	tmp &= mask;
	return tmp;
}

static inline void hri_tcc_write_CTRLA_reg(const void *const hw, hri_tcc_ctrla_reg_t data)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CTRLA.reg = data;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_SWRST | TCC_SYNCBUSY_ENABLE);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_CTRLA_reg(const void *const hw, hri_tcc_ctrla_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CTRLA.reg &= ~mask;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_SWRST | TCC_SYNCBUSY_ENABLE);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_CTRLA_reg(const void *const hw, hri_tcc_ctrla_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CTRLA.reg ^= mask;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_SWRST | TCC_SYNCBUSY_ENABLE);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_ctrla_reg_t hri_tcc_read_CTRLA_reg(const void *const hw)
{
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_SWRST | TCC_SYNCBUSY_ENABLE);
	return ((Tcc *)hw)->CTRLA.reg;
}

static inline void hri_tcc_set_FCTRLA_KEEP_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLA.reg |= TCC_FCTRLA_KEEP;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_FCTRLA_KEEP_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->FCTRLA.reg;
	tmp = (tmp & TCC_FCTRLA_KEEP) >> TCC_FCTRLA_KEEP_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_FCTRLA_KEEP_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->FCTRLA.reg;
	tmp &= ~TCC_FCTRLA_KEEP;
	tmp |= value << TCC_FCTRLA_KEEP_Pos;
	((Tcc *)hw)->FCTRLA.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_FCTRLA_KEEP_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLA.reg &= ~TCC_FCTRLA_KEEP;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_FCTRLA_KEEP_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLA.reg ^= TCC_FCTRLA_KEEP;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_FCTRLA_QUAL_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLA.reg |= TCC_FCTRLA_QUAL;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_FCTRLA_QUAL_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->FCTRLA.reg;
	tmp = (tmp & TCC_FCTRLA_QUAL) >> TCC_FCTRLA_QUAL_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_FCTRLA_QUAL_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->FCTRLA.reg;
	tmp &= ~TCC_FCTRLA_QUAL;
	tmp |= value << TCC_FCTRLA_QUAL_Pos;
	((Tcc *)hw)->FCTRLA.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_FCTRLA_QUAL_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLA.reg &= ~TCC_FCTRLA_QUAL;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_FCTRLA_QUAL_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLA.reg ^= TCC_FCTRLA_QUAL;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_FCTRLA_RESTART_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLA.reg |= TCC_FCTRLA_RESTART;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_FCTRLA_RESTART_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->FCTRLA.reg;
	tmp = (tmp & TCC_FCTRLA_RESTART) >> TCC_FCTRLA_RESTART_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_FCTRLA_RESTART_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->FCTRLA.reg;
	tmp &= ~TCC_FCTRLA_RESTART;
	tmp |= value << TCC_FCTRLA_RESTART_Pos;
	((Tcc *)hw)->FCTRLA.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_FCTRLA_RESTART_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLA.reg &= ~TCC_FCTRLA_RESTART;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_FCTRLA_RESTART_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLA.reg ^= TCC_FCTRLA_RESTART;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_FCTRLA_SRC_bf(const void *const hw, hri_tcc_fctrla_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLA.reg |= TCC_FCTRLA_SRC(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_fctrla_reg_t hri_tcc_get_FCTRLA_SRC_bf(const void *const hw, hri_tcc_fctrla_reg_t mask)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->FCTRLA.reg;
	tmp = (tmp & TCC_FCTRLA_SRC(mask)) >> TCC_FCTRLA_SRC_Pos;
	return tmp;
}

static inline void hri_tcc_write_FCTRLA_SRC_bf(const void *const hw, hri_tcc_fctrla_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->FCTRLA.reg;
	tmp &= ~TCC_FCTRLA_SRC_Msk;
	tmp |= TCC_FCTRLA_SRC(data);
	((Tcc *)hw)->FCTRLA.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_FCTRLA_SRC_bf(const void *const hw, hri_tcc_fctrla_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLA.reg &= ~TCC_FCTRLA_SRC(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_FCTRLA_SRC_bf(const void *const hw, hri_tcc_fctrla_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLA.reg ^= TCC_FCTRLA_SRC(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_fctrla_reg_t hri_tcc_read_FCTRLA_SRC_bf(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->FCTRLA.reg;
	tmp = (tmp & TCC_FCTRLA_SRC_Msk) >> TCC_FCTRLA_SRC_Pos;
	return tmp;
}

static inline void hri_tcc_set_FCTRLA_BLANK_bf(const void *const hw, hri_tcc_fctrla_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLA.reg |= TCC_FCTRLA_BLANK(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_fctrla_reg_t hri_tcc_get_FCTRLA_BLANK_bf(const void *const hw, hri_tcc_fctrla_reg_t mask)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->FCTRLA.reg;
	tmp = (tmp & TCC_FCTRLA_BLANK(mask)) >> TCC_FCTRLA_BLANK_Pos;
	return tmp;
}

static inline void hri_tcc_write_FCTRLA_BLANK_bf(const void *const hw, hri_tcc_fctrla_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->FCTRLA.reg;
	tmp &= ~TCC_FCTRLA_BLANK_Msk;
	tmp |= TCC_FCTRLA_BLANK(data);
	((Tcc *)hw)->FCTRLA.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_FCTRLA_BLANK_bf(const void *const hw, hri_tcc_fctrla_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLA.reg &= ~TCC_FCTRLA_BLANK(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_FCTRLA_BLANK_bf(const void *const hw, hri_tcc_fctrla_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLA.reg ^= TCC_FCTRLA_BLANK(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_fctrla_reg_t hri_tcc_read_FCTRLA_BLANK_bf(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->FCTRLA.reg;
	tmp = (tmp & TCC_FCTRLA_BLANK_Msk) >> TCC_FCTRLA_BLANK_Pos;
	return tmp;
}

static inline void hri_tcc_set_FCTRLA_HALT_bf(const void *const hw, hri_tcc_fctrla_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLA.reg |= TCC_FCTRLA_HALT(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_fctrla_reg_t hri_tcc_get_FCTRLA_HALT_bf(const void *const hw, hri_tcc_fctrla_reg_t mask)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->FCTRLA.reg;
	tmp = (tmp & TCC_FCTRLA_HALT(mask)) >> TCC_FCTRLA_HALT_Pos;
	return tmp;
}

static inline void hri_tcc_write_FCTRLA_HALT_bf(const void *const hw, hri_tcc_fctrla_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->FCTRLA.reg;
	tmp &= ~TCC_FCTRLA_HALT_Msk;
	tmp |= TCC_FCTRLA_HALT(data);
	((Tcc *)hw)->FCTRLA.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_FCTRLA_HALT_bf(const void *const hw, hri_tcc_fctrla_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLA.reg &= ~TCC_FCTRLA_HALT(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_FCTRLA_HALT_bf(const void *const hw, hri_tcc_fctrla_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLA.reg ^= TCC_FCTRLA_HALT(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_fctrla_reg_t hri_tcc_read_FCTRLA_HALT_bf(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->FCTRLA.reg;
	tmp = (tmp & TCC_FCTRLA_HALT_Msk) >> TCC_FCTRLA_HALT_Pos;
	return tmp;
}

static inline void hri_tcc_set_FCTRLA_CHSEL_bf(const void *const hw, hri_tcc_fctrla_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLA.reg |= TCC_FCTRLA_CHSEL(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_fctrla_reg_t hri_tcc_get_FCTRLA_CHSEL_bf(const void *const hw, hri_tcc_fctrla_reg_t mask)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->FCTRLA.reg;
	tmp = (tmp & TCC_FCTRLA_CHSEL(mask)) >> TCC_FCTRLA_CHSEL_Pos;
	return tmp;
}

static inline void hri_tcc_write_FCTRLA_CHSEL_bf(const void *const hw, hri_tcc_fctrla_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->FCTRLA.reg;
	tmp &= ~TCC_FCTRLA_CHSEL_Msk;
	tmp |= TCC_FCTRLA_CHSEL(data);
	((Tcc *)hw)->FCTRLA.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_FCTRLA_CHSEL_bf(const void *const hw, hri_tcc_fctrla_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLA.reg &= ~TCC_FCTRLA_CHSEL(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_FCTRLA_CHSEL_bf(const void *const hw, hri_tcc_fctrla_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLA.reg ^= TCC_FCTRLA_CHSEL(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_fctrla_reg_t hri_tcc_read_FCTRLA_CHSEL_bf(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->FCTRLA.reg;
	tmp = (tmp & TCC_FCTRLA_CHSEL_Msk) >> TCC_FCTRLA_CHSEL_Pos;
	return tmp;
}

static inline void hri_tcc_set_FCTRLA_CAPTURE_bf(const void *const hw, hri_tcc_fctrla_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLA.reg |= TCC_FCTRLA_CAPTURE(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_fctrla_reg_t hri_tcc_get_FCTRLA_CAPTURE_bf(const void *const hw, hri_tcc_fctrla_reg_t mask)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->FCTRLA.reg;
	tmp = (tmp & TCC_FCTRLA_CAPTURE(mask)) >> TCC_FCTRLA_CAPTURE_Pos;
	return tmp;
}

static inline void hri_tcc_write_FCTRLA_CAPTURE_bf(const void *const hw, hri_tcc_fctrla_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->FCTRLA.reg;
	tmp &= ~TCC_FCTRLA_CAPTURE_Msk;
	tmp |= TCC_FCTRLA_CAPTURE(data);
	((Tcc *)hw)->FCTRLA.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_FCTRLA_CAPTURE_bf(const void *const hw, hri_tcc_fctrla_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLA.reg &= ~TCC_FCTRLA_CAPTURE(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_FCTRLA_CAPTURE_bf(const void *const hw, hri_tcc_fctrla_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLA.reg ^= TCC_FCTRLA_CAPTURE(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_fctrla_reg_t hri_tcc_read_FCTRLA_CAPTURE_bf(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->FCTRLA.reg;
	tmp = (tmp & TCC_FCTRLA_CAPTURE_Msk) >> TCC_FCTRLA_CAPTURE_Pos;
	return tmp;
}

static inline void hri_tcc_set_FCTRLA_BLANKVAL_bf(const void *const hw, hri_tcc_fctrla_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLA.reg |= TCC_FCTRLA_BLANKVAL(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_fctrla_reg_t hri_tcc_get_FCTRLA_BLANKVAL_bf(const void *const hw, hri_tcc_fctrla_reg_t mask)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->FCTRLA.reg;
	tmp = (tmp & TCC_FCTRLA_BLANKVAL(mask)) >> TCC_FCTRLA_BLANKVAL_Pos;
	return tmp;
}

static inline void hri_tcc_write_FCTRLA_BLANKVAL_bf(const void *const hw, hri_tcc_fctrla_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->FCTRLA.reg;
	tmp &= ~TCC_FCTRLA_BLANKVAL_Msk;
	tmp |= TCC_FCTRLA_BLANKVAL(data);
	((Tcc *)hw)->FCTRLA.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_FCTRLA_BLANKVAL_bf(const void *const hw, hri_tcc_fctrla_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLA.reg &= ~TCC_FCTRLA_BLANKVAL(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_FCTRLA_BLANKVAL_bf(const void *const hw, hri_tcc_fctrla_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLA.reg ^= TCC_FCTRLA_BLANKVAL(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_fctrla_reg_t hri_tcc_read_FCTRLA_BLANKVAL_bf(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->FCTRLA.reg;
	tmp = (tmp & TCC_FCTRLA_BLANKVAL_Msk) >> TCC_FCTRLA_BLANKVAL_Pos;
	return tmp;
}

static inline void hri_tcc_set_FCTRLA_FILTERVAL_bf(const void *const hw, hri_tcc_fctrla_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLA.reg |= TCC_FCTRLA_FILTERVAL(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_fctrla_reg_t hri_tcc_get_FCTRLA_FILTERVAL_bf(const void *const hw, hri_tcc_fctrla_reg_t mask)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->FCTRLA.reg;
	tmp = (tmp & TCC_FCTRLA_FILTERVAL(mask)) >> TCC_FCTRLA_FILTERVAL_Pos;
	return tmp;
}

static inline void hri_tcc_write_FCTRLA_FILTERVAL_bf(const void *const hw, hri_tcc_fctrla_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->FCTRLA.reg;
	tmp &= ~TCC_FCTRLA_FILTERVAL_Msk;
	tmp |= TCC_FCTRLA_FILTERVAL(data);
	((Tcc *)hw)->FCTRLA.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_FCTRLA_FILTERVAL_bf(const void *const hw, hri_tcc_fctrla_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLA.reg &= ~TCC_FCTRLA_FILTERVAL(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_FCTRLA_FILTERVAL_bf(const void *const hw, hri_tcc_fctrla_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLA.reg ^= TCC_FCTRLA_FILTERVAL(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_fctrla_reg_t hri_tcc_read_FCTRLA_FILTERVAL_bf(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->FCTRLA.reg;
	tmp = (tmp & TCC_FCTRLA_FILTERVAL_Msk) >> TCC_FCTRLA_FILTERVAL_Pos;
	return tmp;
}

static inline void hri_tcc_set_FCTRLA_reg(const void *const hw, hri_tcc_fctrla_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLA.reg |= mask;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_fctrla_reg_t hri_tcc_get_FCTRLA_reg(const void *const hw, hri_tcc_fctrla_reg_t mask)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->FCTRLA.reg;
	tmp &= mask;
	return tmp;
}

static inline void hri_tcc_write_FCTRLA_reg(const void *const hw, hri_tcc_fctrla_reg_t data)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLA.reg = data;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_FCTRLA_reg(const void *const hw, hri_tcc_fctrla_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLA.reg &= ~mask;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_FCTRLA_reg(const void *const hw, hri_tcc_fctrla_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLA.reg ^= mask;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_fctrla_reg_t hri_tcc_read_FCTRLA_reg(const void *const hw)
{
	return ((Tcc *)hw)->FCTRLA.reg;
}

static inline void hri_tcc_set_FCTRLB_KEEP_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLB.reg |= TCC_FCTRLB_KEEP;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_FCTRLB_KEEP_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->FCTRLB.reg;
	tmp = (tmp & TCC_FCTRLB_KEEP) >> TCC_FCTRLB_KEEP_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_FCTRLB_KEEP_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->FCTRLB.reg;
	tmp &= ~TCC_FCTRLB_KEEP;
	tmp |= value << TCC_FCTRLB_KEEP_Pos;
	((Tcc *)hw)->FCTRLB.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_FCTRLB_KEEP_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLB.reg &= ~TCC_FCTRLB_KEEP;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_FCTRLB_KEEP_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLB.reg ^= TCC_FCTRLB_KEEP;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_FCTRLB_QUAL_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLB.reg |= TCC_FCTRLB_QUAL;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_FCTRLB_QUAL_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->FCTRLB.reg;
	tmp = (tmp & TCC_FCTRLB_QUAL) >> TCC_FCTRLB_QUAL_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_FCTRLB_QUAL_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->FCTRLB.reg;
	tmp &= ~TCC_FCTRLB_QUAL;
	tmp |= value << TCC_FCTRLB_QUAL_Pos;
	((Tcc *)hw)->FCTRLB.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_FCTRLB_QUAL_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLB.reg &= ~TCC_FCTRLB_QUAL;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_FCTRLB_QUAL_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLB.reg ^= TCC_FCTRLB_QUAL;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_FCTRLB_RESTART_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLB.reg |= TCC_FCTRLB_RESTART;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_FCTRLB_RESTART_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->FCTRLB.reg;
	tmp = (tmp & TCC_FCTRLB_RESTART) >> TCC_FCTRLB_RESTART_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_FCTRLB_RESTART_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->FCTRLB.reg;
	tmp &= ~TCC_FCTRLB_RESTART;
	tmp |= value << TCC_FCTRLB_RESTART_Pos;
	((Tcc *)hw)->FCTRLB.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_FCTRLB_RESTART_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLB.reg &= ~TCC_FCTRLB_RESTART;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_FCTRLB_RESTART_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLB.reg ^= TCC_FCTRLB_RESTART;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_FCTRLB_SRC_bf(const void *const hw, hri_tcc_fctrlb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLB.reg |= TCC_FCTRLB_SRC(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_fctrlb_reg_t hri_tcc_get_FCTRLB_SRC_bf(const void *const hw, hri_tcc_fctrlb_reg_t mask)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->FCTRLB.reg;
	tmp = (tmp & TCC_FCTRLB_SRC(mask)) >> TCC_FCTRLB_SRC_Pos;
	return tmp;
}

static inline void hri_tcc_write_FCTRLB_SRC_bf(const void *const hw, hri_tcc_fctrlb_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->FCTRLB.reg;
	tmp &= ~TCC_FCTRLB_SRC_Msk;
	tmp |= TCC_FCTRLB_SRC(data);
	((Tcc *)hw)->FCTRLB.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_FCTRLB_SRC_bf(const void *const hw, hri_tcc_fctrlb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLB.reg &= ~TCC_FCTRLB_SRC(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_FCTRLB_SRC_bf(const void *const hw, hri_tcc_fctrlb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLB.reg ^= TCC_FCTRLB_SRC(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_fctrlb_reg_t hri_tcc_read_FCTRLB_SRC_bf(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->FCTRLB.reg;
	tmp = (tmp & TCC_FCTRLB_SRC_Msk) >> TCC_FCTRLB_SRC_Pos;
	return tmp;
}

static inline void hri_tcc_set_FCTRLB_BLANK_bf(const void *const hw, hri_tcc_fctrlb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLB.reg |= TCC_FCTRLB_BLANK(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_fctrlb_reg_t hri_tcc_get_FCTRLB_BLANK_bf(const void *const hw, hri_tcc_fctrlb_reg_t mask)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->FCTRLB.reg;
	tmp = (tmp & TCC_FCTRLB_BLANK(mask)) >> TCC_FCTRLB_BLANK_Pos;
	return tmp;
}

static inline void hri_tcc_write_FCTRLB_BLANK_bf(const void *const hw, hri_tcc_fctrlb_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->FCTRLB.reg;
	tmp &= ~TCC_FCTRLB_BLANK_Msk;
	tmp |= TCC_FCTRLB_BLANK(data);
	((Tcc *)hw)->FCTRLB.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_FCTRLB_BLANK_bf(const void *const hw, hri_tcc_fctrlb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLB.reg &= ~TCC_FCTRLB_BLANK(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_FCTRLB_BLANK_bf(const void *const hw, hri_tcc_fctrlb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLB.reg ^= TCC_FCTRLB_BLANK(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_fctrlb_reg_t hri_tcc_read_FCTRLB_BLANK_bf(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->FCTRLB.reg;
	tmp = (tmp & TCC_FCTRLB_BLANK_Msk) >> TCC_FCTRLB_BLANK_Pos;
	return tmp;
}

static inline void hri_tcc_set_FCTRLB_HALT_bf(const void *const hw, hri_tcc_fctrlb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLB.reg |= TCC_FCTRLB_HALT(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_fctrlb_reg_t hri_tcc_get_FCTRLB_HALT_bf(const void *const hw, hri_tcc_fctrlb_reg_t mask)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->FCTRLB.reg;
	tmp = (tmp & TCC_FCTRLB_HALT(mask)) >> TCC_FCTRLB_HALT_Pos;
	return tmp;
}

static inline void hri_tcc_write_FCTRLB_HALT_bf(const void *const hw, hri_tcc_fctrlb_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->FCTRLB.reg;
	tmp &= ~TCC_FCTRLB_HALT_Msk;
	tmp |= TCC_FCTRLB_HALT(data);
	((Tcc *)hw)->FCTRLB.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_FCTRLB_HALT_bf(const void *const hw, hri_tcc_fctrlb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLB.reg &= ~TCC_FCTRLB_HALT(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_FCTRLB_HALT_bf(const void *const hw, hri_tcc_fctrlb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLB.reg ^= TCC_FCTRLB_HALT(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_fctrlb_reg_t hri_tcc_read_FCTRLB_HALT_bf(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->FCTRLB.reg;
	tmp = (tmp & TCC_FCTRLB_HALT_Msk) >> TCC_FCTRLB_HALT_Pos;
	return tmp;
}

static inline void hri_tcc_set_FCTRLB_CHSEL_bf(const void *const hw, hri_tcc_fctrlb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLB.reg |= TCC_FCTRLB_CHSEL(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_fctrlb_reg_t hri_tcc_get_FCTRLB_CHSEL_bf(const void *const hw, hri_tcc_fctrlb_reg_t mask)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->FCTRLB.reg;
	tmp = (tmp & TCC_FCTRLB_CHSEL(mask)) >> TCC_FCTRLB_CHSEL_Pos;
	return tmp;
}

static inline void hri_tcc_write_FCTRLB_CHSEL_bf(const void *const hw, hri_tcc_fctrlb_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->FCTRLB.reg;
	tmp &= ~TCC_FCTRLB_CHSEL_Msk;
	tmp |= TCC_FCTRLB_CHSEL(data);
	((Tcc *)hw)->FCTRLB.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_FCTRLB_CHSEL_bf(const void *const hw, hri_tcc_fctrlb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLB.reg &= ~TCC_FCTRLB_CHSEL(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_FCTRLB_CHSEL_bf(const void *const hw, hri_tcc_fctrlb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLB.reg ^= TCC_FCTRLB_CHSEL(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_fctrlb_reg_t hri_tcc_read_FCTRLB_CHSEL_bf(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->FCTRLB.reg;
	tmp = (tmp & TCC_FCTRLB_CHSEL_Msk) >> TCC_FCTRLB_CHSEL_Pos;
	return tmp;
}

static inline void hri_tcc_set_FCTRLB_CAPTURE_bf(const void *const hw, hri_tcc_fctrlb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLB.reg |= TCC_FCTRLB_CAPTURE(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_fctrlb_reg_t hri_tcc_get_FCTRLB_CAPTURE_bf(const void *const hw, hri_tcc_fctrlb_reg_t mask)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->FCTRLB.reg;
	tmp = (tmp & TCC_FCTRLB_CAPTURE(mask)) >> TCC_FCTRLB_CAPTURE_Pos;
	return tmp;
}

static inline void hri_tcc_write_FCTRLB_CAPTURE_bf(const void *const hw, hri_tcc_fctrlb_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->FCTRLB.reg;
	tmp &= ~TCC_FCTRLB_CAPTURE_Msk;
	tmp |= TCC_FCTRLB_CAPTURE(data);
	((Tcc *)hw)->FCTRLB.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_FCTRLB_CAPTURE_bf(const void *const hw, hri_tcc_fctrlb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLB.reg &= ~TCC_FCTRLB_CAPTURE(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_FCTRLB_CAPTURE_bf(const void *const hw, hri_tcc_fctrlb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLB.reg ^= TCC_FCTRLB_CAPTURE(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_fctrlb_reg_t hri_tcc_read_FCTRLB_CAPTURE_bf(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->FCTRLB.reg;
	tmp = (tmp & TCC_FCTRLB_CAPTURE_Msk) >> TCC_FCTRLB_CAPTURE_Pos;
	return tmp;
}

static inline void hri_tcc_set_FCTRLB_BLANKVAL_bf(const void *const hw, hri_tcc_fctrlb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLB.reg |= TCC_FCTRLB_BLANKVAL(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_fctrlb_reg_t hri_tcc_get_FCTRLB_BLANKVAL_bf(const void *const hw, hri_tcc_fctrlb_reg_t mask)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->FCTRLB.reg;
	tmp = (tmp & TCC_FCTRLB_BLANKVAL(mask)) >> TCC_FCTRLB_BLANKVAL_Pos;
	return tmp;
}

static inline void hri_tcc_write_FCTRLB_BLANKVAL_bf(const void *const hw, hri_tcc_fctrlb_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->FCTRLB.reg;
	tmp &= ~TCC_FCTRLB_BLANKVAL_Msk;
	tmp |= TCC_FCTRLB_BLANKVAL(data);
	((Tcc *)hw)->FCTRLB.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_FCTRLB_BLANKVAL_bf(const void *const hw, hri_tcc_fctrlb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLB.reg &= ~TCC_FCTRLB_BLANKVAL(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_FCTRLB_BLANKVAL_bf(const void *const hw, hri_tcc_fctrlb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLB.reg ^= TCC_FCTRLB_BLANKVAL(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_fctrlb_reg_t hri_tcc_read_FCTRLB_BLANKVAL_bf(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->FCTRLB.reg;
	tmp = (tmp & TCC_FCTRLB_BLANKVAL_Msk) >> TCC_FCTRLB_BLANKVAL_Pos;
	return tmp;
}

static inline void hri_tcc_set_FCTRLB_FILTERVAL_bf(const void *const hw, hri_tcc_fctrlb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLB.reg |= TCC_FCTRLB_FILTERVAL(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_fctrlb_reg_t hri_tcc_get_FCTRLB_FILTERVAL_bf(const void *const hw, hri_tcc_fctrlb_reg_t mask)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->FCTRLB.reg;
	tmp = (tmp & TCC_FCTRLB_FILTERVAL(mask)) >> TCC_FCTRLB_FILTERVAL_Pos;
	return tmp;
}

static inline void hri_tcc_write_FCTRLB_FILTERVAL_bf(const void *const hw, hri_tcc_fctrlb_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->FCTRLB.reg;
	tmp &= ~TCC_FCTRLB_FILTERVAL_Msk;
	tmp |= TCC_FCTRLB_FILTERVAL(data);
	((Tcc *)hw)->FCTRLB.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_FCTRLB_FILTERVAL_bf(const void *const hw, hri_tcc_fctrlb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLB.reg &= ~TCC_FCTRLB_FILTERVAL(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_FCTRLB_FILTERVAL_bf(const void *const hw, hri_tcc_fctrlb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLB.reg ^= TCC_FCTRLB_FILTERVAL(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_fctrlb_reg_t hri_tcc_read_FCTRLB_FILTERVAL_bf(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->FCTRLB.reg;
	tmp = (tmp & TCC_FCTRLB_FILTERVAL_Msk) >> TCC_FCTRLB_FILTERVAL_Pos;
	return tmp;
}

static inline void hri_tcc_set_FCTRLB_reg(const void *const hw, hri_tcc_fctrlb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLB.reg |= mask;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_fctrlb_reg_t hri_tcc_get_FCTRLB_reg(const void *const hw, hri_tcc_fctrlb_reg_t mask)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->FCTRLB.reg;
	tmp &= mask;
	return tmp;
}

static inline void hri_tcc_write_FCTRLB_reg(const void *const hw, hri_tcc_fctrlb_reg_t data)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLB.reg = data;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_FCTRLB_reg(const void *const hw, hri_tcc_fctrlb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLB.reg &= ~mask;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_FCTRLB_reg(const void *const hw, hri_tcc_fctrlb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->FCTRLB.reg ^= mask;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_fctrlb_reg_t hri_tcc_read_FCTRLB_reg(const void *const hw)
{
	return ((Tcc *)hw)->FCTRLB.reg;
}

static inline void hri_tcc_set_WEXCTRL_DTIEN0_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WEXCTRL.reg |= TCC_WEXCTRL_DTIEN0;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_WEXCTRL_DTIEN0_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->WEXCTRL.reg;
	tmp = (tmp & TCC_WEXCTRL_DTIEN0) >> TCC_WEXCTRL_DTIEN0_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_WEXCTRL_DTIEN0_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->WEXCTRL.reg;
	tmp &= ~TCC_WEXCTRL_DTIEN0;
	tmp |= value << TCC_WEXCTRL_DTIEN0_Pos;
	((Tcc *)hw)->WEXCTRL.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_WEXCTRL_DTIEN0_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WEXCTRL.reg &= ~TCC_WEXCTRL_DTIEN0;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_WEXCTRL_DTIEN0_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WEXCTRL.reg ^= TCC_WEXCTRL_DTIEN0;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_WEXCTRL_DTIEN1_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WEXCTRL.reg |= TCC_WEXCTRL_DTIEN1;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_WEXCTRL_DTIEN1_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->WEXCTRL.reg;
	tmp = (tmp & TCC_WEXCTRL_DTIEN1) >> TCC_WEXCTRL_DTIEN1_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_WEXCTRL_DTIEN1_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->WEXCTRL.reg;
	tmp &= ~TCC_WEXCTRL_DTIEN1;
	tmp |= value << TCC_WEXCTRL_DTIEN1_Pos;
	((Tcc *)hw)->WEXCTRL.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_WEXCTRL_DTIEN1_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WEXCTRL.reg &= ~TCC_WEXCTRL_DTIEN1;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_WEXCTRL_DTIEN1_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WEXCTRL.reg ^= TCC_WEXCTRL_DTIEN1;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_WEXCTRL_DTIEN2_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WEXCTRL.reg |= TCC_WEXCTRL_DTIEN2;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_WEXCTRL_DTIEN2_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->WEXCTRL.reg;
	tmp = (tmp & TCC_WEXCTRL_DTIEN2) >> TCC_WEXCTRL_DTIEN2_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_WEXCTRL_DTIEN2_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->WEXCTRL.reg;
	tmp &= ~TCC_WEXCTRL_DTIEN2;
	tmp |= value << TCC_WEXCTRL_DTIEN2_Pos;
	((Tcc *)hw)->WEXCTRL.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_WEXCTRL_DTIEN2_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WEXCTRL.reg &= ~TCC_WEXCTRL_DTIEN2;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_WEXCTRL_DTIEN2_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WEXCTRL.reg ^= TCC_WEXCTRL_DTIEN2;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_WEXCTRL_DTIEN3_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WEXCTRL.reg |= TCC_WEXCTRL_DTIEN3;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_WEXCTRL_DTIEN3_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->WEXCTRL.reg;
	tmp = (tmp & TCC_WEXCTRL_DTIEN3) >> TCC_WEXCTRL_DTIEN3_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_WEXCTRL_DTIEN3_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->WEXCTRL.reg;
	tmp &= ~TCC_WEXCTRL_DTIEN3;
	tmp |= value << TCC_WEXCTRL_DTIEN3_Pos;
	((Tcc *)hw)->WEXCTRL.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_WEXCTRL_DTIEN3_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WEXCTRL.reg &= ~TCC_WEXCTRL_DTIEN3;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_WEXCTRL_DTIEN3_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WEXCTRL.reg ^= TCC_WEXCTRL_DTIEN3;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_WEXCTRL_OTMX_bf(const void *const hw, hri_tcc_wexctrl_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WEXCTRL.reg |= TCC_WEXCTRL_OTMX(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_wexctrl_reg_t hri_tcc_get_WEXCTRL_OTMX_bf(const void *const hw, hri_tcc_wexctrl_reg_t mask)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->WEXCTRL.reg;
	tmp = (tmp & TCC_WEXCTRL_OTMX(mask)) >> TCC_WEXCTRL_OTMX_Pos;
	return tmp;
}

static inline void hri_tcc_write_WEXCTRL_OTMX_bf(const void *const hw, hri_tcc_wexctrl_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->WEXCTRL.reg;
	tmp &= ~TCC_WEXCTRL_OTMX_Msk;
	tmp |= TCC_WEXCTRL_OTMX(data);
	((Tcc *)hw)->WEXCTRL.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_WEXCTRL_OTMX_bf(const void *const hw, hri_tcc_wexctrl_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WEXCTRL.reg &= ~TCC_WEXCTRL_OTMX(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_WEXCTRL_OTMX_bf(const void *const hw, hri_tcc_wexctrl_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WEXCTRL.reg ^= TCC_WEXCTRL_OTMX(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_wexctrl_reg_t hri_tcc_read_WEXCTRL_OTMX_bf(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->WEXCTRL.reg;
	tmp = (tmp & TCC_WEXCTRL_OTMX_Msk) >> TCC_WEXCTRL_OTMX_Pos;
	return tmp;
}

static inline void hri_tcc_set_WEXCTRL_DTLS_bf(const void *const hw, hri_tcc_wexctrl_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WEXCTRL.reg |= TCC_WEXCTRL_DTLS(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_wexctrl_reg_t hri_tcc_get_WEXCTRL_DTLS_bf(const void *const hw, hri_tcc_wexctrl_reg_t mask)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->WEXCTRL.reg;
	tmp = (tmp & TCC_WEXCTRL_DTLS(mask)) >> TCC_WEXCTRL_DTLS_Pos;
	return tmp;
}

static inline void hri_tcc_write_WEXCTRL_DTLS_bf(const void *const hw, hri_tcc_wexctrl_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->WEXCTRL.reg;
	tmp &= ~TCC_WEXCTRL_DTLS_Msk;
	tmp |= TCC_WEXCTRL_DTLS(data);
	((Tcc *)hw)->WEXCTRL.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_WEXCTRL_DTLS_bf(const void *const hw, hri_tcc_wexctrl_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WEXCTRL.reg &= ~TCC_WEXCTRL_DTLS(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_WEXCTRL_DTLS_bf(const void *const hw, hri_tcc_wexctrl_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WEXCTRL.reg ^= TCC_WEXCTRL_DTLS(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_wexctrl_reg_t hri_tcc_read_WEXCTRL_DTLS_bf(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->WEXCTRL.reg;
	tmp = (tmp & TCC_WEXCTRL_DTLS_Msk) >> TCC_WEXCTRL_DTLS_Pos;
	return tmp;
}

static inline void hri_tcc_set_WEXCTRL_DTHS_bf(const void *const hw, hri_tcc_wexctrl_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WEXCTRL.reg |= TCC_WEXCTRL_DTHS(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_wexctrl_reg_t hri_tcc_get_WEXCTRL_DTHS_bf(const void *const hw, hri_tcc_wexctrl_reg_t mask)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->WEXCTRL.reg;
	tmp = (tmp & TCC_WEXCTRL_DTHS(mask)) >> TCC_WEXCTRL_DTHS_Pos;
	return tmp;
}

static inline void hri_tcc_write_WEXCTRL_DTHS_bf(const void *const hw, hri_tcc_wexctrl_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->WEXCTRL.reg;
	tmp &= ~TCC_WEXCTRL_DTHS_Msk;
	tmp |= TCC_WEXCTRL_DTHS(data);
	((Tcc *)hw)->WEXCTRL.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_WEXCTRL_DTHS_bf(const void *const hw, hri_tcc_wexctrl_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WEXCTRL.reg &= ~TCC_WEXCTRL_DTHS(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_WEXCTRL_DTHS_bf(const void *const hw, hri_tcc_wexctrl_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WEXCTRL.reg ^= TCC_WEXCTRL_DTHS(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_wexctrl_reg_t hri_tcc_read_WEXCTRL_DTHS_bf(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->WEXCTRL.reg;
	tmp = (tmp & TCC_WEXCTRL_DTHS_Msk) >> TCC_WEXCTRL_DTHS_Pos;
	return tmp;
}

static inline void hri_tcc_set_WEXCTRL_reg(const void *const hw, hri_tcc_wexctrl_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WEXCTRL.reg |= mask;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_wexctrl_reg_t hri_tcc_get_WEXCTRL_reg(const void *const hw, hri_tcc_wexctrl_reg_t mask)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->WEXCTRL.reg;
	tmp &= mask;
	return tmp;
}

static inline void hri_tcc_write_WEXCTRL_reg(const void *const hw, hri_tcc_wexctrl_reg_t data)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WEXCTRL.reg = data;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_WEXCTRL_reg(const void *const hw, hri_tcc_wexctrl_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WEXCTRL.reg &= ~mask;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_WEXCTRL_reg(const void *const hw, hri_tcc_wexctrl_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WEXCTRL.reg ^= mask;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_wexctrl_reg_t hri_tcc_read_WEXCTRL_reg(const void *const hw)
{
	return ((Tcc *)hw)->WEXCTRL.reg;
}

static inline void hri_tcc_set_DRVCTRL_NRE0_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg |= TCC_DRVCTRL_NRE0;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_DRVCTRL_NRE0_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->DRVCTRL.reg;
	tmp = (tmp & TCC_DRVCTRL_NRE0) >> TCC_DRVCTRL_NRE0_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_DRVCTRL_NRE0_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->DRVCTRL.reg;
	tmp &= ~TCC_DRVCTRL_NRE0;
	tmp |= value << TCC_DRVCTRL_NRE0_Pos;
	((Tcc *)hw)->DRVCTRL.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_DRVCTRL_NRE0_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg &= ~TCC_DRVCTRL_NRE0;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_DRVCTRL_NRE0_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg ^= TCC_DRVCTRL_NRE0;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_DRVCTRL_NRE1_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg |= TCC_DRVCTRL_NRE1;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_DRVCTRL_NRE1_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->DRVCTRL.reg;
	tmp = (tmp & TCC_DRVCTRL_NRE1) >> TCC_DRVCTRL_NRE1_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_DRVCTRL_NRE1_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->DRVCTRL.reg;
	tmp &= ~TCC_DRVCTRL_NRE1;
	tmp |= value << TCC_DRVCTRL_NRE1_Pos;
	((Tcc *)hw)->DRVCTRL.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_DRVCTRL_NRE1_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg &= ~TCC_DRVCTRL_NRE1;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_DRVCTRL_NRE1_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg ^= TCC_DRVCTRL_NRE1;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_DRVCTRL_NRE2_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg |= TCC_DRVCTRL_NRE2;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_DRVCTRL_NRE2_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->DRVCTRL.reg;
	tmp = (tmp & TCC_DRVCTRL_NRE2) >> TCC_DRVCTRL_NRE2_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_DRVCTRL_NRE2_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->DRVCTRL.reg;
	tmp &= ~TCC_DRVCTRL_NRE2;
	tmp |= value << TCC_DRVCTRL_NRE2_Pos;
	((Tcc *)hw)->DRVCTRL.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_DRVCTRL_NRE2_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg &= ~TCC_DRVCTRL_NRE2;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_DRVCTRL_NRE2_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg ^= TCC_DRVCTRL_NRE2;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_DRVCTRL_NRE3_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg |= TCC_DRVCTRL_NRE3;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_DRVCTRL_NRE3_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->DRVCTRL.reg;
	tmp = (tmp & TCC_DRVCTRL_NRE3) >> TCC_DRVCTRL_NRE3_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_DRVCTRL_NRE3_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->DRVCTRL.reg;
	tmp &= ~TCC_DRVCTRL_NRE3;
	tmp |= value << TCC_DRVCTRL_NRE3_Pos;
	((Tcc *)hw)->DRVCTRL.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_DRVCTRL_NRE3_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg &= ~TCC_DRVCTRL_NRE3;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_DRVCTRL_NRE3_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg ^= TCC_DRVCTRL_NRE3;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_DRVCTRL_NRE4_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg |= TCC_DRVCTRL_NRE4;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_DRVCTRL_NRE4_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->DRVCTRL.reg;
	tmp = (tmp & TCC_DRVCTRL_NRE4) >> TCC_DRVCTRL_NRE4_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_DRVCTRL_NRE4_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->DRVCTRL.reg;
	tmp &= ~TCC_DRVCTRL_NRE4;
	tmp |= value << TCC_DRVCTRL_NRE4_Pos;
	((Tcc *)hw)->DRVCTRL.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_DRVCTRL_NRE4_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg &= ~TCC_DRVCTRL_NRE4;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_DRVCTRL_NRE4_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg ^= TCC_DRVCTRL_NRE4;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_DRVCTRL_NRE5_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg |= TCC_DRVCTRL_NRE5;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_DRVCTRL_NRE5_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->DRVCTRL.reg;
	tmp = (tmp & TCC_DRVCTRL_NRE5) >> TCC_DRVCTRL_NRE5_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_DRVCTRL_NRE5_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->DRVCTRL.reg;
	tmp &= ~TCC_DRVCTRL_NRE5;
	tmp |= value << TCC_DRVCTRL_NRE5_Pos;
	((Tcc *)hw)->DRVCTRL.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_DRVCTRL_NRE5_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg &= ~TCC_DRVCTRL_NRE5;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_DRVCTRL_NRE5_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg ^= TCC_DRVCTRL_NRE5;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_DRVCTRL_NRE6_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg |= TCC_DRVCTRL_NRE6;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_DRVCTRL_NRE6_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->DRVCTRL.reg;
	tmp = (tmp & TCC_DRVCTRL_NRE6) >> TCC_DRVCTRL_NRE6_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_DRVCTRL_NRE6_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->DRVCTRL.reg;
	tmp &= ~TCC_DRVCTRL_NRE6;
	tmp |= value << TCC_DRVCTRL_NRE6_Pos;
	((Tcc *)hw)->DRVCTRL.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_DRVCTRL_NRE6_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg &= ~TCC_DRVCTRL_NRE6;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_DRVCTRL_NRE6_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg ^= TCC_DRVCTRL_NRE6;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_DRVCTRL_NRE7_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg |= TCC_DRVCTRL_NRE7;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_DRVCTRL_NRE7_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->DRVCTRL.reg;
	tmp = (tmp & TCC_DRVCTRL_NRE7) >> TCC_DRVCTRL_NRE7_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_DRVCTRL_NRE7_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->DRVCTRL.reg;
	tmp &= ~TCC_DRVCTRL_NRE7;
	tmp |= value << TCC_DRVCTRL_NRE7_Pos;
	((Tcc *)hw)->DRVCTRL.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_DRVCTRL_NRE7_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg &= ~TCC_DRVCTRL_NRE7;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_DRVCTRL_NRE7_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg ^= TCC_DRVCTRL_NRE7;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_DRVCTRL_NRV0_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg |= TCC_DRVCTRL_NRV0;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_DRVCTRL_NRV0_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->DRVCTRL.reg;
	tmp = (tmp & TCC_DRVCTRL_NRV0) >> TCC_DRVCTRL_NRV0_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_DRVCTRL_NRV0_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->DRVCTRL.reg;
	tmp &= ~TCC_DRVCTRL_NRV0;
	tmp |= value << TCC_DRVCTRL_NRV0_Pos;
	((Tcc *)hw)->DRVCTRL.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_DRVCTRL_NRV0_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg &= ~TCC_DRVCTRL_NRV0;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_DRVCTRL_NRV0_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg ^= TCC_DRVCTRL_NRV0;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_DRVCTRL_NRV1_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg |= TCC_DRVCTRL_NRV1;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_DRVCTRL_NRV1_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->DRVCTRL.reg;
	tmp = (tmp & TCC_DRVCTRL_NRV1) >> TCC_DRVCTRL_NRV1_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_DRVCTRL_NRV1_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->DRVCTRL.reg;
	tmp &= ~TCC_DRVCTRL_NRV1;
	tmp |= value << TCC_DRVCTRL_NRV1_Pos;
	((Tcc *)hw)->DRVCTRL.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_DRVCTRL_NRV1_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg &= ~TCC_DRVCTRL_NRV1;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_DRVCTRL_NRV1_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg ^= TCC_DRVCTRL_NRV1;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_DRVCTRL_NRV2_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg |= TCC_DRVCTRL_NRV2;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_DRVCTRL_NRV2_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->DRVCTRL.reg;
	tmp = (tmp & TCC_DRVCTRL_NRV2) >> TCC_DRVCTRL_NRV2_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_DRVCTRL_NRV2_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->DRVCTRL.reg;
	tmp &= ~TCC_DRVCTRL_NRV2;
	tmp |= value << TCC_DRVCTRL_NRV2_Pos;
	((Tcc *)hw)->DRVCTRL.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_DRVCTRL_NRV2_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg &= ~TCC_DRVCTRL_NRV2;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_DRVCTRL_NRV2_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg ^= TCC_DRVCTRL_NRV2;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_DRVCTRL_NRV3_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg |= TCC_DRVCTRL_NRV3;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_DRVCTRL_NRV3_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->DRVCTRL.reg;
	tmp = (tmp & TCC_DRVCTRL_NRV3) >> TCC_DRVCTRL_NRV3_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_DRVCTRL_NRV3_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->DRVCTRL.reg;
	tmp &= ~TCC_DRVCTRL_NRV3;
	tmp |= value << TCC_DRVCTRL_NRV3_Pos;
	((Tcc *)hw)->DRVCTRL.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_DRVCTRL_NRV3_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg &= ~TCC_DRVCTRL_NRV3;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_DRVCTRL_NRV3_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg ^= TCC_DRVCTRL_NRV3;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_DRVCTRL_NRV4_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg |= TCC_DRVCTRL_NRV4;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_DRVCTRL_NRV4_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->DRVCTRL.reg;
	tmp = (tmp & TCC_DRVCTRL_NRV4) >> TCC_DRVCTRL_NRV4_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_DRVCTRL_NRV4_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->DRVCTRL.reg;
	tmp &= ~TCC_DRVCTRL_NRV4;
	tmp |= value << TCC_DRVCTRL_NRV4_Pos;
	((Tcc *)hw)->DRVCTRL.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_DRVCTRL_NRV4_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg &= ~TCC_DRVCTRL_NRV4;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_DRVCTRL_NRV4_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg ^= TCC_DRVCTRL_NRV4;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_DRVCTRL_NRV5_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg |= TCC_DRVCTRL_NRV5;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_DRVCTRL_NRV5_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->DRVCTRL.reg;
	tmp = (tmp & TCC_DRVCTRL_NRV5) >> TCC_DRVCTRL_NRV5_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_DRVCTRL_NRV5_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->DRVCTRL.reg;
	tmp &= ~TCC_DRVCTRL_NRV5;
	tmp |= value << TCC_DRVCTRL_NRV5_Pos;
	((Tcc *)hw)->DRVCTRL.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_DRVCTRL_NRV5_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg &= ~TCC_DRVCTRL_NRV5;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_DRVCTRL_NRV5_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg ^= TCC_DRVCTRL_NRV5;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_DRVCTRL_NRV6_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg |= TCC_DRVCTRL_NRV6;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_DRVCTRL_NRV6_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->DRVCTRL.reg;
	tmp = (tmp & TCC_DRVCTRL_NRV6) >> TCC_DRVCTRL_NRV6_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_DRVCTRL_NRV6_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->DRVCTRL.reg;
	tmp &= ~TCC_DRVCTRL_NRV6;
	tmp |= value << TCC_DRVCTRL_NRV6_Pos;
	((Tcc *)hw)->DRVCTRL.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_DRVCTRL_NRV6_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg &= ~TCC_DRVCTRL_NRV6;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_DRVCTRL_NRV6_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg ^= TCC_DRVCTRL_NRV6;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_DRVCTRL_NRV7_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg |= TCC_DRVCTRL_NRV7;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_DRVCTRL_NRV7_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->DRVCTRL.reg;
	tmp = (tmp & TCC_DRVCTRL_NRV7) >> TCC_DRVCTRL_NRV7_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_DRVCTRL_NRV7_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->DRVCTRL.reg;
	tmp &= ~TCC_DRVCTRL_NRV7;
	tmp |= value << TCC_DRVCTRL_NRV7_Pos;
	((Tcc *)hw)->DRVCTRL.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_DRVCTRL_NRV7_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg &= ~TCC_DRVCTRL_NRV7;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_DRVCTRL_NRV7_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg ^= TCC_DRVCTRL_NRV7;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_DRVCTRL_INVEN0_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg |= TCC_DRVCTRL_INVEN0;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_DRVCTRL_INVEN0_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->DRVCTRL.reg;
	tmp = (tmp & TCC_DRVCTRL_INVEN0) >> TCC_DRVCTRL_INVEN0_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_DRVCTRL_INVEN0_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->DRVCTRL.reg;
	tmp &= ~TCC_DRVCTRL_INVEN0;
	tmp |= value << TCC_DRVCTRL_INVEN0_Pos;
	((Tcc *)hw)->DRVCTRL.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_DRVCTRL_INVEN0_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg &= ~TCC_DRVCTRL_INVEN0;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_DRVCTRL_INVEN0_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg ^= TCC_DRVCTRL_INVEN0;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_DRVCTRL_INVEN1_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg |= TCC_DRVCTRL_INVEN1;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_DRVCTRL_INVEN1_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->DRVCTRL.reg;
	tmp = (tmp & TCC_DRVCTRL_INVEN1) >> TCC_DRVCTRL_INVEN1_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_DRVCTRL_INVEN1_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->DRVCTRL.reg;
	tmp &= ~TCC_DRVCTRL_INVEN1;
	tmp |= value << TCC_DRVCTRL_INVEN1_Pos;
	((Tcc *)hw)->DRVCTRL.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_DRVCTRL_INVEN1_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg &= ~TCC_DRVCTRL_INVEN1;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_DRVCTRL_INVEN1_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg ^= TCC_DRVCTRL_INVEN1;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_DRVCTRL_INVEN2_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg |= TCC_DRVCTRL_INVEN2;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_DRVCTRL_INVEN2_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->DRVCTRL.reg;
	tmp = (tmp & TCC_DRVCTRL_INVEN2) >> TCC_DRVCTRL_INVEN2_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_DRVCTRL_INVEN2_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->DRVCTRL.reg;
	tmp &= ~TCC_DRVCTRL_INVEN2;
	tmp |= value << TCC_DRVCTRL_INVEN2_Pos;
	((Tcc *)hw)->DRVCTRL.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_DRVCTRL_INVEN2_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg &= ~TCC_DRVCTRL_INVEN2;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_DRVCTRL_INVEN2_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg ^= TCC_DRVCTRL_INVEN2;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_DRVCTRL_INVEN3_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg |= TCC_DRVCTRL_INVEN3;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_DRVCTRL_INVEN3_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->DRVCTRL.reg;
	tmp = (tmp & TCC_DRVCTRL_INVEN3) >> TCC_DRVCTRL_INVEN3_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_DRVCTRL_INVEN3_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->DRVCTRL.reg;
	tmp &= ~TCC_DRVCTRL_INVEN3;
	tmp |= value << TCC_DRVCTRL_INVEN3_Pos;
	((Tcc *)hw)->DRVCTRL.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_DRVCTRL_INVEN3_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg &= ~TCC_DRVCTRL_INVEN3;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_DRVCTRL_INVEN3_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg ^= TCC_DRVCTRL_INVEN3;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_DRVCTRL_INVEN4_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg |= TCC_DRVCTRL_INVEN4;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_DRVCTRL_INVEN4_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->DRVCTRL.reg;
	tmp = (tmp & TCC_DRVCTRL_INVEN4) >> TCC_DRVCTRL_INVEN4_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_DRVCTRL_INVEN4_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->DRVCTRL.reg;
	tmp &= ~TCC_DRVCTRL_INVEN4;
	tmp |= value << TCC_DRVCTRL_INVEN4_Pos;
	((Tcc *)hw)->DRVCTRL.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_DRVCTRL_INVEN4_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg &= ~TCC_DRVCTRL_INVEN4;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_DRVCTRL_INVEN4_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg ^= TCC_DRVCTRL_INVEN4;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_DRVCTRL_INVEN5_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg |= TCC_DRVCTRL_INVEN5;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_DRVCTRL_INVEN5_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->DRVCTRL.reg;
	tmp = (tmp & TCC_DRVCTRL_INVEN5) >> TCC_DRVCTRL_INVEN5_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_DRVCTRL_INVEN5_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->DRVCTRL.reg;
	tmp &= ~TCC_DRVCTRL_INVEN5;
	tmp |= value << TCC_DRVCTRL_INVEN5_Pos;
	((Tcc *)hw)->DRVCTRL.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_DRVCTRL_INVEN5_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg &= ~TCC_DRVCTRL_INVEN5;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_DRVCTRL_INVEN5_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg ^= TCC_DRVCTRL_INVEN5;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_DRVCTRL_INVEN6_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg |= TCC_DRVCTRL_INVEN6;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_DRVCTRL_INVEN6_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->DRVCTRL.reg;
	tmp = (tmp & TCC_DRVCTRL_INVEN6) >> TCC_DRVCTRL_INVEN6_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_DRVCTRL_INVEN6_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->DRVCTRL.reg;
	tmp &= ~TCC_DRVCTRL_INVEN6;
	tmp |= value << TCC_DRVCTRL_INVEN6_Pos;
	((Tcc *)hw)->DRVCTRL.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_DRVCTRL_INVEN6_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg &= ~TCC_DRVCTRL_INVEN6;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_DRVCTRL_INVEN6_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg ^= TCC_DRVCTRL_INVEN6;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_DRVCTRL_INVEN7_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg |= TCC_DRVCTRL_INVEN7;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_DRVCTRL_INVEN7_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->DRVCTRL.reg;
	tmp = (tmp & TCC_DRVCTRL_INVEN7) >> TCC_DRVCTRL_INVEN7_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_DRVCTRL_INVEN7_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->DRVCTRL.reg;
	tmp &= ~TCC_DRVCTRL_INVEN7;
	tmp |= value << TCC_DRVCTRL_INVEN7_Pos;
	((Tcc *)hw)->DRVCTRL.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_DRVCTRL_INVEN7_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg &= ~TCC_DRVCTRL_INVEN7;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_DRVCTRL_INVEN7_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg ^= TCC_DRVCTRL_INVEN7;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_DRVCTRL_FILTERVAL0_bf(const void *const hw, hri_tcc_drvctrl_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg |= TCC_DRVCTRL_FILTERVAL0(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_drvctrl_reg_t hri_tcc_get_DRVCTRL_FILTERVAL0_bf(const void *const hw, hri_tcc_drvctrl_reg_t mask)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->DRVCTRL.reg;
	tmp = (tmp & TCC_DRVCTRL_FILTERVAL0(mask)) >> TCC_DRVCTRL_FILTERVAL0_Pos;
	return tmp;
}

static inline void hri_tcc_write_DRVCTRL_FILTERVAL0_bf(const void *const hw, hri_tcc_drvctrl_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->DRVCTRL.reg;
	tmp &= ~TCC_DRVCTRL_FILTERVAL0_Msk;
	tmp |= TCC_DRVCTRL_FILTERVAL0(data);
	((Tcc *)hw)->DRVCTRL.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_DRVCTRL_FILTERVAL0_bf(const void *const hw, hri_tcc_drvctrl_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg &= ~TCC_DRVCTRL_FILTERVAL0(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_DRVCTRL_FILTERVAL0_bf(const void *const hw, hri_tcc_drvctrl_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg ^= TCC_DRVCTRL_FILTERVAL0(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_drvctrl_reg_t hri_tcc_read_DRVCTRL_FILTERVAL0_bf(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->DRVCTRL.reg;
	tmp = (tmp & TCC_DRVCTRL_FILTERVAL0_Msk) >> TCC_DRVCTRL_FILTERVAL0_Pos;
	return tmp;
}

static inline void hri_tcc_set_DRVCTRL_FILTERVAL1_bf(const void *const hw, hri_tcc_drvctrl_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg |= TCC_DRVCTRL_FILTERVAL1(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_drvctrl_reg_t hri_tcc_get_DRVCTRL_FILTERVAL1_bf(const void *const hw, hri_tcc_drvctrl_reg_t mask)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->DRVCTRL.reg;
	tmp = (tmp & TCC_DRVCTRL_FILTERVAL1(mask)) >> TCC_DRVCTRL_FILTERVAL1_Pos;
	return tmp;
}

static inline void hri_tcc_write_DRVCTRL_FILTERVAL1_bf(const void *const hw, hri_tcc_drvctrl_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->DRVCTRL.reg;
	tmp &= ~TCC_DRVCTRL_FILTERVAL1_Msk;
	tmp |= TCC_DRVCTRL_FILTERVAL1(data);
	((Tcc *)hw)->DRVCTRL.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_DRVCTRL_FILTERVAL1_bf(const void *const hw, hri_tcc_drvctrl_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg &= ~TCC_DRVCTRL_FILTERVAL1(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_DRVCTRL_FILTERVAL1_bf(const void *const hw, hri_tcc_drvctrl_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg ^= TCC_DRVCTRL_FILTERVAL1(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_drvctrl_reg_t hri_tcc_read_DRVCTRL_FILTERVAL1_bf(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->DRVCTRL.reg;
	tmp = (tmp & TCC_DRVCTRL_FILTERVAL1_Msk) >> TCC_DRVCTRL_FILTERVAL1_Pos;
	return tmp;
}

static inline void hri_tcc_set_DRVCTRL_reg(const void *const hw, hri_tcc_drvctrl_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg |= mask;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_drvctrl_reg_t hri_tcc_get_DRVCTRL_reg(const void *const hw, hri_tcc_drvctrl_reg_t mask)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->DRVCTRL.reg;
	tmp &= mask;
	return tmp;
}

static inline void hri_tcc_write_DRVCTRL_reg(const void *const hw, hri_tcc_drvctrl_reg_t data)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg = data;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_DRVCTRL_reg(const void *const hw, hri_tcc_drvctrl_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg &= ~mask;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_DRVCTRL_reg(const void *const hw, hri_tcc_drvctrl_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DRVCTRL.reg ^= mask;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_drvctrl_reg_t hri_tcc_read_DRVCTRL_reg(const void *const hw)
{
	return ((Tcc *)hw)->DRVCTRL.reg;
}

static inline void hri_tcc_set_DBGCTRL_DBGRUN_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DBGCTRL.reg |= TCC_DBGCTRL_DBGRUN;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_DBGCTRL_DBGRUN_bit(const void *const hw)
{
	uint8_t tmp;
	tmp = ((Tcc *)hw)->DBGCTRL.reg;
	tmp = (tmp & TCC_DBGCTRL_DBGRUN) >> TCC_DBGCTRL_DBGRUN_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_DBGCTRL_DBGRUN_bit(const void *const hw, bool value)
{
	uint8_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->DBGCTRL.reg;
	tmp &= ~TCC_DBGCTRL_DBGRUN;
	tmp |= value << TCC_DBGCTRL_DBGRUN_Pos;
	((Tcc *)hw)->DBGCTRL.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_DBGCTRL_DBGRUN_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DBGCTRL.reg &= ~TCC_DBGCTRL_DBGRUN;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_DBGCTRL_DBGRUN_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DBGCTRL.reg ^= TCC_DBGCTRL_DBGRUN;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_DBGCTRL_FDDBD_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DBGCTRL.reg |= TCC_DBGCTRL_FDDBD;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_DBGCTRL_FDDBD_bit(const void *const hw)
{
	uint8_t tmp;
	tmp = ((Tcc *)hw)->DBGCTRL.reg;
	tmp = (tmp & TCC_DBGCTRL_FDDBD) >> TCC_DBGCTRL_FDDBD_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_DBGCTRL_FDDBD_bit(const void *const hw, bool value)
{
	uint8_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->DBGCTRL.reg;
	tmp &= ~TCC_DBGCTRL_FDDBD;
	tmp |= value << TCC_DBGCTRL_FDDBD_Pos;
	((Tcc *)hw)->DBGCTRL.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_DBGCTRL_FDDBD_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DBGCTRL.reg &= ~TCC_DBGCTRL_FDDBD;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_DBGCTRL_FDDBD_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DBGCTRL.reg ^= TCC_DBGCTRL_FDDBD;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_DBGCTRL_reg(const void *const hw, hri_tcc_dbgctrl_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DBGCTRL.reg |= mask;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_dbgctrl_reg_t hri_tcc_get_DBGCTRL_reg(const void *const hw, hri_tcc_dbgctrl_reg_t mask)
{
	uint8_t tmp;
	tmp = ((Tcc *)hw)->DBGCTRL.reg;
	tmp &= mask;
	return tmp;
}

static inline void hri_tcc_write_DBGCTRL_reg(const void *const hw, hri_tcc_dbgctrl_reg_t data)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DBGCTRL.reg = data;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_DBGCTRL_reg(const void *const hw, hri_tcc_dbgctrl_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DBGCTRL.reg &= ~mask;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_DBGCTRL_reg(const void *const hw, hri_tcc_dbgctrl_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->DBGCTRL.reg ^= mask;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_dbgctrl_reg_t hri_tcc_read_DBGCTRL_reg(const void *const hw)
{
	return ((Tcc *)hw)->DBGCTRL.reg;
}

static inline void hri_tcc_set_EVCTRL_OVFEO_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->EVCTRL.reg |= TCC_EVCTRL_OVFEO;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_EVCTRL_OVFEO_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->EVCTRL.reg;
	tmp = (tmp & TCC_EVCTRL_OVFEO) >> TCC_EVCTRL_OVFEO_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_EVCTRL_OVFEO_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->EVCTRL.reg;
	tmp &= ~TCC_EVCTRL_OVFEO;
	tmp |= value << TCC_EVCTRL_OVFEO_Pos;
	((Tcc *)hw)->EVCTRL.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_EVCTRL_OVFEO_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->EVCTRL.reg &= ~TCC_EVCTRL_OVFEO;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_EVCTRL_OVFEO_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->EVCTRL.reg ^= TCC_EVCTRL_OVFEO;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_EVCTRL_TRGEO_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->EVCTRL.reg |= TCC_EVCTRL_TRGEO;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_EVCTRL_TRGEO_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->EVCTRL.reg;
	tmp = (tmp & TCC_EVCTRL_TRGEO) >> TCC_EVCTRL_TRGEO_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_EVCTRL_TRGEO_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->EVCTRL.reg;
	tmp &= ~TCC_EVCTRL_TRGEO;
	tmp |= value << TCC_EVCTRL_TRGEO_Pos;
	((Tcc *)hw)->EVCTRL.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_EVCTRL_TRGEO_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->EVCTRL.reg &= ~TCC_EVCTRL_TRGEO;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_EVCTRL_TRGEO_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->EVCTRL.reg ^= TCC_EVCTRL_TRGEO;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_EVCTRL_CNTEO_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->EVCTRL.reg |= TCC_EVCTRL_CNTEO;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_EVCTRL_CNTEO_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->EVCTRL.reg;
	tmp = (tmp & TCC_EVCTRL_CNTEO) >> TCC_EVCTRL_CNTEO_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_EVCTRL_CNTEO_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->EVCTRL.reg;
	tmp &= ~TCC_EVCTRL_CNTEO;
	tmp |= value << TCC_EVCTRL_CNTEO_Pos;
	((Tcc *)hw)->EVCTRL.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_EVCTRL_CNTEO_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->EVCTRL.reg &= ~TCC_EVCTRL_CNTEO;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_EVCTRL_CNTEO_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->EVCTRL.reg ^= TCC_EVCTRL_CNTEO;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_EVCTRL_TCINV0_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->EVCTRL.reg |= TCC_EVCTRL_TCINV0;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_EVCTRL_TCINV0_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->EVCTRL.reg;
	tmp = (tmp & TCC_EVCTRL_TCINV0) >> TCC_EVCTRL_TCINV0_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_EVCTRL_TCINV0_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->EVCTRL.reg;
	tmp &= ~TCC_EVCTRL_TCINV0;
	tmp |= value << TCC_EVCTRL_TCINV0_Pos;
	((Tcc *)hw)->EVCTRL.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_EVCTRL_TCINV0_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->EVCTRL.reg &= ~TCC_EVCTRL_TCINV0;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_EVCTRL_TCINV0_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->EVCTRL.reg ^= TCC_EVCTRL_TCINV0;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_EVCTRL_TCINV1_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->EVCTRL.reg |= TCC_EVCTRL_TCINV1;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_EVCTRL_TCINV1_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->EVCTRL.reg;
	tmp = (tmp & TCC_EVCTRL_TCINV1) >> TCC_EVCTRL_TCINV1_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_EVCTRL_TCINV1_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->EVCTRL.reg;
	tmp &= ~TCC_EVCTRL_TCINV1;
	tmp |= value << TCC_EVCTRL_TCINV1_Pos;
	((Tcc *)hw)->EVCTRL.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_EVCTRL_TCINV1_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->EVCTRL.reg &= ~TCC_EVCTRL_TCINV1;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_EVCTRL_TCINV1_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->EVCTRL.reg ^= TCC_EVCTRL_TCINV1;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_EVCTRL_TCEI0_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->EVCTRL.reg |= TCC_EVCTRL_TCEI0;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_EVCTRL_TCEI0_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->EVCTRL.reg;
	tmp = (tmp & TCC_EVCTRL_TCEI0) >> TCC_EVCTRL_TCEI0_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_EVCTRL_TCEI0_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->EVCTRL.reg;
	tmp &= ~TCC_EVCTRL_TCEI0;
	tmp |= value << TCC_EVCTRL_TCEI0_Pos;
	((Tcc *)hw)->EVCTRL.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_EVCTRL_TCEI0_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->EVCTRL.reg &= ~TCC_EVCTRL_TCEI0;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_EVCTRL_TCEI0_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->EVCTRL.reg ^= TCC_EVCTRL_TCEI0;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_EVCTRL_TCEI1_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->EVCTRL.reg |= TCC_EVCTRL_TCEI1;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_EVCTRL_TCEI1_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->EVCTRL.reg;
	tmp = (tmp & TCC_EVCTRL_TCEI1) >> TCC_EVCTRL_TCEI1_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_EVCTRL_TCEI1_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->EVCTRL.reg;
	tmp &= ~TCC_EVCTRL_TCEI1;
	tmp |= value << TCC_EVCTRL_TCEI1_Pos;
	((Tcc *)hw)->EVCTRL.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_EVCTRL_TCEI1_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->EVCTRL.reg &= ~TCC_EVCTRL_TCEI1;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_EVCTRL_TCEI1_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->EVCTRL.reg ^= TCC_EVCTRL_TCEI1;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_EVCTRL_MCEI0_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->EVCTRL.reg |= TCC_EVCTRL_MCEI0;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_EVCTRL_MCEI0_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->EVCTRL.reg;
	tmp = (tmp & TCC_EVCTRL_MCEI0) >> TCC_EVCTRL_MCEI0_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_EVCTRL_MCEI0_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->EVCTRL.reg;
	tmp &= ~TCC_EVCTRL_MCEI0;
	tmp |= value << TCC_EVCTRL_MCEI0_Pos;
	((Tcc *)hw)->EVCTRL.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_EVCTRL_MCEI0_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->EVCTRL.reg &= ~TCC_EVCTRL_MCEI0;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_EVCTRL_MCEI0_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->EVCTRL.reg ^= TCC_EVCTRL_MCEI0;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_EVCTRL_MCEI1_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->EVCTRL.reg |= TCC_EVCTRL_MCEI1;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_EVCTRL_MCEI1_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->EVCTRL.reg;
	tmp = (tmp & TCC_EVCTRL_MCEI1) >> TCC_EVCTRL_MCEI1_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_EVCTRL_MCEI1_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->EVCTRL.reg;
	tmp &= ~TCC_EVCTRL_MCEI1;
	tmp |= value << TCC_EVCTRL_MCEI1_Pos;
	((Tcc *)hw)->EVCTRL.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_EVCTRL_MCEI1_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->EVCTRL.reg &= ~TCC_EVCTRL_MCEI1;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_EVCTRL_MCEI1_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->EVCTRL.reg ^= TCC_EVCTRL_MCEI1;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_EVCTRL_MCEI2_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->EVCTRL.reg |= TCC_EVCTRL_MCEI2;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_EVCTRL_MCEI2_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->EVCTRL.reg;
	tmp = (tmp & TCC_EVCTRL_MCEI2) >> TCC_EVCTRL_MCEI2_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_EVCTRL_MCEI2_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->EVCTRL.reg;
	tmp &= ~TCC_EVCTRL_MCEI2;
	tmp |= value << TCC_EVCTRL_MCEI2_Pos;
	((Tcc *)hw)->EVCTRL.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_EVCTRL_MCEI2_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->EVCTRL.reg &= ~TCC_EVCTRL_MCEI2;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_EVCTRL_MCEI2_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->EVCTRL.reg ^= TCC_EVCTRL_MCEI2;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_EVCTRL_MCEI3_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->EVCTRL.reg |= TCC_EVCTRL_MCEI3;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_EVCTRL_MCEI3_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->EVCTRL.reg;
	tmp = (tmp & TCC_EVCTRL_MCEI3) >> TCC_EVCTRL_MCEI3_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_EVCTRL_MCEI3_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->EVCTRL.reg;
	tmp &= ~TCC_EVCTRL_MCEI3;
	tmp |= value << TCC_EVCTRL_MCEI3_Pos;
	((Tcc *)hw)->EVCTRL.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_EVCTRL_MCEI3_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->EVCTRL.reg &= ~TCC_EVCTRL_MCEI3;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_EVCTRL_MCEI3_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->EVCTRL.reg ^= TCC_EVCTRL_MCEI3;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_EVCTRL_MCEO0_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->EVCTRL.reg |= TCC_EVCTRL_MCEO0;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_EVCTRL_MCEO0_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->EVCTRL.reg;
	tmp = (tmp & TCC_EVCTRL_MCEO0) >> TCC_EVCTRL_MCEO0_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_EVCTRL_MCEO0_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->EVCTRL.reg;
	tmp &= ~TCC_EVCTRL_MCEO0;
	tmp |= value << TCC_EVCTRL_MCEO0_Pos;
	((Tcc *)hw)->EVCTRL.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_EVCTRL_MCEO0_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->EVCTRL.reg &= ~TCC_EVCTRL_MCEO0;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_EVCTRL_MCEO0_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->EVCTRL.reg ^= TCC_EVCTRL_MCEO0;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_EVCTRL_MCEO1_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->EVCTRL.reg |= TCC_EVCTRL_MCEO1;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_EVCTRL_MCEO1_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->EVCTRL.reg;
	tmp = (tmp & TCC_EVCTRL_MCEO1) >> TCC_EVCTRL_MCEO1_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_EVCTRL_MCEO1_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->EVCTRL.reg;
	tmp &= ~TCC_EVCTRL_MCEO1;
	tmp |= value << TCC_EVCTRL_MCEO1_Pos;
	((Tcc *)hw)->EVCTRL.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_EVCTRL_MCEO1_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->EVCTRL.reg &= ~TCC_EVCTRL_MCEO1;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_EVCTRL_MCEO1_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->EVCTRL.reg ^= TCC_EVCTRL_MCEO1;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_EVCTRL_MCEO2_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->EVCTRL.reg |= TCC_EVCTRL_MCEO2;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_EVCTRL_MCEO2_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->EVCTRL.reg;
	tmp = (tmp & TCC_EVCTRL_MCEO2) >> TCC_EVCTRL_MCEO2_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_EVCTRL_MCEO2_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->EVCTRL.reg;
	tmp &= ~TCC_EVCTRL_MCEO2;
	tmp |= value << TCC_EVCTRL_MCEO2_Pos;
	((Tcc *)hw)->EVCTRL.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_EVCTRL_MCEO2_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->EVCTRL.reg &= ~TCC_EVCTRL_MCEO2;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_EVCTRL_MCEO2_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->EVCTRL.reg ^= TCC_EVCTRL_MCEO2;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_EVCTRL_MCEO3_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->EVCTRL.reg |= TCC_EVCTRL_MCEO3;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_EVCTRL_MCEO3_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->EVCTRL.reg;
	tmp = (tmp & TCC_EVCTRL_MCEO3) >> TCC_EVCTRL_MCEO3_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_EVCTRL_MCEO3_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->EVCTRL.reg;
	tmp &= ~TCC_EVCTRL_MCEO3;
	tmp |= value << TCC_EVCTRL_MCEO3_Pos;
	((Tcc *)hw)->EVCTRL.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_EVCTRL_MCEO3_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->EVCTRL.reg &= ~TCC_EVCTRL_MCEO3;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_EVCTRL_MCEO3_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->EVCTRL.reg ^= TCC_EVCTRL_MCEO3;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_EVCTRL_EVACT0_bf(const void *const hw, hri_tcc_evctrl_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->EVCTRL.reg |= TCC_EVCTRL_EVACT0(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_evctrl_reg_t hri_tcc_get_EVCTRL_EVACT0_bf(const void *const hw, hri_tcc_evctrl_reg_t mask)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->EVCTRL.reg;
	tmp = (tmp & TCC_EVCTRL_EVACT0(mask)) >> TCC_EVCTRL_EVACT0_Pos;
	return tmp;
}

static inline void hri_tcc_write_EVCTRL_EVACT0_bf(const void *const hw, hri_tcc_evctrl_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->EVCTRL.reg;
	tmp &= ~TCC_EVCTRL_EVACT0_Msk;
	tmp |= TCC_EVCTRL_EVACT0(data);
	((Tcc *)hw)->EVCTRL.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_EVCTRL_EVACT0_bf(const void *const hw, hri_tcc_evctrl_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->EVCTRL.reg &= ~TCC_EVCTRL_EVACT0(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_EVCTRL_EVACT0_bf(const void *const hw, hri_tcc_evctrl_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->EVCTRL.reg ^= TCC_EVCTRL_EVACT0(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_evctrl_reg_t hri_tcc_read_EVCTRL_EVACT0_bf(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->EVCTRL.reg;
	tmp = (tmp & TCC_EVCTRL_EVACT0_Msk) >> TCC_EVCTRL_EVACT0_Pos;
	return tmp;
}

static inline void hri_tcc_set_EVCTRL_EVACT1_bf(const void *const hw, hri_tcc_evctrl_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->EVCTRL.reg |= TCC_EVCTRL_EVACT1(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_evctrl_reg_t hri_tcc_get_EVCTRL_EVACT1_bf(const void *const hw, hri_tcc_evctrl_reg_t mask)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->EVCTRL.reg;
	tmp = (tmp & TCC_EVCTRL_EVACT1(mask)) >> TCC_EVCTRL_EVACT1_Pos;
	return tmp;
}

static inline void hri_tcc_write_EVCTRL_EVACT1_bf(const void *const hw, hri_tcc_evctrl_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->EVCTRL.reg;
	tmp &= ~TCC_EVCTRL_EVACT1_Msk;
	tmp |= TCC_EVCTRL_EVACT1(data);
	((Tcc *)hw)->EVCTRL.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_EVCTRL_EVACT1_bf(const void *const hw, hri_tcc_evctrl_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->EVCTRL.reg &= ~TCC_EVCTRL_EVACT1(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_EVCTRL_EVACT1_bf(const void *const hw, hri_tcc_evctrl_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->EVCTRL.reg ^= TCC_EVCTRL_EVACT1(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_evctrl_reg_t hri_tcc_read_EVCTRL_EVACT1_bf(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->EVCTRL.reg;
	tmp = (tmp & TCC_EVCTRL_EVACT1_Msk) >> TCC_EVCTRL_EVACT1_Pos;
	return tmp;
}

static inline void hri_tcc_set_EVCTRL_CNTSEL_bf(const void *const hw, hri_tcc_evctrl_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->EVCTRL.reg |= TCC_EVCTRL_CNTSEL(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_evctrl_reg_t hri_tcc_get_EVCTRL_CNTSEL_bf(const void *const hw, hri_tcc_evctrl_reg_t mask)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->EVCTRL.reg;
	tmp = (tmp & TCC_EVCTRL_CNTSEL(mask)) >> TCC_EVCTRL_CNTSEL_Pos;
	return tmp;
}

static inline void hri_tcc_write_EVCTRL_CNTSEL_bf(const void *const hw, hri_tcc_evctrl_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->EVCTRL.reg;
	tmp &= ~TCC_EVCTRL_CNTSEL_Msk;
	tmp |= TCC_EVCTRL_CNTSEL(data);
	((Tcc *)hw)->EVCTRL.reg = tmp;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_EVCTRL_CNTSEL_bf(const void *const hw, hri_tcc_evctrl_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->EVCTRL.reg &= ~TCC_EVCTRL_CNTSEL(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_EVCTRL_CNTSEL_bf(const void *const hw, hri_tcc_evctrl_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->EVCTRL.reg ^= TCC_EVCTRL_CNTSEL(mask);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_evctrl_reg_t hri_tcc_read_EVCTRL_CNTSEL_bf(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->EVCTRL.reg;
	tmp = (tmp & TCC_EVCTRL_CNTSEL_Msk) >> TCC_EVCTRL_CNTSEL_Pos;
	return tmp;
}

static inline void hri_tcc_set_EVCTRL_reg(const void *const hw, hri_tcc_evctrl_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->EVCTRL.reg |= mask;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_evctrl_reg_t hri_tcc_get_EVCTRL_reg(const void *const hw, hri_tcc_evctrl_reg_t mask)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->EVCTRL.reg;
	tmp &= mask;
	return tmp;
}

static inline void hri_tcc_write_EVCTRL_reg(const void *const hw, hri_tcc_evctrl_reg_t data)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->EVCTRL.reg = data;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_EVCTRL_reg(const void *const hw, hri_tcc_evctrl_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->EVCTRL.reg &= ~mask;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_EVCTRL_reg(const void *const hw, hri_tcc_evctrl_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->EVCTRL.reg ^= mask;
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_evctrl_reg_t hri_tcc_read_EVCTRL_reg(const void *const hw)
{
	return ((Tcc *)hw)->EVCTRL.reg;
}

static inline void hri_tcc_set_COUNT_DITH6_COUNT_bf(const void *const hw, hri_tcc_count_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->COUNT.reg |= TCC_COUNT_COUNT(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_COUNT);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_count_reg_t hri_tcc_get_COUNT_DITH6_COUNT_bf(const void *const hw, hri_tcc_count_reg_t mask)
{
	uint32_t tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_COUNT);
	tmp = ((Tcc *)hw)->COUNT.reg;
	tmp = (tmp & TCC_COUNT_COUNT(mask)) >> TCC_COUNT_COUNT_Pos;
	return tmp;
}

static inline void hri_tcc_write_COUNT_DITH6_COUNT_bf(const void *const hw, hri_tcc_count_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->COUNT.reg;
	tmp &= ~TCC_COUNT_COUNT_Msk;
	tmp |= TCC_COUNT_COUNT(data);
	((Tcc *)hw)->COUNT.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_COUNT);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_COUNT_DITH6_COUNT_bf(const void *const hw, hri_tcc_count_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->COUNT.reg &= ~TCC_COUNT_COUNT(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_COUNT);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_COUNT_DITH6_COUNT_bf(const void *const hw, hri_tcc_count_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->COUNT.reg ^= TCC_COUNT_COUNT(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_COUNT);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_count_reg_t hri_tcc_read_COUNT_DITH6_COUNT_bf(const void *const hw)
{
	uint32_t tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_COUNT);
	tmp = ((Tcc *)hw)->COUNT.reg;
	tmp = (tmp & TCC_COUNT_COUNT_Msk) >> TCC_COUNT_COUNT_Pos;
	return tmp;
}

static inline void hri_tcc_set_COUNT_DITH5_COUNT_bf(const void *const hw, hri_tcc_count_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->COUNT.reg |= TCC_COUNT_COUNT(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_COUNT);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_count_reg_t hri_tcc_get_COUNT_DITH5_COUNT_bf(const void *const hw, hri_tcc_count_reg_t mask)
{
	uint32_t tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_COUNT);
	tmp = ((Tcc *)hw)->COUNT.reg;
	tmp = (tmp & TCC_COUNT_COUNT(mask)) >> TCC_COUNT_COUNT_Pos;
	return tmp;
}

static inline void hri_tcc_write_COUNT_DITH5_COUNT_bf(const void *const hw, hri_tcc_count_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->COUNT.reg;
	tmp &= ~TCC_COUNT_COUNT_Msk;
	tmp |= TCC_COUNT_COUNT(data);
	((Tcc *)hw)->COUNT.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_COUNT);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_COUNT_DITH5_COUNT_bf(const void *const hw, hri_tcc_count_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->COUNT.reg &= ~TCC_COUNT_COUNT(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_COUNT);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_COUNT_DITH5_COUNT_bf(const void *const hw, hri_tcc_count_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->COUNT.reg ^= TCC_COUNT_COUNT(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_COUNT);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_count_reg_t hri_tcc_read_COUNT_DITH5_COUNT_bf(const void *const hw)
{
	uint32_t tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_COUNT);
	tmp = ((Tcc *)hw)->COUNT.reg;
	tmp = (tmp & TCC_COUNT_COUNT_Msk) >> TCC_COUNT_COUNT_Pos;
	return tmp;
}

static inline void hri_tcc_set_COUNT_DITH4_COUNT_bf(const void *const hw, hri_tcc_count_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->COUNT.reg |= TCC_COUNT_COUNT(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_COUNT);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_count_reg_t hri_tcc_get_COUNT_DITH4_COUNT_bf(const void *const hw, hri_tcc_count_reg_t mask)
{
	uint32_t tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_COUNT);
	tmp = ((Tcc *)hw)->COUNT.reg;
	tmp = (tmp & TCC_COUNT_COUNT(mask)) >> TCC_COUNT_COUNT_Pos;
	return tmp;
}

static inline void hri_tcc_write_COUNT_DITH4_COUNT_bf(const void *const hw, hri_tcc_count_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->COUNT.reg;
	tmp &= ~TCC_COUNT_COUNT_Msk;
	tmp |= TCC_COUNT_COUNT(data);
	((Tcc *)hw)->COUNT.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_COUNT);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_COUNT_DITH4_COUNT_bf(const void *const hw, hri_tcc_count_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->COUNT.reg &= ~TCC_COUNT_COUNT(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_COUNT);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_COUNT_DITH4_COUNT_bf(const void *const hw, hri_tcc_count_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->COUNT.reg ^= TCC_COUNT_COUNT(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_COUNT);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_count_reg_t hri_tcc_read_COUNT_DITH4_COUNT_bf(const void *const hw)
{
	uint32_t tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_COUNT);
	tmp = ((Tcc *)hw)->COUNT.reg;
	tmp = (tmp & TCC_COUNT_COUNT_Msk) >> TCC_COUNT_COUNT_Pos;
	return tmp;
}

static inline void hri_tcc_set_COUNT_COUNT_bf(const void *const hw, hri_tcc_count_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->COUNT.reg |= TCC_COUNT_COUNT(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_COUNT);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_count_reg_t hri_tcc_get_COUNT_COUNT_bf(const void *const hw, hri_tcc_count_reg_t mask)
{
	uint32_t tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_COUNT);
	tmp = ((Tcc *)hw)->COUNT.reg;
	tmp = (tmp & TCC_COUNT_COUNT(mask)) >> TCC_COUNT_COUNT_Pos;
	return tmp;
}

static inline void hri_tcc_write_COUNT_COUNT_bf(const void *const hw, hri_tcc_count_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->COUNT.reg;
	tmp &= ~TCC_COUNT_COUNT_Msk;
	tmp |= TCC_COUNT_COUNT(data);
	((Tcc *)hw)->COUNT.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_COUNT);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_COUNT_COUNT_bf(const void *const hw, hri_tcc_count_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->COUNT.reg &= ~TCC_COUNT_COUNT(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_COUNT);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_COUNT_COUNT_bf(const void *const hw, hri_tcc_count_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->COUNT.reg ^= TCC_COUNT_COUNT(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_COUNT);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_count_reg_t hri_tcc_read_COUNT_COUNT_bf(const void *const hw)
{
	uint32_t tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_COUNT);
	tmp = ((Tcc *)hw)->COUNT.reg;
	tmp = (tmp & TCC_COUNT_COUNT_Msk) >> TCC_COUNT_COUNT_Pos;
	return tmp;
}

static inline void hri_tcc_set_COUNT_reg(const void *const hw, hri_tcc_count_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->COUNT.reg |= mask;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_COUNT);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_count_reg_t hri_tcc_get_COUNT_reg(const void *const hw, hri_tcc_count_reg_t mask)
{
	uint32_t tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_COUNT);
	tmp = ((Tcc *)hw)->COUNT.reg;
	tmp &= mask;
	return tmp;
}

static inline void hri_tcc_write_COUNT_reg(const void *const hw, hri_tcc_count_reg_t data)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->COUNT.reg = data;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_COUNT);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_COUNT_reg(const void *const hw, hri_tcc_count_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->COUNT.reg &= ~mask;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_COUNT);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_COUNT_reg(const void *const hw, hri_tcc_count_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->COUNT.reg ^= mask;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_COUNT);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_count_reg_t hri_tcc_read_COUNT_reg(const void *const hw)
{
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_COUNT);
	return ((Tcc *)hw)->COUNT.reg;
}

static inline void hri_tcc_set_PATT_PGE0_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATT.reg |= TCC_PATT_PGE0;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_PATT_PGE0_bit(const void *const hw)
{
	uint16_t tmp;
	tmp = ((Tcc *)hw)->PATT.reg;
	tmp = (tmp & TCC_PATT_PGE0) >> TCC_PATT_PGE0_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_PATT_PGE0_bit(const void *const hw, bool value)
{
	uint16_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->PATT.reg;
	tmp &= ~TCC_PATT_PGE0;
	tmp |= value << TCC_PATT_PGE0_Pos;
	((Tcc *)hw)->PATT.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_PATT_PGE0_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATT.reg &= ~TCC_PATT_PGE0;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_PATT_PGE0_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATT.reg ^= TCC_PATT_PGE0;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_PATT_PGE1_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATT.reg |= TCC_PATT_PGE1;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_PATT_PGE1_bit(const void *const hw)
{
	uint16_t tmp;
	tmp = ((Tcc *)hw)->PATT.reg;
	tmp = (tmp & TCC_PATT_PGE1) >> TCC_PATT_PGE1_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_PATT_PGE1_bit(const void *const hw, bool value)
{
	uint16_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->PATT.reg;
	tmp &= ~TCC_PATT_PGE1;
	tmp |= value << TCC_PATT_PGE1_Pos;
	((Tcc *)hw)->PATT.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_PATT_PGE1_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATT.reg &= ~TCC_PATT_PGE1;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_PATT_PGE1_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATT.reg ^= TCC_PATT_PGE1;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_PATT_PGE2_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATT.reg |= TCC_PATT_PGE2;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_PATT_PGE2_bit(const void *const hw)
{
	uint16_t tmp;
	tmp = ((Tcc *)hw)->PATT.reg;
	tmp = (tmp & TCC_PATT_PGE2) >> TCC_PATT_PGE2_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_PATT_PGE2_bit(const void *const hw, bool value)
{
	uint16_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->PATT.reg;
	tmp &= ~TCC_PATT_PGE2;
	tmp |= value << TCC_PATT_PGE2_Pos;
	((Tcc *)hw)->PATT.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_PATT_PGE2_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATT.reg &= ~TCC_PATT_PGE2;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_PATT_PGE2_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATT.reg ^= TCC_PATT_PGE2;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_PATT_PGE3_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATT.reg |= TCC_PATT_PGE3;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_PATT_PGE3_bit(const void *const hw)
{
	uint16_t tmp;
	tmp = ((Tcc *)hw)->PATT.reg;
	tmp = (tmp & TCC_PATT_PGE3) >> TCC_PATT_PGE3_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_PATT_PGE3_bit(const void *const hw, bool value)
{
	uint16_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->PATT.reg;
	tmp &= ~TCC_PATT_PGE3;
	tmp |= value << TCC_PATT_PGE3_Pos;
	((Tcc *)hw)->PATT.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_PATT_PGE3_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATT.reg &= ~TCC_PATT_PGE3;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_PATT_PGE3_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATT.reg ^= TCC_PATT_PGE3;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_PATT_PGE4_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATT.reg |= TCC_PATT_PGE4;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_PATT_PGE4_bit(const void *const hw)
{
	uint16_t tmp;
	tmp = ((Tcc *)hw)->PATT.reg;
	tmp = (tmp & TCC_PATT_PGE4) >> TCC_PATT_PGE4_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_PATT_PGE4_bit(const void *const hw, bool value)
{
	uint16_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->PATT.reg;
	tmp &= ~TCC_PATT_PGE4;
	tmp |= value << TCC_PATT_PGE4_Pos;
	((Tcc *)hw)->PATT.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_PATT_PGE4_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATT.reg &= ~TCC_PATT_PGE4;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_PATT_PGE4_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATT.reg ^= TCC_PATT_PGE4;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_PATT_PGE5_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATT.reg |= TCC_PATT_PGE5;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_PATT_PGE5_bit(const void *const hw)
{
	uint16_t tmp;
	tmp = ((Tcc *)hw)->PATT.reg;
	tmp = (tmp & TCC_PATT_PGE5) >> TCC_PATT_PGE5_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_PATT_PGE5_bit(const void *const hw, bool value)
{
	uint16_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->PATT.reg;
	tmp &= ~TCC_PATT_PGE5;
	tmp |= value << TCC_PATT_PGE5_Pos;
	((Tcc *)hw)->PATT.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_PATT_PGE5_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATT.reg &= ~TCC_PATT_PGE5;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_PATT_PGE5_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATT.reg ^= TCC_PATT_PGE5;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_PATT_PGE6_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATT.reg |= TCC_PATT_PGE6;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_PATT_PGE6_bit(const void *const hw)
{
	uint16_t tmp;
	tmp = ((Tcc *)hw)->PATT.reg;
	tmp = (tmp & TCC_PATT_PGE6) >> TCC_PATT_PGE6_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_PATT_PGE6_bit(const void *const hw, bool value)
{
	uint16_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->PATT.reg;
	tmp &= ~TCC_PATT_PGE6;
	tmp |= value << TCC_PATT_PGE6_Pos;
	((Tcc *)hw)->PATT.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_PATT_PGE6_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATT.reg &= ~TCC_PATT_PGE6;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_PATT_PGE6_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATT.reg ^= TCC_PATT_PGE6;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_PATT_PGE7_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATT.reg |= TCC_PATT_PGE7;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_PATT_PGE7_bit(const void *const hw)
{
	uint16_t tmp;
	tmp = ((Tcc *)hw)->PATT.reg;
	tmp = (tmp & TCC_PATT_PGE7) >> TCC_PATT_PGE7_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_PATT_PGE7_bit(const void *const hw, bool value)
{
	uint16_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->PATT.reg;
	tmp &= ~TCC_PATT_PGE7;
	tmp |= value << TCC_PATT_PGE7_Pos;
	((Tcc *)hw)->PATT.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_PATT_PGE7_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATT.reg &= ~TCC_PATT_PGE7;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_PATT_PGE7_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATT.reg ^= TCC_PATT_PGE7;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_PATT_PGV0_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATT.reg |= TCC_PATT_PGV0;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_PATT_PGV0_bit(const void *const hw)
{
	uint16_t tmp;
	tmp = ((Tcc *)hw)->PATT.reg;
	tmp = (tmp & TCC_PATT_PGV0) >> TCC_PATT_PGV0_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_PATT_PGV0_bit(const void *const hw, bool value)
{
	uint16_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->PATT.reg;
	tmp &= ~TCC_PATT_PGV0;
	tmp |= value << TCC_PATT_PGV0_Pos;
	((Tcc *)hw)->PATT.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_PATT_PGV0_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATT.reg &= ~TCC_PATT_PGV0;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_PATT_PGV0_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATT.reg ^= TCC_PATT_PGV0;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_PATT_PGV1_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATT.reg |= TCC_PATT_PGV1;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_PATT_PGV1_bit(const void *const hw)
{
	uint16_t tmp;
	tmp = ((Tcc *)hw)->PATT.reg;
	tmp = (tmp & TCC_PATT_PGV1) >> TCC_PATT_PGV1_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_PATT_PGV1_bit(const void *const hw, bool value)
{
	uint16_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->PATT.reg;
	tmp &= ~TCC_PATT_PGV1;
	tmp |= value << TCC_PATT_PGV1_Pos;
	((Tcc *)hw)->PATT.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_PATT_PGV1_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATT.reg &= ~TCC_PATT_PGV1;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_PATT_PGV1_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATT.reg ^= TCC_PATT_PGV1;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_PATT_PGV2_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATT.reg |= TCC_PATT_PGV2;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_PATT_PGV2_bit(const void *const hw)
{
	uint16_t tmp;
	tmp = ((Tcc *)hw)->PATT.reg;
	tmp = (tmp & TCC_PATT_PGV2) >> TCC_PATT_PGV2_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_PATT_PGV2_bit(const void *const hw, bool value)
{
	uint16_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->PATT.reg;
	tmp &= ~TCC_PATT_PGV2;
	tmp |= value << TCC_PATT_PGV2_Pos;
	((Tcc *)hw)->PATT.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_PATT_PGV2_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATT.reg &= ~TCC_PATT_PGV2;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_PATT_PGV2_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATT.reg ^= TCC_PATT_PGV2;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_PATT_PGV3_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATT.reg |= TCC_PATT_PGV3;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_PATT_PGV3_bit(const void *const hw)
{
	uint16_t tmp;
	tmp = ((Tcc *)hw)->PATT.reg;
	tmp = (tmp & TCC_PATT_PGV3) >> TCC_PATT_PGV3_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_PATT_PGV3_bit(const void *const hw, bool value)
{
	uint16_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->PATT.reg;
	tmp &= ~TCC_PATT_PGV3;
	tmp |= value << TCC_PATT_PGV3_Pos;
	((Tcc *)hw)->PATT.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_PATT_PGV3_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATT.reg &= ~TCC_PATT_PGV3;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_PATT_PGV3_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATT.reg ^= TCC_PATT_PGV3;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_PATT_PGV4_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATT.reg |= TCC_PATT_PGV4;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_PATT_PGV4_bit(const void *const hw)
{
	uint16_t tmp;
	tmp = ((Tcc *)hw)->PATT.reg;
	tmp = (tmp & TCC_PATT_PGV4) >> TCC_PATT_PGV4_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_PATT_PGV4_bit(const void *const hw, bool value)
{
	uint16_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->PATT.reg;
	tmp &= ~TCC_PATT_PGV4;
	tmp |= value << TCC_PATT_PGV4_Pos;
	((Tcc *)hw)->PATT.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_PATT_PGV4_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATT.reg &= ~TCC_PATT_PGV4;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_PATT_PGV4_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATT.reg ^= TCC_PATT_PGV4;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_PATT_PGV5_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATT.reg |= TCC_PATT_PGV5;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_PATT_PGV5_bit(const void *const hw)
{
	uint16_t tmp;
	tmp = ((Tcc *)hw)->PATT.reg;
	tmp = (tmp & TCC_PATT_PGV5) >> TCC_PATT_PGV5_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_PATT_PGV5_bit(const void *const hw, bool value)
{
	uint16_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->PATT.reg;
	tmp &= ~TCC_PATT_PGV5;
	tmp |= value << TCC_PATT_PGV5_Pos;
	((Tcc *)hw)->PATT.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_PATT_PGV5_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATT.reg &= ~TCC_PATT_PGV5;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_PATT_PGV5_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATT.reg ^= TCC_PATT_PGV5;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_PATT_PGV6_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATT.reg |= TCC_PATT_PGV6;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_PATT_PGV6_bit(const void *const hw)
{
	uint16_t tmp;
	tmp = ((Tcc *)hw)->PATT.reg;
	tmp = (tmp & TCC_PATT_PGV6) >> TCC_PATT_PGV6_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_PATT_PGV6_bit(const void *const hw, bool value)
{
	uint16_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->PATT.reg;
	tmp &= ~TCC_PATT_PGV6;
	tmp |= value << TCC_PATT_PGV6_Pos;
	((Tcc *)hw)->PATT.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_PATT_PGV6_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATT.reg &= ~TCC_PATT_PGV6;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_PATT_PGV6_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATT.reg ^= TCC_PATT_PGV6;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_PATT_PGV7_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATT.reg |= TCC_PATT_PGV7;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_PATT_PGV7_bit(const void *const hw)
{
	uint16_t tmp;
	tmp = ((Tcc *)hw)->PATT.reg;
	tmp = (tmp & TCC_PATT_PGV7) >> TCC_PATT_PGV7_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_PATT_PGV7_bit(const void *const hw, bool value)
{
	uint16_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->PATT.reg;
	tmp &= ~TCC_PATT_PGV7;
	tmp |= value << TCC_PATT_PGV7_Pos;
	((Tcc *)hw)->PATT.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_PATT_PGV7_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATT.reg &= ~TCC_PATT_PGV7;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_PATT_PGV7_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATT.reg ^= TCC_PATT_PGV7;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_PATT_reg(const void *const hw, hri_tcc_patt_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATT.reg |= mask;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_patt_reg_t hri_tcc_get_PATT_reg(const void *const hw, hri_tcc_patt_reg_t mask)
{
	uint16_t tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	tmp = ((Tcc *)hw)->PATT.reg;
	tmp &= mask;
	return tmp;
}

static inline void hri_tcc_write_PATT_reg(const void *const hw, hri_tcc_patt_reg_t data)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATT.reg = data;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_PATT_reg(const void *const hw, hri_tcc_patt_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATT.reg &= ~mask;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_PATT_reg(const void *const hw, hri_tcc_patt_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATT.reg ^= mask;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_patt_reg_t hri_tcc_read_PATT_reg(const void *const hw)
{
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	return ((Tcc *)hw)->PATT.reg;
}

static inline void hri_tcc_set_WAVE_CIPEREN_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVE.reg |= TCC_WAVE_CIPEREN;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_WAVE_CIPEREN_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->WAVE.reg;
	tmp = (tmp & TCC_WAVE_CIPEREN) >> TCC_WAVE_CIPEREN_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_WAVE_CIPEREN_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->WAVE.reg;
	tmp &= ~TCC_WAVE_CIPEREN;
	tmp |= value << TCC_WAVE_CIPEREN_Pos;
	((Tcc *)hw)->WAVE.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_WAVE_CIPEREN_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVE.reg &= ~TCC_WAVE_CIPEREN;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_WAVE_CIPEREN_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVE.reg ^= TCC_WAVE_CIPEREN;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_WAVE_CICCEN0_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVE.reg |= TCC_WAVE_CICCEN0;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_WAVE_CICCEN0_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->WAVE.reg;
	tmp = (tmp & TCC_WAVE_CICCEN0) >> TCC_WAVE_CICCEN0_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_WAVE_CICCEN0_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->WAVE.reg;
	tmp &= ~TCC_WAVE_CICCEN0;
	tmp |= value << TCC_WAVE_CICCEN0_Pos;
	((Tcc *)hw)->WAVE.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_WAVE_CICCEN0_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVE.reg &= ~TCC_WAVE_CICCEN0;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_WAVE_CICCEN0_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVE.reg ^= TCC_WAVE_CICCEN0;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_WAVE_CICCEN1_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVE.reg |= TCC_WAVE_CICCEN1;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_WAVE_CICCEN1_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->WAVE.reg;
	tmp = (tmp & TCC_WAVE_CICCEN1) >> TCC_WAVE_CICCEN1_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_WAVE_CICCEN1_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->WAVE.reg;
	tmp &= ~TCC_WAVE_CICCEN1;
	tmp |= value << TCC_WAVE_CICCEN1_Pos;
	((Tcc *)hw)->WAVE.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_WAVE_CICCEN1_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVE.reg &= ~TCC_WAVE_CICCEN1;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_WAVE_CICCEN1_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVE.reg ^= TCC_WAVE_CICCEN1;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_WAVE_CICCEN2_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVE.reg |= TCC_WAVE_CICCEN2;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_WAVE_CICCEN2_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->WAVE.reg;
	tmp = (tmp & TCC_WAVE_CICCEN2) >> TCC_WAVE_CICCEN2_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_WAVE_CICCEN2_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->WAVE.reg;
	tmp &= ~TCC_WAVE_CICCEN2;
	tmp |= value << TCC_WAVE_CICCEN2_Pos;
	((Tcc *)hw)->WAVE.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_WAVE_CICCEN2_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVE.reg &= ~TCC_WAVE_CICCEN2;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_WAVE_CICCEN2_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVE.reg ^= TCC_WAVE_CICCEN2;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_WAVE_CICCEN3_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVE.reg |= TCC_WAVE_CICCEN3;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_WAVE_CICCEN3_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->WAVE.reg;
	tmp = (tmp & TCC_WAVE_CICCEN3) >> TCC_WAVE_CICCEN3_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_WAVE_CICCEN3_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->WAVE.reg;
	tmp &= ~TCC_WAVE_CICCEN3;
	tmp |= value << TCC_WAVE_CICCEN3_Pos;
	((Tcc *)hw)->WAVE.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_WAVE_CICCEN3_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVE.reg &= ~TCC_WAVE_CICCEN3;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_WAVE_CICCEN3_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVE.reg ^= TCC_WAVE_CICCEN3;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_WAVE_POL0_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVE.reg |= TCC_WAVE_POL0;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_WAVE_POL0_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->WAVE.reg;
	tmp = (tmp & TCC_WAVE_POL0) >> TCC_WAVE_POL0_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_WAVE_POL0_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->WAVE.reg;
	tmp &= ~TCC_WAVE_POL0;
	tmp |= value << TCC_WAVE_POL0_Pos;
	((Tcc *)hw)->WAVE.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_WAVE_POL0_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVE.reg &= ~TCC_WAVE_POL0;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_WAVE_POL0_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVE.reg ^= TCC_WAVE_POL0;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_WAVE_POL1_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVE.reg |= TCC_WAVE_POL1;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_WAVE_POL1_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->WAVE.reg;
	tmp = (tmp & TCC_WAVE_POL1) >> TCC_WAVE_POL1_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_WAVE_POL1_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->WAVE.reg;
	tmp &= ~TCC_WAVE_POL1;
	tmp |= value << TCC_WAVE_POL1_Pos;
	((Tcc *)hw)->WAVE.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_WAVE_POL1_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVE.reg &= ~TCC_WAVE_POL1;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_WAVE_POL1_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVE.reg ^= TCC_WAVE_POL1;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_WAVE_POL2_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVE.reg |= TCC_WAVE_POL2;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_WAVE_POL2_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->WAVE.reg;
	tmp = (tmp & TCC_WAVE_POL2) >> TCC_WAVE_POL2_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_WAVE_POL2_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->WAVE.reg;
	tmp &= ~TCC_WAVE_POL2;
	tmp |= value << TCC_WAVE_POL2_Pos;
	((Tcc *)hw)->WAVE.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_WAVE_POL2_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVE.reg &= ~TCC_WAVE_POL2;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_WAVE_POL2_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVE.reg ^= TCC_WAVE_POL2;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_WAVE_POL3_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVE.reg |= TCC_WAVE_POL3;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_WAVE_POL3_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->WAVE.reg;
	tmp = (tmp & TCC_WAVE_POL3) >> TCC_WAVE_POL3_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_WAVE_POL3_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->WAVE.reg;
	tmp &= ~TCC_WAVE_POL3;
	tmp |= value << TCC_WAVE_POL3_Pos;
	((Tcc *)hw)->WAVE.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_WAVE_POL3_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVE.reg &= ~TCC_WAVE_POL3;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_WAVE_POL3_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVE.reg ^= TCC_WAVE_POL3;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_WAVE_SWAP0_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVE.reg |= TCC_WAVE_SWAP0;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_WAVE_SWAP0_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->WAVE.reg;
	tmp = (tmp & TCC_WAVE_SWAP0) >> TCC_WAVE_SWAP0_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_WAVE_SWAP0_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->WAVE.reg;
	tmp &= ~TCC_WAVE_SWAP0;
	tmp |= value << TCC_WAVE_SWAP0_Pos;
	((Tcc *)hw)->WAVE.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_WAVE_SWAP0_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVE.reg &= ~TCC_WAVE_SWAP0;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_WAVE_SWAP0_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVE.reg ^= TCC_WAVE_SWAP0;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_WAVE_SWAP1_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVE.reg |= TCC_WAVE_SWAP1;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_WAVE_SWAP1_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->WAVE.reg;
	tmp = (tmp & TCC_WAVE_SWAP1) >> TCC_WAVE_SWAP1_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_WAVE_SWAP1_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->WAVE.reg;
	tmp &= ~TCC_WAVE_SWAP1;
	tmp |= value << TCC_WAVE_SWAP1_Pos;
	((Tcc *)hw)->WAVE.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_WAVE_SWAP1_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVE.reg &= ~TCC_WAVE_SWAP1;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_WAVE_SWAP1_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVE.reg ^= TCC_WAVE_SWAP1;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_WAVE_SWAP2_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVE.reg |= TCC_WAVE_SWAP2;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_WAVE_SWAP2_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->WAVE.reg;
	tmp = (tmp & TCC_WAVE_SWAP2) >> TCC_WAVE_SWAP2_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_WAVE_SWAP2_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->WAVE.reg;
	tmp &= ~TCC_WAVE_SWAP2;
	tmp |= value << TCC_WAVE_SWAP2_Pos;
	((Tcc *)hw)->WAVE.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_WAVE_SWAP2_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVE.reg &= ~TCC_WAVE_SWAP2;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_WAVE_SWAP2_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVE.reg ^= TCC_WAVE_SWAP2;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_WAVE_SWAP3_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVE.reg |= TCC_WAVE_SWAP3;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_WAVE_SWAP3_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->WAVE.reg;
	tmp = (tmp & TCC_WAVE_SWAP3) >> TCC_WAVE_SWAP3_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_WAVE_SWAP3_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->WAVE.reg;
	tmp &= ~TCC_WAVE_SWAP3;
	tmp |= value << TCC_WAVE_SWAP3_Pos;
	((Tcc *)hw)->WAVE.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_WAVE_SWAP3_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVE.reg &= ~TCC_WAVE_SWAP3;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_WAVE_SWAP3_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVE.reg ^= TCC_WAVE_SWAP3;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_WAVE_WAVEGEN_bf(const void *const hw, hri_tcc_wave_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVE.reg |= TCC_WAVE_WAVEGEN(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_wave_reg_t hri_tcc_get_WAVE_WAVEGEN_bf(const void *const hw, hri_tcc_wave_reg_t mask)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->WAVE.reg;
	tmp = (tmp & TCC_WAVE_WAVEGEN(mask)) >> TCC_WAVE_WAVEGEN_Pos;
	return tmp;
}

static inline void hri_tcc_write_WAVE_WAVEGEN_bf(const void *const hw, hri_tcc_wave_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->WAVE.reg;
	tmp &= ~TCC_WAVE_WAVEGEN_Msk;
	tmp |= TCC_WAVE_WAVEGEN(data);
	((Tcc *)hw)->WAVE.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_WAVE_WAVEGEN_bf(const void *const hw, hri_tcc_wave_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVE.reg &= ~TCC_WAVE_WAVEGEN(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_WAVE_WAVEGEN_bf(const void *const hw, hri_tcc_wave_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVE.reg ^= TCC_WAVE_WAVEGEN(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_wave_reg_t hri_tcc_read_WAVE_WAVEGEN_bf(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->WAVE.reg;
	tmp = (tmp & TCC_WAVE_WAVEGEN_Msk) >> TCC_WAVE_WAVEGEN_Pos;
	return tmp;
}

static inline void hri_tcc_set_WAVE_RAMP_bf(const void *const hw, hri_tcc_wave_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVE.reg |= TCC_WAVE_RAMP(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_wave_reg_t hri_tcc_get_WAVE_RAMP_bf(const void *const hw, hri_tcc_wave_reg_t mask)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->WAVE.reg;
	tmp = (tmp & TCC_WAVE_RAMP(mask)) >> TCC_WAVE_RAMP_Pos;
	return tmp;
}

static inline void hri_tcc_write_WAVE_RAMP_bf(const void *const hw, hri_tcc_wave_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->WAVE.reg;
	tmp &= ~TCC_WAVE_RAMP_Msk;
	tmp |= TCC_WAVE_RAMP(data);
	((Tcc *)hw)->WAVE.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_WAVE_RAMP_bf(const void *const hw, hri_tcc_wave_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVE.reg &= ~TCC_WAVE_RAMP(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_WAVE_RAMP_bf(const void *const hw, hri_tcc_wave_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVE.reg ^= TCC_WAVE_RAMP(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_wave_reg_t hri_tcc_read_WAVE_RAMP_bf(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->WAVE.reg;
	tmp = (tmp & TCC_WAVE_RAMP_Msk) >> TCC_WAVE_RAMP_Pos;
	return tmp;
}

static inline void hri_tcc_set_WAVE_reg(const void *const hw, hri_tcc_wave_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVE.reg |= mask;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_wave_reg_t hri_tcc_get_WAVE_reg(const void *const hw, hri_tcc_wave_reg_t mask)
{
	uint32_t tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	tmp = ((Tcc *)hw)->WAVE.reg;
	tmp &= mask;
	return tmp;
}

static inline void hri_tcc_write_WAVE_reg(const void *const hw, hri_tcc_wave_reg_t data)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVE.reg = data;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_WAVE_reg(const void *const hw, hri_tcc_wave_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVE.reg &= ~mask;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_WAVE_reg(const void *const hw, hri_tcc_wave_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVE.reg ^= mask;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_wave_reg_t hri_tcc_read_WAVE_reg(const void *const hw)
{
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	return ((Tcc *)hw)->WAVE.reg;
}

static inline void hri_tcc_set_PER_DITH4_DITHERCY_bf(const void *const hw, hri_tcc_per_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PER.reg |= TCC_PER_DITH4_DITHERCY(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_per_reg_t hri_tcc_get_PER_DITH4_DITHERCY_bf(const void *const hw, hri_tcc_per_reg_t mask)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->PER.reg;
	tmp = (tmp & TCC_PER_DITH4_DITHERCY(mask)) >> TCC_PER_DITH4_DITHERCY_Pos;
	return tmp;
}

static inline void hri_tcc_write_PER_DITH4_DITHERCY_bf(const void *const hw, hri_tcc_per_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->PER.reg;
	tmp &= ~TCC_PER_DITH4_DITHERCY_Msk;
	tmp |= TCC_PER_DITH4_DITHERCY(data);
	((Tcc *)hw)->PER.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_PER_DITH4_DITHERCY_bf(const void *const hw, hri_tcc_per_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PER.reg &= ~TCC_PER_DITH4_DITHERCY(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_PER_DITH4_DITHERCY_bf(const void *const hw, hri_tcc_per_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PER.reg ^= TCC_PER_DITH4_DITHERCY(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_per_reg_t hri_tcc_read_PER_DITH4_DITHERCY_bf(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->PER.reg;
	tmp = (tmp & TCC_PER_DITH4_DITHERCY_Msk) >> TCC_PER_DITH4_DITHERCY_Pos;
	return tmp;
}

static inline void hri_tcc_set_PER_DITH5_DITHERCY_bf(const void *const hw, hri_tcc_per_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PER.reg |= TCC_PER_DITH5_DITHERCY(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_per_reg_t hri_tcc_get_PER_DITH5_DITHERCY_bf(const void *const hw, hri_tcc_per_reg_t mask)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->PER.reg;
	tmp = (tmp & TCC_PER_DITH5_DITHERCY(mask)) >> TCC_PER_DITH5_DITHERCY_Pos;
	return tmp;
}

static inline void hri_tcc_write_PER_DITH5_DITHERCY_bf(const void *const hw, hri_tcc_per_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->PER.reg;
	tmp &= ~TCC_PER_DITH5_DITHERCY_Msk;
	tmp |= TCC_PER_DITH5_DITHERCY(data);
	((Tcc *)hw)->PER.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_PER_DITH5_DITHERCY_bf(const void *const hw, hri_tcc_per_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PER.reg &= ~TCC_PER_DITH5_DITHERCY(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_PER_DITH5_DITHERCY_bf(const void *const hw, hri_tcc_per_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PER.reg ^= TCC_PER_DITH5_DITHERCY(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_per_reg_t hri_tcc_read_PER_DITH5_DITHERCY_bf(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->PER.reg;
	tmp = (tmp & TCC_PER_DITH5_DITHERCY_Msk) >> TCC_PER_DITH5_DITHERCY_Pos;
	return tmp;
}

static inline void hri_tcc_set_PER_DITH6_DITHERCY_bf(const void *const hw, hri_tcc_per_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PER.reg |= TCC_PER_DITH6_DITHERCY(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_per_reg_t hri_tcc_get_PER_DITH6_DITHERCY_bf(const void *const hw, hri_tcc_per_reg_t mask)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->PER.reg;
	tmp = (tmp & TCC_PER_DITH6_DITHERCY(mask)) >> TCC_PER_DITH6_DITHERCY_Pos;
	return tmp;
}

static inline void hri_tcc_write_PER_DITH6_DITHERCY_bf(const void *const hw, hri_tcc_per_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->PER.reg;
	tmp &= ~TCC_PER_DITH6_DITHERCY_Msk;
	tmp |= TCC_PER_DITH6_DITHERCY(data);
	((Tcc *)hw)->PER.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_PER_DITH6_DITHERCY_bf(const void *const hw, hri_tcc_per_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PER.reg &= ~TCC_PER_DITH6_DITHERCY(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_PER_DITH6_DITHERCY_bf(const void *const hw, hri_tcc_per_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PER.reg ^= TCC_PER_DITH6_DITHERCY(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_per_reg_t hri_tcc_read_PER_DITH6_DITHERCY_bf(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->PER.reg;
	tmp = (tmp & TCC_PER_DITH6_DITHERCY_Msk) >> TCC_PER_DITH6_DITHERCY_Pos;
	return tmp;
}

static inline void hri_tcc_set_PER_DITH6_PER_bf(const void *const hw, hri_tcc_per_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PER.reg |= TCC_PER_PER(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_PER);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_per_reg_t hri_tcc_get_PER_DITH6_PER_bf(const void *const hw, hri_tcc_per_reg_t mask)
{
	uint32_t tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_PER);
	tmp = ((Tcc *)hw)->PER.reg;
	tmp = (tmp & TCC_PER_PER(mask)) >> TCC_PER_PER_Pos;
	return tmp;
}

static inline void hri_tcc_write_PER_DITH6_PER_bf(const void *const hw, hri_tcc_per_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->PER.reg;
	tmp &= ~TCC_PER_PER_Msk;
	tmp |= TCC_PER_PER(data);
	((Tcc *)hw)->PER.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_PER);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_PER_DITH6_PER_bf(const void *const hw, hri_tcc_per_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PER.reg &= ~TCC_PER_PER(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_PER);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_PER_DITH6_PER_bf(const void *const hw, hri_tcc_per_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PER.reg ^= TCC_PER_PER(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_PER);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_per_reg_t hri_tcc_read_PER_DITH6_PER_bf(const void *const hw)
{
	uint32_t tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_PER);
	tmp = ((Tcc *)hw)->PER.reg;
	tmp = (tmp & TCC_PER_PER_Msk) >> TCC_PER_PER_Pos;
	return tmp;
}

static inline void hri_tcc_set_PER_DITH5_PER_bf(const void *const hw, hri_tcc_per_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PER.reg |= TCC_PER_PER(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_PER);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_per_reg_t hri_tcc_get_PER_DITH5_PER_bf(const void *const hw, hri_tcc_per_reg_t mask)
{
	uint32_t tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_PER);
	tmp = ((Tcc *)hw)->PER.reg;
	tmp = (tmp & TCC_PER_PER(mask)) >> TCC_PER_PER_Pos;
	return tmp;
}

static inline void hri_tcc_write_PER_DITH5_PER_bf(const void *const hw, hri_tcc_per_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->PER.reg;
	tmp &= ~TCC_PER_PER_Msk;
	tmp |= TCC_PER_PER(data);
	((Tcc *)hw)->PER.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_PER);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_PER_DITH5_PER_bf(const void *const hw, hri_tcc_per_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PER.reg &= ~TCC_PER_PER(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_PER);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_PER_DITH5_PER_bf(const void *const hw, hri_tcc_per_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PER.reg ^= TCC_PER_PER(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_PER);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_per_reg_t hri_tcc_read_PER_DITH5_PER_bf(const void *const hw)
{
	uint32_t tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_PER);
	tmp = ((Tcc *)hw)->PER.reg;
	tmp = (tmp & TCC_PER_PER_Msk) >> TCC_PER_PER_Pos;
	return tmp;
}

static inline void hri_tcc_set_PER_DITH4_PER_bf(const void *const hw, hri_tcc_per_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PER.reg |= TCC_PER_PER(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_PER);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_per_reg_t hri_tcc_get_PER_DITH4_PER_bf(const void *const hw, hri_tcc_per_reg_t mask)
{
	uint32_t tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_PER);
	tmp = ((Tcc *)hw)->PER.reg;
	tmp = (tmp & TCC_PER_PER(mask)) >> TCC_PER_PER_Pos;
	return tmp;
}

static inline void hri_tcc_write_PER_DITH4_PER_bf(const void *const hw, hri_tcc_per_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->PER.reg;
	tmp &= ~TCC_PER_PER_Msk;
	tmp |= TCC_PER_PER(data);
	((Tcc *)hw)->PER.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_PER);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_PER_DITH4_PER_bf(const void *const hw, hri_tcc_per_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PER.reg &= ~TCC_PER_PER(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_PER);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_PER_DITH4_PER_bf(const void *const hw, hri_tcc_per_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PER.reg ^= TCC_PER_PER(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_PER);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_per_reg_t hri_tcc_read_PER_DITH4_PER_bf(const void *const hw)
{
	uint32_t tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_PER);
	tmp = ((Tcc *)hw)->PER.reg;
	tmp = (tmp & TCC_PER_PER_Msk) >> TCC_PER_PER_Pos;
	return tmp;
}

static inline void hri_tcc_set_PER_PER_bf(const void *const hw, hri_tcc_per_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PER.reg |= TCC_PER_PER(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_PER);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_per_reg_t hri_tcc_get_PER_PER_bf(const void *const hw, hri_tcc_per_reg_t mask)
{
	uint32_t tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_PER);
	tmp = ((Tcc *)hw)->PER.reg;
	tmp = (tmp & TCC_PER_PER(mask)) >> TCC_PER_PER_Pos;
	return tmp;
}

static inline void hri_tcc_write_PER_PER_bf(const void *const hw, hri_tcc_per_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->PER.reg;
	tmp &= ~TCC_PER_PER_Msk;
	tmp |= TCC_PER_PER(data);
	((Tcc *)hw)->PER.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_PER);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_PER_PER_bf(const void *const hw, hri_tcc_per_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PER.reg &= ~TCC_PER_PER(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_PER);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_PER_PER_bf(const void *const hw, hri_tcc_per_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PER.reg ^= TCC_PER_PER(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_PER);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_per_reg_t hri_tcc_read_PER_PER_bf(const void *const hw)
{
	uint32_t tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_PER);
	tmp = ((Tcc *)hw)->PER.reg;
	tmp = (tmp & TCC_PER_PER_Msk) >> TCC_PER_PER_Pos;
	return tmp;
}

static inline void hri_tcc_set_PER_reg(const void *const hw, hri_tcc_per_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PER.reg |= mask;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_PER);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_per_reg_t hri_tcc_get_PER_reg(const void *const hw, hri_tcc_per_reg_t mask)
{
	uint32_t tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_PER);
	tmp = ((Tcc *)hw)->PER.reg;
	tmp &= mask;
	return tmp;
}

static inline void hri_tcc_write_PER_reg(const void *const hw, hri_tcc_per_reg_t data)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PER.reg = data;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_PER);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_PER_reg(const void *const hw, hri_tcc_per_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PER.reg &= ~mask;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_PER);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_PER_reg(const void *const hw, hri_tcc_per_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PER.reg ^= mask;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_PER);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_per_reg_t hri_tcc_read_PER_reg(const void *const hw)
{
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_PER);
	return ((Tcc *)hw)->PER.reg;
}

static inline void hri_tcc_set_CC_DITH4_DITHERCY_bf(const void *const hw, uint8_t index, hri_tcc_cc_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CC[index].reg |= TCC_CC_DITH4_DITHERCY(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_cc_reg_t hri_tcc_get_CC_DITH4_DITHERCY_bf(const void *const hw, uint8_t index,
                                                                hri_tcc_cc_reg_t mask)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->CC[index].reg;
	tmp = (tmp & TCC_CC_DITH4_DITHERCY(mask)) >> TCC_CC_DITH4_DITHERCY_Pos;
	return tmp;
}

static inline void hri_tcc_write_CC_DITH4_DITHERCY_bf(const void *const hw, uint8_t index, hri_tcc_cc_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->CC[index].reg;
	tmp &= ~TCC_CC_DITH4_DITHERCY_Msk;
	tmp |= TCC_CC_DITH4_DITHERCY(data);
	((Tcc *)hw)->CC[index].reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_CC_DITH4_DITHERCY_bf(const void *const hw, uint8_t index, hri_tcc_cc_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CC[index].reg &= ~TCC_CC_DITH4_DITHERCY(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_CC_DITH4_DITHERCY_bf(const void *const hw, uint8_t index, hri_tcc_cc_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CC[index].reg ^= TCC_CC_DITH4_DITHERCY(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_cc_reg_t hri_tcc_read_CC_DITH4_DITHERCY_bf(const void *const hw, uint8_t index)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->CC[index].reg;
	tmp = (tmp & TCC_CC_DITH4_DITHERCY_Msk) >> TCC_CC_DITH4_DITHERCY_Pos;
	return tmp;
}

static inline void hri_tcc_set_CC_DITH5_DITHERCY_bf(const void *const hw, uint8_t index, hri_tcc_cc_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CC[index].reg |= TCC_CC_DITH5_DITHERCY(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_cc_reg_t hri_tcc_get_CC_DITH5_DITHERCY_bf(const void *const hw, uint8_t index,
                                                                hri_tcc_cc_reg_t mask)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->CC[index].reg;
	tmp = (tmp & TCC_CC_DITH5_DITHERCY(mask)) >> TCC_CC_DITH5_DITHERCY_Pos;
	return tmp;
}

static inline void hri_tcc_write_CC_DITH5_DITHERCY_bf(const void *const hw, uint8_t index, hri_tcc_cc_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->CC[index].reg;
	tmp &= ~TCC_CC_DITH5_DITHERCY_Msk;
	tmp |= TCC_CC_DITH5_DITHERCY(data);
	((Tcc *)hw)->CC[index].reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_CC_DITH5_DITHERCY_bf(const void *const hw, uint8_t index, hri_tcc_cc_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CC[index].reg &= ~TCC_CC_DITH5_DITHERCY(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_CC_DITH5_DITHERCY_bf(const void *const hw, uint8_t index, hri_tcc_cc_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CC[index].reg ^= TCC_CC_DITH5_DITHERCY(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_cc_reg_t hri_tcc_read_CC_DITH5_DITHERCY_bf(const void *const hw, uint8_t index)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->CC[index].reg;
	tmp = (tmp & TCC_CC_DITH5_DITHERCY_Msk) >> TCC_CC_DITH5_DITHERCY_Pos;
	return tmp;
}

static inline void hri_tcc_set_CC_DITH6_DITHERCY_bf(const void *const hw, uint8_t index, hri_tcc_cc_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CC[index].reg |= TCC_CC_DITH6_DITHERCY(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_cc_reg_t hri_tcc_get_CC_DITH6_DITHERCY_bf(const void *const hw, uint8_t index,
                                                                hri_tcc_cc_reg_t mask)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->CC[index].reg;
	tmp = (tmp & TCC_CC_DITH6_DITHERCY(mask)) >> TCC_CC_DITH6_DITHERCY_Pos;
	return tmp;
}

static inline void hri_tcc_write_CC_DITH6_DITHERCY_bf(const void *const hw, uint8_t index, hri_tcc_cc_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->CC[index].reg;
	tmp &= ~TCC_CC_DITH6_DITHERCY_Msk;
	tmp |= TCC_CC_DITH6_DITHERCY(data);
	((Tcc *)hw)->CC[index].reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_CC_DITH6_DITHERCY_bf(const void *const hw, uint8_t index, hri_tcc_cc_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CC[index].reg &= ~TCC_CC_DITH6_DITHERCY(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_CC_DITH6_DITHERCY_bf(const void *const hw, uint8_t index, hri_tcc_cc_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CC[index].reg ^= TCC_CC_DITH6_DITHERCY(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_cc_reg_t hri_tcc_read_CC_DITH6_DITHERCY_bf(const void *const hw, uint8_t index)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->CC[index].reg;
	tmp = (tmp & TCC_CC_DITH6_DITHERCY_Msk) >> TCC_CC_DITH6_DITHERCY_Pos;
	return tmp;
}

static inline void hri_tcc_set_CC_DITH6_CC_bf(const void *const hw, uint8_t index, hri_tcc_cc_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CC[index].reg |= TCC_CC_CC(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_CC0 | TCC_SYNCBUSY_CC1 | TCC_SYNCBUSY_CC2 | TCC_SYNCBUSY_CC3);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_cc_reg_t hri_tcc_get_CC_DITH6_CC_bf(const void *const hw, uint8_t index, hri_tcc_cc_reg_t mask)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->CC[index].reg;
	tmp = (tmp & TCC_CC_CC(mask)) >> TCC_CC_CC_Pos;
	return tmp;
}

static inline void hri_tcc_write_CC_DITH6_CC_bf(const void *const hw, uint8_t index, hri_tcc_cc_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->CC[index].reg;
	tmp &= ~TCC_CC_CC_Msk;
	tmp |= TCC_CC_CC(data);
	((Tcc *)hw)->CC[index].reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_CC0 | TCC_SYNCBUSY_CC1 | TCC_SYNCBUSY_CC2 | TCC_SYNCBUSY_CC3);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_CC_DITH6_CC_bf(const void *const hw, uint8_t index, hri_tcc_cc_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CC[index].reg &= ~TCC_CC_CC(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_CC0 | TCC_SYNCBUSY_CC1 | TCC_SYNCBUSY_CC2 | TCC_SYNCBUSY_CC3);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_CC_DITH6_CC_bf(const void *const hw, uint8_t index, hri_tcc_cc_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CC[index].reg ^= TCC_CC_CC(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_CC0 | TCC_SYNCBUSY_CC1 | TCC_SYNCBUSY_CC2 | TCC_SYNCBUSY_CC3);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_cc_reg_t hri_tcc_read_CC_DITH6_CC_bf(const void *const hw, uint8_t index)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->CC[index].reg;
	tmp = (tmp & TCC_CC_CC_Msk) >> TCC_CC_CC_Pos;
	return tmp;
}

static inline void hri_tcc_set_CC_DITH5_CC_bf(const void *const hw, uint8_t index, hri_tcc_cc_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CC[index].reg |= TCC_CC_CC(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_CC0 | TCC_SYNCBUSY_CC1 | TCC_SYNCBUSY_CC2 | TCC_SYNCBUSY_CC3);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_cc_reg_t hri_tcc_get_CC_DITH5_CC_bf(const void *const hw, uint8_t index, hri_tcc_cc_reg_t mask)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->CC[index].reg;
	tmp = (tmp & TCC_CC_CC(mask)) >> TCC_CC_CC_Pos;
	return tmp;
}

static inline void hri_tcc_write_CC_DITH5_CC_bf(const void *const hw, uint8_t index, hri_tcc_cc_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->CC[index].reg;
	tmp &= ~TCC_CC_CC_Msk;
	tmp |= TCC_CC_CC(data);
	((Tcc *)hw)->CC[index].reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_CC0 | TCC_SYNCBUSY_CC1 | TCC_SYNCBUSY_CC2 | TCC_SYNCBUSY_CC3);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_CC_DITH5_CC_bf(const void *const hw, uint8_t index, hri_tcc_cc_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CC[index].reg &= ~TCC_CC_CC(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_CC0 | TCC_SYNCBUSY_CC1 | TCC_SYNCBUSY_CC2 | TCC_SYNCBUSY_CC3);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_CC_DITH5_CC_bf(const void *const hw, uint8_t index, hri_tcc_cc_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CC[index].reg ^= TCC_CC_CC(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_CC0 | TCC_SYNCBUSY_CC1 | TCC_SYNCBUSY_CC2 | TCC_SYNCBUSY_CC3);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_cc_reg_t hri_tcc_read_CC_DITH5_CC_bf(const void *const hw, uint8_t index)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->CC[index].reg;
	tmp = (tmp & TCC_CC_CC_Msk) >> TCC_CC_CC_Pos;
	return tmp;
}

static inline void hri_tcc_set_CC_DITH4_CC_bf(const void *const hw, uint8_t index, hri_tcc_cc_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CC[index].reg |= TCC_CC_CC(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_CC0 | TCC_SYNCBUSY_CC1 | TCC_SYNCBUSY_CC2 | TCC_SYNCBUSY_CC3);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_cc_reg_t hri_tcc_get_CC_DITH4_CC_bf(const void *const hw, uint8_t index, hri_tcc_cc_reg_t mask)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->CC[index].reg;
	tmp = (tmp & TCC_CC_CC(mask)) >> TCC_CC_CC_Pos;
	return tmp;
}

static inline void hri_tcc_write_CC_DITH4_CC_bf(const void *const hw, uint8_t index, hri_tcc_cc_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->CC[index].reg;
	tmp &= ~TCC_CC_CC_Msk;
	tmp |= TCC_CC_CC(data);
	((Tcc *)hw)->CC[index].reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_CC0 | TCC_SYNCBUSY_CC1 | TCC_SYNCBUSY_CC2 | TCC_SYNCBUSY_CC3);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_CC_DITH4_CC_bf(const void *const hw, uint8_t index, hri_tcc_cc_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CC[index].reg &= ~TCC_CC_CC(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_CC0 | TCC_SYNCBUSY_CC1 | TCC_SYNCBUSY_CC2 | TCC_SYNCBUSY_CC3);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_CC_DITH4_CC_bf(const void *const hw, uint8_t index, hri_tcc_cc_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CC[index].reg ^= TCC_CC_CC(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_CC0 | TCC_SYNCBUSY_CC1 | TCC_SYNCBUSY_CC2 | TCC_SYNCBUSY_CC3);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_cc_reg_t hri_tcc_read_CC_DITH4_CC_bf(const void *const hw, uint8_t index)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->CC[index].reg;
	tmp = (tmp & TCC_CC_CC_Msk) >> TCC_CC_CC_Pos;
	return tmp;
}

static inline void hri_tcc_set_CC_CC_bf(const void *const hw, uint8_t index, hri_tcc_cc_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CC[index].reg |= TCC_CC_CC(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_CC0 | TCC_SYNCBUSY_CC1 | TCC_SYNCBUSY_CC2 | TCC_SYNCBUSY_CC3);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_cc_reg_t hri_tcc_get_CC_CC_bf(const void *const hw, uint8_t index, hri_tcc_cc_reg_t mask)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->CC[index].reg;
	tmp = (tmp & TCC_CC_CC(mask)) >> TCC_CC_CC_Pos;
	return tmp;
}

static inline void hri_tcc_write_CC_CC_bf(const void *const hw, uint8_t index, hri_tcc_cc_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->CC[index].reg;
	tmp &= ~TCC_CC_CC_Msk;
	tmp |= TCC_CC_CC(data);
	((Tcc *)hw)->CC[index].reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_CC0 | TCC_SYNCBUSY_CC1 | TCC_SYNCBUSY_CC2 | TCC_SYNCBUSY_CC3);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_CC_CC_bf(const void *const hw, uint8_t index, hri_tcc_cc_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CC[index].reg &= ~TCC_CC_CC(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_CC0 | TCC_SYNCBUSY_CC1 | TCC_SYNCBUSY_CC2 | TCC_SYNCBUSY_CC3);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_CC_CC_bf(const void *const hw, uint8_t index, hri_tcc_cc_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CC[index].reg ^= TCC_CC_CC(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_CC0 | TCC_SYNCBUSY_CC1 | TCC_SYNCBUSY_CC2 | TCC_SYNCBUSY_CC3);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_cc_reg_t hri_tcc_read_CC_CC_bf(const void *const hw, uint8_t index)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->CC[index].reg;
	tmp = (tmp & TCC_CC_CC_Msk) >> TCC_CC_CC_Pos;
	return tmp;
}

static inline void hri_tcc_set_CC_reg(const void *const hw, uint8_t index, hri_tcc_cc_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CC[index].reg |= mask;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_CC0 | TCC_SYNCBUSY_CC1 | TCC_SYNCBUSY_CC2 | TCC_SYNCBUSY_CC3);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_cc_reg_t hri_tcc_get_CC_reg(const void *const hw, uint8_t index, hri_tcc_cc_reg_t mask)
{
	uint32_t tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_CC0 | TCC_SYNCBUSY_CC1 | TCC_SYNCBUSY_CC2 | TCC_SYNCBUSY_CC3);
	tmp = ((Tcc *)hw)->CC[index].reg;
	tmp &= mask;
	return tmp;
}

static inline void hri_tcc_write_CC_reg(const void *const hw, uint8_t index, hri_tcc_cc_reg_t data)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CC[index].reg = data;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_CC0 | TCC_SYNCBUSY_CC1 | TCC_SYNCBUSY_CC2 | TCC_SYNCBUSY_CC3);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_CC_reg(const void *const hw, uint8_t index, hri_tcc_cc_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CC[index].reg &= ~mask;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_CC0 | TCC_SYNCBUSY_CC1 | TCC_SYNCBUSY_CC2 | TCC_SYNCBUSY_CC3);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_CC_reg(const void *const hw, uint8_t index, hri_tcc_cc_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CC[index].reg ^= mask;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_CC0 | TCC_SYNCBUSY_CC1 | TCC_SYNCBUSY_CC2 | TCC_SYNCBUSY_CC3);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_cc_reg_t hri_tcc_read_CC_reg(const void *const hw, uint8_t index)
{
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_CC0 | TCC_SYNCBUSY_CC1 | TCC_SYNCBUSY_CC2 | TCC_SYNCBUSY_CC3);
	return ((Tcc *)hw)->CC[index].reg;
}

static inline void hri_tcc_set_PATTB_PGEB0_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATTB.reg |= TCC_PATTB_PGEB0;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_PATTB_PGEB0_bit(const void *const hw)
{
	uint16_t tmp;
	tmp = ((Tcc *)hw)->PATTB.reg;
	tmp = (tmp & TCC_PATTB_PGEB0) >> TCC_PATTB_PGEB0_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_PATTB_PGEB0_bit(const void *const hw, bool value)
{
	uint16_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->PATTB.reg;
	tmp &= ~TCC_PATTB_PGEB0;
	tmp |= value << TCC_PATTB_PGEB0_Pos;
	((Tcc *)hw)->PATTB.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_PATTB_PGEB0_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATTB.reg &= ~TCC_PATTB_PGEB0;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_PATTB_PGEB0_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATTB.reg ^= TCC_PATTB_PGEB0;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_PATTB_PGEB1_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATTB.reg |= TCC_PATTB_PGEB1;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_PATTB_PGEB1_bit(const void *const hw)
{
	uint16_t tmp;
	tmp = ((Tcc *)hw)->PATTB.reg;
	tmp = (tmp & TCC_PATTB_PGEB1) >> TCC_PATTB_PGEB1_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_PATTB_PGEB1_bit(const void *const hw, bool value)
{
	uint16_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->PATTB.reg;
	tmp &= ~TCC_PATTB_PGEB1;
	tmp |= value << TCC_PATTB_PGEB1_Pos;
	((Tcc *)hw)->PATTB.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_PATTB_PGEB1_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATTB.reg &= ~TCC_PATTB_PGEB1;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_PATTB_PGEB1_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATTB.reg ^= TCC_PATTB_PGEB1;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_PATTB_PGEB2_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATTB.reg |= TCC_PATTB_PGEB2;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_PATTB_PGEB2_bit(const void *const hw)
{
	uint16_t tmp;
	tmp = ((Tcc *)hw)->PATTB.reg;
	tmp = (tmp & TCC_PATTB_PGEB2) >> TCC_PATTB_PGEB2_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_PATTB_PGEB2_bit(const void *const hw, bool value)
{
	uint16_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->PATTB.reg;
	tmp &= ~TCC_PATTB_PGEB2;
	tmp |= value << TCC_PATTB_PGEB2_Pos;
	((Tcc *)hw)->PATTB.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_PATTB_PGEB2_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATTB.reg &= ~TCC_PATTB_PGEB2;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_PATTB_PGEB2_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATTB.reg ^= TCC_PATTB_PGEB2;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_PATTB_PGEB3_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATTB.reg |= TCC_PATTB_PGEB3;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_PATTB_PGEB3_bit(const void *const hw)
{
	uint16_t tmp;
	tmp = ((Tcc *)hw)->PATTB.reg;
	tmp = (tmp & TCC_PATTB_PGEB3) >> TCC_PATTB_PGEB3_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_PATTB_PGEB3_bit(const void *const hw, bool value)
{
	uint16_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->PATTB.reg;
	tmp &= ~TCC_PATTB_PGEB3;
	tmp |= value << TCC_PATTB_PGEB3_Pos;
	((Tcc *)hw)->PATTB.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_PATTB_PGEB3_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATTB.reg &= ~TCC_PATTB_PGEB3;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_PATTB_PGEB3_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATTB.reg ^= TCC_PATTB_PGEB3;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_PATTB_PGEB4_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATTB.reg |= TCC_PATTB_PGEB4;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_PATTB_PGEB4_bit(const void *const hw)
{
	uint16_t tmp;
	tmp = ((Tcc *)hw)->PATTB.reg;
	tmp = (tmp & TCC_PATTB_PGEB4) >> TCC_PATTB_PGEB4_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_PATTB_PGEB4_bit(const void *const hw, bool value)
{
	uint16_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->PATTB.reg;
	tmp &= ~TCC_PATTB_PGEB4;
	tmp |= value << TCC_PATTB_PGEB4_Pos;
	((Tcc *)hw)->PATTB.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_PATTB_PGEB4_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATTB.reg &= ~TCC_PATTB_PGEB4;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_PATTB_PGEB4_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATTB.reg ^= TCC_PATTB_PGEB4;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_PATTB_PGEB5_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATTB.reg |= TCC_PATTB_PGEB5;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_PATTB_PGEB5_bit(const void *const hw)
{
	uint16_t tmp;
	tmp = ((Tcc *)hw)->PATTB.reg;
	tmp = (tmp & TCC_PATTB_PGEB5) >> TCC_PATTB_PGEB5_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_PATTB_PGEB5_bit(const void *const hw, bool value)
{
	uint16_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->PATTB.reg;
	tmp &= ~TCC_PATTB_PGEB5;
	tmp |= value << TCC_PATTB_PGEB5_Pos;
	((Tcc *)hw)->PATTB.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_PATTB_PGEB5_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATTB.reg &= ~TCC_PATTB_PGEB5;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_PATTB_PGEB5_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATTB.reg ^= TCC_PATTB_PGEB5;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_PATTB_PGEB6_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATTB.reg |= TCC_PATTB_PGEB6;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_PATTB_PGEB6_bit(const void *const hw)
{
	uint16_t tmp;
	tmp = ((Tcc *)hw)->PATTB.reg;
	tmp = (tmp & TCC_PATTB_PGEB6) >> TCC_PATTB_PGEB6_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_PATTB_PGEB6_bit(const void *const hw, bool value)
{
	uint16_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->PATTB.reg;
	tmp &= ~TCC_PATTB_PGEB6;
	tmp |= value << TCC_PATTB_PGEB6_Pos;
	((Tcc *)hw)->PATTB.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_PATTB_PGEB6_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATTB.reg &= ~TCC_PATTB_PGEB6;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_PATTB_PGEB6_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATTB.reg ^= TCC_PATTB_PGEB6;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_PATTB_PGEB7_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATTB.reg |= TCC_PATTB_PGEB7;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_PATTB_PGEB7_bit(const void *const hw)
{
	uint16_t tmp;
	tmp = ((Tcc *)hw)->PATTB.reg;
	tmp = (tmp & TCC_PATTB_PGEB7) >> TCC_PATTB_PGEB7_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_PATTB_PGEB7_bit(const void *const hw, bool value)
{
	uint16_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->PATTB.reg;
	tmp &= ~TCC_PATTB_PGEB7;
	tmp |= value << TCC_PATTB_PGEB7_Pos;
	((Tcc *)hw)->PATTB.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_PATTB_PGEB7_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATTB.reg &= ~TCC_PATTB_PGEB7;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_PATTB_PGEB7_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATTB.reg ^= TCC_PATTB_PGEB7;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_PATTB_PGVB0_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATTB.reg |= TCC_PATTB_PGVB0;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_PATTB_PGVB0_bit(const void *const hw)
{
	uint16_t tmp;
	tmp = ((Tcc *)hw)->PATTB.reg;
	tmp = (tmp & TCC_PATTB_PGVB0) >> TCC_PATTB_PGVB0_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_PATTB_PGVB0_bit(const void *const hw, bool value)
{
	uint16_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->PATTB.reg;
	tmp &= ~TCC_PATTB_PGVB0;
	tmp |= value << TCC_PATTB_PGVB0_Pos;
	((Tcc *)hw)->PATTB.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_PATTB_PGVB0_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATTB.reg &= ~TCC_PATTB_PGVB0;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_PATTB_PGVB0_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATTB.reg ^= TCC_PATTB_PGVB0;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_PATTB_PGVB1_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATTB.reg |= TCC_PATTB_PGVB1;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_PATTB_PGVB1_bit(const void *const hw)
{
	uint16_t tmp;
	tmp = ((Tcc *)hw)->PATTB.reg;
	tmp = (tmp & TCC_PATTB_PGVB1) >> TCC_PATTB_PGVB1_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_PATTB_PGVB1_bit(const void *const hw, bool value)
{
	uint16_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->PATTB.reg;
	tmp &= ~TCC_PATTB_PGVB1;
	tmp |= value << TCC_PATTB_PGVB1_Pos;
	((Tcc *)hw)->PATTB.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_PATTB_PGVB1_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATTB.reg &= ~TCC_PATTB_PGVB1;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_PATTB_PGVB1_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATTB.reg ^= TCC_PATTB_PGVB1;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_PATTB_PGVB2_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATTB.reg |= TCC_PATTB_PGVB2;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_PATTB_PGVB2_bit(const void *const hw)
{
	uint16_t tmp;
	tmp = ((Tcc *)hw)->PATTB.reg;
	tmp = (tmp & TCC_PATTB_PGVB2) >> TCC_PATTB_PGVB2_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_PATTB_PGVB2_bit(const void *const hw, bool value)
{
	uint16_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->PATTB.reg;
	tmp &= ~TCC_PATTB_PGVB2;
	tmp |= value << TCC_PATTB_PGVB2_Pos;
	((Tcc *)hw)->PATTB.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_PATTB_PGVB2_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATTB.reg &= ~TCC_PATTB_PGVB2;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_PATTB_PGVB2_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATTB.reg ^= TCC_PATTB_PGVB2;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_PATTB_PGVB3_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATTB.reg |= TCC_PATTB_PGVB3;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_PATTB_PGVB3_bit(const void *const hw)
{
	uint16_t tmp;
	tmp = ((Tcc *)hw)->PATTB.reg;
	tmp = (tmp & TCC_PATTB_PGVB3) >> TCC_PATTB_PGVB3_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_PATTB_PGVB3_bit(const void *const hw, bool value)
{
	uint16_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->PATTB.reg;
	tmp &= ~TCC_PATTB_PGVB3;
	tmp |= value << TCC_PATTB_PGVB3_Pos;
	((Tcc *)hw)->PATTB.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_PATTB_PGVB3_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATTB.reg &= ~TCC_PATTB_PGVB3;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_PATTB_PGVB3_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATTB.reg ^= TCC_PATTB_PGVB3;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_PATTB_PGVB4_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATTB.reg |= TCC_PATTB_PGVB4;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_PATTB_PGVB4_bit(const void *const hw)
{
	uint16_t tmp;
	tmp = ((Tcc *)hw)->PATTB.reg;
	tmp = (tmp & TCC_PATTB_PGVB4) >> TCC_PATTB_PGVB4_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_PATTB_PGVB4_bit(const void *const hw, bool value)
{
	uint16_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->PATTB.reg;
	tmp &= ~TCC_PATTB_PGVB4;
	tmp |= value << TCC_PATTB_PGVB4_Pos;
	((Tcc *)hw)->PATTB.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_PATTB_PGVB4_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATTB.reg &= ~TCC_PATTB_PGVB4;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_PATTB_PGVB4_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATTB.reg ^= TCC_PATTB_PGVB4;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_PATTB_PGVB5_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATTB.reg |= TCC_PATTB_PGVB5;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_PATTB_PGVB5_bit(const void *const hw)
{
	uint16_t tmp;
	tmp = ((Tcc *)hw)->PATTB.reg;
	tmp = (tmp & TCC_PATTB_PGVB5) >> TCC_PATTB_PGVB5_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_PATTB_PGVB5_bit(const void *const hw, bool value)
{
	uint16_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->PATTB.reg;
	tmp &= ~TCC_PATTB_PGVB5;
	tmp |= value << TCC_PATTB_PGVB5_Pos;
	((Tcc *)hw)->PATTB.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_PATTB_PGVB5_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATTB.reg &= ~TCC_PATTB_PGVB5;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_PATTB_PGVB5_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATTB.reg ^= TCC_PATTB_PGVB5;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_PATTB_PGVB6_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATTB.reg |= TCC_PATTB_PGVB6;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_PATTB_PGVB6_bit(const void *const hw)
{
	uint16_t tmp;
	tmp = ((Tcc *)hw)->PATTB.reg;
	tmp = (tmp & TCC_PATTB_PGVB6) >> TCC_PATTB_PGVB6_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_PATTB_PGVB6_bit(const void *const hw, bool value)
{
	uint16_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->PATTB.reg;
	tmp &= ~TCC_PATTB_PGVB6;
	tmp |= value << TCC_PATTB_PGVB6_Pos;
	((Tcc *)hw)->PATTB.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_PATTB_PGVB6_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATTB.reg &= ~TCC_PATTB_PGVB6;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_PATTB_PGVB6_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATTB.reg ^= TCC_PATTB_PGVB6;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_PATTB_PGVB7_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATTB.reg |= TCC_PATTB_PGVB7;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_PATTB_PGVB7_bit(const void *const hw)
{
	uint16_t tmp;
	tmp = ((Tcc *)hw)->PATTB.reg;
	tmp = (tmp & TCC_PATTB_PGVB7) >> TCC_PATTB_PGVB7_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_PATTB_PGVB7_bit(const void *const hw, bool value)
{
	uint16_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->PATTB.reg;
	tmp &= ~TCC_PATTB_PGVB7;
	tmp |= value << TCC_PATTB_PGVB7_Pos;
	((Tcc *)hw)->PATTB.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_PATTB_PGVB7_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATTB.reg &= ~TCC_PATTB_PGVB7;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_PATTB_PGVB7_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATTB.reg ^= TCC_PATTB_PGVB7;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_PATTB_reg(const void *const hw, hri_tcc_pattb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATTB.reg |= mask;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_pattb_reg_t hri_tcc_get_PATTB_reg(const void *const hw, hri_tcc_pattb_reg_t mask)
{
	uint16_t tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	tmp = ((Tcc *)hw)->PATTB.reg;
	tmp &= mask;
	return tmp;
}

static inline void hri_tcc_write_PATTB_reg(const void *const hw, hri_tcc_pattb_reg_t data)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATTB.reg = data;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_PATTB_reg(const void *const hw, hri_tcc_pattb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATTB.reg &= ~mask;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_PATTB_reg(const void *const hw, hri_tcc_pattb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PATTB.reg ^= mask;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_pattb_reg_t hri_tcc_read_PATTB_reg(const void *const hw)
{
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	return ((Tcc *)hw)->PATTB.reg;
}

static inline void hri_tcc_set_WAVEB_CIPERENB_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVEB.reg |= TCC_WAVEB_CIPERENB;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_WAVEB_CIPERENB_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->WAVEB.reg;
	tmp = (tmp & TCC_WAVEB_CIPERENB) >> TCC_WAVEB_CIPERENB_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_WAVEB_CIPERENB_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->WAVEB.reg;
	tmp &= ~TCC_WAVEB_CIPERENB;
	tmp |= value << TCC_WAVEB_CIPERENB_Pos;
	((Tcc *)hw)->WAVEB.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_WAVEB_CIPERENB_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVEB.reg &= ~TCC_WAVEB_CIPERENB;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_WAVEB_CIPERENB_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVEB.reg ^= TCC_WAVEB_CIPERENB;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_WAVEB_CICCENB0_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVEB.reg |= TCC_WAVEB_CICCENB0;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_WAVEB_CICCENB0_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->WAVEB.reg;
	tmp = (tmp & TCC_WAVEB_CICCENB0) >> TCC_WAVEB_CICCENB0_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_WAVEB_CICCENB0_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->WAVEB.reg;
	tmp &= ~TCC_WAVEB_CICCENB0;
	tmp |= value << TCC_WAVEB_CICCENB0_Pos;
	((Tcc *)hw)->WAVEB.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_WAVEB_CICCENB0_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVEB.reg &= ~TCC_WAVEB_CICCENB0;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_WAVEB_CICCENB0_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVEB.reg ^= TCC_WAVEB_CICCENB0;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_WAVEB_CICCENB1_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVEB.reg |= TCC_WAVEB_CICCENB1;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_WAVEB_CICCENB1_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->WAVEB.reg;
	tmp = (tmp & TCC_WAVEB_CICCENB1) >> TCC_WAVEB_CICCENB1_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_WAVEB_CICCENB1_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->WAVEB.reg;
	tmp &= ~TCC_WAVEB_CICCENB1;
	tmp |= value << TCC_WAVEB_CICCENB1_Pos;
	((Tcc *)hw)->WAVEB.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_WAVEB_CICCENB1_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVEB.reg &= ~TCC_WAVEB_CICCENB1;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_WAVEB_CICCENB1_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVEB.reg ^= TCC_WAVEB_CICCENB1;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_WAVEB_CICCENB2_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVEB.reg |= TCC_WAVEB_CICCENB2;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_WAVEB_CICCENB2_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->WAVEB.reg;
	tmp = (tmp & TCC_WAVEB_CICCENB2) >> TCC_WAVEB_CICCENB2_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_WAVEB_CICCENB2_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->WAVEB.reg;
	tmp &= ~TCC_WAVEB_CICCENB2;
	tmp |= value << TCC_WAVEB_CICCENB2_Pos;
	((Tcc *)hw)->WAVEB.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_WAVEB_CICCENB2_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVEB.reg &= ~TCC_WAVEB_CICCENB2;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_WAVEB_CICCENB2_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVEB.reg ^= TCC_WAVEB_CICCENB2;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_WAVEB_CICCENB3_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVEB.reg |= TCC_WAVEB_CICCENB3;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_WAVEB_CICCENB3_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->WAVEB.reg;
	tmp = (tmp & TCC_WAVEB_CICCENB3) >> TCC_WAVEB_CICCENB3_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_WAVEB_CICCENB3_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->WAVEB.reg;
	tmp &= ~TCC_WAVEB_CICCENB3;
	tmp |= value << TCC_WAVEB_CICCENB3_Pos;
	((Tcc *)hw)->WAVEB.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_WAVEB_CICCENB3_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVEB.reg &= ~TCC_WAVEB_CICCENB3;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_WAVEB_CICCENB3_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVEB.reg ^= TCC_WAVEB_CICCENB3;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_WAVEB_POLB0_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVEB.reg |= TCC_WAVEB_POLB0;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_WAVEB_POLB0_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->WAVEB.reg;
	tmp = (tmp & TCC_WAVEB_POLB0) >> TCC_WAVEB_POLB0_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_WAVEB_POLB0_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->WAVEB.reg;
	tmp &= ~TCC_WAVEB_POLB0;
	tmp |= value << TCC_WAVEB_POLB0_Pos;
	((Tcc *)hw)->WAVEB.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_WAVEB_POLB0_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVEB.reg &= ~TCC_WAVEB_POLB0;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_WAVEB_POLB0_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVEB.reg ^= TCC_WAVEB_POLB0;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_WAVEB_POLB1_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVEB.reg |= TCC_WAVEB_POLB1;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_WAVEB_POLB1_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->WAVEB.reg;
	tmp = (tmp & TCC_WAVEB_POLB1) >> TCC_WAVEB_POLB1_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_WAVEB_POLB1_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->WAVEB.reg;
	tmp &= ~TCC_WAVEB_POLB1;
	tmp |= value << TCC_WAVEB_POLB1_Pos;
	((Tcc *)hw)->WAVEB.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_WAVEB_POLB1_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVEB.reg &= ~TCC_WAVEB_POLB1;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_WAVEB_POLB1_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVEB.reg ^= TCC_WAVEB_POLB1;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_WAVEB_POLB2_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVEB.reg |= TCC_WAVEB_POLB2;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_WAVEB_POLB2_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->WAVEB.reg;
	tmp = (tmp & TCC_WAVEB_POLB2) >> TCC_WAVEB_POLB2_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_WAVEB_POLB2_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->WAVEB.reg;
	tmp &= ~TCC_WAVEB_POLB2;
	tmp |= value << TCC_WAVEB_POLB2_Pos;
	((Tcc *)hw)->WAVEB.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_WAVEB_POLB2_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVEB.reg &= ~TCC_WAVEB_POLB2;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_WAVEB_POLB2_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVEB.reg ^= TCC_WAVEB_POLB2;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_WAVEB_POLB3_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVEB.reg |= TCC_WAVEB_POLB3;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_WAVEB_POLB3_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->WAVEB.reg;
	tmp = (tmp & TCC_WAVEB_POLB3) >> TCC_WAVEB_POLB3_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_WAVEB_POLB3_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->WAVEB.reg;
	tmp &= ~TCC_WAVEB_POLB3;
	tmp |= value << TCC_WAVEB_POLB3_Pos;
	((Tcc *)hw)->WAVEB.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_WAVEB_POLB3_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVEB.reg &= ~TCC_WAVEB_POLB3;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_WAVEB_POLB3_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVEB.reg ^= TCC_WAVEB_POLB3;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_WAVEB_SWAPB0_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVEB.reg |= TCC_WAVEB_SWAPB0;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_WAVEB_SWAPB0_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->WAVEB.reg;
	tmp = (tmp & TCC_WAVEB_SWAPB0) >> TCC_WAVEB_SWAPB0_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_WAVEB_SWAPB0_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->WAVEB.reg;
	tmp &= ~TCC_WAVEB_SWAPB0;
	tmp |= value << TCC_WAVEB_SWAPB0_Pos;
	((Tcc *)hw)->WAVEB.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_WAVEB_SWAPB0_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVEB.reg &= ~TCC_WAVEB_SWAPB0;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_WAVEB_SWAPB0_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVEB.reg ^= TCC_WAVEB_SWAPB0;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_WAVEB_SWAPB1_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVEB.reg |= TCC_WAVEB_SWAPB1;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_WAVEB_SWAPB1_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->WAVEB.reg;
	tmp = (tmp & TCC_WAVEB_SWAPB1) >> TCC_WAVEB_SWAPB1_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_WAVEB_SWAPB1_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->WAVEB.reg;
	tmp &= ~TCC_WAVEB_SWAPB1;
	tmp |= value << TCC_WAVEB_SWAPB1_Pos;
	((Tcc *)hw)->WAVEB.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_WAVEB_SWAPB1_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVEB.reg &= ~TCC_WAVEB_SWAPB1;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_WAVEB_SWAPB1_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVEB.reg ^= TCC_WAVEB_SWAPB1;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_WAVEB_SWAPB2_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVEB.reg |= TCC_WAVEB_SWAPB2;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_WAVEB_SWAPB2_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->WAVEB.reg;
	tmp = (tmp & TCC_WAVEB_SWAPB2) >> TCC_WAVEB_SWAPB2_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_WAVEB_SWAPB2_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->WAVEB.reg;
	tmp &= ~TCC_WAVEB_SWAPB2;
	tmp |= value << TCC_WAVEB_SWAPB2_Pos;
	((Tcc *)hw)->WAVEB.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_WAVEB_SWAPB2_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVEB.reg &= ~TCC_WAVEB_SWAPB2;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_WAVEB_SWAPB2_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVEB.reg ^= TCC_WAVEB_SWAPB2;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_WAVEB_SWAPB3_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVEB.reg |= TCC_WAVEB_SWAPB3;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_WAVEB_SWAPB3_bit(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->WAVEB.reg;
	tmp = (tmp & TCC_WAVEB_SWAPB3) >> TCC_WAVEB_SWAPB3_Pos;
	return (bool)tmp;
}

static inline void hri_tcc_write_WAVEB_SWAPB3_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->WAVEB.reg;
	tmp &= ~TCC_WAVEB_SWAPB3;
	tmp |= value << TCC_WAVEB_SWAPB3_Pos;
	((Tcc *)hw)->WAVEB.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_WAVEB_SWAPB3_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVEB.reg &= ~TCC_WAVEB_SWAPB3;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_WAVEB_SWAPB3_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVEB.reg ^= TCC_WAVEB_SWAPB3;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_set_WAVEB_WAVEGENB_bf(const void *const hw, hri_tcc_waveb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVEB.reg |= TCC_WAVEB_WAVEGENB(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_waveb_reg_t hri_tcc_get_WAVEB_WAVEGENB_bf(const void *const hw, hri_tcc_waveb_reg_t mask)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->WAVEB.reg;
	tmp = (tmp & TCC_WAVEB_WAVEGENB(mask)) >> TCC_WAVEB_WAVEGENB_Pos;
	return tmp;
}

static inline void hri_tcc_write_WAVEB_WAVEGENB_bf(const void *const hw, hri_tcc_waveb_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->WAVEB.reg;
	tmp &= ~TCC_WAVEB_WAVEGENB_Msk;
	tmp |= TCC_WAVEB_WAVEGENB(data);
	((Tcc *)hw)->WAVEB.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_WAVEB_WAVEGENB_bf(const void *const hw, hri_tcc_waveb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVEB.reg &= ~TCC_WAVEB_WAVEGENB(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_WAVEB_WAVEGENB_bf(const void *const hw, hri_tcc_waveb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVEB.reg ^= TCC_WAVEB_WAVEGENB(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_waveb_reg_t hri_tcc_read_WAVEB_WAVEGENB_bf(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->WAVEB.reg;
	tmp = (tmp & TCC_WAVEB_WAVEGENB_Msk) >> TCC_WAVEB_WAVEGENB_Pos;
	return tmp;
}

static inline void hri_tcc_set_WAVEB_RAMPB_bf(const void *const hw, hri_tcc_waveb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVEB.reg |= TCC_WAVEB_RAMPB(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_waveb_reg_t hri_tcc_get_WAVEB_RAMPB_bf(const void *const hw, hri_tcc_waveb_reg_t mask)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->WAVEB.reg;
	tmp = (tmp & TCC_WAVEB_RAMPB(mask)) >> TCC_WAVEB_RAMPB_Pos;
	return tmp;
}

static inline void hri_tcc_write_WAVEB_RAMPB_bf(const void *const hw, hri_tcc_waveb_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->WAVEB.reg;
	tmp &= ~TCC_WAVEB_RAMPB_Msk;
	tmp |= TCC_WAVEB_RAMPB(data);
	((Tcc *)hw)->WAVEB.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_WAVEB_RAMPB_bf(const void *const hw, hri_tcc_waveb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVEB.reg &= ~TCC_WAVEB_RAMPB(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_WAVEB_RAMPB_bf(const void *const hw, hri_tcc_waveb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVEB.reg ^= TCC_WAVEB_RAMPB(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_waveb_reg_t hri_tcc_read_WAVEB_RAMPB_bf(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->WAVEB.reg;
	tmp = (tmp & TCC_WAVEB_RAMPB_Msk) >> TCC_WAVEB_RAMPB_Pos;
	return tmp;
}

static inline void hri_tcc_set_WAVEB_reg(const void *const hw, hri_tcc_waveb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVEB.reg |= mask;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_waveb_reg_t hri_tcc_get_WAVEB_reg(const void *const hw, hri_tcc_waveb_reg_t mask)
{
	uint32_t tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	tmp = ((Tcc *)hw)->WAVEB.reg;
	tmp &= mask;
	return tmp;
}

static inline void hri_tcc_write_WAVEB_reg(const void *const hw, hri_tcc_waveb_reg_t data)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVEB.reg = data;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_WAVEB_reg(const void *const hw, hri_tcc_waveb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVEB.reg &= ~mask;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_WAVEB_reg(const void *const hw, hri_tcc_waveb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->WAVEB.reg ^= mask;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_waveb_reg_t hri_tcc_read_WAVEB_reg(const void *const hw)
{
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	return ((Tcc *)hw)->WAVEB.reg;
}

static inline void hri_tcc_set_PERB_DITH4_DITHERCYB_bf(const void *const hw, hri_tcc_perb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PERB.reg |= TCC_PERB_DITH4_DITHERCYB(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_perb_reg_t hri_tcc_get_PERB_DITH4_DITHERCYB_bf(const void *const hw, hri_tcc_perb_reg_t mask)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->PERB.reg;
	tmp = (tmp & TCC_PERB_DITH4_DITHERCYB(mask)) >> TCC_PERB_DITH4_DITHERCYB_Pos;
	return tmp;
}

static inline void hri_tcc_write_PERB_DITH4_DITHERCYB_bf(const void *const hw, hri_tcc_perb_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->PERB.reg;
	tmp &= ~TCC_PERB_DITH4_DITHERCYB_Msk;
	tmp |= TCC_PERB_DITH4_DITHERCYB(data);
	((Tcc *)hw)->PERB.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_PERB_DITH4_DITHERCYB_bf(const void *const hw, hri_tcc_perb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PERB.reg &= ~TCC_PERB_DITH4_DITHERCYB(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_PERB_DITH4_DITHERCYB_bf(const void *const hw, hri_tcc_perb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PERB.reg ^= TCC_PERB_DITH4_DITHERCYB(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_perb_reg_t hri_tcc_read_PERB_DITH4_DITHERCYB_bf(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->PERB.reg;
	tmp = (tmp & TCC_PERB_DITH4_DITHERCYB_Msk) >> TCC_PERB_DITH4_DITHERCYB_Pos;
	return tmp;
}

static inline void hri_tcc_set_PERB_DITH5_DITHERCYB_bf(const void *const hw, hri_tcc_perb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PERB.reg |= TCC_PERB_DITH5_DITHERCYB(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_perb_reg_t hri_tcc_get_PERB_DITH5_DITHERCYB_bf(const void *const hw, hri_tcc_perb_reg_t mask)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->PERB.reg;
	tmp = (tmp & TCC_PERB_DITH5_DITHERCYB(mask)) >> TCC_PERB_DITH5_DITHERCYB_Pos;
	return tmp;
}

static inline void hri_tcc_write_PERB_DITH5_DITHERCYB_bf(const void *const hw, hri_tcc_perb_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->PERB.reg;
	tmp &= ~TCC_PERB_DITH5_DITHERCYB_Msk;
	tmp |= TCC_PERB_DITH5_DITHERCYB(data);
	((Tcc *)hw)->PERB.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_PERB_DITH5_DITHERCYB_bf(const void *const hw, hri_tcc_perb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PERB.reg &= ~TCC_PERB_DITH5_DITHERCYB(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_PERB_DITH5_DITHERCYB_bf(const void *const hw, hri_tcc_perb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PERB.reg ^= TCC_PERB_DITH5_DITHERCYB(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_perb_reg_t hri_tcc_read_PERB_DITH5_DITHERCYB_bf(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->PERB.reg;
	tmp = (tmp & TCC_PERB_DITH5_DITHERCYB_Msk) >> TCC_PERB_DITH5_DITHERCYB_Pos;
	return tmp;
}

static inline void hri_tcc_set_PERB_DITH6_DITHERCYB_bf(const void *const hw, hri_tcc_perb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PERB.reg |= TCC_PERB_DITH6_DITHERCYB(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_perb_reg_t hri_tcc_get_PERB_DITH6_DITHERCYB_bf(const void *const hw, hri_tcc_perb_reg_t mask)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->PERB.reg;
	tmp = (tmp & TCC_PERB_DITH6_DITHERCYB(mask)) >> TCC_PERB_DITH6_DITHERCYB_Pos;
	return tmp;
}

static inline void hri_tcc_write_PERB_DITH6_DITHERCYB_bf(const void *const hw, hri_tcc_perb_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->PERB.reg;
	tmp &= ~TCC_PERB_DITH6_DITHERCYB_Msk;
	tmp |= TCC_PERB_DITH6_DITHERCYB(data);
	((Tcc *)hw)->PERB.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_PERB_DITH6_DITHERCYB_bf(const void *const hw, hri_tcc_perb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PERB.reg &= ~TCC_PERB_DITH6_DITHERCYB(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_PERB_DITH6_DITHERCYB_bf(const void *const hw, hri_tcc_perb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PERB.reg ^= TCC_PERB_DITH6_DITHERCYB(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_perb_reg_t hri_tcc_read_PERB_DITH6_DITHERCYB_bf(const void *const hw)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->PERB.reg;
	tmp = (tmp & TCC_PERB_DITH6_DITHERCYB_Msk) >> TCC_PERB_DITH6_DITHERCYB_Pos;
	return tmp;
}

static inline void hri_tcc_set_PERB_DITH6_PERB_bf(const void *const hw, hri_tcc_perb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PERB.reg |= TCC_PERB_PERB(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_PERB);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_perb_reg_t hri_tcc_get_PERB_DITH6_PERB_bf(const void *const hw, hri_tcc_perb_reg_t mask)
{
	uint32_t tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_PERB);
	tmp = ((Tcc *)hw)->PERB.reg;
	tmp = (tmp & TCC_PERB_PERB(mask)) >> TCC_PERB_PERB_Pos;
	return tmp;
}

static inline void hri_tcc_write_PERB_DITH6_PERB_bf(const void *const hw, hri_tcc_perb_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->PERB.reg;
	tmp &= ~TCC_PERB_PERB_Msk;
	tmp |= TCC_PERB_PERB(data);
	((Tcc *)hw)->PERB.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_PERB);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_PERB_DITH6_PERB_bf(const void *const hw, hri_tcc_perb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PERB.reg &= ~TCC_PERB_PERB(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_PERB);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_PERB_DITH6_PERB_bf(const void *const hw, hri_tcc_perb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PERB.reg ^= TCC_PERB_PERB(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_PERB);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_perb_reg_t hri_tcc_read_PERB_DITH6_PERB_bf(const void *const hw)
{
	uint32_t tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_PERB);
	tmp = ((Tcc *)hw)->PERB.reg;
	tmp = (tmp & TCC_PERB_PERB_Msk) >> TCC_PERB_PERB_Pos;
	return tmp;
}

static inline void hri_tcc_set_PERB_DITH5_PERB_bf(const void *const hw, hri_tcc_perb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PERB.reg |= TCC_PERB_PERB(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_PERB);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_perb_reg_t hri_tcc_get_PERB_DITH5_PERB_bf(const void *const hw, hri_tcc_perb_reg_t mask)
{
	uint32_t tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_PERB);
	tmp = ((Tcc *)hw)->PERB.reg;
	tmp = (tmp & TCC_PERB_PERB(mask)) >> TCC_PERB_PERB_Pos;
	return tmp;
}

static inline void hri_tcc_write_PERB_DITH5_PERB_bf(const void *const hw, hri_tcc_perb_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->PERB.reg;
	tmp &= ~TCC_PERB_PERB_Msk;
	tmp |= TCC_PERB_PERB(data);
	((Tcc *)hw)->PERB.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_PERB);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_PERB_DITH5_PERB_bf(const void *const hw, hri_tcc_perb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PERB.reg &= ~TCC_PERB_PERB(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_PERB);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_PERB_DITH5_PERB_bf(const void *const hw, hri_tcc_perb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PERB.reg ^= TCC_PERB_PERB(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_PERB);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_perb_reg_t hri_tcc_read_PERB_DITH5_PERB_bf(const void *const hw)
{
	uint32_t tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_PERB);
	tmp = ((Tcc *)hw)->PERB.reg;
	tmp = (tmp & TCC_PERB_PERB_Msk) >> TCC_PERB_PERB_Pos;
	return tmp;
}

static inline void hri_tcc_set_PERB_DITH4_PERB_bf(const void *const hw, hri_tcc_perb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PERB.reg |= TCC_PERB_PERB(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_PERB);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_perb_reg_t hri_tcc_get_PERB_DITH4_PERB_bf(const void *const hw, hri_tcc_perb_reg_t mask)
{
	uint32_t tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_PERB);
	tmp = ((Tcc *)hw)->PERB.reg;
	tmp = (tmp & TCC_PERB_PERB(mask)) >> TCC_PERB_PERB_Pos;
	return tmp;
}

static inline void hri_tcc_write_PERB_DITH4_PERB_bf(const void *const hw, hri_tcc_perb_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->PERB.reg;
	tmp &= ~TCC_PERB_PERB_Msk;
	tmp |= TCC_PERB_PERB(data);
	((Tcc *)hw)->PERB.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_PERB);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_PERB_DITH4_PERB_bf(const void *const hw, hri_tcc_perb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PERB.reg &= ~TCC_PERB_PERB(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_PERB);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_PERB_DITH4_PERB_bf(const void *const hw, hri_tcc_perb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PERB.reg ^= TCC_PERB_PERB(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_PERB);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_perb_reg_t hri_tcc_read_PERB_DITH4_PERB_bf(const void *const hw)
{
	uint32_t tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_PERB);
	tmp = ((Tcc *)hw)->PERB.reg;
	tmp = (tmp & TCC_PERB_PERB_Msk) >> TCC_PERB_PERB_Pos;
	return tmp;
}

static inline void hri_tcc_set_PERB_PERB_bf(const void *const hw, hri_tcc_perb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PERB.reg |= TCC_PERB_PERB(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_PERB);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_perb_reg_t hri_tcc_get_PERB_PERB_bf(const void *const hw, hri_tcc_perb_reg_t mask)
{
	uint32_t tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_PERB);
	tmp = ((Tcc *)hw)->PERB.reg;
	tmp = (tmp & TCC_PERB_PERB(mask)) >> TCC_PERB_PERB_Pos;
	return tmp;
}

static inline void hri_tcc_write_PERB_PERB_bf(const void *const hw, hri_tcc_perb_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->PERB.reg;
	tmp &= ~TCC_PERB_PERB_Msk;
	tmp |= TCC_PERB_PERB(data);
	((Tcc *)hw)->PERB.reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_PERB);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_PERB_PERB_bf(const void *const hw, hri_tcc_perb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PERB.reg &= ~TCC_PERB_PERB(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_PERB);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_PERB_PERB_bf(const void *const hw, hri_tcc_perb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PERB.reg ^= TCC_PERB_PERB(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_PERB);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_perb_reg_t hri_tcc_read_PERB_PERB_bf(const void *const hw)
{
	uint32_t tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_PERB);
	tmp = ((Tcc *)hw)->PERB.reg;
	tmp = (tmp & TCC_PERB_PERB_Msk) >> TCC_PERB_PERB_Pos;
	return tmp;
}

static inline void hri_tcc_set_PERB_reg(const void *const hw, hri_tcc_perb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PERB.reg |= mask;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_PERB);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_perb_reg_t hri_tcc_get_PERB_reg(const void *const hw, hri_tcc_perb_reg_t mask)
{
	uint32_t tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_PERB);
	tmp = ((Tcc *)hw)->PERB.reg;
	tmp &= mask;
	return tmp;
}

static inline void hri_tcc_write_PERB_reg(const void *const hw, hri_tcc_perb_reg_t data)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PERB.reg = data;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_PERB);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_PERB_reg(const void *const hw, hri_tcc_perb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PERB.reg &= ~mask;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_PERB);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_PERB_reg(const void *const hw, hri_tcc_perb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->PERB.reg ^= mask;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_PERB);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_perb_reg_t hri_tcc_read_PERB_reg(const void *const hw)
{
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_PERB);
	return ((Tcc *)hw)->PERB.reg;
}

static inline void hri_tcc_set_CCB_DITH4_DITHERCYB_bf(const void *const hw, uint8_t index, hri_tcc_ccb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CCB[index].reg |= TCC_CCB_DITH4_DITHERCYB(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_ccb_reg_t hri_tcc_get_CCB_DITH4_DITHERCYB_bf(const void *const hw, uint8_t index,
                                                                   hri_tcc_ccb_reg_t mask)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->CCB[index].reg;
	tmp = (tmp & TCC_CCB_DITH4_DITHERCYB(mask)) >> TCC_CCB_DITH4_DITHERCYB_Pos;
	return tmp;
}

static inline void hri_tcc_write_CCB_DITH4_DITHERCYB_bf(const void *const hw, uint8_t index, hri_tcc_ccb_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->CCB[index].reg;
	tmp &= ~TCC_CCB_DITH4_DITHERCYB_Msk;
	tmp |= TCC_CCB_DITH4_DITHERCYB(data);
	((Tcc *)hw)->CCB[index].reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_CCB_DITH4_DITHERCYB_bf(const void *const hw, uint8_t index, hri_tcc_ccb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CCB[index].reg &= ~TCC_CCB_DITH4_DITHERCYB(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_CCB_DITH4_DITHERCYB_bf(const void *const hw, uint8_t index, hri_tcc_ccb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CCB[index].reg ^= TCC_CCB_DITH4_DITHERCYB(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_ccb_reg_t hri_tcc_read_CCB_DITH4_DITHERCYB_bf(const void *const hw, uint8_t index)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->CCB[index].reg;
	tmp = (tmp & TCC_CCB_DITH4_DITHERCYB_Msk) >> TCC_CCB_DITH4_DITHERCYB_Pos;
	return tmp;
}

static inline void hri_tcc_set_CCB_DITH5_DITHERCYB_bf(const void *const hw, uint8_t index, hri_tcc_ccb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CCB[index].reg |= TCC_CCB_DITH5_DITHERCYB(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_ccb_reg_t hri_tcc_get_CCB_DITH5_DITHERCYB_bf(const void *const hw, uint8_t index,
                                                                   hri_tcc_ccb_reg_t mask)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->CCB[index].reg;
	tmp = (tmp & TCC_CCB_DITH5_DITHERCYB(mask)) >> TCC_CCB_DITH5_DITHERCYB_Pos;
	return tmp;
}

static inline void hri_tcc_write_CCB_DITH5_DITHERCYB_bf(const void *const hw, uint8_t index, hri_tcc_ccb_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->CCB[index].reg;
	tmp &= ~TCC_CCB_DITH5_DITHERCYB_Msk;
	tmp |= TCC_CCB_DITH5_DITHERCYB(data);
	((Tcc *)hw)->CCB[index].reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_CCB_DITH5_DITHERCYB_bf(const void *const hw, uint8_t index, hri_tcc_ccb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CCB[index].reg &= ~TCC_CCB_DITH5_DITHERCYB(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_CCB_DITH5_DITHERCYB_bf(const void *const hw, uint8_t index, hri_tcc_ccb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CCB[index].reg ^= TCC_CCB_DITH5_DITHERCYB(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_ccb_reg_t hri_tcc_read_CCB_DITH5_DITHERCYB_bf(const void *const hw, uint8_t index)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->CCB[index].reg;
	tmp = (tmp & TCC_CCB_DITH5_DITHERCYB_Msk) >> TCC_CCB_DITH5_DITHERCYB_Pos;
	return tmp;
}

static inline void hri_tcc_set_CCB_DITH6_DITHERCYB_bf(const void *const hw, uint8_t index, hri_tcc_ccb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CCB[index].reg |= TCC_CCB_DITH6_DITHERCYB(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_ccb_reg_t hri_tcc_get_CCB_DITH6_DITHERCYB_bf(const void *const hw, uint8_t index,
                                                                   hri_tcc_ccb_reg_t mask)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->CCB[index].reg;
	tmp = (tmp & TCC_CCB_DITH6_DITHERCYB(mask)) >> TCC_CCB_DITH6_DITHERCYB_Pos;
	return tmp;
}

static inline void hri_tcc_write_CCB_DITH6_DITHERCYB_bf(const void *const hw, uint8_t index, hri_tcc_ccb_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->CCB[index].reg;
	tmp &= ~TCC_CCB_DITH6_DITHERCYB_Msk;
	tmp |= TCC_CCB_DITH6_DITHERCYB(data);
	((Tcc *)hw)->CCB[index].reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_CCB_DITH6_DITHERCYB_bf(const void *const hw, uint8_t index, hri_tcc_ccb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CCB[index].reg &= ~TCC_CCB_DITH6_DITHERCYB(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_CCB_DITH6_DITHERCYB_bf(const void *const hw, uint8_t index, hri_tcc_ccb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CCB[index].reg ^= TCC_CCB_DITH6_DITHERCYB(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_ccb_reg_t hri_tcc_read_CCB_DITH6_DITHERCYB_bf(const void *const hw, uint8_t index)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->CCB[index].reg;
	tmp = (tmp & TCC_CCB_DITH6_DITHERCYB_Msk) >> TCC_CCB_DITH6_DITHERCYB_Pos;
	return tmp;
}

static inline void hri_tcc_set_CCB_DITH6_CCB_bf(const void *const hw, uint8_t index, hri_tcc_ccb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CCB[index].reg |= TCC_CCB_CCB(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_CCB0 | TCC_SYNCBUSY_CCB1 | TCC_SYNCBUSY_CCB2 | TCC_SYNCBUSY_CCB3);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_ccb_reg_t hri_tcc_get_CCB_DITH6_CCB_bf(const void *const hw, uint8_t index,
                                                             hri_tcc_ccb_reg_t mask)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->CCB[index].reg;
	tmp = (tmp & TCC_CCB_CCB(mask)) >> TCC_CCB_CCB_Pos;
	return tmp;
}

static inline void hri_tcc_write_CCB_DITH6_CCB_bf(const void *const hw, uint8_t index, hri_tcc_ccb_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->CCB[index].reg;
	tmp &= ~TCC_CCB_CCB_Msk;
	tmp |= TCC_CCB_CCB(data);
	((Tcc *)hw)->CCB[index].reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_CCB0 | TCC_SYNCBUSY_CCB1 | TCC_SYNCBUSY_CCB2 | TCC_SYNCBUSY_CCB3);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_CCB_DITH6_CCB_bf(const void *const hw, uint8_t index, hri_tcc_ccb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CCB[index].reg &= ~TCC_CCB_CCB(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_CCB0 | TCC_SYNCBUSY_CCB1 | TCC_SYNCBUSY_CCB2 | TCC_SYNCBUSY_CCB3);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_CCB_DITH6_CCB_bf(const void *const hw, uint8_t index, hri_tcc_ccb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CCB[index].reg ^= TCC_CCB_CCB(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_CCB0 | TCC_SYNCBUSY_CCB1 | TCC_SYNCBUSY_CCB2 | TCC_SYNCBUSY_CCB3);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_ccb_reg_t hri_tcc_read_CCB_DITH6_CCB_bf(const void *const hw, uint8_t index)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->CCB[index].reg;
	tmp = (tmp & TCC_CCB_CCB_Msk) >> TCC_CCB_CCB_Pos;
	return tmp;
}

static inline void hri_tcc_set_CCB_DITH5_CCB_bf(const void *const hw, uint8_t index, hri_tcc_ccb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CCB[index].reg |= TCC_CCB_CCB(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_CCB0 | TCC_SYNCBUSY_CCB1 | TCC_SYNCBUSY_CCB2 | TCC_SYNCBUSY_CCB3);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_ccb_reg_t hri_tcc_get_CCB_DITH5_CCB_bf(const void *const hw, uint8_t index,
                                                             hri_tcc_ccb_reg_t mask)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->CCB[index].reg;
	tmp = (tmp & TCC_CCB_CCB(mask)) >> TCC_CCB_CCB_Pos;
	return tmp;
}

static inline void hri_tcc_write_CCB_DITH5_CCB_bf(const void *const hw, uint8_t index, hri_tcc_ccb_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->CCB[index].reg;
	tmp &= ~TCC_CCB_CCB_Msk;
	tmp |= TCC_CCB_CCB(data);
	((Tcc *)hw)->CCB[index].reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_CCB0 | TCC_SYNCBUSY_CCB1 | TCC_SYNCBUSY_CCB2 | TCC_SYNCBUSY_CCB3);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_CCB_DITH5_CCB_bf(const void *const hw, uint8_t index, hri_tcc_ccb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CCB[index].reg &= ~TCC_CCB_CCB(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_CCB0 | TCC_SYNCBUSY_CCB1 | TCC_SYNCBUSY_CCB2 | TCC_SYNCBUSY_CCB3);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_CCB_DITH5_CCB_bf(const void *const hw, uint8_t index, hri_tcc_ccb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CCB[index].reg ^= TCC_CCB_CCB(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_CCB0 | TCC_SYNCBUSY_CCB1 | TCC_SYNCBUSY_CCB2 | TCC_SYNCBUSY_CCB3);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_ccb_reg_t hri_tcc_read_CCB_DITH5_CCB_bf(const void *const hw, uint8_t index)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->CCB[index].reg;
	tmp = (tmp & TCC_CCB_CCB_Msk) >> TCC_CCB_CCB_Pos;
	return tmp;
}

static inline void hri_tcc_set_CCB_DITH4_CCB_bf(const void *const hw, uint8_t index, hri_tcc_ccb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CCB[index].reg |= TCC_CCB_CCB(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_CCB0 | TCC_SYNCBUSY_CCB1 | TCC_SYNCBUSY_CCB2 | TCC_SYNCBUSY_CCB3);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_ccb_reg_t hri_tcc_get_CCB_DITH4_CCB_bf(const void *const hw, uint8_t index,
                                                             hri_tcc_ccb_reg_t mask)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->CCB[index].reg;
	tmp = (tmp & TCC_CCB_CCB(mask)) >> TCC_CCB_CCB_Pos;
	return tmp;
}

static inline void hri_tcc_write_CCB_DITH4_CCB_bf(const void *const hw, uint8_t index, hri_tcc_ccb_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->CCB[index].reg;
	tmp &= ~TCC_CCB_CCB_Msk;
	tmp |= TCC_CCB_CCB(data);
	((Tcc *)hw)->CCB[index].reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_CCB0 | TCC_SYNCBUSY_CCB1 | TCC_SYNCBUSY_CCB2 | TCC_SYNCBUSY_CCB3);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_CCB_DITH4_CCB_bf(const void *const hw, uint8_t index, hri_tcc_ccb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CCB[index].reg &= ~TCC_CCB_CCB(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_CCB0 | TCC_SYNCBUSY_CCB1 | TCC_SYNCBUSY_CCB2 | TCC_SYNCBUSY_CCB3);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_CCB_DITH4_CCB_bf(const void *const hw, uint8_t index, hri_tcc_ccb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CCB[index].reg ^= TCC_CCB_CCB(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_CCB0 | TCC_SYNCBUSY_CCB1 | TCC_SYNCBUSY_CCB2 | TCC_SYNCBUSY_CCB3);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_ccb_reg_t hri_tcc_read_CCB_DITH4_CCB_bf(const void *const hw, uint8_t index)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->CCB[index].reg;
	tmp = (tmp & TCC_CCB_CCB_Msk) >> TCC_CCB_CCB_Pos;
	return tmp;
}

static inline void hri_tcc_set_CCB_CCB_bf(const void *const hw, uint8_t index, hri_tcc_ccb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CCB[index].reg |= TCC_CCB_CCB(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_CCB0 | TCC_SYNCBUSY_CCB1 | TCC_SYNCBUSY_CCB2 | TCC_SYNCBUSY_CCB3);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_ccb_reg_t hri_tcc_get_CCB_CCB_bf(const void *const hw, uint8_t index, hri_tcc_ccb_reg_t mask)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->CCB[index].reg;
	tmp = (tmp & TCC_CCB_CCB(mask)) >> TCC_CCB_CCB_Pos;
	return tmp;
}

static inline void hri_tcc_write_CCB_CCB_bf(const void *const hw, uint8_t index, hri_tcc_ccb_reg_t data)
{
	uint32_t tmp;
	TCC_CRITICAL_SECTION_ENTER();
	tmp = ((Tcc *)hw)->CCB[index].reg;
	tmp &= ~TCC_CCB_CCB_Msk;
	tmp |= TCC_CCB_CCB(data);
	((Tcc *)hw)->CCB[index].reg = tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_CCB0 | TCC_SYNCBUSY_CCB1 | TCC_SYNCBUSY_CCB2 | TCC_SYNCBUSY_CCB3);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_CCB_CCB_bf(const void *const hw, uint8_t index, hri_tcc_ccb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CCB[index].reg &= ~TCC_CCB_CCB(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_CCB0 | TCC_SYNCBUSY_CCB1 | TCC_SYNCBUSY_CCB2 | TCC_SYNCBUSY_CCB3);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_CCB_CCB_bf(const void *const hw, uint8_t index, hri_tcc_ccb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CCB[index].reg ^= TCC_CCB_CCB(mask);
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_CCB0 | TCC_SYNCBUSY_CCB1 | TCC_SYNCBUSY_CCB2 | TCC_SYNCBUSY_CCB3);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_ccb_reg_t hri_tcc_read_CCB_CCB_bf(const void *const hw, uint8_t index)
{
	uint32_t tmp;
	tmp = ((Tcc *)hw)->CCB[index].reg;
	tmp = (tmp & TCC_CCB_CCB_Msk) >> TCC_CCB_CCB_Pos;
	return tmp;
}

static inline void hri_tcc_set_CCB_reg(const void *const hw, uint8_t index, hri_tcc_ccb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CCB[index].reg |= mask;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_CCB0 | TCC_SYNCBUSY_CCB1 | TCC_SYNCBUSY_CCB2 | TCC_SYNCBUSY_CCB3);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_ccb_reg_t hri_tcc_get_CCB_reg(const void *const hw, uint8_t index, hri_tcc_ccb_reg_t mask)
{
	uint32_t tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_CCB0 | TCC_SYNCBUSY_CCB1 | TCC_SYNCBUSY_CCB2 | TCC_SYNCBUSY_CCB3);
	tmp = ((Tcc *)hw)->CCB[index].reg;
	tmp &= mask;
	return tmp;
}

static inline void hri_tcc_write_CCB_reg(const void *const hw, uint8_t index, hri_tcc_ccb_reg_t data)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CCB[index].reg = data;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_CCB0 | TCC_SYNCBUSY_CCB1 | TCC_SYNCBUSY_CCB2 | TCC_SYNCBUSY_CCB3);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_clear_CCB_reg(const void *const hw, uint8_t index, hri_tcc_ccb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CCB[index].reg &= ~mask;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_CCB0 | TCC_SYNCBUSY_CCB1 | TCC_SYNCBUSY_CCB2 | TCC_SYNCBUSY_CCB3);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline void hri_tcc_toggle_CCB_reg(const void *const hw, uint8_t index, hri_tcc_ccb_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->CCB[index].reg ^= mask;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_CCB0 | TCC_SYNCBUSY_CCB1 | TCC_SYNCBUSY_CCB2 | TCC_SYNCBUSY_CCB3);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_ccb_reg_t hri_tcc_read_CCB_reg(const void *const hw, uint8_t index)
{
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_CCB0 | TCC_SYNCBUSY_CCB1 | TCC_SYNCBUSY_CCB2 | TCC_SYNCBUSY_CCB3);
	return ((Tcc *)hw)->CCB[index].reg;
}

static inline bool hri_tcc_get_STATUS_STOP_bit(const void *const hw)
{
	return (((Tcc *)hw)->STATUS.reg & TCC_STATUS_STOP) >> TCC_STATUS_STOP_Pos;
}

static inline void hri_tcc_clear_STATUS_STOP_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->STATUS.reg = TCC_STATUS_STOP;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_STATUS_IDX_bit(const void *const hw)
{
	return (((Tcc *)hw)->STATUS.reg & TCC_STATUS_IDX) >> TCC_STATUS_IDX_Pos;
}

static inline void hri_tcc_clear_STATUS_IDX_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->STATUS.reg = TCC_STATUS_IDX;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_STATUS_DFS_bit(const void *const hw)
{
	return (((Tcc *)hw)->STATUS.reg & TCC_STATUS_DFS) >> TCC_STATUS_DFS_Pos;
}

static inline void hri_tcc_clear_STATUS_DFS_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->STATUS.reg = TCC_STATUS_DFS;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_STATUS_SLAVE_bit(const void *const hw)
{
	return (((Tcc *)hw)->STATUS.reg & TCC_STATUS_SLAVE) >> TCC_STATUS_SLAVE_Pos;
}

static inline void hri_tcc_clear_STATUS_SLAVE_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->STATUS.reg = TCC_STATUS_SLAVE;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_STATUS_PATTBV_bit(const void *const hw)
{
	return (((Tcc *)hw)->STATUS.reg & TCC_STATUS_PATTBV) >> TCC_STATUS_PATTBV_Pos;
}

static inline void hri_tcc_clear_STATUS_PATTBV_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->STATUS.reg = TCC_STATUS_PATTBV;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_STATUS_WAVEBV_bit(const void *const hw)
{
	return (((Tcc *)hw)->STATUS.reg & TCC_STATUS_WAVEBV) >> TCC_STATUS_WAVEBV_Pos;
}

static inline void hri_tcc_clear_STATUS_WAVEBV_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->STATUS.reg = TCC_STATUS_WAVEBV;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_STATUS_PERBV_bit(const void *const hw)
{
	return (((Tcc *)hw)->STATUS.reg & TCC_STATUS_PERBV) >> TCC_STATUS_PERBV_Pos;
}

static inline void hri_tcc_clear_STATUS_PERBV_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->STATUS.reg = TCC_STATUS_PERBV;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_STATUS_FAULTAIN_bit(const void *const hw)
{
	return (((Tcc *)hw)->STATUS.reg & TCC_STATUS_FAULTAIN) >> TCC_STATUS_FAULTAIN_Pos;
}

static inline void hri_tcc_clear_STATUS_FAULTAIN_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->STATUS.reg = TCC_STATUS_FAULTAIN;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_STATUS_FAULTBIN_bit(const void *const hw)
{
	return (((Tcc *)hw)->STATUS.reg & TCC_STATUS_FAULTBIN) >> TCC_STATUS_FAULTBIN_Pos;
}

static inline void hri_tcc_clear_STATUS_FAULTBIN_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->STATUS.reg = TCC_STATUS_FAULTBIN;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_STATUS_FAULT0IN_bit(const void *const hw)
{
	return (((Tcc *)hw)->STATUS.reg & TCC_STATUS_FAULT0IN) >> TCC_STATUS_FAULT0IN_Pos;
}

static inline void hri_tcc_clear_STATUS_FAULT0IN_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->STATUS.reg = TCC_STATUS_FAULT0IN;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_STATUS_FAULT1IN_bit(const void *const hw)
{
	return (((Tcc *)hw)->STATUS.reg & TCC_STATUS_FAULT1IN) >> TCC_STATUS_FAULT1IN_Pos;
}

static inline void hri_tcc_clear_STATUS_FAULT1IN_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->STATUS.reg = TCC_STATUS_FAULT1IN;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_STATUS_FAULTA_bit(const void *const hw)
{
	return (((Tcc *)hw)->STATUS.reg & TCC_STATUS_FAULTA) >> TCC_STATUS_FAULTA_Pos;
}

static inline void hri_tcc_clear_STATUS_FAULTA_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->STATUS.reg = TCC_STATUS_FAULTA;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_STATUS_FAULTB_bit(const void *const hw)
{
	return (((Tcc *)hw)->STATUS.reg & TCC_STATUS_FAULTB) >> TCC_STATUS_FAULTB_Pos;
}

static inline void hri_tcc_clear_STATUS_FAULTB_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->STATUS.reg = TCC_STATUS_FAULTB;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_STATUS_FAULT0_bit(const void *const hw)
{
	return (((Tcc *)hw)->STATUS.reg & TCC_STATUS_FAULT0) >> TCC_STATUS_FAULT0_Pos;
}

static inline void hri_tcc_clear_STATUS_FAULT0_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->STATUS.reg = TCC_STATUS_FAULT0;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_STATUS_FAULT1_bit(const void *const hw)
{
	return (((Tcc *)hw)->STATUS.reg & TCC_STATUS_FAULT1) >> TCC_STATUS_FAULT1_Pos;
}

static inline void hri_tcc_clear_STATUS_FAULT1_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->STATUS.reg = TCC_STATUS_FAULT1;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_STATUS_CCBV0_bit(const void *const hw)
{
	return (((Tcc *)hw)->STATUS.reg & TCC_STATUS_CCBV0) >> TCC_STATUS_CCBV0_Pos;
}

static inline void hri_tcc_clear_STATUS_CCBV0_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->STATUS.reg = TCC_STATUS_CCBV0;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_STATUS_CCBV1_bit(const void *const hw)
{
	return (((Tcc *)hw)->STATUS.reg & TCC_STATUS_CCBV1) >> TCC_STATUS_CCBV1_Pos;
}

static inline void hri_tcc_clear_STATUS_CCBV1_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->STATUS.reg = TCC_STATUS_CCBV1;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_STATUS_CCBV2_bit(const void *const hw)
{
	return (((Tcc *)hw)->STATUS.reg & TCC_STATUS_CCBV2) >> TCC_STATUS_CCBV2_Pos;
}

static inline void hri_tcc_clear_STATUS_CCBV2_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->STATUS.reg = TCC_STATUS_CCBV2;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_STATUS_CCBV3_bit(const void *const hw)
{
	return (((Tcc *)hw)->STATUS.reg & TCC_STATUS_CCBV3) >> TCC_STATUS_CCBV3_Pos;
}

static inline void hri_tcc_clear_STATUS_CCBV3_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->STATUS.reg = TCC_STATUS_CCBV3;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_STATUS_CMP0_bit(const void *const hw)
{
	return (((Tcc *)hw)->STATUS.reg & TCC_STATUS_CMP0) >> TCC_STATUS_CMP0_Pos;
}

static inline void hri_tcc_clear_STATUS_CMP0_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->STATUS.reg = TCC_STATUS_CMP0;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_STATUS_CMP1_bit(const void *const hw)
{
	return (((Tcc *)hw)->STATUS.reg & TCC_STATUS_CMP1) >> TCC_STATUS_CMP1_Pos;
}

static inline void hri_tcc_clear_STATUS_CMP1_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->STATUS.reg = TCC_STATUS_CMP1;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_STATUS_CMP2_bit(const void *const hw)
{
	return (((Tcc *)hw)->STATUS.reg & TCC_STATUS_CMP2) >> TCC_STATUS_CMP2_Pos;
}

static inline void hri_tcc_clear_STATUS_CMP2_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->STATUS.reg = TCC_STATUS_CMP2;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline bool hri_tcc_get_STATUS_CMP3_bit(const void *const hw)
{
	return (((Tcc *)hw)->STATUS.reg & TCC_STATUS_CMP3) >> TCC_STATUS_CMP3_Pos;
}

static inline void hri_tcc_clear_STATUS_CMP3_bit(const void *const hw)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->STATUS.reg = TCC_STATUS_CMP3;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_status_reg_t hri_tcc_get_STATUS_reg(const void *const hw, hri_tcc_status_reg_t mask)
{
	uint32_t tmp;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	tmp = ((Tcc *)hw)->STATUS.reg;
	tmp &= mask;
	return tmp;
}

static inline void hri_tcc_clear_STATUS_reg(const void *const hw, hri_tcc_status_reg_t mask)
{
	TCC_CRITICAL_SECTION_ENTER();
	((Tcc *)hw)->STATUS.reg = mask;
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	TCC_CRITICAL_SECTION_LEAVE();
}

static inline hri_tcc_status_reg_t hri_tcc_read_STATUS_reg(const void *const hw)
{
	hri_tcc_wait_for_sync(hw, TCC_SYNCBUSY_MASK);
	return ((Tcc *)hw)->STATUS.reg;
}

/* Below section is for legacy hri apis name, not recommended to use below left side apis in application */
#define hri_tcc_set_COUNT_DITH4_reg(a, b) hri_tcc_set_COUNT_reg(a, b)
#define hri_tcc_get_COUNT_DITH4_reg(a, b) hri_tcc_get_COUNT_reg(a, b)
#define hri_tcc_write_COUNT_DITH4_reg(a, b) hri_tcc_write_COUNT_reg(a, b)
#define hri_tcc_clear_COUNT_DITH4_reg(a, b) hri_tcc_clear_COUNT_reg(a, b)
#define hri_tcc_toggle_COUNT_DITH4_reg(a, b) hri_tcc_toggle_COUNT_reg(a, b)
#define hri_tcc_read_COUNT_DITH4_reg(a) hri_tcc_read_COUNT_reg(a)
#define hri_tcc_set_COUNT_DITH5_reg(a, b) hri_tcc_set_COUNT_reg(a, b)
#define hri_tcc_get_COUNT_DITH5_reg(a, b) hri_tcc_get_COUNT_reg(a, b)
#define hri_tcc_write_COUNT_DITH5_reg(a, b) hri_tcc_write_COUNT_reg(a, b)
#define hri_tcc_clear_COUNT_DITH5_reg(a, b) hri_tcc_clear_COUNT_reg(a, b)
#define hri_tcc_toggle_COUNT_DITH5_reg(a, b) hri_tcc_toggle_COUNT_reg(a, b)
#define hri_tcc_read_COUNT_DITH5_reg(a) hri_tcc_read_COUNT_reg(a)
#define hri_tcc_set_COUNT_DITH6_reg(a, b) hri_tcc_set_COUNT_reg(a, b)
#define hri_tcc_get_COUNT_DITH6_reg(a, b) hri_tcc_get_COUNT_reg(a, b)
#define hri_tcc_write_COUNT_DITH6_reg(a, b) hri_tcc_write_COUNT_reg(a, b)
#define hri_tcc_clear_COUNT_DITH6_reg(a, b) hri_tcc_clear_COUNT_reg(a, b)
#define hri_tcc_toggle_COUNT_DITH6_reg(a, b) hri_tcc_toggle_COUNT_reg(a, b)
#define hri_tcc_read_COUNT_DITH6_reg(a) hri_tcc_read_COUNT_reg(a)
#define hri_tcc_set_PER_DITH4_reg(a, b) hri_tcc_set_PER_reg(a, b)
#define hri_tcc_get_PER_DITH4_reg(a, b) hri_tcc_get_PER_reg(a, b)
#define hri_tcc_write_PER_DITH4_reg(a, b) hri_tcc_write_PER_reg(a, b)
#define hri_tcc_clear_PER_DITH4_reg(a, b) hri_tcc_clear_PER_reg(a, b)
#define hri_tcc_toggle_PER_DITH4_reg(a, b) hri_tcc_toggle_PER_reg(a, b)
#define hri_tcc_read_PER_DITH4_reg(a) hri_tcc_read_PER_reg(a)
#define hri_tcc_set_PER_DITH5_reg(a, b) hri_tcc_set_PER_reg(a, b)
#define hri_tcc_get_PER_DITH5_reg(a, b) hri_tcc_get_PER_reg(a, b)
#define hri_tcc_write_PER_DITH5_reg(a, b) hri_tcc_write_PER_reg(a, b)
#define hri_tcc_clear_PER_DITH5_reg(a, b) hri_tcc_clear_PER_reg(a, b)
#define hri_tcc_toggle_PER_DITH5_reg(a, b) hri_tcc_toggle_PER_reg(a, b)
#define hri_tcc_read_PER_DITH5_reg(a) hri_tcc_read_PER_reg(a)
#define hri_tcc_set_PER_DITH6_reg(a, b) hri_tcc_set_PER_reg(a, b)
#define hri_tcc_get_PER_DITH6_reg(a, b) hri_tcc_get_PER_reg(a, b)
#define hri_tcc_write_PER_DITH6_reg(a, b) hri_tcc_write_PER_reg(a, b)
#define hri_tcc_clear_PER_DITH6_reg(a, b) hri_tcc_clear_PER_reg(a, b)
#define hri_tcc_toggle_PER_DITH6_reg(a, b) hri_tcc_toggle_PER_reg(a, b)
#define hri_tcc_read_PER_DITH6_reg(a) hri_tcc_read_PER_reg(a)
#define hri_tcc_set_CC_DITH4_reg(a, b, c) hri_tcc_set_CC_reg(a, b, c)
#define hri_tcc_get_CC_DITH4_reg(a, b, c) hri_tcc_get_CC_reg(a, b, c)
#define hri_tcc_write_CC_DITH4_reg(a, b, c) hri_tcc_write_CC_reg(a, b, c)
#define hri_tcc_clear_CC_DITH4_reg(a, b, c) hri_tcc_clear_CC_reg(a, b, c)
#define hri_tcc_toggle_CC_DITH4_reg(a, b, c) hri_tcc_toggle_CC_reg(a, b, c)
#define hri_tcc_read_CC_DITH4_reg(a, b) hri_tcc_read_CC_reg(a, b)
#define hri_tcc_set_CC_DITH5_reg(a, b, c) hri_tcc_set_CC_reg(a, b, c)
#define hri_tcc_get_CC_DITH5_reg(a, b, c) hri_tcc_get_CC_reg(a, b, c)
#define hri_tcc_write_CC_DITH5_reg(a, b, c) hri_tcc_write_CC_reg(a, b, c)
#define hri_tcc_clear_CC_DITH5_reg(a, b, c) hri_tcc_clear_CC_reg(a, b, c)
#define hri_tcc_toggle_CC_DITH5_reg(a, b, c) hri_tcc_toggle_CC_reg(a, b, c)
#define hri_tcc_read_CC_DITH5_reg(a, b) hri_tcc_read_CC_reg(a, b)
#define hri_tcc_set_CC_DITH6_reg(a, b, c) hri_tcc_set_CC_reg(a, b, c)
#define hri_tcc_get_CC_DITH6_reg(a, b, c) hri_tcc_get_CC_reg(a, b, c)
#define hri_tcc_write_CC_DITH6_reg(a, b, c) hri_tcc_write_CC_reg(a, b, c)
#define hri_tcc_clear_CC_DITH6_reg(a, b, c) hri_tcc_clear_CC_reg(a, b, c)
#define hri_tcc_toggle_CC_DITH6_reg(a, b, c) hri_tcc_toggle_CC_reg(a, b, c)
#define hri_tcc_read_CC_DITH6_reg(a, b) hri_tcc_read_CC_reg(a, b)
#define hri_tcc_set_PERB_DITH4_reg(a, b) hri_tcc_set_PERB_reg(a, b)
#define hri_tcc_get_PERB_DITH4_reg(a, b) hri_tcc_get_PERB_reg(a, b)
#define hri_tcc_write_PERB_DITH4_reg(a, b) hri_tcc_write_PERB_reg(a, b)
#define hri_tcc_clear_PERB_DITH4_reg(a, b) hri_tcc_clear_PERB_reg(a, b)
#define hri_tcc_toggle_PERB_DITH4_reg(a, b) hri_tcc_toggle_PERB_reg(a, b)
#define hri_tcc_read_PERB_DITH4_reg(a) hri_tcc_read_PERB_reg(a)
#define hri_tcc_set_PERB_DITH5_reg(a, b) hri_tcc_set_PERB_reg(a, b)
#define hri_tcc_get_PERB_DITH5_reg(a, b) hri_tcc_get_PERB_reg(a, b)
#define hri_tcc_write_PERB_DITH5_reg(a, b) hri_tcc_write_PERB_reg(a, b)
#define hri_tcc_clear_PERB_DITH5_reg(a, b) hri_tcc_clear_PERB_reg(a, b)
#define hri_tcc_toggle_PERB_DITH5_reg(a, b) hri_tcc_toggle_PERB_reg(a, b)
#define hri_tcc_read_PERB_DITH5_reg(a) hri_tcc_read_PERB_reg(a)
#define hri_tcc_set_PERB_DITH6_reg(a, b) hri_tcc_set_PERB_reg(a, b)
#define hri_tcc_get_PERB_DITH6_reg(a, b) hri_tcc_get_PERB_reg(a, b)
#define hri_tcc_write_PERB_DITH6_reg(a, b) hri_tcc_write_PERB_reg(a, b)
#define hri_tcc_clear_PERB_DITH6_reg(a, b) hri_tcc_clear_PERB_reg(a, b)
#define hri_tcc_toggle_PERB_DITH6_reg(a, b) hri_tcc_toggle_PERB_reg(a, b)
#define hri_tcc_read_PERB_DITH6_reg(a) hri_tcc_read_PERB_reg(a)
#define hri_tcc_set_CCB_DITH4_reg(a, b, c) hri_tcc_set_CCB_reg(a, b, c)
#define hri_tcc_get_CCB_DITH4_reg(a, b, c) hri_tcc_get_CCB_reg(a, b, c)
#define hri_tcc_write_CCB_DITH4_reg(a, b, c) hri_tcc_write_CCB_reg(a, b, c)
#define hri_tcc_clear_CCB_DITH4_reg(a, b, c) hri_tcc_clear_CCB_reg(a, b, c)
#define hri_tcc_toggle_CCB_DITH4_reg(a, b, c) hri_tcc_toggle_CCB_reg(a, b, c)
#define hri_tcc_read_CCB_DITH4_reg(a, b) hri_tcc_read_CCB_reg(a, b)
#define hri_tcc_set_CCB_DITH5_reg(a, b, c) hri_tcc_set_CCB_reg(a, b, c)
#define hri_tcc_get_CCB_DITH5_reg(a, b, c) hri_tcc_get_CCB_reg(a, b, c)
#define hri_tcc_write_CCB_DITH5_reg(a, b, c) hri_tcc_write_CCB_reg(a, b, c)
#define hri_tcc_clear_CCB_DITH5_reg(a, b, c) hri_tcc_clear_CCB_reg(a, b, c)
#define hri_tcc_toggle_CCB_DITH5_reg(a, b, c) hri_tcc_toggle_CCB_reg(a, b, c)
#define hri_tcc_read_CCB_DITH5_reg(a, b) hri_tcc_read_CCB_reg(a, b)
#define hri_tcc_set_CCB_DITH6_reg(a, b, c) hri_tcc_set_CCB_reg(a, b, c)
#define hri_tcc_get_CCB_DITH6_reg(a, b, c) hri_tcc_get_CCB_reg(a, b, c)
#define hri_tcc_write_CCB_DITH6_reg(a, b, c) hri_tcc_write_CCB_reg(a, b, c)
#define hri_tcc_clear_CCB_DITH6_reg(a, b, c) hri_tcc_clear_CCB_reg(a, b, c)
#define hri_tcc_toggle_CCB_DITH6_reg(a, b, c) hri_tcc_toggle_CCB_reg(a, b, c)
#define hri_tcc_read_CCB_DITH6_reg(a, b) hri_tcc_read_CCB_reg(a, b)

#ifdef __cplusplus
}
#endif

#endif /* _HRI_TCC_D21C_H_INCLUDED */
#endif /* _SAMD21_TCC_COMPONENT_ */
