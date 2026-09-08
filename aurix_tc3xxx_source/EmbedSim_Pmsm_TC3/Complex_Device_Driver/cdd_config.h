/**********************************************************************************************************************
 * \file        cdd_config.h
 * \brief       System-wide configuration constants and interrupt SRPN table
 *              for AURIX TC3xx bare-metal CDD layer.
 *
 * \copyright   Copyright (C) EmbedSim Project / Paul Abraham 2024
 *              SPDX-License-Identifier: MIT
 *********************************************************************************************************************/

#ifndef CDD_CONFIG_H_
#define CDD_CONFIG_H_

#include "embed_sim_sys_types.h"   /* uint8_T, uint32_T, real32_T, real64_T, boolean_T */
#include "embed_sim_compiler.h"    /* STATIC, INLINE, P2VAR, P2CONST, CONSTP2VAR       */

/**********************************************************************************************************************
 * Mathematical Constants
 *********************************************************************************************************************/
#define PI                  (3.141592653589793)
#define RAD_360             (6.283185307179586)
#define RAD_270             (4.712388980384690)
#define RAD_240             (4.188790204786391)
#define RAD_120             (2.094395102393195)
#define RAD_90              (1.570796326794897)
#define EPSILON_ZERO        (1.0e-10f)

/**********************************************************************************************************************
 * Clock Frequencies [Hz]
 *********************************************************************************************************************/
#define MHZ_300             (300000000.0F)
#define MHZ_200             (200000000.0F)
#define MHZ_160             (160000000.0F)
#define MHZ_100             (100000000.0F)
#define MHZ_50               (50000000.0F)
#define MHZ_20               (20000000.0F)
#define MHZ_5                 (5000000.0F)
#define MHZ_1                 (1000000.0F)

#define EVR_OSC_FREQUENCY       MHZ_100
#define XTAL_OSC_FREQUENCY      MHZ_20
#define SYSCLK_OSC_FREQUENCY    MHZ_20
#define GTM_CMU_CLK0_FREQUENCY  MHZ_200

/**********************************************************************************************************************
 * ISR Macro + SRPN Table
 *********************************************************************************************************************/
#define EMBED_SIM_INTERRUPT(Isr, VectabNum, Prio) \
    void __interrupt(Prio) __vector_table(VectabNum) Isr(void)


#define STM0_CMP0_IR_SRPN                       (50U)
#define CORE_00_QSPI4_TX_SRPN                   (55U)
#define CORE_00_QSPI4_RX_SRPN                   (56U)
#define CORE_00_QSPI4_ERR_SRPN                  (57U)
#define CORE_00_ATOM_00_CH_00_CL_SRPN           (80U)
#define CORE_00_ADC_PHASE_U_SRPN                (90U)
#define CORE_00_ADC_PHASE_V_SRPN                (91U)
#define CORE_00_ADC_PHASE_W_SRPN                (92U)
#define CORE_00_ADC_DC_LINK                     (95U)
#define CORE_00_GPT12_ENCODER_ZERO_SRPN         (100U)


/**********************************************************************************************************************
 * GTM Software Dead-Time  [CLK0 ticks]
 *
 * At 200 MHz CMU CLK0 (5 ns/tick):  28 ticks = 140 ns.
 *********************************************************************************************************************/
#ifndef CDD_GTM_SW_DEAD_TIME_TICKS
#define CDD_GTM_SW_DEAD_TIME_TICKS      (28U)   /**< Software dead-time  [CLK0 ticks] */
#endif


/**********************************************************************************************************************
 * GTM Gate-Driver HS Polarity Flag
 *********************************************************************************************************************/
#ifndef CDD_GTM_HS_ACTIVE_LOW
#define CDD_GTM_HS_ACTIVE_LOW   (1U)   /**< TLE9180D /IHx active LOW (default) */
#endif

/**********************************************************************************************************************
 * Control Loop Frequency
 *********************************************************************************************************************/
#ifndef CDD_CONTROL_LOOP_FREQUENCY
#define CDD_CONTROL_LOOP_FREQUENCY   (20000U)   /**< 20 kHz  [Hz] */
#endif

/**********************************************************************************************************************
 * EVADC SR Enable
 *********************************************************************************************************************/
#define EVADC_ENABLE_PHASE_U_SR     (1U)
#define EVADC_ENABLE_PHASE_V_SR     (1U)
#define EVADC_ENABLE_PHASE_W_SR     (1U)
#define EVADC_ENABLE_DC_LINK_SR     (0U)

#endif /* CDD_CONFIG_H_ */
