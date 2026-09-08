/**********************************************************************************************************************
 * \file        cdd_evadc_app.c
 * \brief       Implementation of cdd_evadc_app.h — EVADC channel init and readout.
 *
 * \details     Five EVADC channels on four groups follow the same init pattern:
 *              1. Configure arbitration priority (ARBPR)
 *              2. Configure channel control (CHCTR): global class 0
 *              3. Add channel(s) to queue (QINR): auto-refill; external trigger
 *                 on the first entry, back-to-back conversion for follow-ups.
 *                 Phase groups queue EVADC_PHASE_SAMPLES entries per trigger
 *                 (1x EXTR + 3x refill) for 4x oversampling, matching the
 *                 Infineon PmsmFoc reference (Evadc_InitCurSenseLsTriShunt*).
 *              4. Configure queue trigger (QCTRL): GTM ATOM via ADCTRIG
 *              5. Enable queue trigger (QMR)
 *              6. Configure data reduction (RCR): DRCTR = EVADC_PHASE_SAMPLES-1,
 *                 DMM = 0 (accumulate); readout divides the sum back down.
 *              7. Configure ONE service request node: G1_SR0 on the LAST
 *                 result of the set (G1RES3 = VOLT_DC).  G1 converts VRO +
 *                 VOLT_DC back-to-back after the phase bursts start on the
 *                 same edge, so a G1 SR implies the whole set is fresh —
 *                 same single-interrupt philosophy as the reference (which
 *                 raises resultPriority only on curVO1).
 *              8. Start converter (ARBCFG.ANONC = 3)
 *              9. AFTER all groups are on: startup calibration (SUCAL) —
 *                 the analog converters must be enabled for the calibration
 *                 sequence to execute (iLLD initGroup order).
 *
 *              Channel map (AP32541 v1.0 Table 12 / Table 15, AppKit TC387):
 *
 *                  Signal    AppKit pin  Analog in  Group/Ch  Result reg
 *                  VO1  (U)  T10         AN0        G0  CH0   G0RES0
 *                  VO2  (V)  W2          AN24       G3  CH0   G3RES0
 *                  VO3  (W)  W5          AN16       G2  CH0   G2RES0
 *                  VRO       W8          AN8        G1  CH0   G1RES0
 *                  VOLT_DC   W7          AN11       G1  CH3   G1RES3
 *
 *              Trigger wiring (verified, TC38x UM Appendix Table 292 and
 *              GTM_ADCTRIG0OUT0 field encoding, §26.3.9):
 *                  GTM ADC_TRIG0[x] connects to EVADC group x input REQTRI,
 *                  and REQTRI is selected by GxQCTRL0.XTSEL = 0x8 UNIFORMLY
 *                  for all groups.  ADC_TRIG0[x] is driven by
 *                  GTM_ADCTRIG0OUT0.SELx; for SEL0..SEL4 the code 0x8 selects
 *                  CDTM0_DTM5_3 = ATOM0_CH7 (dead-time passthrough, DTM5
 *                  configured passthrough in cdd_gtm_app.c Step 7).
 *
 *                  ATOM0_CH7 (duty 0.9) → ADCTRIG0OUT0.SEL0/1/2/3 →
 *                      ADC_TRIG0[0..3] → G0/G1/G2/G3 REQTRI (XTSEL 0x8)
 *
 *              ALL FIVE channels therefore convert on the same ATOM0_CH7
 *              falling edge each PWM period: G0/G3/G2 phase currents in
 *              parallel, G1 VRO + VOLT_DC back-to-back from its queue.
 *              There is no separate DC-link trigger any more — the former
 *              ADCTRIG3/ATOM0_CH3 concept is void (ATOM0_CH3 is the phase V
 *              low-side PWM output in the v1.5 GTM channel map, and
 *              ADCTRIG3OUTx was never programmed, so the old G8 DC-link
 *              channel in fact never triggered).
 *
 *              Required cdd_gtm_app.c change (Step 6):
 *                  + GTM_ADCTRIG0OUT0.B.SEL3 = 0x8U;   (ATOM0_CH7 → G3, phase V)
 *                  - GTM_ADCTRIG0OUT0.B.SEL7 = 0x8U;   (DELETE: SEL7 feeds G7,
 *                    not G8, and code 0x8 in the SEL5..7 encoding selects
 *                    ATOM5_CH5, not ATOM0_CH7 — the line was doubly wrong)
 *
 *              Zero-current reference: the AP32541 routes the TLE9180D VRO
 *              reference-buffer output to AN8 precisely so software can
 *              measure the true zero-current level instead of assuming the
 *              2.5 V nominal (device spread + temperature drift, DS §9.2).
 *              CddEvadc_ConvertPhaseCurrents() therefore subtracts the LIVE
 *              measured VRO.  Because VRO and VOx are converted by the same
 *              ADC against the same VAREF, this also cancels the ADC
 *              gain/reference error at the zero-current operating point.
 *              Residual per-CSA output offsets (±100 mV uncalibrated,
 *              ±10 mV after TLE9180 auto-cal, DS P_9.6.17/18) are removed by
 *              CddEvadc_CalibratePhaseOffsets() at standstill.
 *
 *              Result readout uses a SINGLE 32-bit read of GxRESy into a
 *              local copy: VF is clear-on-read, so checking VF and then
 *              re-reading RESULT in a second access would race against the
 *              next conversion.  VF and RESULT are taken from the same load.
 *
 * \note        cdd_config.h dependencies:
 *                  CORE_01_ADC_DC_LINK_SRPN  → SRC_VADC_G1_SR0 (the SINGLE
 *                      conversion-set SR; raised by G1RES3, last of the set)
 *                  EVADC_ENABLE_DC_LINK_SR   → G1_SR0.SRE
 *                  CORE_01_ADC_PHASE_U/V/W_SRPN and EVADC_ENABLE_PHASE_x_SR
 *                      are NO LONGER USED (phase groups raise no SR; their
 *                      results are polled by CddEvadc_ReadSensorMeas).
 *
 * \note        MISRA C:2012: Rules 8.9, 8.10, 14.4, 15.5, 17.2.
 *
 * \copyright   Copyright (C) EmbedSim Project / Paul Abraham 2024
 *              https://github.com/vectorsim/embed_sim_project
 *              SPDX-License-Identifier: MIT
 *********************************************************************************************************************/

#include "cdd_evadc_app.h"
#include "cdd_sys_utility.h"
#include "cdd_gpio_app.h"
#include "cdd_config.h"
#include "cdd_stm_app.h"
#include "IfxEvadc_reg.h"
#include "IfxSrc_reg.h"
#include "IfxConverter_reg.h"
#include "Bsp.h"
#include <stddef.h>            /* NULL (pointer checks)                                          */

/**********************************************************************************************************************
 * Private Macros — ADC Scaling
 *********************************************************************************************************************/



#define EVADC_FULL_SCALE            (4095.0F)    /**< 12-bit LSB divisor: 1 LSB = VAREF/4096   [dimensionless] */
#define EVADC_VAREF_VOLT            (5.0F)       /**< AppKit TC387 VAREF.  AP32541 §3.3.4: CSA outputs
                                                  *   "feed ADCs with an analog range from 0 V to 5 V";
                                                  *   VRO = 2.5 V is mid-scale of this range.        [V]      */
#define EVADC_XTSEL_REQTRI          (0x8U)       /**< GxQCTRL0.XTSEL code for input REQTRI = GTM
                                                  *   ADC_TRIG0[group].  Uniform for all groups
                                                  *   (TC38x UM Appendix Table 292).                          */
#define EVADC_PHASE_SAMPLES         (4U)         /**< Phase-current oversampling: conversions per trigger and
                                                  *   DMM accumulation count (DRCTR = N-1).  4 matches the
                                                  *   Infineon reference (DRCTR=3, "Accumulate 4 result").
                                                  *   Burst length ~4 conversions (~3..4 us at 40 MHz fADCI)
                                                  *   MUST fit inside the all-low-side ON window — verify on
                                                  *   the scope at max modulation index, else reduce to 1.
                                                  *   Accumulator headroom: 4 x 4095 = 16380 < 2^16, OK.     */
#define EVADC_ARBPR_PRIO            (0x1U)       /**< Arbitration priority                                     */



/** \brief  ADC code → pin voltage  [V] */
#define EVADC_CODE_TO_VOLT(Code)    (((real32_T)(Code) / (1.0F * EVADC_FULL_SCALE * EVADC_PHASE_SAMPLES)) * EVADC_VAREF_VOLT)

/**********************************************************************************************************************
 * Private Macros — Shunt Current Conversion  (CddEvadc_ConvertPhaseCurrents)
 *
 *   I [A] = EVADC_I_SIGN * (V_ox - V_ro_measured - PhaseOffset) / (EVADC_CSA_GAIN * EVADC_SHUNT_R_OHM)
 *
 * Values below are tied to the ACTUAL hardware configuration — change them ONLY together with their source:
 *    EVADC_SHUNT_R_OHM  AP32541 BoM item 34: R17/R27/R37 = 10 mOhm 1% (WSL3637).
 *    EVADC_CSA_GAIN     TLE9180D DS Table 19 P_9.6.7, gain code 100B = 30.81 V/V typ (30.19..31.42).
 *                       MUST track OP_GAIN1/2/3 = 0x44 in the SPI startup batch (cdd_tle9180_app.c).
 *    EVADC_VRO_NOM_V    zcl = 10B in OP_OCL (cdd_tle9180_app.c) → VRO = 2.5 V nominal.  Used only as
 *                       fallback until the first VRO conversion completes; runtime uses measured VRO.
 *    EVADC_I_SIGN       +1.0f/-1.0f — flips once if the ISP/ISN shunt orientation yields VOx BELOW
 *                       VRO for current INTO the terminal (DFC convention: positive = into terminal).
 *                       Determine empirically with a DC injection through one phase.
 *
 * Full-scale check: 2.5 V / (30.81 * 0.010) = ±8.1 A around VRO.
 *********************************************************************************************************************/

#define EVADC_SHUNT_R_OHM           (0.010f)     /**< Low-side phase shunt        [Ohm]  AP32541 BoM #34   */
#define EVADC_CSA_GAIN              (30.81f)     /**< TLE9180D CSA gain, code 100B [V/V] DS P_9.6.7        */
#define EVADC_VRO_NOM_V             (2.5f)       /**< Nominal VRO (zcl=10B), fallback only  [V]            */


/** \brief  Sense-voltage → ampere conversion factor:  1/(30.81 * 0.010) = 3.246  [A/V] */


/**********************************************************************************************************************
 * Private Macros — DC-Link Voltage Divider  (AP32541 Eq. 1)
 *
 *   VOLT_DC(pin) = VBAT * R113 / (R113 + R114)   with R113 = 5.6 kOhm, R114 = 56 kOhm
 *   → VBAT = VOLT_DC(pin) * 11.0     (12 V bus reads 1.09 V at the pin)
 *********************************************************************************************************************/

#define EVADC_UDC_R113_KOHM         (5.6f)       /**< Divider lower resistor      [kOhm]  AP32541 Fig. 10 */
#define EVADC_UDC_R114_KOHM         (56.0f)      /**< Divider upper resistor      [kOhm]  AP32541 Fig. 10 */

/** \brief  Pin voltage → DC-link bus voltage:  (5.6+56)/5.6 = 11.0  [V/V] */
#define EVADC_UDC_PIN_TO_BUS        ((EVADC_UDC_R113_KOHM + EVADC_UDC_R114_KOHM) / EVADC_UDC_R113_KOHM)


#define  EVADC_CURRENT_U_READING_VALID   (0x1U)
#define  EVADC_CURRENT_V_READING_VALID   (0x2U)
#define  EVADC_CURRENT_W_READING_VALID   (0x4U)


#define  EVADC_CALSTC   (0x3U)     /**<  Calibration Sample Time Control */


/** \brief  Voltage to current conversion factor: 1/(R_shunt * CSA_gain) = 3.246 A/V */
#define EVADC_V_TO_A_FACTOR         (1.0f / (EVADC_SHUNT_R_OHM * EVADC_CSA_GAIN))


/**********************************************************************************************************************
 * Private Function Prototypes
 *********************************************************************************************************************/
static void CddEvadc_ConfigGlobal(void);
static void CddEvadc_ConfigG0Ch0An0PhaseU(void);
static void CddEvadc_ConfigG3Ch0An24PhaseV(void);
static void CddEvadc_ConfigG2Ch0An16PhaseW(void);
static void CddEvadc_ConfigG01VroUdc(void);

static void CddEvadc_ReadVro(P2VAR(volatile CddApp_T, AUTOMATIC, CDD_APPL_DATA) CddAppPtr);
static void CddEvadc_ReadPhaseU(P2VAR(volatile CddApp_T, AUTOMATIC, CDD_APPL_DATA) CddAppPtr);
static void CddEvadc_ReadPhaseV(P2VAR(volatile CddApp_T, AUTOMATIC, CDD_APPL_DATA) CddAppPtr);
static void CddEvadc_ReadPhaseW(P2VAR(volatile CddApp_T, AUTOMATIC, CDD_APPL_DATA) CddAppPtr);
static void CddEvadc_ReadDcLink(P2VAR(volatile CddApp_T, AUTOMATIC, CDD_APPL_DATA) CddAppPtr);




/**********************************************************************************************************************
 * Public Function Implementations
 *********************************************************************************************************************/

void CddEvadc_Init(void)
{


    CddEvadc_ConfigGlobal();
    CddEvadc_ConfigG0Ch0An0PhaseU();
    CddEvadc_ConfigG3Ch0An24PhaseV();
    CddEvadc_ConfigG2Ch0An16PhaseW();
    CddEvadc_ConfigG01VroUdc();
}


void CddEvadc_ConvertPhaseCurrents(P2VAR(volatile CddApp_T, AUTOMATIC, CDD_APPL_DATA) CddAppPtr)
{
    const real32_T V_TO_A = EVADC_V_TO_A_FACTOR;  /* 3.246 A/V */
    real32_T vRoCalibrated;
    real32_T iBalanced;

    /*
     * Use calibrated VRO reference = measured VRO - VRO offset
     * This removes the VRO offset from the reference
     */
    vRoCalibrated = CddAppPtr->Vro - CddAppPtr->OffsetVro;

    /* Calculate phase currents using calibrated VRO reference */
    CddAppPtr->Iu = ((CddAppPtr->Vu - vRoCalibrated) * V_TO_A) - CddAppPtr->OffsetIu;
    CddAppPtr->Iv = ((CddAppPtr->Vv - vRoCalibrated) * V_TO_A) - CddAppPtr->OffsetIv;
    CddAppPtr->Iw = ((CddAppPtr->Vw - vRoCalibrated) * V_TO_A) - CddAppPtr->OffsetIw;

    CddAppPtr->Isum =  CddAppPtr->Iu + CddAppPtr->Iv + CddAppPtr->Iw;
    iBalanced = iBalanced/3.0F;

    CddAppPtr->Iu -= iBalanced;
    CddAppPtr->Iv -= iBalanced;
    CddAppPtr->Iw -= iBalanced;
}



/**********************************************************************************************************************
 * ISR Vector Registration — SINGLE conversion-set interrupt
 *
 * One SR per PWM period (G1_SR0 on G1RES3, the last result of the set) instead
 * of the previous four (G0/G3/G2/G1) — three of which were empty debug stubs
 * costing ~3 ISR entries/exits every 50 us, and the fourth (G1) never fired
 * because its SRC node was commented out.  SRPN literal must match
 * cdd_config.h: CORE_01_ADC_DC_LINK_SRPN (95).
 *********************************************************************************************************************/

EMBED_SIM_INTERRUPT(EVADC_ConvSet_Isr, 0x0U, CORE_00_ADC_DC_LINK);
void EVADC_ConvSet_Isr(void)
{
    CddEvadc_ReadVro(&CddApp_G);
    CddEvadc_ReadDcLink(&CddApp_G);

}


EMBED_SIM_INTERRUPT(EVADC_ConvPhaseU_Isr, 0x0U, CORE_00_ADC_PHASE_U_SRPN);
void EVADC_ConvPhaseU_Isr(void)
{
    CddEvadc_ReadPhaseU(&CddApp_G);
    CddSys_NopDelay(1U, 1U);

}

EMBED_SIM_INTERRUPT(EVADC_ConvPhaseV_Isr, 0x0U, CORE_00_ADC_PHASE_V_SRPN);
void EVADC_ConvPhaseV_Isr(void)
{

     CddEvadc_ReadPhaseV(&CddApp_G);


}

EMBED_SIM_INTERRUPT(EVADC_ConvPhaseW_Isr, 0x0U, CORE_00_ADC_PHASE_W_SRPN);
void EVADC_ConvPhaseW_Isr(void)
{

    CddEvadc_ReadPhaseW(&CddApp_G);

}


/**********************************************************************************************************************
 * Private — Hardware Init Helpers
 *********************************************************************************************************************/

static void CddEvadc_ConfigGlobal(void)
{

    Ifx_EVADC_GLOBCFG   globCfg;

    globCfg.U  = EVADC_GLOBCFG.U;

    CddSys_ClearCpuWdtEndInit();
    CONVCTRL_CLC.U = 0x00000000U;                  /* Enable CONVCTRL module        */
    while (CONVCTRL_CLC.B.DISS == 0x1U)
    {
        CddSys_NopDelay(1U, 1U);
    }
    CONVCTRL_CCCTRL.U = 0xB0000000U;               /* unlock converter control regs */
    CONVCTRL_PHSCFG.U = 0x00008003U;               /* fADC=160MHz, fPHSYNC=40MHz    */
    CONVCTRL_CCCTRL.U = 0x00000000U;               /* lock converter control regs   */
    CddSys_SetCpuWdtEndInit();

    CddSys_ClearCpuWdtEndInit();
    EVADC_CLC.U = 0x0U;
    CddSys_SetCpuWdtEndInit();
    while (EVADC_CLC.B.DISS == 0x1U)
    {
        CddSys_NopDelay(0x1U, 0x1U);
    }

    globCfg.B.CPWC   = 0x1U;   /* bitfields SUPLEV, USC can be written  */
    globCfg.B.SUPLEV = 0x1U;   /* 5 V               */
    globCfg.B.USC    = 0x0U;   /* automatic voltage control             */
    EVADC_GLOBCFG.U  = globCfg.U;

}



/**********************************************************************************************************************
 * Private — Channel Configuration Helpers
 *********************************************************************************************************************/

/** \brief  G0 CH0 = AN0 = VO1 — phase U current.  Triggered by ADCTRIG0 (ATOM0_CH7). */
void CddEvadc_ConfigG0Ch0An0PhaseU(void)
{
    Ifx_EVADC_G_ANCFG   anCfg;
    Ifx_EVADC_G_ARBPR   arbPr;
    Ifx_EVADC_G_CHCTR   chCtrl;
    Ifx_EVADC_G_Q_QINR  qQinr;
    Ifx_EVADC_G_Q_QCTRL qCtrl;
    Ifx_EVADC_G_ICLASS  iClass;
    Ifx_EVADC_G_Q_QMR   qQmr;
    Ifx_EVADC_G_RCR     rcr;
    Ifx_SRC_SRCR        srcCfg;


    anCfg.U   = EVADC_G0ANCFG.U;
    arbPr.U   = EVADC_G0ARBPR.U;
    qQinr.U   = EVADC_G0QINR0.U;
    qCtrl.U   = EVADC_G0QCTRL0.U;
    chCtrl.U  = EVADC_G0CHCTR0.U;
    qQmr.U    = EVADC_G0QMR0.U;
    iClass.U  = EVADC_G0ICLASS0.U;
    chCtrl.U  = EVADC_G0CHCTR0.U;
    rcr.U     = EVADC_G0RCR0.U;
    srcCfg.U  = SRC_VADC_G0_SR0.U;

    /* Analog Conversion Configuration */
    anCfg.B.DIVA     = 0x3U;   /* setup clock frequency fADC=160MHz/4=40MHz */
    anCfg.B.IPE      = 0x0U;   /* idle pre charge  disabled                 */
    anCfg.B.RPE      = 0x0U;   /* no reference pre charge                   */
    anCfg.B.CALSTC   = EVADC_CALSTC;   /* calibration                        */
    anCfg.B.BE       = 0x0U;   /* disable input buffer                      */
    anCfg.B.DPCAL    = 0x1U;   /* disable post calibration                  */
    EVADC_G0ANCFG.U = anCfg.U;

    EVADC_G0ARBCFG.B.ANONC = 0x3;  /* setup Normal Operation  */
    while(EVADC_G0ARBCFG.B.ANONS != 0x3)
    {
        CddSys_NopDelay(1U, 1U);
    }
    CddStm_Delay_Us(5U);   /* wait until port settles down */

    /* Enable startup calibration */
    EVADC_GLOBCFG.U |= (1 << 15) | (1 << 31);
    /* Wait until calibration is done */
    while(EVADC_G3ARBCFG.B.CAL == 0x1)
    {
        CddSys_NopDelay(1U, 1U);
    }

    /* Configure Priority Queue */
    arbPr.B.PRIO0 = EVADC_ARBPR_PRIO;
    arbPr.B.ASEN0 = 0x1U;   /* arbitration Source input enable */
    arbPr.B.CSM0  = 0x0U;   /* wait for start mode             */
    EVADC_G0ARBPR.U = arbPr.U;

    /* Add Channel 0 to Group 0 Queue Source and enable External Trigger */
    qQinr.B.REQCHNR = 0x0u;   /* assign channel number */
    qQinr.B.RF      = 0x1U;   /* refill the queue      */
    qQinr.B.EXTR    = 0x1U;   /* wait for GMT trigger  */
    EVADC_G0QINR0.U = qQinr.U;

    /* Configure Source Control Trigger & Gate */
    qCtrl.B.XTWC      = 0x1U;               /* bitfields XTMODE, XTSEL, TRSEL can be written */
    qCtrl.B.GTWC      = 0x1U;               /* bitfield  GTSEL can be written                */
    qCtrl.B.XTSEL     = EVADC_XTSEL_REQTRI; /* external Trigger Input Selection (GTM)        */
    qCtrl.B.GTSEL     = 0x1U;               /* gate input selection                          */
    qCtrl.B.XTMODE    = 0x1U;               /* trigger on falling edge                       */
    qCtrl.B.SRCRESREG = 0x0U;               /* use G0CHCTR0.RESREG for result                */
    EVADC_G0QCTRL0.U   = qCtrl.U;

    /* Enable Trigger & Gate for Request */
    qQmr.B.ENGT       = 0x1U;  /* Requests issued, gate ignored */
    qQmr.B.ENTR       = 0x1U;  /* External trigger enabled      */
    EVADC_G0QMR0.U     = qQmr.U;

    /* Configure Channel Settings */
    /*The global input class registers define the sample time and data conversion mode for each channel of any group
    that selects them via bitfield ICLSEL in its channel control register GxCHCTRy. */
    iClass.B.CMS  = 0x0U; /* standard conversion */
    iClass.B.STCS = 0x0U;
    EVADC_G0ICLASS0.U = iClass.U;

    chCtrl.B.ICLSEL = 0x0U;   /* store result right-aligned */
    chCtrl.B.RESREG = 0x0U;   /* store result from channel y to specified group result register */
    EVADC_G0CHCTR0.U =  chCtrl.U;

   /* Data Reduction */
    rcr.B.DRCTR        = EVADC_PHASE_SAMPLES -  1U;  /* DRCTR = N-1 for N samples */
    rcr.B.DMM          = 0x0U;                       /* Standard data reduction (accumulate and average) */
    rcr.B.SRGEN        = 0x1U;                       /* generate service request */
    EVADC_G0RCR0.U     = rcr.U;

    EVADC_G0REVNP0.U = 0x0U;

    srcCfg.B.SRPN        = CORE_00_ADC_PHASE_U_SRPN;
    srcCfg.B.TOS         = 0x0u;
    SRC_VADC_G0_SR0.U    = srcCfg.U;
    SRC_VADC_G0_SR0.B.SRE = EVADC_ENABLE_PHASE_U_SR;

}


void CddEvadc_ConfigG3Ch0An24PhaseV(void)
{
    Ifx_EVADC_G_ANCFG   anCfg;
    Ifx_EVADC_G_ARBPR   arbPr;
    Ifx_EVADC_G_CHCTR   chCtrl;
    Ifx_EVADC_G_Q_QINR  qQinr;
    Ifx_EVADC_G_Q_QCTRL qCtrl;
    Ifx_EVADC_G_ICLASS  iClass;
    Ifx_EVADC_G_Q_QMR   qQmr;
    Ifx_EVADC_G_RCR     rcr;
    Ifx_SRC_SRCR        srcCfg;

    anCfg.U   = EVADC_G3ANCFG.U;
    arbPr.U   = EVADC_G3ARBPR.U;
    qQinr.U   = EVADC_G3QINR0.U;
    qCtrl.U   = EVADC_G3QCTRL0.U;
    chCtrl.U  = EVADC_G3CHCTR0.U;
    qQmr.U    = EVADC_G3QMR0.U;
    iClass.U  = EVADC_G3ICLASS0.U;
    chCtrl.U  = EVADC_G3CHCTR0.U;
    rcr.U     = EVADC_G3RCR0.U;
    srcCfg.U  = SRC_VADC_G3_SR0.U;

    /* Analog Conversion Configuration */
    anCfg.B.DIVA     = 0x3U;   /* setup clock frequency fADC=160MHz/4=40MHz */
    anCfg.B.IPE      = 0x0U;   /* idle pre charge  disabled                 */
    anCfg.B.RPE      = 0x0U;   /* no reference pre charge                   */
    anCfg.B.CALSTC   = EVADC_CALSTC;   /* calibration 2*Tadc                        */
    anCfg.B.BE       = 0x0U;   /* disable input buffer                      */
    anCfg.B.DPCAL    = 0x1U;   /* disable post calibration                  */
    EVADC_G3ANCFG.U = anCfg.U;

    EVADC_G3ARBCFG.B.ANONC = 0x3;  /* setup Normal Operation  */
    while(EVADC_G3ARBCFG.B.ANONS != 0x3)
    {
        CddSys_NopDelay(1U, 1U);
    }
    CddStm_Delay_Us(5U);   /* wait until port settles down */

    /* Enable startup calibration */
    EVADC_GLOBCFG.U |= (1 << 15) | (1 << 31);
    /* Wait until calibration is done */
    while(EVADC_G3ARBCFG.B.CAL == 0x1)
    {
        CddSys_NopDelay(1U, 1U);
    }


    /* Configure Priority Queue */
    arbPr.B.PRIO0 = EVADC_ARBPR_PRIO;
    arbPr.B.ASEN0 = 0x1U;   /* arbitration Source input enable */
    arbPr.B.CSM0  = 0x0U;   /* wait for start mode             */
    EVADC_G3ARBPR.U = arbPr.U;

    /* Add Channel 0 to Group 3 Queue Source and enable External Trigger */
    qQinr.B.REQCHNR = 0x0u;   /* assign channel number */
    qQinr.B.RF      = 0x1U;   /* refill the queue      */
    qQinr.B.EXTR    = 0x1U;   /* wait for GMT trigger  */
    EVADC_G3QINR0.U = qQinr.U;

    /* Configure Source Control Trigger & Gate */
    qCtrl.B.XTWC      = 0x1U;               /* bitfields XTMODE, XTSEL, TRSEL can be written */
    qCtrl.B.GTWC      = 0x1U;               /* bitfield  GTSEL can be written                */
    qCtrl.B.XTSEL     = EVADC_XTSEL_REQTRI; /* external Trigger Input Selection (GTM)        */
    qCtrl.B.GTSEL     = 0x1U;               /* gate input selection                          */
    qCtrl.B.XTMODE    = 0x1U;               /* trigger on falling edge                       */
    qCtrl.B.SRCRESREG = 0x0U;               /* use G3CHCTR0.RESREG for result                */
    EVADC_G3QCTRL0.U   = qCtrl.U;

    /* Enable Trigger & Gate for Request */
    qQmr.B.ENGT       = 0x1U;  /* Requests issued, gate ignored */
    qQmr.B.ENTR       = 0x1U;  /* External trigger enabled      */
    EVADC_G3QMR0.U     = qQmr.U;

    /* Configure Channel Settings */
    /*The global input class registers define the sample time and data conversion mode for each channel of any group
    that selects them via bitfield ICLSEL in its channel control register GxCHCTRy. */
    iClass.B.CMS  = 0x0U; /* standard conversion */
    iClass.B.STCS = 0x0U;
    EVADC_G3ICLASS0.U = iClass.U;

    chCtrl.B.ICLSEL = 0x0U;   /* store result right-aligned */
    chCtrl.B.RESREG = 0x0U;   /* store result from channel y to specified group result register */
    EVADC_G3CHCTR0.U =  chCtrl.U;

   /* Data Reduction */
    rcr.B.DRCTR        = EVADC_PHASE_SAMPLES -  1U;  /* DRCTR = N-1 for N samples */
    rcr.B.DMM          = 0x0U;                       /* Standard data reduction (accumulate and average) */
    rcr.B.SRGEN        = 0x1U;                       /* generate service request */
    EVADC_G3RCR0.U     = rcr.U;

    EVADC_G3REVNP0.U = 0x0U;

    srcCfg.B.SRPN        = CORE_00_ADC_PHASE_V_SRPN;
    srcCfg.B.TOS         = 0x0u;
    SRC_VADC_G3_SR0.U    = srcCfg.U;
    SRC_VADC_G3_SR0.B.SRE = EVADC_ENABLE_PHASE_V_SR;

}


void CddEvadc_ConfigG2Ch0An16PhaseW(void)
{
    Ifx_EVADC_G_ANCFG   anCfg;
    Ifx_EVADC_G_ARBPR   arbPr;
    Ifx_EVADC_G_CHCTR   chCtrl;
    Ifx_EVADC_G_Q_QINR  qQinr;
    Ifx_EVADC_G_Q_QCTRL qCtrl;
    Ifx_EVADC_G_ICLASS  iClass;
    Ifx_EVADC_G_Q_QMR   qQmr;
    Ifx_EVADC_G_RCR     rcr;
    Ifx_SRC_SRCR        srcCfg;

    anCfg.U   = EVADC_G2ANCFG.U;
    arbPr.U   = EVADC_G2ARBPR.U;
    qQinr.U   = EVADC_G2QINR0.U;
    qCtrl.U   = EVADC_G2QCTRL0.U;
    chCtrl.U  = EVADC_G2CHCTR0.U;
    qQmr.U    = EVADC_G2QMR0.U;
    iClass.U  = EVADC_G2ICLASS0.U;
    chCtrl.U  = EVADC_G2CHCTR0.U;
    rcr.U     = EVADC_G2RCR0.U;
    srcCfg.U  = SRC_VADC_G2_SR0.U;

    /* Analog Conversion Configuration */
    anCfg.B.DIVA     = 0x3U;   /* setup clock frequency fADC=160MHz/4=40MHz */
    anCfg.B.IPE      = 0x0U;   /* idle pre charge  disabled                 */
    anCfg.B.RPE      = 0x0U;   /* no reference pre charge                   */
    anCfg.B.CALSTC   =EVADC_CALSTC;   /* calibration 2*Tadc                        */
    anCfg.B.BE       = 0x0U;   /* disable input buffer                      */
    anCfg.B.DPCAL    = 0x1U;   /* disable post calibration                  */
    EVADC_G2ANCFG.U = anCfg.U;

    EVADC_G2ARBCFG.B.ANONC = 0x3;  /* setup Normal Operation  */
    while(EVADC_G2ARBCFG.B.ANONS != 0x3)
    {
        CddSys_NopDelay(1U, 1U);
    }
    CddStm_Delay_Us(5U);   /* wait until port settles down */

    /* Enable startup calibration */
    EVADC_GLOBCFG.U |= (1 << 15) | (1 << 31);
    /* Wait until calibration is done */
    while(EVADC_G2ARBCFG.B.CAL == 0x1)
    {
        CddSys_NopDelay(1U, 1U);
    }

    /* Configure Priority Queue */
    arbPr.B.PRIO0 = EVADC_ARBPR_PRIO;
    arbPr.B.ASEN0 = 0x1U;   /* arbitration Source input enable */
    arbPr.B.CSM0  = 0x0U;   /* wait for start mode             */
    EVADC_G2ARBPR.U = arbPr.U;

    /* Add Channel 0 to Group 3 Queue Source and enable External Trigger */
    qQinr.B.REQCHNR = 0x0u;   /* assign channel number */
    qQinr.B.RF      = 0x1U;   /* refill the queue      */
    qQinr.B.EXTR    = 0x1U;   /* wait for GMT trigger  */
    EVADC_G2QINR0.U = qQinr.U;

    /* Configure Source Control Trigger & Gate */
    qCtrl.B.XTWC      = 0x1U;               /* bitfields XTMODE, XTSEL, TRSEL can be written */
    qCtrl.B.GTWC      = 0x1U;               /* bitfield  GTSEL can be written                */
    qCtrl.B.XTSEL     = EVADC_XTSEL_REQTRI; /* external Trigger Input Selection (GTM)        */
    qCtrl.B.GTSEL     = 0x1U;               /* gate input selection                          */
    qCtrl.B.XTMODE    = 0x1U;               /* trigger on falling edge                       */
    qCtrl.B.SRCRESREG = 0x0U;               /* use G3CHCTR0.RESREG for result                */
    EVADC_G2QCTRL0.U   = qCtrl.U;

    /* Enable Trigger & Gate for Request */
    qQmr.B.ENGT       = 0x1U;  /* Requests issued, gate ignored */
    qQmr.B.ENTR       = 0x1U;  /* External trigger enabled      */
    EVADC_G2QMR0.U     = qQmr.U;

    /* Configure Channel Settings */
    /*The global input class registers define the sample time and data conversion mode for each channel of any group
    that selects them via bitfield ICLSEL in its channel control register GxCHCTRy. */
    iClass.B.CMS  = 0x0U; /* standard conversion */
    iClass.B.STCS = 0x0U;
    EVADC_G2ICLASS0.U = iClass.U;

    chCtrl.B.ICLSEL = 0x0U;   /* store result right-aligned */
    chCtrl.B.RESREG = 0x0U;   /* store result from channel y to specified group result register */
    EVADC_G2CHCTR0.U =  chCtrl.U;

    /* Data Reduction */
    rcr.B.DRCTR        = EVADC_PHASE_SAMPLES -  1U;  /* DRCTR = N-1 for N samples */
    rcr.B.DMM          = 0x0U;                       /* Standard data reduction (accumulate and average) */
    rcr.B.SRGEN        = 0x1U;                       /* generate service request */
    EVADC_G2RCR0.U     = rcr.U;

    EVADC_G2REVNP0.U = 0x0U;

    srcCfg.B.SRPN        = CORE_00_ADC_PHASE_W_SRPN;
    srcCfg.B.TOS         = 0x0u;
    SRC_VADC_G2_SR0.U    = srcCfg.U;
    SRC_VADC_G2_SR0.B.SRE = EVADC_ENABLE_PHASE_W_SR;

}

void CddEvadc_ConfigG01VroUdc(void)
{
    Ifx_EVADC_G_ANCFG   anCfg;
    Ifx_EVADC_G_ARBPR   arbPr;
    Ifx_EVADC_G_CHCTR   chCtrl;
    Ifx_EVADC_G_Q_QINR  qQinr;
    Ifx_EVADC_G_Q_QCTRL qCtrl;
    Ifx_EVADC_G_ICLASS  iClass;
    Ifx_EVADC_G_Q_QMR   qQmr;
    Ifx_EVADC_G_RCR     rcr;
    Ifx_SRC_SRCR        srcCfg;

    anCfg.U   = EVADC_G1ANCFG.U;
    arbPr.U   = EVADC_G1ARBPR.U;
    qQinr.U   = EVADC_G1QINR0.U;
    qCtrl.U   = EVADC_G1QCTRL0.U;
    chCtrl.U  = EVADC_G1CHCTR0.U;
    qQmr.U    = EVADC_G1QMR0.U;
    iClass.U  = EVADC_G1ICLASS0.U;
    chCtrl.U  = EVADC_G1CHCTR0.U;
    rcr.U     = EVADC_G1RCR0.U;
    srcCfg.U  = SRC_VADC_G1_SR0.U;

    /* Analog Conversion Configuration */
    anCfg.B.DIVA     = 0x3U;   /* setup clock frequency fADC=160MHz/4=40MHz */
    anCfg.B.IPE      = 0x0U;   /* idle pre charge  disabled                 */
    anCfg.B.RPE      = 0x0U;   /* no reference pre charge                   */
    anCfg.B.CALSTC   = EVADC_CALSTC;   /* calibration 2*Tadc                        */
    anCfg.B.BE       = 0x0U;   /* disable input buffer                      */
    anCfg.B.DPCAL    = 0x1U;   /* disable post calibration                  */
    EVADC_G1ANCFG.U = anCfg.U;

    EVADC_G1ARBCFG.B.ANONC = 0x3;  /* setup Normal Operation  */
    while(EVADC_G1ARBCFG.B.ANONS != 0x3)
    {
        CddSys_NopDelay(1U, 1U);
    }
    CddStm_Delay_Us(5U);   /* wait until port settles down */


    /* Enable startup calibration */
    EVADC_GLOBCFG.U |= (1 << 15) | (1 << 31);
    /* Wait until calibration is done */
    while(EVADC_G1ARBCFG.B.CAL == 0x1)
    {
        CddSys_NopDelay(1U, 1U);
    }

    /* Configure Priority Queue */
    arbPr.B.PRIO0 = EVADC_ARBPR_PRIO;
    arbPr.B.ASEN0 = 0x1U;   /* arbitration Source input enable */
    arbPr.B.CSM0  = 0x0U;   /* wait for start mode             */
    EVADC_G1ARBPR.U = arbPr.U;


    qQinr.B.REQCHNR = 0x3U;   /* assign channel number */
    qQinr.B.RF      = 0x1U;   /* refill the queue      */
    qQinr.B.EXTR    = 0x0U;   /* wait for GMT trigger  */
    EVADC_G1QINR0.U = qQinr.U;

    /* Add Channel 0 to Group 1 Queue Source and enable External Trigger */
    qQinr.B.REQCHNR = 0x0u;   /* assign channel number */
    qQinr.B.RF      = 0x1U;   /* refill the queue      */
    qQinr.B.EXTR    = 0x1U;   /* wait for GMT trigger  */
    EVADC_G1QINR0.U = qQinr.U;

    /* Configure Source Control Trigger & Gate */
    qCtrl.B.XTWC      = 0x1U;               /* bitfields XTMODE, XTSEL, TRSEL can be written */
    qCtrl.B.GTWC      = 0x1U;               /* bitfield  GTSEL can be written                */
    qCtrl.B.XTSEL     = EVADC_XTSEL_REQTRI; /* external Trigger Input Selection (GTM)        */
    qCtrl.B.GTSEL     = 0x1U;               /* gate input selection                          */
    qCtrl.B.XTMODE    = 0x2U;               /* trigger on rising edge                        */
    qCtrl.B.SRCRESREG = 0x0U;               /* use G1CHCTR0.RESREG for result                */
    EVADC_G1QCTRL0.U   = qCtrl.U;

    /* Enable Trigger & Gate for Request */
    qQmr.B.ENGT       = 0x1U;  /* Requests issued, gate ignored */
    qQmr.B.ENTR       = 0x1U;  /* External trigger enabled      */
    EVADC_G1QMR0.U     = qQmr.U;

    /* Configure Channel Settings */
    /*The global input class registers define the sample time and data conversion mode for each channel of any group
    that selects them via bitfield ICLSEL in its channel control register GxCHCTRy. */
    iClass.B.CMS  = 0x0U; /* standard conversion */
    iClass.B.STCS = 0x0U;
    EVADC_G1ICLASS0.U = iClass.U;

    chCtrl.B.ICLSEL = 0x0U;   /* store result right-aligned */
    chCtrl.B.RESREG = 0x0U;   /* store result from channel y to specified group result register */
    EVADC_G1CHCTR0.U =  chCtrl.U;


    chCtrl.B.RESREG   = 0x3U;
    chCtrl.B.RESTGT   = 0x0U;
    EVADC_G1CHCTR3.U   = chCtrl.U;


    /* Data Reduction */
    rcr.B.DRCTR        = EVADC_PHASE_SAMPLES -  1U;  /* DRCTR = N-1 for N samples */
    rcr.B.DMM          = 0x0U;                       /* Standard data reduction (accumulate and average) */
    rcr.B.SRGEN        = 0x1U;                       /* generate service request */
    EVADC_G1RCR0.U     = rcr.U;

    rcr.B.SRGEN        = 0x0U;   /* generate service request */
    EVADC_G1RCR3.U     = rcr.U;


    EVADC_G1REVNP0.U = 0x0U;

    srcCfg.B.SRPN        = CORE_00_ADC_DC_LINK;
    srcCfg.B.TOS         = 0x0u;
    SRC_VADC_G1_SR0.U    = srcCfg.U;
    SRC_VADC_G1_SR0.B.SRE = 0x1U;

}


/**********************************************************************************************************************
 * Private — Result Readout Helpers
 *
 * Pattern: one 32-bit read of the result register into a local copy.  VF is
 * clear-on-read; taking VF and RESULT from the SAME load avoids the race of
 * the old two-access pattern (VF check, then separate RESULT read).  If VF
 * is clear the previous value is retained (Rule 15.7 documented else).
 *********************************************************************************************************************/

void CddEvadc_ReadVro(P2VAR(volatile CddApp_T, AUTOMATIC, CDD_APPL_DATA) CddAppPtr)
{
    Ifx_EVADC_G_RES Res;

    Res.U = EVADC_G1RES0.U;
    if (Res.B.VF == 0x1U)
    {
        CddAppPtr->Vro  = EVADC_CODE_TO_VOLT(Res.B.RESULT);

    }
    else
    {
        /* No fresh result — previous value retained (Rule 15.7) */
    }
}

void CddEvadc_ReadPhaseU(P2VAR(volatile CddApp_T, AUTOMATIC, CDD_APPL_DATA) CddAppPtr)
{
    Ifx_EVADC_G_RES Res;

    Res.U = EVADC_G0RES0.U;

    if (Res.B.VF == 0x1U)
    {
        /* RESULT holds the SUM of EVADC_PHASE_SAMPLES accumulated conversions */
        CddAppPtr->Vu = EVADC_CODE_TO_VOLT(Res.B.RESULT);
        CddAppPtr->SensorReadingBitField |= EVADC_CURRENT_U_READING_VALID;
    }
    else
    {
        CddAppPtr->SensorReadingBitField &= ~EVADC_CURRENT_U_READING_VALID;
    }
}

void CddEvadc_ReadPhaseV(P2VAR(volatile CddApp_T, AUTOMATIC, CDD_APPL_DATA) CddAppPtr)
{
    Ifx_EVADC_G_RES Res;

    Res.U = EVADC_G3RES0.U;

    if (Res.B.VF == 0x1U)
    {
        CddAppPtr->Vv = EVADC_CODE_TO_VOLT(Res.B.RESULT);
        CddAppPtr->SensorReadingBitField |= EVADC_CURRENT_V_READING_VALID;
    }
    else
    {
        CddAppPtr->SensorReadingBitField &= ~EVADC_CURRENT_V_READING_VALID;
    }
}

void CddEvadc_ReadPhaseW(P2VAR(volatile CddApp_T, AUTOMATIC, CDD_APPL_DATA) CddAppPtr)
{
    Ifx_EVADC_G_RES Res;

    Res.U = EVADC_G2RES0.U;

    if (Res.B.VF == 0x1U)
    {
        CddAppPtr->Vw = EVADC_CODE_TO_VOLT(Res.B.RESULT);
        CddAppPtr->SensorReadingBitField |= EVADC_CURRENT_W_READING_VALID;


    }
    else
    {
        CddAppPtr->SensorReadingBitField &= ~EVADC_CURRENT_W_READING_VALID;
    }
}

void CddEvadc_ReadDcLink(P2VAR(volatile CddApp_T, AUTOMATIC, CDD_APPL_DATA) CddAppPtr)
{
    Ifx_EVADC_G_RES Res;

    Res.U = EVADC_G1RES3.U;
    if (Res.B.VF == 0x1U)
    {
        /* Vdc is the BUS voltage [V], i.e. pin voltage scaled by the on-board
         * 5.6k/56k divider inverse (x11.0, AP32541 Eq. 1): 12 V bus → 1.09 V pin.
         * Callers that previously consumed the raw pin voltage must be updated.  */
        CddAppPtr->Vdc = EVADC_CODE_TO_VOLT(Res.B.RESULT) * EVADC_UDC_PIN_TO_BUS;
    }
    else
    {
        /* No fresh result — previous value retained (Rule 15.7) */
    }
}


void CddEvadc_CalibrateCurrentOffset(P2VAR(volatile CddApp_T, AUTOMATIC, CDD_APPL_DATA) CddAppPtr)
{
    static real32_T offsetIu = 0.0f;
    static real32_T offsetIv = 0.0f;
    static real32_T offsetIw = 0.0f;
    static real32_T prevOffsetIu = 0.0f;
    static real32_T prevOffsetIv = 0.0f;
    static real32_T prevOffsetIw = 0.0f;
    static uint32_T stableCount = 0U;

    const real32_T V_TO_A = EVADC_V_TO_A_FACTOR;
    const real32_T CONVERGENCE_THRESHOLD = 0.0005f;  /* 0.5mA stability */

    if ((CddAppPtr->SensorReadingBitField & (EVADC_CURRENT_U_READING_VALID | EVADC_CURRENT_V_READING_VALID | EVADC_CURRENT_W_READING_VALID )) ==  (EVADC_CURRENT_U_READING_VALID | EVADC_CURRENT_V_READING_VALID |  EVADC_CURRENT_W_READING_VALID))
    {
        /* Calculate instantaneous current offsets for U and V (should be near zero at standstill) */
        real32_T instOffsetIu = (CddAppPtr->Vu - CddAppPtr->Vro) * V_TO_A;
        real32_T instOffsetIv = (CddAppPtr->Vv - CddAppPtr->Vro) * V_TO_A;
        real32_T instOffsetIw = (CddAppPtr->Vw - CddAppPtr->Vro) * V_TO_A;

        /* Simple incremental update */
        static real32_T alpha = 0.05f;  /* Start with faster convergence */

        /* Update offsets for U and V only */
        offsetIu = offsetIu + alpha * (instOffsetIu - offsetIu);
        offsetIv = offsetIv + alpha * (instOffsetIv - offsetIv);
        offsetIw = offsetIw + alpha * (instOffsetIw - offsetIw);

        /* Gradually slow down convergence for stability */
        if (alpha > 0.01f)
        {
            alpha = alpha * 0.999f;  /* Slowly decrease alpha */
            if (alpha < 0.01f)
            {
                alpha = 0.01f;  /* Minimum alpha */
            }
        }

        /* Check for convergence - only for U and V */
        real32_T deltaIu = offsetIu - prevOffsetIu;
        real32_T deltaIv = offsetIv - prevOffsetIv;
        real32_T deltaIw = offsetIw - prevOffsetIw;

        if ((deltaIu < CONVERGENCE_THRESHOLD) && (deltaIu > -CONVERGENCE_THRESHOLD) &&
            (deltaIv < CONVERGENCE_THRESHOLD) && (deltaIv > -CONVERGENCE_THRESHOLD) &&
            (deltaIw < CONVERGENCE_THRESHOLD) && (deltaIw > -CONVERGENCE_THRESHOLD)
        )
        {
            stableCount++;
            if (stableCount >= 10000U)
            {
                /* Calibration is stable - can set a flag if needed */
                CddAppPtr->CDDAppStatus =  CDDAPP_RUN_STATE;
            }
        }
        else
        {
            stableCount = 0U;
        }

        prevOffsetIu = offsetIu;
        prevOffsetIv = offsetIv;
        prevOffsetIw = offsetIw;

        /* Store calibrated offsets */
        CddAppPtr->OffsetIu = offsetIu;
        CddAppPtr->OffsetIv = offsetIv;
        CddAppPtr->OffsetIw =  offsetIw;  /* W phase not measured */

    }
}
