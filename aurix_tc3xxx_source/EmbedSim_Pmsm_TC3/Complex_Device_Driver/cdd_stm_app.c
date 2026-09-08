/**********************************************************************************************************************
 * \file        cdd_stm_app.c
 * \brief       Implementation of cdd_stm_app.h — STM0 driver.
 *
 * \details     Responsibilities:
 *              1. Build the time-constant table from the live fSTM frequency.
 *              2. Configure STM0 Compare-0 for a 1 ms periodic interrupt.
 *              3. ISR: call System_Tick_Handler() then rearm the compare register.
 *
 *              ISR rearm strategy (drift-free):
 *                  elapsed = now - entry_time          (wraps correctly, uint32)
 *                  periods = (elapsed / 1ms_ticks) + 1
 *                  next    = entry_time + periods * 1ms
 *
 * \note        MISRA C:2012 compliance:
 *              - Rule  8.9  : File-scope variables limited to this TU
 *              - Rule 14.4  : All if-conditions use explicit comparison
 *              - Rule 15.5  : Single exit point per function
 *              - Rule 17.2  : No recursion
 *
 *              MISRA C:2012 deviations:
 *              - DEV-STM-001  Rule 8.4  : Stm_00_Cmp_00_Isr has no matching extern
 *                             declaration; installed via EMBED_SIM_INTERRUPT() only.
 *              - DEV-STM-003  Rule 10.5 : Hardware register fields (.U) cast to uint64_T.
 *                             PRQA S 0303 applied at each tagged site.
 *              - DEV-STM-004  Rule 10.3 : (uint32_T)TimeConst_1ms narrows uint64_T to 32 bits;
 *                             safe for all fSTM ≤ 4.3 GHz.  PRQA S 4342 applied.
 *
 * \note        EmbedSim naming convention:
 *              - Functions      : Pascal_Snake_Case
 *              - Parameters     : PascalCase
 *              - Output pointers: PascalCasePtr
 *              - Local variables: Lower camelCase
 *              - Struct members : PascalCase
 *              - Macros         : UPPER_SNAKE_CASE
 *              - Typedefs       : Pascal_Snake_Case_T
 *
 * \note        EmbedSim code style:
 *              - Indentation    : 4 spaces (no tabs)
 *              - Line width     : Max 120 characters
 *              - Braces         : Same line for functions, next line for control structures
 *              - Comments       : Doxygen style for documentation
 *              - File headers   : Block comments with \file, \brief, \details, \note, \version, etc.
 *              - Function docs  : \brief, \details, \param[in], \param[out], \return
 *              - Pointer alignment: Type* const PtrName (space before *, no space after)
 *              - Type suffix    : _T for typedefs, _G for global variables
 *              - Constants      : UPPER_SNAKE_CASE
 *
 * \copyright   Copyright (C) EmbedSim Project / Paul Abraham 2024
 *              https://github.com/vectorsim/embed_sim_project
 *              SPDX-License-Identifier: MIT
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Includes
 *********************************************************************************************************************/
#include "cdd_stm_app.h"
#include "cdd_task_handler_app.h"
#include "cdd_sys_utility.h"
#include "cdd_config.h"
#include "ifxStm_reg.h"
#include "ifxSrc_reg.h"

/**********************************************************************************************************************
 * Interrupt Vector Table Entry
 *
 * MISRA DEV-STM-001 (Rule 8.4): No extern declaration for this symbol in the header.
 * The ISR is installed exclusively through EMBED_SIM_INTERRUPT() into the .traptab section.
 *********************************************************************************************************************/
EMBED_SIM_INTERRUPT(Stm_00_Cmp_00_Isr, 0x0U, STM0_CMP0_IR_SRPN);

/**********************************************************************************************************************
 * Private Variables
 *********************************************************************************************************************/

/**
 * \brief  Pre-computed STM tick counts for each time constant  [STM ticks].
 *
 * \details Populated once in CddStm_InitTimeTable() from the live fSTM frequency.
 *          Declared extern in cdd_stm_app.h; accessed via TimeConst_xxx macros.
 */
uint64_T CddStm_TimeTable_G[TIMER_COUNT];

/**********************************************************************************************************************
 * Private Function Prototype
 *********************************************************************************************************************/
STATIC void CddStm_InitTimeTable(uint64_T StmFreq);

/**********************************************************************************************************************
 * ISR
 *********************************************************************************************************************/

/**
 * \brief   STM0 Compare-0 ISR — fires every 1 ms.
 *
 * \details Dispatches to System_Tick_Handler() then rearms the compare register.
 *
 * \note    MISRA DEV-STM-004 (Rule 10.3): (uint32_T)TimeConst_1ms narrows uint64_T.
 *          Safe for fSTM ≤ 4.3 GHz.  PRQA S 4342.
 */
void Stm_00_Cmp_00_Isr(void)
{
    uint32_T entryTime = CddStm_GetTimeLow();   /* snapshot at ISR entry   */
    uint32_T elapsed   = 0U;
    uint32_T periods   = 0U;
    uint32_T nextCmp   = 0U;

    System_Tick_Handler();

    elapsed  = CddStm_GetTimeLow() - entryTime;                               /* wraps safely — uint32 modular arithmetic  */
    periods  = (elapsed / (uint32_T)TimeConst_1ms) + 1U;                     /* PRQA S 4342 */ /* MD_STM_4342_TimeConst_1ms_32bit */
    nextCmp  = entryTime + (periods * (uint32_T)TimeConst_1ms);              /* PRQA S 4342 */ /* MD_STM_4342_TimeConst_1ms_32bit */

    STM0_CMP0.B.CMPVAL = nextCmp;
    STM0_ISCR.B.CMP0IRR = 0x1U;   /* Clear STM compare interrupt flag */
}

/**********************************************************************************************************************
 * Public Function Implementations
 *********************************************************************************************************************/

/*--------------------------------------------------------------------------------------------------------------------
 * CddStm_Init
 *------------------------------------------------------------------------------------------------------------------*/
void CddStm_Init(void)
{
    uint64_T stmFreq;

    stmFreq = (uint64_T)CddSys_GetStmFreq();
    CddStm_InitTimeTable(stmFreq);

    /* Configure STM0_CMCON: compare register 0, 32-bit compare width (MSTART=0, MSIZE=31) */
    STM0_CMCON.B.MSTART0 = 0x0U;
    STM0_CMCON.B.MSIZE0  = 0x1FU;

    /* ICR: CMP0 active, rising-edge trigger */
    STM0_ICR.B.CMP0OS = 0x0U;   /* compare trigger on rising edge */

    /* Service request: arm SRC, enable SR */
    SRC_STM0SR0.B.SRPN = STM0_CMP0_IR_SRPN;
    SRC_STM0SR0.B.TOS  = 0x0U;   /* CPU0 */
    SRC_STM0SR0.B.CLRR = 0x1U;   /* Clear any pending request */
    SRC_STM0SR0.B.SRE  = 0x1U;   /* Enable service request    */

    /* Disarm: write TIM0 >> 1 so CMP0 is firmly in the past during setup. */
    STM0_CMP0.B.CMPVAL = (CddStm_GetTimeLow() >> 0x1U);

    /* Enable compare output, then load the real first compare value. */
    STM0_ICR.B.CMP0EN  = 0x1U;
    STM0_CMP0.B.CMPVAL = CddStm_GetTimeLow() + (uint32_T)TimeConst_1ms; /* PRQA S 4342 */
}

/*--------------------------------------------------------------------------------------------------------------------
 * CddStm_GetTime
 *
 * MISRA DEV-STM-003 (Rule 10.5 / PRQA S 0303):
 * STM0_TIM0.U and STM0_CAP.U are hardware-register bitfield unions.
 * Cast to uint64_T is intentional for 64-bit reconstruction.
 *------------------------------------------------------------------------------------------------------------------*/
uint64_T CddStm_GetTime(void)
{
    uint64_T lowerSysTime;
    uint64_T upperSysTime;

    /* Read TIM0 first — hardware latches upper bits into CAP (ds2 P.60) */
    lowerSysTime = (uint64_T)STM0_TIM0.U; /* PRQA S 0303 */
    upperSysTime = (uint64_T)STM0_CAP.U;  /* PRQA S 0303 */

    /* Reconstruct: CAP holds bits [63:32]; TIM0 holds bits [31:0] */
    upperSysTime = (upperSysTime << 0x20U) | lowerSysTime;

    return upperSysTime;
}

/*--------------------------------------------------------------------------------------------------------------------
 * CddStm_GetTimeLow
 *
 * Reading TIM0 latches the upper 32 bits into STM0_CAP as a hardware side-effect.
 * Must therefore always precede any CddStm_GetTime() call within the same critical section.
 *------------------------------------------------------------------------------------------------------------------*/
uint32_T CddStm_GetTimeLow(void)
{
    return STM0_TIM0.U;
}

/*--------------------------------------------------------------------------------------------------------------------
 * CddStm_GetDeadline
 *------------------------------------------------------------------------------------------------------------------*/
uint64_T CddStm_GetDeadline(uint64_T TimeOut)
{
    uint64_T deadline = CddStm_GetTime() + TimeOut;

    return deadline;
}

/*--------------------------------------------------------------------------------------------------------------------
 * CddStm_IsDeadlineElapsed
 *
 * MISRA Rule 14.4: `now > deadline` produces an essentially Boolean result
 * assigned to uint32_T.  Explicit 0x0U / 0x1U encoding avoids an implicit
 * Boolean-to-integer conversion warning.
 *------------------------------------------------------------------------------------------------------------------*/
uint32_T CddStm_IsDeadlineElapsed(uint64_T Deadline)
{
    uint64_T now       = CddStm_GetTime();
    uint32_T isElapsed = 0x0U;

    if (now > Deadline)
    {
        isElapsed = 0x1U;
    }

    return isElapsed;
}

/*--------------------------------------------------------------------------------------------------------------------
 * CddStm_Delay_Us
 *
 * Blocking busy-wait for the requested number of microseconds.
 *
 * Implementation:
 *   deadline = now + (Microseconds × TimeConst_1us)
 *   spin until CddStm_IsDeadlineElapsed(deadline) == 0x1U
 *
 * Passing Microseconds == 0U returns immediately — the deadline equals now,
 * and the while-condition evaluates to false on the first test.
 *
 * MISRA C:2012 notes:
 *   Rule 14.4  : while-condition uses explicit == 0x0U comparison.
 *   Rule 10.3  : (uint64_T)Microseconds widens uint32_T before multiplication;
 *                no truncation.  PRQA S 4342 not required (widening only).
 *------------------------------------------------------------------------------------------------------------------*/
void CddStm_Delay_Us(uint32_T Microseconds)
{
    uint64_T deadline = CddStm_GetDeadline((uint64_T)Microseconds * TimeConst_1us);

    while (CddStm_IsDeadlineElapsed(deadline) == 0x0U)
    {
        CddSys_NopDelay(1U, 1U);
    }
}

/**********************************************************************************************************************
 * Private Function Implementations
 *********************************************************************************************************************/

/*--------------------------------------------------------------------------------------------------------------------
 * CddStm_InitTimeTable
 *
 * All divisions are exact integer divisions; fractional ticks truncated toward zero.
 * At fSTM = 300 MHz:
 *   10 ns  →  3 ticks  (300 000 000 / 100 000 000)
 *   10 s   →  3 000 000 000 ticks  (fits uint64)
 *
 * MISRA Rule 7.2 / 10.3: Divisors on the right are uint32 literals; stmFreq
 * (uint64_T) is promoted in the division — no truncation in the dividend.
 *------------------------------------------------------------------------------------------------------------------*/
STATIC void CddStm_InitTimeTable(uint64_T StmFreq)
{
    CddStm_TimeTable_G[TIMER_INDEX_10NS]  = StmFreq / (1000000000U / 10U);    /* ÷ 100 000 000 → ~3 ticks @ 300 MHz  */
    CddStm_TimeTable_G[TIMER_INDEX_100NS] = StmFreq / (1000000000U / 100U);   /* ÷  10 000 000 → ~30 ticks           */
    CddStm_TimeTable_G[TIMER_INDEX_1US]   = StmFreq / 1000000U;               /* ÷       1 000 → 300 ticks           */
    CddStm_TimeTable_G[TIMER_INDEX_10US]  = StmFreq / 100000U;                /* ÷         100 → 3 000 ticks         */
    CddStm_TimeTable_G[TIMER_INDEX_100US] = StmFreq / 10000U;                 /* ÷          10 → 30 000 ticks        */
    CddStm_TimeTable_G[TIMER_INDEX_1MS]   = StmFreq / 1000U;                  /*              → 300 000 ticks ← ISR  */
    CddStm_TimeTable_G[TIMER_INDEX_10MS]  = StmFreq / 100U;                   /*              → 3 000 000 ticks      */
    CddStm_TimeTable_G[TIMER_INDEX_100MS] = StmFreq / 10U;                    /*              → 30 000 000 ticks     */
    CddStm_TimeTable_G[TIMER_INDEX_1S]    = StmFreq;                          /*              → 300 000 000 ticks    */
    CddStm_TimeTable_G[TIMER_INDEX_10S]   = StmFreq * 10U;                    /* × 10         → 3 000 000 000 ticks  */
    CddStm_TimeTable_G[TIMER_INDEX_100S]  = StmFreq * 100U;                   /* × 100        → 30 000 000 000 ticks */
}
