/**********************************************************************************************************************
 * \file        cdd_stm_app.h
 * \brief       Public interface for the System Timer Module (STM0) driver.
 *
 * \details     Configures STM0 Compare-0 to generate a periodic 1 ms system
 *              tick interrupt on CPU0.  The ISR calls System_Tick_Handler()
 *              owned by cdd_task_handler_app.c.
 *
 *              Hardware capture note (TC3xx Reference Manual ds2 P.60):
 *              Reading STM0_TIM0 automatically latches the upper 32 bits into
 *              STM0_CAP.  CddStm_GetTimeLow() reads TIM0 only;
 *              CddStm_GetTime() reads TIM0 then CAP to reconstruct the full
 *              64-bit counter atomically under interrupt lock.
 *
 *              Typical non-blocking timeout pattern:
 *              \code
 *                  uint64_T deadline = CddStm_GetDeadline(TimeConst_10ms);
 *                  while (CddStm_IsDeadlineElapsed(deadline) == 0x0U) { ; }
 *              \endcode
 *
 * \note        MISRA C:2012 compliance:
 *              - Rule  8.5 : One declaration per function, matching .c definition
 *              - Rule  8.6 : Definitions reside in cdd_stm_app.c
 *
 *              MISRA C:2012 deviations:
 *              - DEV-STM-001  Rule 8.4 : Stm_00_Cmp_00_Isr has no matching extern
 *                             declaration; installed exclusively via EMBED_SIM_INTERRUPT().
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

#ifndef CDD_STM_APP_H_
#define CDD_STM_APP_H_

/**********************************************************************************************************************
 * Includes
 *********************************************************************************************************************/
#include "cdd_config.h"   /* embed_sim_sys_types.h + embed_sim_compiler.h */

/**********************************************************************************************************************
 * Macros — Time-Table Indices
 *********************************************************************************************************************/

/** \brief  Total number of time-constant entries in CddStm_TimeTable_G.    */
#define TIMER_COUNT         (11U)

#define TIMER_INDEX_10NS    (0U)    /**< Index — 10 ns time constant          */
#define TIMER_INDEX_100NS   (1U)    /**< Index — 100 ns time constant         */
#define TIMER_INDEX_1US     (2U)    /**< Index — 1 µs time constant           */
#define TIMER_INDEX_10US    (3U)    /**< Index — 10 µs time constant          */
#define TIMER_INDEX_100US   (4U)    /**< Index — 100 µs time constant         */
#define TIMER_INDEX_1MS     (5U)    /**< Index — 1 ms time constant (ISR arm) */
#define TIMER_INDEX_10MS    (6U)    /**< Index — 10 ms time constant          */
#define TIMER_INDEX_100MS   (7U)    /**< Index — 100 ms time constant         */
#define TIMER_INDEX_1S      (8U)    /**< Index — 1 s time constant            */
#define TIMER_INDEX_10S     (9U)    /**< Index — 10 s time constant           */
#define TIMER_INDEX_100S    (10U)   /**< Index — 100 s time constant          */

/*--------------------------------------------------------------------------------------------------------------------
 * CddStm_TimeTable_G — extern array declaration
 *
 * Allocated and populated in cdd_stm_app.c by CddStm_InitTimeTable().
 * Downstream TUs access individual entries via the TimeConst_xxx macros.
 *------------------------------------------------------------------------------------------------------------------*/
extern uint64_T CddStm_TimeTable_G[TIMER_COUNT];

/*--------------------------------------------------------------------------------------------------------------------
 * TimeConst_xxx — convenience accessors for CddStm_TimeTable_G
 *------------------------------------------------------------------------------------------------------------------*/
#define TimeConst_10ns  (CddStm_TimeTable_G[TIMER_INDEX_10NS])    /**< \brief 10 ns  [STM ticks] */
#define TimeConst_100ns (CddStm_TimeTable_G[TIMER_INDEX_100NS])   /**< \brief 100 ns [STM ticks] */
#define TimeConst_1us   (CddStm_TimeTable_G[TIMER_INDEX_1US])     /**< \brief 1 µs   [STM ticks] */
#define TimeConst_10us  (CddStm_TimeTable_G[TIMER_INDEX_10US])    /**< \brief 10 µs  [STM ticks] */
#define TimeConst_100us (CddStm_TimeTable_G[TIMER_INDEX_100US])   /**< \brief 100 µs [STM ticks] */
#define TimeConst_1ms   (CddStm_TimeTable_G[TIMER_INDEX_1MS])     /**< \brief 1 ms   [STM ticks] */
#define TimeConst_10ms  (CddStm_TimeTable_G[TIMER_INDEX_10MS])    /**< \brief 10 ms  [STM ticks] */
#define TimeConst_100ms (CddStm_TimeTable_G[TIMER_INDEX_100MS])   /**< \brief 100 ms [STM ticks] */
#define TimeConst_1s    (CddStm_TimeTable_G[TIMER_INDEX_1S])      /**< \brief 1 s    [STM ticks] */
#define TimeConst_10s   (CddStm_TimeTable_G[TIMER_INDEX_10S])     /**< \brief 10 s   [STM ticks] */
#define TimeConst_100s  (CddStm_TimeTable_G[TIMER_INDEX_100S])    /**< \brief 100 s  [STM ticks] */

/**********************************************************************************************************************
 * Function Prototypes
 *********************************************************************************************************************/

/**
 * \brief   Initialises STM0 and arms the Compare-0 interrupt for a 1 ms period.
 *
 * \details Computes the full time-constant table from the live fSTM frequency,
 *          configures CMCON / ICR / SRC registers, then loads the first compare value.
 *          Must be called once during system init before global interrupt enable.
 *
 * \return  void
 */
extern void CddStm_Init(void);

/**
 * \brief   Returns the lower 32 bits of the STM0 free-running counter.
 *
 * \details Reads STM0_TIM0.U directly.  As a side-effect the hardware latches
 *          the upper bits into STM0_CAP — required before any CddStm_GetTime() call.
 *
 * \return  Current lower 32-bit STM tick count  [STM ticks]
 */
extern uint32_T CddStm_GetTimeLow(void);

/**
 * \brief   Returns the full 64-bit STM0 free-running counter value.
 *
 * \details Reads TIM0 first (triggers CAP latch per TC3xx RM ds2 P.60), then
 *          reads CAP, and reconstructs the full 64-bit value.
 *
 * \return  Current 64-bit STM tick count  [STM ticks]
 */
extern uint64_T CddStm_GetTime(void);

/**
 * \brief   Computes an absolute deadline from a relative timeout.
 *
 * \param[in]  TimeOut  Timeout in STM ticks.  Use TimeConst_xxx macros.
 * \return  Absolute 64-bit STM tick count at which the deadline expires.
 */
extern uint64_T CddStm_GetDeadline(uint64_T TimeOut);

/**
 * \brief   Tests whether an absolute deadline has been reached or exceeded.
 *
 * \param[in]  Deadline  Absolute STM tick deadline from CddStm_GetDeadline().
 * \return  0x1U if current time > Deadline, 0x0U otherwise.
 */
extern uint32_T CddStm_IsDeadlineElapsed(uint64_T Deadline);

/**
 * \brief   Busy-wait delay of the specified number of microseconds.
 *
 * \details Computes an absolute deadline as
 *              now + (Microseconds × TimeConst_1us)
 *          and spins until CddStm_IsDeadlineElapsed() returns 0x1U.
 *
 *          Minimum resolution is one STM tick (~3.3 ns at 300 MHz).
 *          Passing 0 returns immediately without blocking.
 *
 * \pre     CddStm_Init() must have been called so that CddStm_TimeTable_G
 *          is populated before this function is used.
 *
 * \param[in]  Microseconds  Delay duration [µs], must be >= 0.
 * \return  void
 */
extern void CddStm_Delay_Us(uint32_T Microseconds);

#endif /* CDD_STM_APP_H_ */
