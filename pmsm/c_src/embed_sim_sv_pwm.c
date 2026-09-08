/**********************************************************************************************************************
 * \file      embed_sim_sv_pwm.c
 * \brief     Space Vector PWM (SVPWM) duty-cycle calculation implementation.
 *
 * \note      EmbedSim naming convention:
 *              - Functions      : Pascal_Snake_Case
 *              - Parameters     : PascalCase  (single-letter → Uppercase)
 *              - Output pointers: PascalCase_P
 *              - Local variables: Lower camelCase
 *              - Struct members : PascalCase
 *              - Macros         : UPPER_SNAKE_CASE
 *              - Typedefs       : Pascal_Snake_Case_T
 *
 * \version   2.0.0
 * \date      2026-08-27
 * \author    EmbedSim / EV Light Vehicle Foundation
 *
 * \copyright Copyright (C) 2026 EmbedSim — EV Light Vehicle Foundation, Jaffna, Sri Lanka.
 *            Licensed under the MIT License.
 *********************************************************************************************************************/

#include "embed_sim_sv_pwm.h"
#include <math.h>
#include <stddef.h>

/**********************************************************************************************************************
 * Private Macros
 *********************************************************************************************************************/
#define SVM_ZERO_F   ((MatrixFloat)0.0f)    /**< Zero value constant */

/**********************************************************************************************************************
 * Private Function Prototypes
 *********************************************************************************************************************/

/**
 * \brief   Determine SVPWM sector from electrical angle.
 *
 * \param[in]  AngleRad  Electrical angle [rad]
 *
 * \return  Active sector [SVM_Sector_T]
 */
static SVM_Sector_T SVM_GetSectorFromAngle(MatrixFloat AngleRad);

/**
 * \brief   Calculate switching times T1 and T2 for active vectors.
 *
 * \param[in]  ActiveSector  Current active sector
 * \param[in]  AngleRad      Electrical angle [rad]
 * \param[in]  ModIndex      Modulation index [0.0 … 1.0]
 * \param[out] T1OutPtr      Pointer to T1 switching time [pu]
 * \param[out] T2OutPtr      Pointer to T2 switching time [pu]
 */
static void SVM_CalculateTimes(
    SVM_Sector_T     ActiveSector,
    MatrixFloat      AngleRad,
    MatrixFloat      ModIndex,
    MatrixFloat    * const T1OutPtr,
    MatrixFloat    * const T2OutPtr);

/**
 * \brief   Clamp floating-point value to [0.0, 1.0].
 *
 * \param[in]  Value  Input value
 *
 * \return  Clamped value
 */
static MatrixFloat SVM_ClampFloat(MatrixFloat Value);

/**
 * \brief   Calculate duty cycles from switching times.
 *
 * \param[in]  T1           T1 switching time [pu]
 * \param[in]  T2           T2 switching time [pu]
 * \param[in]  Sector       Active sector
 * \param[out] DutyOutPtr   Pointer to duty cycle output structure
 */
static void SVM_CalculateDutyFromTimes(
    MatrixFloat           T1,
    MatrixFloat           T2,
    SVM_Sector_T          Sector,
    SVM_DutyCycle_T * const DutyOutPtr);

/**********************************************************************************************************************
 * Private Function Implementations
 *********************************************************************************************************************/

/**
 * \brief   Clamp floating-point value to [0.0, 1.0].
 */
static MatrixFloat SVM_ClampFloat(MatrixFloat Value)
{
    MatrixFloat result = Value;

    if (result < SVM_ZERO_F)
    {
        result = SVM_ZERO_F;
    }
    else if (result > ES_MATH_ONE_F)
    {
        result = ES_MATH_ONE_F;
    }

    return result;
}

/**
 * \brief   Determine SVPWM sector from electrical angle.
 */
static SVM_Sector_T SVM_GetSectorFromAngle(MatrixFloat AngleRad)
{
    MatrixFloat angleNorm;
    SVM_Sector_T sector;

    /* Normalize angle to [0, 2π) using fmodf */
    angleNorm = fmodf(AngleRad, SVM_2PI_F);

    /* Handle negative angles */
    if (angleNorm < SVM_ZERO_F)
    {
        angleNorm += SVM_2PI_F;
    }

    /* Determine sector based on angle range */
    if (angleNorm < SVM_PI_OVER_3_F)
    {
        sector = SVM_SECTOR_I;
    }
    else if (angleNorm < SVM_2PI_OVER_3_F)
    {
        sector = SVM_SECTOR_II;
    }
    else if (angleNorm < SVM_PI_F)
    {
        sector = SVM_SECTOR_III;
    }
    else if (angleNorm < SVM_4PI_OVER_3_F)
    {
        sector = SVM_SECTOR_IV;
    }
    else if (angleNorm < SVM_5PI_OVER_3_F)
    {
        sector = SVM_SECTOR_V;
    }
    else
    {
        sector = SVM_SECTOR_VI;
    }

    return sector;
}

/**
 * \brief   Calculate switching times T1 and T2 for active vectors.
 */
static void SVM_CalculateTimes(
    SVM_Sector_T     ActiveSector,
    MatrixFloat      AngleRad,
    MatrixFloat      ModIndex,
    MatrixFloat    * const T1OutPtr,
    MatrixFloat    * const T2OutPtr)
{
    MatrixFloat cosT1 = SVM_ZERO_F;
    MatrixFloat cosT2 = SVM_ZERO_F;
    MatrixFloat scale = SVM_SQRT3_OVER_2_F * ModIndex;
    MatrixFloat sum;

    /* Calculate cosine terms based on sector */
    switch (ActiveSector)
    {
        case SVM_SECTOR_I:
            cosT1 = cosf(AngleRad + SVM_PI_OVER_6_F);
            cosT2 = cosf(AngleRad - SVM_PI_OVER_2_F);
            break;

        case SVM_SECTOR_II:
            cosT1 = cosf(AngleRad - SVM_PI_OVER_6_F);
            cosT2 = cosf(AngleRad - (5.0f * SVM_PI_OVER_6_F));
            break;

        case SVM_SECTOR_III:
            cosT1 = cosf(AngleRad - SVM_PI_OVER_2_F);
            cosT2 = cosf(AngleRad - (7.0f * SVM_PI_OVER_6_F));
            break;

        case SVM_SECTOR_IV:
            cosT1 = cosf(AngleRad - (5.0f * SVM_PI_OVER_6_F));
            cosT2 = cosf(AngleRad - (3.0f * SVM_PI_OVER_2_F));
            break;

        case SVM_SECTOR_V:
            cosT1 = cosf(AngleRad - (7.0f * SVM_PI_OVER_6_F));
            cosT2 = cosf(AngleRad - (11.0f * SVM_PI_OVER_6_F));
            break;

        case SVM_SECTOR_VI:
            cosT1 = cosf(AngleRad - (3.0f * SVM_PI_OVER_2_F));
            cosT2 = cosf(AngleRad - SVM_PI_OVER_6_F);
            break;

        default:
            /* No action for invalid sector */
            break;
    }

    /* Calculate switching times */
    *T1OutPtr = scale * cosT1;
    *T2OutPtr = scale * cosT2;

    /* Clamp negative values to zero */
    if (*T1OutPtr < SVM_ZERO_F)
    {
        *T1OutPtr = SVM_ZERO_F;
    }
    if (*T2OutPtr < SVM_ZERO_F)
    {
        *T2OutPtr = SVM_ZERO_F;
    }

    /* Normalize if sum exceeds unity (overmodulation handling) */
    sum = *T1OutPtr + *T2OutPtr;
    if (sum > ES_MATH_ONE_F)
    {
        *T1OutPtr = *T1OutPtr / sum;
        *T2OutPtr = *T2OutPtr / sum;
    }
}

/**
 * \brief   Calculate duty cycles from switching times.
 */
static void SVM_CalculateDutyFromTimes(
    MatrixFloat           T1,
    MatrixFloat           T2,
    SVM_Sector_T          Sector,
    SVM_DutyCycle_T * const DutyOutPtr)
{
    MatrixFloat t0 = (ES_MATH_ONE_F - T1 - T2) * ES_MATH_HALF_F;
    MatrixFloat ta;
    MatrixFloat tb;
    MatrixFloat tc;

    /* Ensure zero time is non-negative */
    if (t0 < SVM_ZERO_F)
    {
        t0 = SVM_ZERO_F;
    }

    /* Calculate duty cycles based on sector */
    switch (Sector)
    {
        case SVM_SECTOR_I:
            ta = T1 + T2 + t0;
            tb = T2 + t0;
            tc = t0;
            break;

        case SVM_SECTOR_II:
            ta = T1 + t0;
            tb = T1 + T2 + t0;
            tc = t0;
            break;

        case SVM_SECTOR_III:
            ta = t0;
            tb = T1 + T2 + t0;
            tc = T2 + t0;
            break;

        case SVM_SECTOR_IV:
            ta = t0;
            tb = T1 + t0;
            tc = T1 + T2 + t0;
            break;

        case SVM_SECTOR_V:
            ta = T2 + t0;
            tb = t0;
            tc = T1 + T2 + t0;
            break;

        case SVM_SECTOR_VI:
            ta = T1 + T2 + t0;
            tb = t0;
            tc = T1 + t0;
            break;

        default:
            /* Safety fallback for invalid sector */
            ta = SVM_ZERO_F;
            tb = SVM_ZERO_F;
            tc = SVM_ZERO_F;
            break;
    }

    /* Clamp and store duty cycles */
    DutyOutPtr->Ta = SVM_ClampFloat(ta);
    DutyOutPtr->Tb = SVM_ClampFloat(tb);
    DutyOutPtr->Tc = SVM_ClampFloat(tc);
    DutyOutPtr->Sector = Sector;
}

/**********************************************************************************************************************
 * Public Function Implementations
 *********************************************************************************************************************/

/**
 * \brief   Initialize SVPWM module.
 */
void SVM_Init(void)
{
    Transform_Init();
}

/**
 * \brief   Calculate SVPWM duty cycles from modulation index and electrical angle.
 */
MatrixStatus_T SVM_CalculateDutyCycle(
    MatrixFloat                    ModIndex,
    const FocAngle_T     * const   AnglePtr,
    SVM_DutyCycle_T      * const   DutyOutPtr)
{
    MatrixFloat t1;
    MatrixFloat t2;
    MatrixFloat angleRad;
    SVM_Sector_T sector;
    MatrixStatus_T status = MATRIX_SUCCESS;

    /* Validate input parameters */
    if ((AnglePtr == NULL) || (DutyOutPtr == NULL))
    {
        status = MATRIX_ERROR_NULL_PTR;
    }
    else if ((ModIndex < SVM_ZERO_F) || (ModIndex > ES_MATH_ONE_F))
    {
        status = MATRIX_ERROR_OUT_OF_BOUNDS;
    }
    else
    {
        /* Extract angle and determine sector */
        angleRad = AnglePtr->ThetaE;
        sector = SVM_GetSectorFromAngle(angleRad);

        /* Calculate switching times and duty cycles */
        SVM_CalculateTimes(sector, angleRad, ModIndex, &t1, &t2);
        SVM_CalculateDutyFromTimes(t1, t2, sector, DutyOutPtr);
    }

    return status;
}

/**
 * \brief   Calculate SVPWM duty cycles from αβ voltage vector.
 */
MatrixStatus_T SVM_CalculateDutyCycleFromAlphaBeta(
    const FocAlphaBeta_T * const VAlphaBetaPtr,
    const FocAngle_T     * const AnglePtr,
    MatrixFloat                  Vdc,
    SVM_DutyCycle_T      * const DutyOutPtr)
{
    MatrixFloat t1;
    MatrixFloat t2;
    MatrixFloat angleRad;
    MatrixFloat modIndex;
    MatrixFloat magnitude;
    SVM_Sector_T sector;
    MatrixStatus_T status = MATRIX_SUCCESS;

    /* Validate input parameters */
    if ((VAlphaBetaPtr == NULL) || (AnglePtr == NULL) || (DutyOutPtr == NULL))
    {
        status = MATRIX_ERROR_NULL_PTR;
    }
    else if (Vdc <= 0.0F)
    {
        status = MATRIX_ERROR_OUT_OF_BOUNDS;
    }
    else
    {
        /* Calculate magnitude of αβ voltage vector */
        magnitude = sqrtf((VAlphaBetaPtr->Alpha * VAlphaBetaPtr->Alpha) +
                          (VAlphaBetaPtr->Beta  * VAlphaBetaPtr->Beta));

        /* Normalize by Vdc/√3 (maximum linear SVPWM phase voltage) */
        MatrixFloat phaseVoltageMax = Vdc / SVM_SQRT3_F;
        modIndex = magnitude / phaseVoltageMax;

        /* Clamp modulation index to [0, 1] */
        if (modIndex > ES_MATH_ONE_F)
        {
            modIndex = ES_MATH_ONE_F;
        }
        else if (modIndex < SVM_ZERO_F)
        {
            modIndex = SVM_ZERO_F;
        }

        /* Generate SVPWM duty cycles */
        angleRad = AnglePtr->ThetaE;
        sector = SVM_GetSectorFromAngle(angleRad);
        SVM_CalculateTimes(sector, angleRad, modIndex, &t1, &t2);
        SVM_CalculateDutyFromTimes(t1, t2, sector, DutyOutPtr);
    }

    return status;
}

/**
 * \brief   Calculate SVPWM duty cycles from dq voltage vector.
 */
MatrixStatus_T SVM_CalculateDutyCycleFromDq(
    const FocDq_T        * const VDqPtr,
    const FocAngle_T     * const AnglePtr,
    MatrixFloat                  Vdc,
    SVM_DutyCycle_T      * const DutyOutPtr)
{
    MatrixStatus_T status = MATRIX_SUCCESS;
    FocAlphaBeta_T alphaBetaVoltage;

    /* Validate input parameters */
    if ((VDqPtr == NULL) || (AnglePtr == NULL) || (DutyOutPtr == NULL))
    {
        status = MATRIX_ERROR_NULL_PTR;
    }
    else if (Vdc <= 0.0F)
    {
        status = MATRIX_ERROR_OUT_OF_BOUNDS;
    }
    else
    {
        /* Transform dq to αβ using inverse Park transform */
        status = InvPark_Transform_Matrix(VDqPtr, AnglePtr, &alphaBetaVoltage);

        if (status == MATRIX_SUCCESS)
        {
            /* Generate SVPWM duty cycles from αβ voltage */
            status = SVM_CalculateDutyCycleFromAlphaBeta(&alphaBetaVoltage,
                                                         AnglePtr,
                                                         Vdc,
                                                         DutyOutPtr);
        }
    }

    return status;
}

/**
 * \brief   Convert floating-point duty cycles to centre-aligned PWM compare values.
 */
MatrixStatus_T SVM_GetCompareValues(
    const SVM_DutyCycle_T  * const DutyInPtr,
    uint32_T                       TimerPeriod,
    uint32_T               * const CompAOutPtr,
    uint32_T               * const CompBOutPtr,
    uint32_T               * const CompCOutPtr)
{
    uint32_T ticksA;
    uint32_T ticksB;
    uint32_T ticksC;
    MatrixStatus_T status = MATRIX_SUCCESS;

    /* Validate input parameters */
    if ((DutyInPtr == NULL) ||
        (CompAOutPtr == NULL) ||
        (CompBOutPtr == NULL) ||
        (CompCOutPtr == NULL))
    {
        status = MATRIX_ERROR_NULL_PTR;
    }
    else if (TimerPeriod == 0U)
    {
        status = MATRIX_ERROR_DIV_BY_ZERO;
    }
    else
    {
        /* Convert duty cycles to timer ticks */
        ticksA = (uint32_T)(DutyInPtr->Ta * (MatrixFloat)TimerPeriod);
        ticksB = (uint32_T)(DutyInPtr->Tb * (MatrixFloat)TimerPeriod);
        ticksC = (uint32_T)(DutyInPtr->Tc * (MatrixFloat)TimerPeriod);

        /* Calculate centre-aligned compare values */
        *CompAOutPtr = (TimerPeriod - ticksA) / 2U;
        *CompBOutPtr = (TimerPeriod - ticksB) / 2U;
        *CompCOutPtr = (TimerPeriod - ticksC) / 2U;
    }

    return status;
}
