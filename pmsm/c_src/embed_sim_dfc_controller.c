/**********************************************************************************************************************
 * \file      embed_sim_dfc_controller.c
 * \brief     DFC (Direct Field Control) controller implementation.
 *
 * \details   Implements differential‑flatness feedforward with speed and current PI loops.
 *            Startup is a 0.3 s open‑loop voltage ramp (modulation 0.05 → 0.20),
 *            exactly matching the Python version. After startup, the controller runs
 *            closed‑loop DFC indefinitely.
 *
 *            The state is controlled solely by `SwitchToClosedLoop`:
 *            - 0 : startup phase (open‑loop ramp)
 *            - 1 : closed‑loop DFC
 *
 *            Resetting is done via `ControlReInit = 1` (or calling `DFC_Reset()`),
 *            which sets `SwitchToClosedLoop = 0` and clears all integrators.
 *
 * \note      MISRA C:2012 compliance:
 *              - Rule  8.5 : One declaration per identifier.
 *              - Rule  8.6 : No definitions in header files.
 *              - Rule 17.2 : No recursion.
 *              - Rule 14.7 : Single return point.
 *
 * \note      EmbedSim naming convention:
 *              - Functions      : Pascal_Snake_Case
 *              - Parameters     : PascalCase
 *              - Output pointers: PascalCasePtr
 *              - Local variables: Lower camelCase
 *              - Struct members : PascalCase
 *              - Macros         : UPPER_SNAKE_CASE
 *              - Typedefs       : Pascal_Snake_Case_T
 *
 * \version   2.0.0
 * \date      2026-08-23
 * \author    EmbedSim / EV Light Vehicle Foundation
 *
 * \copyright Copyright (C) 2026 EmbedSim — EV Light Vehicle Foundation, Jaffna, Sri Lanka.
 *            Licensed under the MIT License.
 *********************************************************************************************************************/

#include "embed_sim_dfc_controller.h"
#include "embed_sim_sv_pwm.h"
#include "embed_sim_coordinate_transform.h"
#include "embed_sim_matrix.h"
#include "embed_sim_control.h"
#include <math.h>

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/

/**
 * \def DFC_MAX_CURRENT
 * \brief   Maximum current limit (A).
 */
#define DFC_MAX_CURRENT                 (100.0F)

/**
 * \def DFC_MAX_IQ_DOT_F
 * \brief   Maximum current derivative limit (A/s).
 */
#define DFC_MAX_IQ_DOT_F                (1000.0F)

/**
 * \def DFC_EPSILON_F
 * \brief   Numerical protection epsilon.
 */
#define DFC_EPSILON_F                   (1.0e-6F)

/**
 * \def DFC_SQRT3_F
 * \brief   Square root of 3 (for SVM voltage limit).
 */
#define DFC_SQRT3_F                     (1.7320508075688772F)

/**
 * \def DFC_STARTUP_DURATION_S
 * \brief   Startup duration in seconds (0.3s matches Python).
 */
#define DFC_STARTUP_DURATION_S          (3.0F)



/*********************************************************************************************************************/
/*--------------------------------------------------Private Data-----------------------------------------------------*/
/*********************************************************************************************************************/


/*********************************************************************************************************************/
/*--------------------------------------------Private Functions-----------------------------------------------------*/
/*********************************************************************************************************************/

/**
 * \brief   Transform phase currents to the dq rotating reference frame.
 *
 * \param[in]  inputPtr   Pointer to control input structure (phase currents).
 * \param[in]  machinePtr Pointer to machine parameters (pole pairs).
 * \param[out] focDqPtr   Pointer to dq current output structure.
 */
static void DFC_CurrentsToDq(EmbedSimCtrlInput_T* const inputPtr,
                             const EmbedSimMachineParam_T* const machinePtr,
                             FocDq_T* const focDqPtr)
{
    FocUvw_T currents;
    FocAlphaBeta_T alphaBeta;
    FocAngle_T angle;

    /* Pack phase currents into UVW structure */
    currents.U = inputPtr->Iu;
    currents.V = inputPtr->Iv;
    currents.W = inputPtr->Iw;

    /* Compute electrical angle from mechanical position and pole pairs */
    angle.ThetaE = inputPtr->RotorPositionObsEstM * machinePtr->PolePairs;
    EmbedSim_WrapAngleTwoPi(&angle.ThetaE);

    /* Clarke and Park transforms to obtain dq currents */
    Clarke_Transform_Matrix(&currents, &alphaBeta);
    Park_Transform_Matrix(&alphaBeta, &angle, focDqPtr);
}

/**
 * \brief   Reset the DFC controller state.
 *
 * \details Clears all integrators and resets the startup angle.
 *          The caller must also set `ControlReInit = 1` (or the reset
 *          will take effect on the next step when `ControlReInit` is
 *          handled in DFC_Step). This function is typically called
 *          from `EmbedSim_ResetController()`.
 *
 * \return  void
 */
void DFC_Reset(EmbedSimMachine_T* const MotorPtr)
{
    /* Clear all integrator states to prevent carry-over from previous runs */
    MotorPtr->MachinePtr->SpeedIntegralError = 0.0F;
    MotorPtr->MachinePtr->IdIntegralError    = 0.0F;
    MotorPtr->MachinePtr->IqIntegralError    = 0.0F;
    MotorPtr->MachinePtr->SvmModulationIndex = 0.0F;
    MotorPtr->InputPtr->SwitchToClosedLoop   = 0x0U;
    MotorPtr->InputPtr->ControlReInit        = 0x0U;
    MotorPtr->MachinePtr ->SvmStartUpTimer   = 0.0F;

}

/*********************************************************************************************************************/
/*--------------------------------------------Public Functions------------------------------------------------------*/
/*********************************************************************************************************************/

/**
 * \brief   Initialize the DFC controller.
 *
 * \details Resets all integrators and the startup angle.
 *          The controller will start in open‑loop (SwitchToClosedLoop = 0)
 *          and run the startup ramp on the next step.
 *
 * \return  void
 */
void DFC_Init(EmbedSimMachine_T* const MotorPtr)
{
    DFC_Reset(MotorPtr);
    MotorPtr->MachinePtr->SvmRotorThetaE  = 0.0F;
    MotorPtr->MachinePtr->SvmStartUpTimer = 0.0F;
    MotorPtr->MachinePtr->SvmRotorThetaE  = 0.0F;
}


/**
 * \brief   Execute one step of Differential Flatness Control.
 *
 * \details The controller state is determined by `SwitchToClosedLoop`:
 *          - **0** : Startup phase – runs open‑loop voltage ramp for
 *                    0.3 seconds (modulation 0.05 → 0.20).
 *          - **1** : Normal closed‑loop DFC.
 *
 *          During startup, the controller can be configured to use either
 *          the actual rotor position (DFC_USE_SENSOR_DURING_STARTUP = 1)
 *          or a calculated angle from reference speed (DFC_USE_SENSOR_DURING_STARTUP = 0).
 *
 *          Resetting is done by setting `ControlReInit = 1`, which
 *          clears integrators and forces `SwitchToClosedLoop = 0`
 *          to re‑enter the startup ramp.
 *
 *          This function has a single return point for MISRA compliance.
 *
 * \param[in]  motorPtr  Pointer to the motor structure.
 *
 * \return  void
 */
void DFC_Step(EmbedSimMachine_T* const MotorPtr)
{
    EmbedSimCtrlInput_T*    const inputPtr   = MotorPtr->InputPtr;
    EmbedSimCtrlOutput_T*   const outputPtr  = MotorPtr->OutputPtr;
    EmbedSimMachineParam_T* const machinePtr = MotorPtr->MachinePtr;

    /*
     * ---------- Local variables (one per line, MISRA Rule 8.5) ----------
     */
    real32_T omegaRef;
    real32_T omegaRefDot;
    real32_T omegaRefDDot;
    real32_T omegaMeas;

    real32_T speedError;
    real32_T torqueCorrection;
    real32_T torqueFeedforward;
    real32_T torqueRequired;

    real32_T torqueConstant;
    real32_T iqRef;
    real32_T iqRefDot;
    real32_T vdRef;
    real32_T vqRef;

    real32_T idError;
    real32_T iqError;
    real32_T vdCorr;
    real32_T vqCorr;

    FocDq_T dqCurrentMeas;
    FocDq_T dqVoltage;
    FocAngle_T focAngle;
    FocAlphaBeta_T abVoltage;
    SVM_DutyCycle_T svmDC;          /* unused in closed‑loop, kept for compatibility */
    SVM_DutyCycle_T startupSvmDC;
    real32_T tauStart;
    FocUvw_T phase;                 /* for direct PWM generation */
    real32_T vHalf;
    real32_T vMag;

    tauStart = 0.0F;
    vMag     = 0.0F;

    /* ================================================================
     * 1. Startup phase (open‑loop voltage ramp)
     * ================================================================ */
    if(inputPtr->SwitchToClosedLoop == 0x0U)
    {
        machinePtr->SvmStartUpTimer += inputPtr->SampleTime;
        tauStart = (machinePtr->SvmStartUpTimer / DFC_STARTUP_DURATION_S);

        machinePtr->SvmModulationIndex = DFC_STARTUP_MOD_MIN + (tauStart * (DFC_STARTUP_MOD_MAX - DFC_STARTUP_MOD_MIN));
        machinePtr->SvmModulationIndex = EmbedSim_ClampValue(machinePtr->SvmModulationIndex, DFC_STARTUP_MOD_MIN, DFC_STARTUP_MOD_MAX);
        machinePtr->SvmRotorThetaE += (machinePtr->PolePairs * CON_RPM_TO_RAD(inputPtr->AngularVelocityRefRpmM * tauStart) * inputPtr->SampleTime);
        EmbedSim_WrapAngleTwoPi(&machinePtr->SvmRotorThetaE);
        SVM_CalculateDutyCycle(machinePtr->SvmModulationIndex, &machinePtr->SvmRotorThetaE, &startupSvmDC);

        outputPtr->DutyU = startupSvmDC.Ta;
        outputPtr->DutyV = startupSvmDC.Tb;
        outputPtr->DutyW = startupSvmDC.Tc;
        outputPtr->SvmSector = startupSvmDC.Sector;
        outputPtr->Valid = 0x1U;

        /* Switch to closed‑loop after 3 seconds */
        if (machinePtr->SvmStartUpTimer > DFC_STARTUP_DURATION_S)
        {
            inputPtr->SwitchToClosedLoop = 0x1U;
            machinePtr->SvmStartUpTimer  = 0.0F;

            /* Clear integrators to avoid windup from open‑loop */
            machinePtr->SpeedIntegralError = 0.0F;
            machinePtr->IdIntegralError    = 0.0F;
            machinePtr->IqIntegralError    = 0.0F;

            /* Initialise model angle from sensor */
            machinePtr->SvmRotorThetaE = inputPtr->RotorPositionObsEstM * machinePtr->PolePairs;
            EmbedSim_WrapAngleTwoPi(&machinePtr->SvmRotorThetaE);

            /* Re‑initialise trajectory from measured speed */
            inputPtr->ControlReInit = 0x1U;
        }
    }

    /* ================================================================
     * 2. Normal DFC (closed‑loop)
     * ================================================================ */
    if (inputPtr->SwitchToClosedLoop == 1U)
    {
        /* Generate smooth S‑curve trajectory */
        EmbedSim_CalculateJerkLimitedTrajectory(inputPtr, machinePtr);

        /* Read references and measured speed (all in rad/s) */
        omegaRef     = inputPtr->RotorVelocityRefM;
        omegaRefDot  = inputPtr->RotorAccelerationRefM;
        omegaRefDDot = inputPtr->RotorJerkRefM;
        omegaMeas    = CON_RPM_TO_RAD(inputPtr->RotorSpeedObsEstM);

        /* ---------- Speed PI (outer loop) ---------- */
        speedError = omegaRef - omegaMeas;

        /* Integral update with sample‑time compensation */
        machinePtr->SpeedIntegralError += speedError * inputPtr->SampleTime;
        machinePtr->SpeedIntegralError = EmbedSim_ClampValue(machinePtr->SpeedIntegralError, -machinePtr->ParamPidIntegralLimit, machinePtr->ParamPidIntegralLimit);

        torqueCorrection = (machinePtr->ParamPidSpeedQProp * speedError) +
                           (machinePtr->ParamPidSpeedQInteg * machinePtr->SpeedIntegralError);

        /* ---------- Mechanical flatness (feedforward torque) ---------- */
        torqueFeedforward = (machinePtr->J * omegaRefDot) +
                            (machinePtr->B * omegaRef) +
                            machinePtr->TorqueLoad;
        torqueRequired = torqueFeedforward + torqueCorrection;

        /* ---------- Electrical flatness (current reference) ---------- */
        torqueConstant = 1.5F * machinePtr->PolePairs * machinePtr->FluxPm;
        if (fabsf(torqueConstant) > DFC_EPSILON_F)
        {
            iqRef = torqueRequired / torqueConstant;
            iqRef = EmbedSim_ClampValue(iqRef, -DFC_MAX_CURRENT, DFC_MAX_CURRENT);

            iqRefDot = ((machinePtr->J * omegaRefDDot) +
                        (machinePtr->B * omegaRefDot)) / torqueConstant;
            iqRefDot = EmbedSim_ClampValue(iqRefDot, -DFC_MAX_IQ_DOT_F, DFC_MAX_IQ_DOT_F);
        }
        else
        {
            iqRef    = 0.0F;
            iqRefDot = 0.0F;
        }

        /* ---------- Voltage feedforward (flatness) ---------- */
        vdRef = -machinePtr->PolePairs * omegaRef * machinePtr->Lq * iqRef;
        vqRef = (machinePtr->Rs * iqRef) +
                (machinePtr->Lq * iqRefDot) +
                (machinePtr->PolePairs * omegaRef * machinePtr->FluxPm);

        /* ---------- Measure currents and compute errors ---------- */
        DFC_CurrentsToDq(inputPtr, machinePtr, &dqCurrentMeas);

        idError = 0.0F - dqCurrentMeas.D;
        iqError = iqRef - dqCurrentMeas.Q;

        /* ---------- Current PI (inner loops) ---------- */
        machinePtr->IdIntegralError += idError * inputPtr->SampleTime;
        machinePtr->IqIntegralError += iqError * inputPtr->SampleTime;

        machinePtr->IdIntegralError = EmbedSim_ClampValue(machinePtr->IdIntegralError, -machinePtr->ParamPidIntegralLimit, machinePtr->ParamPidIntegralLimit);
        machinePtr->IqIntegralError = EmbedSim_ClampValue(machinePtr->IqIntegralError, -machinePtr->ParamPidIntegralLimit, machinePtr->ParamPidIntegralLimit);

        /* Clamp errors to prevent excessive correction */
        idError = EmbedSim_ClampValue(idError, -DFC_MAX_CURRENT, DFC_MAX_CURRENT);
        iqError = EmbedSim_ClampValue(iqError, -DFC_MAX_CURRENT, DFC_MAX_CURRENT);

        /* PI correction voltages */
        vdCorr = (machinePtr->ParamPidCurrentDProp * idError) +
                 (machinePtr->ParamPidCurrentDInteg * machinePtr->IdIntegralError);
        vqCorr = (machinePtr->ParamPidCurrentQProp * iqError) +
                 (machinePtr->ParamPidCurrentQInteg * machinePtr->IqIntegralError);

        /* Final dq voltage commands */
        dqVoltage.D = vdRef + vdCorr;
        dqVoltage.Q = vqRef + vqCorr;

        /* ---------- Inverse Park: dq → αβ ---------- */
        focAngle.ThetaE = machinePtr->SvmRotorThetaE;
        EmbedSim_WrapAngleTwoPi(&focAngle.ThetaE);
        InvPark_Transform_Matrix(&dqVoltage, &focAngle, &abVoltage);


#if ES_SIM_OP_MODE == 0x0U

        vMag = sqrtf(abVoltage.Alpha*abVoltage.Alpha + abVoltage.Beta*abVoltage.Beta);
        machinePtr->SvmModulationIndex = vMag / (machinePtr->Vdc / DFC_SQRT3_F);
        machinePtr->SvmModulationIndex = EmbedSim_ClampValue(machinePtr->SvmModulationIndex, 0.0F, 0.80F);

        SVM_CalculateDutyCycle(machinePtr->SvmModulationIndex, &focAngle, &svmDC);

        outputPtr->DutyU = svmDC.Ta;
        outputPtr->DutyV = svmDC.Tb;
        outputPtr->DutyW = svmDC.Tc;
        outputPtr->SvmSector = svmDC.Sector;
        outputPtr->Valid = 0x1U;
#else
        /* ---------- Direct PWM generation (matching Python) ---------- */
        vHalf = machinePtr->Vdc / 2.0f;
        InvClarke_Transform_Matrix(&abVoltage, &phase);
        /* Clamp to duty cycles */
        outputPtr->DutyU = EmbedSim_ClampValue((phase.U / vHalf + 1.0f) / 2.0f, 0.0f, 1.0f);
        outputPtr->DutyV = EmbedSim_ClampValue((phase.V / vHalf + 1.0f) / 2.0f, 0.0f, 1.0f);
        outputPtr->DutyW = EmbedSim_ClampValue((phase.W / vHalf + 1.0f) / 2.0f, 0.0f, 1.0f);

        /* Sector is not used, but keep for compatibility */
        outputPtr->SvmSector = 0U;
        outputPtr->Valid = 0x1U;
#endif

    }
}
