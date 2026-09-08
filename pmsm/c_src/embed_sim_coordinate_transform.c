/**********************************************************************************************************************
 * \file      embed_sim_coordinate_transform.c
 * \brief     Matrix-based Clarke, Park, Inverse-Park and Inverse-Clarke transforms
 *            for FOC motor control using EmbedSim matrix library.
 *
 * \details   All transforms use matrix multiplication from embed_sim_matrix.h
 *            for clarity and code reuse.
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
 * \note      EmbedSim code style:
 *              - Indentation    : 4 spaces (no tabs)
 *              - Line width     : Max 120 characters
 *              - Braces         : Same line for functions, next line for control structures *              - Comments       : Doxygen style for documentation
 *              - File headers   : Block comments with \file, \brief, \details, \note, \version, etc.
 *              - Function docs  : \brief, \details, \param[in], \param[out], \return
 *              - Pointer alignment: Type* const PtrName (space before *, no space after)
 *              - Type suffix    : _T for typedefs, _G for global variables
 *              - Constants      : UPPER_SNAKE_CASE
 *
 * \version   2.1.0
 * \date      2025-05-24
 * \author    EmbedSim / EV Light Vehicle Foundation
 *
 * \copyright Copyright (C) 2025 EmbedSim — EV Light Vehicle Foundation, Jaffna, Sri Lanka.
 *            Licensed under the MIT License.
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * Includes
 *********************************************************************************************************************/
#include "embed_sim_coordinate_transform.h"
#include "embed_sim_compiler.h"
#include <math.h>
#include <stddef.h>

/**********************************************************************************************************************
 * Module-Private Matrix Buffers (static allocation)
 *
 * static gives these file scope only — not visible outside this translation unit.
 * Naming: UPPER_SNAKE_CASE prefix + _G suffix per EmbedSim convention.
 *********************************************************************************************************************/
static MatrixElement clarkeMatrixData_G[CLARKE_ROWS * CLARKE_COLS];
static MatrixElement invClarkeMatrixData_G[INV_CLARKE_ROWS * INV_CLARKE_COLS];
static MatrixElement parkCosMatrixData_G[PARK_ROWS * PARK_COLS];
static MatrixElement parkSinMatrixData_G[PARK_ROWS * PARK_COLS];
static MatrixElement invParkCosMatrixData_G[INV_PARK_ROWS * INV_PARK_COLS];
static MatrixElement invParkSinMatrixData_G[INV_PARK_ROWS * INV_PARK_COLS];

/**********************************************************************************************************************
 * Module-Private Matrix Handles
 *********************************************************************************************************************/
static Matrix_T clarkeMatrix_G;
static Matrix_T invClarkeMatrix_G;
static Matrix_T parkCosMatrix_G;
static Matrix_T parkSinMatrix_G;
static Matrix_T invParkCosMatrix_G;
static Matrix_T invParkSinMatrix_G;

/**********************************************************************************************************************
 * Public Functions
 *********************************************************************************************************************/

/*--------------------------------------------------------------------------------------------------------------------
 * Transform_Init
 *------------------------------------------------------------------------------------------------------------------*/
void Transform_Init(void)
{
    /* Full Clarke transform matrix (2×3):
     * [ 2/3,  -1/3,  -1/3 ]
     * [ 0,     1/√3, -1/√3 ]
     */
    Matrix_Init(&clarkeMatrix_G, clarkeMatrixData_G, CLARKE_ROWS, CLARKE_COLS);
    Matrix_SetElementFloat(&clarkeMatrix_G, 0U, 0U,  2.0f / 3.0f);          /* 2/3  */
    Matrix_SetElementFloat(&clarkeMatrix_G, 0U, 1U, -1.0f / 3.0f);          /* -1/3 */
    Matrix_SetElementFloat(&clarkeMatrix_G, 0U, 2U, -1.0f / 3.0f);          /* -1/3 */
    Matrix_SetElementFloat(&clarkeMatrix_G, 1U, 0U,  0.0f);                 /* 0    */
    Matrix_SetElementFloat(&clarkeMatrix_G, 1U, 1U,  ES_MATH_INV_SQRT3_F);  /* 1/√3 */
    Matrix_SetElementFloat(&clarkeMatrix_G, 1U, 2U, -ES_MATH_INV_SQRT3_F);  /* -1/√3 */

    /* Initialize Inverse-Clarke transform matrix */
    Matrix_Init(&invClarkeMatrix_G, invClarkeMatrixData_G, INV_CLARKE_ROWS, INV_CLARKE_COLS);
    Matrix_SetElementFloat(&invClarkeMatrix_G, 0U, 0U,  ES_MATH_ONE_F);
    Matrix_SetElementFloat(&invClarkeMatrix_G, 0U, 1U,  0.0f);
    Matrix_SetElementFloat(&invClarkeMatrix_G, 1U, 0U, -ES_MATH_HALF_F);
    Matrix_SetElementFloat(&invClarkeMatrix_G, 1U, 1U,  ES_MATH_HALF_SQRT3_F);
    Matrix_SetElementFloat(&invClarkeMatrix_G, 2U, 0U, -ES_MATH_HALF_F);
    Matrix_SetElementFloat(&invClarkeMatrix_G, 2U, 1U, -ES_MATH_HALF_SQRT3_F);

    /* Initialize Park matrices (will be updated per transform call) */
    Matrix_Init(&parkCosMatrix_G, parkCosMatrixData_G, PARK_ROWS, PARK_COLS);
    Matrix_Init(&parkSinMatrix_G, parkSinMatrixData_G, PARK_ROWS, PARK_COLS);
    Matrix_Init(&invParkCosMatrix_G, invParkCosMatrixData_G, INV_PARK_ROWS, INV_PARK_COLS);
    Matrix_Init(&invParkSinMatrix_G, invParkSinMatrixData_G, INV_PARK_ROWS, INV_PARK_COLS);
}

/*--------------------------------------------------------------------------------------------------------------------
 * Clarke_Transform_Matrix
 *------------------------------------------------------------------------------------------------------------------*/
MatrixStatus_T Clarke_Transform_Matrix(const FocUvw_T* const InPtr, FocAlphaBeta_T* const OutPtr)
{
    MatrixStatus_T    status;
    MatrixElement     inputBuffer[CLARKE_COLS];
    MatrixElement     outputBuffer[CLARKE_ROWS];
    Matrix_T          inputVec;
    Matrix_T          outputVec;

    status = MATRIX_SUCCESS;

    if ((InPtr == NULL) || (OutPtr == NULL))
    {
        status = MATRIX_ERROR_NULL_PTR;
    }
    else
    {
        /* Create input vector [U; V; W] (3×1) */
        Matrix_Init(&inputVec, inputBuffer, CLARKE_COLS, 1U);
        Matrix_SetElementFloat(&inputVec, 0U, 0U, InPtr->U);
        Matrix_SetElementFloat(&inputVec, 1U, 0U, InPtr->V);
        Matrix_SetElementFloat(&inputVec, 2U, 0U, InPtr->W);

        /* Create output vector (2×1) */
        Matrix_Init(&outputVec, outputBuffer, CLARKE_ROWS, 1U);

        /* Multiply: output = Clarke_matrix (2×3) × input (3×1) = output (2×1) */
        status = Matrix_Multiply(&clarkeMatrix_G, &inputVec, &outputVec);

        if (status == MATRIX_SUCCESS)
        {
            Matrix_GetElementFloat(&outputVec, 0U, 0U, &OutPtr->Alpha);
            Matrix_GetElementFloat(&outputVec, 1U, 0U, &OutPtr->Beta);
        }
        else
        {
            /* Multiplication failed — status already set */
        }
    }

    return status;
}

/*--------------------------------------------------------------------------------------------------------------------
 * Park_Transform_Matrix
 *------------------------------------------------------------------------------------------------------------------*/
MatrixStatus_T Park_Transform_Matrix(
    const FocAlphaBeta_T* const InPtr,
    const FocAngle_T* const AnglePtr,
    FocDq_T* const OutPtr)
{
    MatrixStatus_T status;
    MatrixElement  inputBuffer[PARK_COLS];
    MatrixElement  outputBuffer[PARK_ROWS];
    Matrix_T       inputVec;
    Matrix_T       outputVec;
    MatrixFloat    cosTheta;
    MatrixFloat    sinTheta;

    status = MATRIX_SUCCESS;

    if ((InPtr == NULL) || (AnglePtr == NULL) || (OutPtr == NULL))
    {
        status = MATRIX_ERROR_NULL_PTR;
    }
    else
    {
        /* Compute sin and cos of electrical angle */
        cosTheta = cosf(AnglePtr->ThetaE);
        sinTheta = sinf(AnglePtr->ThetaE);

        /* Build Park transform matrix: [cosθ, sinθ; -sinθ, cosθ] */
        Matrix_SetElementFloat(&parkCosMatrix_G, 0U, 0U,  cosTheta);
        Matrix_SetElementFloat(&parkCosMatrix_G, 0U, 1U,  sinTheta);
        Matrix_SetElementFloat(&parkCosMatrix_G, 1U, 0U, -sinTheta);
        Matrix_SetElementFloat(&parkCosMatrix_G, 1U, 1U,  cosTheta);

        /* Create input vector [Alpha; Beta] (2×1) */
        Matrix_Init(&inputVec, inputBuffer, PARK_COLS, 1U);
        Matrix_SetElementFloat(&inputVec, 0U, 0U, InPtr->Alpha);
        Matrix_SetElementFloat(&inputVec, 1U, 0U, InPtr->Beta);

        /* Create output vector (2×1) */
        Matrix_Init(&outputVec, outputBuffer, PARK_ROWS, 1U);

        /* Multiply: output = Park_matrix × input */
        status = Matrix_Multiply(&parkCosMatrix_G, &inputVec, &outputVec);

        if (status == MATRIX_SUCCESS)
        {
            Matrix_GetElementFloat(&outputVec, 0U, 0U, &OutPtr->D);
            Matrix_GetElementFloat(&outputVec, 1U, 0U, &OutPtr->Q);
        }
        else
        {
            /* Multiplication failed — status already set */
        }
    }

    return status;
}

/*--------------------------------------------------------------------------------------------------------------------
 * InvPark_Transform_Matrix
 *------------------------------------------------------------------------------------------------------------------*/
MatrixStatus_T InvPark_Transform_Matrix(
    const FocDq_T* const InPtr,
    const FocAngle_T* const AnglePtr,
    FocAlphaBeta_T* const OutPtr)
{
    MatrixStatus_T status;
    MatrixElement  inputBuffer[INV_PARK_COLS];
    MatrixElement  outputBuffer[INV_PARK_ROWS];
    Matrix_T       inputVec;
    Matrix_T       outputVec;
    MatrixFloat    cosTheta;
    MatrixFloat    sinTheta;

    status = MATRIX_SUCCESS;

    if ((InPtr == NULL) || (AnglePtr == NULL) || (OutPtr == NULL))
    {
        status = MATRIX_ERROR_NULL_PTR;
    }
    else
    {
        /* Compute sin and cos of electrical angle */
        cosTheta = cosf(AnglePtr->ThetaE);
        sinTheta = sinf(AnglePtr->ThetaE);

        /* Build Inverse-Park transform matrix: [cosθ, -sinθ; sinθ, cosθ] */
        Matrix_SetElementFloat(&invParkCosMatrix_G, 0U, 0U,  cosTheta);
        Matrix_SetElementFloat(&invParkCosMatrix_G, 0U, 1U, -sinTheta);
        Matrix_SetElementFloat(&invParkCosMatrix_G, 1U, 0U,  sinTheta);
        Matrix_SetElementFloat(&invParkCosMatrix_G, 1U, 1U,  cosTheta);

        /* Create input vector [D; Q] (2×1) */
        Matrix_Init(&inputVec, inputBuffer, INV_PARK_COLS, 1U);
        Matrix_SetElementFloat(&inputVec, 0U, 0U, InPtr->D);
        Matrix_SetElementFloat(&inputVec, 1U, 0U, InPtr->Q);

        /* Create output vector (2×1) */
        Matrix_Init(&outputVec, outputBuffer, INV_PARK_ROWS, 1U);

        /* Multiply: output = InvPark_matrix × input */
        status = Matrix_Multiply(&invParkCosMatrix_G, &inputVec, &outputVec);

        if (status == MATRIX_SUCCESS)
        {
            Matrix_GetElementFloat(&outputVec, 0U, 0U, &OutPtr->Alpha);
            Matrix_GetElementFloat(&outputVec, 1U, 0U, &OutPtr->Beta);
        }
        else
        {
            /* Multiplication failed — status already set */
        }
    }

    return status;
}

/*--------------------------------------------------------------------------------------------------------------------
 * InvClarke_Transform_Matrix
 *------------------------------------------------------------------------------------------------------------------*/
MatrixStatus_T InvClarke_Transform_Matrix(
    const FocAlphaBeta_T* const InPtr,
    FocUvw_T* const OutPtr)
{
    MatrixStatus_T status;
    MatrixElement  inputBuffer[INV_CLARKE_COLS];
    MatrixElement  outputBuffer[INV_CLARKE_ROWS];
    Matrix_T       inputVec;
    Matrix_T       outputVec;

    status = MATRIX_SUCCESS;

    if ((InPtr == NULL) || (OutPtr == NULL))
    {
        status = MATRIX_ERROR_NULL_PTR;
    }
    else
    {
        /* Create input vector [Alpha; Beta] (2×1) */
        Matrix_Init(&inputVec, inputBuffer, INV_CLARKE_COLS, 1U);
        Matrix_SetElementFloat(&inputVec, 0U, 0U, InPtr->Alpha);
        Matrix_SetElementFloat(&inputVec, 1U, 0U, InPtr->Beta);

        /* Create output vector (3×1) */
        Matrix_Init(&outputVec, outputBuffer, INV_CLARKE_ROWS, 1U);

        /* Multiply: output = InvClarke_matrix × input */
        status = Matrix_Multiply(&invClarkeMatrix_G, &inputVec, &outputVec);

        if (status == MATRIX_SUCCESS)
        {
            Matrix_GetElementFloat(&outputVec, 0U, 0U, &OutPtr->U);
            Matrix_GetElementFloat(&outputVec, 1U, 0U, &OutPtr->V);
            Matrix_GetElementFloat(&outputVec, 2U, 0U, &OutPtr->W);
        }
        else
        {
            /* Multiplication failed — status already set */
        }
    }

    return status;
}
