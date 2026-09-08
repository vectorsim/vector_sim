/**********************************************************************************************************************
 * \file      embed_sim_coordinate_transform.h
 * \brief     Clarke, Park, Inverse-Park and Inverse-Clarke transforms for FOC motor control.
 *
 * \details   All transforms use matrix operations from embed_sim_matrix.h for
 *            clarity and maintainability. Matrix-based implementation using
 *            EmbedSim matrix library.
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
 *              - Braces         : Same line for functions, next line for control structures
 *              - Comments       : Doxygen style for documentation
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

#ifndef EMBED_SIM_COORDINATE_TRANSFORM_H_
#define EMBED_SIM_COORDINATE_TRANSFORM_H_

/**********************************************************************************************************************
 * Includes
 *********************************************************************************************************************/
#include "embed_sim_foc_types.h"
#include "embed_sim_matrix.h"

/**********************************************************************************************************************
 * Transform Matrices Dimensions
 *********************************************************************************************************************/
#define CLARKE_ROWS      (2U)    /**< Clarke output: Alpha, Beta          */
#define CLARKE_COLS      (3U)    /**< Clarke input: U, V, W               */
#define PARK_ROWS        (2U)    /**< Park output: D, Q                   */
#define PARK_COLS        (2U)    /**< Park input: Alpha, Beta             */
#define INV_PARK_ROWS    (2U)    /**< Inverse-Park output: Alpha, Beta    */
#define INV_PARK_COLS    (2U)    /**< Inverse-Park input: D, Q            */
#define INV_CLARKE_ROWS  (3U)    /**< Inverse-Clarke output: U, V, W      */
#define INV_CLARKE_COLS  (2U)    /**< Inverse-Clarke input: Alpha, Beta   */

/* Transform matrix buffers and handles are module-private (STATIC in .c).
 * No extern declarations — access only via the public API below.          */

/**********************************************************************************************************************
 * Function Prototypes
 *********************************************************************************************************************/

/**
 * \brief   Initialize transform matrices (call once at startup)
 */
extern void Transform_Init(void);

/**
 * \brief   Apply the amplitude-invariant Clarke transform using matrix multiplication.
 *
 * \details Formula (now using all 3 phases):
 * \code
 *   [Alpha]   = [ 2/3   -1/3   -1/3 ]   [U]
 *   [Beta ]     [ 0     1/√3  -1/√3 ]   [V]
 *                                       [W]
 * \endcode
 *
 * \param[in]  InPtr   UVW phase signals (must not be NULL)
 * \param[out] OutPtr  αβ output (must not be NULL)
 * \return  MATRIX_SUCCESS or error code
 */
extern MatrixStatus_T Clarke_Transform_Matrix(const FocUvw_T* const InPtr, FocAlphaBeta_T* const OutPtr);

/**
 * \brief   Apply the Park (forward) transform using matrix multiplication.
 *
 * \details Formula:
 * \code
 *   [D] = [cos(θ)   sin(θ)] * [Alpha]
 *   [Q]   [-sin(θ)  cos(θ)]   [Beta]
 * \endcode
 *
 * \param[in]  InPtr    αβ input (must not be NULL)
 * \param[in]  AnglePtr Electrical rotor angle (must not be NULL)
 * \param[out] OutPtr   dq output (must not be NULL)
 * \return  MATRIX_SUCCESS or error code
 */
extern MatrixStatus_T Park_Transform_Matrix(const FocAlphaBeta_T* const InPtr, const FocAngle_T* const AnglePtr, FocDq_T* const OutPtr);

/**
 * \brief   Apply the Inverse-Park transform using matrix multiplication.
 *
 * \details Formula:
 * \code
 *   [Alpha] = [cos(θ)   -sin(θ)] * [D]
 *   [Beta]    [sin(θ)    cos(θ)]   [Q]
 * \endcode
 *
 * \param[in]  InPtr    dq input (must not be NULL)
 * \param[in]  AnglePtr Electrical rotor angle (must not be NULL)
 * \param[out] OutPtr   αβ output (must not be NULL)
 * \return  MATRIX_SUCCESS or error code
 */
extern MatrixStatus_T InvPark_Transform_Matrix(const FocDq_T* const InPtr, const FocAngle_T* const AnglePtr, FocAlphaBeta_T* const OutPtr);

/**
 * \brief   Apply the Inverse-Clarke transform using matrix multiplication.
 *
 * \details Formula:
 * \code
 *   [U]   [1        0      ]   [Alpha]
 *   [V] = [-1/2     √3/2] * [Beta]
 *   [W]   [-1/2    -√3/2]
 * \endcode
 *
 * \param[in]  InPtr   αβ input (must not be NULL)
 * \param[out] OutPtr  UVW output (must not be NULL)
 * \return  MATRIX_SUCCESS or error code
 */
extern MatrixStatus_T InvClarke_Transform_Matrix(const FocAlphaBeta_T* const InPtr, FocUvw_T* const OutPtr);

#endif /* EMBED_SIM_COORDINATE_TRANSFORM_H_ */
