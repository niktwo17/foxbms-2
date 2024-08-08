/**
 *
 * @copyright &copy; 2010 - 2024, Fraunhofer-Gesellschaft zur Foerderung der angewandten Forschung e.V.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * We kindly request you to use one or more of the following phrases to refer to
 * foxBMS in your hardware, software, documentation or advertising materials:
 *
 * - "This product uses parts of foxBMS&reg;"
 * - "This product includes parts of foxBMS&reg;"
 * - "This product is derived from foxBMS&reg;"
 *
 */

/**
 * @file    c-024.c
 * @author  foxBMS Team
 * @date    2021-06-04 (date of creation)
 * @updated 2024-08-08 (date of last update)
 * @version v1.7.0
 * @ingroup GUIDELINES
 * @prefix  ABC
 *
 * @brief   Example code to show the application of the C coding guidelines
 * @details This code implements an example for C:024
 */

/*========== Includes =======================================================*/
#include <stdint.h>

/*========== Macros and Definitions =========================================*/
/* Remapping of types */
typedef uint8_t abc_mySpecialType;
typedef uint32_t ABC_TICK_TYPE_t;

/* Using typedefs for the creation of function pointers */
/* Pointer to functions with uint32_t return datatype and two function parameters of type uint32_t */
typedef uint32_t (*ABC_CALLBACK_FUNCTION_POINTER_t)(uint32_t, uint32_t);

/*========== Static Constant and Variable Definitions =======================*/
static abc_mySpecialType a = 0;

/*========== Extern Constant and Variable Definitions =======================*/

/*========== Static Function Prototypes =====================================*/
static void ABC_CallTheCallback(void);
static uint32_t ABC_BuildSum(uint32_t varA, uint32_t varB); /* Function prototype */

/* local variable of type CALLBACK_FUNCTION_POINTER_t
   The variable can not be initialized in the section where all other static
   variables are introduced, since the function is not known at that time. */
static ABC_CALLBACK_FUNCTION_POINTER_t abc_callback = &ABC_BuildSum;

/*========== Static Function Implementations ================================*/
/* implement the prototype so that we have valid code */
static uint32_t ABC_BuildSum(uint32_t varA, uint32_t varB) {
    /* just an example: */
    return varA + varB;
}

/* call of function ABC_BuildSum()*/
static void ABC_CallTheCallback(void) {
    uint32_t temp = abc_callback(37u, 42u);
}

/*========== Extern Function Implementations ================================*/

/*========== Externalized Static Function Implementations (Unit Test) =======*/
#ifdef UNITY_UNIT_TEST
#endif
