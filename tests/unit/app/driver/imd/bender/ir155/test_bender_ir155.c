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
 * @file    test_bender_ir155.c
 * @author  foxBMS Team
 * @date    2020-04-01 (date of creation)
 * @updated 2024-08-08 (date of last update)
 * @version v1.7.0
 * @ingroup UNIT_TEST_IMPLEMENTATION
 * @prefix  TEST
 *
 * @brief   Tests for the Bender IR155 driver
 *
 */

/*========== Includes =======================================================*/
#include "unity.h"
#include "Mockbender_ir155_helper.h"
#include "Mockdatabase.h"
#include "Mockdiag.h"
#include "Mockfram.h"
#include "Mockio.h"
#include "Mockos.h"

#include "bender_ir155_cfg.h"

#include "bender_ir155.h"
#include "imd.h"

/*========== Unit Testing Framework Directives ==============================*/
TEST_INCLUDE_PATH("../../src/app/driver/config")
TEST_INCLUDE_PATH("../../src/app/driver/fram")
TEST_INCLUDE_PATH("../../src/app/driver/imd")
TEST_INCLUDE_PATH("../../src/app/driver/imd/bender/ir155")
TEST_INCLUDE_PATH("../../src/app/driver/imd/bender/ir155/config")
TEST_INCLUDE_PATH("../../src/app/driver/io")
TEST_INCLUDE_PATH("../../src/app/driver/pwm")
TEST_INCLUDE_PATH("../../src/app/engine/diag")
TEST_INCLUDE_PATH("../../src/app/task/config")
TEST_INCLUDE_PATH("../../src/app/task/ftask")

/*========== Definitions and Implementations for Unit Test ==================*/
IR155_STATE_s ir155_state = {
    .ir155Initialized                     = false,
    .measurement.isMeasurementValid       = false,
    .measurement.isUndervoltageDetected   = false,
    .measurement.measurementState         = IR155_UNINITIALIZED,
    .measurement.measurementMode          = IR155_UNKNOWN,
    .measurement.digitalStatusPin         = STD_PIN_LOW,
    .measurement.resistance_kOhm          = 0,
    .measurement.pwmSignal.dutyCycle_perc = 0.0f,
    .measurement.pwmSignal.frequency_Hz   = 0.0f,
    .periodTriggerTime_ms                 = IMD_PERIODIC_CALL_TIME_ms,
};

FRAM_INSULATION_FLAG_s fram_insulationFlags = {.groundErrorDetected = false};

/*========== Setup and Teardown =============================================*/
void setUp(void) {
}

void tearDown(void) {
}

/*========== Test Cases =====================================================*/
void testIMD_ProcessInitializationState(void) {
    IR155_Initialize_Expect(IMD_PERIODIC_CALL_TIME_ms);
    TEST_ASSERT_EQUAL(IMD_FSM_STATE_IMD_ENABLE, IMD_ProcessInitializationState());
}
void test_IMD_ProcessEnableState(void) {
    IO_PinSet_Expect(&IR155_SUPPLY_ENABLE_PORT->DOUT, IR155_SUPPLY_ENABLE_PIN);
    TEST_ASSERT_EQUAL(IMD_FSM_STATE_RUNNING, IMD_ProcessEnableState());
}

void test_IMD_ProcessShutdownState(void) {
    IO_PinReset_Expect(&IR155_SUPPLY_ENABLE_PORT->DOUT, IR155_SUPPLY_ENABLE_PIN);
    TEST_ASSERT_EQUAL(IMD_FSM_STATE_IMD_ENABLE, IMD_ProcessShutdownState());
}
