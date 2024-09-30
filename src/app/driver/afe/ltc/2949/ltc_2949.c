/*========== Includes =======================================================*/
/* clang-format off */
#include "ltc_6813-1_cfg.h"
#include "ltc.h"
/* clang-format on */

#include "HL_spi.h"
#include "HL_system.h"

#include "afe_plausibility.h"
#include "database.h"
#include "diag.h"
#include "io.h"
#include "ltc_pec.h"
#include "os.h"
#include "pex.h"

#include <stdbool.h>
#include <stdint.h>

/**
 * TODO: TO BE CONFIRMED!!! time for the second initialization of the LTC
 */
#define LTC_MCU_SECOND_INITIALIZATION_TIME (LTC_TREADY_US / 1000)

/**
 * TODO: TO BE CONFIRMED!!! time for the first initialization of the LTC
 */
#define LTC_MCU_FIRST_INITIALIZATION_TIME (LTC_TWAKE_US / 1000)

#define MCU_SM_SHORTTIME (1)

LTC_MCU_STATEMACH_e ltc_mcu_state = MCU_SM_INIT;

static uint16_t ltc_cmdRDCVA[4] = {0x00, 0x04, 0x07, 0xC2};
static uint16_t ltc_cmdRDCVB[4] = {0x00, 0x06, 0x9A, 0x94};
static uint16_t ltc_cmdRDCVC[4] = {0x00, 0x08, 0x5E, 0x52};

/* Cells */
static uint16_t ltc_cmdADCV_fast_DCP0[4] =
    {0x02, 0xE0, 0x38, 0x06}; /*!< All cells, fast mode, discharge not permitted (DCP=0)      */
static uint16_t ltc_cmdADCV_normal_DCP0[4] =
    {0x03, 0x60, 0xF4, 0x6C}; /*!< All cells, normal mode, discharge not permitted (DCP=0)    */
static uint16_t ltc_cmdADCV_filtered_DCP0[4] =
    {0x03, 0xE0, 0xB0, 0x4A}; /*!< All cells, filtered mode, discharge not permitted (DCP=0)  */

static uint16_t command[4] = {0};
//static uint16_t ltc_dcmd0[8] = {0};

static STD_RETURN_TYPE_e LTC_Init(
    SPI_INTERFACE_CONFIG_s *pSpiInterface,
    uint16_t *pTxBuff,
    uint16_t *pRxBuff,
    uint32_t frameLength);
static STD_RETURN_TYPE_e LTC_CheckPec(
    LTC_MCU_STATE_s *ltc_state,
    uint16_t *DataBufferSPI_RX_with_PEC,
    uint8_t stringNumber);
static STD_RETURN_TYPE_e LTC_ReadRegister(
    uint16_t *Command,
    SPI_INTERFACE_CONFIG_s *pSpiInterface,
    uint16_t *pTxBuff,
    uint16_t *pRxBuff,
    uint32_t frameLength);
/*static STD_RETURN_TYPE_e LTC_WriteRegister(
    uint16_t *Command,
    SPI_INTERFACE_CONFIG_s *pSpiInterface,
    uint16_t *pTxBuff,
    uint16_t *pRxBuff,
    uint32_t frameLength);
    */

static void LTC_ResetErrorTable(LTC_MCU_STATE_s *ltc_state);
static void LTC_InitializeDatabase(LTC_MCU_STATE_s *ltc_state);
static void LTC_SaveLastStates(LTC_MCU_STATE_s *ltc_state);
static void LTC_StateTransition(
    LTC_MCU_STATE_s *ltc_state,
    LTC_MCU_STATEMACH_e state,
    uint8_t substate,
    uint16_t timer_ms);
static void LTC_CondBasedStateTransition(
    LTC_MCU_STATE_s *ltc_state,
    STD_RETURN_TYPE_e retVal,
    DIAG_ID_e diagCode,
    LTC_MCU_STATEMACH_e state_ok,
    uint8_t substate_ok,
    uint16_t timer_ms_ok,
    LTC_MCU_STATEMACH_e state_nok,
    uint8_t substate_nok,
    uint16_t timer_ms_nok);

static void LTC_SaveRxToVoltageBuffer(LTC_MCU_STATE_s *ltc_state, uint16_t *pRxBuff, uint8_t stringNumber);
static void LTC_SaveRxToCurrentBuffer(LTC_MCU_STATE_s *ltc_state, uint16_t *pRxBuff, uint8_t stringNumber);
static void LTC_SaveRxToTempBuffer(LTC_MCU_STATE_s *ltc_state, uint16_t *pRxBuff, uint8_t stringNumber);
static uint16_t LTC_GetMeasurementTimeCycle(LTC_ADCMODE_e adcMode);
static STD_RETURN_TYPE_e LTC_StartVoltageMeasurement(SPI_INTERFACE_CONFIG_s *pSpiInterface, LTC_ADCMODE_e adcMode);
static STD_RETURN_TYPE_e LTC_StartCurrentMeasurement(SPI_INTERFACE_CONFIG_s *pSpiInterface, LTC_ADCMODE_e adcMode);
static STD_RETURN_TYPE_e LTC_StartTempMeasurement(SPI_INTERFACE_CONFIG_s *pSpiInterface, LTC_ADCMODE_e adcMode);

void LTC_MCU_Trigger(LTC_MCU_STATE_s *ltc_mcu_state);
uint8_t LTC_MCU_CheckReEntrance(LTC_MCU_STATE_s *ltc_state);

static LTC_RETURN_TYPE_e LTC_MCU_CheckStateRequest(LTC_MCU_STATE_s *ltc_state, LTC_REQUEST_s statereq);
static LTC_REQUEST_s LTC_MCU_TransferStateRequest(LTC_MCU_STATE_s *ltc_state);
static void LTC_SetTransferTimes(LTC_MCU_STATE_s *ltc_state);
static uint32_t LTC_GetSpiClock(SPI_INTERFACE_CONFIG_s *pSpiInterface);

static DATA_BLOCK_MCU_s ltc_currentSensor  = {.header.uniqueId = DATA_BLOCK_ID_MCU};
static LTC_MCU_ERRORTABLE_s ltc_errorTable = {0}; /*!< init in LTC_ResetErrorTable-function */

/**
 * PEC buffer for RX and TX
 * @{
 */
/* AXIVION Disable Style MisraC2012-1.2: The Pec buffer must be put in the shared RAM section for performance reasons */
#pragma SET_DATA_SECTION(".sharedRAM")
uint16_t ltc_mcu_RxPecBuffer[MCU_N_BYTES_FOR_DATA_TRANSMISSION] = {0};
uint16_t ltc_mcu_TxPecBuffer[MCU_N_BYTES_FOR_DATA_TRANSMISSION] = {0};
uint16_t ltc_mcu_Rx_RDCV[20]                                    = {0};
#pragma SET_DATA_SECTION()
/* AXIVION Enable Style MisraC2012-1.2: only Pec buffer needed to be in the shared RAM section */
/**@}*/

/* Commands */
//static uint16_t ltc_cmdWRCFG[4] = {0x00, 0x01, 0x3D, 0x6E};

LTC_MCU_STATE_s ltc_mcu_stateBase = {
    .timer                   = 0,
    .statereq                = {.request = LTC_STATE_NO_REQUEST, .string = 0xFFu},
    .state                   = MCU_SM_INIT,
    .substate                = INIT_START,
    .laststate               = MCU_SM_INIT,
    .lastsubstate            = INIT_START,
    .adcModereq              = LTC_ADCMODE_FAST_DCP0,
    .adcMode                 = LTC_ADCMODE_FAST_DCP0,
    .adcMeasChreq            = LTC_ADCMEAS_UNDEFINED,
    .adcMeasCh               = LTC_ADCMEAS_UNDEFINED,
    .numberOfMeasuredMux     = 32,
    .triggerentry            = 0,
    .ErrRetryCounter         = 0,
    .ErrRequestCounter       = 0,
    .VoltageSampleTime       = 0,
    .CurrentSampleTime       = 0,
    .commandDataTransferTime = 3,
    .commandTransferTime     = 3,
    .gpioClocksTransferTime  = 3,
    .first_measurement_made  = false,
    .check_spi_flag          = STD_NOT_OK,
    .transmit_ongoing        = false,
    .dummyByte_ongoing       = STD_NOT_OK,
    .spiDiagErrorEntry       = DIAG_ID_AFE_SPI,
    .pecDiagErrorEntry       = DIAG_ID_AFE_COM_INTEGRITY,
    .voltMeasDiagErrorEntry  = DIAG_ID_AFE_CELL_VOLTAGE_MEAS_ERROR,
    .tempMeasDiagErrorEntry  = DIAG_ID_AFE_CELL_TEMPERATURE_MEAS_ERROR,
    .ltcData.pSpiInterface   = spi_ltcInterface,
    .ltcData.txBuffer        = ltc_mcu_TxPecBuffer,
    .ltcData.rxBuffer        = ltc_mcu_Rx_RDCV,
    .ltcData.frameLength     = 12,
    .ltcData.errorTable      = &ltc_errorTable,
    .ltcData.currentSensor   = &ltc_currentSensor,
    .currentString           = 0u,
    .requestedString         = 0u,
};

void LTC_MCU_Trigger(LTC_MCU_STATE_s *ltc_state) {
    FAS_ASSERT(ltc_state != NULL_PTR);
    STD_RETURN_TYPE_e retVal           = STD_OK;
    STD_RETURN_TYPE_e continueFunction = STD_OK;
    LTC_REQUEST_s statereq             = {.request = LTC_STATE_NO_REQUEST, .string = 0x0u};
    FAS_ASSERT(ltc_state != NULL_PTR);

    /* Check re-entrance of function */
    if (LTC_MCU_CheckReEntrance(ltc_state) > 0u) {
        continueFunction = STD_NOT_OK;
    }

    if (ltc_state->check_spi_flag == STD_NOT_OK) {
        if (ltc_state->timer > 0u) {
            if ((--ltc_state->timer) > 0u) {
                ltc_state->triggerentry--;
                continueFunction = STD_NOT_OK; /* handle state machine only if timer has elapsed */
            }
        }
    } else {
        if (MCU_IsTransmitOngoing(ltc_state) == true) {
            if (ltc_state->timer > 0u) {
                if ((--ltc_state->timer) > 0u) {
                    ltc_state->triggerentry--;
                    continueFunction = STD_NOT_OK; /* handle state machine only if timer has elapsed */
                }
            }
        }
    }
    if (continueFunction == STD_OK) {
        switch (ltc_state->state) {
                /****************************UNINITIALIZED***********************************/
            case MCU_SM_INIT:
                if (ltc_state->substate == INIT_START) {
                    statereq = LTC_MCU_TransferStateRequest(ltc_state);
                    if ((statereq.request == LTC_STATE_INIT_REQUEST) || (statereq.request == LTC_STATE_NO_REQUEST)) {
                        LTC_SaveLastStates(ltc_state);
                        LTC_InitializeDatabase(ltc_state);
                        LTC_ResetErrorTable(ltc_state);
                        LTC_StateTransition(ltc_state, MCU_SM_INIT, INIT_STRING, MCU_SM_SHORTTIME);
                        retVal = STD_OK;
                    } else if (statereq.request == LTC_STATE_NO_REQUEST) {
                        /* no actual request pending */
                    } else {
                        ltc_state->ErrRequestCounter++; /* illegal request pending */
                    }
                } else if (ltc_state->substate == INIT_STRING) {
                    LTC_SaveLastStates(ltc_state);
                    // this logic is taken from the LTC6813-1.c, but we start at the last LTC string and only iterate over the MCU strings
                    // Check for consistency in definitions here!!!
                    ltc_state->currentString       = 0;
                    ltc_state->spiSeqPtr           = ltc_state->ltcData.pSpiInterface + BS_NR_OF_TOTAL_STRINGS;
                    ltc_state->spiNumberInterfaces = BS_NR_OF_MCU;
                    ltc_state->spiSeqEndPtr = ltc_state->ltcData.pSpiInterface + BS_NR_OF_MCU + BS_NR_OF_TOTAL_STRINGS;
                    LTC_StateTransition(ltc_state, MCU_SM_INIT, INIT_ENTRY, MCU_SM_SHORTTIME);
                } else if (ltc_state->substate == INIT_ENTRY) {
                    LTC_SaveLastStates(ltc_state);
                    LTC_SetTransferTimes(ltc_state);
                    retVal =
                        LTC_TRANSMIT_WAKE_UP(ltc_state->spiSeqPtr); /* Send dummy byte to wake up the daisy chain */
                    LTC_CondBasedStateTransition(
                        ltc_state,
                        retVal,
                        ltc_state->spiDiagErrorEntry,
                        MCU_SM_INIT,
                        INIT_RE_ENTRY,
                        LTC_MCU_FIRST_INITIALIZATION_TIME,
                        MCU_SM_INIT,
                        INIT_ENTRY,
                        MCU_SM_SHORTTIME);
                } else if (ltc_state->substate == INIT_RE_ENTRY) {
                    LTC_SaveLastStates(ltc_state);
                    retVal = LTC_TRANSMIT_WAKE_UP(
                        ltc_state->spiSeqPtr); /* Send dummy byte again to wake up the daisy chain */
                    LTC_CondBasedStateTransition(
                        ltc_state,
                        retVal,
                        ltc_state->spiDiagErrorEntry,
                        MCU_SM_INIT,
                        INIT_MAIN,
                        LTC_MCU_SECOND_INITIALIZATION_TIME,
                        MCU_SM_INIT,
                        INIT_RE_ENTRY,
                        MCU_SM_SHORTTIME);

                } else if (ltc_state->substate == INIT_MAIN) {
                    LTC_SaveLastStates(ltc_state);
                    ltc_state->check_spi_flag = STD_OK;
                    MCU_SetTransmitOngoing(ltc_state);
                    retVal = LTC_Init(
                        ltc_state->spiSeqPtr,
                        ltc_state->ltcData.txBuffer,
                        ltc_state->ltcData.rxBuffer,
                        20); /* Initialize main LTC loop */
                    ltc_state->lastsubstate = ltc_state->substate;
                    DIAG_CheckEvent(retVal, ltc_state->spiDiagErrorEntry, DIAG_STRING, ltc_state->currentString);
                    LTC_StateTransition(
                        ltc_state,
                        MCU_SM_INIT,
                        INIT_EXIT,
                        ltc_state->commandDataTransferTime + LTC_TRANSMISSION_TIMEOUT);
                } else if (ltc_state->substate == INIT_EXIT) {
                    LTC_SaveLastStates(ltc_state);
                    ++ltc_state->spiSeqPtr;
                    ++ltc_state->currentString;
                    if (ltc_state->spiSeqPtr >= ltc_state->spiSeqEndPtr) {
                        LTC_StateTransition(ltc_state, MCU_SM_MEAS_ENTRY, MEAS_VOLT, MCU_SM_SHORTTIME);
                    } else {
                        LTC_StateTransition(ltc_state, MCU_SM_INIT, INIT_ENTRY, MCU_SM_SHORTTIME);
                    }
                }
                /* waiting for Initialization Request */

                break;
                /****************************MEASUREMENT**********************************/
            case MCU_SM_MEAS_ENTRY:
                // no saving last states anymore???
                ltc_state->adcMode             = LTC_VOLTAGE_MEASUREMENT_MODE;
                ltc_state->spiSeqPtr           = ltc_state->ltcData.pSpiInterface + BS_NR_OF_TOTAL_STRINGS;
                ltc_state->spiNumberInterfaces = BS_NR_OF_MCU;
                ltc_state->spiSeqEndPtr  = ltc_state->ltcData.pSpiInterface + BS_NR_OF_MCU + BS_NR_OF_TOTAL_STRINGS;
                ltc_state->currentString = 0;

                //ltc_state->check_spi_flag = STD_NOT_OK;
                retVal = STD_OK;
                LTC_StateTransition(ltc_state, MCU_SM_MEAS_REQ, MEAS_VOLT, MCU_SM_SHORTTIME);
                break;

            case MCU_SM_MEAS_REQ:
                ltc_state->check_spi_flag = STD_NOT_OK;
                if (ltc_state->substate == MEAS_VOLT) {
                    retVal = LTC_StartVoltageMeasurement(ltc_state->spiSeqPtr, ltc_state->adcMode);
                } else if (ltc_state->substate == MEAS_TEMP) {
                    retVal = LTC_StartTempMeasurement(ltc_state->spiSeqPtr, ltc_state->adcMode);
                } else if (ltc_state->substate == MEAS_CURRENT) {
                    retVal = LTC_StartCurrentMeasurement(ltc_state->spiSeqPtr, ltc_state->adcMode);
                } else {
                    retVal = STD_NOT_OK;
                    break;
                }
                LTC_CondBasedStateTransition(
                    ltc_state,
                    retVal,
                    ltc_state->spiDiagErrorEntry,
                    MCU_SM_MEAS_READ,
                    ltc_state->substate,
                    (ltc_state->commandTransferTime + LTC_GetMeasurementTimeCycle(ltc_state->adcMode)),
                    MCU_SM_MEAS_READ,
                    ltc_state->substate,
                    MCU_SM_SHORTTIME);
                break;

            case MCU_SM_MEAS_READ:
                ltc_state->check_spi_flag = STD_OK;

                if (ltc_state->substate == MEAS_VOLT) {
                    for (uint8_t i = 0; i < 4; i++) {
                        command[i] = ltc_cmdRDCVA[i];
                    }
                } else if (ltc_state->substate == MEAS_TEMP) {
                    for (uint8_t i = 0; i < 4; i++) {
                        command[i] = ltc_cmdRDCVB[i];
                    }
                } else if (ltc_state->substate == MEAS_CURRENT) {
                    for (uint8_t i = 0; i < 4; i++) {
                        command[i] = ltc_cmdRDCVC[i];
                    }
                } else {
                    retVal                    = STD_NOT_OK;
                    ltc_state->check_spi_flag = STD_NOT_OK;
                    break;
                }
                ltc_state->ltcData.frameLength = 20;  //total package of data
                MCU_SetTransmitOngoing(ltc_state);
                retVal = LTC_ReadRegister(
                    command,
                    ltc_state->spiSeqPtr,
                    ltc_state->ltcData.txBuffer,
                    ltc_state->ltcData.rxBuffer,
                    ltc_state->ltcData.frameLength);
                LTC_CondBasedStateTransition(
                    ltc_state,
                    retVal,
                    ltc_state->spiDiagErrorEntry,
                    MCU_SM_MEAS_SAVE,
                    ltc_state->substate,
                    (ltc_state->commandDataTransferTime + LTC_TRANSMISSION_TIMEOUT),
                    MCU_SM_MEAS_SAVE,
                    ltc_state->substate,
                    LTC_STATEMACH_SHORTTIME);
                ltc_state->check_spi_flag = STD_NOT_OK;
                break;

            case MCU_SM_MEAS_SAVE:
                ltc_state->check_spi_flag = STD_OK;
                retVal = LTC_CheckPec(ltc_state, ltc_state->ltcData.rxBuffer, ltc_state->currentString);
                DIAG_CheckEvent(retVal, ltc_state->pecDiagErrorEntry, DIAG_STRING, ltc_state->currentString);
                if (ltc_state->substate == MEAS_VOLT) {
                    LTC_SaveRxToVoltageBuffer(ltc_state, ltc_state->ltcData.rxBuffer, ltc_state->currentString);
                } else if (ltc_state->substate == MEAS_TEMP) {
                    LTC_SaveRxToTempBuffer(ltc_state, ltc_state->ltcData.rxBuffer, ltc_state->currentString);
                } else if (ltc_state->substate == MEAS_CURRENT) {
                    LTC_SaveRxToCurrentBuffer(ltc_state, ltc_state->ltcData.rxBuffer, ltc_state->currentString);
                } else {
                    retVal = STD_NOT_OK;
                    break;
                }
                LTC_StateTransition(ltc_state, MCU_SM_MEAS_FINISH, ltc_state->substate, MCU_SM_SHORTTIME);
                ltc_state->check_spi_flag = STD_NOT_OK;
                break;

            case MCU_SM_MEAS_FINISH:
                if (ltc_state->substate == MEAS_VOLT) {
                    //ltc_state->adcMode = ;
                    LTC_StateTransition(ltc_state, MCU_SM_MEAS_REQ, MEAS_CURRENT, MCU_SM_SHORTTIME);
                } else if (ltc_state->substate == MEAS_TEMP) {
                    //ltc_state->adcMode = ;
                    LTC_StateTransition(ltc_state, MCU_SM_MEAS_REQ, MEAS_CURRENT, MCU_SM_SHORTTIME);
                } else if (ltc_state->substate == MEAS_CURRENT) {
                    //ltc_state->adcMode = ;
                    ++ltc_state->spiSeqPtr;
                    ++ltc_state->currentString;
                    if (ltc_state->spiSeqPtr >= ltc_state->spiSeqEndPtr) {
                        LTC_StateTransition(
                            ltc_state,
                            MCU_SM_MEAS_ENTRY,
                            MEAS_VOLT,
                            MCU_SM_SHORTTIME);  //this will reset the counter
                    } else {
                        LTC_StateTransition(ltc_state, MCU_SM_MEAS_REQ, MEAS_VOLT, MCU_SM_SHORTTIME);
                    }
                } else {
                    retVal = STD_NOT_OK;
                    break;
                }
                break;

            default:
                retVal = STD_NOT_OK;
                break;
        }
        ltc_state->triggerentry--; /* reentrance counter */
    }
}

/**
 * @brief   sets the transfer time needed to receive/send data with the LTC daisy-chain.
 *
 * This function gets the clock frequency and uses the number of LTCs in the daisy-chain.
 *
 * @param  ltc_state:  state of the ltc state machine
 *
 */
static void LTC_SetTransferTimes(LTC_MCU_STATE_s *ltc_state) {
    FAS_ASSERT(ltc_state != NULL_PTR);
    uint32_t transferTime_us = 0;
    uint32_t SPI_Clock       = 0;

    SPI_Clock = LTC_GetSpiClock(ltc_state->ltcData.pSpiInterface);

    /* Transmission of a command and data */
    /* Multiplication by 1000*1000 to get us */
    transferTime_us = (8u * 1000u * 1000u) / (SPI_Clock);
    transferTime_us *= MCU_N_BYTES_FOR_DATA_TRANSMISSION;
    transferTime_us                    = transferTime_us + LTC_SPI_WAKEUP_WAIT_TIME_US;
    ltc_state->commandDataTransferTime = (transferTime_us / 1000u) + 1u;

    /* Transmission of a command */
    /* Multiplication by 1000*1000 to get us */
    transferTime_us                = ((4u) * 8u * 1000u * 1000u) / (SPI_Clock);
    transferTime_us                = transferTime_us + LTC_SPI_WAKEUP_WAIT_TIME_US;
    ltc_state->commandTransferTime = (transferTime_us / 1000u) + 1u;

    /* Transmission of a command + 9 clocks */
    /* Multiplication by 1000*1000 to get us */
    transferTime_us                   = ((4u + 9u) * 8u * 1000u * 1000u) / (SPI_Clock);
    transferTime_us                   = transferTime_us + LTC_SPI_WAKEUP_WAIT_TIME_US;
    ltc_state->gpioClocksTransferTime = (transferTime_us / 1000u) + 1u;
}

static uint32_t LTC_GetSpiClock(SPI_INTERFACE_CONFIG_s *pSpiInterface) {
    FAS_ASSERT(pSpiInterface != NULL_PTR);
    uint32_t SPI_Clock = 0;
    uint32_t prescaler = 0;
    /* Get SPI prescaler */
    prescaler = ((pSpiInterface->pNode->FMT0) >> 8u) & 0xFFu;
    SPI_Clock = (uint32_t)(AVCLK1_FREQ * 1000000u) / (prescaler + 1u);
    return SPI_Clock;
}

static LTC_REQUEST_s LTC_MCU_TransferStateRequest(LTC_MCU_STATE_s *ltc_state) {
    FAS_ASSERT(ltc_state != NULL_PTR);
    LTC_REQUEST_s retval = {.request = LTC_STATE_NO_REQUEST, .string = 0x0u};

    OS_EnterTaskCritical();
    retval.request              = ltc_state->statereq.request;
    retval.string               = ltc_state->statereq.string;
    ltc_state->requestedString  = ltc_state->statereq.string;
    ltc_state->statereq.request = LTC_STATE_NO_REQUEST;
    ltc_state->statereq.string  = 0x0u;
    OS_ExitTaskCritical();

    return (retval);
}

LTC_RETURN_TYPE_e LTC_MCU_SetStateRequest(LTC_MCU_STATE_s *ltc_state, LTC_REQUEST_s statereq) {
    FAS_ASSERT(ltc_state != NULL_PTR);
    LTC_RETURN_TYPE_e retVal = LTC_ERROR;

    OS_EnterTaskCritical();
    retVal = LTC_MCU_CheckStateRequest(ltc_state, statereq);

    if ((retVal == LTC_OK) || (retVal == LTC_BUSY_OK) || (retVal == LTC_OK_FROM_ERROR)) {
        ltc_state->statereq = statereq;
    }
    OS_ExitTaskCritical();

    return (retVal);
}

/**@brief checks the state requests that are made.**This function checks the validity of the state
     requests.*The results of the checked is returned immediately.**@param ltc_state
    : state of the ltc state machine *@param statereq state request to be checked **@ return result of the state request
          that was made,
    taken from LTC_RETURN_TYPE_e */

static LTC_RETURN_TYPE_e LTC_MCU_CheckStateRequest(LTC_MCU_STATE_s *ltc_state, LTC_REQUEST_s statereq) {
    FAS_ASSERT(ltc_state != NULL_PTR);
    LTC_RETURN_TYPE_e retVal = LTC_OK;
    if (statereq.string >= BS_NR_OF_MCU) {
        retVal = LTC_ILLEGAL_REQUEST;
    } else if (ltc_state->statereq.request == LTC_STATE_NO_REQUEST) {
        /* init only allowed from the uninitialized state */
        if (statereq.request == LTC_STATE_INIT_REQUEST) {
            if (ltc_state->state == MCU_SM_INIT) {
                retVal = LTC_OK;
            } else {
                retVal = LTC_ALREADY_INITIALIZED;
            }
        } else {
            retVal = LTC_OK;
        }
    } else {
        retVal = LTC_REQUEST_PENDING;
    }

    return retVal;
}

/**
 * @brief   re-entrance check of LTC state machine trigger function
 *
 * This function is not re-entrant and should only be called time- or event-triggered.
 * It increments the triggerentry counter from the state variable ltc_state.
 * It should never be called by two different processes, so if it is the case, triggerentry
 * should never be higher than 0 when this function is called.
 *
 * @param  ltc_state:  state of the ltc state machine
 *
 * @return  retval  0 if no further instance of the function is active, 0xff else
 *
 */
uint8_t LTC_MCU_CheckReEntrance(LTC_MCU_STATE_s *ltc_state) {
    FAS_ASSERT(ltc_state != NULL_PTR);
    uint8_t retval = 0;

    OS_EnterTaskCritical();
    if (!ltc_state->triggerentry) {
        ltc_state->triggerentry++;
    } else {
        retval = 0xFF; /* multiple calls of function */
    }
    OS_ExitTaskCritical();

    return (retval);
}

/**
 * @brief   in the database, initializes the fields related to the LTC drivers.
 *
 * This function loops through all the LTC-related data fields in the database
 * and sets them to 0. It should be called in the initialization or re-initialization
 * routine of the LTC driver.
 *
 * @param  ltc_state:  state of the ltc state machine
 *
 */
static void LTC_InitializeDatabase(LTC_MCU_STATE_s *ltc_state) {
    for (uint8_t s = 0u; s < BS_NR_OF_MCU; s++) {
        ltc_state->ltcData.currentSensor->current_mA[s]                          = 0;
        ltc_state->ltcData.currentSensor->invalidCurrentMeasurement[s]           = 0;
        ltc_state->ltcData.currentSensor->newCurrent                             = 0;
        ltc_state->ltcData.currentSensor->previousTimestampCurrent[s]            = 0;
        ltc_state->ltcData.currentSensor->timestampCurrent[s]                    = 0;
        ltc_state->ltcData.currentSensor->sensorTemperature_ddegC[s]             = 0;
        ltc_state->ltcData.currentSensor->invalidSensorTemperatureMeasurement[s] = 0;
        ltc_state->ltcData.currentSensor->power_W[s]                             = 0;
        ltc_state->ltcData.currentSensor->newPower                               = 0;
        ltc_state->ltcData.currentSensor->previousTimestampPower[s]              = 0;
        ltc_state->ltcData.currentSensor->timestampPower[s]                      = 0;
        ltc_state->ltcData.currentSensor->currentCounter_As[s]                   = 0;
        ltc_state->ltcData.currentSensor->invalidCurrentCountingMeasurement[s]   = 0;
        ltc_state->ltcData.currentSensor->previousTimestampCurrentCounting[s]    = 0;
        ltc_state->ltcData.currentSensor->timestampCurrentCounting[s]            = 0;
        ltc_state->ltcData.currentSensor->energyCounter_Wh[s]                    = 0;
        ltc_state->ltcData.currentSensor->invalidEnergyCountingMeasurement[s]    = 0;
        ltc_state->ltcData.currentSensor->previousTimestampEnergyCounting[s]     = 0;
        ltc_state->ltcData.currentSensor->timestampEnergyCounting[s]             = 0;
        ltc_state->ltcData.currentSensor->invalidHighVoltageMeasurement[s]       = 0;
        ltc_state->ltcData.currentSensor->highVoltage_mV[s]                      = 0;
        ltc_state->ltcData.currentSensor->previousTimestampHighVoltage[s]        = 0;
        ltc_state->ltcData.currentSensor->timestampHighVoltage[s]                = 0;
    }
    DATA_WRITE_DATA(ltc_state->ltcData.currentSensor);
}

/**
 * @brief   resets the error table.
 *
 * This function should be called during initialization or before starting a new measurement cycle
 *
 * @param  ltc_state:  state of the ltc state machine
 *
 */
static void LTC_ResetErrorTable(LTC_MCU_STATE_s *ltc_state) {
    FAS_ASSERT(ltc_state != NULL_PTR);
    for (uint8_t s = 0u; s < BS_NR_OF_MCU; s++) {
        ltc_state->ltcData.errorTable->PEC_valid[s] = false;
    }
}

/**
 * @brief   brief missing
 * @details Gets the measurement time needed by the LTC analog front-end,
 *          depending on the measurement mode
 * @param   adcMode     LTC ADCmeasurement mode (fast, normal or filtered)
 * @return  measurement time in ms
 */
static uint16_t LTC_GetMeasurementTimeCycle(LTC_ADCMODE_e adcMode) {
    uint16_t retVal = LTC_ADCMEAS_UNDEFINED; /* default */

    if ((adcMode == LTC_ADCMODE_FAST_DCP0) || (adcMode == LTC_ADCMODE_FAST_DCP1)) {
        retVal = LTC_STATEMACH_MEAS_ALL_CELLS_FAST_TCYCLE;
    } else if ((adcMode == LTC_ADCMODE_NORMAL_DCP0) || (adcMode == LTC_ADCMODE_NORMAL_DCP1)) {
        retVal = LTC_STATEMACH_MEAS_ALL_CELLS_NORMAL_TCYCLE;
    } else if ((adcMode == LTC_ADCMODE_FILTERED_DCP0) || (adcMode == LTC_ADCMODE_FILTERED_DCP1)) {
        retVal = LTC_STATEMACH_MEAS_ALL_CELLS_FILTERED_TCYCLE;
    }
    //TODO: Implement this function!!!
    return retVal;
}

/**
 * @brief   saves the voltage values read from the LTC.
 *
 * After a voltage measurement was initiated to measure the voltages of the cells,
 * the result is read via SPI.
 * This function is called to store the result from the transmission in a buffer.
 *
 * @param   ltc_state      state of the ltc state machine
 * @param   pRxBuff        receive buffer
 * @param  stringNumber    string addressed
 *
 */
static void LTC_SaveRxToVoltageBuffer(LTC_MCU_STATE_s *ltc_state, uint16_t *pRxBuff, uint8_t stringNumber) {
    FAS_ASSERT(ltc_state != NULL_PTR);
    FAS_ASSERT(pRxBuff != NULL_PTR);
    uint16_t val_ui       = 0;
    uint16_t voltage      = 0;
    uint16_t buffer_LSB   = 0;
    uint16_t buffer_MSB   = 0;
    bool continueFunction = true;

    if (continueFunction == true) {
        /* Retrieve data without command and CRC*/
        /* parse all three voltages (3 * 2bytes) contained in one register */
        // DEPENDS on amount of data in register...
        for (uint8_t c = 0u; c < 3u; c++) {
            /* index considering maximum number of cells */
            buffer_MSB = pRxBuff[4u + (2u * c) + 1u];
            buffer_LSB = pRxBuff[4u + (2u * c)];
            val_ui     = buffer_LSB | (buffer_MSB << 8u);
            /* val_ui = *((uint16_t *)(&pRxBuff[4+2*j+i*8])); */
            voltage += ((val_ui)) * 100e-6f * 1000.0f; /* Unit V -> in mV */

            /* Check PEC for every LTC
            if (ltc_state->ltcData.errorTable->PEC_valid[stringNumber] == true) {
                ltc_state->ltcData.currentSensor->invalidHighVoltageMeasurement[stringNumber] = 0;
                ltc_state->ltcData.currentSensor->highVoltage_mV[stringNumber]                = voltage;
            } else {
                // PEC_valid == false: Invalidate only flags of this voltage register
                ltc_state->ltcData.currentSensor->invalidHighVoltageMeasurement[stringNumber] = 1;
            }
            */
            ltc_state->ltcData.currentSensor->highVoltage_mV[stringNumber] = voltage;
        }
    }
}

/**
 * @brief   saves the current values read from the LTC.
 *
 * After a current measurement was initiated to measure the voltages of the cells,
 * the result is read via SPI.
 * This function is called to store the result from the transmission in a buffer.
 *
 * @param   ltc_state      state of the ltc state machine
 * @param   pRxBuff        receive buffer
 * @param  stringNumber    string addressed
 *
 */
static void LTC_SaveRxToCurrentBuffer(LTC_MCU_STATE_s *ltc_state, uint16_t *pRxBuff, uint8_t stringNumber) {
    FAS_ASSERT(ltc_state != NULL_PTR);
    FAS_ASSERT(pRxBuff != NULL_PTR);
    uint16_t val_ui       = 0;
    uint16_t current      = 0;
    uint16_t buffer_LSB   = 0;
    uint16_t buffer_MSB   = 0;
    bool continueFunction = true;

    if (continueFunction == true) {
        /* Retrieve data without command and CRC*/
        /* parse all three voltages (3 * 2bytes) contained in one register */
        // DEPENDS on amount of data in register...
        for (uint8_t c = 0u; c < 3u; c++) {
            /* index considering maximum number of cells */
            buffer_MSB = pRxBuff[4u + (2u * c) + 1u];
            buffer_LSB = pRxBuff[4u + (2u * c)];
            val_ui     = buffer_LSB | (buffer_MSB << 8u);
            /* val_ui = *((uint16_t *)(&pRxBuff[4+2*j+i*8])); */
            current += ((val_ui)) * 100e-6f * 1000.0f; /* Unit V -> in mV */

            ltc_state->ltcData.currentSensor->current_mA[stringNumber] = current;
        }
    }
}

static void LTC_SaveRxToTempBuffer(LTC_MCU_STATE_s *ltc_state, uint16_t *pRxBuff, uint8_t stringNumber) {
    FAS_ASSERT(ltc_state != NULL_PTR);
    FAS_ASSERT(pRxBuff != NULL_PTR);
    uint16_t val_ui       = 0;
    uint16_t temp         = 0;
    uint16_t buffer_LSB   = 0;
    uint16_t buffer_MSB   = 0;
    bool continueFunction = true;

    if (continueFunction == true) {
        /* Retrieve data without command and CRC*/
        /* parse all three voltages (3 * 2bytes) contained in one register */
        // DEPENDS on amount of data in register...
        for (uint8_t c = 0u; c < 3u; c++) {
            /* index considering maximum number of cells */
            buffer_MSB = pRxBuff[4u + (2u * c) + 1u];
            buffer_LSB = pRxBuff[4u + (2u * c)];
            val_ui     = buffer_LSB | (buffer_MSB << 8u);
            /* val_ui = *((uint16_t *)(&pRxBuff[4+2*j+i*8])); */
            temp += ((val_ui)) * 100e-6f * 1000.0f; /* Unit V -> in mV */

            ltc_state->ltcData.currentSensor->sensorTemperature_ddegC[stringNumber] = temp;
        }
    }
}
/**
 * @brief   tells the LTC to start measuring voltage
 *
 * This function sends an instruction via SPI, in order to start voltage measurement.
 *
 * @param   pSpiInterface        pointer to SPI configuration
 * @param   adcMode              LTC ADCmeasurement mode (fast, normal or filtered) *
 * @return  retVal      #STD_OK if dummy byte was sent correctly by SPI, #STD_NOT_OK otherwise
 *
 */
static STD_RETURN_TYPE_e LTC_StartVoltageMeasurement(SPI_INTERFACE_CONFIG_s *pSpiInterface, LTC_ADCMODE_e adcMode) {
    FAS_ASSERT(pSpiInterface != NULL_PTR);
    STD_RETURN_TYPE_e retVal = STD_OK;

    if (adcMode == LTC_ADCMODE_FAST_DCP0) {
        retVal = LTC_TRANSMIT_COMMAND(pSpiInterface, ltc_cmdADCV_fast_DCP0);
    } else if (adcMode == LTC_ADCMODE_NORMAL_DCP0) {
        retVal = LTC_TRANSMIT_COMMAND(pSpiInterface, ltc_cmdADCV_normal_DCP0);
    } else if (adcMode == LTC_ADCMODE_FILTERED_DCP0) {
        retVal = LTC_TRANSMIT_COMMAND(pSpiInterface, ltc_cmdADCV_filtered_DCP0);
    } else {
        retVal = STD_NOT_OK;
    }
    return retVal;
}

/**
 * @brief   tells the LTC to start measuring current
 *
 * This function sends an instruction via SPI, in order to start current measurement.
 *
 * @param   pSpiInterface        pointer to SPI configuration
 * @param   adcMode              LTC ADCmeasurement mode (fast, normal or filtered) *
 * @return  retVal      #STD_OK if dummy byte was sent correctly by SPI, #STD_NOT_OK otherwise
 *
 */
static STD_RETURN_TYPE_e LTC_StartCurrentMeasurement(SPI_INTERFACE_CONFIG_s *pSpiInterface, LTC_ADCMODE_e adcMode) {
    FAS_ASSERT(pSpiInterface != NULL_PTR);
    STD_RETURN_TYPE_e retVal = STD_OK;

    if (adcMode == LTC_ADCMODE_FAST_DCP0) {
        retVal = LTC_TRANSMIT_COMMAND(pSpiInterface, ltc_cmdADCV_fast_DCP0);
    } else if (adcMode == LTC_ADCMODE_NORMAL_DCP0) {
        retVal = LTC_TRANSMIT_COMMAND(pSpiInterface, ltc_cmdADCV_normal_DCP0);
    } else if (adcMode == LTC_ADCMODE_FILTERED_DCP0) {
        retVal = LTC_TRANSMIT_COMMAND(pSpiInterface, ltc_cmdADCV_filtered_DCP0);
    } else {
        retVal = STD_NOT_OK;
    }
    return retVal;
}

/**
 * @brief   tells the LTC to start measuring temperature
 *
 * This function sends an instruction via SPI, in order to start temperature measurement.
 *
 * @param   pSpiInterface        pointer to SPI configuration
 * @param   adcMode              LTC ADCmeasurement mode (fast, normal or filtered) *
 * @return  retVal      #STD_OK if dummy byte was sent correctly by SPI, #STD_NOT_OK otherwise
 *
 */
static STD_RETURN_TYPE_e LTC_StartTempMeasurement(SPI_INTERFACE_CONFIG_s *pSpiInterface, LTC_ADCMODE_e adcMode) {
    FAS_ASSERT(pSpiInterface != NULL_PTR);
    STD_RETURN_TYPE_e retVal = STD_OK;

    if (adcMode == LTC_ADCMODE_FAST_DCP0) {
        retVal = LTC_TRANSMIT_COMMAND(pSpiInterface, ltc_cmdADCV_fast_DCP0);
    } else if (adcMode == LTC_ADCMODE_NORMAL_DCP0) {
        retVal = LTC_TRANSMIT_COMMAND(pSpiInterface, ltc_cmdADCV_normal_DCP0);
    } else if (adcMode == LTC_ADCMODE_FILTERED_DCP0) {
        retVal = LTC_TRANSMIT_COMMAND(pSpiInterface, ltc_cmdADCV_filtered_DCP0);
    } else {
        retVal = STD_NOT_OK;
    }
    return retVal;
}

/**
 * @brief Saves the last state and the last substate
 *
 * @param  ltc_state:  state of the ltc state machine
 */
static void LTC_SaveLastStates(LTC_MCU_STATE_s *ltc_state) {
    ltc_state->laststate    = ltc_state->state;
    ltc_state->lastsubstate = ltc_state->substate;
}

/**
 * @brief   function for setting LTC_Trigger state transitions
 *
 * @param  ltc_state:  state of the ltc state machine
 * @param  state:      state to transition into
 * @param  substate:   substate to transition into
 * @param  timer_ms:   transition into state, substate after timer elapsed
 */
static void LTC_StateTransition(
    LTC_MCU_STATE_s *ltc_state,
    LTC_MCU_STATEMACH_e state,
    uint8_t substate,
    uint16_t timer_ms) {
    ltc_state->state    = state;
    ltc_state->substate = substate;
    ltc_state->timer    = timer_ms;
}

/**
 * @brief   condition-based state transition depending on retVal
 *
 * If retVal is #STD_OK, after timer_ms_ok is elapsed the LTC state machine will
 * transition into state_ok and substate_ok, otherwise after timer_ms_nok the
 * state machine will transition to state_nok and substate_nok. Depending on
 * value of retVal the corresponding diagnosis entry will be called.
 *
 * @param  ltc_state    state of the ltc state machine
 * @param  retVal       condition to determine if state machine will transition
 *                      into ok or nok states
 * @param  diagCode     symbolic IDs for diagnosis entry, called with
 *                      #DIAG_EVENT_OK if retVal is #STD_OK, #DIAG_EVENT_NOT_OK
 *                      otherwise
 * @param  state_ok     state to transition into if retVal is #STD_OK
 * @param  substate_ok  substate to transition into if retVal is #STD_OK
 * @param  timer_ms_ok  transition into state_ok, substate_ok after timer_ms_ok
 *                      elapsed
 * @param  state_nok    state to transition into if retVal is #STD_NOT_OK
 * @param  substate_nok substate to transition into if retVal is #STD_NOT_OK
 * @param  timer_ms_nok transition into state_nok, substate_nok after
 *                      timer_ms_nok elapsed
 */
static void LTC_CondBasedStateTransition(
    LTC_MCU_STATE_s *ltc_state,
    STD_RETURN_TYPE_e retVal,
    DIAG_ID_e diagCode,
    LTC_MCU_STATEMACH_e state_ok,
    uint8_t substate_ok,
    uint16_t timer_ms_ok,
    LTC_MCU_STATEMACH_e state_nok,
    uint8_t substate_nok,
    uint16_t timer_ms_nok) {
    if ((retVal != STD_OK)) {
        DIAG_Handler(diagCode, DIAG_EVENT_NOT_OK, DIAG_STRING, ltc_state->currentString);
        LTC_StateTransition(ltc_state, state_nok, substate_nok, timer_ms_nok);
    } else {
        DIAG_Handler(diagCode, DIAG_EVENT_OK, DIAG_STRING, ltc_state->currentString);
        LTC_StateTransition(ltc_state, state_ok, substate_ok, timer_ms_ok);
    }
}

/**
 * @brief   initialize the daisy-chain.
 *
 * To initialize the LTC6804 daisy-chain, a dummy byte (0x00) is sent.
 *
 * @param   pSpiInterface        pointer to SPI configuration
 * @param   pTxBuff              transmit buffer
 * @param   pRxBuff              receive buffer
 * @param   frameLength          number of words to transmit
 *
 * @return  retVal  #STD_OK if dummy byte was sent correctly by SPI, #STD_NOT_OK otherwise
 *
 */
static STD_RETURN_TYPE_e LTC_Init(
    SPI_INTERFACE_CONFIG_s *pSpiInterface,
    uint16_t *pTxBuff,
    uint16_t *pRxBuff,
    uint32_t frameLength) {
    FAS_ASSERT(pSpiInterface != NULL_PTR);
    FAS_ASSERT(pTxBuff != NULL_PTR);
    FAS_ASSERT(pRxBuff != NULL_PTR);
    STD_RETURN_TYPE_e retVal = STD_NOT_OK;

    uint8_t PEC_1[1];
    uint8_t PEC_2[2];
    uint16_t PEC_result = 0;

    /* now construct the message to be sent: it contains the wanted data, PLUS the needed PECs */
    pTxBuff[0]  = 0xFE;
    pTxBuff[1]  = 0xF5;
    pTxBuff[4]  = 0x40;
    pTxBuff[5]  = 0x08;
    PEC_2[0]    = pTxBuff[0];
    PEC_2[1]    = pTxBuff[1];
    PEC_1[0]    = pTxBuff[5];
    PEC_result  = LTC_CalculatePec15(2, PEC_2);
    pTxBuff[2]  = (PEC_result >> 8u) & 0xFFu;
    pTxBuff[3]  = PEC_result & 0xFFu;
    PEC_result  = LTC_CalculatePec15(1, PEC_1);
    pTxBuff[6]  = (PEC_result >> 8u) & 0xFFu;
    pTxBuff[7]  = PEC_result & 0xFFu;
    frameLength = 8;

    retVal = LTC_TRANSMIT_RECEIVE_DATA(pSpiInterface, pTxBuff, pRxBuff, frameLength);

    return retVal;
}

static STD_RETURN_TYPE_e LTC_CheckPec(
    LTC_MCU_STATE_s *ltc_state,
    uint16_t *DataBufferSPI_RX_with_PEC,
    uint8_t stringNumber) {
    FAS_ASSERT(ltc_state != NULL_PTR);
    FAS_ASSERT(DataBufferSPI_RX_with_PEC != NULL_PTR);
    STD_RETURN_TYPE_e retVal = STD_OK;
    uint8_t PEC_TX[2];
    uint16_t PEC_result                       = 0;
    uint8_t PEC_Check[LTC_DATA_SIZE_IN_BYTES] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    /* check all PECs and put data without command and PEC in DataBufferSPI_RX (easier to use) */

    for (uint8_t i = 0u; i < (uint8_t)((ltc_state->ltcData.frameLength - 4) / LTC_DATA_SIZE_IN_BYTES); i++) {
        PEC_Check[0] = DataBufferSPI_RX_with_PEC[4u + 8u * i];
        PEC_Check[1] = DataBufferSPI_RX_with_PEC[5u + 8u * i];
        PEC_Check[2] = DataBufferSPI_RX_with_PEC[6u + 8u * i];
        PEC_Check[3] = DataBufferSPI_RX_with_PEC[7u + 8u * i];
        PEC_Check[4] = DataBufferSPI_RX_with_PEC[8u + 8u * i];
        PEC_Check[5] = DataBufferSPI_RX_with_PEC[9u + 8u * i];

        PEC_result = LTC_CalculatePec15(LTC_DATA_SIZE_IN_BYTES, PEC_Check);
        PEC_TX[0]  = (uint8_t)((PEC_result >> 8u) & 0xFFu);
        PEC_TX[1]  = (uint8_t)(PEC_result & 0xFFu);

        /* if calculated PEC not equal to received PEC */
        if ((PEC_TX[0] != DataBufferSPI_RX_with_PEC[10u + 8u * i]) ||
            (PEC_TX[1] != DataBufferSPI_RX_with_PEC[11u + 8u * i])) {
            /* update error table of the corresponding LTC only if PEC check is activated */
            if (LTC_DISCARD_PEC == false) {
                ltc_state->ltcData.errorTable->PEC_valid[stringNumber] &= false;
                retVal |= STD_NOT_OK;
            } else {
                ltc_state->ltcData.errorTable->PEC_valid[stringNumber] &= true;
            }
        } else {
            /* update error table of the corresponding LTC */
            ltc_state->ltcData.errorTable->PEC_valid[stringNumber] &= true;
        }
    }
    return retVal;
}

/**
 * @brief   send command to the LTC daisy-chain and receives data from the LTC
 *          daisy-chain.
 * @details This is the core function to receive data from the LTC6813-1
 *          daisy-chain.
 *          A 2 byte command is sent with the corresponding PEC.
 *          *Example*: read configuration register (RDCFG).
 *          Only command has to be set, the function calculates the PEC
 *          automatically.
 *          - The data sent is:
 *            - 2 bytes (COMMAND) 2 bytes (PEC)
 *          - The data received is:
 *            - 6 bytes (LTC1) 2 bytes (PEC) +
 *            - 6 bytes (LTC2) 2 bytes (PEC) +
 *            - 6 bytes (LTC3) 2 bytes (PEC) +
 *            - ... +
 *            - 6 bytes (LTC{LTC_N_LTC}) 2 bytes (PEC)
 *
 *          The function does not check the PECs. This has to be done
 *          elsewhere.
 *
 * @param   Command         command sent to the daisy-chain
 * @param   pSpiInterface   pointer to SPI configuration
 * @param   pTxBuff         transmit buffer
 * @param   pRxBuff         receive buffer
 * @param   frameLength     number of words to transmit
 *
 * @return  #STD_OK if SPI transmission is OK, #STD_NOT_OK otherwise
 */
static STD_RETURN_TYPE_e LTC_ReadRegister(
    uint16_t *Command,
    SPI_INTERFACE_CONFIG_s *pSpiInterface,
    uint16_t *pTxBuff,
    uint16_t *pRxBuff,
    uint32_t frameLength) {
    FAS_ASSERT(Command != NULL_PTR);
    FAS_ASSERT(pSpiInterface != NULL_PTR);
    FAS_ASSERT(pTxBuff != NULL_PTR);
    FAS_ASSERT(pRxBuff != NULL_PTR);
    STD_RETURN_TYPE_e retVal = STD_NOT_OK;

    /* DataBufferSPI_RX_with_PEC contains the data to receive.
       The transmission function checks the PECs.
       It constructs DataBufferSPI_RX, which contains the received data without PEC (easier to use). */

    for (uint16_t i = 0; i < LTC_N_BYTES_FOR_DATA_TRANSMISSION; i++) {
        pTxBuff[i] = 0x00;
    }

    pTxBuff[0] = Command[0];
    pTxBuff[1] = Command[1];
    pTxBuff[2] = Command[2];
    pTxBuff[3] = Command[3];

    retVal = LTC_TRANSMIT_RECEIVE_DATA(pSpiInterface, pTxBuff, pRxBuff, frameLength);

    return retVal;
}

/**
 * @brief   sends command and data to the LTC daisy-chain.
 * @details This is the core function to transmit data to the LTC6813-1
 *          daisy-chain.
 *          The data sent is:
 *          - COMMAND +
 *          - 6 bytes (LTC1) +
 *          - 6 bytes (LTC2) +
 *          - 6 bytes (LTC3) +
 *          - ... +
 *          - 6 bytes (LTC{LTC_N_LTC})
 *
 *          A 2 byte command is sent with the corresponding PEC.
 *          *Example*: write configuration register (WRCFG).
 *          The command has to be set and then the function calculates the PEC
 *          automatically.
 *          The function calculates the needed PEC to send the data to the
 *          daisy-chain.
 *          The sent data has the format:
 *          - 2 byte-COMMAND (2 bytes PEC) +
 *          - 6 bytes (LTC1) (2 bytes PEC) +
 *          - 6 bytes (LTC2) (2 bytes PEC) +
 *          - 6 bytes (LTC3) (2 bytes PEC) +
 *          - ... +
 *          - 6 bytes (LTC{LTC_N_LTC}) (2 bytes PEC)
 *
 *          The function returns 0. The only way to check if the transmission
 *          was successful is to read the results of the write operation.
 *          (example: read configuration register after writing to it)
 *
 * @param   Command         command sent to the daisy-chain
 * @param   pSpiInterface   pointer to SPI configuration
 * @param   pTxBuff         transmit buffer
 * @param   pRxBuff         receive buffer
 * @param   frameLength     number of words to transmit
 *
 * @return  STD_OK if SPI transmission is OK, STD_NOT_OK otherwise
 */
/*
static STD_RETURN_TYPE_e LTC_WriteRegister(
    uint16_t *Command,
    SPI_INTERFACE_CONFIG_s *pSpiInterface,
    uint16_t *pTxBuff,
    uint16_t *pRxBuff,
    uint32_t frameLength) {
    FAS_ASSERT(Command != NULL_PTR);
    FAS_ASSERT(pSpiInterface != NULL_PTR);
    FAS_ASSERT(pTxBuff != NULL_PTR);
    FAS_ASSERT(pRxBuff != NULL_PTR);
    STD_RETURN_TYPE_e retVal = STD_NOT_OK;

    uint16_t PEC_result                       = 0;
    uint8_t PEC_Check[LTC_DATA_SIZE_IN_BYTES] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    pTxBuff[0] = Command[0];
    pTxBuff[1] = Command[1];
    pTxBuff[2] = Command[2];
    pTxBuff[3] = Command[3];

    // Calculate PEC of all data (1 PEC value for 6 bytes)
    for (uint16_t i = 0u; i < LTC_N_LTC; i++) {
        PEC_Check[0] = pTxBuff[4u + (i * 8u)];
        PEC_Check[1] = pTxBuff[5u + (i * 8u)];
        PEC_Check[2] = pTxBuff[6u + (i * 8u)];
        PEC_Check[3] = pTxBuff[7u + (i * 8u)];
        PEC_Check[4] = pTxBuff[8u + (i * 8u)];
        PEC_Check[5] = pTxBuff[9u + (i * 8u)];

        PEC_result              = LTC_CalculatePec15(LTC_DATA_SIZE_IN_BYTES, PEC_Check);
        pTxBuff[10u + (i * 8u)] = (PEC_result >> 8u) & 0xFFu;
        pTxBuff[11u + (i * 8u)] = PEC_result & 0xFFu;
    }

    retVal = LTC_TRANSMIT_RECEIVE_DATA(pSpiInterface, pTxBuff, pRxBuff, frameLength);

    return retVal;
}
*/
