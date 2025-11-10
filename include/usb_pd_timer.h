/**
 * @file usb_pd_timer.h
 * @brief USB PD protocol timer definitions and API
 * 
 * @details Defines all USB Power Delivery specification timers (PD 2.0/3.0)
 *          and provides timer management API for policy engine and state machines.
 * 
 * @copyright Copyright 2016 - 2017 NXP
 * @copyright All rights reserved.
 * 
 * @license SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __PD_TIMER_H__
#define __PD_TIMER_H__

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/**
 * @brief Convert milliseconds to timer ticks
 * 
 * @param MSEC Milliseconds
 * @return Timer ticks (1 tick = 1ms)
 */
#define mSec(MSEC) (MSEC)

/**
 * @brief USB PD timer identifiers
 * 
 * @details Enumeration of all timers used by USB PD state machines:
 *          - Policy Engine timers (PSM)
 *          - Device Policy Manager timers (DPM)
 *          - Type-C state machine timers
 *          - VDM (Vendor Defined Message) timers
 */
typedef enum tTimer
{
    _tStartTimer           = 0,
    tSenderResponseTimer   = 0,
    tSourceCapabilityTimer = 1,
    tSinkWaitCapTimer      = 2,
    tSinkRequestTimer      = 3,
    tDrSwapWaitTimer       = 4,
    tPrSwapWaitTimer       = 5,
    tPSTransitionTimer     = 6,
    tPSSourceOffTimer      = 7,
    tPSSourceOnTimer       = 8,
    tNoResponseTimer = 9, /* The NoResponseTimer is used by the Policy Engine in a Source or Sink to determine that its
                            Port Partner is not responding after a Hard Reset. */
    tSwapSourceStartTimer = 10, /* pr swap */
    tPSHardResetTimer     = 11, /* source hard_reset */
    tVconnOnTimer         = 12, /* vconn swap */
    tVDMResponseTimer     = 13,
    tVDMModeEntryTimer    = 14,
    tVDMModeExitTimer     = 15,
    tSinkTxTimer          = 16,

    tSrcRecoverTimer            = 17, /* hard_reset tSrcRecover */
    timrPsmRdyEvalDelayTimer    = 18,
    tMsgHardResetCompleteTimer  = 19, /* MSG */
    tSinkPPSPeriodicTimer       = 20,
    tSourcePPSCommTimer         = 21,
    _tMaxPSMTimer               = tMsgHardResetCompleteTimer,  /* The maximum PSM timer */
    tAMETimeoutTimer            = 22,                          /* T_AME_TIMEOUT */
    _tMaxDpmTimer               = tAMETimeoutTimer,            /* The maximum port timer */
    tPDDebounceTimer            = 23,                          /* TypeC */
    tPDDebounce2Timer           = 24,                          /* TypeC */
    tCCDebounceTimer            = 25,                          /* TypeC */
    tCCDebounceTimer1           = 26,                          /* TypeC */
    tCCDebounceTimer2           = 27,                          /* TypeC */
    tDRPToggleTimer             = 28,                          /* TypeC */
    tTypeCVbusMinDischargeTimer = 29,                          /* TypeC */
    tTypeCVbusMaxDischargeTimer = 30,                          /* TypeC */
    _tMaxTypeCTimer             = tTypeCVbusMaxDischargeTimer, /* The maximum TypeC timer */
    tVDMBusyTimer               = 31,
    tDelayTimer                 = 32, /* tSrcRecover, T_SRC_TRANSITION (DPM), T_ERROR_RECOVERY (CONNECT), */
    timrCCFilter                = 33,

    tDRPTryWaitTimer             = 34,
    tDRPTryTimer                 = 35,
    tTryTimeoutTimer             = 36,
    tBISTContModeTimer           = 37,
    tFRSwapSignalTimer           = 38,
    tVBUSONTimer                 = 39,
    timrChunkSenderResponseTimer = 40,
    timrChunkSenderRequestTimer  = 41,
    timrMsgSendWaitResultTimer   = 42,
    timrFRSwapWaitPowerStable    = 43,
    tDiscoverIdentityTimer       = 44,
    tDpmComandRetryTimer         = 45,
    tTimerCount                  = 46,
    _tEnd                        = tTimerCount - 1,

/* don't implement yet */
#if 0
    tUFPDebounceTimer = 35,
    tUSBSuspendTimer = 36, /* USB */
    tUSBTimer = 37,        /* USB */
    tHciSpDebounceTimer = 38,
    tHostifWakeupTimer = 39,
#endif
} tTimer_t;

/** @brief Maximum number of timers supported */
#define PD_MAX_TIMER_COUNT (46U)

/* USB PD Specification Timer Constants (all in milliseconds) */

/** @brief Fast Role Swap power stabilization wait (50ms) */
#define T_FRSWAP_WAIT_POWER_STABLE (50U)

/** @brief Wait for message send result (20ms) */
#define T_WAIT_SEND_RESULT         mSec(20)

/** @brief PPS timeout (15s) */
#define T_PPS_TIMEOUT              mSec(15000)

/** @brief PPS request interval (10s) */
#define T_PPS_REQUEST              mSec(10000)

/** @brief Hard reset completion max (5ms) */
#define T_HARD_RESET_COMPLETE      mSec(5)

/** @brief UFP connection debounce max (15ms) */
#define T_UFP_CONNECT_DEBOUNCE     mSec(15)

/** @brief Power role swap source start (25ms - spec: 20ms min) */
#define T_SWAP_SOURCE_START        mSec(25)

/** @brief Source transition time (30ms - spec: 25-35ms) */
#define T_SRC_TRANSITION           mSec(30)

/** @brief Sender response timeout (26ms - spec: 24-30ms) */
#define T_SENDER_RESPONSE          mSec(26)

/** @brief VDM sender response (27ms - spec: 24-30ms) */
#define T_VDM_SENDER_RESPONSE      mSec(27)

/** @brief Chunked message sender response (25ms - spec: 24-30ms) */
#define T_CHUNK_SENDER_RESPONSE    mSec(25)

/** @brief Chunked message sender request (25ms - spec: 24-30ms) */
#define T_CHUNK_SENDER_REQUEST     mSec(25)

/** @brief Power supply hard reset (26ms - spec: 25-35ms) */
#define T_PS_HARD_RESET            mSec(26)

/** @brief VBUS safe 0V max (650ms - spec: 0-650ms) */
#define T_SAFE0V_MAX               mSec(650)

/** @brief Error recovery time (30ms - spec: 25ms min) */
#define T_ERROR_RECOVERY           mSec(30)

/** @brief BIST continuous mode (45ms - spec: 30-60ms) */
#define T_BIST_CONT_MODE           mSec(45)

/** @brief Discover identity timer (45ms - spec: 40-50ms) */
#define T_DISCOVER_IDENTITY        mSec(45)

/** @brief VDM mode entry wait (45ms - spec: 40-50ms) */
#define T_VDM_WAIT_MODE_ENTRY      mSec(45)

/** @brief VDM mode exit wait (45ms - spec: 40-50ms) */
#define T_VDM_WAIT_MODE_EXIT       mSec(45)

/** @brief VDM busy timer (55ms - spec: 50ms min) */
#define T_VDM_BUSY                 mSec(55)

/** @brief VCONN source on max (100ms) */
#define T_VCONN_SOURCE_ON          mSec(100)

/** @brief VCONN powered accessory source on max (100ms) */
#define T_VCONN_PA_SOURCE_ON       mSec(100)

/** @brief Sink request timer (100ms - spec: 100ms min, no max) */
#define T_SINK_REQUEST             mSec(100)

/** @brief Data role swap wait (100ms - spec: 100ms min, no max) */
#define T_DRSWAP_WAIT              mSec(100)

/** @brief Data role swap wait for alt mode exit (200ms) */
#define T_DRSWAP_WAIT_ALT_MODE     mSec(200)

/** @brief Power role swap wait (100ms - spec: 100ms min, no max) */
#define T_PRSWAP_WAIT              mSec(100)

/** @brief Policy engine ready evaluation delay (120ms - arbitrary) */
#define T_PSM_RDY_EVAL_DELAY       mSec(120)

#if (defined PD_CONFIG_TRY_SRC_SUPPORT) && (PD_CONFIG_TRY_SRC_SUPPORT)
/** @brief Sink wait for capabilities (311ms - spec: 310-620ms, Try.SRC variant) */
#define T_SINK_WAIT_CAP mSec(311)
#else
/** @brief Sink wait for capabilities (400ms - spec: 310-620ms) */
#define T_SINK_WAIT_CAP mSec(400)
#endif

/** @brief Power supply source on (400ms - spec: 390-480ms) */
#define T_PS_SOURCE_ON           mSec(400)

/** @brief Power supply transition (500ms - spec: 450-550ms) */
#define T_PS_TRANSITION          mSec(500)

/** @brief Swap recovery time (500ms - spec: 500-1000ms) */
#define T_SWAP_RECOVER           mSec(500)

/** @brief Source recovery time (700ms - spec: 660-1000ms) */
#define T_SRC_RECOVER            mSec(700)

/** @brief Power supply source off (800ms - spec: 750-920ms) */
#define T_PS_SOURCE_OFF          mSec(800)

/** @brief Alternate mode entry timeout (900ms - spec: 1000ms max) */
#define T_AME_TIMEOUT            mSec(900)

/** @brief Hard reset max total time (2000ms) */
#define T_HARD_RESET_MAX         mSec(2000)

/** @brief No response timer (5000ms - spec: 4500-5500ms) */
#define T_NO_RESPONSE            mSec(5000)

/** @brief Host interface wakeup (150ms) */
#define T_HOSTIF_WAKEUP          mSec(150)

/** @brief USB FET overlap time (30ms) */
#define T_USBFET2_OVERLAP        mSec(30)

/** @brief USB FET transition (1ms) */
#define T_USBFET_TRANSITION      mSec(1)

/** @brief Auto power role swap test (45s) */
#define T_AUTO_PR_SWAP_TEST      mSec(45000)

/** @brief PSM source ready evaluation delay (100ms - spec: 100ms min) */
#define T_PSM_SRC_RDY_EVAL_DELAY mSec(100)

/** @brief PSM sink ready evaluation delay (115ms - spec: 100ms min) */
#define T_PSM_SNK_RDY_EVAL_DELAY mSec(115)

/** @brief VCONN stable time (50ms - spec: 50ms min) */
#define T_VCONN_STABLE    mSec(50)

/** @brief Send source capabilities interval (150ms - spec: 100-200ms) */
#define T_SEND_SOURCE_CAP mSec(150)

/** @brief Source new power available (80ms - spec: 285ms max) */
#define T_SRC_NEW_POWER   mSec(80)

/** @brief Minimum VBUS discharge time (14ms - corresponds to tccDebounce) */
#define T_MIN_VBUS_DISCHARGE mSec(14)

/** @brief Maximum VBUS discharge time (650ms - tVBUSOFF) */
#define T_MAX_VBUS_DISCHARGE mSec(650)

/** @brief Maximum VBUS on time (275ms - tVBUSON) */
#define T_MAX_VBUS_ON        mSec(275)

/** @brief Sink transmit timer (18ms - spec: 20ms max) */
#define T_SINK_TX           mSec(18)

/** @brief DPM command retry interval (5ms) */
#define T_DPM_COMMAND_RETRY mSec(5)

/** @brief CC line filter max (1ms) */
#define T_CC_FILTER_MAX mSec(1)

/*******************************************************************************
 * API
 ******************************************************************************/

/**
 * @brief Initialize timer subsystem for a PD instance
 * 
 * @param[in] pdHandle PD instance handle
 */
void PD_TimerInit(pd_handle pdHandle);

/**
 * @brief Check if timer is invalid or has timed out
 * 
 * @param[in] pdHandle PD instance handle
 * @param[in] timrName Timer identifier
 * @return 1 if timer is not running, 0 if running
 */
uint8_t PD_TimerCheckInvalidOrTimeOut(pd_handle pdHandle, tTimer_t timrName);

/**
 * @brief Check if timer has validly timed out
 * 
 * @param[in] pdHandle PD instance handle
 * @param[in] timrName Timer identifier
 * @return 1 if timer has timed out, 0 otherwise
 */
uint8_t PD_TimerCheckValidTimeOut(pd_handle pdHandle, tTimer_t timrName);

/**
 * @brief Check if timer has started
 * 
 * @param[in] pdHandle PD instance handle
 * @param[in] timrName Timer identifier
 * @return 1 if timer has started (running or timed out), 0 otherwise
 */
uint8_t PD_TimerCheckStarted(pd_handle pdHandle, tTimer_t timrName);

/**
 * @brief Start a timer with specified duration
 * 
 * @param[in] pdHandle PD instance handle
 * @param[in] timrName Timer identifier
 * @param[in] timrTime Duration in milliseconds (must be > 0)
 * @return kStatus_PD_Success on success, kStatus_PD_Error if invalid
 */
pd_status_t PD_TimerStart(pd_handle pdHandle, tTimer_t timrName, uint16_t timrTime);

/**
 * @brief Clear a timer (stop and reset)
 * 
 * @param[in] pdHandle PD instance handle
 * @param[in] timrName Timer identifier
 * @return kStatus_PD_Success on success, kStatus_PD_Error if invalid
 */
pd_status_t PD_TimerClear(pd_handle pdHandle, tTimer_t timrName);

/**
 * @brief Cancel a range of timers
 * 
 * @param[in] pdHandle PD instance handle
 * @param[in] timrBegin Start of timer range (inclusive)
 * @param[in] timrEnd End of timer range (inclusive)
 */
void PD_TimerCancelAllTimers(pd_handle pdHandle, tTimer_t timrBegin, tTimer_t timrEnd);

/**
 * @brief Check if any timers are currently running
 * 
 * @param[in] pdHandle PD instance handle
 * @return 1 if any timer is running, 0 if all idle
 */
uint8_t PD_TimerBusy(pd_handle pdHandle);

#endif
