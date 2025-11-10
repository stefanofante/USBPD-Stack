/**
 * @file usb_pd_timer.c
 * @brief USB PD protocol timer management
 * 
 * @details Implements software timers for USB Power Delivery state machine.
 *          Uses bitmaps for timer state tracking (running/timeout) and 1ms ISR
 *          for decrement-based countdown. Timers are spec-compliant for PD 2.0/3.0.
 * 
 * @copyright Copyright 2016 - 2017 NXP
 * @copyright All rights reserved.
 * 
 * @license SPDX-License-Identifier: BSD-3-Clause
 */

#include "usb_pd_config.h"
#include "usb_pd.h"
#include "usb_pd_phy.h"
#include "usb_pd_timer.h"
#include "usb_pd_interface.h"
#if (defined PD_CONFIG_ALT_MODE_SUPPORT) && (PD_CONFIG_ALT_MODE_SUPPORT)
#include "usb_pd_alt_mode.h"
#endif

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/**
 * @brief Timer timeout callback dispatcher
 * 
 * @param[in] pdInstance Pointer to PD instance
 * @param[in] timrName Timer identifier (cast from tTimer_t)
 * 
 * @details Handles timer expiration events. Most timers post a PD_TASK_EVENT_TIME_OUT
 *          event to the policy engine for processing.
 * 
 * @note 1. Timer has started & timer time out: timrsTimeOutState == 1
 * @note 2. Timer not start || timer time out: timrsRunningState == 0
 */
static void PD_TimerCallback(pd_instance_t *pdInstance, uint8_t timrName)
{
    switch ((tTimer_t)timrName)
    {
        case tMsgHardResetCompleteTimer:
            /* do nothing */
            break;
#if 0
        /* if has USB function */
        case tUSBSuspendTimer:
        case tUSBTimer:
            break;
#endif

        default:
            /* do nothing, code will use _PD_TimerCheckInvalidOrTimeOut to check the timr */
            PD_StackSetEvent(pdInstance, PD_TASK_EVENT_TIME_OUT);
            break;
    }
}

/**
 * @brief Check if a timer bit is set in a bitmap array
 * 
 * @param[in] dataArray Pointer to 32-bit bitmap array (running or timeout state)
 * @param[in] timrName Timer identifier
 * @return 1 if bit is set, 0 otherwise
 * 
 * @details Uses bitwise operations to efficiently check timer state in packed bitmaps.
 */
static uint8_t PD_TimerCheckBit(volatile uint32_t *dataArray, tTimer_t timrName)
{
    if ((uint8_t)timrName >= PD_MAX_TIMER_COUNT)
    {
        return 0;
    }

    /* timr is not running */
    if ((dataArray[((uint8_t)timrName / 32U)] & (0x00000001UL << ((uint8_t)timrName & 0x1FU))) != 0U)
    {
        return 1;
    }
    return 0;
}

/**
 * @brief Clear a timer (stop and reset)
 * 
 * @param[in] pdHandle PD instance handle
 * @param[in] timrName Timer identifier
 * @return kStatus_PD_Success on success, kStatus_PD_Error if invalid timer
 * 
 * @details Clears both running and timeout states. Thread-safe via critical section.
 */
pd_status_t PD_TimerClear(pd_handle pdHandle, tTimer_t timrName)
{
    uint32_t bitMape;
    pd_instance_t *pdInstance = (pd_instance_t *)pdHandle;
    OSA_SR_ALLOC();

    if ((uint8_t)timrName >= PD_MAX_TIMER_COUNT)
    {
        return kStatus_PD_Error;
    }

    bitMape = (uint32_t)(0x00000001UL << ((uint8_t)timrName & 0x1FU));

    OSA_ENTER_CRITICAL();
    pdInstance->timrsRunningState[((uint8_t)timrName / 32U)] &= (~bitMape);
    pdInstance->timrsTimeOutState[((uint8_t)timrName / 32U)] &= (~bitMape);
    OSA_EXIT_CRITICAL();

    return kStatus_PD_Success;
}

/**
 * @brief Check if a timer has started (running or timed out)
 * 
 * @param[in] pdHandle PD instance handle
 * @param[in] timrName Timer identifier
 * @return 1 if timer has started, 0 otherwise
 * 
 * @details Returns true if timer is currently running OR has already timed out.
 */
uint8_t PD_TimerCheckStarted(pd_handle pdHandle, tTimer_t timrName)
{
    pd_instance_t *pdInstance = (pd_instance_t *)pdHandle;
    uint8_t retVal;

    if (PD_TimerCheckBit(&pdInstance->timrsRunningState[0], timrName) != 0U)
    {
        retVal = 1;
    }
    else if (PD_TimerCheckBit(&pdInstance->timrsTimeOutState[0], timrName) != 0U)
    {
        retVal = 1;
    }
    else
    {
        retVal = 0;
    }

    return retVal;
}

/**
 * @brief Start a timer with specified duration
 * 
 * @param[in] pdHandle PD instance handle
 * @param[in] timrName Timer identifier
 * @param[in] timrTime Duration in milliseconds (must be > 0)
 * @return kStatus_PD_Success on success, kStatus_PD_Error if invalid parameters
 * 
 * @details Sets running state, clears timeout state, and initializes countdown value.
 *          Thread-safe via critical section.
 */
pd_status_t PD_TimerStart(pd_handle pdHandle, tTimer_t timrName, uint16_t timrTime)
{
    uint32_t bitMape;
    pd_instance_t *pdInstance = (pd_instance_t *)pdHandle;
    OSA_SR_ALLOC();

    if ((uint8_t)timrName >= PD_MAX_TIMER_COUNT)
    {
        return kStatus_PD_Error;
    }
    if (timrTime == 0U)
    {
        return kStatus_PD_Error;
    }

    bitMape = (uint32_t)(0x00000001UL << ((uint8_t)timrName & 0x1FU));

    OSA_ENTER_CRITICAL();
    pdInstance->timrsTimeValue[timrName] = timrTime;
    pdInstance->timrsTimeOutState[((uint8_t)timrName / 32U)] &= (~(bitMape));
    pdInstance->timrsRunningState[((uint8_t)timrName / 32U)] |= bitMape;
    OSA_EXIT_CRITICAL();

    return kStatus_PD_Success;
}

/**
 * @brief Check if timer is invalid or has timed out
 * 
 * @param[in] pdHandle PD instance handle
 * @param[in] timrName Timer identifier
 * @return 1 if timer is not running (invalid or timed out), 0 if running
 * 
 * @details Used by policy engine to check if timer has expired.
 */
uint8_t PD_TimerCheckInvalidOrTimeOut(pd_handle pdHandle, tTimer_t timrName)
{
    pd_instance_t *pdInstance = (pd_instance_t *)pdHandle;

    return (PD_TimerCheckBit(&pdInstance->timrsRunningState[0], timrName) == 0U) ? 1U : 0U;
}

/**
 * @brief Check if timer has validly timed out
 * 
 * @param[in] pdHandle PD instance handle
 * @param[in] timrName Timer identifier
 * @return 1 if timer has timed out, 0 otherwise
 * 
 * @details Checks the timeout state bitmap to determine if timer expired.
 */
uint8_t PD_TimerCheckValidTimeOut(pd_handle pdHandle, tTimer_t timrName)
{
    pd_instance_t *pdInstance = (pd_instance_t *)pdHandle;

    return PD_TimerCheckBit(&pdInstance->timrsTimeOutState[0], timrName);
}

/**
 * @brief Cancel a range of timers
 * 
 * @param[in] pdHandle PD instance handle
 * @param[in] timrBegin Start of timer range (inclusive)
 * @param[in] timrEnd End of timer range (inclusive)
 * 
 * @details Clears both running and timeout states for all timers in range.
 *          Thread-safe via critical section.
 */
void PD_TimerCancelAllTimers(pd_handle pdHandle, tTimer_t timrBegin, tTimer_t timrEnd)
{
    uint8_t index;
    uint32_t bitMape;
    pd_instance_t *pdInstance = (pd_instance_t *)pdHandle;
    OSA_SR_ALLOC();

    if (((uint8_t)timrBegin >= PD_MAX_TIMER_COUNT) || ((uint8_t)timrEnd >= PD_MAX_TIMER_COUNT))
    {
        return;
    }
    OSA_ENTER_CRITICAL();
    for (index = (uint8_t)timrBegin; index <= (uint8_t)timrEnd; ++index)
    {
        bitMape = (uint32_t)(0x00000001UL << (index & 0x1FU));
        pdInstance->timrsRunningState[(index / 32U)] &= (~bitMape);
        pdInstance->timrsTimeOutState[(index / 32U)] &= (~bitMape);
    }
    OSA_EXIT_CRITICAL();
}

/**
 * @brief 1ms ISR timer tick function
 * 
 * @param[in] pdHandle PD instance handle
 * 
 * @details Called every 1ms by platform timer ISR (FreeRTOS or Arduino cooperative).
 *          Decrements all running timers and triggers callbacks on expiration.
 *          Also calls PD_AltModeTimer1msISR() if alternate modes are enabled.
 * 
 * @warning Must be called from ISR context or with interrupts disabled
 */
void PD_TimerIsrFunction(pd_handle pdHandle)
{
    pd_instance_t *pdInstance = (pd_instance_t *)pdHandle;
    uint8_t index;
    uint8_t index8;
    uint32_t bitMape;
    uint8_t currentIndex;

    if (pdHandle == NULL)
    {
        return;
    }

    for (index = 0; index < (((uint8_t)tTimerCount + 31U) / 32U); ++index)
    {
        if (pdInstance->timrsRunningState[index] != 0U)
        {
            for (index8 = 0; index8 < 32U; ++index8)
            {
                bitMape      = (uint32_t)(0x00000001UL << index8);
                currentIndex = (uint8_t)((index * 32U) + index8);
                if (currentIndex < (uint8_t)tTimerCount)
                {
                    if ((pdInstance->timrsRunningState[index] & bitMape) != 0U)
                    {
                        if (((pdInstance->timrsTimeValue[currentIndex])--) == 0U)
                        {
                            PD_TimerCallback(pdInstance, currentIndex);
                            pdInstance->timrsTimeValue[currentIndex] = 0;
                            pdInstance->timrsRunningState[index] &= (~(bitMape));
                            pdInstance->timrsTimeOutState[index] |= (bitMape);
                        }
                    }
                }
            }
        }
    }
#if (defined PD_CONFIG_ALT_MODE_SUPPORT) && (PD_CONFIG_ALT_MODE_SUPPORT)
    PD_AltModeTimer1msISR();
#endif
}

/**
 * @brief Initialize timer subsystem for a PD instance
 * 
 * @param[in] pdHandle PD instance handle
 * 
 * @details Clears all timer bitmaps and countdown values to zero.
 */
void PD_TimerInit(pd_handle pdHandle)
{
    uint8_t index8;
    pd_instance_t *pdInstance = (pd_instance_t *)pdHandle;

    for (index8 = 0; index8 < ((PD_MAX_TIMER_COUNT + 31U) / 32U); ++index8)
    {
        pdInstance->timrsRunningState[index8] = 0;
        pdInstance->timrsTimeOutState[index8] = 0;
    }
    for (index8 = 0; index8 < PD_MAX_TIMER_COUNT; ++index8)
    {
        pdInstance->timrsTimeValue[index8] = 0;
    }
}

/**
 * @brief Check if any timers are currently running
 * 
 * @param[in] pdHandle PD instance handle
 * @return 1 if any timer is running, 0 if all idle
 * 
 * @details Scans all running state bitmaps for non-zero values.
 */
uint8_t PD_TimerBusy(pd_handle pdHandle)
{
    pd_instance_t *pdInstance = (pd_instance_t *)pdHandle;
    uint32_t index32;

    for (index32 = 0; index32 < (((uint8_t)tTimerCount + 31U) / 32U); ++index32)
    {
        if (pdInstance->timrsRunningState[index32] != 0U)
        {
            return 1;
        }
    }
    return 0;
}
