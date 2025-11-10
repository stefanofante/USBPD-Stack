/**
 * @file usb_pd_port_stm32.c
 * @brief STM32 port layer implementation for USB PD stack
 * 
 * @details Provides FreeRTOS-based timer orchestration for STM32 platforms.
 *          Uses a 1ms software timer to drive the PD stack's protocol timing.
 * 
 * @author Stefano Fante (STLINE SRL)
 * @date 2025
 */

#include "usb_pd_port_stm32.h"

#if defined(PD_CONFIG_TARGET_STM32) && (PD_CONFIG_TARGET_STM32)

#include <stddef.h>
#include <string.h>

#include "FreeRTOS.h"
#include "timers.h"
#include "usb_pd_timer.h"

/** @brief External timer ISR function from USB PD core */
extern void PD_TimerIsrFunction(pd_handle pdHandle);

/** @brief Static storage for the FreeRTOS software timer */
static StaticTimer_t s_timerStorage;

/** @brief Handle to the FreeRTOS software timer (1ms period) */
static TimerHandle_t s_timerHandle;

/** @brief Array of registered PD instances to service in timer callback */
static pd_instance_t *s_registeredInstances[PD_CONFIG_MAX_PORT];

/**
 * @brief FreeRTOS timer callback invoked every 1ms
 * 
 * @details Takes a critical-section snapshot of all registered PD instances
 *          and invokes the protocol timer for each active instance.
 * 
 * @param[in] timer FreeRTOS timer handle (unused)
 */
static void PD_PortStm32_TimerCallback(TimerHandle_t timer)
{
    (void)timer;

    pd_instance_t *snapshot[PD_CONFIG_MAX_PORT] = {0};

    taskENTER_CRITICAL();
    memcpy(snapshot, s_registeredInstances, sizeof(snapshot));
    taskEXIT_CRITICAL();

    for (size_t i = 0; i < PD_CONFIG_MAX_PORT; ++i)
    {
        if (snapshot[i] != NULL)
        {
            PD_TimerIsrFunction((pd_handle)snapshot[i]);
        }
    }
}

/**
 * @brief Lazy initialization helper for the FreeRTOS timer
 * 
 * @details Creates a static software timer with 1ms period if not already created.
 *          The timer runs continuously (auto-reload) when started.
 */
static void PD_PortStm32_EnsureTimer(void)
{
    if (s_timerHandle == NULL)
    {
        s_timerHandle = xTimerCreateStatic("pd_tick",
                                           pdMS_TO_TICKS(1),
                                           pdTRUE,
                                           NULL,
                                           PD_PortStm32_TimerCallback,
                                           &s_timerStorage);
    }
}

/**
 * @brief Register a PD instance with the STM32 port layer
 * 
 * @details Adds the instance to the scheduler and starts the FreeRTOS timer
 *          if this is the first active instance. Safe to call from task context.
 * 
 * @param[in] instance Pointer to the PD instance to register
 * 
 * @note The timer is started automatically when the first instance is registered
 */
void PD_PortStm32_RegisterInstance(pd_instance_t *instance)
{
    if (instance == NULL)
    {
        return;
    }

    PD_PortStm32_EnsureTimer();

    bool needStart = false;
    bool inserted  = false;

    taskENTER_CRITICAL();
    for (size_t i = 0; i < PD_CONFIG_MAX_PORT; ++i)
    {
        if (s_registeredInstances[i] == NULL)
        {
            s_registeredInstances[i] = instance;
            inserted                 = true;
            break;
        }
    }

    if (inserted && (s_timerHandle != NULL))
    {
        needStart = (xTimerIsTimerActive(s_timerHandle) == pdFALSE);
    }
    taskEXIT_CRITICAL();

    if (inserted && (s_timerHandle != NULL) && needStart)
    {
        (void)xTimerStart(s_timerHandle, 0);
    }
}

/**
 * @brief Unregister a PD instance from the STM32 port layer
 * 
 * @details Removes the instance from the scheduler and stops the FreeRTOS timer
 *          if no instances remain active. Safe to call from task context.
 * 
 * @param[in] instance Pointer to the PD instance to unregister
 * 
 * @note The timer is stopped automatically when the last instance is unregistered
 */
void PD_PortStm32_UnregisterInstance(pd_instance_t *instance)
{
    if (instance == NULL)
    {
        return;
    }

    bool hasInstances = false;

    taskENTER_CRITICAL();
    for (size_t i = 0; i < PD_CONFIG_MAX_PORT; ++i)
    {
        if (s_registeredInstances[i] == instance)
        {
            s_registeredInstances[i] = NULL;
        }
        if (s_registeredInstances[i] != NULL)
        {
            hasInstances = true;
        }
    }
    taskEXIT_CRITICAL();

    if ((s_timerHandle != NULL) && !hasInstances)
    {
        (void)xTimerStop(s_timerHandle, 0);
    }
}

#endif /* PD_CONFIG_TARGET_STM32 */
