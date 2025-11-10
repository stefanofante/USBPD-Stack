/**
 * @file usb_pd_port_arduino.cpp
 * @brief Arduino cooperative task scheduler for USB PD stack
 * 
 * @details Implements a lightweight cooperative multitasking approach using
 *          micros() polling. No hardware timers or RTOS are required. The user
 *          must call PD_PortArduino_TaskTick() from their loop() function.
 * 
 * @author Stefano Fante (STLINE SRL)
 * @date 2025
 */

#include "usb_pd_port_arduino.h"

#if defined(PD_CONFIG_TARGET_ARDUINO) && (PD_CONFIG_TARGET_ARDUINO)

#include <Arduino.h>
#include <stddef.h>
#include <string.h>

#include "usb_pd_timer.h"

extern "C" void PD_TimerIsrFunction(pd_handle pdHandle);

namespace
{
/** @brief Array of registered PD instances */
static pd_instance_t *s_registeredInstances[PD_CONFIG_MAX_PORT];

/** @brief Last micros() value when a 1ms tick was processed */
static uint32_t s_lastTickMicros;

/**
 * @brief Create a snapshot of registered instances for safe iteration
 * 
 * @param[out] out Destination array (must be PD_CONFIG_MAX_PORT elements)
 */
static void SnapshotInstances(pd_instance_t **out)
{
    memcpy(out, s_registeredInstances, sizeof(s_registeredInstances));
}
}

extern "C" {

/**
 * @brief Register a PD instance with the Arduino port layer
 * 
 * @param[in] instance Pointer to the PD instance to register
 * 
 * @details Adds the instance to the internal registry and initializes the
 *          micros() timestamp for cooperative scheduling. Uses noInterrupts()
 *          for thread safety.
 * 
 * @note If this is the first instance, s_lastTickMicros is initialized to
 *       enable PD_PortArduino_TaskTick() operation.
 */
void PD_PortArduino_RegisterInstance(pd_instance_t *instance)
{
    if (instance == NULL)
    {
        return;
    }

    noInterrupts();
    for (size_t i = 0; i < PD_CONFIG_MAX_PORT; ++i)
    {
        if (s_registeredInstances[i] == instance)
        {
            interrupts();
            return;
        }
    }

    for (size_t i = 0; i < PD_CONFIG_MAX_PORT; ++i)
    {
        if (s_registeredInstances[i] == NULL)
        {
            s_registeredInstances[i] = instance;
            if (s_lastTickMicros == 0U)
            {
                s_lastTickMicros = micros();
            }
            break;
        }
    }
    interrupts();
}

/**
 * @brief Unregister a PD instance from the Arduino port layer
 * 
 * @param[in] instance Pointer to the PD instance to unregister
 * 
 * @details Removes the instance from the registry. If no instances remain,
 *          resets s_lastTickMicros to 0 to stop PD_PortArduino_TaskTick().
 * 
 * @note Uses noInterrupts() to prevent race conditions during unregistration.
 */
void PD_PortArduino_UnregisterInstance(pd_instance_t *instance)
{
    if (instance == NULL)
    {
        return;
    }

    bool anyInstance = false;

    noInterrupts();
    for (size_t i = 0; i < PD_CONFIG_MAX_PORT; ++i)
    {
        if (s_registeredInstances[i] == instance)
        {
            s_registeredInstances[i] = NULL;
        }
        if (s_registeredInstances[i] != NULL)
        {
            anyInstance = true;
        }
    }

    if (!anyInstance)
    {
        s_lastTickMicros = 0U;
    }
    interrupts();
}

/**
 * @brief Cooperative task tick function to be called from Arduino loop()
 * 
 * @details Polls micros() and invokes PD_TimerIsrFunction() for each
 *          registered instance when 1ms has elapsed. Implements a catch-up
 *          mechanism (up to 8 ticks) to handle loop delays.
 * 
 * @note This function MUST be called regularly from loop() for the USB PD
 *       stack to operate correctly. Missing calls will delay timer events.
 * 
 * @warning The catch-up is bounded to 8ms to avoid starving the main loop.
 */
void PD_PortArduino_TaskTick(void)
{
    if (s_lastTickMicros == 0U)
    {
        return;
    }

    uint32_t now      = micros();
    uint32_t elapsed  = static_cast<uint32_t>(now - s_lastTickMicros);
    if (elapsed < 1000U)
    {
        return;
    }

    uint32_t tickBudget = elapsed / 1000U;
    if (tickBudget > 8U)
    {
        tickBudget = 8U; /* Keep catch-up bounded to avoid starving loop() */
    }

    s_lastTickMicros += tickBudget * 1000U;

    pd_instance_t *snapshot[PD_CONFIG_MAX_PORT] = {};
    noInterrupts();
    SnapshotInstances(snapshot);
    interrupts();

    while (tickBudget-- > 0U)
    {
        for (size_t i = 0; i < PD_CONFIG_MAX_PORT; ++i)
        {
            if (snapshot[i] != NULL)
            {
                PD_TimerIsrFunction((pd_handle)snapshot[i]);
            }
        }
    }
}

}

#endif /* PD_CONFIG_TARGET_ARDUINO */
