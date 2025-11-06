#include "usb_pd_port_arduino.h"

#if defined(PD_CONFIG_TARGET_ARDUINO) && (PD_CONFIG_TARGET_ARDUINO)

#include <Arduino.h>
#include <stddef.h>
#include <string.h>

#include "usb_pd_timer.h"

extern "C" void PD_TimerIsrFunction(pd_handle pdHandle);

namespace
{
static pd_instance_t *s_registeredInstances[PD_CONFIG_MAX_PORT];
static uint32_t s_lastTickMicros;

static void SnapshotInstances(pd_instance_t **out)
{
    memcpy(out, s_registeredInstances, sizeof(s_registeredInstances));
}
}

extern "C" {

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
