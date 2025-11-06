#include "support/fsl_os_abstraction.h"

#if defined(PD_CONFIG_TARGET_ARDUINO) && (PD_CONFIG_TARGET_ARDUINO)

#include <Arduino.h>
#include <stdlib.h>

struct _osa_event
{
    volatile uint32_t flags;
    uint8_t autoClear;
};

struct _osa_mutex
{
    volatile bool locked;
};

extern "C" {

ota_status_t OSA_EventCreate(osa_event_handle_t *event, uint8_t autoClear)
{
    if ((event == NULL) || (*event != NULL))
    {
        return KOSA_StatusError;
    }

    osa_event_handle_t handle = static_cast<osa_event_handle_t>(malloc(sizeof(*handle)));
    if (handle == NULL)
    {
        return KOSA_StatusError;
    }

    handle->flags     = 0;
    handle->autoClear = autoClear;
    *event            = handle;
    return KOSA_StatusSuccess;
}

ota_status_t OSA_EventDestroy(osa_event_handle_t *event)
{
    if ((event == NULL) || (*event == NULL))
    {
        return KOSA_StatusError;
    }

    free(*event);
    *event = NULL;
    return KOSA_StatusSuccess;
}

ota_status_t OSA_EventSet(osa_event_handle_t *event, uint32_t flags)
{
    if ((event == NULL) || (*event == NULL))
    {
        return KOSA_StatusError;
    }

    noInterrupts();
    (*event)->flags |= flags;
    interrupts();
    return KOSA_StatusSuccess;
}

ota_status_t OSA_EventClear(osa_event_handle_t *event, uint32_t flags)
{
    if ((event == NULL) || (*event == NULL))
    {
        return KOSA_StatusError;
    }

    noInterrupts();
    (*event)->flags &= ~flags;
    interrupts();
    return KOSA_StatusSuccess;
}

ota_status_t OSA_EventGet(osa_event_handle_t *event, uint32_t mask, uint32_t *flags)
{
    if ((event == NULL) || (*event == NULL) || (flags == NULL))
    {
        return KOSA_StatusError;
    }

    *flags = (*event)->flags & mask;
    return KOSA_StatusSuccess;
}

ota_status_t OSA_EventWait(osa_event_handle_t *event,
                           uint32_t flagsToWait,
                           uint8_t clearOnExit,
                           uint32_t timeout,
                           uint32_t *setFlags)
{
    if ((event == NULL) || (*event == NULL))
    {
        return KOSA_StatusError;
    }

    uint32_t startTick = millis();

    for (;;)
    {
        uint32_t currentFlags = (*event)->flags & flagsToWait;
        if (currentFlags != 0U)
        {
            if ((clearOnExit != 0U) || ((*event)->autoClear != 0U))
            {
                noInterrupts();
                (*event)->flags &= ~flagsToWait;
                interrupts();
            }
            if (setFlags != NULL)
            {
                *setFlags = currentFlags;
            }
            return KOSA_StatusSuccess;
        }

        if (timeout != osaWaitForever_c)
        {
            uint32_t elapsed = millis() - startTick;
            if (elapsed >= timeout)
            {
                return KOSA_StatusTimeout;
            }
        }

        yield();
    }
}

ota_status_t OSA_MutexCreate(osa_mutex_handle_t *mutex)
{
    if ((mutex == NULL) || (*mutex != NULL))
    {
        return KOSA_StatusError;
    }

    osa_mutex_handle_t handle = static_cast<osa_mutex_handle_t>(malloc(sizeof(*handle)));
    if (handle == NULL)
    {
        return KOSA_StatusError;
    }

    handle->locked = false;
    *mutex         = handle;
    return KOSA_StatusSuccess;
}

ota_status_t OSA_MutexDestroy(osa_mutex_handle_t *mutex)
{
    if ((mutex == NULL) || (*mutex == NULL))
    {
        return KOSA_StatusError;
    }

    free(*mutex);
    *mutex = NULL;
    return KOSA_StatusSuccess;
}

ota_status_t OSA_MutexLock(osa_mutex_handle_t *mutex, uint32_t timeout)
{
    if ((mutex == NULL) || (*mutex == NULL))
    {
        return KOSA_StatusError;
    }

    uint32_t startTick = millis();

    for (;;)
    {
        noInterrupts();
        bool available = !(*mutex)->locked;
        if (available)
        {
            (*mutex)->locked = true;
            interrupts();
            return KOSA_StatusSuccess;
        }
        interrupts();

        if (timeout != osaWaitForever_c)
        {
            uint32_t elapsed = millis() - startTick;
            if (elapsed >= timeout)
            {
                return KOSA_StatusTimeout;
            }
        }

        yield();
    }
}

ota_status_t OSA_MutexUnlock(osa_mutex_handle_t *mutex)
{
    if ((mutex == NULL) || (*mutex == NULL))
    {
        return KOSA_StatusError;
    }

    noInterrupts();
    (*mutex)->locked = false;
    interrupts();
    return KOSA_StatusSuccess;
}

void PD_PortArduino_EnterCritical(void)
{
    noInterrupts();
}

void PD_PortArduino_ExitCritical(void)
{
    interrupts();
}

} /* extern "C" */

#endif /* PD_CONFIG_TARGET_ARDUINO */
