#include "support/fsl_os_abstraction.h"

#if defined(PD_CONFIG_TARGET_ESP32S3) && (PD_CONFIG_TARGET_ESP32S3)

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

osa_status_t OSA_EventCreate(osa_event_handle_t *event, uint8_t autoClear)
{
    (void)autoClear;
    if ((event == NULL) || (*event != NULL))
    {
        return KOSA_StatusError;
    }

    EventGroupHandle_t handle = xEventGroupCreate();
    if (handle == NULL)
    {
        return KOSA_StatusError;
    }

    *event = handle;
    return KOSA_StatusSuccess;
}

osa_status_t OSA_EventDestroy(osa_event_handle_t *event)
{
    if ((event == NULL) || (*event == NULL))
    {
        return KOSA_StatusError;
    }

    vEventGroupDelete(*event);
    *event = NULL;
    return KOSA_StatusSuccess;
}

osa_status_t OSA_EventSet(osa_event_handle_t *event, uint32_t flags)
{
    if ((event == NULL) || (*event == NULL))
    {
        return KOSA_StatusError;
    }

    return (xEventGroupSetBits(*event, flags) != 0) ? KOSA_StatusSuccess : KOSA_StatusError;
}

osa_status_t OSA_EventClear(osa_event_handle_t *event, uint32_t flags)
{
    if ((event == NULL) || (*event == NULL))
    {
        return KOSA_StatusError;
    }

    return (xEventGroupClearBits(*event, flags) == pdPASS) ? KOSA_StatusSuccess : KOSA_StatusError;
}

osa_status_t OSA_EventGet(osa_event_handle_t *event, uint32_t mask, uint32_t *flags)
{
    if ((event == NULL) || (*event == NULL) || (flags == NULL))
    {
        return KOSA_StatusError;
    }

    EventBits_t bits = xEventGroupGetBits(*event);
    *flags           = bits & mask;
    return KOSA_StatusSuccess;
}

osa_status_t OSA_EventWait(osa_event_handle_t *event,
                           uint32_t flagsToWait,
                           uint8_t clearOnExit,
                           uint32_t timeout,
                           uint32_t *setFlags)
{
    if ((event == NULL) || (*event == NULL))
    {
        return KOSA_StatusError;
    }

    TickType_t ticks = (timeout == osaWaitForever_c) ? portMAX_DELAY : pdMS_TO_TICKS(timeout);
    EventBits_t bits = xEventGroupWaitBits(*event, flagsToWait, (clearOnExit != 0U), pdFALSE, ticks);

    if (setFlags != NULL)
    {
        *setFlags = bits & flagsToWait;
    }

    if ((bits & flagsToWait) != 0U)
    {
        return KOSA_StatusSuccess;
    }
    return KOSA_StatusTimeout;
}

osa_status_t OSA_MutexCreate(osa_mutex_handle_t *mutex)
{
    if ((mutex == NULL) || (*mutex != NULL))
    {
        return KOSA_StatusError;
    }

    SemaphoreHandle_t handle = xSemaphoreCreateMutex();
    if (handle == NULL)
    {
        return KOSA_StatusError;
    }

    *mutex = handle;
    return KOSA_StatusSuccess;
}

osa_status_t OSA_MutexDestroy(osa_mutex_handle_t *mutex)
{
    if ((mutex == NULL) || (*mutex == NULL))
    {
        return KOSA_StatusError;
    }

    vSemaphoreDelete(*mutex);
    *mutex = NULL;
    return KOSA_StatusSuccess;
}

osa_status_t OSA_MutexLock(osa_mutex_handle_t *mutex, uint32_t timeout)
{
    if ((mutex == NULL) || (*mutex == NULL))
    {
        return KOSA_StatusError;
    }

    TickType_t ticks = (timeout == osaWaitForever_c) ? portMAX_DELAY : pdMS_TO_TICKS(timeout);
    return (xSemaphoreTake(*mutex, ticks) == pdTRUE) ? KOSA_StatusSuccess : KOSA_StatusTimeout;
}

osa_status_t OSA_MutexUnlock(osa_mutex_handle_t *mutex)
{
    if ((mutex == NULL) || (*mutex == NULL))
    {
        return KOSA_StatusError;
    }

    return (xSemaphoreGive(*mutex) == pdTRUE) ? KOSA_StatusSuccess : KOSA_StatusError;
}

#endif /* PD_CONFIG_TARGET_ESP32S3 */
