#include "usb_pd_port_esp32s3.h"

#if defined(PD_CONFIG_TARGET_ESP32S3) && (PD_CONFIG_TARGET_ESP32S3)

#include <string.h>

#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "usb_pd_timer.h"

extern void PD_TimerIsrFunction(pd_handle pdHandle);

static StaticTimer_t s_timerStorage;
static TimerHandle_t s_timerHandle;
static pd_instance_t *s_registeredInstances[PD_CONFIG_MAX_PORT];

static void PD_PortEsp32S3_TimerCallback(TimerHandle_t timer)
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

static void PD_PortEsp32S3_EnsureTimer(void)
{
    if (s_timerHandle == NULL)
    {
        s_timerHandle = xTimerCreateStatic("pd_tick",
                                           pdMS_TO_TICKS(1),
                                           pdTRUE,
                                           NULL,
                                           PD_PortEsp32S3_TimerCallback,
                                           &s_timerStorage);
    }
}

void PD_PortEsp32S3_RegisterInstance(pd_instance_t *instance)
{
    if (instance == NULL)
    {
        return;
    }

    PD_PortEsp32S3_EnsureTimer();

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

void PD_PortEsp32S3_UnregisterInstance(pd_instance_t *instance)
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

#endif /* PD_CONFIG_TARGET_ESP32S3 */

void PD_WaitUsec(uint32_t us)
{
    esp_rom_delay_us(us);
}
