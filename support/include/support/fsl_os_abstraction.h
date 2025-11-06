#ifndef FSL_OS_ABSTRACTION_H
#define FSL_OS_ABSTRACTION_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum _osa_status
{
    KOSA_StatusSuccess = 0,
    KOSA_StatusError,
    KOSA_StatusTimeout,
} osa_status_t;

#if defined(PD_CONFIG_TARGET_ESP32S3) && (PD_CONFIG_TARGET_ESP32S3)
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

typedef EventGroupHandle_t osa_event_handle_t;
typedef osa_event_handle_t osa_event_handle_storage_t[1];
#define OSA_EVENT_HANDLE_DEFINE(name) osa_event_handle_storage_t name

typedef SemaphoreHandle_t osa_mutex_handle_t;
typedef osa_mutex_handle_t osa_mutex_handle_storage_t[1];
#define OSA_MUTEX_HANDLE_DEFINE(name) osa_mutex_handle_storage_t name

#define OSA_SR_ALLOC()
#define OSA_ENTER_CRITICAL() taskENTER_CRITICAL()
#define OSA_EXIT_CRITICAL() taskEXIT_CRITICAL()

#elif defined(PD_CONFIG_TARGET_ARDUINO) && (PD_CONFIG_TARGET_ARDUINO)

typedef struct _osa_event *osa_event_handle_t;
typedef osa_event_handle_t osa_event_handle_storage_t[1];
#define OSA_EVENT_HANDLE_DEFINE(name) osa_event_handle_storage_t name

typedef struct _osa_mutex *osa_mutex_handle_t;
typedef osa_mutex_handle_t osa_mutex_handle_storage_t[1];
#define OSA_MUTEX_HANDLE_DEFINE(name) osa_mutex_handle_storage_t name

void PD_PortArduino_EnterCritical(void);
void PD_PortArduino_ExitCritical(void);

#define OSA_SR_ALLOC()
#define OSA_ENTER_CRITICAL() PD_PortArduino_EnterCritical()
#define OSA_EXIT_CRITICAL() PD_PortArduino_ExitCritical()

#elif defined(PD_CONFIG_TARGET_STM32) && (PD_CONFIG_TARGET_STM32)
#include "FreeRTOS.h"
#include "event_groups.h"
#include "semphr.h"
#include "task.h"

typedef EventGroupHandle_t osa_event_handle_t;
typedef osa_event_handle_t osa_event_handle_storage_t[1];
#define OSA_EVENT_HANDLE_DEFINE(name) osa_event_handle_storage_t name

typedef SemaphoreHandle_t osa_mutex_handle_t;
typedef osa_mutex_handle_t osa_mutex_handle_storage_t[1];
#define OSA_MUTEX_HANDLE_DEFINE(name) osa_mutex_handle_storage_t name

#define OSA_SR_ALLOC()
#define OSA_ENTER_CRITICAL() taskENTER_CRITICAL()
#define OSA_EXIT_CRITICAL() taskEXIT_CRITICAL()

#else
#error "Unsupported OS abstraction target."
#endif

#define osaWaitForever_c (0xFFFFFFFFU)

osa_status_t OSA_EventCreate(osa_event_handle_t *event, uint8_t autoClear);
osa_status_t OSA_EventDestroy(osa_event_handle_t *event);
osa_status_t OSA_EventSet(osa_event_handle_t *event, uint32_t flags);
osa_status_t OSA_EventClear(osa_event_handle_t *event, uint32_t flags);
osa_status_t OSA_EventGet(osa_event_handle_t *event, uint32_t mask, uint32_t *flags);
osa_status_t OSA_EventWait(osa_event_handle_t *event,
                           uint32_t flagsToWait,
                           uint8_t clearOnExit,
                           uint32_t timeout,
                           uint32_t *setFlags);

osa_status_t OSA_MutexCreate(osa_mutex_handle_t *mutex);
osa_status_t OSA_MutexDestroy(osa_mutex_handle_t *mutex);
osa_status_t OSA_MutexLock(osa_mutex_handle_t *mutex, uint32_t timeout);
osa_status_t OSA_MutexUnlock(osa_mutex_handle_t *mutex);

#ifdef __cplusplus
}
#endif

#endif /* FSL_OS_ABSTRACTION_H */
