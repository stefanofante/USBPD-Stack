/**
 * @file fsl_os_abstraction.h
 * @brief Operating system abstraction layer for synchronization primitives
 * 
 * @details Provides platform-independent event groups, mutexes, and critical
 *          sections. Supports FreeRTOS (ESP32-S3, STM32) and cooperative
 *          multitasking (Arduino).
 * 
 * @author Stefano Fante (STLINE SRL)
 * @date 2025
 */

#ifndef FSL_OS_ABSTRACTION_H
#define FSL_OS_ABSTRACTION_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief OS abstraction status codes
 */
typedef enum _osa_status
{
    KOSA_StatusSuccess = 0,  /**< Operation succeeded */
    KOSA_StatusError,        /**< Operation failed */
    KOSA_StatusTimeout,      /**< Operation timed out */
} osa_status_t;

#if defined(PD_CONFIG_TARGET_ESP32S3) && (PD_CONFIG_TARGET_ESP32S3)
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

/** @brief Event group handle type (FreeRTOS ESP32-S3) */
typedef EventGroupHandle_t osa_event_handle_t;
typedef osa_event_handle_t osa_event_handle_storage_t[1];

/**
 * @brief Define an event group handle storage variable
 */
#define OSA_EVENT_HANDLE_DEFINE(name) osa_event_handle_storage_t name

/** @brief Mutex handle type (FreeRTOS ESP32-S3) */
typedef SemaphoreHandle_t osa_mutex_handle_t;
typedef osa_mutex_handle_t osa_mutex_handle_storage_t[1];

/**
 * @brief Define a mutex handle storage variable
 */
#define OSA_MUTEX_HANDLE_DEFINE(name) osa_mutex_handle_storage_t name

/**
 * @brief Allocate critical section variable (no-op for FreeRTOS)
 */
#define OSA_SR_ALLOC()

/**
 * @brief Enter critical section (disable task switching)
 */
#define OSA_ENTER_CRITICAL() taskENTER_CRITICAL()

/**
 * @brief Exit critical section (re-enable task switching)
 */
#define OSA_EXIT_CRITICAL() taskEXIT_CRITICAL()

#elif defined(PD_CONFIG_TARGET_ARDUINO) && (PD_CONFIG_TARGET_ARDUINO)

/** @brief Event group handle type (Arduino cooperative) */
typedef struct _osa_event *osa_event_handle_t;
typedef osa_event_handle_t osa_event_handle_storage_t[1];

/**
 * @brief Define an event group handle storage variable
 */
#define OSA_EVENT_HANDLE_DEFINE(name) osa_event_handle_storage_t name

/** @brief Mutex handle type (Arduino cooperative) */
typedef struct _osa_mutex *osa_mutex_handle_t;
typedef osa_mutex_handle_t osa_mutex_handle_storage_t[1];

/**
 * @brief Define a mutex handle storage variable
 */
#define OSA_MUTEX_HANDLE_DEFINE(name) osa_mutex_handle_storage_t name

void PD_PortArduino_EnterCritical(void);
void PD_PortArduino_ExitCritical(void);

/**
 * @brief Allocate critical section variable (no-op for Arduino)
 */
#define OSA_SR_ALLOC()

/**
 * @brief Enter critical section (disable interrupts)
 */
#define OSA_ENTER_CRITICAL() PD_PortArduino_EnterCritical()

/**
 * @brief Exit critical section (re-enable interrupts)
 */
#define OSA_EXIT_CRITICAL() PD_PortArduino_ExitCritical()

#elif defined(PD_CONFIG_TARGET_STM32) && (PD_CONFIG_TARGET_STM32)
#include "FreeRTOS.h"
#include "event_groups.h"
#include "semphr.h"
#include "task.h"

/** @brief Event group handle type (FreeRTOS STM32) */
typedef EventGroupHandle_t osa_event_handle_t;
typedef osa_event_handle_t osa_event_handle_storage_t[1];

/**
 * @brief Define an event group handle storage variable
 */
#define OSA_EVENT_HANDLE_DEFINE(name) osa_event_handle_storage_t name

/** @brief Mutex handle type (FreeRTOS STM32) */
typedef SemaphoreHandle_t osa_mutex_handle_t;
typedef osa_mutex_handle_t osa_mutex_handle_storage_t[1];

/**
 * @brief Define a mutex handle storage variable
 */
#define OSA_MUTEX_HANDLE_DEFINE(name) osa_mutex_handle_storage_t name

/**
 * @brief Allocate critical section variable (no-op for FreeRTOS)
 */
#define OSA_SR_ALLOC()

/**
 * @brief Enter critical section (disable task switching)
 */
#define OSA_ENTER_CRITICAL() taskENTER_CRITICAL()

/**
 * @brief Exit critical section (re-enable task switching)
 */
#define OSA_EXIT_CRITICAL() taskEXIT_CRITICAL()

#else
#error "Unsupported OS abstraction target."
#endif

/**
 * @brief Wait forever constant (infinite timeout)
 */
#define osaWaitForever_c (0xFFFFFFFFU)

/**
 * @brief Create an event group
 * 
 * @param[out] event Pointer to event handle
 * @param[in] autoClear Auto-clear flags on wait (1=true, 0=false)
 * @return KOSA_StatusSuccess on success
 */
osa_status_t OSA_EventCreate(osa_event_handle_t *event, uint8_t autoClear);

/**
 * @brief Destroy an event group
 * 
 * @param[in] event Pointer to event handle
 * @return KOSA_StatusSuccess on success
 */
osa_status_t OSA_EventDestroy(osa_event_handle_t *event);

/**
 * @brief Set event flags
 * 
 * @param[in] event Pointer to event handle
 * @param[in] flags Flags to set (bitwise OR)
 * @return KOSA_StatusSuccess on success
 */
osa_status_t OSA_EventSet(osa_event_handle_t *event, uint32_t flags);

/**
 * @brief Clear event flags
 * 
 * @param[in] event Pointer to event handle
 * @param[in] flags Flags to clear (bitwise AND NOT)
 * @return KOSA_StatusSuccess on success
 */
osa_status_t OSA_EventClear(osa_event_handle_t *event, uint32_t flags);

/**
 * @brief Get current event flags (non-blocking)
 * 
 * @param[in] event Pointer to event handle
 * @param[in] mask Mask to apply to flags
 * @param[out] flags Pointer to receive current flags
 * @return KOSA_StatusSuccess on success
 */
osa_status_t OSA_EventGet(osa_event_handle_t *event, uint32_t mask, uint32_t *flags);

/**
 * @brief Wait for event flags with timeout
 * 
 * @param[in] event Pointer to event handle
 * @param[in] flagsToWait Flags to wait for (bitwise OR)
 * @param[in] clearOnExit Clear flags on exit (1=true, 0=false)
 * @param[in] timeout Timeout in milliseconds (osaWaitForever_c for infinite)
 * @param[out] setFlags Pointer to receive flags that were set
 * @return KOSA_StatusSuccess on success, KOSA_StatusTimeout on timeout
 */
osa_status_t OSA_EventWait(osa_event_handle_t *event,
                           uint32_t flagsToWait,
                           uint8_t clearOnExit,
                           uint32_t timeout,
                           uint32_t *setFlags);

/**
 * @brief Create a mutex
 * 
 * @param[out] mutex Pointer to mutex handle
 * @return KOSA_StatusSuccess on success
 */
osa_status_t OSA_MutexCreate(osa_mutex_handle_t *mutex);

/**
 * @brief Destroy a mutex
 * 
 * @param[in] mutex Pointer to mutex handle
 * @return KOSA_StatusSuccess on success
 */
osa_status_t OSA_MutexDestroy(osa_mutex_handle_t *mutex);

/**
 * @brief Lock a mutex with timeout
 * 
 * @param[in] mutex Pointer to mutex handle
 * @param[in] timeout Timeout in milliseconds (osaWaitForever_c for infinite)
 * @return KOSA_StatusSuccess on success, KOSA_StatusTimeout on timeout
 */
osa_status_t OSA_MutexLock(osa_mutex_handle_t *mutex, uint32_t timeout);

/**
 * @brief Unlock a mutex
 * 
 * @param[in] mutex Pointer to mutex handle
 * @return KOSA_StatusSuccess on success
 */
osa_status_t OSA_MutexUnlock(osa_mutex_handle_t *mutex);

#ifdef __cplusplus
}
#endif

#endif /* FSL_OS_ABSTRACTION_H */
