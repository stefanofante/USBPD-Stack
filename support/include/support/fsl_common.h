/**
 * @file fsl_common.h
 * @brief Common macros and type definitions for HAL adapters
 * 
 * @details Provides platform-independent macros, weak function attributes,
 *          and common utility macros (ARRAY_SIZE, MIN, MAX, alignment).
 * 
 * @author Stefano Fante (STLINE SRL)
 * @date 2025
 */

#ifndef FSL_COMMON_H
#define FSL_COMMON_H

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Weak function attribute (allows overriding by user code)
 */
#define USB_WEAK_FUN __attribute__((weak))

/**
 * @brief Global scope marker (reserved for future use)
 */
#define USB_GLOBAL

/**
 * @brief Calculate number of elements in an array
 * 
 * @param x Array variable
 * @return Number of elements
 */
#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

/**
 * @brief Return minimum of two values
 * 
 * @param a First value
 * @param b Second value
 * @return Minimum value
 */
#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

/**
 * @brief Return maximum of two values
 * 
 * @param a First value
 * @param b Second value
 * @return Maximum value
 */
#ifndef MAX
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

/**
 * @brief Align variable to x-byte boundary
 * 
 * @param x Alignment boundary (must be power of 2)
 */
#define SDK_ALIGN(x) __attribute__((aligned(x)))

/**
 * @brief FreeRTOS identifier constant
 */
#define FSL_RTOS_FREE_RTOS (1)

/**
 * @brief Auto-detect FreeRTOS on ESP32-S3 and STM32 platforms
 */
#if (defined(PD_CONFIG_TARGET_ESP32S3) && (PD_CONFIG_TARGET_ESP32S3)) || \
	(defined(PD_CONFIG_TARGET_STM32) && (PD_CONFIG_TARGET_STM32))
#ifndef SDK_OS_FREE_RTOS
#define SDK_OS_FREE_RTOS (1)
#endif
#endif

/**
 * @brief Generic status code type
 */
typedef int32_t status_t;

#ifdef __cplusplus
}
#endif

#endif /* FSL_COMMON_H */
