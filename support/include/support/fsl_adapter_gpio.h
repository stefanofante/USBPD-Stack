/**
 * @file fsl_adapter_gpio.h
 * @brief Platform-independent GPIO HAL abstraction layer
 * 
 * @details Provides a unified GPIO interface across STM32, ESP32-S3, and Arduino
 *          platforms. Supports digital I/O and edge/level interrupts.
 * 
 * @author Stefano Fante (STLINE SRL)
 * @date 2025
 */

#ifndef FSL_ADAPTER_GPIO_H
#define FSL_ADAPTER_GPIO_H

#include <stdbool.h>
#include <stdint.h>
#include "support/fsl_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief GPIO pin direction enumeration
 */
typedef enum _hal_gpio_direction
{
    kHAL_GpioDirectionIn,   /**< Input direction */
    kHAL_GpioDirectionOut,  /**< Output direction */
} hal_gpio_direction_t;

/**
 * @brief GPIO operation status codes
 */
typedef enum _hal_gpio_status
{
    kStatus_HAL_GpioSuccess = 0,  /**< Operation succeeded */
    kStatus_HAL_GpioError,        /**< Operation failed */
} hal_gpio_status_t;

/**
 * @brief GPIO interrupt trigger modes
 */
typedef enum _hal_gpio_interrupt_trigger
{
    kHAL_GpioInterruptDisable,      /**< Disable interrupt */
    kHAL_GpioInterruptRisingEdge,   /**< Trigger on rising edge */
    kHAL_GpioInterruptFallingEdge,  /**< Trigger on falling edge */
    kHAL_GpioInterruptEitherEdge,   /**< Trigger on both edges */
    kHAL_GpioInterruptLogicOne,     /**< Trigger on high level */
    kHAL_GpioInterruptLogicZero,    /**< Trigger on low level */
} hal_gpio_interrupt_trigger_t;

/**
 * @brief GPIO pull resistor configuration
 */
typedef enum _hal_gpio_pull_mode
{
    kHAL_GpioNoPull,    /**< No pull resistor */
    kHAL_GpioPullUp,    /**< Pull-up resistor enabled */
    kHAL_GpioPullDown,  /**< Pull-down resistor enabled */
} hal_gpio_pull_mode_t;

/**
 * @brief GPIO pin configuration structure
 */
typedef struct _hal_gpio_pin_config
{
    hal_gpio_direction_t direction;  /**< Pin direction (input or output) */
    uint8_t port;                    /**< Port number (platform-specific) */
    uint8_t pin;                     /**< Pin number within port */
    uint8_t level;                   /**< Initial output level (0=low, 1=high) */
    hal_gpio_pull_mode_t pullMode;   /**< Pull resistor configuration */
    uint32_t intrFlags;              /**< Platform-specific flags (e.g., STM32 passes pd_phy_stm32_config_t) */
} hal_gpio_pin_config_t;

/**
 * @brief GPIO interrupt callback function type
 * 
 * @param[in] callbackParam User-defined parameter passed during callback installation
 */
typedef void (*hal_gpio_callback_t)(void *callbackParam);

/** @brief Opaque GPIO handle type */
typedef struct _hal_gpio_handle *hal_gpio_handle_t;

/** @brief GPIO handle storage array type */
typedef hal_gpio_handle_t hal_gpio_handle_storage_t[1];

/**
 * @brief Macro to define a GPIO handle storage variable
 * 
 * @param name Variable name for the handle storage
 */
#define GPIO_HANDLE_DEFINE(name) hal_gpio_handle_storage_t name

/**
 * @brief Initialize a GPIO pin
 * 
 * @param[in,out] handle Pointer to GPIO handle (will be allocated)
 * @param[in] config Pin configuration parameters
 * @return kStatus_HAL_GpioSuccess on success, kStatus_HAL_GpioError otherwise
 */
hal_gpio_status_t HAL_GpioInit(hal_gpio_handle_t *handle, const hal_gpio_pin_config_t *config);

/**
 * @brief Deinitialize a GPIO pin
 * 
 * @param[in,out] handle Pointer to GPIO handle (will be freed and set to NULL)
 * @return kStatus_HAL_GpioSuccess on success, kStatus_HAL_GpioError otherwise
 */
hal_gpio_status_t HAL_GpioDeinit(hal_gpio_handle_t *handle);

/**
 * @brief Install an interrupt callback for a GPIO pin
 * 
 * @param[in] handle Pointer to GPIO handle
 * @param[in] callback Callback function to invoke on interrupt
 * @param[in] param User parameter passed to callback
 * @return kStatus_HAL_GpioSuccess on success, kStatus_HAL_GpioError otherwise
 */
hal_gpio_status_t HAL_GpioInstallCallback(hal_gpio_handle_t *handle, hal_gpio_callback_t callback, void *param);

/**
 * @brief Set GPIO interrupt trigger mode
 * 
 * @param[in] handle Pointer to GPIO handle
 * @param[in] triggerMode Interrupt trigger mode
 * @return kStatus_HAL_GpioSuccess on success, kStatus_HAL_GpioError otherwise
 */
hal_gpio_status_t HAL_GpioSetTriggerMode(hal_gpio_handle_t *handle, hal_gpio_interrupt_trigger_t triggerMode);

/**
 * @brief Configure wake-up functionality for a GPIO pin
 * 
 * @param[in] handle Pointer to GPIO handle
 * @param[in] enable Enable (1) or disable (0) wake-up
 * @return kStatus_HAL_GpioSuccess on success, kStatus_HAL_GpioError otherwise
 * 
 * @note Not all platforms support wake-up functionality
 */
hal_gpio_status_t HAL_GpioWakeUpSetting(hal_gpio_handle_t *handle, uint8_t enable);

/**
 * @brief Read the current logic level of a GPIO input
 * 
 * @param[in] handle Pointer to GPIO handle
 * @param[out] value Pointer to receive the value (0 or 1)
 * @return kStatus_HAL_GpioSuccess on success, kStatus_HAL_GpioError otherwise
 */
hal_gpio_status_t HAL_GpioGetInput(hal_gpio_handle_t *handle, uint8_t *value);

#ifdef __cplusplus
}
#endif

#endif /* FSL_ADAPTER_GPIO_H */
