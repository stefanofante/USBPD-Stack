/**
 * @file fsl_adapter_gpio_esp32.c
 * @brief GPIO HAL adapter for ESP32-S3 using ESP-IDF
 * 
 * @details Implements platform-independent GPIO abstraction on ESP32-S3.
 *          Handles ISR service installation and interrupt callbacks.
 * 
 * @author Stefano Fante (STLINE SRL)
 * @date 2025
 */

#include "support/fsl_adapter_gpio.h"

#if defined(PD_CONFIG_TARGET_ESP32S3) && (PD_CONFIG_TARGET_ESP32S3)

#include <stdlib.h>

#include "driver/gpio.h"
#include "esp_err.h"

/**
 * @brief Internal GPIO handle structure for ESP32-S3
 */
typedef struct _hal_gpio_handle
{
    gpio_num_t pin;                          /**< ESP32 GPIO pin number */
    hal_gpio_direction_t direction;          /**< Input or output direction */
    hal_gpio_interrupt_trigger_t trigger;    /**< Interrupt trigger type */
    hal_gpio_callback_t callback;            /**< User callback for interrupts */
    void *callbackParam;                     /**< User parameter for callback */
    uint32_t isrFlags;                       /**< ESP-IDF ISR allocation flags */
    bool isrAttached;                        /**< True if ISR handler is attached */
} hal_gpio_handle_obj_t;

/** @brief Global flag tracking ISR service installation */
static bool s_isrServiceInstalled = false;

/**
 * @brief Map HAL interrupt trigger to ESP-IDF gpio_int_type_t
 * 
 * @param[in] trigger HAL interrupt trigger enumeration
 * @return ESP-IDF GPIO interrupt type
 */
static gpio_int_type_t PD_GpioMapTrigger(hal_gpio_interrupt_trigger_t trigger)
{
    switch (trigger)
    {
        case kHAL_GpioInterruptRisingEdge:
            return GPIO_INTR_POSEDGE;
        case kHAL_GpioInterruptFallingEdge:
            return GPIO_INTR_NEGEDGE;
        case kHAL_GpioInterruptEitherEdge:
            return GPIO_INTR_ANYEDGE;
        case kHAL_GpioInterruptLogicOne:
            return GPIO_INTR_HIGH_LEVEL;
        case kHAL_GpioInterruptLogicZero:
            return GPIO_INTR_LOW_LEVEL;
        case kHAL_GpioInterruptDisable:
        default:
            return GPIO_INTR_DISABLE;
    }
}

/**
 * @brief Ensure GPIO ISR service is installed
 * 
 * @param[in] intrFlags ESP-IDF ISR allocation flags
 * 
 * @details Calls gpio_install_isr_service() once per process. Ignores
 *          ESP_ERR_INVALID_STATE if already installed.
 */
static void PD_GpioEnsureIsrService(uint32_t intrFlags)
{
    if (!s_isrServiceInstalled)
    {
        esp_err_t err = gpio_install_isr_service(intrFlags);
        if ((err == ESP_OK) || (err == ESP_ERR_INVALID_STATE))
        {
            s_isrServiceInstalled = true;
        }
    }
}

/**
 * @brief ISR thunk to dispatch GPIO interrupts to user callbacks
 * 
 * @param[in] arg Pointer to hal_gpio_handle_obj_t
 */
static void PD_GpioIsrThunk(void *arg)
{
    hal_gpio_handle_obj_t *handle = (hal_gpio_handle_obj_t *)arg;
    if ((handle != NULL) && (handle->callback != NULL))
    {
        handle->callback(handle->callbackParam);
    }
}

/**
 * @brief Apply pull-up/pull-down configuration to a GPIO pin
 * 
 * @param[in] pin ESP32 GPIO pin number
 * @param[in] mode HAL pull mode enumeration
 */
static void PD_GpioApplyPullMode(gpio_num_t pin, hal_gpio_pull_mode_t mode)
{
    switch (mode)
    {
        case kHAL_GpioPullUp:
            gpio_pullup_en(pin);
            gpio_pulldown_dis(pin);
            break;
        case kHAL_GpioPullDown:
            gpio_pulldown_en(pin);
            gpio_pullup_dis(pin);
            break;
        case kHAL_GpioNoPull:
        default:
            gpio_pullup_dis(pin);
            gpio_pulldown_dis(pin);
            break;
    }
}

/**
 * @brief Initialize a GPIO pin
 * 
 * @param[in,out] handle Pointer to GPIO handle (will be allocated)
 * @param[in] config Pin configuration
 * 
 * @return kStatus_HAL_GpioSuccess on success, kStatus_HAL_GpioError otherwise
 * 
 * @details Calls ESP-IDF gpio_config() and sets initial output level.
 */
hal_gpio_status_t HAL_GpioInit(hal_gpio_handle_t *handle, const hal_gpio_pin_config_t *config)
{
    if ((handle == NULL) || (config == NULL) || (*handle != NULL))
    {
        return kStatus_HAL_GpioError;
    }

    hal_gpio_handle_obj_t *obj = calloc(1, sizeof(*obj));
    if (obj == NULL)
    {
        return kStatus_HAL_GpioError;
    }

    obj->pin       = (gpio_num_t)config->pin;
    obj->direction = config->direction;
    obj->trigger   = kHAL_GpioInterruptDisable;
    obj->isrFlags  = config->intrFlags;

    gpio_config_t gpioCfg = {
        .pin_bit_mask = 1ULL << obj->pin,
        .mode         = (config->direction == kHAL_GpioDirectionIn) ? GPIO_MODE_INPUT : GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };

    if (gpio_config(&gpioCfg) != ESP_OK)
    {
        free(obj);
        return kStatus_HAL_GpioError;
    }

    PD_GpioApplyPullMode(obj->pin, config->pullMode);

    if (config->direction == kHAL_GpioDirectionOut)
    {
        gpio_set_level(obj->pin, config->level ? 1 : 0);
    }

    *handle = obj;
    return kStatus_HAL_GpioSuccess;
}

/**
 * @brief Deinitialize a GPIO pin
 * 
 * @param[in,out] handle Pointer to GPIO handle (will be freed)
 * @return kStatus_HAL_GpioSuccess on success, kStatus_HAL_GpioError otherwise
 * 
 * @details Removes ISR handler and calls gpio_reset_pin().
 */
hal_gpio_status_t HAL_GpioDeinit(hal_gpio_handle_t *handle)
{
    if ((handle == NULL) || (*handle == NULL))
    {
        return kStatus_HAL_GpioError;
    }

    hal_gpio_handle_obj_t *obj = *handle;
    if (obj->isrAttached)
    {
        gpio_isr_handler_remove(obj->pin);
        obj->isrAttached = false;
    }

    gpio_reset_pin(obj->pin);
    free(obj);
    *handle = NULL;

    return kStatus_HAL_GpioSuccess;
}

/**
 * @brief Install an interrupt callback for a GPIO pin
 * 
 * @param[in] handle Pointer to GPIO handle
 * @param[in] callback Callback function
 * @param[in] param User parameter
 * 
 * @return kStatus_HAL_GpioSuccess on success, kStatus_HAL_GpioError otherwise
 * 
 * @details Ensures ISR service is installed and adds the ISR handler.
 */
hal_gpio_status_t HAL_GpioInstallCallback(hal_gpio_handle_t *handle, hal_gpio_callback_t callback, void *param)
{
    if ((handle == NULL) || (*handle == NULL))
    {
        return kStatus_HAL_GpioError;
    }

    hal_gpio_handle_obj_t *obj = *handle;
    obj->callback      = callback;
    obj->callbackParam = param;

    PD_GpioEnsureIsrService(obj->isrFlags);
    if (!s_isrServiceInstalled)
    {
        return kStatus_HAL_GpioError;
    }

    if (!obj->isrAttached)
    {
        if (gpio_isr_handler_add(obj->pin, PD_GpioIsrThunk, obj) != ESP_OK)
        {
            return kStatus_HAL_GpioError;
        }
        obj->isrAttached = true;
    }

    return kStatus_HAL_GpioSuccess;
}

/**
 * @brief Set GPIO interrupt trigger mode
 * 
 * @param[in] handle Pointer to GPIO handle
 * @param[in] triggerMode Trigger mode (rising, falling, both, level)
 * 
 * @return kStatus_HAL_GpioSuccess on success, kStatus_HAL_GpioError otherwise
 */
hal_gpio_status_t HAL_GpioSetTriggerMode(hal_gpio_handle_t *handle, hal_gpio_interrupt_trigger_t triggerMode)
{
    if ((handle == NULL) || (*handle == NULL))
    {
        return kStatus_HAL_GpioError;
    }

    hal_gpio_handle_obj_t *obj = *handle;
    gpio_int_type_t intrType   = PD_GpioMapTrigger(triggerMode);

    if (intrType == GPIO_INTR_DISABLE)
    {
        gpio_intr_disable(obj->pin);
    }
    else
    {
        gpio_set_intr_type(obj->pin, intrType);
        gpio_intr_enable(obj->pin);
    }

    obj->trigger = triggerMode;
    return kStatus_HAL_GpioSuccess;
}

/**
 * @brief Configure wake-up functionality on ESP32-S3
 * 
 * @param[in] handle Pointer to GPIO handle
 * @param[in] enable Enable (1) or disable (0) wake-up
 * 
 * @return kStatus_HAL_GpioSuccess on success, kStatus_HAL_GpioError otherwise
 * 
 * @details Calls gpio_wakeup_enable() or gpio_wakeup_disable().
 */
hal_gpio_status_t HAL_GpioWakeUpSetting(hal_gpio_handle_t *handle, uint8_t enable)
{
    if ((handle == NULL) || (*handle == NULL))
    {
        return kStatus_HAL_GpioError;
    }

    hal_gpio_handle_obj_t *obj = *handle;
    if (enable != 0U)
    {
        gpio_wakeup_enable(obj->pin, PD_GpioMapTrigger(obj->trigger));
    }
    else
    {
        gpio_wakeup_disable(obj->pin);
    }

    return kStatus_HAL_GpioSuccess;
}

/**
 * @brief Read the current logic level of a GPIO input
 * 
 * @param[in] handle Pointer to GPIO handle
 * @param[out] value Pointer to receive the value (0 or 1)
 * 
 * @return kStatus_HAL_GpioSuccess on success, kStatus_HAL_GpioError otherwise
 */
hal_gpio_status_t HAL_GpioGetInput(hal_gpio_handle_t *handle, uint8_t *value)
{
    if ((handle == NULL) || (*handle == NULL) || (value == NULL))
    {
        return kStatus_HAL_GpioError;
    }

    hal_gpio_handle_obj_t *obj = *handle;
    *value                     = (uint8_t)gpio_get_level(obj->pin);
    return kStatus_HAL_GpioSuccess;
}

#endif /* PD_CONFIG_TARGET_ESP32S3 */
