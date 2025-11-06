#include "fsl_adapter_gpio.h"

#if defined(PD_CONFIG_TARGET_ESP32S3) && (PD_CONFIG_TARGET_ESP32S3)

#include <stdlib.h>

#include "driver/gpio.h"
#include "esp_err.h"

typedef struct _hal_gpio_handle
{
    gpio_num_t pin;
    hal_gpio_direction_t direction;
    hal_gpio_interrupt_trigger_t trigger;
    hal_gpio_callback_t callback;
    void *callbackParam;
    uint32_t isrFlags;
    bool isrAttached;
} hal_gpio_handle_obj_t;

static bool s_isrServiceInstalled = false;

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

static void PD_GpioIsrThunk(void *arg)
{
    hal_gpio_handle_obj_t *handle = (hal_gpio_handle_obj_t *)arg;
    if ((handle != NULL) && (handle->callback != NULL))
    {
        handle->callback(handle->callbackParam);
    }
}

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
