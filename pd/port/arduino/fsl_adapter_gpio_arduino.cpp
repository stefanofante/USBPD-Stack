#include "fsl_adapter_gpio.h"

#if defined(PD_CONFIG_TARGET_ARDUINO) && (PD_CONFIG_TARGET_ARDUINO)

#include <Arduino.h>
#include <stdlib.h>

struct _hal_gpio_handle
{
    int pin;
    hal_gpio_direction_t direction;
    hal_gpio_interrupt_trigger_t trigger;
    hal_gpio_callback_t callback;
    void *callbackParam;
    bool callbackAttached;
    int8_t interruptSlot;
};

namespace
{
constexpr uint8_t kMaxInterruptSlots = 8;

struct InterruptSlot
{
    _hal_gpio_handle *handle;
};

InterruptSlot s_interruptSlots[kMaxInterruptSlots] = {};

#define DECLARE_DISPATCHER(index)                               \
    static void PD_GpioIsrDispatcher##index(void)               \
    {                                                           \
        InterruptSlot &slot = s_interruptSlots[index];          \
        if ((slot.handle != nullptr) &&                         \
            (slot.handle->callback != nullptr))                 \
        {                                                       \
            slot.handle->callback(slot.handle->callbackParam);  \
        }                                                       \
    }

DECLARE_DISPATCHER(0)
DECLARE_DISPATCHER(1)
DECLARE_DISPATCHER(2)
DECLARE_DISPATCHER(3)
DECLARE_DISPATCHER(4)
DECLARE_DISPATCHER(5)
DECLARE_DISPATCHER(6)
DECLARE_DISPATCHER(7)

static void (*const s_dispatchers[kMaxInterruptSlots])(void) = {
    PD_GpioIsrDispatcher0,
    PD_GpioIsrDispatcher1,
    PD_GpioIsrDispatcher2,
    PD_GpioIsrDispatcher3,
    PD_GpioIsrDispatcher4,
    PD_GpioIsrDispatcher5,
    PD_GpioIsrDispatcher6,
    PD_GpioIsrDispatcher7,
};

hal_gpio_status_t MapTriggerToArduino(hal_gpio_interrupt_trigger_t trigger, int &mode)
{
    switch (trigger)
    {
        case kHAL_GpioInterruptDisable:
            mode = 0;
            return kStatus_HAL_GpioSuccess;
        case kHAL_GpioInterruptRisingEdge:
            mode = RISING;
            return kStatus_HAL_GpioSuccess;
        case kHAL_GpioInterruptFallingEdge:
            mode = FALLING;
            return kStatus_HAL_GpioSuccess;
        case kHAL_GpioInterruptEitherEdge:
            mode = CHANGE;
            return kStatus_HAL_GpioSuccess;
        case kHAL_GpioInterruptLogicOne:
            mode = HIGH;
            return kStatus_HAL_GpioSuccess;
        case kHAL_GpioInterruptLogicZero:
            mode = LOW;
            return kStatus_HAL_GpioSuccess;
        default:
            return kStatus_HAL_GpioError;
    }
}

int8_t AllocateInterruptSlot(_hal_gpio_handle *handle)
{
    for (uint8_t i = 0; i < kMaxInterruptSlots; ++i)
    {
        if (s_interruptSlots[i].handle == nullptr)
        {
            s_interruptSlots[i].handle = handle;
            return static_cast<int8_t>(i);
        }
    }
    return -1;
}

void FreeInterruptSlot(int8_t slotIndex)
{
    if ((slotIndex >= 0) && (slotIndex < static_cast<int8_t>(kMaxInterruptSlots)))
    {
        s_interruptSlots[slotIndex].handle = nullptr;
    }
}

} // namespace

extern "C" {

hal_gpio_status_t HAL_GpioInit(hal_gpio_handle_t *handle, const hal_gpio_pin_config_t *config)
{
    if ((handle == NULL) || (config == NULL) || (*handle != NULL))
    {
        return kStatus_HAL_GpioError;
    }

    _hal_gpio_handle *obj = static_cast<_hal_gpio_handle *>(malloc(sizeof(_hal_gpio_handle)));
    if (obj == NULL)
    {
        return kStatus_HAL_GpioError;
    }

    obj->pin              = config->pin;
    obj->direction        = config->direction;
    obj->trigger          = kHAL_GpioInterruptDisable;
    obj->callback         = NULL;
    obj->callbackParam    = NULL;
    obj->callbackAttached = false;
    obj->interruptSlot    = -1;

    int mode = OUTPUT;
    if (config->direction == kHAL_GpioDirectionOut)
    {
        mode = OUTPUT;
    }
    else
    {
        switch (config->pullMode)
        {
            case kHAL_GpioPullUp:
#ifdef INPUT_PULLUP
                mode = INPUT_PULLUP;
#else
                mode = INPUT;
#endif
                break;
            case kHAL_GpioPullDown:
#ifdef INPUT_PULLDOWN
                mode = INPUT_PULLDOWN;
#else
                mode = INPUT;
#endif
                break;
            case kHAL_GpioNoPull:
            default:
                mode = INPUT;
                break;
        }
    }

    pinMode(obj->pin, mode);
    if (config->direction == kHAL_GpioDirectionOut)
    {
        digitalWrite(obj->pin, config->level ? HIGH : LOW);
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

    _hal_gpio_handle *obj = *handle;
    if (obj->callbackAttached)
    {
        detachInterrupt(digitalPinToInterrupt(obj->pin));
        obj->callbackAttached = false;
    }
    FreeInterruptSlot(obj->interruptSlot);
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

    _hal_gpio_handle *obj = *handle;

    if ((callback == NULL) && (obj->callbackAttached))
    {
        int interruptNumber = digitalPinToInterrupt(obj->pin);
        if (interruptNumber != NOT_AN_INTERRUPT)
        {
            detachInterrupt(interruptNumber);
        }
        obj->callbackAttached = false;
    }

    if ((callback == NULL) && (obj->interruptSlot >= 0))
    {
        FreeInterruptSlot(obj->interruptSlot);
        obj->interruptSlot = -1;
    }

    obj->callback      = callback;
    obj->callbackParam = param;

    if ((callback != NULL) && (obj->interruptSlot < 0))
    {
        int8_t slot = AllocateInterruptSlot(obj);
        if (slot < 0)
        {
            obj->callback      = NULL;
            obj->callbackParam = NULL;
            return kStatus_HAL_GpioError;
        }
        obj->interruptSlot = slot;
    }

    return kStatus_HAL_GpioSuccess;
}

hal_gpio_status_t HAL_GpioSetTriggerMode(hal_gpio_handle_t *handle, hal_gpio_interrupt_trigger_t triggerMode)
{
    if ((handle == NULL) || (*handle == NULL))
    {
        return kStatus_HAL_GpioError;
    }

    _hal_gpio_handle *obj = *handle;
    int interruptMode      = 0;
    if (MapTriggerToArduino(triggerMode, interruptMode) != kStatus_HAL_GpioSuccess)
    {
        return kStatus_HAL_GpioError;
    }

    int interruptNumber = digitalPinToInterrupt(obj->pin);
    if (interruptNumber == NOT_AN_INTERRUPT)
    {
        return kStatus_HAL_GpioError;
    }

    if ((obj->callback != NULL) && (triggerMode != kHAL_GpioInterruptDisable) && (obj->interruptSlot >= 0))
    {
        attachInterrupt(interruptNumber, s_dispatchers[obj->interruptSlot], interruptMode);
        obj->callbackAttached = true;
    }
    else
    {
        detachInterrupt(interruptNumber);
        obj->callbackAttached = false;
    }

    obj->trigger = triggerMode;
    return kStatus_HAL_GpioSuccess;
}

hal_gpio_status_t HAL_GpioWakeUpSetting(hal_gpio_handle_t *handle, uint8_t enable)
{
    (void)handle;
    (void)enable;
    return kStatus_HAL_GpioSuccess;
}

hal_gpio_status_t HAL_GpioGetInput(hal_gpio_handle_t *handle, uint8_t *value)
{
    if ((handle == NULL) || (*handle == NULL) || (value == NULL))
    {
        return kStatus_HAL_GpioError;
    }

    _hal_gpio_handle *obj = *handle;
    *value                = static_cast<uint8_t>(digitalRead(obj->pin));
    return kStatus_HAL_GpioSuccess;
}

} /* extern "C" */

#endif /* PD_CONFIG_TARGET_ARDUINO */
