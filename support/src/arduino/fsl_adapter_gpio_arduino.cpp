/**
 * @file fsl_adapter_gpio_arduino.cpp
 * @brief GPIO HAL adapter for Arduino platforms
 * 
 * @details Implements platform-independent GPIO abstraction using Arduino
 *          pinMode/digitalWrite/digitalRead/attachInterrupt APIs.
 * 
 * @author Stefano Fante (STLINE SRL)
 * @date 2025
 */

#include "support/fsl_adapter_gpio.h"

#if defined(PD_CONFIG_TARGET_ARDUINO) && (PD_CONFIG_TARGET_ARDUINO)

#include <Arduino.h>
#include <stdlib.h>

/**
 * @brief Internal GPIO handle structure for Arduino
 */
struct _hal_gpio_handle
{
    int pin;                             /**< Arduino pin number */
    hal_gpio_direction_t direction;      /**< Input or output direction */
    hal_gpio_interrupt_trigger_t trigger; /**< Interrupt trigger type */
    hal_gpio_callback_t callback;        /**< User callback for interrupts */
    void *callbackParam;                 /**< User parameter for callback */
    bool callbackAttached;               /**< True if interrupt is attached */
    int8_t interruptSlot;                /**< Index into global dispatcher array */
};

namespace
{
/** @brief Maximum number of concurrent GPIO interrupts */
constexpr uint8_t kMaxInterruptSlots = 8;

/**
 * @brief Interrupt slot binding handle to dispatcher
 */
struct InterruptSlot
{
    _hal_gpio_handle *handle;  /**< Handle for this slot */
};

/** @brief Global interrupt slot registry */
InterruptSlot s_interruptSlots[kMaxInterruptSlots] = {};

/**
 * @brief Generate static ISR dispatcher for a specific slot index
 */
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

/** @brief Array of function pointers for attachInterrupt() */
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

/**
 * @brief Map HAL trigger mode to Arduino interrupt mode
 * 
 * @param[in] trigger HAL interrupt trigger enumeration
 * @param[out] mode Arduino interrupt mode (RISING, FALLING, CHANGE, etc.)
 * @return kStatus_HAL_GpioSuccess on success
 */
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

/**
 * @brief Allocate a free interrupt slot
 * 
 * @param[in] handle GPIO handle to bind
 * @return Slot index (0-7) or -1 if no slots available
 */
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

/**
 * @brief Free an interrupt slot
 * 
 * @param[in] slotIndex Slot index to free
 */
void FreeInterruptSlot(int8_t slotIndex)
{
    if ((slotIndex >= 0) && (slotIndex < static_cast<int8_t>(kMaxInterruptSlots)))
    {
        s_interruptSlots[slotIndex].handle = nullptr;
    }
}

} // namespace

extern "C" {

/**
 * @brief Initialize a GPIO pin
 * 
 * @param[in,out] handle Pointer to GPIO handle (will be allocated)
 * @param[in] config Pin configuration
 * @return kStatus_HAL_GpioSuccess on success
 * 
 * @details Calls pinMode() and digitalWrite() for output pins.
 */
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

/**
 * @brief Deinitialize a GPIO pin
 * 
 * @param[in,out] handle Pointer to GPIO handle (will be freed)
 * @return kStatus_HAL_GpioSuccess on success
 * 
 * @details Calls detachInterrupt() and frees memory.
 */
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

/**
 * @brief Install an interrupt callback
 * 
 * @param[in] handle Pointer to GPIO handle
 * @param[in] callback Callback function
 * @param[in] param User parameter
 * @return kStatus_HAL_GpioSuccess on success
 * 
 * @details Allocates an interrupt slot and prepares for attachInterrupt().
 */
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

/**
 * @brief Set GPIO interrupt trigger mode
 * 
 * @param[in] handle Pointer to GPIO handle
 * @param[in] triggerMode Trigger mode (rising, falling, change, etc.)
 * @return kStatus_HAL_GpioSuccess on success
 * 
 * @details Calls attachInterrupt() or detachInterrupt() based on trigger mode.
 */
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

/**
 * @brief Configure wake-up functionality (stub for Arduino)
 * 
 * @param[in] handle Pointer to GPIO handle
 * @param[in] enable Enable (1) or disable (0) wake-up
 * @return kStatus_HAL_GpioSuccess (no-op on Arduino)
 */
hal_gpio_status_t HAL_GpioWakeUpSetting(hal_gpio_handle_t *handle, uint8_t enable)
{
    (void)handle;
    (void)enable;
    return kStatus_HAL_GpioSuccess;
}

/**
 * @brief Read the current logic level of a GPIO input
 * 
 * @param[in] handle Pointer to GPIO handle
 * @param[out] value Pointer to receive the value (0 or 1)
 * @return kStatus_HAL_GpioSuccess on success
 * 
 * @details Calls Arduino digitalRead().
 */
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
