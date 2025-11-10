/**
 * @file fsl_adapter_gpio_stm32.c
 * @brief GPIO HAL adapter for STM32 using Cube HAL
 * 
 * @details Implements platform-independent GPIO abstraction on STM32 F3/F4/H7
 *          platforms. Handles EXTI interrupt routing and callback dispatch.
 * 
 * @author Stefano Fante (STLINE SRL)
 * @date 2025
 */

#include "support/fsl_adapter_gpio.h"

#if defined(PD_CONFIG_TARGET_STM32) && (PD_CONFIG_TARGET_STM32)

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

#include "support/fsl_common.h"
#include "usb_pd.h"
#include "port/stm32/usb_pd_port_stm32.h"

#if defined(STM32F3xx)
#include "stm32f3xx_hal.h"
#elif defined(STM32F4xx)
#include "stm32f4xx_hal.h"
#elif defined(STM32H7xx)
#include "stm32h7xx_hal.h"
#else
#error "Define STM32F3xx, STM32F4xx, or STM32H7xx to include the correct HAL headers."
#endif

/**
 * @brief Internal GPIO handle structure for STM32
 * 
 * @details Stores HAL GPIO configuration, interrupt settings, and callback
 *          information for each GPIO instance.
 */
typedef struct _hal_gpio_handle
{
    GPIO_TypeDef *port;                      /**< STM32 GPIO port (GPIOA, GPIOB, etc.) */
    uint16_t pin;                            /**< GPIO pin mask (GPIO_PIN_0, GPIO_PIN_1, etc.) */
    const pd_phy_stm32_config_t *platformConfig; /**< Platform-specific configuration */
    hal_gpio_direction_t direction;          /**< Input or output direction */
    hal_gpio_interrupt_trigger_t trigger;    /**< Interrupt trigger type */
    hal_gpio_callback_t callback;            /**< User callback for interrupts */
    void *callbackParam;                     /**< User parameter for callback */
    IRQn_Type irqn;                          /**< NVIC IRQ number for EXTI line */
    uint32_t preemptPrio;                    /**< NVIC preemption priority */
    uint32_t subPrio;                        /**< NVIC sub-priority */
} hal_gpio_handle_obj_t;

/** @brief Global registry of active GPIO handles */
static hal_gpio_handle_obj_t *s_registeredHandles[PD_CONFIG_MAX_PORT];

/**
 * @brief Add a GPIO handle to the internal registry
 * 
 * @param[in] handle GPIO handle to register
 * @return true if registered successfully, false if registry full
 */
static bool PD_GpioTrackHandle(hal_gpio_handle_obj_t *handle)
{
    for (size_t i = 0; i < ARRAY_SIZE(s_registeredHandles); ++i)
    {
        if (s_registeredHandles[i] == NULL)
        {
            s_registeredHandles[i] = handle;
            return true;
        }
    }
    return false;
}

/**
 * @brief Remove a GPIO handle from the internal registry
 * 
 * @param[in] handle GPIO handle to unregister
 */
static void PD_GpioUntrackHandle(hal_gpio_handle_obj_t *handle)
{
    for (size_t i = 0; i < ARRAY_SIZE(s_registeredHandles); ++i)
    {
        if (s_registeredHandles[i] == handle)
        {
            s_registeredHandles[i] = NULL;
            return;
        }
    }
}

/**
 * @brief Map HAL pull mode to STM32 Cube HAL pull constant
 * 
 * @param[in] pull HAL pull mode enumeration
 * @return STM32 HAL GPIO pull constant (GPIO_PULLUP, GPIO_PULLDOWN, GPIO_NOPULL)
 */
static uint32_t PD_GpioMapPull(hal_gpio_pull_mode_t pull)
{
    switch (pull)
    {
        case kHAL_GpioPullUp:
            return GPIO_PULLUP;
        case kHAL_GpioPullDown:
            return GPIO_PULLDOWN;
        case kHAL_GpioNoPull:
        default:
            return GPIO_NOPULL;
    }
}

/**
 * @brief Map HAL direction to STM32 Cube HAL GPIO mode
 * 
 * @param[in] direction HAL direction enumeration
 * @return STM32 HAL GPIO mode (GPIO_MODE_OUTPUT_PP or GPIO_MODE_INPUT)
 */
static uint32_t PD_GpioMapMode(hal_gpio_direction_t direction)
{
    return (direction == kHAL_GpioDirectionOut) ? GPIO_MODE_OUTPUT_PP : GPIO_MODE_INPUT;
}

/**
 * @brief Map HAL interrupt trigger to STM32 Cube HAL EXTI mode
 * 
 * @param[in] trigger HAL interrupt trigger enumeration
 * @return STM32 HAL GPIO EXTI mode (GPIO_MODE_IT_RISING, GPIO_MODE_IT_FALLING, etc.)
 */
static uint32_t PD_GpioMapExtiMode(hal_gpio_interrupt_trigger_t trigger)
{
    switch (trigger)
    {
        case kHAL_GpioInterruptRisingEdge:
            return GPIO_MODE_IT_RISING;
        case kHAL_GpioInterruptFallingEdge:
            return GPIO_MODE_IT_FALLING;
        case kHAL_GpioInterruptEitherEdge:
            return GPIO_MODE_IT_RISING_FALLING;
        case kHAL_GpioInterruptLogicOne:
            return GPIO_MODE_IT_RISING;
        case kHAL_GpioInterruptLogicZero:
            return GPIO_MODE_IT_FALLING;
        case kHAL_GpioInterruptDisable:
        default:
            return GPIO_MODE_INPUT;
    }
}

/**
 * @brief Initialize a GPIO pin
 * 
 * @param[in,out] handle Pointer to GPIO handle (will be allocated and returned)
 * @param[in] config Pin configuration (direction, level, pull mode, etc.)
 * 
 * @return kStatus_HAL_GpioSuccess on success, kStatus_HAL_GpioError otherwise
 * 
 * @details Allocates a handle, configures the pin via STM32 HAL_GPIO_Init(),
 *          and registers the handle for EXTI interrupt dispatch.
 * 
 * @note The platformConfig pointer is passed via config->intrFlags.
 */
hal_gpio_status_t HAL_GpioInit(hal_gpio_handle_t *handle, const hal_gpio_pin_config_t *config)
{
    if ((handle == NULL) || (config == NULL) || (*handle != NULL))
    {
        return kStatus_HAL_GpioError;
    }

    const pd_phy_stm32_config_t *platformConfig = (const pd_phy_stm32_config_t *)(uintptr_t)config->intrFlags;
    if (platformConfig == NULL)
    {
        return kStatus_HAL_GpioError;
    }

    hal_gpio_handle_obj_t *obj = calloc(1, sizeof(*obj));
    if (obj == NULL)
    {
        return kStatus_HAL_GpioError;
    }

    obj->port           = platformConfig->alertGpio;
    obj->pin            = platformConfig->alertPinMask;
    obj->platformConfig = platformConfig;
    obj->direction      = config->direction;
    obj->trigger        = kHAL_GpioInterruptDisable;
    obj->irqn           = platformConfig->alertIRQn;
    obj->preemptPrio = platformConfig->alertPreemptPriority;
    obj->subPrio     = platformConfig->alertSubPriority;

    GPIO_InitTypeDef init = {
        .Pin       = obj->pin,
        .Mode      = PD_GpioMapMode(obj->direction),
        .Pull      = PD_GpioMapPull(platformConfig->alertPullMode),
        .Speed     = GPIO_SPEED_FREQ_HIGH,
        .Alternate = 0,
    };

    HAL_GPIO_Init(obj->port, &init);
    if (obj->direction == kHAL_GpioDirectionOut)
    {
        HAL_GPIO_WritePin(obj->port, obj->pin, (config->level != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }

    if (!PD_GpioTrackHandle(obj))
    {
        HAL_GPIO_DeInit(obj->port, obj->pin);
        free(obj);
        return kStatus_HAL_GpioError;
    }
    *handle = obj;
    return kStatus_HAL_GpioSuccess;
}

/**
 * @brief Deinitialize a GPIO pin
 * 
 * @param[in,out] handle Pointer to GPIO handle (will be freed and set to NULL)
 * @return kStatus_HAL_GpioSuccess on success, kStatus_HAL_GpioError otherwise
 * 
 * @details Calls HAL_GPIO_DeInit(), unregisters the handle, and frees memory.
 */
hal_gpio_status_t HAL_GpioDeinit(hal_gpio_handle_t *handle)
{
    if ((handle == NULL) || (*handle == NULL))
    {
        return kStatus_HAL_GpioError;
    }

    hal_gpio_handle_obj_t *obj = *handle;
    HAL_GPIO_DeInit(obj->port, obj->pin);
    PD_GpioUntrackHandle(obj);
    free(obj);
    *handle = NULL;
    return kStatus_HAL_GpioSuccess;
}

/**
 * @brief Install an interrupt callback for a GPIO pin
 * 
 * @param[in] handle Pointer to GPIO handle
 * @param[in] callback Callback function to invoke on interrupt
 * @param[in] param User parameter passed to callback
 * 
 * @return kStatus_HAL_GpioSuccess on success, kStatus_HAL_GpioError otherwise
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
    return kStatus_HAL_GpioSuccess;
}

/**
 * @brief Set GPIO interrupt trigger mode
 * 
 * @param[in] handle Pointer to GPIO handle
 * @param[in] triggerMode Trigger mode (rising, falling, both edges, etc.)
 * 
 * @return kStatus_HAL_GpioSuccess on success, kStatus_HAL_GpioError otherwise
 * 
 * @details Reconfigures the GPIO in EXTI mode and enables/disables NVIC IRQ.
 */
hal_gpio_status_t HAL_GpioSetTriggerMode(hal_gpio_handle_t *handle, hal_gpio_interrupt_trigger_t triggerMode)
{
    if ((handle == NULL) || (*handle == NULL))
    {
        return kStatus_HAL_GpioError;
    }

    hal_gpio_handle_obj_t *obj = *handle;
    if (obj->platformConfig == NULL)
    {
        return kStatus_HAL_GpioError;
    }
    obj->trigger               = triggerMode;

    GPIO_InitTypeDef init = {
        .Pin       = obj->pin,
        .Mode      = PD_GpioMapExtiMode(triggerMode),
        .Pull      = PD_GpioMapPull(obj->platformConfig->alertPullMode),
        .Speed     = GPIO_SPEED_FREQ_HIGH,
        .Alternate = 0,
    };

    HAL_GPIO_Init(obj->port, &init);

    if (triggerMode != kHAL_GpioInterruptDisable)
    {
        __HAL_GPIO_EXTI_CLEAR_IT(obj->pin);
        HAL_NVIC_ClearPendingIRQ(obj->irqn);
        HAL_NVIC_SetPriority(obj->irqn, obj->preemptPrio, obj->subPrio);
        HAL_NVIC_EnableIRQ(obj->irqn);
    }
    else
    {
        HAL_NVIC_DisableIRQ(obj->irqn);
    }

    return kStatus_HAL_GpioSuccess;
}

/**
 * @brief Configure wake-up functionality (stub for STM32)
 * 
 * @param[in] handle Pointer to GPIO handle
 * @param[in] enable Enable (1) or disable (0) wake-up
 * 
 * @return kStatus_HAL_GpioSuccess (no-op on STM32)
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
    *value = (uint8_t)(HAL_GPIO_ReadPin(obj->port, obj->pin) != GPIO_PIN_RESET);
    return kStatus_HAL_GpioSuccess;
}

/**
 * @brief Dispatch EXTI interrupt to registered GPIO callbacks
 * 
 * @param[in] gpioPin STM32 GPIO pin mask (GPIO_PIN_0, GPIO_PIN_1, etc.)
 * 
 * @details Called by HAL_GPIO_EXTI_Callback() to route interrupts to user callbacks.
 */
void PD_PortStm32_DispatchExti(uint16_t gpioPin)
{
    for (size_t i = 0; i < ARRAY_SIZE(s_registeredHandles); ++i)
    {
        hal_gpio_handle_obj_t *obj = s_registeredHandles[i];
        if ((obj != NULL) && (obj->pin == gpioPin) && (obj->callback != NULL))
        {
            obj->callback(obj->callbackParam);
        }
    }
}

/**
 * @brief STM32 HAL EXTI callback (weak, can be overridden)
 * 
 * @param[in] GPIO_Pin STM32 GPIO pin mask that triggered the interrupt
 * 
 * @details Default implementation calls PD_PortStm32_DispatchExti() to route
 *          to user callbacks.
 */
__weak void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    PD_PortStm32_DispatchExti(GPIO_Pin);
}

#endif /* PD_CONFIG_TARGET_STM32 */
