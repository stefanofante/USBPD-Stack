#ifndef FSL_ADAPTER_GPIO_H
#define FSL_ADAPTER_GPIO_H

#include <stdbool.h>
#include <stdint.h>
#include "support/fsl_common.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum _hal_gpio_direction
{
    kHAL_GpioDirectionIn,
    kHAL_GpioDirectionOut,
} hal_gpio_direction_t;

typedef enum _hal_gpio_status
{
    kStatus_HAL_GpioSuccess = 0,
    kStatus_HAL_GpioError,
} hal_gpio_status_t;

typedef enum _hal_gpio_interrupt_trigger
{
    kHAL_GpioInterruptDisable,
    kHAL_GpioInterruptRisingEdge,
    kHAL_GpioInterruptFallingEdge,
    kHAL_GpioInterruptEitherEdge,
    kHAL_GpioInterruptLogicOne,
    kHAL_GpioInterruptLogicZero,
} hal_gpio_interrupt_trigger_t;

typedef enum _hal_gpio_pull_mode
{
    kHAL_GpioNoPull,
    kHAL_GpioPullUp,
    kHAL_GpioPullDown,
} hal_gpio_pull_mode_t;

typedef struct _hal_gpio_pin_config
{
    hal_gpio_direction_t direction;
    uint8_t port;
    uint8_t pin;
    uint8_t level;
    hal_gpio_pull_mode_t pullMode;
    uint32_t intrFlags; /* Platform-specific flags; STM32 uses this to pass pd_phy_stm32_config_t */
} hal_gpio_pin_config_t;

typedef void (*hal_gpio_callback_t)(void *callbackParam);

typedef struct _hal_gpio_handle *hal_gpio_handle_t;
typedef hal_gpio_handle_t hal_gpio_handle_storage_t[1];
#define GPIO_HANDLE_DEFINE(name) hal_gpio_handle_storage_t name

hal_gpio_status_t HAL_GpioInit(hal_gpio_handle_t *handle, const hal_gpio_pin_config_t *config);
hal_gpio_status_t HAL_GpioDeinit(hal_gpio_handle_t *handle);
hal_gpio_status_t HAL_GpioInstallCallback(hal_gpio_handle_t *handle, hal_gpio_callback_t callback, void *param);
hal_gpio_status_t HAL_GpioSetTriggerMode(hal_gpio_handle_t *handle, hal_gpio_interrupt_trigger_t triggerMode);
hal_gpio_status_t HAL_GpioWakeUpSetting(hal_gpio_handle_t *handle, uint8_t enable);
hal_gpio_status_t HAL_GpioGetInput(hal_gpio_handle_t *handle, uint8_t *value);

#ifdef __cplusplus
}
#endif

#endif /* FSL_ADAPTER_GPIO_H */
