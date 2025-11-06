#ifndef USB_PD_PORT_STM32_H
#define USB_PD_PORT_STM32_H

#include <stdbool.h>
#include <stdint.h>

#include "support/fsl_adapter_gpio.h"
#include "usb_pd.h"

#if defined(STM32F3xx)
#include "stm32f3xx_hal.h"
#elif defined(STM32F4xx)
#include "stm32f4xx_hal.h"
#elif defined(STM32H7xx)
#include "stm32h7xx_hal.h"
#else
#error "Define STM32F3xx, STM32F4xx, or STM32H7xx before including usb_pd_port_stm32.h."
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _pd_phy_stm32_config
{
    pd_phy_config_t base;
    I2C_HandleTypeDef *hi2c;
    uint32_t i2cTimeoutMs;
    GPIO_TypeDef *sclPort;
    uint16_t sclPin;
    GPIO_TypeDef *sdaPort;
    uint16_t sdaPin;
    GPIO_TypeDef *alertGpio;
    uint16_t alertPinMask;
    hal_gpio_pull_mode_t alertPullMode;
    IRQn_Type alertIRQn;
    uint32_t alertPreemptPriority;
    uint32_t alertSubPriority;
} pd_phy_stm32_config_t;

void PD_PortStm32_RegisterInstance(pd_instance_t *instance);
void PD_PortStm32_UnregisterInstance(pd_instance_t *instance);
void PD_PortStm32_DispatchExti(uint16_t gpioPin);

#ifdef __cplusplus
}
#endif

#endif /* USB_PD_PORT_STM32_H */
