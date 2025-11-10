/**
 * @file usb_pd_port_stm32.h
 * @brief STM32 port layer public API for USB PD stack
 * 
 * @details Defines platform-specific configuration structure and registration
 *          functions for STM32 F3/F4/H7 families using STM32Cube HAL and FreeRTOS.
 * 
 * @author Stefano Fante (STLINE SRL)
 * @date 2025
 */

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

/**
 * @brief STM32-specific PHY configuration structure
 * 
 * @details Extends the base pd_phy_config_t with STM32 HAL handles, GPIO ports,
 *          I2C timeout settings, and EXTI interrupt configuration.
 */
typedef struct _pd_phy_stm32_config
{
    pd_phy_config_t base;              /**< Base PHY configuration (common fields) */
    I2C_HandleTypeDef *hi2c;           /**< STM32 HAL I2C handle for TCPC communication */
    uint32_t i2cTimeoutMs;             /**< I2C transaction timeout in milliseconds */
    GPIO_TypeDef *sclPort;             /**< GPIO port for I2C SCL pin */
    uint16_t sclPin;                   /**< GPIO pin number for I2C SCL */
    GPIO_TypeDef *sdaPort;             /**< GPIO port for I2C SDA pin */
    uint16_t sdaPin;                   /**< GPIO pin number for I2C SDA */
    GPIO_TypeDef *alertGpio;           /**< GPIO port for PTN5110 ALERT pin */
    uint16_t alertPinMask;             /**< GPIO pin mask for ALERT (e.g., GPIO_PIN_0) */
    hal_gpio_pull_mode_t alertPullMode; /**< Pull resistor configuration for ALERT pin */
    IRQn_Type alertIRQn;               /**< EXTI IRQ number for ALERT pin */
    uint32_t alertPreemptPriority;     /**< NVIC preempt priority for ALERT interrupt */
    uint32_t alertSubPriority;         /**< NVIC sub priority for ALERT interrupt */
} pd_phy_stm32_config_t;

/**
 * @brief Register a PD instance with the STM32 port layer
 * 
 * @param[in] instance Pointer to the PD instance to register
 * 
 * @note Starts the 1ms FreeRTOS timer if this is the first active instance
 */
void PD_PortStm32_RegisterInstance(pd_instance_t *instance);

/**
 * @brief Unregister a PD instance from the STM32 port layer
 * 
 * @param[in] instance Pointer to the PD instance to unregister
 * 
 * @note Stops the FreeRTOS timer when the last instance is unregistered
 */
void PD_PortStm32_UnregisterInstance(pd_instance_t *instance);

/**
 * @brief Dispatch EXTI interrupt for PTN5110 ALERT pin
 * 
 * @details Call this function from your HAL_GPIO_EXTI_Callback to route
 *          ALERT events to the registered PD instances.
 * 
 * @param[in] gpioPin GPIO pin number that triggered the interrupt
 * 
 * @note Ensure gpioPin matches the alertPinMask in pd_phy_stm32_config_t
 */
void PD_PortStm32_DispatchExti(uint16_t gpioPin);

#ifdef __cplusplus
}
#endif

#endif /* USB_PD_PORT_STM32_H */
