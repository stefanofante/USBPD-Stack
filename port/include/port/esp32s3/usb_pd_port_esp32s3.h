/**
 * @file usb_pd_port_esp32s3.h
 * @brief ESP32-S3 port layer public API for USB PD stack
 * 
 * @details Defines platform-specific configuration structure and registration
 *          functions for ESP32-S3 using ESP-IDF and FreeRTOS.
 * 
 * @author Stefano Fante (STLINE SRL)
 * @date 2025
 */

#ifndef USB_PD_PORT_ESP32S3_H
#define USB_PD_PORT_ESP32S3_H

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "support/fsl_adapter_gpio.h"
#include "usb_pd.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief ESP32-S3-specific PHY configuration structure
 * 
 * @details Extends the base pd_phy_config_t with ESP32-S3 GPIO pin numbers,
 *          I2C bus settings, and interrupt configuration.
 */
typedef struct _pd_phy_esp32s3_config
{
    pd_phy_config_t base;              /**< Base PHY configuration (common fields) */
    int sdaPin;                        /**< GPIO pin number for I2C SDA */
    int sclPin;                        /**< GPIO pin number for I2C SCL */
    uint32_t i2cBusSpeed_Hz;           /**< I2C bus frequency in Hz (e.g., 100000 for 100kHz) */
    bool enableInternalPullup;         /**< Enable ESP32-S3 internal pull-ups on I2C pins */
    gpio_int_type_t alertInterruptType; /**< Interrupt trigger type for PTN5110 ALERT pin */
    hal_gpio_pull_mode_t alertPullMode; /**< Pull resistor configuration for ALERT pin */
    uint32_t alertIsrFlags;            /**< ESP-IDF ISR allocation flags (e.g., ESP_INTR_FLAG_IRAM) */
} pd_phy_esp32s3_config_t;

/**
 * @brief Register a PD instance with the ESP32-S3 port layer
 * 
 * @param[in] instance Pointer to the PD instance to register
 * 
 * @note Starts the 1ms FreeRTOS timer if this is the first active instance
 */
void PD_PortEsp32S3_RegisterInstance(pd_instance_t *instance);

/**
 * @brief Unregister a PD instance from the ESP32-S3 port layer
 * 
 * @param[in] instance Pointer to the PD instance to unregister
 * 
 * @note Stops the FreeRTOS timer when the last instance is unregistered
 */
void PD_PortEsp32S3_UnregisterInstance(pd_instance_t *instance);

#ifdef __cplusplus
}
#endif

#endif /* USB_PD_PORT_ESP32S3_H */
