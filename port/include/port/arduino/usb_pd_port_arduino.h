/**
 * @file usb_pd_port_arduino.h
 * @brief Arduino port layer public API for USB PD stack
 * 
 * @details Defines platform-specific configuration and cooperative scheduling
 *          functions for Arduino-based platforms using TwoWire I2C.
 * 
 * @author Stefano Fante (STLINE SRL)
 * @date 2025
 */

#ifndef USB_PD_PORT_ARDUINO_H
#define USB_PD_PORT_ARDUINO_H

#include <stdbool.h>
#include <stdint.h>

#include "support/fsl_adapter_gpio.h"
#include "usb_pd.h"

#ifdef __cplusplus
extern "C" {
#endif

struct TwoWire;

/**
 * @brief Arduino-specific PHY configuration structure
 * 
 * @details Extends base pd_phy_config_t with TwoWire pointer, GPIO pin
 *          numbers, and I2C settings for Arduino environments.
 */
typedef struct _pd_phy_arduino_config
{
    pd_phy_config_t base;              /**< Base PHY configuration (common fields) */
    struct TwoWire *wire;              /**< Pointer to Arduino TwoWire (I2C) object */
    int sdaPin;                        /**< Arduino pin number for I2C SDA */
    int sclPin;                        /**< Arduino pin number for I2C SCL */
    uint32_t i2cBusSpeed_Hz;           /**< I2C bus frequency in Hz (e.g., 100000, 400000) */
    bool enableInternalPullup;         /**< Enable internal pull-up resistors on I2C pins */
    hal_gpio_pull_mode_t alertPullMode; /**< Pull resistor mode for PTN5110 ALERT pin */
    bool alertActiveLow;               /**< True if ALERT is active-low, false if active-high */
} pd_phy_arduino_config_t;

/**
 * @brief Register a PD instance with the Arduino port layer
 * 
 * @param[in] instance Pointer to the PD instance to register
 * 
 * @note Starts micros() tracking when the first instance is registered
 */
void PD_PortArduino_RegisterInstance(pd_instance_t *instance);

/**
 * @brief Unregister a PD instance from the Arduino port layer
 * 
 * @param[in] instance Pointer to the PD instance to unregister
 * 
 * @note Stops tracking when the last instance is unregistered
 */
void PD_PortArduino_UnregisterInstance(pd_instance_t *instance);

/**
 * @brief Cooperative task tick - MUST be called from loop()
 * 
 * @note Call this function every iteration of loop() to drive the USB PD
 *       state machine. Missing calls will cause delayed timer events.
 */
void PD_PortArduino_TaskTick(void);

/**
 * @brief Enter critical section (disable interrupts)
 * 
 * @note Arduino implementation uses noInterrupts()
 */
void PD_PortArduino_EnterCritical(void);

/**
 * @brief Exit critical section (re-enable interrupts)
 * 
 * @note Arduino implementation uses interrupts()
 */
void PD_PortArduino_ExitCritical(void);

#ifdef __cplusplus
}
#endif

#endif /* USB_PD_PORT_ARDUINO_H */
