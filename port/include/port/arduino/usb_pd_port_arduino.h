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

typedef struct _pd_phy_arduino_config
{
    pd_phy_config_t base;
    struct TwoWire *wire;
    int sdaPin;
    int sclPin;
    uint32_t i2cBusSpeed_Hz;
    bool enableInternalPullup;
    hal_gpio_pull_mode_t alertPullMode;
    bool alertActiveLow;
} pd_phy_arduino_config_t;

void PD_PortArduino_RegisterInstance(pd_instance_t *instance);
void PD_PortArduino_UnregisterInstance(pd_instance_t *instance);
void PD_PortArduino_TaskTick(void);
void PD_PortArduino_EnterCritical(void);
void PD_PortArduino_ExitCritical(void);

#ifdef __cplusplus
}
#endif

#endif /* USB_PD_PORT_ARDUINO_H */
