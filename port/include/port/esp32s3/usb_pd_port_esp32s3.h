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

typedef struct _pd_phy_esp32s3_config
{
    pd_phy_config_t base;
    int sdaPin;
    int sclPin;
    uint32_t i2cBusSpeed_Hz;
    bool enableInternalPullup;
    gpio_int_type_t alertInterruptType;
    hal_gpio_pull_mode_t alertPullMode;
    uint32_t alertIsrFlags;
} pd_phy_esp32s3_config_t;

void PD_PortEsp32S3_RegisterInstance(pd_instance_t *instance);
void PD_PortEsp32S3_UnregisterInstance(pd_instance_t *instance);

#ifdef __cplusplus
}
#endif

#endif /* USB_PD_PORT_ESP32S3_H */
