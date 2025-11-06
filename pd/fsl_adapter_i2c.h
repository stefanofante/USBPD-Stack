#ifndef FSL_ADAPTER_I2C_H
#define FSL_ADAPTER_I2C_H

#include <stdbool.h>
#include <stdint.h>
#include "fsl_common.h"

typedef enum _hal_i2c_status
{
    kStatus_HAL_I2cSuccess = 0,
    kStatus_HAL_I2cError,
} hal_i2c_status_t;

typedef enum _pd_status pd_status_t;
typedef void (*PD_I2cReleaseBus)(void);

#ifdef __cplusplus
extern "C" {
#endif

typedef enum _hal_i2c_direction
{
    kHAL_I2cWrite = 0,
    kHAL_I2cRead  = 1,
} hal_i2c_direction_t;

typedef struct _hal_i2c_master_config
{
    uint32_t baudRate_Bps;
    uint32_t srcClock_Hz;
    uint8_t instance;
    bool enableMaster;
} hal_i2c_master_config_t;

typedef struct _hal_i2c_master_transfer
{
    uint8_t *data;
    uint32_t dataSize;
    uint32_t subaddress;
    uint8_t subaddressSize;
    uint16_t flags;
    uint8_t slaveAddress;
    hal_i2c_direction_t direction;
} hal_i2c_master_transfer_t;

typedef struct _hal_i2c_master_handle *hal_i2c_master_handle_t;
typedef hal_i2c_master_handle_t hal_i2c_master_handle_storage_t[1];
#define HAL_I2C_MASTER_HANDLE_DEFINE(name) hal_i2c_master_handle_storage_t name

hal_i2c_status_t HAL_I2cMasterInit(hal_i2c_master_handle_t *handle, const hal_i2c_master_config_t *config);
hal_i2c_status_t HAL_I2cMasterDeinit(hal_i2c_master_handle_t *handle);
hal_i2c_status_t HAL_I2cMasterTransferBlocking(hal_i2c_master_handle_t *handle,
                                               hal_i2c_master_transfer_t *xfer);

#if defined(PD_CONFIG_TARGET_ESP32S3) && (PD_CONFIG_TARGET_ESP32S3)
typedef struct _pd_i2c_esp32_config
{
    int sdaPin;
    int sclPin;
    uint32_t busSpeed_Hz;
    bool enableInternalPullup;
} pd_i2c_esp32_config_t;

pd_status_t PD_I2cConfigureEsp32(uint8_t instance,
                                 const pd_i2c_esp32_config_t *config,
                                 PD_I2cReleaseBus *releaseBusOut);
#endif

#ifdef __cplusplus
}
#endif

#endif /* FSL_ADAPTER_I2C_H */
