/**
 * @file fsl_adapter_i2c.h
 * @brief Platform-independent I2C master HAL abstraction layer
 * 
 * @details Provides a unified I2C master interface across STM32, ESP32-S3, and
 *          Arduino platforms. Supports blocking transfers with sub-addressing.
 * 
 * @author Stefano Fante (STLINE SRL)
 * @date 2025
 */

#ifndef FSL_ADAPTER_I2C_H
#define FSL_ADAPTER_I2C_H

#include <stdbool.h>
#include <stdint.h>
#include "support/fsl_common.h"

/**
 * @brief I2C operation status codes
 */
typedef enum _hal_i2c_status
{
    kStatus_HAL_I2cSuccess = 0,  /**< Operation succeeded */
    kStatus_HAL_I2cError,        /**< Operation failed */
} hal_i2c_status_t;

typedef enum _pd_status pd_status_t;

/**
 * @brief I2C bus release callback function type
 * 
 * @details Used to recover from stuck I2C bus conditions by toggling SCL/SDA
 */
typedef void (*PD_I2cReleaseBus)(void);

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief I2C transfer direction
 */
typedef enum _hal_i2c_direction
{
    kHAL_I2cWrite = 0,  /**< Write to slave */
    kHAL_I2cRead  = 1,  /**< Read from slave */
} hal_i2c_direction_t;

/**
 * @brief I2C master configuration structure
 */
typedef struct _hal_i2c_master_config
{
    uint32_t baudRate_Bps;  /**< I2C bus frequency in bps (e.g., 100000, 400000) */
    uint32_t srcClock_Hz;   /**< Source clock frequency in Hz (platform-specific) */
    uint8_t instance;       /**< I2C instance number (0, 1, etc.) */
    bool enableMaster;      /**< Enable master mode */
} hal_i2c_master_config_t;

/**
 * @brief I2C master transfer descriptor
 */
typedef struct _hal_i2c_master_transfer
{
    uint8_t *data;                  /**< Pointer to data buffer */
    uint32_t dataSize;              /**< Number of data bytes */
    uint32_t subaddress;            /**< Register/sub-address (big-endian) */
    uint8_t subaddressSize;         /**< Number of sub-address bytes (0, 1, or 2 typical) */
    uint16_t flags;                 /**< Transfer flags (platform-specific) */
    uint8_t slaveAddress;           /**< 7-bit slave address */
    hal_i2c_direction_t direction;  /**< Read or write direction */
} hal_i2c_master_transfer_t;

/** @brief Opaque I2C master handle type */
typedef struct _hal_i2c_master_handle *hal_i2c_master_handle_t;

/** @brief I2C master handle storage array type */
typedef hal_i2c_master_handle_t hal_i2c_master_handle_storage_t[1];

/**
 * @brief Macro to define an I2C master handle storage variable
 * 
 * @param name Variable name for the handle storage
 */
#define HAL_I2C_MASTER_HANDLE_DEFINE(name) hal_i2c_master_handle_storage_t name

/**
 * @brief Initialize an I2C master handle
 * 
 * @param[in,out] handle Pointer to I2C master handle (will be allocated)
 * @param[in] config I2C master configuration
 * @return kStatus_HAL_I2cSuccess on success, kStatus_HAL_I2cError otherwise
 * 
 * @note Platform-specific configuration must be done first via
 *       PD_I2cConfigure{Stm32|Esp32|Arduino}()
 */
hal_i2c_status_t HAL_I2cMasterInit(hal_i2c_master_handle_t *handle, const hal_i2c_master_config_t *config);

/**
 * @brief Deinitialize an I2C master handle
 * 
 * @param[in,out] handle Pointer to I2C master handle (will be freed and set to NULL)
 * @return kStatus_HAL_I2cSuccess on success, kStatus_HAL_I2cError otherwise
 */
hal_i2c_status_t HAL_I2cMasterDeinit(hal_i2c_master_handle_t *handle);

/**
 * @brief Perform a blocking I2C master transfer
 * 
 * @param[in] handle I2C master handle
 * @param[in,out] xfer Transfer descriptor (read or write)
 * @return kStatus_HAL_I2cSuccess on success, kStatus_HAL_I2cError otherwise
 * 
 * @details Supports register read/write with optional sub-address bytes
 */
hal_i2c_status_t HAL_I2cMasterTransferBlocking(hal_i2c_master_handle_t *handle,
                                               hal_i2c_master_transfer_t *xfer);

#if defined(PD_CONFIG_TARGET_ESP32S3) && (PD_CONFIG_TARGET_ESP32S3)
/**
 * @brief ESP32-S3 I2C configuration structure
 */
typedef struct _pd_i2c_esp32_config
{
    int sdaPin;                  /**< GPIO pin number for SDA */
    int sclPin;                  /**< GPIO pin number for SCL */
    uint32_t busSpeed_Hz;        /**< I2C bus frequency in Hz */
    bool enableInternalPullup;   /**< Enable ESP32 internal pull-up resistors */
} pd_i2c_esp32_config_t;

/**
 * @brief Configure I2C hardware for ESP32-S3
 * 
 * @param[in] instance I2C instance number
 * @param[in] config ESP32-S3 hardware configuration
 * @param[out] releaseBusOut Pointer to receive bus release function
 * @return kStatus_PD_Success on success, kStatus_PD_Error otherwise
 */
pd_status_t PD_I2cConfigureEsp32(uint8_t instance,
                                 const pd_i2c_esp32_config_t *config,
                                 PD_I2cReleaseBus *releaseBusOut);
#endif

#if defined(PD_CONFIG_TARGET_ARDUINO) && (PD_CONFIG_TARGET_ARDUINO)
struct TwoWire;

/**
 * @brief Arduino I2C configuration structure
 */
typedef struct _pd_i2c_arduino_config
{
    struct TwoWire *wire;        /**< Pointer to Arduino TwoWire object */
    int sdaPin;                  /**< Arduino pin number for SDA */
    int sclPin;                  /**< Arduino pin number for SCL */
    uint32_t busSpeed_Hz;        /**< I2C bus frequency in Hz */
    bool enableInternalPullup;   /**< Enable internal pull-up resistors */
} pd_i2c_arduino_config_t;

/**
 * @brief Configure I2C hardware for Arduino
 * 
 * @param[in] instance I2C instance number
 * @param[in] config Arduino hardware configuration
 * @param[out] releaseBusOut Pointer to receive bus release function
 * @return kStatus_PD_Success on success, kStatus_PD_Error otherwise
 */
pd_status_t PD_I2cConfigureArduino(uint8_t instance,
                                   const pd_i2c_arduino_config_t *config,
                                   PD_I2cReleaseBus *releaseBusOut);
#endif

#if defined(PD_CONFIG_TARGET_STM32) && (PD_CONFIG_TARGET_STM32)
/**
 * @brief STM32 I2C configuration structure
 */
typedef struct _pd_i2c_stm32_config
{
    void *hi2c;         /**< Pointer to I2C_HandleTypeDef (STM32 HAL handle) */
    uint32_t timeoutMs; /**< Transfer timeout in milliseconds */
    void *sclPort;      /**< Pointer to GPIO_TypeDef for SCL (for bus release) */
    uint16_t sclPin;    /**< GPIO pin mask for SCL */
    void *sdaPort;      /**< Pointer to GPIO_TypeDef for SDA (for bus release) */
    uint16_t sdaPin;    /**< GPIO pin mask for SDA */
} pd_i2c_stm32_config_t;

/**
 * @brief Configure I2C hardware for STM32
 * 
 * @param[in] instance I2C instance number
 * @param[in] config STM32 hardware configuration
 * @param[out] releaseBusOut Pointer to receive bus release function
 * @return kStatus_PD_Success on success, kStatus_PD_Error otherwise
 */
pd_status_t PD_I2cConfigureStm32(uint8_t instance,
                                 const pd_i2c_stm32_config_t *config,
                                 PD_I2cReleaseBus *releaseBusOut);
#endif

#ifdef __cplusplus
}
#endif

#endif /* FSL_ADAPTER_I2C_H */
