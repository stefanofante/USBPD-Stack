/**
 * @file fsl_adapter_i2c_arduino.cpp
 * @brief I2C master HAL adapter for Arduino using TwoWire
 * 
 * @details Implements platform-independent I2C abstraction using Arduino Wire
 *          library. Includes I2C bus release logic.
 * 
 * @author Stefano Fante (STLINE SRL)
 * @date 2025
 */

#include "support/fsl_adapter_i2c.h"

#if defined(PD_CONFIG_TARGET_ARDUINO) && (PD_CONFIG_TARGET_ARDUINO)

#include <Arduino.h>
#include <Wire.h>
#include <stdlib.h>
#include <string.h>

#include "usb_pd.h"

namespace
{
/**
 * @brief I2C instance configuration
 */
struct I2cInstanceConfig
{
    bool configured;       /**< True if configured via PD_I2cConfigureArduino() */
    TwoWire *wire;         /**< Pointer to Arduino TwoWire object */
    int sdaPin;            /**< SDA pin number */
    int sclPin;            /**< SCL pin number */
    uint32_t busSpeed;     /**< I2C bus frequency in Hz */
    bool internalPullup;   /**< Enable internal pull-up resistors */
};

/**
 * @brief Internal I2C master handle
 */
struct _hal_i2c_master_handle
{
    uint8_t instance;  /**< I2C instance number */
};

/** @brief Global I2C configuration storage */
I2cInstanceConfig s_i2cConfig[USB_PD_I2C_INSTANCE_COUNT];

/**
 * @brief Release a stuck I2C bus by toggling SCL
 * 
 * @param[in] instance I2C instance number
 */
void ReleaseBusOnInstance(uint8_t instance)
{
    if (instance >= USB_PD_I2C_INSTANCE_COUNT)
    {
        return;
    }

    const I2cInstanceConfig &cfg = s_i2cConfig[instance];
    if ((cfg.sdaPin < 0) || (cfg.sclPin < 0))
    {
        return;
    }

    pinMode(cfg.sdaPin, INPUT_PULLUP);
    pinMode(cfg.sclPin, INPUT_PULLUP);

    for (uint8_t i = 0; i < 9U; ++i)
    {
        pinMode(cfg.sclPin, OUTPUT);
        digitalWrite(cfg.sclPin, LOW);
        delayMicroseconds(5);
        digitalWrite(cfg.sclPin, HIGH);
        delayMicroseconds(5);
    }

    pinMode(cfg.sdaPin, OUTPUT);
    digitalWrite(cfg.sdaPin, LOW);
    delayMicroseconds(5);
    digitalWrite(cfg.sdaPin, HIGH);
    delayMicroseconds(5);
    pinMode(cfg.sdaPin, INPUT_PULLUP);
    pinMode(cfg.sclPin, INPUT_PULLUP);
}

/**
 * @brief Release bus for I2C instance 0
 */
void ReleaseBus0(void)
{
    ReleaseBusOnInstance(0);
}

/**
 * @brief Release bus for I2C instance 1
 */
void ReleaseBus1(void)
{
    ReleaseBusOnInstance(1);
}

/** @brief Array of bus release function pointers */
const PD_I2cReleaseBus s_releaseHandlers[USB_PD_I2C_INSTANCE_COUNT] = {
    ReleaseBus0,
    ReleaseBus1,
};

/**
 * @brief Get TwoWire pointer for a given instance
 * 
 * @param[in] instance I2C instance number
 * @return Pointer to TwoWire or nullptr if not configured
 */
TwoWire *GetWire(uint8_t instance)
{
    if (instance >= USB_PD_I2C_INSTANCE_COUNT)
    {
        return nullptr;
    }
    if (!s_i2cConfig[instance].configured)
    {
        return nullptr;
    }
    return s_i2cConfig[instance].wire;
}

} // namespace

extern "C" {

/**
 * @brief Configure I2C hardware for Arduino
 * 
 * @param[in] instance I2C instance number
 * @param[in] config Hardware configuration (TwoWire pointer, pins, speed)
 * @param[out] releaseBusOut Pointer to receive bus release function
 * @return kStatus_PD_Success on success
 */
pd_status_t PD_I2cConfigureArduino(uint8_t instance,
                                   const pd_i2c_arduino_config_t *config,
                                   PD_I2cReleaseBus *releaseBusOut)
{
    if ((config == NULL) || (config->wire == NULL) || (instance >= USB_PD_I2C_INSTANCE_COUNT))
    {
        return kStatus_PD_Error;
    }

    s_i2cConfig[instance].configured      = true;
    s_i2cConfig[instance].wire            = config->wire;
    s_i2cConfig[instance].sdaPin          = config->sdaPin;
    s_i2cConfig[instance].sclPin          = config->sclPin;
    s_i2cConfig[instance].busSpeed        = config->busSpeed_Hz;
    s_i2cConfig[instance].internalPullup  = config->enableInternalPullup;

    if (releaseBusOut != NULL)
    {
        *releaseBusOut = (instance < USB_PD_I2C_INSTANCE_COUNT) ? s_releaseHandlers[instance] : NULL;
    }

    return kStatus_PD_Success;
}

/**
 * @brief Initialize an I2C master handle
 * 
 * @param[out] handle Pointer to handle (will be allocated)
 * @param[in] config Configuration (instance number)
 * @return kStatus_HAL_I2cSuccess on success
 * 
 * @details Calls TwoWire::begin() and setClock().
 */
hal_i2c_status_t HAL_I2cMasterInit(hal_i2c_master_handle_t *handle, const hal_i2c_master_config_t *config)
{
    if ((handle == NULL) || (config == NULL))
    {
        return kStatus_HAL_I2cError;
    }

    uint8_t instance = config->instance;
    TwoWire *wire    = GetWire(instance);
    if (wire == nullptr)
    {
        return kStatus_HAL_I2cError;
    }

    hal_i2c_master_handle_t storage = static_cast<hal_i2c_master_handle_t>(malloc(sizeof(**handle)));
    if (storage == NULL)
    {
        return kStatus_HAL_I2cError;
    }

    storage->instance = instance;

    wire->begin();
    if (s_i2cConfig[instance].busSpeed != 0U)
    {
        wire->setClock(s_i2cConfig[instance].busSpeed);
    }

    *handle = storage;
    return kStatus_HAL_I2cSuccess;
}

/**
 * @brief Deinitialize an I2C master handle
 * 
 * @param[in,out] handle Pointer to handle (will be freed)
 * @return kStatus_HAL_I2cSuccess on success
 */
hal_i2c_status_t HAL_I2cMasterDeinit(hal_i2c_master_handle_t *handle)
{
    if ((handle == NULL) || (*handle == NULL))
    {
        return kStatus_HAL_I2cError;
    }

    free(*handle);
    *handle = NULL;
    return kStatus_HAL_I2cSuccess;
}

/**
 * @brief Perform an I2C write with optional sub-address
 * 
 * @param[in] wire TwoWire reference
 * @param[in] slaveAddress 7-bit slave address
 * @param[in] subaddress Register address (big-endian)
 * @param[in] subaddressSize Number of sub-address bytes (0, 1, or 2)
 * @param[in] data Data to write
 * @param[in] length Number of bytes to write
 * @return kStatus_HAL_I2cSuccess on success
 * 
 * @details Calls TwoWire::beginTransmission(), write(), and endTransmission().
 */
static hal_i2c_status_t I2cWrite(TwoWire &wire,
                                 uint8_t slaveAddress,
                                 uint32_t subaddress,
                                 uint8_t subaddressSize,
                                 const uint8_t *data,
                                 uint32_t length)
{
    wire.beginTransmission(slaveAddress);
    for (int8_t idx = static_cast<int8_t>(subaddressSize) - 1; idx >= 0; --idx)
    {
        wire.write(static_cast<uint8_t>((subaddress >> (idx * 8)) & 0xFFU));
    }
    for (uint32_t i = 0; i < length; ++i)
    {
        wire.write(data[i]);
    }

    uint8_t result = wire.endTransmission(true);
    return (result == 0U) ? kStatus_HAL_I2cSuccess : kStatus_HAL_I2cError;
}

/**
 * @brief Perform an I2C read with optional sub-address
 * 
 * @param[in] wire TwoWire reference
 * @param[in] slaveAddress 7-bit slave address
 * @param[in] subaddress Register address (big-endian)
 * @param[in] subaddressSize Number of sub-address bytes (0, 1, or 2)
 * @param[out] data Buffer to receive data
 * @param[in] length Number of bytes to read
 * @return kStatus_HAL_I2cSuccess on success
 * 
 * @details Writes sub-address with repeated start, then calls requestFrom().
 */
static hal_i2c_status_t I2cRead(TwoWire &wire,
                                uint8_t slaveAddress,
                                uint32_t subaddress,
                                uint8_t subaddressSize,
                                uint8_t *data,
                                uint32_t length)
{
    if (subaddressSize != 0U)
    {
        wire.beginTransmission(slaveAddress);
        for (int8_t idx = static_cast<int8_t>(subaddressSize) - 1; idx >= 0; --idx)
        {
            wire.write(static_cast<uint8_t>((subaddress >> (idx * 8)) & 0xFFU));
        }
        uint8_t result = wire.endTransmission(false);
        if (result != 0U)
        {
            return kStatus_HAL_I2cError;
        }
    }

    uint32_t received = wire.requestFrom(static_cast<int>(slaveAddress), static_cast<int>(length), true);
    if (received != length)
    {
        return kStatus_HAL_I2cError;
    }

    for (uint32_t i = 0; i < length; ++i)
    {
        if (wire.available())
        {
            data[i] = wire.read();
        }
        else
        {
            return kStatus_HAL_I2cError;
        }
    }

    return kStatus_HAL_I2cSuccess;
}

/**
 * @brief Perform a blocking I2C transfer
 * 
 * @param[in] handle I2C master handle
 * @param[in,out] xfer Transfer descriptor
 * @return kStatus_HAL_I2cSuccess on success
 * 
 * @details Calls I2cRead() or I2cWrite() based on xfer->direction.
 */
hal_i2c_status_t HAL_I2cMasterTransferBlocking(hal_i2c_master_handle_t *handle, hal_i2c_master_transfer_t *xfer)
{
    if ((handle == NULL) || (*handle == NULL) || (xfer == NULL))
    {
        return kStatus_HAL_I2cError;
    }

    uint8_t instance = (*handle)->instance;
    TwoWire *wire    = GetWire(instance);
    if (wire == nullptr)
    {
        return kStatus_HAL_I2cError;
    }

    if (xfer->direction == kHAL_I2cRead)
    {
        return I2cRead(*wire, xfer->slaveAddress, xfer->subaddress, xfer->subaddressSize, xfer->data, xfer->dataSize);
    }
    else
    {
        return I2cWrite(*wire, xfer->slaveAddress, xfer->subaddress, xfer->subaddressSize, xfer->data, xfer->dataSize);
    }
}

} /* extern "C" */

#endif /* PD_CONFIG_TARGET_ARDUINO */
