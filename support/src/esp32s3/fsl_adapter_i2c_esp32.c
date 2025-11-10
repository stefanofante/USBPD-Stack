/**
 * @file fsl_adapter_i2c_esp32.c
 * @brief I2C master HAL adapter for ESP32-S3 using ESP-IDF
 * 
 * @details Implements platform-independent I2C abstraction on ESP32-S3.
 *          Includes reference counting for driver installation/deletion and
 *          I2C bus release logic.
 * 
 * @author Stefano Fante (STLINE SRL)
 * @date 2025
 */

#include "support/fsl_adapter_i2c.h"

#if defined(PD_CONFIG_TARGET_ESP32S3) && (PD_CONFIG_TARGET_ESP32S3)

#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "support/fsl_common.h"
#include "usb_pd.h"

/** @brief I2C command timeout in milliseconds */
#define I2C_CMD_TIMEOUT_MS (100)

/**
 * @brief Internal I2C master handle structure for ESP32-S3
 */
typedef struct _hal_i2c_master_handle
{
    i2c_port_t port;  /**< ESP32 I2C port number */
} hal_i2c_master_handle_obj_t;

/**
 * @brief Internal I2C hardware configuration
 */
typedef struct _pd_i2c_hw_config_internal
{
    bool configured;     /**< True if configured via PD_I2cConfigureEsp32() */
    int sdaPin;          /**< GPIO pin for SDA */
    int sclPin;          /**< GPIO pin for SCL */
    uint32_t busSpeed;   /**< I2C bus frequency in Hz */
    bool internalPullup; /**< Enable internal pull-up resistors */
} pd_i2c_hw_config_internal_t;

#if I2C_NUM_MAX < 2
#error "ESP32-S3 port requires at least two I2C controllers"
#endif

/** @brief Global I2C configuration storage */
static pd_i2c_hw_config_internal_t s_i2cHwConfig[I2C_NUM_MAX];

/** @brief Reference counting for I2C driver instances */
static int s_i2cDriverRefCount[I2C_NUM_MAX];

static void PD_I2cReleaseBusByInstance(uint8_t instance);
static void PD_I2cReleaseBus0(void);
static void PD_I2cReleaseBus1(void);

/** @brief Array of bus release function pointers */
static const PD_I2cReleaseBus s_releaseHandlers[I2C_NUM_MAX] = {
    PD_I2cReleaseBus0,
    PD_I2cReleaseBus1,
};

/**
 * @brief Map instance number to ESP-IDF i2c_port_t
 * 
 * @param[in] instance I2C instance number
 * @return ESP-IDF port number or I2C_NUM_MAX if invalid
 */
static inline i2c_port_t PD_I2cMapInstance(uint8_t instance)
{
    return (instance < I2C_NUM_MAX) ? (i2c_port_t)instance : I2C_NUM_MAX;
}

/**
 * @brief Install ESP-IDF I2C driver for a specific port
 * 
 * @param[in] port ESP-IDF I2C port number
 * @return ESP_OK on success, error code otherwise
 * 
 * @details Resets GPIO pins, calls i2c_param_config() and i2c_driver_install().
 */
static esp_err_t PD_I2cInstallDriver(i2c_port_t port)
{
    const pd_i2c_hw_config_internal_t *cfg = &s_i2cHwConfig[port];
    if (!cfg->configured)
    {
        return ESP_ERR_INVALID_STATE;
    }

    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = cfg->sdaPin,
        .sda_pullup_en    = cfg->internalPullup ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .scl_io_num       = cfg->sclPin,
        .scl_pullup_en    = cfg->internalPullup ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .master.clk_speed = cfg->busSpeed,
        .clk_flags        = 0,
    };

    (void)gpio_reset_pin(cfg->sdaPin);
    (void)gpio_reset_pin(cfg->sclPin);

    esp_err_t err = i2c_param_config(port, &conf);
    if (err != ESP_OK)
    {
        return err;
    }

    err = i2c_driver_install(port, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK)
    {
        return err;
    }

    return ESP_OK;
}

/**
 * @brief Initialize an I2C master handle
 * 
 * @param[out] handle Pointer to handle (will be allocated)
 * @param[in] config Configuration (instance number)
 * 
 * @return kStatus_HAL_I2cSuccess on success, kStatus_HAL_I2cError otherwise
 * 
 * @details Installs the driver on first use (reference-counted).
 * 
 * @note Hardware must first be configured via PD_I2cConfigureEsp32().
 */
hal_i2c_status_t HAL_I2cMasterInit(hal_i2c_master_handle_t *handle, const hal_i2c_master_config_t *config)
{
    if ((handle == NULL) || (config == NULL))
    {
        return kStatus_HAL_I2cError;
    }

    i2c_port_t port = PD_I2cMapInstance(config->instance);
    if (port >= I2C_NUM_MAX)
    {
        return kStatus_HAL_I2cError;
    }

    if (!s_i2cHwConfig[port].configured)
    {
        return kStatus_HAL_I2cError;
    }

    hal_i2c_master_handle_obj_t *obj = calloc(1, sizeof(*obj));
    if (obj == NULL)
    {
        return kStatus_HAL_I2cError;
    }

    obj->port = port;

    taskENTER_CRITICAL();
    if (s_i2cDriverRefCount[port] == 0)
    {
        if (PD_I2cInstallDriver(port) != ESP_OK)
        {
            taskEXIT_CRITICAL();
            free(obj);
            return kStatus_HAL_I2cError;
        }
    }
    s_i2cDriverRefCount[port]++;
    taskEXIT_CRITICAL();

    *handle = obj;
    return kStatus_HAL_I2cSuccess;
}

/**
 * @brief Deinitialize an I2C master handle
 * 
 * @param[in,out] handle Pointer to handle (will be freed)
 * @return kStatus_HAL_I2cSuccess on success, kStatus_HAL_I2cError otherwise
 * 
 * @details Deletes the driver when last handle is deinitialized (reference-counted).
 */
hal_i2c_status_t HAL_I2cMasterDeinit(hal_i2c_master_handle_t *handle)
{
    if ((handle == NULL) || (*handle == NULL))
    {
        return kStatus_HAL_I2cError;
    }

    hal_i2c_master_handle_obj_t *obj = *handle;
    taskENTER_CRITICAL();
    if (s_i2cDriverRefCount[obj->port] > 0)
    {
        s_i2cDriverRefCount[obj->port]--;
        if (s_i2cDriverRefCount[obj->port] == 0)
        {
            i2c_driver_delete(obj->port);
        }
    }
    taskEXIT_CRITICAL();

    free(obj);
    *handle = NULL;

    return kStatus_HAL_I2cSuccess;
}

/**
 * @brief Write multi-byte sub-address to I2C command link
 * 
 * @param[in] cmd I2C command handle
 * @param[in] subaddress Sub-address (register address)
 * @param[in] subaddressSize Number of bytes in sub-address (1 or 2 typical)
 * 
 * @return ESP_OK on success, error code otherwise
 * 
 * @details Writes sub-address bytes in big-endian order.
 */
static esp_err_t PD_I2cWriteSubaddress(i2c_cmd_handle_t cmd, uint32_t subaddress, uint8_t subaddressSize)
{
    for (int8_t shift = (int8_t)subaddressSize - 1; shift >= 0; --shift)
    {
        uint8_t byte = (uint8_t)((subaddress >> (uint32_t)(shift * 8)) & 0xFFU);
        esp_err_t err = i2c_master_write_byte(cmd, byte, true);
        if (err != ESP_OK)
        {
            return err;
        }
    }
    return ESP_OK;
}

/**
 * @brief Perform a blocking I2C transfer (read or write with optional sub-address)
 * 
 * @param[in] handle I2C master handle
 * @param[in,out] xfer Transfer descriptor
 * 
 * @return kStatus_HAL_I2cSuccess on success, kStatus_HAL_I2cError otherwise
 * 
 * @details Builds an I2C command link and executes it via i2c_master_cmd_begin().
 *          Supports register read/write with multi-byte sub-addresses.
 */
hal_i2c_status_t HAL_I2cMasterTransferBlocking(hal_i2c_master_handle_t *handle, hal_i2c_master_transfer_t *xfer)
{
    if ((handle == NULL) || (*handle == NULL) || (xfer == NULL))
    {
        return kStatus_HAL_I2cError;
    }

    hal_i2c_master_handle_obj_t *obj = *handle;
    i2c_cmd_handle_t cmd           = i2c_cmd_link_create();
    if (cmd == NULL)
    {
        return kStatus_HAL_I2cError;
    }

    esp_err_t err = i2c_master_start(cmd);
    if (err != ESP_OK)
    {
        i2c_cmd_link_delete(cmd);
        return kStatus_HAL_I2cError;
    }

    if (xfer->direction == kHAL_I2cRead)
    {
        err = i2c_master_write_byte(cmd, (uint8_t)((xfer->slaveAddress << 1u) | I2C_MASTER_WRITE), true);
        if ((err == ESP_OK) && (xfer->subaddressSize > 0U))
        {
            err = PD_I2cWriteSubaddress(cmd, xfer->subaddress, xfer->subaddressSize);
        }
        if (err == ESP_OK)
        {
            err = i2c_master_start(cmd);
        }
        if (err == ESP_OK)
        {
            err = i2c_master_write_byte(cmd, (uint8_t)((xfer->slaveAddress << 1u) | I2C_MASTER_READ), true);
        }
        if (err == ESP_OK)
        {
            if (xfer->dataSize > 1U)
            {
                err = i2c_master_read(cmd, xfer->data, xfer->dataSize - 1U, I2C_MASTER_ACK);
            }
            if ((err == ESP_OK) && (xfer->dataSize > 0U))
            {
                err = i2c_master_read_byte(cmd, &xfer->data[xfer->dataSize - 1U], I2C_MASTER_NACK);
            }
        }
    }
    else
    {
        err = i2c_master_write_byte(cmd, (uint8_t)((xfer->slaveAddress << 1u) | I2C_MASTER_WRITE), true);
        if ((err == ESP_OK) && (xfer->subaddressSize > 0U))
        {
            err = PD_I2cWriteSubaddress(cmd, xfer->subaddress, xfer->subaddressSize);
        }
        if ((err == ESP_OK) && (xfer->dataSize > 0U))
        {
            err = i2c_master_write(cmd, xfer->data, xfer->dataSize, true);
        }
    }

    if (err == ESP_OK)
    {
        err = i2c_master_stop(cmd);
    }

    if (err == ESP_OK)
    {
        err = i2c_master_cmd_begin(obj->port, cmd, pdMS_TO_TICKS(I2C_CMD_TIMEOUT_MS));
    }

    i2c_cmd_link_delete(cmd);
    return (err == ESP_OK) ? kStatus_HAL_I2cSuccess : kStatus_HAL_I2cError;
}

/**
 * @brief Configure I2C hardware for a specific instance
 * 
 * @param[in] instance I2C instance number
 * @param[in] config Hardware configuration
 * @param[out] releaseBusOut Pointer to receive bus release function pointer
 * 
 * @return kStatus_PD_Success on success, kStatus_PD_Error otherwise
 * 
 * @note This must be called before HAL_I2cMasterInit() for the instance.
 */
pd_status_t PD_I2cConfigureEsp32(uint8_t instance,
                                 const pd_i2c_esp32_config_t *config,
                                 PD_I2cReleaseBus *releaseBusOut)
{
    if ((config == NULL) || (config->sdaPin < 0) || (config->sclPin < 0) || (config->busSpeed_Hz == 0U))
    {
        return kStatus_PD_Error;
    }

    i2c_port_t port = PD_I2cMapInstance(instance);
    if (port >= I2C_NUM_MAX)
    {
        return kStatus_PD_Error;
    }

    taskENTER_CRITICAL();
    s_i2cHwConfig[port].configured      = true;
    s_i2cHwConfig[port].sdaPin          = config->sdaPin;
    s_i2cHwConfig[port].sclPin          = config->sclPin;
    s_i2cHwConfig[port].busSpeed        = config->busSpeed_Hz;
    s_i2cHwConfig[port].internalPullup  = config->enableInternalPullup;
    taskEXIT_CRITICAL();

    if ((releaseBusOut != NULL) && (port < I2C_NUM_MAX))
    {
        *releaseBusOut = s_releaseHandlers[port];
    }

    return kStatus_PD_Success;
}

/**
 * @brief Release a stuck I2C bus by toggling SCL/SDA pins
 * 
 * @param[in] instance I2C instance number
 * 
 * @details Generates 9 SCL clock cycles to release devices stuck in clock
 *          stretching. Temporarily reconfigures I2C pins as GPIO outputs.
 */
static void PD_I2cReleaseBusByInstance(uint8_t instance)
{
    i2c_port_t port = PD_I2cMapInstance(instance);
    if (port >= I2C_NUM_MAX)
    {
        return;
    }

    const pd_i2c_hw_config_internal_t *cfg = &s_i2cHwConfig[port];
    if (!cfg->configured)
    {
        return;
    }

    gpio_reset_pin(cfg->sdaPin);
    gpio_reset_pin(cfg->sclPin);
    gpio_set_direction(cfg->sdaPin, GPIO_MODE_OUTPUT_OD);
    gpio_set_direction(cfg->sclPin, GPIO_MODE_OUTPUT_OD);

    gpio_set_level(cfg->sdaPin, 1);
    gpio_set_level(cfg->sclPin, 1);
    esp_rom_delay_us(5);

    for (uint8_t i = 0; i < 9U; ++i)
    {
        gpio_set_level(cfg->sclPin, 0);
        esp_rom_delay_us(5);
        gpio_set_level(cfg->sclPin, 1);
        esp_rom_delay_us(5);
    }

    gpio_set_level(cfg->sdaPin, 0);
    esp_rom_delay_us(5);
    gpio_set_level(cfg->sdaPin, 1);
    esp_rom_delay_us(5);
}

/**
 * @brief Release bus for I2C instance 0
 */
static void PD_I2cReleaseBus0(void)
{
    PD_I2cReleaseBusByInstance(0);
}

/**
 * @brief Release bus for I2C instance 1
 */
static void PD_I2cReleaseBus1(void)
{
    PD_I2cReleaseBusByInstance(1);
}

#endif /* PD_CONFIG_TARGET_ESP32S3 */
