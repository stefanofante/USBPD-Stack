/**
 * @file fsl_adapter_i2c_stm32.c
 * @brief I2C master HAL adapter for STM32 using Cube HAL
 * 
 * @details Implements platform-independent I2C abstraction on STM32 F3/F4/H7.
 *          Includes I2C bus release (clock stretching recovery) logic.
 * 
 * @author Stefano Fante (STLINE SRL)
 * @date 2025
 */

#include "support/fsl_adapter_i2c.h"

#if defined(PD_CONFIG_TARGET_STM32) && (PD_CONFIG_TARGET_STM32)

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "usb_pd.h"

#if defined(STM32F3xx)
#include "stm32f3xx_hal.h"
#elif defined(STM32F4xx)
#include "stm32f4xx_hal.h"
#elif defined(STM32H7xx)
#include "stm32h7xx_hal.h"
#else
#error "Define STM32F3xx, STM32F4xx, or STM32H7xx to include the correct HAL headers."
#endif

/** @brief Default I2C timeout in milliseconds */
#define I2C_DEFAULT_TIMEOUT_MS (10U)

/**
 * @brief Internal I2C master handle structure for STM32
 */
typedef struct _hal_i2c_master_handle
{
    I2C_HandleTypeDef *hi2c;   /**< STM32 HAL I2C handle */
    uint32_t timeoutMs;        /**< Transfer timeout in milliseconds */
} hal_i2c_master_handle_obj_t;

/**
 * @brief Internal I2C hardware configuration
 */
typedef struct _pd_i2c_hw_config_internal
{
    I2C_HandleTypeDef *hi2c;   /**< STM32 HAL I2C handle */
    uint32_t timeoutMs;        /**< Transfer timeout in milliseconds */
    GPIO_TypeDef *sclPort;     /**< GPIO port for SCL (for bus release) */
    uint16_t sclPin;           /**< GPIO pin mask for SCL */
    GPIO_TypeDef *sdaPort;     /**< GPIO port for SDA (for bus release) */
    uint16_t sdaPin;           /**< GPIO pin mask for SDA */
} pd_i2c_hw_config_internal_t;

/** @brief Global I2C configuration storage (supports up to 4 instances) */
static pd_i2c_hw_config_internal_t s_i2cHwConfig[USB_PD_I2C_INSTANCE_COUNT];

static void PD_I2cReleaseBusByInstance(uint8_t instance);
static void PD_I2cReleaseBus0(void);
static void PD_I2cReleaseBus1(void);
static void PD_I2cReleaseBus2(void);
static void PD_I2cReleaseBus3(void);

/** @brief Array of bus release function pointers (one per I2C instance) */
static const PD_I2cReleaseBus s_releaseHandlers[USB_PD_I2C_INSTANCE_COUNT] = {
    PD_I2cReleaseBus0,
    PD_I2cReleaseBus1,
    PD_I2cReleaseBus2,
    PD_I2cReleaseBus3,
};

/**
 * @brief Check if an I2C instance number is valid
 * 
 * @param[in] instance I2C instance number
 * @return true if instance is within valid range, false otherwise
 */
static inline bool PD_I2cInstanceValid(uint8_t instance)
{
    return instance < USB_PD_I2C_INSTANCE_COUNT;
}

/**
 * @brief Perform a memory-addressed I2C transfer (read or write)
 * 
 * @param[in] obj I2C handle object
 * @param[in,out] xfer Transfer descriptor (address, data, direction)
 * @param[in] timeout Timeout in milliseconds
 * 
 * @return kStatus_HAL_I2cSuccess on success, kStatus_HAL_I2cError otherwise
 * 
 * @details Supports register read/write with 0, 1, or 2-byte sub-addresses.
 */
static hal_i2c_status_t PD_I2cMemTransfer(hal_i2c_master_handle_obj_t *obj,
                                          hal_i2c_master_transfer_t *xfer,
                                          uint32_t timeout)
{
    HAL_StatusTypeDef status;

    if (xfer->direction == kHAL_I2cRead)
    {
        if (xfer->subaddressSize == 0U)
        {
            status = HAL_I2C_Master_Receive(obj->hi2c,
                                            (uint16_t)(xfer->slaveAddress << 1U),
                                            xfer->data,
                                            xfer->dataSize,
                                            timeout);
        }
        else if (xfer->subaddressSize <= 2U)
        {
            uint16_t memAddSize = (xfer->subaddressSize == 1U) ? I2C_MEMADD_SIZE_8BIT : I2C_MEMADD_SIZE_16BIT;
            status = HAL_I2C_Mem_Read(obj->hi2c,
                                      (uint16_t)(xfer->slaveAddress << 1U),
                                      (uint16_t)xfer->subaddress,
                                      memAddSize,
                                      xfer->data,
                                      xfer->dataSize,
                                      timeout);
        }
        else
        {
            status = HAL_ERROR;
        }
    }
    else
    {
        if (xfer->subaddressSize == 0U)
        {
            status = HAL_I2C_Master_Transmit(obj->hi2c,
                                             (uint16_t)(xfer->slaveAddress << 1U),
                                             xfer->data,
                                             xfer->dataSize,
                                             timeout);
        }
        else if (xfer->subaddressSize <= 2U)
        {
            uint16_t memAddSize = (xfer->subaddressSize == 1U) ? I2C_MEMADD_SIZE_8BIT : I2C_MEMADD_SIZE_16BIT;
            status = HAL_I2C_Mem_Write(obj->hi2c,
                                       (uint16_t)(xfer->slaveAddress << 1U),
                                       (uint16_t)xfer->subaddress,
                                       memAddSize,
                                       xfer->data,
                                       xfer->dataSize,
                                       timeout);
        }
        else
        {
            status = HAL_ERROR;
        }
    }

    return (status == HAL_OK) ? kStatus_HAL_I2cSuccess : kStatus_HAL_I2cError;
}

/**
 * @brief Initialize an I2C master handle
 * 
 * @param[out] handle Pointer to handle (will be allocated)
 * @param[in] config Configuration (instance number)
 * 
 * @return kStatus_HAL_I2cSuccess on success, kStatus_HAL_I2cError otherwise
 * 
 * @note The hardware must first be configured via PD_I2cConfigureStm32().
 */
hal_i2c_status_t HAL_I2cMasterInit(hal_i2c_master_handle_t *handle, const hal_i2c_master_config_t *config)
{
    if ((handle == NULL) || (config == NULL) || !PD_I2cInstanceValid(config->instance))
    {
        return kStatus_HAL_I2cError;
    }

    pd_i2c_hw_config_internal_t *hwCfg = &s_i2cHwConfig[config->instance];
    if (hwCfg->hi2c == NULL)
    {
        return kStatus_HAL_I2cError;
    }

    hal_i2c_master_handle_obj_t *obj = calloc(1, sizeof(*obj));
    if (obj == NULL)
    {
        return kStatus_HAL_I2cError;
    }

    obj->hi2c     = hwCfg->hi2c;
    obj->timeoutMs = (hwCfg->timeoutMs != 0U) ? hwCfg->timeoutMs : I2C_DEFAULT_TIMEOUT_MS;

    *handle = obj;
    return kStatus_HAL_I2cSuccess;
}

/**
 * @brief Deinitialize an I2C master handle
 * 
 * @param[in,out] handle Pointer to handle (will be freed and set to NULL)
 * @return kStatus_HAL_I2cSuccess on success, kStatus_HAL_I2cError otherwise
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
 * @brief Perform a blocking I2C transfer
 * 
 * @param[in] handle I2C master handle
 * @param[in,out] xfer Transfer descriptor
 * 
 * @return kStatus_HAL_I2cSuccess on success, kStatus_HAL_I2cError otherwise
 */
hal_i2c_status_t HAL_I2cMasterTransferBlocking(hal_i2c_master_handle_t *handle, hal_i2c_master_transfer_t *xfer)
{
    if ((handle == NULL) || (*handle == NULL) || (xfer == NULL))
    {
        return kStatus_HAL_I2cError;
    }

    hal_i2c_master_handle_obj_t *obj = *handle;
    return PD_I2cMemTransfer(obj, xfer, obj->timeoutMs);
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
    if (!PD_I2cInstanceValid(instance))
    {
        return;
    }

    const pd_i2c_hw_config_internal_t *cfg = &s_i2cHwConfig[instance];
    if ((cfg->sclPort == NULL) || (cfg->sdaPort == NULL))
    {
        return;
    }

    GPIO_InitTypeDef init = {
        .Mode      = GPIO_MODE_OUTPUT_OD,
        .Pull      = GPIO_PULLUP,
        .Speed     = GPIO_SPEED_FREQ_HIGH,
        .Alternate = 0,
    };

    init.Pin = cfg->sclPin;
    HAL_GPIO_Init(cfg->sclPort, &init);
    init.Pin = cfg->sdaPin;
    HAL_GPIO_Init(cfg->sdaPort, &init);

    HAL_GPIO_WritePin(cfg->sclPort, cfg->sclPin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(cfg->sdaPort, cfg->sdaPin, GPIO_PIN_SET);
    HAL_Delay(1);

    for (uint8_t i = 0; i < 9U; ++i)
    {
        HAL_GPIO_WritePin(cfg->sclPort, cfg->sclPin, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(cfg->sclPort, cfg->sclPin, GPIO_PIN_SET);
        HAL_Delay(1);
    }

    HAL_GPIO_WritePin(cfg->sdaPort, cfg->sdaPin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(cfg->sdaPort, cfg->sdaPin, GPIO_PIN_SET);
    HAL_Delay(1);
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

/**
 * @brief Release bus for I2C instance 2
 */
static void PD_I2cReleaseBus2(void)
{
    PD_I2cReleaseBusByInstance(2);
}

/**
 * @brief Release bus for I2C instance 3
 */
static void PD_I2cReleaseBus3(void)
{
    PD_I2cReleaseBusByInstance(3);
}

/**
 * @brief Configure I2C hardware for a specific instance
 * 
 * @param[in] instance I2C instance number (0-3)
 * @param[in] config Hardware configuration (HAL handle, GPIOs, timeout)
 * @param[out] releaseBusOut Pointer to receive bus release function pointer
 * 
 * @return kStatus_PD_Success on success, kStatus_PD_Error otherwise
 * 
 * @note This must be called before HAL_I2cMasterInit() for the instance.
 */
pd_status_t PD_I2cConfigureStm32(uint8_t instance,
                                 const pd_i2c_stm32_config_t *config,
                                 PD_I2cReleaseBus *releaseBusOut)
{
    if (!PD_I2cInstanceValid(instance) || (config == NULL) || (config->hi2c == NULL))
    {
        return kStatus_PD_Error;
    }

    pd_i2c_hw_config_internal_t *hwCfg = &s_i2cHwConfig[instance];
    hwCfg->hi2c       = (I2C_HandleTypeDef *)config->hi2c;
    hwCfg->timeoutMs  = config->timeoutMs;
    hwCfg->sclPort    = (GPIO_TypeDef *)config->sclPort;
    hwCfg->sclPin     = config->sclPin;
    hwCfg->sdaPort    = (GPIO_TypeDef *)config->sdaPort;
    hwCfg->sdaPin     = config->sdaPin;

    if (releaseBusOut != NULL)
    {
        *releaseBusOut = s_releaseHandlers[instance];
    }

    return kStatus_PD_Success;
}

#endif /* PD_CONFIG_TARGET_STM32 */
