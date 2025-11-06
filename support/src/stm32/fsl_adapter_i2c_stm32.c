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

#define I2C_DEFAULT_TIMEOUT_MS (10U)

typedef struct _hal_i2c_master_handle
{
    I2C_HandleTypeDef *hi2c;
    uint32_t timeoutMs;
} hal_i2c_master_handle_obj_t;

typedef struct _pd_i2c_hw_config_internal
{
    I2C_HandleTypeDef *hi2c;
    uint32_t timeoutMs;
    GPIO_TypeDef *sclPort;
    uint16_t sclPin;
    GPIO_TypeDef *sdaPort;
    uint16_t sdaPin;
} pd_i2c_hw_config_internal_t;

static pd_i2c_hw_config_internal_t s_i2cHwConfig[USB_PD_I2C_INSTANCE_COUNT];

static void PD_I2cReleaseBusByInstance(uint8_t instance);
static void PD_I2cReleaseBus0(void);
static void PD_I2cReleaseBus1(void);
static void PD_I2cReleaseBus2(void);
static void PD_I2cReleaseBus3(void);

static const PD_I2cReleaseBus s_releaseHandlers[USB_PD_I2C_INSTANCE_COUNT] = {
    PD_I2cReleaseBus0,
    PD_I2cReleaseBus1,
    PD_I2cReleaseBus2,
    PD_I2cReleaseBus3,
};

static inline bool PD_I2cInstanceValid(uint8_t instance)
{
    return instance < USB_PD_I2C_INSTANCE_COUNT;
}

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

hal_i2c_status_t HAL_I2cMasterTransferBlocking(hal_i2c_master_handle_t *handle, hal_i2c_master_transfer_t *xfer)
{
    if ((handle == NULL) || (*handle == NULL) || (xfer == NULL))
    {
        return kStatus_HAL_I2cError;
    }

    hal_i2c_master_handle_obj_t *obj = *handle;
    return PD_I2cMemTransfer(obj, xfer, obj->timeoutMs);
}

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

static void PD_I2cReleaseBus0(void)
{
    PD_I2cReleaseBusByInstance(0);
}

static void PD_I2cReleaseBus1(void)
{
    PD_I2cReleaseBusByInstance(1);
}

static void PD_I2cReleaseBus2(void)
{
    PD_I2cReleaseBusByInstance(2);
}

static void PD_I2cReleaseBus3(void)
{
    PD_I2cReleaseBusByInstance(3);
}

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
