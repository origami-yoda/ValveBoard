#include "ltc2990.h"
#include <stdio.h>

static HAL_StatusTypeDef LTC2990_ReadReg16(LTC2990_handle *handle, uint8_t reg_msb, uint16_t *value)
{
    uint8_t data[2];
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(handle->hi2c, handle->device_addr, reg_msb, 1, &data[0], 1, LTC2990_I2C_TIMEOUT);

    if (status != HAL_OK)
    {
        return status;
    }

    status = HAL_I2C_Mem_Read(handle->hi2c, handle->device_addr, reg_msb + 1, 1, &data[1], 1, LTC2990_I2C_TIMEOUT);

    if (status != HAL_OK)
    {
        return status;
    }

    *value = (data[0] << 8 | data[1]);

    return HAL_OK;
}

static HAL_StatusTypeDef LTC2990_WriteReg(LTC2990_handle *handle, uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(handle->hi2c, handle->device_addr, reg, 1, &value, 1, LTC2990_I2C_TIMEOUT);
}

HAL_StatusTypeDef LTC2990_Init(LTC2990_handle *handle, I2C_HandleTypeDef *hi2c)
{
    handle->hi2c = hi2c;
    handle->device_addr = LTC2990_I2C_ADDR;

    uint8_t config = LTC2990_SINGLE_ACQUSITION | LTC2990_TEMP_FORMAT_CELSIUS | LTC2990_CONFIG;

    HAL_StatusTypeDef status = LTC2990_WriteReg(handle, LTC2990_CONTROL_REG, config);
    if (status != HAL_OK)
    {
        return status;
    }

    osDelay(10);
    return HAL_OK;
}

HAL_StatusTypeDef LTC2990_ReadVoltages(LTC2990_handle *handle, float *voltage)
{
    HAL_StatusTypeDef status;

    // Any value written to the trigger register will start conversion
    uint8_t trigger = 0x01;

    status = LTC2990_WriteReg(handle, LTC2990_TRIGGER_REG, trigger);

    if (status != HAL_OK)
    {
        printf("Trigger failed\r\n");
        return status;
    }

    uint32_t start_time = HAL_GetTick();
    uint8_t status_reg;

    do
    {
        osDelay(1);
        status = HAL_I2C_Mem_Read(handle->hi2c, handle->device_addr, LTC2990_STATUS_REG, 1, &status_reg, 1, LTC2990_I2C_TIMEOUT);
        if (status != HAL_OK)
        {
            printf("Status Register read failed\r\n");
            return status;
        }

        if ((HAL_GetTick() - start_time) > LTC2990_CONVERSION_TIMEOUT)
        {
            printf("Took too long\r\n");
            return HAL_TIMEOUT;
        }
    } while (status_reg & LTC2990_BUSY_BIT);

    uint16_t v1_raw, v2_raw;
    status = LTC2990_ReadReg16(handle, LTC2990_V1_MSB_REG, &v1_raw);
    if (status != HAL_OK)
    {
        printf("Failed reading V1\r\n");
        return status;
    }

    status = LTC2990_ReadReg16(handle, LTC2990_V2_MSB_REG, &v2_raw);
    if (status != HAL_OK)
    {
        printf("Failed reading V2\r\n");
        return status;
    }

    // validate data
    if (!(v1_raw & 0x8000) || !(v2_raw & 0x8000))
    {
        printf("Invalid Data\r\n");
        return HAL_ERROR;
    }

    // discard the two MSB bits
    // first bit is data-valid bit
    // second bit is sign bit
    // we don't need the sign bit because this is a single-ended measurement
    uint16_t v1_adc = v1_raw & 0x3FFF;
    uint16_t v2_adc = v2_raw & 0x3FFF;

    printf("Extracted V1: %d\r\n", v1_adc);

    // convert ADC values to voltage
    float v1_input = v1_adc * SINGLE_ENDED_LSB;
    float v2_input = v2_adc * SINGLE_ENDED_LSB;

    printf("Normalized V1: %f\r\n", v1_input);

    // apply voltage divider ratios
    voltage[0] = v1_input * V1_DIVIDER_RATIO;
    voltage[1] = v2_input * V2_DIVIDER_RATIO;

    printf("Divider V1: %f\r\n", voltage[0]);
    return HAL_OK;
}