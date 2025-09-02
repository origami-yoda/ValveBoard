#ifndef LTC2990_H
#define LTC2990_H

#include "stm32g4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#define LTC2990_I2C_ADDR (0x4C << 1)

#define LTC2990_STATUS_REG 0x00
#define LTC2990_CONTROL_REG 0x01
#define LTC2990_TRIGGER_REG 0x02
#define LTC2990_TINT_MSB_REG 0x04
#define LTC2990_TINT_LSB_REG 0x05
#define LTC2990_V1_MSB_REG 0x06
#define LTC2990_V1_LSB_REG 0x07
#define LTC2990_V2_MSB_REG 0x08
#define LTC2990_V2_LSB_REG 0x09
#define LTC2990_V3_MSB_REG 0x0A
#define LTC2990_V3_LSB_REG 0x0B
#define LTC2990_V4_MSB_REG 0x0C
#define LTC2990_V4_LSB_REG 0x0D
#define LTC2990_VCC_MSB_REG 0x0E
#define LTC2990_VCC_LSB_REG 0x0F

#define LTC2990_TEMP_FORMAT_CELSIUS 0x00
#define LTC2990_TEMP_FORMAT_KELVIN 0x80
#define LTC2990_REPEAT_ACQUSITION 0x00
#define LTC2990_SINGLE_ACQUSITION 0x40

#define LTC2990_MODE_INTERNAL_TEMP_ONLY 0x00
#define LTC2990_MODE_TR1_V1_V1V2 0x08 // TR1, V1 or V1 – V2 Only per Mode
#define LTC2990_MODE_TR1_V3_V3V4 0x10 // TR2, V3 or V3 – V4 Only per Mode
#define LTC2990_MODE_ALL_MEASUREMENTS 0x18

#define LTC2990_MODE_V1_V2_TR2 0x00  // V1, V2, TR2 (default)
#define LTC2990_MODE_V1V2_TR2 0x01  // V1-V2, TR2
#define LTC2990_MODE_V1V2_V3_V4 0x02  // V1-V2, V3, V4
#define LTC2990_MODE_TR1_V3_V4 0x03  // TR1, V3, V4
#define LTC2990_MODE_TR1_V3V4 0x04  // TR1, V3-V4
#define LTC2990_MODE_TR1_TR2 0x05  // TR1, TR2
#define LTC2990_MODE_V1V2_V3V4 0x06  // V1-V2, V3-V4
#define LTC2990_MODE_V1_V2_V3_V4 0x07  // V1, V2, V3, V4

#define LTC2990_CONFIG (LTC2990_MODE_ALL_MEASUREMENTS | LTC2990_MODE_V1_V2_V3_V4)

// Status register bits
#define LTC2990_DATA_VALID_BIT 0x80
#define LTC2990_BUSY_BIT 0x01

#define V1_DIVIDER_RATIO ((18.0f + 10.0f)/10.0f) // ((R26)/(R26 + R29))^-1
#define V2_DIVIDER_RATIO ((30.0f + 10.0f)/10.0f) // ((R33)/(R33 + R32))^-1

#define SINGLE_ENDED_LSB 305.18e-6f

#define LTC2990_I2C_TIMEOUT 50
#define LTC2990_CONVERSION_TIMEOUT 100

typedef struct
{
    I2C_HandleTypeDef *hi2c;
    uint8_t device_addr;
} LTC2990_handle;

HAL_StatusTypeDef LTC2990_Init(LTC2990_handle *handle, I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef LTC2990_ReadVoltages(LTC2990_handle *handle, float *voltage);

#endif /* LTC2990_H */