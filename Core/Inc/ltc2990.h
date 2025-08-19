#ifndef LTC2990_H
#define LTC2990_H

#include "stm32g4xx_hal.h"
#include <stdbool.h>

#ifndef LTC2990_ADDR7        
#define LTC2990_ADDR7 0x41
#endif

void LTC2990_Init(I2C_HandleTypeDef *hi2c);

void LTC2990_ReadVoltages(I2C_HandleTypeDef *hi2c, float out_v[4]);

#endif /* LTC2990_H */