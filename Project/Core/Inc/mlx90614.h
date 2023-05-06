#ifndef __MLX90614_H__
#define __MLX90614_H__

#include "stm32f1xx_hal.h"
#include "main.h"
#include "i2c.h"

#define TEMPERATUREMIN 0x27AD
#define TEMPERATUREMAX 0x7FFF

float MLX90614_GetTemperature();

#endif
