#include "mlx90614.h"

uint8_t Data[2] = {0};

float MLX90614_GetTemperature()
{
	uint16_t temp = 0;
  float temperatureValue = 0;
  HAL_I2C_Mem_Read(&hi2c1, 0x00, 0x07, 1, Data, 2, 0x20);
  temp = ((Data[1] << 8) | Data[0]);
  if ((TEMPERATUREMIN <= temp) && (temp <= TEMPERATUREMAX))
  {
		temperatureValue = (((float)temp * 2) - 27315) / 100;
  }
  else
  {
		temperatureValue = -100.0;
  }
  return temperatureValue;
}
