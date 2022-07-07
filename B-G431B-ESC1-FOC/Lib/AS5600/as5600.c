/*
 * as5600.c
 *
 *  Created on: Jul 6, 2022
 *      Author: TK
 */

#include "as5600.h"

void AS5600_init(AS5600 *sensor, I2C_HandleTypeDef *hi2c) {
  sensor->hi2c = hi2c;
  sensor->n_rotations = 0;
  sensor->position = 0;
  sensor->position_prev = 0;
}

float AS5600_getPosition(AS5600 *sensor) {
  uint8_t buf[2];
  HAL_I2C_Mem_Read(sensor->hi2c, 0b0110110<<1, 0x0E, I2C_MEMADD_SIZE_8BIT, buf, 2, 10);

  const uint16_t cpr = 4096;
  uint16_t raw_angle = ((uint16_t)buf[0] << 8) | buf[1];
  float position_relative = ((float)raw_angle / (float)cpr) * (2*M_PI);

  float delta_position = position_relative - sensor->position_prev;

  if (fabsf(delta_position) > 0.8 * (2*M_PI)) {
    sensor->n_rotations += (delta_position > 0) ? -1 : 1;
  }

  sensor->position_prev = position_relative;

  return position_relative + sensor->n_rotations * (2*M_PI);
}

