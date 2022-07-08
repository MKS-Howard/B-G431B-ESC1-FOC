/*
 * as5600.c
 *
 *  Created on: Jul 6, 2022
 *      Author: TK
 */

#include "as5600.h"

void AS5600_init(AS5600 *sensor, I2C_HandleTypeDef *hi2c, TIM_HandleTypeDef *htim) {
  sensor->hi2c = hi2c;
  sensor->htim = htim;
  sensor->n_rotations = 0;
  sensor->prev_time = 0;
  sensor->prev_position = 0;
  sensor->position = 0;
  sensor->velocity = 0;
}

void AS5600_update(AS5600 *sensor) {
  uint8_t buf[2];
  HAL_I2C_Mem_Read(sensor->hi2c, 0b0110110<<1, 0x0E, I2C_MEMADD_SIZE_8BIT, buf, 2, 10);

  const uint16_t cpr = 4096;
  uint16_t raw_angle = ((uint16_t)buf[0] << 8) | buf[1];
  float position_relative = ((float)raw_angle / (float)cpr) * (2*M_PI);

  float delta_position = position_relative - sensor->prev_position;

  if (fabsf(delta_position) > 0.8 * (2*M_PI)) {
    sensor->n_rotations += (delta_position > 0) ? -1 : 1;
  }

  uint16_t curr_time = __HAL_TIM_GET_COUNTER(sensor->htim);  // counter is 1MHz
  uint16_t delta_time = curr_time - sensor->prev_time;
  if (delta_time == 0) {
    delta_time = 1;
  }

  float curr_position = position_relative + sensor->n_rotations * (2*M_PI);

  const float VEL_ALPHA = 0.2;

  sensor->prev_position = position_relative;
  sensor->prev_time = curr_time;
  sensor->velocity = (VEL_ALPHA * ((curr_position - sensor->position) * 100000 / (float)delta_time)) + ((1-VEL_ALPHA) * sensor->velocity);
  sensor->position = curr_position;
}
