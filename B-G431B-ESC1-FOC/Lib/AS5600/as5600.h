/*
 * as5600.h
 *
 *  Created on: Jul 6, 2022
 *      Author: TK
 */

#ifndef AS5600_AS5600_H_
#define AS5600_AS5600_H_

#include <math.h>
#include "stm32g4xx_hal.h"

typedef struct {
  I2C_HandleTypeDef *hi2c;
  TIM_HandleTypeDef *htim;
  int32_t n_rotations;
  uint16_t prev_time;
  float prev_position;
  float position;
  float velocity;
} AS5600;

void AS5600_init(AS5600 *sensor, I2C_HandleTypeDef *hi2c, TIM_HandleTypeDef *htim);

void AS5600_update(AS5600 *sensor);

static inline float AS5600_getPosition(AS5600 *sensor) {
  return sensor->position;
}

static inline float AS5600_getVelocity(AS5600 *sensor) {
  return sensor->velocity;
}


#endif /* AS5600_AS5600_H_ */
