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
  int32_t n_rotations;
  float position;
  float position_prev;
} AS5600;

void AS5600_init(AS5600 *sensor, I2C_HandleTypeDef *hi2c);

float AS5600_getPosition(AS5600 *sensor);

#endif /* AS5600_AS5600_H_ */
