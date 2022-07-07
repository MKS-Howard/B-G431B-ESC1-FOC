/*
 * foc.h
 *
 *  Created on: Jul 6, 2022
 *      Author: TK
 */

#ifndef INC_FOC_H_
#define INC_FOC_H_

#include <stdint.h>
#include <math.h>
#include "stm32g4xx_hal.h"

#include "FEB_math/FEB_math.h"
#include "AS5600/as5600.h"


#define GPIO_LED_Pin GPIO_PIN_6
#define GPIO_LED_GPIO_Port GPIOC
#define GPIO_BUTTON_Pin GPIO_PIN_10
#define GPIO_BUTTON_GPIO_Port GPIOC

#define FOC_MODE_IDLE             0x00U
#define FOC_MODE_CALIBRATION      0x01U
#define FOC_MODE_TORQUE           0x02U
#define FOC_MODE_VELOCITY         0x03U
#define FOC_MODE_POSITION         0x04U


void FOC_runCalibrationSequence();

void FOC_updatePositionPID(float target_position);

void FOC_updateTorque();

void FOC_setFluxAngle(float angle_setpoint, float voltage_setpoint);

void FOC_generateInvClarkSVPWM(float v_q, float v_d, float theta);

void FOC_setBridgeOutput(uint8_t enabled, float a, float b, float c);

#endif /* INC_FOC_H_ */
