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

typedef struct {
  uint8_t mode;
  float encoder_flux_angle_offset;
  uint8_t n_pole_pairs;
  int8_t motor_direction;
  uint8_t is_closed_loop;

  float position_kp;
  float position_ki;
  float position_ki_threshold;
  float position_kd;
  float position_limit_lower;
  float position_limit_upper;
  float velocity_kp;
  float velocity_ki;
  float velocity_ki_threshold;
  float velocity_limit_upper;
  float velocity_limit_lower;
  float torque_kp;
  float torque_limit_upper;
  float torque_limit_lower;
  float flux_kp;
  float current_limit;
  float current_sample_filter_rate;
} FOC_Config;

typedef struct {
  float position_setpoint;
  float position_measured;
  float position_accumulated;
  float velocity_setpoint;
  float velocity_measured;
  float velocity_accumulated;
  float phase_current_measured[3];
  float torque_setpoint;
  float flux_setpoint;
  float bus_voltage_measured;
  float phase_voltage_setpoint[3];

  float i_alpha;
  float i_beta;
  float i_q;
  float i_d;
  float v_q;
  float v_d;
  float v_alpha;
  float v_beta;
} FOC_Param;

void FOC_init(FOC_Config *config, FOC_Param *param, AS5600 *encoder);

void FOC_runCalibrationSequence();

void FOC_updatePositionVelocityPID();

void FOC_updateTorque();

void FOC_setFluxAngle(float angle_setpoint, float voltage_setpoint);

void FOC_generateInvClarkSVPWM(float v_q, float v_d, float sin_theta, float cos_theta);

void FOC_setBridgeOutput(uint8_t enabled, float a, float b, float c);

#endif /* INC_FOC_H_ */
