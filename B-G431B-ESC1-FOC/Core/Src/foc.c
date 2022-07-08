
#include "foc.h"

extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart2;

extern uint16_t adc1_dma_data[3];
extern uint16_t adc2_dma_data[2];
extern uint16_t adc_opamp_current_offset[3];


FOC_Config *_config;
FOC_Param *_param;

AS5600 *_encoder;

void FOC_init(FOC_Config *config, FOC_Param *param, AS5600* encoder) {
  _config = config;
  _param = param;
  _encoder = encoder;
}

void FOC_runCalibrationSequence() {
  _config->mode = FOC_MODE_CALIBRATION;
  // calibration sequence
  HAL_GPIO_WritePin(GPIO_LED_GPIO_Port, GPIO_LED_Pin, GPIO_PIN_SET);

  // get current offset
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
  HAL_Delay(500);

  adc_opamp_current_offset[0] = adc1_dma_data[0];
  adc_opamp_current_offset[1] = adc2_dma_data[0];
  adc_opamp_current_offset[2] = adc2_dma_data[1];

  {
    char str[128];
    sprintf(str, "phase current offset: %d\t%d\t%d\r\n", adc_opamp_current_offset[0], adc_opamp_current_offset[1], adc_opamp_current_offset[2]);
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 10);
  }

  // open loop calibration
  float flux_angle_setpoint = 0;
  float voltage_setpoint = 1;

  FOC_setFluxAngle(flux_angle_setpoint, voltage_setpoint);
  HAL_Delay(500);

  AS5600_update(_encoder);
  float start_position = AS5600_getPosition(_encoder);

  // move one electrical revolution forward
  for (int16_t i=0; i<=500; i+=1) {
    flux_angle_setpoint = (i / 500.0f) * (2*M_PI);
    FOC_setFluxAngle(flux_angle_setpoint, voltage_setpoint);
    HAL_Delay(2);
  }
  HAL_Delay(500);

  AS5600_update(_encoder);
  float end_position = AS5600_getPosition(_encoder);

  for (int16_t i=500; i>=0; i-=1) {
    flux_angle_setpoint = (i / 500.0f) * (2*M_PI);
    FOC_setFluxAngle(flux_angle_setpoint, voltage_setpoint);
    HAL_Delay(2);
  }

  flux_angle_setpoint = 0;
  FOC_setFluxAngle(flux_angle_setpoint, voltage_setpoint);
  HAL_Delay(500);

  AS5600_update(_encoder);
  start_position = 0.5 * AS5600_getPosition(_encoder) + 0.5 * start_position;
  HAL_Delay(500);

  // release motor
  FOC_setFluxAngle(0, 0);


  float delta_position = end_position - start_position;

  {
    char str[128];
    sprintf(str, "initial encoder angle: %f\r\n", start_position);
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 10);
    sprintf(str, "end encoder angle: %f\r\n", end_position);
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 10);
    sprintf(str, "delta angle: %f\r\n", delta_position);
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 10);
  }


  if (fabsf(delta_position) < 0.1) {
    // motor did not rotate
    HAL_UART_Transmit(&huart2, (uint8_t *)"ERROR: motor not rotating\r\n", strlen("ERROR: motor not rotating\r\n"), 10);
  }

  if (fabsf(fabsf(delta_position)*_config->n_pole_pairs-(2*M_PI)) > 0.5f) {
    HAL_UART_Transmit(&huart2, (uint8_t *)"ERROR: motor pole pair mismatch\r\n", strlen("ERROR: motor pole pair mismatch\r\n"), 10);
  }


  // set electrical angle
  _config->encoder_flux_angle_offset = wrapTo2Pi(start_position * _config->n_pole_pairs);

  {
    char str[128];
    sprintf(str, "offset angle: %f\r\n", _config->encoder_flux_angle_offset);
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 10);
  }

  HAL_Delay(1000);

  _config->mode = FOC_MODE_IDLE;
  HAL_GPIO_WritePin(GPIO_LED_GPIO_Port, GPIO_LED_Pin, GPIO_PIN_RESET);
}

void FOC_updatePositionVelocityPID() {
  // statically update reading, assuming that the FOC torque loop has fetched from encoder
  _param->position_measured = AS5600_getPosition(_encoder);
  _param->velocity_measured = AS5600_getVelocity(_encoder);

  if (_config->mode == FOC_MODE_POSITION) {
    if (_config->position_limit_lower != 0 && _config->position_limit_upper != 0) {
      _param->position_setpoint = clampf(_param->position_setpoint, _config->position_limit_lower, _config->position_limit_upper);
    }

    float position_error = _param->position_setpoint - _param->position_measured;
    _param->position_accumulated += position_error;
    _param->position_accumulated = clampf(_param->position_accumulated, -_config->position_ki_threshold, _config->position_ki_threshold);  // integral anti-windup


    _param->torque_setpoint = (_config->position_kp * position_error)
        + (-_config->position_kd * _param->velocity_measured)
        + (_config->position_ki * _param->position_accumulated);
  }
  else if (_config->mode == FOC_MODE_VELOCITY) {
    if (_config->velocity_limit_lower != 0 && _config->velocity_limit_upper != 0) {
      _param->velocity_setpoint = clampf(_param->velocity_setpoint, _config->velocity_limit_lower, _config->velocity_limit_upper);
    }

    float velocity_error = _param->velocity_setpoint - _param->velocity_measured;
    _param->velocity_accumulated += velocity_error;
    _param->velocity_accumulated = clampf(_param->velocity_accumulated, -_config->velocity_ki_threshold, _config->velocity_ki_threshold);  // integral anti-windup

    _param->torque_setpoint = (_config->velocity_kp * velocity_error)
        + (_config->velocity_ki * _param->velocity_accumulated);
  }
  // because we are using BLDC motor, desired flux will always be 0 for maximum performance
  // so no need to do expensive computations
  _param->flux_setpoint = 0;
  //flux_setpoint = clampf(flux_setpoint, -_config->current_limit, _config->current_limit);

}

void FOC_updateTorque() {
  if (_config->mode == FOC_MODE_IDLE) {
    FOC_setBridgeOutput(0, 0, 0, 0);
    return;
  }
  if (_config->mode == FOC_MODE_CALIBRATION) {
    return;
  }

  AS5600_update(_encoder);
  _param->position_measured = AS5600_getPosition(_encoder);

  _param->torque_setpoint = clampf(_param->torque_setpoint, _config->torque_limit_lower, _config->torque_limit_upper);

  float i_a = _param->phase_current_measured[0];
  float i_b = _param->phase_current_measured[1];
  float i_c = _param->phase_current_measured[2];

  _param->i_alpha = i_a + (cosf((2./3.) * M_PI) * i_b) + (cosf((2./3.) * M_PI) * i_c);
  _param->i_beta  = (sinf((2./3.) * M_PI) * i_b) - (sinf((2./3.) * M_PI) * i_c);

  // match the correct current magnitude after transform
  _param->i_alpha *= 2 / 3.;
  _param->i_beta *= 2 / 3.;

  // convert mechanical encoder revolutions into electrical revolutions
  float theta = wrapTo2Pi((_param->position_measured * (float)_config->n_pole_pairs) - _config->encoder_flux_angle_offset);

  // TODO: replace with CORDIC
  float sin_theta = sinf(theta);
  float cos_theta = cosf(theta);

  float i_q_raw= -(_param->i_alpha * sin_theta) + (_param->i_beta * cos_theta);
  float i_d_raw =  (_param->i_alpha * cos_theta) + (_param->i_beta * sin_theta);

  _param->i_q = _config->current_sample_filter_rate * i_q_raw + (1 - _config->current_sample_filter_rate) * _param->i_q;
  _param->i_d = _config->current_sample_filter_rate * i_d_raw + (1 - _config->current_sample_filter_rate) * _param->i_d;

  float i_q_error = _param->torque_setpoint - (_config->is_closed_loop ? _param->i_q : 0);
  float i_d_error = _param->flux_setpoint   - (_config->is_closed_loop ? _param->i_d : 0);

  i_q_error = clampf(i_q_error, -_config->current_limit, _config->current_limit);
  i_d_error = clampf(i_d_error, -_config->current_limit, _config->current_limit);

  _param->v_q = i_q_error * _config->torque_kp;  // kp = 0.02
  _param->v_d = i_d_error * _config->flux_kp;

  // clamp voltage
  if (_param->bus_voltage_measured > 0) {
    float v_max_sq = _param->bus_voltage_measured * _param->bus_voltage_measured * 1.15; // CSVPWM over modulation
    float v_norm = _param->v_q * _param->v_q + _param->v_d * _param->v_d;
    if (v_norm > v_max_sq) {
      float k = sqrtf(fabsf(v_norm / v_max_sq));
      _param->v_q *= k;
      _param->v_d *= k;
    }
  }

  FOC_generateInvClarkSVPWM(_param->v_q, _param->v_d, sin_theta, cos_theta);
}

/**
 * set flux angle within one electrical revolution.
 *
 * @param angle_setpoint: the electrical revolution angle, in radian.
 */
void FOC_setFluxAngle(float angle_setpoint, float voltage_setpoint) {
  float theta = wrapTo2Pi(angle_setpoint);
  float sin_theta = sinf(theta);
  float cos_theta = cosf(theta);
  float v_q = 0.0f;
  float v_d = clampf(voltage_setpoint, -2, 14);

  FOC_generateInvClarkSVPWM(v_q, v_d, sin_theta, cos_theta);
}

void FOC_generateInvClarkSVPWM(float v_q, float v_d, float sin_theta, float cos_theta) {
  _param->v_alpha = -v_q * sin_theta + v_d * cos_theta;
  _param->v_beta  =  v_q * cos_theta + v_d * sin_theta;

  float v_a = _param->v_alpha;
  float v_b = (-.5 * _param->v_alpha) + ((sqrtf(3.)/2.) * _param->v_beta);
  float v_c = (-.5 * _param->v_alpha) - ((sqrtf(3.)/2.) * _param->v_beta);

  float v_neutral = .5 * (fmaxf(fmaxf(v_a, v_b), v_c) + fminf(fminf(v_a, v_b), v_c));

  _param->phase_voltage_setpoint[0] = v_a - v_neutral;
  _param->phase_voltage_setpoint[1] = v_b - v_neutral;
  _param->phase_voltage_setpoint[2] = v_c - v_neutral;

  float pwm_duty_cycle_a = .5f * ((_param->phase_voltage_setpoint[0] / _param->bus_voltage_measured) + 1.f);
  float pwm_duty_cycle_b = .5f * ((_param->phase_voltage_setpoint[1] / _param->bus_voltage_measured) + 1.f);
  float pwm_duty_cycle_c = .5f * ((_param->phase_voltage_setpoint[2] / _param->bus_voltage_measured) + 1.f);

  FOC_setBridgeOutput(1, pwm_duty_cycle_a, pwm_duty_cycle_b, pwm_duty_cycle_c);
}

void FOC_setBridgeOutput(uint8_t enabled, float a, float b, float c) {
  if (!enabled) {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    return;
  }

  a = clampf(a, 0.0f, 0.98f);  // prevent hi-side switching bootstrap circuit loses voltage
  b = clampf(b, 0.0f, 0.98f);
  c = clampf(c, 0.0f, 0.98f);

  uint16_t ccr_a = (uint16_t)((float)(__HAL_TIM_GET_AUTORELOAD(&htim1)+1) * a);
  uint16_t ccr_b = (uint16_t)((float)(__HAL_TIM_GET_AUTORELOAD(&htim1)+1) * b);
  uint16_t ccr_c = (uint16_t)((float)(__HAL_TIM_GET_AUTORELOAD(&htim1)+1) * c);

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr_a);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr_b);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccr_c);
}
