
#include "foc.h"

extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart2;

extern AS5600 encoder;

extern float phase_voltage_setpoint[3];
extern float bus_voltage_measured;
extern uint8_t foc_mode;
extern float encoder_flux_angle_offset;
extern uint8_t n_pole_pairs;
extern float adc1_dma_data[5];
extern float adc2_dma_data[5];
extern float adc_opamp_current_offset[3];

extern float position_setpoint;
extern float position_measured;
extern float torque_setpoint;
extern float flux_setpoint;
extern float phase_current_measured[3];

extern float i_q_filtered;
extern float i_d_filtered;
extern float v_q;
extern float v_d;

void FOC_runCalibrationSequence() {
  uint8_t prev_foc_mode = foc_mode;
  foc_mode = FOC_MODE_CALIBRATION;
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

  float start_position = AS5600_getPosition(&encoder);

  // move one electrical revolution forward
  for (int16_t i=0; i<=500; i+=1) {
    flux_angle_setpoint = (i / 500.0f) * (2*M_PI);
    FOC_setFluxAngle(flux_angle_setpoint, voltage_setpoint);
    HAL_Delay(2);
  }
  HAL_Delay(500);

  float end_position = AS5600_getPosition(&encoder);

  for (int16_t i=500; i>=0; i-=1) {
    flux_angle_setpoint = (i / 500.0f) * (2*M_PI);
    FOC_setFluxAngle(flux_angle_setpoint, voltage_setpoint);
    HAL_Delay(2);
  }

  flux_angle_setpoint = 0;
  FOC_setFluxAngle(flux_angle_setpoint, voltage_setpoint);
  HAL_Delay(500);

  start_position = 0.5 * AS5600_getPosition(&encoder) + 0.5 * start_position;
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

  if (fabsf(fabsf(delta_position)*n_pole_pairs-(2*M_PI)) > 0.5f) {
    HAL_UART_Transmit(&huart2, (uint8_t *)"ERROR: motor pole pair mismatch\r\n", strlen("ERROR: motor pole pair mismatch\r\n"), 10);
  }


  // set electrical angle
  encoder_flux_angle_offset = wrapTo2Pi(start_position * n_pole_pairs);

  {
    char str[128];
    sprintf(str, "offset angle: %f\r\n", encoder_flux_angle_offset);
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 10);
  }

  HAL_Delay(1000);

  foc_mode = prev_foc_mode;
  HAL_GPIO_WritePin(GPIO_LED_GPIO_Port, GPIO_LED_Pin, GPIO_PIN_RESET);
}



void FOC_updatePositionPID(float target_position) {
  const float alpha = 0.2;
  position_setpoint = (alpha * target_position) + (1-alpha) * position_setpoint;

  float position_error = position_setpoint - position_measured;

  float position_kp = 0.2; //30;

  torque_setpoint = position_kp * position_error;

  flux_setpoint = 0;

  float maximum_current = 1;
  torque_setpoint = clampf(torque_setpoint, -maximum_current, maximum_current);
  flux_setpoint = clampf(flux_setpoint, -maximum_current, maximum_current);
}


void FOC_updateTorque() {
  if (foc_mode == FOC_MODE_IDLE || foc_mode == FOC_MODE_CALIBRATION) {
    return;
  }
  position_measured = AS5600_getPosition(&encoder);

  float i_a = phase_current_measured[0];
  float i_b = phase_current_measured[1];
  float i_c = phase_current_measured[2];

  float i_alpha = i_a + (cosf((2./3.) * M_PI) * i_b) + (cosf((2./3.) * M_PI) * i_c);
  float i_beta = sinf((2./3.) * M_PI) * i_b - sinf((2./3.) * M_PI) * i_c;

  i_alpha *= 2 / 3.;
  i_beta *= 2 / 3.;


  float theta = wrapTo2Pi((position_measured * (float)n_pole_pairs) - encoder_flux_angle_offset);

  float i_q = -i_alpha * sinf(theta) + i_beta * cosf(theta);
  float i_d = i_alpha * cosf(theta) + i_beta * sinf(theta);

  const float ALPHA = 0.1;

  i_q_filtered = ALPHA * i_q + (1-ALPHA) * i_q_filtered;
  i_d_filtered = ALPHA * i_d + (1-ALPHA) * i_d_filtered;


  uint8_t closed_loop = 1;

//    float i_q_setpoint = 0.4;
//    float i_d_setpoint = 0;
  float i_q_setpoint = torque_setpoint;
  float i_d_setpoint = flux_setpoint;

  float i_q_error = i_q_setpoint - (closed_loop ? i_q_filtered : 0);
  float i_d_error = i_d_setpoint - (closed_loop ? i_d_filtered : 0);

  float v_q_kp = 5;
  float v_d_kp = 5;

  v_q = i_q_error * v_q_kp;  // kp = 0.02
  v_d = i_d_error * v_d_kp;

  // clamp voltage
  if (bus_voltage_measured > 0) {
    float v_max_sq = bus_voltage_measured * bus_voltage_measured * 1.15; // CSVPWM over modulation
    float v_norm = v_q * v_q + v_d * v_d;
    if (v_norm > v_max_sq) {
      float k = sqrtf(fabsf(v_norm / v_max_sq));
      v_q *= k;
      v_d *= k;
    }
  }

//    if (fabsf(v_q) < 0.1) v_q = 0;
//    if (fabsf(v_d) < 0.1) v_d = 0;

  FOC_generateInvClarkSVPWM(v_q, v_d, theta);

}

/**
 * set flux angle within one electrical revolution.
 *
 * @param angle_setpoint: the electrical revolution angle, in radian.
 */
void FOC_setFluxAngle(float angle_setpoint, float voltage_setpoint) {
  float theta = wrapTo2Pi(angle_setpoint);
  float v_q = 0.0f;
  float v_d = clampf(voltage_setpoint, -2, 14);

  FOC_generateInvClarkSVPWM(v_q, v_d, theta);
}

void FOC_generateInvClarkSVPWM(float v_q, float v_d, float theta) {
  float v_alpha = -v_q * sinf(theta) + v_d * cosf(theta);
  float v_beta = v_q * cosf(theta) + v_d * sinf(theta);
  float v_a = v_alpha;
  float v_b = (-0.5 * v_alpha) + ((sqrtf(3.0f)/2.) * v_beta);
  float v_c = (-0.5 * v_alpha) - ((sqrtf(3.0f)/2.) * v_beta);

  float v_neutral = .5f * (fmaxf(fmaxf(v_a, v_b), v_c) + fminf(fminf(v_a, v_b), v_c));

  phase_voltage_setpoint[0] = v_a - v_neutral;
  phase_voltage_setpoint[1] = v_b - v_neutral;
  phase_voltage_setpoint[2] = v_c - v_neutral;

  float pwm_duty_cycle_a = .5f * ((phase_voltage_setpoint[0] / bus_voltage_measured) + 1.f);
  float pwm_duty_cycle_b = .5f * ((phase_voltage_setpoint[1] / bus_voltage_measured) + 1.f);
  float pwm_duty_cycle_c = .5f * ((phase_voltage_setpoint[2] / bus_voltage_measured) + 1.f);

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
