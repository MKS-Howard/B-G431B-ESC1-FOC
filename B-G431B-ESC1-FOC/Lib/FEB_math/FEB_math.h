/*
 * FEB_math.h
 *
 *  Created on: Jul 6, 2022
 *      Author: TK
 */

#ifndef FEB_MATH_FEB_MATH_H_
#define FEB_MATH_FEB_MATH_H_

static inline float clampf(float value, float min, float max) {
  return (value > max) ? max : ((value < min) ? min : value);
}

static inline float wrapTo2Pi(float value) {
  value = fmodf(value, 2*M_PI);
  return value >= 0.0f ? value: (value + 2*M_PI);
}

#endif /* FEB_MATH_FEB_MATH_H_ */
