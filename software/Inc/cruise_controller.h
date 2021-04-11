#pragma once

#include <cstdint>

#define CC_MAX_SPEED 970

class CruiseController
{
public:
  int16_t step(int16_t speed)
  {
    int16_t error = speed - target;

    int16_t p = error;

    if ((error <= 0 || i <= INT16_MAX - error) &&
        (error >= 0 || i >= INT16_MIN - error))
      i += error;

    int16_t d = previous_error - error;
    previous_error = error;

    return p_coef * p + i_coef * i + d_coef * d;
  }

  void start(int16_t target, int16_t start_output)
  {
    reset();
    this->target = target;
    this->i = start_output / i_coef;
  }

  void reset()
  {
    this->previous_error = 0;
    this->i = 0;
  }

private:
  static constexpr const float p_coef = -5.f;
  static constexpr const float i_coef = -0.025f;
  static constexpr const float d_coef = -16.0f;

  int16_t target = 0;

  int16_t previous_error = 0;
  int16_t i = 0;
};
