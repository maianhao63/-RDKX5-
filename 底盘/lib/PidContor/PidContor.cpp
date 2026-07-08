#include "pidcontor.h"

#include <Arduino.h>
Pidcontor::Pidcontor(float kp, float ki, float kd)

{

reset();

update_pid(kp, ki, kd);

}
float Pidcontor::update(float current)

{

float error = target_ - current;

derror_ = error - error_last_;

error_last_ = error;
error_sum_ += error;

if (error_sum_ > intergral_up_)   error_sum_ = intergral_up_;

if (error_sum_ < -intergral_up_)  error_sum_ = -intergral_up_;

float output = kp_ * error + ki_ * error_sum_ + kd_ * derror_;

if (output > out_max_)   output = out_max_;

if (output < out_min_)   output = out_min_;

return output;
}
void Pidcontor::update_target(float target)

{

target_ = target;

}
void Pidcontor::update_pid(float kp, float ki, float kd)

{

reset();

kp_ = kp;

ki_ = ki;

kd_ = kd;

}
void Pidcontor::reset()

{

target_ = 0.0f;

out_min_ = -100.0f;

out_max_ = 100.0f;

kp_ = ki_ = kd_ = 0.0f;

error_sum_ = 0.0f;

derror_ = 0.0f;

error_last_ = 0.0f;

}
void Pidcontor::out_limit(float out_min, float out_max)

{

out_min_ = out_min;

out_max_ = out_max;

}