#ifndef _PID_CONTOR_H_
#define _PID_CONTOR_H_
class Pidcontor

{

public:

Pidcontor() = default;

Pidcontor(float kp, float ki, float kd);
float update(float current);

void update_target(float target);

void update_pid(float kp, float ki, float kd);

void reset();

void out_limit(float out_min, float out_max);
private:

float target_ = 0.0f;

float out_min_ = -100.0f;

float out_max_ = 100.0f;

float kp_ = 0.0f;

float ki_ = 0.0f;

float kd_ = 0.0f;

float error_sum_ = 0.0f;

float derror_ = 0.0f;

float error_last_ = 0.0f;

// 积分限幅缩小，防止转弯积分饱和超转

float intergral_up_ = 800.0f;

};
#endif