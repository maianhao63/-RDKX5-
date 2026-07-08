#include "Kinematics.h"
void Kinematics::set_motor_param(uint8_t id, float per_pulse_distance)

{

motor_param_[id].per_pulse_distance = per_pulse_distance;

}
void Kinematics::set_wheel_base(float wheel_base_mm)

{

wheel_base_ = wheel_base_mm;

half_wheel_base_ = wheel_base_mm / 2.0f;

}
int16_t Kinematics::get_motor_speed(uint8_t id)

{

return motor_param_[id].motor_speed;

}
void Kinematics::update_motor_speed(uint64_t current_time, int32_t m1, int32_t m2, int32_t m3, int32_t m4)

{

uint32_t dt = current_time - last_update_time;

if (dt == 0) return;

last_update_time = current_time;
int32_t d1 = m1 - motor_param_[0].last_encoder_tick;

int32_t d2 = m2 - motor_param_[1].last_encoder_tick;

int32_t d3 = m3 - motor_param_[2].last_encoder_tick;

int32_t d4 = m4 - motor_param_[3].last_encoder_tick;
motor_param_[0].last_encoder_tick = m1;

motor_param_[1].last_encoder_tick = m2;

motor_param_[2].last_encoder_tick = m3;

motor_param_[3].last_encoder_tick = m4;
motor_param_[0].motor_speed = (float)d1 * motor_param_[0].per_pulse_distance * 1000.0f / dt;

motor_param_[1].motor_speed = (float)d2 * motor_param_[1].per_pulse_distance * 1000.0f / dt;

motor_param_[2].motor_speed = (float)d3 * motor_param_[2].per_pulse_distance * 1000.0f / dt;

motor_param_[3].motor_speed = (float)d4 * motor_param_[3].per_pulse_distance * 1000.0f / dt;
update_odom(dt);

}
void Kinematics::kinematic_forward(float w1, float w2, float w3, float w4,

float &vx, float &vy, float &vth)

{

float left  = (w1 + w3) * 0.5f;

float right = (w2 + w4) * 0.5f;

vx  = (left + right) * 0.5f;

vy  = 0.0f;

vth = (right - left) / wheel_base_;

}
void Kinematics::kinematic_inverse(float vx, float vy, float vth,

float &o1, float &o2, float &o3, float &o4)

{

float delta = vth * half_wheel_base_;

o1 = vx - delta;

o3 = vx - delta;

o2 = vx + delta;

o4 = vx + delta;

}
odom_t &Kinematics::get_odom()

{

return odom_;

}
void Kinematics::TransAngleInPI(float angle, float &out)

{

out = angle;

while (out > PI)  out -= 2 * PI;

while (out < -PI) out += 2 * PI;

}
void Kinematics::update_odom(uint16_t dt)

{

float dt_s = (float)dt / 1000.0f;

float vx, vy, vth;

kinematic_forward(

motor_param_[0].motor_speed,

motor_param_[1].motor_speed,

motor_param_[2].motor_speed,

motor_param_[3].motor_speed,

vx, vy, vth

);
odom_.linear_x_speed = vx / 1000.0f;

odom_.angular_speed  = vth;
float dx = odom_.linear_x_speed * cos(odom_.yaw) * dt_s;

float dy = odom_.linear_x_speed * sin(odom_.yaw) * dt_s;

float dyaw = odom_.angular_speed * dt_s;
odom_.x += dx;

odom_.y += dy;

odom_.yaw += dyaw;

TransAngleInPI(odom_.yaw, odom_.yaw);

}