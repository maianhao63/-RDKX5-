#ifndef KINEMATICS_H

#define KINEMATICS_H
#include <Arduino.h>

#include <math.h>
typedef struct {

float per_pulse_distance;

int16_t motor_speed;

int64_t last_encoder_tick;

} motor_param_t;
typedef struct {

float x;

float y;

float yaw;

float linear_x_speed;

float linear_y_speed;

float angular_speed;

} odom_t;
class Kinematics

{

public:

Kinematics() = default;

~Kinematics() = default;
void set_motor_param(uint8_t id, float per_pulse_distance);

void set_wheel_base(float wheel_base_mm);

void update_odom(uint16_t dt);

odom_t &get_odom();

static void TransAngleInPI(float angle, float &out_angle);
void kinematic_inverse(float linear_x_speed, float linear_y_speed,

float angular_speed, float &out_wheel1_speed,

float &out_wheel2_speed, float &out_wheel3_speed,

float &out_wheel4_speed);

void kinematic_forward(float wheel1_speed, float wheel2_speed,

float wheel3_speed, float wheel4_speed,

float &linear_x_speed, float &linear_y_speed,

float &angular_speed);

void update_motor_speed(uint64_t current_time, int32_t m1, int32_t m2, int32_t m3, int32_t m4);

int16_t get_motor_speed(uint8_t id);
private:

float wheel_base_ = 340.0f;

float half_wheel_base_ = 170.0f;

motor_param_t motor_param_[4];

uint64_t last_update_time = 0;

odom_t odom_;

};
#endif