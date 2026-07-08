#ifndef ARM_KINEMATICS_H
#define ARM_KINEMATICS_H

#include <Arduino.h>

#define RAD2DEG 57.29578f
#define DEG2RAD 0.0174533f

#define BASE_H    75.0f
#define L1       125.0f
#define L2       115.0f
#define WRIST_D   55.0f

float J1, J2, J3, J4, J5, J6;
bool has_solution = false;

void IK_6DOF(float X, float Y, float Z)
{
  has_solution = false;

  J1 = atan2f(Y, X) * RAD2DEG;
  float d = sqrtf(X*X + Y*Y);
  float z_arm = Z - BASE_H;

  float wx = d - WRIST_D;
  float wz = z_arm;
  float r = sqrtf(wx*wx + wz*wz);

  float beta = acosf(constrain((r*r + L1*L1 - L2*L2)/(2*L1*r), -1, 1));
  float alpha = atan2f(wz, wx);

  J2 = (alpha - beta) * RAD2DEG;
  J3 = acosf(constrain((L1*L1 + L2*L2 - r*r)/(2*L1*L2), -1, 1)) * RAD2DEG;

  J4 = 0.0f;
  J5 = 90.0f;
  J6 = 90.0f;

  has_solution = true;
}

#endif
