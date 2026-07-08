// #include <Arduino.h>
// #include "Wire.h"
// #include <Esp32McpwmMotor.h>
// #include <Esp32PcntEncoder.h>
// #include "pidcontor.h"
// #include "Kinematics.h"

// Kinematics kinema;
// PidContor pid[4];
// Esp32McpwmMotor motor;
// Esp32PcntEncoder encoders[4];


// int64_t last_ticks[4];
// int32_t  delta_ticks[4];
// int64_t last_update_time;
// float speed[4];

// float target_line_speed_x=-100;
// float target_line_speed_y=0.0f;
// float target_angular_speed =0.0f;
// float wheel_speed[4];

// //v=((v1+v3)/2+(v2+v4)/2)/2,
// //w=((v2+v4)/2-(v1+v3)/2)*l,
// // vl=(v1+v3)/2
// // vr=(v2+v4)/2

// void setup()
// {
//     Serial.begin(115200);
//     motor.attachMotor(0,5,4);
//     motor.attachMotor(1,15,16);
//     motor.attachMotor(2,3,8);
//     motor.attachMotor(3,46,9);

//     encoders[0].init(0,6,7);
//     encoders[1].init(1,18,17);
//     encoders[2].init(2,20,19);
//     encoders[3].init(3,11,10);

//     for(int i=0;i<4;i++){
//         pid[i].update_pid(0.625,0.125,0.0);
//         pid[i].limit(100,-100);  
//     }

//     kinema.set_wheel_distance(360);
//     kinema.set_motor_param(0,0.1051566);
//     kinema.set_motor_param(1,0.1051566);
//     kinema.set_motor_param(2,0.1051566);
//     kinema.set_motor_param(3,0.1051566);

//     kinema.kinematics_inverse(target_line_speed_x,target_line_speed_y,target_angular_speed,wheel_speed[0],wheel_speed[1],wheel_speed[2],wheel_speed[3]);


//     for(int i=0;i<4;i++){
//       pid[i].update_target(wheel_speed[i]);
//     }
// }

// void motorSpeedcontor()
// {
   
//     uint64_t dt = millis() - last_update_time;
//     kinema.update_motor_speed(millis(),encoders[0].getTicks(),encoders[1].getTicks(),encoders[2].getTicks(),encoders[3].getTicks());
//     for(int8_t i=0;i<4;i++){
//         delta_ticks[i]= encoders[i].getTicks()-last_ticks[i];
//         speed[i]=float(delta_ticks[i] * 105.1566)/dt;
//         last_ticks[i]=encoders[i].getTicks();
//         motor.updateMotorSpeed(i,pid[i].update(kinema.get_motor_speed(i)));

//     }
//         last_update_time=millis();
//         Serial.printf("speed: 1=%fmm/s 2=%fmm/s 3=%fmm/s 4=%fmm/s\n",speed[0],speed[1],speed[2],speed[3]);
// }

// void loop()
// {

//     delay(10);
//     motorSpeedcontor();

// }