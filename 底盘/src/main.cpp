#include <Arduino.h>
#include <Esp32PcntEncoder.h>
#include "pidcontor.h"
#include "Kinematics.h"
#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <micro_ros_utilities/string_utilities.h>
#include <cstring>

// ================= TB6612 电机引脚 =================
#define M0_AIN1  5
#define M0_AIN2  4
#define M0_PWM   7

#define M1_BIN1  15
#define M1_BIN2  16
#define M1_PWM   8

#define M2_AIN1  10
#define M2_AIN2  11
#define M2_PWM   9

#define M3_BIN1  3
#define M3_BIN2  6
#define M3_PWM   47

// 独立LEDC硬件通道
#define CH_M0 0
#define CH_M1 1
#define CH_M2 2
#define CH_M3 3

#define PWM_FREQ 10000
#define PWM_RES  8
#define PWM_MAX  255

// 底盘速度参数
float target_linear_x_speed = 0.0f;
float target_angular_speed  = 0.0f;
unsigned long last_cmd_time = 0;
const unsigned long CMD_TIMEOUT = 300;

// 硬件实例
Kinematics kinematics;
Esp32PcntEncoder encoders[4];
Pidcontor pid_controller[4];



int64_t last_ticks[4] = {0};
uint64_t last_update_time = 0;
float out_speed[4]      = {0};

// microROS对象
rcl_subscription_t cmd_vel_subscriber;
geometry_msgs__msg__Twist cmd_vel_msg;
rcl_publisher_t odom_publisher;
nav_msgs__msg__Odometry odom_msg;
rcl_timer_t timer;

// TB6612底层驱动
void setMotor(int motor_id, float pwm_out)
{
    int pwm = constrain((int)pwm_out, -80, 80);
    uint8_t duty = map(abs(pwm), 0, 80, 30, 255);
    // 改回来：pwm正数=前进，负数=后退
    bool forward = pwm > 0;
    switch(motor_id)
    {
        case 0:
            digitalWrite(M0_AIN1, forward ? HIGH : LOW);
            digitalWrite(M0_AIN2, forward ? LOW : HIGH);
            ledcWrite(CH_M0, duty);
            break;
        case 1:
            digitalWrite(M1_BIN1, forward ? HIGH : LOW);
            digitalWrite(M1_BIN2, forward ? LOW : HIGH);
            ledcWrite(CH_M1, duty);
            break;
        case 2:
            digitalWrite(M2_AIN1, forward ? HIGH : LOW);
            digitalWrite(M2_AIN2, forward ? LOW : HIGH);
            ledcWrite(CH_M2, duty);
            break;
        case 3:
            digitalWrite(M3_BIN1, forward ? HIGH : LOW);
            digitalWrite(M3_BIN2, forward ? LOW : HIGH);
            ledcWrite(CH_M3, duty);
            break;
    }
}



// cmd_vel订阅回调 【已删除负号，解决所有按键只前进】
void cmd_vel_callback(const void *msg_recv)
{
    const geometry_msgs__msg__Twist *twist = (const geometry_msgs__msg__Twist *)msg_recv;
    target_linear_x_speed = twist->linear.x * 1000.0f;
    target_angular_speed  = twist->angular.z;
    last_cmd_time = millis();
    Serial.printf("收到速度 X:%.2f 旋转:%.2f\n", target_linear_x_speed, target_angular_speed);
}

// 里程计定时发布
void callback_publisher(rcl_timer_t *timer, int64_t last_call_time)
{
    odom_t odom = kinematics.get_odom();
    int64_t now_ns = rmw_uros_epoch_nanos();
    odom_msg.header.stamp.sec      = static_cast<int32_t>(now_ns / 1000000000LL);
    odom_msg.header.stamp.nanosec  = static_cast<uint32_t>(now_ns % 1000000000LL);

    odom_msg.pose.pose.position.x = -odom.x;
    odom_msg.pose.pose.position.y = -odom.y;
    odom_msg.pose.pose.position.z = 0.0f;
    odom_msg.pose.pose.orientation.x = 0.0f;
    odom_msg.pose.pose.orientation.y = 0.0f;
    odom_msg.pose.pose.orientation.z = sin(odom.yaw * 0.5f);
    odom_msg.pose.pose.orientation.w = cos(odom.yaw * 0.5f);

    // 修复字段名 linear_x_speed
    odom_msg.twist.twist.linear.x  = -odom.linear_x_speed;
    odom_msg.twist.twist.angular.z = odom.angular_speed;

    const float pose_cov[36] = {
        0.01, 0,    0,    0,    0,    0,
        0,    0.01, 0,    0,    0,    0,
        0,    0,    0.02, 0,    0,    0,
        0,    0,    0,    0.01, 0,    0,
        0,    0,    0,    0,    0.01, 0,
        0,    0,    0,    0,    0,    0.05
    };
    const float twist_cov[36] = {
        0.01, 0,    0,    0,    0,    0,
        0,    0.01, 0,    0,    0,    0,
        0,    0,    0.02, 0,    0,    0,
        0,    0,    0,    0.01, 0,    0,
        0,    0,    0,    0,    0.01, 0,
        0,    0,    0,    0,    0,    0.15
    };
    memcpy(odom_msg.pose.covariance, pose_cov, sizeof(pose_cov));
    memcpy(odom_msg.twist.covariance, twist_cov, sizeof(twist_cov));
    (void)rcl_publish(&odom_publisher, &odom_msg, NULL);
}

// microROS后台线程
void micro_ros_task (void *parameter)
{
    IPAddress agent_ip;
    agent_ip.fromString("10.73.251.21");
    char ssid[] = "red";
    char pwd[]  = "20040910";
    set_microros_wifi_transports(ssid, pwd, agent_ip, 8888);
    delay(1000);

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rclc_support_init(&support, 0, NULL, &allocator);
    rcl_node_t node;
    rclc_node_init_default(&node, "fishbot_motion_control", "", &support);

    rclc_subscription_init_default(
        &cmd_vel_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel");

    nav_msgs__msg__Odometry__init(&odom_msg);
    odom_msg.header.frame_id = micro_ros_string_utilities_set(odom_msg.header.frame_id, "odom");
    odom_msg.child_frame_id  = micro_ros_string_utilities_set(odom_msg.child_frame_id, "base_footprint");

    rclc_publisher_init_best_effort(
        &odom_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "/odom");

    while (!rmw_uros_epoch_synchronized()) {
        rmw_uros_sync_session(100);
        delay(5);
    }
    rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(20), callback_publisher);

    rclc_executor_t executor;
    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg, cmd_vel_callback, ON_NEW_DATA);
    rclc_executor_add_timer(&executor, &timer);
    rclc_executor_spin(&executor);
}

// 闭环控制：300ms无指令强制四轮刹车
void motorSpeedControl()
{
    if (millis() - last_cmd_time > CMD_TIMEOUT)
    {
        target_linear_x_speed = 0.0f;
        target_angular_speed  = 0.0f;
        setMotor(0, 0);
        setMotor(1, 0);
        setMotor(2, 0);
        setMotor(3, 0);
        return;
    }

    uint64_t dt = millis() - last_update_time;
    if (dt == 0) return;

    // ==========四轮差速手写逆解，替换麦克纳姆库函数==========
    float L = 340.0f; // 匹配你底盘实际轮距mm
    float v_left  = target_linear_x_speed - target_angular_speed * L / 2.0f;
    float v_right = target_linear_x_speed + target_angular_speed * L / 2.0f;
    out_speed[0] = v_left;
    out_speed[3] = v_left;
    out_speed[1] = v_right;
    out_speed[2] = v_right;
    // ======================================================

    const float LEFT_FACTOR  = 1.00;
    const float RIGHT_FACTOR = 1.00;
    out_speed[0] *= LEFT_FACTOR;
    out_speed[3] *= LEFT_FACTOR;
    out_speed[1] *= RIGHT_FACTOR;
    out_speed[2] *= RIGHT_FACTOR;

    for (int i = 0; i < 4; i++)
    {
        int64_t ticks = encoders[i].getTicks();
        int32_t delta = ticks - last_ticks[i];
        last_ticks[i] = ticks;
        float current_speed = (float)delta * 0.1051566f / dt * 1000.0f;
        pid_controller[i].update_target(out_speed[i]);
        float pwm = pid_controller[i].update(current_speed);
        setMotor(i, pwm);
    }

    kinematics.update_motor_speed(millis(),
        encoders[0].getTicks(), encoders[1].getTicks(),
        encoders[2].getTicks(), encoders[3].getTicks());
    last_update_time = millis();
}

void setup()
{
    Serial.begin(115200);
    delay(500);

    // 方向IO上电双低刹车
    pinMode(M0_AIN1, OUTPUT);
    pinMode(M0_AIN2, OUTPUT);
    pinMode(M1_BIN1, OUTPUT);
    pinMode(M1_BIN2, OUTPUT);
    pinMode(M2_AIN1, OUTPUT);
    pinMode(M2_AIN2, OUTPUT);
    pinMode(M3_BIN1, OUTPUT);
    pinMode(M3_BIN2, OUTPUT);

    digitalWrite(M0_AIN1, LOW);
    digitalWrite(M0_AIN2, LOW);
    digitalWrite(M1_BIN1, LOW);
    digitalWrite(M1_BIN2, LOW);
    digitalWrite(M2_AIN1, LOW);
    digitalWrite(M2_AIN2, LOW);
    digitalWrite(M3_BIN1, LOW);
    digitalWrite(M3_BIN2, LOW);

    // 四路PWM完整三参数初始化
    ledcSetup(CH_M0, PWM_FREQ, PWM_RES);
    ledcAttachPin(M0_PWM, CH_M0);
    ledcSetup(CH_M1, PWM_FREQ, PWM_RES);
    ledcAttachPin(M1_PWM, CH_M1);
    ledcSetup(CH_M2, PWM_FREQ, PWM_RES);
    ledcAttachPin(M2_PWM, CH_M2);
    ledcSetup(CH_M3, PWM_FREQ, PWM_RES);
    ledcAttachPin(M3_PWM, CH_M3);

    // 四路编码器
    encoders[0].init(0, 17, 18);
    encoders[1].init(1, 20, 48);
    encoders[2].init(2, 21, 35);
    encoders[3].init(3, 36, 45);

    // 修复字段 linear_x_speed
    odom_t &odom = kinematics.get_odom();
    odom.x = 0.0f; odom.y = 0.0f; odom.yaw = 0.0f;
    odom.linear_x_speed = 0.0f; odom.angular_speed = 0.0f;

    // PID上电重置抑制漂移
   for (int i = 0; i < 4; i++)
{
    pid_controller[i].reset();
    pid_controller[i].update_pid(0.6f, 0.05f, 0.08f);
    // 函数名out_limit(最小,最大)
    pid_controller[i].out_limit(-80, 80);
    last_ticks[i] = encoders[i].getTicks();
}

    // 修复函数 set_wheel_base
    kinematics.set_wheel_base(340.0f);
    float pulse_dist = 0.1051566f;
    for (int i = 0; i < 4; i++)
    {
        kinematics.set_motor_param(i, pulse_dist);
    }

    last_update_time = millis();
    last_cmd_time    = millis();

    xTaskCreate(micro_ros_task, "micro_ros", 10240, NULL, 1, NULL);
}

void loop()
{
    motorSpeedControl();
    delay(10);
}
