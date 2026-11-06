#ifndef BASE_H
#define BASE_H
#include <iostream>
#include <string>
#include <chrono>
#include <cstring>
#include <memory> 
#pragma once 

#define PI 3.1415926
// 弹速
#define bullet_speed 22
#define _std_radian(angle) ((angle) + round((0 - (angle)) / (2 * PI)) * (2 * PI))
using time_point = std::chrono::steady_clock::time_point;

struct Data_Save {
    time_point timestamp;
    double camera_yaw_angle;
    double camera_pitch_angle;  
    double camera_roll_angle;
    int armor_or_buff = 1;       // 自瞄或大符
    int color;               // 敌方颜色
};

struct Data_Get
{
    int size;
    u_char raw_data[21];
    Data_Get() {
        memset(raw_data, 0, sizeof(raw_data));
        raw_data[0] = 0xAA;
        raw_data[1] = 0xBB;
        raw_data[20] = 0xBB;
        size = 21;
    }
    void Vision2Control(int32_t x=0, int32_t y=0, int32_t ui_x=0, int32_t ui_y=0, uint8_t found=0, uint8_t fire=0);
};

struct Gimbal_Angle
{
    double target_yaw;        // 目标yaw 
    double target_pitch;      // 目标pitch
    double manage_yaw;        // 控制yaw
    double yaw_vel;           // 控制yaw的速度
    double yaw_acc;           // 控制yaw的加速度
};

struct Path_Point{
    double time;
    double angle;
    double angle_vel;
    double angle_acc;
    Path_Point(double t, double a, double v,double acc) : time(t), angle(a), angle_vel(v), angle_acc(acc){}
};

static const std::string YamlPath = AUTOAIM_LEGACY_ROOT "/text.yaml";
#endif
