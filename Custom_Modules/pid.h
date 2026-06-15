#ifndef __PID_H
#define __PID_H

// 引入标准头文件，解决NULL和浮点型定义问题
#include <stdint.h>
#include <stddef.h>

// PID结构体定义（位置环PID，适配高精度定位）
typedef struct
{
    float Kp;          // 比例系数（核心，决定响应速度）
    float Ki;          // 积分系数（消除静差，不宜过大）
    float Kd;          // 微分系数（抑制超调，提升稳定性）
    float integral;    // 积分累积值（防止饱和）
    float last_error;  // 上一次偏差值（微分项计算用）
} PID_TypeDef;

// PID函数声明
void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd);  // PID初始化
void PID_ResetIntegral(PID_TypeDef *pid);                       // PID积分清零
float PID_Calc(PID_TypeDef *pid, float target, float actual);   // PID计算（输入目标/实际值，输出速度脉冲频率指令）

#endif /* __PID_H */