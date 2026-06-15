#include "pid.h"
#include "motor.h"

// PID输出限幅值（对应最大控制输出脉冲频率）
#define PID_OUT_MAX    200.0f   // 速度脉冲频率上限
#define PID_OUT_MIN    -200.0f  // 速度脉冲频率下限
#define PID_I_OUT_MAX  100.0f   // 积分累加和上限（积分器饱和限幅值，非直接输出上限）

/**
 * @brief  PID初始化函数
 * @param  pid: PID结构体指针
 * @param  kp: 比例系数
 * @param  ki: 积分系数
 * @param  kd: 微分系数
 * @retval 无
 */
void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd)
{
    if(pid == NULL) return;
    
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
}

/**
 * @brief  PID积分清零函数
 * @param  pid: PID结构体指针
 * @retval 无
 */
void PID_ResetIntegral(PID_TypeDef *pid)
{
    if(pid == NULL) return;
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
}

/**
 * @brief  PID计算函数（位置式PID，用于闭环位置控制）
 * @param  pid: PID结构体指针
 * @param  target: 目标位置（磁栅尺脉冲数）
 * @param  actual: 实际位置（磁栅尺脉冲数）
 * @retval PID输出控制量（脉冲频率指令，需外部转换为方向与频率）
 */
float PID_Calc(PID_TypeDef *pid, float target, float actual)
{
    if(pid == NULL) return 0.0f;
    
    float error = target - actual;    // 位置误差
    float p_out = pid->Kp * error;    // 比例项
    float d_out = 0.0f;                // 微分项
    
    // 积分项（带限幅与积分分离，防止大偏差时积分饱和）
    // 只有当误差在较小范围内（例如±100脉冲 = ±100μm）才进行积分累加
    if(error > -100.0f && error < 100.0f)
    {
        pid->integral += error;
    }
    
    if(pid->integral > PID_I_OUT_MAX) pid->integral = PID_I_OUT_MAX;
    if(pid->integral < -PID_I_OUT_MAX) pid->integral = -PID_I_OUT_MAX;
    float i_out = pid->Ki * pid->integral;
    
    // 微分项（抑制超调）
    d_out = pid->Kd * (error - pid->last_error);
    pid->last_error = error;
    
    // 总输出
    float total_out = p_out + i_out + d_out;
    
    // 输出限幅
    if(total_out > PID_OUT_MAX) total_out = PID_OUT_MAX;
    if(total_out < PID_OUT_MIN) total_out = PID_OUT_MIN;
    
    return total_out;
}
