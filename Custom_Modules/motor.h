/**
 * @file    motor.h
 * @brief   步进电机驱动模块 (DM542 + TIM1_CH1 STEP/DIR 控制)
 * @note    TIM1_CH1(PA8)输出STEP脉冲, PA4控制DIR方向, ENA悬空使驱动器默认持续使能
 *          电机方向宏定义 MOTOR_DIR_FORWARD(0) / MOTOR_DIR_BACKWARD(1) 在 main.h 中统一定义
 */

#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

/**
 * @brief  设置电机方向
 * @param  dir: MOTOR_DIR_FORWARD(0) 或 MOTOR_DIR_BACKWARD(1)
 * @note   切换方向时会自动先停STEP脉冲，插入5μs方向建立延时，再恢复脉冲
 */
void Motor_Set_Dir(uint8_t dir);

/**
 * @brief  设置电机速度（通过调整TIM1 STEP频率实现）
 * @param  speed_cmd: 速度命令值，0=停止，1~5000对应20~20000Hz STEP频率（由于20Hz下限保护，1~4的输出均为20Hz）
 * @note   内部将 speed_cmd*4 换算为STEP频率(Hz)，50%占空比方波输出
 */
void Motor_Set_PWM(uint16_t speed_cmd);

/**
 * @brief  启动电机脉冲输出 (开启TIM1_CH1 PFM方波)
 */
void Motor_Start(void);

/**
 * @brief  停止电机（关闭STEP脉冲输出）
 */
void Motor_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
