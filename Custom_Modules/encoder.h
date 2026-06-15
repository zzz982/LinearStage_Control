/**
 * @file    encoder.h
 * @brief   磁栅尺编码器模块 (SIKO MSK200/1, TIM3 X4正交解码, 1μm/计数)
 * @note    磁栅尺参数: 极距2.0mm, 信号周期4μm, X4后分辨率=1μm/计数
 *          TIM3 16位计数器由软件32位累加器扩展，支持全行程(±110mm)无绕回
 */

#ifndef __ENCODER_H
#define __ENCODER_H

#include <stdint.h>

/**
 * @brief  读取磁栅尺当前位置（32位累加值，单位: 脉冲，1脉冲=1μm）
 * @retval 当前位置（有符号32位，1计数=1μm）
 */
int32_t Mag_Get_Pos(void);

/**
 * @brief  重置磁栅尺位置计数器（硬件计数器+软件累加器同时清零）
 */
void Mag_Reset_Pos(void);

/**
 * @brief  计算磁栅尺速度（相邻两次位置差分）
 * @param  last_pos: 上一次位置
 * @param  curr_pos: 当前位置
 * @retval 速度值（μm/采样周期）
 */
int32_t Mag_Get_Speed(int32_t last_pos, int32_t curr_pos);

#endif /* __ENCODER_H */
