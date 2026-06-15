#include "encoder.h"
#include "tim.h"      // 引入 htim3 定时器句柄（→main.h→ENCODER_INVERT宏）

// 【新增】32位位置累加：跟踪TIM3 16位计数器的溢出，提供无绕回的32位位置值
static uint16_t last_raw_cnt = 0;
static int32_t accumulated_pos = 0;
static uint8_t pos_initialized = 0;

/**
 * @brief  获取磁栅尺当前位置（32位，无绕回）
 * @retval 当前位置（脉冲数，有符号32位）
 * @note   通过累加相邻两次16位读数的差值，自动处理计数器0~65535的溢出一致性
 *         原理：int16_t 类型的差值会自动处理16位溢出：
 *           正向绕回：65535→0，delta=(int16_t)(0-65535)=1   ✓
 *           反向绕回：0→65535，delta=(int16_t)(65535-0)=-1  ✓
 */
int32_t Mag_Get_Pos(void)
{
    uint16_t raw_cnt = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    
    if(!pos_initialized)
    {
        // 第一次调用：初始化累加器与硬件计数器同步
        pos_initialized = 1;
        last_raw_cnt = raw_cnt;
        accumulated_pos = (int32_t)raw_cnt;
    }
    else
    {
        // 计算相邻两次读数的差值（int16_t自动处理65535↔0的溢出）
        int16_t delta = (int16_t)(raw_cnt - last_raw_cnt);
        accumulated_pos += delta;
        last_raw_cnt = raw_cnt;
    }
    
    #if ENCODER_INVERT
    return -accumulated_pos;
    #else
    return accumulated_pos;
    #endif
}

/**
 * @brief  获取磁栅尺速度（相邻两次位置差分）
 * @param  last_pos: 上一次位置
 * @param  curr_pos: 当前位置
 * @retval 速度（脉冲/采样周期）
 */
int32_t Mag_Get_Speed(int32_t last_pos, int32_t curr_pos)
{
    return curr_pos - last_pos;
}

/**
 * @brief  重置磁栅尺位置计数器（同时重置硬件计数器和软件累加器）
 */
void Mag_Reset_Pos(void)
{
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    accumulated_pos = 0;
    last_raw_cnt = 0;
    // pos_initialized 保持为1使后续调用正常累加
}
