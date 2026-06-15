/**
 * @file    homing.h
 * @brief   六阶段自动回零模块（限位粗定位 + Z相精定位）
 * @note    六阶段流程: FAST→BACK→SLOW→OFFSET→Z_FINE→DONE
 *          Z_FINE阶段: 极低速(0.2mm/s)单向搜索Z脉冲，±1μm回零精度
 *          若Z_FINE超8mm未捕获Z脉冲，降级回退使用OFFSET方案归零
 */

#ifndef __HOMING_H
#define __HOMING_H

#include <stdint.h>

/* 回零状态宏定义 */
#define HOMING_UNFINISHED  0  /**< 回零未完成（进行中） */
#define HOMING_FINISHED    1  /**< 回零成功完成 */
#define HOMING_ERROR       2  /**< 回零失败（超行程/限位异常） */

/* Z相精定位阶段参数 */
#define HOMING_Z_FINE_SPEED       40U   /**< 极低速 speed_cmd=40 → ~160 STEP/s → ~0.2mm/s */
#define HOMING_Z_FINE_MAX_DIST    8000L /**< Z_FINE最大搜索距离 8mm (4个极距，警告：由于OFFSET仅退避2mm，若Z脉冲前2mm内丢失，电机将会触上限位堵转) */

/**
 * @brief  启动回零流程（入口函数，由KEY1长按触发）
 * @note   自动判断当前位置是否已在限位，选择FAST或BACK作为起始阶段
 */
void Homing_Start(void);

/**
 * @brief  获取回零当前状态
 * @retval HOMING_UNFINISHED / HOMING_FINISHED / HOMING_ERROR
 */
uint8_t Homing_Get_State(void);

/**
 * @brief  回零主循环（状态机，需在主循环中持续调用）
 */
void Homing_Loop(void);

#endif /* __HOMING_H */
