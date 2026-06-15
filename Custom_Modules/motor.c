/* Includes ------------------------------------------------------------------*/
/**
 * @file    motor.c
 * @brief   步进电机驱动 —— TIM1_CH1(PA8)动态STEP频率输出 + PA4方向控制
 * @note    TIM1时钟72MHz, PSC=71→1MHz, ARR=1MHz/step_hz, 占空比固定50%
 *          方向切换时自动停STEP→延时5μs→切换DIR→恢复脉冲
 */
#include "motor.h"
/* USER CODE BEGIN Includes */
#include "main.h"
#include "tim.h"
/* USER CODE END Includes */

/*
 * TIM1_CH1(PA8) is the STEP/PUL signal for DM542.
 * Motor_Set_PWM() keeps the old public name, but the argument is now a speed
 * command that is converted to STEP frequency. The output duty is fixed at 50%.
 */
#define MOTOR_TIM_CLOCK_HZ        72000000UL
#define MOTOR_STEP_HZ_PER_UNIT    4UL
#define MOTOR_STEP_MIN_HZ         20UL
#define MOTOR_STEP_MAX_HZ         20000UL
#define MOTOR_SPEED_INVALID       0xFFFFU
#define MOTOR_DIR_SETUP_DELAY_US  5UL

static uint16_t motor_last_speed_cmd = MOTOR_SPEED_INVALID;
static uint8_t motor_dir_valid = 0;
static uint8_t motor_current_dir = MOTOR_DIR_FORWARD;

static void Motor_Delay_Us(uint32_t us)
{
  uint32_t cycles = (SystemCoreClock / 1000000UL) * us;
  while(cycles-- > 0UL)
  {
    __NOP();
  }
}

static void Motor_Disable_Output(void)
{
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  motor_last_speed_cmd = MOTOR_SPEED_INVALID;
}

void Motor_Set_Dir(uint8_t dir)
{
  if(motor_dir_valid && motor_current_dir == dir)
  {
    return;
  }

  Motor_Disable_Output();

  if(dir == MOTOR_DIR_FORWARD)
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  }

  motor_current_dir = dir;
  motor_dir_valid = 1;
  Motor_Delay_Us(MOTOR_DIR_SETUP_DELAY_US);
}

void Motor_Set_PWM(uint16_t speed_cmd)
{
  uint32_t step_hz;
  uint32_t arr_val;

  if(speed_cmd == 0U)
  {
    Motor_Disable_Output();  // 彻底关停通道，下次启动走Scenario A(含EGR)
    return;
  }

  if(speed_cmd == motor_last_speed_cmd)
  {
    return;
  }

  step_hz = (uint32_t)speed_cmd * MOTOR_STEP_HZ_PER_UNIT;
  if(step_hz < MOTOR_STEP_MIN_HZ) step_hz = MOTOR_STEP_MIN_HZ;
  if(step_hz > MOTOR_STEP_MAX_HZ) step_hz = MOTOR_STEP_MAX_HZ;

  // 使用固定预分频 71，此时定时器计数频率严格为 1MHz
  // ARR = 1MHz / 目标步进频率
  arr_val = 1000000UL / step_hz;

  if(motor_last_speed_cmd == 0U || motor_last_speed_cmd == MOTOR_SPEED_INVALID)
  {
      // 场景 A: 电机从完全停止状态全新启动
      // 此时必须重置分频器并强制刷新寄存器（使用 EGR）以启动第一个干净的脉冲
      __HAL_TIM_SET_PRESCALER(&htim1, 71);
      __HAL_TIM_SET_AUTORELOAD(&htim1, (uint16_t)(arr_val - 1UL));
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t)(arr_val / 2UL));
      __HAL_TIM_SET_COUNTER(&htim1, 0);
      htim1.Instance->EGR = TIM_EGR_UG;
  }
  else
  {
      // 场景 B: 电机已经在运转中（处于加减速等调速状态）
      // 【绝对不能】使用 EGR=UG，必须利用硬件 Preload 机制无缝衔接频率
      // 硬件会在当前脉冲周期结束后，自动应用这个新的频率，做到平滑变速
      __HAL_TIM_SET_AUTORELOAD(&htim1, (uint16_t)(arr_val - 1UL));
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t)(arr_val / 2UL));
  }

  motor_last_speed_cmd = speed_cmd;
}

void Motor_Start(void)
{
  if(HAL_TIM_PWM_GetState(&htim1) == HAL_TIM_STATE_READY)
  {
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  }
}

void Motor_Stop(void)
{
  Motor_Disable_Output();
}
