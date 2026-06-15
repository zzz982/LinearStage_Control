/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define KEY2_Pin GPIO_PIN_13
#define KEY2_GPIO_Port GPIOC
#define KEY1_Pin GPIO_PIN_0
#define KEY1_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

// ================== 全工程统一宏定义（main.c / motor.c / homing.c 共享） ==================
// 电机方向宏定义（避免硬编码0/1导致逻辑错误）
#define MOTOR_DIR_FORWARD  0
#define MOTOR_DIR_BACKWARD 1

// 编码器方向反转（0=正常，1=反转，用于硬件A/B接反时换向）
#define ENCODER_INVERT  0

// 系统运行模式宏定义
#define MODE_VEL    1

// ================== 限位开关定义（仅保留PB0终点限位，已统一） ==================
#define RIGHT_LIMIT_PIN   GPIO_PIN_0
#define LIMIT_PORT        GPIOB

// ================== 按键定义（100%匹配开发板丝印：KEY1=PA0，KEY2=PC13） ==================
#define KEY1_PIN          GPIO_PIN_0   // 板载KEY1，PA0，模式切换/回零
#define KEY1_PORT         GPIOA
#define KEY2_PIN          GPIO_PIN_13  // 板载KEY2，PC13，暂停/继续
#define KEY2_PORT         GPIOC

// 串口打印缓冲区
#define UART_BUF_SIZE     100

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
