/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Linear Stage Control - 闭环PID + 回零 + LED状态指示 + 限位保护
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "motor.h"
#include "pid.h"
#include "encoder.h"
#include "homing.h"
#include <stdarg.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// 系统运行模式枚举
typedef enum {
    SYS_MODE_IDLE,      // 空闲/待命
    SYS_MODE_VEL,       // 速度模式（开环匀速）
    SYS_MODE_POS,       // 位置模式（PID闭环点到点）
    SYS_MODE_HOMING     // 回零模式
} SystemMode_TypeDef;

// LED状态枚�?
typedef enum {
    LED_OFF = 0,
    LED_GREEN,          // 绿色 - 速度模式
    LED_BLUE,           // 蓝色 - 位置模式 / VEL反转
    LED_YELLOW,         // �?�?�? - (暂未使用)
    LED_RED_FLASH,      // 红色闪烁 - 暂停 / 回零�?
    LED_RED,            // 红色 - 停止/错误
    LED_PURPLE,         // �?�?�? - (暂未使用)
    LED_WHITE,          // �?�?�?�? - 启动
    LED_BLUE_FLASH      // 蓝色闪烁 - POS到位
} LED_State_TypeDef;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
char uart_print_buf[128];

// MODE_VEL / MODE_POS 已在 main.h 中统一定义
#define MODE_POS    2

// 编码器方�?

// 限位引脚定义（RIGHT_LIMIT_PIN/LIMIT_PORT 已在 main.h 中统一定义�?
// 硬件：SN04-N �?YH-CD44 CH4(NPN模式) �?输出D �?PB0
// NPN触发=拉低→光耦LED亮→输出D高电平→PB0=SET
#define LIMIT_ACTIVE    GPIO_PIN_SET   // 限位触发时PB0的电�?

// LED引脚定义（低电平点亮�?
#define LED_R_PORT   GPIOA
#define LED_R_PIN    GPIO_PIN_1
#define LED_G_PORT   GPIOA
#define LED_G_PIN    GPIO_PIN_2
#define LED_B_PORT   GPIOA
#define LED_B_PIN    GPIO_PIN_3

// 限位开关配�?�?2026-05-11 硬件已接好，启用
#define LIMIT_ENABLED   1      // 0=限位开关未接，1=已接  [限位已开启]
#define HOMING_ENABLED  1      // 0=回零禁用�?=回零启用（需要限位开关）

// PID参数（初始值，需实测调整�?
#define PID_KP   5.0f   // �?.0，降下来防止高速过�?
#define PID_KI   0.03f  // �?.02�?.05冲过头，折中试一�?
#define PID_KD   1.5f   // �?.0，提高抑制震�?
#define PID_ERR_THRESHOLD  3   // 到达目标阈值：3脉冲(3μm) [实测误差±1μm，设3μm足够]
#define PULSE_PER_MM  1000.0f   // 根据110mm实测反推，对应订货后缀 0.002/250 (F=250脉冲/mm, X4=1000计数/mm, 1计数=1μm)

// 位置模式参数
#define POS_MAX_SPEED    200   // 最大速度命令（motor.c中换算为STEP频率）
#define POS_TARGET_MM    -50.0f // 默认目标位置（mm），�?FORWARD=远离限位
#define POS_RAMP_RATE    2     // 缓启缓停斜率 (PWM/cycle)，原8太快肉眼看不出来

// 自动循环测试（上电→回零→POS→VEL随机位移→POS→循环）
#define AUTO_TEST_ENABLED  1   // 1=启用自动循环测试，0=手动控制
#define AUTO_TEST_POS_STABLE_S 5    // POS到位后稳定N秒再切VEL

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern volatile uint32_t z_pulse_count;

// 系统级变�?
static SystemMode_TypeDef sys_mode = SYS_MODE_IDLE; // 这里不要随便改，后面会覆�?
static LED_State_TypeDef led_state = LED_OFF;
    static uint32_t led_flash_tick = 0;

// 上电按键屏蔽标志
static uint8_t key_mask_ready = 0;        // 0=初始化期间，屏蔽按键�?=可以响应按键
static uint32_t power_on_tick = 0;         // 上电时刻，用于时间门槛屏�?
static uint8_t ramp_needs_reset = 0;       // 进入POS模式时重置斜�?

// PID相关
static PID_TypeDef pos_pid;
static int32_t target_pos_pulse = 0;
static int32_t current_pos = 0;
static uint8_t pos_arrived_flag = 0;

// 用于速度模式
uint8_t motion_mode = MODE_VEL;  // 冗余遗留变量，当前已被 sys_mode 状态机替代
uint8_t current_dir = MOTOR_DIR_FORWARD;
uint8_t motor_pause_flag = 0;
float vel_target = 220.0f;  // 速度命令，motor.c中约等于 speed_cmd*4 STEP/s
uint16_t g_current_speed_cmd = 0;  // 当前速度命令，供STATUS打印

// 软件限位（仅回零后生效）
#define SOFT_LIMIT_FORWARD  -100000   // -100mm (限位开关离起点~0.5mm，可用行程约100mm)
static uint8_t homing_done_once = 0;  // 0=未曾回零�?=至少回零过一�?

// 自动循环测试状�?
#if AUTO_TEST_ENABLED
static uint32_t test_cycle = 0;           // 已完成循环数
static int32_t test_vel_target = 0;       // VEL随机目标位置(计数)
static uint32_t test_stable_tick = 0;     // POS到位后稳定计�?
static int32_t test_vel_start_pos = 0;    // VEL启动位置
static uint8_t test_vel_dir = 0;          // VEL随机方向
static uint32_t test_phase_tick = 0;      // 当前阶段开始时�?
#define TEST_VEL_DIST_MIN_MM  5.0f        // VEL最小位�?mm)
#define TEST_VEL_DIST_MAX_MM  100.0f      // VEL最大位�?mm)

// 简单伪随机�?
static uint32_t test_rand_seed = 0;
static uint32_t Test_Rand(void) {
    test_rand_seed = test_rand_seed * 1103515245 + 12345;
    return (test_rand_seed >> 16) & 0x7FFF;
}
static int32_t Test_RandRange(int32_t min, int32_t max) {
    return min + (int32_t)(Test_Rand() % (uint32_t)(max - min + 1));
}
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void System_Init(void);
void LED_Set(LED_State_TypeDef state);
void System_Mode_Switch(SystemMode_TypeDef mode);
void Position_Control_Loop(void);
void Vel_Control_Loop(void);
#if AUTO_TEST_ENABLED
void AutoTest_Control(void);
#endif
void UART_Log(const char* fmt, ...);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  LED控制函数
  * @param  state: LED状�?
  */
void LED_Set(LED_State_TypeDef state)
{
    led_state = state;
    
    // 默认全部熄灭（低电平点亮，所以SET=灭）
    HAL_GPIO_WritePin(LED_R_PORT, LED_R_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_G_PORT, LED_G_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_B_PORT, LED_B_PIN, GPIO_PIN_SET);
    
    switch(state)
    {
        case LED_OFF:
            break;
        case LED_GREEN:
            HAL_GPIO_WritePin(LED_G_PORT, LED_G_PIN, GPIO_PIN_RESET);
            break;
        case LED_BLUE:
            HAL_GPIO_WritePin(LED_B_PORT, LED_B_PIN, GPIO_PIN_RESET);
            break;
        case LED_YELLOW:
            HAL_GPIO_WritePin(LED_R_PORT, LED_R_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_G_PORT, LED_G_PIN, GPIO_PIN_RESET);
            break;
        case LED_RED:
            HAL_GPIO_WritePin(LED_R_PORT, LED_R_PIN, GPIO_PIN_RESET);
            break;
        case LED_PURPLE:
            HAL_GPIO_WritePin(LED_R_PORT, LED_R_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_B_PORT, LED_B_PIN, GPIO_PIN_RESET);
            break;
        case LED_WHITE:
            HAL_GPIO_WritePin(LED_R_PORT, LED_R_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_G_PORT, LED_G_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_B_PORT, LED_B_PIN, GPIO_PIN_RESET);
            break;
        default:
            break;
    }
}

/**
  * @brief  LED闪烁处理（每调用一次更新一次）
  */
void LED_Update(void)
{
    if(led_state != LED_RED_FLASH && led_state != LED_BLUE_FLASH) return;
    
    uint32_t now = HAL_GetTick();
    if(now - led_flash_tick >= 300)  // 300ms切换一�?
    {
        led_flash_tick = now;
        static uint8_t flash_on = 0;
        flash_on = !flash_on;
        
        if(led_state == LED_RED_FLASH)
        {
            // 红色闪烁（暂�?回零�?
            if(flash_on)
                HAL_GPIO_WritePin(LED_R_PORT, LED_R_PIN, GPIO_PIN_RESET);
            else
                HAL_GPIO_WritePin(LED_R_PORT, LED_R_PIN, GPIO_PIN_SET);
        }
        else // LED_BLUE_FLASH
        {
            // 蓝色闪烁（到位）
            if(flash_on)
                HAL_GPIO_WritePin(LED_B_PORT, LED_B_PIN, GPIO_PIN_RESET);
            else
                HAL_GPIO_WritePin(LED_B_PORT, LED_B_PIN, GPIO_PIN_SET);
        }
    }
}

/**
  * @brief  系统初始�?
  */
void System_Init(void)
{
    // PID初始�?
    PID_Init(&pos_pid, PID_KP, PID_KI, PID_KD);

    // 启动编码�?
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

    // 读取初始位置
    current_pos = Mag_Get_Pos();

    // 测试：依次点亮每盏灯（验证LED是否正常工作�?
    LED_Set(LED_RED);
    HAL_Delay(500);
    LED_Set(LED_GREEN);
    HAL_Delay(500);
    LED_Set(LED_BLUE);
    HAL_Delay(500);
    LED_Set(LED_OFF);
    HAL_Delay(200);

    // 发送启动信�?
    UART_Log("=========================================\r\n");
    UART_Log("  Linear Stage Control System v2.1\r\n");
    UART_Log("=========================================\r\n");
    UART_Log("KEY1短按: 切换速度/位置模式\r\n");
    UART_Log("KEY2短按: 暂停/停止\r\n");
    UART_Log("-----------------------------------------\r\n");
}

/**
 * @brief  串口日志打印封装（变参）
 */
void UART_Log(const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vsnprintf(uart_print_buf, sizeof(uart_print_buf), fmt, args);
    va_end(args);
    HAL_UART_Transmit(&huart1, (uint8_t*)uart_print_buf, strlen(uart_print_buf), 100);
}

/**
  * @brief  切换系统模式
  *         注意：该函数在主线程中调用，重置的pos_pid结构体和sys_mode在TIM4中断中被频繁读取，
  *         存在非原子操作下的数据竞争风险。后续建议在重置期间加入临界区中断锁。
  */
void System_Mode_Switch(SystemMode_TypeDef mode)
{
    sys_mode = mode;
    pos_arrived_flag = 0;
    motor_pause_flag = 0; // 任何模式切换都重置暂停状�?
    
    switch(mode)
    {
        case SYS_MODE_IDLE:
            Motor_Stop();
            LED_Set(LED_RED);
            UART_Log("[SYS] Mode -> IDLE (停止)\r\n");
            break;
            
        case SYS_MODE_VEL:
            LED_Set(LED_GREEN);
            UART_Log("[SYS] Mode -> VELOCITY (速度模式)\r\n");
            break;
            
        case SYS_MODE_POS:
            Motor_Stop();                                 // 先停掉VEL残留运动
            PID_Init(&pos_pid, PID_KP, PID_KI, PID_KD);  // 清零历史积分，避免切换冲�?
            ramp_needs_reset = 1;                         // 下一轮斜坡从0开�?
            LED_Set(LED_BLUE);
            UART_Log("[SYS] Mode -> POSITION (位置模式)\r\n");
            UART_Log("[POS] Target: %ld pulse (%.1fmm)\r\n",
                    (long)target_pos_pulse, target_pos_pulse / PULSE_PER_MM);
            break;
            
        case SYS_MODE_HOMING:
            #if HOMING_ENABLED == 1
            LED_Set(LED_RED_FLASH);
            UART_Log("[SYS] Mode -> HOMING\r\n");
            Homing_Start();
            #else
            // 【重要】如果不使用限位，禁止切换到回零模式，直接切�?IDLE
            Motor_Stop();
            LED_Set(LED_RED);
            UART_Log("[ERROR] Homing disabled or limit switch unavailable\r\n");
            sys_mode = SYS_MODE_IDLE; // 直接覆盖模式，防止后续被错误执行
            #endif
            break;
    }
}

/**
  * @brief  位置模式控制循环（PID闭环 + 持续纠偏）
  *         到达目标后蓝灯闪烁监控，位置被扰动后自动重新定位
  *         注意：该函数运行在TIM4定时中断中。主线程在切换模式或重置PID时读写部分共享结构体（如pos_pid），
  *         由于未加中断锁，存在并发数据竞争风险。
  */
void Position_Control_Loop(void)
{
    if(sys_mode != SYS_MODE_POS) return;
    
    // 读取当前位置
    current_pos = Mag_Get_Pos();
    
    // 计算位置误差
    int32_t error = target_pos_pulse - current_pos;

    static float pos_ramp_speed = 0.0f;

    if(ramp_needs_reset) {
        pos_ramp_speed = 0.0f;
        ramp_needs_reset = 0;
    }
    
    // ===== 到达目标范围（小阈值） =====
    if(error > -PID_ERR_THRESHOLD && error < PID_ERR_THRESHOLD && pos_ramp_speed > -30.0f && pos_ramp_speed < 30.0f)
    {
        if(!pos_arrived_flag)
        {
            // 首次到达，清空PID积分（软着陆防冲过头），打印到位信息，切蓝�?
            Motor_Stop();
            pos_arrived_flag = 1;
            PID_ResetIntegral(&pos_pid);
            LED_Set(LED_BLUE_FLASH);
            UART_Log("[POS] Arrived! Target=%ld, Actual=%ld, Error=%ld, Ramp=%u\r\n",
                    (long)target_pos_pulse, (long)current_pos, (long)error,
                    (unsigned int)(pos_ramp_speed > 0 ? (uint16_t)pos_ramp_speed : (uint16_t)(-pos_ramp_speed)));
        }
        // 已到达→保持蓝闪监控，不做动�?
        return;
    }
    
    // ===== 已到达状态下的滞回判断（大阈值，防止震荡�?----
    // 已到�?蓝闪)状态下，误差必须超�?WAKEUP_THRESHOLD才重新定�?
    // 【滞回设计】PID_ERR_THRESHOLD=3(±3μm) < WAKEUP_THRESHOLD=8(±8μm)，确保不回弹震荡
    #define WAKEUP_THRESHOLD  8   // 8脉冲(8μm)——到位后|error|<8保持蓝闪不动，|error|�?才唤醒重定位
    if(pos_arrived_flag)
    {
        if(error > -WAKEUP_THRESHOLD && error < WAKEUP_THRESHOLD)
        {
            // 误差�?脉冲(7μm)以内 �?保持蓝闪不动
            return;
        }
        
        // 误差超过8脉冲(8μm) �?重新定位
        pos_arrived_flag = 0;
        ramp_needs_reset = 1;   // 【重写】斜坡归�?方向检测：避免扰动后电机以遗留的高速跑反方�?
        PID_ResetIntegral(&pos_pid); // 【新增】清空PID积分，保证轻柔启动，防止一启动又冲过头
        LED_Set(LED_BLUE); // 蓝灯常亮表示重新定位�?
        UART_Log("[POS] Disturbed! Error=%ld, re-correcting...\r\n",
                (long)error);
    }
    
    // ===== PID计算与电机控�?=====
    float pid_output = PID_Calc(&pos_pid, (float)target_pos_pulse, (float)current_pos);
    
    // ==== 极近距离降速限制（软着陆蠕动模式） ====
    // 强制限制最�?0个脉�?20μm)内的最大速度，防止大比例系数导致的高速过�?
    if(error > -20 && error < 20)
    {
        // 限制在非常低的速度（PWM 20~30 之间蠕动），可以彻底消除物理过冲惯�?
        if(pid_output > 25.0f) pid_output = 25.0f;
        if(pid_output < -25.0f) pid_output = -25.0f;
    }

    
    // 限幅
    if(pid_output > POS_MAX_SPEED) pid_output = POS_MAX_SPEED;
    if(pid_output < -POS_MAX_SPEED) pid_output = -POS_MAX_SPEED;
    
    // ===== 斜坡管理（每周期更新，全真实时平滑加减速） =====
    
    // 【方向突变检测】每周期检查：PID输出和当前斜坡符号相反时，立即归零
    if((pid_output > 0 && pos_ramp_speed < 0) || (pid_output < 0 && pos_ramp_speed > 0)) {
        pos_ramp_speed = 0.0f;
    }
    
    // 每周期更新斜坡（无门控），保证到位检查与斜坡减速同步
    {
        float diff = pid_output - pos_ramp_speed;
        float abs_diff = diff > 0 ? diff : -diff;
        float dynamic_ramp = POS_RAMP_RATE / 10.0f;  // 每周期的斜率=原每10周期斜率的1/10

        // 判断是加速还是减速
        uint8_t is_decel = 0;
        if ((pos_ramp_speed > 0 && diff < 0) || (pos_ramp_speed < 0 && diff > 0)) {
            is_decel = 1;
        }

        if (is_decel) {
            // 指数衰减减速：每次消除当前差值(ramp-pid)的30%
            // 永远追不到pid_output以下，不会交叉抖振
            // 误差大时减速快(一开始≈5/周期)，接近时自然放缓
            float decel_diff = pos_ramp_speed - pid_output;
            if(decel_diff < 0) decel_diff = -decel_diff;
            dynamic_ramp = decel_diff * 0.15f;
            if(dynamic_ramp < 1.0f) dynamic_ramp = 1.0f;
        } else {
            if (abs_diff > 100) dynamic_ramp = POS_RAMP_RATE * 1.5f / 10.0f;
            else if (abs_diff < 30) dynamic_ramp = POS_RAMP_RATE * 0.5f / 10.0f;
        }
        
        if(pid_output > pos_ramp_speed)
        {
            pos_ramp_speed += dynamic_ramp;
            if(pos_ramp_speed > pid_output) pos_ramp_speed = pid_output;
        }
        else if(pid_output < pos_ramp_speed)
        {
            pos_ramp_speed -= dynamic_ramp;
            if(pos_ramp_speed < pid_output) pos_ramp_speed = pid_output;
        }
    }
    uint16_t final_pwm = (uint16_t)(pos_ramp_speed > 0 ? pos_ramp_speed : -pos_ramp_speed);
    g_current_speed_cmd = final_pwm;
    
    // 调试日志（每次变化都打印，100ms节流防止刷屏）
    static uint16_t last_ramp_print = 0;
    static uint32_t last_ramp_print_tick = 0;
    uint32_t ramp_print_now = HAL_GetTick();
    if(final_pwm != last_ramp_print &&
       ramp_print_now - last_ramp_print_tick >= 100)
    {
        last_ramp_print = final_pwm;
        last_ramp_print_tick = ramp_print_now;
        UART_Log("[RAMP] SpeedCmd=%d\r\n", final_pwm);
    }
    
    // 电机控制
    // 【修复】日志验证：FORWARD=递减(Pos�?，BACKWARD=递增(Pos�?
    // PID正输�?需要增大位置→驱动BACKWARD(递增); 负输�?需要减小位置→驱动FORWARD(递减)
    if(pos_ramp_speed > 0)
    {
        Motor_Set_Dir(MOTOR_DIR_BACKWARD);
        Motor_Set_PWM(final_pwm);
        Motor_Start();  // 【关键修复】重新定位时必须调用 Motor_Start 启动PWM
    }
    else
    {
        Motor_Set_Dir(MOTOR_DIR_FORWARD);
        Motor_Set_PWM(final_pwm);
        Motor_Start();  // 【关键修复】重新定位时必须调用 Motor_Start 启动PWM
    }
}

/**
  * @brief  速度模式控制循环（开环）
  */
void Vel_Control_Loop(void)
{
    if(sys_mode != SYS_MODE_VEL) return;
    if(motor_pause_flag) return;

    g_current_speed_cmd = (uint16_t)vel_target;
    Motor_Set_Dir(current_dir);
    Motor_Set_PWM((uint16_t)vel_target);
    Motor_Start();  // �若 PWM 通道已运行则无操作，若被 Stop 过则重启
}

#if AUTO_TEST_ENABLED
/**
  * @brief  自动循环测试控制
  *         上电回零 �?VEL测�?0s �?POS -50mm �?VEL随机位移 �?POS �?循环
  */
void AutoTest_Control(void)
{
    uint32_t now = HAL_GetTick();
    static uint8_t test_phase = 0; // 0=等待回零, 1=等待POS到位, 2=等待稳定, 3=VEL�?
    
    // 阶段0: 等待回零完成
    if(test_phase == 0)
    {
        if(sys_mode == SYS_MODE_IDLE && homing_done_once)
        {
            HAL_Delay(500);
            System_Mode_Switch(SYS_MODE_POS);
            test_phase = 1;
            test_phase_tick = now;
            UART_Log("[TEST] 回零完成, 进入POS\r\n");
        }
        return;
    }
    
    // 阶段1: 等待POS到位
    if(test_phase == 1)
    {
        if(sys_mode == SYS_MODE_POS && pos_arrived_flag)
        {
            test_stable_tick = now;
            test_phase = 2;
        }
        return;
    }
    
    // 阶段2: 到位后稳定等�?
    if(test_phase == 2)
    {
        if(sys_mode != SYS_MODE_POS || !pos_arrived_flag)
        {
            test_phase = 1; // 被扰动唤醒，回阶�?
            return;
        }
        if(now - test_stable_tick > (uint32_t)(AUTO_TEST_POS_STABLE_S * 1000))
        {
            // 稳定时间�?�?计算随机VEL位移
            int32_t cur = Mag_Get_Pos();
            int32_t fwd_limit  = (int32_t)(-100.0f * PULSE_PER_MM); // -100mm（对齐软限位�?
            int32_t back_limit = (int32_t)(-2.0f * PULSE_PER_MM);    // -2mm 避免直撞起点限位
            
            int32_t dir = (Test_Rand() % 2) ? MOTOR_DIR_BACKWARD : MOTOR_DIR_FORWARD;
            int32_t target = cur;
            
            if(dir == MOTOR_DIR_FORWARD)
            {
                // FORWARD: 从当前到 fwd_limit(-100mm) 之间随机
                int32_t margin = cur - fwd_limit;
                if(margin < (int32_t)(TEST_VEL_DIST_MIN_MM * PULSE_PER_MM))
                    dir = MOTOR_DIR_BACKWARD; // 太近了，强制反向
                else
                    target = Test_RandRange(fwd_limit, cur - (int32_t)(TEST_VEL_DIST_MIN_MM * PULSE_PER_MM));
            }
            
            if(dir == MOTOR_DIR_BACKWARD)
            {
                // BACKWARD: 从当前到 back_limit(-2mm) 之间随机
                int32_t margin = back_limit - cur;
                if(margin < (int32_t)(TEST_VEL_DIST_MIN_MM * PULSE_PER_MM))
                {
                    dir = MOTOR_DIR_FORWARD;
                    target = fwd_limit; // 太近了，强制FORWARD
                }
                else
                {
                    target = Test_RandRange(cur + (int32_t)(TEST_VEL_DIST_MIN_MM * PULSE_PER_MM), back_limit);
                }
            }
            
            test_vel_target = target;
            test_vel_dir = dir;
            test_vel_start_pos = cur;
            test_cycle++;
            
            // 切VEL
            current_dir = dir;
            System_Mode_Switch(SYS_MODE_VEL);
            Motor_Set_Dir(dir);
            Motor_Set_PWM((uint16_t)vel_target);
            Motor_Start();
            test_phase = 3;
            
            UART_Log("[TEST] Cycle#%lu | VEL %s -> %.1fmm | 起点:%.1fmm\r\n",
                    (unsigned long)test_cycle,
                    (dir == MOTOR_DIR_FORWARD) ? "FORWARD" : "BACKWARD",
                    (float)target / PULSE_PER_MM,
                    (float)cur / PULSE_PER_MM);
        }
        return;
    }
    
    // 阶段3: VEL中，检测到达目标或限位触发
    if(test_phase == 3)
    {
        if(sys_mode != SYS_MODE_VEL) return;
        current_pos = Mag_Get_Pos();
        int32_t moved;
        
        if(test_vel_dir == MOTOR_DIR_FORWARD)
        {
            moved = test_vel_start_pos - current_pos;
            if(current_pos <= test_vel_target || current_dir != MOTOR_DIR_FORWARD) // 到达或软限位反转
            {
                Motor_Stop();
                System_Mode_Switch(SYS_MODE_POS);
                test_phase = 1;
                test_phase_tick = now;
                UART_Log("[TEST] VEL完成, 切回POS | 终点:%.1fmm | 位移:%.1fmm\r\n",
                        (float)current_pos / PULSE_PER_MM, (float)moved / PULSE_PER_MM);
                return;
            }
        }
        else
        {
            moved = current_pos - test_vel_start_pos;
            if(current_pos >= test_vel_target || current_dir != MOTOR_DIR_BACKWARD) // 到达或起点限位反�?
            {
                Motor_Stop();
                System_Mode_Switch(SYS_MODE_POS);
                test_phase = 1;
                test_phase_tick = now;
                UART_Log("[TEST] VEL完成, 切回POS | 终点:%.1fmm | 位移:%.1fmm\r\n",
                        (float)current_pos / PULSE_PER_MM, (float)moved / PULSE_PER_MM);
                return;
            }
        }
    }
}
#endif

/**
  * @brief  The application entry point.
  */
int main(void)
{
    /* USER CODE BEGIN 1 */
    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_TIM1_Init();
    MX_USART1_UART_Init();
    MX_TIM3_Init();
    MX_TIM2_Init();   // TIM2：Z相输入捕获 (PA15, AFIO完全重映射, ICFilter=15, ~3.6μs滤波)
    MX_TIM4_Init();

    /* USER CODE BEGIN 2 */
    // TIM1由motor.c动态配置STEP频率，上电保持无脉冲
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    Motor_Stop();
    Motor_Set_Dir(MOTOR_DIR_FORWARD);
    
    // 【修复】所有初始化（包括自检延迟）全部完成后，才开启按键和切换模式
    // 顺序不能乱：�?System_Init（内部含LED自检），再开启按键响应，最后切模式
    System_Init();
    
    // 强制清除挂起的按键状态（防止初始化期间的抖动被记录）
    HAL_Delay(100); 
    key_mask_ready = 1;
    power_on_tick = HAL_GetTick();

    // 设置默认目标位置
    target_pos_pulse = (int32_t)(POS_TARGET_MM * PULSE_PER_MM);

    // 默认进入速度模式（此时自检已完成，key_mask_ready已开启）
    #if AUTO_TEST_ENABLED
        System_Mode_Switch(SYS_MODE_HOMING);
        test_rand_seed = HAL_GetTick();
        test_phase_tick = HAL_GetTick();
    #else
        System_Mode_Switch(SYS_MODE_VEL);
    #endif

    // 启动 TIM4 定时器中断，以 1kHz 频率独立驱动 PID 闭环控制
    HAL_TIM_Base_Start_IT(&htim4);
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    static uint32_t print_timer = 0;
    static uint32_t key1_press_start = 0;
    static uint8_t key1_was_pressed = 0;
    static uint8_t key1_wait_release = 0;

    while (1)
    {
        // ========== LED闪烁更新 ==========
        LED_Update();

        // ========== 按键处理 ==========
        uint32_t now = HAL_GetTick();
        // 【基于原理图PDF规格书的正确按键逻辑�?
        // 野火F103C8T6核心板原理图P5确认�?
        //   SW1(PA0 WK_UP) + SW2(PC13) �?电路完全一样！
        //     4.7KΩ下拉电阻(R9/R11)到GND + 1KΩ保护电阻(R8/R10) + 0.1μF消抖电容(C14/C15)
        //     未按�?0(低电�?, 按下=1(高电�? �?**高电平触�?*
        //   SW3(NRST) �?4.7KΩ上拉�?.3V, 按下接GND �?**低电平触�?复位)**
        // 软件方案：连�?0次确�?滤波，确保抗干扰
        static uint8_t k1_filter_cnt = 0;
        static uint8_t k2_filter_cnt = 0;
        static uint8_t key1_state = GPIO_PIN_RESET; // 默认PA0硬件下拉�?
        static uint8_t key2_state = GPIO_PIN_RESET; // 默认PC13硬件下拉�?

        uint8_t raw_k1 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
        uint8_t raw_k2 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);

        // 两路按键电路完全一样：连续50ms读到HIGH(=1)才认为有效按�?
        if(raw_k1 == GPIO_PIN_SET) {
            if(k1_filter_cnt < 50) k1_filter_cnt++;
            else key1_state = GPIO_PIN_SET;
        } else {
            k1_filter_cnt = 0;
            key1_state = GPIO_PIN_RESET;
        }

        if(raw_k2 == GPIO_PIN_SET) {
            if(k2_filter_cnt < 50) k2_filter_cnt++;
            else key2_state = GPIO_PIN_SET;
        } else {
            k2_filter_cnt = 0;
            key2_state = GPIO_PIN_RESET;
        }

        // 上电屏蔽：仅在初始化完成标志�?时才响应按键
        // 初始化期�?System_Init() 里的 HAL_Delay() 不会影响此判�?
        if(key_mask_ready && (now - power_on_tick > 1500))
        {
            // KEY1: 短按切换模式，长按回�?
            // 【硬件规格】PA0(WK_UP) 硬件下拉，按�?1即触�?
            if(key1_state == GPIO_PIN_SET)
            {
                if(!key1_wait_release && !key1_was_pressed)
                {
                    key1_press_start = now;
                    key1_was_pressed = 1;
                }
            }
            else // 未按�?(GPIO_PIN_RESET)
            {
                if(key1_wait_release)
                {
                    key1_wait_release = 0;
                    key1_was_pressed = 0;
                }
                else if(key1_was_pressed)
                {
                    uint32_t press_time = now - key1_press_start;
                    if(press_time < 30)
                    {
                        // 抖动，忽�?
                    }
                    else if(press_time < 1500)
                    {
                        // 短按：切换速度/位置模式
                        if(sys_mode == SYS_MODE_VEL)
                        {
                            System_Mode_Switch(SYS_MODE_POS);
                        }
                        else if(sys_mode == SYS_MODE_POS)
                        {
                            System_Mode_Switch(SYS_MODE_VEL);
                        }
                        else if(sys_mode == SYS_MODE_IDLE)
                        {
                            System_Mode_Switch(SYS_MODE_POS);
                        }
                    }
                    key1_was_pressed = 0;
                }
            }

            // KEY1长按检测（持续按下超过2秒）
            if(key1_was_pressed && (now - key1_press_start > 2000))
            {
                key1_was_pressed = 0;  // 防止重复触发
                key1_wait_release = 1;
                #if HOMING_ENABLED == 1
                if(sys_mode == SYS_MODE_HOMING)
                {
                    // 已在回零中，K1长按=取消回零→IDLE
                    System_Mode_Switch(SYS_MODE_IDLE);
                    UART_Log("[HOMING] Cancelled by KEY1 long press\r\n");
                }
                else
                {
                    System_Mode_Switch(SYS_MODE_HOMING);
                }
                #else
                UART_Log("[WARN] KEY1 long press ignored (homing disabled)\r\n");
                #endif
            }
        } // end if key1 mask

            // KEY2: 暂停/恢复（速度模式）或停止（位置模式）
        // 【功能扩展】增加长按（>2s）反转功�?- 2026-04-30
        //   短按: VEL模式 暂停/恢复 | POS模式 停止→IDLE
        //   长按: VEL模式 切换正反转（>2s�?
                        if(key_mask_ready && (now - power_on_tick > 1500))
        {
            static uint8_t key2_last_was_low = 0;
            static uint8_t key2_debounced = 0;   
            static uint32_t key2_press_tick = 0;
            static uint8_t key2_wait_release = 0;   // 非阻塞：等待K2松开标志
            
            // 等待K2松开（非阻塞替代之前�?while(HAL_Delay)�?
            if(key2_wait_release)
            {
                if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET)
                {
                    key2_wait_release = 0;
                    key2_last_was_low = 1;
                }
                goto end_key2;  // 跳过其余K2处理
            }
            
            // 安全检查：必须经历过一次低电平(0)才响应后续高电平(1)
            if(key2_state == GPIO_PIN_RESET)
            {
                key2_last_was_low = 1;
            }
            
            if(key2_last_was_low)
            {
                // ===== 按下检�?=====
                if(key2_state == GPIO_PIN_SET)
                {
                    if(key2_press_tick == 0)
                    {
                        key2_press_tick = now;   // 首次检测到按下，启动定时器
                        key2_debounced = 0;
                    }
                    
                    // ---- 长按检�?(> 2000ms) ----
                    if(key2_debounced && (now - key2_press_tick > 2000))
                    {
                        key2_press_tick = 0;
                        key2_debounced = 0;
                        key2_last_was_low = 0;   // 防连�?
                        
                        // 只在速度模式下支持反�?
                        if(sys_mode == SYS_MODE_VEL)
                        {
                            // 切换方向
                            current_dir = !current_dir;
                            Motor_Set_Dir(current_dir);
                            Motor_Set_PWM((uint16_t)vel_target);
                            Motor_Start();
                            
                            if(current_dir == MOTOR_DIR_FORWARD)
                            {
                                LED_Set(LED_GREEN);
                                UART_Log("[VEL] Direction -> FORWARD\r\n");
                            }
                            else
                            {
                                LED_Set(LED_BLUE);
                                UART_Log("[VEL] Direction -> BACKWARD\r\n");
                            }
                        }
                        
                        // 非阻塞：标记等待K2松开
                        key2_wait_release = 1;
                    }
                    
                    // ---- 50ms消抖完成 ----
                    if(!key2_debounced && (now - key2_press_tick > 50))
                    {
                        key2_debounced = 1;  // 消抖完成，等待后续判断长短按
                    }
                }
                // ===== 松开检测（短按触发时机�?====
                else
                {
                    if(key2_debounced && key2_press_tick != 0 && (now - key2_press_tick < 2000))
                    {
                        // 消抖后松开了，且时�?2s �?短按
                        key2_press_tick = 0;
                        key2_debounced = 0;
                        key2_last_was_low = 0;  // 防连�?
                        
                        if(sys_mode == SYS_MODE_POS)
                        {
                            System_Mode_Switch(SYS_MODE_IDLE);
                            UART_Log("[POS] Stopped by KEY2\r\n");
                        }
                        else if(sys_mode == SYS_MODE_IDLE)
                        {
                            System_Mode_Switch(SYS_MODE_POS);
                            UART_Log("[IDLE] -> POS by KEY2\r\n");
                        }
                        else if(sys_mode == SYS_MODE_VEL)
                        {
                            motor_pause_flag = !motor_pause_flag;
                            if(motor_pause_flag)
                            {
                                Motor_Stop();
                                LED_Set(LED_RED_FLASH);
                                UART_Log("[VEL] Paused\r\n");
                            }
                            else
                            {
                                Motor_Set_Dir(current_dir);
                                Motor_Set_PWM((uint16_t)vel_target);
                                Motor_Start();
                                LED_Set(LED_GREEN);
                                UART_Log("[VEL] Resumed\r\n");
                            }
                        }
                        
                        // 非阻塞：标记等待K2松开
                        key2_wait_release = 1;
                    }
                    else
                    {
                        // 消抖还没完成就松开了—抖动忽�?
                        key2_press_tick = 0;
                        key2_debounced = 0;
                    }
                }
            } // end if key2_last_was_low
            end_key2:;
        }  // end if(时间门槛)
        // ========== 限位保护（带消抖，防噪声误触发）==========
        #if LIMIT_ENABLED == 1
        static uint8_t limit_debounce = 0;
        static uint8_t limit_last = 0;
        static uint8_t limit_reversed = 0;  // 已反转等待限位松开标志
        uint8_t limit_raw = (HAL_GPIO_ReadPin(LIMIT_PORT, RIGHT_LIMIT_PIN) == LIMIT_ACTIVE) ? 1 : 0;
        if(limit_raw == limit_last) {
            if(limit_debounce < 50) limit_debounce++;
        } else {
            limit_debounce = 0;
            limit_last = limit_raw;
        }
        uint8_t limit_stable = (limit_debounce >= 50 && limit_last == 1) ? 1 : 0;
        
        // 限位已松开 �?清除反转标志
        if(!limit_stable && limit_reversed) {
            limit_reversed = 0;
        }
        
        if(key_mask_ready && (now - power_on_tick > 2000) && limit_stable && sys_mode != SYS_MODE_IDLE && sys_mode != SYS_MODE_HOMING)
        {
            // VEL模式下向限位方向(BACKWARD)撞限 �?自动反转，不切模�?
            if(sys_mode == SYS_MODE_VEL && current_dir == MOTOR_DIR_BACKWARD)
            {
                current_dir = MOTOR_DIR_FORWARD;
                Motor_Set_Dir(current_dir);
                Motor_Set_PWM((uint16_t)vel_target);
                Motor_Start();
                limit_reversed = 1;
                UART_Log("[LIMIT] 起点限位触发! 反转至FORWARD\r\n");
            }
            // 已反转等待限位松开期间，忽略后续限位触�?
            else if(!limit_reversed)
            {
                System_Mode_Switch(SYS_MODE_IDLE); // POS模式撞限位→停止
                UART_Log("[LIMIT] 限位触发! 系统停止\r\n");
            }
        }
        #else
        // 【限位功能关闭】如果不使用限位开关，强制不处理任�?PB0 状�?
        #endif
        
        // ========== 模式执行 ==========
        switch(sys_mode)
        {
            case SYS_MODE_HOMING:
                #if HOMING_ENABLED == 1
                Homing_Loop();
                if(Homing_Get_State() == HOMING_FINISHED)
                {
                    current_pos = 0;
                    homing_done_once = 1;  // 标记已回零，启用软限�?
                    UART_Log("[HOMING] 回零完成! 位置已清零\r\n");
                    System_Mode_Switch(SYS_MODE_IDLE);
                }
                else if(Homing_Get_State() == HOMING_ERROR)
                {
                    Motor_Stop();
                    UART_Log("[HOMING] 回零超时/失败! 强制退出\r\n");
                    System_Mode_Switch(SYS_MODE_IDLE);
                }
                #else
                // 如果在没有启用的情况下进来了，强制退�?
                Motor_Stop();
                System_Mode_Switch(SYS_MODE_IDLE);
                #endif
                break;

            case SYS_MODE_IDLE:
                // IDLE 模式下，强制保持电机停止，防止失�?
                Motor_Stop();
                motor_pause_flag = 0;
                break;

            case SYS_MODE_VEL:
                if(!motor_pause_flag)
                {
                    Vel_Control_Loop();
                }
                break;

            case SYS_MODE_POS:
                // Position_Control_Loop() 已移至 TIM4 1kHz 抢占式硬件中断中运行
                // 彻底解决主循环中由于串口打印（HAL_UART_Transmit 阻塞 ~8.7ms）造成的控制周期抖动问题！
                break;

            default:
                break;
        }
        
        // ========== 软件限位（仅回零后生效，VEL模式下自动反转）==========
        if(homing_done_once && sys_mode == SYS_MODE_VEL && !motor_pause_flag)
        {
            current_pos = Mag_Get_Pos();
            if(current_pos < SOFT_LIMIT_FORWARD && current_dir == MOTOR_DIR_FORWARD)
            {
                // 反转方向：从FORWARD(远离限位)切到BACKWARD(回程)
                current_dir = MOTOR_DIR_BACKWARD;
                Motor_Set_Dir(current_dir);
                LED_Set(LED_BLUE);
                UART_Log("[LIMIT] 软件限位触发! Pos:%ld -> 自动反转\r\n", (long)current_pos);
            }
        }
        
        // ========== 500ms状态打�?==========
        print_timer++;
        if(print_timer >= 500)
        {
            print_timer = 0;
            current_pos = Mag_Get_Pos();
            
            const char* mode_str = "IDLE";
            switch(sys_mode)
            {
                case SYS_MODE_IDLE:    mode_str = "IDLE";    break;
                case SYS_MODE_VEL:     
                    mode_str = motor_pause_flag ? "VEL(PAUSE)" : "VEL";
                    break;
                case SYS_MODE_POS:     mode_str = "POS";     break;
                case SYS_MODE_HOMING:  mode_str = "HOMING";  break;
            }
            
            uint8_t pb0_raw = HAL_GPIO_ReadPin(LIMIT_PORT, RIGHT_LIMIT_PIN);
            UART_Log("[STATUS] Mode:%s | Pos:%ld(%.2fmm) | Spd:%u | Z:%u | K1:%d K2:%d | LMT:%d\r\n",
                    mode_str,
                    (long)current_pos, current_pos / PULSE_PER_MM, (unsigned)g_current_speed_cmd,
                    (unsigned int)z_pulse_count,
                    key1_state, key2_state, pb0_raw);
        }
        
        #if AUTO_TEST_ENABLED
        AutoTest_Control();
        #endif
        
        HAL_Delay(1);
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line); */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
