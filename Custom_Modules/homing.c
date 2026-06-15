#include "homing.h"
#include "motor.h"
#include "encoder.h"
#include "main.h"

/* 回零参数：位置单位为磁栅尺计数，1 count = 1 um */
#define HOMING_FAST_SPEED        200U     /* speed_cmd=200 -> about 800 STEP/s */
#define HOMING_SLOW_SPEED        100U     /* speed_cmd=100 -> about 400 STEP/s */
#define HOMING_BACK_DIST         5000L    /* 退离限位 5 mm */
#define HOMING_OFFSET_DIST       2000L    /* 进入Z相精定位前，向前退离限位 2 mm */
#define HOMING_RELEASE_MAX_DIST  8000L    /* 退离 8 mm 仍未释放限位则报错 */
#define HOMING_MAX_TRAVEL        120000L  /* 找限位最大允许行程 120 mm */
/* 无时间超时保护，由 HOMING_MAX_TRAVEL(120mm行程上限) 和 HOMING_RELEASE_MAX_DIST(8mm限位释放超时) 兜底 */
#define LIMIT_FILTER_COUNT       50U

typedef enum {
    HOMING_PHASE_IDLE,
    HOMING_PHASE_FAST,
    HOMING_PHASE_BACK,
    HOMING_PHASE_SLOW,
    HOMING_PHASE_OFFSET,
    HOMING_PHASE_Z_FINE,
    HOMING_PHASE_DONE,
    HOMING_PHASE_ERROR
} HomingPhase;

static HomingPhase homing_phase = HOMING_PHASE_IDLE;
static uint8_t homing_done = HOMING_UNFINISHED;
static int32_t start_pos = 0;
static int32_t homing_origin_pos = 0;
static uint8_t limit_debounce = 0;
static uint8_t limit_stable = 0;

extern volatile uint32_t z_pulse_count;
extern int32_t z_last_pos;  // Z脉冲位置记录，跨文件共享（由stm32f1xx_it.c维护，homing/Z_FINE阶段修改）

static uint8_t Homing_Read_Limit_Raw(void)
{
    return (HAL_GPIO_ReadPin(LIMIT_PORT, RIGHT_LIMIT_PIN) == GPIO_PIN_SET) ? 1U : 0U;
}

static int32_t Homing_Abs_Diff(int32_t a, int32_t b)
{
    return (a > b) ? (a - b) : (b - a);
}

static void Homing_Reset_Limit_Filter(void)
{
    limit_stable = Homing_Read_Limit_Raw();
    limit_debounce = 0;
}

static void Homing_Update_Limit_Filter(void)
{
    uint8_t limit_raw = Homing_Read_Limit_Raw();

    if(limit_raw == limit_stable)
    {
        limit_debounce = 0;
    }
    else
    {
        if(limit_debounce < LIMIT_FILTER_COUNT)
        {
            limit_debounce++;
        }
        else
        {
            limit_stable = limit_raw;
            limit_debounce = 0;
        }
    }
}

static void Homing_Fail(void)
{
    Motor_Stop();
    homing_phase = HOMING_PHASE_ERROR;
    homing_done = HOMING_ERROR;
}

void Homing_Start(void)
{
    int32_t current_pos = Mag_Get_Pos();

    Motor_Stop();
    Homing_Reset_Limit_Filter();

    homing_done = HOMING_UNFINISHED;
    start_pos = current_pos;
    homing_origin_pos = current_pos;

    if(limit_stable)
    {
        homing_phase = HOMING_PHASE_BACK;
        Motor_Set_Dir(MOTOR_DIR_FORWARD);
    }
    else
    {
        homing_phase = HOMING_PHASE_FAST;
        Motor_Set_Dir(MOTOR_DIR_BACKWARD);
    }
}

uint8_t Homing_Get_State(void)
{
    return homing_done;
}

void Homing_Loop(void)
{
    int32_t current_pos;
    int32_t moved;

    if(homing_phase == HOMING_PHASE_IDLE ||
       homing_phase == HOMING_PHASE_DONE ||
       homing_phase == HOMING_PHASE_ERROR)
    {
        return;
    }

    current_pos = Mag_Get_Pos();
    Homing_Update_Limit_Filter();

    // 行程保护：超出HOMING_MAX_TRAVEL(120mm)则报错
    if(Homing_Abs_Diff(current_pos, homing_origin_pos) > HOMING_MAX_TRAVEL)
    {
        Homing_Fail();
        return;
    }

    switch(homing_phase)
    {
        case HOMING_PHASE_FAST:
            if(limit_stable)
            {
                Motor_Stop();
                start_pos = current_pos;
                homing_phase = HOMING_PHASE_BACK;
                Motor_Set_Dir(MOTOR_DIR_FORWARD);
                break;
            }

            Motor_Set_Dir(MOTOR_DIR_BACKWARD);
            Motor_Set_PWM(HOMING_FAST_SPEED);
            Motor_Start();
            break;

        case HOMING_PHASE_BACK:
            Motor_Set_Dir(MOTOR_DIR_FORWARD);
            Motor_Set_PWM(HOMING_SLOW_SPEED);
            Motor_Start();

            moved = Homing_Abs_Diff(current_pos, start_pos);
            if(moved > HOMING_BACK_DIST && !limit_stable)
            {
                Motor_Stop();
                start_pos = current_pos;
                homing_phase = HOMING_PHASE_SLOW;
                Motor_Set_Dir(MOTOR_DIR_BACKWARD);
            }
            else if(moved > HOMING_RELEASE_MAX_DIST && limit_stable)
            {
                Homing_Fail();
            }
            break;

        case HOMING_PHASE_SLOW:
            if(limit_stable)
            {
                Motor_Stop();
                start_pos = current_pos;
                homing_phase = HOMING_PHASE_OFFSET;
                Motor_Set_Dir(MOTOR_DIR_FORWARD);
                break;
            }

            Motor_Set_Dir(MOTOR_DIR_BACKWARD);
            Motor_Set_PWM(HOMING_SLOW_SPEED);
            Motor_Start();
            break;

        case HOMING_PHASE_OFFSET:
            Motor_Set_Dir(MOTOR_DIR_FORWARD);
            Motor_Set_PWM(HOMING_SLOW_SPEED);
            Motor_Start();

            moved = Homing_Abs_Diff(current_pos, start_pos);
            
            if(moved > HOMING_OFFSET_DIST && !limit_stable)
            {
                Motor_Stop();
                // OFFSET完成，进入Z相精定位阶段
                homing_phase = HOMING_PHASE_Z_FINE;
                start_pos = current_pos;
                z_pulse_count = 0;
                z_last_pos = current_pos - 5000;  // 设置偏移值，确保OFST阶段经过的位置不会阻挡首个Z脉冲（≥1800校验）
                Motor_Set_Dir(MOTOR_DIR_BACKWARD);
            }
            else if(moved > HOMING_RELEASE_MAX_DIST && limit_stable)
            {
                Homing_Fail();
            }
            break;

        case HOMING_PHASE_Z_FINE:
            // 警告：Z_FINE向限位反向慢速搜索，由于OFFSET只退2mm，若Z脉冲丢失，会在2mm处物理接触限位开关并堵转
            Motor_Set_Dir(MOTOR_DIR_BACKWARD);
            Motor_Set_PWM(HOMING_Z_FINE_SPEED);
            Motor_Start();

            moved = Homing_Abs_Diff(current_pos, start_pos);

            if(z_pulse_count >= 1)
            {
                // 捕获到Z脉冲：当前位置即物理零点
                Motor_Stop();
                Mag_Reset_Pos();
                homing_phase = HOMING_PHASE_DONE;
                homing_done = HOMING_FINISHED;
            }
            else if(moved > HOMING_Z_FINE_MAX_DIST)
            {
                // 超距未找到Z脉冲：降级回退，按OFFSET方案归零
                Motor_Stop();
                Mag_Reset_Pos();
                z_pulse_count = 0;
                homing_phase = HOMING_PHASE_DONE;
                homing_done = HOMING_FINISHED;
            }
            break;

        case HOMING_PHASE_IDLE:
        case HOMING_PHASE_DONE:
        case HOMING_PHASE_ERROR:
        default:
            break;
    }
}
