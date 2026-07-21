#ifndef __STATE_H
#define __STATE_H

#include <stdint.h>

/* 任务状态 */
typedef enum
{
    TASK_WAIT = 0,
    TASK_TRACK,
    TASK_BRAKE,
    TASK_SEARCH,
    TASK_AIM,
    TASK_EXECUTE,
    TASK_FINISH,
    TASK_ERROR
} TaskState_t;

/* 错误类型 */
typedef enum
{
    ERROR_NONE = 0,

    ERROR_EMERGENCY_STOP,
    ERROR_LOW_VOLTAGE,
    ERROR_MOTOR_STALL,
    ERROR_GIMBAL_LIMIT,

    ERROR_IMU_OFFLINE,
    ERROR_VISION_OFFLINE,

    ERROR_LINE_LOST_TIMEOUT,
    ERROR_BRAKE_TIMEOUT,
    ERROR_SEARCH_TIMEOUT,
    ERROR_AIM_TIMEOUT,
    ERROR_ACTION_TIMEOUT,

    ERROR_INVALID_STATE
} TaskErrorCode_t;

/* 超时时间，可按实际项目修改 */
#define LINE_LOST_TIMEOUT_MS    2000U
#define BRAKE_TIMEOUT_MS        2000U
#define SEARCH_TIMEOUT_MS       5000U
#define AIM_TIMEOUT_MS          3000U
#define ACTION_TIMEOUT_MS      10000U

/* 状态机全局变量 */
extern TaskState_t g_task_state;
extern TaskErrorCode_t g_error_code;
extern uint32_t g_state_enter_time;

/* 正常任务事件 */
extern uint8_t g_start_flag;
extern uint8_t g_reset_flag;
extern uint8_t g_stop_point_flag;
extern uint8_t g_car_stable;
extern uint8_t g_target_valid;
extern uint8_t g_aim_finished;
extern uint8_t g_action_finished;

/* 全局异常信号 */
extern uint8_t g_emergency_stop;
extern uint8_t g_low_voltage;
extern uint8_t g_imu_online;
extern uint8_t g_vision_online;
extern uint8_t g_motor_stall;
extern uint8_t g_gimbal_limit;

/* 寻迹模块统计的连续丢线时间 */
extern uint32_t g_line_lost_duration_ms;

/* 四个核心函数 */
void TaskFsm_Run(void);
void TaskFsm_ChangeState(TaskState_t new_state);
uint8_t TaskFsm_CheckError(void);
void TaskFsm_EnterError(void);

#endif /* __STATE_H */
