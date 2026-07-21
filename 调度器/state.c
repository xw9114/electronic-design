#include "state.h"
#include "main.h"      /* HAL_GetTick() */
#include <stdio.h>     /* printf() */

/* =============================
 * 全局变量定义
 * ============================= */
TaskState_t g_task_state = TASK_WAIT;
TaskErrorCode_t g_error_code = ERROR_NONE;
uint32_t g_state_enter_time = 0U;

uint8_t g_start_flag = 0U;
uint8_t g_reset_flag = 0U;
uint8_t g_stop_point_flag = 0U;
uint8_t g_car_stable = 0U;
uint8_t g_target_valid = 0U;
uint8_t g_aim_finished = 0U;
uint8_t g_action_finished = 0U;

uint8_t g_emergency_stop = 0U;
uint8_t g_low_voltage = 0U;
uint8_t g_imu_online = 1U;
uint8_t g_vision_online = 1U;
uint8_t g_motor_stall = 0U;
uint8_t g_gimbal_limit = 0U;

uint32_t g_line_lost_duration_ms = 0U;

/* =========================================================
 * 函数1：状态分配与任务运行
 * ========================================================= */
void TaskFsm_Run(void)
{
    switch (g_task_state)
    {
        case TASK_WAIT:
        {
            /* 放：保持停止、等待启动 */

            if (TaskFsm_CheckError() == 1U)
            {
                TaskFsm_EnterError();
                return;
            }

            if (g_start_flag == 1U)
            {
                g_start_flag = 0U;
                TaskFsm_ChangeState(TASK_TRACK);
                return;
            }

            break;
        }

        case TASK_TRACK:
        {
            /* 放：小车寻迹、停车点检测、丢线时间统计 */

            if (TaskFsm_CheckError() == 1U)
            {
                TaskFsm_EnterError();
                return;
            }

            if (g_stop_point_flag == 1U)
            {
                g_stop_point_flag = 0U;
                TaskFsm_ChangeState(TASK_BRAKE);
                return;
            }

            break;
        }

        case TASK_BRAKE:
        {
            /* 放：停车控制、停稳判断 */

            if (TaskFsm_CheckError() == 1U)
            {
                TaskFsm_EnterError();
                return;
            }

            if (g_car_stable == 1U)
            {
                g_car_stable = 0U;
                TaskFsm_ChangeState(TASK_SEARCH);
                return;
            }

            break;
        }

        case TASK_SEARCH:
        {
            /* 放：云台搜索、读取视觉识别结果 */

            if (TaskFsm_CheckError() == 1U)
            {
                TaskFsm_EnterError();
                return;
            }

            if (g_target_valid == 1U)
            {
                TaskFsm_ChangeState(TASK_AIM);
                return;
            }

            break;
        }

        case TASK_AIM:
        {
            /* 放：视觉闭环瞄准、稳定判断 */

            if (TaskFsm_CheckError() == 1U)
            {
                TaskFsm_EnterError();
                return;
            }

            if (g_aim_finished == 1U)
            {
                g_aim_finished = 0U;
                TaskFsm_ChangeState(TASK_EXECUTE);
                return;
            }

            break;
        }

        case TASK_EXECUTE:
        {
            /* 放：画圆、抓取、发射或其他执行动作 */

            if (TaskFsm_CheckError() == 1U)
            {
                TaskFsm_EnterError();
                return;
            }

            if (g_action_finished == 1U)
            {
                g_action_finished = 0U;
                TaskFsm_ChangeState(TASK_FINISH);
                return;
            }

            break;
        }

        case TASK_FINISH:
        {
            /* 放：保持停止、显示任务完成 */

            if (g_start_flag == 1U)
            {
                g_start_flag = 0U;
                TaskFsm_ChangeState(TASK_WAIT);
                return;
            }

            break;
        }

        case TASK_ERROR:
        {
            /* 放：持续保持安全、等待人工复位 */

            if (g_reset_flag == 1U)
            {
                g_reset_flag = 0U;
                g_error_code = ERROR_NONE;
                TaskFsm_ChangeState(TASK_WAIT);
                return;
            }

            break;
        }

        default:
        {
            g_error_code = ERROR_INVALID_STATE;
            TaskFsm_EnterError();
            return;
        }
    }
}

/* =========================================================
 * 函数2：改变状态
 * ========================================================= */
void TaskFsm_ChangeState(TaskState_t new_state)
{
    g_task_state = new_state;
    g_state_enter_time = HAL_GetTick();

    switch (new_state)
    {
        case TASK_WAIT:
        {
            /* 停止小车、停止云台、清除等待状态临时量 */
            break;
        }

        case TASK_TRACK:
        {
            /* 清零寻迹 PID、清零连续丢线时间 */
            g_line_lost_duration_ms = 0U;
            break;
        }

        case TASK_BRAKE:
        {
            /* 下达停车命令、清零停稳计数 */
            g_car_stable = 0U;
            break;
        }

        case TASK_SEARCH:
        {
            /* 设置初始搜索方向、清零搜索参数 */
            g_target_valid = 0U;
            break;
        }

        case TASK_AIM:
        {
            /* 清零云台 PID、清零瞄准稳定计数 */
            g_aim_finished = 0U;
            break;
        }

        case TASK_EXECUTE:
        {
            /* 启动执行动作、清除上一次完成标志 */
            g_action_finished = 0U;
            break;
        }

        case TASK_FINISH:
        {
            /* 停止全部运动、打开完成指示灯 */
            break;
        }

        case TASK_ERROR:
        {
            /* 立即停止全部运动、关闭激光、打开错误指示灯 */
            break;
        }

        default:
        {
            break;
        }
    }
}

/* =========================================================
 * 函数3：检查错误
 *
 * 只负责：
 * 1. 判断发生了什么错误
 * 2. 把错误类型写入 g_error_code
 * 3. 返回 1
 *
 * 不负责打印，也不负责切换状态。
 * ========================================================= */
uint8_t TaskFsm_CheckError(void)
{
    uint32_t state_time;

    state_time = HAL_GetTick() - g_state_enter_time;
    g_error_code = ERROR_NONE;

    /* 所有状态都检查的错误 */
    if (g_emergency_stop == 1U)
    {
        g_error_code = ERROR_EMERGENCY_STOP;
        return 1U;
    }

    if (g_low_voltage == 1U)
    {
        g_error_code = ERROR_LOW_VOLTAGE;
        return 1U;
    }

    if (g_motor_stall == 1U)
    {
        g_error_code = ERROR_MOTOR_STALL;
        return 1U;
    }

    if (g_gimbal_limit == 1U)
    {
        g_error_code = ERROR_GIMBAL_LIMIT;
        return 1U;
    }

    /* 根据当前任务检查不同错误 */
    switch (g_task_state)
    {
        case TASK_WAIT:
        {
            break;
        }

        case TASK_TRACK:
        {
            if (g_imu_online == 0U)
            {
                g_error_code = ERROR_IMU_OFFLINE;
                return 1U;
            }

            if (g_line_lost_duration_ms > LINE_LOST_TIMEOUT_MS)
            {
                g_error_code = ERROR_LINE_LOST_TIMEOUT;
                return 1U;
            }

            break;
        }

        case TASK_BRAKE:
        {
            if (g_imu_online == 0U)
            {
                g_error_code = ERROR_IMU_OFFLINE;
                return 1U;
            }

            if (state_time > BRAKE_TIMEOUT_MS)
            {
                g_error_code = ERROR_BRAKE_TIMEOUT;
                return 1U;
            }

            break;
        }

        case TASK_SEARCH:
        {
            if (g_vision_online == 0U)
            {
                g_error_code = ERROR_VISION_OFFLINE;
                return 1U;
            }

            if (state_time > SEARCH_TIMEOUT_MS)
            {
                g_error_code = ERROR_SEARCH_TIMEOUT;
                return 1U;
            }

            break;
        }

        case TASK_AIM:
        {
            if (g_vision_online == 0U)
            {
                g_error_code = ERROR_VISION_OFFLINE;
                return 1U;
            }

            if (state_time > AIM_TIMEOUT_MS)
            {
                g_error_code = ERROR_AIM_TIMEOUT;
                return 1U;
            }

            break;
        }

        case TASK_EXECUTE:
        {
            if (state_time > ACTION_TIMEOUT_MS)
            {
                g_error_code = ERROR_ACTION_TIMEOUT;
                return 1U;
            }

            break;
        }

        case TASK_FINISH:
        case TASK_ERROR:
        {
            break;
        }

        default:
        {
            g_error_code = ERROR_INVALID_STATE;
            return 1U;
        }
    }

    return 0U;
}

/* =========================================================
 * 函数4：打印错误并进入错误状态
 * ========================================================= */
void TaskFsm_EnterError(void)
{
    uint32_t state_time;

    state_time = HAL_GetTick() - g_state_enter_time;

    printf("\r\n========== FSM ERROR ==========\r\n");

    printf("State: ");

    switch (g_task_state)
    {
        case TASK_WAIT:    printf("WAIT\r\n");    break;
        case TASK_TRACK:   printf("TRACK\r\n");   break;
        case TASK_BRAKE:   printf("BRAKE\r\n");   break;
        case TASK_SEARCH:  printf("SEARCH\r\n");  break;
        case TASK_AIM:     printf("AIM\r\n");     break;
        case TASK_EXECUTE: printf("EXECUTE\r\n"); break;
        case TASK_FINISH:  printf("FINISH\r\n");  break;
        case TASK_ERROR:   printf("ERROR\r\n");   break;
        default:           printf("UNKNOWN STATE\r\n"); break;
    }

    printf("Error: ");

    switch (g_error_code)
    {
        case ERROR_NONE:              printf("NONE\r\n");              break;
        case ERROR_EMERGENCY_STOP:    printf("EMERGENCY STOP\r\n");    break;
        case ERROR_LOW_VOLTAGE:       printf("LOW VOLTAGE\r\n");       break;
        case ERROR_MOTOR_STALL:       printf("MOTOR STALL\r\n");       break;
        case ERROR_GIMBAL_LIMIT:      printf("GIMBAL LIMIT\r\n");      break;
        case ERROR_IMU_OFFLINE:       printf("IMU OFFLINE\r\n");       break;
        case ERROR_VISION_OFFLINE:    printf("VISION OFFLINE\r\n");    break;
        case ERROR_LINE_LOST_TIMEOUT: printf("LINE LOST TIMEOUT\r\n"); break;
        case ERROR_BRAKE_TIMEOUT:     printf("BRAKE TIMEOUT\r\n");     break;
        case ERROR_SEARCH_TIMEOUT:    printf("SEARCH TIMEOUT\r\n");    break;
        case ERROR_AIM_TIMEOUT:       printf("AIM TIMEOUT\r\n");       break;
        case ERROR_ACTION_TIMEOUT:    printf("ACTION TIMEOUT\r\n");    break;
        case ERROR_INVALID_STATE:     printf("INVALID STATE\r\n");     break;
        default:                      printf("UNKNOWN ERROR\r\n");     break;
    }

    printf("State time: %lu ms\r\n", (unsigned long)state_time);
    printf("===============================\r\n");

    /* 必须最后切换，否则无法打印错误发生时的原状态 */
    TaskFsm_ChangeState(TASK_ERROR);
}
