#include "main.h"
#include "state.h"

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_USART1_UART_Init();
    MX_TIM1_Init();

    /* 其他模块初始化：电机、编码器、惯导、视觉、云台、PID…… */

    TaskFsm_ChangeState(TASK_WAIT);

    while (1)
    {
        /*
         * 先持续更新全局数据，例如：
         * Key_Update();
         * Imu_Update();
         * Encoder_Update();
         * Vision_ReceiveAndParse();
         * Battery_Update();
         * MotorFault_Update();
         * GimbalLimit_Update();
         *
         * 对应模块更新 state.h 中声明的全局变量。
         */

        TaskFsm_Run();
    }
}
