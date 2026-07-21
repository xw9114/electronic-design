/*
 * main_example.c
 *
 * 把需要的内容复制到你的main.c。
 */

#include "main.h"
#include "buzzer.h"

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    /*
     * 如果CubeMX已经初始化其他GPIO，
     * 可以保留MX_GPIO_Init()。
     */
    MX_GPIO_Init();

    Buzzer_Init();

    /* 非阻塞播放开机提示音 */
    Buzzer_PlaySound(BUZZER_SOUND_STARTUP);

    while (1)
    {
        /*
         * 必须高频调用。
         * 函数内部没有HAL_Delay()。
         */
        Buzzer_Task();

        /*
         * 其他任务可以同时执行：
         *
         * TaskFsm_Run();
         * Motor_Task();
         * Vision_Task();
         * IMU_Task();
         */
    }
}
