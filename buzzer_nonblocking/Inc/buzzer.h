#ifndef __BUZZER_H
#define __BUZZER_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ==================== 硬件配置 ==================== */
/* 默认：PA8，高电平响 */
#define BUZZER_GPIO_PORT              GPIOA
#define BUZZER_GPIO_PIN               GPIO_PIN_8
#define BUZZER_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()

#define BUZZER_ACTIVE_LEVEL           GPIO_PIN_SET
#define BUZZER_INACTIVE_LEVEL         GPIO_PIN_RESET

/* ==================== 预定义提示音 ==================== */
typedef enum
{
    BUZZER_SOUND_KEY = 0,
    BUZZER_SOUND_OK,
    BUZZER_SOUND_WARNING,
    BUZZER_SOUND_ERROR,
    BUZZER_SOUND_STARTUP
} Buzzer_Sound_t;

/* 一个声音步骤：响声时间 + 静音时间 */
typedef struct
{
    uint16_t on_time_ms;
    uint16_t off_time_ms;
} Buzzer_Step_t;

/* 初始化 */
void Buzzer_Init(void);

/* 持续响，必须手动关闭 */
void Buzzer_On(void);

/* 关闭并终止当前播放 */
void Buzzer_Off(void);

/* 终止当前所有蜂鸣器动作 */
void Buzzer_Stop(void);

/* 非阻塞响指定时间 */
void Buzzer_Beep(uint32_t duration_ms);

/* 播放预定义提示音 */
void Buzzer_PlaySound(Buzzer_Sound_t sound);

/*
 * 播放自定义声音序列。
 *
 * steps必须是static数组或全局数组。
 * repeat_count：
 *   1 = 播放一次
 *   2 = 播放两次
 *   0 = 无限循环
 */
bool Buzzer_PlayPattern(const Buzzer_Step_t *steps,
                        uint8_t step_count,
                        uint8_t repeat_count);

/* 必须在while(1)中持续调用 */
void Buzzer_Task(void);

/* 查询是否正在播放 */
bool Buzzer_IsBusy(void);

#endif
