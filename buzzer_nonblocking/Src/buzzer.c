#include "buzzer.h"

typedef enum
{
    BUZZER_STATE_IDLE = 0,
    BUZZER_STATE_MANUAL_ON,
    BUZZER_STATE_PATTERN_ON,
    BUZZER_STATE_PATTERN_OFF
} Buzzer_State_t;

static Buzzer_State_t g_buzzer_state = BUZZER_STATE_IDLE;

static const Buzzer_Step_t *g_buzzer_steps = NULL;
static uint8_t g_buzzer_step_count = 0;
static uint8_t g_buzzer_step_index = 0;
static uint8_t g_buzzer_repeat_count = 0;
static uint8_t g_buzzer_repeat_finished = 0;
static uint32_t g_buzzer_deadline = 0;

static Buzzer_Step_t g_single_beep_step;

/* ==================== 预定义声音 ==================== */

static const Buzzer_Step_t g_sound_key[] =
{
    {50, 0}
};

static const Buzzer_Step_t g_sound_ok[] =
{
    {80, 80},
    {80, 0}
};

static const Buzzer_Step_t g_sound_warning[] =
{
    {150, 120},
    {150, 120},
    {150, 0}
};

static const Buzzer_Step_t g_sound_error[] =
{
    {500, 200},
    {500, 0}
};

static const Buzzer_Step_t g_sound_startup[] =
{
    {100, 100},
    {100, 100},
    {300, 0}
};

/* ==================== 内部函数 ==================== */

static void Buzzer_WriteOutput(bool enable)
{
    HAL_GPIO_WritePin(
        BUZZER_GPIO_PORT,
        BUZZER_GPIO_PIN,
        enable ? BUZZER_ACTIVE_LEVEL : BUZZER_INACTIVE_LEVEL
    );
}

/* 支持HAL_GetTick()溢出 */
static bool Buzzer_TimeReached(uint32_t now, uint32_t target_time)
{
    return ((int32_t)(now - target_time) >= 0);
}

static void Buzzer_StartCurrentStep(uint32_t now)
{
    const Buzzer_Step_t *current_step;

    if ((g_buzzer_steps == NULL) ||
        (g_buzzer_step_count == 0) ||
        (g_buzzer_step_index >= g_buzzer_step_count))
    {
        Buzzer_Stop();
        return;
    }

    current_step = &g_buzzer_steps[g_buzzer_step_index];

    if (current_step->on_time_ms > 0)
    {
        Buzzer_WriteOutput(true);
        g_buzzer_state = BUZZER_STATE_PATTERN_ON;
        g_buzzer_deadline = now + current_step->on_time_ms;
    }
    else if (current_step->off_time_ms > 0)
    {
        Buzzer_WriteOutput(false);
        g_buzzer_state = BUZZER_STATE_PATTERN_OFF;
        g_buzzer_deadline = now + current_step->off_time_ms;
    }
    else
    {
        Buzzer_Stop();
    }
}

static void Buzzer_GotoNextStep(uint32_t now)
{
    g_buzzer_step_index++;

    if (g_buzzer_step_index >= g_buzzer_step_count)
    {
        g_buzzer_step_index = 0;

        /* repeat_count为0表示无限循环 */
        if (g_buzzer_repeat_count != 0)
        {
            g_buzzer_repeat_finished++;

            if (g_buzzer_repeat_finished >= g_buzzer_repeat_count)
            {
                Buzzer_Stop();
                return;
            }
        }
    }

    Buzzer_StartCurrentStep(now);
}

/* ==================== 对外接口 ==================== */

void Buzzer_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    BUZZER_GPIO_CLK_ENABLE();

    /*
     * 初始化GPIO前先写关闭电平，
     * 尽量避免上电瞬间误响。
     */
    Buzzer_WriteOutput(false);

    GPIO_InitStruct.Pin = BUZZER_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(BUZZER_GPIO_PORT, &GPIO_InitStruct);

    Buzzer_Stop();
}

void Buzzer_On(void)
{
    g_buzzer_steps = NULL;
    g_buzzer_step_count = 0;
    g_buzzer_step_index = 0;
    g_buzzer_repeat_count = 0;
    g_buzzer_repeat_finished = 0;

    g_buzzer_state = BUZZER_STATE_MANUAL_ON;
    Buzzer_WriteOutput(true);
}

void Buzzer_Off(void)
{
    Buzzer_Stop();
}

void Buzzer_Stop(void)
{
    Buzzer_WriteOutput(false);

    g_buzzer_state = BUZZER_STATE_IDLE;
    g_buzzer_steps = NULL;
    g_buzzer_step_count = 0;
    g_buzzer_step_index = 0;
    g_buzzer_repeat_count = 0;
    g_buzzer_repeat_finished = 0;
    g_buzzer_deadline = 0;
}

void Buzzer_Beep(uint32_t duration_ms)
{
    if (duration_ms == 0)
    {
        Buzzer_Stop();
        return;
    }

    /*
     * Buzzer_Step_t使用uint16_t保存时间，
     * 最大单次响声时间为65535ms。
     */
    if (duration_ms > 65535U)
    {
        duration_ms = 65535U;
    }

    g_single_beep_step.on_time_ms = (uint16_t)duration_ms;
    g_single_beep_step.off_time_ms = 0;

    (void)Buzzer_PlayPattern(&g_single_beep_step, 1, 1);
}

void Buzzer_PlaySound(Buzzer_Sound_t sound)
{
    switch (sound)
    {
        case BUZZER_SOUND_KEY:
            (void)Buzzer_PlayPattern(
                g_sound_key,
                (uint8_t)(sizeof(g_sound_key) / sizeof(g_sound_key[0])),
                1
            );
            break;

        case BUZZER_SOUND_OK:
            (void)Buzzer_PlayPattern(
                g_sound_ok,
                (uint8_t)(sizeof(g_sound_ok) / sizeof(g_sound_ok[0])),
                1
            );
            break;

        case BUZZER_SOUND_WARNING:
            (void)Buzzer_PlayPattern(
                g_sound_warning,
                (uint8_t)(sizeof(g_sound_warning) / sizeof(g_sound_warning[0])),
                1
            );
            break;

        case BUZZER_SOUND_ERROR:
            (void)Buzzer_PlayPattern(
                g_sound_error,
                (uint8_t)(sizeof(g_sound_error) / sizeof(g_sound_error[0])),
                1
            );
            break;

        case BUZZER_SOUND_STARTUP:
            (void)Buzzer_PlayPattern(
                g_sound_startup,
                (uint8_t)(sizeof(g_sound_startup) / sizeof(g_sound_startup[0])),
                1
            );
            break;

        default:
            Buzzer_Stop();
            break;
    }
}

bool Buzzer_PlayPattern(const Buzzer_Step_t *steps,
                        uint8_t step_count,
                        uint8_t repeat_count)
{
    uint8_t i;
    uint32_t now;

    if ((steps == NULL) || (step_count == 0))
    {
        return false;
    }

    for (i = 0; i < step_count; i++)
    {
        if ((steps[i].on_time_ms == 0) &&
            (steps[i].off_time_ms == 0))
        {
            return false;
        }
    }

    /* 新提示音覆盖旧提示音 */
    Buzzer_WriteOutput(false);

    g_buzzer_steps = steps;
    g_buzzer_step_count = step_count;
    g_buzzer_step_index = 0;
    g_buzzer_repeat_count = repeat_count;
    g_buzzer_repeat_finished = 0;

    now = HAL_GetTick();
    Buzzer_StartCurrentStep(now);

    return true;
}

void Buzzer_Task(void)
{
    uint32_t now;
    const Buzzer_Step_t *current_step;

    now = HAL_GetTick();

    switch (g_buzzer_state)
    {
        case BUZZER_STATE_IDLE:
            break;

        case BUZZER_STATE_MANUAL_ON:
            break;

        case BUZZER_STATE_PATTERN_ON:
            if (!Buzzer_TimeReached(now, g_buzzer_deadline))
            {
                break;
            }

            Buzzer_WriteOutput(false);

            if ((g_buzzer_steps == NULL) ||
                (g_buzzer_step_index >= g_buzzer_step_count))
            {
                Buzzer_Stop();
                break;
            }

            current_step = &g_buzzer_steps[g_buzzer_step_index];

            if (current_step->off_time_ms > 0)
            {
                g_buzzer_state = BUZZER_STATE_PATTERN_OFF;
                g_buzzer_deadline = now + current_step->off_time_ms;
            }
            else
            {
                Buzzer_GotoNextStep(now);
            }
            break;

        case BUZZER_STATE_PATTERN_OFF:
            if (!Buzzer_TimeReached(now, g_buzzer_deadline))
            {
                break;
            }

            Buzzer_GotoNextStep(now);
            break;

        default:
            Buzzer_Stop();
            break;
    }
}

bool Buzzer_IsBusy(void)
{
    return (g_buzzer_state != BUZZER_STATE_IDLE);
}
