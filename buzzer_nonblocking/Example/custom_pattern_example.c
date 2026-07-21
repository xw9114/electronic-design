#include "buzzer.h"

/*
 * 自定义：响100ms，停100ms。
 * 整个步骤重复3次。
 *
 * 必须定义为static或全局数组，
 * 不能使用播放函数返回后失效的局部数组。
 */
static const Buzzer_Step_t g_my_sound[] =
{
    {100, 100}
};

void Example_PlayCustomSound(void)
{
    (void)Buzzer_PlayPattern(
        g_my_sound,
        (uint8_t)(sizeof(g_my_sound) / sizeof(g_my_sound[0])),
        3
    );
}
