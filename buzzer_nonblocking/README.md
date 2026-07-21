# STM32 非阻塞有源蜂鸣器驱动

## 文件结构

```text
buzzer_nonblocking/
├─ Inc/
│  └─ buzzer.h
├─ Src/
│  └─ buzzer.c
└─ Example/
   ├─ main_example.c
   └─ custom_pattern_example.c
```

## 默认硬件配置

- 蜂鸣器类型：有源蜂鸣器
- GPIO：PA8
- 高电平响
- STM32 HAL库
- 使用 `HAL_GetTick()` 计时

在 `Inc/buzzer.h` 中修改以下宏即可更换引脚：

```c
#define BUZZER_GPIO_PORT              GPIOA
#define BUZZER_GPIO_PIN               GPIO_PIN_8
#define BUZZER_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
```

如果你的蜂鸣器是低电平响，修改为：

```c
#define BUZZER_ACTIVE_LEVEL           GPIO_PIN_RESET
#define BUZZER_INACTIVE_LEVEL         GPIO_PIN_SET
```

## 加入工程

1. 将 `Inc/buzzer.h` 放入工程头文件目录。
2. 将 `Src/buzzer.c` 放入工程源文件目录。
3. 在 Keil 中将 `buzzer.c` 添加到工程。
4. 在 `main.c` 中：

```c
#include "buzzer.h"
```

初始化：

```c
Buzzer_Init();
```

主循环持续调用：

```c
while (1)
{
    Buzzer_Task();
}
```

## 常用调用

非阻塞响200ms：

```c
Buzzer_Beep(200);
```

成功提示：

```c
Buzzer_PlaySound(BUZZER_SOUND_OK);
```

错误提示：

```c
Buzzer_PlaySound(BUZZER_SOUND_ERROR);
```

持续响：

```c
Buzzer_On();
```

关闭：

```c
Buzzer_Off();
```

查询状态：

```c
if (!Buzzer_IsBusy())
{
    /* 已播放结束 */
}
```

## 注意

不要在主循环中不断重复调用：

```c
Buzzer_PlaySound(BUZZER_SOUND_ERROR);
```

否则提示音每次都会重新开始。应在进入某个状态的瞬间调用一次。
