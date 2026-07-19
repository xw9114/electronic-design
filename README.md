# 电赛测控驱动库

面向电子设计竞赛测控类项目的嵌入式驱动库，当前收录 **维特智能 JY61P 姿态传感器 UART 驱动与姿态读取模块**。

本仓库的目标是把比赛中常用、可复用的传感器驱动整理成独立 `.c/.h` 文件，方便直接移植到 STM32、GD32、MSPM0、ESP32 等 MCU 工程。

## 当前驱动目录

| 驱动 | 文件 | 状态 | 适用场景 |
|---|---|---|---|
| JY61P 姿态传感器 | [`jy61p_attitude.c`](./jy61p_attitude.c) / [`jy61p_attitude.h`](./jy61p_attitude.h) | 可用 | 小车航向保持、原地转角、姿态反馈、控制器微分项输入 |

| 树莓派视觉控制脚本 | [`树莓派_1.py`](./树莓派_1.py) | 可用 | 树莓派 USB 摄像头、A4 黑框与圆心跟踪、Emm42 双电机控制 |
> 后续如果继续加入编码器、超声波、TOF、气压计、通信模块等驱动，建议按同样的“一组 `.c/.h` + README 表格入口”方式维护。
## 树莓派视觉控制脚本

[`树莓派_1.py`](./树莓派_1.py) 是一个基于 Raspberry Pi 的视觉跟踪与双电机控制程序，主要功能包括：

- 使用 USB 摄像头读取 `/dev/video0` 视频；
- 通过 OpenCV 检测 A4 黑色矩形框并进行透视变换；
- 在透视图中检测目标圆心并计算跟踪误差；
- 通过 `/dev/serial0` 使用 Emm42 V5.0 串口协议控制两个电机；
- 可选开启 Flask 网页预览，默认地址为 `http://xw.local:5000`。

### 安装依赖

```bash
sudo apt update
sudo apt install -y python3-opencv python3-numpy python3-serial python3-flask
```

### 运行

```bash
python3 树莓派_1.py
```

使用 GPIO 串口时请确保为 3.3V TTL 电平；如果电机驱动器为 RS-232/RS-485 电平，需先使用对应的电平转换模块。

## JY61P 驱动特性

当前 JY61P 模块基于维特智能 WIT 标准串口协议实现：

| 帧类型 | 含义 |
|---|---|
| `0x51` | 加速度 + 温度 |
| `0x52` | 角速度 |
| `0x53` | 欧拉角 |
| `0x59` | 四元数 |

姿态输出策略：

- **Roll**：直接使用模块输出角度；
- **Pitch**：直接使用模块输出角度；
- **Yaw**：进行跨 `±180°` 连续化，并支持软件清零；
- **Z 轴角速度**：继续输出给控制器的微分项使用。

驱动特性：

- C99 实现；
- 无动态内存；
- 不依赖具体 HAL 或 RTOS；
- UART 字节流输入；
- 内置帧头同步与校验；
- 支持启动陀螺零偏标定；
- 支持静止状态下在线零偏微调；
- 支持数据有效性与丢失判断。

## 快速接入

把下面两个文件复制到你的 MCU 工程中：

```text
jy61p_attitude.c
jy61p_attitude.h
```

业务代码中包含头文件：

```c
#include "jy61p_attitude.h"
```

### 初始化

```c
static Jy61pAttitude imu;

void app_init(void)
{
    JY61P_Init(&imu, 100.0f);
    JY61P_StartCalibration(&imu, millis(), 1000);
}
```

说明：

- `100.0f` 是 JY61P 的标称输出频率，按传感器实际配置修改；
- `millis()` 是用户工程中的毫秒计时函数；
- `1000` 表示上电后静止标定 1000ms。

### 串口接收

单字节接收：

```c
void uart_rx_callback(uint8_t byte)
{
    JY61P_InputByte(&imu, byte, millis());
}
```

DMA 或缓冲区批量接收：

```c
JY61P_InputBytes(&imu, rx_buffer, rx_len, millis());
```

### 控制循环读取

```c
void control_loop(void)
{
    if (!JY61P_IsValid(&imu, millis())) {
        // 数据超时：建议停车、降级或保持上一安全状态
        return;
    }

    float roll = JY61P_GetRollDeg(&imu);
    float pitch = JY61P_GetPitchDeg(&imu);
    float yaw = JY61P_GetYawDeg(&imu);
    float yaw_rate = JY61P_GetYawRateDps(&imu);

    float heading_error = JY61P_Wrap180(target_yaw - yaw);
    // turn_cmd = kp * heading_error - kd * yaw_rate;
}
```

## 常用 API

| API | 说明 |
|---|---|
| `JY61P_Init` | 初始化驱动状态 |
| `JY61P_StartCalibration` | 启动陀螺零偏标定 |
| `JY61P_InputByte` | 输入一个 UART 字节 |
| `JY61P_InputBytes` | 输入一段 UART 数据 |
| `JY61P_IsValid` | 判断姿态数据是否仍在有效时间内 |
| `JY61P_IsLost` | 判断姿态数据是否长时间丢失 |
| `JY61P_SetYawZero` | 将当前航向设为软件零点 |
| `JY61P_SetYawZeroValue` | 将当前航向设为指定角度 |
| `JY61P_GetRollDeg` | 获取 Roll 角 |
| `JY61P_GetPitchDeg` | 获取 Pitch 角 |
| `JY61P_GetYawDeg` | 获取连续 Yaw，已扣软件零点 |
| `JY61P_GetYawWrappedDeg` | 获取限制在 `[-180°, 180°]` 的 Yaw |
| `JY61P_GetYawRateDps` | 获取 Z 轴角速度 |
| `JY61P_GetAccG` | 获取三轴加速度 |
| `JY61P_GetGyroDps` | 获取三轴角速度 |
| `JY61P_GetQuaternion` | 获取四元数 |
| `JY61P_GetTemperatureC` | 获取温度 |
| `JY61P_GetFrameCount` | 获取已接收有效帧数量 |
| `JY61P_GetBadFrameCount` | 获取校验失败帧数量 |

## 可调参数

参数位于 [`jy61p_attitude.h`](./jy61p_attitude.h)，可在工程编译选项中提前定义同名宏覆盖默认值。

| 宏 | 默认值 | 说明 |
|---|---:|---|
| `JY61P_ACC_RANGE_G` | `16.0f` | 加速度量程 |
| `JY61P_GYRO_RANGE_DPS` | `2000.0f` | 角速度量程 |
| `JY61P_ANGLE_RANGE_DEG` | `180.0f` | 欧拉角量程 |
| `JY61P_VALID_TIMEOUT_MS` | `50u` | 姿态数据有效超时 |
| `JY61P_LOST_TIMEOUT_MS` | `200u` | 姿态数据丢失超时 |
| `JY61P_CAL_GYRO_ACCEPT_DPS` | `5.0f` | 启动标定时可接受的角速度阈值 |
| `JY61P_STATIONARY_GYRO_MAX_DPS` | `1.0f` | 静止在线零偏更新角速度阈值 |
| `JY61P_STATIONARY_ACC_TOL_G` | `0.10f` | 静止在线零偏更新加速度模长容差 |

如果你用维特上位机修改过 JY61P 的量程，需要同步修改 `JY61P_ACC_RANGE_G` 和 `JY61P_GYRO_RANGE_DPS`。

## 电赛使用建议

### 航向保持

```c
float yaw = JY61P_GetYawDeg(&imu);
float yaw_rate = JY61P_GetYawRateDps(&imu);
float error = JY61P_Wrap180(target_yaw - yaw);

// yaw_rate 可作为控制器 D 项输入
// turn_cmd = kp * error - kd * yaw_rate;
```

### 原地转角

使用连续 Yaw，不要直接使用限制在 `[-180°, 180°]` 的角度：

```c
float start_yaw = JY61P_GetYawDeg(&imu);
float target_yaw = start_yaw + 90.0f;
```

### 推荐启动流程

```text
上电静止
  ↓
初始化串口
  ↓
JY61P_Init()
  ↓
JY61P_StartCalibration()
  ↓
等待 JY61P_IsCalibrating() 返回 false
  ↓
JY61P_SetYawZero()
  ↓
进入正常控制
```

## 编译检查

桌面环境可用 GCC 做语法检查：

```bash
gcc -std=c99 -Wall -Wextra -Werror -c jy61p_attitude.c
```

如果链接阶段提示 `sqrtf` 或 `fabsf` 未定义，GCC 工程通常需要链接数学库：

```bash
-lm
```

嵌入式 IDE 中请确认启用 C99，并链接标准数学库。

## TaskFsm_Run 任务状态机完整逻辑模板

下面是任务状态机的完整抽象模板，用于整理电赛小车等嵌入式项目的主循环分层、状态枚举、信号结构、错误检查和状态切换逻辑。

### 一、整个程序的总体结构

```c
int main(void)
{
    单片机基础初始化

    各个硬件外设初始化

    各个软件控制模块初始化

    初始化任务状态机

    while (1)
    {
        全局传感器数据读取

        全局通信数据接收与解析

        全局设备在线状态检查

        全局安全状态检查

        TaskFsm_Run();
    }
}
```

整体分成两层：

```text
while(1) 外层
负责持续获得最新数据

TaskFsm_Run()
负责根据当前任务状态，选择需要执行的控制任务
```

---

### 二、`while(1)` 里面放什么

完整模板：

```c
while (1)
{
    /* 1. 全局读取传感器数据 */
    读取惯导数据
    读取编码器数据
    读取电池电压
    读取急停按键
    读取限位开关

    /* 2. 全局处理通信 */
    接收并解析视觉串口数据
    接收并解析电机驱动器数据
    接收并解析其他控制板数据

    /* 3. 更新全局状态 */
    更新惯导是否在线
    更新视觉是否在线
    更新电机是否在线
    更新小车当前速度
    更新云台当前位置

    /* 4. 运行任务状态机 */
    TaskFsm_Run();
}
```

#### 为什么这些内容放在 `while(1)` 外层

这些数据可能被多个状态使用。

例如惯导数据可能用于：

```text
寻迹时保持航向
转弯时控制角度
停车时判断车体稳定
云台瞄准时进行姿态补偿
```

因此惯导数据最好持续更新，而不是只在某个状态里读取。

类似地，视觉串口也应该持续接收，否则不使用视觉的状态持续时间较长时，串口缓冲区可能堆积数据。

---

### 三、全局读取和按状态执行的区别

#### 全局持续执行

放在 `while(1)` 中：

```text
传感器原始数据读取
串口数据接收
通信数据解析
设备在线状态判断
电池电压检测
急停检测
编码器采集
```

这些功能负责：

> 给状态机提供最新信息。

#### 按状态执行

放在 `TaskFsm_Run()` 对应的 `case` 中：

```text
小车寻迹控制
停车控制
云台搜索
视觉瞄准
定角转弯
画圆
执行机构动作
```

这些功能负责：

> 根据当前任务阶段控制设备运动。

所以推荐的分工是：

```text
while(1)：持续获取信息
TaskFsm_Run()：根据当前状态使用信息
```

---

### 四、任务状态枚举中放什么

任务状态枚举中放的是：

> 整个比赛任务可能处于的各个阶段。

模板：

```c
typedef enum
{
    系统初始化状态,

    等待启动状态,

    小车寻迹状态,

    小车停车状态,

    云台搜索目标状态,

    云台瞄准目标状态,

    执行动作状态,

    任务完成状态,

    错误状态

} TaskState_t;
```

用实际名称表示，可以写成：

```c
typedef enum
{
    TASK_STATE_INIT = 0,

    TASK_STATE_WAIT_START,

    TASK_STATE_TRACK_LINE,

    TASK_STATE_BRAKE,

    TASK_STATE_SEARCH_TARGET,

    TASK_STATE_AIM_TARGET,

    TASK_STATE_EXECUTE,

    TASK_STATE_FINISH,

    TASK_STATE_ERROR

} TaskState_t;
```

#### 枚举的作用

定义当前状态变量：

```c
TaskState_t task_state;
```

它一次只能保存一个状态：

```c
task_state = TASK_STATE_TRACK_LINE;
```

表示：

> 系统当前正在寻迹。

枚举回答的是：

```text
系统现在处于哪个任务阶段？
```

---

### 五、错误类型枚举中放什么

错误枚举保存各种可能出现的错误原因。

```c
typedef enum
{
    没有错误,

    惯导通信掉线,

    视觉通信掉线,

    电机通信掉线,

    小车长时间丢线,

    电机堵转,

    云台越过限位,

    停车状态超时,

    搜索目标超时,

    瞄准目标超时,

    执行动作超时,

    电池电压过低,

    状态机出现非法状态,

    未知错误

} ErrorCode_t;
```

实际代码可以写成：

```c
typedef enum
{
    ERROR_NONE = 0,

    ERROR_IMU_OFFLINE,
    ERROR_VISION_OFFLINE,
    ERROR_MOTOR_OFFLINE,

    ERROR_LINE_LOST,
    ERROR_MOTOR_STALL,
    ERROR_GIMBAL_LIMIT,

    ERROR_BRAKE_TIMEOUT,
    ERROR_SEARCH_TIMEOUT,
    ERROR_AIM_TIMEOUT,
    ERROR_ACTION_TIMEOUT,

    ERROR_LOW_VOLTAGE,

    ERROR_INVALID_STATE,
    ERROR_UNKNOWN

} ErrorCode_t;
```

定义当前错误：

```c
ErrorCode_t error_code;
```

例如：

```c
error_code = ERROR_VISION_OFFLINE;
```

表示当前错误原因是视觉通信掉线。

---

### 六、任务信号结构体中放什么

任务信号结构体保存：

> 各个传感器和控制模块产生的实时状态、事件标志和任务完成标志。

模板：

```c
typedef struct
{
    /* 按键事件 */
    启动事件;
    复位事件;
    急停状态;

    /* 小车相关 */
    是否检测到停车点;
    是否丢线;
    小车是否已经停稳;

    /* 视觉相关 */
    视觉是否在线;
    当前是否识别到目标;
    目标水平误差;
    目标垂直误差;

    /* 惯导相关 */
    惯导是否在线;
    当前航向角;
    当前角速度;

    /* 云台相关 */
    云台是否已经瞄准完成;
    云台是否到达限位;

    /* 执行机构相关 */
    动作是否完成;

    /* 电源相关 */
    电池电压是否过低;

} TaskSignals_t;
```

实际代码可以写成：

```c
typedef struct
{
    /* 按键与安全 */
    uint8_t start_event;
    uint8_t reset_event;
    uint8_t emergency_stop;

    /* 小车 */
    uint8_t stop_point_detected;
    uint8_t line_lost;
    uint8_t car_stable;

    /* 视觉 */
    uint8_t vision_online;
    uint8_t target_valid;
    int16_t target_error_x;
    int16_t target_error_y;

    /* 惯导 */
    uint8_t imu_online;
    float yaw;
    float gyro_z;

    /* 云台 */
    uint8_t aim_finished;
    uint8_t gimbal_limit;

    /* 执行机构 */
    uint8_t action_finished;

    /* 电源 */
    uint8_t low_voltage;

} TaskSignals_t;
```

定义全局变量：

```c
TaskSignals_t task_signals = {0};
```

---

### 七、状态机运行结构体中放什么

这个结构体保存状态机本身需要记住的数据。

```c
typedef struct
{
    当前任务状态;

    当前错误代码;

    进入当前状态的时间;

    搜索重试次数;

    瞄准稳定计数;

    目标丢失计数;

} TaskFsm_t;
```

实际写法：

```c
typedef struct
{
    TaskState_t state;

    ErrorCode_t error_code;

    uint32_t state_enter_time;

    uint8_t search_retry_count;

    uint16_t aim_stable_count;

    uint16_t target_lost_count;

} TaskFsm_t;
```

定义全局状态机变量：

```c
TaskFsm_t task_fsm = {0};
```

这个结构体回答的是：

```text
当前状态是什么？
什么时候进入这个状态？
当前错误是什么？
已经搜索了几次？
已经连续稳定了多少次？
```

---

### 八、三个类型之间的关系

#### `TaskState_t`

保存：

```text
系统当前处于哪个任务阶段
```

例如：

```c
task_fsm.state = TASK_STATE_TRACK_LINE;
```

#### `ErrorCode_t`

保存：

```text
系统发生了什么错误
```

例如：

```c
task_fsm.error_code = ERROR_IMU_OFFLINE;
```

#### `TaskSignals_t`

保存：

```text
传感器和模块当前检测到了什么
```

例如：

```c
task_signals.stop_point_detected = 1U;
task_signals.target_valid = 1U;
task_signals.imu_online = 1U;
```

状态机根据 `TaskSignals_t` 中的信息，修改 `TaskState_t`。

---

### 九、状态机初始化

在进入 `while(1)` 之前调用：

```c
任务状态机初始化
```

文字模板：

```c
void TaskFsm_Init(void)
{
    当前状态设置为系统初始化状态

    当前错误设置为没有错误

    记录当前时间

    清零搜索重试次数

    清零瞄准稳定计数

    清零目标丢失计数

    执行系统初始化状态的一次性任务
}
```

它只在上电时执行一次。

在 `main()` 中：

```c
int main(void)
{
    单片机基础初始化

    外设初始化

    控制模块初始化

    TaskFsm_Init();

    while (1)
    {
        全局数据更新

        TaskFsm_Run();
    }
}
```

---

### 十、状态切换函数的文字结构

```c
void TaskFsm_ChangeState(新状态)
{
    把当前状态修改成新状态

    记录进入新状态的时间

    执行新状态的一次性初始化
}
```

它负责三件事：

```text
1. 修改当前状态
2. 记录进入时间
3. 调用新状态的进入函数
```

调用过程：

```text
当前状态正常完成
        ↓
调用状态切换函数
        ↓
当前状态变成下一个状态
        ↓
重新记录进入时间
        ↓
执行新状态的一次性初始化
```

---

### 十一、状态进入函数的文字结构

```c
void TaskFsm_OnEnter(新状态)
{
    switch（新状态）
    {
        case 等待启动状态:
        {
            执行等待状态的一次性初始化

            break;
        }

        case 小车寻迹状态:
        {
            清零寻迹PID

            清除停车点事件

            break;
        }

        case 小车停车状态:
        {
            下达停车命令

            清零停车稳定计数

            break;
        }

        case 云台搜索状态:
        {
            清零搜索计数

            设置初始搜索方向

            启动云台搜索

            break;
        }

        case 云台瞄准状态:
        {
            清零云台PID

            清零瞄准稳定计数

            break;
        }

        case 执行动作状态:
        {
            启动执行动作

            清除动作完成事件

            break;
        }

        case 任务完成状态:
        {
            停止全部运动

            打开完成指示

            break;
        }

        case 错误状态:
        {
            停止全部运动

            关闭激光和执行机构

            打开错误指示

            break;
        }

        default:
        {
            break;
        }
    }
}
```

这个函数中的内容只在进入状态时执行一次。

---

### 十二、统一错误检查函数的结构

```c
统一错误检查函数
{
    根据当前状态检查对应错误

    检查全局严重错误

    检查当前状态是否超时

    if（发现错误）
    {
        记录错误原因

        切换到错误状态

        返回“发现错误”
    }

    返回“没有错误”
}
```

它可以检查：

```text
惯导掉线
视觉掉线
电机掉线
急停按键
低电压
电机堵转
云台越限
当前状态执行超时
```

返回含义：

```text
返回 1：已经进入错误状态
返回 0：没有错误
```

---

### 十三、完整的 `TaskFsm_Run()` 文字模板

```c
void TaskFsm_Run(void)
{
    switch（当前状态）
    {
        case 系统初始化状态:
        {
            if（统一错误检查发现错误）
            {
                return;
            }

            if（系统初始化完成）
            {
                切换到等待启动状态

                return;
            }

            break;
        }


        case 等待启动状态:
        {
            等待启动期间反复执行的任务

            if（统一错误检查发现错误）
            {
                return;
            }

            if（检测到启动事件）
            {
                清除启动事件

                切换到小车寻迹状态

                return;
            }

            break;
        }


        case 小车寻迹状态:
        {
            小车寻迹任务反复执行

            停车点检测任务反复执行

            if（统一错误检查发现错误）
            {
                return;
            }

            if（检测到停车点事件）
            {
                清除停车点事件

                切换到小车停车状态

                return;
            }

            break;
        }


        case 小车停车状态:
        {
            停车控制任务反复执行

            小车稳定判断任务反复执行

            if（统一错误检查发现错误）
            {
                return;
            }

            if（小车已经稳定停止）
            {
                切换到云台搜索目标状态

                return;
            }

            break;
        }


        case 云台搜索目标状态:
        {
            云台搜索任务反复执行

            if（统一错误检查发现错误）
            {
                return;
            }

            if（视觉已经识别到有效目标）
            {
                切换到云台瞄准目标状态

                return;
            }

            break;
        }


        case 云台瞄准目标状态:
        {
            视觉闭环瞄准任务反复执行

            瞄准稳定判断任务反复执行

            if（统一错误检查发现错误）
            {
                return;
            }

            if（已经稳定瞄准目标）
            {
                清除瞄准完成事件

                切换到执行动作状态

                return;
            }

            break;
        }


        case 执行动作状态:
        {
            动作执行任务反复执行

            if（统一错误检查发现错误）
            {
                return;
            }

            if（动作已经执行完成）
            {
                清除动作完成事件

                切换到任务完成状态

                return;
            }

            break;
        }


        case 任务完成状态:
        {
            任务完成后的保持任务反复执行

            if（检测到重新启动事件）
            {
                清除重新启动事件

                切换到等待启动状态

                return;
            }

            break;
        }


        case 错误状态:
        {
            错误状态下的安全任务反复执行

            错误显示任务反复执行

            if（检测到错误复位事件）
            {
                清除错误代码

                清除错误复位事件

                切换到等待启动状态

                return;
            }

            break;
        }


        default:
        {
            记录非法状态错误

            切换到错误状态

            return;
        }
    }
}
```

---

### 十四、完整主程序模板

```c
int main(void)
{
    /* 单片机和硬件初始化 */
    HAL基础初始化

    系统时钟初始化

    GPIO初始化

    串口初始化

    定时器初始化

    ADC初始化

    编码器初始化

    电机初始化

    云台初始化


    /* 软件控制模块初始化 */
    寻迹控制初始化

    小车速度PID初始化

    云台PID初始化

    视觉通信初始化

    惯导通信初始化

    执行动作模块初始化


    /* 状态机初始化 */
    TaskFsm_Init();


    while (1)
    {
        /* 全局传感器和通信数据更新 */
        读取惯导数据

        读取编码器数据

        接收并解析视觉数据

        读取电池电压

        读取限位开关

        读取启动、复位和急停按键


        /* 更新全局状态 */
        更新惯导在线标志

        更新视觉在线标志

        更新电机在线标志

        更新电池低电压标志

        更新急停标志


        /* 运行当前任务状态 */
        TaskFsm_Run();
    }
}
```

---

### 十五、完整程序的数据流

```text
传感器和通信模块
        ↓
更新 TaskSignals_t
        ↓
TaskFsm_Run() 读取这些信息
        ↓
根据 task_fsm.state 选择当前 case
        ↓
执行当前状态任务
        ↓
统一检查错误
        ↓
判断当前任务是否完成
        ↓
调用 TaskFsm_ChangeState()
        ↓
修改 task_fsm.state
        ↓
记录 state_enter_time
        ↓
调用 TaskFsm_OnEnter()
        ↓
执行新状态的一次性初始化
```

---

### 十六、最后的总体职责划分

| 部分                      | 作用                 |
| ----------------------- | ------------------ |
| `while(1)`              | 持续读取全局传感器、通信和安全数据  |
| `TaskSignals_t`         | 保存传感器状态、实时状态和任务事件  |
| `TaskState_t`           | 定义任务流程中所有状态        |
| `ErrorCode_t`           | 定义所有错误原因           |
| `TaskFsm_t`             | 保存当前状态、错误、进入时间和计数器 |
| `TaskFsm_Run()`         | 执行当前状态并判断是否退出      |
| 统一错误检查                  | 检查设备异常、通信异常和状态超时   |
| `TaskFsm_ChangeState()` | 修改状态、记录时间、调用进入动作   |
| `TaskFsm_OnEnter()`     | 执行新状态的一次性初始化       |

最简洁地概括：

```text
while(1) 负责更新数据
TaskFsm_Run() 负责执行任务
TaskFsm_CheckError() 负责发现错误
TaskFsm_ChangeState() 负责切换状态
TaskFsm_OnEnter() 负责进入状态时初始化一次
```

## 仓库维护原则

- 每个驱动尽量保持独立 `.c/.h`，降低移植成本；
- 驱动层不直接绑定具体 HAL；
- 不在驱动层写死串口外设；
- 不默认写传感器 Flash 配置；
- README 中只记录已经存在的驱动，未实现内容标记为后续计划。
