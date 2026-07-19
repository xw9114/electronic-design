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

## TaskFsm_Run 状态机模板

下面是任务状态机的文字抽象模板，适合电赛小车这类“等待启动 → 寻迹 → 停车 → 搜索目标 → 瞄准目标 → 执行动作 → 任务完成 / 错误处理”的流程。

### 完整抽象模板

```c
void TaskFsm_Run(void)
{
    switch（当前状态）
    {
        case 等待启动状态:
        {
            当前状态需要反复执行的任务

            if（统一错误检查发现错误）
            {
                return;
            }

            if（检测到启动条件）
            {
                清除启动标志

                切换到寻迹状态

                return;
            }

            break;
        }

        case 小车寻迹状态:
        {
            当前状态需要反复执行的任务

            if（统一错误检查发现错误）
            {
                return;
            }

            if（检测到停车点）
            {
                清除停车点标志

                切换到停车状态

                return;
            }

            break;
        }

        case 停车状态:
        {
            当前状态需要反复执行的任务

            if（统一错误检查发现错误）
            {
                return;
            }

            if（小车已经稳定停止）
            {
                切换到搜索目标状态

                return;
            }

            break;
        }

        case 搜索目标状态:
        {
            当前状态需要反复执行的任务

            if（统一错误检查发现错误）
            {
                return;
            }

            if（已经识别到目标）
            {
                切换到瞄准目标状态

                return;
            }

            break;
        }

        case 瞄准目标状态:
        {
            当前状态需要反复执行的任务

            if（统一错误检查发现错误）
            {
                return;
            }

            if（已经稳定瞄准目标）
            {
                清除瞄准完成标志

                切换到执行动作状态

                return;
            }

            break;
        }

        case 执行动作状态:
        {
            当前状态需要反复执行的任务

            if（统一错误检查发现错误）
            {
                return;
            }

            if（动作已经执行完成）
            {
                清除动作完成标志

                切换到任务完成状态

                return;
            }

            break;
        }

        case 任务完成状态:
        {
            当前状态需要反复执行的任务

            if（检测到重新启动条件）
            {
                清除启动标志

                切换到等待启动状态

                return;
            }

            break;
        }

        case 错误状态:
        {
            当前状态需要反复执行的安全任务

            if（检测到错误复位条件）
            {
                清除错误代码和复位标志

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

### 每个 `case` 的统一结构

除了完成状态和错误状态，普通任务状态基本都可以写成：

```c
case 当前任务状态:
{
    当前任务反复执行

    if（统一错误检查发现错误）
    {
        return;
    }

    if（当前任务正常完成）
    {
        清除对应标志位

        切换到下一个状态

        return;
    }

    break;
}
```

## 仓库维护原则

- 每个驱动尽量保持独立 `.c/.h`，降低移植成本；
- 驱动层不直接绑定具体 HAL；
- 不在驱动层写死串口外设；
- 不默认写传感器 Flash 配置；
- README 中只记录已经存在的驱动，未实现内容标记为后续计划。
