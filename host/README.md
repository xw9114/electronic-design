# 远程实时调参脚本

`tune_console.py` 面向 STM32H0 + Horco `DAPLINK-WIRELESS / ESP32-S3`，实现设计书中的：

- TCP/Wi-Fi 连接；
- USB-TTL 串口直连测试；
- 18 字节 PID 参数下行帧；
- 20 字节遥测帧接收与校验；
- `revision` 参数生效确认；
- 误差驱动的自适应步长；
- CSV 遥测记录。

脚本只依赖 Python 标准库。使用 `--serial-port` 时需要额外安装 `pyserial`。

## 1. TCP/Wi-Fi 连接

先确认电脑和 DAPLINK-WIRELESS 在同一网络，并从模块配置或路由器获得实际 TCP 端口。`dsp_link` 可以替换成模块 IP，也可以是能解析到模块的主机名：

```powershell
python host\tune_console.py --tcp dsp_link <TCP_PORT> --loop-id 1
```

例如：

```powershell
python host\tune_console.py --tcp 192.168.4.1 5000 --loop-id 1
```

脚本默认只连接和接收遥测，不会自动改变 STM32 参数。显式发送初始值：

```powershell
python host\tune_console.py --tcp dsp_link <TCP_PORT> `
  --loop-id 1 --kp 10.0 --ki 0.0 --kd 0.15 --aux 0 --send-initial
```

如果模块工作在 TCP 客户端模式，应将 `--tcp` 的目标改为模块实际连接的服务器地址；脚本本身不假定模块的监听/连接方向。

## 2. USB-TTL 直连

先不经过无线模块，用 3.3 V TTL USB 转串口验证 STM32 固件和协议：

```powershell
python host\tune_console.py --serial-port COM8 --baudrate 115200 --loop-id 1
```

接线必须是：无线模块或 USB-TTL `TX -> STM32 UART_RX`、`RX -> STM32 UART_TX`、`GND -> GND`。不要把 RS-232 电平直接接到 STM32H0。

## 3. 交互命令

连接后在 `tune>` 提示符输入：

```text
select 1
set kp 10.0
set kd 0.15
set aux 0
send
step kp up
step kd down
show
stats
quit
```

`step` 根据最近一帧遥测的 `abs(error)` 选择步长：大于 20 使用 1.0，5 到 20 使用 0.2，小于 5 使用 0.05。没有遥测时使用粗调步长。

## 4. 遥测记录

```powershell
python host\tune_console.py --tcp dsp_link <TCP_PORT> --log logs\run.csv
```

脚本会记录所有通过校验的遥测帧。看到返回遥测中的 `revision` 发生变化，才说明 MCU 已在控制周期边界提交新参数。

## 5. 调试顺序

1. 先用 `--serial-port` 验证 STM32H0 的 UART 和协议；
2. 确认能收到 20 字节遥测帧；
3. 发送一组低风险参数，确认 `revision` 增加；
4. 再切换到 `--tcp dsp_link <TCP_PORT>`；
5. 最后进行悬空电机、低速和整车调参。

如果脚本能连接但始终没有遥测，优先检查 STM32 是否实现了 `AA FE` 上行帧、TX/RX 是否交叉、DAPLINK 的 UART 透传模式和实际 TCP 端口。
