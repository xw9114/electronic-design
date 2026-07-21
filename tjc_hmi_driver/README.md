# 淘晶驰 TJC 串口屏通用驱动

适用于淘晶驰 T1/X 系列常见字符串指令协议。

## 文件

- `tjc_hmi.h`：接口和事件定义
- `tjc_hmi.c`：发送指令、接收分帧、返回数据解析
- `tjc_stm32_hal_example.c`：STM32 HAL 示例
- `tjc_port_template.c`：其他单片机移植模板

## 通信原则

1. MCU TX 接屏幕 RX。
2. MCU RX 接屏幕 TX。
3. MCU 与屏幕共地。
4. UART 参数必须与屏幕工程一致。
5. 每条发送给屏幕的指令后面必须跟 `FF FF FF`。

## USART HMI 工程准备

先在 USART HMI 上位机中建立页面和控件，例如：

- 文本控件：`t0`
- 数字控件：`n0`
- 进度条：`j0`
- 按钮：`b0`

编译并下载到屏幕。

若希望点击按钮后 MCU 收到 `0x65` 事件，需要在按钮的按下事件或弹起事件中勾选“发送键值”。

## 最小调用

```c
TJC_SetText(&g_hmi, "t0", "Ready");
TJC_SetValue(&g_hmi, "n0", 123);
TJC_SetValue(&g_hmi, "j0", 75);
TJC_SetPageByName(&g_hmi, "main");
```

这些函数实际发送：

```text
t0.txt="Ready" FF FF FF
n0.val=123 FF FF FF
j0.val=75 FF FF FF
page main FF FF FF
```

## 中文显示

屏幕工程需要提前建立包含相应汉字的字库。MCU 发送字符串的编码还必须与屏幕工程编码一致。初次调试建议先发送英文和数字，确认串口链路没有问题后再处理中文编码。

## 常见错误码

- `00 FF FF FF`：无效指令
- `02 FF FF FF`：控件 ID 无效
- `03 FF FF FF`：页面 ID 无效
- `1A FF FF FF`：变量名称无效
- `1C FF FF FF`：赋值失败
- `24 FF FF FF`：串口缓冲区溢出
