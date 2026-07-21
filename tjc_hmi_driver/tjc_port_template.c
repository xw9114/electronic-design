/*
 * 其他单片机接入模板，例如 MSPM0、51、GD32、ESP32。
 *
 * 只需要完成：
 * 1. 底层 UART 发送函数；
 * 2. UART 接收中断中调用 TJC_RxByte()。
 */

#include "tjc_hmi.h"

static TJC_Handle g_hmi;

static bool Board_UART_Write(const uint8_t *data, uint16_t length)
{
    /*
     * 替换为你的芯片 UART 发送代码。
     *
     * for (uint16_t i = 0; i < length; ++i)
     * {
     *     UART_SendByte(data[i]);
     * }
     */
    (void)data;
    (void)length;
    return true;
}

static void HMI_EventCallback(const TJC_Event *event)
{
    if (event->type == TJC_EVENT_TOUCH)
    {
        /* 根据 page_id、component_id、touch_event 执行动作 */
    }
}

void HMI_Init(void)
{
    TJC_Init(&g_hmi, Board_UART_Write, HMI_EventCallback);
}

/* 放进实际 UART RX 中断 */
void Board_UART_RxInterruptHandler(void)
{
    uint8_t received_byte = 0U; /* 替换为读取 UART RX 寄存器 */
    TJC_RxByte(&g_hmi, received_byte);
}
