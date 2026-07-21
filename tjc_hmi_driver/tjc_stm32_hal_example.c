/*
 * STM32 HAL 接入示例
 *
 * 假设 CubeMX 已经生成 huart1。
 * 将 tjc_hmi.c / tjc_hmi.h 加入工程后使用。
 */

#include "main.h"
#include "tjc_hmi.h"

extern UART_HandleTypeDef huart1;

static TJC_Handle g_hmi;
static uint8_t g_hmi_rx_byte;

static bool HMI_UART_Write(const uint8_t *data, uint16_t length)
{
    return HAL_UART_Transmit(&huart1,
                             (uint8_t *)data,
                             length,
                             100U) == HAL_OK;
}

static void HMI_EventCallback(const TJC_Event *event)
{
    switch (event->type)
    {
        case TJC_EVENT_TOUCH:
            /*
             * 例如：
             * page 0 中 b0 的控件 ID 为 1，
             * 且收到“弹起”事件。
             */
            if ((event->page_id == 0U) &&
                (event->component_id == 1U) &&
                (event->touch_event == 0U))
            {
                /* 在这里执行按钮功能 */
            }
            break;

        case TJC_EVENT_NUMBER:
            /* event->number 是 get n0.val 的返回值 */
            break;

        case TJC_EVENT_STRING:
            /*
             * event->data 不是以 '\0' 结尾的 C 字符串。
             * 需要保存时，请按 event->data_length 复制。
             */
            break;

        case TJC_EVENT_ERROR:
            /* event->code 是屏幕返回的错误码 */
            break;

        default:
            break;
    }
}

void HMI_Init(void)
{
    TJC_Init(&g_hmi, HMI_UART_Write, HMI_EventCallback);

    /* 启动单字节中断接收 */
    HAL_UART_Receive_IT(&huart1, &g_hmi_rx_byte, 1U);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == huart1.Instance)
    {
        TJC_RxByte(&g_hmi, g_hmi_rx_byte);
        HAL_UART_Receive_IT(&huart1, &g_hmi_rx_byte, 1U);
    }
}

void HMI_Demo(void)
{
    TJC_SetPageByName(&g_hmi, "main");

    TJC_SetText(&g_hmi, "t0", "Ready");
    TJC_SetValue(&g_hmi, "n0", 123);
    TJC_SetValue(&g_hmi, "j0", 75);

    TJC_SetVisible(&g_hmi, "b0", true);
    TJC_SetTouchEnabled(&g_hmi, "b0", true);

    /* 请求屏幕返回控件值 */
    TJC_GetValue(&g_hmi, "n0");
}
