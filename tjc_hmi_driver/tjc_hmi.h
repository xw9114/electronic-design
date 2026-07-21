#ifndef TJC_HMI_H
#define TJC_HMI_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef TJC_RX_BUFFER_SIZE
#define TJC_RX_BUFFER_SIZE 128U
#endif

#ifndef TJC_TX_BUFFER_SIZE
#define TJC_TX_BUFFER_SIZE 128U
#endif

/*
 * 底层发送函数：
 * 将 data[0..length-1] 通过 UART 发出。
 * 发送成功返回 true，失败返回 false。
 */
typedef bool (*TJC_WriteFunction)(const uint8_t *data, uint16_t length);

typedef enum
{
    TJC_EVENT_NONE = 0,
    TJC_EVENT_TOUCH,       /* 0x65：控件点击 */
    TJC_EVENT_PAGE,        /* 0x66：当前页面 ID */
    TJC_EVENT_COORDINATE,  /* 0x67/0x68：触摸坐标 */
    TJC_EVENT_STRING,      /* 0x70：字符串返回 */
    TJC_EVENT_NUMBER,      /* 0x71：数值返回 */
    TJC_EVENT_STARTUP,     /* 0x88：屏幕启动成功 */
    TJC_EVENT_ACK,         /* 0x01：指令执行成功 */
    TJC_EVENT_ERROR,       /* 单字节错误码 */
    TJC_EVENT_STATUS,      /* 睡眠、唤醒、升级等状态 */
    TJC_EVENT_RAW          /* 未识别的数据包 */
} TJC_EventType;

typedef struct
{
    TJC_EventType type;

    uint8_t code;

    uint8_t page_id;
    uint8_t component_id;
    uint8_t touch_event; /* 0=弹起，1=按下 */

    uint16_t x;
    uint16_t y;

    int32_t number;

    /*
     * 仅在事件回调执行期间有效。
     * 若需要长期保存，请在回调里复制出去。
     */
    const uint8_t *data;
    uint16_t data_length;
} TJC_Event;

typedef void (*TJC_EventCallback)(const TJC_Event *event);

typedef struct
{
    TJC_WriteFunction write;
    TJC_EventCallback event_callback;

    uint8_t rx_buffer[TJC_RX_BUFFER_SIZE];
    uint16_t rx_length;
} TJC_Handle;

/* 初始化 */
void TJC_Init(TJC_Handle *handle,
              TJC_WriteFunction write_function,
              TJC_EventCallback event_callback);

/* 发送任意淘晶驰指令，函数会自动追加 FF FF FF */
bool TJC_SendCommand(TJC_Handle *handle, const char *command);

/* 常用控件操作 */
bool TJC_SetText(TJC_Handle *handle, const char *object, const char *text);
bool TJC_SetValue(TJC_Handle *handle, const char *object, int32_t value);
bool TJC_SetPageByName(TJC_Handle *handle, const char *page_name);
bool TJC_SetPageById(TJC_Handle *handle, uint8_t page_id);
bool TJC_SetVisible(TJC_Handle *handle, const char *object, bool visible);
bool TJC_SetTouchEnabled(TJC_Handle *handle, const char *object, bool enabled);
bool TJC_Click(TJC_Handle *handle, const char *object, bool press);
bool TJC_GetValue(TJC_Handle *handle, const char *object);
bool TJC_GetText(TJC_Handle *handle, const char *object);
bool TJC_GetCurrentPage(TJC_Handle *handle);
bool TJC_Reset(TJC_Handle *handle);

/*
 * 接收处理：
 * 在 UART 接收中断、DMA 空闲中断或轮询代码中，把收到的字节逐个送入。
 */
void TJC_RxByte(TJC_Handle *handle, uint8_t byte);
void TJC_RxData(TJC_Handle *handle, const uint8_t *data, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif
