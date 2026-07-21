#include "tjc_hmi.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static const uint8_t g_tjc_end_bytes[3] = {0xFFU, 0xFFU, 0xFFU};

static bool TJC_SendFormat(TJC_Handle *handle, const char *format, ...)
{
    char command[TJC_TX_BUFFER_SIZE];
    va_list args;
    int length;

    if ((handle == NULL) || (format == NULL))
    {
        return false;
    }

    va_start(args, format);
    length = vsnprintf(command, sizeof(command), format, args);
    va_end(args);

    if ((length < 0) || ((size_t)length >= sizeof(command)))
    {
        return false;
    }

    return TJC_SendCommand(handle, command);
}

static void TJC_DispatchPacket(TJC_Handle *handle,
                               const uint8_t *packet,
                               uint16_t length)
{
    TJC_Event event;

    if ((handle == NULL) || (packet == NULL) || (length == 0U))
    {
        return;
    }

    memset(&event, 0, sizeof(event));
    event.type = TJC_EVENT_RAW;
    event.code = packet[0];
    event.data = packet;
    event.data_length = length;

    switch (packet[0])
    {
        case 0x65U:
            if (length >= 4U)
            {
                event.type = TJC_EVENT_TOUCH;
                event.page_id = packet[1];
                event.component_id = packet[2];
                event.touch_event = packet[3];
            }
            break;

        case 0x66U:
            if (length >= 2U)
            {
                event.type = TJC_EVENT_PAGE;
                event.page_id = packet[1];
            }
            break;

        case 0x67U:
        case 0x68U:
            if (length >= 6U)
            {
                event.type = TJC_EVENT_COORDINATE;
                event.x = (uint16_t)(((uint16_t)packet[1] << 8U) | packet[2]);
                event.y = (uint16_t)(((uint16_t)packet[3] << 8U) | packet[4]);
                event.touch_event = packet[5];
            }
            break;

        case 0x70U:
            event.type = TJC_EVENT_STRING;
            event.data = &packet[1];
            event.data_length = (uint16_t)(length - 1U);
            break;

        case 0x71U:
            if (length >= 5U)
            {
                event.type = TJC_EVENT_NUMBER;
                event.number = (int32_t)(
                    ((uint32_t)packet[1]) |
                    ((uint32_t)packet[2] << 8U) |
                    ((uint32_t)packet[3] << 16U) |
                    ((uint32_t)packet[4] << 24U));
            }
            break;

        case 0x88U:
            event.type = TJC_EVENT_STARTUP;
            break;

        case 0x01U:
            event.type = TJC_EVENT_ACK;
            break;

        case 0x86U:
        case 0x87U:
        case 0x89U:
        case 0xFDU:
        case 0xFEU:
            event.type = TJC_EVENT_STATUS;
            break;

        default:
            /*
             * 淘晶驰错误返回通常只有一个状态字节，
             * 后面紧跟 FF FF FF。
             */
            if (length == 1U)
            {
                event.type = TJC_EVENT_ERROR;
            }
            break;
    }

    if (handle->event_callback != NULL)
    {
        handle->event_callback(&event);
    }
}

void TJC_Init(TJC_Handle *handle,
              TJC_WriteFunction write_function,
              TJC_EventCallback event_callback)
{
    if (handle == NULL)
    {
        return;
    }

    memset(handle, 0, sizeof(*handle));
    handle->write = write_function;
    handle->event_callback = event_callback;
}

bool TJC_SendCommand(TJC_Handle *handle, const char *command)
{
    size_t command_length;

    if ((handle == NULL) || (handle->write == NULL) || (command == NULL))
    {
        return false;
    }

    command_length = strlen(command);

    if (command_length > UINT16_MAX)
    {
        return false;
    }

    if (!handle->write((const uint8_t *)command, (uint16_t)command_length))
    {
        return false;
    }

    return handle->write(g_tjc_end_bytes, sizeof(g_tjc_end_bytes));
}

bool TJC_SetText(TJC_Handle *handle, const char *object, const char *text)
{
    if ((object == NULL) || (text == NULL))
    {
        return false;
    }

    /*
     * text 中不要直接包含英文双引号。
     * 中文编码必须与屏幕工程使用的编码一致。
     */
    return TJC_SendFormat(handle, "%s.txt=\"%s\"", object, text);
}

bool TJC_SetValue(TJC_Handle *handle, const char *object, int32_t value)
{
    if (object == NULL)
    {
        return false;
    }

    return TJC_SendFormat(handle, "%s.val=%ld", object, (long)value);
}

bool TJC_SetPageByName(TJC_Handle *handle, const char *page_name)
{
    if (page_name == NULL)
    {
        return false;
    }

    return TJC_SendFormat(handle, "page %s", page_name);
}

bool TJC_SetPageById(TJC_Handle *handle, uint8_t page_id)
{
    return TJC_SendFormat(handle, "page %u", (unsigned int)page_id);
}

bool TJC_SetVisible(TJC_Handle *handle, const char *object, bool visible)
{
    if (object == NULL)
    {
        return false;
    }

    return TJC_SendFormat(handle, "vis %s,%u",
                          object,
                          visible ? 1U : 0U);
}

bool TJC_SetTouchEnabled(TJC_Handle *handle,
                         const char *object,
                         bool enabled)
{
    if (object == NULL)
    {
        return false;
    }

    return TJC_SendFormat(handle, "tsw %s,%u",
                          object,
                          enabled ? 1U : 0U);
}

bool TJC_Click(TJC_Handle *handle, const char *object, bool press)
{
    if (object == NULL)
    {
        return false;
    }

    return TJC_SendFormat(handle, "click %s,%u",
                          object,
                          press ? 1U : 0U);
}

bool TJC_GetValue(TJC_Handle *handle, const char *object)
{
    if (object == NULL)
    {
        return false;
    }

    return TJC_SendFormat(handle, "get %s.val", object);
}

bool TJC_GetText(TJC_Handle *handle, const char *object)
{
    if (object == NULL)
    {
        return false;
    }

    return TJC_SendFormat(handle, "get %s.txt", object);
}

bool TJC_GetCurrentPage(TJC_Handle *handle)
{
    return TJC_SendCommand(handle, "sendme");
}

bool TJC_Reset(TJC_Handle *handle)
{
    return TJC_SendCommand(handle, "rest");
}

void TJC_RxByte(TJC_Handle *handle, uint8_t byte)
{
    uint16_t packet_length;

    if (handle == NULL)
    {
        return;
    }

    if (handle->rx_length >= TJC_RX_BUFFER_SIZE)
    {
        /* 缓冲区溢出后丢弃当前帧，重新同步。 */
        handle->rx_length = 0U;
    }

    handle->rx_buffer[handle->rx_length++] = byte;

    if ((handle->rx_length >= 3U) &&
        (handle->rx_buffer[handle->rx_length - 1U] == 0xFFU) &&
        (handle->rx_buffer[handle->rx_length - 2U] == 0xFFU) &&
        (handle->rx_buffer[handle->rx_length - 3U] == 0xFFU))
    {
        packet_length = (uint16_t)(handle->rx_length - 3U);

        if (packet_length > 0U)
        {
            TJC_DispatchPacket(handle,
                               handle->rx_buffer,
                               packet_length);
        }

        handle->rx_length = 0U;
    }
}

void TJC_RxData(TJC_Handle *handle, const uint8_t *data, uint16_t length)
{
    uint16_t index;

    if ((handle == NULL) || (data == NULL))
    {
        return;
    }

    for (index = 0U; index < length; ++index)
    {
        TJC_RxByte(handle, data[index]);
    }
}
