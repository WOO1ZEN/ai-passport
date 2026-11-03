// main/framed_proto.c —— OMNI / HALO / PACE 的 P1 Framed 线格式(纯 C)。
//
// 线格式(与 Android FramedFrameCodec.kt、pace_ble_test/libs/common/frame_codec.py
// 以及 omni_halo_ble_sdk 逐字节一致):
//   请求 APP→设备: 41 14 | Length u16LE | Opcode u8 | Plen u16LE | Payload | CRC16
//   响应 设备→APP: 45 54 | Length u16LE | Result u8 | Opcode u8 | Plen u16LE | Payload | CRC16
//   Length = Plen + 5(请求) / Plen + 6(响应);整帧长 = Length + 4。
//   CRC16 = Modbus(poly 0xA001, init 0xFFFF, 无最终异或),覆盖 [Length .. Payload],
//   线上先发低字节(即 framed_crc16() 返回值的低字节在前)。
//
// 只读用途:0x07/0x08 登录、0x01 设备信息、0x02 状态;0x01/0x02 的 payload 布局
// 与 NOVA 相同,因此解析复用 main/nova_proto.h,本文件只负责成帧/验帧。
#include "framed_proto.h"

#include <string.h>

uint16_t framed_crc16(const uint8_t *data, size_t len)
{
    // 标准 Modbus CRC16:初值 0xFFFF,多项式 0xA001,输入输出均不反转。
    // 返回值即"线上两字节按小端解释"的数值,发送时先写低字节。
    uint16_t crc = 0xFFFFu;

    if (data == NULL) {
        return crc;
    }
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i];
        for (size_t bit = 0; bit < 8; bit++) {
            if ((crc & 1u) != 0u) {
                crc = (uint16_t)((crc >> 1) ^ 0xA001u);
            } else {
                crc = (uint16_t)(crc >> 1);
            }
        }
    }
    return crc;
}

// 把 16 位 CRC 写成线上两字节(低字节在前)。
static void framed_crc_write(uint16_t crc, uint8_t out[2])
{
    out[0] = (uint8_t)(crc & 0xFFu);
    out[1] = (uint8_t)((crc >> 8) & 0xFFu);
}

size_t framed_build_request(uint8_t opcode, const void *payload, size_t payload_len,
                            uint8_t *out, size_t out_size)
{
    if (out == NULL || payload_len > FRAMED_MAX_PAYLOAD ||
        out_size < 9u + payload_len) {
        return 0;
    }
    if (payload_len > 0 && payload == NULL) {
        return 0;
    }

    const uint16_t length = (uint16_t)(payload_len + 5u);
    out[0] = FRAMED_APP_HEADER0;
    out[1] = FRAMED_APP_HEADER1;
    out[2] = (uint8_t)(length & 0xFFu);
    out[3] = (uint8_t)((length >> 8) & 0xFFu);
    out[4] = opcode;
    out[5] = (uint8_t)(payload_len & 0xFFu);
    out[6] = (uint8_t)((payload_len >> 8) & 0xFFu);
    if (payload_len > 0) {
        memcpy(out + 7, payload, payload_len);
    }

    // CRC 覆盖 Length(2) + Opcode(1) + Plen(2) + Payload = 5 + payload_len 字节。
    const uint16_t crc = framed_crc16(out + 2, 5u + payload_len);
    framed_crc_write(crc, out + 7 + payload_len);
    return 9u + payload_len;
}

size_t framed_extract_responses(uint8_t *buf, size_t *buf_len,
                                nova_response_t *out, size_t max_out)
{
    if (buf == NULL || buf_len == NULL || out == NULL || max_out == 0) {
        return 0;
    }

    const size_t len = *buf_len;
    size_t consumed = 0;
    size_t count = 0;

    while (count < max_out) {
        // 在剩余字节里找设备帧头 45 54;没有完整帧头时保留末尾可能被截断的 0x45。
        size_t i = consumed;
        while (i + 1 < len &&
               !(buf[i] == FRAMED_DEV_HEADER0 && buf[i + 1] == FRAMED_DEV_HEADER1)) {
            i++;
        }
        if (i + 1 >= len) {
            consumed = (len > 0 && buf[len - 1] == FRAMED_DEV_HEADER0) ? len - 1 : len;
            break;
        }
        consumed = i;

        const size_t remaining = len - consumed;
        if (remaining < 4) {
            break;  // Length 域还不完整,保留等待
        }
        const uint16_t length =
            (uint16_t)(buf[consumed + 2] | ((uint16_t)buf[consumed + 3] << 8));
        const size_t total = (size_t)length + 4u;

        // P1 响应最小合法帧:Length=6(空 payload)→ 整帧 10B;最大 4096B。
        if (length < 6 || total > 4096u) {
            consumed++;
            continue;
        }
        if (remaining < total) {
            break;  // 半帧:保留等待
        }

        const uint16_t plen =
            (uint16_t)(buf[consumed + 6] | ((uint16_t)buf[consumed + 7] << 8));
        const uint16_t crc = framed_crc16(buf + consumed + 2, total - 4u);
        const bool crc_ok = buf[consumed + total - 2] == (uint8_t)(crc & 0xFFu) &&
                            buf[consumed + total - 1] == (uint8_t)((crc >> 8) & 0xFFu);

        // 结构不自洽、payload 超过静态上限或 CRC 错:丢 1 字节重新找帧头,
        // 不要整帧丢弃,否则可能连同紧随其后的合法帧一起吞掉。
        if (!crc_ok || plen > FRAMED_MAX_PAYLOAD || length != (uint16_t)(plen + 6u)) {
            consumed++;
            continue;
        }

        nova_response_t *dst = &out[count];
        dst->result = buf[consumed + 4];
        dst->opcode = buf[consumed + 5];
        dst->payload_len = plen;
        if (plen > 0) {
            memcpy(dst->payload, buf + consumed + 8, plen);
        }
        count++;
        consumed += total;
    }

    if (consumed > 0) {
        memmove(buf, buf + consumed, len - consumed);
        *buf_len = len - consumed;
    }
    return count;
}
