// main/aura_proto.c —— AURA 的 P2 "Frame_ID" 线格式(纯 C)。
//
// 发送固定 Frame_ID 布局(与 Android AuraP2FrameCodec.kt / aura_ble_test 一致):
//   请求: 41 14 | Length u16LE | Frame_ID 00 00 | Opcode u8 | Plen u16LE | Payload | CRC16
//          Length = Plen + 7;整帧 = Length + 4。
// 设备回复存在两种布局,本模块都必须能解析:
//   Frame_ID  : 45 54 | Length u16LE | 00 00 | Result u8 | Opcode u8 | Plen u16LE |
//               Payload | CRC16           (Length = Plen + 8)
//   Opcode LE : 45 54 | Length u16LE | Opcode u16LE | Payload | CRC16
//                                        (Length = Plen + 4,无 Result 字节)
// 布局判别:CRC 通过后,Length >= 8 且 bytes[4..5] == 00 00 视为 Frame_ID,
// 否则按 Opcode(LE) 解析。
// CRC16 与 P1 相同(Modbus,覆盖 [Length .. Payload],低字节在前),直接复用
// main/framed_proto.h 的 framed_crc16()。
//
// 只读用途:0x01 设备信息、0x02 状态;AURA 无需 0x07/0x08 登录。
// 0x01/0x02 的 payload 布局与 NOVA 相同,解析在 main/nova_proto.c。
#include "aura_proto.h"

#include <string.h>

#include "framed_proto.h"

// Frame_ID 布局固定字段:Length 之后的两个 00 字节是布局标记,不是序号。
#define AURA_FRAME_ID_0 0x00
#define AURA_FRAME_ID_1 0x00

bool aura_parse_device_info(const nova_response_t *resp, nova_device_info_t *out)
{
    if (resp == NULL || out == NULL || resp->opcode != NOVA_OP_DEVICE_INFO ||
        resp->payload_len < 3) {
        return false;
    }

    const size_t sn_len = resp->payload[0];
    const size_t hw_len = resp->payload[1];
    const size_t fw_len = resp->payload[2];
    const size_t used = 3u + sn_len + hw_len + fw_len;

    // AURA 参考解析器(aura_ble_test p2_parsers.py)接受 0x01 尾部填充:
    // 只要求 payload 至少覆盖三个长度前缀声明的区间,声明长度之后的字节忽略。
    // 长度前缀本身仍受与 nova_parse_device_info 相同的安全上限约束,
    // 确保写入 out 时一定留得下 NUL。
    if (sn_len > NOVA_MAX_SN_LEN ||
        hw_len >= sizeof(out->hardware_version) ||
        fw_len >= sizeof(out->firmware_version) ||
        (size_t)resp->payload_len < used) {
        return false;
    }

    memcpy(out->sn, resp->payload + 3, sn_len);
    out->sn[sn_len] = '\0';
    memcpy(out->hardware_version, resp->payload + 3 + sn_len, hw_len);
    out->hardware_version[hw_len] = '\0';
    memcpy(out->firmware_version, resp->payload + 3 + sn_len + hw_len, fw_len);
    out->firmware_version[fw_len] = '\0';
    return true;
}

static void aura_crc_write(uint16_t crc, uint8_t out[2])
{
    // 线上低字节在前(与 P1/Modbus 一致)。
    out[0] = (uint8_t)(crc & 0xFFu);
    out[1] = (uint8_t)((crc >> 8) & 0xFFu);
}

size_t aura_build_request(uint8_t opcode, const void *payload, size_t payload_len,
                          uint8_t *out, size_t out_size)
{
    if (out == NULL || payload_len > AURA_MAX_PAYLOAD ||
        out_size < 11u + payload_len) {
        return 0;
    }
    if (payload_len > 0 && payload == NULL) {
        return 0;
    }

    const uint16_t length = (uint16_t)(payload_len + 7u);
    out[0] = FRAMED_APP_HEADER0;
    out[1] = FRAMED_APP_HEADER1;
    out[2] = (uint8_t)(length & 0xFFu);
    out[3] = (uint8_t)((length >> 8) & 0xFFu);
    out[4] = AURA_FRAME_ID_0;
    out[5] = AURA_FRAME_ID_1;
    out[6] = opcode;
    out[7] = (uint8_t)(payload_len & 0xFFu);
    out[8] = (uint8_t)((payload_len >> 8) & 0xFFu);
    if (payload_len > 0) {
        memcpy(out + 9, payload, payload_len);
    }

    // CRC 覆盖 Length + Frame_ID + Opcode + Plen + Payload = length 字节。
    const uint16_t crc = framed_crc16(out + 2, length);
    aura_crc_write(crc, out + 9 + payload_len);
    return 11u + payload_len;
}

size_t aura_extract_responses(uint8_t *buf, size_t *buf_len,
                              nova_response_t *out, size_t max_out)
{
    if (buf == NULL || buf_len == NULL || out == NULL || max_out == 0) {
        return 0;
    }

    const size_t len = *buf_len;
    size_t consumed = 0;
    size_t count = 0;

    while (count < max_out) {
        // 找设备帧头 45 54;没有完整帧头时保留末尾可能被截断的 0x45。
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
            break;  // Length 域还不完整
        }
        const uint16_t length =
            (uint16_t)(buf[consumed + 2] | ((uint16_t)buf[consumed + 3] << 8));
        const size_t total = (size_t)length + 4u;

        // Opcode(LE) 最小帧 8B(Length=4);Frame_ID 最小帧 12B(Length=8)。
        if (length < 4 || total > 4096u) {
            consumed++;
            continue;
        }
        if (remaining < total) {
            break;  // 半帧:保留等待
        }

        // CRC 覆盖 Length 起到 CRC 前的全部字节(整帧去掉 2B 头 + 2B CRC)。
        const uint16_t crc = framed_crc16(buf + consumed + 2, total - 4u);
        const bool crc_ok = buf[consumed + total - 2] == (uint8_t)(crc & 0xFFu) &&
                            buf[consumed + total - 1] == (uint8_t)((crc >> 8) & 0xFFu);
        if (!crc_ok) {
            // 坏 CRC:丢 1 字节重新找帧头(不要整帧丢弃,避免吞掉后续合法帧)。
            consumed++;
            continue;
        }

        const bool frame_id_layout = length >= 8 &&
                                     buf[consumed + 4] == AURA_FRAME_ID_0 &&
                                     buf[consumed + 5] == AURA_FRAME_ID_1;

        uint8_t result = 0;
        uint8_t opcode = 0;
        uint16_t plen = 0;
        size_t payload_off = 0;

        if (frame_id_layout) {
            result = buf[consumed + 6];
            opcode = buf[consumed + 7];
            plen = (uint16_t)(buf[consumed + 8] |
                              ((uint16_t)buf[consumed + 9] << 8));
            if (plen > AURA_MAX_PAYLOAD || length != (uint16_t)(plen + 8u)) {
                consumed++;
                continue;
            }
            payload_off = 10;
        } else {
            // Opcode(LE) 布局没有 Result 字节,查询应答视为 result=0。
            // nova_response_t.opcode 是 u8;只读/上报 opcode 都在低字节
            // (如 0x02、0x11、0x8C),高字节为 0,这里显式取低字节。
            const uint16_t opcode16 =
                (uint16_t)(buf[consumed + 4] |
                           ((uint16_t)buf[consumed + 5] << 8));
            opcode = (uint8_t)(opcode16 & 0xFFu);
            plen = (uint16_t)(length - 4u);
            if (plen > AURA_MAX_PAYLOAD) {
                consumed++;
                continue;
            }
            payload_off = 6;
        }

        nova_response_t *dst = &out[count];
        dst->result = result;
        dst->opcode = opcode;
        dst->payload_len = plen;
        if (plen > 0) {
            memcpy(dst->payload, buf + consumed + payload_off, plen);
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
