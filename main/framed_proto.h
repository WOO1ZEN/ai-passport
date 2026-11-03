// main/framed_proto.h —— OMNI / HALO / PACE 的 P1 Framed 线格式(纯 C)。
//
// P1 帧结构(无 Frame_ID 字段):
//   请求 APP→设备: 41 14 | Length u16LE | Opcode u8 | Plen u16LE | Payload | CRC16
//   响应 设备→APP: 45 54 | Length u16LE | Result u8 | Opcode u8 | Plen u16LE | Payload | CRC16
//   Length = PayloadLen + 5(请求) / PayloadLen + 6(响应);整帧 = Length + 4。
//   CRC16 = Modbus(0xA001, init 0xFFFF),覆盖 Length+数据域(不含 41 14/45 54 与 CRC 本身),
//   线上低字节在前。
//
// 只读用途:0x07/0x08 登录、0x01 设备信息、0x02 状态;解析复用 main/nova_proto.h
// (三款机型的 0x01/0x02 payload 布局与 NOVA 相同)。GATT UUID 也复用 NOVA 的那组。
#pragma once

#include "nova_proto.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define FRAMED_APP_HEADER0 0x41
#define FRAMED_APP_HEADER1 0x14
#define FRAMED_DEV_HEADER0 0x45
#define FRAMED_DEV_HEADER1 0x54

#define FRAMED_MAX_PAYLOAD 512
// 请求最大:2(头)+2(len)+1(op)+2(plen)+payload+2(crc)
#define FRAMED_MAX_REQUEST (9 + FRAMED_MAX_PAYLOAD)
// 响应最大:2(头)+2(len)+1(result)+1(op)+2(plen)+payload+2(crc)
#define FRAMED_MAX_RESPONSE (10 + FRAMED_MAX_PAYLOAD)

// Modbus CRC16(0xA001,init 0xFFFF),返回值即线上两字节按小端解释后的数值。
uint16_t framed_crc16(const uint8_t *data, size_t len);

// 组 P1 请求帧;参数不合法或缓冲区不足返回 0。
size_t framed_build_request(uint8_t opcode, const void *payload, size_t payload_len,
                            uint8_t *out, size_t out_size);

// 从累积缓冲区提取完整响应帧:半包保留、粘包逐帧、坏 CRC/坏长度丢 1 字节重同步。
// 返回写入 out 的帧数;*buf_len 更新为剩余未消费字节数。
size_t framed_extract_responses(uint8_t *buf, size_t *buf_len,
                                nova_response_t *out, size_t max_out);
