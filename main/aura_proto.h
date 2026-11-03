// main/aura_proto.h —— AURA 的 P2 线格式(纯 C)。
//
// AURA 使用 P2 "Frame_ID" 布局(当前 Android 实现与 Python 工具实际发送的格式):
//   请求 APP→设备: 41 14 | Length u16LE | Frame_ID 00 00 | Opcode u8 | Plen u16LE | Payload | CRC16
//   响应 设备→APP: 45 54 | Length u16LE | Frame_ID 00 00 | Result u8 | Opcode u8 | Plen u16LE | Payload | CRC16
//   Length = PayloadLen + 7(请求) / PayloadLen + 8(响应);整帧 = Length + 4。
// CRC16 同 P1(Modbus,覆盖 Length 起到 CRC 前的全部字节,低字节在前)。
//
// 设备也可能回兼容的 Opcode(LE) 布局(no Frame_ID):45 54 | Length | Opcode u16LE | Payload | CRC16。
// 本模块两种布局都能解析;只读用途:0x01 设备信息、0x02 状态(AURA 无需登录认证)。
// 0x01 payload 布局与 NOVA 相同但允许末尾填充(aura_parse_device_info);0x02
// 的 level+code 解析复用 main/nova_proto.h 的 nova_parse_status。
#pragma once

#include "nova_proto.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define AURA_MAX_PAYLOAD 512
// 请求最大:2+2+2+1+2+payload+2
#define AURA_MAX_REQUEST (11 + AURA_MAX_PAYLOAD)
// 响应最大(P2 Frame_ID):2+2+2+1+1+2+payload+2
#define AURA_MAX_RESPONSE (12 + AURA_MAX_PAYLOAD)

// 解析 AURA 的 0x01 设备信息:AURA 参考解析器接受末尾填充字节,只要求
// plen >= 3+snLen+hwLen+fwLen,因此不能直接用要求精确长度的 nova_parse_device_info。
bool aura_parse_device_info(const nova_response_t *resp, nova_device_info_t *out);

// 组 P2 Frame_ID 请求帧;参数不合法或缓冲区不足返回 0。
size_t aura_build_request(uint8_t opcode, const void *payload, size_t payload_len,
                          uint8_t *out, size_t out_size);

// 从累积缓冲区提取完整响应帧(同时兼容 P2 Frame_ID 与 Opcode(LE) 两种设备回复):
// 半包保留、粘包逐帧、坏 CRC/坏长度丢 1 字节重同步。
// 返回写入 out 的帧数;*buf_len 更新为剩余未消费字节数。
size_t aura_extract_responses(uint8_t *buf, size_t *buf_len,
                              nova_response_t *out, size_t max_out);
