#pragma once

#include <Arduino.h>

#ifndef NODE_ID
#define NODE_ID 1
#endif

#ifndef BROADCAST_ID
#define BROADCAST_ID 255
#endif

// Giới hạn này cố ý nhỏ hơn nhiều so với FIFO tối đa 255 byte của SX127x.
// Packet nhỏ giúp multi-hop ổn định hơn, ACK nhanh hơn và ít bị lỗi hơn.
constexpr uint8_t LORA_PROTOCOL_VERSION = 1;
constexpr uint8_t LORA_PAYLOAD_MAX = 64;
constexpr uint16_t LORA_MESSAGE_MAX = 1024;
constexpr uint16_t LORA_ENCODED_MAX = 1280;
constexpr uint8_t LORA_MAX_FRAGMENTS = (LORA_ENCODED_MAX + LORA_PAYLOAD_MAX - 1) / LORA_PAYLOAD_MAX;
constexpr uint8_t LORA_TTL_DEFAULT = 5;
constexpr uint8_t LORA_MAX_RETRY = 3;
constexpr unsigned long LORA_ACK_TIMEOUT_MS = 1200;

// DATA là gói dữ liệu thật; ACK là xác nhận cho từng hop.
enum LoRaPacketType : uint8_t {
  LORA_PKT_DATA = 1,
  LORA_PKT_ACK = 2,
};

// ACK_REQ yêu cầu node kế tiếp phản hồi ACK; RELAYED đánh dấu gói đã qua relay.
enum LoRaPacketFlags : uint8_t {
  LORA_FLAG_ACK_REQ = 0x01,
  LORA_FLAG_RELAYED = 0x02,
};

// Codec cho payload. NONE gửi raw bytes; STATIC_HUFFMAN gửi bitstream đã nén.
enum LoRaPayloadCodec : uint8_t {
  LORA_CODEC_NONE = 0,
  LORA_CODEC_STATIC_HUFFMAN = 1,
};

#pragma pack(push, 1)
struct LoRaPacket {
  uint8_t version;
  uint8_t type;
  uint8_t src;
  uint8_t dst;
  uint8_t prevHop;
  uint8_t nextHop;
  uint16_t seq;
  uint16_t msgId;
  uint8_t fragIndex;
  uint8_t fragCount;
  uint8_t ttl;
  uint8_t flags;
  uint8_t codec;
  uint16_t rawLen;
  uint16_t encodedBitLen;
  uint8_t payloadLen;
  uint8_t payload[LORA_PAYLOAD_MAX];
  uint16_t crc;
};
#pragma pack(pop)

static_assert(sizeof(LoRaPacket) <= 255, "LoRaPacket must fit inside SX127x FIFO");

// Gửi text tới dst. Hàm này tự nén Huffman nếu nén có lợi, rồi chia fragment.
bool lora_send_text(uint8_t dst, const char *text);

// Gọi liên tục trong loop(); hàm này nhận packet, ACK, relay và ghép fragment.
void lora_process();
