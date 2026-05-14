#include "lora_packet.h"

#include <LoRa.h>

#include "static_huffman.h"

namespace {
uint16_t nextSeq = 1;
uint16_t nextMsgId = 1;

// Bộ nhớ chống xử lý trùng. Retry có thể làm receiver nhận lại cùng src+seq.
struct SeenPacket {
  uint8_t src;
  uint16_t seq;
};

SeenPacket seenPackets[12] = {};
uint8_t seenIndex = 0;

// Bộ nhớ ghép fragment cho một message tại một thời điểm.
// BTL/prototype thường chỉ gửi tuần tự, nên giữ một buffer nhỏ sẽ dễ debug.
struct RxMessageBuffer {
  bool active;
  uint8_t src;
  uint16_t msgId;
  uint8_t codec;
  uint16_t rawLen;
  uint16_t encodedBitLen;
  uint8_t fragCount;
  bool received[LORA_MAX_FRAGMENTS];
  uint8_t data[LORA_ENCODED_MAX];
};

RxMessageBuffer rxBuffer = {};

// Metric tối thiểu để tính packet loss ở phía sender.
uint32_t txFragments = 0;
uint32_t txFailedFragments = 0;

uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;

  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;

    for (uint8_t bit = 0; bit < 8; ++bit) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc <<= 1;
      }
    }
  }

  return crc;
}

uint16_t packet_crc(const LoRaPacket &packet) {
  LoRaPacket copy = packet;
  copy.crc = 0;
  return crc16_ccitt(reinterpret_cast<const uint8_t *>(&copy), sizeof(copy));
}

bool validate_packet(const LoRaPacket &packet) {
  if (packet.version != LORA_PROTOCOL_VERSION) {
    return false;
  }

  if (packet.payloadLen > LORA_PAYLOAD_MAX) {
    return false;
  }

  if (packet.fragCount == 0 || packet.fragCount > LORA_MAX_FRAGMENTS) {
    return false;
  }

  if (packet.fragIndex >= packet.fragCount) {
    return false;
  }

  return packet.crc == packet_crc(packet);
}

void finalize_packet(LoRaPacket &packet) {
  packet.crc = 0;
  packet.crc = packet_crc(packet);
}

uint8_t get_next_hop(uint8_t dst) {
  // Trong mạng Flooding Mesh (hoặc Broadcast), ta đánh dấu nextHop là BROADCAST_ID
  // để ám chỉ tất cả mọi người cùng nghe và xử lý.
  return BROADCAST_ID;
}

bool send_raw_packet(LoRaPacket &packet) {
  finalize_packet(packet);

  // LoRa half-duplex: chuyển sang idle để gửi, gửi xong quay lại receive.
  LoRa.idle();
  LoRa.beginPacket();
  LoRa.write(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
  int result = LoRa.endPacket();
  LoRa.receive();

  return result == 1;
}

bool read_packet(LoRaPacket &packet) {
  int packetSize = LoRa.parsePacket();
  if (packetSize == 0) {
    return false;
  }

  if (packetSize != static_cast<int>(sizeof(LoRaPacket))) {
    while (LoRa.available()) {
      LoRa.read();
    }
    return false;
  }

  size_t bytesRead = LoRa.readBytes(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
  return bytesRead == sizeof(packet) && validate_packet(packet);
}

bool wait_for_ack(uint8_t expectedFrom, uint16_t seq, unsigned long &rttMs) {
  unsigned long startedAt = millis();

  while (millis() - startedAt < LORA_ACK_TIMEOUT_MS) {
    LoRaPacket packet;
    if (!read_packet(packet)) {
      delay(5);
      continue;
    }

    if (packet.type == LORA_PKT_ACK &&
        packet.src == expectedFrom &&
        packet.dst == NODE_ID &&
        packet.seq == seq) {
      rttMs = millis() - startedAt;
      return true;
    }
  }

  rttMs = millis() - startedAt;
  return false;
}

bool send_with_retry(LoRaPacket &packet) {
  // Gửi Broadcast trong mạng Mesh thì không yêu cầu ACK, vì nếu yêu cầu
  // tất cả các node cùng trả lời sẽ gây bão mạng.
  if (packet.dst == BROADCAST_ID || packet.nextHop == BROADCAST_ID) {
     packet.flags &= ~LORA_FLAG_ACK_REQ; // Tắt cờ ACK
  }

  for (uint8_t attempt = 0; attempt < LORA_MAX_RETRY; ++attempt) {
    // Tăng random delay (Jitter) để tránh đụng độ sóng khi các node cùng relay
    if (packet.flags & LORA_FLAG_RELAYED) {
       delay(random(100, 600)); 
    } else {
       delay(random(40, 160));
    }

    if (!send_raw_packet(packet)) {
      continue;
    }

    if ((packet.flags & LORA_FLAG_ACK_REQ) == 0) {
      return true; // Không yêu cầu ACK thì gửi 1 lần (hoặc có thể lặp 3 lần không cần chờ)
    }

    unsigned long rttMs = 0;
    if (wait_for_ack(packet.nextHop, packet.seq, rttMs)) {
      Serial.print("ACK seq=");
      Serial.print(packet.seq);
      Serial.print(" from=");
      Serial.print(packet.nextHop);
      Serial.print(" rtt_ms=");
      Serial.println(rttMs);
      return true;
    }

    Serial.print("Retry seq=");
    Serial.print(packet.seq);
    Serial.print(" attempt=");
    Serial.println(attempt + 1);
  }

  return false;
}

void send_ack(const LoRaPacket &packet) {
  LoRaPacket ack = {};
  ack.version = LORA_PROTOCOL_VERSION;
  ack.type = LORA_PKT_ACK;
  ack.src = NODE_ID;
  ack.dst = packet.prevHop;
  ack.prevHop = NODE_ID;
  ack.nextHop = packet.prevHop;
  ack.seq = packet.seq;
  ack.msgId = packet.msgId;
  ack.fragIndex = packet.fragIndex;
  ack.fragCount = packet.fragCount;
  ack.ttl = 1;
  ack.flags = 0;
  ack.codec = LORA_CODEC_NONE;
  ack.rawLen = 0;
  ack.encodedBitLen = 0;
  ack.payloadLen = 0;

  send_raw_packet(ack);
}

bool was_seen(uint8_t src, uint16_t seq) {
  for (const SeenPacket &seen : seenPackets) {
    if (seen.src == src && seen.seq == seq) {
      return true;
    }
  }

  return false;
}

void mark_seen(uint8_t src, uint16_t seq) {
  seenPackets[seenIndex] = {src, seq};
  seenIndex = (seenIndex + 1) % (sizeof(seenPackets) / sizeof(seenPackets[0]));
}

bool all_fragments_received(const RxMessageBuffer &buffer) {
  for (uint8_t i = 0; i < buffer.fragCount; ++i) {
    if (!buffer.received[i]) {
      return false;
    }
  }

  return true;
}

void reset_rx_buffer(const LoRaPacket &packet) {
  memset(&rxBuffer, 0, sizeof(rxBuffer));
  rxBuffer.active = true;
  rxBuffer.src = packet.src;
  rxBuffer.msgId = packet.msgId;
  rxBuffer.codec = packet.codec;
  rxBuffer.rawLen = packet.rawLen;
  rxBuffer.encodedBitLen = packet.encodedBitLen;
  rxBuffer.fragCount = packet.fragCount;
}

void print_link_metric() {
  Serial.print("RSSI=");
  Serial.print(LoRa.packetRssi());
  Serial.print(" SNR=");
  Serial.println(LoRa.packetSnr());
}

void print_complete_message(const uint8_t *encodedData,
                            uint16_t encodedBytes,
                            uint8_t codec,
                            uint16_t rawLen,
                            uint16_t encodedBitLen) {
  uint8_t decoded[LORA_MESSAGE_MAX + 1] = {};
  bool ok = false;

  if (codec == LORA_CODEC_STATIC_HUFFMAN) {
    ok = huffman_decompress(encodedData, encodedBitLen, decoded, rawLen);
  } else {
    if (rawLen <= encodedBytes && rawLen <= LORA_MESSAGE_MAX) {
      memcpy(decoded, encodedData, rawLen);
      ok = true;
    } else {
      Serial.println("Decode error: rawLen invalid for LORA_CODEC_NONE");
    }
  }

  if (!ok) {
    Serial.println("Decode failed");
    return;
  }

  decoded[rawLen] = '\0';

  Serial.print("MESSAGE from node ");
  Serial.print(rxBuffer.src);
  Serial.print(" msgId=");
  Serial.print(rxBuffer.msgId);
  Serial.print(" text=");
  Serial.println(reinterpret_cast<char *>(decoded));
}

void handle_final_fragment(const LoRaPacket &packet) {
  if (!rxBuffer.active || rxBuffer.src != packet.src || rxBuffer.msgId != packet.msgId) {
    reset_rx_buffer(packet);
  }

  uint16_t offset = static_cast<uint16_t>(packet.fragIndex) * LORA_PAYLOAD_MAX;
  if (offset + packet.payloadLen > LORA_ENCODED_MAX) {
    Serial.println("Drop fragment: reassembly buffer overflow");
    return;
  }

  memcpy(rxBuffer.data + offset, packet.payload, packet.payloadLen);
  rxBuffer.received[packet.fragIndex] = true;

  Serial.print("RX frag ");
  Serial.print(packet.fragIndex + 1);
  Serial.print("/");
  Serial.print(packet.fragCount);
  Serial.print(" msgId=");
  Serial.println(packet.msgId);

  if (!all_fragments_received(rxBuffer)) {
    return;
  }

  uint16_t encodedBytes = rxBuffer.encodedBitLen > 0 ? (rxBuffer.encodedBitLen + 7) / 8 : 0;
  if (rxBuffer.codec == LORA_CODEC_NONE) {
    encodedBytes = rxBuffer.rawLen;
  }

  print_complete_message(rxBuffer.data,
                         encodedBytes,
                         rxBuffer.codec,
                         rxBuffer.rawLen,
                         rxBuffer.encodedBitLen);
  rxBuffer.active = false;
}

void relay_packet(LoRaPacket &packet) {
  if (packet.ttl <= 1) {
    Serial.println("Drop packet: TTL expired");
    return;
  }

  packet.ttl--;
  packet.prevHop = NODE_ID;
  packet.nextHop = get_next_hop(packet.dst);
  packet.flags |= LORA_FLAG_RELAYED | LORA_FLAG_ACK_REQ;

  Serial.print("Relay msgId=");
  Serial.print(packet.msgId);
  Serial.print(" frag=");
  Serial.print(packet.fragIndex + 1);
  Serial.print("/");
  Serial.print(packet.fragCount);
  Serial.print(" nextHop=");
  Serial.println(packet.nextHop);

  send_with_retry(packet);
}

void print_compression_metric(uint8_t codec,
                              uint16_t rawLen,
                              uint16_t encodedBytes,
                              uint16_t encodedBits,
                              float entropy) {
  Serial.print("codec=");
  Serial.print(codec == LORA_CODEC_STATIC_HUFFMAN ? "huffman" : "none");
  Serial.print(" raw_bytes=");
  Serial.print(rawLen);
  Serial.print(" encoded_bytes=");
  Serial.print(encodedBytes);
  Serial.print(" encoded_bits=");
  Serial.print(encodedBits);
  Serial.print(" entropy=");
  Serial.print(entropy, 3);
  Serial.print(" compression_ratio=");
  Serial.println(static_cast<float>(encodedBytes) / static_cast<float>(rawLen), 3);
}
}  // namespace

bool lora_send_text(uint8_t dst, const char *text) {
  if (text == nullptr) {
    return false;
  }

  uint16_t rawLen = min(static_cast<size_t>(LORA_MESSAGE_MAX), strlen(text));
  if (rawLen == 0) {
    return false;
  }

  uint8_t raw[LORA_MESSAGE_MAX] = {};
  uint8_t encoded[LORA_ENCODED_MAX] = {};
  uint16_t encodedBits = 0;
  uint16_t encodedBytes = 0;
  uint8_t codec = LORA_CODEC_NONE;

  memcpy(raw, text, rawLen);
  float entropy = huffman_entropy(raw, rawLen);

  bool compressed = huffman_compress(raw, rawLen, encoded, LORA_ENCODED_MAX, encodedBits, encodedBytes);
  if (compressed && encodedBytes < rawLen) {
    codec = LORA_CODEC_STATIC_HUFFMAN;
  } else {
    memcpy(encoded, raw, rawLen);
    encodedBytes = rawLen;
    encodedBits = rawLen * 8;
  }

  print_compression_metric(codec, rawLen, encodedBytes, encodedBits, entropy);

  uint8_t fragCount = encodedBytes == 0 ? 1 : (encodedBytes + LORA_PAYLOAD_MAX - 1) / LORA_PAYLOAD_MAX;
  if (fragCount == 0 || fragCount > LORA_MAX_FRAGMENTS) {
    Serial.println("Message too long after encoding");
    return false;
  }

  uint16_t msgId = nextMsgId++;
  bool allOk = true;

  // Reset tx_fragments để đếm cho mỗi lần gửi thay vì cộng dồn từ khi khởi động
  txFragments = 0;
  txFailedFragments = 0;

  for (uint8_t fragIndex = 0; fragIndex < fragCount; ++fragIndex) {
    uint16_t offset = static_cast<uint16_t>(fragIndex) * LORA_PAYLOAD_MAX;
    uint8_t chunkLen = min(static_cast<uint16_t>(LORA_PAYLOAD_MAX), static_cast<uint16_t>(encodedBytes - offset));

    LoRaPacket packet = {};
    packet.version = LORA_PROTOCOL_VERSION;
    packet.type = LORA_PKT_DATA;
    packet.src = NODE_ID;
    packet.dst = dst;
    packet.prevHop = NODE_ID;
    packet.nextHop = get_next_hop(dst);
    packet.seq = nextSeq++;
    packet.msgId = msgId;
    packet.fragIndex = fragIndex;
    packet.fragCount = fragCount;
    packet.ttl = LORA_TTL_DEFAULT;
    packet.flags = LORA_FLAG_ACK_REQ;
    packet.codec = codec;
    packet.rawLen = rawLen;
    packet.encodedBitLen = encodedBits;
    packet.payloadLen = chunkLen;
    memcpy(packet.payload, encoded + offset, chunkLen);

    if (fragIndex > 0 && dst == BROADCAST_ID) {
      // Cho các node trung gian (relay) thời gian rảnh để phát lại (forward) gói tin trước đó.
      // Nếu không có delay, node trung gian sẽ bận TX và điếc (không nhận được) gói tiếp theo.
      delay(1500);
    }

    txFragments++;
    if (!send_with_retry(packet)) {
      txFailedFragments++;
      allOk = false;
    }
  }

  Serial.print("tx_fragments=");
  Serial.print(txFragments);
  Serial.print(" tx_failed=");
  Serial.print(txFailedFragments);
  Serial.print(" packet_loss_est=");
  Serial.println(static_cast<float>(txFailedFragments) / static_cast<float>(txFragments), 3);

  return allOk;
}

void lora_process() {
  LoRaPacket packet;
  if (!read_packet(packet)) {
    return;
  }

  // Bỏ qua các gói do chính node này gửi ra nhưng lại vọng lại
  if (packet.src == NODE_ID) {
    return;
  }

  if (packet.type == LORA_PKT_ACK) {
    return;
  }

  if (packet.type != LORA_PKT_DATA) {
    return;
  }

  print_link_metric();
  
  // Chỉ gửi ACK nếu gói tin yêu cầu
  if (packet.flags & LORA_FLAG_ACK_REQ) {
    send_ack(packet);
  }

  bool duplicate = was_seen(packet.src, packet.seq);
  if (!duplicate) {
    mark_seen(packet.src, packet.seq);
  } else {
    // Đã thấy gói này rồi, bỏ qua không xử lý hay relay tiếp
    return;
  }

  // Xử lý dữ liệu đến tay (đích là mình hoặc là gói Broadcast)
  if (packet.dst == NODE_ID || packet.dst == BROADCAST_ID) {
    handle_final_fragment(packet);
  }

  // Nếu là gói Broadcast (Flooding) thì tiếp tục Relay cho các node khác (nếu TTL > 1)
  if (packet.dst == BROADCAST_ID) {
    relay_packet(packet);
  }
}
