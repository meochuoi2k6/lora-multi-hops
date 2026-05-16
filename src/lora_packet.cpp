#include "lora_packet.h"
#include "lora_setup.h"
#include <Arduino.h>
#include <stdint.h>
#include <SPI.h>
#include <LoRa.h>
#include <static_huffman.h>

#include <string.h> // Để dùng memcpy, memset

// ====================================================================
//                          CÁC HÀM PRIVATE 
// ====================================================================
namespace {

/// Biến đếm số thứ tự gói tin (Sequence Number) để chống trùng lặp.
uint16_t nextSeq = 1;

/// Biến đếm ID thông điệp (Message ID) tiếp theo để gom các mảnh phân mảnh.
uint16_t nextMsgId = 1;

/**
 * @brief Cấu trúc lưu trữ thông tin nhận diện một gói tin đã xử lý.
 * @brief - `src`: ID của node gốc tạo ra gói tin
 * @brief - `seq`: Số thứ tự của gói tin đó
 * @note Dùng để chống trùng lặp (Deduplication) trong mạng Flooding Mesh.
 */
struct SeenPacket {
  uint8_t src;  ///< ID của node gốc tạo ra gói tin
  uint16_t seq; ///< Số thứ tự của gói tin đó
};

/// Bộ đệm vòng (Circular Buffer) lưu lại 12 gói tin gần nhất đã nghe thấy.
SeenPacket seenPackets[12] = {};

/**
 * @brief Bộ đệm dùng để ghép mảnh (Reassembly) các gói tin đến.
 * @brief - `active`: Có đang trong quá trình nhận dở dang một tin nhắn không?
 * @brief - `src`: ID của người đang gửi tin nhắn này
 * @brief - `msgId`: ID của tin nhắn đang nhận dở
 * @brief - `codec`: Thuật toán nén của tin nhắn này
 * @brief - `rawLen`: Chiều dài dữ liệu gốc
 * @brief - `encodedBitLen`: Chiều dài bit sau nén
 * @brief - `fragCount`: Tổng số mảnh phải nhận
 * @brief - `received`: Mảng đánh dấu (true/false) xem mảnh nào đã nhận được
 * @brief - `data`: Bộ đệm chứa dữ liệu thô của các mảnh ghép lại
 * @note Vì LoRa chỉ gửi tối đa 64 byte mỗi lần, một thông điệp dài 
 * sẽ bị cắt làm nhiều gói (fragment). Bộ đệm này giữ các mảnh lại cho 
 * đến khi nhận đủ thì mới giải mã toàn bộ.
 */
struct RxMessageBuffer {
  bool active;                           ///< Có đang trong quá trình nhận dở dang một tin nhắn không?
  uint8_t src;                           ///< ID của người đang gửi tin nhắn này
  uint16_t msgId;                        ///< ID của tin nhắn đang nhận dở
  uint8_t codec;                         ///< Thuật toán nén của tin nhắn này
  uint16_t rawLen;                       ///< Chiều dài dữ liệu gốc
  uint16_t encodedBitLen;                ///< Chiều dài bit sau nén
  uint8_t fragCount;                     ///< Tổng số mảnh phải nhận
  bool received[LORA_MAX_FRAGMENTS];     ///< Mảng đánh dấu (true/false) xem mảnh nào đã nhận được
  uint8_t data[LORA_ENCODED_MAX];        ///< Bộ đệm chứa dữ liệu thô của các mảnh ghép lại
};

/// Khởi tạo một bộ đệm trống
RxMessageBuffer rxBuffer = {};

/**
 * @brief Kiểm tra xem gói tin này đã từng được nhận và xử lý hay chưa.
 * @param src ID của node gốc gửi gói tin.
 * @param seq Số thứ tự của gói tin.
 * @return true Nếu đã từng xử lý gói này (cần Drop để tránh Broadcast Storm).
 * @return false Nếu đây là gói tin mới tinh.
 */
bool was_seen(uint8_t src, uint16_t seq) {
    for(const SeenPacket &seen : seenPackets) {
        if (seen.src == src && seen.seq == seq) {
            return true;
        }
    }
    return false;
}

/// Con trỏ vị trí (Index) sẽ ghi đè vào phần tử tiếp theo trong mảng seenPackets.
uint8_t seenIndex = 0;

/**
 * @brief Ghi nhớ một gói tin vào bộ đệm để lần sau gặp lại sẽ bỏ qua.
 * @note Hàm sử dụng kỹ thuật "Bộ đệm vòng" (Circular Buffer). 
 * Khi mảng đầy, nó sẽ tự động quay vòng lại index 0 và ghi đè lên gói tin cũ nhất.
 * @param src ID của node gốc gửi gói tin.
 * @param seq Số thứ tự của gói tin.
 */
void mark_seen(uint8_t src, uint16_t seq) {
    seenPackets[seenIndex] = {src, seq};
    seenIndex = (seenIndex + 1) % (sizeof(seenPackets) / sizeof(seenPackets[0]));
}

uint32_t txFragments = 0; // Biến đếm số phân mảnh đã gửi thành công
uint32_t txFailedFragments = 0; // Biến đếm số phân mảnh đã gửi thất bại (sau nhiều lần thử)

/**
 * @brief Tính toán CRC16-CCITT cho dữ liệu đầu vào.
 * @param data Con trỏ đến mảng byte cần tính CRC.
 * @param len Độ dài của mảng dữ liệu (số byte).
 * @return Giá trị CRC16 tính được.
 */
uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  
  for (size_t i = 0; i < len; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc = (crc << 1);
      }
    }
  }
  return crc;
}

/**
 * @brief Tính toán CRC cho một gói tin LoRaPacket.
 * @param packet Gói tin cần tính CRC.
 * @return Giá trị CRC16 của gói tin.
 */
uint16_t packet_crc(const LoRaPacket &packet) {
    LoRaPacket copy = packet;
    copy.crc = 0; // Đặt trường CRC về 0 trước khi tính toán
    return crc16_ccitt(reinterpret_cast<const uint8_t*>(&copy), sizeof(copy)); //ep kiểu để tính toán trên toàn bộ cấu trúc
}

/***
 * @brief Kiểm tra tính hợp lệ của một gói tin dựa trên phiên bản, độ dài, chỉ số phân mảnh, và CRC.
 * @param packet Gói tin cần kiểm tra.
 * @return - `true` Nếu gói tin hợp lệ và có thể xử lý tiếp.
 * @return - `false` Nếu gói tin không hợp lệ (phiên bản sai, độ dài payload vượt quá giới hạn, chỉ số phân mảnh không hợp lệ, hoặc CRC không khớp).
 * @note Việc kiểm tra này rất quan trọng để đảm bảo rằng node chỉ xử lý các gói tin đúng định dạng và không bị lỗi trong quá trình truyền tải, từ đó tránh lãng phí tài nguyên cho các gói tin hỏng hoặc giả mạo.
 */
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

/** @brief Hoàn thiện gói tin trước khi gửi bằng cách tính toán và điền giá trị CRC.
 * @param packet Gói tin cần hoàn thiện.
 */
void finalize_packet(LoRaPacket &packet) {
  packet.crc = 0;
  packet.crc = packet_crc(packet);
}

/** @brief Lấy địa chỉ node tiếp theo trong quá trình định tuyến.
 * @param dst Địa chỉ đích của gói tin.
 * @return Địa chỉ node tiếp theo.
 */
uint8_t get_next_hop(uint8_t dst) {
  return BROADCAST_ID;
}

/** @brief Kiểm tra xem tất cả các phân mảnh của một tin nhắn có được nhận đầy đủ hay không.
 * @param buffer Bộ đệm nhận tin nhắn.
 * @return - `TRUE` Nếu tất cả các phân mảnh đều được nhận.
 * @return - `FALSE` Nếu còn phân mảnh nào chưa được nhận.
 */
bool all_fragments_received(const RxMessageBuffer &buffer) {
  for (uint8_t i = 0; i < buffer.fragCount; ++i) {
    if (!buffer.received[i]) {
      return false;
    }
  }
  return true;
}

/**
 * @brief Chuẩn bị gói tin trước khi gửi bằng cách tính toán và điền giá trị CRC.
 * @param packet Gói tin cần chuẩn bị.
 * @note Hàm này sẽ đặt trường CRC của gói tin dựa trên nội dung của các trường khác. 
 * Việc này đảm bảo rằng người nhận có thể kiểm tra tính toàn vẹn của gói tin bằng cách so sánh CRC nhận được với CRC tính toán lại từ dữ liệu.
 */
bool send_raw_packet(LoRaPacket &packet) {
    packet.crc = packet_crc(packet);
    LoRa.idle(); // Đảm bảo radio ở trạng thái idle trước khi gửi
    LoRa.beginPacket();
    LoRa.write(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
    
    if(LoRa.endPacket()) {
        LoRa.receive(); // Quay lại chế độ nhận sau khi gửi
        return true;
    }
    txFragments++; // Tăng bộ đếm số phân mảnh đã gửi
    LoRa.receive(); // Quay lại chế độ nhận nếu gửi thất bại
    return false;
}

/**
 * @brief Đọc một gói tin từ radio LoRa.
 * @param packet Con trỏ đến cấu trúc LoRaPacket để lưu trữ gói tin đọc được.
 * @return - `TRUE` Nếu đọc được gói tin hợp lệ.
 * @return - `FALSE` Nếu không đọc được gói tin hoặc gói tin không hợp lệ.
 */
bool read_packet(LoRaPacket &packet) {
  int packetSize = LoRa.parsePacket();
  if (packetSize == 0) {
    return false;
  }

  if (packetSize != static_cast<int>(sizeof(LoRaPacket))) {
    while (LoRa.available()) {
      LoRa.read(); // Đọc và bỏ qua dữ liệu thừa trong FIFO nếu kích thước không khớp.
    }
    return false;
  }

  size_t bytesRead = LoRa.readBytes(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
  
  //so sánh số byte đọc được với kích thước của cấu trúc packet, đồng thời kiểm tra tính hợp lệ của gói tin qua CRC và các trường khác
  return bytesRead == sizeof(packet) && validate_packet(packet); //so sánh 
}

/**
 * @brief Chờ đợi một gói tin ACK hợp lệ từ node tiếp theo sau khi gửi một gói DATA.
 * @param expectedFrom ID của node mà mình mong đợi sẽ gửi ACK về (thường là nextHop của gói DATA vừa gửi).
 * @param seq Số thứ tự của gói DATA mà mình đang chờ ACK cho nó.
 * @param rttMs Tham số đầu ra để trả về thời gian trôi qua từ khi bắt đầu chờ đến khi nhận được ACK (Round-Trip Time). Sử dụng để tính delay đường truyền.
 * @return - `TRUE` Nếu nhận được ACK hợp lệ trong thời gian chờ.
 * @return - `FALSE` Nếu hết thời gian chờ mà không nhận được ACK hợp lệ nào.
 */
bool wait_for_ack(uint8_t expectedFrom, uint16_t seq, unsigned long &rttMs) {
    unsigned long startedAt = millis();
    
    while (millis() - startedAt < LORA_ACK_TIMEOUT_MS) {
        
      LoRaPacket packet;
        
        if(!read_packet(packet)){
            delay(5); // Nhường CPU cho các tác vụ khác nếu không có gói tin nào đến
            continue;
        }
        
        if(packet.type == LORA_PACKET_ACK &&
           packet.src == expectedFrom &&
           packet.dst == NODE_ID &&
           packet.seq == seq) {
            rttMs = millis() - startedAt;
            return true; // Nhận được ACK hợp lệ
        }
    }
    rttMs = millis() - startedAt;
    return false; // Hết giờ (Timeout)
}
/**
 * @brief Gửi một gói tin ACK để xác nhận đã nhận thành công một gói DATA.
 * @param receivedPacket Gói tin DATA vừa nhận được mà mình muốn trả lời ACK.
 * @note `ack` sẽ được gửi về node vừa gửi gói DATA này đến mình `prevHop`, 
 * với cùng số thứ tự `seq` và ID thông điệp `msgId` để người nhận dễ dàng đối chiếu.
 * Tuy nhiên, một vài trường của `ack` sẽ không được gửi đi nhằm tiết kiệm dung lượng gói.
 */
void send_ack(const LoRaPacket &receivedPacket) {
    
  LoRaPacket ack;
  
  ack = {
      .version = LORA_PROTOCOL_VERSION,
      .type = LORA_PACKET_ACK,
      .src = NODE_ID,
      .dst = receivedPacket.prevHop,
      .prevHop = NODE_ID,
      .nextHop = receivedPacket.prevHop,
      .seq = receivedPacket.seq,
      .msgId = receivedPacket.msgId,
      .fragIndex = receivedPacket.fragIndex,
      .fragCount = receivedPacket.fragCount,
      .ttl = 1,
      .flags = 0,
      .codec = LORA_CODEC_NONE,
      .rawLen = 0,
      .encodedBitLen = 0,
      .payloadLen = 0,
  };

  send_raw_packet(ack);

}

/**
 * @brief Gửi một gói tin DATA với cơ chế ARQ (Automatic Repeat reQuest) để đảm bảo độ tin cậy.
 * @param packet Gói tin DATA đã được chuẩn bị sẵn sàng để gửi.
 * @return - `TRUE` Nếu gói tin được gửi thành công và nhận được ACK hợp lệ (nếu yêu cầu).
 * @return - `FALSE` Nếu sau nhiều lần thử vẫn không gửi được gói tin hoặc không nhận được ACK hợp lệ.
 * @note Hàm này sẽ tự động lặp lại việc gửi gói tin lên đến `LORA_MAX_RETRY` lần nếu có yêu cầu ACK nhưng không nhận được phản hồi. 
 * Nếu gói tin không yêu cầu ACK, nó sẽ chỉ gửi một lần duy nhất.
 */
bool send_with_retry(LoRaPacket &packet) {
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

/**
 * @brief Đặt lại bộ đệm nhận (Rx Buffer) để chuẩn bị cho việc nhận một thông điệp mới.
 * @param packet Gói tin đầu tiên của thông điệp mới mà mình vừa nhận được. Thông tin từ gói này sẽ được dùng để khởi tạo bộ đệm.
 * @note Hàm này sẽ xóa sạch dữ liệu cũ trong `rxBuffer`, đánh dấu nó là đang hoạt động, và điền các trường như `src`, `msgId`, `codec`, `rawLen`, `encodedBitLen`, và `fragCount` dựa trên gói tin đầu tiên của thông điệp mới. 
 * Điều này giúp chuẩn bị sẵn sàng cho việc ghép mảnh (Reassembly) các gói tin tiếp theo của cùng một thông điệp.
 */
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

/**
 * @brief In ra các thông số chất lượng tín hiệu (Link Metric) của gói tin vừa nhận.
 * @note Các thông số bao gồm RSSI (Received Signal Strength Indicator) và SNR (Signal-to-Noise Ratio).
 */
void print_link_metric() {
  Serial.print("RSSI=");
  Serial.print(LoRa.packetRssi());
  Serial.print(" SNR=");
  Serial.println(LoRa.packetSnr());
}

/**
 * @brief Giải nén và in ra màn hình thông điệp hoàn chỉnh sau khi đã nhận đủ các mảnh.
 * @param encodedData Con trỏ đến bộ đệm chứa dữ liệu đã được ghép mảnh.
 * @param encodedBytes Tổng số byte của dữ liệu sau khi ghép.
 * @param codec Thuật toán nén đã sử dụng (VD: LORA_CODEC_STATIC_HUFFMAN hoặc LORA_CODEC_NONE).
 * @param rawLen Chiều dài thực tế của dữ liệu gốc trước khi nén.
 * @param encodedBitLen Chiều dài thực tế tính bằng bit (nếu dùng nén Huffman).
 */
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

/**
 * @brief Xử lý một mảnh dữ liệu (fragment) nhận được, lưu vào bộ đệm và kiểm tra xem đã đủ chưa.
 * @param packet Gói tin chứa mảnh dữ liệu vừa nhận.
 * @note Hàm này quản lý trạng thái của bộ đệm ghép mảnh (`rxBuffer`). Khi nhận đủ tất cả
 * các mảnh của một thông điệp, nó sẽ gọi hàm in ra kết quả cuối cùng.
 */
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

/**
 * @brief Chuyển tiếp (Relay) gói tin đi tiếp trong mạng Mesh (Flooding).
 * @param packet Gói tin cần được chuyển tiếp.
 * @note Hàm này đánh dấu node hiện tại là `prevHop`, đổi `nextHop` thành `BROADCAST_ID`,
 * thêm cờ `LORA_FLAG_RELAYED` và gửi gói tin đi tiếp với cơ chế ARQ.
 */
void relay_packet(LoRaPacket &packet) {
  packet.prevHop = NODE_ID;
  packet.nextHop = BROADCAST_ID; // Flooding
  packet.flags |= LORA_FLAG_RELAYED; // Đánh dấu đã được relay
  send_with_retry(packet);
}

/**
 * @brief In ra các thông số đánh giá hiệu quả của thuật toán nén.
 * @param codec Thuật toán nén đã sử dụng.
 * @param rawLen Chiều dài dữ liệu gốc (byte).
 * @param encodedBytes Chiều dài dữ liệu sau nén (byte).
 * @param encodedBits Chiều dài dữ liệu sau nén (bit).
 * @param entropy Giá trị entropy của dữ liệu gốc, giúp đánh giá mức độ nén tối ưu.
 */
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

} //namespace

// ====================================================================
//                          CÁC HÀM PUBLIC 
// ====================================================================

/**
 * @brief Đóng gói và gửi một chuỗi văn bản tới node đích.
 * @param dst Địa chỉ ID của node đích.
 * @param text Chuỗi ký tự (C-string) cần gửi.
 * @return true Nếu toàn bộ các mảnh đã được phát đi thành công.
 * @return false Nếu dữ liệu rỗng, quá dài, hoặc gửi thất bại (Packet Loss).
 * @note Hàm sẽ tính toán kích thước, nén dữ liệu (nếu tối ưu), 
 * @note cắt nhỏ thành các mảnh (fragment) và phát đi tuần tự.
 */
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
    packet.type = LORA_PACKET_DATA;
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

/**
 * @brief Đọc và xử lý các gói tin LoRa đến.
 * @note Hàm này thực hiện việc lọc gói lỗi, trả lời ACK, trung chuyển (Relay) và
 * ghép mảnh thông điệp. Cần được gọi liên tục bên trong hàm loop().
 */
void lora_process() {
  LoRaPacket packet;
  if (!read_packet(packet)) {
    return;
  }

  // Bỏ qua các gói do chính node này gửi ra nhưng lại vọng lại
  if (packet.src == NODE_ID) {
    return;
  }

  if (packet.type == LORA_PACKET_ACK) {
    return;
  }

  if (packet.type != LORA_PACKET_DATA) {
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