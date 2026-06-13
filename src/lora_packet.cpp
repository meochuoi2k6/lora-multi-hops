#include "lora_packet.h"
#include "lora_setup.h"
#include <Arduino.h>
#include <stdint.h>
#include <SPI.h>
#include <LoRa.h>
#include <static_huffman.h>
#include "bluetooth_input.h"

#include <string.h> // Để dùng memcpy, memset

// ====================================================================
//                          CÁC HÀM PRIVATE 
// ====================================================================
namespace {

/// Biến đếm số thứ tự gói tin (Sequence Number) để chống trùng lặp.
/// Sẽ được gán giá trị random lúc khởi động để tránh trùng lặp sau khi reset mạch.
uint16_t nextSeq = 1;

void process_incoming_packet(LoRaPacket packet);

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

/// Bộ đệm vòng (Circular Buffer) lưu lại 64 gói tin gần nhất đã nghe thấy.
SeenPacket seenPackets[64] = {};

/**
 * @brief Kích thước tối đa của Hàng đợi thu (RX Queue).
 * @note Giúp lưu trữ tạm thời các gói tin bay đến trong lúc mạch đang bận (ví dụ: đang chờ ACK 
 * hoặc đang thực hiện delay nhường sóng). 15 gói là đủ để hứng các vụ nổ sóng (burst).
 */
#define MAX_RX_QUEUE 15

LoRaPacket rxPacketQueue[MAX_RX_QUEUE]; ///< Mảng chứa các gói tin trong hàng đợi
uint8_t rxQueueHead = 0;                ///< Vị trí lấy gói tin ra (Pop)
uint8_t rxQueueTail = 0;                ///< Vị trí đẩy gói tin vào (Push)
uint8_t rxQueueCount = 0;               ///< Số lượng gói tin hiện có trong hàng đợi

/**
 * @brief Đưa một gói tin vừa nhận được vào Hàng đợi thu (RX Queue).
 * @param packet Gói tin LoRa thô vừa đọc từ module SX1278.
 * @return `true` nếu đẩy vào thành công, `false` nếu hàng đợi đã đầy (bị rơi gói).
 */
bool enqueue_rx_packet(const LoRaPacket &packet) {
    if (rxQueueCount >= MAX_RX_QUEUE) {
        Serial.println("RX Queue FULL! Packet dropped.");
        return false;
    }
    rxPacketQueue[rxQueueTail] = packet;
    rxQueueTail = (rxQueueTail + 1) % MAX_RX_QUEUE;
    rxQueueCount++;
    return true;
}

/**
 * @brief Lấy một gói tin ra khỏi Hàng đợi thu (RX Queue) để xử lý.
 * @param packet Biến chứa kết quả trả về.
 * @return `true` nếu lấy thành công, `false` nếu hàng đợi đang rỗng.
 */
bool dequeue_rx_packet(LoRaPacket &packet) {
    if (rxQueueCount == 0) return false;
    packet = rxPacketQueue[rxQueueHead];
    rxQueueHead = (rxQueueHead + 1) % MAX_RX_QUEUE;
    rxQueueCount--;
    return true;
}

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
 * @brief - `lastUpdate`: Thời gian nhận mảnh cuối cùng (dùng để timeout dump queue)
 * @note Vì LoRa chỉ gửi tối đa 64 byte mỗi lần, một thông điệp dài 
 * sẽ bị cắt làm nhiều gói (fragment). Bộ đệm này giữ các mảnh lại cho 
 * đến khi nhận đủ thì mới giải mã toàn bộ.
 */
struct RxMessageBuffer {
  bool active;                           ///< Có đang trong quá trình nhận dở dang một tin nhắn không?
  uint8_t src;                           ///< ID của người đang gửi tin nhắn này
  uint8_t dst;                           ///< ID đích
  uint16_t msgId;                        ///< ID của tin nhắn đang nhận dở
  uint8_t codec;                         ///< Thuật toán nén của tin nhắn này
  uint16_t rawLen;                       ///< Chiều dài dữ liệu gốc
  uint16_t encodedBitLen;                ///< Chiều dài bit sau nén
  uint8_t fragCount;                     ///< Tổng số mảnh phải nhận
  bool received[LORA_MAX_FRAGMENTS];     ///< Mảng đánh dấu (true/false) xem mảnh nào đã nhận được
  uint8_t data[LORA_ENCODED_MAX];        ///< Bộ đệm chứa dữ liệu thô của các mảnh ghép lại
  unsigned long lastUpdate;              ///< Thời gian nhận mảnh cuối cùng (dùng để timeout dump queue)
  bool completed;                        ///< Đánh dấu đã in ra màn hình chưa
};

/// Hỗ trợ nhận đồng thời từ 4 người gửi khác nhau
constexpr uint8_t MAX_RX_BUFFERS = 4;
RxMessageBuffer rxBuffers[MAX_RX_BUFFERS] = {};

/**
 * @brief Tìm bộ đệm đang ghép dở thông điệp của một node
 */
RxMessageBuffer* get_rx_buffer(uint8_t src, uint16_t msgId) {
  for (uint8_t i = 0; i < MAX_RX_BUFFERS; ++i) {
    if (rxBuffers[i].active && rxBuffers[i].src == src && rxBuffers[i].msgId == msgId) {
      return &rxBuffers[i];
    }
  }
  return nullptr;
}

/**
 * @brief Cấp phát bộ đệm mới. Nếu đầy sẽ hất văng (evict) bộ đệm cũ nhất.
 */
RxMessageBuffer* allocate_rx_buffer() {
  for (uint8_t i = 0; i < MAX_RX_BUFFERS; ++i) {
    if (!rxBuffers[i].active) {
      return &rxBuffers[i];
    }
  }
  
  uint8_t oldestIdx = 0;
  unsigned long oldestTime = rxBuffers[0].lastUpdate;
  for (uint8_t i = 1; i < MAX_RX_BUFFERS; ++i) {
    if (rxBuffers[i].lastUpdate < oldestTime) {
      oldestTime = rxBuffers[i].lastUpdate;
      oldestIdx = i;
    }
  }
  
  Serial.print("RX Buffers full! Evicting old buffer of src=");
  Serial.print(rxBuffers[oldestIdx].src);
  Serial.print(" msgId=");
  Serial.println(rxBuffers[oldestIdx].msgId);
  
  rxBuffers[oldestIdx].active = false;
  return &rxBuffers[oldestIdx];
}

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
        } else {
            enqueue_rx_packet(packet); // Không bỏ đi mà cất vào RX Queue
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
      .dst = receivedPacket.src, // Gửi ACK về tận người tạo ra gói tin
      .prevHop = NODE_ID,
      .nextHop = BROADCAST_ID, // Flooding ACK trên đường về
      .seq = receivedPacket.seq,
      .msgId = receivedPacket.msgId,
      .fragIndex = receivedPacket.fragIndex,
      .fragCount = receivedPacket.fragCount,
      .ttl = 5, // Cho phép ACK nhảy tối đa 5 bước (Multi-hop ACK)
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
  if (packet.dst == BROADCAST_ID) {
     packet.flags &= ~LORA_FLAG_ACK_REQ; // Chỉ tắt ACK nếu đích thực sự là Broadcast (Group Chat)
  }

  for (uint8_t attempt = 0; attempt < LORA_MAX_RETRY; ++attempt) {
    // Tăng random delay (Jitter) kết hợp với CSMA cơ bản để tránh đụng độ sóng
    if (packet.flags & LORA_FLAG_RELAYED) {
       delay(random(150, 1000)); // Khoảng random rộng hơn cho relay để tránh bão broadcast
    } else {
       delay(random(50, 200));
    }

    if (!send_raw_packet(packet)) {
      continue;
    }

    if ((packet.flags & LORA_FLAG_ACK_REQ) == 0) {
      return true; // Không yêu cầu ACK thì gửi 1 lần là xong
    }

    unsigned long rttMs = 0;
    // Đợi ACK trực tiếp từ đích đến (End-to-End ACK) thay vì từ nextHop
    if (wait_for_ack(packet.dst, packet.seq, rttMs)) {
      Serial.print("ACK seq=");
      Serial.print(packet.seq);
      Serial.print(" from=");
      Serial.print(packet.dst);
      Serial.print(" rtt_ms=");
      Serial.println(rttMs);

      bluetooth_input_print_metric_ack(rttMs);
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
void reset_rx_buffer(RxMessageBuffer* buf, const LoRaPacket &packet) {
  memset(buf, 0, sizeof(RxMessageBuffer));
  buf->active = true;
  buf->src = packet.src;
  buf->dst = packet.dst;
  buf->msgId = packet.msgId;
  buf->codec = packet.codec;
  buf->rawLen = packet.rawLen;
  buf->encodedBitLen = packet.encodedBitLen;
  buf->fragCount = packet.fragCount;
  buf->lastUpdate = millis();
  buf->completed = false;
}

/**
 * @brief In ra các thông số chất lượng tín hiệu (Link Metric) của gói tin vừa nhận.
 * @note Các thông số bao gồm RSSI (Received Signal Strength Indicator) và SNR (Signal-to-Noise Ratio).
 */
void print_link_metric() {
  int rssi = LoRa.packetRssi();
  float snr = LoRa.packetSnr();
  Serial.print("RSSI=");
  Serial.print(rssi);
  Serial.print(" SNR=");
  Serial.println(snr);
}

/**
 * @brief Giải nén và in ra màn hình thông điệp hoàn chỉnh sau khi đã nhận đủ các mảnh.
 * @param encodedData Con trỏ đến bộ đệm chứa dữ liệu đã được ghép mảnh.
 * @param encodedBytes Tổng số byte của dữ liệu sau khi ghép.
 * @param codec Thuật toán nén đã sử dụng (VD: LORA_CODEC_STATIC_HUFFMAN hoặc LORA_CODEC_NONE).
 * @param rawLen Chiều dài thực tế của dữ liệu gốc trước khi nén.
 * @param encodedBitLen Chiều dài thực tế tính bằng bit (nếu dùng nén Huffman).
 */
void print_complete_message(const RxMessageBuffer* buf) {
  uint8_t decoded[LORA_MESSAGE_MAX + 1] = {};
  bool ok = false;

  uint16_t encodedBytes = buf->encodedBitLen > 0 ? (buf->encodedBitLen + 7) / 8 : 0;
  if (buf->codec == LORA_CODEC_NONE) {
    encodedBytes = buf->rawLen;
  }

  if (buf->codec == LORA_CODEC_STATIC_HUFFMAN) {
    ok = huffman_decompress(buf->data, buf->encodedBitLen, decoded, buf->rawLen);
  } else {
    if (buf->rawLen <= encodedBytes && buf->rawLen <= LORA_MESSAGE_MAX) {
      memcpy(decoded, buf->data, buf->rawLen);
      ok = true;
    } else {
      Serial.println("Decode error: rawLen invalid for LORA_CODEC_NONE");
    }
  }

  if (!ok) {
    Serial.println("Decode failed");
    return;
  }

  decoded[buf->rawLen] = '\0';

  Serial.print("MESSAGE from node ");
  Serial.print(buf->src);
  Serial.print(" msgId=");
  Serial.print(buf->msgId);
  Serial.print(" text=");
  Serial.println(reinterpret_cast<char *>(decoded));

  // Đo lường Packet Loss ở máy thu (RX side) dành cho Broadcast Mode và Dev Mode
  Serial.print("RX_METRIC: fragments_received=");
  Serial.print(buf->fragCount);
  Serial.print("/");
  Serial.print(buf->fragCount);
  Serial.println(" (0% packet loss)");
  bluetooth_input_print_received(buf->src, buf->dst, buf->msgId, reinterpret_cast<char *>(decoded));
  bluetooth_input_print_metric_rx_loss(buf->fragCount, buf->fragCount);
  bluetooth_input_print_metric_rx(LoRa.packetRssi(), LoRa.packetSnr());
}

/**
 * @brief Xử lý một mảnh dữ liệu (fragment) nhận được, lưu vào bộ đệm và kiểm tra xem đã đủ chưa.
 * @param packet Gói tin chứa mảnh dữ liệu vừa nhận.
 * @note Hàm này quản lý trạng thái của bộ đệm ghép mảnh (`rxBuffer`). Khi nhận đủ tất cả
 * các mảnh của một thông điệp, nó sẽ gọi hàm in ra kết quả cuối cùng.
 */
void handle_final_fragment(const LoRaPacket &packet) {
  RxMessageBuffer* buf = get_rx_buffer(packet.src, packet.msgId);
  if (!buf) {
    buf = allocate_rx_buffer();
    reset_rx_buffer(buf, packet);
  }

  buf->lastUpdate = millis();

  uint16_t offset = static_cast<uint16_t>(packet.fragIndex) * LORA_PAYLOAD_MAX;
  if (offset + packet.payloadLen > LORA_ENCODED_MAX) {
    Serial.println("Drop fragment: reassembly buffer overflow");
    return;
  }

  memcpy(buf->data + offset, packet.payload, packet.payloadLen);
  buf->received[packet.fragIndex] = true;

  Serial.print("RX frag ");
  Serial.print(packet.fragIndex + 1);
  Serial.print("/");
  Serial.print(packet.fragCount);
  Serial.print(" msgId=");
  Serial.println(packet.msgId);

  if (!all_fragments_received(*buf)) {
    return;
  }

  // Chỉ in ra 1 lần khi vừa nhận đủ mảnh cuối cùng
  // Không set buf->active = false ở đây để giữ cho is_channel_busy() hoạt động chính xác
  // cho đến khi gói END thực sự được nhận.
  if (!buf->completed) {
      print_complete_message(buf);
      buf->completed = true;
  }
}

/**
 * @brief Chuyển tiếp (Relay) gói tin đi tiếp trong mạng Mesh (Flooding).
 * @param packet Gói tin cần được chuyển tiếp.
 * @note Hàm này đánh dấu node hiện tại là `prevHop`, đổi `nextHop` thành `BROADCAST_ID`,
 * thêm cờ `LORA_FLAG_RELAYED` và gửi gói tin đi tiếp với cơ chế ARQ.
 */
void relay_packet(LoRaPacket &packet) {
  if (packet.ttl <= 1) {
    return; // Dừng relay nếu gói tin đã hết hạn TTL
  }
  packet.ttl--; // Giảm TTL sau mỗi chặng
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
  float ratio = static_cast<float>(encodedBytes) / static_cast<float>(rawLen);
  Serial.println(ratio, 3);

  bluetooth_input_print_metric_tx(entropy, ratio);
}


/**
 * @brief Xử lý logic cho một gói tin vừa lấy ra từ phần cứng hoặc RX Queue.
 * @param packet Gói tin cần xử lý.
 * @note Hàm này đảm nhiệm các chức năng cốt lõi: lọc gói rác, gửi ACK, kiểm tra trùng lặp (Deduplication),
 * ghép mảnh thông điệp (Reassembly) và chuyển tiếp gói (Relay) cho mạng Mesh.
 */
void process_incoming_packet(LoRaPacket packet) {
  // Bỏ qua các gói do chính node này gửi ra nhưng lại vọng lại
  if (packet.src == NODE_ID) {
    return;
  }

  if (packet.type == LORA_PACKET_ACK) {
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
    if (packet.type == LORA_PACKET_PING) {
      Serial.print("RX: LORA_PACKET_PING from ");
      Serial.println(packet.src);
      bluetooth_input_print_ping(packet.src);
    } else if (packet.type == LORA_PACKET_START) {
      Serial.println("RX: LORA_PACKET_START");
      RxMessageBuffer* buf = get_rx_buffer(packet.src, packet.msgId);
      if (!buf) {
        buf = allocate_rx_buffer();
      }
      reset_rx_buffer(buf, packet);
    } else if (packet.type == LORA_PACKET_END) {
      Serial.println("RX: LORA_PACKET_END");
      RxMessageBuffer* buf = get_rx_buffer(packet.src, packet.msgId);
      if (buf) {
        if (!all_fragments_received(*buf)) {
          Serial.print("Packet Loss at RX: Missing fragments for msgId=");
          Serial.println(packet.msgId);
          uint8_t missing = 0;
          for(int i = 0; i < buf->fragCount; i++){
              if(!buf->received[i]) {
                  missing++;
                  Serial.print(" - Missed frag: ");
                  Serial.println(i + 1);
              }
          }
          Serial.print("Total missed: "); 
          Serial.print(missing);
          Serial.print("/");
          Serial.println(buf->fragCount);
          
          uint8_t receivedCount = buf->fragCount - missing;
          bluetooth_input_print_received(buf->src, buf->dst, buf->msgId, "<Lỗi: Mất gói tin do nhiễu sóng>");
          bluetooth_input_print_metric_rx_loss(receivedCount, buf->fragCount);
          bluetooth_input_print_metric_rx(LoRa.packetRssi(), LoRa.packetSnr());

          buf->active = false; // Dump queue
        }
      }
    } else if (packet.type == LORA_PACKET_DATA) {
      handle_final_fragment(packet);
    }
  }

  // Nếu gói tin không gửi đích danh cho chính mình, ta sẽ đóng vai trò trạm trung chuyển (Unicast Flooding)
  if (packet.dst != NODE_ID) {
    relay_packet(packet);
  }
}

} //namespace

// ====================================================================
//                          CÁC HÀM PUBLIC 
// ====================================================================

/**
 * @brief Khởi tạo các tham số ngẫu nhiên cho session (chống lỗi trùng lặp khi reset).
 */
void lora_packet_setup() {
    // Khởi tạo Seq và MsgId ngẫu nhiên để khi mạch bị reset (mất điện), 
    // các gói tin mới sinh ra sẽ không bị trùng số thứ tự với các gói tin cũ 
    // còn sót lại trong bộ nhớ cache của các node khác trong mạng.
    nextSeq = random(1000, 60000);
    nextMsgId = random(1000, 60000);
    Serial.printf("Session init: nextSeq=%d, nextMsgId=%d\n", nextSeq, nextMsgId);
}


/**
 * @brief Đóng gói và gửi một chuỗi văn bản tới node đích.
 * @param dst Địa chỉ ID của node đích.
 * @param text Chuỗi ký tự (C-string) cần gửi.
 * @return true Nếu toàn bộ các mảnh đã được phát đi thành công.
 * @return false Nếu dữ liệu rỗng, quá dài, hoặc gửi thất bại (Packet Loss).
 * @note Hàm sẽ tính toán kích thước, nén dữ liệu (nếu tối ưu), 
 * @note cắt nhỏ thành các mảnh (fragment) và phát đi tuần tự.
 */
TxMessage txQueue[MAX_TX_QUEUE];
unsigned long lastTxTime = 0;

void lora_queue_text(uint8_t dst, const char *text) {
  for (int i = 0; i < MAX_TX_QUEUE; ++i) {
    if (!txQueue[i].active) {
      txQueue[i].dst = dst;
      strncpy(txQueue[i].text, text, LORA_MESSAGE_MAX - 1);
      txQueue[i].text[LORA_MESSAGE_MAX - 1] = '\0';
      txQueue[i].active = true;
      Serial.print("Đã đưa tin nhắn vào TX Queue tại vị trí: ");
      Serial.println(i);
      return;
    }
  }
  Serial.println("Lỗi: TX Queue đã đầy!");
}

bool is_channel_busy() {
  // Điều kiện 1: Đang có RX Buffer hoạt động trong 3 giây gần đây
  for (uint8_t i = 0; i < MAX_RX_BUFFERS; ++i) {
    if (rxBuffers[i].active && (millis() - rxBuffers[i].lastUpdate < 3000)) {
      return true;
    }
  }
  // Điều kiện 2: Đo năng lượng sóng vật lý (LBT)
  int currentRssi = LoRa.rssi();
  if (currentRssi > -85) {
    return true; // Kênh đang bị chiếm dụng
  }
  return false;
}

bool lora_send_text_internal(uint8_t dst, const char *text) {
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

  // Phát gói START
  LoRaPacket startPacket = {};
  startPacket.version = LORA_PROTOCOL_VERSION;
  startPacket.type = LORA_PACKET_START;
  startPacket.src = NODE_ID;
  startPacket.dst = dst;
  startPacket.prevHop = NODE_ID;
  startPacket.nextHop = get_next_hop(dst);
  startPacket.seq = nextSeq++;
  startPacket.msgId = msgId;
  startPacket.fragIndex = 0;
  startPacket.fragCount = fragCount;
  startPacket.ttl = LORA_TTL_DEFAULT;
  startPacket.flags = LORA_FLAG_ACK_REQ;
  startPacket.codec = codec;
  startPacket.rawLen = rawLen;
  startPacket.encodedBitLen = encodedBits;
  startPacket.payloadLen = 0;
  if (!send_with_retry(startPacket) && startPacket.dst != BROADCAST_ID) {
     Serial.println("TX Failed: START packet no ACK. Dumping TX queue!");
     return false;
  }

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

    if (dst == BROADCAST_ID) {
      // Cho các node trung gian (relay) thời gian rảnh để phát lại (forward) gói tin trước đó (START hoặc DATA cũ).
      delay(1500);
    }

    txFragments++;
    if (!send_with_retry(packet)) {
      txFailedFragments++;
      allOk = false;
      if (dst != BROADCAST_ID) {
          Serial.println("TX Failed: No ACK. Dumping remaining TX queue!");
          break; // Dump queue khi gửi mà ko có phản hồi
      }
    }
  }

  // Phát gói END
  if (allOk) {
    if (dst == BROADCAST_ID) {
      // Tương tự, đợi các node relay xong gói DATA cuối cùng rồi mới bồi gói END
      delay(1500);
    }
    LoRaPacket endPacket = startPacket;
    endPacket.type = LORA_PACKET_END;
    endPacket.seq = nextSeq++;
    send_with_retry(endPacket);
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
  static unsigned long lastPingTime = millis();
  static unsigned long txBackoffTime = 0;
  static bool isBackingOff = false;

  // Phát Ping ngầm mỗi 3 phút
  if (millis() - lastPingTime > 180000) {
    lastPingTime = millis();
    LoRaPacket ping = {};
    ping.version = LORA_PROTOCOL_VERSION;
    ping.type = LORA_PACKET_PING;
    ping.src = NODE_ID;
    ping.dst = BROADCAST_ID;
    ping.prevHop = NODE_ID;
    ping.nextHop = BROADCAST_ID;
    ping.seq = nextSeq++;
    ping.msgId = nextMsgId++;
    ping.fragIndex = 0;
    ping.fragCount = 1;
    ping.ttl = 3;
    ping.flags = 0;
    ping.codec = LORA_CODEC_NONE;
    ping.rawLen = 0;
    ping.encodedBitLen = 0;
    ping.payloadLen = 0;
    send_with_retry(ping);
  }

  // Xả hàng đợi TX (Flush TX Queue) với Rate Limiting 5s và CSMA/CA LBT
  bool has_tx = false;
  for (int i = 0; i < MAX_TX_QUEUE; ++i) {
    if (txQueue[i].active) {
      has_tx = true;
      break;
    }
  }

  if (has_tx && (millis() - lastTxTime >= 5000)) {
    if (is_channel_busy()) {
      // Channel bận -> Đặt lịch Backoff tương lai (Random 1-3s)
      txBackoffTime = millis() + random(1000, 3000);
      isBackingOff = true;
    } else {
      if (isBackingOff) {
        // Đang chờ đếm ngược Backoff
        if (millis() >= txBackoffTime) {
          isBackingOff = false;
          // Xả hàng đợi!
          for (int i = 0; i < MAX_TX_QUEUE; ++i) {
            if (txQueue[i].active) {
              Serial.println("Channel is FREE. Xả hàng đợi sau khi Backoff...");
              txQueue[i].active = false;
              lora_send_text_internal(txQueue[i].dst, txQueue[i].text);
              lastTxTime = millis();
              break;
            }
          }
        }
      } else {
        // Không bận và không backoff -> Gửi luôn với Jitter siêu nhỏ
        delay(random(10, 50));
        if (!is_channel_busy()) {
          for (int i = 0; i < MAX_TX_QUEUE; ++i) {
            if (txQueue[i].active) {
              Serial.println("Channel is FREE. Xả hàng đợi TX Queue lập tức...");
              txQueue[i].active = false;
              lora_send_text_internal(txQueue[i].dst, txQueue[i].text);
              lastTxTime = millis();
              break;
            }
          }
        } else {
          // Bị chen ngang trong lúc Jitter
          txBackoffTime = millis() + random(1000, 3000);
          isBackingOff = true;
        }
      }
    }
  }

  // Kiểm tra Timeout (Dump Queue) liên tục trên toàn bộ các bộ đệm RX
  for (uint8_t i = 0; i < MAX_RX_BUFFERS; ++i) {
    if (rxBuffers[i].active && (millis() - rxBuffers[i].lastUpdate > 10000)) {
       Serial.print("Packet Loss: Timeout 10s. Dump queue for src=");
       Serial.println(rxBuffers[i].src);
       rxBuffers[i].active = false;
    }
  }

  // 1. Xả hàng đợi RX (các gói đến trong lúc đang busy)
  LoRaPacket queuedPacket;
  while (dequeue_rx_packet(queuedPacket)) {
    process_incoming_packet(queuedPacket);
  }

  // 2. Đọc gói tin mới từ phần cứng LoRa
  LoRaPacket packet;
  if (read_packet(packet)) {
    process_incoming_packet(packet);
  }
}