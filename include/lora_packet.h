/**
 * @file lora_packet.h
 * @brief Định nghĩa cấu trúc và hàm xử lý packet dữ liệu cho mạng LoRa multi-hop.
 * * File này quy định "ngôn ngữ chung" để các node trong mạng hiểu nhau, bao gồm 
 * định dạng header, các cờ điều khiển định tuyến và thuật toán nén dữ liệu.
 */
#pragma once

#include <stdint.h>


/**
 * @brief Địa chỉ đích đặc biệt dành cho thông điệp toàn mạng (Broadcast).
 * @note Giá trị 255 (0xFF) là mức tối đa của kiểu dữ liệu uint8_t. 
 * Trong mạng lưới, khi một gói tin có dst hoặc nextHop được gán bằng 255:
 * 1. Mọi node nhận được đều phải mở ra xử lý và chuyển tiếp (Relay).
 * 2. Cấm tuyệt đối việc trả lời ACK để tránh bão mạng (Collision/ACK Storm).
 */
constexpr uint8_t BROADCAST_ID = 255;

/**
 * @brief Phiên bản của giao thức LoRa Mesh.
 * @note Dùng để kiểm tra tính tương thích. Nếu node nhận thấy version khác
 * với cấu hình của nó, nó sẽ drop gói tin để tránh đọc sai cấu trúc bộ nhớ.
 */
constexpr uint8_t LORA_PROTOCOL_VERSION = 1;

/**
 * @brief Kích thước tối đa của payload thực tế trong mỗi lần phát sóng (64 bytes).
 * @note Mặc dù FIFO của chip SX1278 hỗ trợ tối đa 255 bytes, nhưng trong mạng 
 * Mesh, ta chủ động giới hạn ở 64 bytes vì 2 lý do:
 * @note 1. Giảm Time-on-Air (thời gian chiếm sóng), từ đó giảm xác suất đụng độ (Collision).
 * @note 2. Gói tin càng dài, xác suất xuất hiện lỗi bit (Bit Error) do nhiễu càng cao. 
 * Gói ngắn giúp tối ưu hóa Throughput thực tế và giảm thiểu việc phải truyền lại cả mảng lớn.
 */
constexpr uint8_t LORA_PAYLOAD_MAX = 64;

/**
 * @brief Kích thước tối đa của thông điệp gốc (text) từ ứng dụng.
 * @note Giới hạn ở 1024 bytes (1 KB) để phù hợp với bộ nhớ RAM (SRAM) 
 * hạn hẹp của vi điều khiển ESP32, đồng thời tránh việc phân mảnh 
 * thành quá nhiều gói tin.
 */
constexpr uint16_t LORA_MESSAGE_MAX = 1024;

/**
 * @brief Kích thước bộ đệm nháp (buffer) dùng khi chạy thử nén Huffman.
 * @note Mặc dù hệ thống có cơ chế "chống cháy" (chỉ gửi dữ liệu nén nếu kết quả 
 * nhỏ hơn dữ liệu gốc), hàm huffman_compress() vẫn cần một bộ đệm đủ lớn 
 * để "thử" ghi dữ liệu nén ra. Với các nguồn tin có Entropy cao, dữ liệu 
 * nén tạm thời có thể phình to hơn 1024. Cấp 1280 bytes giúp tránh lỗi 
 * tràn bộ nhớ (Buffer Overflow) trong quá trình tính toán.
 */
constexpr uint16_t LORA_ENCODED_MAX = 1280;

/**
 * @brief Số lượng mảnh ghép (fragment) tối đa cần tính toán.
 * * Tự động tính toán bằng công thức: (Max_Buffer + Payload - 1) / Payload.
 * Việc cộng thêm (LORA_PAYLOAD_MAX - 1) là mẹo làm tròn lên (Ceil) trong C++.
 */
constexpr uint8_t LORA_MAX_FRAGMENTS = (LORA_ENCODED_MAX + LORA_PAYLOAD_MAX - 1) / LORA_PAYLOAD_MAX;

/**
 * @brief Số bước nhảy (Hop) tối đa cho một gói tin.
 * @note Dùng để chống hiện tượng Bão mạng (Broadcast Storm) hoặc Vòng lặp vô hạn 
 * (Infinite Loop) trong mạng Flooding. Mỗi lần qua 1 node, TTL trừ 1. 
 * Con số 5 là đủ để phủ sóng một khu vực rộng với mạng Mesh dân dụng.
 */
constexpr uint8_t LORA_TTL_DEFAULT = 5;

/**
 * @brief Số lần tối đa cố gắng gửi lại (Retransmission limit).
 * @note Nằm trong cơ chế ARQ (Automatic Repeat reQuest). Giới hạn ở 3 lần để 
 * cân bằng giữa độ tin cậy (Reliability) và việc tránh làm tắc nghẽn kênh truyền.
 */
constexpr uint8_t LORA_MAX_RETRY = 3;

/**
 * @brief Thời gian chờ xác nhận (ACK Timeout) tính bằng mili-giây.
 * @note Ở cấu hình SF7, BW 125kHz, thời gian truyền một gói LoRa max payload 
 * tốn khoảng 100-150ms. Với mạng Multi-hop, gói ACK phải nhảy ngược qua nhiều 
 * trạm Relay (mỗi trạm delay 150-1000ms), nên RTT tối đa cho 4-5 chặng 
 * có thể lên đến 4-5 giây. Giá trị 5000ms đảm bảo đủ thời gian cho ACK quay về.
 */
constexpr unsigned long LORA_ACK_TIMEOUT_MS = 5000;
/**
 * @brief Phân loại mục đích của gói tin.
 * @brief - `LORA_PACKET_DATA`: Gói tin mang dữ liệu thực tế (Payload).
 * @brief - `LORA_PACKET_ACK`: Gói tin xác nhận (Acknowledge)
 */
enum LoRaPacketType : uint8_t {
    LORA_PACKET_DATA = 1, ///< Gói tin mang dữ liệu thực tế (Payload).
    LORA_PACKET_ACK = 2,  ///< Gói tin xác nhận (Acknowledge) báo đã nhận thành công.
    LORA_PACKET_START = 3,///< Gói tin báo hiệu bắt đầu phiên truyền.
    LORA_PACKET_END = 4,  ///< Gói tin báo hiệu kết thúc phiên truyền.
    LORA_PACKET_PING = 5  ///< Gói tin Heartbeat/Ping báo trạng thái Online.
};

/**
 * @brief Các cờ trạng thái (bitmask) điều khiển luồng định tuyến và độ tin cậy.
 * @brief - `LORA_FLAG_ACK_REQ`: Nếu được đặt, node nhận phải trả lời bằng gói ACK.
 * @brief - `LORA_FLAG_RELAYED`: Đánh dấu gói tin đã được
 * @note Việc sử dụng bitmask cho phép kết hợp nhiều cờ trong một byte flags, giúp tiết kiệm băng thông và vẫn linh hoạt trong việc mở rộng sau này nếu cần thêm cờ mới.
 */
enum LoRaPacketFlags : uint8_t {
    LORA_FLAG_ACK_REQ = 0x01, ///< Yêu cầu node tiếp theo phải trả lời bằng gói ACK (ARQ).
    LORA_FLAG_RELAYED = 0x02  ///< Đánh dấu gói tin này đã được trung chuyển (tránh đụng sóng).
};

/**
 * @brief Các thuật toán mã hóa nguồn (Source Coding) đang áp dụng:
 * 
 *1- `LORA_CODEC_NONE` Gửi dữ liệu thô (Raw byte), không áp dụng nén. Phù hợp với dữ liệu đã có Entropy cao hoặc khi độ trễ phải cực thấp.
 * 
 *2- `LORA_CODEC_STATIC_HUFFMAN` Áp dụng thuật toán nén tĩnh Huffman,
 * 
 * @note - Tùy vào đặc điểm dữ liệu đầu vào, hệ thống có thể lựa chọn thuật toán phù hợp để tối ưu hóa hiệu suất truyền tải.
 * @warning - Ta không thể sử dụng thuật toán nén động (Dynamic Huffman) vì nó yêu cầu phải gửi bảng mã (Codebook) kèm theo, sẽ làm tăng overhead và phức tạp hơn nhiều. Thuật toán tĩnh (Static) sử dụng một bảng mã đã được xây dựng sẵn dựa trên phân tích thống kê của tập dữ liệu mẫu, giúp giảm Entropy trung bình mà không cần gửi thêm thông tin về mã hóa.
 */
enum LoRaPayloadCodec : uint8_t {
    LORA_CODEC_NONE = 0,            ///< Gửi dữ liệu thô (Raw byte), không nén.
    LORA_CODEC_STATIC_HUFFMAN = 1   ///< Nén bằng thuật toán tĩnh Huffman (giảm Entropy).
};

// Ép compiler đóng gói sát các biến (1-byte alignment) để không bị lệch byte khi gửi qua sóng RF.
#pragma pack(push, 1)
/***
 * @brief Cấu trúc gói tin LoRa cơ bản (LoRaPacket).
 *
 * @brief - `version`: Phiên bản protocol (LORA_PROTOCOL_VERSION)
 * @brief - `type`: Loại gói tin (LoRaPacketType)
 * @brief - `src`: ID node gốc (người tạo ra tin nhắn)
 * @brief - `dst`: ID node đích cuối cùng
 * @brief - `prevHop`: ID node vừa chuyển tiếp (relay) gói này
 * @brief - `nextHop`: ID node tiếp theo sẽ nhận (hoặc BROADCAST_ID)
 * @brief - `seq`: Số thứ tự gói tin (dùng để chống trùng lặp)
 * @brief - `msgId`: ID của thông điệp (dùng để gom các mảnh fragment)
 * @brief - `fragIndex`: Vị trí của mảnh hiện tại (0, 1, 2...)
 * @brief - `fragCount`: Tổng số mảnh của thông điệp
 * @brief - `ttl`: Time-To-Live, số bước nhảy tối đa còn lại
 * @brief - `flags`: Cờ điều khiển (LoRaPacketFlags)
 * @brief - `codec`: Thuật toán nén đang dùng (LoRaPayloadCodec)
 * @brief - `rawLen`: Chiều dài dữ liệu gốc trước khi nén (byte)
 * @brief - `encodedBitLen`: Chiều dài thực tế sau khi nén (bit)
 * @brief - `payloadLen`: Số byte thực tế chứa trong mảng payload của gói này
 * @brief - `payload[]`: Mảng chứa dữ liệu (tối đa 64 bytes)
 * 
 * @note Mỗi gói tin được đóng gói theo cấu trúc này trước khi gửi qua sóng RF.
 */
struct LoRaPacket {
  uint8_t version;     ///< Phiên bản protocol (LORA_PROTOCOL_VERSION)
  uint8_t type;        ///< Loại gói tin (LoRaPacketType)
  uint8_t src;         ///< ID node gốc (người tạo ra tin nhắn)
  uint8_t dst;         ///< ID node đích cuối cùng
  uint8_t prevHop;     ///< ID node vừa chuyển tiếp (relay) gói này
  uint8_t nextHop;     ///< ID node tiếp theo sẽ nhận (hoặc BROADCAST_ID)
  uint16_t seq;        ///< Số thứ tự gói tin (dùng để chống trùng lặp)
  uint16_t msgId;      ///< ID của thông điệp (dùng để gom các mảnh fragment)
  uint8_t fragIndex;   ///< Vị trí của mảnh hiện tại (0, 1, 2...)
  uint8_t fragCount;   ///< Tổng số mảnh của thông điệp
  uint8_t ttl;         ///< Time-To-Live, số bước nhảy tối đa còn lại
  uint8_t flags;       ///< Cờ điều khiển (LoRaPacketFlags)
  uint8_t codec;       ///< Thuật toán nén đang dùng (LoRaPayloadCodec)
  uint16_t rawLen;     ///< Chiều dài dữ liệu gốc trước khi nén (byte)
  uint16_t encodedBitLen; ///< Chiều dài thực tế sau khi nén (bit)
  uint8_t payloadLen;  ///< Số byte thực tế chứa trong mảng payload của gói này
  uint8_t payload[LORA_PAYLOAD_MAX]; ///< Mảng chứa dữ liệu (tối đa 64 bytes)
};
#pragma pack(pop)

static_assert(sizeof(LoRaPacket) <= 255, "LoRaPacket must fit inside SX127x FIFO");

#define MAX_TX_QUEUE 5

/**
 * @brief Cấu trúc lưu trữ tin nhắn chờ gửi (TX Queue)
 */
struct TxMessage {
    bool active;
    uint8_t dst;
    char text[LORA_MESSAGE_MAX];
};

/**
 * @brief Đưa một chuỗi văn bản vào hàng đợi TX (TX Queue).
 * * Hàm này không gửi ngay, mà sẽ giao cho lora_process() xử lý khi rảnh (CSMA/LBT).
 * @param dst Địa chỉ ID của node đích.
 * @param text Chuỗi ký tự (C-string) cần gửi.
 */
void lora_queue_text(uint8_t dst, const char *text);

/**
 * @brief Đóng gói và gửi một chuỗi văn bản tới node đích (Nội bộ).
 * * Được gọi tự động bởi lora_process() khi xả hàng đợi.
 */
bool lora_send_text_internal(uint8_t dst, const char *text);

/**
 * @brief Khởi tạo các tham số ngẫu nhiên cho session (chống lỗi trùng lặp khi reset).
 */
void lora_packet_setup();


/**
 * @brief Đọc và xử lý các gói tin LoRa đến.
 * * Hàm này thực hiện:
 * @brief 1. Đọc gói tin từ phần cứng LoRa.
 * @brief 2. Lọc bỏ các gói lỗi CRC hoặc lặp lại (Deduplication).
 * @brief 3. Trả lời ACK (nếu gói yêu cầu).
 * @brief 4. Trung chuyển (Relay) nếu là mạng Mesh Flooding.
 * @brief 5. Ghép mảnh và giải mã Huffman để in ra thông điệp cuối cùng.
 * * @note Hàm này KHÔNG block (không dùng delay dài). Phải được gọi liên tục bên trong loop().
 */
void lora_process();