#pragma once

#include <stdint.h>

/**
 * @file lora_setup.h
 * @brief Khai báo các chân kết nối và hàm khởi tạo module vô tuyến LoRa.
 * * File này quản lý giao tiếp vật lý giữa vi điều khiển ESP32 và IC RF SX127x 
 * thông qua bus SPI.
 */

/// @brief Chân cấp xung nhịp đồng bộ cho bus SPI.
#define SCK_PIN 18

/// @brief Chân nhận dữ liệu từ module LoRa về vi điều khiển.
#define MISO_PIN 19

/// @brief Chân truyền dữ liệu từ vi điều khiển sang module LoRa.
#define MOSI_PIN 23

/// @brief Chân Chip Select (Slave Select) để kích hoạt module LoRa trên bus SPI.
#define NSS_PIN 5

/// @brief Chân phần cứng để hard-reset IC LoRa.
#define RST_PIN 14

/// @brief Chân ngắt (Interrupt) báo hiệu sự kiện TxDone hoặc RxDone từ LoRa.
#define DIO0_PIN 2

/**
 * @brief Tần số hoạt động của module LoRa.
 *
 * Hệ thống hiện sử dụng băng tần ISM 433 MHz,
 * cụ thể là `433.05 - 434.79 MHz` với tần số
 * trung tâm phổ biến là `433.92 MHz`.
 *
 * Theo quy định tại Việt Nam, đây là băng tần
 * vô tuyến công suất thấp (SRD/ISM) được phép
 * sử dụng không cần cấp giấy phép trong giới
 * hạn công suất cho phép.
 *
 * Giá trị được truyền trực tiếp cho:
 * `LoRa.begin(LORA_FREQUENCY)`.
 *
 * @note
 * Các băng tần LoRa phổ biến khác:
 * - `433 MHz`
 * - `868 MHz`
 * - `920 - 925 MHz`
 *
 * @warning
 * Băng `920 - 925 MHz` tại Việt Nam yêu cầu
 * sử dụng kỹ thuật trải phổ nhảy tần (FHSS)
 * theo quy chuẩn LPWAN hiện hành.
 */
constexpr uint32_t LORA_FREQUENCY = 433000000UL;

/**
 * @brief Tần số SPI dùng để giao tiếp với module SX1278.
 *
 * Theo datasheet của SX1278, giao tiếp SPI hỗ trợ
 * xung clock tối đa khoảng `10 MHz`.
 *
 * Hệ thống hiện sử dụng `8 MHz` nhằm cân bằng giữa:
 * - tốc độ truyền dữ liệu
 * - độ ổn định tín hiệu
 *
 * Giá trị này phù hợp với đa số module LoRa SX1278
 * phổ biến khi sử dụng cùng ESP32.
 *
 * @note
 * Nếu sử dụng breadboard, dây jumper dài hoặc module
 * clone chất lượng thấp, giao tiếp SPI có thể trở nên
 * không ổn định ở tần số cao.
 *
 * Khi gặp lỗi như:
 * - đọc sai register
 * - CRC lỗi
 * - packet corruption
 * - LoRa init thất bại ngẫu nhiên
 *
 * nên giảm xuống:
 * - 4 MHz
 * - 2 MHz
 */
constexpr uint32_t SPI_FREQUENCY = 8000000UL;

/**
 * @brief Spreading Factor (SF) của LoRa.
 *
 * SF quyết định:
 * - tốc độ truyền dữ liệu
 * - độ nhạy thu
 * - khoảng cách truyền
 *
 * SF càng lớn:
 * - truyền càng chậm
 * - khoảng cách càng xa
 * - khả năng chống nhiễu càng tốt
 *
 * Giá trị hiện tại `SF7` ưu tiên:
 * - tốc độ cao
 * - độ trễ thấp
 * - phù hợp cho mạng multi-hop cự ly gần và trung bình
 *
 * Giá trị hợp lệ: `7 - 12`
 */
constexpr uint8_t SPREADING_FACTOR = 7;

/**
 * @brief Băng thông tín hiệu LoRa (Hz).
 *
 * Băng thông càng lớn:
 * - tốc độ truyền càng cao
 * - thời gian on-air càng ngắn
 *
 * Tuy nhiên:
 * - độ nhạy thu sẽ giảm
 * - khoảng cách truyền có thể ngắn hơn
 *
 * Giá trị `125 kHz` là cấu hình phổ biến và
 * cân bằng tốt giữa tốc độ và độ ổn định.
 */
constexpr uint32_t SIGNAL_BANDWIDTH = 125000UL;

/**
 * @brief Coding Rate (CR) của LoRa.
 *
 * LoRa sử dụng cơ chế Forward Error Correction (FEC)
 * để tăng khả năng phát hiện và sửa lỗi.
 *
 * Giá trị hiện tại `5` tương ứng với:
 * `4/5`
 *
 * Nghĩa là:
 * - cứ 4 bit dữ liệu thực
 * - sẽ thêm 1 bit sửa lỗi
 *
 * Coding Rate càng cao:
 * - độ tin cậy càng tốt
 * - nhưng tốc độ truyền sẽ giảm
 *
 * Giá trị hợp lệ: `5 - 8`
 * tương ứng:
 * - 4/5
 * - 4/6
 * - 4/7
 * - 4/8
 */
constexpr uint8_t CODING_RATE_PER_4_BITS = 5;

/**
 * @brief Sync Word của mạng LoRa.
 *
 * Sync Word hoạt động như mã nhận diện mạng.
 * Node chỉ nhận packet có cùng Sync Word.
 *
 * Giá trị `0x12` thường được sử dụng cho:
 * - mạng LoRa private
 * - ứng dụng tùy chỉnh
 *
 * Trong khi:
 * - `0x34` thường dùng cho LoRaWAN.
 */
constexpr uint8_t SYNC_WORD = 0x12;

/**
 * @brief Độ dài preamble của packet LoRa.
 *
 * Preamble là chuỗi đồng bộ được gửi trước payload
 * để receiver phát hiện và khóa tín hiệu.
 *
 * Preamble dài hơn:
 * - giúp bắt tín hiệu dễ hơn
 * - cải thiện độ ổn định ở khoảng cách xa
 *
 * Tuy nhiên:
 * - làm tăng thời gian on-air
 * - giảm throughput thực tế
 *
 * Giá trị `8` là cấu hình phổ biến cho
 * đa số ứng dụng LoRa thông thường.
 */
constexpr uint8_t PREAMBLE_LENGTH = 8;


/**
 * @brief Khởi tạo và cấu hình các thông số vô tuyến cho module LoRa.
 * * Hàm này thiết lập bus SPI, liên kết các chân điều khiển, khởi động LoRa ở 
 * dải tần 433 MHz và cấu hình các thông số vật lý (SF, BW, CR) phù hợp cho mạng lưới (mesh).
 * * @return true Nếu khởi tạo và tìm thấy IC LoRa thành công.
 * @return false Nếu không giao tiếp được với IC LoRa (thường do lỗi phần cứng/dây dẫn).
 * * @note Bắt buộc phải gọi hàm này trong setup() trước khi thực hiện truyền/nhận dữ liệu.
 */
bool init_lora();