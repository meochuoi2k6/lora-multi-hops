/**
 * @file main.cpp
 * @brief Điểm vào (Entry point) của chương trình ESP32 LoRa Multi-Hop.
 * 
 * Khởi tạo phần cứng (Serial, LoRa, Bluetooth) và chứa vòng lặp chính (loop)
 * để liên tục xử lý các luồng dữ liệu vào/ra từ mạch.
 */

#include <Arduino.h>

#include "bluetooth_input.h"
#include "lora_packet.h"
#include "lora_setup.h"

/**
 * @brief Cấu hình ban đầu cho vi điều khiển ESP32.
 * 
 * Các công việc được thực hiện trong setup():
 * 1. Khởi tạo Serial port để debug.
 * 2. Khởi tạo bộ sinh số ngẫu nhiên (dùng cho thuật toán CSMA chống đụng độ sóng).
 * 3. Khởi tạo IC LoRa SX1278 (tần số, băng thông, công suất).
 * 4. Khởi tạo giao tiếp Bluetooth Classic để kết nối với App điện thoại.
 */
void setup() {
  Serial.begin(115200);
  delay(1000);

  // Tạo seed (mầm) cho hàm random từ phần cứng ESP32.
  // Mầm này giúp hàm random() tạo ra các độ trễ (Jitter) thực sự ngẫu nhiên,
  // ngăn chặn tình trạng nhiều node cùng phát sóng 1 lúc gây đụng độ (Collision).
  randomSeed(esp_random());

  Serial.println();
  Serial.print("=== NODE ID: ");
  Serial.print(NODE_ID);
  Serial.println(" ===");

  // Khởi động module LoRa. Nếu lỗi (hỏng mạch, lỏng dây nối SPI), hệ thống sẽ treo ở đây.
  if (!init_lora()) {
    while (true) {
      delay(1000);
    }
  }

  // Khởi tạo các tham số packet (nextSeq, msgId) ngẫu nhiên để chống lỗi drop nhầm gói cũ
  lora_packet_setup();

  Serial.println("LoRa mesh ready");

  // Khởi động Bluetooth để chờ kết nối từ App Flutter.
  if (!bluetooth_input_setup()) {
    while (true) {
      delay(1000);
    }
  }
}

/**
 * @brief Vòng lặp chính của vi điều khiển, chạy liên tục với tốc độ cao.
 * 
 * Có 2 nhiệm vụ chính chạy song song (non-blocking):
 * 1. Đọc và xử lý sóng LoRa.
 * 2. Đọc và xử lý dữ liệu từ App qua Bluetooth.
 */
void loop() {
  // 1. Quản lý LoRa: Nhận gói tin, gửi ACK, ghép mảnh (Reassembly), giải nén Huffman,
  // kiểm tra Timeout mạng lưới, và chuyển tiếp (Relay) tin nhắn nếu cần.
  lora_process();

  // 2. Quản lý Bluetooth: Nhận luồng text từ App, cắt thành các khung (framing)
  // và đẩy vào hàm lora_send_text để phát sóng đi.
  bluetooth_input_process();
}

