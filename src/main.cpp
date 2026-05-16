#include <Arduino.h>

#include "lora_packet.h"
#include "lora_setup.h"

String inputBuffer = "";

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Dùng random delay trước khi gửi/retry để giảm khả năng nhiều node đụng sóng.
  randomSeed(esp_random());

  Serial.println();
  Serial.print("=== NODE ID: ");
  Serial.print(NODE_ID);
  Serial.println(" ===");
  Serial.println("Type a message and press Enter to broadcast:");

  if (!init_lora()) {
    while (true) {
      delay(1000);
    }
  }

  Serial.println("LoRa mesh ready");
}

void loop() {
  // Hàm này xử lý cả nhận packet, gửi ACK, relay, ghép fragment và giải nén.
  lora_process();

  // Đọc dữ liệu từ người dùng gõ qua Serial Monitor
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        Serial.println(); // Xuống dòng sau khi bấm Enter
        Serial.print(">> Gửi tin nhắn: ");
        Serial.println(inputBuffer);
        
        // Gửi broadcast tới toàn mạng
        if (lora_send_text(BROADCAST_ID, inputBuffer.c_str())) {
          Serial.println("Gửi thành công!");
        } else {
          Serial.println("Gửi thất bại (hoặc dữ liệu quá dài).");
        }
        
        inputBuffer = ""; // Xóa buffer
        Serial.println("\nType a message and press Enter to broadcast:");
      }
    } else {
      inputBuffer += c;
      Serial.print(c); // Local Echo: Hiển thị ký tự vừa gõ lên màn hình
    }
  }
}
