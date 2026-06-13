/**
 * @file bluetooth_input.cpp
 * @brief Giao diện giao tiếp Bluetooth Classic giữa ESP32 và App Flutter.
 * 
 * Nơi tiếp nhận chuỗi ký tự từ App (có xử lý Framing), đẩy vào LoRa để phát đi,
 * và ngược lại: nhận số liệu Metric/Tin nhắn từ LoRa để đẩy lên màn hình điện thoại.
 */

#include "bluetooth_input.h"

#include <BluetoothSerial.h>

#include "lora_packet.h"

namespace {
BluetoothSerial SerialBT;
String bluetoothBuffer;
bool btFramingMode = false;

/**
 * @brief In ra màn hình Terminal giả lập (nếu dùng các app Bluetooth Serial).
 */
void print_prompt() {
  SerialBT.println();
  SerialBT.println("Nhap tin nhan roi bam Enter de gui broadcast:");
}

/**
 * @brief Xử lý chuỗi tin nhắn vừa ghép xong (đã nhận đủ [START] và [END]) để đẩy vào mạng LoRa.
 * @note Nếu chuỗi có định dạng `X:Nội_dung`, mạch sẽ gửi riêng (Unicast) cho Node ID = X.
 * Nếu không có ID, mạch sẽ gửi Broadcast.
 */
void send_buffered_message() {
  bluetoothBuffer.trim();
  if (bluetoothBuffer.length() == 0) {
    print_prompt();
    return;
  }

  uint8_t dst = BROADCAST_ID;
  int colonIndex = bluetoothBuffer.indexOf(':');
  if (colonIndex > 0) {
    String dstStr = bluetoothBuffer.substring(0, colonIndex);
    dst = dstStr.toInt();
    bluetoothBuffer = bluetoothBuffer.substring(colonIndex + 1);
  }

  Serial.print("BT input to ");
  Serial.print(dst);
  Serial.print(": ");
  Serial.println(bluetoothBuffer);

  SerialBT.print("Dang gui toi ");
  SerialBT.print(dst);
  SerialBT.print(": ");
  SerialBT.println(bluetoothBuffer);

  // Đẩy văn bản vào hàm nén và phát của lora_packet.cpp
  lora_queue_text(dst, bluetoothBuffer.c_str());
  SerialBT.println("Da dua vao hang doi TX (cho gui sau 5s hoac khi ranh)");

  bluetoothBuffer = "";
  print_prompt();
}
}  // namespace

/**
 * @brief Khởi tạo Bluetooth Serial.
 * @return true Nếu phần cứng Bluetooth sẵn sàng.
 */
bool bluetooth_input_setup() {
  String deviceName = "LoRaNode-" + String(NODE_ID);

  // Cấp phát trước bộ nhớ (1100 byte) để tránh việc String cấp phát lại liên tục gây chậm vòng loop()
  bluetoothBuffer.reserve(1100);

  if (!SerialBT.begin(deviceName)) {
    Serial.println("Bluetooth init failed");
    return false;
  }

  Serial.print("Bluetooth ready: ");
  Serial.println(deviceName);

  SerialBT.print("Da ket noi toi ");
  SerialBT.println(deviceName);
  print_prompt();

  return true;
}

/**
 * @brief Đọc liên tục luồng byte từ App Flutter đẩy xuống qua Bluetooth.
 * 
 * Hỗ trợ giao thức Framing (đóng khung tin nhắn):
 * Khi App gửi `[START]`, hàm bắt đầu ghép nối các ký tự.
 * Khi App gửi `[END]`, hàm kết thúc ghép và kích hoạt lora_send_text.
 * Giúp khắc phục lỗi tin nhắn quá dài bị đứt khúc khi truyền qua Bluetooth.
 */
void bluetooth_input_process() {
  while (SerialBT.available() > 0) {
    char c = static_cast<char>(SerialBT.read());

    if (c == '\n' || c == '\r') {
      bluetoothBuffer.trim();
      if (bluetoothBuffer == "[START]") {
        btFramingMode = true;
        bluetoothBuffer = "";
        continue;
      } else if (bluetoothBuffer.endsWith("[END]")) {
        btFramingMode = false;
        bluetoothBuffer.replace("[END]", "");
        send_buffered_message();
        continue;
      }

      if (!btFramingMode) {
        send_buffered_message();
      } else {
        // Nếu đang trong framing mode, dấu \n chỉ là ký tự xuống dòng bình thường
        if (bluetoothBuffer.length() < LORA_MESSAGE_MAX) {
            bluetoothBuffer += '\n';
        }
      }
      continue;
    }

    // Hỗ trợ xóa lùi nếu gõ tay qua Terminal
    if (c == '\b' || c == 127) {
      if (bluetoothBuffer.length() > 0) {
        bluetoothBuffer.remove(bluetoothBuffer.length() - 1);
      }
      continue;
    }

    if (bluetoothBuffer.length() >= LORA_MESSAGE_MAX) {
      SerialBT.println();
      SerialBT.println("Bo qua ky tu: tin nhan da cham gioi han LORA_MESSAGE_MAX");
      continue;
    }

    bluetoothBuffer += c;
    SerialBT.print(c);
  }
}

/**
 * @brief Báo hiệu lên App khi có tin nhắn MỚI đến.
 * @param src ID người gửi.
 * @param dst ID người nhận (có thể là Broadcast).
 * @param msgId ID của tin nhắn (để đồng bộ).
 * @param text Nội dung văn bản (đã giải nén).
 */
void bluetooth_input_print_received(uint8_t src, uint8_t dst, uint16_t msgId, const char *text) {
  SerialBT.println();
  SerialBT.print("[RX] SRC:");
  SerialBT.print(src);
  SerialBT.print(" DST:");
  SerialBT.print(dst);
  SerialBT.print(" MSG:");
  SerialBT.println(text);
  print_prompt();
}

/**
 * @brief Báo cáo kết quả thuật toán Lý thuyết thông tin (Nén Huffman).
 * @param entropy Chỉ số Entropy của gói tin (bits/char).
 * @param ratio Tỷ số nén (Kích thước sau nén / Kích thước gốc).
 */
void bluetooth_input_print_metric_tx(float entropy, float ratio) {
  SerialBT.println();
  SerialBT.print("[METRIC] TYPE:TX ENTROPY:");
  SerialBT.print(entropy, 3);
  SerialBT.print(" RATIO:");
  SerialBT.println(ratio, 3);
}

/**
 * @brief Báo cáo độ trễ khứ hồi (Delay / RTT) khi nhận được gói ACK từ đích.
 * @param rttMs Thời gian từ lúc phát đi đến lúc nhận phản hồi (tính bằng milli-giây).
 */
void bluetooth_input_print_metric_ack(unsigned long rttMs) {
  SerialBT.println();
  SerialBT.print("[METRIC] TYPE:ACK RTT:");
  SerialBT.println(rttMs);
}

/**
 * @brief Báo cáo chất lượng kênh truyền vật lý LoRa (dùng cho khảo sát, vẽ bản đồ phủ sóng).
 * @param rssi Cường độ tín hiệu nhận được (Received Signal Strength Indicator), đo bằng dBm.
 * @param snr Tỷ số tín hiệu trên nhiễu (Signal-to-Noise Ratio), đo bằng dB.
 */
void bluetooth_input_print_metric_rx(int rssi, float snr) {
  SerialBT.println();
  SerialBT.print("[METRIC] TYPE:RX RSSI:");
  SerialBT.print(rssi);
  SerialBT.print(" SNR:");
  SerialBT.println(snr, 1);
}

/**
 * @brief Báo cáo lên App rằng một thiết bị vừa phát tín hiệu "Còn sống" (Heartbeat).
 * @param src ID của mạch phát Ping.
 */
void bluetooth_input_print_ping(uint8_t src) {
  SerialBT.println();
  SerialBT.print("[PING] SRC:");
  SerialBT.println(src);
}

/**
 * @brief Báo cáo thống kê mất gói (Packet Loss) bên nhận lên App.
 * @param fragsReceived Số lượng phân mảnh đã nhận được thành công.
 * @param fragsTotal Tổng số lượng phân mảnh đáng lẽ phải nhận.
 */
void bluetooth_input_print_metric_rx_loss(uint8_t fragsReceived, uint8_t fragsTotal) {
  SerialBT.println();
  SerialBT.print("[METRIC] TYPE:RX_LOSS RECV:");
  SerialBT.print(fragsReceived);
  SerialBT.print(" TOTAL:");
  SerialBT.print(fragsTotal);
  SerialBT.print(" LOSS_PCT:");
  
  if (fragsTotal == 0) {
     SerialBT.println("0.0");
  } else {
     float lossPct = 100.0f * (1.0f - (float)fragsReceived / (float)fragsTotal);
     SerialBT.println(lossPct, 1);
  }
}

