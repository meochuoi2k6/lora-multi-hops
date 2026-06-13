/**
 * @file lora_setup.cpp
 * @brief Chứa các cấu hình vật lý (PHY) cho chip LoRa SX1278.
 * 
 * Nơi thiết lập tần số, băng thông, hệ số mã hóa (Coding Rate), và công suất
 * phát sóng để tối ưu hóa phạm vi thu phát cho mạng Mesh đa chặng (Multi-hop).
 */

#include "lora_setup.h"
#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>

/**
 * @brief Khởi tạo và cấu hình IC LoRa SX127x.
 * 
 * Hàm này thiết lập giao tiếp SPI, cấu hình các chân điều khiển, 
 * và ghi đè các tham số nội bộ của chip (Registers) để chạy ở dải tần ISM.
 * 
 * @return true Nếu giao tiếp SPI thành công và chip LoRa phản hồi đúng.
 * @return false Nếu không tìm thấy chip LoRa (hỏng mạch, sai chân nối).
 */
bool init_lora(){
    // Khai báo chân SPI cho ESP32.
    SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, NSS_PIN);
  
    // Liên kết chân điều khiển của LoRa với ESP32.
    LoRa.setPins(NSS_PIN, RST_PIN, DIO0_PIN);
   
    // Sử dụng đối tượng SPI đã khai báo để giao tiếp với LoRa.
    LoRa.setSPI(SPI);
    
    // Tần số SPI 8 MHz phù hợp với dây jumper ngắn, tránh nhiễu tín hiệu trên bus.
    LoRa.setSPIFrequency(SPI_FREQUENCY); 

    // Khởi động LoRa ở tần số ISM (VD: 433 MHz).
    // Tại Việt Nam, 433MHz là băng tần miễn cấp phép (ISM/SRD).
    if (!LoRa.begin(LORA_FREQUENCY)) {
        Serial.println("Starting LoRa failed!");
        return false;
    }
    
    // Cấu hình các thông số vô tuyến cho mạng lưới multi-hop: 
    // Sử dụng chân PA_BOOST ở mức công suất 20 dBm.
    LoRa.setTxPower(17, PA_OUTPUT_PA_BOOST_PIN);

    // SF quyết định độ nhạy thu và thời gian chiếm sóng (Airtime). 
    // SF7 là mức cân bằng tốt nhất giữa tốc độ (bitrate) và khoảng cách ở mạng nội bộ.
    LoRa.setSpreadingFactor(SPREADING_FACTOR);

    // Băng thông tín hiệu (VD: 125kHz). Băng thông rộng = truyền nhanh nhưng tầm ngắn.
    LoRa.setSignalBandwidth(SIGNAL_BANDWIDTH);

    // Hệ số sửa lỗi (FEC). 4/5 nghĩa là cứ 4 bit dữ liệu thì có 1 bit sửa lỗi. Giúp chống nhiễu nhẹ.
    LoRa.setCodingRate4(CODING_RATE_PER_4_BITS);

    // Byte đồng bộ (Sync Word). Dùng để phân tách mạng lưới, các mạch khác Sync Word sẽ phớt lờ nhau.
    LoRa.setSyncWord(SYNC_WORD);

    // Độ dài chuỗi mào đầu (Preamble) giúp phần cứng đánh thức bộ thu và đồng bộ hóa tín hiệu.
    LoRa.setPreambleLength(PREAMBLE_LENGTH);

    // Kích hoạt tính năng kiểm tra lỗi CRC phần cứng. Nếu sóng bị nhiễu làm sai bit,
    // chip SX1278 sẽ tự loại bỏ gói tin mà không đánh thức vi điều khiển.
    LoRa.enableCrc();

    // Setup xong thì đưa module về chế độ lắng nghe packet chờ tín hiệu đến.
    LoRa.receive();

    return true;
}