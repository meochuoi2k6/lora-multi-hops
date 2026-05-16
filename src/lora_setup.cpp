#include "lora_setup.h"
#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>

bool init_lora(){
    // Khai báo chân SPI cho ESP32.
    SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, NSS_PIN);
  
    // Liên kết chân điều khiển của LoRa với ESP32.
    LoRa.setPins(NSS_PIN, RST_PIN, DIO0_PIN);
   
    // Sử dụng đối tượng SPI đã khai báo để giao tiếp với LoRa.
    LoRa.setSPI(SPI);
    
    // Tần số SPI 8 MHz phù hợp với dây ngắn và tốc độ truyền ổn định.
    LoRa.setSPIFrequency(SPI_FREQUENCY); 

    // Khởi động LoRa ở tần số 433 MHz.
    //Ở Việt Nam, chỉ nên sử dụng các tần số nhất định, xem thêm tại biến LORA_FREQUENCY.
    if (!LoRa.begin(LORA_FREQUENCY)) {
        Serial.println("Starting LoRa failed!");
        return false;
    }
    
    // Cấu hình các thông số vô tuyến cho mạng lưới multi-hop.
    LoRa.setSpreadingFactor(SPREADING_FACTOR);
    LoRa.setSignalBandwidth(SIGNAL_BANDWIDTH);
    LoRa.setCodingRate4(CODING_RATE_PER_4_BITS);
    LoRa.setSyncWord(SYNC_WORD);
    LoRa.setPreambleLength(PREAMBLE_LENGTH);

    // Kích hoạt CRC phần cứng của LoRa để đảm bảo tính toàn vẹn dữ liệu.
    LoRa.enableCrc();

    // Setup xong thi dua module ve che do lang nghe packet.
    LoRa.receive();

    return true;
}