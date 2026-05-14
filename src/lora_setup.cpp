#include <Arduino.h>
#include <LoRa.h>
#include <SPI.h>

#define SCK_PIN 18
#define MISO_PIN 19
#define MOSI_PIN 23
#define NSS_PIN 5
#define RST_PIN 14
#define DIO0_PIN 2

bool lora_setup() {
    // ESP32 cho phep chon chan SPI bang phan mem, nen ta khai bao dung wiring o day.
    SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, NSS_PIN);

    // setPins phai goi truoc LoRa.begin() de thu vien biet chan CS/RESET/IRQ.
    LoRa.setPins(NSS_PIN, RST_PIN, DIO0_PIN);
    LoRa.setSPI(SPI);

    // 8 MHz thuong on voi day ngan. Neu breadboard/day dai chap chon, giam xuong 1E6.
    LoRa.setSPIFrequency(8E6);
    
    // Tan so 433 MHz phu hop voi nhieu module SX1278 pho bien.
    if (!LoRa.begin(433E6)) {
        Serial.println("Starting LoRa failed!");
        return false;
    }

    // Moi node trong mang multi-hop phai dung cung cac thong so radio nay.
    LoRa.setSpreadingFactor(7);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(5);
    LoRa.setSyncWord(0x12);
    LoRa.setPreambleLength(8);

    // CRC phan cung cua LoRa; packet van co CRC16 rieng o tang protocol.
    LoRa.enableCrc();

    // Setup xong thi dua module ve che do lang nghe packet.
    LoRa.receive();

    return true;
}
