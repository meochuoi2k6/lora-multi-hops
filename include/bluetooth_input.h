#pragma once

#include <Arduino.h>

/**
 * @brief Khoi tao Bluetooth Classic SPP de nhap tin nhan tu dien thoai/PC.
 *
 * Serial van duoc giu lai lam kenh debug, con du lieu nguoi dung se nhap qua
 * Bluetooth. Ten thiet bi se co dang "LoRaNode-<NODE_ID>".
 */
bool bluetooth_input_setup();

/**
 * @brief Doc tung ky tu tu Bluetooth, gom thanh mot dong, roi gui qua LoRa.
 *
 * Goi ham nay lien tuc trong loop(). Khi nguoi dung bam Enter tren app
 * Bluetooth terminal, dong text se duoc broadcast vao mang LoRa.
 */
void bluetooth_input_process();

/**
 * @brief In message LoRa nhan duoc ra Bluetooth terminal.
 *
 * Serial van ghi log day du, con Bluetooth dung de nguoi demo nhin thay noi dung
 * ngay tren dien thoai/PC ma khong can mo Serial Monitor.
 */
void bluetooth_input_print_received(uint8_t src, uint8_t dst, uint16_t msgId, const char *text);

/**
 * @brief In cac thong so do dac ra Bluetooth
 */
void bluetooth_input_print_metric_tx(float entropy, float ratio);
void bluetooth_input_print_metric_ack(unsigned long rttMs);
void bluetooth_input_print_metric_rx(int rssi, float snr);
void bluetooth_input_print_metric_rx_loss(uint8_t fragsReceived, uint8_t fragsTotal);
void bluetooth_input_print_ping(uint8_t src);

