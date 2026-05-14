#pragma once

#include <Arduino.h>

// Static Huffman dùng một mô hình tần suất cố định ở mọi node.
// Vì vậy sender và receiver tự dựng cùng một cây mã, không cần gửi cây trong packet.
constexpr uint16_t HUFFMAN_SYMBOLS = 128;

struct HuffmanStats {
  uint16_t originalBytes;
  uint16_t compressedBytes;
  uint16_t compressedBits;
  float entropy;
  float compressionRatio;
};

void huffman_init();
float huffman_entropy(const uint8_t *input, uint16_t inputLen);
bool huffman_compress(const uint8_t *input,
                      uint16_t inputLen,
                      uint8_t *output,
                      uint16_t outputMax,
                      uint16_t &compressedBits,
                      uint16_t &compressedBytes);
bool huffman_decompress(const uint8_t *input,
                        uint16_t compressedBits,
                        uint8_t *output,
                        uint16_t outputLen);

