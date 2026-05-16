/**
 * @file static_huffman.h
 * @brief Định nghĩa các hàm và cấu trúc cho thuật toán nén tĩnh Huffman.
 * @note Static Huffman dùng một mô hình tần suất cố định ở mọi node.
 * Vì vậy sender và receiver tự dựng cùng một cây mã, không cần gửi cây trong packet,
 * giúp tiết kiệm băng thông và tối ưu cho dữ liệu text (ASCII).
 */
#pragma once

#include <Arduino.h>

/**
 * @brief Số lượng ký tự (symbol) tối đa được hỗ trợ.
 * @note Sử dụng 128 cho bảng mã chuẩn ASCII cơ bản.
 */
constexpr uint16_t HUFFMAN_SYMBOLS = 128;

/**
 * @brief Cấu trúc lưu trữ các thông số thống kê về quá trình nén.
 * @brief - `originalBytes`: Số byte của dữ liệu gốc.
 * @brief - `compressedBytes`: Số byte của dữ liệu sau khi nén.
 * @brief - `compressedBits`: Số bit thực tế của dữ liệu nén.
 * @brief - `entropy`: Giá trị entropy của dữ liệu (mức độ thông tin).
 * @brief - `compressionRatio`: Tỉ lệ nén đạt được (compressedBytes / originalBytes).
 */
struct HuffmanStats {
  uint16_t originalBytes;     ///< Số byte của dữ liệu gốc.
  uint16_t compressedBytes;   ///< Số byte của dữ liệu sau khi nén.
  uint16_t compressedBits;    ///< Số bit thực tế của dữ liệu nén.
  float entropy;              ///< Giá trị entropy của dữ liệu (mức độ thông tin).
  float compressionRatio;     ///< Tỉ lệ nén đạt được (compressedBytes / originalBytes).
};

/**
 * @brief Khởi tạo cây Huffman và bảng mã.
 * @note Hàm này chỉ cần gọi 1 lần. Nó sẽ tự động dựng cây và tính toán 
 * bảng mã dựa trên bảng tần suất tĩnh được định nghĩa sẵn.
 */
void huffman_init();

/**
 * @brief Tính toán entropy của input, giúp đánh giá mức độ nén có thể đạt được với Huffman.
 * @param input Dữ liệu gốc cần tính entropy.
 * @param inputLen Độ dài của dữ liệu gốc.
 * @return Giá trị entropy tính theo bit trên ký tự. Entropy càng thấp thì khả năng nén càng tốt.
 */
float huffman_entropy(const uint8_t *input, uint16_t inputLen);

/**
 * @brief Nén dữ liệu bằng thuật toán Huffman tĩnh.
 * @param input Con trỏ tới mảng dữ liệu đầu vào.
 * @param inputLen Độ dài dữ liệu đầu vào (tính bằng byte).
 * @param output Con trỏ tới bộ nhớ đầu ra để lưu dữ liệu đã nén.
 * @param outputMax Độ dài tối đa của bộ nhớ đầu ra để tránh tràn bộ đệm (Buffer Overflow).
 * @param compressedBits Biến tham chiếu để trả về tổng số bit của dữ liệu sau khi nén.
 * @param compressedBytes Biến tham chiếu để trả về tổng số byte của dữ liệu sau khi nén.
 * @return true nếu quá trình nén thành công, false nếu lỗi (kích thước quá lớn, hoặc ký tự ngoài mảng hỗ trợ).
 */
bool huffman_compress(const uint8_t *input,
                      uint16_t inputLen,
                      uint8_t *output,
                      uint16_t outputMax,
                      uint16_t &compressedBits,
                      uint16_t &compressedBytes);

/**
 * @brief Giải nén dữ liệu đã được nén bằng thuật toán Huffman tĩnh.
 * @param input Con trỏ tới mảng dữ liệu nén.
 * @param compressedBits Số bit thực tế của dữ liệu đã nén.
 * @param output Con trỏ tới mảng kết quả để lưu dữ liệu giải nén.
 * @param outputLen Độ dài cần giải nén (độ dài của dữ liệu gốc ban đầu).
 * @return true nếu quá trình giải nén thành công và khớp độ dài, false nếu có lỗi trong luồng bit.
 */
bool huffman_decompress(const uint8_t *input,
                        uint16_t compressedBits,
                        uint8_t *output,
                        uint16_t outputLen);


