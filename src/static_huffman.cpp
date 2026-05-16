#include "static_huffman.h"

#include <math.h>

namespace {
constexpr uint16_t HUFFMAN_MAX_NODES = HUFFMAN_SYMBOLS * 2 - 1;
constexpr int16_t HUFFMAN_NO_NODE = -1;

/**
 * @brief Cấu trúc đại diện cho một Node trong cây Huffman.
 * @brief - `freq`: Tần suất xuất hiện của node.
 * @brief - `parent`: Index của node cha trong mảng nodes.
 * @brief - `left`: Index của node con bên trái.
 * @brief - `right`: Index của node con bên phải.
 * @brief - `symbol`: Ký tự (nếu là node lá), hoặc HUFFMAN_NO_NODE.
 * @note Lưu trữ thông tin về tần suất, node cha, node con trái/phải và ký tự đại diện.
 */
struct HuffmanNode {
  uint16_t freq;       ///< Tần suất xuất hiện của node.
  int16_t parent;      ///< Index của node cha trong mảng nodes.
  int16_t left;        ///< Index của node con bên trái.
  int16_t right;       ///< Index của node con bên phải.
  int16_t symbol;      ///< Ký tự (nếu là node lá), hoặc HUFFMAN_NO_NODE.
};

/**
 * @brief Cấu trúc lưu trữ mã Huffman cho một ký tự.
 * @brief - `bits`: Chuỗi bit mã hóa.
 * @brief - `bitLen`: Độ dài thực tế của chuỗi bit.
 * @note Chứa chuỗi bit đã mã hóa và độ dài của chuỗi bit đó.
 */
struct HuffmanCode {
  uint32_t bits;       ///< Chuỗi bit mã hóa.
  uint8_t bitLen;      ///< Độ dài thực tế của chuỗi bit.
};

HuffmanNode nodes[HUFFMAN_MAX_NODES];
HuffmanCode codes[HUFFMAN_SYMBOLS];
int16_t rootIndex = HUFFMAN_NO_NODE;
bool initialized = false;

/**
 * @brief Lấy tần suất xuất hiện tĩnh cho một ký tự ASCII.
 * @note Các tần suất này được định nghĩa cố định tối ưu cho văn bản text/log tiếng Anh.
 * Ký tự càng hay xuất hiện (như khoảng trắng, e, t, a) thì tần suất càng cao và sẽ được cấp mã càng ngắn.
 * @param symbol Ký tự ASCII cần lấy tần suất.
 * @return Giá trị tần suất giả định của ký tự đó.
 */
uint16_t static_frequency(uint8_t symbol) {
  // Các tần suất này được chọn cho text/log ASCII thường gặp.
  // Ký tự càng hay xuất hiện thì Huffman sẽ cấp mã càng ngắn.
  switch (symbol) {
    case ' ': return 90;
    case 'e': case 'E': return 80;
    case 't': case 'T': return 65;
    case 'a': case 'A': return 60;
    case 'o': case 'O': return 58;
    case 'i': case 'I': return 55;
    case 'n': case 'N': return 54;
    case 's': case 'S': return 50;
    case 'r': case 'R': return 48;
    case 'h': case 'H': return 42;
    case 'l': case 'L': return 40;
    case 'd': case 'D': return 36;
    case 'c': case 'C': return 32;
    case 'u': case 'U': return 30;
    case 'm': case 'M': return 28;
    case 'p': case 'P': return 26;
    case 'g': case 'G': return 24;
    case 'y': case 'Y': return 22;
    case 'b': case 'B': return 20;
    case 'f': case 'F': return 18;
    case 'v': case 'V': return 14;
    case 'k': case 'K': return 12;
    case 'w': case 'W': return 12;
    case 'x': case 'X': return 8;
    case 'q': case 'Q': return 6;
    case 'j': case 'J': return 6;
    case 'z': case 'Z': return 6;
    case '0': case '1': case '2': case '3': case '4':
    case '5': case '6': case '7': case '8': case '9':
      return 30;
    case '=': case ',': case '.': case ':': case ';':
    case '-': case '_': case '/':
      return 18;
    default:
      return 2;
  }
}

/**
 * @brief Tìm node tự do (chưa có parent) có tần suất nhỏ nhất trong danh sách.
 * @param nodeCount Số lượng node hiện tại đang xét.
 * @param skipNode Index của node cần bỏ qua (thường là node min thứ nhất vừa tìm được).
 * @return Index của node có tần suất nhỏ nhất.
 */
int16_t select_min_free_node(uint16_t nodeCount, int16_t skipNode) {
  int16_t selected = HUFFMAN_NO_NODE;

  for (uint16_t i = 0; i < nodeCount; ++i) {
    if (static_cast<int16_t>(i) == skipNode || nodes[i].parent != HUFFMAN_NO_NODE) {
      continue;
    }

    if (selected == HUFFMAN_NO_NODE ||
        nodes[i].freq < nodes[selected].freq ||
        (nodes[i].freq == nodes[selected].freq && i < static_cast<uint16_t>(selected))) {
      selected = i;
    }
  }

  return selected;
}

/**
 * @brief Duyệt ngược từ lá lên gốc để xây dựng bảng mã bit cho từng ký tự.
 * @note Hàm này duyệt cây Huffman đã xây dựng để lấy ra chuỗi bit đại diện cho mỗi ký tự 
 * (0 nếu rẽ trái, 1 nếu rẽ phải). Kết quả được lưu vào mảng `codes`.
 */
void build_codes() {
  for (uint16_t symbol = 0; symbol < HUFFMAN_SYMBOLS; ++symbol) {
    uint32_t reversedBits = 0;
    uint8_t bitLen = 0;
    int16_t current = symbol;

    while (nodes[current].parent != HUFFMAN_NO_NODE) {
      int16_t parent = nodes[current].parent;
      uint8_t bit = (nodes[parent].right == current) ? 1 : 0;
      reversedBits |= static_cast<uint32_t>(bit) << bitLen;
      bitLen++;
      current = parent;
    }

    uint32_t bits = 0;
    for (uint8_t i = 0; i < bitLen; ++i) {
      uint8_t edge = (reversedBits >> i) & 0x01;
      bits |= static_cast<uint32_t>(edge) << i;
    }

    codes[symbol] = {bits, bitLen};
  }
}

/**
 * @brief Ghi 1 bit (0 hoặc 1) vào một mảng byte ở vị trí bitIndex cho trước.
 * @param output Mảng byte đích.
 * @param bitIndex Vị trí bit muốn ghi vào (tính từ 0).
 * @param bit Giá trị bit cần ghi (chỉ lấy 0 hoặc 1).
 */
void write_bit(uint8_t *output, uint16_t bitIndex, uint8_t bit) {
  uint16_t byteIndex = bitIndex / 8;
  uint8_t bitOffset = 7 - (bitIndex % 8);

  if (bit) {
    output[byteIndex] |= (1 << bitOffset);
  }
}

/**
 * @brief Đọc giá trị của 1 bit tại vị trí bitIndex trong mảng byte.
 * @param input Mảng byte nguồn.
 * @param bitIndex Vị trí bit muốn đọc.
 * @return Giá trị của bit (0 hoặc 1).
 */
uint8_t read_bit(const uint8_t *input, uint16_t bitIndex) {
  uint16_t byteIndex = bitIndex / 8;
  uint8_t bitOffset = 7 - (bitIndex % 8);
  return (input[byteIndex] >> bitOffset) & 0x01;
}
}  // namespace

/**
 * @brief Khởi tạo cây Huffman và bảng mã.
 * @note Hàm này chỉ chạy 1 lần duy nhất trong toàn bộ chu kỳ sống của thiết bị.
 * Nó tạo các node lá từ bảng tần suất tĩnh, sau đó gộp dần 2 node nhỏ nhất 
 * để tạo thành cây nhị phân, và cuối cùng sinh ra bảng mã bit rút gọn.
 */
void huffman_init() {
  if (initialized) {
    return;
  }

  for (uint16_t i = 0; i < HUFFMAN_SYMBOLS; ++i) {
    nodes[i] = {
      static_frequency(static_cast<uint8_t>(i)),
      HUFFMAN_NO_NODE,
      HUFFMAN_NO_NODE,
      HUFFMAN_NO_NODE,
      static_cast<int16_t>(i),
    };
  }

  uint16_t nodeCount = HUFFMAN_SYMBOLS;
  while (nodeCount < HUFFMAN_MAX_NODES) {
    int16_t left = select_min_free_node(nodeCount, HUFFMAN_NO_NODE);
    int16_t right = select_min_free_node(nodeCount, left);

    nodes[left].parent = nodeCount;
    nodes[right].parent = nodeCount;
    nodes[nodeCount] = {
      static_cast<uint16_t>(nodes[left].freq + nodes[right].freq),
      HUFFMAN_NO_NODE,
      left,
      right,
      HUFFMAN_NO_NODE,
    };

    rootIndex = nodeCount;
    nodeCount++;
  }

  build_codes();
  initialized = true;
}

/**
 * @brief Tính toán entropy của input, giúp đánh giá mức độ nén có thể đạt được với Huffman. Entropy càng thấp thì khả năng nén càng tốt.
 * @param input Dữ liệu gốc cần tính entropy.
 * @param inputLen Độ dài của dữ liệu gốc.
 * @return Giá trị entropy tính theo bit trên ký tự. Ví dụ nếu entropy = 4.5 thì trung bình mỗi ký tự có thể được nén xuống còn 4.
 */
float huffman_entropy(const uint8_t *input, uint16_t inputLen) {
  if (input == nullptr || inputLen == 0) {
    return 0.0f;
  }

  uint16_t counts[HUFFMAN_SYMBOLS] = {};
  for (uint16_t i = 0; i < inputLen; ++i) {
    if (input[i] < HUFFMAN_SYMBOLS) {
      counts[input[i]]++;
    }
  }

  float entropy = 0.0f;
  for (uint16_t i = 0; i < HUFFMAN_SYMBOLS; ++i) {
    if (counts[i] == 0) {
      continue;
    }

    float p = static_cast<float>(counts[i]) / static_cast<float>(inputLen);
    entropy -= p * (logf(p) / logf(2.0f));
  }

  return entropy;
} 

/**
 * @brief Nén dữ liệu bằng Huffman
 * @param input Con trỏ tới dữ liệu đầu vào
 * @param inputLen Độ dài dữ liệu
 * @param output Con trỏ tới bộ nhớ đầu ra
 * @param outputMax Độ dài tối đa của bộ nhớ đầu ra
 * @param compressedBits Con trỏ để trả về độ dài của dữ liệu đã nén (tính theo bit)
 * @param compressedBytes Con trỏ để trả về độ dài của dữ liệu đã nén (tính theo byte)
 * @return true nếu nén thành công, false nếu không thành công
 */
bool huffman_compress(const uint8_t *input,
                      uint16_t inputLen,
                      uint8_t *output,
                      uint16_t outputMax,
                      uint16_t &compressedBits,
                      uint16_t &compressedBytes) {
  if (input == nullptr || output == nullptr || inputLen == 0) {
    return false;
  }

  huffman_init();
  memset(output, 0, outputMax);

  compressedBits = 0;
  for (uint16_t i = 0; i < inputLen; ++i) {
    uint8_t symbol = input[i];
    if (symbol >= HUFFMAN_SYMBOLS) {
      return false;
    }

    HuffmanCode code = codes[symbol];
    if ((compressedBits + code.bitLen + 7) / 8 > outputMax) {
      return false;
    }

    for (int8_t bit = code.bitLen - 1; bit >= 0; --bit) {
      write_bit(output, compressedBits, (code.bits >> bit) & 0x01);
      compressedBits++;
    }
  }

  compressedBytes = (compressedBits + 7) / 8;
  return true;
}

/**
 * @brief Giải nén dữ liệu đã được nén bằng thuật toán Huffman tĩnh.
 * @param input Con trỏ tới mảng dữ liệu nén.
 * @param compressedBits Số lượng bit của dữ liệu nén.
 * @param output Con trỏ tới mảng lưu kết quả giải nén.
 * @param outputLen Độ dài mong đợi của dữ liệu sau khi giải nén (rawLen).
 * @return true Nếu giải nén thành công và số lượng ký tự giải nén khớp với độ dài mong đợi.
 * @return false Nếu luồng bit bị hỏng, trỏ tới node không hợp lệ, hoặc dữ liệu đầu vào rỗng.
 * @note Hàm sẽ duyệt cây Huffman từ gốc xuống lá dựa trên từng bit đọc được. 
 * Gặp bit 0 thì rẽ trái, bit 1 thì rẽ phải. Khi chạm node lá, nó sẽ ghi ký tự tương ứng ra output.
 */
bool huffman_decompress(const uint8_t *input,
                        uint16_t compressedBits,
                        uint8_t *output,
                        uint16_t outputLen) {
  if (input == nullptr || output == nullptr || outputLen == 0) {
    return false;
  }

  huffman_init();

  uint16_t outIndex = 0;
  int16_t current = rootIndex;

  for (uint16_t bitIndex = 0; bitIndex < compressedBits && outIndex < outputLen; ++bitIndex) {
    uint8_t bit = read_bit(input, bitIndex);
    current = bit ? nodes[current].right : nodes[current].left;

    if (current == HUFFMAN_NO_NODE) {
      return false;
    }

    if (nodes[current].symbol != HUFFMAN_NO_NODE) {
      output[outIndex++] = static_cast<uint8_t>(nodes[current].symbol);
      current = rootIndex;
    }
  }

  return outIndex == outputLen;
}

