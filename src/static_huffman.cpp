#include "static_huffman.h"

#include <math.h>

namespace {
constexpr uint16_t HUFFMAN_MAX_NODES = HUFFMAN_SYMBOLS * 2 - 1;
constexpr int16_t HUFFMAN_NO_NODE = -1;

struct HuffmanNode {
  uint16_t freq;
  int16_t parent;
  int16_t left;
  int16_t right;
  int16_t symbol;
};

struct HuffmanCode {
  uint32_t bits;
  uint8_t bitLen;
};

HuffmanNode nodes[HUFFMAN_MAX_NODES];
HuffmanCode codes[HUFFMAN_SYMBOLS];
int16_t rootIndex = HUFFMAN_NO_NODE;
bool initialized = false;

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

void write_bit(uint8_t *output, uint16_t bitIndex, uint8_t bit) {
  uint16_t byteIndex = bitIndex / 8;
  uint8_t bitOffset = 7 - (bitIndex % 8);

  if (bit) {
    output[byteIndex] |= (1 << bitOffset);
  }
}

uint8_t read_bit(const uint8_t *input, uint16_t bitIndex) {
  uint16_t byteIndex = bitIndex / 8;
  uint8_t bitOffset = 7 - (bitIndex % 8);
  return (input[byteIndex] >> bitOffset) & 0x01;
}
}  // namespace

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

