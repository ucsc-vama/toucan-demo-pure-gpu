#include "Image.h"
#include <cassert>
#include <iostream>
using namespace puregpu;
int main(int argc, char **argv) {
  assert(argc == 2);
  Image image(argv[1]);
  assert(image.entry == 0x80000000);
  assert(image.tohost == 0x80001000 && image.fromhost == 0x80001040);
  toucanGPUSim::SimDebugInfo symbols;
  for (int i = 15; i >= 0; i--) {
    symbols.memDebugInfo["TestHarness.mem.ram"].push_back({unsigned(i) * 4096, 4, 4096});
  }
  toucanGPUSim::SimDesignInfo design;
  design.memPoolSize = 4096 * 16;
  design.memPool.resize(design.memPoolSize, 15);
  Memory ram(symbols, design.memPoolSize);
  ram.load(design, image);
  for (const auto &s : image.segments) {
    for (uint64_t i = 0; i < s.size; i++) {
      uint64_t a = s.address + i;
      unsigned nibble = (a - ram.base) % 8 * 2, word = (a - ram.base) / 8;
      unsigned value =
          design.memPool[nibble * 4096 + word] | (design.memPool[(nibble + 1) * 4096 + word] << 4);
      assert(value == (i < s.data.size() ? s.data[i] : 0));
    }
  }
  bool rejected = false;
  try {
    Memory bad(symbols, 1);
  } catch (const std::runtime_error &) {
    rejected = true;
  }
  assert(rejected);
  image.segments[0].address = 0;
  rejected = false;
  try {
    ram.load(design, image);
  } catch (const std::runtime_error &) {
    rejected = true;
  }
  assert(rejected);
  std::cout << "PASS: ELF symbols, address mapping, fragment order, segment bytes, zero-filled BSS "
               "and bounds\n";
}
