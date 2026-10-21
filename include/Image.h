#pragma once
#include "ToucanGPUGenDataTypes.h"
#include <algorithm>
#include <cstring>
#include <elf.h>
#include <iterator>
#include <stdexcept>

namespace puregpu {
struct Segment {
  uint64_t address, size;
  std::vector<uint8_t> data;
};
// Supported workloads are statically linked little-endian RV64 executable ELFs
// with PT_LOAD segments, an entry point at 0x80000000, and a symbol table retaining
// tohost/fromhost. HTIF calls must fit the subset implemented by Controller;
// arbitrary Linux executables or programs needing a host filesystem are unsupported.
// The bundled vvadd benchmark has been exercised on all 16 supplied paper designs.
struct Image {
  uint64_t entry = 0, tohost = 0, fromhost = 0;
  std::vector<Segment> segments;
  explicit Image(const std::string &path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) {
      throw std::runtime_error("Cannot open ELF: " + path);
    }
    std::vector<uint8_t> bytes{std::istreambuf_iterator<char>(f), {}};
    auto range = [&](uint64_t off, uint64_t n) {
      if (off > bytes.size() || n > bytes.size() - off) {
        throw std::runtime_error("Truncated ELF");
      }
    };
    range(0, sizeof(Elf64_Ehdr));
    Elf64_Ehdr h;
    std::memcpy(&h, bytes.data(), sizeof h);
    if (std::memcmp(h.e_ident, ELFMAG, SELFMAG) || h.e_ident[EI_CLASS] != ELFCLASS64 ||
        h.e_ident[EI_DATA] != ELFDATA2LSB || h.e_machine != EM_RISCV || h.e_type != ET_EXEC ||
        h.e_phentsize != sizeof(Elf64_Phdr) || h.e_shentsize != sizeof(Elf64_Shdr)) {
      throw std::runtime_error("Require a little-endian RV64 executable ELF with symbols");
    }
    entry = h.e_entry;
    range(h.e_phoff, uint64_t(h.e_phnum) * h.e_phentsize);
    range(h.e_shoff, uint64_t(h.e_shnum) * h.e_shentsize);
    for (unsigned i = 0; i < h.e_phnum; i++) {
      Elf64_Phdr ph;
      std::memcpy(&ph, bytes.data() + h.e_phoff + i * sizeof ph, sizeof ph);
      if (ph.p_type != PT_LOAD) {
        continue;
      }
      if (ph.p_filesz > ph.p_memsz) {
        throw std::runtime_error("Invalid ELF segment size");
      }
      range(ph.p_offset, ph.p_filesz);
      segments.push_back(
          {ph.p_paddr,
           ph.p_memsz,
           {bytes.begin() + ph.p_offset, bytes.begin() + ph.p_offset + ph.p_filesz}});
    }
    for (unsigned i = 0; i < h.e_shnum; i++) {
      Elf64_Shdr sh;
      std::memcpy(&sh, bytes.data() + h.e_shoff + i * sizeof sh, sizeof sh);
      if (sh.sh_type != SHT_SYMTAB) {
        continue;
      }
      if (sh.sh_link >= h.e_shnum || sh.sh_entsize != sizeof(Elf64_Sym) ||
          sh.sh_size % sizeof(Elf64_Sym)) {
        throw std::runtime_error("Invalid ELF symbol table");
      }
      Elf64_Shdr strings;
      std::memcpy(&strings, bytes.data() + h.e_shoff + sh.sh_link * sizeof sh, sizeof strings);
      range(sh.sh_offset, sh.sh_size);
      range(strings.sh_offset, strings.sh_size);
      for (uint64_t off = 0; off < sh.sh_size; off += sizeof(Elf64_Sym)) {
        Elf64_Sym sym;
        std::memcpy(&sym, bytes.data() + sh.sh_offset + off, sizeof sym);
        if (sym.st_name >= strings.sh_size || sym.st_shndx == SHN_UNDEF) {
          continue;
        }
        const char *name =
            reinterpret_cast<const char *>(bytes.data() + strings.sh_offset + sym.st_name);
        if (!std::memchr(name, 0, strings.sh_size - sym.st_name)) {
          throw std::runtime_error("Invalid ELF string");
        }
        if (std::strcmp(name, "tohost") == 0) {
          tohost = sym.st_value;
        }
        if (std::strcmp(name, "fromhost") == 0) {
          fromhost = sym.st_value;
        }
      }
    }
    if (segments.empty() || !tohost || !fromhost) {
      throw std::runtime_error("Missing load segments or HTIF symbols");
    }
    // Supplied netlists change the reset boot vector from 0x10040 to 0x10000.
    // That ROM path jumps to the preloaded program here instead of waiting for a
    // host loader. Original-demo bytecode with the old vector is not a substitute.
    if (entry != 0x80000000) {
      throw std::runtime_error("Boot ROM jumps to 0x80000000; ELF entry differs");
    }
  }
};
// Symbols list memory fragments most-significant first, each with its own depth.
struct Memory {
  std::string name;
  std::vector<std::tuple<uint32_t, uint32_t, uint32_t>> fragments;
  uint64_t base = 0x80000000, size = 0;
  Memory(const toucanGPUSim::SimDebugInfo &symbols, uint64_t pool_size,
         const std::string &requested = "") {
    for (const auto &[key, parts] : symbols.memDebugInfo) {
      if (!requested.empty() ? key != requested : key.find("TestHarness.mem.") != 0) {
        continue;
      }
      uint64_t width = 0, depth = 0;
      bool valid = true;
      for (auto [pos, bits, n] : parts) {
        valid &= bits > 0 && bits <= 4 && uint64_t(pos) + n <= pool_size && (!depth || depth == n);
        width += bits;
        depth = n;
      }
      if (!valid || width != 64 || depth < 1024) {
        continue;
      }
      if (!name.empty()) {
        throw std::runtime_error("Ambiguous RAM; select --memory NAME");
      }
      name = key;
      fragments = parts;
      size = depth * 8;
    }
    if (name.empty()) {
      throw std::runtime_error("Cannot identify 64-bit external RAM from full symbols");
    }
  }
  bool contains(uint64_t a, uint64_t n) const {
    return a >= base && a - base <= size && n <= size - (a - base);
  }
  void byte(std::vector<uint8_t> &pool, uint64_t a, uint8_t value) const {
    uint64_t word = (a - base) / 8;
    unsigned bit = (a - base) % 8 * 8, shift = 0;
    for (auto it = fragments.rbegin(); it != fragments.rend(); ++it) {
      auto [pos, bits, depth] = *it;
      for (unsigned b = 0; b < bits; b++) {
        if (shift + b >= bit && shift + b < bit + 8) {
          auto &v = pool.at(pos + word);
          unsigned mask = 1u << b;
          v = (v & ~mask) | (((value >> (shift + b - bit)) & 1u) << b);
        }
      }
      shift += bits;
    }
  }
  // Populate physical load segments and zero BSS before reset starts. Boot uses
  // the ROM entry path; the controller does not redirect the hart's PC via DMI.
  void load(toucanGPUSim::SimDesignInfo &design, const Image &image) const {
    if (!contains(image.tohost, 8) || !contains(image.fromhost, 8) || (image.tohost & 7) ||
        (image.fromhost & 7)) {
      throw std::runtime_error("HTIF symbols outside RAM or unaligned");
    }
    for (const auto &s : image.segments) {
      if (!contains(s.address, s.size)) {
        throw std::runtime_error("ELF segment outside RAM");
      }
      // The fresh memory pool is zero, including BSS. Explicit writes also handle overlap.
      for (uint64_t i = 0; i < s.size; i++) {
        byte(design.memPool, s.address + i, i < s.data.size() ? s.data[i] : 0);
      }
    }
  }
};
} // namespace puregpu
