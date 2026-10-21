// Host entry point: load the design and ELF, resolve controller signals, then run
// simulation and DMI/HTIF service in one cooperative GPU kernel. The runtime is
// adapted from toucanGPURT 132daee9bfbd30c0b43f4f2cfbd2bcba7e7fd7a7 with a preload
// callback and the controller kernel in PureGpuKernel.cuh. Supplied designs use
// Toucan's 5161430 flattened-name fix, required by the signal lookups below.
#include "ToucanGPUSim.h"
#include "Image.h"
#include "PureGpu.h"
#include <chrono>
#include <iostream>
#include <iomanip>
using namespace puregpu;
static uint64_t number(const std::string &s) {
  if (s.empty() || s.front() == '-') {
    throw std::runtime_error("Invalid integer: " + s);
  }
  size_t used;
  auto n = std::stoull(s, &used, 0);
  if (used != s.size()) {
    throw std::runtime_error("Invalid integer: " + s);
  }
  return n;
}
int main(int argc, char **argv) try {
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0]
              << " DESIGN_DIRECTORY PROGRAM.ELF [--max-cycles N] [--poll-cycles N] [--memory NAME] "
                 "[--set CYCLE:SIGNAL=HEX] [--inspect]\n";
    return 2;
  }
  Config config;
  std::string directory = argv[1], memory_name;
  Image image(argv[2]);
  bool inspect = false;
  std::vector<std::string> requested;
  for (int i = 3; i < argc; i++) {
    std::string opt = argv[i];
    if (opt == "--inspect") {
      inspect = true;
      continue;
    }
    if (i + 1 == argc) {
      throw std::runtime_error("Missing option value");
    }
    std::string val = argv[++i];
    if (opt == "--max-cycles") {
      config.max_cycles = number(val);
    } else if (opt == "--poll-cycles") {
      auto n = number(val);
      if (!n || n > UINT32_MAX) {
        throw std::runtime_error("Invalid polling interval");
      }
      config.poll_interval = n;
    } else if (opt == "--memory") {
      memory_name = val;
    } else if (opt == "--set") {
      requested.push_back(val);
    } else {
      throw std::runtime_error("Unknown option: " + opt);
    }
  }
  if (config.max_cycles <= config.reset_cycles) {
    throw std::runtime_error("Cycle limit must exceed reset length");
  }
  std::vector<Poke> pokes;
  auto prepare = [&](toucanGPUSim::SimDesignInfo &design, toucanGPUSim::SimDebugInfo &symbols) {
    Memory memory(symbols, design.memPoolSize, memory_name);
    memory.load(design, image);
    config.tohost = image.tohost;
    config.fromhost = image.fromhost;
    config.ram_size = memory.size;
    std::cout << "Preloaded ELF entry=0x" << std::hex << image.entry << " tohost=0x" << image.tohost
              << " fromhost=0x" << image.fromhost << std::dec << " into " << memory.name << "\n";
    const char *names[] = {"TestHarness.reset",
                           "TestHarness.SimDTM.debug_req_valid",
                           "TestHarness.SimDTM.debug_req_bits_addr",
                           "TestHarness.SimDTM.debug_req_bits_op",
                           "TestHarness.SimDTM.debug_req_bits_data",
                           "TestHarness.SimDTM.debug_resp_ready",
                           "TestHarness.SimDTM.debug_req_ready",
                           "TestHarness.SimDTM.debug_resp_valid",
                           "TestHarness.SimDTM.debug_resp_bits_resp",
                           "TestHarness.SimDTM.debug_resp_bits_data"};
    unsigned expected[] = {1, 1, 7, 2, 32, 1, 1, 1, 2, 32};
    for (unsigned n = 0; n < SignalCount; n++) {
      auto it = symbols.regDebugInfo.find(names[n]);
      if (it == symbols.regDebugInfo.end() || it->second.size() > 16) {
        throw std::runtime_error(std::string("Missing/invalid signal: ") + names[n]);
      }
      auto &signal = config.signals[n];
      unsigned width = 0;
      for (auto [slot, bits] : it->second) {
        if (slot >= design.regPoolSize || bits < 1 || bits > 4) {
          throw std::runtime_error("Invalid register fragment");
        }
        signal.slots[signal.count] = slot;
        signal.widths[signal.count++] = bits;
        width += bits;
      }
      if (width != expected[n]) {
        throw std::runtime_error("Signal width mismatch");
      }
    }
    // --set CYCLE:SIGNAL=HEX is repeatable and supports any register width, but
    // only names retained in the full symbol map can be resolved into fragments.
    for (auto request : requested) {
      auto colon = request.find(':'), equal = request.rfind('=');
      if (colon == std::string::npos || equal == std::string::npos || equal <= colon) {
        throw std::runtime_error("Expected CYCLE:SIGNAL=HEX");
      }
      auto cycle = number(request.substr(0, colon));
      if (cycle >= config.max_cycles) {
        throw std::runtime_error("Register write is outside cycle limit");
      }
      auto name = request.substr(colon + 1, equal - colon - 1), hex = request.substr(equal + 1);
      if (hex.starts_with("0x")) {
        hex.erase(0, 2);
      }
      if (hex.empty() || hex.find_first_not_of("0123456789abcdefABCDEF") != std::string::npos) {
        throw std::runtime_error("Expected hexadecimal register value");
      }
      auto it = symbols.regDebugInfo.find(name);
      if (it == symbols.regDebugInfo.end()) {
        throw std::runtime_error("Unknown register: " + name);
      }
      size_t bit = 0;
      for (auto f = it->second.rbegin(); f != it->second.rend(); ++f) {
        auto [slot, bits] = *f;
        unsigned value = 0;
        if (bits < 1 || bits > 4 || slot >= design.regPoolSize) {
          throw std::runtime_error("Invalid register fragment");
        }
        for (unsigned b = 0; b < bits; b++, bit++) {
          if (bit / 4 < hex.size()) {
            char ch = hex[hex.size() - 1 - bit / 4];
            unsigned digit = ch <= '9' ? ch - '0' : (ch | 32) - 'a' + 10;
            value |= ((digit >> (bit % 4)) & 1) << b;
          }
        }
        pokes.push_back({cycle, slot, uint8_t(value)});
      }
      for (; bit < hex.size() * 4; bit++) {
        char ch = hex[hex.size() - 1 - bit / 4];
        unsigned digit = ch <= '9' ? ch - '0' : (ch | 32) - 'a' + 10;
        if ((digit >> (bit % 4)) & 1) {
          throw std::runtime_error("Register value too wide: " + name);
        }
      }
    }
    std::stable_sort(pokes.begin(), pokes.end(),
                     [](const auto &a, const auto &b) { return a.cycle < b.cycle; });
  };
  std::string bin = directory + "/GPUSimDesign.bin",
              symbols_path = directory + "/GPUSimSymbols.bin";
  // --inspect validates ELF preload, RAM bounds, and controller/register symbols
  // entirely on the host; it does not launch CUDA or establish workload correctness.
  if (inspect) {
    toucanGPUSim::SimDesignInfo design;
    toucanGPUSim::SimDebugInfo symbols;
    std::ifstream a(bin, std::ios::binary), b(symbols_path, std::ios::binary);
    if (!a || !b) {
      throw std::runtime_error("Missing compiled design or full symbols");
    }
    toucanGPUSim::deserializeSimDesignInfo(a, design);
    toucanGPUSim::deserializeSimDebugInfo(b, symbols);
    if (!a || !b) {
      throw std::runtime_error("Truncated design artifacts");
    }
    design.Init();
    prepare(design, symbols);
    std::cout << "Preload and signal validation passed (no GPU execution)\n";
    return 0;
  }
  toucanGPUSim::ToucanSimulator sim;
  if (sim.init(0, bin, symbols_path, false, false, prepare)) {
    return 2;
  }
  std::vector<uint8_t> output;
  auto start = std::chrono::steady_clock::now();
  auto result = run(config, pokes, output);
  // Throughput uses completed simulation cycles and elapsed run() wall time,
  // including kernel setup and output retrieval. Printed guest PMU counters are
  // workload output, not the numerator used for simulation Hz.
  double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
  if (!output.empty()) {
    std::cout.write(reinterpret_cast<const char *>(output.data()), output.size());
  }
  std::cout << "\nSimulation cycles: " << result.cycles << "\nSimulation time: " << seconds
            << " s\nSimulation speed: " << result.cycles / seconds
            << " Hz\nDMI transactions: " << result.transactions << '\n';
  // Success requires both a clean controller result and HTIF exit status zero.
  // Historical vvadd validation also checked that PMU output was present; this
  // generic executable does not require benchmark-specific output strings.
  if (result.error != Error::None) {
    std::cerr << "FAILED: controller error=" << error_name(result.error)
              << " phase=" << result.phase << " DMI address=0x" << std::hex << result.dmi_address
              << '\n';
    return 2;
  }
  std::cout << (result.exit_code ? "FAILED" : "PASS") << ": HTIF exit code " << result.exit_code
            << '\n';
  return result.exit_code ? 1 : 0;
} catch (const std::exception &e) {
  std::cerr << "Error: " << e.what() << '\n';
  return 2;
}
