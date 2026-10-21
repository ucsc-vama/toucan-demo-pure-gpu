#pragma once
#include "Controller.h"
#include <vector>
namespace puregpu {
enum SignalId {
  Reset,
  ReqValid,
  ReqAddr,
  ReqOp,
  ReqData,
  RespReady,
  ReqReady,
  RespValid,
  RespCode,
  RespData,
  SignalCount
};
struct Signal {
  uint32_t slots[16]{};
  uint8_t widths[16]{};
  uint32_t count = 0;
};
// A one-time register deposit, not a persistent force. Cycle 0 precedes the first
// evaluation; cycle N follows N completed cycles and controller updates. Normal
// simulated logic can overwrite the value during the next evaluation.
struct Poke {
  uint64_t cycle;
  uint32_t slot;
  uint8_t value;
};
struct Config {
  Signal signals[SignalCount];
  // --max-cycles bounds simulated cycles, including reset (default: 10 million).
  uint64_t max_cycles = 10000000, tohost = 0, fromhost = 0;
  uint64_t ram_base = 0x80000000, ram_size = 0;
  // --poll-cycles controls the idle interval between HTIF polls on the GPU, not
  // host polling. Smaller intervals reduce response latency but add debug traffic.
  uint32_t reset_cycles = 20, poll_interval = 4096, timeout = 100000;
  // Fixed GPU console capacity; filling it raises OutputFull rather than wrapping.
  uint32_t output_capacity = 1024 * 1024;
};
struct Result {
  uint64_t cycles = 0, exit_code = 0, transactions = 0;
  uint32_t output_size = 0, phase = 0, dmi_address = 0;
  Error error = Error::None;
};
Result run(const Config &, const std::vector<Poke> &, std::vector<uint8_t> &);
} // namespace puregpu
