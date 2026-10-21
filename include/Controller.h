#pragma once
#include <cstdint>
#ifdef __CUDACC__
#define PG_HD __host__ __device__
#else
#define PG_HD
#endif
namespace puregpu {
// One owner; tick() is called after every completed simulated cycle.
struct Pins {
  bool req_valid = false, resp_ready = false, req_ready = false, resp_valid = false;
  uint32_t addr = 0, op = 0, data = 0, response = 0, response_data = 0;
};
enum class Error : uint32_t {
  None,
  DmiResponse,
  DmiTimeout,
  AbstractCommand,
  UnsupportedDebug,
  UnsupportedHtif,
  UnsupportedSyscall,
  BadAddress,
  OutputFull,
  DesignStop,
  CycleLimit,
  Protocol
};
inline const char *error_name(Error error) {
  switch (error) {
  case Error::None:
    return "none";
  case Error::DmiResponse:
    return "nonzero DMI response";
  case Error::DmiTimeout:
    return "DMI transaction/poll timeout";
  case Error::AbstractCommand:
    return "debug abstract command failed";
  case Error::UnsupportedDebug:
    return "debug module lacks RV64 data/program buffer capacity";
  case Error::UnsupportedHtif:
    return "unsupported HTIF device/command";
  case Error::UnsupportedSyscall:
    return "unsupported syscall or output descriptor";
  case Error::BadAddress:
    return "invalid HTIF memory address";
  case Error::OutputFull:
    return "console buffer full";
  case Error::DesignStop:
    return "RTL assertion/stop";
  case Error::CycleLimit:
    return "simulation cycle limit";
  case Error::Protocol:
    return "DMI protocol/internal queue error";
  }
  return "unknown";
}
// GPU-side DMI/HTIF service for hart 0. HTIF supports syscall 64 (write to
// stdout/stderr), syscall 93 (exit), and the odd-valued encoded test exit status.
// Other devices, commands, syscalls, or descriptors fail explicitly. Memory
// requests use the hart's debug loads/stores so cached program writes are visible.
// DMI timeouts, RTL stops, output overflow, and nonzero exit status are failures.
struct Controller {
  static constexpr uint32_t DATA0 = 4, DMCONTROL = 0x10, DMSTATUS = 0x11, ABSTRACTCS = 0x16,
                            COMMAND = 0x17, PROGBUF0 = 0x20;
  static constexpr uint32_t ACTIVE = 1, HALT = 0x80000000, RESUME = 0x40000000;
  struct Op {
    uint32_t address, value, mask, expected, error_mask;
    int slot, half, source;
  };
  Op ops[96]{};
  uint64_t scratch[8]{};
  uint32_t count = 0, cursor = 0, age = 0;
  bool offered = false, accepted = false;
  Error error = Error::None;
  bool done = false;
  uint64_t exit_code = 0, cycles = 0, transactions = 0;
  uint64_t tohost = 0, fromhost = 0, ram_base = 0x80000000, ram_size = 0x10000000;
  uint64_t descriptor = 0, args[8]{}, buffer_pos = 0, output_length = 0;
  uint32_t argument = 0, idle = 0, poll_interval = 4096, timeout = 100000;
  enum Phase {
    Start,
    Started,
    Poll,
    Clear,
    Descriptor,
    Buffer,
    Return,
    AckRead,
    AckWrite,
    Sleeping
  };
  Phase phase = Start;
  PG_HD void fail(Error e) {
    error = e;
    done = true;
  }
  PG_HD bool in_ram(uint64_t a, uint64_t n) const {
    return a >= ram_base && a - ram_base <= ram_size && n <= ram_size - (a - ram_base);
  }
  PG_HD void add(uint32_t a, uint32_t v = 0, uint32_t mask = 0, uint32_t expected = 0,
                 uint32_t err = 0, int slot = -1, int half = 0, int source = -1) {
    if (count >= 96) {
      fail(Error::Protocol);
      return;
    }
    ops[count++] = {a, v, mask, expected, err, slot, half, source};
  }
  // source=-1 immediate write, >=0 scratch write, -2 read, -3 poll.
  PG_HD void wr(uint32_t a, uint32_t v) { add(a, v); }
  PG_HD void rd(uint32_t a, int slot, int half = 0) { add(a, 0, 0, 0, 0, slot, half, -2); }
  PG_HD void poll(uint32_t a, uint32_t mask, uint32_t value, uint32_t errors = 0) {
    add(a, 0, mask, value, errors, -1, 0, -3);
  }
  PG_HD void command(uint32_t c) {
    wr(COMMAND, c);
    poll(ABSTRACTCS, 0x1000, 0, 0x700);
  }
  PG_HD void get_reg(uint32_t r, int slot) {
    command(0x00320000 | 0x1000 | r);
    rd(DATA0, slot);
    rd(DATA0 + 1, slot, 1);
  }
  PG_HD void set_reg(uint32_t r, int slot, bool execute = false) {
    add(DATA0, 0, 0, 0, 0, -1, 0, slot);
    add(DATA0 + 1, 0, 0, 0, 0, -1, 1, slot);
    command(0x00330000 | (execute ? 0x40000 : 0) | 0x1000 | r);
  }
  PG_HD void memory(uint64_t address, bool write = false, uint64_t value = 0) {
    count = cursor = 0;
    if ((address & 7) || !in_ram(address, 8)) {
      fail(Error::BadAddress);
      return;
    }
    scratch[2] = address;
    scratch[3] = value;
    // Preserve x8/x9 and the hart's PC through the debug ROM's halt/resume path.
    wr(DMCONTROL, ACTIVE);
    rd(DMSTATUS, 7);
    wr(DMCONTROL, ACTIVE | HALT);
    poll(DMSTATUS, 0x200, 0x200);
    wr(DMCONTROL, ACTIVE);
    rd(DMSTATUS, 7);
    get_reg(8, 0);
    get_reg(9, 1);
    set_reg(8, 2);
    wr(PROGBUF0, write ? 0x00943023 : 0x00043483); // sd/ld x9,0(x8)
    wr(PROGBUF0 + 1, 0x00100073);                  // ebreak
    if (write) {
      set_reg(9, 3, true);
    } else {
      command(0x00360000 | 0x1000);
      get_reg(9, 3);
    } // read x0 + postexec
    set_reg(8, 0);
    set_reg(9, 1);
    wr(DMCONTROL, ACTIVE | RESUME);
    poll(DMSTATUS, 0x20000, 0x20000);
    wr(DMCONTROL, ACTIVE);
    rd(DMSTATUS, 7);
  }
  template <class Sink> PG_HD void advance(Sink &sink) {
    switch (phase) {
    case Start:
      wr(DMCONTROL, ACTIVE);
      poll(DMCONTROL, ACTIVE, ACTIVE);
      rd(ABSTRACTCS, 4);
      phase = Started;
      break;
    case Started:
      if (((scratch[4] >> 24) & 31) < 2 || (scratch[4] & 15) < 2) {
        fail(Error::UnsupportedDebug);
        break;
      }
      phase = Poll;
      memory(tohost);
      break;
    case Poll:
      if (!scratch[3]) {
        phase = Sleeping;
        idle = poll_interval;
        break;
      }
      if (scratch[3] >> 48) {
        fail(Error::UnsupportedHtif);
        break;
      }
      if (scratch[3] & 1) {
        exit_code = scratch[3] >> 1;
        done = true;
        break;
      }
      descriptor = scratch[3];
      if ((descriptor & 7) || !in_ram(descriptor, 64)) {
        fail(Error::BadAddress);
        break;
      }
      phase = Clear;
      memory(tohost, true, 0);
      break;
    case Clear:
      argument = 0;
      phase = Descriptor;
      memory(descriptor);
      break;
    case Descriptor:
      args[argument++] = scratch[3];
      if (argument < 8) {
        memory(descriptor + argument * 8);
        break;
      }
      if (args[0] == 93) {
        exit_code = args[1];
        done = true;
        break;
      }
      if (args[0] != 64 || (args[1] != 1 && args[1] != 2)) {
        fail(Error::UnsupportedSyscall);
        break;
      }
      if (!in_ram(args[2], args[3])) {
        fail(Error::BadAddress);
        break;
      }
      buffer_pos = 0;
      output_length = args[3];
      if (!output_length) {
        phase = Return;
        memory(descriptor, true, 0);
      } else {
        phase = Buffer;
        memory(args[2] & ~uint64_t(7));
      }
      break;
    case Buffer: {
      // Buffer and descriptors are read through the hart, not backing RAM.
      uint32_t offset = (args[2] + buffer_pos) & 7;
      for (uint32_t i = offset; i < 8 && buffer_pos < output_length; i++, buffer_pos++) {
        if (!sink.put(uint8_t(scratch[3] >> (8 * i)))) {
          fail(Error::OutputFull);
          return;
        }
      }
      if (buffer_pos < output_length) {
        memory((args[2] + buffer_pos) & ~uint64_t(7));
      } else {
        phase = Return;
        memory(descriptor, true, output_length);
      }
      break;
    }
    case Return:
      phase = AckRead;
      memory(fromhost);
      break;
    case AckRead:
      if (scratch[3]) {
        memory(fromhost);
        break;
      }
      phase = AckWrite;
      memory(fromhost, true, 1);
      break;
    case AckWrite:
      phase = Sleeping;
      idle = poll_interval;
      break;
    case Sleeping:
      phase = Poll;
      memory(tohost);
      break;
    }
  }
  template <class Sink> PG_HD void tick(Pins &p, Sink &sink) {
    ++cycles;
    if (done) {
      p.req_valid = p.resp_ready = false;
      return;
    }
    if (offered) {
      if (++age > timeout) {
        fail(Error::DmiTimeout);
        return;
      }
      if (p.req_valid && p.req_ready) {
        p.req_valid = false;
        accepted = true;
      }
      if (p.resp_valid) {
        if (!accepted) {
          fail(Error::Protocol);
          return;
        }
        if (p.response) {
          fail(Error::DmiResponse);
          return;
        }
        const auto &op = ops[cursor];
        if (p.response_data & op.error_mask) {
          fail(Error::AbstractCommand);
          return;
        }
        if (op.slot >= 0) {
          uint64_t mask = uint64_t(0xffffffff) << (32 * op.half);
          scratch[op.slot] =
              (scratch[op.slot] & ~mask) | (uint64_t(p.response_data) << (32 * op.half));
        }
        if (op.source != -3 || (p.response_data & op.mask) == op.expected) {
          ++cursor;
          age = 0;
        }
        offered = accepted = false;
        ++transactions;
      } else {
        return;
      }
    }
    p.req_valid = p.resp_ready = false;
    if (cursor == count) {
      count = cursor = 0;
      if (idle) {
        --idle;
        return;
      }
      advance(sink);
    }
    if (done || !count) {
      return;
    }
    const auto &op = ops[cursor];
    p.addr = op.address;
    p.op = op.source < -1 ? 1 : 2;
    p.data = op.source >= 0 ? uint32_t(scratch[op.source] >> (32 * op.half)) : op.value;
    p.req_valid = p.resp_ready = true;
    offered = true;
    accepted = false;
  }
};
} // namespace puregpu
