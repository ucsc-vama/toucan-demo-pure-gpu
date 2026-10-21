#include "Controller.h"
#include <cassert>
#include <iostream>
#include <map>
#include <string>
using namespace puregpu;
struct Sink {
  std::string text;
  size_t capacity = 1024;
  bool put(uint8_t b) {
    if (text.size() == capacity) {
      return false;
    }
    text += char(b);
    return true;
  }
};
struct Target {
  std::map<uint64_t, uint64_t> memory;
  uint64_t regs[32]{};
  uint32_t data[2]{}, program[2]{};
  bool halted = false, resume_ack = false;
  uint32_t active = 0;
  unsigned halts = 0, busy_reads = 0;
  uint32_t transact(uint32_t addr, uint32_t op, uint32_t value) {
    if (op == 2) {
      if (addr == Controller::DMCONTROL) {
        active = value & 1;
        if (value & Controller::HALT) {
          halted = true;
          resume_ack = false;
          ++halts;
        }
        if (value & Controller::RESUME) {
          halted = false;
          resume_ack = true;
        }
      }
      if (addr >= 4 && addr <= 5) {
        data[addr - 4] = value;
      }
      if (addr >= 0x20 && addr <= 0x21) {
        program[addr - 0x20] = value;
      }
      if (addr == Controller::COMMAND) {
        assert(halted);
        assert(((value >> 20) & 7) == 3);
        busy_reads = 2;
        unsigned reg = value & 31;
        if (value & 0x20000) {
          if (value & 0x10000) {
            if (reg) {
              regs[reg] = uint64_t(data[0]) | (uint64_t(data[1]) << 32);
            }
          } else {
            data[0] = uint32_t(regs[reg]);
            data[1] = uint32_t(regs[reg] >> 32);
          }
        }
        if (value & 0x40000) {
          assert(program[1] == 0x00100073);
          if (program[0] == 0x00043483) {
            regs[9] = memory[regs[8]];
          } else {
            assert(program[0] == 0x00943023);
            memory[regs[8]] = regs[9];
          }
        }
      }
      return 0;
    }
    assert(op == 1);
    if (addr == 4 || addr == 5) {
      return data[addr - 4];
    }
    if (addr == Controller::DMCONTROL) {
      return active;
    }
    if (addr == Controller::DMSTATUS) {
      return (halted ? 0x200 : 0) | (resume_ack ? 0x20000 : 0);
    }
    if (addr == Controller::ABSTRACTCS) {
      if (busy_reads) {
        --busy_reads;
        return (2u << 24) | 0x1002;
      }
      return (2u << 24) | 2;
    }
    return 0;
  }
};
static void test_run(unsigned latency, bool failure = false) {
  Controller c;
  Pins p;
  Sink sink;
  Target target;
  c.tohost = 0x80001000;
  c.fromhost = 0x80001040;
  c.poll_interval = 7;
  target.regs[8] = 0xfedcba9876543210;
  target.regs[9] = 0x123456789abcdef0;
  uint64_t desc = 0x80002000, buf = 0x80003003;
  std::string expected = "mcycle = 123\nminstret = 456\n";
  for (size_t i = 0; i < expected.size(); i++) {
    target.memory[(buf + i) & ~uint64_t(7)] |= uint64_t(uint8_t(expected[i]))
                                               << (8 * ((buf + i) & 7));
  }
  uint64_t args[] = {64, 1, buf, expected.size(), 0, 0, 0, 0};
  for (int i = 0; i < 8; i++) {
    target.memory[desc + i * 8] = args[i];
  }
  bool request_sent = false, ack_seen = false, pending = false;
  uint32_t countdown = 0, response = 0;
  for (unsigned cycle = 0; cycle < 200000 && !c.done; cycle++) {
    p.req_ready = cycle % 5 != 0;
    p.resp_valid = false;
    if (pending && countdown-- == 0) {
      p.resp_valid = true;
      p.response_data = response;
      pending = false;
    }
    if (p.req_valid && p.req_ready) {
      assert(!pending);
      response = target.transact(p.addr, p.op, p.data);
      if (!latency) {
        p.resp_valid = true;
        p.response_data = response;
      } else {
        pending = true;
        countdown = latency - 1;
      }
    }
    c.tick(p, sink);
    if (!target.halted) {
      assert(target.regs[8] == 0xfedcba9876543210 && target.regs[9] == 0x123456789abcdef0);
      if (!request_sent && target.halts >= 2) {
        target.memory[c.tohost] = desc;
        request_sent = true;
      }
      if (target.memory[c.fromhost]) {
        assert(target.memory[c.fromhost] == 1);
        assert(target.memory[desc] == expected.size());
        ack_seen = true;
        target.memory[c.fromhost] = 0;
        target.memory[c.tohost] = failure ? 7 : 1;
      }
    }
  }
  assert(c.done && c.error == Error::None && c.exit_code == (failure ? 3u : 0u));
  assert(ack_seen && sink.text == expected && c.transactions > 0);
}
int main() {
  test_run(0);
  test_run(1);
  test_run(9);
  test_run(3, true);
  {
    Controller c;
    Pins p;
    Sink sink;
    c.timeout = 5;
    for (int i = 0; i < 20; i++) {
      c.tick(p, sink);
    }
    assert(c.error == Error::DmiTimeout);
  }
  {
    Controller c;
    Pins p;
    Sink sink;
    c.tick(p, sink);
    p.req_ready = true;
    p.resp_valid = true;
    p.response = 2;
    c.tick(p, sink);
    assert(c.error == Error::DmiResponse);
  }
  {
    Controller c;
    Pins p;
    Sink sink;
    c.phase = Controller::Poll;
    c.scratch[3] = 1ull << 56;
    c.tick(p, sink);
    assert(c.error == Error::UnsupportedHtif);
  }
  {
    Controller c;
    Pins p;
    Sink sink;
    c.phase = Controller::Descriptor;
    c.argument = 7;
    c.args[0] = 123;
    c.tick(p, sink);
    assert(c.error == Error::UnsupportedSyscall);
  }
  {
    Controller c;
    Pins p;
    Sink sink;
    c.phase = Controller::Buffer;
    c.output_length = 2;
    sink.capacity = 1;
    c.tick(p, sink);
    assert(c.error == Error::OutputFull);
  }
  {
    Controller c;
    c.memory(0x70000000);
    assert(c.error == Error::BadAddress);
  }
  {
    Controller c;
    Pins p;
    Sink sink;
    c.phase = Controller::Descriptor;
    c.argument = 7;
    c.args[0] = 93;
    c.args[1] = 17;
    c.tick(p, sink);
    assert(c.done && c.exit_code == 17);
  }
  {
    Controller c;
    Pins p;
    Sink sink;
    c.phase = Controller::Started;
    c.tick(p, sink);
    assert(c.error == Error::UnsupportedDebug);
  }
  {
    Controller c;
    Pins p;
    Sink sink;
    c.poll(Controller::ABSTRACTCS, 0x1000, 0, 0x700);
    c.tick(p, sink);
    p.req_ready = true;
    p.resp_valid = true;
    p.response_data = 0x200;
    c.tick(p, sink);
    assert(c.error == Error::AbstractCommand);
  }
  {
    Controller c;
    Pins p;
    Sink sink;
    c.timeout = 6;
    c.poll(Controller::DMSTATUS, 0x200, 0x200);
    for (int i = 0; i < 20 && !c.done; i++) {
      p.req_ready = true;
      p.resp_valid = p.req_valid;
      c.tick(p, sink);
    }
    assert(c.error == Error::DmiTimeout);
  }
  std::cout << "PASS: DMI backpressure/delays, preserved registers, unaligned stdout, HTIF "
               "acknowledgement, exit and error paths\n";
}
