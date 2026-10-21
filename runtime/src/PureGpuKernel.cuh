// Included by SimEval.cu to share the interpreter's device pools and evaluator.
// One cooperative kernel evaluates the design and services DMI/HTIF; the host
// neither steps cycles nor services console requests while simulation runs.
#include "PureGpu.h"
#include <algorithm>
#include <stdexcept>
namespace puregpu {
struct DeviceState {
  Config config;
  Controller controller;
  Pins pins;
  Result result;
  uint8_t *output;
  Poke *pokes;
  uint32_t poke_count, poke_index;
  bool finished;
};
__device__ uint64_t read_signal(const Signal &s) {
  uint64_t v = 0;
  for (uint32_t i = 0; i < s.count; i++) {
    v = (v << s.widths[i]) | (regPool[s.slots[i]] & ((1u << s.widths[i]) - 1));
  }
  return v;
}
__device__ void write_signal(const Signal &s, uint64_t v) {
  for (int i = int(s.count) - 1; i >= 0; i--) {
    regPool[s.slots[i]] = v & ((1u << s.widths[i]) - 1);
    v >>= s.widths[i];
  }
}
__device__ void drive(DeviceState &s) {
  auto &p = s.pins;
  auto *sig = s.config.signals;
  write_signal(sig[ReqValid], p.req_valid);
  write_signal(sig[ReqAddr], p.addr);
  write_signal(sig[ReqOp], p.op);
  write_signal(sig[ReqData], p.data);
  write_signal(sig[RespReady], p.resp_ready);
}
__device__ void poke(DeviceState &s, uint64_t cycle) {
  while (s.poke_index < s.poke_count && s.pokes[s.poke_index].cycle == cycle) {
    auto p = s.pokes[s.poke_index++];
    regPool[p.slot] = p.value;
  }
}
struct OutputSink {
  DeviceState &state;
  __device__ bool put(uint8_t ch) {
    if (state.result.output_size >= state.config.output_capacity) {
      return false;
    }
    state.output[state.result.output_size++] = ch;
    return true;
  }
};
__global__ void simulate(DeviceState *state) {
  auto grid = cg::this_grid();
  if (grid.thread_rank() == 0) {
    shouldStop = 0;
    write_signal(state->config.signals[Reset], 1);
    drive(*state);
    poke(*state, 0);
  }
  grid.sync();
  for (uint64_t cycle = 0; cycle < state->config.max_cycles; cycle++) {
    for (uint32_t part = blockIdx.x; part < numParts_Region0; part += gridDim.x) {
      evalEachPartition(part);
    }
    grid.sync();
    for (uint32_t part = numParts_Region0 + blockIdx.x; part < numTotalParts; part += gridDim.x) {
      evalEachPartition(part);
    }
    grid.sync();
    if (grid.thread_rank() == 0) {
      auto &s = *state;
      s.result.cycles = cycle + 1;
      if (shouldStop) {
        s.controller.fail(Error::DesignStop);
      }
      if (cycle + 1 == s.config.reset_cycles) {
        write_signal(s.config.signals[Reset], 0);
      }
      if (cycle >= s.config.reset_cycles && !s.controller.done) {
        auto &p = s.pins;
        auto *sig = s.config.signals;
        p.req_ready = read_signal(sig[ReqReady]);
        p.resp_valid = read_signal(sig[RespValid]);
        p.response = read_signal(sig[RespCode]);
        p.response_data = read_signal(sig[RespData]);
        OutputSink output{s};
        s.controller.tick(p, output);
        drive(s);
      }
      poke(s, cycle + 1);
      s.finished = s.controller.done;
    }
    grid.sync(); // all blocks see controller writes and stop together
    if (state->finished) {
      break;
    }
  }
  if (grid.thread_rank() == 0) {
    auto &s = *state;
    if (!s.controller.done) {
      s.controller.fail(Error::CycleLimit);
    }
    s.result.error = s.controller.error;
    s.result.exit_code = s.controller.exit_code;
    s.result.transactions = s.controller.transactions;
    s.result.phase = s.controller.phase;
    s.result.dmi_address = s.pins.addr;
  }
}
Result run(const Config &config, const std::vector<Poke> &pokes, std::vector<uint8_t> &output) {
  DeviceState state{};
  state.config = config;
  state.controller.tohost = config.tohost;
  state.controller.fromhost = config.fromhost;
  state.controller.ram_base = config.ram_base;
  state.controller.ram_size = config.ram_size;
  state.controller.poll_interval = config.poll_interval;
  state.controller.timeout = config.timeout;
  // Console bytes live in a separate allocation in GPU global memory (1 MiB by
  // default), not in simulated RISC-V RAM. OutputSink appends without wrapping;
  // the host drains it only after completion, so there is no live output stream.
  gpuErrchk(cudaMalloc(&state.output, config.output_capacity));
  if (!pokes.empty()) {
    gpuErrchk(cudaMalloc(&state.pokes, pokes.size() * sizeof(Poke)));
    gpuErrchk(
        cudaMemcpy(state.pokes, pokes.data(), pokes.size() * sizeof(Poke), cudaMemcpyHostToDevice));
  }
  state.poke_count = pokes.size();
  DeviceState *device;
  gpuErrchk(cudaMalloc(&device, sizeof state));
  gpuErrchk(cudaMemcpy(device, &state, sizeof state, cudaMemcpyHostToDevice));
  size_t shared = 0;
  for (const auto &p : gpuPartInfos) {
    shared = std::max(shared, size_t(p.valuePoolSize));
  }
  gpuErrchk(cudaFuncSetAttribute(simulate, cudaFuncAttributeMaxDynamicSharedMemorySize, shared));
  int id;
  gpuErrchk(cudaGetDevice(&id));
  cudaDeviceProp prop;
  gpuErrchk(cudaGetDeviceProperties(&prop, id));
  int threads = prop.maxThreadsPerBlock, resident = 0;
  for (; threads >= 64; threads /= 2) {
    gpuErrchk(cudaOccupancyMaxActiveBlocksPerMultiprocessor(&resident, simulate, threads, shared));
    if (resident) {
      break;
    }
  }
  if (!resident) {
    throw std::runtime_error("GPU cannot launch the controller kernel cooperatively");
  }
  uint32_t r0, r1;
  gpuErrchk(cudaMemcpyFromSymbol(&r0, numParts_Region0, sizeof r0));
  gpuErrchk(cudaMemcpyFromSymbol(&r1, numParts_Region1, sizeof r1));
  int blocks = std::min(int(std::max(r0, r1)), resident * prop.multiProcessorCount);
  std::cout << "Pure GPU kernel: " << blocks << " blocks, " << threads << " threads/block\n";
  void *args[] = {&device};
  gpuErrchk(cudaLaunchCooperativeKernel((void *)simulate, blocks, threads, args, shared));
  gpuErrchk(cudaDeviceSynchronize()); // the only execution-time host wait; no cycle stepping
  gpuErrchk(cudaMemcpy(&state, device, sizeof state, cudaMemcpyDeviceToHost));
  output.resize(state.result.output_size);
  if (!output.empty()) {
    gpuErrchk(cudaMemcpy(output.data(), state.output, output.size(), cudaMemcpyDeviceToHost));
  }
  gpuErrchk(cudaFree(state.output));
  if (state.pokes) {
    gpuErrchk(cudaFree(state.pokes));
  }
  gpuErrchk(cudaFree(device));
  return state.result;
}
} // namespace puregpu
