# Toucan pure-GPU demo

A demo for Toucan using example RISC-V cores, with simulation and program I/O
running entirely on the GPU.
Use the supplied design archive with this emulator.

## Run

Requires Linux, CUDA (tested build: 13.1), GNU make, and a C++20 compiler.
The default GPU target is `sm_89`; override with `make CUDA_ARCH=...`.

```sh
git submodule update --init --recursive
# Download the design archive and its .sha256 file from the release assets first.
sha256sum -c toucan-pure-gpu-designs.tar.gz.sha256
tar -xzf toucan-pure-gpu-designs.tar.gz
make -j4
./emulator designs/rocket21-1c riscv-benchmarks-bin/vvadd.riscv
# Or: ./emulator designs/boom21-small riscv-benchmarks-bin/vvadd.riscv
```

Output appears after simulation finishes. Run `./emulator` for command-line options.

## Check designs

```sh
make test
./emulator designs/rocket21-1c riscv-benchmarks-bin/vvadd.riscv --inspect
```

## All design files

`toucan-pure-gpu-designs.tar.gz` packages all 16 paper designs. Each
`designs/<name>/` contains the modified FIRRTL, freshly lowered MLIR, GPU bytecode,
and both symbol files. The included `designs/manifest.json` records SHA-256 hashes
for every design file.

Git contains only the FIRRTL and MLIR sources for `rocket21-1c` and `boom21-small`.
All GPU bytecode and symbol binaries, plus the other 14 designs, are distributed
in the release archive and ignored by Git after extraction.

## Implementation notes

Source comments document the supported workloads and runtime behavior:

- [include/Image.h](include/Image.h): ELF requirements, boot vector, and memory preload.
- [include/Controller.h](include/Controller.h): supported HTIF operations and debug access.
- [include/PureGpu.h](include/PureGpu.h): cycle limits, polling, and register deposits.
- [runtime/src/PureGpuKernel.cuh](runtime/src/PureGpuKernel.cuh): GPU execution and output buffering.
- [src/main.cpp](src/main.cpp): command-line handling, inspection, and timing.
