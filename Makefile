NVCC ?= /usr/local/cuda/bin/nvcc
CXX ?= g++
CUDA_ARCH ?= sm_89
FLAGS := -O3 -std=c++20 --maxrregcount=64 -arch=$(CUDA_ARCH) -lineinfo -Iinclude -Iruntime/include
SOURCES := $(wildcard runtime/src/*.cpp runtime/src/*.cu) src/main.cpp
OBJECTS := $(addprefix build/,$(addsuffix .o,$(SOURCES)))
HEADERS := $(wildcard include/*.h runtime/include/*.h runtime/src/*.cuh)
.PHONY: all test clean
all: emulator
emulator: $(OBJECTS)
	$(NVCC) $(FLAGS) $^ -o $@
build/%.o: % $(HEADERS)
	mkdir -p $(dir $@)
	$(NVCC) $(FLAGS) -c $< -o $@
build/controller-test: tests/controller.cpp include/Controller.h
	mkdir -p build
	$(CXX) -std=c++20 -O2 -Wall -Wextra -Werror -Iinclude $< -o $@
build/image-test: tests/image.cpp include/Image.h
	mkdir -p build
	$(CXX) -std=c++20 -O2 -Wall -Wextra -Werror -Iinclude -Iruntime/include $< -o $@
test: build/controller-test build/image-test
	./build/controller-test
	./build/image-test riscv-benchmarks-bin/vvadd.riscv
clean:
	rm -rf build/runtime build/src emulator build/controller-test build/image-test
