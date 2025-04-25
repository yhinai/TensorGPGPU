// Copyright © 2019-2023
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// The intrinsics implemented use RISC-V assembler pseudo-directives defined here:
// https://sourceware.org/binutils/docs/as/RISC_002dV_002dFormats.html

#ifndef __VX_INTRINSICS_H__
#define __VX_INTRINSICS_H__

#include <stddef.h>
#include <stdint.h>
#include <VX_types.h>

#if defined(__clang__)
#define __UNIFORM__   __attribute__((annotate("vortex.uniform")))
#else
#define __UNIFORM__
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define RISCV_CUSTOM0   0x0B
#define RISCV_CUSTOM1   0x2B
#define RISCV_CUSTOM2   0x5B
#define RISCV_CUSTOM3   0x7B

#define RISCV_INSN_R(opcode7, funct3, funct7, rd, rs1, rs2) ( \
    ((funct7 & 0x7F) << 25) | \
    ((rs2 & 0x1F) << 20) | \
    ((rs1 & 0x1F) << 15) | \
    ((funct3 & 0x7) << 12) | \
    ((rd & 0x1F) << 7) | \
    (opcode7 & 0x7F) \
)

#define RISCV_INSN_R4(opcode7, funct3, funct2, rd, rs1, rs2, rs3) ( \
    ((rs3 & 0x1F) << 27) | \
    ((funct2 & 0x3) << 25) | \
    ((rs2 & 0x1F) << 20) | \
    ((rs1 & 0x1F) << 15) | \
    ((funct3 & 0x7) << 12) | \
    ((rd & 0x1F) << 7) | \
    (opcode7 & 0x7F) \
)

#define csr_read(csr) ({                        \
	size_t __r;	               		            \
	__asm__ __volatile__ ("csrr %0, %1" : "=r" (__r) : "i" (csr) : "memory"); \
	__r;							            \
})

#define csr_write(csr, val)	({                  \
	size_t __v = (size_t)(val);                 \
	if (__builtin_constant_p(val) && __v < 32)  \
        __asm__ __volatile__ ("csrw %0, %1" :: "i" (csr), "i" (__v) : "memory");  \
    else                                        \
        __asm__ __volatile__ ("csrw %0, %1"	:: "i" (csr), "r" (__v) : "memory");  \
})

#define csr_swap(csr, val) ({                   \
    size_t __r;                                 \
	size_t __v = (size_t)(val);	                \
	if (__builtin_constant_p(val) && __v < 32)  \
        __asm__ __volatile__ ("csrrw %0, %1, %2" : "=r" (__r) : "i" (csr), "i" (__v) : "memory"); \
    else                                        \
        __asm__ __volatile__ ("csrrw %0, %1, %2" : "=r" (__r) : "i" (csr), "r" (__v) : "memory"); \
	__r;						                \
})

#define csr_read_set(csr, val) ({               \
	size_t __r;                                 \
	size_t __v = (size_t)(val);	                \
    if (__builtin_constant_p(val) && __v < 32)  \
	    __asm__ __volatile__ ("csrrs %0, %1, %2" : "=r" (__r) : "i" (csr), "i" (__v) : "memory"); \
    else                                        \
        __asm__ __volatile__ ("csrrs %0, %1, %2" : "=r" (__r) : "i" (csr), "r" (__v) : "memory"); \
	__r;							            \
})

#define csr_set(csr, val) ({                    \
	size_t __v = (size_t)(val);	                \
    if (__builtin_constant_p(val) && __v < 32)  \
	    __asm__ __volatile__ ("csrs %0, %1"	:: "i" (csr), "i" (__v) : "memory");  \
    else                                        \
        __asm__ __volatile__ ("csrs %0, %1"	:: "i" (csr), "r" (__v) : "memory");  \
})

#define csr_read_clear(csr, val) ({             \
	size_t __r;                                 \
	size_t __v = (size_t)(val);	                \
    if (__builtin_constant_p(val) && __v < 32)  \
	    __asm__ __volatile__ ("csrrc %0, %1, %2" : "=r" (__r) : "i" (csr), "i" (__v) : "memory"); \
    else                                        \
        __asm__ __volatile__ ("csrrc %0, %1, %2" : "=r" (__r) : "i" (csr), "r" (__v) : "memory"); \
	__r;							            \
})

#define csr_clear(csr, val)	({                  \
	size_t __v = (size_t)(val);                 \
	if (__builtin_constant_p(val) && __v < 32)  \
        __asm__ __volatile__ ("csrc %0, %1" :: "i" (csr), "i" (__v) : "memory"); \
    else                                        \
        __asm__ __volatile__ ("csrc %0, %1"	:: "i" (csr), "r" (__v) : "memory"); \
})

// Set thread mask
inline void vx_tmc(int thread_mask) {
    __asm__ volatile (".insn r %0, 0, 0, x0, %1, x0" :: "i"(RISCV_CUSTOM0), "r"(thread_mask));
}

// disable all threads in the current warp
inline void vx_tmc_zero() {
    __asm__ volatile (".insn r %0, 0, 0, x0, x0, x0" :: "i"(RISCV_CUSTOM0));
}

// switch execution to single thread zero
inline void vx_tmc_one() {
    __asm__ volatile (
        "li a0, 1\n\t"  // Load immediate value 1 into a0 (x10) register
        ".insn r %0, 0, 0, x0, a0, x0" :: "i"(RISCV_CUSTOM0) : "a0"
    );
}

// Set thread predicate
inline void vx_pred(int condition, int thread_mask) {
    __asm__ volatile (".insn r %0, 5, 0, x0, %1, %2" :: "i"(RISCV_CUSTOM0), "r"(condition), "r"(thread_mask));
}

// Set thread not predicate
inline void vx_pred_n(int condition, int thread_mask) {
    __asm__ volatile (".insn r %0, 5, 0, x1, %1, %2" :: "i"(RISCV_CUSTOM0), "r"(condition), "r"(thread_mask));
}

// Spawn warps
typedef void (*vx_wspawn_pfn)();
inline void vx_wspawn(int num_warps, vx_wspawn_pfn func_ptr) {
    __asm__ volatile (".insn r %0, 1, 0, x0, %1, %2" :: "i"(RISCV_CUSTOM0), "r"(num_warps), "r"(func_ptr));
}

// Split on a predicate
inline int vx_split(int predicate) {
    int ret;
    __asm__ volatile (".insn r %1, 2, 0, %0, %2, x0" : "=r"(ret) : "i"(RISCV_CUSTOM0), "r"(predicate));
    return ret;
}

// Split on a not predicate
inline int vx_split_n(int predicate) {
    int ret;
    __asm__ volatile (".insn r %1, 2, 0, %0, %2, x1" : "=r"(ret) : "i"(RISCV_CUSTOM0), "r"(predicate));
    return ret;
}

// Join
inline void vx_join(int stack_ptr) {
    __asm__ volatile (".insn r %0, 3, 0, x0, %1, x0" :: "i"(RISCV_CUSTOM0), "r"(stack_ptr));
}

// Warp Barrier
inline void vx_barrier(int barried_id, int num_warps) {
    __asm__ volatile (".insn r %0, 4, 0, x0, %1, %2" :: "i"(RISCV_CUSTOM0), "r"(barried_id), "r"(num_warps));
}

// Return current thread identifier
inline int vx_thread_id() {
    int ret;
    __asm__ volatile ("csrr %0, %1" : "=r"(ret) : "i"(VX_CSR_THREAD_ID));
    return ret;
}

// Return current warp identifier
inline int vx_warp_id() {
    int ret;
    __asm__ volatile ("csrr %0, %1" : "=r"(ret) : "i"(VX_CSR_WARP_ID));
    return ret;
}

// Return current core identifier
inline int vx_core_id() {
    int ret;
    __asm__ volatile ("csrr %0, %1" : "=r"(ret) : "i"(VX_CSR_CORE_ID));
    return ret;
}

// Return active threads mask
inline int vx_active_threads() {
    int ret;
    __asm__ volatile ("csrr %0, %1" : "=r"(ret) : "i"(VX_CSR_ACTIVE_THREADS));
    return ret;
}

// Return active warps mask
inline int vx_active_warps() {
    int ret;
    __asm__ volatile ("csrr %0, %1" : "=r"(ret) : "i"(VX_CSR_ACTIVE_WARPS));
    return ret;
}

// Return the number of threads per warp
inline int vx_num_threads() {
    int ret;
    __asm__ volatile ("csrr %0, %1" : "=r"(ret) : "i"(VX_CSR_NUM_THREADS));
    return ret;
}

// Return the number of warps per core
inline int vx_num_warps() {
    int ret;
    __asm__ volatile ("csrr %0, %1" : "=r"(ret) : "i"(VX_CSR_NUM_WARPS));
    return ret;
}

// Return the number of cores per cluster
inline int vx_num_cores() {
    int ret;
    __asm__ volatile ("csrr %0, %1" : "=r"(ret) : "i"(VX_CSR_NUM_CORES));
    return ret;
}

// Return the hart identifier (thread id accross the processor)
inline int vx_hart_id() {
    int ret;
    __asm__ volatile ("csrr %0, %1" : "=r"(ret) : "i"(VX_CSR_MHARTID));
    return ret;
}

inline void vx_fence() {
    __asm__ volatile ("fence iorw, iorw");
}

// Tensor unit intrinsics

typedef float mf32x8_t   __attribute__((vector_size(32))); // 8 x f32 registers
typedef int32_t mi32x8_t __attribute__((vector_size(32))); // 8 x i32 registers
typedef int16_t mi16x8_t __attribute__((vector_size(16))); // 8 x i16 registers
typedef int8_t  mi8x8_t  __attribute__((vector_size(8)));  // 8 x i8 registers

inline mf32x8_t vx_mmadd_f32(mf32x8_t a, mf32x8_t b, mf32x8_t c) {
    mf32x8_t ret;

    register float fa0 __asm__("f0") = a[0];
    register float fa1 __asm__("f1") = a[1];
    register float fa2 __asm__("f2") = a[2];
    register float fa3 __asm__("f3") = a[3];
    register float fa4 __asm__("f4") = a[4];
    register float fa5 __asm__("f5") = a[5];
    register float fa6 __asm__("f6") = a[6];
    register float fa7 __asm__("f7") = a[7];

    register float fb0 __asm__("f8") = b[0];
    register float fb1 __asm__("f9") = b[1];
    register float fb2 __asm__("f10") = b[2];
    register float fb3 __asm__("f11") = b[3];
    register float fb4 __asm__("f12") = b[4];
    register float fb5 __asm__("f13") = b[5];
    register float fb6 __asm__("f14") = b[6];
    register float fb7 __asm__("f15") = b[7];

    register float fc0 __asm__("f16") = c[0];
    register float fc1 __asm__("f17") = c[1];
    register float fc2 __asm__("f18") = c[2];
    register float fc3 __asm__("f19") = c[3];
    register float fc4 __asm__("f20") = c[4];
    register float fc5 __asm__("f21") = c[5];
    register float fc6 __asm__("f22") = c[6];
    register float fc7 __asm__("f23") = c[7];

    register float fd0 __asm__("f24");
    register float fd1 __asm__("f25");
    register float fd2 __asm__("f26");
    register float fd3 __asm__("f27");
    register float fd4 __asm__("f28");
    register float fd5 __asm__("f29");
    register float fd6 __asm__("f30");
    register float fd7 __asm__("f31");

    __asm__ volatile (
        ".word %8"
        : "=f"(fd0), "=f"(fd1), "=f"(fd2), "=f"(fd3), "=f"(fd4), "=f"(fd5), "=f"(fd6), "=f"(fd7)
        : "i"(RISCV_INSN_R4(RISCV_CUSTOM1, 0, 0, 24, 0, 8, 16)),
          "f"(fa0), "f"(fa1), "f"(fa2), "f"(fa3), "f"(fa4), "f"(fa5), "f"(fa6), "f"(fa7),
          "f"(fb0), "f"(fb1), "f"(fb2), "f"(fb3), "f"(fb4), "f"(fb5), "f"(fb6), "f"(fb7),
          "f"(fc0), "f"(fc1), "f"(fc2), "f"(fc3), "f"(fc4), "f"(fc5), "f"(fc6), "f"(fc7)
        : "memory"
    );

    ret[0] = fd0;
    ret[1] = fd1;
    ret[2] = fd2;
    ret[3] = fd3;
    ret[4] = fd4;
    ret[5] = fd5;
    ret[6] = fd6;
    ret[7] = fd7;

    return ret;
}

inline mf32x8_t vx_mload_f32(size_t addr) {
    mf32x8_t ret;

    register size_t a0 __asm__("a0") = addr;

    register float fd0 __asm__("f0");
    register float fd1 __asm__("f1");
    register float fd2 __asm__("f2");
    register float fd3 __asm__("f3");
    register float fd4 __asm__("f4");
    register float fd5 __asm__("f5");
    register float fd6 __asm__("f6");
    register float fd7 __asm__("f7");

    __asm__ volatile (
        ".word %8"
        : "=f"(fd0), "=f"(fd1), "=f"(fd2), "=f"(fd3), "=f"(fd4), "=f"(fd5), "=f"(fd6), "=f"(fd7)
        : "i"(RISCV_INSN_R(RISCV_CUSTOM1, 0, 1, 0, 10, 0)),
          "r"(a0)
        : "memory"
    );

    ret[0] = fd0;
    ret[1] = fd1;
    ret[2] = fd2;
    ret[3] = fd3;
    ret[4] = fd4;
    ret[5] = fd5;
    ret[6] = fd6;
    ret[7] = fd7;

    return ret;
}

inline void vx_mstore_f32(size_t addr, mf32x8_t data) {
    register size_t a0 __asm__("a0") = addr;

    register float fa0 __asm__("f0") = data[0];
    register float fa1 __asm__("f1") = data[1];
    register float fa2 __asm__("f2") = data[2];
    register float fa3 __asm__("f3") = data[3];
    register float fa4 __asm__("f4") = data[4];
    register float fa5 __asm__("f5") = data[5];
    register float fa6 __asm__("f6") = data[6];
    register float fa7 __asm__("f7") = data[7];

    __asm__ volatile (
        ".word %0"
        :
        : "i"(RISCV_INSN_R(RISCV_CUSTOM1, 1, 1, 0, 10, 0)),
          "r"(a0),
          "f"(fa0), "f"(fa1), "f"(fa2), "f"(fa3), "f"(fa4), "f"(fa5), "f"(fa6), "f"(fa7)
        : "memory"
    );
}

#ifdef __cplusplus
}
#endif

#endif // __VX_INTRINSICS_H__
