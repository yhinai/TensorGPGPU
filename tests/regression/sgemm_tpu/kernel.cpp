#include <vx_spawn.h>
#include "common.h"

int main() {
	kernel_arg_t* arg = (kernel_arg_t*)csr_read(VX_CSR_MSCRATCH);
	//return vx_spawn_threads(2, arg->grid_dim, nullptr, (vx_kernel_func_cb)kernel_body, arg);

    //float x = *(float*)(arg->A_addr);
    //float y = *(float*)(arg->B_addr);

    mf32x8_t A = vx_mload_f32(0x1000);
    mf32x8_t B = vx_mload_f32(0x2000);
    mf32x8_t C = vx_mload_f32(0x3000);

    //mf32x8_t A = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0};
    //mf32x8_t B = {2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
    //mf32x8_t C = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};

    mf32x8_t D = vx_mmadd_f32(A, B, C);

    *(float*)(0x10000)=D[0];
    *(float*)(0x10010)=D[1];
    *(float*)(0x10020)=D[2];
    *(float*)(0x10030)=D[3];
    *(float*)(0x10040)=D[4];
    *(float*)(0x10050)=D[5];
    *(float*)(0x10060)=D[6];
    *(float*)(0x10070)=D[7];

    uint32_t d = 0x4000;
    vx_mstore_f32(d, D);

    //float z = x + y;

    //*(float*)(arg->C_addr) = z;


    return 0;
}
