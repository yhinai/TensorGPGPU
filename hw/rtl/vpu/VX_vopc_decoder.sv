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

`include "VX_define.vh"

module VX_vopc_decoder import VX_gpu_pkg::*; #(
    parameter `STRING INSTANCE_ID = "",
    parameter ISSUE_ID = 0
) (
    input wire          clk,
    input wire          reset,

    VX_vpu_states_if.master vpu_states_if,

    input wire          valid,

    input instr_data_t  instr_in,
    output instr_data_t instr_out
);

    `UNUSED_SPARAM (INSTANCE_ID)

    `UNUSED_VAR (reset)
    `UNUSED_VAR (valid)
    `UNUSED_VAR (instr_in)

    logic valid_r;

    logic [EX_BITS-1:0]                ex_type_r, ex_type_n;
    logic [INST_OP_BITS-1:0]           op_type_r, op_type_n;
    op_args_t                          op_args_r, op_args_n;
    logic                              wb_r, wb_n;
    logic [NR_BITS-1:0]                rd_r, rd_n;
    logic [`SIMD_WIDTH-1:0][`XLEN-1:0] rs1_data_r, rs1_data_n;
    logic [`SIMD_WIDTH-1:0][`XLEN-1:0] rs2_data_r, rs2_data_n;
    logic [`SIMD_WIDTH-1:0][`XLEN-1:0] rs3_data_r, rs3_data_n;
    logic is_vset_r, is_vset_n;

    vpu_states_t [PER_ISSUE_WARPS-1:0] vpu_states;

    always @(*) begin
        is_vset_n   = 0;
        ex_type_n   = 'x;
        op_type_n   = 'x;
        op_args_n   = 'x;
        wb_n        = 'x;
        rd_n        = 'x;
        rs1_data_n  = 'x;
        rs2_data_n  = 'x;
        rs3_data_n  = 'x;
    end

    always @(posedge clk) begin
        if (reset) begin
            vpu_states <= '0;
        end
        valid_r     <= valid;
        is_vset_r   <= is_vset_n;
        ex_type_r   <= ex_type_n;
        op_type_r   <= op_type_n;
        op_args_r   <= op_args_n;
        wb_r        <= wb_n;
        rd_r        <= rd_n;
        rs1_data_r  <= rs1_data_n;
        rs2_data_r  <= rs2_data_n;
        rs3_data_r  <= rs3_data_n;
    end

    // vpu states
    assign vpu_states_if.valid = valid_r && is_vset_r;
    assign vpu_states_if.wid   = wis_to_wid(instr_in.wis, ISSUE_ID);
    assign vpu_states_if.data  = vpu_states[instr_in.wis];

    // decoded instruction
    assign instr_out.uuid       = instr_in.uuid;
    assign instr_out.lid        = instr_in.lid;
    assign instr_out.wis        = instr_in.wis;
    assign instr_out.sid        = instr_in.sid;
    assign instr_out.tmask      = instr_in.tmask;
    assign instr_out.PC         = instr_in.PC;
    assign instr_out.ex_type    = ex_type_r;
    assign instr_out.op_type    = op_type_r;
    assign instr_out.op_args    = op_args_r;
    assign instr_out.wb         = wb_r;
    assign instr_out.rd         = rd_r;
    assign instr_out.rs1_data   = rs1_data_r;
    assign instr_out.rs2_data   = rs2_data_r;
    assign instr_out.rs3_data   = rs3_data_r;
    assign instr_out.sop        = instr_in.sop;
    assign instr_out.eop        = instr_in.eop;

endmodule
