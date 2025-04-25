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

module VX_operands import VX_gpu_pkg::*; #(
    parameter `STRING INSTANCE_ID = "",
    parameter ISSUE_ID = 0
) (
    input wire              clk,
    input wire              reset,

`ifdef PERF_ENABLE
    output wire [PERF_CTR_BITS-1:0] perf_stalls,
`endif

    VX_writeback_if.slave   writeback_if,
    VX_scoreboard_if.slave  scoreboard_if,
    VX_operands_if.master   operands_if
);
    localparam NUM_OPDS  = NUM_SRC_OPDS + 1;
    localparam SCB_DATAW = UUID_WIDTH + ISSUE_WIS_W + `NUM_THREADS + PC_BITS + EX_BITS + INST_OP_BITS + INST_ARGS_BITS + NUM_OPDS + (REG_IDX_BITS * NUM_OPDS);
    localparam OPD_DATAW = UUID_WIDTH + VL_WIDTH + ISSUE_WIS_W + SIMD_IDX_W + `SIMD_WIDTH + PC_BITS + EX_BITS + INST_OP_BITS + INST_ARGS_BITS + 1 + NR_BITS + (NUM_SRC_OPDS * `SIMD_WIDTH * `XLEN) + 1 + 1;

    VX_scoreboard_if per_opc_scoreboard_if[`NUM_OPCS]();
    VX_operands_if per_opc_operands_if[`NUM_OPCS]();
    VX_gpr_if per_opc_gpr_if[`NUM_OPCS]();

    wire [OPC_WIDTH-1:0] enqueue_opc, dequeue_opc;
    wire [`NUM_OPCS-1:0][`NUM_OPCS-1:0] per_opc_wait_mask;
    wire [`NUM_OPCS-1:0] per_opc_busy;

    wire operands_fire = operands_if.valid && operands_if.ready;
    wire operands_eop_fire = operands_fire && operands_if.data.eop;

    VX_opc_sched #(
        .INSTANCE_ID (`SFORMATF(("%s-sched", INSTANCE_ID))),
        .ISSUE_ID    (ISSUE_ID)
    ) opc_sched (
        .clk          (clk),
        .reset        (reset),
        .enqueue_if   (scoreboard_if),
        .dequeue      (operands_eop_fire),
        .enqueue_opc  (enqueue_opc),
        .dequeue_opc  (dequeue_opc),
        .opc_wait_mask(per_opc_wait_mask),
        .opc_busy     (per_opc_busy)
    );

`IGNORE_UNOPTFLAT_BEGIN
    `AOS_TO_ITF (per_opc_scoreboard, per_opc_scoreboard_if, `NUM_OPCS, SCB_DATAW)
`IGNORE_UNOPTFLAT_END

    // collector unit selection

    reg [`NUM_OPCS-1:0] select_opcs;
    always @(*) begin
        select_opcs = ~per_opc_busy;
    end

    VX_stream_arb #(
        .NUM_INPUTS  (1),
        .NUM_OUTPUTS (`NUM_OPCS),
        .DATAW       (SCB_DATAW),
        .ARBITER     ("P"),
        .OUT_BUF     (0)
    ) input_arb (
        .clk       (clk),
        .reset     (reset),
        .valid_in  (scoreboard_if.valid),
        .data_in   (scoreboard_if.data),
        .ready_in  (scoreboard_if.ready),
        .valid_out (per_opc_scoreboard_valid),
        .data_out  (per_opc_scoreboard_data),
        .ready_out (select_opcs),
        .sel_out   (enqueue_opc)
    );
    `UNUSED_VAR (per_opc_scoreboard_ready)

    for (genvar i = 0; i < `NUM_OPCS; ++i) begin : g_collectors
        VX_opc_unit #(
            .INSTANCE_ID (`SFORMATF(("%s-collector%0d", INSTANCE_ID, i))),
            .ISSUE_ID (ISSUE_ID)
        ) opc_unit (
            .clk          (clk),
            .reset        (reset),
            .wait_mask    (per_opc_wait_mask[i]),
            .scoreboard_if(per_opc_scoreboard_if[i]),
            .gpr_if       (per_opc_gpr_if[i]),
            .operands_if  (per_opc_operands_if[i])
        );
    end

    VX_gpr_unit #(
        .INSTANCE_ID (`SFORMATF(("%s-gpr", INSTANCE_ID))),
        .NUM_REQS    (`NUM_OPCS),
        .NUM_BANKS   (`NUM_GPR_BANKS)
    ) gpr_unit (
        .clk          (clk),
        .reset        (reset),
    `ifdef PERF_ENABLE
        .perf_stalls  (perf_stalls),
    `endif
        .writeback_if (writeback_if),
        .gpr_if       (per_opc_gpr_if)
    );

    `ITF_TO_AOS (per_opc_operands, per_opc_operands_if, `NUM_OPCS, OPD_DATAW)

    VX_stream_arb #(
        .NUM_INPUTS  (`NUM_OPCS),
        .NUM_OUTPUTS (1),
        .DATAW       (OPD_DATAW),
        .ARBITER     ("P"),
        .OUT_BUF     (3)
    ) output_arb (
        .clk       (clk),
        .reset     (reset),
        .valid_in  (per_opc_operands_valid),
        .data_in   (per_opc_operands_data),
        .ready_in  (per_opc_operands_ready),
        .valid_out (operands_if.valid),
        .data_out  (operands_if.data),
        .ready_out (operands_if.ready),
        .sel_out   (dequeue_opc)
    );

endmodule
