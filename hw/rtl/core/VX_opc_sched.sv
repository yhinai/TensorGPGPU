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

module VX_opc_sched import VX_gpu_pkg::*; #(
    parameter `STRING INSTANCE_ID = "",
    parameter ISSUE_ID = 0
) (
    input wire              clk,
    input wire              reset,

    VX_scoreboard_if.slave  enqueue_if,
    input wire              dequeue,
    input wire [OPC_WIDTH-1:0] enqueue_opc,
    input wire [OPC_WIDTH-1:0] dequeue_opc,
    output wire [`NUM_OPCS-1:0][`NUM_OPCS-1:0] opc_wait_mask,
    output wire [`NUM_OPCS-1:0] opc_busy
);
    `UNUSED_SPARAM (INSTANCE_ID)
    `UNUSED_PARAM (ISSUE_ID)

    wire enqueue = enqueue_if.valid && enqueue_if.ready;

    wire [NR_BITS-1:0] scb_rd  = to_reg_number(enqueue_if.data.rd);
    wire [NR_BITS-1:0] scb_rs1 = to_reg_number(enqueue_if.data.rs1);
    wire [NR_BITS-1:0] scb_rs2 = to_reg_number(enqueue_if.data.rs2);
    wire [NR_BITS-1:0] scb_rs3 = to_reg_number(enqueue_if.data.rs3);

    wire [NUM_SRC_OPDS-1:0][NR_BITS-1:0] scb_src_regs = {scb_rs3, scb_rs2, scb_rs1};

    reg [NUM_REGS-1:0] scb_pending_regs;
    always @(*) begin
        scb_pending_regs = '0;
        for (integer i = 0; i < NUM_SRC_OPDS; ++i) begin
            if (enqueue_if.data.used_rs[i]) begin
                scb_pending_regs[scb_src_regs[i]] = 1;
            end
        end
    end

    reg [`NUM_OPCS-1:0] per_opc_busy;
    reg [`NUM_OPCS-1:0][NUM_REGS-1:0] per_opc_pending_regs;
    reg [`NUM_OPCS-1:0][ISSUE_WIS_W-1:0] per_opc_pending_wis;
    reg [`NUM_OPCS-1:0] per_opc_pending_lsu;
    reg [`NUM_OPCS-1:0][`NUM_OPCS-1:0] per_opc_wait_mask;

    // LD/ST memory instrctions should be issued in order
    wire scoreboard_is_lsu = (enqueue_if.data.ex_type == EX_LSU);

    always @(posedge clk) begin
        if (reset) begin
            per_opc_busy         <= '0;
            per_opc_pending_regs <= '0;
            per_opc_pending_wis  <= '0;
            per_opc_pending_lsu  <= '0;
            per_opc_wait_mask    <= '0;
        end else begin
            if (enqueue) begin
                for (int i = 0; i < `NUM_OPCS; ++i) begin
                    if (((per_opc_pending_regs[i][scb_rd] != 0 && per_opc_pending_wis[i] == enqueue_if.data.wis)
                      || (per_opc_pending_lsu[i] && scoreboard_is_lsu))
                    && ~(dequeue && dequeue_opc == OPC_WIDTH'(i))) begin
                        per_opc_wait_mask[enqueue_opc][i] <= 1;
                    end
                end
                per_opc_busy[enqueue_opc]         <= 1;
                per_opc_pending_regs[enqueue_opc] <= scb_pending_regs;
                per_opc_pending_wis[enqueue_opc]  <= enqueue_if.data.wis;
                per_opc_pending_lsu[enqueue_opc]  <= scoreboard_is_lsu;
            end
            if (dequeue) begin
                for (int i = 0; i < `NUM_OPCS; ++i) begin
                    if (per_opc_wait_mask[i][dequeue_opc]) begin
                        per_opc_wait_mask[i][dequeue_opc] <= 0;
                    end
                end
                per_opc_busy[dequeue_opc]         <= '0;
                per_opc_pending_regs[dequeue_opc] <= '0;
                per_opc_pending_wis[dequeue_opc]  <= '0;
                per_opc_pending_lsu[dequeue_opc]  <= 0;
            end
        end
    end

    assign opc_wait_mask = per_opc_wait_mask;
    assign opc_busy = per_opc_busy;

endmodule
