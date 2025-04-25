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

module VX_opc_unit import VX_gpu_pkg::*; #(
    parameter `STRING INSTANCE_ID = "",
    parameter ISSUE_ID = 0
) (
    input wire              clk,
    input wire              reset,

    input reg [`NUM_OPCS-1:0] wait_mask,

    VX_scoreboard_if.slave  scoreboard_if,
    VX_gpr_if.master        gpr_if,
    VX_operands_if.master   operands_if
);
    `UNUSED_SPARAM (INSTANCE_ID)
    `UNUSED_PARAM (ISSUE_ID)

    localparam NUM_OPDS  = NUM_SRC_OPDS + 1;
    localparam IN_DATAW  = UUID_WIDTH + ISSUE_WIS_W + `NUM_THREADS + PC_BITS + EX_BITS + INST_OP_BITS + INST_ARGS_BITS + NUM_OPDS + (NUM_OPDS * REG_IDX_BITS);
    localparam OUT_DATAW = UUID_WIDTH + ISSUE_WIS_W + SIMD_IDX_W + VL_WIDTH + `SIMD_WIDTH + PC_BITS + EX_BITS + INST_OP_BITS + INST_ARGS_BITS + 1 + NR_BITS + (NUM_SRC_OPDS * `SIMD_WIDTH * `XLEN) + 1 + 1;

    localparam STATE_IDLE     = 0;
    localparam STATE_FETCH    = 1;
    localparam STATE_DISPATCH = 2;
    localparam STATE_WIDTH    = 2;

    VX_scoreboard_if staging_if();

    reg [NUM_SRC_OPDS-1:0] opds_needed, opds_needed_n;
    reg [NUM_SRC_OPDS-1:0] opds_busy, opds_busy_n;
    reg [STATE_WIDTH-1:0] state, state_n;
    wire output_ready;

    wire [`SIMD_WIDTH-1:0] simd_out;
    wire [SIMD_IDX_W-1:0] simd_pid;
    wire simd_sop, simd_eop;

    // hold current instruction
    VX_pipe_buffer #(
        .DATAW (IN_DATAW)
    ) stanging_buf (
        .clk      (clk),
        .reset    (reset),
        .valid_in (scoreboard_if.valid),
        .data_in  (scoreboard_if.data),
        .ready_in (scoreboard_if.ready),
        .valid_out(staging_if.valid),
        .data_out (staging_if.data),
        .ready_out(staging_if.ready)
    );

    // wire enqueue = (state == STATE_IDLE) && staging_if.ready;
    wire dispatch_fire = (state == STATE_DISPATCH) && output_ready;

    // pop current instruction
    assign staging_if.ready = dispatch_fire && simd_eop;

    // Reg File request and response handshake
    wire gpr_req_fire = gpr_if.req_valid && gpr_if.req_ready;
    wire gpr_rsp_fire = gpr_if.rsp_valid;

    // calculate register numbers
    wire [NR_S_BITS-1:0] stg_rd  = to_sreg_number(staging_if.data.rd);
    wire [NR_S_BITS-1:0] stg_rs1 = to_sreg_number(staging_if.data.rs1);
    wire [NR_S_BITS-1:0] stg_rs2 = to_sreg_number(staging_if.data.rs2);
    wire [NR_S_BITS-1:0] stg_rs3 = to_sreg_number(staging_if.data.rs3);
    wire [NUM_SRC_OPDS-1:0][NR_S_BITS-1:0] stg_src_regs = {stg_rs3, stg_rs2, stg_rs1};

    // determine source operands to fetch
    wire [NUM_SRC_OPDS-1:0] opds_to_fetch;
    for (genvar i = 0; i < NUM_SRC_OPDS; ++i) begin : g_opds_to_fetch
        assign opds_to_fetch[i] = staging_if.data.used_rs[i] && (stg_src_regs[i] != 0);
    end

    // control state machine
    always @(*) begin
        state_n = state;
        opds_needed_n = opds_needed;
        opds_busy_n = opds_busy;

        case (state)
        STATE_IDLE: begin
            if (staging_if.valid) begin
                opds_needed_n = opds_to_fetch;
                opds_busy_n = opds_to_fetch;
                if (opds_to_fetch == 0) begin
                    state_n = STATE_DISPATCH;
                end else begin
                    state_n = STATE_FETCH;
                end
            end
        end
        STATE_FETCH: begin
            if (gpr_req_fire) begin
                opds_needed_n[gpr_if.req_data.opd_id] = 0;
            end
            if (gpr_rsp_fire) begin
                opds_busy_n[gpr_if.rsp_data.opd_id] = 0;
            end
            if (opds_busy_n == 0) begin
                state_n = STATE_DISPATCH;
            end
        end
        STATE_DISPATCH: begin
            if (output_ready) begin
                if (simd_eop) begin
                    state_n = STATE_IDLE;
                end else if (opds_to_fetch != 0) begin
                    opds_needed_n = opds_to_fetch;
                    opds_busy_n = opds_to_fetch;
                    state_n = STATE_FETCH;
                end
            end
        end
        endcase
    end

    // select next operand to fetch
    wire [SRC_OPD_WIDTH-1:0] opd_id;
    wire opd_fetch_valid;
    VX_priority_encoder #(
        .N (NUM_SRC_OPDS)
    ) opd_id_sel (
        .data_in   (opds_needed),
        .index_out (opd_id),
        .valid_out (opd_fetch_valid),
        `UNUSED_PIN (onehot_out)
    );

    // create GPR request
    assign gpr_if.req_valid = opd_fetch_valid;
    assign gpr_if.req_data.opd_id = opd_id;
    assign gpr_if.req_data.sid = simd_pid;
    assign gpr_if.req_data.wis = staging_if.data.wis;
    assign gpr_if.req_data.reg_id = stg_src_regs[opd_id];

    // operands buffer
    reg [NUM_SRC_OPDS-1:0][`SIMD_WIDTH-1:0][`XLEN-1:0] opd_values;

    // handle GPR response
    always @(posedge clk) begin
        if (reset || dispatch_fire) begin
            for (integer i = 0; i < NUM_SRC_OPDS; ++i) begin
                opd_values[i] <= '0;
            end
        end else begin
            if (gpr_rsp_fire) begin
                opd_values[gpr_if.rsp_data.opd_id] <= gpr_if.rsp_data.data;
            end
        end
    end

    // state machine update
    always @(posedge clk) begin
        if (reset) begin
            state <= STATE_IDLE;
            opds_needed <= '0;
            opds_busy <= '0;
        end else begin
            state <= state_n;
            opds_needed <= opds_needed_n;
            opds_busy <= opds_busy_n;
        end
    end

    // simd iterator (skip requests with inactive threads)
    VX_nz_iterator #(
        .DATAW   (`SIMD_WIDTH),
        .N       (SIMD_COUNT),
        .OUT_REG (1)
    ) simd_iter (
        .clk     (clk),
        .reset   (reset),
        .valid_in(staging_if.valid),
        .data_in (staging_if.data.tmask),
        .next    (dispatch_fire),
        `UNUSED_PIN (valid_out),
        .data_out(simd_out),
        .pid     (simd_pid),
        .sop     (simd_sop),
        .eop     (simd_eop)
    );

    // hold dispatch until dependency check is ready
    wire dep_check_ready = (wait_mask == 0);
    wire output_ready_w;
    assign output_ready = output_ready_w && dep_check_ready;
    wire output_valid = (state == STATE_DISPATCH) && dep_check_ready;

    // buffer out dispatch response
    VX_elastic_buffer #(
        .DATAW   (OUT_DATAW),
        .SIZE    (0),
        .OUT_REG (0)
    ) out_buf (
        .clk      (clk),
        .reset    (reset),
        .valid_in (output_valid),
        .data_in  ({
            staging_if.data.uuid,
            VL_WIDTH'(0),
            staging_if.data.wis,
            simd_pid,
            simd_out,
            staging_if.data.PC,
            staging_if.data.ex_type,
            staging_if.data.op_type,
            staging_if.data.op_args,
            staging_if.data.wb,
            NR_BITS'(stg_rd),
            opd_values[0],
            opd_values[1],
            opd_values[2],
            simd_sop,
            simd_eop
        }),
        .ready_in (output_ready_w),
        .valid_out(operands_if.valid),
        .data_out (operands_if.data),
        .ready_out(operands_if.ready)
    );

`ifdef DBG_TRACE_PIPELINE
    always @(posedge clk) begin
        if (scoreboard_if.valid && scoreboard_if.ready) begin
            `TRACE(1, ("%t: %s-input: wid=%0d, PC=0x%0h, ex=", $time, INSTANCE_ID, wis_to_wid(scoreboard_if.data.wis, ISSUE_ID), {scoreboard_if.data.PC, 1'b0}))
            trace_ex_type(1, scoreboard_if.data.ex_type);
            `TRACE(1, (", op="))
            trace_ex_op(1, scoreboard_if.data.ex_type, scoreboard_if.data.op_type, scoreboard_if.data.op_args);
            `TRACE(1, (", tmask=%b, wb=%b, used_rs=%b, rd=%0d, rs1=%0d, rs2=%0d, rs3=%0d (#%0d)\n", scoreboard_if.data.tmask, scoreboard_if.data.wb, scoreboard_if.data.used_rs, to_reg_number(scoreboard_if.data.rd), to_reg_number(scoreboard_if.data.rs1), to_reg_number(scoreboard_if.data.rs2), to_reg_number(scoreboard_if.data.rs3), scoreboard_if.data.uuid))
        end
        if (gpr_if.req_valid && gpr_if.req_ready) begin
            `TRACE(1, ("%t: %s-gpr-req: opd=%0d, wis=%0d, sid=%0d, reg=%0d\n", $time, INSTANCE_ID, gpr_if.req_data.opd_id, wis_to_wid(gpr_if.req_data.wis, ISSUE_ID), gpr_if.req_data.sid, gpr_if.req_data.reg_id))
        end
        if (gpr_if.rsp_valid) begin
            `TRACE(1, ("%t: %s-gpr-rsp: opd=%0d, data=", $time, INSTANCE_ID, gpr_if.rsp_data.opd_id))
            `TRACE_ARRAY1D(1, "0x%0h", gpr_if.rsp_data.data, `SIMD_WIDTH)
            `TRACE(1, ("\n"))
        end
        if (operands_if.valid && operands_if.ready) begin
            `TRACE(1, ("%t: %s-output: wid=%0d, sid=%0d, PC=0x%0h, ex=", $time, INSTANCE_ID, wis_to_wid(operands_if.data.wis, ISSUE_ID), operands_if.data.sid, {operands_if.data.PC, 1'b0}))
            trace_ex_type(1, operands_if.data.ex_type);
            `TRACE(1, (", op="))
            trace_ex_op(1, operands_if.data.ex_type, operands_if.data.op_type, operands_if.data.op_args);
            `TRACE(1, (", tmask=%b, wb=%b, rd=%0d, rs1_data=", operands_if.data.tmask, operands_if.data.wb, operands_if.data.rd))
            `TRACE_ARRAY1D(1, "0x%0h", operands_if.data.rs1_data, `SIMD_WIDTH)
            `TRACE(1, (", rs2_data="))
            `TRACE_ARRAY1D(1, "0x%0h", operands_if.data.rs2_data, `SIMD_WIDTH)
            `TRACE(1, (", rs3_data="))
            `TRACE_ARRAY1D(1, "0x%0h", operands_if.data.rs3_data, `SIMD_WIDTH)
            `TRACE(1, (", "))
            trace_op_args(1, operands_if.data.ex_type, operands_if.data.op_type, operands_if.data.op_args);
            `TRACE(1, (", sop=%b, eop=%b (#%0d)\n", operands_if.data.sop, operands_if.data.eop, operands_if.data.uuid))
        end
    end
`endif

endmodule
