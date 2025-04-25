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

module VX_vredopc_unit import VX_gpu_pkg::*; #(
    parameter `STRING INSTANCE_ID = "",
    parameter ISSUE_ID = 0
) (
    input wire              clk,
    input wire              reset,

    input reg [`NUM_OPCS-1:0] wait_mask,

    // Scoreboard Interface
    VX_scoreboard_if.slave  scoreboard_if,

    // Writeback interface
    VX_writeback_if.slave   writeback_in_if,

    VX_writeback_if.master   writeback_out_if, // -> To vgpr writeback interface

    // Vector Reg File
    VX_vgpr_if.master       vgpr_if,

    // To Dispatch Unit
    VX_operands_if.master   operands_if
);
    `UNUSED_SPARAM (INSTANCE_ID)
    `UNUSED_PARAM (ISSUE_ID)

    localparam NUM_OPDS  = NUM_SRC_OPDS + 1;
    localparam SCB_DATAW = UUID_WIDTH + ISSUE_WIS_W + `NUM_THREADS + PC_BITS + EX_BITS + INST_OP_BITS + INST_ARGS_BITS + NUM_OPDS + (NUM_OPDS * REG_IDX_BITS);
    localparam OUT_DATAW = UUID_WIDTH + ISSUE_WIS_W + SIMD_IDX_W + VL_WIDTH + `SIMD_WIDTH + PC_BITS + EX_BITS + INST_OP_BITS + INST_ARGS_BITS + 1 + NR_BITS + (NUM_SRC_OPDS * `SIMD_WIDTH * `XLEN) + 1 + 1;

    localparam STATE_IDLE      = 0;
    localparam STATE_FETCH     = 1;
    localparam STATE_DISPATCH  = 2;
    localparam STATE_WIDTH     = 2;

    VX_scoreboard_if staging_if();

    wire output_ready;

    wire [`SIMD_WIDTH-1:0] simd_out;
    wire [SIMD_IDX_W-1:0] simd_pid;
    wire simd_sop, simd_eop;

    // Just a pipeline buffer to hold outputs from SB
    VX_pipe_buffer #(
        .DATAW (SCB_DATAW)
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

    // Calculate Register Numbers
    wire [NR_BITS-1:0] stg_rd  = to_reg_number(staging_if.data.rd);
    wire [NR_BITS-1:0] stg_rs1 = to_reg_number(staging_if.data.rs1);
    wire [NR_BITS-1:0] stg_rs2 = to_reg_number(staging_if.data.rs2);

    // Determine source operands to fetch
    wire opds_to_fetch_rs1 = staging_if.data.used_rs[0] && (stg_rs1 != 0) ;
    wire opds_to_fetch_rs2 = staging_if.data.used_rs[1] && (stg_rs2 != 0) ;

    // First opd to fetch
    wire [NUM_SRC_OPDS-1:0] opds_to_fetch_first  = {0, opds_to_fetch_rs2, opds_to_fetch_rs1};
    wire [NUM_SRC_OPDS-1:0] opds_to_fetch_second = {0, opds_to_fetch_rs2, 0};



    // ** SubModule 3 : Request and Response fire signals **
    // GP Reg File: Fire Request + Fire Response
    wire gpr_req_fire = gpr_if.req_valid && gpr_if.req_ready;
    wire gpr_rsp_fire = gpr_if.rsp_valid;

    // Vec Reg File: Fire Request + Fire Response
    wire vgpr_req_fire = vgpr_if.req_valid && vgpr_if.req_ready;
    wire vgpr_rsp_fire = vgpr_if.rsp_valid;

    // ** SubModule 4 : Dequeue Signals **

    // dequeue         : True if a request is sent out to dispatch
    // next_simd       : For reduction unit, next_simd condition is same as dequeue/any dispatch
    // simd_last_packet: Reset once all simd has been fired, then switch to next lane
    // last_dispatch   : Special case of next_simd when eop
    wire dequeue          = (state == STATE_DISPATCH) && output_ready;
    wire next_simd        = dequeue;
    wire simd_last_packet = dequeue && simd_eop;
    wire last_dispatch    = dequeue && (lane_counter == VL_BITS'(VL_COUNT)) && (simd_eop);


    // Pop from staging buff
    assign staging_if.ready = last_dispatch;

    // ** SubModule 5 : Writeback to register file **

    // TO FIX : Not sure if this works
    assign writeback_out_if.data = writeback_in_if.data;


    // ** SubModule 6 : Handle Writeback Interface **
    // TO FIX: Not sure if I'm correct for this logic

    reg [SIMD_COUNT-1:0][`SIMD_WIDTH-1:0][`XLEN-1:0] temp_data;
    //reg [`SIMD_WIDTH-1:0] temp_simd;
    reg [SIMD_IDX_W] simd_rsp_ctr;

    always @(posedge clk) begin
        if (reset) begin
            simd_rsp_ctr <= 0;
        end else begin
            if (writeback_in_if.valid && writeback_in_if.data.PC == staging_if.data.PC) begin
                simd_rsp_ctr <= simd_rsp_ctr + 1;
            end
        end

        if (writeback_in_if.valid && writeback_in_if.data.PC == staging_if.data.PC) begin
            for (integer i = 0; i < `SIMD_WIDTH; ++i) begin
                if (writeback_in_if.data.tmask[i]) begin
                    temp_data[writeback_in_if.data.sid][i] <= writeback_in_if.data.data[i];
                end
            end
        end
    end

    // Accumulates Partial Writes from WB interface (for reduction)
    // Check if is reduction --> Using PC value to check
    // NOTE: Need a better way of determining is_reduction_signal
    for (genvar i = 0; i < SIMD_WIDTH; i++) begin

        if (wb_state == STATE_COLLECTION) begin

            // Writeback to register file
            if (writeback_in_if.data.eop == 1) begin

                // TO FIX : Not sure if this works
                writeback_out_if.valid = 1;

            end else if (writeback_in_if.data.PC == staging_if.data.PC)
                && (writeback_in_if.data.tmask[writeback_in_if.data.sid * SIMD_WIDTH + i] == 1)) begin

                //temp_data[i] = writeback_in_if.data.data[simd_id * SIMD_WIDTH + i];
               // temp_simd[i] = 1;

                writeback_out_if.valid = 0;

            end

        end else if (wb_state == STATE_DISPATCH_READY) begin
            //temp_simd[i] = '0;
            //temp_data[i] = '0;
            writeback_out_if.valid = 0;
        end
    end

    // TO FIX : Need to find a way to know if all simd collected
    // Check if all writeback dispatches have been received
    wire all_simd_collected;

    // Basic State Machine to control partial write states
    localparam STATE_COLLECTION     = 0;
    localparam STATE_DISPATCH_READY = 1;
    reg wb_state, wb_state_n;
    reg [`SIMD_WIDTH-1:0] process_simd;
    reg [`SIMD_WIDTH-1:0][`XLEN-1:0] process_data;
    reg wb_collection_ready;

    always@(*) begin
        wb_state_n = wb_state;

        case (wb_state)
        STATE_COLLECTION: begin

            // Checks when the last packet is sent
            // Statement is guranteed to occur before the "all_simd_collected"
            if(simd_last_packet) begin
                wb_collection_ready = 0;
            end

            // Checks when the last packet is received
            if(all_simd_collected) begin
                wb_state_n = STATE_DISPATCH_READY;
            end

        end

        STATE_DISPATCH_READY: begin
            wb_state_n = STATE_COLLECTION;

            process_simd = temp_simd;
            process_data = temp_data;
            wb_collection_ready = 1;
        end
        endcase

    end

    always@(posedge clk) begin
        if(reset) begin
            wb_state <= 1;
        end else begin
            wb_state <= wb_state_n;
        end
    end




    // ** SubModule 8 : FSM for rf **
    // Use centralized state machine for both gpr and vrf
    reg [STATE_WIDTH-1:0]  state, state_n;
    reg [NUM_SRC_OPDS-1:0] v_opds_needed, v_opds_needed_n;
    reg [NUM_SRC_OPDS-1:0] v_opds_busy, v_opds_busy_n;

    reg [VL_WIDTH-1:0] lane_counter, lane_counter_n;

    reg in_first_dispatch;

    // TO FIX: NEED TO KNOW ACTUAL SIZE
    // reg ext_counter, ext_counter_n;

    always @(*) begin
        state_n = state;
        v_opds_needed_n = v_opds_needed;
        v_opds_busy_n = v_opds_busy;

        lane_counter_n = lane_counter;

        case (state)

        STATE_IDLE: begin
            if (staging_if.valid) begin
                v_opds_needed_n = opds_to_fetch_first;
                v_opds_busy_n = opds_to_fetch_first;

                lane_counter_n = '0;
                in_first_dispatch = 1;

                if (opds_to_fetch_first == 0) begin
                    state_n = STATE_DISPATCH;
                end else begin
                    state_n = STATE_FETCH;
                end

            end
        end

        STATE_FETCH: begin
            if (vgpr_req_fire) begin
                v_opds_needed_n[vgpr_if.req_data.opd_id] = 0;
            end
            if (vgpr_rsp_fire) begin
                v_opds_busy_n[vgpr_if.rsp_data.opd_id] = 0;
            end

            if (v_opds_busy_n == 0) begin
                state_n = STATE_DISPATCH;
            end
        end

        STATE_DISPATCH: begin

           // if (output_ready) begin <-- Don't need this since following conditions require output_ready

               // Different scheduling algo than vopc:
               // In vopc: Finish lane -> Then next simd
               // Here   : Finish simd (eop) -> Then increment lane counter
               if(simd_last_packet) begin

                    // Update Signal for Fetch to know
                    in_first_dispatch = 0;

                    // Next lane
                    if (lane_counter == VL_BITS'(VL_COUNT)) begin

                        // Ending Condition
                        lane_counter_n = '0;
                        state_n = STATE_IDLE;

                    end else begin
                        lane_counter_n = lane_counter + 1;
                    end


                    // Here means to start the "next round/lane"
                    // Don't start until all simd have been collected
                    // Probably "clearer" to have more states to clarify
                    if ( (opds_to_fetch_second != 0) && (wb_collection_ready) ) begin
                        v_opds_needed_n = opds_to_fetch_second;
                        v_opds_busy_n = opds_to_fetch_second;
                        state_n = STATE_FETCH;
                    end

                // Continue Dispatch but next SIMD
               end else if (dequeue) begin

                    // Second mask fetch
                    if (opds_to_fetch_first != 0) begin

                        if(in_first_dispatch) begin
                            v_opds_needed_n = opds_to_fetch_first;
                            v_opds_busy_n = opds_to_fetch_first;
                        end else begin
                            v_opds_needed_n = opds_to_fetch_second;
                            v_opds_busy_n = opds_to_fetch_second;
                        end

                        state_n = STATE_FETCH;
                    end

               end

           //end

        end
        endcase
    end


    // ** SubModule 11 : Operand Fetch Response from vgpr **
    wire [SRC_OPD_WIDTH-1:0] v_opd_id;
    wire v_opd_fetch_valid;

    VX_priority_encoder #(
        .N (NUM_SRC_OPDS)
    ) v_opd_id_sel (
        .data_in   (v_opds_needed),
        .index_out (v_opd_id),
        .valid_out (v_opd_fetch_valid),
        `UNUSED_PIN (onehot_out)
    );

    // operands fetch request
    assign vgpr_if.req_valid = v_opd_fetch_valid;
    assign vgpr_if.req_data.opd_id = v_opd_id;

    assign vgpr_if.req_data.sid = simd_pid;
    assign vgpr_if.req_data.wis = staging_if.data.wis;

    assign vgpr_if.req_data.lid = lane_counter;
    assign vgpr_if.req_data.reg_id = NR_V_BITS'(stg_src_regs[v_opd_id]);

    // For the following sections
    reg [NUM_SRC_OPDS-1:0][`SIMD_WIDTH-1:0][`XLEN-1:0] opd_values;

    // operands fetch response
    always @(posedge clk) begin

        // Reset only last dispatch
        if (reset || last_dispatch) begin
            for (integer i = 0; i < NUM_SRC_OPDS; ++i) begin
                opd_values[i] <= '0;
            end
        end else begin
            if (vgpr_rsp_fire) begin
                opd_values[vgpr_if.rsp_data.opd_id] <= vgpr_if.rsp_data.data;
            end
        end
    end


    // ** SubModule 12 : state machine update **
    // ******************
    always @(posedge clk) begin
        if (reset) begin
            state <= STATE_IDLE;

            v_opds_needed <= '0;
            v_opds_busy <= '0;

            lane_counter <= '0;

        end else begin
            state <= state_n;

            v_opds_needed <= v_opds_needed_n;
            v_opds_busy <= v_opds_busy_n;

            lane_counter <= lane_counter_n;
        end
    end

    // wait for dependency check
    wire dep_check_ready = (wait_mask == 0);

    /*****************************************************************/
    // Set Ready to dispatch signal
    wire output_ready_w;
    assign output_ready = output_ready_w && ~dep_check_ready;
    wire output_valid = (state == STATE_DISPATCH) && ~dep_check_ready;

    // ** SubModule : NonZero Iterator (skip threads) **
    // simd iterator
    VX_nz_iterator #(
        .DATAW   (`SIMD_WIDTH),
        .N       (SIMD_COUNT),
        .OUT_REG (1)
    ) simd_iter (
        .clk     (clk),
        .reset   (reset),
        .valid_in(staging_if.valid),
        .data_in (staging_if.data.tmask),
        .next    (next_simd),
        `UNUSED_PIN (valid_out),
        .data_out(simd_out_first),
        .pid     (simd_pid_first),
        .sop     (simd_sop_first),
        .eop     (simd_eop_first)
    );


    // TO FIX : Not sure if this is correct
    // NOTE : Likely can somehow merge both nz iterators
    // ** SubModule : NonZero Iterator after first dispatches **
    VX_nz_iterator #(
        .DATAW   (`SIMD_WIDTH),
        .N       (1),
        .OUT_REG (1)
    ) simd_iter (
        .clk     (clk),
        .reset   (reset),
        .valid_in(wb_collection_ready),
        .data_in (process_simd),
        .next    (next_simd),
        `UNUSED_PIN (valid_out),
        .data_out(simd_out_second),
        .pid     (simd_pid_second),
        .sop     (simd_sop_second),
        .eop     (simd_eop_second)
    );


    wire [`SIMD_WIDTH-1:0] simd_out_first, simd_out_second;
    wire [SIMD_IDX_W-1:0] simd_pid_first, simd_pid_second;
    wire simd_sop_first, simd_eop_first;
    wire simd_sop_second, simd_eop_second;

    always @(*) begin
        if(in_first_dispatch) begin
            simd_out = simd_out_first;
            simd_pid = simd_pid_first;
            simd_sop = simd_sop_first;
            simd_eop = simd_eop_first;
        end else begin
            simd_out = simd_out_second;
            simd_pid = simd_pid_second;
            simd_sop = simd_sop_second;
            simd_eop = simd_eop_second;
        end
    end


    // ** SubModule : Send to Dispatch **


    // Control Output, whether its first or intermediate
    wire [NUM_SRC_OPDS-1:0][`SIMD_WIDTH-1:0][`XLEN-1:0] opd_values_to_dispatch;

    always @(*) begin
        if(in_first_dispatch) begin
            opd_values_to_dispatch = opd_values;
        end else begin

            // TO FIX : Not sure if I need to mask
            opd_values_to_dispatch = {0, opd_values[1], process_data}

        end
    end

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
            '0, // lane counter of destination is always 0
            staging_if.data.wis,
            simd_pid,
            simd_out,
            staging_if.data.PC,
            staging_if.data.ex_type,
            staging_if.data.op_type,
            staging_if.data.op_args,
            staging_if.data.wb,
            stg_rd, // TODO
            opd_values_to_dispatch[0],
            opd_values_to_dispatch[1],
            opd_values_to_dispatch[2],
            simd_sop, // TODO
            last_dispatch
        }),
        .ready_in (output_ready_w),
        .valid_out(operands_if.valid),
        .data_out (operands_if.data),
        .ready_out(operands_if.ready)
    );

    // NOT YET FIX *******************
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
