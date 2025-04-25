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

`define USED_IREG(x) \
    x``_v.id = ``x; \
    x``_v.rtype = 0; \
    x``_v.ext = 0; \
    use_``x = 1

`define USED_FREG(x) \
    x``_v.id = ``x; \
    x``_v.rtype = 1; \
    x``_v.ext = 0; \
    use_``x = 1

`define USED_VREG(x) \
    x``_v.id = ``x; \
    x``_v.rtype = 2; \
    x``_v.ext = 0; \
    use_``x = 1

module VX_decode import VX_gpu_pkg::*; #(
    parameter `STRING INSTANCE_ID = ""
) (
    input wire              clk,
    input wire              reset,

    // inputs
    VX_fetch_if.slave       fetch_if,
`ifdef EXT_V_ENABLE
    VX_vpu_states_if.slave  vpu_states_if,
`endif
    // outputs
    VX_decode_if.master     decode_if,
    VX_decode_sched_if.master decode_sched_if
);

    `UNUSED_SPARAM (INSTANCE_ID)
    `UNUSED_VAR (clk)
    `UNUSED_VAR (reset)

    localparam NUM_OPDS = NUM_SRC_OPDS + 1;
    localparam DATAW = UUID_WIDTH + NW_WIDTH + `NUM_THREADS + PC_BITS + EX_BITS + INST_OP_BITS + INST_ARGS_BITS + NUM_OPDS + (REG_IDX_BITS * NUM_OPDS);

    reg [EX_BITS-1:0] ex_type;
    reg [INST_OP_BITS-1:0] op_type;
    op_args_t op_args;
    reg_idx_t rd_v, rs1_v, rs2_v, rs3_v;
    reg use_rd, use_rs1, use_rs2, use_rs3;
    reg is_wstall;

    wire [31:0] instr = fetch_if.data.instr;
    wire [6:0] opcode = instr[6:0];
    wire [1:0] funct2 = instr[26:25];
    wire [2:0] funct3 = instr[14:12];
    wire [4:0] funct5 = instr[31:27];
    wire [6:0] funct7 = instr[31:25];
    wire [11:0] u_12  = instr[31:20];

    wire [4:0] rd  = instr[11:7];
    wire [4:0] rs1 = instr[19:15];
    wire [4:0] rs2 = instr[24:20];
    wire [4:0] rs3 = instr[31:27];

    `UNUSED_VAR (funct2)
    `UNUSED_VAR (funct5)
    `UNUSED_VAR (rs3)
    `UNUSED_VAR (use_rd)
    `UNUSED_VAR (use_rs1)
    `UNUSED_VAR (use_rs2)
    `UNUSED_VAR (use_rs3)

    wire is_itype_sh = funct3[0] && ~funct3[1];
    wire is_fpu_csr = (u_12 <= `VX_CSR_FCSR);

    wire [19:0] ui_imm  = instr[31:12];
`ifdef XLEN_64
    wire [11:0] i_imm   = is_itype_sh ? {6'b0, instr[25:20]} : u_12;
    wire [11:0] iw_imm  = is_itype_sh ? {7'b0, instr[24:20]} : u_12;
`else
    wire [11:0] i_imm   = is_itype_sh ? {7'b0, instr[24:20]} : u_12;
`endif
    wire [11:0] s_imm   = {funct7, rd};
    wire [12:0] b_imm   = {instr[31], instr[7], instr[30:25], instr[11:8], 1'b0};
    wire [20:0] jal_imm = {instr[31], instr[19:12], instr[20], instr[30:21], 1'b0};

    reg [INST_ALU_BITS-1:0] r_type;
    always @(*) begin
        case (funct3)
            3'h0: r_type = (opcode[5] && funct7[5]) ? INST_ALU_SUB : INST_ALU_ADD;
            3'h1: r_type = INST_ALU_SLL;
            3'h2: r_type = INST_ALU_SLT;
            3'h3: r_type = INST_ALU_SLTU;
            3'h4: r_type = INST_ALU_XOR;
            3'h5: r_type = funct7[5] ? INST_ALU_SRA : INST_ALU_SRL;
            3'h6: r_type = INST_ALU_OR;
            3'h7: r_type = INST_ALU_AND;
        endcase
    end

    reg [INST_BR_BITS-1:0] b_type;
    always @(*) begin
        case (funct3)
            3'h0: b_type = INST_BR_EQ;
            3'h1: b_type = INST_BR_NE;
            3'h4: b_type = INST_BR_LT;
            3'h5: b_type = INST_BR_GE;
            3'h6: b_type = INST_BR_LTU;
            3'h7: b_type = INST_BR_GEU;
            default: b_type = 'x;
        endcase
    end

    reg [INST_BR_BITS-1:0] s_type;
    always @(*) begin
        case (u_12)
            12'h000: s_type = INST_OP_BITS'(INST_BR_ECALL);
            12'h001: s_type = INST_OP_BITS'(INST_BR_EBREAK);
            12'h002: s_type = INST_OP_BITS'(INST_BR_URET);
            12'h102: s_type = INST_OP_BITS'(INST_BR_SRET);
            12'h302: s_type = INST_OP_BITS'(INST_BR_MRET);
            default: s_type = 'x;
        endcase
    end

`ifdef EXT_M_ENABLE
    reg [INST_M_BITS-1:0] m_type;
    always @(*) begin
        case (funct3)
            3'h0: m_type = INST_M_MUL;
            3'h1: m_type = INST_M_MULH;
            3'h2: m_type = INST_M_MULHSU;
            3'h3: m_type = INST_M_MULHU;
            3'h4: m_type = INST_M_DIV;
            3'h5: m_type = INST_M_DIVU;
            3'h6: m_type = INST_M_REM;
            3'h7: m_type = INST_M_REMU;
        endcase
    end
`endif

`ifdef EXT_V_ENABLE

    wire [2:0]    nf = instr[31:29];
    wire         mew = instr[28];
    wire [1:0]   mop = instr[27:26];
    wire          vm = instr[25];
    wire [5:0] funct6= instr[31:26];
    wire [1:0]  vset = instr[31:30];
    wire [7:0]  zimm = instr[27:20];

    vpu_states_t [`NUM_WARPS-1:0] vpu_states;

    always @(posedge clk) begin
        if (reset) begin
            vpu_states <= '0;
        end else begin
            if (vpu_states_if.valid) begin
                vpu_states[vpu_states_if.wid] <= vpu_states_if.data;
            end
        end
    end

    `UNUSED_VAR (vpu_states)

    reg [INST_VPU_OP_BITS-1:0] vop_type;
    always @(*) begin
        case (funct6)
            /*6'd0:  vop_type = INST_OP_BITS'(INST_VPU_VADD);
            6'd2:  vop_type = INST_OP_BITS'(INST_VPU_VSUB);
            6'd4:  vop_type = INST_OP_BITS'(INST_VPU_VMINU);
            6'd5:  vop_type = INST_OP_BITS'(INST_VPU_VMIN);
            6'd6:  vop_type = INST_OP_BITS'(INST_VPU_VMAXU);
            6'd7:  vop_type = INST_OP_BITS'(INST_VPU_VMAX);
            6'd9:  vop_type = INST_OP_BITS'(INST_ALU_AND);
            6'd10: vop_type = INST_OP_BITS'(INST_ALU_OR);
            6'd11: vop_type = INST_OP_BITS'(INST_ALU_XOR);
            6'd24: vop_type = INST_OP_BITS'(INST_VPU_VMSEQ);
            6'd25: vop_type = INST_OP_BITS'(INST_VPU_VMSNE);
            6'd26: vop_type = INST_OP_BITS'(INST_ALU_SLTU);
            6'd27: vop_type = INST_OP_BITS'(INST_ALU_SLT);
            6'd28: vop_type = INST_OP_BITS'(INST_VPU_VMSLEU);
            6'd29: vop_type = INST_OP_BITS'(INST_VPU_VMSLE);
            6'd30: vop_type = INST_OP_BITS'(INST_VPU_VMSGTU);
            6'd31: vop_type = INST_OP_BITS'(INST_VPU_VMSGT);*/
            default: vop_type = 'x;
        endcase
    end

    `STATIC_ASSERT($bits(vpu_args_t) == $bits(op_args_t), ("vpu_args_t size mismatch: current=%0d, expected=%0d", $bits(vpu_args_t), $bits(op_args_t)));
`endif

    `STATIC_ASSERT($bits(alu_args_t)  == $bits(op_args_t), ("alu_args_t size mismatch: current=%0d, expected=%0d", $bits(alu_args_t), $bits(op_args_t)));
    `STATIC_ASSERT($bits(fpu_args_t)  == $bits(op_args_t), ("fpu_args_t size mismatch: current=%0d, expected=%0d", $bits(fpu_args_t), $bits(op_args_t)));
    `STATIC_ASSERT($bits(lsu_args_t)  == $bits(op_args_t), ("lsu_args_t size mismatch: current=%0d, expected=%0d", $bits(lsu_args_t), $bits(op_args_t)));
    `STATIC_ASSERT($bits(csr_args_t)  == $bits(op_args_t), ("csr_args_t size mismatch: current=%0d, expected=%0d", $bits(csr_args_t), $bits(op_args_t)));
    `STATIC_ASSERT($bits(wctl_args_t) == $bits(op_args_t), ("wctl_args_t size mismatch: current=%0d, expected=%0d", $bits(wctl_args_t), $bits(op_args_t)));

    always @(*) begin

        ex_type   = 'x;
        op_type   = 'x;
        op_args   = 'x;
        rd_v      = 'x;
        rs1_v     = 'x;
        rs2_v     = 'x;
        rs3_v     = 'x;
        use_rd    = 0;
        use_rs1   = 0;
        use_rs2   = 0;
        use_rs3   = 0;
        is_wstall = 0;

        case (opcode)
            INST_I: begin
                ex_type = EX_ALU;
                op_type = INST_OP_BITS'(r_type);
                op_args.alu.xtype = ALU_TYPE_ARITH;
                op_args.alu.is_w = 0;
                op_args.alu.use_PC = 0;
                op_args.alu.use_imm = 1;
                op_args.alu.imm = `SEXT(`XLEN, i_imm);
                `USED_IREG (rd);
                `USED_IREG (rs1);
            end
            INST_R: begin
                ex_type = EX_ALU;
                op_args.alu.is_w = 0;
                op_args.alu.use_PC = 0;
                op_args.alu.use_imm = 0;
                `USED_IREG (rd);
                `USED_IREG (rs1);
                `USED_IREG (rs2);
                case (funct7)
                `ifdef EXT_M_ENABLE
                    INST_R_F7_MUL: begin
                        // MUL, MULH, MULHSU, MULHU
                        op_type = INST_OP_BITS'(m_type);
                        op_args.alu.xtype = ALU_TYPE_MULDIV;
                    end
                `endif
                `ifdef EXT_ZICOND_ENABLE
                    INST_R_F7_ZICOND: begin
                        // CZERO-EQZ, CZERO-NEZ
                        op_type = funct3[1] ? INST_OP_BITS'(INST_ALU_CZNE) : INST_OP_BITS'(INST_ALU_CZEQ);
                        op_args.alu.xtype = ALU_TYPE_ARITH;
                    end
                `endif
                    default: begin
                        op_type = INST_OP_BITS'(r_type);
                        op_args.alu.xtype = ALU_TYPE_ARITH;
                    end
                endcase
            end
        `ifdef XLEN_64
            INST_I_W: begin
                // ADDIW, SLLIW, SRLIW, SRAIW
                ex_type = EX_ALU;
                op_type = INST_OP_BITS'(r_type);
                op_args.alu.xtype = ALU_TYPE_ARITH;
                op_args.alu.is_w = 1;
                op_args.alu.use_PC = 0;
                op_args.alu.use_imm = 1;
                op_args.alu.imm = `SEXT(`XLEN, iw_imm);
                `USED_IREG (rd);
                `USED_IREG (rs1);
            end
            INST_R_W: begin
                ex_type = EX_ALU;
                op_args.alu.is_w = 1;
                op_args.alu.use_PC = 0;
                op_args.alu.use_imm = 0;
                `USED_IREG (rd);
                `USED_IREG (rs1);
                `USED_IREG (rs2);
                case (funct7)
                `ifdef EXT_M_ENABLE
                    INST_R_F7_MUL: begin
                        // MULW, DIVW, DIVUW, REMW, REMUW
                        op_type = INST_OP_BITS'(m_type);
                        op_args.alu.xtype = ALU_TYPE_MULDIV;
                    end
                `endif
                    default: begin
                        // ADDW, SUBW, SLLW, SRLW, SRAW
                        op_type = INST_OP_BITS'(r_type);
                        op_args.alu.xtype = ALU_TYPE_ARITH;
                    end
                endcase
            end
        `endif
            INST_LUI: begin
                ex_type = EX_ALU;
                op_type = INST_OP_BITS'(INST_ALU_LUI);
                op_args.alu.xtype = ALU_TYPE_ARITH;
                op_args.alu.is_w = 0;
                op_args.alu.use_PC = 0;
                op_args.alu.use_imm = 1;
                op_args.alu.imm = {{`XLEN-31{ui_imm[19]}}, ui_imm[18:0], 12'(0)};
                `USED_IREG (rd);
            end
            INST_AUIPC: begin
                ex_type = EX_ALU;
                op_type = INST_OP_BITS'(INST_ALU_AUIPC);
                op_args.alu.xtype = ALU_TYPE_ARITH;
                op_args.alu.is_w = 0;
                op_args.alu.use_PC = 1;
                op_args.alu.use_imm = 1;
                op_args.alu.imm = {{`XLEN-31{ui_imm[19]}}, ui_imm[18:0], 12'(0)};
                `USED_IREG (rd);
            end
            INST_JAL: begin
                ex_type = EX_ALU;
                op_type = INST_OP_BITS'(INST_BR_JAL);
                op_args.alu.xtype = ALU_TYPE_BRANCH;
                op_args.alu.is_w = 0;
                op_args.alu.use_PC = 1;
                op_args.alu.use_imm = 1;
                op_args.alu.imm = `SEXT(`XLEN, jal_imm);
                is_wstall = 1;
                `USED_IREG (rd);
            end
            INST_JALR: begin
                ex_type = EX_ALU;
                op_type = INST_OP_BITS'(INST_BR_JALR);
                op_args.alu.xtype = ALU_TYPE_BRANCH;
                op_args.alu.is_w = 0;
                op_args.alu.use_PC = 0;
                op_args.alu.use_imm = 1;
                op_args.alu.imm = `SEXT(`XLEN, u_12);
                is_wstall = 1;
                `USED_IREG (rd);
                `USED_IREG (rs1);
            end
            INST_B: begin
                ex_type = EX_ALU;
                op_type = INST_OP_BITS'(b_type);
                op_args.alu.xtype = ALU_TYPE_BRANCH;
                op_args.alu.is_w = 0;
                op_args.alu.use_PC = 1;
                op_args.alu.use_imm = 1;
                op_args.alu.imm = `SEXT(`XLEN, b_imm);
                is_wstall = 1;
                `USED_IREG (rs1);
                `USED_IREG (rs2);
            end
            INST_FENCE: begin
                ex_type = EX_LSU;
                op_type = INST_LSU_FENCE;
                op_args.lsu.is_store = 0;
                op_args.lsu.is_float = 0;
                op_args.lsu.offset = 0;
            end
            INST_SYS : begin
                if (funct3[1:0] != 0) begin
                    ex_type = EX_SFU;
                    op_type = INST_OP_BITS'(inst_sfu_csr(funct3));
                    op_args.csr.addr = u_12;
                    op_args.csr.use_imm = funct3[2];
                    is_wstall = is_fpu_csr; // only stall for FPU CSRs
                    `USED_IREG (rd);
                    if (funct3[2]) begin
                        op_args.csr.imm = rs1;
                    end else begin
                        `USED_IREG (rs1);
                    end
                end else begin
                    ex_type = EX_ALU;
                    op_type = INST_OP_BITS'(s_type);
                    op_args.alu.xtype = ALU_TYPE_BRANCH;
                    op_args.alu.is_w = 0;
                    op_args.alu.use_imm = 1;
                    op_args.alu.use_PC  = 1;
                    op_args.alu.imm = `XLEN'd4;
                    is_wstall = 1;
                    `USED_IREG (rd);
                end
            end
        `ifdef EXT_V_ENABLE
            INST_FL: begin
                if (funct3[2] == 1 || funct3 == 0) begin
                    ex_type = EX_LSU;
                    op_type = INST_OP_BITS'({1'b0, funct3});
                    op_args.lsu.is_store = 0;
                    op_args.lsu.is_float = opcode[2];
                    op_args.lsu.offset = u_12;
                    `USED_IREG (rs1);
                    `USED_FREG (rd);
                end else begin
                    ex_type = EX_VPU;
                    op_type = INST_OP_BITS'(INST_VPU_VLD);
                    op_args.vpu.vld.nf = nf;
                    op_args.vpu.vld.mop = mop;
                    op_args.vpu.vld.mew = mew;
                    op_args.vpu.vld.width = funct3;
                    `USED_IREG (rs1);
                    `USED_VREG (rd);
                    case (mop)
                        2'b00: begin
                            op_args.vpu.vld.lumop = rs2;
                        end
                        2'b10: begin
                            `USED_IREG (rs2);
                        end
                        default: begin
                            `USED_VREG (rs2);
                        end
                    endcase
                end
            end
        `elsif EXT_F_ENABLE
            INST_FL,
        `endif
            INST_L: begin
                ex_type = EX_LSU;
                op_type = INST_OP_BITS'({1'b0, funct3});
                op_args.lsu.is_store = 0;
                op_args.lsu.is_float = opcode[2];
                op_args.lsu.offset = u_12;
                `USED_IREG (rs1);
                `USED_IREG (rd);
            `ifdef EXT_F_ENABLE
                rd_v.rtype = REG_TYPE_BITS'(opcode[2]);
            `endif
            end
        `ifdef EXT_V_ENABLE
            INST_FS: begin
                if (funct3[2] == 1 || funct3 == 0) begin
                    ex_type = EX_LSU;
                    op_type = INST_OP_BITS'({1'b1, funct3});
                    op_args.lsu.is_store = 1;
                    op_args.lsu.is_float = opcode[2];
                    op_args.lsu.offset = s_imm;
                    `USED_IREG (rs1);
                    `USED_FREG (rs2);
                end else begin
                    ex_type = EX_VPU;
                    op_type = INST_OP_BITS'(INST_VPU_VST);
                    op_args.vpu.vst.nf = nf;
                    op_args.vpu.vst.mop = mop;
                    op_args.vpu.vst.mew = mew;
                    op_args.vpu.vst.width = funct3;
                    `USED_IREG (rs1);
                    `USED_VREG (rs3);
                    case (mop)
                        2'b00: begin
                            op_args.vpu.vst.sumop = rs2;
                        end
                        2'b10: begin
                            `USED_IREG (rs2);
                        end
                        default: begin
                            `USED_VREG (rs2);
                        end
                    endcase
                end
            end
        `elsif EXT_F_ENABLE
            INST_FS,
        `endif
            INST_S: begin
                ex_type = EX_LSU;
                op_type = INST_OP_BITS'({1'b1, funct3});
                op_args.lsu.is_store = 1;
                op_args.lsu.is_float = opcode[2];
                op_args.lsu.offset = s_imm;
                `USED_IREG (rs1);
                `USED_IREG (rs2);
            `ifdef EXT_F_ENABLE
                rs2_v.rtype = REG_TYPE_BITS'(opcode[2]);
            `endif
            end
        `ifdef EXT_F_ENABLE
            INST_FMADD,  // 7'b1000011
            INST_FMSUB,  // 7'b1000111
            INST_FNMSUB, // 7'b1001011
            INST_FNMADD: // 7'b1001111
            begin
                ex_type = EX_FPU;
                op_type = INST_OP_BITS'({2'b00, 1'b1, opcode[3]});
                op_args.fpu.frm = funct3;
                op_args.fpu.fmt[0] = funct2[0]; // float / double
                op_args.fpu.fmt[1] = opcode[3] ^ opcode[2]; // SUB
                `USED_FREG (rd);
                `USED_FREG (rs1);
                `USED_FREG (rs2);
                `USED_FREG (rs3);
            end
            INST_FCI: begin
                ex_type = EX_FPU;
                op_args.fpu.frm = funct3;
                op_args.fpu.fmt[0] = funct2[0]; // float / double
                op_args.fpu.fmt[1] = rs2[1];   // int32 / int64
                case (funct5)
                    5'b00000, // FADD
                    5'b00001, // FSUB
                    5'b00010: // FMUL
                    begin
                        op_type = INST_OP_BITS'({2'b00, 1'b0, funct5[1]});
                        op_args.fpu.fmt[1] = funct5[0]; // SUB
                        `USED_FREG (rd);
                        `USED_FREG (rs1);
                        `USED_FREG (rs2);
                    end
                    5'b00100: begin
                        // NCP: FSGNJ=0, FSGNJN=1, FSGNJX=2
                        op_type = INST_OP_BITS'(INST_FPU_MISC);
                        op_args.fpu.frm = INST_FRM_BITS'(funct3[1:0]);
                        `USED_FREG (rd);
                        `USED_FREG (rs1);
                        `USED_FREG (rs2);
                    end
                    5'b00101: begin
                        // NCP: FMIN=6, FMAX=7
                        op_type = INST_OP_BITS'(INST_FPU_MISC);
                        op_args.fpu.frm = INST_FRM_BITS'(funct3[0] ? 7 : 6);
                        `USED_FREG (rd);
                        `USED_FREG (rs1);
                        `USED_FREG (rs2);
                    end
                `ifdef FLEN_64
                    5'b01000: begin
                        // FCVT.S.D, FCVT.D.S
                        op_type = INST_OP_BITS'(INST_FPU_F2F);
                        `USED_FREG (rd);
                        `USED_FREG (rs1);
                    end
                `endif
                    5'b00011: begin
                        // FDIV
                        op_type = INST_OP_BITS'(INST_FPU_DIV);
                        `USED_FREG (rd);
                        `USED_FREG (rs1);
                        `USED_FREG (rs2);
                    end
                    5'b01011: begin
                        // FSQRT
                        op_type = INST_OP_BITS'(INST_FPU_SQRT);
                        `USED_FREG (rd);
                        `USED_FREG (rs1);
                    end
                    5'b10100: begin
                        // FCMP
                        op_type = INST_OP_BITS'(INST_FPU_CMP);
                        `USED_IREG (rd);
                        `USED_FREG (rs1);
                        `USED_FREG (rs2);
                    end
                    5'b11000: begin
                        // FCVT.W.X, FCVT.WU.X
                        op_type = (rs2[0]) ? INST_OP_BITS'(INST_FPU_F2U) : INST_OP_BITS'(INST_FPU_F2I);
                        `USED_IREG (rd);
                        `USED_FREG (rs1);
                    end
                    5'b11010: begin
                        // FCVT.X.W, FCVT.X.WU
                        op_type = (rs2[0]) ? INST_OP_BITS'(INST_FPU_U2F) : INST_OP_BITS'(INST_FPU_I2F);
                        `USED_FREG (rd);
                        `USED_IREG (rs1);
                    end
                    5'b11100: begin
                        if (funct3[0]) begin
                            // NCP: FCLASS=3
                            op_type = INST_OP_BITS'(INST_FPU_MISC);
                            op_args.fpu.frm = INST_FRM_BITS'(3);
                        end else begin
                            // NCP: FMV.X.W=4
                            op_type = INST_OP_BITS'(INST_FPU_MISC);
                            op_args.fpu.frm = INST_FRM_BITS'(4);
                        end
                        `USED_IREG (rd);
                        `USED_FREG (rs1);
                    end
                    5'b11110: begin
                        // NCP: FMV.W.X=5
                        op_type = INST_OP_BITS'(INST_FPU_MISC);
                        op_args.fpu.frm = INST_FRM_BITS'(5);
                        `USED_FREG (rd);
                        `USED_IREG (rs1);
                    end
                default:;
                endcase
            end
        `endif
            INST_EXT1: begin
                case (funct7)
                    7'h00: begin
                        ex_type = EX_SFU;
                        is_wstall = 1;
                        case (funct3)
                            3'h0: begin // TMC
                                op_type = INST_OP_BITS'(INST_SFU_TMC);
                                `USED_IREG (rs1);
                            end
                            3'h1: begin // WSPAWN
                                op_type = INST_OP_BITS'(INST_SFU_WSPAWN);
                                `USED_IREG (rs1);
                                `USED_IREG (rs2);
                            end
                            3'h2: begin // SPLIT
                                op_type = INST_OP_BITS'(INST_SFU_SPLIT);
                                op_args.wctl.is_neg = rs2[0];
                                `USED_IREG (rs1);
                                `USED_IREG (rd);
                            end
                            3'h3: begin // JOIN
                                op_type = INST_OP_BITS'(INST_SFU_JOIN);
                                `USED_IREG (rs1);
                            end
                            3'h4: begin // BAR
                                op_type = INST_OP_BITS'(INST_SFU_BAR);
                                `USED_IREG (rs1);
                                `USED_IREG (rs2);
                            end
                            3'h5: begin // PRED
                                op_type = INST_OP_BITS'(INST_SFU_PRED);
                                op_args.wctl.is_neg = rd[0];
                                `USED_IREG (rs1);
                                `USED_IREG (rs2);
                            end
                            default:;
                        endcase
                    end
                    default:;
                endcase
            end
        `ifdef EXT_V_ENABLE
            INST_V: begin
                ex_type = EX_VPU;
                case (funct3)
                    3'd0, 3'd1, 3'd2: begin // OPIVV, OPFVV, OPMVV
                        op_type = INST_OP_BITS'(INST_VPU_VOP);
                        op_args.vpu.vop.op = vop_type;
                        op_args.vpu.vop.vm = vm;
                        op_args.vpu.vop.use_imm = 0;
                        `USED_VREG (rs1);
                        `USED_VREG (rs2);
                        `USED_VREG (rd);
                    end
                    3'd3: begin // OPIVI
                        op_type = INST_OP_BITS'(INST_VPU_VOP);
                        op_args.vpu.vop.op = vop_type;
                        op_args.vpu.vop.vm = vm;
                        op_args.vpu.vop.use_imm = 1;
                        op_args.vpu.vop.imm = rs1;
                        `USED_VREG (rs2);
                        `USED_VREG (rd);
                    end
                    3'd4, 3'd6: begin // OPIVX, OPIVV
                        op_type = INST_OP_BITS'(INST_VPU_VOP);
                        op_args.vpu.vop.op = vop_type;
                        op_args.vpu.vop.vm = vm;
                        op_args.vpu.vop.use_imm = 0;
                        `USED_IREG (rs1);
                        `USED_VREG (rs2);
                        `USED_VREG (rd);
                    end
                    3'd5: begin // OPFVF
                        op_type = INST_OP_BITS'(INST_VPU_VOP);
                        op_args.vpu.vop.op = vop_type;
                        op_args.vpu.vop.vm = vm;
                        op_args.vpu.vop.use_imm = 0;
                        `USED_FREG (rs1);
                        `USED_VREG (rs2);
                        `USED_VREG (rd);
                    end
                    3'd7: begin // VSETX
                        op_type = INST_OP_BITS'(INST_VPU_VSET);
                        op_args.vpu.vset.vset = vset;
                        op_args.vpu.vset.use_imm = 0;
                        op_args.vpu.vset.use_zimm = 0;
                        `USED_IREG (rd);
                        if(vset == 2'b10) begin
                            `USED_IREG (rs2);
                            `USED_IREG (rs1);
                        end else begin
                            op_args.vpu.vset.use_zimm = 1;
                            op_args.vpu.vset.zimm = zimm;
                            if (vset[0]) begin
                                op_args.vpu.vset.use_imm = 1;
                                op_args.vpu.vset.imm = rs1;
                            end else begin
                                `USED_IREG (rs1);
                            end
                        end
                    end
                endcase
            end
        `endif
            default:;
        endcase
    end

    // disable write to integer register r0
    wire wb = use_rd && (rd_v != 0);

    wire [2:0] used_rs = {use_rs3, use_rs2, use_rs1};

    VX_elastic_buffer #(
        .DATAW (DATAW),
        .SIZE  (0)
    ) req_buf (
        .clk       (clk),
        .reset     (reset),
        .valid_in  (fetch_if.valid),
        .ready_in  (fetch_if.ready),
        .data_in   ({fetch_if.data.uuid,  fetch_if.data.wid,  fetch_if.data.tmask,  fetch_if.data.PC,  ex_type,                op_type,                op_args,                wb,                used_rs,                rd_v,              rs1_v,              rs2_v,              rs3_v}),
        .data_out  ({decode_if.data.uuid, decode_if.data.wid, decode_if.data.tmask, decode_if.data.PC, decode_if.data.ex_type, decode_if.data.op_type, decode_if.data.op_args, decode_if.data.wb, decode_if.data.used_rs, decode_if.data.rd, decode_if.data.rs1, decode_if.data.rs2, decode_if.data.rs3}),
        .valid_out (decode_if.valid),
        .ready_out (decode_if.ready)
    );

    ///////////////////////////////////////////////////////////////////////////

    wire fetch_fire = fetch_if.valid && fetch_if.ready;

    assign decode_sched_if.valid  = fetch_fire;
    assign decode_sched_if.wid    = fetch_if.data.wid;
    assign decode_sched_if.unlock = ~is_wstall;

`ifndef L1_ENABLE
    assign fetch_if.ibuf_pop = decode_if.ibuf_pop;
`endif

`ifdef DBG_TRACE_PIPELINE
    always @(posedge clk) begin
        if (decode_if.valid && decode_if.ready) begin
            `TRACE(1, ("%t: %s: wid=%0d, PC=0x%0h, instr=0x%0h, ex=", $time, INSTANCE_ID, decode_if.data.wid, {decode_if.data.PC, 1'd0}, instr))
            trace_ex_type(1, decode_if.data.ex_type);
            `TRACE(1, (", op="))
            trace_ex_op(1, decode_if.data.ex_type, decode_if.data.op_type, decode_if.data.op_args);
            `TRACE(1, (", tmask=%b, wb=%b, used_rs=%b, rd=", decode_if.data.tmask, decode_if.data.wb, decode_if.data.used_rs))
            trace_reg_idx(1, decode_if.data.rd);
            `TRACE(1, (", rs1="))
            trace_reg_idx(1, decode_if.data.rs1);
            `TRACE(1, (", rs2="))
            trace_reg_idx(1, decode_if.data.rs2);
            `TRACE(1, (", rs3="))
            trace_reg_idx(1, decode_if.data.rs3);
            `TRACE(1, (", "))
            trace_op_args(1, decode_if.data.ex_type, decode_if.data.op_type, decode_if.data.op_args);
            `TRACE(1, (" (#%0d)\n", decode_if.data.uuid))
        end
    end
`endif

endmodule
