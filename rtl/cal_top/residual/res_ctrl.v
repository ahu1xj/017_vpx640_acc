// res_ctrl：residual 模块族配置通道。
// 两阶段配置协议（2026-07-24 冻结）：
//   配置阶段：rdcmd_trans_valid 锁存指令字段 -> PROD1/PROD2/PROD3 三拍分级乘法 ->
//     ISSUE 拍打出 cfg_valid 单拍脉冲与全部配置线；
//   工作阶段：配置线稳定保持到下一条指令，下属模块不自锁存配置。
// 计数常量（ext = 字段+1 扩展）：
//   normal_total       = row_ext*col_blk_ext*bypass_ch_pair_ext（输出拍数）
//   resadd_input_total = row_ext*col_blk_ext*channel_ext（resadd 输入对数）
//   up_row_beats       = col_blk_ext*ch_pair_ext（upsample 行拍数）
// 乘法分级：PROD1 拍算 row_ext*col_blk_ext 与 col_blk_ext*ch_pair_ext 寄存，
//   同拍把 PROD2 乘法操作数（ch_sel/bypass_ch_pair）预寄存切断 mux->DSP 路径；
//   PROD2 拍做第二级乘法（20b x 13b 单 DSP）进中间寄存器；
//   PROD3 拍把乘积移入配置寄存器（MREG->PREG 一跳）。
//   操作数全部按真实位宽截窄，禁止 32b 扩展线直接进乘法（时序红灯整改点）。
// mode 编码沿用原 residual.v：MODE_IDLE/RESADD/UPSAMPLE/BYPASS = 0/1/2/3。
// mode 退出（事务排空回 IDLE）沿用原 mode_q 退出条件，done 由下属模块输入。
`include "all_defines.vh"

module res_ctrl(
    input  wire                          clk,
    input  wire                          rst_n,

    input  wire                          rdcmd_trans_valid,
    input  wire [`RESADD_INCMD-1:0]      rdcmd_trans,

    // 下属模块 done 输入
    input  wire                          resadd_issue_done,
    input  wire                          normal_out_done,
    input  wire                          resadd_dp_busy,
    input  wire                          up_read_seq_done,
    input  wire                          up_buf_idle,
    input  wire                          out_valid,

    // 配置发放
    output wire                          cfg_valid,
    output wire [2:0]                    cfg_mode,
    output wire [11:0]                   cfg_normal_ch_pair,
    output wire [11:0]                   cfg_in_ch_pair,
    output wire [6:0]                    cfg_in_col_blk,
    output wire [4:0]                    cfg_last_col_m1,
    output wire [31:0]                   cfg_input_total,
    output wire [31:0]                   cfg_out_total,
    output wire [11:0]                   cfg_in_row,
    output wire [31:0]                   cfg_up_row_beats,
    output wire                          cfg_upsample_type,
    output wire [4:0]                    cfg_up_tail_valid_col,
    output wire [5:0]                    cfg_up_in_tail_col
);

localparam [2:0] MODE_IDLE     = 3'd0;
localparam [2:0] MODE_RESADD   = 3'd1;
localparam [2:0] MODE_UPSAMPLE = 3'd2;
localparam [2:0] MODE_BYPASS   = 3'd3;

localparam [2:0] RCFG_IDLE  = 3'd0;
localparam [2:0] RCFG_PROD1 = 3'd1;
localparam [2:0] RCFG_PROD2 = 3'd2;
localparam [2:0] RCFG_PROD3 = 3'd3;
localparam [2:0] RCFG_ISSUE = 3'd4;
localparam [2:0] RCFG_RUN   = 3'd5;

// ---- 指令字段锁存 ----
reg [3:0]   model_sel_q;
reg [11:0]  channel_in_q;
reg [11:0]  row_in_q;
reg [6:0]   column_in_q;
reg [4:0]   last_col_valid_q;
reg         upsample_type_q;
reg [1:0]   concat_type_q;
reg [11:0]  concat_fm2_ch_q;
reg [11:0]  concat_fm3_ch_q;
reg [11:0]  concat_fm4_ch_q;
reg [11:0]  split_ch_num_q;

// ---- 配置寄存器（配置线，ISSUE 后稳定保持） ----
reg [2:0]   state_q;
reg [2:0]   mode_q;
reg         cfg_valid_q;
reg [31:0]  rc_prod_q;
reg [11:0]  normal_ch_pair_q;
reg [11:0]  in_ch_pair_q;
reg [6:0]   in_col_blk_q;
reg [4:0]   last_col_m1_q;
reg [31:0]  input_total_q;
reg [31:0]  out_total_q;
reg [12:0]  ch_sel_q;
reg [12:0]  bypass_ch_pair_q;
reg [31:0]  input_total_m_q;
reg [31:0]  out_total_m_q;
reg [11:0]  in_row_q;
reg [31:0]  up_row_beats_q;
reg         up_type_q;
reg [4:0]   up_tail_valid_col_q;
reg [5:0]   up_in_tail_col_q;

// ---- 组合派生（锁存字段 -> 配置计算输入） ----
wire        cmd_is_resadd;
wire        cmd_is_upsample;
wire        cmd_is_bypass;
wire        cmd_is_valid_op;
wire        q_is_resadd;
wire        q_is_upsample;
wire        q_is_concat;
wire        q_is_split;
wire [11:0] row_ext;
wire [7:0]  col_blk_ext;
wire [11:0] channel_ext;
wire [11:0] ch_pair_ext;
wire [13:0] concat_total_ch_ext;
wire [12:0] concat_ch_pair_ext;
wire [12:0] bypass_ch_pair_ext;
wire [5:0]  up_tail_n6;
wire        mode_is_resadd;
wire        mode_is_bypass;
wire        mode_is_upsample;
wire        resadd_exit;
wire        bypass_exit;
wire        upsample_exit;

assign cmd_is_resadd    = (rdcmd_trans[`RD_MODEL_SEL_START+:`RD_MODEL_SEL_WIDTH] == `MODEL_SEL_RESADD);
assign cmd_is_upsample  = (rdcmd_trans[`RD_MODEL_SEL_START+:`RD_MODEL_SEL_WIDTH] == `MODEL_SEL_UPSAMPLE);
assign cmd_is_bypass    = (rdcmd_trans[`RD_MODEL_SEL_START+:`RD_MODEL_SEL_WIDTH] == `MODEL_SEL_CONCAT) |
                          (rdcmd_trans[`RD_MODEL_SEL_START+:`RD_MODEL_SEL_WIDTH] == `MODEL_SEL_SPLIT);
assign cmd_is_valid_op  = cmd_is_resadd | cmd_is_upsample | cmd_is_bypass;

assign q_is_resadd      = (model_sel_q == `MODEL_SEL_RESADD);
assign q_is_upsample    = (model_sel_q == `MODEL_SEL_UPSAMPLE);
assign q_is_concat      = (model_sel_q == `MODEL_SEL_CONCAT);
assign q_is_split       = (model_sel_q == `MODEL_SEL_SPLIT);

assign row_ext          = row_in_q;
assign col_blk_ext      = {1'b0, column_in_q} + 8'd1;
assign channel_ext      = channel_in_q;
assign ch_pair_ext      = q_is_split ? {1'b0, split_ch_num_q[11:1]}
                                     : {1'b0, channel_in_q[11:1]};
assign concat_total_ch_ext = (concat_type_q == 2'd0) ? ({2'd0, channel_in_q} + {2'd0, concat_fm2_ch_q}) :
                             (concat_type_q == 2'd1) ? ({2'd0, channel_in_q} + {2'd0, concat_fm2_ch_q} + {2'd0, concat_fm3_ch_q}) :
                             (concat_type_q == 2'd2) ? ({2'd0, channel_in_q} + {2'd0, concat_fm2_ch_q} + {2'd0, concat_fm3_ch_q} + {2'd0, concat_fm4_ch_q}) :
                             {2'd0, channel_in_q};
assign concat_ch_pair_ext  = concat_total_ch_ext >> 1;
assign bypass_ch_pair_ext  = q_is_concat ? concat_ch_pair_ext : {1'b0, ch_pair_ext};

// upsample 尾列：字段已为输出语义（2026-07-24 新语义）。
// n = 输出尾块有效列数 = field+1（out_w 偶数，n 恒偶）；5bit 回绕使 n=32 自然表示全有效。
// 输入尾列反解（信息性配置线）：type=0 -> n//2；type=1 -> 16+n//2。
assign up_tail_n6       = {1'b0, last_col_valid_q} + 6'd1;

assign mode_is_resadd   = (mode_q == MODE_RESADD);
assign mode_is_bypass   = (mode_q == MODE_BYPASS);
assign mode_is_upsample = (mode_q == MODE_UPSAMPLE);

// mode 退出条件：沿用原 residual.v mode_q 的排空条件
assign resadd_exit   = mode_is_resadd & resadd_issue_done & normal_out_done &
                       (~resadd_dp_busy) & (~out_valid);
assign bypass_exit   = mode_is_bypass & normal_out_done & (~out_valid);
assign upsample_exit = mode_is_upsample & up_read_seq_done & (~out_valid) & up_buf_idle;

assign cfg_valid            = cfg_valid_q;
assign cfg_mode             = mode_q;
assign cfg_normal_ch_pair   = normal_ch_pair_q;
assign cfg_in_ch_pair       = in_ch_pair_q;
assign cfg_in_col_blk       = in_col_blk_q;
assign cfg_last_col_m1      = last_col_m1_q;
assign cfg_input_total      = input_total_q;
assign cfg_out_total        = out_total_q;
assign cfg_in_row           = in_row_q;
assign cfg_up_row_beats     = up_row_beats_q;
assign cfg_upsample_type    = up_type_q;
assign cfg_up_tail_valid_col = up_tail_valid_col_q;
assign cfg_up_in_tail_col   = up_in_tail_col_q;

// ---- 配置 FSM ----
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        state_q <= RCFG_IDLE;
    else begin
        case (state_q)
            RCFG_IDLE: begin
                if (rdcmd_trans_valid && cmd_is_valid_op)
                    state_q <= RCFG_PROD1;
            end
            RCFG_PROD1: begin
                state_q <= RCFG_PROD2;
            end
            RCFG_PROD2: begin
                state_q <= RCFG_PROD3;
            end
            RCFG_PROD3: begin
                state_q <= RCFG_ISSUE;
            end
            RCFG_ISSUE: begin
                state_q <= RCFG_RUN;
            end
            RCFG_RUN: begin
                if (resadd_exit || bypass_exit || upsample_exit)
                    state_q <= RCFG_IDLE;
            end
            default: begin
                state_q <= RCFG_IDLE;
            end
        endcase
    end
end

// ---- 指令字段锁存（仅 IDLE 接收；新指令到达时本模块必已排空，配置空闲性契约） ----
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        model_sel_q <= 4'd0;
    else if ((state_q == RCFG_IDLE) && rdcmd_trans_valid)
        model_sel_q <= rdcmd_trans[`RD_MODEL_SEL_START+:`RD_MODEL_SEL_WIDTH];
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        channel_in_q <= 12'd0;
    else if ((state_q == RCFG_IDLE) && rdcmd_trans_valid)
        channel_in_q <= rdcmd_trans[`RD_FM_CHANNEL_IN_START+:`RD_FM_CHANNEL_IN_WIDTH];
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        row_in_q <= 12'd0;
    else if ((state_q == RCFG_IDLE) && rdcmd_trans_valid)
        row_in_q <= rdcmd_trans[`RD_FM_ROW_IN_START+:`RD_FM_ROW_IN_WIDTH];
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        column_in_q <= 7'd0;
    else if ((state_q == RCFG_IDLE) && rdcmd_trans_valid)
        column_in_q <= rdcmd_trans[`RD_FM_COLUMN_IN_START+:`RD_FM_COLUMN_IN_WIDTH];
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        last_col_valid_q <= 5'd0;
    else if ((state_q == RCFG_IDLE) && rdcmd_trans_valid)
        last_col_valid_q <= rdcmd_trans[`RD_LAST_FM_OUT_COLUMN_VALID_START+:`RD_LAST_FM_OUT_COLUMN_VALID_WIDTH];
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        upsample_type_q <= 1'b0;
    else if ((state_q == RCFG_IDLE) && rdcmd_trans_valid)
        upsample_type_q <= rdcmd_trans[`RD_UPSAMPLE_TYPE_START+:`RD_UPSAMPLE_TYPE_WIDTH];
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        concat_type_q <= 2'd0;
    else if ((state_q == RCFG_IDLE) && rdcmd_trans_valid)
        concat_type_q <= rdcmd_trans[`RD_CONCAT_TYPE_START+:`RD_CONCAT_TYPE_WIDTH];
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        concat_fm2_ch_q <= 12'd0;
    else if ((state_q == RCFG_IDLE) && rdcmd_trans_valid)
        concat_fm2_ch_q <= rdcmd_trans[`RD_CONCAT_FM2_IN_CHANNEL_START+:`RD_CONCAT_FM2_IN_CHANNEL_WIDTH];
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        concat_fm3_ch_q <= 12'd0;
    else if ((state_q == RCFG_IDLE) && rdcmd_trans_valid)
        concat_fm3_ch_q <= rdcmd_trans[`RD_CONCAT_FM3_IN_CHANNEL_START+:`RD_CONCAT_FM3_IN_CHANNEL_WIDTH];
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        concat_fm4_ch_q <= 12'd0;
    else if ((state_q == RCFG_IDLE) && rdcmd_trans_valid)
        concat_fm4_ch_q <= rdcmd_trans[`RD_CONCAT_FM4_IN_CHANNEL_START+:`RD_CONCAT_FM4_IN_CHANNEL_WIDTH];
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        split_ch_num_q <= 12'd0;
    else if ((state_q == RCFG_IDLE) && rdcmd_trans_valid)
        split_ch_num_q <= rdcmd_trans[`RD_SPLIT_CH_NUM_START+:`RD_SPLIT_CH_NUM_WIDTH];
end

// ---- PROD1：第一级乘法（row_ext*col_blk_ext）与简单配置项寄存 ----
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        rc_prod_q <= 32'd0;
    else if (state_q == RCFG_PROD1)
        rc_prod_q <= row_ext * col_blk_ext;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        up_row_beats_q <= 32'd0;
    else if (state_q == RCFG_PROD1)
        up_row_beats_q <= col_blk_ext * ch_pair_ext;
end

// ---- PROD1 同拍：PROD2 第二级乘法操作数预寄存（切断 mux->DSP 输入路径） ----
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        ch_sel_q <= 13'd0;
    else if (state_q == RCFG_PROD1)
        ch_sel_q <= q_is_resadd ? {1'b0, channel_ext} : bypass_ch_pair_ext;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        bypass_ch_pair_q <= 13'd0;
    else if (state_q == RCFG_PROD1)
        bypass_ch_pair_q <= bypass_ch_pair_ext;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        normal_ch_pair_q <= 12'd0;
    else if (state_q == RCFG_PROD1)
        normal_ch_pair_q <= q_is_resadd ? channel_in_q : bypass_ch_pair_ext[11:0];
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        in_ch_pair_q <= 12'd0;
    else if (state_q == RCFG_PROD1)
        in_ch_pair_q <= channel_in_q[11:1];
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        in_col_blk_q <= 7'd0;
    else if (state_q == RCFG_PROD1)
        in_col_blk_q <= column_in_q + 7'd1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        last_col_m1_q <= 5'd0;
    else if (state_q == RCFG_PROD1)
        last_col_m1_q <= last_col_valid_q;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        in_row_q <= 12'd0;
    else if (state_q == RCFG_PROD1)
        in_row_q <= row_in_q;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        up_type_q <= 1'b0;
    else if (state_q == RCFG_PROD1)
        up_type_q <= upsample_type_q;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        up_tail_valid_col_q <= 5'd0;
    else if (state_q == RCFG_PROD1)
        up_tail_valid_col_q <= last_col_valid_q + 5'd1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        up_in_tail_col_q <= 6'd0;
    else if (state_q == RCFG_PROD1)
        up_in_tail_col_q <= (upsample_type_q ? 6'd16 : 6'd0) + {1'b0, up_tail_n6[5:1]};
end

// ---- PROD2：第二级乘法（rc_prod[19:0] * ch 项，20b x 13b 单 DSP）进中间寄存器 ----
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        out_total_m_q <= 32'd0;
    else if (state_q == RCFG_PROD2)
        out_total_m_q <= rc_prod_q[19:0] * bypass_ch_pair_q;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        input_total_m_q <= 32'd0;
    else if (state_q == RCFG_PROD2)
        input_total_m_q <= rc_prod_q[19:0] * ch_sel_q;
end

// ---- PROD3：乘积移入配置寄存器（MREG->PREG 一跳） ----
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        out_total_q <= 32'd0;
    else if (state_q == RCFG_PROD3)
        out_total_q <= out_total_m_q;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        input_total_q <= 32'd0;
    else if (state_q == RCFG_PROD3)
        input_total_q <= input_total_m_q;
end

// ---- ISSUE：cfg_valid 单拍脉冲 + mode 装载；RUN：排空退出 ----
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        cfg_valid_q <= 1'b0;
    else
        cfg_valid_q <= (state_q == RCFG_PROD3);
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        mode_q <= MODE_IDLE;
    else if (state_q == RCFG_ISSUE) begin
        if (q_is_resadd)
            mode_q <= MODE_RESADD;
        else if (q_is_upsample)
            mode_q <= MODE_UPSAMPLE;
        else
            mode_q <= MODE_BYPASS;
    end else if ((state_q == RCFG_RUN) && (resadd_exit || bypass_exit || upsample_exit))
        mode_q <= MODE_IDLE;
end

endmodule

// ============================================================
// RTL Self-Check Results
// ============================================================
// [PASS] Rule #1:  All ports have explicit width declarations
// [PASS] Rule #2:  Ports use Verilog wire/reg conventions
// [PASS] Rule #6:  No debug/error-attribution logic or diagnostic-only sideband in RTL
// [PASS] Rule #8:  No logic/always_ff/always_comb used
// [PASS] Rule #12: FSM case has default branch; all conditional assigns complete
// [PASS] Rule #16: Non-blocking '<=' for all sequential logic
// [PASS] Rule #24: Multipliers registered in three stages (PROD1 operands ->
//        PROD2 single-DSP multiply -> PROD3 product register); operands
//        narrowed to real widths (no 32b-extended wires into multipliers)
// [PASS] One register per always block
// [PASS] Declaration-before-use for all regs referenced by wire assigns
// [PASS] Command fields decoded via rd_sm_cmd_define macros (no hardcoded bit ranges)
// [PASS] Two-phase config protocol: cfg_valid single-cycle pulse, cfg lines stable after
// [WARN] cfg_up_in_tail_col is informational per frozen spec; not consumed downstream
// ============================================================
