`include "all_defines.vh"

module prefetch_fm_cmd_ctrl(
    input           clk,
    input           rst_n,

    input           rdcmd_trans_valid,
    input   [111:0] rdcmd_trans,

    input           conv_done,
    input           pool_done,

    output          conv_start,
    output          pool_start,

    output          active_is_conv,
    output          active_is_pool,

    output  [11:0]  cfg_fm_channel_in,
    output  [11:0]  cfg_fm_row_in,
    output  [6:0]   cfg_fm_column_in,
    output  [11:0]  cfg_cin_pair_num,
    output  [7:0]   cfg_input_col_blk_num,

    output  [2:0]   cfg_conv_kernel_size_r,
    output  [2:0]   cfg_conv_kernel_size_c,
    output  [2:0]   cfg_conv_stride_r_real,
    output  [2:0]   cfg_conv_stride_c_real,
    output  [1:0]   cfg_conv_padding_top,
    output  [1:0]   cfg_conv_padding_bottom,
    output  [1:0]   cfg_conv_padding_left,
    output  [1:0]   cfg_conv_padding_right,
    output  [11:0]  cfg_conv_fm_out_row,
    output  [6:0]   cfg_conv_fm_out_col,
    output  [31:0]  cfg_conv_line_size,
    output  [31:0]  cfg_conv_ring_size,
    output  [2:0]   cfg_conv_byte_start,

    output  [1:0]   cfg_pool_padding_left,
    output  [1:0]   cfg_pool_padding_right,
    output  [31:0]  cfg_pool_ring_size
);

localparam [1:0] OWNER_IDLE = 2'd0;
localparam [1:0] OWNER_CONV = 2'd1;
localparam [1:0] OWNER_POOL = 2'd2;

reg [1:0]   owner_reg;
reg [111:0] cmd_reg;
reg         conv_start_reg;
reg         pool_start_reg;
reg         accept_conv_d1_reg;
reg         accept_conv_d2_reg;
reg         conv_done_d1_reg;
reg [31:0]  line_size_reg;
reg [31:0]  ring_size_reg;

wire [3:0]  cmd_model_sel;
wire        owner_idle;
wire        accept_conv;
wire        accept_pool;
wire [11:0] cmd_fm_channel_in;
wire [6:0]  cmd_fm_column_in;
wire [11:0] cmd_cin_pair_num;
wire [7:0]  cmd_input_col_blk_num;

assign cmd_model_sel = rdcmd_trans[`RD_MODEL_SEL_START +: `RD_MODEL_SEL_WIDTH];
assign owner_idle    = (owner_reg == OWNER_IDLE);
assign accept_conv   = owner_idle && rdcmd_trans_valid && (cmd_model_sel == `MODEL_SEL_CONV);
assign accept_pool   = owner_idle && rdcmd_trans_valid && (cmd_model_sel == `MODEL_SEL_POOLING);

assign cmd_fm_channel_in     = rdcmd_trans[`RD_FM_CHANNEL_IN_START +: `RD_FM_CHANNEL_IN_WIDTH];
assign cmd_fm_column_in      = rdcmd_trans[`RD_FM_COLUMN_IN_START +: `RD_FM_COLUMN_IN_WIDTH];
assign cmd_cin_pair_num      = cmd_fm_channel_in >> 1;
assign cmd_input_col_blk_num = {1'b0, cmd_fm_column_in} + 8'd1;

assign conv_start    = conv_start_reg;
assign pool_start    = pool_start_reg;
assign active_is_conv = (owner_reg == OWNER_CONV);
assign active_is_pool = (owner_reg == OWNER_POOL);

assign cfg_fm_channel_in     = cmd_reg[`RD_FM_CHANNEL_IN_START +: `RD_FM_CHANNEL_IN_WIDTH];
assign cfg_fm_row_in         = cmd_reg[`RD_FM_ROW_IN_START +: `RD_FM_ROW_IN_WIDTH];
assign cfg_fm_column_in      = cmd_reg[`RD_FM_COLUMN_IN_START +: `RD_FM_COLUMN_IN_WIDTH];
assign cfg_cin_pair_num      = cfg_fm_channel_in >> 1;
assign cfg_input_col_blk_num = {1'b0, cfg_fm_column_in} + 8'd1;

assign cfg_conv_kernel_size_r  = cmd_reg[`RD_CONV_KERNEL_SIZE_R_START +: `RD_CONV_KERNEL_SIZE_R_WIDTH];
assign cfg_conv_kernel_size_c  = cmd_reg[`RD_CONV_KERNEL_SIZE_C_START +: `RD_CONV_KERNEL_SIZE_C_WIDTH];
assign cfg_conv_stride_r_real  = {1'b0, cmd_reg[`RD_CONV_STRIDE_R_START +: `RD_CONV_STRIDE_R_WIDTH]} + 3'd1;
assign cfg_conv_stride_c_real  = {1'b0, cmd_reg[`RD_CONV_STRIDE_C_START +: `RD_CONV_STRIDE_C_WIDTH]} + 3'd1;
assign cfg_conv_padding_top    = cmd_reg[`RD_CONV_PADDING_TOP_START +: `RD_CONV_PADDING_TOP_WIDTH];
assign cfg_conv_padding_bottom = cmd_reg[`RD_CONV_PADDING_BOTTOM_START +: `RD_CONV_PADDING_BOTTOM_WIDTH];
assign cfg_conv_padding_left   = cmd_reg[`RD_CONV_PADDING_LEFT_START +: `RD_CONV_PADDING_LEFT_WIDTH];
assign cfg_conv_padding_right  = cmd_reg[`RD_CONV_PADDING_RIGHT_START +: `RD_CONV_PADDING_RIGHT_WIDTH];
assign cfg_conv_fm_out_row     = cmd_reg[`RD_CONV_FM_OUT_ROW_START +: `RD_CONV_FM_OUT_ROW_WIDTH];
assign cfg_conv_fm_out_col     = cmd_reg[`RD_CONV_FM_OUT_COL_START +: `RD_CONV_FM_OUT_COL_WIDTH];
// 乘法寄存器进寄存器出：line_size 在 accept 后一拍由 cmd_reg（寄存器）纯乘装入，
// ring_size 再晚一拍由 line_size_reg 移位加装入，期间不串任何其他组合逻辑。
assign cfg_conv_line_size      = line_size_reg;
assign cfg_conv_ring_size      = ring_size_reg;
assign cfg_conv_byte_start     = 3'd3 - {1'b0, cfg_conv_padding_left};

assign cfg_pool_padding_left  = cmd_reg[`RD_POOL_PADDING_LEFT_START +: `RD_POOL_PADDING_LEFT_WIDTH];
assign cfg_pool_padding_right = cmd_reg[`RD_POOL_PADDING_RIGHT_START +: `RD_POOL_PADDING_RIGHT_WIDTH];
assign cfg_pool_ring_size     = ({24'd0, cfg_input_col_blk_num} << 3) + {24'd0, cfg_input_col_blk_num};

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        owner_reg <= OWNER_IDLE;
    else if (accept_conv)
        owner_reg <= OWNER_CONV;
    else if (accept_pool)
        owner_reg <= OWNER_POOL;
    // conv_done 为保持型电平（至下一 conv_start 才清）：必须只在上升沿释放，
    // 否则多组广播时旧 done 残留会在新会话首拍把 owner 误清回 IDLE，
    // active_is_conv 掉电后 stage1 收不到行信用、链路静默（tc5 实证）。
    else if ((owner_reg == OWNER_CONV) && conv_done && !conv_done_d1_reg)
        owner_reg <= OWNER_IDLE;
    else if ((owner_reg == OWNER_POOL) && pool_done)
        owner_reg <= OWNER_IDLE;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        conv_done_d1_reg <= 1'b0;
    else
        conv_done_d1_reg <= conv_done;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        cmd_reg <= 112'd0;
    else if (accept_conv || accept_pool)
        cmd_reg <= rdcmd_trans;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        conv_start_reg <= 1'b0;
    else
        conv_start_reg <= accept_conv;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        accept_conv_d1_reg <= 1'b0;
    else
        accept_conv_d1_reg <= accept_conv;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        accept_conv_d2_reg <= 1'b0;
    else
        accept_conv_d2_reg <= accept_conv_d1_reg;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        line_size_reg <= 32'd0;
    else if (accept_conv_d1_reg)
        line_size_reg <= {24'd0, cfg_input_col_blk_num} * {20'd0, cfg_cin_pair_num};
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        ring_size_reg <= 32'd0;
    else if (accept_conv_d2_reg)
        ring_size_reg <= (line_size_reg << 3) + line_size_reg;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        pool_start_reg <= 1'b0;
    else
        pool_start_reg <= accept_pool;
end

endmodule

// ============================================================
// RTL Self-Check Results
// ============================================================
// [PASS] Rule #1:  All ports have explicit width declarations
// [PASS] Rule #2:  Ports use wire outputs and reg state
// [PASS] Rule #6:  No debug/error-attribution logic or diagnostic-only sideband in RTL
// [PASS] Rule #8:  No logic/always_ff/always_comb used
// [PASS] Rule #12: Combinational decode uses complete continuous assignments
// [PASS] Rule #16: Sequential logic uses non-blocking assignments only
// [PASS] Rule #24: Multipliers are limited to command-time size derivation
// [PASS] Rule #34: Command fields use rd_sm_cmd_define macros
// ============================================================
