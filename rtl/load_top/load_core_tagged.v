`timescale 1ns / 1ps
`include "all_defines.vh"

// =============================================================================
// load_core_tagged
// Combined load command generation and tagged DDR data output core.
// Command issue and returned data are decoupled, so the core keeps separate
// command-side and data-side sequencers while sharing one latched 160b command
// and one set of precomputed geometry/length values.
//
// Timing rework note (20260724):
// Per-command DDR address offsets used to be combinational counter*stride
// multiplies feeding the DMA command mux (long DSP -> add -> FIFO paths).
// All command counters only advance on dma_cmd_fire, so each variable offset
// is now a synchronous accumulator that adds its stride under exactly the
// same conditions that advance the matching counter; the value presented at
// every command issue window is identical to the old multiply. Constant
// multiplies (x256/x512/x64/x2048) remain constant expressions (synthesis
// maps them to wires). Remaining variable multiplies live only in the
// PREP1/PREP2 staging, register-in register-out.
// =============================================================================
module load_core_tagged #(
    parameter LUT_BYTES        = 256,
    parameter QUAN_BLOCK_BYTES = 512
)(
    input  wire         clk,
    input  wire         rst_n,

    input  wire         s_axis_load_cmd_tvalid,
    input  wire [`RD_SM_CMD_WIDTH-1:0] s_axis_load_cmd_tdata,
    output wire         s_axis_load_cmd_tready,

    output wire         m_axis_dma_cmd_tvalid,
    output wire [`CMD2_CMD_WIDTH-1:0]  m_axis_dma_cmd_tdata,
    input  wire         m_axis_dma_cmd_tready,

    output reg          m_axis_cal_cmd_tvalid,
    output reg  [111:0] m_axis_cal_cmd_tdata,
    input  wire         m_axis_cal_cmd_tready,

    output reg          load_ba_rden,
    output reg  [10:0]  load_ba_rdaddr,
    input  wire [31:0]  load_ba_rddata,

    input  wire         store_fm_done,

    input  wire         s_axis_ddr_tvalid,
    input  wire [511:0] s_axis_ddr_tdata,
    output wire         s_axis_ddr_tready,

    output reg          m_axis_load_data_tvalid,
    output reg  [511:0] m_axis_load_data_tdata,
    output reg  [2:0]   m_axis_load_data_tuser,
    input  wire         m_axis_load_data_tready,

    output wire         m_axis_pso512_tvalid,
    output wire [511:0] m_axis_pso512_tdata,
    output wire         m_axis_pso512_tlast,
    input  wire         m_axis_pso512_tready
);

localparam IDLE       = 4'd0;
localparam LUT        = 4'd1;
localparam PAD_TOP    = 4'd2;
localparam PAD_BOTTOM = 4'd3;
localparam WEIGHT     = 4'd4;
localparam BIAS       = 4'd5;
localparam QUAN       = 4'd6;
localparam FM         = 4'd7;
localparam RESADD_FM  = 4'd8;
localparam CONCAT_FM  = 4'd9;
localparam POOL_FM    = 4'd10;
localparam PREP1      = 4'd11;
localparam PREP2      = 4'd12;
localparam PREP3      = 4'd13;
localparam PREP2B     = 4'd14;

localparam [2:0] LOAD_TAG_FM         = 3'd0;
localparam [2:0] LOAD_TAG_CPAD       = 3'd1;
localparam [2:0] LOAD_TAG_WEIGHT     = 3'd2;
localparam [2:0] LOAD_TAG_BIAS       = 3'd3;
localparam [2:0] LOAD_TAG_QUAN_PARAM = 3'd4;
localparam [2:0] LOAD_TAG_ACT        = 3'd5;
localparam [2:0] LOAD_TAG_QUAN_FM1   = 3'd6;
localparam [2:0] LOAD_TAG_QUAN_FM2   = 3'd7;

// =============================================================================
// Command accept and latch
// =============================================================================
reg [3:0] cmd_state;
reg [3:0] cmd_next_state;
reg [3:0] data_state;
reg [3:0] data_next_state;

reg [`RD_SM_CMD_WIDTH-1:0] cmd_r;
reg         fm_done_latch;

wire        load_cmd_is_export;
wire        load_cmd_can_accept;
wire        cal_cmd_forward_ready;
wire        load_cmd_accept;
wire        dma_cmd_raw_tvalid;
wire        dma_cmd_raw_tready;
wire [`CMD2_CMD_WIDTH-1:0] dma_cmd_raw_tdata;

assign load_cmd_is_export =
    (s_axis_load_cmd_tdata[`RD_MODEL_SEL_START +: `RD_MODEL_SEL_WIDTH] == `MODEL_SEL_EXPORT);
// 末层导出标记：连续推理时本轮承接凭证随末层导出收回，
// 下一轮首条计算指令重新等新一轮 FM 上传的 store_fm_done（2026-08-05 跨轮残留修复）
assign load_cmd_export_last = load_cmd_is_export &&
    s_axis_load_cmd_tdata[`RD_EXPORT_LAST_START +: `RD_EXPORT_LAST_WIDTH];
assign load_cmd_can_accept = fm_done_latch;
assign cal_cmd_forward_ready = m_axis_cal_cmd_tready &&
                               !m_axis_cal_cmd_tvalid;
assign s_axis_load_cmd_tready = (cmd_state == IDLE) &&
                                (data_state == IDLE) &&
                                load_cmd_can_accept &&
                                cal_cmd_forward_ready;
assign load_cmd_accept = s_axis_load_cmd_tvalid && s_axis_load_cmd_tready;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        fm_done_latch <= 1'b0;
    else if (store_fm_done)
        fm_done_latch <= 1'b1;
    else if (load_cmd_accept && (!load_cmd_is_export || load_cmd_export_last))
        fm_done_latch <= 1'b0;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        cmd_r <= {`RD_SM_CMD_WIDTH{1'b0}};
    else if (load_cmd_accept)
        cmd_r <= s_axis_load_cmd_tdata;
end

// =============================================================================
// Latched command fields
// =============================================================================
wire [`RD_MODEL_SEL_WIDTH-1:0]                   model_sel      = cmd_r[`RD_MODEL_SEL_START      +: `RD_MODEL_SEL_WIDTH];
wire [`RD_FM_CHANNEL_IN_WIDTH-1:0]               fm_in_channel  = cmd_r[`RD_FM_CHANNEL_IN_START  +: `RD_FM_CHANNEL_IN_WIDTH];
wire [`RD_FM_ROW_IN_WIDTH-1:0]                   row_in_size    = cmd_r[`RD_FM_ROW_IN_START      +: `RD_FM_ROW_IN_WIDTH];
wire [`RD_FM_COLUMN_IN_WIDTH-1:0]                column_in_size = cmd_r[`RD_FM_COLUMN_IN_START   +: `RD_FM_COLUMN_IN_WIDTH];
wire [`RD_FM_CHANNEL_OUT_WIDTH-1:0]              fm_channel_out = cmd_r[`RD_FM_CHANNEL_OUT_START +: `RD_FM_CHANNEL_OUT_WIDTH];
wire [`RD_CONV_KERNEL_SIZE_R_WIDTH-1:0]          kernel_size_r  = cmd_r[`RD_CONV_KERNEL_SIZE_R_START +: `RD_CONV_KERNEL_SIZE_R_WIDTH];
wire [`RD_CONV_KERNEL_SIZE_C_WIDTH-1:0]          kernel_size_c  = cmd_r[`RD_CONV_KERNEL_SIZE_C_START +: `RD_CONV_KERNEL_SIZE_C_WIDTH];
wire [`RD_CONV_PADDING_TOP_WIDTH-1:0]            padding_top    = cmd_r[`RD_CONV_PADDING_TOP_START    +: `RD_CONV_PADDING_TOP_WIDTH];
wire [`RD_CONV_PADDING_BOTTOM_WIDTH-1:0]         padding_bottom = cmd_r[`RD_CONV_PADDING_BOTTOM_START +: `RD_CONV_PADDING_BOTTOM_WIDTH];
wire [`RD_CONV_CIRCULAR_PADDING_EN_WIDTH-1:0]    circ_pad_en    = cmd_r[`RD_CONV_CIRCULAR_PADDING_EN_START  +: `RD_CONV_CIRCULAR_PADDING_EN_WIDTH];
wire [`RD_CONV_CIRCULAR_PADDING_SEL_WIDTH-1:0]   circ_pad_sel   = cmd_r[`RD_CONV_CIRCULAR_PADDING_SEL_START +: `RD_CONV_CIRCULAR_PADDING_SEL_WIDTH];
wire [`RD_CONV_OP_TYPE_WIDTH-1:0]                op_type        = cmd_r[`RD_CONV_OP_TYPE_START               +: `RD_CONV_OP_TYPE_WIDTH];
wire [`RD_EXPORT_OUT_TYPE_WIDTH-1:0]             export_out_type= cmd_r[`RD_EXPORT_OUT_TYPE_START            +: `RD_EXPORT_OUT_TYPE_WIDTH];
wire [`RD_POOL_CIRCULAR_PADDING_WIDTH-1:0]       pool_circ_en   = cmd_r[`RD_POOL_CIRCULAR_PADDING_START     +: `RD_POOL_CIRCULAR_PADDING_WIDTH];
wire [`RD_POOL_CIRCULAR_PADDING_SEL_WIDTH-1:0]   pool_circ_sel  = cmd_r[`RD_POOL_CIRCULAR_PADDING_SEL_START +: `RD_POOL_CIRCULAR_PADDING_SEL_WIDTH];
wire [`RD_CONCAT_TYPE_WIDTH-1:0]                 concat_type_f  = cmd_r[`RD_CONCAT_TYPE_START +: `RD_CONCAT_TYPE_WIDTH];
wire [1:0]                                       concat_last    = (concat_type_f == 2'd0) ? 2'd1 :
                                                                  (concat_type_f == 2'd1) ? 2'd2 : 2'd3;
wire [`RD_CONCAT_FM2_IN_CHANNEL_WIDTH-1:0]       fm2_channel    = cmd_r[`RD_CONCAT_FM2_IN_CHANNEL_START +: `RD_CONCAT_FM2_IN_CHANNEL_WIDTH];
wire [`RD_CONCAT_FM3_IN_CHANNEL_WIDTH-1:0]       fm3_channel    = cmd_r[`RD_CONCAT_FM3_IN_CHANNEL_START +: `RD_CONCAT_FM3_IN_CHANNEL_WIDTH];
wire [`RD_CONCAT_FM4_IN_CHANNEL_WIDTH-1:0]       fm4_channel    = cmd_r[`RD_CONCAT_FM4_IN_CHANNEL_START +: `RD_CONCAT_FM4_IN_CHANNEL_WIDTH];
wire [`RD_BASE_ADDR_POINT_FM_WIDTH-1:0]          ptr_fm         = cmd_r[`RD_BASE_ADDR_POINT_FM_START          +: `RD_BASE_ADDR_POINT_FM_WIDTH];
wire [`RD_BASE_ADDR_POINT_QUAN_WIDTH-1:0]        ptr_quan       = cmd_r[`RD_BASE_ADDR_POINT_QUAN_START        +: `RD_BASE_ADDR_POINT_QUAN_WIDTH];
wire [`RD_CONV_BASE_ADDR_POINT_WEIGHT_WIDTH-1:0] ptr_weight     = cmd_r[`RD_CONV_BASE_ADDR_POINT_WEIGHT_START +: `RD_CONV_BASE_ADDR_POINT_WEIGHT_WIDTH];
wire [`RD_CONV_BASE_ADDR_POINT_BIAS_WIDTH-1:0]   ptr_bias       = cmd_r[`RD_CONV_BASE_ADDR_POINT_BIAS_START   +: `RD_CONV_BASE_ADDR_POINT_BIAS_WIDTH];
wire [`RD_CONV_BASE_ADDR_POINT_ACT_WIDTH-1:0]    ptr_act        = cmd_r[`RD_CONV_BASE_ADDR_POINT_ACT_START    +: `RD_CONV_BASE_ADDR_POINT_ACT_WIDTH];
wire [`RD_MULTI_FM2_WIDTH-1:0]                   ptr_fm2        = cmd_r[`RD_MULTI_FM2_START +: `RD_MULTI_FM2_WIDTH];
wire [`RD_MULTI_FM3_WIDTH-1:0]                   ptr_fm3        = cmd_r[`RD_MULTI_FM3_START +: `RD_MULTI_FM3_WIDTH];
wire [`RD_MULTI_FM4_WIDTH-1:0]                   ptr_fm4        = cmd_r[`RD_MULTI_FM4_START +: `RD_MULTI_FM4_WIDTH];
wire [`RD_SPLIT_CH_START_WIDTH-1:0]              split_ch_start = cmd_r[`RD_SPLIT_CH_START_START +: `RD_SPLIT_CH_START_WIDTH];
wire [`RD_SPLIT_CH_NUM_WIDTH-1:0]                split_ch_num   = cmd_r[`RD_SPLIT_CH_NUM_START   +: `RD_SPLIT_CH_NUM_WIDTH];

// =============================================================================
// Geometry derived directly from command fields (real widths: col<=128,
// row<=4095, tile bytes = ch*32 <= 18b; row_bytes <= 2^24)
// =============================================================================
wire [7:0]  col_tiles_i            = {1'b0, column_in_size} + 8'd1;
wire [11:0] row_count_i            = row_in_size;
wire [17:0] bytes_per_tile_i       = {1'b0, fm_in_channel,  5'b0};
wire [17:0] fm2_bytes_per_tile_i   = {1'b0, fm2_channel, 5'b0};
wire [17:0] fm3_bytes_per_tile_i   = {1'b0, fm3_channel, 5'b0};
wire [17:0] fm4_bytes_per_tile_i   = {1'b0, fm4_channel, 5'b0};
wire [17:0] split_tile_bytes_i     = {1'b0, split_ch_num, 5'b0};

// =============================================================================
// PREP1/2/3 staged geometry/length registers
// =============================================================================
reg [31:0] col_tiles;
reg [31:0] row_count;
reg [31:0] bytes_per_tile;
reg [31:0] fm2_bytes_per_tile;
reg [31:0] fm3_bytes_per_tile;
reg [31:0] fm4_bytes_per_tile;
reg [31:0] row_bytes;
reg [31:0] fm2_row_bytes;
reg [31:0] fm3_row_bytes;
reg [31:0] fm4_row_bytes;
reg [31:0] fm_length;
reg [31:0] fm2_length;
reg [31:0] fm3_length;
reg [31:0] fm4_length;
reg [31:0] split_row_bytes;
reg [31:0] split_fm_length;
reg [31:0] pad_top_len;
reg [31:0] pad_bot_len;
reg [31:0] pad_top_off;
reg [31:0] weight_area;
reg [31:0] weight_block;
reg [31:0] pad_top_beats_m1;
reg [31:0] pad_bot_beats_m1;
reg [31:0] weight_beats_m1;
reg [31:0] quan_beats_m1;
reg [31:0] fm_beats_m1;
reg [31:0] bytes_per_tile_beats_m1;
reg [31:0] fm2_bytes_per_tile_beats_m1;
reg [31:0] fm3_bytes_per_tile_beats_m1;
reg [31:0] fm4_bytes_per_tile_beats_m1;
reg [31:0] split_fm_beats_m1;

// PREP2 multiply intermediates (DSP MREG); moved to config registers in PREP2B.
reg [31:0] fm_length_m;
reg [31:0] fm2_length_m;
reg [31:0] fm3_length_m;
reg [31:0] fm4_length_m;
reg [31:0] split_fm_length_m;
reg [31:0] pad_top_len_m;
reg [31:0] pad_bot_len_m;
reg [31:0] pad_top_off_m;
reg [31:0] weight_block_m;
// PREP1 stage: pad_top_off 的被乘数预减（把 减+乘 同拍链拆开，时序整改点）
reg [11:0] row_m_pad_r;

wire [31:0] export_length = export_out_type ? (fm_length << 1) : fm_length;
wire [31:0] fm_length_sel = (model_sel == `MODEL_SEL_EXPORT) ? export_length : fm_length;
wire [31:0] bias_block    = 32'd256;
wire [31:0] quan_block    = QUAN_BLOCK_BYTES;
wire [31:0] lut_len       = LUT_BYTES;
// 编译器 q blob 恒 512B（CONV 每 64 通道组一块；RESADD/CONCAT/SPLIT 全层一块），
// 量化搬运长度恒为单块 quan_block。旧公式 quan_total=(fm_channel_out+1)*512B 对
// RESADD/CONCAT 会数倍于真实 blob，过读数据残留在 quan_param_fifo，把后续层的
// 量化参数整组顶错位（2026-07-25 整网 l14 实证：resadd 后首个 conv 量化全错）。
wire [31:0] quan_len      = quan_block;

// ---- PREP1 stage: per-tile geometry ----
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        col_tiles <= 32'd0;
    else if (cmd_state == PREP1)
        col_tiles <= col_tiles_i;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        row_count <= 32'd0;
    else if (cmd_state == PREP1)
        row_count <= row_count_i;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        bytes_per_tile <= 32'd0;
    else if (cmd_state == PREP1)
        bytes_per_tile <= bytes_per_tile_i;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        fm2_bytes_per_tile <= 32'd0;
    else if (cmd_state == PREP1)
        fm2_bytes_per_tile <= fm2_bytes_per_tile_i;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        fm3_bytes_per_tile <= 32'd0;
    else if (cmd_state == PREP1)
        fm3_bytes_per_tile <= fm3_bytes_per_tile_i;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        fm4_bytes_per_tile <= 32'd0;
    else if (cmd_state == PREP1)
        fm4_bytes_per_tile <= fm4_bytes_per_tile_i;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        row_bytes <= 32'd0;
    else if (cmd_state == PREP1)
        row_bytes <= col_tiles_i * bytes_per_tile_i;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        fm2_row_bytes <= 32'd0;
    else if (cmd_state == PREP1)
        fm2_row_bytes <= col_tiles_i * fm2_bytes_per_tile_i;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        fm3_row_bytes <= 32'd0;
    else if (cmd_state == PREP1)
        fm3_row_bytes <= col_tiles_i * fm3_bytes_per_tile_i;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        fm4_row_bytes <= 32'd0;
    else if (cmd_state == PREP1)
        fm4_row_bytes <= col_tiles_i * fm4_bytes_per_tile_i;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        split_row_bytes <= 32'd0;
    else if (cmd_state == PREP1)
        split_row_bytes <= col_tiles_i * split_tile_bytes_i;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        weight_area <= 32'd0;
    else if (cmd_state == PREP1)
        weight_area <= {5'd0, kernel_size_r} * {5'd0, kernel_size_c};
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        row_m_pad_r <= 12'd0;
    else if (cmd_state == PREP1)
        row_m_pad_r <= row_count_i[11:0] - {10'd0, padding_top};
end

// ---- PREP2 stage: whole-map length multiplies into intermediates (MREG) ----
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        fm_length_m <= 32'd0;
    else if (cmd_state == PREP2)
        fm_length_m <= row_count[11:0] * row_bytes[23:0];
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        fm2_length_m <= 32'd0;
    else if (cmd_state == PREP2)
        fm2_length_m <= row_count[11:0] * fm2_row_bytes[23:0];
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        fm3_length_m <= 32'd0;
    else if (cmd_state == PREP2)
        fm3_length_m <= row_count[11:0] * fm3_row_bytes[23:0];
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        fm4_length_m <= 32'd0;
    else if (cmd_state == PREP2)
        fm4_length_m <= row_count[11:0] * fm4_row_bytes[23:0];
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        split_fm_length_m <= 32'd0;
    else if (cmd_state == PREP2)
        split_fm_length_m <= row_count[11:0] * split_row_bytes[23:0];
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        pad_top_len_m <= 32'd0;
    else if (cmd_state == PREP2)
        pad_top_len_m <= row_bytes[23:0] * {22'd0, padding_top};
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        pad_bot_len_m <= 32'd0;
    else if (cmd_state == PREP2)
        pad_bot_len_m <= row_bytes[23:0] * {22'd0, padding_bottom};
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        pad_top_off_m <= 32'd0;
    else if (cmd_state == PREP2)
        pad_top_off_m <= row_m_pad_r * row_bytes[23:0];
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        weight_block_m <= 32'd0;
    else if (cmd_state == PREP2)
        weight_block_m <= (weight_area[5:0] * fm_in_channel) << 6;
end

// ---- PREP2B stage: products -> config registers (PREG hop) ----
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        fm_length <= 32'd0;
    else if (cmd_state == PREP2B)
        fm_length <= fm_length_m;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        fm2_length <= 32'd0;
    else if (cmd_state == PREP2B)
        fm2_length <= fm2_length_m;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        fm3_length <= 32'd0;
    else if (cmd_state == PREP2B)
        fm3_length <= fm3_length_m;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        fm4_length <= 32'd0;
    else if (cmd_state == PREP2B)
        fm4_length <= fm4_length_m;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        split_fm_length <= 32'd0;
    else if (cmd_state == PREP2B)
        split_fm_length <= split_fm_length_m;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        pad_top_len <= 32'd0;
    else if (cmd_state == PREP2B)
        pad_top_len <= pad_top_len_m;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        pad_bot_len <= 32'd0;
    else if (cmd_state == PREP2B)
        pad_bot_len <= pad_bot_len_m;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        pad_top_off <= 32'd0;
    else if (cmd_state == PREP2B)
        pad_top_off <= pad_top_off_m;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        weight_block <= 32'd0;
    else if (cmd_state == PREP2B)
        weight_block <= weight_block_m;
end

// ---- PREP3 stage: beat counts ----
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        pad_top_beats_m1 <= 32'd0;
    else if (cmd_state == PREP3)
        pad_top_beats_m1 <= (pad_top_len >> 6) - 32'd1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        pad_bot_beats_m1 <= 32'd0;
    else if (cmd_state == PREP3)
        pad_bot_beats_m1 <= (pad_bot_len >> 6) - 32'd1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        weight_beats_m1 <= 32'd0;
    else if (cmd_state == PREP3)
        weight_beats_m1 <= (weight_block >> 6) - 32'd1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        quan_beats_m1 <= 32'd0;
    else if (cmd_state == PREP3)
        quan_beats_m1 <= (quan_len >> 6) - 32'd1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        fm_beats_m1 <= 32'd0;
    else if (cmd_state == PREP3)
        fm_beats_m1 <= (fm_length_sel >> 6) - 32'd1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        bytes_per_tile_beats_m1 <= 32'd0;
    else if (cmd_state == PREP3)
        bytes_per_tile_beats_m1 <= (bytes_per_tile >> 6) - 32'd1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        fm2_bytes_per_tile_beats_m1 <= 32'd0;
    else if (cmd_state == PREP3)
        fm2_bytes_per_tile_beats_m1 <= (fm2_bytes_per_tile >> 6) - 32'd1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        fm3_bytes_per_tile_beats_m1 <= 32'd0;
    else if (cmd_state == PREP3)
        fm3_bytes_per_tile_beats_m1 <= (fm3_bytes_per_tile >> 6) - 32'd1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        fm4_bytes_per_tile_beats_m1 <= 32'd0;
    else if (cmd_state == PREP3)
        fm4_bytes_per_tile_beats_m1 <= (fm4_bytes_per_tile >> 6) - 32'd1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        split_fm_beats_m1 <= 32'd0;
    else if (cmd_state == PREP3)
        split_fm_beats_m1 <= (split_fm_length >> 6) - 32'd1;
end

// =============================================================================
// Counter / accumulator register declarations
// =============================================================================
reg  [5:0]  cmd_c_out_cnt;
reg  [1:0]  cmd_concat_cnt;
reg [11:0]  cmd_concat_row_cnt;
reg  [6:0]  cmd_concat_col_cnt;
reg [10:0]  cmd_pool_ch_pair_cnt;
reg [11:0]  cmd_pool_row_cnt;
reg  [6:0]  cmd_pool_col_cnt;
reg [11:0]  cmd_res_row_cnt;
reg  [6:0]  cmd_res_col_cnt;
reg  [5:0]  cmd_res_grp_cnt;
reg         cmd_switch_cnt;
reg [11:0]  cmd_split_row_cnt;
reg  [6:0]  cmd_split_col_cnt;

// Offset accumulators: each mirrors its counter's update schedule and adds the
// matching stride, so at every command issue window acc == counter * stride.
reg [31:0]  weight_off_r;
reg [31:0]  pool_row_off_r;
reg [31:0]  pool_col_off_r;
reg [31:0]  res_row_off_r;
reg [31:0]  res_col_off_r;
reg [31:0]  concat_row_off_0;
reg [31:0]  concat_row_off_1;
reg [31:0]  concat_row_off_2;
reg [31:0]  concat_row_off_3;
reg [31:0]  concat_col_off_0;
reg [31:0]  concat_col_off_1;
reg [31:0]  concat_col_off_2;
reg [31:0]  concat_col_off_3;
reg [31:0]  split_row_off_r;
reg [31:0]  split_col_off_r;

reg  [5:0]  data_c_out_cnt;
reg  [1:0]  data_concat_cnt;
reg [11:0]  data_concat_row_cnt;
reg  [6:0]  data_concat_col_cnt;
reg [10:0]  data_pool_ch_pair_cnt;
reg [11:0]  data_pool_row_cnt;
reg  [6:0]  data_pool_col_cnt;
reg [11:0]  data_res_row_cnt;
reg  [6:0]  data_res_col_cnt;
reg  [5:0]  data_res_grp_cnt;
reg         data_switch_cnt;

// =============================================================================
// Counter limit / per-group helper wires
// =============================================================================
wire [6:0]  col_tiles_m1       = column_in_size;
wire [11:0] row_count_m1       = row_in_size - 12'd1;
wire [11:0] pool_ch_pair_count = {1'b0, fm_in_channel[11:1]};
wire [10:0] pool_ch_pair_total = pool_ch_pair_count[10:0] - 11'd1;
wire [5:0]  cout_last          = fm_channel_out;

// RESADD 64 通道组粒度: 每 64 通道 (32 通道对) 一组, 供流顺序
// [FM1 组 i][FM2 组 i][FM1 组 i+1]..., quan 的 resadd_port_sel 同粒度切换,
// quan→residual 的 FM1 FIFO 只需容纳一组 (无需整 tile)。
// 组数 = ceil(fm_in_channel/64); 末组通道数 = ch%64 (为 0 则 64)。
wire [6:0]  res_grp_total    = ({1'b0, fm_in_channel} + 13'd63) >> 6;
wire [5:0]  res_grp_total_m1 = res_grp_total[5:0] - 6'd1;
wire [11:0] res_grp_ch_last  = (fm_in_channel[5:0] != 6'd0)
                               ? {6'd0, fm_in_channel[5:0]} : 12'd64;
wire [11:0] cmd_res_grp_ch   = (cmd_res_grp_cnt  == res_grp_total_m1)
                               ? res_grp_ch_last : 12'd64;
wire [11:0] data_res_grp_ch  = (data_res_grp_cnt == res_grp_total_m1)
                               ? res_grp_ch_last : 12'd64;
wire [31:0] res_grp_bytes    = {20'd0, cmd_res_grp_ch} << 5;          // 组通道数 × 32B
wire [31:0] res_grp_beats_m1 = ({20'd0, data_res_grp_ch} >> 1) - 32'd1; // 组通道对数拍 -1
wire [31:0] cmd_res_grp_off  = {26'd0, cmd_res_grp_cnt} << 11;        // 64ch × 32B = 2048B

// Constant-stride offsets: synthesis maps these to wires (power-of-2 strides).
wire [31:0] bias_off         = {26'b0, cmd_c_out_cnt} * bias_block;
wire [31:0] quan_off         = {26'b0, cmd_c_out_cnt} * quan_block;
wire [31:0] quan_addr_off    = (model_sel == `MODEL_SEL_CONV) ? quan_off : 32'd0;
wire [31:0] cmd_pool_ch_off  = {21'b0, cmd_pool_ch_pair_cnt} * 32'd64;
wire [31:0] split_ch_off     = {15'b0, split_ch_start, 5'b0};
wire [31:0] split_strip_len  = {15'b0, split_ch_num, 5'b0};

// Forward declarations: assigned later, referenced by always blocks above them.
wire        dma_cmd_fire;
wire        beat_fire;
wire        data_cmd_done;
wire        active_tready;
wire        data_state_is_export;
wire        data_state_is_compute;
wire        load_data_out_ready;
wire        load_data_out_fire;
wire        load_data_pipe_ready;
wire        load_data_in_fire;

// =============================================================================
// Command-side counters (advance only on dma_cmd_fire)
// =============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        cmd_c_out_cnt <= 6'b0;
    else if (cmd_state == IDLE)
        cmd_c_out_cnt <= 6'b0;
    else if (dma_cmd_fire && cmd_state == FM && model_sel == `MODEL_SEL_CONV)
        cmd_c_out_cnt <= (cmd_c_out_cnt < cout_last) ? cmd_c_out_cnt + 6'd1 : 6'd0;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        cmd_concat_cnt <= 2'b0;
    else if (cmd_state == IDLE)
        cmd_concat_cnt <= 2'b0;
    else if (dma_cmd_fire && cmd_state == CONCAT_FM) begin
        if (cmd_concat_cnt < concat_last)
            cmd_concat_cnt <= cmd_concat_cnt + 2'd1;
        else
            cmd_concat_cnt <= 2'b0;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        cmd_concat_col_cnt <= 7'b0;
    else if (cmd_state == IDLE)
        cmd_concat_col_cnt <= 7'b0;
    else if (dma_cmd_fire && cmd_state == CONCAT_FM &&
             cmd_concat_cnt == concat_last) begin
        if (cmd_concat_col_cnt < col_tiles_m1)
            cmd_concat_col_cnt <= cmd_concat_col_cnt + 7'd1;
        else
            cmd_concat_col_cnt <= 7'b0;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        cmd_concat_row_cnt <= 12'b0;
    else if (cmd_state == IDLE)
        cmd_concat_row_cnt <= 12'b0;
    else if (dma_cmd_fire && cmd_state == CONCAT_FM &&
             cmd_concat_cnt == concat_last &&
             cmd_concat_col_cnt == col_tiles_m1 &&
             cmd_concat_row_cnt < row_count_m1)
        cmd_concat_row_cnt <= cmd_concat_row_cnt + 12'd1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        cmd_pool_col_cnt <= 7'b0;
    else if (cmd_state == IDLE)
        cmd_pool_col_cnt <= 7'b0;
    else if (dma_cmd_fire && cmd_state == POOL_FM) begin
        if (cmd_pool_col_cnt < col_tiles_m1)
            cmd_pool_col_cnt <= cmd_pool_col_cnt + 7'd1;
        else
            cmd_pool_col_cnt <= 7'b0;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        cmd_pool_row_cnt <= 12'b0;
    else if (cmd_state == IDLE)
        cmd_pool_row_cnt <= 12'b0;
    else if (dma_cmd_fire && cmd_state == POOL_FM &&
             cmd_pool_col_cnt == col_tiles_m1) begin
        if (cmd_pool_row_cnt < row_count_m1)
            cmd_pool_row_cnt <= cmd_pool_row_cnt + 12'd1;
        else
            cmd_pool_row_cnt <= 12'b0;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        cmd_pool_ch_pair_cnt <= 11'b0;
    else if (cmd_state == IDLE)
        cmd_pool_ch_pair_cnt <= 11'b0;
    else if (dma_cmd_fire && cmd_state == POOL_FM &&
             cmd_pool_col_cnt == col_tiles_m1 &&
             cmd_pool_row_cnt == row_count_m1)
        cmd_pool_ch_pair_cnt <= cmd_pool_ch_pair_cnt + 11'd1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        cmd_switch_cnt <= 1'b0;
    else if (cmd_state == IDLE)
        cmd_switch_cnt <= 1'b0;
    else if (dma_cmd_fire && cmd_state == RESADD_FM)
        cmd_switch_cnt <= ~cmd_switch_cnt;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        cmd_res_grp_cnt <= 6'b0;
    else if (cmd_state == IDLE)
        cmd_res_grp_cnt <= 6'b0;
    else if (dma_cmd_fire && cmd_state == RESADD_FM && cmd_switch_cnt == 1'b1) begin
        if (cmd_res_grp_cnt < res_grp_total_m1)
            cmd_res_grp_cnt <= cmd_res_grp_cnt + 6'd1;
        else
            cmd_res_grp_cnt <= 6'd0;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        cmd_res_col_cnt <= 7'b0;
    else if (cmd_state == IDLE)
        cmd_res_col_cnt <= 7'b0;
    else if (dma_cmd_fire && cmd_state == RESADD_FM && cmd_switch_cnt == 1'b1 &&
             cmd_res_grp_cnt == res_grp_total_m1) begin
        if (cmd_res_col_cnt < col_tiles_m1)
            cmd_res_col_cnt <= cmd_res_col_cnt + 7'd1;
        else
            cmd_res_col_cnt <= 7'b0;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        cmd_res_row_cnt <= 12'b0;
    else if (cmd_state == IDLE)
        cmd_res_row_cnt <= 12'b0;
    else if (dma_cmd_fire && cmd_state == RESADD_FM && cmd_switch_cnt == 1'b1 &&
             cmd_res_grp_cnt == res_grp_total_m1 &&
             cmd_res_col_cnt == col_tiles_m1 &&
             cmd_res_row_cnt < row_count_m1)
        cmd_res_row_cnt <= cmd_res_row_cnt + 12'd1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        cmd_split_col_cnt <= 7'b0;
    else if (cmd_state == IDLE)
        cmd_split_col_cnt <= 7'b0;
    else if (dma_cmd_fire && cmd_state == FM && model_sel == `MODEL_SEL_SPLIT) begin
        if (cmd_split_col_cnt < col_tiles_m1)
            cmd_split_col_cnt <= cmd_split_col_cnt + 7'd1;
        else
            cmd_split_col_cnt <= 7'b0;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        cmd_split_row_cnt <= 12'b0;
    else if (cmd_state == IDLE)
        cmd_split_row_cnt <= 12'b0;
    else if (dma_cmd_fire && cmd_state == FM && model_sel == `MODEL_SEL_SPLIT &&
             cmd_split_col_cnt == col_tiles_m1 &&
             cmd_split_row_cnt < row_count_m1)
        cmd_split_row_cnt <= cmd_split_row_cnt + 12'd1;
end

// =============================================================================
// Command-side offset accumulators (same update schedule as the counters)
// =============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        weight_off_r <= 32'd0;
    else if (cmd_state == IDLE)
        weight_off_r <= 32'd0;
    else if (dma_cmd_fire && cmd_state == FM && model_sel == `MODEL_SEL_CONV)
        weight_off_r <= (cmd_c_out_cnt < cout_last) ? weight_off_r + weight_block : 32'd0;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        pool_col_off_r <= 32'd0;
    else if (cmd_state == IDLE)
        pool_col_off_r <= 32'd0;
    else if (dma_cmd_fire && cmd_state == POOL_FM) begin
        if (cmd_pool_col_cnt < col_tiles_m1)
            pool_col_off_r <= pool_col_off_r + bytes_per_tile;
        else
            pool_col_off_r <= 32'd0;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        pool_row_off_r <= 32'd0;
    else if (cmd_state == IDLE)
        pool_row_off_r <= 32'd0;
    else if (dma_cmd_fire && cmd_state == POOL_FM &&
             cmd_pool_col_cnt == col_tiles_m1) begin
        if (cmd_pool_row_cnt < row_count_m1)
            pool_row_off_r <= pool_row_off_r + row_bytes;
        else
            pool_row_off_r <= 32'd0;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        res_col_off_r <= 32'd0;
    else if (cmd_state == IDLE)
        res_col_off_r <= 32'd0;
    else if (dma_cmd_fire && cmd_state == RESADD_FM && cmd_switch_cnt == 1'b1 &&
             cmd_res_grp_cnt == res_grp_total_m1) begin
        if (cmd_res_col_cnt < col_tiles_m1)
            res_col_off_r <= res_col_off_r + bytes_per_tile;
        else
            res_col_off_r <= 32'd0;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        res_row_off_r <= 32'd0;
    else if (cmd_state == IDLE)
        res_row_off_r <= 32'd0;
    else if (dma_cmd_fire && cmd_state == RESADD_FM && cmd_switch_cnt == 1'b1 &&
             cmd_res_grp_cnt == res_grp_total_m1 &&
             cmd_res_col_cnt == col_tiles_m1 &&
             cmd_res_row_cnt < row_count_m1)
        res_row_off_r <= res_row_off_r + row_bytes;
end

// CONCAT: col/row counters are shared by all streams, so each stream keeps its
// own row/col offset accumulator; every accumulator advances with its own
// per-stream stride when the shared counter advances.
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        concat_col_off_0 <= 32'd0;
    else if (cmd_state == IDLE)
        concat_col_off_0 <= 32'd0;
    else if (dma_cmd_fire && cmd_state == CONCAT_FM &&
             cmd_concat_cnt == concat_last) begin
        if (cmd_concat_col_cnt < col_tiles_m1)
            concat_col_off_0 <= concat_col_off_0 + bytes_per_tile;
        else
            concat_col_off_0 <= 32'd0;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        concat_col_off_1 <= 32'd0;
    else if (cmd_state == IDLE)
        concat_col_off_1 <= 32'd0;
    else if (dma_cmd_fire && cmd_state == CONCAT_FM &&
             cmd_concat_cnt == concat_last) begin
        if (cmd_concat_col_cnt < col_tiles_m1)
            concat_col_off_1 <= concat_col_off_1 + fm2_bytes_per_tile;
        else
            concat_col_off_1 <= 32'd0;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        concat_col_off_2 <= 32'd0;
    else if (cmd_state == IDLE)
        concat_col_off_2 <= 32'd0;
    else if (dma_cmd_fire && cmd_state == CONCAT_FM &&
             cmd_concat_cnt == concat_last) begin
        if (cmd_concat_col_cnt < col_tiles_m1)
            concat_col_off_2 <= concat_col_off_2 + fm3_bytes_per_tile;
        else
            concat_col_off_2 <= 32'd0;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        concat_col_off_3 <= 32'd0;
    else if (cmd_state == IDLE)
        concat_col_off_3 <= 32'd0;
    else if (dma_cmd_fire && cmd_state == CONCAT_FM &&
             cmd_concat_cnt == concat_last) begin
        if (cmd_concat_col_cnt < col_tiles_m1)
            concat_col_off_3 <= concat_col_off_3 + fm4_bytes_per_tile;
        else
            concat_col_off_3 <= 32'd0;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        concat_row_off_0 <= 32'd0;
    else if (cmd_state == IDLE)
        concat_row_off_0 <= 32'd0;
    else if (dma_cmd_fire && cmd_state == CONCAT_FM &&
             cmd_concat_cnt == concat_last &&
             cmd_concat_col_cnt == col_tiles_m1 &&
             cmd_concat_row_cnt < row_count_m1)
        concat_row_off_0 <= concat_row_off_0 + row_bytes;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        concat_row_off_1 <= 32'd0;
    else if (cmd_state == IDLE)
        concat_row_off_1 <= 32'd0;
    else if (dma_cmd_fire && cmd_state == CONCAT_FM &&
             cmd_concat_cnt == concat_last &&
             cmd_concat_col_cnt == col_tiles_m1 &&
             cmd_concat_row_cnt < row_count_m1)
        concat_row_off_1 <= concat_row_off_1 + fm2_row_bytes;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        concat_row_off_2 <= 32'd0;
    else if (cmd_state == IDLE)
        concat_row_off_2 <= 32'd0;
    else if (dma_cmd_fire && cmd_state == CONCAT_FM &&
             cmd_concat_cnt == concat_last &&
             cmd_concat_col_cnt == col_tiles_m1 &&
             cmd_concat_row_cnt < row_count_m1)
        concat_row_off_2 <= concat_row_off_2 + fm3_row_bytes;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        concat_row_off_3 <= 32'd0;
    else if (cmd_state == IDLE)
        concat_row_off_3 <= 32'd0;
    else if (dma_cmd_fire && cmd_state == CONCAT_FM &&
             cmd_concat_cnt == concat_last &&
             cmd_concat_col_cnt == col_tiles_m1 &&
             cmd_concat_row_cnt < row_count_m1)
        concat_row_off_3 <= concat_row_off_3 + fm4_row_bytes;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        split_col_off_r <= 32'd0;
    else if (cmd_state == IDLE)
        split_col_off_r <= 32'd0;
    else if (dma_cmd_fire && cmd_state == FM && model_sel == `MODEL_SEL_SPLIT) begin
        if (cmd_split_col_cnt < col_tiles_m1)
            split_col_off_r <= split_col_off_r + bytes_per_tile;
        else
            split_col_off_r <= 32'd0;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        split_row_off_r <= 32'd0;
    else if (cmd_state == IDLE)
        split_row_off_r <= 32'd0;
    else if (dma_cmd_fire && cmd_state == FM && model_sel == `MODEL_SEL_SPLIT &&
             cmd_split_col_cnt == col_tiles_m1 &&
             cmd_split_row_cnt < row_count_m1)
        split_row_off_r <= split_row_off_r + row_bytes;
end

// =============================================================================
// CONCAT stream muxes
// =============================================================================
reg [10:0] cmd_concat_ptr;
always @(*) begin
    case (cmd_concat_cnt)
        2'd0:    cmd_concat_ptr = ptr_fm;
        2'd1:    cmd_concat_ptr = ptr_fm2;
        2'd2:    cmd_concat_ptr = ptr_fm3;
        2'd3:    cmd_concat_ptr = ptr_fm4;
        default: cmd_concat_ptr = ptr_fm;
    endcase
end

reg [31:0] cmd_concat_bytes_per_tile;
always @(*) begin
    case (cmd_concat_cnt)
        2'd0:    cmd_concat_bytes_per_tile = bytes_per_tile;
        2'd1:    cmd_concat_bytes_per_tile = fm2_bytes_per_tile;
        2'd2:    cmd_concat_bytes_per_tile = fm3_bytes_per_tile;
        2'd3:    cmd_concat_bytes_per_tile = fm4_bytes_per_tile;
        default: cmd_concat_bytes_per_tile = bytes_per_tile;
    endcase
end

wire [31:0] cmd_concat_row_off = (cmd_concat_cnt == 2'd1) ? concat_row_off_1 :
                                 (cmd_concat_cnt == 2'd2) ? concat_row_off_2 :
                                 (cmd_concat_cnt == 2'd3) ? concat_row_off_3 :
                                                            concat_row_off_0;
wire [31:0] cmd_concat_col_off = (cmd_concat_cnt == 2'd1) ? concat_col_off_1 :
                                 (cmd_concat_cnt == 2'd2) ? concat_col_off_2 :
                                 (cmd_concat_cnt == 2'd3) ? concat_col_off_3 :
                                                            concat_col_off_0;

reg [31:0] data_concat_beats_m1;
always @(*) begin
    case (data_concat_cnt)
        2'd0:    data_concat_beats_m1 = bytes_per_tile_beats_m1;
        2'd1:    data_concat_beats_m1 = fm2_bytes_per_tile_beats_m1;
        2'd2:    data_concat_beats_m1 = fm3_bytes_per_tile_beats_m1;
        2'd3:    data_concat_beats_m1 = fm4_bytes_per_tile_beats_m1;
        default: data_concat_beats_m1 = bytes_per_tile_beats_m1;
    endcase
end

// =============================================================================
// Base address read / hold
// =============================================================================
reg  addr_ready;
reg  addr_wait;
reg  base_addr_valid;
reg [31:0] base_addr_r;

wire cmd_active_state = (cmd_state != IDLE) &&
                        (cmd_state != PREP1) &&
                        (cmd_state != PREP2) &&
                        (cmd_state != PREP2B) &&
                        (cmd_state != PREP3);
wire [31:0] dma_cmd_base_addr = base_addr_valid ? base_addr_r : load_ba_rddata;

// BA RAM 换 BMG IP（DO_REG=1）后读出为 2 拍：地址给出后第 2 拍数据有效，
// addr_ready 经 addr_wait 再延一拍，消费点（dma_cmd_tvalid/base_addr_r 锁存）随之对齐。
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        addr_wait <= 1'b0;
    else if (dma_cmd_fire)
        addr_wait <= 1'b0;
    else if (cmd_active_state && !addr_ready)
        addr_wait <= 1'b1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        addr_ready <= 1'b0;
    else if (dma_cmd_fire)
        addr_ready <= 1'b0;
    else if (cmd_active_state && !addr_ready)
        addr_ready <= addr_wait;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        base_addr_valid <= 1'b0;
    else if (!cmd_active_state || dma_cmd_fire)
        base_addr_valid <= 1'b0;
    else if (addr_ready && !base_addr_valid && !dma_cmd_raw_tready)
        base_addr_valid <= 1'b1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        base_addr_r <= 32'b0;
    else if (!cmd_active_state || dma_cmd_fire)
        base_addr_r <= 32'b0;
    else if (addr_ready && !base_addr_valid && !dma_cmd_raw_tready)
        base_addr_r <= load_ba_rddata;
end

// =============================================================================
// Data-side counters (advance only on data_cmd_done)
// =============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        data_c_out_cnt <= 6'b0;
    else if (data_state == IDLE)
        data_c_out_cnt <= 6'b0;
    else if (data_cmd_done && data_state == FM && model_sel == `MODEL_SEL_CONV)
        data_c_out_cnt <= (data_c_out_cnt < cout_last) ? data_c_out_cnt + 6'd1 : 6'd0;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        data_concat_cnt <= 2'b0;
    else if (data_state == IDLE)
        data_concat_cnt <= 2'b0;
    else if (data_cmd_done && data_state == CONCAT_FM) begin
        if (data_concat_cnt < concat_last)
            data_concat_cnt <= data_concat_cnt + 2'd1;
        else
            data_concat_cnt <= 2'b0;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        data_concat_col_cnt <= 7'b0;
    else if (data_state == IDLE)
        data_concat_col_cnt <= 7'b0;
    else if (data_cmd_done && data_state == CONCAT_FM &&
             data_concat_cnt == concat_last) begin
        if (data_concat_col_cnt < col_tiles_m1)
            data_concat_col_cnt <= data_concat_col_cnt + 7'd1;
        else
            data_concat_col_cnt <= 7'b0;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        data_concat_row_cnt <= 12'b0;
    else if (data_state == IDLE)
        data_concat_row_cnt <= 12'b0;
    else if (data_cmd_done && data_state == CONCAT_FM &&
             data_concat_cnt == concat_last &&
             data_concat_col_cnt == col_tiles_m1 &&
             data_concat_row_cnt < row_count_m1)
        data_concat_row_cnt <= data_concat_row_cnt + 12'd1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        data_pool_col_cnt <= 7'b0;
    else if (data_state == IDLE)
        data_pool_col_cnt <= 7'b0;
    else if (data_cmd_done && data_state == POOL_FM) begin
        if (data_pool_col_cnt < col_tiles_m1)
            data_pool_col_cnt <= data_pool_col_cnt + 7'd1;
        else
            data_pool_col_cnt <= 7'b0;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        data_pool_row_cnt <= 12'b0;
    else if (data_state == IDLE)
        data_pool_row_cnt <= 12'b0;
    else if (data_cmd_done && data_state == POOL_FM &&
             data_pool_col_cnt == col_tiles_m1) begin
        if (data_pool_row_cnt < row_count_m1)
            data_pool_row_cnt <= data_pool_row_cnt + 12'd1;
        else
            data_pool_row_cnt <= 12'b0;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        data_pool_ch_pair_cnt <= 11'b0;
    else if (data_state == IDLE)
        data_pool_ch_pair_cnt <= 11'b0;
    else if (data_cmd_done && data_state == POOL_FM &&
             data_pool_col_cnt == col_tiles_m1 &&
             data_pool_row_cnt == row_count_m1)
        data_pool_ch_pair_cnt <= data_pool_ch_pair_cnt + 11'd1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        data_switch_cnt <= 1'b0;
    else if (data_state == IDLE)
        data_switch_cnt <= 1'b0;
    else if (data_cmd_done && data_state == RESADD_FM)
        data_switch_cnt <= ~data_switch_cnt;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        data_res_grp_cnt <= 6'b0;
    else if (data_state == IDLE)
        data_res_grp_cnt <= 6'b0;
    else if (data_cmd_done && data_state == RESADD_FM && data_switch_cnt == 1'b1) begin
        if (data_res_grp_cnt < res_grp_total_m1)
            data_res_grp_cnt <= data_res_grp_cnt + 6'd1;
        else
            data_res_grp_cnt <= 6'd0;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        data_res_col_cnt <= 7'b0;
    else if (data_state == IDLE)
        data_res_col_cnt <= 7'b0;
    else if (data_cmd_done && data_state == RESADD_FM && data_switch_cnt == 1'b1 &&
             data_res_grp_cnt == res_grp_total_m1) begin
        if (data_res_col_cnt < col_tiles_m1)
            data_res_col_cnt <= data_res_col_cnt + 7'd1;
        else
            data_res_col_cnt <= 7'b0;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        data_res_row_cnt <= 12'b0;
    else if (data_state == IDLE)
        data_res_row_cnt <= 12'b0;
    else if (data_cmd_done && data_state == RESADD_FM && data_switch_cnt == 1'b1 &&
             data_res_grp_cnt == res_grp_total_m1 &&
             data_res_col_cnt == col_tiles_m1 &&
             data_res_row_cnt < row_count_m1)
        data_res_row_cnt <= data_res_row_cnt + 12'd1;
end

// =============================================================================
// Data beat counting
// =============================================================================
reg [31:0] data_exp_beats_m1;
always @(*) begin
    case (data_state)
        LUT:        data_exp_beats_m1 = LUT_BYTES / 64 - 1;
        PAD_TOP:    data_exp_beats_m1 = pad_top_beats_m1;
        PAD_BOTTOM: data_exp_beats_m1 = pad_bot_beats_m1;
        WEIGHT:     data_exp_beats_m1 = weight_beats_m1;
        BIAS:       data_exp_beats_m1 = 32'd3;
        QUAN:       data_exp_beats_m1 = quan_beats_m1;
        FM:         data_exp_beats_m1 = (model_sel == `MODEL_SEL_SPLIT) ? split_fm_beats_m1 : fm_beats_m1;
        POOL_FM:    data_exp_beats_m1 = 32'd0;
        RESADD_FM:  data_exp_beats_m1 = res_grp_beats_m1;
        CONCAT_FM:  data_exp_beats_m1 = data_concat_beats_m1;
        default:    data_exp_beats_m1 = 32'd0;
    endcase
end

reg [31:0] data_beat_cnt;

assign beat_fire      = s_axis_ddr_tvalid && active_tready;
assign data_cmd_done  = beat_fire && (data_beat_cnt == data_exp_beats_m1);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        data_beat_cnt <= 32'd0;
    else if (data_state == IDLE)
        data_beat_cnt <= 32'd0;
    else if (data_cmd_done)
        data_beat_cnt <= 32'd0;
    else if (beat_fire)
        data_beat_cnt <= data_beat_cnt + 32'd1;
end

// =============================================================================
// Sequencers
// =============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        cmd_state <= IDLE;
    else
        cmd_state <= cmd_next_state;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        data_state <= IDLE;
    else
        data_state <= data_next_state;
end

always @(*) begin
    cmd_next_state = cmd_state;
    case (cmd_state)
        IDLE: begin
            if (load_cmd_accept)
                cmd_next_state = PREP1;
        end
        PREP1: cmd_next_state = PREP2;
        PREP2: cmd_next_state = PREP2B;
        PREP2B: cmd_next_state = PREP3;
        PREP3: begin
            case (model_sel)
                `MODEL_SEL_CONV: begin
                    if (op_type)
                        cmd_next_state = LUT;
                    else if (circ_pad_en && circ_pad_sel && padding_top > 0)
                        cmd_next_state = PAD_TOP;
                    else if (circ_pad_en && circ_pad_sel && padding_bottom > 0)
                        cmd_next_state = PAD_BOTTOM;
                    else
                        cmd_next_state = WEIGHT;
                end
                `MODEL_SEL_POOLING: begin
                    if (pool_circ_en && pool_circ_sel && padding_top > 0)
                        cmd_next_state = PAD_TOP;
                    else if (pool_circ_en && pool_circ_sel && padding_bottom > 0)
                        cmd_next_state = PAD_BOTTOM;
                    else
                        cmd_next_state = POOL_FM;
                end
                `MODEL_SEL_RESADD:   cmd_next_state = QUAN;
                `MODEL_SEL_CONCAT:   cmd_next_state = QUAN;
                `MODEL_SEL_UPSAMPLE: cmd_next_state = FM;
                `MODEL_SEL_SPLIT:    cmd_next_state = QUAN;
                `MODEL_SEL_EXPORT:   cmd_next_state = FM;
                default:             cmd_next_state = IDLE;
            endcase
        end
        LUT: begin
            if (dma_cmd_fire) begin
                if (circ_pad_en && circ_pad_sel && padding_top > 0)
                    cmd_next_state = PAD_TOP;
                else if (circ_pad_en && circ_pad_sel && padding_bottom > 0)
                    cmd_next_state = PAD_BOTTOM;
                else
                    cmd_next_state = WEIGHT;
            end
        end
        PAD_TOP: begin
            if (dma_cmd_fire) begin
                if ((model_sel == `MODEL_SEL_CONV   && circ_pad_en  && circ_pad_sel  && padding_bottom > 0) ||
                    (model_sel == `MODEL_SEL_POOLING && pool_circ_en && pool_circ_sel && padding_bottom > 0))
                    cmd_next_state = PAD_BOTTOM;
                else begin
                    case (model_sel)
                        `MODEL_SEL_CONV:    cmd_next_state = WEIGHT;
                        `MODEL_SEL_POOLING: cmd_next_state = POOL_FM;
                        default:            cmd_next_state = IDLE;
                    endcase
                end
            end
        end
        PAD_BOTTOM: begin
            if (dma_cmd_fire) begin
                case (model_sel)
                    `MODEL_SEL_CONV:    cmd_next_state = WEIGHT;
                    `MODEL_SEL_POOLING: cmd_next_state = POOL_FM;
                    default:            cmd_next_state = IDLE;
                endcase
            end
        end
        WEIGHT: begin
            if (dma_cmd_fire)
                cmd_next_state = BIAS;
        end
        BIAS: begin
            if (dma_cmd_fire)
                cmd_next_state = QUAN;
        end
        QUAN: begin
            if (dma_cmd_fire) begin
                case (model_sel)
                    `MODEL_SEL_RESADD: cmd_next_state = RESADD_FM;
                    `MODEL_SEL_CONCAT: cmd_next_state = CONCAT_FM;
                    default:           cmd_next_state = FM;
                endcase
            end
        end
        FM: begin
            if (dma_cmd_fire) begin
                if (model_sel == `MODEL_SEL_CONV && cmd_c_out_cnt < cout_last)
                    cmd_next_state = WEIGHT;
                else if (model_sel == `MODEL_SEL_SPLIT &&
                         (cmd_split_col_cnt != col_tiles_m1 || cmd_split_row_cnt != row_count_m1))
                    cmd_next_state = FM;
                else
                    cmd_next_state = IDLE;
            end
        end
        POOL_FM: begin
            if (dma_cmd_fire &&
                cmd_pool_col_cnt     == col_tiles_m1 &&
                cmd_pool_row_cnt     == row_count_m1 &&
                cmd_pool_ch_pair_cnt == pool_ch_pair_total)
                cmd_next_state = IDLE;
        end
        RESADD_FM: begin
            if (dma_cmd_fire &&
                cmd_switch_cnt  == 1'b1 &&
                cmd_res_grp_cnt == res_grp_total_m1 &&
                cmd_res_col_cnt == col_tiles_m1 &&
                cmd_res_row_cnt == row_count_m1)
                cmd_next_state = IDLE;
        end
        CONCAT_FM: begin
            if (dma_cmd_fire &&
                cmd_concat_cnt     == concat_last &&
                cmd_concat_col_cnt == col_tiles_m1 &&
                cmd_concat_row_cnt == row_count_m1)
                cmd_next_state = IDLE;
        end
        default: cmd_next_state = IDLE;
    endcase
end

always @(*) begin
    data_next_state = data_state;
    case (data_state)
        IDLE: begin
            if (load_cmd_accept)
                data_next_state = PREP1;
        end
        PREP1: data_next_state = PREP2;
        PREP2: data_next_state = PREP2B;
        PREP2B: data_next_state = PREP3;
        PREP3: begin
            case (model_sel)
                `MODEL_SEL_CONV: begin
                    if (op_type)
                        data_next_state = LUT;
                    else if (circ_pad_en && circ_pad_sel && padding_top > 0)
                        data_next_state = PAD_TOP;
                    else if (circ_pad_en && circ_pad_sel && padding_bottom > 0)
                        data_next_state = PAD_BOTTOM;
                    else
                        data_next_state = WEIGHT;
                end
                `MODEL_SEL_POOLING: begin
                    if (pool_circ_en && pool_circ_sel && padding_top > 0)
                        data_next_state = PAD_TOP;
                    else if (pool_circ_en && pool_circ_sel && padding_bottom > 0)
                        data_next_state = PAD_BOTTOM;
                    else
                        data_next_state = POOL_FM;
                end
                `MODEL_SEL_RESADD:   data_next_state = QUAN;
                `MODEL_SEL_CONCAT:   data_next_state = QUAN;
                `MODEL_SEL_UPSAMPLE: data_next_state = FM;
                `MODEL_SEL_SPLIT:    data_next_state = QUAN;
                `MODEL_SEL_EXPORT:   data_next_state = FM;
                default:             data_next_state = IDLE;
            endcase
        end
        LUT: begin
            if (data_cmd_done) begin
                if (circ_pad_en && circ_pad_sel && padding_top > 0)
                    data_next_state = PAD_TOP;
                else if (circ_pad_en && circ_pad_sel && padding_bottom > 0)
                    data_next_state = PAD_BOTTOM;
                else
                    data_next_state = WEIGHT;
            end
        end
        PAD_TOP: begin
            if (data_cmd_done) begin
                if ((model_sel == `MODEL_SEL_CONV   && circ_pad_en  && circ_pad_sel  && padding_bottom > 0) ||
                    (model_sel == `MODEL_SEL_POOLING && pool_circ_en && pool_circ_sel && padding_bottom > 0))
                    data_next_state = PAD_BOTTOM;
                else begin
                    case (model_sel)
                        `MODEL_SEL_CONV:    data_next_state = WEIGHT;
                        `MODEL_SEL_POOLING: data_next_state = POOL_FM;
                        default:            data_next_state = IDLE;
                    endcase
                end
            end
        end
        PAD_BOTTOM: begin
            if (data_cmd_done) begin
                case (model_sel)
                    `MODEL_SEL_CONV:    data_next_state = WEIGHT;
                    `MODEL_SEL_POOLING: data_next_state = POOL_FM;
                    default:            data_next_state = IDLE;
                endcase
            end
        end
        WEIGHT: begin
            if (data_cmd_done)
                data_next_state = BIAS;
        end
        BIAS: begin
            if (data_cmd_done)
                data_next_state = QUAN;
        end
        QUAN: begin
            if (data_cmd_done) begin
                case (model_sel)
                    `MODEL_SEL_RESADD: data_next_state = RESADD_FM;
                    `MODEL_SEL_CONCAT: data_next_state = CONCAT_FM;
                    default:           data_next_state = FM;
                endcase
            end
        end
        FM: begin
            if (data_cmd_done) begin
                if (model_sel == `MODEL_SEL_CONV && data_c_out_cnt < cout_last)
                    data_next_state = WEIGHT;
                else
                    data_next_state = IDLE;
            end
        end
        POOL_FM: begin
            if (data_cmd_done &&
                data_pool_col_cnt     == col_tiles_m1 &&
                data_pool_row_cnt     == row_count_m1 &&
                data_pool_ch_pair_cnt == pool_ch_pair_total)
                data_next_state = IDLE;
        end
        RESADD_FM: begin
            if (data_cmd_done &&
                data_switch_cnt  == 1'b1 &&
                data_res_grp_cnt == res_grp_total_m1 &&
                data_res_col_cnt == col_tiles_m1 &&
                data_res_row_cnt == row_count_m1)
                data_next_state = IDLE;
        end
        CONCAT_FM: begin
            if (data_cmd_done &&
                data_concat_cnt     == concat_last &&
                data_concat_col_cnt == col_tiles_m1 &&
                data_concat_row_cnt == row_count_m1)
                data_next_state = IDLE;
        end
        default: data_next_state = IDLE;
    endcase
end

// =============================================================================
// Base address RAM read mux
// =============================================================================
always @(*) begin
    load_ba_rden = 1'b0;
    if (cmd_active_state && !addr_ready)
        load_ba_rden = 1'b1;
end

always @(*) begin
    load_ba_rdaddr = 11'b0;
    if (cmd_active_state && !addr_ready) begin
        case (cmd_state)
            LUT:                          load_ba_rdaddr = ptr_act;
            PAD_TOP, PAD_BOTTOM, FM,
            POOL_FM:                      load_ba_rdaddr = ptr_fm;
            WEIGHT:                       load_ba_rdaddr = ptr_weight;
            BIAS:                         load_ba_rdaddr = ptr_bias;
            QUAN:                         load_ba_rdaddr = ptr_quan;
            RESADD_FM:                    load_ba_rdaddr = cmd_switch_cnt ? ptr_fm2 : ptr_fm;
            CONCAT_FM:                    load_ba_rdaddr = cmd_concat_ptr;
            default:                      load_ba_rdaddr = ptr_fm;
        endcase
    end
end

// =============================================================================
// DMA read command issue
// =============================================================================
assign dma_cmd_fire = dma_cmd_raw_tvalid && dma_cmd_raw_tready;
assign dma_cmd_raw_tvalid = cmd_active_state && addr_ready;

reg [`CMD2_CMD_WIDTH-1:0] dma_cmd_raw_tdata_reg;
assign dma_cmd_raw_tdata = dma_cmd_raw_tdata_reg;

always @(*) begin
    dma_cmd_raw_tdata_reg = {`CMD2_CMD_WIDTH{1'b0}};
    if (cmd_active_state && addr_ready) begin
        case (cmd_state)
            LUT:
                dma_cmd_raw_tdata_reg = {dma_cmd_base_addr, lut_len};
            PAD_TOP:
                dma_cmd_raw_tdata_reg = {dma_cmd_base_addr + pad_top_off, pad_top_len};
            PAD_BOTTOM:
                dma_cmd_raw_tdata_reg = {dma_cmd_base_addr, pad_bot_len};
            WEIGHT:
                dma_cmd_raw_tdata_reg = {dma_cmd_base_addr + weight_off_r, weight_block};
            BIAS:
                dma_cmd_raw_tdata_reg = {dma_cmd_base_addr + bias_off, bias_block};
            QUAN:
                dma_cmd_raw_tdata_reg = {dma_cmd_base_addr + quan_addr_off, quan_len};
            FM:
                dma_cmd_raw_tdata_reg = (model_sel == `MODEL_SEL_SPLIT) ?
                                       {dma_cmd_base_addr + split_row_off_r + split_col_off_r + split_ch_off, split_strip_len} :
                                       {dma_cmd_base_addr, fm_length_sel};
            POOL_FM:
                dma_cmd_raw_tdata_reg = {dma_cmd_base_addr + pool_row_off_r + pool_col_off_r + cmd_pool_ch_off, 32'd64};
            RESADD_FM:
                dma_cmd_raw_tdata_reg = {dma_cmd_base_addr + res_row_off_r + res_col_off_r + cmd_res_grp_off, res_grp_bytes};
            CONCAT_FM:
                dma_cmd_raw_tdata_reg = {dma_cmd_base_addr + cmd_concat_row_off + cmd_concat_col_off, cmd_concat_bytes_per_tile};
            default:
                dma_cmd_raw_tdata_reg = {`CMD2_CMD_WIDTH{1'b0}};
        endcase
    end
end

// =============================================================================
// CAL command forward
// =============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        m_axis_cal_cmd_tvalid <= 1'b0;
    else if (m_axis_cal_cmd_tvalid && m_axis_cal_cmd_tready)
        m_axis_cal_cmd_tvalid <= 1'b0;
    else if (load_cmd_accept && !load_cmd_is_export)
        m_axis_cal_cmd_tvalid <= 1'b1;
        // 2026-08-05：EXPORT 不转发 cal_cmd——cal 域无消费者（无 MODEL_SEL_EXPORT
        // 译码），转发只会让 dispatch 空等握手（多导出头模型供需错位根因）。
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        m_axis_cal_cmd_tdata <= 112'b0;
    else if (load_cmd_accept)
        m_axis_cal_cmd_tdata <= {7'b0, s_axis_load_cmd_tdata[104:0]};
end

// =============================================================================
// DDR return tagged data output
// =============================================================================
reg active_tready_r;
reg [2:0] load_data_tag_mux;

assign data_state_is_export =
    (data_state == FM) && (model_sel == `MODEL_SEL_EXPORT);

assign data_state_is_compute =
    (data_state == LUT) ||
    (data_state == PAD_TOP) ||
    (data_state == PAD_BOTTOM) ||
    (data_state == WEIGHT) ||
    (data_state == BIAS) ||
    (data_state == QUAN) ||
    ((data_state == FM) && !data_state_is_export) ||
    (data_state == POOL_FM) ||
    (data_state == RESADD_FM) ||
    (data_state == CONCAT_FM);

// Backpressure rule (tc6 deadlock fix): throttle only the DDR return source.
// 刹车，不作为当拍传输判据——FIFO 的 s_axis_tready 只认 full 不认
// Do not duplicate the same throttle on out_ready; that would freeze the core.
// valid/data 而 FIFO 仍在写，同一拍被反复写入直至爆 FIFO、复制行灌死下游。
// 正确分工：out_ready 只跟 FIFO tready（在途最后一拍正常写入，随后 valid
// pipe_ready uses only local holding/elastic capacity for accepted in-flight beats.
// 落在 128 拍余量内）。
assign load_data_out_ready  = m_axis_load_data_tready;
assign load_data_out_fire   = m_axis_load_data_tvalid && load_data_out_ready;
assign load_data_pipe_ready = (!m_axis_load_data_tvalid || load_data_out_fire);
assign load_data_in_fire    = beat_fire && data_state_is_compute;

always @(*) begin
    case (data_state)
        LUT:                 active_tready_r = load_data_pipe_ready;
        PAD_TOP, PAD_BOTTOM: active_tready_r = load_data_pipe_ready;
        WEIGHT:              active_tready_r = load_data_pipe_ready;
        BIAS:                active_tready_r = load_data_pipe_ready;
        QUAN:                active_tready_r = load_data_pipe_ready;
        FM: begin
            case (model_sel)
                `MODEL_SEL_EXPORT:                     active_tready_r = m_axis_pso512_tready;
                `MODEL_SEL_UPSAMPLE, `MODEL_SEL_SPLIT: active_tready_r = load_data_pipe_ready;
                default:                               active_tready_r = load_data_pipe_ready;
            endcase
        end
        POOL_FM:   active_tready_r = load_data_pipe_ready;
        RESADD_FM: active_tready_r = load_data_pipe_ready;
        CONCAT_FM: active_tready_r = load_data_pipe_ready;
        default:   active_tready_r = 1'b0;
    endcase
end

assign active_tready     = active_tready_r;
assign s_axis_ddr_tready = active_tready_r;

always @(*) begin
    load_data_tag_mux = LOAD_TAG_FM;
    case (data_state)
        LUT:                 load_data_tag_mux = LOAD_TAG_ACT;
        PAD_TOP, PAD_BOTTOM: load_data_tag_mux = LOAD_TAG_CPAD;
        WEIGHT:              load_data_tag_mux = LOAD_TAG_WEIGHT;
        BIAS:                load_data_tag_mux = LOAD_TAG_BIAS;
        QUAN:                load_data_tag_mux = LOAD_TAG_QUAN_PARAM;
        FM: begin
            case (model_sel)
                `MODEL_SEL_UPSAMPLE: load_data_tag_mux = LOAD_TAG_QUAN_FM1;
                `MODEL_SEL_SPLIT:    load_data_tag_mux = LOAD_TAG_QUAN_FM1;
                default:             load_data_tag_mux = LOAD_TAG_FM;
            endcase
        end
        POOL_FM:   load_data_tag_mux = LOAD_TAG_FM;
        RESADD_FM: load_data_tag_mux = data_switch_cnt ? LOAD_TAG_QUAN_FM2 : LOAD_TAG_QUAN_FM1;
        CONCAT_FM: load_data_tag_mux = LOAD_TAG_QUAN_FM1;
        default:   load_data_tag_mux = LOAD_TAG_FM;
    endcase
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        m_axis_load_data_tvalid <= 1'b0;
    else if (load_data_in_fire)
        m_axis_load_data_tvalid <= 1'b1;
    else if (load_data_out_fire)
        m_axis_load_data_tvalid <= 1'b0;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        m_axis_load_data_tdata <= 512'b0;
    else if (load_data_in_fire)
        m_axis_load_data_tdata <= s_axis_ddr_tdata;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        m_axis_load_data_tuser <= 3'b0;
    else if (load_data_in_fire)
        m_axis_load_data_tuser <= load_data_tag_mux;
end

assign m_axis_pso512_tdata = s_axis_ddr_tdata;

assign m_axis_pso512_tvalid =
    (data_state == FM && model_sel == `MODEL_SEL_EXPORT) ?
    s_axis_ddr_tvalid : 1'b0;

assign m_axis_pso512_tlast =
    (data_state == FM && model_sel == `MODEL_SEL_EXPORT) ?
    (data_beat_cnt == data_exp_beats_m1) : 1'b0;

axis_skid_buffer #(
    .DATA_WIDTH (`CMD2_CMD_WIDTH)
) u_dma_cmd_skid (
    .clk           (clk),
    .rst_n         (rst_n),
    .s_axis_tvalid (dma_cmd_raw_tvalid),
    .s_axis_tready (dma_cmd_raw_tready),
    .s_axis_tdata  (dma_cmd_raw_tdata),
    .m_axis_tvalid (m_axis_dma_cmd_tvalid),
    .m_axis_tready (m_axis_dma_cmd_tready),
    .m_axis_tdata  (m_axis_dma_cmd_tdata)
);

endmodule

// ============================================================
// RTL Self-Check Results
// ============================================================
// [PASS] Rule #1:  All ports have explicit width declarations
// [PASS] Rule #2:  New compute data output valid/data/user are registered regs
// [WARN] Legacy combinational output regs for DMA command and BA RAM controls
//        are inherited unchanged from the current load_core timing baseline
// [PASS] Rule #6:  No debug/error-attribution logic or diagnostic sideband in RTL
// [PASS] Rule #8:  No 'logic'/'always_ff'/'always_comb' used
// [PASS] Rule #12: All combinational branches covered (default present)
// [PASS] Rule #16: Blocking '=' for comb, non-blocking '<=' for seq
// [PASS] One-register-per-always-block: every reg has its own always block
// [PASS] Declaration-before-use: dma_cmd_fire/beat_fire/data_cmd_done/
//        active_tready/load_data_* wires declared before use
// [PASS] Compute data stream has no TLAST; destination is carried only by TUSER
// [PASS] Modified compute data path does not drive data/user/valid directly from
//        DDR input or data_state combinational logic to output ports
// [WARN] Rule #24: Multipliers retained only in PREP1/PREP2 staging
//                  (register-in register-out); constant multiplies are
//                  power-of-2 and map to wires; command-path variable
//                  multiplies replaced by synchronous stride accumulators
// ============================================================
