`include "all_defines.vh"

module activate(
    input  wire         clk,
    input  wire         rst_n,

    input  wire         rdcmd_trans_valid,
    input  wire [111:0] rdcmd_trans,

    input  wire         s_axis_act_tvalid,
    input  wire [511:0] s_axis_act_tdata,
    output wire         s_axis_act_tready,

    input  wire         s_axis_quan_act_tvalid,
    input  wire [511:0] s_axis_quan_act_tdata,
    output wire         s_axis_quan_act_tready,

    output wire         m_axis_act_tvalid,
    output wire [511:0] m_axis_act_tdata,
    input  wire         m_axis_act_tready
);

localparam [1:0] ACT_LUT_ROW_LAST   = 2'd3;

reg         cmd_act_en_r;
reg         cmd_out_type_r;
reg [1:0]   lut_wr_addr;
reg         pipe0_valid;
reg         pipe0_act_en;
reg         pipe0_out_type;
reg [255:0] pipe0_half_data;
reg [511:0] pipe0_full_data;
reg         pipe1_valid;
reg         pipe1_out_type;
reg [255:0] pipe1_half_data;
reg [511:0] pipe1_full_data;
reg [255:0] half_buf_data;
reg         half_buf_valid;
reg         out_word_valid_d;
reg [511:0] out_word_data_d;
reg         out_valid;
reg [511:0] out_data;

// ---- 指令过滤 / LUT 配置窗口 / 数据通道 ----
reg [5:0]   grp_cnt_r;
reg         cfg_win_r;
reg         data_open_r;

// ---- 输出无效列 mask 配置与计数 ----
reg [5:0]   out_valid_last_r;
reg [6:0]   out_col_last_r;
reg [13:0]  words_per_blk_r;
reg [13:0]  col_word_cnt_r;
reg [6:0]   col_blk_cnt_r;

wire [`RD_MODEL_SEL_WIDTH-1:0] cmd_model_sel;
wire                           cmd_is_conv;
wire                           cmd_layer_head;
wire                           cmd_act_en_next;
wire                           cmd_out_type_next;
wire                           cfg_done_fire;
wire                           act_lut_fire;
wire                           s_axis_quan_act_fire;
wire                           act_pipe_advance;
wire                           output_accept;
wire                           output_can_load;
wire                           output_skid_s_ready;
wire                           out_word_valid;
wire                           word_form_fire;
wire                           col_word_last;
wire                           col_blk_last;
wire [255:0]                   activated_half;
wire [511:0]                   out_word_data;
wire [511:0]                   out_word_data_masked;

wire [7:0]                     lane_in_byte       [0:31];
wire [7:0]                     lane_lut_addr      [0:31];
wire [1:0]                     lane_lut_rd_row    [0:31];
wire [5:0]                     lane_lut_rd_col    [0:31];
wire [511:0]                   lane_lut_rd_data   [0:31];
reg  [5:0]                     lane_lut_col_r     [0:31];

genvar gi;
genvar mk;

assign cmd_model_sel     = rdcmd_trans[`RD_MODEL_SEL_START +: `RD_MODEL_SEL_WIDTH];
assign cmd_is_conv       = (cmd_model_sel == `MODEL_SEL_CONV);
assign cmd_act_en_next   = cmd_is_conv & rdcmd_trans[`RD_CONV_OP_TYPE_START];
assign cmd_out_type_next = cmd_is_conv & rdcmd_trans[`RD_CONV_OUT_TYPE_START];

// CONV 多组广播只认首条（fm_channel_out = 组数-1，dispatch 每层发 field+1 条）
assign cmd_layer_head    = rdcmd_trans_valid && cmd_is_conv && (grp_cnt_r == 6'd0);
assign cfg_done_fire     = act_lut_fire && (lut_wr_addr == ACT_LUT_ROW_LAST);

assign act_lut_fire           = s_axis_act_tvalid & s_axis_act_tready;
assign output_accept          = out_valid & output_skid_s_ready;
assign output_can_load        = (~out_valid) | output_accept;
assign act_pipe_advance       = output_can_load;
assign s_axis_quan_act_tready = data_open_r & act_pipe_advance;
assign s_axis_quan_act_fire   = s_axis_quan_act_tvalid & s_axis_quan_act_tready;
assign out_word_valid         = pipe1_valid & (pipe1_out_type | half_buf_valid);

assign word_form_fire    = act_pipe_advance & out_word_valid;
assign col_word_last     = (col_word_cnt_r == (words_per_blk_r - 14'd1));
assign col_blk_last      = (col_blk_cnt_r == out_col_last_r);

assign s_axis_act_tready = cfg_win_r;

assign out_word_data = pipe1_out_type ? pipe1_full_data : {pipe1_half_data, half_buf_data};

generate
    for (gi = 0; gi < 32; gi = gi + 1) begin: act_lut_lane
        assign lane_in_byte[gi]    = s_axis_quan_act_tdata[(gi * 8) +: 8];
        assign lane_lut_addr[gi]   = lane_in_byte[gi];
        assign lane_lut_rd_row[gi] = lane_lut_addr[gi][7:6];
        assign lane_lut_rd_col[gi] = lane_lut_addr[gi][5:0];
        assign activated_half[(gi * 8) +: 8] =
            lane_lut_rd_data[gi][(lane_lut_col_r[gi] * 8) +: 8];

        sync_1r1w_ram #(
            .DATA_WIDTH (512),
            .ADDR_WIDTH (2),
            .DEPTH      (4)
        ) u_act_lut_ram (
            .clk     (clk),
            .wr_en   (act_lut_fire),
            .wr_addr (lut_wr_addr),
            .wr_data (s_axis_act_tdata),
            .rd_en   (s_axis_quan_act_fire & cmd_act_en_r & (~cmd_out_type_r)),
            .rd_addr (lane_lut_rd_row[gi]),
            .rd_data (lane_lut_rd_data[gi])
        );

        always @(posedge clk or negedge rst_n) begin
            if (!rst_n)
                lane_lut_col_r[gi] <= 6'd0;
            else if (s_axis_quan_act_fire & cmd_act_en_r & (~cmd_out_type_r))
                lane_lut_col_r[gi] <= lane_lut_rd_col[gi];
        end
    end
endgenerate

// ---- beat-level elastic output boundary ----
// The complete activation pipeline advances only when its registered output
// slot can move into this two-entry registered skid.  The skid source ready is
// derived only from local occupancy, so store-side ready cannot propagate
// combinationally into the LUT/half-pack pipeline.
axis_skid_buffer #(
    .DATA_WIDTH (512)
) u_activation_output_skid (
    .clk           (clk),
    .rst_n         (rst_n),
    .s_axis_tvalid (out_valid),
    .s_axis_tready (output_skid_s_ready),
    .s_axis_tdata  (out_data),
    .m_axis_tvalid (m_axis_act_tvalid),
    .m_axis_tready (m_axis_act_tready),
    .m_axis_tdata  (m_axis_act_tdata)
);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        grp_cnt_r <= 6'd0;
    else if (rdcmd_trans_valid && cmd_is_conv) begin
        if (grp_cnt_r == 6'd0)
            grp_cnt_r <= rdcmd_trans[`RD_FM_CHANNEL_OUT_START +: `RD_FM_CHANNEL_OUT_WIDTH];
        else
            grp_cnt_r <= grp_cnt_r - 6'd1;
    end
end

// ---- LUT 配置窗口：act_en 的 layer-head 打开，收满 4 拍（256B）即关 ----
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        cfg_win_r <= 1'b0;
    else if (cmd_layer_head && cmd_act_en_next)
        cfg_win_r <= 1'b1;
    else if (cfg_done_fire)
        cfg_win_r <= 1'b0;
end

// ---- 数据接收通道：act_en 待配置期间关闭；配置完成或 bypass 层打开；非 CONV 关闭 ----
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        data_open_r <= 1'b0;
    else if (cmd_layer_head)
        data_open_r <= ~cmd_act_en_next;
    else if (rdcmd_trans_valid && !cmd_is_conv)
        data_open_r <= 1'b0;
    else if (cfg_done_fire)
        data_open_r <= 1'b1;
end

// ---- 无效列 mask 配置（layer-head 锁存一次） ----
// out_valid_last：输出末组有效列(last_fm_out_column_valid+1)，直接使用编码值。
// 字段对 CONV 为输出语义（编译器按输出形状填写），输出流为 output 粒度
// （预取 window_out 隔点抽取密排），stride=1/2 均不折算。
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        out_valid_last_r <= 6'd32;
        out_col_last_r   <= 7'd0;
        words_per_blk_r  <= 14'd32;
    end
    else if (cmd_layer_head) begin
        out_valid_last_r <= {1'b0, rdcmd_trans[`RD_LAST_FM_OUT_COLUMN_VALID_START +: `RD_LAST_FM_OUT_COLUMN_VALID_WIDTH]} + 6'd1;
        out_col_last_r   <= rdcmd_trans[`RD_CONV_FM_OUT_COL_START +: `RD_CONV_FM_OUT_COL_WIDTH];
        // words_per_blk 按"每条广播的块宽"取定：dispatch 对多组 conv 逐组串行广播，
        // 每条广播的流为 64ch 组主序（(row,cb) 块内只含本组 64 通道），
        // 故块宽恒为 64ch×32col（int8 32 字 / int16 64 字），与 fm_channel_out 无关。
        // 旧式 (fm_channel_out+1)<<5/<<6 按"全层组间交织流"计宽，在组主序实流下
        // 会让尾块掩码周期性错落到 cb0（(组×行+行)%4≥2 处清零，tc6 实证）。
        if (cmd_out_type_next)
            words_per_blk_r <= 14'd64;
        else
            words_per_blk_r <= 14'd32;
    end
end

// ---- 输出列位置跟踪（形成输出 word 时推进；每条 conv 组广播复位） ----
// 组主序流下，列块计数必须按广播对齐：不复位时奇数块/行的组会让下一组错位。
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        col_word_cnt_r <= 14'd0;
    else if (rdcmd_trans_valid && cmd_is_conv)
        col_word_cnt_r <= 14'd0;
    else if (word_form_fire)
        col_word_cnt_r <= col_word_last ? 14'd0 : (col_word_cnt_r + 14'd1);
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        col_blk_cnt_r <= 7'd0;
    else if (rdcmd_trans_valid && cmd_is_conv)
        col_blk_cnt_r <= 7'd0;
    else if (word_form_fire && col_word_last)
        col_blk_cnt_r <= col_blk_last ? 7'd0 : (col_blk_cnt_r + 7'd1);
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        cmd_act_en_r <= 1'b0;
    else if (rdcmd_trans_valid)
        cmd_act_en_r <= cmd_act_en_next;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        cmd_out_type_r <= 1'b0;
    else if (rdcmd_trans_valid)
        cmd_out_type_r <= cmd_out_type_next;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        lut_wr_addr <= 2'd0;
    else if (cmd_layer_head && cmd_act_en_next)
        lut_wr_addr <= 2'd0;
    else if (act_lut_fire)
        lut_wr_addr <= (lut_wr_addr == ACT_LUT_ROW_LAST) ? 2'd0 : (lut_wr_addr + 2'd1);
end

// Data is accepted only on a real ready/valid handshake. When the output
// boundary stalls, every activation stage and the int8 half-pair buffer hold.
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        pipe0_valid <= 1'b0;
    else if (act_pipe_advance)
        pipe0_valid <= s_axis_quan_act_fire;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        pipe0_act_en <= 1'b0;
    else if (s_axis_quan_act_fire)
        pipe0_act_en <= cmd_act_en_r & (~cmd_out_type_r);
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        pipe0_out_type <= 1'b0;
    else if (s_axis_quan_act_fire)
        pipe0_out_type <= cmd_out_type_r;
end

always @(posedge clk) begin
    if (s_axis_quan_act_fire)
        pipe0_half_data <= s_axis_quan_act_tdata[255:0];
end

always @(posedge clk) begin
    if (s_axis_quan_act_fire)
        pipe0_full_data <= s_axis_quan_act_tdata;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        pipe1_valid <= 1'b0;
    else if (act_pipe_advance)
        pipe1_valid <= pipe0_valid;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        pipe1_out_type <= 1'b0;
    else if (act_pipe_advance)
        pipe1_out_type <= pipe0_out_type;
end

always @(posedge clk) begin
    if (act_pipe_advance & pipe0_valid)
        pipe1_half_data <= pipe0_act_en ? activated_half : pipe0_half_data;
end

always @(posedge clk) begin
    if (act_pipe_advance & pipe0_valid)
        pipe1_full_data <= pipe0_full_data;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        half_buf_valid <= 1'b0;
    else if (rdcmd_trans_valid)
        half_buf_valid <= 1'b0;
    else if (act_pipe_advance & pipe1_valid & (~pipe1_out_type)) begin
        if (half_buf_valid)
            half_buf_valid <= 1'b0;
        else
            half_buf_valid <= 1'b1;
    end
end

always @(posedge clk) begin
    if (act_pipe_advance & pipe1_valid & (~pipe1_out_type) & (~half_buf_valid))
        half_buf_data <= pipe1_half_data;
end

// ---- 输出无效列 mask：末组（col_blk_last）内超出 out_valid_last 的元素清 0 ----
// int8：32 字节列×双半区，按 8bit 粒度；int16：32 个 16bit 元素，按 16bit 粒度
generate
    for (mk = 0; mk < 32; mk = mk + 1) begin : gen_out_col_mask
        assign out_word_data_masked[(mk*8) +: 8] =
            (col_blk_last && (pipe1_out_type ?
                ({8'd0, out_valid_last_r} <= {8'd0, mk[5:1]}) :
                ({8'd0, out_valid_last_r} <= {8'd0, mk[4:0]}))) ? 8'd0 :
            out_word_data[(mk*8) +: 8];
        assign out_word_data_masked[256+(mk*8) +: 8] =
            (col_blk_last && (pipe1_out_type ?
                ({8'd0, out_valid_last_r} <= (6'd16 + {2'd0, mk[5:1]})) :
                ({8'd0, out_valid_last_r} <= {8'd0, mk[4:0]}))) ? 8'd0 :
            out_word_data[256+(mk*8) +: 8];
    end
endgenerate

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        out_word_valid_d <= 1'b0;
    else if (act_pipe_advance)
        out_word_valid_d <= out_word_valid;
end

always @(posedge clk) begin
    if (act_pipe_advance & out_word_valid)
        out_word_data_d <= out_word_data_masked;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        out_valid <= 1'b0;
    else if (act_pipe_advance)
        out_valid <= out_word_valid_d;
end

always @(posedge clk) begin
    if (act_pipe_advance & out_word_valid_d)
        out_data <= out_word_data_d;
end

// ============================================================
// RTL Self-Check Results
// ============================================================
// [PASS] Ports have explicit direction and width declarations.
// [PASS] Pure Verilog RTL only; no SystemVerilog keywords used.
// [PASS] Clocked logic uses nonblocking assignments.
// [PASS] 32 parallel activation lanes each instantiate one standard 1R1W RAM.
// [PASS] LUT writes use 4 packed 512-bit rows; LUT reads are synchronous.
// [PASS] Back-to-back LUT reads align each lane's RAM row data with its delayed column index.
// [PASS] LUT addresses use raw 8-bit quantized values; software owns offline table ordering.
// [PASS] int8 activation/bypass packs two consecutive 256-bit channel halves into 512 bits.
// [PASS] int16 bypass forwards the full 512-bit quantized beat.
// [PASS] LUT config window opens only on CONV layer-head with act_en and closes after
//        4 beats; data channel opens on config done or bypass layers (2026-07-18 change).
// [PASS] Output invalid-column mask zeroes elements beyond out_valid_last in the last
//        out_col_blk (int8 byte / int16 halfword granular); field is output-semantic
//        for CONV, used directly for stride 1/2 (no stride rescale), 2026-07-20 fix;
//        column tracking resets on every conv group broadcast and block width
//        is fixed at one 64ch group (group-major stream), 2026-07-20 fix.
// [PASS] No debug-only ports, system tasks, initial blocks, or delays.
// [PASS] AXIS input ready is beat-level; cal-top owns the 128-deep boundary FIFO.
// [PASS] One local advance condition holds data, valid, LUT metadata and half-pair state together.
// [PASS] Two-entry registered output skid replaces the redundant internal deep FIFO.
// [PASS] No-backpressure path accepts one quant beat per cycle; no periodic bubble or wait state.
// ============================================================

endmodule
