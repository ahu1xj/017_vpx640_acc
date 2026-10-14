// prefetch_fm CONV 预取 v2：五级累加器 token 流水。
// stage1 锚点 -> fifo1 -> stage2 通道对展开 -> fifo2 -> stage3 kh 展开 -> fifo3
//   -> stage4 slot 展开 -> fifo4 -> 取数+70B 拼接 -> fifo_win -> 取窗输出。
// 全部运行期乘法消除：stride 只支持 1/2，步长为配置期二选一；
// 逐拍路径只有加法、比较和选择。规格见 docs/specs/cal_top/fram/prefetch_fm_rtl_spec.md。
module prefetch_fm_conv(
    input           clk,
    input           rst_n,

    input           conv_start,
    input   [11:0]  cfg_fm_channel_in,
    input   [11:0]  cfg_fm_row_in,
    input   [6:0]   cfg_fm_column_in,
    input   [11:0]  cfg_cin_pair_num,
    input   [7:0]   cfg_input_col_blk_num,
    input   [2:0]   cfg_kernel_size_r,
    input   [2:0]   cfg_kernel_size_c,
    input   [2:0]   cfg_stride_r_real,
    input   [2:0]   cfg_stride_c_real,
    input   [1:0]   cfg_padding_top,
    input   [1:0]   cfg_padding_left,
    input   [11:0]  cfg_fm_out_row,
    input   [6:0]   cfg_fm_out_col,
    input   [31:0]  cfg_line_size,
    input   [31:0]  cfg_ring_size,
    input   [2:0]   cfg_byte_start,

    input           fm_add_sig,
    output          fm_sub_sig,

    output          frden,
    output  [13:0]  frdaddr,
    input           frdvalid,
    input   [511:0] frddata,

    output          m_axis_ffmtem_tvalid,
    output  [255:0] m_axis_ffmtem_tdata,
    input           m_axis_ffmtem_tready,

    output          conv_done
);

// --------------------------------------------------------
// 配置期派生常量：乘法寄存器进寄存器出，中间不串组合逻辑。
// c0(conv_start)      : col_step_reg / e_max_reg
// c1(conv_start_d1)   : row_step_reg / pad_top_line_reg（cfg_line_size 已寄存）
// c2(conv_start_d2)   : row_base_init_reg（cfg_ring_size / pad_top_line 已寄存）
// stride 只支持 1/2，步长为配置期二选一；slot 参数由 e_max_reg 比较派生。
// --------------------------------------------------------
reg                conv_start_d1_reg;
reg                conv_start_d2_reg;
reg [31:0]         col_step_reg;
reg [6:0]          e_max_reg;
reg [31:0]         row_step_reg;
reg [31:0]         pad_top_line_reg;
reg [31:0]         row_base_init_reg;

wire signed [15:0] origin_row_init;
wire [1:0]         slot_start;
wire [1:0]         slot_last;
wire [2:0]         slot_num;
wire [11:0]        win_init_off;

// --------------------------------------------------------
// 级间 token / FIFO 握手
// --------------------------------------------------------
wire               t1_full;
wire               t1_empty;
wire               t1_wr_en;
wire [58:0]        t1_wr_data;
wire               t1_rd_en;
wire [58:0]        t1_rd_data;

wire               t2_full;
wire               t2_empty;
wire               t2_wr_en;
wire [59:0]        t2_wr_data;
wire               t2_rd_en;
wire [59:0]        t2_rd_data;

wire               t3_full;
wire               t3_empty;
wire               t3_wr_en;
wire [43:0]        t3_wr_data;
wire               t3_rd_en;
wire [43:0]        t3_rd_data;

wire               t4_full;
wire               t4_empty;
wire               t4_wr_en;
wire [34:0]        t4_wr_data;
wire               t4_rd_en;
wire [34:0]        t4_rd_data;

wire               win_full;
wire               win_empty;
wire               win_wr_en;
wire [559:0]       win_wr_data;
wire               win_rd_en;
wire [559:0]       win_rd_data;

wire               asm_busy;
wire               win_busy;
wire               window_raw_valid;
wire [255:0]        window_raw_data;
wire               window_raw_ready;

// --------------------------------------------------------
// conv_done 跟踪：最后一个 out_row 的 fm_sub_sig 发出、
// 且拼接/输出全部排空后拉高，保持到下一次 conv_start。
// --------------------------------------------------------
reg [11:0]         sub_cnt_reg;
reg                conv_done_reg;

wire               done_cond;

assign origin_row_init = -$signed({14'd0, cfg_padding_top});

assign slot_start = (cfg_padding_left == 2'd0) ? 2'd1 : 2'd0;
assign slot_last  = 2'd1 + ((e_max_reg >= 7'd35) ? 2'd1 : 2'd0)
                         + ((e_max_reg >= 7'd67) ? 2'd1 : 2'd0);
assign slot_num   = {1'b0, slot_last} - {1'b0, slot_start} + 3'd1;
assign win_init_off = (slot_start == 2'd1) ? cfg_cin_pair_num : 12'd0;

// done_cond 必须排除 sub_cnt==0 的初始态：复位后 cmd_reg 未装载、
// cfg_fm_out_row 读为 0，0==0 会假置位 conv_done，导致 owner 被提前释放。
assign done_cond = (sub_cnt_reg == cfg_fm_out_row) && (sub_cnt_reg != 12'd0)
                 && !asm_busy && !win_busy && win_empty && !m_axis_ffmtem_tvalid;

assign conv_done = conv_done_reg;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        conv_start_d1_reg <= 1'b0;
    else
        conv_start_d1_reg <= conv_start;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        conv_start_d2_reg <= 1'b0;
    else
        conv_start_d2_reg <= conv_start_d1_reg;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        col_step_reg <= 32'd0;
    else if (conv_start)
        col_step_reg <= cfg_stride_c_real[1] ? {19'd0, cfg_cin_pair_num, 1'b0}
                                             : {20'd0, cfg_cin_pair_num};
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        e_max_reg <= 7'd0;
    else if (conv_start)
        e_max_reg <= {4'd0, cfg_byte_start} + {4'd0, (cfg_kernel_size_c - 3'd1)}
                   + (cfg_stride_c_real[1] ? 7'd62 : 7'd31);
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        row_step_reg <= 32'd0;
    else if (conv_start_d1_reg)
        row_step_reg <= cfg_stride_r_real[1] ? {cfg_line_size[30:0], 1'b0}
                                             : cfg_line_size;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        pad_top_line_reg <= 32'd0;
    else if (conv_start_d1_reg)
        pad_top_line_reg <= (cfg_padding_top[0] ? cfg_line_size : 32'd0)
                          + (cfg_padding_top[1] ? {cfg_line_size[30:0], 1'b0} : 32'd0);
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        row_base_init_reg <= 32'd0;
    else if (conv_start_d2_reg)
        row_base_init_reg <= (cfg_padding_top == 2'd0) ? 32'd0
                                                       : (cfg_ring_size - pad_top_line_reg);
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        sub_cnt_reg <= 12'd0;
    else if (conv_start)
        sub_cnt_reg <= 12'd0;
    else if (fm_sub_sig)
        sub_cnt_reg <= sub_cnt_reg + 12'd1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        conv_done_reg <= 1'b0;
    else if (conv_start)
        conv_done_reg <= 1'b0;
    else if (done_cond)
        conv_done_reg <= 1'b1;
end

prefetch_fm_conv_stage1_anchor u_stage1 (
    .clk                 (clk),
    .rst_n               (rst_n),
    .conv_start          (conv_start),
    .fm_add_sig          (fm_add_sig),
    .cfg_fm_out_row      (cfg_fm_out_row),
    .cfg_fm_out_col      (cfg_fm_out_col),
    .cfg_stride_r_real   (cfg_stride_r_real),
    .cfg_stride_c_real   (cfg_stride_c_real),
    .cfg_cin_pair_num    (cfg_cin_pair_num),
    .cfg_ring_size       (cfg_ring_size),
    .cfg_row_step        (row_step_reg),
    .cfg_col_step        (col_step_reg),
    .cfg_row_base_init   (row_base_init_reg),
    .cfg_origin_row_init (origin_row_init),
    .t1_full             (t1_full),
    .t1_wr_en            (t1_wr_en),
    .t1_wr_data          (t1_wr_data)
);

prefetch_fm_sync_fifo #(
    .DATA_WIDTH (59),
    .DEPTH      (2)
) u_fifo1 (
    .clk     (clk),
    .rst_n   (rst_n),
    .wr_en   (t1_wr_en),
    .wr_data (t1_wr_data),
    .full    (t1_full),
    .rd_en   (t1_rd_en),
    .rd_data (t1_rd_data),
    .empty   (t1_empty)
);

prefetch_fm_conv_stage2_cin u_stage2 (
    .clk              (clk),
    .rst_n            (rst_n),
    .cfg_cin_pair_num (cfg_cin_pair_num),
    .t1_empty         (t1_empty),
    .t1_rd_data       (t1_rd_data),
    .t1_rd_en         (t1_rd_en),
    .t2_full          (t2_full),
    .t2_wr_en         (t2_wr_en),
    .t2_wr_data       (t2_wr_data)
);

prefetch_fm_sync_fifo #(
    .DATA_WIDTH (60),
    .DEPTH      (2)
) u_fifo2 (
    .clk     (clk),
    .rst_n   (rst_n),
    .wr_en   (t2_wr_en),
    .wr_data (t2_wr_data),
    .full    (t2_full),
    .rd_en   (t2_rd_en),
    .rd_data (t2_rd_data),
    .empty   (t2_empty)
);

prefetch_fm_conv_stage3_kh u_stage3 (
    .clk               (clk),
    .rst_n             (rst_n),
    .cfg_kernel_size_r (cfg_kernel_size_r),
    .cfg_fm_row_in     (cfg_fm_row_in),
    .cfg_line_size     (cfg_line_size),
    .cfg_ring_size     (cfg_ring_size),
    .t2_empty          (t2_empty),
    .t2_rd_data        (t2_rd_data),
    .t2_rd_en          (t2_rd_en),
    .t3_full           (t3_full),
    .t3_wr_en          (t3_wr_en),
    .t3_wr_data        (t3_wr_data)
);

prefetch_fm_sync_fifo #(
    .DATA_WIDTH (44),
    .DEPTH      (2)
) u_fifo3 (
    .clk     (clk),
    .rst_n   (rst_n),
    .wr_en   (t3_wr_en),
    .wr_data (t3_wr_data),
    .full    (t3_full),
    .rd_en   (t3_rd_en),
    .rd_data (t3_rd_data),
    .empty   (t3_empty)
);

prefetch_fm_conv_stage4_slot u_stage4 (
    .clk                   (clk),
    .rst_n                 (rst_n),
    .cfg_cin_pair_num      (cfg_cin_pair_num),
    .cfg_input_col_blk_num (cfg_input_col_blk_num),
    .cfg_ring_size         (cfg_ring_size),
    .cfg_slot_num          (slot_num),
    .cfg_slot_start        (slot_start),
    .cfg_win_init_off      (win_init_off),
    .t3_empty              (t3_empty),
    .t3_rd_data            (t3_rd_data),
    .t3_rd_en              (t3_rd_en),
    .t4_full               (t4_full),
    .t4_wr_en              (t4_wr_en),
    .t4_wr_data            (t4_wr_data)
);

prefetch_fm_sync_fifo #(
    .DATA_WIDTH (35),
    .DEPTH      (2)
) u_fifo4 (
    .clk     (clk),
    .rst_n   (rst_n),
    .wr_en   (t4_wr_en),
    .wr_data (t4_wr_data),
    .full    (t4_full),
    .rd_en   (t4_rd_en),
    .rd_data (t4_rd_data),
    .empty   (t4_empty)
);

prefetch_fm_conv_read_asm u_read_asm (
    .clk             (clk),
    .rst_n           (rst_n),
    .cfg_slot_num    (slot_num),
    .cfg_slot_start  (slot_start),
    .cfg_slot_last   (slot_last),
    .t4_empty        (t4_empty),
    .t4_rd_data      (t4_rd_data),
    .t4_rd_en        (t4_rd_en),
    .frden           (frden),
    .frdaddr         (frdaddr),
    .frdvalid        (frdvalid),
    .frddata         (frddata),
    .win_full        (win_full),
    .win_wr_en       (win_wr_en),
    .win_wr_data     (win_wr_data),
    .fm_sub_sig      (fm_sub_sig),
    .asm_busy        (asm_busy)
);

prefetch_fm_sync_fifo #(
    .DATA_WIDTH (560),
    .DEPTH      (2)
) u_fifo_win (
    .clk     (clk),
    .rst_n   (rst_n),
    .wr_en   (win_wr_en),
    .wr_data (win_wr_data),
    .full    (win_full),
    .rd_en   (win_rd_en),
    .rd_data (win_rd_data),
    .empty   (win_empty)
);

prefetch_fm_conv_window_out u_window_out (
    .clk                 (clk),
    .rst_n               (rst_n),
    .cfg_kernel_size_c   (cfg_kernel_size_c),
    .cfg_byte_start      (cfg_byte_start),
    .cfg_stride_c_real   (cfg_stride_c_real),
    .win_empty           (win_empty),
    .win_rd_data         (win_rd_data),
    .win_rd_en           (win_rd_en),
    .m_axis_ffmtem_tvalid(window_raw_valid),
    .m_axis_ffmtem_tdata (window_raw_data),
    .m_axis_ffmtem_tready(window_raw_ready),
    .win_busy            (win_busy)
);

axis_skid_buffer #(.DATA_WIDTH(256)) u_ffmtem_output_skid (
    .clk           (clk),
    .rst_n         (rst_n),
    .s_axis_tvalid (window_raw_valid),
    .s_axis_tready (window_raw_ready),
    .s_axis_tdata  (window_raw_data),
    .m_axis_tvalid (m_axis_ffmtem_tvalid),
    .m_axis_tready (m_axis_ffmtem_tready),
    .m_axis_tdata  (m_axis_ffmtem_tdata)
);

endmodule

// ============================================================
// RTL Self-Check Results
// ============================================================
// [PASS] Rule #1:  All ports have explicit width declarations
// [PASS] Rule #2:  Ports use Verilog wire/reg conventions
// [PASS] Rule #6:  No debug/error-attribution logic or diagnostic-only sideband in RTL
// [PASS] Rule #8:  No logic/always_ff/always_comb used
// [PASS] Rule #12: All conditional assigns have both branches
// [PASS] Rule #16: Non-blocking '<=' for all sequential logic
// [PASS] Rule #24: No multipliers; config derivations are shift/select/add only
// [PASS] Rule #34: prefetch_fm_conv external port list preserved (top unchanged)
// [PASS] All submodule instantiations use named ports with explicit FIFO params
// [PASS] Output serializer holds valid/data under backpressure and feeds a local skid buffer
// ============================================================
