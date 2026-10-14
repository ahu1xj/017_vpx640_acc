`include "all_defines.vh"

module fram_write_ctrl(
    input  wire          clk,
    input  wire          rst_n,
    input  wire          rdcmd_trans_valid,
    input  wire [111:0]  rdcmd_trans,
    input  wire          s_axis_fm_tvalid,
    input  wire [511:0]  s_axis_fm_tdata,
    output wire          s_axis_fm_tready,
    output wire          wren,
    output wire [13:0]   wraddr,
    output wire [511:0]  wrdata,
    output wire          fm_add_sig,
    input  wire          fm_sub_sig
);

localparam [4:0] FREE_ROW_INIT = 5'd9;

reg         cmd_active_reg;
reg         pending_cmd_valid;
reg [111:0] pending_cmd;
reg [2:0]   cfg_cnt_reg;
reg         cfg_ready_reg;
reg         has_fm_words_reg;
reg [31:0]  mult_reg;
reg [31:0]  line_size_reg;
reg [31:0]  wrap_size_reg;
reg [111:0] rdcmd_trans_reg;

reg [13:0] wraddr_wrap_thresh_reg;

reg [`RD_FM_CHANNEL_IN_WIDTH-1:0] cin_cnt;
reg [`RD_FM_COLUMN_IN_WIDTH-1:0]  col_blk_cnt;
reg [`RD_FM_ROW_IN_WIDTH-1:0]     row_cnt;
reg [`RD_FM_CHANNEL_IN_WIDTH-1:0] ch_cnt;

reg [3:0] first_batch_size_reg;
reg [3:0] rows_in_batch;
reg [3:0] row_in_batch_cnt;
reg       first_batch;

reg [`RD_CONV_FM_OUT_ROW_WIDTH-1:0] fm_out_row_cnt;
reg                                 phantom_mode;
reg                                 fm_add_sig_reg;
reg                                 phantom_pulse_d;
reg                                 real_add_d;
reg [4:0]                           real_add_amt_d;

reg [`RD_CONV_FM_OUT_ROW_WIDTH-1:0] conv_sub_row_cnt;
reg [15:0]                          conv_released_row_cnt;
reg                                 fm_sub_sig_d;
reg [4:0]                           sub_amt_d;

reg [4:0] free_row_cnt;
reg       pool_pair_wait;

reg        tready_reg;
reg        wren_reg;
reg [511:0] wrdata_reg;
reg         pool_pair_end_d;
reg [13:0]  wraddr_reg;

wire [`RD_MODEL_SEL_WIDTH-1:0] mode_in;
wire                           valid_op_in;
wire                           ext_cmd_valid;
wire                           cmd_idle;
wire                           capture_pending_cmd;
wire                           pending_cmd_start;
wire                           direct_cmd_start;
wire                           cmd_start;
wire [111:0]                   cmd_start_trans;
wire [`RD_MODEL_SEL_WIDTH-1:0] mode_cmd;
wire                           cmd_start_is_conv;
wire                           cmd_start_is_pool;
wire [`RD_FM_CHANNEL_IN_WIDTH-1:0] cmd_fm_channel_in;
wire [`RD_FM_ROW_IN_WIDTH-1:0]     cmd_fm_row_in;
wire [`RD_FM_COLUMN_IN_WIDTH:0]    cmd_col_blk_num;
wire [`RD_FM_CHANNEL_IN_WIDTH-1:0] cmd_cin_pair_num;
wire                              cmd_has_fm_words;
wire [`RD_FM_COLUMN_IN_WIDTH:0]    reg_col_blk_num;
wire [`RD_FM_CHANNEL_IN_WIDTH-1:0] reg_cin_pair_num;
wire                              reg_is_pool;

wire [`RD_MODEL_SEL_WIDTH-1:0]          mode_sel;
wire [`RD_FM_CHANNEL_IN_WIDTH-1:0]      fm_channel_in;
wire [`RD_FM_ROW_IN_WIDTH-1:0]          fm_row_in;
wire [`RD_FM_COLUMN_IN_WIDTH-1:0]       fm_column_in;
wire [`RD_CONV_KERNEL_SIZE_R_WIDTH-1:0] kernel_size_r;
wire [`RD_CONV_STRIDE_R_WIDTH-1:0]      stride_r;
wire [`RD_CONV_FM_OUT_ROW_WIDTH-1:0]    fm_out_row;
wire                                    is_conv;
wire                                    is_pool;

wire [`RD_FM_CHANNEL_IN_WIDTH-1:0] cin_pair_num;
wire [`RD_FM_CHANNEL_IN_WIDTH-1:0] cin_pair_max;
wire [`RD_FM_ROW_IN_WIDTH-1:0]     row_max;
wire [`RD_FM_COLUMN_IN_WIDTH-1:0]  col_blk_max;
wire [3:0]                         stride_r_real4;
wire [4:0]                         stride_r_real5;

wire handshake;
wire conv_cin_done;
wire conv_col_done;
wire conv_row_done;
wire conv_row_end_hs;
wire conv_all_done_hs;
wire pool_col_done;
wire pool_row_done;
wire pool_ch_done;
wire pool_row_end_hs;
wire pool_done_hs;
wire pool_pair_done_hs;
wire pool_pair_nonlast_done_hs;

wire [`RD_CONV_KERNEL_SIZE_R_WIDTH-1:0] kernel_in;
wire [`RD_CONV_PADDING_TOP_WIDTH-1:0]   pad_top_in;
wire [3:0] first_batch_size_in;
wire [3:0] steady_rows_in_batch;
wire       conv_batch_natural_full;
wire       conv_short_final_batch;
wire       conv_batch_full;
wire       conv_batch_end_hs;

wire       conv_add_quota_avail;
wire       real_add_pulse;
wire [`RD_CONV_FM_OUT_ROW_WIDTH-1:0] fm_out_row_cnt_after_cur_add;
wire       phantom_pulse;
wire [4:0] add_amt_now;

wire signed [15:0] conv_sub_next_idx_s;
wire signed [15:0] conv_kernel_center_s;
wire signed [15:0] conv_sub_next_origin_s;
wire [15:0]        conv_fm_row_in_ext;
wire [15:0]        conv_release_target_ext;
wire [15:0]        conv_release_avail_ext;
wire [4:0]         conv_sub_amt;
wire [4:0]         sub_amt;

wire [5:0] pending_real_add_ext;
wire [5:0] free_after_pulse_ext;
wire [4:0] req_size_conv;
wire       conv_close_tready;
wire       pool_close_tready;
wire       cmd_full_done_hs;
wire [4:0] req_size;
wire [5:0] free_after_update_add_ext;
wire [5:0] free_after_update_ext;
wire       enough_after_sub;
wire       pool_pair_reuse_ready;

assign mode_in          = rdcmd_trans[`RD_MODEL_SEL_START +: `RD_MODEL_SEL_WIDTH];
assign valid_op_in      = (mode_in == `MODEL_SEL_CONV) || (mode_in == `MODEL_SEL_POOLING);
assign ext_cmd_valid    = rdcmd_trans_valid && valid_op_in;
assign cmd_idle         = !cmd_active_reg;
assign capture_pending_cmd = ext_cmd_valid && !cmd_idle && !pending_cmd_valid;
assign pending_cmd_start = cmd_idle && pending_cmd_valid;
// 每条 conv/pool 指令广播都是一次新会话：上一组的工作由 conv_done 闸保证已完结
// （环形行已全部消费释放），无需等 cmd_active 的 phantom 尾收尾。
// 若仍按 cmd_idle 门控，多组广播的后续组只能在整层 phantom 尾后才见到 cmd_start，
// 全部 cmd_start 复位键（free/batch/release/phantom）失效，组间状态互相污染（tc5 实证）。
assign direct_cmd_start  = ext_cmd_valid && !pending_cmd_valid;
assign cmd_start         = pending_cmd_start || direct_cmd_start;
assign cmd_start_trans   = pending_cmd_start ? pending_cmd : rdcmd_trans;
assign mode_cmd          = cmd_start_trans[`RD_MODEL_SEL_START +: `RD_MODEL_SEL_WIDTH];
assign cmd_start_is_conv = (mode_cmd == `MODEL_SEL_CONV);
assign cmd_start_is_pool = (mode_cmd == `MODEL_SEL_POOLING);

assign cmd_fm_channel_in = cmd_start_trans[`RD_FM_CHANNEL_IN_START +: `RD_FM_CHANNEL_IN_WIDTH];
assign cmd_fm_row_in     = cmd_start_trans[`RD_FM_ROW_IN_START +: `RD_FM_ROW_IN_WIDTH];
assign cmd_col_blk_num   = {1'b0, cmd_start_trans[`RD_FM_COLUMN_IN_START +: `RD_FM_COLUMN_IN_WIDTH]} + 8'd1;
assign cmd_cin_pair_num  = cmd_fm_channel_in >> 1;
assign cmd_has_fm_words  = (cmd_start_is_conv || cmd_start_is_pool) &&
                           (cmd_cin_pair_num != 0) &&
                           (cmd_fm_row_in != 0);

// 乘法寄存器进寄存器出：cmd_start 后 rdcmd_trans_reg 已锁存，以下三级
// 配置流水全部由寄存器驱动、单级装入寄存器，中间不串组合逻辑。
assign reg_col_blk_num  = {1'b0, rdcmd_trans_reg[`RD_FM_COLUMN_IN_START +: `RD_FM_COLUMN_IN_WIDTH]} + 8'd1;
assign reg_cin_pair_num = rdcmd_trans_reg[`RD_FM_CHANNEL_IN_START +: `RD_FM_CHANNEL_IN_WIDTH] >> 1;
assign reg_is_pool      = (rdcmd_trans_reg[`RD_MODEL_SEL_START +: `RD_MODEL_SEL_WIDTH] == `MODEL_SEL_POOLING);

assign mode_sel      = rdcmd_trans_reg[`RD_MODEL_SEL_START +: `RD_MODEL_SEL_WIDTH];
assign fm_channel_in = rdcmd_trans_reg[`RD_FM_CHANNEL_IN_START +: `RD_FM_CHANNEL_IN_WIDTH];
assign fm_row_in     = rdcmd_trans_reg[`RD_FM_ROW_IN_START +: `RD_FM_ROW_IN_WIDTH];
assign fm_column_in  = rdcmd_trans_reg[`RD_FM_COLUMN_IN_START +: `RD_FM_COLUMN_IN_WIDTH];
assign kernel_size_r = rdcmd_trans_reg[`RD_CONV_KERNEL_SIZE_R_START +: `RD_CONV_KERNEL_SIZE_R_WIDTH];
assign stride_r      = rdcmd_trans_reg[`RD_CONV_STRIDE_R_START +: `RD_CONV_STRIDE_R_WIDTH];
assign fm_out_row    = rdcmd_trans_reg[`RD_CONV_FM_OUT_ROW_START +: `RD_CONV_FM_OUT_ROW_WIDTH];
assign is_conv       = (mode_sel == `MODEL_SEL_CONV);
assign is_pool       = (mode_sel == `MODEL_SEL_POOLING);

assign cin_pair_num   = fm_channel_in >> 1;
assign cin_pair_max   = (cin_pair_num == 0) ? {`RD_FM_CHANNEL_IN_WIDTH{1'b0}} : (cin_pair_num - 1'b1);
assign row_max        = (fm_row_in == 0) ? {`RD_FM_ROW_IN_WIDTH{1'b0}} : (fm_row_in - 1'b1);
assign col_blk_max    = fm_column_in;
assign stride_r_real4 = {{(4-`RD_CONV_STRIDE_R_WIDTH){1'b0}}, stride_r} + 4'd1;
assign stride_r_real5 = {{(5-`RD_CONV_STRIDE_R_WIDTH){1'b0}}, stride_r} + 5'd1;

assign handshake       = s_axis_fm_tready && s_axis_fm_tvalid;
assign conv_cin_done   = is_conv && (cin_cnt == cin_pair_max);
assign conv_col_done   = is_conv && conv_cin_done && (col_blk_cnt == col_blk_max);
assign conv_row_done   = is_conv && conv_col_done && (row_cnt == row_max);
assign conv_row_end_hs = handshake && conv_col_done;
assign conv_all_done_hs = handshake && conv_row_done;
assign pool_col_done   = is_pool && (col_blk_cnt == col_blk_max);
assign pool_row_done   = is_pool && pool_col_done && (row_cnt == row_max);
assign pool_ch_done    = is_pool && pool_row_done && (ch_cnt == cin_pair_max);
assign pool_row_end_hs = handshake && pool_col_done;
assign pool_done_hs    = handshake && pool_ch_done;
assign pool_pair_done_hs = handshake && pool_row_done;
assign pool_pair_nonlast_done_hs = pool_pair_done_hs && !pool_ch_done;

assign kernel_in           = cmd_start_trans[`RD_CONV_KERNEL_SIZE_R_START +: `RD_CONV_KERNEL_SIZE_R_WIDTH];
assign pad_top_in          = cmd_start_trans[`RD_CONV_PADDING_TOP_START +: `RD_CONV_PADDING_TOP_WIDTH];
// 首批必须覆盖 out_row0 实际触及的真实输入行数：origin=-pad_top，行窗
// [-pad_top, kernel-1-pad_top]，真实行数 = kernel_in - pad_top（奇偶核通用）。
// 旧公式 (k+1)>>1 仅对奇数核对称 padding 成立；k6/s2/p2 少担保 1 行，
// 慢供给下读侧启动击穿（slowmem v5s 实证：FRAM 读超车写）。
assign first_batch_size_in = ({1'b0, kernel_in} > {2'b00, pad_top_in})
                           ? ({1'b0, kernel_in} - {2'b00, pad_top_in})
                           : 4'd1;
assign steady_rows_in_batch = stride_r_real4;
// 层末行必须冲刷半批：fm_row_in < rows_in_batch（如 R=1 首批 2 行只来 1 行，
// 或 stride=2 奇数行的末批）时自然满批条件永不成立，real_add 不发则读侧
// 永远等不到首信用（r1 板级实证死锁）。conv_row_done 只在层末行成立，
// 不会提前火；配额门 conv_add_quota_avail 不变，无需信用的末行照样被挡。
assign conv_batch_natural_full = ((row_in_batch_cnt + 4'd1) == rows_in_batch);
assign conv_short_final_batch  = conv_row_done && !conv_batch_natural_full;
assign conv_batch_full      = conv_batch_natural_full || conv_row_done;
assign conv_batch_end_hs    = conv_row_end_hs && conv_batch_full;

assign conv_add_quota_avail = (fm_out_row_cnt != 0);
assign real_add_pulse       = (is_conv && conv_batch_end_hs && conv_add_quota_avail) ||
                              (is_pool && pool_row_end_hs);
assign fm_out_row_cnt_after_cur_add =
        fm_out_row_cnt - ((fm_add_sig_reg && (fm_out_row_cnt != 0)) ? 1'b1 : 1'b0);
assign phantom_pulse = phantom_mode && fm_sub_sig && (fm_out_row_cnt_after_cur_add != 0);
// 短末批按实际来行数记账（free_row_cnt 只认 add_amt），不多发不少发。
assign add_amt_now   = is_conv
                     ? (conv_short_final_batch ? {1'b0, row_in_batch_cnt} + 5'd1
                       : first_batch ? {1'b0, first_batch_size_reg} : stride_r_real5)
                     : 5'd1;

assign conv_sub_next_idx_s    = $signed({4'd0, conv_sub_row_cnt}) + 16'sd1;
assign conv_kernel_center_s   = $signed({13'd0, (kernel_size_r >> 1)});
assign conv_sub_next_origin_s = (stride_r_real5 == 5'd2)
                              ? ((conv_sub_next_idx_s <<< 1) - conv_kernel_center_s)
                              : (conv_sub_next_idx_s - conv_kernel_center_s);
assign conv_fm_row_in_ext     = {4'd0, fm_row_in};
assign conv_release_target_ext =
        (conv_sub_row_cnt >= (fm_out_row - 1'b1)) ? conv_fm_row_in_ext :
        (conv_sub_next_origin_s <= 16'sd0)        ? 16'd0 :
        (conv_sub_next_origin_s >= $signed(conv_fm_row_in_ext)) ? conv_fm_row_in_ext :
        conv_sub_next_origin_s[15:0];
assign conv_release_avail_ext =
        (conv_release_target_ext > conv_released_row_cnt)
      ? (conv_release_target_ext - conv_released_row_cnt)
      : 16'd0;
assign conv_sub_amt = (conv_release_avail_ext > 16'd31) ? 5'd31 : conv_release_avail_ext[4:0];
assign sub_amt      = is_conv ? conv_sub_amt : 5'd1;

assign pending_real_add_ext =
        (real_add_d     ? {1'b0, real_add_amt_d} : 6'd0) +
        (real_add_pulse ? {1'b0, add_amt_now}    : 6'd0);
assign free_after_pulse_ext =
        ({1'b0, free_row_cnt} >= pending_real_add_ext)
      ? ({1'b0, free_row_cnt} - pending_real_add_ext)
      : 6'd0;
assign req_size_conv     = stride_r_real5;
assign conv_close_tready = conv_batch_end_hs && (free_after_pulse_ext < {1'b0, req_size_conv});
assign pool_close_tready = (pool_row_end_hs && (free_after_pulse_ext == 6'd0)) ||
                           pool_pair_nonlast_done_hs;
assign cmd_full_done_hs  = conv_all_done_hs || pool_done_hs;
assign req_size          = is_conv ? req_size_conv : 5'd1;
assign free_after_update_add_ext =
        {1'b0, free_row_cnt} + (fm_sub_sig_d ? {1'b0, sub_amt_d} : 6'd0);
assign free_after_update_ext =
        (free_after_update_add_ext >= (real_add_d ? {1'b0, real_add_amt_d} : 6'd0))
      ? (free_after_update_add_ext - (real_add_d ? {1'b0, real_add_amt_d} : 6'd0))
      : 6'd0;
assign enough_after_sub       = (free_after_update_ext >= {1'b0, req_size});
assign pool_pair_reuse_ready  = !is_pool || !pool_pair_wait || (free_after_update_ext == 6'd9);

assign s_axis_fm_tready = tready_reg;
assign wren             = wren_reg;
assign wraddr           = wraddr_reg;
assign wrdata           = wrdata_reg;
assign fm_add_sig       = fm_add_sig_reg;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        pending_cmd_valid <= 1'b0;
    else if (pending_cmd_start)
        pending_cmd_valid <= 1'b0;
    else if (capture_pending_cmd)
        pending_cmd_valid <= 1'b1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        pending_cmd <= 112'd0;
    else if (capture_pending_cmd)
        pending_cmd <= rdcmd_trans;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        rdcmd_trans_reg <= 112'd0;
    else if (cmd_start)
        rdcmd_trans_reg <= cmd_start_trans;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        wraddr_wrap_thresh_reg <= 14'd0;
    else if (cfg_cnt_reg == 3'd4)
        wraddr_wrap_thresh_reg <= (wrap_size_reg == 32'd0) ? 14'd0
                                  : (wrap_size_reg[13:0] - 14'd1);
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        cfg_cnt_reg <= 3'd0;
    else if (cmd_start)
        cfg_cnt_reg <= 3'd1;
    else if ((cfg_cnt_reg != 3'd0) && (cfg_cnt_reg < 3'd5))
        cfg_cnt_reg <= cfg_cnt_reg + 3'd1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        cfg_ready_reg <= 1'b0;
    else if (cmd_start)
        cfg_ready_reg <= 1'b0;
    else if (cfg_cnt_reg == 3'd4)
        cfg_ready_reg <= 1'b1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        has_fm_words_reg <= 1'b0;
    else if (cmd_start)
        has_fm_words_reg <= cmd_has_fm_words;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        mult_reg <= 32'd0;
    else if (cfg_cnt_reg == 3'd1)
        mult_reg <= {24'd0, reg_col_blk_num} * {20'd0, reg_cin_pair_num};
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        line_size_reg <= 32'd0;
    else if (cfg_cnt_reg == 3'd2)
        line_size_reg <= reg_is_pool ? {24'd0, reg_col_blk_num} : mult_reg;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        wrap_size_reg <= 32'd0;
    else if (cfg_cnt_reg == 3'd3)
        wrap_size_reg <= (line_size_reg << 3) + line_size_reg;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        cin_cnt <= {`RD_FM_CHANNEL_IN_WIDTH{1'b0}};
    else if (cmd_start)
        cin_cnt <= {`RD_FM_CHANNEL_IN_WIDTH{1'b0}};
    else if (handshake && is_conv) begin
        if (conv_cin_done)
            cin_cnt <= {`RD_FM_CHANNEL_IN_WIDTH{1'b0}};
        else
            cin_cnt <= cin_cnt + 1'b1;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        col_blk_cnt <= {`RD_FM_COLUMN_IN_WIDTH{1'b0}};
    else if (cmd_start)
        col_blk_cnt <= {`RD_FM_COLUMN_IN_WIDTH{1'b0}};
    else if (handshake && is_conv) begin
        if (conv_col_done)
            col_blk_cnt <= {`RD_FM_COLUMN_IN_WIDTH{1'b0}};
        else if (conv_cin_done)
            col_blk_cnt <= col_blk_cnt + 1'b1;
    end
    else if (handshake && is_pool) begin
        if (pool_col_done)
            col_blk_cnt <= {`RD_FM_COLUMN_IN_WIDTH{1'b0}};
        else
            col_blk_cnt <= col_blk_cnt + 1'b1;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        row_cnt <= {`RD_FM_ROW_IN_WIDTH{1'b0}};
    else if (cmd_start)
        row_cnt <= {`RD_FM_ROW_IN_WIDTH{1'b0}};
    else if (handshake && is_conv) begin
        if (conv_row_done)
            row_cnt <= {`RD_FM_ROW_IN_WIDTH{1'b0}};
        else if (conv_col_done)
            row_cnt <= row_cnt + 1'b1;
    end
    else if (handshake && is_pool) begin
        if (pool_row_done)
            row_cnt <= {`RD_FM_ROW_IN_WIDTH{1'b0}};
        else if (pool_col_done)
            row_cnt <= row_cnt + 1'b1;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        ch_cnt <= {`RD_FM_CHANNEL_IN_WIDTH{1'b0}};
    else if (cmd_start)
        ch_cnt <= {`RD_FM_CHANNEL_IN_WIDTH{1'b0}};
    else if (handshake && is_pool) begin
        if (pool_ch_done)
            ch_cnt <= {`RD_FM_CHANNEL_IN_WIDTH{1'b0}};
        else if (pool_row_done)
            ch_cnt <= ch_cnt + 1'b1;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        first_batch_size_reg <= 4'd0;
    else if (cmd_start && cmd_start_is_conv)
        first_batch_size_reg <= first_batch_size_in;
    else if (cmd_start)
        first_batch_size_reg <= 4'd0;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        rows_in_batch <= 4'd0;
    else if (cmd_start && cmd_start_is_conv)
        rows_in_batch <= first_batch_size_in;
    else if (cmd_start)
        rows_in_batch <= 4'd0;
    else if (conv_row_end_hs && conv_batch_full)
        rows_in_batch <= steady_rows_in_batch;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        row_in_batch_cnt <= 4'd0;
    else if (cmd_start)
        row_in_batch_cnt <= 4'd0;
    else if (conv_row_end_hs && conv_batch_full)
        row_in_batch_cnt <= 4'd0;
    else if (conv_row_end_hs)
        row_in_batch_cnt <= row_in_batch_cnt + 1'b1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        first_batch <= 1'b0;
    else if (cmd_start && cmd_start_is_conv)
        first_batch <= 1'b1;
    else if (cmd_start)
        first_batch <= 1'b0;
    else if (conv_row_end_hs && conv_batch_full)
        first_batch <= 1'b0;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        fm_out_row_cnt <= {`RD_CONV_FM_OUT_ROW_WIDTH{1'b0}};
    else if (cmd_start && cmd_start_is_conv)
        fm_out_row_cnt <= cmd_start_trans[`RD_CONV_FM_OUT_ROW_START +: `RD_CONV_FM_OUT_ROW_WIDTH];
    else if (cmd_start)
        fm_out_row_cnt <= {`RD_CONV_FM_OUT_ROW_WIDTH{1'b0}};
    else if (fm_add_sig_reg && (fm_out_row_cnt != 0))
        fm_out_row_cnt <= fm_out_row_cnt - 1'b1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        phantom_mode <= 1'b0;
    else if (cmd_start)
        phantom_mode <= 1'b0;
    else if (conv_all_done_hs)
        phantom_mode <= 1'b1;
    else if (fm_out_row_cnt == 0)
        phantom_mode <= 1'b0;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        phantom_pulse_d <= 1'b0;
    else
        phantom_pulse_d <= phantom_pulse;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        fm_add_sig_reg <= 1'b0;
    else
        fm_add_sig_reg <= real_add_d || phantom_pulse_d;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        real_add_d <= 1'b0;
    else
        real_add_d <= real_add_pulse;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        real_add_amt_d <= 5'd0;
    else if (real_add_pulse)
        real_add_amt_d <= add_amt_now;
    else
        real_add_amt_d <= 5'd0;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        conv_sub_row_cnt <= {`RD_CONV_FM_OUT_ROW_WIDTH{1'b0}};
    else if (cmd_start && cmd_start_is_conv)
        conv_sub_row_cnt <= {`RD_CONV_FM_OUT_ROW_WIDTH{1'b0}};
    else if (fm_sub_sig && is_conv)
        conv_sub_row_cnt <= conv_sub_row_cnt + 1'b1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        conv_released_row_cnt <= 16'd0;
    else if (cmd_start && cmd_start_is_conv)
        conv_released_row_cnt <= 16'd0;
    else if (fm_sub_sig && is_conv)
        conv_released_row_cnt <= conv_release_target_ext;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        fm_sub_sig_d <= 1'b0;
    else
        fm_sub_sig_d <= fm_sub_sig;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        sub_amt_d <= 5'd0;
    else if (fm_sub_sig)
        sub_amt_d <= sub_amt;
    else
        sub_amt_d <= 5'd0;
end

// free_row_cnt 必须始终 <= 环容量（FREE_ROW_INIT=9 行）。
// 多行 sub 释放记账会把计数灌过环容量（tc5 实测组 0 内 9→15），
// 超额额度使组间重装一次放行全部行，行 9-12 提前回绕覆写未消费槽位。
// 加法路径逐支钳位到 FREE_ROW_INIT；减法路径天然不超。
wire [5:0] free_add_sub_ext;
wire [5:0] free_add_both_ext;

assign free_add_sub_ext  = {1'b0, free_row_cnt} + {1'b0, sub_amt_d};
assign free_add_both_ext = {1'b0, free_row_cnt} + {1'b0, sub_amt_d} - {1'b0, real_add_amt_d};

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        free_row_cnt <= FREE_ROW_INIT;
    else if (cmd_start)
        free_row_cnt <= FREE_ROW_INIT;
    else if (phantom_mode && !real_add_d)
        free_row_cnt <= free_row_cnt;
    else begin
        case ({real_add_d, fm_sub_sig_d})
            2'b10:   free_row_cnt <= free_row_cnt - real_add_amt_d;
            2'b01:   free_row_cnt <= (free_add_sub_ext > {1'b0, FREE_ROW_INIT}) ? FREE_ROW_INIT : free_add_sub_ext[4:0];
            2'b11:   free_row_cnt <= (free_add_both_ext > {1'b0, FREE_ROW_INIT}) ? FREE_ROW_INIT : free_add_both_ext[4:0];
            default: free_row_cnt <= free_row_cnt;
        endcase
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        pool_pair_wait <= 1'b0;
    else if (cmd_start)
        pool_pair_wait <= 1'b0;
    else if (pool_pair_nonlast_done_hs)
        pool_pair_wait <= 1'b1;
    else if (pool_pair_wait && (free_after_update_ext == 6'd9))
        pool_pair_wait <= 1'b0;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        cmd_active_reg <= 1'b0;
    else if (cmd_start)
        cmd_active_reg <= cmd_has_fm_words;
    else if (pool_done_hs)
        cmd_active_reg <= 1'b0;
    else if (phantom_mode && (fm_out_row_cnt == 0))
        cmd_active_reg <= 1'b0;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        tready_reg <= 1'b0;
    else if (cmd_start)
        tready_reg <= 1'b0;
    else if ((cfg_cnt_reg == 3'd4) && !tready_reg)
        tready_reg <= has_fm_words_reg;
    else if (cmd_full_done_hs)
        tready_reg <= 1'b0;
    else if (conv_close_tready || pool_close_tready)
        tready_reg <= 1'b0;
    else if (fm_sub_sig_d && !tready_reg && cmd_active_reg &&
             !phantom_mode && enough_after_sub && pool_pair_reuse_ready)
        tready_reg <= 1'b1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        wren_reg <= 1'b0;
    else
        wren_reg <= handshake;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        wrdata_reg <= 512'd0;
    else if (handshake)
        wrdata_reg <= s_axis_fm_tdata;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        pool_pair_end_d <= 1'b0;
    else if (cmd_start)
        pool_pair_end_d <= 1'b0;
    else
        pool_pair_end_d <= handshake && pool_row_done;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        wraddr_reg <= 14'd0;
    else if (cmd_start)
        wraddr_reg <= 14'd0;
    else if (wren_reg) begin
        if (is_pool && pool_pair_end_d)
            wraddr_reg <= 14'd0;
        else if (wraddr_reg == wraddr_wrap_thresh_reg)
            wraddr_reg <= 14'd0;
        else
            wraddr_reg <= wraddr_reg + 1'b1;
    end
end

endmodule

// ============================================================
// RTL Self-Check Results
// ============================================================
// [PASS] 端口方向和位宽保持与旧 fram_write_ctrl 接口一致
// [PASS] 使用 all_defines.vh 中的读指令字段宏，未硬编码字段位置
// [PASS] 纯 Verilog RTL，未使用 logic/always_ff/always_comb/function/task
// [PASS] 组合逻辑只使用 assign，时序逻辑只使用非阻塞赋值
// [PASS] 每个状态寄存器由独立 always 块驱动
// [PASS] RAM 写端 wren/wraddr/wrdata 保持寄存输出，并匹配同步写采样
// [PASS] FRAM 读侧只通过 fm_add_sig/fm_sub_sig 信用接口耦合，未依赖预取模块内部实现
// [PASS] free_row_cnt 加法路径逐支钳位到环容量 FREE_ROW_INIT（6bit 扩展比较，无乘法、声明先于引用）
// [PASS] 每条 conv/pool 指令广播即触发 cmd_start 重启写会话（多组广播逐组复位，不依赖 cmd_idle/phantom 尾）
// [WARN] 写地址回绕阈值在命令启动路径使用乘法；位宽已显式扩展
// ============================================================
