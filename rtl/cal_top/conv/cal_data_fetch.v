`include "all_defines.vh"

module cal_data_fetch(
input                                       clk,
input                                       rst_n,
input                                       rdcmd_trans_valid,
input       [`FETCH_INCMD-1:0]              rdcmd_trans,
// quan 行接收反压：行末拍（pixel_last）须等 quan 可接收，
// 防止 conv_valid 落在 quan conv_busy 窗口内被静默丢行（FRAM-009）
input                                       conv_ready,
//padding_input axis
input                                       s_axis_pfmtem_tvalid,
input       [`FETCH_PDDATA_WIDTH-1:0]       s_axis_pfmtem_tdata,
output                                      s_axis_pfmtem_tready,
//featuremap_input axis
input                                       s_axis_ffmtem_tvalid,
input       [`FETCH_FMDATA_WIDTH-1:0]       s_axis_ffmtem_tdata,
output                                      s_axis_ffmtem_tready,
//weight_input axis
input                                       s_axis_wetem_tvalid,
input       [`FETCH_WTDATA_WIDTH-1:0]       s_axis_wetem_tdata,
output                                      s_axis_wetem_tready,
//bias_input bias_ctrl
input                                       bias_valid,
input       [`FETCH_BADATA_WIDTH-1:0]       bias_data,
output                                      initial_valid,
output                                      data_valid,
output                                      end_data_valid,
output      [`FETCH_FMDATA_WIDTH-1:0]       fm_data_256,
output      [`FETCH_WTDATA_WIDTH-1:0]       weight_in,
output      [`FETCH_BADATA_WIDTH-1:0]       bias_in
);

localparam [2:0] ST_IDLE   = 3'd0;
localparam [2:0] ST_PREP0  = 3'd1;
localparam [2:0] ST_PREP1  = 3'd2;
localparam [2:0] ST_SEND   = 3'd3;
localparam [2:0] ST_PREP1B = 3'd4;
localparam [2:0] ST_PREP1C = 3'd5;

wire [`RD_MODEL_SEL_WIDTH-1:0] cal_model;
assign cal_model = rdcmd_trans[`RD_MODEL_SEL_START+:`RD_MODEL_SEL_WIDTH];

wire conv_cmd_valid;
assign conv_cmd_valid = rdcmd_trans_valid & (cal_model == `MODEL_SEL_CONV);

reg [2:0] state;
reg [`FETCH_INCMD-1:0] rdcmd_trans_r;

reg [`RD_FM_CHANNEL_IN_WIDTH-1:0]      fm_channel_in_num;
reg [`RD_FM_ROW_IN_WIDTH-1:0]          fm_row_in_num;
reg [15:0]                             fm_col_in_num;
reg [`RD_CONV_KERNEL_SIZE_R_WIDTH-1:0] kernel_row_num;
reg [`RD_CONV_KERNEL_SIZE_C_WIDTH-1:0] kernel_col_num;
reg [15:0]                             stride_r_num;
reg [15:0]                             stride_c_num;
reg [`RD_CONV_PADDING_TOP_WIDTH-1:0]   padding_top_num;
reg [`RD_CONV_PADDING_BOTTOM_WIDTH-1:0] padding_bottom_num;
reg [`RD_CONV_PADDING_LEFT_WIDTH-1:0]  padding_left_num;
reg [`RD_CONV_PADDING_RIGHT_WIDTH-1:0] padding_right_num;
reg [`RD_CONV_FM_OUT_ROW_WIDTH-1:0]    fm_out_row_num;
reg [`RD_CONV_FM_OUT_COL_WIDTH-1:0]    fm_out_col_last_num;
reg                                    circular_padding_en_r;

reg [`RD_FM_CHANNEL_IN_WIDTH-1:0]      channel_cnt;
reg [`RD_CONV_KERNEL_SIZE_R_WIDTH-1:0] active_kernel_row_cnt;
reg [`RD_CONV_KERNEL_SIZE_C_WIDTH-1:0] kernel_col_cnt;
reg [`RD_CONV_FM_OUT_ROW_WIDTH-1:0]    out_row_cnt;
reg [`RD_CONV_FM_OUT_COL_WIDTH-1:0]    out_col_blk_cnt;

reg [15:0] base_input_row_r;
reg [15:0] valid_kh_start_r;
reg [15:0] valid_kh_end_r;
reg [15:0] active_kh_start_r;
reg [15:0] active_kh_count_r;
reg [15:0] active_real_kh_r;

wire [`RD_FM_CHANNEL_IN_WIDTH-1:0]      fm_channel_in_field;
wire [`RD_FM_ROW_IN_WIDTH-1:0]          fm_row_in_field;
wire [`RD_FM_COLUMN_IN_WIDTH-1:0]       fm_col_in_field;
wire [`RD_CONV_KERNEL_SIZE_R_WIDTH-1:0] kernel_size_r_field;
wire [`RD_CONV_KERNEL_SIZE_C_WIDTH-1:0] kernel_size_c_field;
wire [`RD_CONV_STRIDE_R_WIDTH-1:0]      stride_r_field;
wire [`RD_CONV_STRIDE_C_WIDTH-1:0]      stride_c_field;
wire [`RD_CONV_PADDING_TOP_WIDTH-1:0]   padding_top_field;
wire [`RD_CONV_PADDING_BOTTOM_WIDTH-1:0] padding_bottom_field;
wire [`RD_CONV_PADDING_LEFT_WIDTH-1:0]  padding_left_field;
wire [`RD_CONV_PADDING_RIGHT_WIDTH-1:0] padding_right_field;
wire [`RD_CONV_FM_OUT_ROW_WIDTH-1:0]    fm_out_row_field;
wire [`RD_CONV_FM_OUT_COL_WIDTH-1:0]    fm_out_col_field;
wire                                    circular_padding_en_field;

assign fm_channel_in_field       = rdcmd_trans[`RD_FM_CHANNEL_IN_START+:`RD_FM_CHANNEL_IN_WIDTH];
assign fm_row_in_field           = rdcmd_trans[`RD_FM_ROW_IN_START+:`RD_FM_ROW_IN_WIDTH];
assign fm_col_in_field           = rdcmd_trans[`RD_FM_COLUMN_IN_START+:`RD_FM_COLUMN_IN_WIDTH];
assign kernel_size_r_field       = rdcmd_trans[`RD_CONV_KERNEL_SIZE_R_START+:`RD_CONV_KERNEL_SIZE_R_WIDTH];
assign kernel_size_c_field       = rdcmd_trans[`RD_CONV_KERNEL_SIZE_C_START+:`RD_CONV_KERNEL_SIZE_C_WIDTH];
assign stride_r_field            = rdcmd_trans[`RD_CONV_STRIDE_R_START+:`RD_CONV_STRIDE_R_WIDTH];
assign stride_c_field            = rdcmd_trans[`RD_CONV_STRIDE_C_START+:`RD_CONV_STRIDE_C_WIDTH];
assign padding_top_field         = rdcmd_trans[`RD_CONV_PADDING_TOP_START+:`RD_CONV_PADDING_TOP_WIDTH];
assign padding_bottom_field      = rdcmd_trans[`RD_CONV_PADDING_BOTTOM_START+:`RD_CONV_PADDING_BOTTOM_WIDTH];
assign padding_left_field        = rdcmd_trans[`RD_CONV_PADDING_LEFT_START+:`RD_CONV_PADDING_LEFT_WIDTH];
assign padding_right_field       = rdcmd_trans[`RD_CONV_PADDING_RIGHT_START+:`RD_CONV_PADDING_RIGHT_WIDTH];
assign fm_out_row_field          = rdcmd_trans[`RD_CONV_FM_OUT_ROW_START+:`RD_CONV_FM_OUT_ROW_WIDTH];
assign fm_out_col_field          = rdcmd_trans[`RD_CONV_FM_OUT_COL_START+:`RD_CONV_FM_OUT_COL_WIDTH];
assign circular_padding_en_field = rdcmd_trans[`RD_CONV_CIRCULAR_PADDING_EN_START+:`RD_CONV_CIRCULAR_PADDING_EN_WIDTH];

wire [15:0] cfg_fm_channel_in_num;
wire [15:0] cfg_fm_row_in_num;
wire [15:0] cfg_fm_col_in_num;
wire [15:0] cfg_kernel_row_num;
wire [15:0] cfg_kernel_col_num;
wire [15:0] cfg_stride_r_num;
wire [15:0] cfg_stride_c_num;
wire [15:0] cfg_fm_out_row_num;
wire [15:0] kernel_row_num_ext;

assign cfg_fm_channel_in_num = (fm_channel_in_field == 0) ? 16'd1 : {4'b0, fm_channel_in_field};
assign cfg_fm_row_in_num     = (fm_row_in_field == 0) ? 16'd1 : {4'b0, fm_row_in_field};
assign cfg_fm_col_in_num     = ({9'b0, fm_col_in_field} + 16'd1) << 5;
assign cfg_kernel_row_num    = (kernel_size_r_field == 0) ? 16'd1 : {13'b0, kernel_size_r_field};
assign cfg_kernel_col_num    = (kernel_size_c_field == 0) ? 16'd1 : {13'b0, kernel_size_c_field};
assign cfg_stride_r_num      = {14'b0, stride_r_field} + 16'd1;
assign cfg_stride_c_num      = {14'b0, stride_c_field} + 16'd1;
assign cfg_fm_out_row_num    = (fm_out_row_field == 0) ? 16'd1 : {4'b0, fm_out_row_field};
assign kernel_row_num_ext    = {13'b0, kernel_row_num};

wire [15:0] base_input_row_next;
// stride_r_num <= 4：12b x 3b 小乘，LUT 实现，避免落入 DSP 长路（时序整改点）
assign base_input_row_next = {4'b0, out_row_cnt} * {13'b0, stride_r_num[2:0]};

wire signed [16:0] valid_start_calc_s;
wire signed [16:0] valid_end_calc_s;
assign valid_start_calc_s = $signed({1'b0, {14'b0, padding_top_num}}) - $signed({1'b0, base_input_row_r});
assign valid_end_calc_s   = $signed({1'b0, {14'b0, padding_top_num}}) + $signed({1'b0, {4'b0, fm_row_in_num}}) - $signed({1'b0, base_input_row_r});

// PREP1 级：两个 17b 有符号加减独占一拍，打断 padding_top_num -> 加减 ->
// 钳位 -> 计数 -> ==0 -> CE 的组合长链（时序整改点，计算值不变）
reg signed [16:0] valid_start_s_r;
reg signed [16:0] valid_end_s_r;

wire [15:0] kh_start_clamp;
wire [15:0] kh_end_clamp;
wire [15:0] kh_count_comb;
assign kh_start_clamp = (valid_start_s_r <= 17'sd0) ? 16'd0 :
                        (valid_start_s_r >= $signed({1'b0, kernel_row_num_ext})) ? kernel_row_num_ext :
                        valid_start_s_r[15:0];
assign kh_end_clamp   = (valid_end_s_r <= 17'sd0) ? 16'd0 :
                        (valid_end_s_r >= $signed({1'b0, kernel_row_num_ext})) ? kernel_row_num_ext :
                        valid_end_s_r[15:0];
assign kh_count_comb  = (kh_end_clamp > kh_start_clamp) ? (kh_end_clamp - kh_start_clamp) : 16'd0;

wire active_kh_empty;
assign active_kh_empty = (active_kh_count_r == 16'd0);

wire active_real_kh_is_padding;
assign active_real_kh_is_padding = (active_real_kh_r < valid_kh_start_r) | (active_real_kh_r >= valid_kh_end_r);

wire use_padding_data;
assign use_padding_data = circular_padding_en_r & active_real_kh_is_padding;

wire selected_fm_valid;
assign selected_fm_valid = use_padding_data ? s_axis_pfmtem_tvalid : s_axis_ffmtem_tvalid;

wire channel_last;
wire kernel_col_last;
wire active_kernel_row_last;
wire out_col_blk_last;
wire out_row_last;
wire pixel_last;
wire block_done_fire;
wire [2:0] active_kh_last_idx;

assign channel_last           = (channel_cnt == (fm_channel_in_num - 1'b1));
assign kernel_col_last        = (kernel_col_cnt == (kernel_col_num - 1'b1));
assign active_kh_last_idx     = active_kh_count_r[2:0] - 3'd1;
assign active_kernel_row_last = (active_kernel_row_cnt == active_kh_last_idx);
assign out_col_blk_last       = (out_col_blk_cnt == fm_out_col_last_num);
assign out_row_last           = (out_row_cnt == (fm_out_row_num - 1'b1));
assign pixel_last             = channel_last & kernel_col_last & active_kernel_row_last;

wire fetch_allow;
// 末拍信用闭环（20260728）：conv_ready 要等 quant 收到 conv_valid（busy 拉起）才变 0，
// k1x1 等短块场景下一块仅数拍，块 N+1 末拍 fire 时块 N 的 conv_valid 尚未到达 quant，
// 单看 conv_ready 会误放行导致该块 conv_valid 被 busy 丢弃（tc121 实测）。
// 故末拍 fire 后置 last_outstanding，见到 conv_ready 拉低（quant 已收下）才清除；
// 末拍放行条件收紧为 conv_ready & ~last_outstanding，保证任一时刻至多一块在途。
reg last_outstanding;

assign fetch_allow = (state == ST_SEND) & bias_valid & ~active_kh_empty
                     & (~pixel_last | (conv_ready & ~last_outstanding));

wire fire;
assign fire = fetch_allow & selected_fm_valid & s_axis_wetem_tvalid;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        last_outstanding <= 1'b0;
    else if (state == ST_IDLE)
        last_outstanding <= 1'b0;
    else if (fire & pixel_last)
        last_outstanding <= 1'b1;
    else if (!conv_ready)
        last_outstanding <= 1'b0;
end

assign block_done_fire        = fire & pixel_last & out_col_blk_last & out_row_last;

assign s_axis_pfmtem_tready = fire & use_padding_data;
assign s_axis_ffmtem_tready = fire & ~use_padding_data;
assign s_axis_wetem_tready  = fire;

assign data_valid     = fire;
assign initial_valid  = fire & (channel_cnt == 0) & (kernel_col_cnt == 0) & (active_kernel_row_cnt == 0);
assign end_data_valid = fire & pixel_last;
assign fm_data_256    = use_padding_data ? s_axis_pfmtem_tdata : s_axis_ffmtem_tdata;
assign weight_in      = s_axis_wetem_tdata;
assign bias_in        = bias_data;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        state <= ST_IDLE;
    else begin
        case (state)
            ST_IDLE: begin
                if (conv_cmd_valid)
                    state <= ST_PREP0;
            end
            ST_PREP0: begin
                state <= ST_PREP1;
            end
            ST_PREP1: begin
                state <= ST_PREP1B;
            end
            ST_PREP1B: begin
                state <= ST_PREP1C;
            end
            ST_PREP1C: begin
                state <= (active_kh_count_r == 16'd0) ? ST_PREP0 : ST_SEND;
            end
            ST_SEND: begin
                if (block_done_fire)
                    state <= ST_IDLE;
                else if (fire & pixel_last & out_col_blk_last)
                    state <= ST_PREP0;
            end
            default: begin
                state <= ST_IDLE;
            end
        endcase
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        rdcmd_trans_r <= 0;
    else if (conv_cmd_valid && (state == ST_IDLE))
        rdcmd_trans_r <= rdcmd_trans;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        fm_channel_in_num <= 0;
    else if (conv_cmd_valid && (state == ST_IDLE))
        fm_channel_in_num <= cfg_fm_channel_in_num[`RD_FM_CHANNEL_IN_WIDTH-1:0];
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        fm_row_in_num <= 0;
    else if (conv_cmd_valid && (state == ST_IDLE))
        fm_row_in_num <= cfg_fm_row_in_num[`RD_FM_ROW_IN_WIDTH-1:0];
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        fm_col_in_num <= 0;
    else if (conv_cmd_valid && (state == ST_IDLE))
        fm_col_in_num <= cfg_fm_col_in_num;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        kernel_row_num <= 0;
    else if (conv_cmd_valid && (state == ST_IDLE))
        kernel_row_num <= cfg_kernel_row_num[`RD_CONV_KERNEL_SIZE_R_WIDTH-1:0];
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        kernel_col_num <= 0;
    else if (conv_cmd_valid && (state == ST_IDLE))
        kernel_col_num <= cfg_kernel_col_num[`RD_CONV_KERNEL_SIZE_C_WIDTH-1:0];
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        stride_r_num <= 0;
    else if (conv_cmd_valid && (state == ST_IDLE))
        stride_r_num <= cfg_stride_r_num;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        stride_c_num <= 0;
    else if (conv_cmd_valid && (state == ST_IDLE))
        stride_c_num <= cfg_stride_c_num;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        padding_top_num <= 0;
    else if (conv_cmd_valid && (state == ST_IDLE))
        padding_top_num <= padding_top_field;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        padding_bottom_num <= 0;
    else if (conv_cmd_valid && (state == ST_IDLE))
        padding_bottom_num <= padding_bottom_field;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        padding_left_num <= 0;
    else if (conv_cmd_valid && (state == ST_IDLE))
        padding_left_num <= padding_left_field;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        padding_right_num <= 0;
    else if (conv_cmd_valid && (state == ST_IDLE))
        padding_right_num <= padding_right_field;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        fm_out_row_num <= 0;
    else if (conv_cmd_valid && (state == ST_IDLE))
        fm_out_row_num <= cfg_fm_out_row_num[`RD_CONV_FM_OUT_ROW_WIDTH-1:0];
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        fm_out_col_last_num <= 0;
    else if (conv_cmd_valid && (state == ST_IDLE))
        fm_out_col_last_num <= fm_out_col_field;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        circular_padding_en_r <= 0;
    else if (conv_cmd_valid && (state == ST_IDLE))
        circular_padding_en_r <= circular_padding_en_field;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        base_input_row_r <= 0;
    else if (state == ST_PREP0)
        base_input_row_r <= base_input_row_next;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        valid_start_s_r <= 17'sd0;
    else if (state == ST_PREP1)
        valid_start_s_r <= valid_start_calc_s;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        valid_end_s_r <= 17'sd0;
    else if (state == ST_PREP1)
        valid_end_s_r <= valid_end_calc_s;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        valid_kh_start_r <= 0;
    else if (state == ST_PREP1B)
        valid_kh_start_r <= kh_start_clamp;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        valid_kh_end_r <= 0;
    else if (state == ST_PREP1B)
        valid_kh_end_r <= kh_end_clamp;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        active_kh_start_r <= 0;
    else if (state == ST_PREP1B)
        active_kh_start_r <= circular_padding_en_r ? 16'd0 : kh_start_clamp;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        active_kh_count_r <= 0;
    else if (state == ST_PREP1B)
        active_kh_count_r <= circular_padding_en_r ? {13'b0, kernel_row_num} : kh_count_comb;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        active_real_kh_r <= 0;
    else if (state == ST_PREP1B)
        active_real_kh_r <= circular_padding_en_r ? 16'd0 : kh_start_clamp;
    else if (fire && kernel_col_last) begin
        if (active_kernel_row_last)
            active_real_kh_r <= active_kh_start_r;
        else
            active_real_kh_r <= active_real_kh_r + 16'd1;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        channel_cnt <= 0;
    else if (conv_cmd_valid && (state == ST_IDLE))
        channel_cnt <= 0;
    else if (state == ST_PREP1)
        channel_cnt <= 0;
    else if (fire && kernel_col_last && active_kernel_row_last) begin
        if (channel_last)
            channel_cnt <= 0;
        else
            channel_cnt <= channel_cnt + 1'b1;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        kernel_col_cnt <= 0;
    else if (conv_cmd_valid && (state == ST_IDLE))
        kernel_col_cnt <= 0;
    else if (state == ST_PREP1)
        kernel_col_cnt <= 0;
    else if (fire) begin
        if (kernel_col_last)
            kernel_col_cnt <= 0;
        else
            kernel_col_cnt <= kernel_col_cnt + 1'b1;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        active_kernel_row_cnt <= 0;
    else if (conv_cmd_valid && (state == ST_IDLE))
        active_kernel_row_cnt <= 0;
    else if (state == ST_PREP1)
        active_kernel_row_cnt <= 0;
    else if (fire && kernel_col_last) begin
        if (active_kernel_row_last)
            active_kernel_row_cnt <= 0;
        else
            active_kernel_row_cnt <= active_kernel_row_cnt + 1'b1;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        out_col_blk_cnt <= 0;
    else if (conv_cmd_valid && (state == ST_IDLE))
        out_col_blk_cnt <= 0;
    else if (fire && pixel_last) begin
        if (out_col_blk_last)
            out_col_blk_cnt <= 0;
        else
            out_col_blk_cnt <= out_col_blk_cnt + 1'b1;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        out_row_cnt <= 0;
    else if (conv_cmd_valid && (state == ST_IDLE))
        out_row_cnt <= 0;
    else if (state == ST_PREP1C && (active_kh_count_r == 16'd0)) begin
        if (out_row_last)
            out_row_cnt <= 0;
        else
            out_row_cnt <= out_row_cnt + 1'b1;
    end
    else if (fire && pixel_last && out_col_blk_last) begin
        if (out_row_last)
            out_row_cnt <= 0;
        else
            out_row_cnt <= out_row_cnt + 1'b1;
    end
end

endmodule
