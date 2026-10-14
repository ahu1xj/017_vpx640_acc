`include "all_defines.vh"

module prefetch_weights(
    input  wire             clk,
    input  wire             rst_n,
    input  wire             rdcmd_trans_valid,
    input  wire [111:0]     rdcmd_trans,
    output wire             wrden,
    output wire [13:0]      wrdaddr,
    input  wire [511:0]     wrddata,
    output wire             m_aixs_wetem_tvalid,
    output wire [511:0]     m_aixs_wetem_tdata,
    input  wire             m_aixs_wetem_tready,
    input  wire             we_update
);

wire [`RD_MODEL_SEL_WIDTH-1:0]          cmd_model_sel;
wire [`RD_FM_CHANNEL_IN_WIDTH-1:0]      cmd_fm_channel_in;
wire [`RD_FM_ROW_IN_WIDTH-1:0]          cmd_fm_row_in;
wire [`RD_CONV_KERNEL_SIZE_R_WIDTH-1:0] cmd_kernel_r;
wire [`RD_CONV_KERNEL_SIZE_C_WIDTH-1:0] cmd_kernel_c;
wire [`RD_CONV_STRIDE_R_WIDTH-1:0]      cmd_stride_r;
wire [`RD_CONV_PADDING_TOP_WIDTH-1:0]   cmd_padding_top;
wire [`RD_CONV_FM_OUT_ROW_WIDTH-1:0]    cmd_fm_out_row;
wire [`RD_CONV_FM_OUT_COL_WIDTH-1:0]    cmd_fm_out_col;
wire                                    cmd_cpad_en;
wire                                    cmd_is_conv;
wire                                    cmd_fields_valid;
wire                                    cmd_accept;

wire                                    calc_ready_for_cmd;
wire                                    read_ready_for_cmd;
wire [13:0]                             weight_last_addr;

wire                                    param_fifo_s_tvalid;
wire [41:0]                             param_fifo_s_tdata;
wire                                    param_fifo_s_tready;
wire                                    param_fifo_m_tvalid;
wire [41:0]                             param_fifo_m_tdata;
wire                                    param_fifo_m_tready;

assign cmd_model_sel      = rdcmd_trans[`RD_MODEL_SEL_START +: `RD_MODEL_SEL_WIDTH];
assign cmd_fm_channel_in  = rdcmd_trans[`RD_FM_CHANNEL_IN_START +: `RD_FM_CHANNEL_IN_WIDTH];
assign cmd_fm_row_in      = rdcmd_trans[`RD_FM_ROW_IN_START +: `RD_FM_ROW_IN_WIDTH];
assign cmd_kernel_r       = rdcmd_trans[`RD_CONV_KERNEL_SIZE_R_START +: `RD_CONV_KERNEL_SIZE_R_WIDTH];
assign cmd_kernel_c       = rdcmd_trans[`RD_CONV_KERNEL_SIZE_C_START +: `RD_CONV_KERNEL_SIZE_C_WIDTH];
assign cmd_stride_r       = rdcmd_trans[`RD_CONV_STRIDE_R_START +: `RD_CONV_STRIDE_R_WIDTH];
assign cmd_padding_top    = rdcmd_trans[`RD_CONV_PADDING_TOP_START +: `RD_CONV_PADDING_TOP_WIDTH];
assign cmd_fm_out_row     = rdcmd_trans[`RD_CONV_FM_OUT_ROW_START +: `RD_CONV_FM_OUT_ROW_WIDTH];
assign cmd_fm_out_col     = rdcmd_trans[`RD_CONV_FM_OUT_COL_START +: `RD_CONV_FM_OUT_COL_WIDTH];
assign cmd_cpad_en        = rdcmd_trans[`RD_CONV_CIRCULAR_PADDING_EN_START];

assign cmd_is_conv        = rdcmd_trans_valid && (cmd_model_sel == `MODEL_SEL_CONV);
assign cmd_fields_valid   = (cmd_fm_channel_in != {`RD_FM_CHANNEL_IN_WIDTH{1'b0}}) &&
                            (cmd_fm_row_in     != {`RD_FM_ROW_IN_WIDTH{1'b0}}) &&
                            (cmd_kernel_r      != {`RD_CONV_KERNEL_SIZE_R_WIDTH{1'b0}}) &&
                            (cmd_kernel_c      != {`RD_CONV_KERNEL_SIZE_C_WIDTH{1'b0}}) &&
                            (cmd_fm_out_row    != {`RD_CONV_FM_OUT_ROW_WIDTH{1'b0}});
assign cmd_accept         = calc_ready_for_cmd &&
                            read_ready_for_cmd &&
                            cmd_is_conv &&
                            cmd_fields_valid;

prefetch_weights_row_param_calc u_row_param_calc (
    .clk                 (clk),
    .rst_n               (rst_n),
    .cmd_accept          (cmd_accept),
    .cmd_fm_channel_in   (cmd_fm_channel_in),
    .cmd_fm_row_in       (cmd_fm_row_in),
    .cmd_kernel_r        (cmd_kernel_r),
    .cmd_kernel_c        (cmd_kernel_c),
    .cmd_stride_r        (cmd_stride_r),
    .cmd_padding_top     (cmd_padding_top),
    .cmd_cpad_en         (cmd_cpad_en),
    .cmd_fm_out_row      (cmd_fm_out_row),
    .ready_for_cmd       (calc_ready_for_cmd),
    .weight_last_addr    (weight_last_addr),
    .param_fifo_s_tvalid (param_fifo_s_tvalid),
    .param_fifo_s_tdata  (param_fifo_s_tdata),
    .param_fifo_s_tready (param_fifo_s_tready)
);

// ----------------------------------------------------------------
// 行参数 2 深度寄存器队列（替代 axis_data_fifo_42_sync IP）
// 计算线 5 拍/行，读取线每行消耗 1 个参数；真实层每行读取拍数
// （out_col_blks × Cin × valid_rows × Kc）远大于 5，2 深度即可保证
// 行切换不插气泡。深度不足只会出现在 Cin 极小的退化层，且与 FIFO
// 一样只受计算线速率限制，无死锁。
// ----------------------------------------------------------------
reg [41:0]  param_q0_data_reg;
reg [41:0]  param_q1_data_reg;
reg         param_q0_valid_reg;
reg         param_q1_valid_reg;

wire        param_q_push;
wire        param_q_pop;

assign param_fifo_s_tready = !(param_q0_valid_reg && param_q1_valid_reg);
assign param_fifo_m_tvalid = param_q0_valid_reg;
assign param_fifo_m_tdata  = param_q0_data_reg;
assign param_q_push        = param_fifo_s_tvalid && param_fifo_s_tready;
assign param_q_pop         = param_fifo_m_tvalid && param_fifo_m_tready;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        param_q0_valid_reg <= 1'b0;
        param_q1_valid_reg <= 1'b0;
    end
    else begin
        case ({param_q_push, param_q_pop})
            2'b10: begin
                if (!param_q0_valid_reg)
                    param_q0_valid_reg <= 1'b1;
                else
                    param_q1_valid_reg <= 1'b1;
            end
            2'b01: begin
                param_q0_valid_reg <= param_q1_valid_reg;
                param_q1_valid_reg <= 1'b0;
            end
            2'b11: begin
                param_q0_valid_reg <= 1'b1;
                param_q1_valid_reg <= param_q1_valid_reg;
            end
            default: begin
                param_q0_valid_reg <= param_q0_valid_reg;
                param_q1_valid_reg <= param_q1_valid_reg;
            end
        endcase
    end
end

always @(posedge clk) begin
    case ({param_q_push, param_q_pop})
        2'b10: begin
            if (!param_q0_valid_reg)
                param_q0_data_reg <= param_fifo_s_tdata;
            else
                param_q1_data_reg <= param_fifo_s_tdata;
        end
        2'b01:
            param_q0_data_reg <= param_q1_data_reg;
        2'b11: begin
            if (param_q1_valid_reg) begin
                param_q0_data_reg <= param_q1_data_reg;
                param_q1_data_reg <= param_fifo_s_tdata;
            end
            else begin
                param_q0_data_reg <= param_fifo_s_tdata;
            end
        end
        default: begin
            param_q0_data_reg <= param_q0_data_reg;
            param_q1_data_reg <= param_q1_data_reg;
        end
    endcase
end

prefetch_weights_read_ctrl u_read_ctrl (
    .clk                   (clk),
    .rst_n                 (rst_n),
    .cmd_accept            (cmd_accept),
    .cmd_fm_out_row        (cmd_fm_out_row),
    .cmd_fm_out_col        (cmd_fm_out_col),
    .weight_last_addr      (weight_last_addr),
    .ready_for_cmd         (read_ready_for_cmd),
    .param_fifo_m_tvalid   (param_fifo_m_tvalid),
    .param_fifo_m_tdata    (param_fifo_m_tdata),
    .param_fifo_m_tready   (param_fifo_m_tready),
    .wrden                 (wrden),
    .wrdaddr               (wrdaddr),
    .wrddata               (wrddata),
    .m_aixs_wetem_tvalid   (m_aixs_wetem_tvalid),
    .m_aixs_wetem_tdata    (m_aixs_wetem_tdata),
    .m_aixs_wetem_tready   (m_aixs_wetem_tready),
    .we_update             (we_update)
);

endmodule
