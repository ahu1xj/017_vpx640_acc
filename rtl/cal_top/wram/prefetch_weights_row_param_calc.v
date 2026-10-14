module prefetch_weights_row_param_calc(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        cmd_accept,
    input  wire [11:0] cmd_fm_channel_in,
    input  wire [11:0] cmd_fm_row_in,
    input  wire [2:0]  cmd_kernel_r,
    input  wire [2:0]  cmd_kernel_c,
    input  wire [1:0]  cmd_stride_r,
    input  wire [1:0]  cmd_padding_top,
    input  wire        cmd_cpad_en,
    input  wire [11:0] cmd_fm_out_row,
    output wire        ready_for_cmd,
    output wire [13:0] weight_last_addr,
    output wire        param_fifo_s_tvalid,
    output wire [41:0] param_fifo_s_tdata,
    input  wire        param_fifo_s_tready
);

localparam        CALC_IDLE         = 1'b0;
localparam        CALC_WORK         = 1'b1;

localparam [3:0]  CALC_PHASE_KAREA0 = 4'd0;
localparam [3:0]  CALC_PHASE_KAREA1 = 4'd1;
localparam [3:0]  CALC_PHASE_WLAST0 = 4'd2;
localparam [3:0]  CALC_PHASE_WLAST1 = 4'd3;
localparam [3:0]  CALC_PHASE_ROW0   = 4'd4;
localparam [3:0]  CALC_PHASE_ROW1   = 4'd5;
localparam [3:0]  CALC_PHASE_ROW1B  = 4'd9;
localparam [3:0]  CALC_PHASE_ROW2   = 4'd6;
localparam [3:0]  CALC_PHASE_ROW3   = 4'd7;
localparam [3:0]  CALC_PHASE_PUSH   = 4'd8;

localparam [41:0] PARAM_ZERO        = 42'd0;

reg        calc_state_reg;
reg [3:0]  calc_phase_reg;
reg [11:0] calc_out_row_cnt_reg;

reg [11:0] fm_channel_in_reg;
reg [11:0] fm_row_in_reg;
reg [2:0]  kernel_r_reg;
reg [2:0]  kernel_c_reg;
reg        stride_r_is_two_reg;
reg [1:0]  padding_top_reg;
reg        cpad_en_reg;
reg [11:0] fm_out_row_reg;

reg [13:0] kernel_area_reg;
reg [13:0] weight_last_addr_reg;

reg [13:0] kernel_mul_a_reg;
reg [13:0] kernel_mul_b_reg;
reg [27:0] kernel_mul_product_reg;
reg [13:0] weight_mul_a_reg;
reg [13:0] weight_mul_b_reg;
reg [27:0] weight_mul_product_reg;

reg [12:0] row_base_reg;
reg [3:0]  head_skip_rows_reg;
reg [3:0]  valid_rows_reg;
reg [3:0]  skip_rows_reg;

reg [13:0] head_mul_a_reg;
reg [13:0] head_mul_b_reg;
reg [27:0] head_mul_product_reg;
reg [13:0] valid_mul_a_reg;
reg [13:0] valid_mul_b_reg;
reg [27:0] valid_mul_product_reg;
reg [13:0] skip_mul_a_reg;
reg [13:0] skip_mul_b_reg;
reg [27:0] skip_mul_product_reg;

reg        param_s_valid_reg;
reg [41:0] param_s_data_reg;

wire [13:0] padding_top_ext;
wire [13:0] fm_row_in_ext;
wire [13:0] kernel_r_ext;
wire [13:0] row_end_calc;
wire [13:0] valid_high_calc;
wire [13:0] top_delta_calc;
wire [13:0] bottom_delta_calc;
wire [2:0]  head_skip_rows_calc;

wire [13:0] row_init_addr_calc;
wire [13:0] row_valid_beats_calc;
wire [13:0] row_valid_beats_m1_calc;
wire [14:0] row_skip_step_calc;
wire [41:0] row_param_data_calc;

// ROW1 级寄存：valid_rows 长链中间值（时序整改点，计算值不变）
reg [13:0] top_delta_r;
reg [13:0] bottom_delta_r;
reg [2:0]  head_skip_r;
// ROW1B 级组合：由寄存中间值出最终配置
wire [3:0]  remain_rows_comb;
wire [2:0]  tail_skip_comb;
wire [3:0]  valid_rows_comb;
wire [3:0]  skip_rows_comb;
assign remain_rows_comb = {1'b0, kernel_r_reg} - {1'b0, head_skip_r};
assign tail_skip_comb   = (bottom_delta_r > {10'd0, remain_rows_comb}) ?
                          remain_rows_comb[2:0] :
                          bottom_delta_r[2:0];
assign valid_rows_comb  = remain_rows_comb - {1'b0, tail_skip_comb};
assign skip_rows_comb   = {1'b0, head_skip_r} + {1'b0, tail_skip_comb};

wire        param_push_fire;
wire        calc_row_last;

assign ready_for_cmd             = (calc_state_reg == CALC_IDLE) && !param_s_valid_reg;
assign weight_last_addr          = weight_last_addr_reg;

assign padding_top_ext           = {12'd0, padding_top_reg};
assign fm_row_in_ext             = {2'd0, fm_row_in_reg};
assign kernel_r_ext              = {11'd0, kernel_r_reg};
assign row_end_calc              = {1'b0, row_base_reg} + kernel_r_ext;
assign valid_high_calc           = padding_top_ext + fm_row_in_ext;
assign top_delta_calc            = (padding_top_ext > {1'b0, row_base_reg}) ?
                                   (padding_top_ext - {1'b0, row_base_reg}) :
                                   14'd0;
assign bottom_delta_calc         = (row_end_calc > valid_high_calc) ?
                                   (row_end_calc - valid_high_calc) :
                                   14'd0;
assign head_skip_rows_calc       = (top_delta_calc > kernel_r_ext) ? kernel_r_reg : top_delta_calc[2:0];

assign row_init_addr_calc        = head_mul_product_reg[13:0];
assign row_valid_beats_calc      = valid_mul_product_reg[13:0];
assign row_valid_beats_m1_calc   = (row_valid_beats_calc == 14'd0) ? 14'd0 : (row_valid_beats_calc - 14'd1);
assign row_skip_step_calc        = {1'b0, skip_mul_product_reg[13:0]} + 15'd1;
assign row_param_data_calc       = {row_init_addr_calc, row_valid_beats_m1_calc, row_skip_step_calc[13:0]};

assign param_fifo_s_tvalid       = param_s_valid_reg;
assign param_fifo_s_tdata        = param_s_valid_reg ? param_s_data_reg : PARAM_ZERO;
assign param_push_fire           = param_fifo_s_tvalid && param_fifo_s_tready;
assign calc_row_last             = (calc_out_row_cnt_reg == (fm_out_row_reg - 12'd1));

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        kernel_mul_product_reg <= 28'd0;
    else
        kernel_mul_product_reg <= kernel_mul_a_reg * kernel_mul_b_reg;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        weight_mul_product_reg <= 28'd0;
    else
        weight_mul_product_reg <= weight_mul_a_reg * weight_mul_b_reg;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        head_mul_product_reg <= 28'd0;
    else
        head_mul_product_reg <= head_mul_a_reg * head_mul_b_reg;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        valid_mul_product_reg <= 28'd0;
    else
        valid_mul_product_reg <= valid_mul_a_reg * valid_mul_b_reg;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        skip_mul_product_reg <= 28'd0;
    else
        skip_mul_product_reg <= skip_mul_a_reg * skip_mul_b_reg;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        calc_state_reg <= CALC_IDLE;
    else if (cmd_accept)
        calc_state_reg <= CALC_WORK;
    else if ((calc_state_reg == CALC_WORK) &&
             (calc_phase_reg == CALC_PHASE_PUSH) &&
             param_push_fire &&
             calc_row_last)
        calc_state_reg <= CALC_IDLE;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        calc_phase_reg <= CALC_PHASE_KAREA0;
    else if (cmd_accept)
        calc_phase_reg <= CALC_PHASE_KAREA0;
    else if (calc_state_reg == CALC_WORK) begin
        case (calc_phase_reg)
            CALC_PHASE_KAREA0: calc_phase_reg <= CALC_PHASE_KAREA1;
            CALC_PHASE_KAREA1: calc_phase_reg <= CALC_PHASE_WLAST0;
            CALC_PHASE_WLAST0: calc_phase_reg <= CALC_PHASE_WLAST1;
            CALC_PHASE_WLAST1: calc_phase_reg <= CALC_PHASE_ROW0;
            CALC_PHASE_ROW0:   calc_phase_reg <= CALC_PHASE_ROW1;
            CALC_PHASE_ROW1:   calc_phase_reg <= CALC_PHASE_ROW1B;
            CALC_PHASE_ROW1B:  calc_phase_reg <= CALC_PHASE_ROW2;
            CALC_PHASE_ROW2:   calc_phase_reg <= CALC_PHASE_ROW3;
            CALC_PHASE_ROW3:   calc_phase_reg <= CALC_PHASE_PUSH;
            CALC_PHASE_PUSH: begin
                if (param_push_fire && !calc_row_last)
                    calc_phase_reg <= CALC_PHASE_ROW0;
            end
            default: calc_phase_reg <= CALC_PHASE_KAREA0;
        endcase
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        calc_out_row_cnt_reg <= 12'd0;
    else if (cmd_accept)
        calc_out_row_cnt_reg <= 12'd0;
    else if ((calc_state_reg == CALC_WORK) &&
             (calc_phase_reg == CALC_PHASE_PUSH) &&
             param_push_fire &&
             !calc_row_last)
        calc_out_row_cnt_reg <= calc_out_row_cnt_reg + 12'd1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        fm_channel_in_reg <= 12'd0;
    else if (cmd_accept)
        fm_channel_in_reg <= cmd_fm_channel_in;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        fm_row_in_reg <= 12'd0;
    else if (cmd_accept)
        fm_row_in_reg <= cmd_fm_row_in;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        kernel_r_reg <= 3'd0;
    else if (cmd_accept)
        kernel_r_reg <= cmd_kernel_r;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        kernel_c_reg <= 3'd0;
    else if (cmd_accept)
        kernel_c_reg <= cmd_kernel_c;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        stride_r_is_two_reg <= 1'b0;
    else if (cmd_accept)
        stride_r_is_two_reg <= (cmd_stride_r != 2'd0);
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        padding_top_reg <= 2'd0;
    else if (cmd_accept)
        padding_top_reg <= cmd_padding_top;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        cpad_en_reg <= 1'b0;
    else if (cmd_accept)
        cpad_en_reg <= cmd_cpad_en;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        fm_out_row_reg <= 12'd0;
    else if (cmd_accept)
        fm_out_row_reg <= cmd_fm_out_row;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        kernel_area_reg <= 14'd0;
    else if (cmd_accept)
        kernel_area_reg <= 14'd0;
    else if ((calc_state_reg == CALC_WORK) && (calc_phase_reg == CALC_PHASE_KAREA1))
        kernel_area_reg <= kernel_mul_product_reg[13:0];
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        weight_last_addr_reg <= 14'd0;
    else if (cmd_accept)
        weight_last_addr_reg <= 14'd0;
    else if ((calc_state_reg == CALC_WORK) && (calc_phase_reg == CALC_PHASE_WLAST1))
        weight_last_addr_reg <= weight_mul_product_reg[13:0] - 14'd1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        kernel_mul_a_reg <= 14'd0;
    else if (cmd_accept)
        kernel_mul_a_reg <= {11'd0, cmd_kernel_r};
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        kernel_mul_b_reg <= 14'd0;
    else if (cmd_accept)
        kernel_mul_b_reg <= {11'd0, cmd_kernel_c};
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        weight_mul_a_reg <= 14'd0;
    else if (cmd_accept)
        weight_mul_a_reg <= 14'd0;
    else if ((calc_state_reg == CALC_WORK) && (calc_phase_reg == CALC_PHASE_KAREA1))
        weight_mul_a_reg <= kernel_mul_product_reg[13:0];
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        weight_mul_b_reg <= 14'd0;
    else if (cmd_accept)
        weight_mul_b_reg <= 14'd0;
    else if ((calc_state_reg == CALC_WORK) && (calc_phase_reg == CALC_PHASE_KAREA1))
        weight_mul_b_reg <= {2'd0, fm_channel_in_reg};
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        row_base_reg <= 13'd0;
    else if (cmd_accept)
        row_base_reg <= 13'd0;
    else if ((calc_state_reg == CALC_WORK) && (calc_phase_reg == CALC_PHASE_WLAST1))
        row_base_reg <= 13'd0;
    else if ((calc_state_reg == CALC_WORK) && (calc_phase_reg == CALC_PHASE_ROW0)) begin
        if (stride_r_is_two_reg)
            row_base_reg <= {calc_out_row_cnt_reg, 1'b0};
        else
            row_base_reg <= {1'b0, calc_out_row_cnt_reg};
    end
end

// ROW1 级：valid_rows 长链中间值寄存（top/bottom delta 与 head_skip）
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        top_delta_r <= 14'd0;
    else if (cmd_accept)
        top_delta_r <= 14'd0;
    else if ((calc_state_reg == CALC_WORK) && (calc_phase_reg == CALC_PHASE_ROW1))
        top_delta_r <= top_delta_calc;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        bottom_delta_r <= 14'd0;
    else if (cmd_accept)
        bottom_delta_r <= 14'd0;
    else if ((calc_state_reg == CALC_WORK) && (calc_phase_reg == CALC_PHASE_ROW1))
        bottom_delta_r <= bottom_delta_calc;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        head_skip_r <= 3'd0;
    else if (cmd_accept)
        head_skip_r <= 3'd0;
    else if ((calc_state_reg == CALC_WORK) && (calc_phase_reg == CALC_PHASE_ROW1))
        head_skip_r <= head_skip_rows_calc;
end

// cpad_en=1（循环 padding）时 padding 行是真实数据（PRAM 供给），算阵处理全部
// kh 行（cal_data_fetch: active_kh = 0..Kr-1），权重必须全量供给：
// 跳行参数清零，replay 退化为从 0 连续自增（skip_step = 0*Kc + 1 = 1）
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        head_skip_rows_reg <= 4'd0;
    else if (cmd_accept)
        head_skip_rows_reg <= 4'd0;
    else if ((calc_state_reg == CALC_WORK) && (calc_phase_reg == CALC_PHASE_ROW1B))
        head_skip_rows_reg <= cpad_en_reg ? 4'd0 : {1'b0, head_skip_r};
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        valid_rows_reg <= 4'd0;
    else if (cmd_accept)
        valid_rows_reg <= 4'd0;
    else if ((calc_state_reg == CALC_WORK) && (calc_phase_reg == CALC_PHASE_ROW1B))
        valid_rows_reg <= cpad_en_reg ? {1'b0, kernel_r_reg} : valid_rows_comb;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        skip_rows_reg <= 4'd0;
    else if (cmd_accept)
        skip_rows_reg <= 4'd0;
    else if ((calc_state_reg == CALC_WORK) && (calc_phase_reg == CALC_PHASE_ROW1B))
        skip_rows_reg <= cpad_en_reg ? 4'd0 : skip_rows_comb;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        head_mul_a_reg <= 14'd0;
    else if (cmd_accept)
        head_mul_a_reg <= 14'd0;
    else if ((calc_state_reg == CALC_WORK) && (calc_phase_reg == CALC_PHASE_ROW2))
        head_mul_a_reg <= {10'd0, head_skip_rows_reg};
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        head_mul_b_reg <= 14'd0;
    else if (cmd_accept)
        head_mul_b_reg <= 14'd0;
    else if ((calc_state_reg == CALC_WORK) && (calc_phase_reg == CALC_PHASE_ROW2))
        head_mul_b_reg <= {11'd0, kernel_c_reg};
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        valid_mul_a_reg <= 14'd0;
    else if (cmd_accept)
        valid_mul_a_reg <= 14'd0;
    else if ((calc_state_reg == CALC_WORK) && (calc_phase_reg == CALC_PHASE_ROW2))
        valid_mul_a_reg <= {10'd0, valid_rows_reg};
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        valid_mul_b_reg <= 14'd0;
    else if (cmd_accept)
        valid_mul_b_reg <= 14'd0;
    else if ((calc_state_reg == CALC_WORK) && (calc_phase_reg == CALC_PHASE_ROW2))
        valid_mul_b_reg <= {11'd0, kernel_c_reg};
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        skip_mul_a_reg <= 14'd0;
    else if (cmd_accept)
        skip_mul_a_reg <= 14'd0;
    else if ((calc_state_reg == CALC_WORK) && (calc_phase_reg == CALC_PHASE_ROW2))
        skip_mul_a_reg <= {10'd0, skip_rows_reg};
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        skip_mul_b_reg <= 14'd0;
    else if (cmd_accept)
        skip_mul_b_reg <= 14'd0;
    else if ((calc_state_reg == CALC_WORK) && (calc_phase_reg == CALC_PHASE_ROW2))
        skip_mul_b_reg <= {11'd0, kernel_c_reg};
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        param_s_valid_reg <= 1'b0;
    else if (cmd_accept)
        param_s_valid_reg <= 1'b0;
    else if (param_push_fire)
        param_s_valid_reg <= 1'b0;
    else if ((calc_state_reg == CALC_WORK) &&
             (calc_phase_reg == CALC_PHASE_PUSH) &&
             !param_s_valid_reg)
        param_s_valid_reg <= 1'b1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        param_s_data_reg <= 42'd0;
    else if (cmd_accept)
        param_s_data_reg <= 42'd0;
    else if ((calc_state_reg == CALC_WORK) &&
             (calc_phase_reg == CALC_PHASE_PUSH) &&
             !param_s_valid_reg)
        param_s_data_reg <= row_param_data_calc;
end

endmodule
