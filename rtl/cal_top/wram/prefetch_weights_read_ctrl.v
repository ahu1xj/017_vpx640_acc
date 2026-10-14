module prefetch_weights_read_ctrl(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        cmd_accept,
    input  wire [11:0] cmd_fm_out_row,
    input  wire [6:0]  cmd_fm_out_col,
    input  wire [13:0] weight_last_addr,
    output wire        ready_for_cmd,
    input  wire        param_fifo_m_tvalid,
    input  wire [41:0] param_fifo_m_tdata,
    output wire        param_fifo_m_tready,
    output wire        wrden,
    output wire [13:0] wrdaddr,
    input  wire [511:0] wrddata,
    output wire        m_aixs_wetem_tvalid,
    output wire [511:0] m_aixs_wetem_tdata,
    input  wire        m_aixs_wetem_tready,
    input  wire        we_update
);

localparam        READ_IDLE         = 1'b0;
localparam        READ_WORK         = 1'b1;

reg        read_state_reg;
reg        cur_param_valid_reg;
reg [11:0] read_out_row_cnt_reg;
reg [6:0]  read_out_col_cnt_reg;

reg [11:0] fm_out_row_reg;
reg [6:0]  fm_out_col_last_reg;

reg        update_seen_reg;
reg [13:0] cur_init_addr_reg;
reg [13:0] cur_valid_beats_m1_reg;
reg [13:0] cur_skip_step_reg;
reg [13:0] valid_cnt_reg;
reg [13:0] rdaddr_reg;
reg        out_valid_reg;
reg        out_valid_r1;
reg [3:0]  output_credit_count_reg;

wire        param_pop_fire;
wire [13:0] param_m_init_addr;
wire [13:0] param_m_valid_beats_m1;
wire [13:0] param_m_skip_step;

wire        read_row_last;
wire        read_col_last;
wire        read_fire;
wire        valid_cnt_last;
wire [14:0] addr_plus1_w;
wire [14:0] addr_plus_step_w;
wire [14:0] addr_next_ext;
wire        addr_wrap;
wire        first_param_need;
wire        wait_param_need;
wire        next_row_param_need;
wire        read_done_fire;
wire        output_credit_available;
wire        output_fire;
wire        output_fifo_s_ready;

assign ready_for_cmd         = (read_state_reg == READ_IDLE) && !cur_param_valid_reg;
assign param_pop_fire        = param_fifo_m_tvalid && param_fifo_m_tready;
assign param_m_init_addr     = param_fifo_m_tdata[41:28];
assign param_m_valid_beats_m1 = param_fifo_m_tdata[27:14];
assign param_m_skip_step     = param_fifo_m_tdata[13:0];

assign read_row_last         = (read_out_row_cnt_reg == (fm_out_row_reg - 12'd1));
assign read_col_last         = (read_out_col_cnt_reg == fm_out_col_last_reg);
assign read_fire             = (read_state_reg == READ_WORK) &&
                               cur_param_valid_reg &&
                               output_credit_available;
assign valid_cnt_last        = (valid_cnt_reg == cur_valid_beats_m1_reg);
// 并行加法后选择，替代先选步长再加法的串行链（时序整改点，功能等价）
assign addr_plus1_w        = {1'b0, rdaddr_reg} + 15'd1;
assign addr_plus_step_w    = {1'b0, rdaddr_reg} + {1'b0, cur_skip_step_reg};
assign addr_next_ext       = valid_cnt_last ? addr_plus_step_w : addr_plus1_w;
assign addr_wrap             = (addr_next_ext > {1'b0, weight_last_addr});
assign first_param_need      = (read_state_reg == READ_IDLE) && update_seen_reg;
assign wait_param_need       = (read_state_reg == READ_WORK) && !cur_param_valid_reg;
assign next_row_param_need   = read_fire && addr_wrap && read_col_last && !read_row_last;
assign param_fifo_m_tready   = first_param_need || wait_param_need || next_row_param_need;
assign read_done_fire        = read_fire && addr_wrap && read_col_last && read_row_last;

assign wrden                   = read_fire;
assign wrdaddr                 = rdaddr_reg;
assign output_credit_available = (output_credit_count_reg < 4'd8);
assign output_fire             = m_aixs_wetem_tvalid && m_aixs_wetem_tready;


// The WRAM has a fixed two-cycle response latency. Reserve one credit at each
// read issue and retire it only when the final AXI-stream word is accepted.
// The eight-entry FIFO therefore covers both RAM responses in flight and all
// queued output words without relying on an advisory almost-full threshold.
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        output_credit_count_reg <= 4'd0;
    else begin
        case ({read_fire, output_fire})
            2'b10: output_credit_count_reg <= output_credit_count_reg + 4'd1;
            2'b01: output_credit_count_reg <= output_credit_count_reg - 4'd1;
            default: output_credit_count_reg <= output_credit_count_reg;
        endcase
    end
end

sync_fifo_axis #(
    .WIDTH (512),
    .DEPTH (8),
    .PTR_W (3),
    .CNT_W (4)
) u_wetem_elastic_fifo (
    .clk           (clk),
    .rst_n         (rst_n),
    .s_axis_tvalid (out_valid_reg),
    .s_axis_tready (output_fifo_s_ready),
    .s_axis_tdata  (wrddata),
    .m_axis_tvalid (m_aixs_wetem_tvalid),
    .m_axis_tready (m_aixs_wetem_tready),
    .m_axis_tdata  (m_aixs_wetem_tdata),
    .data_count    ()
);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        read_state_reg <= READ_IDLE;
    else if (cmd_accept)
        read_state_reg <= READ_IDLE;
    else if (read_done_fire)
        read_state_reg <= READ_IDLE;
    else if ((read_state_reg == READ_IDLE) && param_pop_fire)
        read_state_reg <= READ_WORK;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        cur_param_valid_reg <= 1'b0;
    else if (cmd_accept)
        cur_param_valid_reg <= 1'b0;
    else if (read_done_fire)
        cur_param_valid_reg <= 1'b0;
    else if (read_fire && addr_wrap && read_col_last && !read_row_last)
        cur_param_valid_reg <= param_pop_fire;
    else if (param_pop_fire)
        cur_param_valid_reg <= 1'b1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        read_out_row_cnt_reg <= 12'd0;
    else if (cmd_accept)
        read_out_row_cnt_reg <= 12'd0;
    else if (read_fire && addr_wrap && read_col_last) begin
        if (read_row_last)
            read_out_row_cnt_reg <= 12'd0;
        else
            read_out_row_cnt_reg <= read_out_row_cnt_reg + 12'd1;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        read_out_col_cnt_reg <= 7'd0;
    else if (cmd_accept)
        read_out_col_cnt_reg <= 7'd0;
    else if (read_fire && addr_wrap) begin
        if (read_col_last)
            read_out_col_cnt_reg <= 7'd0;
        else
            read_out_col_cnt_reg <= read_out_col_cnt_reg + 7'd1;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        fm_out_row_reg <= 12'd0;
    else if (cmd_accept)
        fm_out_row_reg <= cmd_fm_out_row;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        fm_out_col_last_reg <= 7'd0;
    else if (cmd_accept)
        fm_out_col_last_reg <= cmd_fm_out_col;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        update_seen_reg <= 1'b0;
    else if (cmd_accept)
        update_seen_reg <= 1'b0;
    else if (read_done_fire)
        update_seen_reg <= 1'b0;
    else if (we_update)
        update_seen_reg <= 1'b1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        cur_init_addr_reg <= 14'd0;
    else if (cmd_accept)
        cur_init_addr_reg <= 14'd0;
    else if (param_pop_fire)
        cur_init_addr_reg <= param_m_init_addr;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        cur_valid_beats_m1_reg <= 14'd0;
    else if (cmd_accept)
        cur_valid_beats_m1_reg <= 14'd0;
    else if (param_pop_fire)
        cur_valid_beats_m1_reg <= param_m_valid_beats_m1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        cur_skip_step_reg <= 14'd1;
    else if (cmd_accept)
        cur_skip_step_reg <= 14'd1;
    else if (param_pop_fire)
        cur_skip_step_reg <= param_m_skip_step;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        valid_cnt_reg <= 14'd0;
    else if (cmd_accept)
        valid_cnt_reg <= 14'd0;
    else if (param_pop_fire)
        valid_cnt_reg <= 14'd0;
    else if (read_fire) begin
        if (addr_wrap)
            valid_cnt_reg <= 14'd0;
        else if (valid_cnt_last)
            valid_cnt_reg <= 14'd0;
        else
            valid_cnt_reg <= valid_cnt_reg + 14'd1;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        rdaddr_reg <= 14'd0;
    else if (cmd_accept)
        rdaddr_reg <= 14'd0;
    else if ((read_state_reg == READ_IDLE) && param_pop_fire)
        rdaddr_reg <= param_m_init_addr;
    else if ((read_state_reg == READ_WORK) && !cur_param_valid_reg && param_pop_fire)
        rdaddr_reg <= param_m_init_addr;
    else if (read_fire) begin
        if (addr_wrap) begin
            if (!read_col_last)
                rdaddr_reg <= cur_init_addr_reg;
            else if (!read_row_last && param_pop_fire)
                rdaddr_reg <= param_m_init_addr;
            else
                rdaddr_reg <= 14'd0;
        end
        else
            rdaddr_reg <= addr_next_ext[13:0];
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        out_valid_r1 <= 1'b0;
    else
        out_valid_r1 <= read_fire;
end

// RAM 换 BMG IP 后读延迟 2 拍（enb 常使能），wetem valid 打 2 级与 wrddata 对齐
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        out_valid_reg <= 1'b0;
    else
        out_valid_reg <= out_valid_r1;
end

endmodule
