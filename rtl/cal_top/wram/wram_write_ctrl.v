`include "all_defines.vh"

module wram_write_ctrl(
    input  wire             clk,
    input  wire             rst_n,
    input  wire             rdcmd_trans_valid,
    input  wire [111:0]     rdcmd_trans,
    input  wire             s_axis_we_tvalid,
    input  wire [511:0]     s_axis_we_tdata,
    output wire             s_axis_we_tready,
    output wire             wren,
    output wire [13:0]      wraddr,
    output wire [511:0]     wrdata,
    output wire             we_update
);

wire [`RD_MODEL_SEL_WIDTH-1:0]         cmd_mode_sel;
wire [`RD_FM_CHANNEL_IN_WIDTH-1:0]     cmd_channel_in;
wire [`RD_CONV_KERNEL_SIZE_R_WIDTH-1:0] cmd_kernel_r;
wire [`RD_CONV_KERNEL_SIZE_C_WIDTH-1:0] cmd_kernel_c;
wire                                    cmd_is_conv;

reg [11:0] channel_in_r;
reg [5:0]  krkc_r;
reg        cfg_s1_r;
reg        cfg_s2_r;
reg [19:0] total_words_reg;
reg [19:0] word_cnt_reg;
reg [13:0] addr_cnt_reg;
reg        tready_reg;
reg        wren_reg;
reg [13:0] wraddr_reg;
reg [511:0] wrdata_reg;
reg        we_update_d0;
reg        we_update_d1;

wire handshake;
wire last_word;

assign cmd_mode_sel   = rdcmd_trans[`RD_MODEL_SEL_START +: `RD_MODEL_SEL_WIDTH];
assign cmd_channel_in = rdcmd_trans[`RD_FM_CHANNEL_IN_START +: `RD_FM_CHANNEL_IN_WIDTH];
assign cmd_kernel_r   = rdcmd_trans[`RD_CONV_KERNEL_SIZE_R_START +: `RD_CONV_KERNEL_SIZE_R_WIDTH];
assign cmd_kernel_c   = rdcmd_trans[`RD_CONV_KERNEL_SIZE_C_START +: `RD_CONV_KERNEL_SIZE_C_WIDTH];
assign cmd_is_conv    = rdcmd_trans_valid && (cmd_mode_sel == `MODEL_SEL_CONV);

assign s_axis_we_tready = tready_reg;
assign handshake        = s_axis_we_tvalid && s_axis_we_tready;
assign last_word        = handshake && (word_cnt_reg == (total_words_reg - 20'd1));

assign wren      = wren_reg;
assign wraddr    = wraddr_reg;
assign wrdata    = wrdata_reg;
assign we_update = we_update_d0 || we_update_d1;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        channel_in_r <= 12'd0;
    else if (cmd_is_conv)
        channel_in_r <= cmd_channel_in;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        krkc_r <= 6'd0;
    else if (cmd_is_conv)
        krkc_r <= {3'd0, cmd_kernel_r} * {3'd0, cmd_kernel_c};
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        cfg_s1_r <= 1'b0;
    else
        cfg_s1_r <= cmd_is_conv;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        cfg_s2_r <= 1'b0;
    else
        cfg_s2_r <= cfg_s1_r;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        total_words_reg <= 20'd0;
    else if (cfg_s1_r)
        total_words_reg <= {8'd0, channel_in_r} * {14'd0, krkc_r};
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        word_cnt_reg <= 20'd0;
    else if (cmd_is_conv)
        word_cnt_reg <= 20'd0;
    else if (last_word)
        word_cnt_reg <= 20'd0;
    else if (handshake)
        word_cnt_reg <= word_cnt_reg + 20'd1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        addr_cnt_reg <= 14'd0;
    else if (cmd_is_conv)
        addr_cnt_reg <= 14'd0;
    else if (last_word)
        addr_cnt_reg <= 14'd0;
    else if (handshake)
        addr_cnt_reg <= addr_cnt_reg + 14'd1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        tready_reg <= 1'b0;
    else if (cfg_s2_r)
        tready_reg <= (total_words_reg != 20'd0);
    else if (last_word)
        tready_reg <= 1'b0;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        wren_reg <= 1'b0;
    else
        wren_reg <= handshake;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        wraddr_reg <= 14'd0;
    else if (handshake)
        wraddr_reg <= addr_cnt_reg;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        wrdata_reg <= 512'd0;
    else if (handshake)
        wrdata_reg <= s_axis_we_tdata;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        we_update_d0 <= 1'b0;
    else
        we_update_d0 <= last_word;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        we_update_d1 <= 1'b0;
    else
        we_update_d1 <= we_update_d0;
end

endmodule

// ============================================================
// RTL Self-Check Results
// ============================================================
// [PASS] 端口方向和位宽保持与旧 wram_write_ctrl 接口一致
// [PASS] 使用 all_defines.vh 中的旧项目字段宏，未在模块内重复硬编码字段位置
// [PASS] 纯 Verilog RTL，未使用 logic/always_ff/always_comb/function/task
// [PASS] 组合逻辑只使用 assign，时序逻辑只使用非阻塞赋值
// [PASS] 每个寄存器由独立 always 块驱动
// [PASS] 未增加 debug 端口、诊断 sideband 或内部错误上报逻辑
// [WARN] 使用乘法计算权重 beat 数：已改为指令分级流水（kr*kc 与 ch 打拍后
//        再相乘），寄存器进寄存器出；total_words/tready 较旧实现晚 2 拍有效，
//        远早于权重数据到达，契约安全
// ============================================================
