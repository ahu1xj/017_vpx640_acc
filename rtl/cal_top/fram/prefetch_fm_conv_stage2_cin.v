// Stage2 通道对展开级：FRAM 一个 word 含两个输入通道，通道对维是地址最内连续维，
// 所以通道展开 = 地址自增 + 1bit 高低通道指示。
// 输入 token: { virt_addr[31:0], row_s[15:0], col_s[9:0], last } = 59bit
// 输出 token: { addr[31:0], row_s[15:0], col_s[9:0], hilow, last } = 60bit
module prefetch_fm_conv_stage2_cin(
    input           clk,
    input           rst_n,

    input   [11:0]  cfg_cin_pair_num,

    input           t1_empty,
    input   [58:0]  t1_rd_data,
    output          t1_rd_en,

    input           t2_full,
    output          t2_wr_en,
    output  [59:0]  t2_wr_data
);

reg                busy_reg;
reg [31:0]         addr_reg;
reg [15:0]         row_s_reg;
reg [9:0]          col_s_reg;
reg                last_reg;
reg [11:0]         pair_cnt_reg;
reg                hilow_reg;

wire               load_fire;
wire               push_fire;
wire               exp_last;
wire               last_out;

// 零转身：父 token 末子项发射当拍即装载下一个父 token（show-ahead FIFO 直采），
// 子项流在父间无缝衔接；下游满时 push 停顿，装载自然让位。
assign load_fire = (!busy_reg || (push_fire && exp_last)) && !t1_empty;
assign push_fire = busy_reg && !t2_full;
assign exp_last  = (pair_cnt_reg == (cfg_cin_pair_num - 12'd1)) && hilow_reg;
assign last_out  = last_reg && exp_last;

assign t1_rd_en   = load_fire;
assign t2_wr_en   = push_fire;
assign t2_wr_data = {addr_reg, row_s_reg, col_s_reg, hilow_reg, last_out};

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        busy_reg <= 1'b0;
    else if (load_fire)
        busy_reg <= 1'b1;
    else if (push_fire && exp_last)
        busy_reg <= 1'b0;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        addr_reg <= 32'd0;
    else if (load_fire)
        addr_reg <= t1_rd_data[58:27];
    else if (push_fire && hilow_reg)
        addr_reg <= addr_reg + 32'd1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        row_s_reg <= 16'd0;
    else if (load_fire)
        row_s_reg <= t1_rd_data[26:11];
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        col_s_reg <= 10'd0;
    else if (load_fire)
        col_s_reg <= t1_rd_data[10:1];
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        last_reg <= 1'b0;
    else if (load_fire)
        last_reg <= t1_rd_data[0];
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        pair_cnt_reg <= 12'd0;
    else if (load_fire)
        pair_cnt_reg <= 12'd0;
    else if (push_fire && hilow_reg)
        pair_cnt_reg <= pair_cnt_reg + 12'd1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        hilow_reg <= 1'b0;
    else if (load_fire)
        hilow_reg <= 1'b0;
    else if (push_fire)
        hilow_reg <= ~hilow_reg;
end

endmodule

// ============================================================
// RTL Self-Check Results
// ============================================================
// [PASS] Rule #1:  All ports have explicit width declarations
// [PASS] Rule #2:  Ports use Verilog wire/reg conventions
// [PASS] Rule #6:  No debug/error-attribution logic or diagnostic-only sideband in RTL
// [PASS] Rule #8:  No logic/always_ff/always_comb used
// [PASS] Rule #12: All always blocks hold state when not written (no latch)
// [PASS] Rule #16: Non-blocking '<=' for all sequential logic
// [PASS] Rule #24: No multipliers; channel expansion is addr+1 only
// [PASS] One register per always block
// [PASS] Declaration-before-use for all regs referenced by wire assigns
// ============================================================
