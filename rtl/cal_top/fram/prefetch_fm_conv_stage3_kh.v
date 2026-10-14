// Stage3 核高展开级：按 kernel_size_r 展开，地址步进恒为 line_size（跨行），
// 行号步进恒为 +1；行无效（纵向 padding）只 gate 写使能，不影响累加器步进。
// ring 回绕：依赖不变量 I1（入口地址在 [-cin_pair_num, ring_size)），
// 每步 += line_size 后有符号比较，单次条件减 ring_size；负地址永不触发回绕。
// 输入 token: { addr[31:0], row_s[15:0], col_s[9:0], hilow, last } = 60bit
// 输出 token: { addr[31:0], col_s[9:0], hilow, last } = 44bit
module prefetch_fm_conv_stage3_kh(
    input                   clk,
    input                   rst_n,

    input   [2:0]           cfg_kernel_size_r,
    input   [11:0]          cfg_fm_row_in,
    input   [31:0]          cfg_line_size,
    input   [31:0]          cfg_ring_size,

    input                   t2_empty,
    input   [59:0]          t2_rd_data,
    output                  t2_rd_en,

    input                   t3_full,
    output                  t3_wr_en,
    output  [43:0]          t3_wr_data
);

reg                busy_reg;
reg signed [31:0]  addr_reg;
reg signed [15:0]  row_s_reg;
reg [9:0]          col_s_reg;
reg                hilow_reg;
reg                last_reg;
reg [2:0]          kh_cnt_reg;

wire signed [15:0] fm_row_in_s;
wire               row_valid;
wire               kh_last_cnt;
wire               last_real;
wire               load_fire;
wire               push_fire;
wire               step_advance;
wire signed [32:0] addr_sum;
wire signed [32:0] ring_size_s;
wire signed [32:0] addr_wrap;
wire               last_out;

assign fm_row_in_s  = $signed({4'd0, cfg_fm_row_in});
assign row_valid    = (row_s_reg >= 16'sd0) && (row_s_reg < fm_row_in_s);
assign kh_last_cnt  = (kh_cnt_reg == (cfg_kernel_size_r - 3'd1));
assign last_real    = kh_last_cnt || ((row_s_reg + 16'sd1) >= fm_row_in_s);

// 零转身：父 token 末步（含无效行丢弃步）完成当拍即装载下一个父 token；
// KH 无效行语义不变——只 gate 写使能、累加器照常步进。
assign load_fire    = (!busy_reg || (step_advance && kh_last_cnt)) && !t2_empty;
assign push_fire    = busy_reg && row_valid && !t3_full;
assign step_advance = busy_reg && (!row_valid || !t3_full);

assign addr_sum    = {addr_reg[31], addr_reg} + $signed({1'b0, cfg_line_size});
assign ring_size_s = $signed({1'b0, cfg_ring_size});
assign addr_wrap   = (addr_sum >= ring_size_s) ? (addr_sum - ring_size_s) : addr_sum;

assign last_out = last_reg && last_real;

assign t2_rd_en   = load_fire;
assign t3_wr_en   = push_fire;
assign t3_wr_data = {addr_reg, col_s_reg, hilow_reg, last_out};

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        busy_reg <= 1'b0;
    else if (load_fire)
        busy_reg <= 1'b1;
    else if (step_advance && kh_last_cnt)
        busy_reg <= 1'b0;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        addr_reg <= 32'sd0;
    else if (load_fire)
        addr_reg <= t2_rd_data[59:28];
    else if (step_advance)
        addr_reg <= addr_wrap[31:0];
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        row_s_reg <= 16'sd0;
    else if (load_fire)
        row_s_reg <= t2_rd_data[27:12];
    else if (step_advance)
        row_s_reg <= row_s_reg + 16'sd1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        col_s_reg <= 10'd0;
    else if (load_fire)
        col_s_reg <= t2_rd_data[11:2];
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        hilow_reg <= 1'b0;
    else if (load_fire)
        hilow_reg <= t2_rd_data[1];
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        last_reg <= 1'b0;
    else if (load_fire)
        last_reg <= t2_rd_data[0];
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        kh_cnt_reg <= 3'd0;
    else if (load_fire)
        kh_cnt_reg <= 3'd0;
    else if (step_advance)
        kh_cnt_reg <= kh_cnt_reg + 3'd1;
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
// [PASS] Rule #24: No multipliers; kh offset is addr += line_size accumulator
// [PASS] One register per always block
// [PASS] Declaration-before-use for all regs referenced by wire assigns
// ============================================================
