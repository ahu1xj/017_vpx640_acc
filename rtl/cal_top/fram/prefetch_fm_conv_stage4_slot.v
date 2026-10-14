// Stage4 列 slot 展开级：按配置期派生的 slot_num 展开，起始列 = col_s + slot_start，
// 起始地址 = addr + win_init_off；每拍地址 += cin_pair_num、列号 += 1。
// 列无效打标不丢弃（70B 拼接级靠定拍计数切窗口，无效列块必须占一拍）。
// 输入 token: { addr[31:0], col_s[9:0], hilow, last } = 44bit
// 输出 token: { addr[31:0], hilow, col_valid, last } = 35bit
module prefetch_fm_conv_stage4_slot(
    input                   clk,
    input                   rst_n,

    input   [11:0]          cfg_cin_pair_num,
    input   [7:0]           cfg_input_col_blk_num,
    input   [31:0]          cfg_ring_size,
    input   [2:0]           cfg_slot_num,
    input   [1:0]           cfg_slot_start,
    input   [11:0]          cfg_win_init_off,

    input                   t3_empty,
    input   [43:0]          t3_rd_data,
    output                  t3_rd_en,

    input                   t4_full,
    output                  t4_wr_en,
    output  [34:0]          t4_wr_data
);

reg                busy_reg;
reg signed [31:0]  addr_reg;
reg signed [9:0]   col_s_reg;
reg                hilow_reg;
reg                last_reg;
reg [2:0]          slot_cnt_reg;

wire signed [9:0]  input_col_blk_num_s;
wire               col_valid;
wire               slot_last_cnt;
wire               load_fire;
wire               push_fire;
wire               last_out;
wire signed [16:0] ring_size_s;
wire signed [16:0] addr_load_sum;
wire signed [16:0] addr_load_wrap;
wire signed [16:0] addr_step_sum;
wire signed [16:0] addr_step_wrap;

assign input_col_blk_num_s = $signed({2'd0, cfg_input_col_blk_num});
assign col_valid     = (col_s_reg >= 10'sd0) && (col_s_reg < input_col_blk_num_s);
assign slot_last_cnt = (slot_cnt_reg == (cfg_slot_num - 3'd1));

// 零转身：父 token 末 slot 发射当拍即装载下一个父 token，子项流父间无缝。
assign load_fire = (!busy_reg || (push_fire && slot_last_cnt)) && !t3_empty;
assign push_fire = busy_reg && !t4_full;

assign last_out = last_reg && slot_last_cnt;

// 列偏移引入的回绕由本级就地吸收：行基地址以回绕形式存放时，
// 装载 win_init_off 或步进 cin_pair_num 可能把地址推过 ring_size；
// 入口地址 < ring_size 且步进 <= line_size，单次条件减足够；负地址不触发回绕。
// 时序整改：合法值域 addr ∈ [-2^14, 16384)（padding 区行为符号扩展负地址）、
// off/cin_pair <= 4095，和恒在 17b 有符号范围内；33b 运算截窄为 17b 保符号，
// CARRY 链级数减半；含负地址的合法输入下与旧实现逐 bit 等价。
assign ring_size_s   = $signed({2'b00, cfg_ring_size[14:0]});
assign addr_load_sum = $signed({t3_rd_data[27], t3_rd_data[27:12]}) + $signed({5'd0, cfg_win_init_off});
assign addr_load_wrap = (addr_load_sum >= ring_size_s)
                      ? (addr_load_sum - ring_size_s) : addr_load_sum;
assign addr_step_sum = $signed({addr_reg[15], addr_reg[15:0]}) + $signed({5'd0, cfg_cin_pair_num});
assign addr_step_wrap = (addr_step_sum >= ring_size_s)
                      ? (addr_step_sum - ring_size_s) : addr_step_sum;

assign t3_rd_en   = load_fire;
assign t4_wr_en   = push_fire;
assign t4_wr_data = {addr_reg, hilow_reg, col_valid, last_out};

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        busy_reg <= 1'b0;
    else if (load_fire)
        busy_reg <= 1'b1;
    else if (push_fire && slot_last_cnt)
        busy_reg <= 1'b0;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        addr_reg <= 32'sd0;
    else if (load_fire)
        addr_reg <= {{15{addr_load_wrap[16]}}, addr_load_wrap};
    else if (push_fire)
        addr_reg <= {{15{addr_step_wrap[16]}}, addr_step_wrap};
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        col_s_reg <= 10'sd0;
    else if (load_fire)
        col_s_reg <= $signed(t3_rd_data[11:2]) + $signed({8'd0, cfg_slot_start});
    else if (push_fire)
        col_s_reg <= col_s_reg + 10'sd1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        hilow_reg <= 1'b0;
    else if (load_fire)
        hilow_reg <= t3_rd_data[1];
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        last_reg <= 1'b0;
    else if (load_fire)
        last_reg <= t3_rd_data[0];
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        slot_cnt_reg <= 3'd0;
    else if (load_fire)
        slot_cnt_reg <= 3'd0;
    else if (push_fire)
        slot_cnt_reg <= slot_cnt_reg + 3'd1;
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
// [PASS] Rule #24: No multipliers; slot offset is addr += cin_pair_num accumulator
// [PASS] One register per always block
// [PASS] Declaration-before-use for all regs referenced by wire assigns
// ============================================================
