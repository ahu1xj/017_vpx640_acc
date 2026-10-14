// prefetch_padding_pool: POOL padding 行供给。
// 遍历 ch_pair -> out_row -> kh（仅 padding 行）-> 行片段：
//   padding_left 个 16'h8080 -> 每 col_blk 读 1 word 出 32 个 16bit beat -> padding_right 个 16'h8080
// beat j = {odd[8j +: 8], even[8j +: 8]}（低字节偶通道、高字节奇通道，与 prefetch_fm_pool 一致）。
// 读地址 = region(top 4096/bottom 0) + row_base(phy_row) + col_base + ch_pair_cnt。
// 单 word buffer + 移位链预载：OUT32 期间预取下一 col_blk，稳态 1 beat/拍。
// 去重门控: 顶 pad 行只在 out_row==0 接受, 底 pad 行只在 row_last 接受——
//   SAME 关系 (out_h-1)*s+kh = in+pt+pb 保证全部顶行必落在 orow0 窗口内、
//   全部底行必落在末窗内 (pad_total<=kh-1 恒成立); stride_r=1 时重叠窗口会
//   重复命中同一 pad 行, 不门控则多供的行使 ppool FIFO 整体错位 (tc298 实证:
//   底环绕行吃到顶行重复份)。stride=2 窗口不重叠时各 pad 行本就只在首/末窗
//   出现一次, 门控前后行为逐拍一致。
module prefetch_padding_pool(
    input               clk,
    input               rst_n,

    input               pool_start,
    input      [11:0]   cfg_fm_row_in,
    input      [7:0]    cfg_col_blk_num,
    input      [11:0]   cfg_ch_pair_num,
    input      [19:0]   cfg_line_size,
    input      [2:0]    cfg_kernel_size_r,
    input      [2:0]    cfg_stride_r_real,
    input      [1:0]    cfg_padding_top,
    input      [1:0]    cfg_padding_bottom,
    input      [1:0]    cfg_padding_left,
    input      [1:0]    cfg_padding_right,
    input      [1:0]    cfg_top_row_offset,
    input      [11:0]   cfg_pool_out_row,
    output          rd_en,
    output  [11:0]  rd_addr,
    output          rd_region,
    input      [511:0]  rd_data,

    output              out_valid,
    output     [15:0]   out_data,
    input               out_ready,
    output              pool_done
);

localparam [2:0] P_SCAN    = 3'd0;
localparam [2:0] P_COLWAIT = 3'd1;
localparam [2:0] P_LPAD    = 3'd2;
localparam [2:0] P_OUT32   = 3'd3;
localparam [2:0] P_RPAD    = 3'd4;

// ---- 遍历/扫描寄存器 ----
reg                active_reg;
reg [2:0]          state_reg;
reg [11:0]         ch_pair_cnt_reg;
reg [11:0]         out_row_cnt_reg;
reg [2:0]          kh_reg;
reg signed [15:0]  origin_row_s_reg;
reg                scan_done_reg;

// ---- 当前行参数 ----
reg [1:0]          phy_row_reg;
reg                is_top_reg;
reg [11:0]         row_ch_pair_reg;

// ---- 取数引擎（单 word buffer） ----
reg                fetch_pend_reg;
reg [7:0]          fetch_col_cnt_reg;
reg [19:0]         fetch_col_base_reg;
reg                pend_reg;
reg                pend_d2_reg;
reg                pend_d3_reg;
reg                word_rdy_reg;
reg [511:0]        word_buf_reg;

// ---- 输出引擎 ----
reg [255:0]        even_shift_reg;
reg [255:0]        odd_shift_reg;
reg                shifts_loaded_reg;
reg [7:0]          out_col_cnt_reg;
reg [5:0]          beat_cnt_reg;
reg [1:0]          lpad_cnt_reg;
reg [1:0]          rpad_cnt_reg;

// ---- 组合信号 ----
wire signed [15:0] fm_row_in_s;
wire signed [15:0] in_row_s;
wire               is_top_pad;
wire               is_bot_pad;
wire               is_padding_row;
wire signed [15:0] top_pad_idx_s;
wire [1:0]         phy_row_next;
wire               kh_last;
wire               row_last;
wire               chp_last;
wire               scan_last_step;
wire               state_scan;
wire               state_colwait;
wire               state_lpad;
wire               state_out32;
wire               state_rpad;
wire               scan_step_fire;
wire               row_accept_fire;
wire [19:0]        row_base_cur;
wire [19:0]        fetch_addr_sum;
wire               fetch_issue_fire;
wire               shift_load_fire;
wire               lpad_last;
wire               lpad_done_fire;
wire               beat_last;
wire               out_col_last;
wire               word_fetch_more;
wire               rpad_last;
wire               rpad_done_fire;
wire               out_active;
wire               out_step_fire;
wire [15:0]        out_data_next;
wire               pool_done_body;

assign fm_row_in_s  = $signed({4'd0, cfg_fm_row_in});
assign in_row_s     = origin_row_s_reg + $signed({13'd0, kh_reg});
assign is_top_pad   = (in_row_s < 16'sd0);
assign is_bot_pad   = (in_row_s >= fm_row_in_s);
assign is_padding_row = is_top_pad || is_bot_pad;
assign top_pad_idx_s = in_row_s + $signed({14'd0, cfg_padding_top});
assign phy_row_next  = is_top_pad ? (top_pad_idx_s[1:0] + cfg_top_row_offset)
                                  : (in_row_s[1:0] - cfg_fm_row_in[1:0]);

assign kh_last  = (kh_reg == (cfg_kernel_size_r - 3'd1));
assign row_last = (out_row_cnt_reg == (cfg_pool_out_row - 12'd1));
assign chp_last = (ch_pair_cnt_reg == (cfg_ch_pair_num - 12'd1));
assign scan_last_step = kh_last && row_last && chp_last;

assign state_scan    = (state_reg == P_SCAN);
assign state_colwait = (state_reg == P_COLWAIT);
assign state_lpad    = (state_reg == P_LPAD);
assign state_out32   = (state_reg == P_OUT32);
assign state_rpad    = (state_reg == P_RPAD);

assign scan_step_fire = active_reg && state_scan && !scan_done_reg;
assign row_accept_fire = scan_step_fire && is_padding_row
                      && (is_top_pad ? (out_row_cnt_reg == 12'd0) : row_last);

assign row_base_cur = (phy_row_reg == 2'd0) ? 20'd0 :
                      (phy_row_reg == 2'd1) ? cfg_line_size :
                      {cfg_line_size[18:0], 1'b0};
assign fetch_addr_sum = {3'd0, row_base_cur} + fetch_col_base_reg
                      + {8'd0, row_ch_pair_reg};
assign rd_region = is_top_reg;

assign fetch_issue_fire = active_reg && fetch_pend_reg && !word_rdy_reg;
assign shift_load_fire  = active_reg && word_rdy_reg && !state_out32;

assign lpad_last      = (lpad_cnt_reg == (cfg_padding_left - 2'd1));
assign lpad_done_fire = active_reg && state_lpad && out_ready && lpad_last;
assign beat_last      = (beat_cnt_reg == 6'd31);
assign out_col_last   = (out_col_cnt_reg == (cfg_col_blk_num - 8'd1));
assign word_fetch_more = (fetch_col_cnt_reg < cfg_col_blk_num);
assign rpad_last      = (rpad_cnt_reg == (cfg_padding_right - 2'd1));
assign rpad_done_fire = active_reg && state_rpad && out_ready && rpad_last;

assign out_active    = active_reg && (state_lpad || state_out32 || state_rpad);
assign out_step_fire = out_active && out_ready;
assign out_data_next = state_out32 ? {odd_shift_reg[7:0], even_shift_reg[7:0]}
                                   : 16'h8080;

assign rd_en     = fetch_issue_fire;
assign rd_addr   = fetch_addr_sum[11:0];
assign out_valid = out_active;
assign out_data  = out_data_next;
assign pool_done_body = scan_done_reg && state_scan && !row_accept_fire;
assign pool_done = active_reg && pool_done_body;

// ---- 任务活跃边界 ----
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        active_reg <= 1'b0;
    else if (pool_start)
        active_reg <= 1'b1;
    else if (pool_done_body)
        active_reg <= 1'b0;
end

// ---- 主状态机 ----
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        state_reg <= P_SCAN;
    else if (pool_start)
        state_reg <= P_SCAN;
    else begin
        case (state_reg)
            P_SCAN:
                if (row_accept_fire) begin
                    if (cfg_padding_left != 2'd0)
                        state_reg <= P_LPAD;
                    else
                        state_reg <= P_COLWAIT;
                end
                else
                    state_reg <= P_SCAN;
            P_COLWAIT:
                if (shifts_loaded_reg)
                    state_reg <= P_OUT32;
                else
                    state_reg <= P_COLWAIT;
            P_LPAD:
                if (lpad_done_fire) begin
                    if (shifts_loaded_reg)
                        state_reg <= P_OUT32;
                    else
                        state_reg <= P_COLWAIT;
                end
                else
                    state_reg <= P_LPAD;
            P_OUT32:
                if (out_step_fire && beat_last && out_col_last) begin
                    if (cfg_padding_right != 2'd0)
                        state_reg <= P_RPAD;
                    else
                        state_reg <= P_SCAN;
                end
                else if (out_step_fire && beat_last && !word_rdy_reg)
                    state_reg <= P_COLWAIT;
                else
                    state_reg <= P_OUT32;
            P_RPAD:
                if (rpad_done_fire)
                    state_reg <= P_SCAN;
                else
                    state_reg <= P_RPAD;
            default:
                state_reg <= P_SCAN;
        endcase
    end
end

// ---- 遍历计数器组：kh -> out_row -> ch_pair ----
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        kh_reg <= 3'd0;
    else if (pool_start)
        kh_reg <= 3'd0;
    else if (scan_step_fire)
        kh_reg <= kh_last ? 3'd0 : (kh_reg + 3'd1);
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        out_row_cnt_reg <= 12'd0;
    else if (pool_start)
        out_row_cnt_reg <= 12'd0;
    else if (scan_step_fire && kh_last)
        out_row_cnt_reg <= row_last ? 12'd0 : (out_row_cnt_reg + 12'd1);
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        ch_pair_cnt_reg <= 12'd0;
    else if (pool_start)
        ch_pair_cnt_reg <= 12'd0;
    else if (scan_step_fire && kh_last && row_last && !chp_last)
        ch_pair_cnt_reg <= ch_pair_cnt_reg + 12'd1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        origin_row_s_reg <= 16'sd0;
    else if (pool_start)
        origin_row_s_reg <= -$signed({14'd0, cfg_padding_top});
    else if (scan_step_fire && kh_last) begin
        if (row_last)
            origin_row_s_reg <= -$signed({14'd0, cfg_padding_top});
        else
            origin_row_s_reg <= origin_row_s_reg + $signed({13'd0, cfg_stride_r_real});
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        scan_done_reg <= 1'b0;
    else if (pool_start)
        scan_done_reg <= 1'b0;
    else if (scan_step_fire && scan_last_step)
        scan_done_reg <= 1'b1;
end

// ---- 行参数锁存 ----
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        phy_row_reg     <= 2'd0;
        is_top_reg      <= 1'b0;
        row_ch_pair_reg <= 12'd0;
    end
    else if (pool_start) begin
        phy_row_reg     <= 2'd0;
        is_top_reg      <= 1'b0;
        row_ch_pair_reg <= 12'd0;
    end
    else if (row_accept_fire) begin
        phy_row_reg     <= phy_row_next;
        is_top_reg      <= is_top_pad;
        row_ch_pair_reg <= ch_pair_cnt_reg;
    end
end

// ---- 取数引擎 ----
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        fetch_pend_reg <= 1'b0;
    else if (pool_start)
        fetch_pend_reg <= 1'b0;
    else if (fetch_issue_fire)
        fetch_pend_reg <= 1'b0;
    else if (row_accept_fire)
        fetch_pend_reg <= 1'b1;
    else if (shift_load_fire && word_fetch_more)
        fetch_pend_reg <= 1'b1;
    else if (out_step_fire && state_out32 && beat_last && !out_col_last && word_rdy_reg && word_fetch_more)
        fetch_pend_reg <= 1'b1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        fetch_col_cnt_reg <= 8'd0;
    else if (pool_start)
        fetch_col_cnt_reg <= 8'd0;
    else if (row_accept_fire)
        fetch_col_cnt_reg <= 8'd0;
    else if (fetch_issue_fire)
        fetch_col_cnt_reg <= fetch_col_cnt_reg + 8'd1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        fetch_col_base_reg <= 20'd0;
    else if (pool_start)
        fetch_col_base_reg <= 20'd0;
    else if (row_accept_fire)
        fetch_col_base_reg <= 20'd0;
    else if (fetch_issue_fire)
        fetch_col_base_reg <= fetch_col_base_reg + {8'd0, cfg_ch_pair_num};
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        pend_reg <= 1'b0;
    else if (pool_start)
        pend_reg <= 1'b0;
    else
        pend_reg <= fetch_issue_fire;
end

// 读口在顶层寄存 1 拍，BMG 再 2 拍，数据在 issue 后第 3 拍装入
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        pend_d2_reg <= 1'b0;
    else if (pool_start)
        pend_d2_reg <= 1'b0;
    else
        pend_d2_reg <= pend_reg;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        pend_d3_reg <= 1'b0;
    else if (pool_start)
        pend_d3_reg <= 1'b0;
    else
        pend_d3_reg <= pend_d2_reg;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        word_rdy_reg <= 1'b0;
    else if (pool_start)
        word_rdy_reg <= 1'b0;
    else if (shift_load_fire)
        word_rdy_reg <= 1'b0;
    else if (out_step_fire && state_out32 && beat_last && !out_col_last && word_rdy_reg)
        word_rdy_reg <= 1'b0;
    else if (pend_d3_reg)
        word_rdy_reg <= 1'b1;
end

always @(posedge clk) begin
    if (pend_d3_reg)
        word_buf_reg <= rd_data;
end

// ---- 输出移位链 ----
always @(posedge clk) begin
    if (shift_load_fire)
        even_shift_reg <= word_buf_reg[255:0];
    else if (out_step_fire && state_out32 && beat_last && !out_col_last && word_rdy_reg)
        even_shift_reg <= word_buf_reg[255:0];
    else if (out_step_fire && state_out32 && !beat_last)
        even_shift_reg <= {8'd0, even_shift_reg[255:8]};
end

always @(posedge clk) begin
    if (shift_load_fire)
        odd_shift_reg <= word_buf_reg[511:256];
    else if (out_step_fire && state_out32 && beat_last && !out_col_last && word_rdy_reg)
        odd_shift_reg <= word_buf_reg[511:256];
    else if (out_step_fire && state_out32 && !beat_last)
        odd_shift_reg <= {8'd0, odd_shift_reg[255:8]};
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        shifts_loaded_reg <= 1'b0;
    else if (pool_start)
        shifts_loaded_reg <= 1'b0;
    else if (shift_load_fire)
        shifts_loaded_reg <= 1'b1;
    else if (state_colwait && shifts_loaded_reg)
        shifts_loaded_reg <= 1'b0;
    else if (lpad_done_fire && shifts_loaded_reg)
        shifts_loaded_reg <= 1'b0;
end

// ---- 输出计数器 ----
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        lpad_cnt_reg <= 2'd0;
    else if (pool_start)
        lpad_cnt_reg <= 2'd0;
    else if (state_lpad && out_ready)
        lpad_cnt_reg <= lpad_last ? 2'd0 : (lpad_cnt_reg + 2'd1);
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        rpad_cnt_reg <= 2'd0;
    else if (pool_start)
        rpad_cnt_reg <= 2'd0;
    else if (state_rpad && out_ready)
        rpad_cnt_reg <= rpad_last ? 2'd0 : (rpad_cnt_reg + 2'd1);
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        beat_cnt_reg <= 6'd0;
    else if (pool_start)
        beat_cnt_reg <= 6'd0;
    else if (state_out32 && out_ready)
        beat_cnt_reg <= beat_last ? 6'd0 : (beat_cnt_reg + 6'd1);
    else if (state_colwait)
        beat_cnt_reg <= 6'd0;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        out_col_cnt_reg <= 8'd0;
    else if (pool_start)
        out_col_cnt_reg <= 8'd0;
    else if (row_accept_fire)
        out_col_cnt_reg <= 8'd0;
    else if (out_step_fire && state_out32 && beat_last && !out_col_last)
        out_col_cnt_reg <= out_col_cnt_reg + 8'd1;
end

// ---- 输出寄存器 ----
endmodule

// ============================================================
// RTL Self-Check Results
// ============================================================
// [PASS] Rule #1:  All ports have explicit width declarations
// [PASS] Rule #2:  Ports use Verilog wire/reg conventions
// [PASS] Rule #6:  No debug/error-attribution logic or diagnostic-only sideband in RTL
// [PASS] Rule #8:  No logic/always_ff/always_comb used
// [PASS] Rule #12: FSM case has default; no combinational always blocks
// [PASS] Rule #16: Sequential logic uses non-blocking '<=' only
// [PASS] Rule #24: No multiplier in run path; fetch addresses use running accumulator
// [PASS] Rule #26: FSM states use localparam explicit encoding
// [PASS] Rule #43: No function/task used
// [PASS] Start boundary: scan/read/output/done are inactive before pool_start;
//        active gating adds no normal-run pipeline stage
// [PASS] Control metadata registers have explicit reset; wide data path registers are not reset
// ============================================================
