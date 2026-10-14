// prefetch_padding_conv: CONV padding 行窗口供给。
// 遍历与 prefetch_fm_conv 一致（out_row -> out_col_blk -> cin -> kh -> kw），
// 只把 kh 限制在 padding 行（真实行 1 拍跳过）。
// 结构：scan（遍历/参数）+ 读引擎（4 slot 背靠背）+ 输出引擎（窗口 beat）三轨，
// 双 chunk bank 使读引擎在输出期间预取下一窗口，稳态 max(4, kc) 拍/窗口。
// 窗口格式与取窗公式同 prefetch_fm_conv：
//   window = {slot0[255:232], slot1, slot2, slot3[23:0]}
//   beat(kw) byte j = window[8*(byte_start + kw + j*stride_c_real) +: 8]
module prefetch_padding_conv(
    input               clk,
    input               rst_n,

    input               conv_start,
    input      [11:0]   cfg_fm_channel_in,
    input      [11:0]   cfg_fm_row_in,
    input      [7:0]    cfg_col_blk_num,
    input      [11:0]   cfg_ch_pair_num,
    input      [19:0]   cfg_line_size,
    input      [2:0]    cfg_kernel_size_r,
    input      [2:0]    cfg_kernel_size_c,
    input      [2:0]    cfg_stride_r_real,
    input      [2:0]    cfg_stride_c_real,
    input      [1:0]    cfg_padding_top,
    input      [1:0]    cfg_top_row_offset,
    input      [2:0]    cfg_byte_start,
    input      [11:0]   cfg_fm_out_row,
    input      [6:0]    cfg_fm_out_col,
    output          rd_en,
    output  [11:0]  rd_addr,
    output          rd_region,
    input      [511:0]  rd_data,

    output              out_valid,
    output     [255:0]  out_data,
    input               out_ready,
    output              conv_done
);

localparam [1:0] I_IDLE = 2'd0;
localparam [1:0] I_READ = 2'd1;
localparam [1:0] O_WAIT = 2'd0;
localparam [1:0] O_OUT  = 2'd1;

// ---- 遍历/扫描寄存器 ----
reg                active_reg;
reg [11:0]         out_row_cnt_reg;
reg [6:0]          out_col_blk_reg;
reg [11:0]         cin_reg;
reg [2:0]          kh_reg;
reg signed [15:0]  origin_row_s_reg;
reg signed [9:0]   left_col_s_reg;
reg signed [21:0]  left_off_s_reg;
reg                scan_done_reg;

// ---- 窗口参数（next/cur 两级） ----
reg                win_next_valid_reg;
reg [1:0]          win_next_phy_row_reg;
reg                win_next_is_top_reg;
reg signed [21:0]  win_next_left_off_s_reg;
reg signed [9:0]   win_next_left_col_s_reg;
reg [11:0]         win_next_cp_reg;
reg                win_next_cin_odd_reg;
reg [1:0]          win_cur_phy_row_reg;
reg                win_cur_is_top_reg;
reg signed [21:0]  win_cur_left_off_s_reg;
reg signed [9:0]   win_cur_left_col_s_reg;
reg [11:0]         win_cur_cp_reg;
reg                win_cur_cin_odd_reg;

// ---- 读引擎 ----
reg [1:0]          issue_state_reg;
reg [1:0]          slot_cnt_reg;
reg signed [21:0]  slot_off_s_reg;
reg                prod_bank_reg;
reg                pend_reg;
reg [1:0]          pend_slot_reg;
reg                pend_bank_reg;
reg                pend_zero_reg;
reg                pend_odd_reg;
reg                pend_d2_reg;
reg [1:0]          pend_slot_d2_reg;
reg                pend_bank_d2_reg;
reg                pend_zero_d2_reg;
reg                pend_odd_d2_reg;
reg                pend_d3_reg;
reg [1:0]          pend_slot_d3_reg;
reg                pend_bank_d3_reg;
reg                pend_zero_d3_reg;
reg                pend_odd_d3_reg;
reg [255:0]        chunk_bank0_slot0_reg;
reg [255:0]        chunk_bank0_slot1_reg;
reg [255:0]        chunk_bank0_slot2_reg;
reg [255:0]        chunk_bank0_slot3_reg;
reg [255:0]        chunk_bank1_slot0_reg;
reg [255:0]        chunk_bank1_slot1_reg;
reg [255:0]        chunk_bank1_slot2_reg;
reg [255:0]        chunk_bank1_slot3_reg;
reg                bank_rdy0_reg;
reg                bank_rdy1_reg;

// ---- 输出引擎 ----
reg [1:0]          out_state_reg;
reg                cons_bank_reg;
reg [2:0]          kw_reg;

// ---- 组合信号 ----
wire               scan_stall;
wire signed [15:0] fm_row_in_s;
wire signed [15:0] in_row_s;
wire               is_top_pad;
wire               is_bot_pad;
wire               is_padding_row;
wire signed [15:0] top_pad_idx_s;
wire [1:0]         phy_row_next;
wire               kh_last;
wire               cin_last;
wire               col_last;
wire               row_last;
wire               scan_last_step;
wire               scan_step_fire;
wire [21:0]        col_step_off;
wire               issue_state_read;
wire               bank_rdy_prod;
wire               win_take_fire;
wire               slot_last;
wire               chain_fire;
wire signed [9:0]  slot_cb_s;
wire               slot_valid;
wire [19:0]        row_base_win;
wire [21:0]        slot_addr_sum;
wire [255:0]       capture_chunk;
wire               pend_fire;
wire               pend_last_fire;
wire               prod_set_rdy0;
wire               prod_set_rdy1;
wire               out_state_out;
wire               kw_last;
wire               out_step_fire;
wire               out_drain_fire;
wire               out_chain_fire;
wire               cons_clear_rdy0;
wire               cons_clear_rdy1;
wire [255:0]       cons_slot0;
wire [255:0]       cons_slot1;
wire [255:0]       cons_slot2;
wire [255:0]       cons_slot3;
wire [559:0]       window_data;
wire [4:0]         gather_base;
wire [255:0]       out_data_comb;
wire               conv_done_body;
genvar             gl;

// ================= 扫描（遍历 + 参数锁存） =================
assign fm_row_in_s  = $signed({4'd0, cfg_fm_row_in});
assign in_row_s     = origin_row_s_reg + $signed({13'd0, kh_reg});
assign is_top_pad   = (in_row_s < 16'sd0);
assign is_bot_pad   = (in_row_s >= fm_row_in_s);
assign is_padding_row = is_top_pad || is_bot_pad;
assign top_pad_idx_s = in_row_s + $signed({14'd0, cfg_padding_top});
assign phy_row_next  = is_top_pad ? (top_pad_idx_s[1:0] + cfg_top_row_offset)
                                  : (in_row_s[1:0] - cfg_fm_row_in[1:0]);

assign kh_last  = (kh_reg == (cfg_kernel_size_r - 3'd1));
assign cin_last = (cin_reg == (cfg_fm_channel_in - 12'd1));
assign col_last = (out_col_blk_reg == cfg_fm_out_col);
assign row_last = (out_row_cnt_reg == (cfg_fm_out_row - 12'd1));
assign scan_last_step = kh_last && cin_last && col_last && row_last;

assign scan_stall    = win_next_valid_reg;
assign scan_step_fire = active_reg && !scan_done_reg && !scan_stall;
assign col_step_off  = (cfg_stride_c_real == 3'd2)
                     ? {9'd0, cfg_ch_pair_num, 1'b0}
                     : {10'd0, cfg_ch_pair_num};

// ================= 读引擎 =================
assign issue_state_read = active_reg && (issue_state_reg == I_READ);
assign bank_rdy_prod    = prod_bank_reg ? bank_rdy1_reg : bank_rdy0_reg;
assign win_take_fire    = active_reg && !issue_state_read && win_next_valid_reg &&
                          !bank_rdy_prod;
assign slot_last        = (slot_cnt_reg == 2'd3);
assign chain_fire       = active_reg && issue_state_read && slot_last && win_next_valid_reg &&
                          !(prod_bank_reg ? bank_rdy0_reg : bank_rdy1_reg);

assign slot_cb_s   = (issue_state_read ? win_cur_left_col_s_reg : win_next_left_col_s_reg)
                   + $signed({8'd0, (issue_state_read ? slot_cnt_reg : 2'd0)});
assign slot_valid  = (slot_cb_s >= 10'sd0) &&
                     (slot_cb_s < $signed({2'd0, cfg_col_blk_num}));
assign row_base_win = (issue_state_read ?
                       ((win_cur_phy_row_reg == 2'd0) ? 20'd0 :
                        (win_cur_phy_row_reg == 2'd1) ? cfg_line_size :
                        {cfg_line_size[18:0], 1'b0}) :
                       ((win_next_phy_row_reg == 2'd0) ? 20'd0 :
                        (win_next_phy_row_reg == 2'd1) ? cfg_line_size :
                        {cfg_line_size[18:0], 1'b0}));
assign slot_addr_sum = {11'd0, row_base_win}
                     + (issue_state_read ? slot_off_s_reg : win_next_left_off_s_reg)
                     + {10'd0, (issue_state_read ? win_cur_cp_reg : win_next_cp_reg)};
assign rd_region = issue_state_read ? win_cur_is_top_reg : win_next_is_top_reg;

assign rd_en = active_reg && ((win_take_fire && slot_valid) ||
               (issue_state_read && slot_valid));
assign rd_addr = slot_addr_sum[11:0];

// 读口在顶层寄存 1 拍 + BMG 2 拍，捕获流水线共 3 拍（issue -> prden -> 数据 -> 装入）
assign capture_chunk  = pend_odd_d3_reg ? rd_data[511:256] : rd_data[255:0];
assign pend_fire      = pend_d3_reg;
assign pend_last_fire = pend_fire && (pend_slot_d3_reg == 2'd3);
assign prod_set_rdy0  = pend_last_fire && !pend_bank_d3_reg;
assign prod_set_rdy1  = pend_last_fire && pend_bank_d3_reg;

// ================= 输出引擎 =================
assign out_state_out  = (out_state_reg == O_OUT);
assign kw_last        = (kw_reg == (cfg_kernel_size_c - 3'd1));
assign out_step_fire  = active_reg && out_state_out && out_ready;
assign out_drain_fire = out_step_fire && kw_last;
assign out_chain_fire = out_drain_fire && (cons_bank_reg ? bank_rdy0_reg : bank_rdy1_reg);
assign cons_clear_rdy0 = out_drain_fire && !cons_bank_reg;
assign cons_clear_rdy1 = out_drain_fire && cons_bank_reg;

assign cons_slot0 = cons_bank_reg ? chunk_bank1_slot0_reg : chunk_bank0_slot0_reg;
assign cons_slot1 = cons_bank_reg ? chunk_bank1_slot1_reg : chunk_bank0_slot1_reg;
assign cons_slot2 = cons_bank_reg ? chunk_bank1_slot2_reg : chunk_bank0_slot2_reg;
assign cons_slot3 = cons_bank_reg ? chunk_bank1_slot3_reg : chunk_bank0_slot3_reg;

assign window_data[23:0]    = cons_slot0[255:232];
assign window_data[279:24]  = cons_slot1;
assign window_data[535:280] = cons_slot2;
assign window_data[559:536] = cons_slot3[23:0];

assign out_valid = active_reg && out_state_out;
assign out_data  = out_data_comb;

assign conv_done_body = scan_done_reg && !win_next_valid_reg && !issue_state_read &&
                        !pend_reg && !pend_d2_reg && !pend_d3_reg &&
                        !bank_rdy0_reg && !bank_rdy1_reg && !out_state_out;
assign conv_done = active_reg && conv_done_body;

// 取窗显式拼接（FRAM-008 口径：不放行组合 procedural for；原 lane*stride 乘法消除）。
// beat(kw) 第 gl 字节 = window_data 第 (byte_start + kw + (stride==2 ? 2gl : gl)) 字节。
assign gather_base = {2'd0, cfg_byte_start} + {2'd0, kw_reg};

generate
    for (gl = 0; gl < 32; gl = gl + 1) begin: out_byte_gather
        assign out_data_comb[(8*gl) +: 8] =
            window_data[((cfg_stride_c_real == 3'd2) ?
                         (({1'b0, gather_base} + gl + gl) << 3) :
                         (({1'b0, gather_base} + gl) << 3)) +: 8];
    end
endgenerate

// ---- 任务活跃边界 ----
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        active_reg <= 1'b0;
    else if (conv_start)
        active_reg <= 1'b1;
    else if (conv_done_body)
        active_reg <= 1'b0;
end

// ---- 遍历计数器组：kh -> cin -> out_col_blk -> out_row ----
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        kh_reg <= 3'd0;
    else if (conv_start)
        kh_reg <= 3'd0;
    else if (scan_step_fire)
        kh_reg <= kh_last ? 3'd0 : (kh_reg + 3'd1);
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        cin_reg <= 12'd0;
    else if (conv_start)
        cin_reg <= 12'd0;
    else if (scan_step_fire && kh_last)
        cin_reg <= cin_last ? 12'd0 : (cin_reg + 12'd1);
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        out_col_blk_reg <= 7'd0;
    else if (conv_start)
        out_col_blk_reg <= 7'd0;
    else if (scan_step_fire && kh_last && cin_last)
        out_col_blk_reg <= col_last ? 7'd0 : (out_col_blk_reg + 7'd1);
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        out_row_cnt_reg <= 12'd0;
    else if (conv_start)
        out_row_cnt_reg <= 12'd0;
    else if (scan_step_fire && kh_last && cin_last && col_last && !row_last)
        out_row_cnt_reg <= out_row_cnt_reg + 12'd1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        origin_row_s_reg <= 16'sd0;
    else if (conv_start)
        origin_row_s_reg <= -$signed({14'd0, cfg_padding_top});
    else if (scan_step_fire && kh_last && cin_last && col_last && !row_last)
        origin_row_s_reg <= origin_row_s_reg + $signed({13'd0, cfg_stride_r_real});
end

// left_col_blk = -1 + out_col_blk*stride_c；left_off = left_col_blk * ch_pair_num（移位加）
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        left_col_s_reg <= -10'sd1;
    else if (conv_start)
        left_col_s_reg <= -10'sd1;
    else if (scan_step_fire && kh_last && cin_last) begin
        if (col_last)
            left_col_s_reg <= -10'sd1;
        else
            left_col_s_reg <= left_col_s_reg + $signed({7'd0, cfg_stride_c_real});
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        left_off_s_reg <= 11'sd0;
    else if (conv_start)
        left_off_s_reg <= -$signed({10'd0, cfg_ch_pair_num});
    else if (scan_step_fire && kh_last && cin_last) begin
        if (col_last)
            left_off_s_reg <= -$signed({10'd0, cfg_ch_pair_num});
        else
            left_off_s_reg <= left_off_s_reg + $signed(col_step_off[21:0]);
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        scan_done_reg <= 1'b0;
    else if (conv_start)
        scan_done_reg <= 1'b0;
    else if (scan_step_fire && scan_last_step)
        scan_done_reg <= 1'b1;
end

// ---- 窗口参数 next 级 ----
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        win_next_valid_reg <= 1'b0;
    else if (conv_start)
        win_next_valid_reg <= 1'b0;
    else if (win_take_fire || chain_fire)
        win_next_valid_reg <= 1'b0;
    else if (scan_step_fire && is_padding_row)
        win_next_valid_reg <= 1'b1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        win_next_phy_row_reg    <= 2'd0;
        win_next_is_top_reg     <= 1'b0;
        win_next_left_off_s_reg <= 22'sd0;
        win_next_left_col_s_reg <= 10'sd0;
        win_next_cp_reg         <= 12'd0;
        win_next_cin_odd_reg    <= 1'b0;
    end
    else if (conv_start) begin
        win_next_phy_row_reg    <= 2'd0;
        win_next_is_top_reg     <= 1'b0;
        win_next_left_off_s_reg <= 22'sd0;
        win_next_left_col_s_reg <= 10'sd0;
        win_next_cp_reg         <= 12'd0;
        win_next_cin_odd_reg    <= 1'b0;
    end
    else if (scan_step_fire && is_padding_row) begin
        win_next_phy_row_reg  <= phy_row_next;
        win_next_is_top_reg   <= is_top_pad;
        win_next_left_off_s_reg <= left_off_s_reg;
        win_next_left_col_s_reg <= left_col_s_reg;
        win_next_cp_reg       <= {1'b0, cin_reg[11:1]};
        win_next_cin_odd_reg  <= cin_reg[0];
    end
end

// ---- 窗口参数 cur 级与读引擎 ----
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        win_cur_phy_row_reg    <= 2'd0;
        win_cur_is_top_reg     <= 1'b0;
        win_cur_left_off_s_reg <= 22'sd0;
        win_cur_left_col_s_reg <= 10'sd0;
        win_cur_cp_reg         <= 12'd0;
        win_cur_cin_odd_reg    <= 1'b0;
    end
    else if (conv_start) begin
        win_cur_phy_row_reg    <= 2'd0;
        win_cur_is_top_reg     <= 1'b0;
        win_cur_left_off_s_reg <= 22'sd0;
        win_cur_left_col_s_reg <= 10'sd0;
        win_cur_cp_reg         <= 12'd0;
        win_cur_cin_odd_reg    <= 1'b0;
    end
    else if (win_take_fire || chain_fire) begin
        win_cur_phy_row_reg  <= win_next_phy_row_reg;
        win_cur_is_top_reg   <= win_next_is_top_reg;
        win_cur_left_off_s_reg <= win_next_left_off_s_reg;
        win_cur_left_col_s_reg <= win_next_left_col_s_reg;
        win_cur_cp_reg       <= win_next_cp_reg;
        win_cur_cin_odd_reg  <= win_next_cin_odd_reg;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        issue_state_reg <= I_IDLE;
    else if (conv_start)
        issue_state_reg <= I_IDLE;
    else if (win_take_fire)
        issue_state_reg <= I_READ;
    else if (issue_state_read && slot_last && !chain_fire)
        issue_state_reg <= I_IDLE;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        slot_cnt_reg <= 2'd0;
    else if (conv_start)
        slot_cnt_reg <= 2'd0;
    else if (win_take_fire)
        slot_cnt_reg <= 2'd1;
    else if (issue_state_read) begin
        if (slot_last)
            slot_cnt_reg <= 2'd0;
        else
            slot_cnt_reg <= slot_cnt_reg + 2'd1;
    end
end

// slot 地址累加器：窗口首 slot 用 left_off，之后逐拍 += ch_pair_num；
// chain 时下一拍是新窗口 slot0，装载 left_off 本身
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        slot_off_s_reg <= 22'sd0;
    else if (conv_start)
        slot_off_s_reg <= 22'sd0;
    else if (win_take_fire)
        slot_off_s_reg <= win_next_left_off_s_reg + $signed({10'd0, cfg_ch_pair_num});
    else if (chain_fire)
        slot_off_s_reg <= win_next_left_off_s_reg;
    else if (issue_state_read && !slot_last)
        slot_off_s_reg <= slot_off_s_reg + $signed({10'd0, cfg_ch_pair_num});
end

// prod_bank：本窗口 4 个 slot 期间保持稳定；slot3 发完才翻转到下一窗口用 bank
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        prod_bank_reg <= 1'b0;
    else if (conv_start)
        prod_bank_reg <= 1'b0;
    else if (issue_state_read && slot_last)
        prod_bank_reg <= ~prod_bank_reg;
end

// ---- 读数据捕获流水线（读口顶层寄存 1 拍 + RAM 同步 1 拍，共 2 拍；越界 slot 直接装 0） ----
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        pend_reg <= 1'b0;
    else if (conv_start)
        pend_reg <= 1'b0;
    else
        pend_reg <= win_take_fire || issue_state_read;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        pend_slot_reg <= 2'd0;
        pend_bank_reg <= 1'b0;
        pend_zero_reg <= 1'b0;
        pend_odd_reg  <= 1'b0;
    end
    else if (conv_start) begin
        pend_slot_reg <= 2'd0;
        pend_bank_reg <= 1'b0;
        pend_zero_reg <= 1'b0;
        pend_odd_reg  <= 1'b0;
    end
    else if (win_take_fire || issue_state_read) begin
        pend_slot_reg <= issue_state_read ? slot_cnt_reg : 2'd0;
        pend_bank_reg <= prod_bank_reg;
        pend_zero_reg <= !slot_valid;
        pend_odd_reg  <= issue_state_read ? win_cur_cin_odd_reg : win_next_cin_odd_reg;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        pend_d2_reg <= 1'b0;
    else if (conv_start)
        pend_d2_reg <= 1'b0;
    else
        pend_d2_reg <= pend_reg;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        pend_slot_d2_reg <= 2'd0;
        pend_bank_d2_reg <= 1'b0;
        pend_zero_d2_reg <= 1'b0;
        pend_odd_d2_reg  <= 1'b0;
    end
    else if (conv_start) begin
        pend_slot_d2_reg <= 2'd0;
        pend_bank_d2_reg <= 1'b0;
        pend_zero_d2_reg <= 1'b0;
        pend_odd_d2_reg  <= 1'b0;
    end
    else if (pend_reg) begin
        pend_slot_d2_reg <= pend_slot_reg;
        pend_bank_d2_reg <= pend_bank_reg;
        pend_zero_d2_reg <= pend_zero_reg;
        pend_odd_d2_reg  <= pend_odd_reg;
    end
end

// BMG 读延迟 2 拍（顶层寄存 1 拍 + RAM 2 拍，issue 后第 3 拍装入），捕获链再加一级
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        pend_d3_reg <= 1'b0;
    else if (conv_start)
        pend_d3_reg <= 1'b0;
    else
        pend_d3_reg <= pend_d2_reg;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        pend_slot_d3_reg <= 2'd0;
        pend_bank_d3_reg <= 1'b0;
        pend_zero_d3_reg <= 1'b0;
        pend_odd_d3_reg  <= 1'b0;
    end
    else if (conv_start) begin
        pend_slot_d3_reg <= 2'd0;
        pend_bank_d3_reg <= 1'b0;
        pend_zero_d3_reg <= 1'b0;
        pend_odd_d3_reg  <= 1'b0;
    end
    else if (pend_d2_reg) begin
        pend_slot_d3_reg <= pend_slot_d2_reg;
        pend_bank_d3_reg <= pend_bank_d2_reg;
        pend_zero_d3_reg <= pend_zero_d2_reg;
        pend_odd_d3_reg  <= pend_odd_d2_reg;
    end
end

always @(posedge clk) begin
    if (pend_fire) begin
        case ({pend_bank_d3_reg, pend_slot_d3_reg})
            3'b000:  chunk_bank0_slot0_reg <= pend_zero_d3_reg ? 256'd0 : capture_chunk;
            3'b001:  chunk_bank0_slot1_reg <= pend_zero_d3_reg ? 256'd0 : capture_chunk;
            3'b010:  chunk_bank0_slot2_reg <= pend_zero_d3_reg ? 256'd0 : capture_chunk;
            3'b011:  chunk_bank0_slot3_reg <= pend_zero_d3_reg ? 256'd0 : capture_chunk;
            3'b100:  chunk_bank1_slot0_reg <= pend_zero_d3_reg ? 256'd0 : capture_chunk;
            3'b101:  chunk_bank1_slot1_reg <= pend_zero_d3_reg ? 256'd0 : capture_chunk;
            3'b110:  chunk_bank1_slot2_reg <= pend_zero_d3_reg ? 256'd0 : capture_chunk;
            default: chunk_bank1_slot3_reg <= pend_zero_d3_reg ? 256'd0 : capture_chunk;
        endcase
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        bank_rdy0_reg <= 1'b0;
    else if (conv_start)
        bank_rdy0_reg <= 1'b0;
    else if (cons_clear_rdy0)
        bank_rdy0_reg <= 1'b0;
    else if (prod_set_rdy0)
        bank_rdy0_reg <= 1'b1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        bank_rdy1_reg <= 1'b0;
    else if (conv_start)
        bank_rdy1_reg <= 1'b0;
    else if (cons_clear_rdy1)
        bank_rdy1_reg <= 1'b0;
    else if (prod_set_rdy1)
        bank_rdy1_reg <= 1'b1;
end

// ---- 输出引擎 ----
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        out_state_reg <= O_WAIT;
    else if (conv_start)
        out_state_reg <= O_WAIT;
    else if (!out_state_out && (cons_bank_reg ? bank_rdy1_reg : bank_rdy0_reg))
        out_state_reg <= O_OUT;
    else if (out_drain_fire && !out_chain_fire)
        out_state_reg <= O_WAIT;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        cons_bank_reg <= 1'b0;
    else if (conv_start)
        cons_bank_reg <= 1'b0;
    else if (out_drain_fire)
        cons_bank_reg <= ~cons_bank_reg;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        kw_reg <= 3'd0;
    else if (conv_start)
        kw_reg <= 3'd0;
    else if (!out_state_out)
        kw_reg <= 3'd0;
    else if (out_step_fire)
        kw_reg <= kw_last ? 3'd0 : (kw_reg + 3'd1);
end

endmodule

// ============================================================
// RTL Self-Check Results
// ============================================================
// [PASS] Rule #1:  All ports have explicit width declarations
// [PASS] Rule #2:  Ports use Verilog wire/reg conventions
// [PASS] Rule #6:  No debug/error-attribution logic or diagnostic-only sideband in RTL
// [PASS] Rule #8:  No logic/always_ff/always_comb used
// [PASS] Rule #12: Byte-gather is explicit generate wiring (no comb procedural for); chunk case has default
// [PASS] Rule #16: Non-blocking '<=' for sequential; no comb always blocks left
// [PASS] Rule #24: No multiplier anywhere; lane offset is shift/add (stride 1 or 2)
// [PASS] Rule #26: Issue/output engines use localparam explicit encoding
// [PASS] Rule #43: No function/task used
// [PASS] Start boundary: scan/read/output/done are inactive before conv_start;
//        active gating adds no normal-run pipeline stage
// [PASS] Control metadata registers have explicit reset; wide data banks are not reset
// [WARN] Byte-gather index relies on the geometry contract
//        byte_start + (kc-1) + 31*stride_c_real <= 69 (same as prefetch_fm_conv)
// ============================================================
