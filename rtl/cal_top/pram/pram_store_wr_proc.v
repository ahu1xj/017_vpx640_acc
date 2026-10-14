`include "all_defines.vh"

// pram_store_wr_proc: store 流写入 FSM 与地址生成。
// 接受 104bit wrcmd（{5'd0, wr_sm_cmd[98:0]}，直接用 WR 宏解码），自-paced：
// 每条 wrcmd 写完才回 IDLE 接受下一条。
// PRAM 拆块后输出 region 位 + 区内 12bit 地址：region=1（top）存 FM 末 3 行
//   供上边 padding，region=0（bottom）存 FM 首 3 行供下边 padding，不再有 4096 中界。
//   本模块按写回流时间序先收首 3 行、后收末 3 行，因此首 3 行落 bottom、末 3 行落 top。
// 小行数契约（2026-07-31 定案，取代 2026-07-25 的"小行数全 DDR 绕路"版）：
//   全行数缓存，编译器保证 cpad 模式 pad_top/bottom <= FM 行数 R：
//   R>=6  : 首3落 bottom、末3落 top，无交集（现状逐位保持）；
//   3<=R<6: 交集行（R-3..2）一拍双写——同拍 bot/top 两使能两地址，不停顿不暂存；
//   1<=R<3: 全部 R 行一拍双写（bottom 行 i / top 行 i+3-R）。
//   top 区行地址由独立累加器 top_row_base 产生，bottom 区沿用 row_base。
// CONV 跳写: row(0..2,跨区清) -> col_blk -> ch_pair_in_group，
//   addr = row_base + col_base + group_pair_offset + ch_pair_cnt，region=(word_cnt>=top_words)
// POOL 跳写: ch_pair(外) -> row(0..5) -> col_blk(内)，
//   addr = row_base + col_base + ch_pair_cnt，region=(row_cnt>=3)
// FSM: IDLE / CALC1 / CALC2 / CALC3 / CALC4 / RECV。
module pram_store_wr_proc(
    input               clk,
    input               rst_n,

    input               s_axis_wrcmd_tvalid,
    input      [103:0]  s_axis_wrcmd_tdata,
    output              s_axis_wrcmd_tready,

    input               s_axis_cpad_store_tvalid,
    input      [511:0]  s_axis_cpad_store_tdata,
    output              s_axis_cpad_store_tready,

    output              store_head_fire,
    output              store_wr_en,
    output              store_wr_bot_en,
    output              store_wr_top_en,
    output     [11:0]   store_wr_bot_addr,
    output     [11:0]   store_wr_top_addr,
    output     [511:0]  store_wr_data,
    output              proc_idle
);

localparam [2:0] S_IDLE  = 3'd0;
localparam [2:0] S_CALC1 = 3'd1;
localparam [2:0] S_CALC2 = 3'd2;
localparam [2:0] S_CALC3 = 3'd3;
localparam [2:0] S_CALC4 = 3'd4;
localparam [2:0] S_RECV  = 3'd5;

reg [2:0]    state_reg;
reg          mode_pool_reg;
reg [11:0]   ch_end_reg;
reg [6:0]    col_end_reg;
reg [5:0]    ch_start_reg;
reg [11:0]   ch_total_reg;
reg [11:0]   ch_unit_reg;
reg [11:0]   total_ch_pair_num_reg;
reg [7:0]    col_blk_num_reg;
reg [19:0]   stream_words_per_row_reg;
reg [19:0]   full_line_size_reg;
reg [14:0]   top_words_reg;
reg [14:0]   total_words_reg;
reg [14:0]   word_cnt_reg;
reg [11:0]   ch_pair_cnt_reg;
reg [7:0]    col_cnt_reg;
reg [2:0]    row_cnt_reg;
reg [19:0]   row_base_reg;
reg [19:0]   col_base_reg;
reg [11:0]   row_num_reg;
// 小行数双写：top 区行基址累加器 + "已进入 top 区"标记
reg [19:0]   top_row_base_reg;

wire [2:0]   cmd_data_origin;
wire         cmd_is_conv;
wire         cmd_is_pool;
wire [11:0]  cmd_ch_end;
wire [6:0]   cmd_col_end;
wire [5:0]   cmd_ch_start;
wire [11:0]  cmd_ch_total;
wire [11:0]  cmd_row_end;
wire         accept_fire;
wire         state_calc4;
wire         state_recv;
wire [11:0]  conv_start_ch;
wire [11:0]  ch_unit_next;
wire [11:0]  total_ch_pair_num_next;
wire         swpr_zero;
wire         beat_fire;
wire         last_beat_fire;
wire         ch_pair_last;
wire         col_last;
wire         conv_row_last;
wire         pool_row_last;
wire         pool_row_base_reset;
wire         wr_region_next;
wire [11:0]  group_pair_offset;
wire [19:0]  addr_sum;
wire [11:0]  wraddr_next;
wire [14:0]  few_rows_total_next;
wire [14:0]  total_words_next;
// 小行数双写控制
wire         few_rows_mode;
wire         top_wr;
wire [19:0]  top_addr_sum;

assign cmd_data_origin = s_axis_wrcmd_tdata[`WR_DATA_ORIGIN_START +: `WR_DATA_ORIGIN_WIDTH];
assign cmd_is_conv     = (cmd_data_origin == `DATA_ORIGIN_ACTIVE);
assign cmd_is_pool     = (cmd_data_origin == `DATA_ORIGIN_POOLING);
assign cmd_ch_end      = s_axis_wrcmd_tdata[`WR_FM_CHANNEL_END_START +: `WR_FM_CHANNEL_END_WIDTH];
assign cmd_col_end     = s_axis_wrcmd_tdata[`WR_FM_COLUMN_END_START +: `WR_FM_COLUMN_END_WIDTH];
assign cmd_ch_start    = s_axis_wrcmd_tdata[`WR_FM_CHANNEL_START_START +: `WR_FM_CHANNEL_START_WIDTH];
assign cmd_ch_total    = s_axis_wrcmd_tdata[`WR_FM_CHANNEL_TOTAL_START +: `WR_FM_CHANNEL_TOTAL_WIDTH];
assign cmd_row_end     = s_axis_wrcmd_tdata[`WR_FM_ROW_END_START +: `WR_FM_ROW_END_WIDTH];

assign accept_fire  = (state_reg == S_IDLE) && s_axis_wrcmd_tvalid;
assign state_calc4  = (state_reg == S_CALC4);
assign state_recv   = (state_reg == S_RECV);

assign conv_start_ch = {ch_start_reg, 6'd0};
// fm_channel_end = 本批末通道（含）索引 - 1（按通道对计数，恒为偶数）；
// 组通道范围 [start*64, end+1]（含端点），ch_pair 数 = (end - start + 2) >> 1
assign ch_unit_next  = mode_pool_reg ? ((ch_end_reg + 12'd2) >> 1)
                                     : ((ch_end_reg - conv_start_ch + 12'd2) >> 1);
assign total_ch_pair_num_next = mode_pool_reg ? ((ch_end_reg + 12'd2) >> 1)
                                              : (ch_total_reg >> 1);

assign swpr_zero      = (stream_words_per_row_reg == 20'd0);
assign beat_fire      = state_recv && s_axis_cpad_store_tvalid;
assign last_beat_fire = beat_fire && (word_cnt_reg == (total_words_reg - 15'd1));

// 小行数总长：R<6 时 store tap 只发 R 行（首3∪末3 并集），按 R 行收满；
// R>=6 时维持原契约 3+3=6 行（top_words×2）。case+移位加，无乘法。
assign few_rows_total_next = (row_num_reg == 12'd1) ? stream_words_per_row_reg[14:0] :
                             (row_num_reg == 12'd2) ? {stream_words_per_row_reg[13:0], 1'b0} :
                             (row_num_reg == 12'd3) ? {stream_words_per_row_reg[13:0], 1'b0}
                                                    + stream_words_per_row_reg[14:0] :
                             (row_num_reg == 12'd4) ? {stream_words_per_row_reg[12:0], 2'b00} :
                                                      {stream_words_per_row_reg[12:0], 2'b00}
                                                    + stream_words_per_row_reg[14:0];
assign total_words_next = (row_num_reg >= 12'd6) ? {top_words_reg[13:0], 1'b0}
                                                 : few_rows_total_next;

assign ch_pair_last = (ch_pair_cnt_reg == (ch_unit_reg - 12'd1));
assign col_last     = (col_cnt_reg == (col_blk_num_reg - 8'd1));
// 小行数（R<6）时行计数上限改为真实行数 R（原契约末行 index 恒 2/5）
assign conv_row_last = !mode_pool_reg && (few_rows_mode ? (row_cnt_reg == {1'b0, row_num_reg[2:0]} - 3'd1)
                                                        : (row_cnt_reg == 3'd2));
assign pool_row_last = mode_pool_reg && (few_rows_mode ? (row_cnt_reg == {1'b0, row_num_reg[2:0]} - 3'd1)
                                                       : (row_cnt_reg == 3'd5));
assign pool_row_base_reset = mode_pool_reg &&
                              (pool_row_last || (row_cnt_reg == 3'd2));

assign wr_region_next = mode_pool_reg ? (row_cnt_reg >= 3'd3)
                                      : (word_cnt_reg >= top_words_reg);
assign group_pair_offset = {ch_start_reg, 5'd0};
assign addr_sum = {8'd0, row_base_reg} + {8'd0, col_base_reg}
                + (mode_pool_reg ? 20'd0 : {8'd0, group_pair_offset})
                + {8'd0, ch_pair_cnt_reg};
assign wraddr_next = addr_sum[11:0];

// ===== 小行数双写（3<=R<6 交集行 / 1<=R<3 全行 一拍双写） =====
// few_rows_mode = R<6（含 R<3）；top 集合 = 行 i 满足 i+3 >= R；
// top_addr 与 addr_sum 同构，仅行基址换 top_row_base（top 区独立累加器）
assign few_rows_mode = (row_num_reg < 12'd6);
assign top_wr        = few_rows_mode && (({1'b0, row_cnt_reg} + 4'd3) >= row_num_reg[3:0]);
assign top_addr_sum  = {8'd0, top_row_base_reg} + {8'd0, col_base_reg}
                     + (mode_pool_reg ? 20'd0 : {8'd0, group_pair_offset})
                     + {8'd0, ch_pair_cnt_reg};

assign s_axis_wrcmd_tready      = (state_reg == S_IDLE);
assign s_axis_cpad_store_tready = state_recv;
// store_phase 翻转点：仅计算层层首 wrcmd（POOL 唯一一条 / CONV start==0 的 group0）。
// RES 与 PS+FM 只缓存不翻相位：RESADD 广播不翻 load_phase；
// PS+FM 的捕获写完由其后续首个计算层首 wrcmd 的自-paced 接受蕴含（store FSM 顺序处理）。
assign store_head_fire = accept_fire &&
                         (cmd_is_pool || (cmd_is_conv && (cmd_ch_start == 6'd0)));
assign store_wr_en     = beat_fire;
// 双写口：R>=6 退化为 region 单选（与原逐位等价）；R<6 按行判定 bot/top，
// 交集行同拍双使能——两块独立 RAM 各自写口，同拍双地址写，不停顿不暂存
assign store_wr_bot_en   = few_rows_mode ? (row_cnt_reg < 3'd3) : (!wr_region_next);
assign store_wr_top_en   = few_rows_mode ? top_wr : wr_region_next;
assign store_wr_bot_addr = wraddr_next;
assign store_wr_top_addr = few_rows_mode ? top_addr_sum[11:0] : wraddr_next;
assign store_wr_data  = s_axis_cpad_store_tdata;
// proc 回 IDLE = 当前 wrcmd 数据全部写完（数据仅在 S_RECV 接收）；
// 供 pram_wr_switch 无转发层（int16）自愈翻相位的"写完"判据
assign proc_idle      = (state_reg == S_IDLE);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        state_reg <= S_IDLE;
    else begin
        case (state_reg)
            S_IDLE:
                if (accept_fire)
                    state_reg <= S_CALC1;
                else
                    state_reg <= S_IDLE;
            S_CALC1:
                state_reg <= S_CALC2;
            S_CALC2:
                state_reg <= S_CALC3;
            S_CALC3:
                state_reg <= S_CALC4;
            S_CALC4:
                if (swpr_zero)
                    state_reg <= S_IDLE;
                else
                    state_reg <= S_RECV;
            S_RECV:
                if (last_beat_fire)
                    state_reg <= S_IDLE;
                else
                    state_reg <= S_RECV;
            default:
                state_reg <= S_IDLE;
        endcase
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        mode_pool_reg <= 1'b0;
        ch_end_reg    <= 12'd0;
        col_end_reg   <= 7'd0;
        ch_start_reg  <= 6'd0;
        ch_total_reg  <= 12'd0;
        row_num_reg   <= 12'd0;
    end
    else if (accept_fire) begin
        mode_pool_reg <= cmd_is_pool;
        ch_end_reg    <= cmd_ch_end;
        col_end_reg   <= cmd_col_end;
        ch_start_reg  <= cmd_ch_start;
        ch_total_reg  <= cmd_ch_total;
        row_num_reg   <= cmd_row_end;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        ch_unit_reg           <= 12'd0;
        total_ch_pair_num_reg <= 12'd0;
        col_blk_num_reg       <= 8'd0;
    end
    else if (state_reg == S_CALC1) begin
        ch_unit_reg          <= ch_unit_next;
        total_ch_pair_num_reg <= total_ch_pair_num_next;
        col_blk_num_reg      <= {1'b0, col_end_reg} + 8'd1;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        stream_words_per_row_reg <= 20'd0;
        full_line_size_reg       <= 20'd0;
    end
    else if (state_reg == S_CALC2) begin
        stream_words_per_row_reg <= {12'd0, col_blk_num_reg} * {8'd0, ch_unit_reg};
        full_line_size_reg       <= {12'd0, col_blk_num_reg} * {8'd0, total_ch_pair_num_reg};
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        top_words_reg <= 15'd0;
    else if (state_reg == S_CALC3)
        top_words_reg <= {stream_words_per_row_reg[13:0], 1'b0} + stream_words_per_row_reg[14:0];
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        total_words_reg <= 15'd0;
    else if (state_calc4)
        total_words_reg <= total_words_next;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        word_cnt_reg <= 15'd0;
    else if (accept_fire)
        word_cnt_reg <= 15'd0;
    else if (beat_fire)
        word_cnt_reg <= word_cnt_reg + 15'd1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        ch_pair_cnt_reg <= 12'd0;
    else if (accept_fire)
        ch_pair_cnt_reg <= 12'd0;
    else if (beat_fire) begin
        if (mode_pool_reg) begin
            if (col_last && pool_row_last) begin
                if (ch_pair_last)
                    ch_pair_cnt_reg <= 12'd0;
                else
                    ch_pair_cnt_reg <= ch_pair_cnt_reg + 12'd1;
            end
        end
        else begin
            if (ch_pair_last)
                ch_pair_cnt_reg <= 12'd0;
            else
                ch_pair_cnt_reg <= ch_pair_cnt_reg + 12'd1;
        end
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        col_cnt_reg <= 8'd0;
    else if (accept_fire)
        col_cnt_reg <= 8'd0;
    else if (beat_fire) begin
        if (mode_pool_reg) begin
            if (col_last)
                col_cnt_reg <= 8'd0;
            else
                col_cnt_reg <= col_cnt_reg + 8'd1;
        end
        else begin
            if (ch_pair_last) begin
                if (col_last)
                    col_cnt_reg <= 8'd0;
                else
                    col_cnt_reg <= col_cnt_reg + 8'd1;
            end
        end
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        row_cnt_reg <= 3'd0;
    else if (accept_fire)
        row_cnt_reg <= 3'd0;
    else if (beat_fire) begin
        if (mode_pool_reg) begin
            if (col_last) begin
                if (pool_row_last)
                    row_cnt_reg <= 3'd0;
                else
                    row_cnt_reg <= row_cnt_reg + 3'd1;
            end
        end
        else begin
            if (ch_pair_last && col_last) begin
                if (conv_row_last)
                    row_cnt_reg <= 3'd0;
                else
                    row_cnt_reg <= row_cnt_reg + 3'd1;
            end
        end
    end
end

// row_base: CONV 每行 += full_line_size（跨区/换层清 0）；POOL 同理
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        row_base_reg <= 20'd0;
    else if (accept_fire)
        row_base_reg <= 20'd0;
    else if (beat_fire) begin
        if (mode_pool_reg) begin
            if (col_last) begin
                if (pool_row_base_reset)
                    row_base_reg <= 20'd0;
                else
                    row_base_reg <= row_base_reg + full_line_size_reg;
            end
        end
        else begin
            if (ch_pair_last && col_last) begin
                if (conv_row_last)
                    row_base_reg <= 20'd0;
                else
                    row_base_reg <= row_base_reg + full_line_size_reg;
            end
        end
    end
end

// col_base: 每 col_blk += 行内 ch_pair 步长（CONV=total_ch_pair_num，POOL=ch_pair_num）
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        col_base_reg <= 20'd0;
    else if (accept_fire)
        col_base_reg <= 20'd0;
    else if (beat_fire) begin
        if (mode_pool_reg) begin
            if (col_last)
                col_base_reg <= 20'd0;
            else
                col_base_reg <= col_base_reg + {8'd0, ch_unit_reg};
        end
        else begin
            if (ch_pair_last) begin
                if (col_last)
                    col_base_reg <= 20'd0;
                else
                    col_base_reg <= col_base_reg + {8'd0, total_ch_pair_num_reg};
            end
        end
    end
end

// ===== 小行数 top 区行基址 =====
// top_row_base 于 CALC4 装载初值（R=1→2L，R=2→L，R>=3→0）。
// top 区每一行的最后一个 beat 直接推进行基址；不能用"已见 top beat"门控，
// 否则 col_blk_num==1 时首行同时也是首个 top beat，行末推进会被漏掉。
// POOL 每个 channel pair 的最后一行结束后恢复本 command 的初始 top offset，
// 避免下一个 channel pair 继承上一个 pair 的 top 行基址。
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        top_row_base_reg <= 20'd0;
    else if (state_calc4)
        top_row_base_reg <= (row_num_reg == 12'd1) ? {full_line_size_reg, 1'b0} :
                            (row_num_reg == 12'd2) ? full_line_size_reg : 20'd0;
    else if (beat_fire && top_wr &&
             (mode_pool_reg ? col_last : (ch_pair_last && col_last))) begin
        if (mode_pool_reg && pool_row_last)
            top_row_base_reg <= (row_num_reg == 12'd1) ? {full_line_size_reg, 1'b0} :
                                (row_num_reg == 12'd2) ? full_line_size_reg : 20'd0;
        else
            top_row_base_reg <= top_row_base_reg + full_line_size_reg;
    end
end

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
// [PASS] Rule #24: Two multipliers (stream_words_per_row/full_line_size) in config phase only
// [PASS] Rule #26: FSM states use localparam explicit encoding
// [PASS] Rule #43: No function/task used; 3x multiply via shift-add
// [PASS] Dual-write: overlap rows write both region RAMs in the SAME cycle
//        (no stall, no hold register, no backpressure added to cpad store stream)
// [PASS] Timing: top_row_base init latched at CALC4, row-end increments only;
//        no new combinational depth beyond existing addr_sum shape
// ============================================================
