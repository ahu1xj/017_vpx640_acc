// pram_load_wr_proc: load 流写入 FSM 与地址生成。
// 输入流本身即 PRAM 目标布局（row -> col_blk -> ch_pair，top region 在前）。
// PRAM 拆块后输出 region 位 + 区内 12bit 地址：word_cnt < top_words 写 top
// region（FM 末 3 行），否则写 bottom region（FM 首 3 行），不再有 4096 中界。
// FSM: IDLE / WAIT_SYNC / CALC1 / CALC2 / CALC3 / CALC4 / RECV。
// CALC4 为零总长判决级：total_words 在 CALC3 寄存后再判零，
// 切断 line_size(乘法) -> 移位加 -> 判零 -> FSM 的组合长链（时序整改点）。
module pram_load_wr_proc(
    input               clk,
    input               rst_n,

    input               load_start,
    input      [11:0]   cfg_ch_in,
    input      [6:0]    cfg_col_in,
    input      [1:0]    cfg_pad_top,
    input      [1:0]    cfg_pad_bottom,
    input               in_sync,

    input               s_axis_cpad_load_tvalid,
    input      [511:0]  s_axis_cpad_load_tdata,
    output              s_axis_cpad_load_tready,

    output              load_wr_en,
    output              load_wr_region,
    output     [11:0]   load_wr_addr,
    output     [511:0]  load_wr_data,
    output              load_commit_pulse
);

localparam [2:0] S_IDLE      = 3'd0;
localparam [2:0] S_WAIT_SYNC = 3'd1;
localparam [2:0] S_CALC1     = 3'd2;
localparam [2:0] S_CALC2     = 3'd3;
localparam [2:0] S_CALC3     = 3'd4;
localparam [2:0] S_RECV      = 3'd5;
localparam [2:0] S_CALC4     = 3'd6;

reg [2:0]    state_reg;
reg [11:0]   cfg_ch_in_reg;
reg [6:0]    cfg_col_in_reg;
reg [1:0]    pad_top_reg;
reg [1:0]    pad_bottom_reg;
reg [11:0]   ch_pair_num_reg;
reg [7:0]    col_blk_num_reg;
reg [19:0]   line_size_reg;
reg [14:0]   top_words_reg;
reg [14:0]   bottom_words_reg;
reg [14:0]   total_words_reg;
reg [14:0]   word_cnt_reg;
reg          commit_reg;

wire         state_calc3;
wire         state_calc4;
wire         state_recv;
wire [14:0]  top_words_next;
wire [14:0]  bottom_words_next;
wire         total_zero_next;
wire         beat_fire;
wire         last_beat_fire;
wire         commit_next;
wire         wr_region_next;
wire [14:0]  wraddr_next;

assign state_calc3     = (state_reg == S_CALC3);
assign state_calc4     = (state_reg == S_CALC4);
assign state_recv      = (state_reg == S_RECV);

// padding <= 3，case 移位加实现，无逐拍乘法
assign top_words_next = (pad_top_reg == 2'd0) ? 15'd0 :
                        (pad_top_reg == 2'd1) ? line_size_reg[14:0] :
                        (pad_top_reg == 2'd2) ? {line_size_reg[13:0], 1'b0} :
                        {line_size_reg[13:0], 1'b0} + line_size_reg[14:0];
assign bottom_words_next = (pad_bottom_reg == 2'd0) ? 15'd0 :
                           (pad_bottom_reg == 2'd1) ? line_size_reg[14:0] :
                           (pad_bottom_reg == 2'd2) ? {line_size_reg[13:0], 1'b0} :
                           {line_size_reg[13:0], 1'b0} + line_size_reg[14:0];
assign total_zero_next = (total_words_reg == 15'd0);

assign beat_fire      = state_recv && s_axis_cpad_load_tvalid;
assign last_beat_fire = beat_fire && (word_cnt_reg == (total_words_reg - 15'd1));
assign commit_next    = last_beat_fire || (state_calc4 && total_zero_next);

assign wr_region_next = (word_cnt_reg < top_words_reg);
assign wraddr_next    = (word_cnt_reg < top_words_reg)
                      ? word_cnt_reg
                      : (word_cnt_reg - top_words_reg);

assign s_axis_cpad_load_tready = state_recv;
assign load_wr_en     = beat_fire;
assign load_wr_region = wr_region_next;
assign load_wr_addr   = wraddr_next[11:0];
assign load_wr_data   = s_axis_cpad_load_tdata;
assign load_commit_pulse = commit_reg;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        state_reg <= S_IDLE;
    else begin
        case (state_reg)
            S_IDLE:
                if (load_start)
                    state_reg <= S_WAIT_SYNC;
                else
                    state_reg <= S_IDLE;
            S_WAIT_SYNC:
                if (in_sync)
                    state_reg <= S_CALC1;
                else
                    state_reg <= S_WAIT_SYNC;
            S_CALC1:
                state_reg <= S_CALC2;
            S_CALC2:
                state_reg <= S_CALC3;
            S_CALC3:
                state_reg <= S_CALC4;
            S_CALC4:
                if (total_zero_next)
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
        cfg_ch_in_reg  <= 12'd0;
        cfg_col_in_reg <= 7'd0;
        pad_top_reg    <= 2'd0;
        pad_bottom_reg <= 2'd0;
    end
    else if (load_start) begin
        cfg_ch_in_reg  <= cfg_ch_in;
        cfg_col_in_reg <= cfg_col_in;
        pad_top_reg    <= cfg_pad_top;
        pad_bottom_reg <= cfg_pad_bottom;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        ch_pair_num_reg <= 12'd0;
        col_blk_num_reg <= 8'd0;
    end
    else if (state_reg == S_CALC1) begin
        ch_pair_num_reg <= cfg_ch_in_reg >> 1;
        col_blk_num_reg <= {1'b0, cfg_col_in_reg} + 8'd1;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        line_size_reg <= 20'd0;
    else if (state_reg == S_CALC2)
        line_size_reg <= {12'd0, col_blk_num_reg} * {8'd0, ch_pair_num_reg};
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        top_words_reg    <= 15'd0;
        bottom_words_reg <= 15'd0;
        total_words_reg  <= 15'd0;
    end
    else if (state_calc3) begin
        top_words_reg    <= top_words_next;
        bottom_words_reg <= bottom_words_next;
        total_words_reg  <= top_words_next + bottom_words_next;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        word_cnt_reg <= 15'd0;
    else if (state_calc3)
        word_cnt_reg <= 15'd0;
    else if (beat_fire)
        word_cnt_reg <= word_cnt_reg + 15'd1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        commit_reg <= 1'b0;
    else
        commit_reg <= commit_next;
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
// [PASS] Rule #24: One multiplier (line_size) in config phase, explicit 20-bit width
// [PASS] Rule #26: FSM states use localparam explicit encoding
// [PASS] Rule #43: No function/task used; padding multiply via case+shift
// ============================================================
