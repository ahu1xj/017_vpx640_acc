// Stage1 锚点级：按行信用驱动，为每个 (out_row, out_col_blk) 生成左上角锚点 token。
// 地址全部运行累加：行内 virt_addr += col_step，行头装载 row_base - cin_pair_num；
// row_base 行步进 += row_step 并按不变量 I2 单次条件减 ring_size（回绕就地吸收）。
// token: { virt_addr[31:0], row_s[15:0], col_s[9:0], last } = 59bit
module prefetch_fm_conv_stage1_anchor(
    input                   clk,
    input                   rst_n,

    input                   conv_start,
    input                   fm_add_sig,

    input   [11:0]          cfg_fm_out_row,
    input   [6:0]           cfg_fm_out_col,
    input   [2:0]           cfg_stride_r_real,
    input   [2:0]           cfg_stride_c_real,
    input   [11:0]          cfg_cin_pair_num,
    input   [31:0]          cfg_ring_size,
    input   [31:0]          cfg_row_step,
    input   [31:0]          cfg_col_step,
    input   [31:0]          cfg_row_base_init,
    input   signed [15:0]   cfg_origin_row_init,

    input                   t1_full,
    output                  t1_wr_en,
    output  [58:0]          t1_wr_data
);

localparam [2:0] S1_IDLE     = 3'd0;
localparam [2:0] S1_CFG      = 3'd1;
localparam [2:0] S1_ROW_WAIT = 3'd2;
localparam [2:0] S1_EMIT     = 3'd3;
localparam [2:0] S1_DONE     = 3'd4;

reg [2:0]          state_reg;
reg [1:0]          cfg_cnt_reg;
reg [12:0]         row_credit_cnt_reg;
reg [11:0]         out_row_cnt_reg;
reg [6:0]          out_col_blk_reg;
reg [31:0]         row_base_reg;
reg signed [31:0]  virt_addr_reg;
reg signed [15:0]  origin_row_s_reg;
reg signed [9:0]   left_col_blk_s_reg;

wire               state_idle;
wire               state_cfg;
wire               state_row_wait;
wire               state_emit;
wire               row_start_fire;
wire               cfg_done;
wire               emit_fire;
wire               col_is_last;
wire               row_is_last;
wire [32:0]        row_base_sum;
wire [32:0]        ring_size_ext;
wire [32:0]        row_base_wrap;

assign state_idle     = (state_reg == S1_IDLE);
assign state_cfg      = (state_reg == S1_CFG);
assign state_row_wait = (state_reg == S1_ROW_WAIT);
assign state_emit     = (state_reg == S1_EMIT);

// 等待 wrapper 侧配置寄存器（row_base_init_reg 在 conv_start 后第 3 拍就绪）
assign cfg_done       = state_cfg && (cfg_cnt_reg == 2'd2);
assign row_start_fire = state_row_wait && (row_credit_cnt_reg != 13'd0);
assign emit_fire      = state_emit && !t1_full;
assign col_is_last    = (out_col_blk_reg == cfg_fm_out_col);
assign row_is_last    = (out_row_cnt_reg == (cfg_fm_out_row - 12'd1));

assign row_base_sum  = {1'b0, row_base_reg} + {1'b0, cfg_row_step};
assign ring_size_ext = {1'b0, cfg_ring_size};
assign row_base_wrap = (row_base_sum >= ring_size_ext)
                     ? (row_base_sum - ring_size_ext)
                     : row_base_sum;

assign t1_wr_en   = emit_fire;
assign t1_wr_data = {virt_addr_reg, origin_row_s_reg, left_col_blk_s_reg, col_is_last};

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        state_reg <= S1_IDLE;
    else begin
        case (state_reg)
            S1_IDLE:
                if (conv_start)
                    state_reg <= S1_CFG;
                else
                    state_reg <= S1_IDLE;
            S1_CFG:
                if (cfg_done)
                    state_reg <= S1_ROW_WAIT;
                else
                    state_reg <= S1_CFG;
            S1_ROW_WAIT:
                if (row_start_fire)
                    state_reg <= S1_EMIT;
                else
                    state_reg <= S1_ROW_WAIT;
            S1_EMIT:
                if (emit_fire && col_is_last && row_is_last)
                    state_reg <= S1_DONE;
                else if (emit_fire && col_is_last)
                    state_reg <= S1_ROW_WAIT;
                else
                    state_reg <= S1_EMIT;
            S1_DONE:
                state_reg <= S1_IDLE;
            default:
                state_reg <= S1_IDLE;
        endcase
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        cfg_cnt_reg <= 2'd0;
    else if (conv_start)
        cfg_cnt_reg <= 2'd0;
    else if (state_cfg && !cfg_done)
        cfg_cnt_reg <= cfg_cnt_reg + 2'd1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        row_credit_cnt_reg <= 13'd0;
    else if (conv_start)
        row_credit_cnt_reg <= 13'd0;
    else if (state_idle || (state_reg == S1_DONE))
        row_credit_cnt_reg <= 13'd0;
    else begin
        case ({fm_add_sig, row_start_fire})
            2'b10:   row_credit_cnt_reg <= row_credit_cnt_reg + 13'd1;
            2'b01:   row_credit_cnt_reg <= row_credit_cnt_reg - 13'd1;
            default: row_credit_cnt_reg <= row_credit_cnt_reg;
        endcase
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        out_row_cnt_reg <= 12'd0;
    else if (conv_start)
        out_row_cnt_reg <= 12'd0;
    else if (emit_fire && col_is_last && !row_is_last)
        out_row_cnt_reg <= out_row_cnt_reg + 12'd1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        out_col_blk_reg <= 7'd0;
    else if (conv_start)
        out_col_blk_reg <= 7'd0;
    else if (row_start_fire)
        out_col_blk_reg <= 7'd0;
    else if (emit_fire && !col_is_last)
        out_col_blk_reg <= out_col_blk_reg + 7'd1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        row_base_reg <= 32'd0;
    else if (cfg_done)
        row_base_reg <= cfg_row_base_init;
    else if (emit_fire && col_is_last && !row_is_last)
        row_base_reg <= row_base_wrap[31:0];
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        virt_addr_reg <= 32'sd0;
    else if (row_start_fire)
        virt_addr_reg <= $signed({1'b0, row_base_reg}) - $signed({21'd0, cfg_cin_pair_num});
    else if (emit_fire && !col_is_last)
        virt_addr_reg <= virt_addr_reg + $signed({1'b0, cfg_col_step});
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        origin_row_s_reg <= 16'sd0;
    else if (conv_start)
        origin_row_s_reg <= cfg_origin_row_init;
    else if (emit_fire && col_is_last && !row_is_last)
        origin_row_s_reg <= origin_row_s_reg + $signed({13'd0, cfg_stride_r_real});
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        left_col_blk_s_reg <= -10'sd1;
    else if (row_start_fire)
        left_col_blk_s_reg <= -10'sd1;
    else if (emit_fire && !col_is_last)
        left_col_blk_s_reg <= left_col_blk_s_reg + $signed({7'd0, cfg_stride_c_real});
end

endmodule

// ============================================================
// RTL Self-Check Results
// ============================================================
// [PASS] Rule #1:  All ports have explicit width declarations
// [PASS] Rule #2:  Ports use Verilog wire/reg conventions
// [PASS] Rule #6:  No debug/error-attribution logic or diagnostic-only sideband in RTL
// [PASS] Rule #8:  No logic/always_ff/always_comb used
// [PASS] Rule #12: FSM case has default branch
// [PASS] Rule #16: Non-blocking '<=' for all sequential logic
// [PASS] Rule #24: No multipliers; address advance is pure accumulator
// [PASS] One register per always block
// [PASS] Declaration-before-use for all regs referenced by wire assigns
// ============================================================
