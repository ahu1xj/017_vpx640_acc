// res_input_ctrl：residual 模块族输入控制。
// 职责（2026-07-24 冻结架构）：
// - RESADD/BYPASS 首拍保持配置边界的一拍启动对齐，运行中 ready 由本地 datapath/output elastic occupancy 精确产生；
//   UPSAMPLE ready 仍由行缓冲写侧寄存控制。
// - RESADD：res_det = mode & ~done & dp_free；两路 valid 齐即收，原子成对。
//   fm1/fm2 ready 互以对方 valid 门控，保证不会单路先走。
//   逐点缩减流（int16->int8 且两路合一）无积压，base 一拍滞后的下溢由
//   datapath pipe 在途 1 组天然吸收。
// - BYPASS（CONCAT/SPLIT）：ready = mode_bypass & ~done & out_space；
//   直接使用本地输出寄存/skid 空间，不再依赖深 FIFO credit 或一拍预测。
// - UPSAMPLE 写侧：整行接收进乒乓行缓冲；行边界检查另一缓冲 release；
//   released[1:0] 由 datapath 的 release 脉冲置位、写启动清零（此处以
//   valid/loading 寄存器组承载同一语义）；ready 寄存器化，带末拍/末行预测。
// - 位置计数（ch 内 col 外，与原逻辑一致）与尾列 sideband（位置恒等映射）：
//   sideband_valid_col = 尾块 ? field+1 : 0（5bit 回绕，0=全有效约定）。
// 反压只由本地 datapath 和 registered skid occupancy 决定；store_top 下游
// ready 不组合穿透本模块。
module res_input_ctrl #(
    parameter UPSAMPLE_ROW_BUF_ADDR_WIDTH = 9
) (
    input  wire                          clk,
    input  wire                          rst_n,

    input  wire                          cfg_valid,
    input  wire [2:0]                    cfg_mode,
    input  wire [11:0]                   cfg_normal_ch_pair,
    input  wire [6:0]                    cfg_in_col_blk,
    input  wire [4:0]                    cfg_last_col_m1,
    input  wire [31:0]                   cfg_input_total,
    input  wire [11:0]                   cfg_in_row,
    input  wire [31:0]                   cfg_up_row_beats,

    input  wire                          out_space,

    // 上游流
    input  wire                          s_axis_res_fm1_tvalid,
    output wire                          s_axis_res_fm1_tready,
    input  wire                          s_axis_res_fm2_tvalid,
    output wire                          s_axis_res_fm2_tready,

    // resadd datapath
    input  wire                          resadd_dp_free,
    output wire                          pair_load,
    output wire [4:0]                    pair_valid_col,

    // bypass 转发（输出控制施加掩码）
    output wire                          bypass_accept,
    output wire [4:0]                    bypass_valid_col,

    // upsample 行缓冲写侧（RAM 在 datapath 内）
    output wire [1:0]                    up_wr_en,
    output wire [UPSAMPLE_ROW_BUF_ADDR_WIDTH-1:0] up_wr_addr,
    input  wire                          up_buf_release,
    input  wire                          up_buf_release_sel,
    output wire                          up_buf0_valid,
    output wire                          up_buf1_valid,
    output wire [11:0]                   up_buf0_row,
    output wire [11:0]                   up_buf1_row,

    // done 输出
    output wire                          resadd_issue_done,
    output wire                          up_buf_idle
);

localparam [2:0] MODE_RESADD   = 3'd1;
localparam [2:0] MODE_UPSAMPLE = 3'd2;
localparam [2:0] MODE_BYPASS   = 3'd3;

localparam integer UP_CNT_WIDTH = UPSAMPLE_ROW_BUF_ADDR_WIDTH + 1;

// ---- 寄存器 ----
reg                                     normal_mode_armed_q;
reg                                     up_ready_q;
reg [31:0]                              issue_cnt_q;
reg [11:0]                              ch_idx_q;
reg [6:0]                               col_idx_q;
reg [11:0]                              up_wr_row_cnt_q;
reg [UP_CNT_WIDTH-1:0]                  up_buf0_wr_cnt_q;
reg [UP_CNT_WIDTH-1:0]                  up_buf1_wr_cnt_q;
reg                                     up_buf0_loading_q;
reg                                     up_buf1_loading_q;
reg                                     up_buf0_valid_q;
reg                                     up_buf1_valid_q;
reg [11:0]                              up_buf0_row_q;
reg [11:0]                              up_buf1_row_q;
reg                                     up_wr_buf_sel_q;

// ---- 组合信号 ----
wire                                    mode_is_resadd;
wire                                    mode_is_upsample;
wire                                    mode_is_bypass;
wire                                    issue_done;
wire                                    res_det;
wire                                    pair_accept;
wire                                    bypass_det;
wire                                    normal_step;
wire                                    ch_last;
wire                                    col_last;
wire [4:0]                              sideband_valid_col;
wire                                    up_accept_prev;
wire                                    up_sel_last_beat_prev;
wire                                    up_done_pred;
wire                                    up_buf0_last_beat;
wire                                    up_buf1_last_beat;
wire                                    up_buf0_can;
wire                                    up_buf1_can;
wire                                    up_buf0_can_cur;
wire                                    up_buf1_can_cur;
wire                                    up_accept;
wire                                    up_buf0_write_fire;
wire                                    up_buf1_write_fire;
wire                                    up_wr_row_last_beat;
wire                                    up_buf0_write_done;
wire                                    up_buf1_write_done;
wire [UP_CNT_WIDTH-1:0]                 up_row_beats_m1;

assign mode_is_resadd   = (cfg_mode == MODE_RESADD);
assign mode_is_upsample = (cfg_mode == MODE_UPSAMPLE);
assign mode_is_bypass   = (cfg_mode == MODE_BYPASS);

// ---------------- 位置计数与尾列 sideband（ch 内 col 外，恒等映射） ----------------
assign issue_done         = (issue_cnt_q >= cfg_input_total);
assign ch_last            = (cfg_normal_ch_pair <= 12'd1) |
                            (ch_idx_q == (cfg_normal_ch_pair - 12'd1));
assign col_last           = (cfg_in_col_blk <= 7'd1) |
                            (col_idx_q == (cfg_in_col_blk - 7'd1));
assign sideband_valid_col = col_last ? (cfg_last_col_m1 + 5'd1) : 5'd0;

// ---------------- RESADD：本地精确 ready ----------------
// 逐点缩减流无积压：dp_free 在 pipe 可装载/可排空时同拍放行。
assign res_det      = mode_is_resadd & normal_mode_armed_q & (~cfg_valid) &
                      (~issue_done) & resadd_dp_free;
assign pair_accept  = res_det & s_axis_res_fm1_tvalid & s_axis_res_fm2_tvalid;

// ---------------- BYPASS：本地精确 ready ----------------
assign bypass_det     = mode_is_bypass & normal_mode_armed_q & (~cfg_valid) &
                        (~issue_done) & out_space;
assign bypass_accept  = bypass_det & s_axis_res_fm1_tvalid;

assign normal_step        = pair_accept | bypass_accept;

// ---------------- UPSAMPLE 写侧 ----------------
assign up_row_beats_m1    = cfg_up_row_beats[UP_CNT_WIDTH-1:0] - {{UP_CNT_WIDTH-1{1'b0}}, 1'b1};
assign up_accept_prev     = up_ready_q & s_axis_res_fm1_tvalid;
assign up_buf0_last_beat  = (up_buf0_wr_cnt_q == up_row_beats_m1);
assign up_buf1_last_beat  = (up_buf1_wr_cnt_q == up_row_beats_m1);
assign up_sel_last_beat_prev = up_accept_prev &
                               ((up_wr_buf_sel_q == 1'b0) ? up_buf0_last_beat : up_buf1_last_beat);
assign up_done_pred       = ({1'b0, up_wr_row_cnt_q} + {12'd0, up_sel_last_beat_prev})
                            >= {1'b0, cfg_in_row};
// ready 寄存器化的一拍在途预测：上一拍若收进末拍，本拍该缓冲已满（valid 置位），
// 必须提前封锁；读占用期间 valid 恒 1（release 才清），故 ~valid 已覆盖 read_busy。
assign up_buf0_can        = (~up_buf0_valid_q) &
                            ~(up_accept_prev & (up_wr_buf_sel_q == 1'b0) & up_buf0_last_beat);
assign up_buf1_can        = (~up_buf1_valid_q) &
                            ~(up_accept_prev & (up_wr_buf_sel_q == 1'b1) & up_buf1_last_beat);
assign up_buf0_can_cur    = ~up_buf0_valid_q;
assign up_buf1_can_cur    = ~up_buf1_valid_q;

assign up_accept          = up_ready_q & s_axis_res_fm1_tvalid;
assign up_buf0_write_fire = up_accept & (up_wr_buf_sel_q == 1'b0);
assign up_buf1_write_fire = up_accept & (up_wr_buf_sel_q == 1'b1);
assign up_wr_row_last_beat = (up_wr_buf_sel_q == 1'b0) ? up_buf0_last_beat : up_buf1_last_beat;
assign up_buf0_write_done = up_buf0_write_fire & up_wr_row_last_beat;
assign up_buf1_write_done = up_buf1_write_fire & up_wr_row_last_beat;

// ---------------- ready / 端口输出 ----------------
assign s_axis_res_fm1_tready = (res_det & s_axis_res_fm2_tvalid) |
                               bypass_det | up_ready_q;
assign s_axis_res_fm2_tready = res_det & s_axis_res_fm1_tvalid;

assign pair_load        = pair_accept;
assign pair_valid_col   = sideband_valid_col;
assign bypass_valid_col = sideband_valid_col;

assign up_wr_en         = {up_buf1_write_fire, up_buf0_write_fire};
assign up_wr_addr       = (up_wr_buf_sel_q == 1'b0) ?
                          up_buf0_wr_cnt_q[UPSAMPLE_ROW_BUF_ADDR_WIDTH-1:0] :
                          up_buf1_wr_cnt_q[UPSAMPLE_ROW_BUF_ADDR_WIDTH-1:0];
assign up_buf0_valid    = up_buf0_valid_q;
assign up_buf1_valid    = up_buf1_valid_q;
assign up_buf0_row      = up_buf0_row_q;
assign up_buf1_row      = up_buf1_row_q;

assign resadd_issue_done = issue_done;
assign up_buf_idle       = (~up_buf0_valid_q) & (~up_buf1_valid_q) &
                           (~up_buf0_loading_q) & (~up_buf1_loading_q);

// ---------------- normal 模式启动对齐 + UPSAMPLE registered ready ----------------
// res_ctrl 在 cfg_valid 后一拍才把 mode 切到 RESADD/BYPASS；这里再寄存一拍
// normal_mode_armed_q，保持原接口首拍对齐。运行中停止仍由 dp_free/out_space
// 当拍精确控制，不引入 registered-ready 的额外 over-accept。
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        normal_mode_armed_q <= 1'b0;
    else if (cfg_valid)
        normal_mode_armed_q <= 1'b0;
    else if (!(mode_is_resadd | mode_is_bypass))
        normal_mode_armed_q <= 1'b0;
    else
        normal_mode_armed_q <= 1'b1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        up_ready_q <= 1'b0;
    else if (cfg_valid)
        up_ready_q <= 1'b0;
    else
        up_ready_q <= mode_is_upsample & (~up_done_pred) &
                      ((up_wr_buf_sel_q == 1'b0) ? up_buf0_can : up_buf1_can);
end

// ---------------- 接收/位置计数 ----------------
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        issue_cnt_q <= 32'd0;
    else if (cfg_valid)
        issue_cnt_q <= 32'd0;
    else if (normal_step)
        issue_cnt_q <= issue_cnt_q + 32'd1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        ch_idx_q <= 12'd0;
    else if (cfg_valid)
        ch_idx_q <= 12'd0;
    else if (normal_step) begin
        if (ch_last)
            ch_idx_q <= 12'd0;
        else
            ch_idx_q <= ch_idx_q + 12'd1;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        col_idx_q <= 7'd0;
    else if (cfg_valid)
        col_idx_q <= 7'd0;
    else if (normal_step && ch_last) begin
        if (col_last)
            col_idx_q <= 7'd0;
        else
            col_idx_q <= col_idx_q + 7'd1;
    end
end

// ---------------- UPSAMPLE 写侧寄存器组 ----------------
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        up_wr_row_cnt_q <= 12'd0;
    else if (cfg_valid)
        up_wr_row_cnt_q <= 12'd0;
    else if (up_buf0_write_done || up_buf1_write_done)
        up_wr_row_cnt_q <= up_wr_row_cnt_q + 12'd1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        up_wr_buf_sel_q <= 1'b0;
    else if (cfg_valid)
        up_wr_buf_sel_q <= 1'b0;
    else if (up_buf0_write_done)
        up_wr_buf_sel_q <= 1'b1;
    else if (up_buf1_write_done)
        up_wr_buf_sel_q <= 1'b0;
    else if ((up_wr_buf_sel_q == 1'b0) && (!up_buf0_can_cur) && up_buf1_can_cur)
        up_wr_buf_sel_q <= 1'b1;
    else if ((up_wr_buf_sel_q == 1'b1) && (!up_buf1_can_cur) && up_buf0_can_cur)
        up_wr_buf_sel_q <= 1'b0;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        up_buf0_wr_cnt_q <= {UP_CNT_WIDTH{1'b0}};
    else if (cfg_valid)
        up_buf0_wr_cnt_q <= {UP_CNT_WIDTH{1'b0}};
    else if (up_buf0_write_done)
        up_buf0_wr_cnt_q <= {UP_CNT_WIDTH{1'b0}};
    else if (up_buf0_write_fire)
        up_buf0_wr_cnt_q <= up_buf0_wr_cnt_q + {{UP_CNT_WIDTH-1{1'b0}}, 1'b1};
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        up_buf1_wr_cnt_q <= {UP_CNT_WIDTH{1'b0}};
    else if (cfg_valid)
        up_buf1_wr_cnt_q <= {UP_CNT_WIDTH{1'b0}};
    else if (up_buf1_write_done)
        up_buf1_wr_cnt_q <= {UP_CNT_WIDTH{1'b0}};
    else if (up_buf1_write_fire)
        up_buf1_wr_cnt_q <= up_buf1_wr_cnt_q + {{UP_CNT_WIDTH-1{1'b0}}, 1'b1};
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        up_buf0_loading_q <= 1'b0;
    else if (cfg_valid)
        up_buf0_loading_q <= 1'b0;
    else if (up_buf0_write_done)
        up_buf0_loading_q <= 1'b0;
    else if (up_buf0_write_fire)
        up_buf0_loading_q <= 1'b1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        up_buf1_loading_q <= 1'b0;
    else if (cfg_valid)
        up_buf1_loading_q <= 1'b0;
    else if (up_buf1_write_done)
        up_buf1_loading_q <= 1'b0;
    else if (up_buf1_write_fire)
        up_buf1_loading_q <= 1'b1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        up_buf0_valid_q <= 1'b0;
    else if (cfg_valid)
        up_buf0_valid_q <= 1'b0;
    else if (up_buf_release && (up_buf_release_sel == 1'b0))
        up_buf0_valid_q <= 1'b0;
    else if (up_buf0_write_done)
        up_buf0_valid_q <= 1'b1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        up_buf1_valid_q <= 1'b0;
    else if (cfg_valid)
        up_buf1_valid_q <= 1'b0;
    else if (up_buf_release && (up_buf_release_sel == 1'b1))
        up_buf1_valid_q <= 1'b0;
    else if (up_buf1_write_done)
        up_buf1_valid_q <= 1'b1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        up_buf0_row_q <= 12'd0;
    else if (up_buf0_write_fire && (!up_buf0_loading_q))
        up_buf0_row_q <= up_wr_row_cnt_q;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        up_buf1_row_q <= 12'd0;
    else if (up_buf1_write_fire && (!up_buf1_loading_q))
        up_buf1_row_q <= up_wr_row_cnt_q;
end

endmodule

// ============================================================
// RTL Self-Check Results
// ============================================================
// [PASS] Rule #1:  All ports have explicit width declarations
// [PASS] Rule #2:  Ports use Verilog wire/reg conventions
// [PASS] Rule #6:  No debug/error-attribution logic or diagnostic-only sideband in RTL
// [PASS] Rule #8:  No logic/always_ff/always_comb used
// [PASS] Rule #12: All conditional assigns have both branches (no latch)
// [PASS] Rule #16: Non-blocking '<=' for all sequential logic
// [PASS] One register per always block
// [PASS] Declaration-before-use for all regs referenced by wire assigns
// [PASS] RESADD/BYPASS use one-cycle start alignment plus exact local run-time ready; UPSAMPLE ready remains registered
// [PASS] RESADD continuous acceptance uses exact local dp_free; FM1/FM2 pair acceptance is atomic
// [PASS] No deep-FIFO credit prediction or registered-ready over-accept path
// [PASS] After start alignment, ready depends only on local datapath/output-skid occupancy
// [PASS] Tail sideband is point-wise identity from ch/col counters (0 = all-valid convention)
// ============================================================
