`include "all_defines.vh"

// pram_wr_switch: rdcmd layer-head 判定、load/store 相位锁步、buffer_valid 提交、
// PRAM 双写端口寄存 mux。协议见 docs/specs/cal_top/pram/pram_write_ctrl_rtl_spec.md：
//   in_sync = (load_phase == store_phase)；读 buffer = load_phase；
//   store 写 ~load_phase，load 写 load_phase，两端口真并行。
module pram_wr_switch(
    input               clk,
    input               rst_n,

    input               rdcmd_trans_valid,
    input      [111:0]  rdcmd_trans,

    // load proc 接口
    output              load_start,
    output     [11:0]   load_cfg_ch_in,
    output     [6:0]    load_cfg_col_in,
    output     [1:0]    load_cfg_pad_top,
    output     [1:0]    load_cfg_pad_bottom,
    output              in_sync,
    input               load_commit_pulse,
    input               load_wr_en,
    input               load_wr_region,
    input      [11:0]   load_wr_addr,
    input      [511:0]  load_wr_data,

    // store proc 接口
    input               store_head_fire,
    input               store_wr_en,
    input               store_wr_bot_en,
    input               store_wr_top_en,
    input      [11:0]   store_wr_bot_addr,
    input      [11:0]   store_wr_top_addr,
    input      [511:0]  store_wr_data,
    input               store_proc_idle,
    input               wrcmd_pending,

    // PRAM0 bottom（首 3 行）写端口
    output              wren0b,
    output     [11:0]   wraddr0b,
    output     [511:0]  wrdata0b,

    // PRAM0 top（末 3 行）写端口
    output              wren0t,
    output     [11:0]   wraddr0t,
    output     [511:0]  wrdata0t,

    // PRAM1 bottom 写端口
    output              wren1b,
    output     [11:0]   wraddr1b,
    output     [511:0]  wrdata1b,

    // PRAM1 top 写端口
    output              wren1t,
    output     [11:0]   wraddr1t,
    output     [511:0]  wrdata1t,

    output     [1:0]    buffer_valid
);

reg          load_phase_reg;
reg          store_phase_reg;
reg          rel_pending_reg;
reg          noint16_pending_reg;
reg [1:0]    buffer_valid_reg;
reg [5:0]    group_cnt_reg;
reg          wren0b_reg;
reg          wren0t_reg;
reg          wren1b_reg;
reg          wren1t_reg;
reg [11:0]   wraddr0b_reg;
reg [11:0]   wraddr0t_reg;
reg [511:0]  wrdata0_reg;
reg [11:0]   wraddr1b_reg;
reg [11:0]   wraddr1t_reg;
reg [511:0]  wrdata1_reg;

wire [3:0]   cmd_model_sel;
wire         cmd_is_conv;
wire         cmd_is_pool;
wire         cmd_cpad_en;
wire         cmd_cpad_sel;
wire         cmd_out_type;
wire [5:0]   cmd_channel_out;
wire         broadcast_fire;
wire         layer_head_fire;
wire         layer_head_cpad_fire;
wire         noint16_layer_head_fire;
wire         self_heal_fire;
wire [5:0]   group_cnt_load;
wire         rel_release_fire;
wire [1:0]   one_hot_load;
wire         load_hit_buf0;
wire         store_hit_buf0;

assign cmd_model_sel   = rdcmd_trans[`RD_MODEL_SEL_START +: `RD_MODEL_SEL_WIDTH];
assign cmd_is_conv     = (cmd_model_sel == `MODEL_SEL_CONV);
assign cmd_is_pool     = (cmd_model_sel == `MODEL_SEL_POOLING);
assign cmd_cpad_en     = cmd_is_conv ? rdcmd_trans[`RD_CONV_CIRCULAR_PADDING_EN_START]
                                     : rdcmd_trans[`RD_POOL_CIRCULAR_PADDING_START];
assign cmd_cpad_sel    = cmd_is_conv ? rdcmd_trans[`RD_CONV_CIRCULAR_PADDING_SEL_START]
                                     : rdcmd_trans[`RD_POOL_CIRCULAR_PADDING_SEL_START];
assign cmd_channel_out = rdcmd_trans[`RD_FM_CHANNEL_OUT_START +: `RD_FM_CHANNEL_OUT_WIDTH];
assign cmd_out_type    = rdcmd_trans[`RD_CONV_OUT_TYPE_START +: `RD_CONV_OUT_TYPE_WIDTH];

assign broadcast_fire      = rdcmd_trans_valid && (cmd_is_conv || cmd_is_pool);
assign layer_head_fire     = broadcast_fire && (cmd_is_pool || (group_cnt_reg == 6'd0));
assign layer_head_cpad_fire = layer_head_fire && cmd_cpad_en;
// 无转发层：int16 卷积（store 侧按 fm_out_type==1 过滤 wrcmd，永远等不到
// 该层首条 wrcmd），load_phase 翻而 store_phase 不翻会永久失步，需自愈。
// 覆盖 cpad_en=0 的 int16 层——其自身无需 buffer_valid，但相位奇偶必须保持，
// 否则下一个 cpad 层 sync 不上（tc315 死锁根因）。
assign noint16_layer_head_fire = layer_head_fire && cmd_is_conv && cmd_out_type;
// 自愈翻相位判据：store 侧当前数据全部写完（proc 回 IDLE 且 wrcmd 队列空）。
// proc 只在 RECV 收数据、收满才回 IDLE，故该条件成立时写指令队列、处理中
// 指令、circular 数据三者全空，绝不在写流中途翻相位。
assign self_heal_fire    = noint16_pending_reg && store_proc_idle && !wrcmd_pending;
// fm_channel_out = 组数-1（dispatch 每层发 fm_channel_out+1 条广播，与指令文档一致）：
// layer-head 之后还有 fm_channel_out 条组广播
assign group_cnt_load      = cmd_channel_out;

assign in_sync         = (load_phase_reg == store_phase_reg);
assign rel_release_fire = rel_pending_reg && in_sync && !layer_head_cpad_fire;
assign one_hot_load    = load_phase_reg ? 2'b10 : 2'b01;

assign load_start        = layer_head_cpad_fire && cmd_cpad_sel;
assign load_cfg_ch_in    = rdcmd_trans[`RD_FM_CHANNEL_IN_START +: `RD_FM_CHANNEL_IN_WIDTH];
assign load_cfg_col_in   = rdcmd_trans[`RD_FM_COLUMN_IN_START +: `RD_FM_COLUMN_IN_WIDTH];
assign load_cfg_pad_top  = rdcmd_trans[`RD_CONV_PADDING_TOP_START +: `RD_CONV_PADDING_TOP_WIDTH];
assign load_cfg_pad_bottom = rdcmd_trans[`RD_CONV_PADDING_BOTTOM_START +: `RD_CONV_PADDING_BOTTOM_WIDTH];

assign load_hit_buf0  = !load_phase_reg;
// store 写目标 = ~store_phase（当前 store era 的另一块 buffer）：
// 计算层时 store_phase==load_phase；RES/PS+FM 的 wrcmd 不翻 store_phase，
// 其流跨越下一广播 drain 时仍写入正确 era 的 buffer（下一计算层的读 buffer）
assign store_hit_buf0 = store_phase_reg;

assign wren0b   = wren0b_reg;
assign wren0t   = wren0t_reg;
assign wren1b   = wren1b_reg;
assign wren1t   = wren1t_reg;
assign wraddr0b = wraddr0b_reg;
assign wrdata0b = wrdata0_reg;
assign wraddr0t = wraddr0t_reg;
assign wrdata0t = wrdata0_reg;
assign wraddr1b = wraddr1b_reg;
assign wrdata1b = wrdata1_reg;
assign wraddr1t = wraddr1t_reg;
assign wrdata1t = wrdata1_reg;
assign buffer_valid = buffer_valid_reg;

// load_phase: 每条 CONV/POOL layer-head 广播（含 cpad_en=0）翻转一次
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        load_phase_reg <= 1'b0;
    else if (layer_head_fire)
        load_phase_reg <= ~load_phase_reg;
end

// store_phase: 接受新一层首条 wrcmd（自-paced，蕴含上一层写完）翻转一次；
// 无转发层（int16）等不到 wrcmd，由 self_heal_fire 在上层写完后补翻
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        store_phase_reg <= 1'b0;
    else if (store_head_fire || self_heal_fire)
        store_phase_reg <= ~store_phase_reg;
end

// 无转发层挂起：int16 卷积层头置位，自愈翻相位后清位
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        noint16_pending_reg <= 1'b0;
    else if (noint16_layer_head_fire)
        noint16_pending_reg <= 1'b1;
    else if (self_heal_fire)
        noint16_pending_reg <= 1'b0;
end

// CONV 多组广播过滤：layer-head 时装载剩余组数，后续组广播只递减
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        group_cnt_reg <= 6'd0;
    else if (broadcast_fire && cmd_is_conv) begin
        if (group_cnt_reg == 6'd0)
            group_cnt_reg <= group_cnt_load;
        else
            group_cnt_reg <= group_cnt_reg - 6'd1;
    end
end

// sel=0 放行挂起：layer-head 置位（sel=0），in_sync 成立即释放
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        rel_pending_reg <= 1'b0;
    else if (layer_head_cpad_fire)
        rel_pending_reg <= ~cmd_cpad_sel;
    else if (rel_release_fire)
        rel_pending_reg <= 1'b0;
end

// buffer_valid：广播一律先拉低一拍；sel=0 相位同步放行；sel=1 load commit 放行
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        buffer_valid_reg <= 2'b00;
    else if (layer_head_cpad_fire)
        buffer_valid_reg <= 2'b00;
    else if (rel_release_fire)
        buffer_valid_reg <= one_hot_load;
    else if (load_commit_pulse)
        buffer_valid_reg <= one_hot_load;
end

// 写端口寄存 mux：load 写 load_phase 侧，store 写 ~load_phase 侧，恒不冲突；
// store 侧 bot/top 使能直通（小行数交集行同拍双使能，两块独立 RAM 同拍写）
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        wren0b_reg <= 1'b0;
    else
        wren0b_reg <= (load_wr_en && load_hit_buf0 && !load_wr_region) ||
                      (store_wr_en && store_hit_buf0 && store_wr_bot_en);
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        wren0t_reg <= 1'b0;
    else
        wren0t_reg <= (load_wr_en && load_hit_buf0 && load_wr_region) ||
                      (store_wr_en && store_hit_buf0 && store_wr_top_en);
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        wren1b_reg <= 1'b0;
    else
        wren1b_reg <= (load_wr_en && !load_hit_buf0 && !load_wr_region) ||
                      (store_wr_en && !store_hit_buf0 && store_wr_bot_en);
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        wren1t_reg <= 1'b0;
    else
        wren1t_reg <= (load_wr_en && !load_hit_buf0 && load_wr_region) ||
                      (store_wr_en && !store_hit_buf0 && store_wr_top_en);
end

// buf0 bottom 地址 mux（store_hit_buf0 时写 store，否则写 load）
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        wraddr0b_reg <= 12'd0;
    else if (store_hit_buf0)
        wraddr0b_reg <= store_wr_bot_addr;
    else
        wraddr0b_reg <= load_wr_addr;
end

always @(posedge clk) begin
    if (store_hit_buf0)
        wrdata0_reg <= store_wr_data;
    else
        wrdata0_reg <= load_wr_data;
end

// buf0 top 地址 mux
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        wraddr0t_reg <= 12'd0;
    else if (store_hit_buf0)
        wraddr0t_reg <= store_wr_top_addr;
    else
        wraddr0t_reg <= load_wr_addr;
end

// buf1 bottom 地址 mux
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        wraddr1b_reg <= 12'd0;
    else if (!store_hit_buf0)
        wraddr1b_reg <= store_wr_bot_addr;
    else
        wraddr1b_reg <= load_wr_addr;
end

always @(posedge clk) begin
    if (!store_hit_buf0)
        wrdata1_reg <= store_wr_data;
    else
        wrdata1_reg <= load_wr_data;
end

// buf1 top 地址 mux
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        wraddr1t_reg <= 12'd0;
    else if (!store_hit_buf0)
        wraddr1t_reg <= store_wr_top_addr;
    else
        wraddr1t_reg <= load_wr_addr;
end

endmodule

// ============================================================
// RTL Self-Check Results
// ============================================================
// [PASS] Rule #1:  All ports have explicit width declarations
// [PASS] Rule #2:  Ports use Verilog wire/reg conventions
// [PASS] Rule #6:  No debug/error-attribution logic or diagnostic-only sideband in RTL
// [PASS] Rule #8:  No logic/always_ff/always_comb used
// [PASS] Rule #12: No combinational always blocks, no latch risk
// [PASS] Rule #16: Sequential logic uses non-blocking '<=' only
// [PASS] Rule #26: No FSM in this module; counters have explicit widths
// [PASS] Rule #43: No function/task used; field decode uses macros
// [PASS] Dual-write: store bot/top enables pass through to independent region
//        RAM write ports; per-region address registers (wraddrXb/Xt split);
//        load path behavior unchanged
// ============================================================
