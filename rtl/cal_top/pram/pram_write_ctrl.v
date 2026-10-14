// pram_write_ctrl: 循环 padding RAM（PRAM）写入控制顶层。
// 子模块划分（规格 docs/specs/cal_top/pram/pram_write_ctrl_rtl_spec.md）：
//   pram_load_wr_proc  - load 流写入 FSM 与地址生成
//   pram_store_wr_proc - store 流写入 FSM 与地址生成
//   pram_wr_switch     - 相位锁步、buffer_valid 提交、双写端口寄存 mux
module pram_write_ctrl(
    input               clk,
    input               rst_n,

    input               rdcmd_trans_valid,
    input      [111:0]  rdcmd_trans,

    input               s_axis_wrcmd_tvalid,
    input      [103:0]  s_axis_wrcmd_tdata,
    output              s_axis_wrcmd_tready,

    input               s_axis_cpad_load_tvalid,
    input      [511:0]  s_axis_cpad_load_tdata,
    output              s_axis_cpad_load_tready,

    input               s_axis_cpad_store_tvalid,
    input      [511:0]  s_axis_cpad_store_tdata,
    output              s_axis_cpad_store_tready,

    output              wren0b,
    output     [11:0]   wraddr0b,
    output     [511:0]  wrdata0b,

    output              wren0t,
    output     [11:0]   wraddr0t,
    output     [511:0]  wrdata0t,

    output              wren1b,
    output     [11:0]   wraddr1b,
    output     [511:0]  wrdata1b,

    output              wren1t,
    output     [11:0]   wraddr1t,
    output     [511:0]  wrdata1t,

    output     [1:0]    buffer_valid
);

wire         load_start;
wire [11:0]  load_cfg_ch_in;
wire [6:0]   load_cfg_col_in;
wire [1:0]   load_cfg_pad_top;
wire [1:0]   load_cfg_pad_bottom;
wire         in_sync;
wire         load_commit_pulse;
wire         load_wr_en;
wire         load_wr_region;
wire [11:0]  load_wr_addr;
wire [511:0] load_wr_data;
wire         store_head_fire;
wire         store_wr_en;
wire         store_wr_bot_en;
wire         store_wr_top_en;
wire [11:0]  store_wr_bot_addr;
wire [11:0]  store_wr_top_addr;
wire [511:0] store_wr_data;
wire         store_proc_idle;

pram_load_wr_proc u_load_proc(
    .clk                     (clk),
    .rst_n                   (rst_n),
    .load_start              (load_start),
    .cfg_ch_in               (load_cfg_ch_in),
    .cfg_col_in              (load_cfg_col_in),
    .cfg_pad_top             (load_cfg_pad_top),
    .cfg_pad_bottom          (load_cfg_pad_bottom),
    .in_sync                 (in_sync),
    .s_axis_cpad_load_tvalid (s_axis_cpad_load_tvalid),
    .s_axis_cpad_load_tdata  (s_axis_cpad_load_tdata),
    .s_axis_cpad_load_tready (s_axis_cpad_load_tready),
    .load_wr_en              (load_wr_en),
    .load_wr_region          (load_wr_region),
    .load_wr_addr            (load_wr_addr),
    .load_wr_data            (load_wr_data),
    .load_commit_pulse       (load_commit_pulse)
);

pram_store_wr_proc u_store_proc(
    .clk                      (clk),
    .rst_n                    (rst_n),
    .s_axis_wrcmd_tvalid      (s_axis_wrcmd_tvalid),
    .s_axis_wrcmd_tdata       (s_axis_wrcmd_tdata),
    .s_axis_wrcmd_tready      (s_axis_wrcmd_tready),
    .s_axis_cpad_store_tvalid (s_axis_cpad_store_tvalid),
    .s_axis_cpad_store_tdata  (s_axis_cpad_store_tdata),
    .s_axis_cpad_store_tready (s_axis_cpad_store_tready),
    .store_head_fire          (store_head_fire),
    .store_wr_en              (store_wr_en),
    .store_wr_bot_en          (store_wr_bot_en),
    .store_wr_top_en          (store_wr_top_en),
    .store_wr_bot_addr        (store_wr_bot_addr),
    .store_wr_top_addr        (store_wr_top_addr),
    .store_wr_data            (store_wr_data),
    .proc_idle                (store_proc_idle)
);

pram_wr_switch u_switch(
    .clk                (clk),
    .rst_n              (rst_n),
    .rdcmd_trans_valid  (rdcmd_trans_valid),
    .rdcmd_trans        (rdcmd_trans),
    .load_start         (load_start),
    .load_cfg_ch_in     (load_cfg_ch_in),
    .load_cfg_col_in    (load_cfg_col_in),
    .load_cfg_pad_top   (load_cfg_pad_top),
    .load_cfg_pad_bottom(load_cfg_pad_bottom),
    .in_sync            (in_sync),
    .load_commit_pulse  (load_commit_pulse),
    .load_wr_en         (load_wr_en),
    .load_wr_region     (load_wr_region),
    .load_wr_addr       (load_wr_addr),
    .load_wr_data       (load_wr_data),
    .store_head_fire    (store_head_fire),
    .store_wr_en        (store_wr_en),
    .store_wr_bot_en    (store_wr_bot_en),
    .store_wr_top_en    (store_wr_top_en),
    .store_wr_bot_addr  (store_wr_bot_addr),
    .store_wr_top_addr  (store_wr_top_addr),
    .store_wr_data      (store_wr_data),
    .store_proc_idle    (store_proc_idle),
    .wrcmd_pending      (s_axis_wrcmd_tvalid),
    .wren0b             (wren0b),
    .wraddr0b           (wraddr0b),
    .wrdata0b           (wrdata0b),
    .wren0t             (wren0t),
    .wraddr0t           (wraddr0t),
    .wrdata0t           (wrdata0t),
    .wren1b             (wren1b),
    .wraddr1b           (wraddr1b),
    .wrdata1b           (wrdata1b),
    .wren1t             (wren1t),
    .wraddr1t           (wraddr1t),
    .wrdata1t           (wrdata1t),
    .buffer_valid       (buffer_valid)
);

endmodule

// ============================================================
// RTL Self-Check Results
// ============================================================
// [PASS] Rule #1:  All ports have explicit width declarations
// [PASS] Rule #2:  Ports use Verilog wire/reg conventions
// [PASS] Rule #6:  No debug/error-attribution logic or diagnostic-only sideband in RTL
// [PASS] Rule #8:  No logic/always_ff/always_comb used
// [PASS] Rule #34: Port list matches the fixed integration contract in the spec
// [NOTE] Module name changed from the old shell's cpad_wr_ctrl to pram_write_ctrl
//        per spec; file name unchanged
// ============================================================
