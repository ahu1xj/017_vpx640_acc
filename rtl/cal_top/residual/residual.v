// residual：RESADD/UPSAMPLE/CONCAT/SPLIT 模块族顶层。
// 模块名 resadd、参数与端口列表与原 residual.v 完全一致（cal_top 例化不变）。
// 仅做例化与连线：
//   res_ctrl              配置通道（两阶段配置协议 + 分级乘法）
//   res_input_ctrl        输入控制（RESADD/BYPASS 精确 ready、位置计数、UPSAMPLE 写侧）
//   resadd_datapath       残差加 datapath（16 加法器两拍、256b 掩码、pack）
//   res_upsample_datapath 上采样 datapath + 两块行缓冲 RAM（读侧照抄原实现）
//   res_output_ctrl       输出控制（三源静态选择、bypass 掩码、输出寄存）
// 数据直连：resadd 对数据与 bypass 数据直接取自 s_axis（sideband 随行），
// 不经过 input ctrl 转发 512b 总线。
`include "all_defines.vh"

module resadd #(
    parameter UPSAMPLE_ROW_BUF_ADDR_WIDTH  = 9,
    parameter UPSAMPLE_ROW_BUF_DEPTH       = 512
) (
    input  wire                                      clk,
    input  wire                                      rst_n,
    input  wire                                      rdcmd_trans_valid,
    input  wire [`RESADD_INCMD-1:0]                  rdcmd_trans,
    input  wire                                      s_axis_res_fm1_tvalid,
    input  wire [`RESADD_FMINDATA_WIDTH-1:0]         s_axis_res_fm1_tdata,
    output wire                                      s_axis_res_fm1_tready,
    input  wire                                      s_axis_res_fm2_tvalid,
    input  wire [`RESADD_FMINDATA_WIDTH-1:0]         s_axis_res_fm2_tdata,
    output wire                                      s_axis_res_fm2_tready,
    output wire                                      m_axis_res_tvalid,
    output wire [`RESADD_OUTDATA_WIDTH-1:0]          m_axis_res_tdata,
    input  wire                                      m_axis_res_tready
);

// ---- 配置通道 ----
wire         cfg_valid;
wire [2:0]   cfg_mode;
wire [11:0]  cfg_normal_ch_pair;
wire [11:0]  cfg_in_ch_pair;
wire [6:0]   cfg_in_col_blk;
wire [4:0]   cfg_last_col_m1;
wire [31:0]  cfg_input_total;
wire [31:0]  cfg_out_total;
wire [11:0]  cfg_in_row;
wire [31:0]  cfg_up_row_beats;
wire         cfg_upsample_type;
wire [4:0]   cfg_up_tail_valid_col;
wire [5:0]   cfg_up_in_tail_col;

// ---- 模块间连线 ----
wire         out_space;
wire         out_valid;
wire         normal_out_done;
wire         resadd_issue_done;
wire         resadd_dp_free;
wire         resadd_dp_busy;
wire         pair_load;
wire [4:0]   pair_valid_col;
wire         resadd_out_valid;
wire [511:0] resadd_out_data;
wire         bypass_accept;
wire [4:0]   bypass_valid_col;
wire [1:0]   up_wr_en;
wire [UPSAMPLE_ROW_BUF_ADDR_WIDTH-1:0] up_wr_addr;
wire         up_buf_release;
wire         up_buf_release_sel;
wire         up_buf0_valid;
wire         up_buf1_valid;
wire [11:0]  up_buf0_row;
wire [11:0]  up_buf1_row;
wire         up_buf_idle;
wire         up_out_valid;
wire [511:0] up_out_data;
wire         up_read_seq_done;
wire         output_ctrl_tvalid;
wire [511:0] output_ctrl_tdata;
wire         output_skid_s_ready;

res_ctrl u_res_ctrl (
    .clk                    (clk),
    .rst_n                  (rst_n),
    .rdcmd_trans_valid      (rdcmd_trans_valid),
    .rdcmd_trans            (rdcmd_trans),
    .resadd_issue_done      (resadd_issue_done),
    .normal_out_done        (normal_out_done),
    .resadd_dp_busy         (resadd_dp_busy),
    .up_read_seq_done       (up_read_seq_done),
    .up_buf_idle            (up_buf_idle),
    .out_valid              (out_valid),
    .cfg_valid              (cfg_valid),
    .cfg_mode               (cfg_mode),
    .cfg_normal_ch_pair     (cfg_normal_ch_pair),
    .cfg_in_ch_pair         (cfg_in_ch_pair),
    .cfg_in_col_blk         (cfg_in_col_blk),
    .cfg_last_col_m1        (cfg_last_col_m1),
    .cfg_input_total        (cfg_input_total),
    .cfg_out_total          (cfg_out_total),
    .cfg_in_row             (cfg_in_row),
    .cfg_up_row_beats       (cfg_up_row_beats),
    .cfg_upsample_type      (cfg_upsample_type),
    .cfg_up_tail_valid_col  (cfg_up_tail_valid_col),
    .cfg_up_in_tail_col     (cfg_up_in_tail_col)
);

res_input_ctrl #(
    .UPSAMPLE_ROW_BUF_ADDR_WIDTH (UPSAMPLE_ROW_BUF_ADDR_WIDTH)
) u_res_input_ctrl (
    .clk                    (clk),
    .rst_n                  (rst_n),
    .cfg_valid              (cfg_valid),
    .cfg_mode               (cfg_mode),
    .cfg_normal_ch_pair     (cfg_normal_ch_pair),
    .cfg_in_col_blk         (cfg_in_col_blk),
    .cfg_last_col_m1        (cfg_last_col_m1),
    .cfg_input_total        (cfg_input_total),
    .cfg_in_row             (cfg_in_row),
    .cfg_up_row_beats       (cfg_up_row_beats),
    .out_space              (out_space),
    .s_axis_res_fm1_tvalid  (s_axis_res_fm1_tvalid),
    .s_axis_res_fm1_tready  (s_axis_res_fm1_tready),
    .s_axis_res_fm2_tvalid  (s_axis_res_fm2_tvalid),
    .s_axis_res_fm2_tready  (s_axis_res_fm2_tready),
    .resadd_dp_free         (resadd_dp_free),
    .pair_load              (pair_load),
    .pair_valid_col         (pair_valid_col),
    .bypass_accept          (bypass_accept),
    .bypass_valid_col       (bypass_valid_col),
    .up_wr_en               (up_wr_en),
    .up_wr_addr             (up_wr_addr),
    .up_buf_release         (up_buf_release),
    .up_buf_release_sel     (up_buf_release_sel),
    .up_buf0_valid          (up_buf0_valid),
    .up_buf1_valid          (up_buf1_valid),
    .up_buf0_row            (up_buf0_row),
    .up_buf1_row            (up_buf1_row),
    .resadd_issue_done      (resadd_issue_done),
    .up_buf_idle            (up_buf_idle)
);

resadd_datapath u_resadd_datapath (
    .clk                    (clk),
    .rst_n                  (rst_n),
    .cfg_valid              (cfg_valid),
    .pair_load              (pair_load),
    .pair_valid_col         (pair_valid_col),
    .pair_fm1_data          (s_axis_res_fm1_tdata),
    .pair_fm2_data          (s_axis_res_fm2_tdata),
    .out_space              (out_space),
    .dp_free                (resadd_dp_free),
    .dp_busy                (resadd_dp_busy),
    .dp_out_valid           (resadd_out_valid),
    .dp_out_data            (resadd_out_data)
);

res_upsample_datapath #(
    .UPSAMPLE_ROW_BUF_ADDR_WIDTH (UPSAMPLE_ROW_BUF_ADDR_WIDTH),
    .UPSAMPLE_ROW_BUF_DEPTH      (UPSAMPLE_ROW_BUF_DEPTH)
) u_res_upsample_datapath (
    .clk                    (clk),
    .rst_n                  (rst_n),
    .cfg_valid              (cfg_valid),
    .cfg_mode               (cfg_mode),
    .cfg_in_row             (cfg_in_row),
    .cfg_in_col_blk         (cfg_in_col_blk),
    .cfg_in_ch_pair         (cfg_in_ch_pair),
    .cfg_upsample_type      (cfg_upsample_type),
    .cfg_up_tail_valid_col  (cfg_up_tail_valid_col),
    .up_wr_en               (up_wr_en),
    .up_wr_addr             (up_wr_addr),
    .up_wr_data             (s_axis_res_fm1_tdata),
    .up_buf0_valid          (up_buf0_valid),
    .up_buf1_valid          (up_buf1_valid),
    .up_buf0_row            (up_buf0_row),
    .up_buf1_row            (up_buf1_row),
    .up_buf_release         (up_buf_release),
    .up_buf_release_sel     (up_buf_release_sel),
    .out_space              (out_space),
    .up_out_valid           (up_out_valid),
    .up_out_data            (up_out_data),
    .up_read_seq_done       (up_read_seq_done)
);

res_output_ctrl u_res_output_ctrl (
    .clk                    (clk),
    .rst_n                  (rst_n),
    .cfg_valid              (cfg_valid),
    .cfg_mode               (cfg_mode),
    .cfg_out_total          (cfg_out_total),
    .resadd_out_valid       (resadd_out_valid),
    .resadd_out_data        (resadd_out_data),
    .bypass_accept          (bypass_accept),
    .bypass_valid_col       (bypass_valid_col),
    .bypass_data            (s_axis_res_fm1_tdata),
    .up_out_valid           (up_out_valid),
    .up_out_data            (up_out_data),
    .m_axis_res_tready      (output_skid_s_ready),
    .m_axis_res_tvalid      (output_ctrl_tvalid),
    .m_axis_res_tdata      (output_ctrl_tdata),
    .out_space              (out_space),
    .out_valid              (out_valid),
    .normal_out_done        (normal_out_done)
);


// The residual algorithm/output register terminates in a two-entry registered
// skid.  The store_top asynchronous FIFO remains the only deep buffering and
// CDC boundary after this module.
axis_skid_buffer #(
    .DATA_WIDTH (512)
) u_res_output_skid (
    .clk           (clk),
    .rst_n         (rst_n),
    .s_axis_tvalid (output_ctrl_tvalid),
    .s_axis_tready (output_skid_s_ready),
    .s_axis_tdata  (output_ctrl_tdata),
    .m_axis_tvalid (m_axis_res_tvalid),
    .m_axis_tready (m_axis_res_tready),
    .m_axis_tdata  (m_axis_res_tdata)
);
endmodule

// ============================================================
// RTL Self-Check Results
// ============================================================
// [PASS] Rule #1:  All ports have explicit width declarations
// [PASS] Rule #2:  Ports use Verilog wire/reg conventions
// [PASS] Rule #6:  No debug/error-attribution logic or diagnostic-only sideband in RTL
// [PASS] Rule #8:  No logic/always_ff/always_comb used
// [PASS] Rule #34: Module name/parameters/port list identical to legacy residual.v
// [PASS] Residual output uses a registered 2-entry skid; no hand-written deep FIFO/credit gate
// [PASS] All submodule instantiations use named ports
// [PASS] Legacy `ifdef SIM` $display blocks removed (not ported)
// ============================================================
