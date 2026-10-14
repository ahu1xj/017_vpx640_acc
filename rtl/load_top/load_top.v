`timescale 1ns / 1ps
`include "all_defines.vh"

// =============================================================================
// load_top
// Top wrapper for 160b load command FIFO, load_core_tagged, merged load-data
// async FIFO, cal_clk-domain load_data_dispatch and ps_buffer.
//
// 重构（ui_clk 时序收拢）：DDR 返回流不再在 ui_clk 域就地分发给 8 条异步
// FIFO，而是由 load_core_tagged 打成一条带 TUSER 目的地标签的流，经一条
// 异步 FIFO 跨到 cal_clk 域，由 load_data_dispatch 直连分发给各消费者
// （无中间 FIFO）。EXPORT 的 pso512 通路与 cal_cmd 通路不变。
// cal_clk 域逻辑复位用 cal_rst_n（cnn_acc_top 内同步释放），dma 域用 rst_n。
// =============================================================================
module load_top #(
    parameter LUT_BYTES        = 256,
    parameter QUAN_BLOCK_BYTES = 512
)(
    input  wire         clk,
    input  wire         rst_n,

    input  wire         s_axis_load_cmd_tvalid,
    input  wire [`RD_SM_CMD_WIDTH-1:0] s_axis_load_cmd_tdata,
    output wire         s_axis_load_cmd_tready,

    output wire         m_axis_dma_cmd_tvalid,
    output wire [`CMD2_CMD_WIDTH-1:0]  m_axis_dma_cmd_tdata,
    input  wire         m_axis_dma_cmd_tready,

    input  wire         s_axis_rdddr_tvalid,
    input  wire [511:0] s_axis_rdddr_tdata,
    output wire         s_axis_rdddr_tready,

    output wire         m_axis_pso_tvalid,
    output wire [511:0] m_axis_pso_tdata,
    output wire         m_axis_pso_tlast,
    input  wire         m_axis_pso_tready,

    input  wire         cal_clk,
    input  wire         cal_rst_n,

    output wire         m_axis_fm_tvalid,
    output wire [511:0] m_axis_fm_tdata,
    input  wire         m_axis_fm_tready,

    output wire         m_axis_cpad_tvalid,
    output wire [511:0] m_axis_cpad_tdata,
    input  wire         m_axis_cpad_tready,

    output wire         m_axis_we_tvalid,
    output wire [511:0] m_axis_we_tdata,
    input  wire         m_axis_we_tready,

    output wire         m_axis_bias_tvalid,
    output wire [511:0] m_axis_bias_tdata,
    input  wire         m_axis_bias_tready,

    output wire         m_axis_quan_param_tvalid,
    output wire [511:0] m_axis_quan_param_tdata,
    input  wire         m_axis_quan_param_tready,

    output wire         m_axis_quan_fm1_tvalid,
    output wire [511:0] m_axis_quan_fm1_tdata,
    input  wire         m_axis_quan_fm1_tready,

    output wire         m_axis_quan_fm2_tvalid,
    output wire [511:0] m_axis_quan_fm2_tdata,
    input  wire         m_axis_quan_fm2_tready,

    output wire         m_axis_act_tvalid,
    output wire [511:0] m_axis_act_tdata,
    input  wire         m_axis_act_tready,

    output wire         m_axis_cal_cmd_tvalid,
    output wire [111:0] m_axis_cal_cmd_tdata,
    input  wire         m_axis_cal_cmd_tready,

    output wire         load_ba_rden,
    output wire [10:0]  load_ba_rdaddr,
    input  wire [31:0]  load_ba_rddata,

    input  wire         store_fm_done
);

wire         load_cmd_fifo_tvalid;
wire [`RD_SM_CMD_WIDTH-1:0] load_cmd_fifo_tdata;
wire         load_cmd_fifo_tready;

wire         cal_cmd_core_tvalid;
wire [111:0] cal_cmd_core_tdata;
wire         cal_cmd_core_tready;

// load_core_tagged 标签化输出（ui_clk 域）
wire         ld_core_tvalid;
wire [511:0] ld_core_tdata;
wire [2:0]   ld_core_tuser;
wire         ld_core_tready;

// 合并 FIFO 读出（cal_clk 域）
wire         ld_fifo_tvalid;
wire [511:0] ld_fifo_tdata;
wire [2:0]   ld_fifo_tuser;
wire         ld_fifo_tready;

wire         pso512_tvalid;
wire [511:0] pso512_tdata;
wire         pso512_tlast;
wire         pso512_tready;

wire         pso_fifo_tvalid;
wire [511:0] pso_fifo_tdata;
wire         pso_fifo_tlast;
wire         pso_fifo_tready;

axis_data_fifo_160_sync u_load_cmd_fifo (
    .s_axis_aresetn(rst_n),
    .s_axis_aclk   (clk),
    .s_axis_tvalid (s_axis_load_cmd_tvalid),
    .s_axis_tdata  (s_axis_load_cmd_tdata),
    .s_axis_tready (s_axis_load_cmd_tready),
    .m_axis_tvalid (load_cmd_fifo_tvalid),
    .m_axis_tdata  (load_cmd_fifo_tdata),
    .m_axis_tready (load_cmd_fifo_tready)
);

load_core_tagged #(
    .LUT_BYTES        (LUT_BYTES),
    .QUAN_BLOCK_BYTES (QUAN_BLOCK_BYTES)
) u_load_core (
    .clk                      (clk),
    .rst_n                    (rst_n),

    .s_axis_load_cmd_tvalid   (load_cmd_fifo_tvalid),
    .s_axis_load_cmd_tdata    (load_cmd_fifo_tdata),
    .s_axis_load_cmd_tready   (load_cmd_fifo_tready),

    .m_axis_dma_cmd_tvalid    (m_axis_dma_cmd_tvalid),
    .m_axis_dma_cmd_tdata     (m_axis_dma_cmd_tdata),
    .m_axis_dma_cmd_tready    (m_axis_dma_cmd_tready),

    .m_axis_cal_cmd_tvalid    (cal_cmd_core_tvalid),
    .m_axis_cal_cmd_tdata     (cal_cmd_core_tdata),
    .m_axis_cal_cmd_tready    (cal_cmd_core_tready),

    .load_ba_rden             (load_ba_rden),
    .load_ba_rdaddr           (load_ba_rdaddr),
    .load_ba_rddata           (load_ba_rddata),

    .store_fm_done            (store_fm_done),

    .s_axis_ddr_tvalid        (s_axis_rdddr_tvalid),
    .s_axis_ddr_tdata         (s_axis_rdddr_tdata),
    .s_axis_ddr_tready        (s_axis_rdddr_tready),

    .m_axis_load_data_tvalid  (ld_core_tvalid),
    .m_axis_load_data_tdata   (ld_core_tdata),
    .m_axis_load_data_tuser   (ld_core_tuser),
    .m_axis_load_data_tready  (ld_core_tready),

    .m_axis_pso512_tvalid     (pso512_tvalid),
    .m_axis_pso512_tdata      (pso512_tdata),
    .m_axis_pso512_tlast      (pso512_tlast),
    .m_axis_pso512_tready     (pso512_tready)
);

axis_data_fifo_112_async u_cal_cmd_fifo (
    .s_axis_aclk   (clk),
    .s_axis_aresetn(rst_n),
    .s_axis_tvalid (cal_cmd_core_tvalid),
    .s_axis_tdata  (cal_cmd_core_tdata),
    .s_axis_tready (cal_cmd_core_tready),
    .m_axis_aclk   (cal_clk),
    .m_axis_tvalid (m_axis_cal_cmd_tvalid),
    .m_axis_tdata  (m_axis_cal_cmd_tdata),
    .m_axis_tready (m_axis_cal_cmd_tready)
);

// Tagged async FIFO: ui_clk -> cal_clk, depth 512, standard valid/ready.
load_data_fifo_async u_load_data_fifo (
    .s_axis_aclk   (clk),
    .s_axis_aresetn(rst_n),
    .s_axis_tvalid (ld_core_tvalid),
    .s_axis_tdata  (ld_core_tdata),
    .s_axis_tuser  (ld_core_tuser),
    .s_axis_tready (ld_core_tready),
    .m_axis_aclk   (cal_clk),
    .m_axis_tvalid (ld_fifo_tvalid),
    .m_axis_tdata  (ld_fifo_tdata),
    .m_axis_tuser  (ld_fifo_tuser),
    .m_axis_tready (ld_fifo_tready)
);

// cal_clk 域分流：直连各消费者，无中间 FIFO
load_data_dispatch u_load_data_dispatch (
    .clk                    (cal_clk),
    .rst_n                  (cal_rst_n),

    .s_axis_load_data_tvalid(ld_fifo_tvalid),
    .s_axis_load_data_tdata (ld_fifo_tdata),
    .s_axis_load_data_tuser (ld_fifo_tuser),
    .s_axis_load_data_tready(ld_fifo_tready),

    .m_axis_fm_tvalid       (m_axis_fm_tvalid),
    .m_axis_fm_tdata        (m_axis_fm_tdata),
    .m_axis_fm_tready       (m_axis_fm_tready),

    .m_axis_cpad_tvalid     (m_axis_cpad_tvalid),
    .m_axis_cpad_tdata      (m_axis_cpad_tdata),
    .m_axis_cpad_tready     (m_axis_cpad_tready),

    .m_axis_we_tvalid       (m_axis_we_tvalid),
    .m_axis_we_tdata        (m_axis_we_tdata),
    .m_axis_we_tready       (m_axis_we_tready),

    .m_axis_bias_tvalid     (m_axis_bias_tvalid),
    .m_axis_bias_tdata      (m_axis_bias_tdata),
    .m_axis_bias_tready     (m_axis_bias_tready),

    .m_axis_quan_param_tvalid(m_axis_quan_param_tvalid),
    .m_axis_quan_param_tdata (m_axis_quan_param_tdata),
    .m_axis_quan_param_tready(m_axis_quan_param_tready),

    .m_axis_quan_fm1_tvalid (m_axis_quan_fm1_tvalid),
    .m_axis_quan_fm1_tdata  (m_axis_quan_fm1_tdata),
    .m_axis_quan_fm1_tready (m_axis_quan_fm1_tready),

    .m_axis_quan_fm2_tvalid (m_axis_quan_fm2_tvalid),
    .m_axis_quan_fm2_tdata  (m_axis_quan_fm2_tdata),
    .m_axis_quan_fm2_tready (m_axis_quan_fm2_tready),

    .m_axis_act_tvalid      (m_axis_act_tvalid),
    .m_axis_act_tdata       (m_axis_act_tdata),
    .m_axis_act_tready      (m_axis_act_tready)
);

axis_data_fifo_512_tlast_sync u_pso_fifo (
    .s_axis_aclk   (clk),
    .s_axis_aresetn(rst_n),
    .s_axis_tvalid (pso512_tvalid),
    .s_axis_tdata  (pso512_tdata),
    .s_axis_tlast  (pso512_tlast),
    .s_axis_tready (pso512_tready),
    .m_axis_tvalid (pso_fifo_tvalid),
    .m_axis_tdata  (pso_fifo_tdata),
    .m_axis_tlast  (pso_fifo_tlast),
    .m_axis_tready (pso_fifo_tready)
);

// 006：删除 ps_buffer（512 拆 32），export 原始 512bit 直出，
// 由 cnn_acc_15e_top 内的 512->64 位宽拆分器接 PS DMA。
assign m_axis_pso_tvalid = pso_fifo_tvalid;
assign m_axis_pso_tdata  = pso_fifo_tdata;
assign m_axis_pso_tlast  = pso_fifo_tlast;
assign pso_fifo_tready   = m_axis_pso_tready;

endmodule
