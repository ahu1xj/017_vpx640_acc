`timescale 1ns / 1ps
`include "all_defines.vh"

// =============================================================================
// axi_ddr_top —— DMA 复写版顶层（端口合同与原版 dma_axi4_master 一致）
//
// 数据通路（写侧已完成）:
//
//   s_axis_wrcmd2(64b,usr_clk) ─► [axis_fifo_64x16]  ─► decoder_wrcmd ─► write_driver ─► m_axi_aw/w/b
//                                    CDC dma→ui          (命令→突发)
//   s_axis_wrddr (512b,usr_clk)─► [axis_fifo_512x256] ─► dma_app_unpack ─► write_driver ─► m_axi_w
//                                    CDC dma→ui          512→N×AXI      └─ beat_fifo 在 write_driver 内
//                                                       (512 下直通)        (ui_clk 同步，credit 落脚点)
//
// 数据通路（读侧 TODO）:
//
//   s_axis_rdcmd2 ─► [axis_fifo_64x16] ─► decoder_rdcmd ─► read_driver ─► m_axi_ar
//   m_axis_rdddr  ◄─ [axis_fifo_512x256_np] ◄────────────── read_driver ◄─ m_axi_r
//
// 时序约定:
//   usr_clk (dma_clk 100M): 四条 s_axis/m_axis 命令与数据口
//   clk     (ui_clk  200M): 命令 FIFO 出侧、decoder、driver、m_axi_*
//
// 纯 Verilog-2001。
// =============================================================================
module axi_ddr_top #(
    parameter AXI_ADDR_WIDTH = 32,
    parameter AXI_DATA_WIDTH = `DMA_AXI_SELECTED_DATA_WIDTH,
    parameter AXI_ID_WIDTH   = 4,
    parameter AXI_STRB_WIDTH = `DMA_AXI_SELECTED_STRB_WIDTH
) (
    input  wire                       clk,          // ui_clk  200MHz
    input  wire                       usr_clk,      // dma_clk 100MHz
    input  wire                       rst_n,

    // ---------------- 写命令（usr_clk 域）----------------
    input  wire                       s_axis_wrcmd2_tvalid,
    output wire                       s_axis_wrcmd2_tready,
    input  wire [63:0]                s_axis_wrcmd2_tdata,

    // ---------------- 写数据（usr_clk 域）----------------
    input  wire                       s_axis_wrddr_tvalid,
    output wire                       s_axis_wrddr_tready,
    input  wire [511:0]               s_axis_wrddr_tdata,

    // ---------------- 读命令（usr_clk 域）----------------
    input  wire                       s_axis_rdcmd2_tvalid,
    output wire                       s_axis_rdcmd2_tready,
    input  wire [63:0]                s_axis_rdcmd2_tdata,

    // ---------------- 读数据（usr_clk 域）----------------
    output wire                       m_axis_rdddr_tvalid,
    input  wire                       m_axis_rdddr_tready,
    output wire [511:0]               m_axis_rdddr_tdata,

    // ---------------- AXI4 主口（clk 域）----------------
    output wire [AXI_ID_WIDTH-1:0]    m_axi_awid,
    output wire [AXI_ADDR_WIDTH-1:0]  m_axi_awaddr,
    output wire [7:0]                 m_axi_awlen,
    output wire [2:0]                 m_axi_awsize,
    output wire [1:0]                 m_axi_awburst,
    output wire                       m_axi_awlock,
    output wire [3:0]                 m_axi_awcache,
    output wire [2:0]                 m_axi_awprot,
    output wire [3:0]                 m_axi_awqos,
    output wire                       m_axi_awvalid,
    input  wire                       m_axi_awready,

    output wire [AXI_DATA_WIDTH-1:0]  m_axi_wdata,
    output wire [AXI_STRB_WIDTH-1:0]  m_axi_wstrb,
    output wire                       m_axi_wlast,
    output wire                       m_axi_wvalid,
    input  wire                       m_axi_wready,

    input  wire [AXI_ID_WIDTH-1:0]    m_axi_bid,
    input  wire [1:0]                 m_axi_bresp,
    input  wire                       m_axi_bvalid,
    output wire                       m_axi_bready,

    output wire [AXI_ID_WIDTH-1:0]    m_axi_arid,
    output wire [AXI_ADDR_WIDTH-1:0]  m_axi_araddr,
    output wire [7:0]                 m_axi_arlen,
    output wire [2:0]                 m_axi_arsize,
    output wire [1:0]                 m_axi_arburst,
    output wire                       m_axi_arlock,
    output wire [3:0]                 m_axi_arcache,
    output wire [2:0]                 m_axi_arprot,
    output wire [3:0]                 m_axi_arqos,
    output wire                       m_axi_arvalid,
    input  wire                       m_axi_arready,

    input  wire [AXI_ID_WIDTH-1:0]    m_axi_rid,
    input  wire [AXI_DATA_WIDTH-1:0]  m_axi_rdata,
    input  wire [1:0]                 m_axi_rresp,
    input  wire                       m_axi_rlast,
    input  wire                       m_axi_rvalid,
    output wire                       m_axi_rready
);

//------------------------------------------------------------------
// 命令 FIFO 出侧（clk 域）
//------------------------------------------------------------------
wire        wrcmd2_fifo_m_valid;
wire        wrcmd2_fifo_m_ready;
wire [63:0] wrcmd2_fifo_m_data;
wire [31:0] wrcmd2_fifo_wr_count;
wire [31:0] wrcmd2_fifo_rd_count;

//------------------------------------------------------------------
// 数据 FIFO 出侧（clk 域）
//------------------------------------------------------------------
wire         wrddr_fifo_m_valid;
wire         wrddr_fifo_m_ready;
wire [511:0] wrddr_fifo_m_data;
wire [31:0]  wrddr_fifo_wr_count;
wire [31:0]  wrddr_fifo_rd_count;

//------------------------------------------------------------------
// unpack 出侧（AXI 宽度的拍流，clk 域）
//------------------------------------------------------------------
wire [AXI_DATA_WIDTH-1:0] upk_beat_tdata;
wire                      upk_beat_tvalid;
wire                      upk_beat_tready;

//------------------------------------------------------------------
// decoder_wrcmd ↔ write_driver（突发接口，clk 域）
//------------------------------------------------------------------
wire [AXI_ADDR_WIDTH-1:0] dec_burst_addr;
wire [7:0]                dec_burst_len;
wire                      dec_cmd_valid;
wire                      dec_cmd_tready;
wire [3:0]                dec_cmd_err;

//------------------------------------------------------------------
// write_driver 侧带信号
//------------------------------------------------------------------
wire wr_busy;
wire wr_resp_err;
wire wr_starve_err;

//------------------------------------------------------------------
// 写命令 FIFO：64b，usr_clk → clk（异步）
//------------------------------------------------------------------
axis_fifo_64x16 u_wrcmd2_fifo (
    .s_axis_aresetn     (rst_n),
    .s_axis_aclk        (usr_clk),
    .m_axis_aclk        (clk),
    .s_axis_tvalid      (s_axis_wrcmd2_tvalid),
    .s_axis_tready      (s_axis_wrcmd2_tready),
    .s_axis_tdata       (s_axis_wrcmd2_tdata),
    .m_axis_tvalid      (wrcmd2_fifo_m_valid),
    .m_axis_tready      (wrcmd2_fifo_m_ready),
    .m_axis_tdata       (wrcmd2_fifo_m_data),
    .axis_wr_data_count (wrcmd2_fifo_wr_count),
    .axis_rd_data_count (wrcmd2_fifo_rd_count)
);

//------------------------------------------------------------------
// 写数据 FIFO：512b，usr_clk → clk（异步）
//   出侧是 app 拍（恒 512b），经 unpack 转成 AXI 宽度的拍流
//------------------------------------------------------------------
axis_fifo_512x256 u_wrddr_fifo (
    .s_axis_aresetn     (rst_n),
    .s_axis_aclk        (usr_clk),
    .m_axis_aclk        (clk),
    .s_axis_tvalid      (s_axis_wrddr_tvalid),
    .s_axis_tready      (s_axis_wrddr_tready),
    .s_axis_tdata       (s_axis_wrddr_tdata),
    .m_axis_tvalid      (wrddr_fifo_m_valid),
    .m_axis_tready      (wrddr_fifo_m_ready),
    .m_axis_tdata       (wrddr_fifo_m_data),
    .axis_wr_data_count (wrddr_fifo_wr_count),
    .axis_rd_data_count (wrddr_fifo_rd_count)
);

//------------------------------------------------------------------
// app 拍（512b）→ AXI 拍（N 拍）
//   ★ 放在 wrddr_fifo 与 write_driver 之间：
//     beat_fifo 因此恒为 AXI 宽 → 「1 字 = 1 拍」→ credit 记账零改动
//   ★ 512 位宽下 BEATS_PER_APP=1，本模块纯直通
//------------------------------------------------------------------
dma_app_unpack #(
    .AXI_DATA_WIDTH (AXI_DATA_WIDTH),
    .APP_DATA_WIDTH (`DMA_APP_DATA_WIDTH)
) u_dma_app_unpack (
    .clk           (clk),
    .rst_n         (rst_n),
    .s_app_tdata   (wrddr_fifo_m_data),
    .s_app_tvalid  (wrddr_fifo_m_valid),
    .s_app_tready  (wrddr_fifo_m_ready),
    .m_beat_tdata  (upk_beat_tdata),
    .m_beat_tvalid (upk_beat_tvalid),
    .m_beat_tready (upk_beat_tready)
);

//------------------------------------------------------------------
// 命令 → 突发序列
//------------------------------------------------------------------
decoder_wrcmd #(
    .AXI_DATA_WIDTH  (AXI_DATA_WIDTH),
    .MAX_BURST_BEATS (`DMA_AXI_MAX_BURST_BEATS),
    .MEM_BYTES       (32'h4000_0000)
) u_decoder_wrcmd (
    .clk          (clk),
    .rst_n        (rst_n),
    .cmd_data     (wrcmd2_fifo_m_data),
    .cmd_valid    (wrcmd2_fifo_m_valid),
    .cmd_tready   (dec_cmd_tready),
    .s_axis_tready(wrcmd2_fifo_m_ready),
    .cmd_address  (dec_burst_addr),
    .burst_len    (dec_burst_len),
    .cmd_tvalid   (dec_cmd_valid),
    .cmd_err      (dec_cmd_err)
);

//------------------------------------------------------------------
// 突发 + 数据 → AXI 写通道
//------------------------------------------------------------------
write_driver #(
    .AXI_DATA_WIDTH     (AXI_DATA_WIDTH),
    .AXI_ADDR_WIDTH     (AXI_ADDR_WIDTH),
    .AXI_ID_WIDTH       (AXI_ID_WIDTH),
    .AXI_STRB_WIDTH     (AXI_STRB_WIDTH),
    .WR_BEAT_FIFO_DEPTH (512)
) u_write_driver (
    .clk           (clk),
    .rst_n         (rst_n),

    .burst_addr    (dec_burst_addr),
    .burst_len     (dec_burst_len),
    .cmd_valid     (dec_cmd_valid),
    .cmd_tready    (dec_cmd_tready),

    .s_beat_tdata  (upk_beat_tdata),
    .s_beat_tvalid (upk_beat_tvalid),
    .s_beat_tready (upk_beat_tready),

    .m_axi_awid    (m_axi_awid),
    .m_axi_awaddr  (m_axi_awaddr),
    .m_axi_awlen   (m_axi_awlen),
    .m_axi_awsize  (m_axi_awsize),
    .m_axi_awburst (m_axi_awburst),
    .m_axi_awlock  (m_axi_awlock),
    .m_axi_awcache (m_axi_awcache),
    .m_axi_awprot  (m_axi_awprot),
    .m_axi_awqos   (m_axi_awqos),
    .m_axi_awvalid (m_axi_awvalid),
    .m_axi_awready (m_axi_awready),

    .m_axi_wdata   (m_axi_wdata),
    .m_axi_wstrb   (m_axi_wstrb),
    .m_axi_wlast   (m_axi_wlast),
    .m_axi_wvalid  (m_axi_wvalid),
    .m_axi_wready  (m_axi_wready),

    .m_axi_bid     (m_axi_bid),
    .m_axi_bresp   (m_axi_bresp),
    .m_axi_bvalid  (m_axi_bvalid),
    .m_axi_bready  (m_axi_bready),

    .write_busy    (wr_busy),
    .wr_resp_err   (wr_resp_err),
    .wr_starve_err (wr_starve_err)
);

//------------------------------------------------------------------
// 读通路
//
//   s_axis_rdcmd2(64b,usr_clk) ─► [axis_fifo_64x16] ─► decoder_wrcmd ─► read_driver ─► m_axi_ar/r
//                                    CDC usr→clk       (命令→突发)          │
//                                                                     dma_app_pack
//                                                                      N×AXI → 512
//                                                                          │
//   m_axis_rdddr(512b,usr_clk) ◄─ [axis_fifo_512x256_np] ◄──────────────────┘
//                                    CDC clk→usr
//
//   ★ read_driver 吃 wr_busy 做防死锁门；rdddr_fifo 的写侧计数做空间判据
//------------------------------------------------------------------
wire        rdcmd2_fifo_m_valid;
wire        rdcmd2_fifo_m_ready;
wire [63:0] rdcmd2_fifo_m_data;
wire [31:0] rdcmd2_fifo_wr_count;
wire [31:0] rdcmd2_fifo_rd_count;

wire [AXI_ADDR_WIDTH-1:0] rdec_burst_addr;
wire [7:0]                rdec_burst_len;
wire                      rdec_cmd_valid;
wire                      rdec_cmd_tready;
wire [3:0]                rdec_cmd_err;

wire [AXI_DATA_WIDTH-1:0] rdr_beat_tdata;
wire                      rdr_beat_tvalid;
wire                      rdr_beat_tready;

wire [511:0] rdddr_fifo_s_tdata;
wire         rdddr_fifo_s_tvalid;
wire         rdddr_fifo_s_tready;
wire [31:0]  rdddr_fifo_wr_count;
wire [31:0]  rdddr_fifo_rd_count;

wire rd_resp_err;

//------------------------------------------------------------------
// 读命令 FIFO：64b，usr_clk → clk（异步）
//------------------------------------------------------------------
axis_fifo_64x16 u_rdcmd2_fifo (
    .s_axis_aresetn     (rst_n),
    .s_axis_aclk        (usr_clk),
    .m_axis_aclk        (clk),
    .s_axis_tvalid      (s_axis_rdcmd2_tvalid),
    .s_axis_tready      (s_axis_rdcmd2_tready),
    .s_axis_tdata       (s_axis_rdcmd2_tdata),
    .m_axis_tvalid      (rdcmd2_fifo_m_valid),
    .m_axis_tready      (rdcmd2_fifo_m_ready),
    .m_axis_tdata       (rdcmd2_fifo_m_data),
    .axis_wr_data_count (rdcmd2_fifo_wr_count),
    .axis_rd_data_count (rdcmd2_fifo_rd_count)
);

//------------------------------------------------------------------
// 命令 → 突发序列（★ 与写侧同一份代码，例化第二次）
//------------------------------------------------------------------
decoder_wrcmd #(
    .AXI_DATA_WIDTH  (AXI_DATA_WIDTH),
    .MAX_BURST_BEATS (`DMA_AXI_MAX_BURST_BEATS),
    .MEM_BYTES       (32'h4000_0000)
) u_decoder_rdcmd (
    .clk          (clk),
    .rst_n        (rst_n),
    .cmd_data     (rdcmd2_fifo_m_data),
    .cmd_valid    (rdcmd2_fifo_m_valid),
    .cmd_tready   (rdec_cmd_tready),
    .s_axis_tready(rdcmd2_fifo_m_ready),
    .cmd_address  (rdec_burst_addr),
    .burst_len    (rdec_burst_len),
    .cmd_tvalid   (rdec_cmd_valid),
    .cmd_err      (rdec_cmd_err)
);

//------------------------------------------------------------------
// 突发 → AXI 读通道 → 数据拍流
//------------------------------------------------------------------
read_driver #(
    .AXI_DATA_WIDTH   (AXI_DATA_WIDTH),
    .AXI_ADDR_WIDTH   (AXI_ADDR_WIDTH),
    .AXI_ID_WIDTH     (AXI_ID_WIDTH),
    .RDDDR_FIFO_DEPTH (256)
) u_read_driver (
    .clk           (clk),
    .rst_n         (rst_n),

    .burst_addr    (rdec_burst_addr),
    .burst_len     (rdec_burst_len),
    .cmd_valid     (rdec_cmd_valid),
    .cmd_tready    (rdec_cmd_tready),

    .m_beat_tdata  (rdr_beat_tdata),
    .m_beat_tvalid (rdr_beat_tvalid),
    .m_beat_tready (rdr_beat_tready),

    .rdddr_count   (rdddr_fifo_wr_count[10:0]),   // ★ 写侧计数 = clk 域占用
    .write_busy    (wr_busy),                     // ★ 防死锁门

    .m_axi_arid    (m_axi_arid),
    .m_axi_araddr  (m_axi_araddr),
    .m_axi_arlen   (m_axi_arlen),
    .m_axi_arsize  (m_axi_arsize),
    .m_axi_arburst (m_axi_arburst),
    .m_axi_arlock  (m_axi_arlock),
    .m_axi_arcache (m_axi_arcache),
    .m_axi_arprot  (m_axi_arprot),
    .m_axi_arqos   (m_axi_arqos),
    .m_axi_arvalid (m_axi_arvalid),
    .m_axi_arready (m_axi_arready),

    .m_axi_rid     (m_axi_rid),
    .m_axi_rdata   (m_axi_rdata),
    .m_axi_rresp   (m_axi_rresp),
    .m_axi_rlast   (m_axi_rlast),
    .m_axi_rvalid  (m_axi_rvalid),
    .m_axi_rready  (m_axi_rready),

    .rd_resp_err   (rd_resp_err)
);

//------------------------------------------------------------------
// AXI 拍（N 拍）→ app 拍（512b）
//   ★ 放在 read_driver 与 rdddr_fifo 之间（与写侧 unpack 镜像对称）
//   ★ 512 位宽下 BEATS_PER_APP=1，本模块纯直通
//------------------------------------------------------------------
dma_app_pack #(
    .AXI_DATA_WIDTH (AXI_DATA_WIDTH),
    .APP_DATA_WIDTH (`DMA_APP_DATA_WIDTH)
) u_dma_app_pack (
    .clk          (clk),
    .rst_n        (rst_n),
    .s_beat_tdata (rdr_beat_tdata),
    .s_beat_tvalid(rdr_beat_tvalid),
    .s_beat_tready(rdr_beat_tready),
    .m_app_tdata  (rdddr_fifo_s_tdata),
    .m_app_tvalid (rdddr_fifo_s_tvalid),
    .m_app_tready (rdddr_fifo_s_tready)
);

//------------------------------------------------------------------
// 读数据 FIFO：512b，clk → usr_clk（异步）
//------------------------------------------------------------------
axis_fifo_512x256_np u_rdddr_fifo (
    .s_axis_aresetn     (rst_n),
    .s_axis_aclk        (clk),
    .m_axis_aclk        (usr_clk),
    .s_axis_tvalid      (rdddr_fifo_s_tvalid),
    .s_axis_tready      (rdddr_fifo_s_tready),
    .s_axis_tdata       (rdddr_fifo_s_tdata),
    .m_axis_tvalid      (m_axis_rdddr_tvalid),
    .m_axis_tready      (m_axis_rdddr_tready),
    .m_axis_tdata       (m_axis_rdddr_tdata),
    .axis_wr_data_count (rdddr_fifo_wr_count),
    .axis_rd_data_count (rdddr_fifo_rd_count)
);

//------------------------------------------------------------------
// 未使用的 FIFO 计数 / decoder 错误 / driver 诊断（挂 ILA 用）
//------------------------------------------------------------------
(* mark_debug = "true" *) wire [31:0] dbg_wrcmd2_wr_count = wrcmd2_fifo_wr_count;
(* mark_debug = "true" *) wire [31:0] dbg_wrddr_wr_count  = wrddr_fifo_wr_count;
(* mark_debug = "true" *) wire [3:0]  dbg_dec_wr_cmd_err  = dec_cmd_err;
(* mark_debug = "true" *) wire        dbg_wr_busy         = wr_busy;
(* mark_debug = "true" *) wire        dbg_wr_resp_err     = wr_resp_err;
(* mark_debug = "true" *) wire        dbg_wr_starve_err   = wr_starve_err;

(* mark_debug = "true" *) wire [31:0] dbg_rdcmd2_wr_count = rdcmd2_fifo_wr_count;
(* mark_debug = "true" *) wire [31:0] dbg_rdddr_wr_count  = rdddr_fifo_wr_count;
(* mark_debug = "true" *) wire [3:0]  dbg_dec_rd_cmd_err  = rdec_cmd_err;
(* mark_debug = "true" *) wire        dbg_rd_resp_err     = rd_resp_err;

endmodule
