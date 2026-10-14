// VPX640 accelerator integration from hash-locked 013 application RTL.
// MIG GROUP0: AXI512/32/4, sys200 differential, UI200/CAL160/DMA100 MHz.
// SRIO Bank A: x4 5Gbps, ref125/log250 MHz, 8-bit local ID0x12, peer0xAA.
// No separate business clock wizard; no board reset pin is introduced.
// P3 startup and protocol validation are required; this source is not signoff.
`timescale 1ns/1ps

module vpx640_acc_top
#(
    // 仿真专用: 1 时 SRIO 核进入 sim_train 快速链路训练模式;
    // 上板/综合必须为 0(真训练)
    parameter SIM_TRAIN = 0
)
(
    // SRIO 参考钟与串行收发(VPX640 Bank A, 125 MHz reference)
    input  wire        srio_clkp,
    input  wire        srio_clkn,
    input  wire        srio_rxp0,
    input  wire        srio_rxn0,
    input  wire        srio_rxp1,
    input  wire        srio_rxn1,
    input  wire        srio_rxp2,
    input  wire        srio_rxn2,
    input  wire        srio_rxp3,
    input  wire        srio_rxn3,
    output wire        srio_txp0,
    output wire        srio_txn0,
    output wire        srio_txp1,
    output wire        srio_txn1,
    output wire        srio_txp2,
    output wire        srio_txn2,
    output wire        srio_txp3,
    output wire        srio_txn3,

    // Board 200 MHz differential clock directly into the MIG input buffer
    input  wire        sys_clk_p,
    input  wire        sys_clk_n,

    // DDR3 物理接口
    inout  wire [63:0] ddr3_dq,
    inout  wire [7:0]  ddr3_dqs_p,
    inout  wire [7:0]  ddr3_dqs_n,
    output wire [15:0] ddr3_addr,
    output wire [2:0]  ddr3_ba,
    output wire        ddr3_ras_n,
    output wire        ddr3_cas_n,
    output wire        ddr3_we_n,
    output wire        ddr3_reset_n,
    output wire        ddr3_ck_p,
    output wire        ddr3_ck_n,
    output wire        ddr3_cke,
    output wire        ddr3_cs_n,
    output wire [7:0]  ddr3_dm,
    output wire        ddr3_odt
);

// MIG is the sole source of UI/CAL/DMA clocks; no external input buffer.
wire        cal_clk; // 160 MHz from ui_addn_clk_0
wire        dma_clk; // 100 MHz from ui_addn_clk_1

wire        ui_clk;
wire        ui_clk_sync_rst;
wire        mmcm_locked;
wire        init_calib_complete;
wire        peripheral_aresetn;

// ui 域复位: 直接用 MIG 自产的 ui_clk_sync_rst(官方例程同款形态)。该信号
// 已含 sys_rst 输入与 MMCM 锁状态, 且是 ui_clk 域已同步信号, 无需再过
// 同步器。校准闸门不进 ui 域复位: 核内 dma/cal 域由 proc_sys_reset 的
// aux_reset_in 闸住(见 cnn_acc_top), 校准完成前核不出复位、不会提前访存。
reg peripheral_aresetn_r;
always @(posedge ui_clk) begin
    peripheral_aresetn_r <= ~ui_clk_sync_rst;
end
assign peripheral_aresetn = peripheral_aresetn_r;

// dma 域复位: 由 cnn_acc_top 内 rst_dma_clk(ext=ui 域复位, aux=校准完成)
// 产生并导出, 传输通路(rx_path/tx_ctrl 的 dma 侧)与核共用同一复位——
// 同源、同闸门、同相位(20260813 复位链重构)。
(* mark_debug = "true" *) wire        dma_rst_n;

// AXI: cnn_acc(512) 直连 MIG(512), 全部 ui_clk 域(20260814 去 axi_dw 位宽转换,
// 改走 DMA_AXI_DATA_WIDTH_512 宏的直连方案, 与 060 线全链路 512 一致)
wire [3:0]   axi_awid;
wire [31:0]  axi_awaddr;
wire [7:0]   axi_awlen;
wire [2:0]   axi_awsize;
wire [1:0]   axi_awburst;
wire [0:0]   axi_awlock;
wire [3:0]   axi_awcache;
wire [2:0]   axi_awprot;
wire [3:0]   axi_awqos;
(* mark_debug = "true" *) wire         axi_awvalid;   // ILA v5
(* mark_debug = "true" *) wire         axi_awready;   // ILA v5
wire [511:0] axi_wdata;
wire [63:0]  axi_wstrb;
(* mark_debug = "true" *) wire         axi_wlast;   // ILA v5
(* mark_debug = "true" *) wire         axi_wvalid;   // ILA v5
(* mark_debug = "true" *) wire         axi_wready;   // ILA v5
wire [3:0]   axi_bid;
wire [1:0]   axi_bresp;
(* mark_debug = "true" *) wire         axi_bvalid;   // ILA v5
(* mark_debug = "true" *) wire         axi_bready;   // ILA v5
wire [3:0]   axi_arid;
(* mark_debug = "true" *) wire [31:0]  axi_araddr;   // ILA v5
wire [7:0]   axi_arlen;
wire [2:0]   axi_arsize;
wire [1:0]   axi_arburst;
wire [0:0]   axi_arlock;
wire [3:0]   axi_arcache;
wire [2:0]   axi_arprot;
wire [3:0]   axi_arqos;
(* mark_debug = "true" *) wire         axi_arvalid;   // ILA v5
(* mark_debug = "true" *) wire         axi_arready;   // ILA v5
wire [3:0]   axi_rid;
wire [511:0] axi_rdata;
wire [1:0]   axi_rresp;
(* mark_debug = "true" *) wire         axi_rlast;   // ILA v5
(* mark_debug = "true" *) wire         axi_rvalid;   // ILA v5
(* mark_debug = "true" *) wire         axi_rready;   // ILA v5

// TB 探针兼容别名: 直连后 256/512 为同一总线, 保留 mig_* 名供 AXIMON 层级引用
wire [31:0]  mig_awaddr  = axi_awaddr;
wire [7:0]   mig_awlen   = axi_awlen;
wire [2:0]   mig_awsize  = axi_awsize;
wire         mig_awvalid = axi_awvalid;
wire         mig_awready = axi_awready;
wire         mig_wlast   = axi_wlast;
wire         mig_wvalid  = axi_wvalid;
wire         mig_wready  = axi_wready;
wire         mig_bvalid  = axi_bvalid;
wire         mig_bready  = axi_bready;
wire [31:0]  mig_araddr  = axi_araddr;
wire [7:0]   mig_arlen   = axi_arlen;
wire [2:0]   mig_arsize  = axi_arsize;
wire         mig_arvalid = axi_arvalid;
wire         mig_arready = axi_arready;
wire         mig_rlast   = axi_rlast;
wire         mig_rvalid  = axi_rvalid;
wire         mig_rready  = axi_rready;

mig_7series_0 u_mig (
    // DDR3 物理
    .ddr3_dq              (ddr3_dq),
    .ddr3_dqs_n           (ddr3_dqs_n),
    .ddr3_dqs_p           (ddr3_dqs_p),
    .ddr3_addr            (ddr3_addr),
    .ddr3_ba              (ddr3_ba),
    .ddr3_ras_n           (ddr3_ras_n),
    .ddr3_cas_n           (ddr3_cas_n),
    .ddr3_we_n            (ddr3_we_n),
    .ddr3_reset_n         (ddr3_reset_n),
    .ddr3_ck_p            (ddr3_ck_p),
    .ddr3_ck_n            (ddr3_ck_n),
    .ddr3_cke             (ddr3_cke),
    .ddr3_cs_n            (ddr3_cs_n),
    .ddr3_dm              (ddr3_dm),
    .ddr3_odt             (ddr3_odt),
    // 时钟与复位
    .sys_clk_p            (sys_clk_p),
    .sys_clk_n            (sys_clk_n),
    .sys_rst              (1'b1), // Active-low; board power-up/FPGA GSR startup
    .ui_clk               (ui_clk),
    .ui_addn_clk_0        (cal_clk),
    .ui_addn_clk_1        (dma_clk),
    .ui_addn_clk_2        (),
    .ui_addn_clk_3        (),
    .ui_addn_clk_4        (),
    .ui_clk_sync_rst      (ui_clk_sync_rst),
    .mmcm_locked          (mmcm_locked),
    .aresetn              (peripheral_aresetn),
    .init_calib_complete  (init_calib_complete),
    // 自刷新/ZQ 不用
    .app_sr_req           (1'b0),
    .app_ref_req          (1'b0),
    .app_zq_req           (1'b0),
    .app_sr_active        (),
    .app_ref_ack          (),
    .app_zq_ack           (),
    // AXI 从口(512 直连 cnn_acc 主口)
    .s_axi_awid           (axi_awid),
    .s_axi_awaddr         (axi_awaddr),
    .s_axi_awlen          (axi_awlen),
    .s_axi_awsize         (axi_awsize),
    .s_axi_awburst        (axi_awburst),
    .s_axi_awlock         (axi_awlock),
    .s_axi_awcache        (axi_awcache),
    .s_axi_awprot         (axi_awprot),
    .s_axi_awqos          (axi_awqos),
    .s_axi_awvalid        (axi_awvalid),
    .s_axi_awready        (axi_awready),
    .s_axi_wdata          (axi_wdata),
    .s_axi_wstrb          (axi_wstrb),
    .s_axi_wlast          (axi_wlast),
    .s_axi_wvalid         (axi_wvalid),
    .s_axi_wready         (axi_wready),
    .s_axi_bid            (axi_bid),
    .s_axi_bresp          (axi_bresp),
    .s_axi_bvalid         (axi_bvalid),
    .s_axi_bready         (axi_bready),
    .s_axi_arid           (axi_arid),
    .s_axi_araddr         (axi_araddr),
    .s_axi_arlen          (axi_arlen),
    .s_axi_arsize         (axi_arsize),
    .s_axi_arburst        (axi_arburst),
    .s_axi_arlock         (axi_arlock),
    .s_axi_arcache        (axi_arcache),
    .s_axi_arprot         (axi_arprot),
    .s_axi_arqos          (axi_arqos),
    .s_axi_arvalid        (axi_arvalid),
    .s_axi_arready        (axi_arready),
    .s_axi_rid            (axi_rid),
    .s_axi_rdata          (axi_rdata),
    .s_axi_rresp          (axi_rresp),
    .s_axi_rlast          (axi_rlast),
    .s_axi_rvalid         (axi_rvalid),
    .s_axi_rready         (axi_rready),
    // 温度输出
    .device_temp          ()
);

// ==================================================================
// SRIO: 探针模块(内含核) + 结果控制器 + 门铃发射机
// ==================================================================
wire        log_clk;
wire        log_rst;
wire        refclk_out;
wire        srio_rst;

// SRIO 上电复位(探针工程同款: refclk_out 计数释放); 仿真 VPX640_SIM_SHORT_POR
// 时缩到 2^8 拍(约 1.6us@156.25M), 上板 2^16(约 420us)
// 计数器异步清零挂核的 clk_lock_out(20260813 修复: 原无复位写法在仿真里
// X 初值卡死(X+1=X), 上板靠 FPGA 0 初态侥幸能跑; 挂 lock 后仿真/上板
// 行为一致且更稳——核时钟未锁定期间复位永不释放)
`ifdef VPX640_SIM_SHORT_POR
localparam SRIO_RST_BITS = 8;
`else
localparam SRIO_RST_BITS = 16;
`endif
wire        srio_clk_lock;
reg [SRIO_RST_BITS-1:0] srio_rst_cnt;
always @(posedge refclk_out or negedge srio_clk_lock) begin
    if (!srio_clk_lock) begin
        srio_rst_cnt <= {SRIO_RST_BITS{1'b0}};
    end else if (srio_rst_cnt != {SRIO_RST_BITS{1'b1}}) begin
        srio_rst_cnt <= srio_rst_cnt + 1'b1;
    end
end
assign srio_rst = ~(&srio_rst_cnt);

// 探针 <-> 加速器 512b 流(ILA v3: 主观测口, 20260817)
(* mark_debug = "true" *) wire [511:0] ps_tdata;      // srio_rx_path -> 加速器 s_axis_ps
(* mark_debug = "true" *) wire         ps_tvalid;
(* mark_debug = "true" *) wire         ps_tready;
(* mark_debug = "true" *) wire [511:0] pso_tdata;   // ILA v5     // 加速器 m_axis_pso -> 发通路
(* mark_debug = "true" *) wire         pso_tvalid;   // ILA v5
(* mark_debug = "true" *) wire         pso_tready;   // ILA v5
(* mark_debug = "true" *) wire         pso_tlast;    // ILA v5(20260819 v3.1: 每导出层末拍)

// 配置抽头(log_clk 域): 探针 -> srio_tx_ctrl
wire        cfg_wr_en;
wire [7:0]  cfg_wr_addr;
wire [31:0] cfg_wr_data;

// 收方向 64b 净荷(log_clk 域): 探针 -> srio_rx_path
wire [63:0] rx64_tdata;
wire        rx64_tvalid;
wire        rx64_tready;

// ireq(log_clk 域): srio_tx_ctrl(SWRITE 分包+门铃) -> 探针(核)
wire        ireq_tvalid;
wire        ireq_tready;
wire        ireq_tlast;
wire [63:0] ireq_tdata;
wire [7:0]  ireq_tkeep;
wire [31:0] ireq_tuser;
// iresp(20260823 门铃确认制): 核输出 -> srio_tx_v2(WB_DB_WAIT 等 DONE)
wire        iresp_tvalid;
wire        iresp_tready;
wire        iresp_tlast;
wire [63:0] iresp_tdata;

// 链路状态与仿真排空
wire        link_up;
wire [31:0] dbg_err_cnt;   // 非契约包丢弃计数(曾 ILA, v3 撤)
wire        rx_drain;
wire        tx_drain;

srio2axis #(
    .SIM_TRAIN_EN (SIM_TRAIN[0])
) u_srio2axis (
    .srio_rst           (srio_rst),
    .srio_clkp          (srio_clkp),
    .srio_clkn          (srio_clkn),
    .srio_clk_out       (refclk_out),
    .srio_rxn0          (srio_rxn0),
    .srio_rxp0          (srio_rxp0),
    .srio_rxn1          (srio_rxn1),
    .srio_rxp1          (srio_rxp1),
    .srio_rxn2          (srio_rxn2),
    .srio_rxp2          (srio_rxp2),
    .srio_rxn3          (srio_rxn3),
    .srio_rxp3          (srio_rxp3),
    .srio_txn0          (srio_txn0),
    .srio_txp0          (srio_txp0),
    .srio_txn1          (srio_txn1),
    .srio_txp1          (srio_txp1),
    .srio_txn2          (srio_txn2),
    .srio_txp2          (srio_txp2),
    .srio_txn3          (srio_txn3),
    .srio_txp3          (srio_txp3),
    // 配置抽头 -> srio_tx_ctrl(log_clk 域)
    .cfg_wr_en          (cfg_wr_en),
    .cfg_wr_addr        (cfg_wr_addr),
    .cfg_wr_data        (cfg_wr_data),
    // 收方向 64b 净荷 -> srio_rx_path(手术G)
    .rx64_tdata         (rx64_tdata),
    .rx64_tvalid        (rx64_tvalid),
    .rx64_tready        (rx64_tready),
    // ireq <- srio_tx_ctrl(SWRITE 分包+门铃); iresp -> srio_tx_ctrl(门铃确认制)
    .s_axis_ireq_tvalid (ireq_tvalid),
    .s_axis_ireq_tready (ireq_tready),
    .s_axis_ireq_tlast  (ireq_tlast),
    .s_axis_ireq_tdata  (ireq_tdata),
    .s_axis_ireq_tkeep  (ireq_tkeep),
    .s_axis_ireq_tuser  (ireq_tuser),
    .m_axis_iresp_tvalid(iresp_tvalid),
    .m_axis_iresp_tready(iresp_tready),
    .m_axis_iresp_tlast (iresp_tlast),
    .m_axis_iresp_tdata (iresp_tdata),
    .m_axis_iresp_tkeep (),
    .m_axis_iresp_tuser (),
    // log 域引出
    .log_clk_out        (log_clk),
    .log_rst_out        (log_rst),
    // 核时钟锁定引出(SRIO 上电复位计数器的异步清零)
    .clk_lock_out       (srio_clk_lock),
    .link_up_out        (link_up),
    .dbg_err_cnt        (dbg_err_cnt)
);

// ==================================================================
// 收方向: 64b 窄带跨钟 + dma_clk 域 64->512 拼装(协议 v5 通路重构)
// ==================================================================
srio_rx_path u_rx_path (
    .log_clk          (log_clk),
    .log_rst          (log_rst),
    .dma_clk          (dma_clk),
    .dma_rst_n        (dma_rst_n),
    .rx64_tdata       (rx64_tdata),
    .rx64_tvalid      (rx64_tvalid),
    .rx64_tready      (rx64_tready),
    .m_axis_ps_tdata  (ps_tdata),
    .m_axis_ps_tvalid (ps_tvalid),
    .m_axis_ps_tready (ps_tready),
    .drain_done       (rx_drain)
);

// ==================================================================
// 发方向(20260819 v3.1): AXIS 双生成 FIFO + last 驱动分包 + 每层门铃
//   (实例名沿用 u_tx_ctrl, TB 层级引用不动; 协议 v5 §6 + 20260819 修正案)
// ==================================================================
srio_tx_v2 #(
    .DSP_ID  (16'h00AA),
    .FPGA_ID (16'h0012)
) u_tx_ctrl (
    .dma_clk          (dma_clk),
    .dma_rst_n        (dma_rst_n),
    .log_clk          (log_clk),
    .log_rst          (log_rst),
    .s_axis_pso_tdata (pso_tdata),
    .s_axis_pso_tvalid(pso_tvalid),
    .s_axis_pso_tlast (pso_tlast),
    .s_axis_pso_tready(pso_tready),
    .cfg_wr_en        (cfg_wr_en),
    .cfg_wr_addr      (cfg_wr_addr),
    .cfg_wr_data      (cfg_wr_data),
    .link_up          (link_up),
    .ireq_tdata       (ireq_tdata),
    .ireq_tkeep       (ireq_tkeep),
    .ireq_tlast       (ireq_tlast),
    .ireq_tvalid      (ireq_tvalid),
    .ireq_tuser       (ireq_tuser),
    .ireq_tready      (ireq_tready),
    .iresp_tvalid     (iresp_tvalid),
    .iresp_tready     (iresp_tready),
    .iresp_tdata      (iresp_tdata),
    .iresp_tlast      (iresp_tlast),
    .drain_done       (tx_drain)
);

// ==================================================================
// CNN 加速器核(rtl_006)
// ==================================================================
cnn_acc_top #(
    .AXI_ADDR_WIDTH (32),
    .AXI_DATA_WIDTH (512),     // 与 DMA_AXI_DATA_WIDTH_512 宏契约一致, 直连 MIG 512 从口
    .AXI_ID_WIDTH   (4),
    .AXI_STRB_WIDTH (64)
) u_cnn_acc_top (
    .s_axis_ps_tvalid  (ps_tvalid),
    .s_axis_ps_tdata   (ps_tdata),
    .s_axis_ps_tready  (ps_tready),
    .m_axis_pso_tvalid (pso_tvalid),
    .m_axis_pso_tdata  (pso_tdata),
    .m_axis_pso_tlast  (pso_tlast),
    .m_axis_pso_tready (pso_tready),
    .clk               (ui_clk),
    .cal_clk           (cal_clk),
    .dma_clk           (dma_clk),
    .rst_n             (peripheral_aresetn),
    .init_calib_complete (init_calib_complete),
    .dma_rst_n         (dma_rst_n),
    .m_axi_awid        (axi_awid),
    .m_axi_awaddr      (axi_awaddr),
    .m_axi_awlen       (axi_awlen),
    .m_axi_awsize      (axi_awsize),
    .m_axi_awburst     (axi_awburst),
    .m_axi_awlock      (axi_awlock),
    .m_axi_awcache     (axi_awcache),
    .m_axi_awprot      (axi_awprot),
    .m_axi_awqos       (axi_awqos),
    .m_axi_awvalid     (axi_awvalid),
    .m_axi_awready     (axi_awready),
    .m_axi_wdata       (axi_wdata),
    .m_axi_wstrb       (axi_wstrb),
    .m_axi_wlast       (axi_wlast),
    .m_axi_wvalid      (axi_wvalid),
    .m_axi_wready      (axi_wready),
    .m_axi_bid         (axi_bid),
    .m_axi_bresp       (axi_bresp),
    .m_axi_bvalid      (axi_bvalid),
    .m_axi_bready      (axi_bready),
    .m_axi_arid        (axi_arid),
    .m_axi_araddr      (axi_araddr),
    .m_axi_arlen       (axi_arlen),
    .m_axi_arsize      (axi_arsize),
    .m_axi_arburst     (axi_arburst),
    .m_axi_arlock      (axi_arlock),
    .m_axi_arcache     (axi_arcache),
    .m_axi_arprot      (axi_arprot),
    .m_axi_arqos       (axi_arqos),
    .m_axi_arvalid     (axi_arvalid),
    .m_axi_arready     (axi_arready),
    .m_axi_rid         (axi_rid),
    .m_axi_rdata       (axi_rdata),
    .m_axi_rresp       (axi_rresp),
    .m_axi_rlast       (axi_rlast),
    .m_axi_rvalid      (axi_rvalid),
    .m_axi_rready      (axi_rready)
);

endmodule

/* RTL SELF-CHECK:
 * PASS: migrated wrapper is pure Verilog, ports explicitly wire.
 * PASS: UI reset is registered; CAL/DMA release via existing proc_sys_reset IP.
 * PASS: MIG owns differential buffer and all accelerator clocks.
 * PASS: no new state machine, debug port, or illegal-input recovery.
 * WARN: chip startup with sys_rst tied inactive must pass real MIG simulation.
 * WARN: inherited accelerator RTL is baseline code, not blanket style-certified.
 * WARN: top-level CDC/DRC/timing pending after mandatory chip smokes.
 */
