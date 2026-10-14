`timescale 1ns / 1ps
`include "all_defines.vh"

module cnn_acc_top #(
    parameter AXI_ADDR_WIDTH = 32,
    parameter AXI_DATA_WIDTH = `DMA_AXI_SELECTED_DATA_WIDTH,
    parameter AXI_ID_WIDTH   = 4,
    parameter AXI_STRB_WIDTH = `DMA_AXI_SELECTED_STRB_WIDTH
) (
    input  wire                       s_axis_ps_tvalid,
    input  wire [511:0]               s_axis_ps_tdata,
    output wire                       s_axis_ps_tready,

    output wire                       m_axis_pso_tvalid,
    output wire [511:0]               m_axis_pso_tdata,
    output wire                       m_axis_pso_tlast,
    input  wire                       m_axis_pso_tready,

    input  wire                       clk,
    input  wire                       cal_clk,
    input  wire                       dma_clk,
    input  wire                       rst_n,
    input  wire                       init_calib_complete,
    output wire                       dma_rst_n,

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

// 20260817 ILA v5 探针保命标记(仅属性, 不动逻辑): 派发指令流 + psi 流
(* mark_debug = "true" *) wire                         init_load_cmd_tvalid;
(* mark_debug = "true" *) wire [`RD_SM_CMD_WIDTH-1:0]  init_load_cmd_tdata;
(* mark_debug = "true" *) wire                         init_load_cmd_tready;

(* mark_debug = "true" *) wire                         init_wrcmd_tvalid;
(* mark_debug = "true" *) wire [`WR_SM_CMD_WIDTH-1:0]  init_wrcmd_tdata;
(* mark_debug = "true" *) wire                         init_wrcmd_tready;

(* mark_debug = "true" *) wire                         init_psi_tvalid;
(* mark_debug = "true" *) wire [511:0]                 init_psi_tdata;
(* mark_debug = "true" *) wire                         init_psi_tready;

wire                         load_ba_rden;
wire [10:0]                  load_ba_rdaddr;
wire [31:0]                  load_ba_rddata;
wire                         store_ba_rden;
wire [10:0]                  store_ba_rdaddr;
wire [31:0]                  store_ba_rddata;

wire                         load_dma_cmd_tvalid;
wire [`CMD2_CMD_WIDTH-1:0]   load_dma_cmd_tdata;
wire                         load_dma_cmd_tready;

wire                         dma_rdddr_tvalid;
wire [511:0]                 dma_rdddr_tdata;
wire                         dma_rdddr_tready;

wire                         store_wrcmd2_tvalid;
wire [`CMD2_CMD_WIDTH-1:0]   store_wrcmd2_tdata;
wire                         store_wrcmd2_tready;

wire                         store_wrddr_tvalid;
wire [511:0]                 store_wrddr_tdata;
wire                         store_wrddr_tready;

wire                         load_cal_cmd_tvalid;
wire [111:0]                 load_cal_cmd_tdata;
wire                         load_cal_cmd_tready;

wire                         load_fm_tvalid;
wire [511:0]                 load_fm_tdata;
wire                         load_fm_tready;
wire                         load_cpad_tvalid;
wire [511:0]                 load_cpad_tdata;
wire                         load_cpad_tready;
wire                         load_we_tvalid;
wire [511:0]                 load_we_tdata;
wire                         load_we_tready;
wire                         load_bias_tvalid;
wire [511:0]                 load_bias_tdata;
wire                         load_bias_tready;
wire                         load_quan_param_tvalid;
wire [511:0]                 load_quan_param_tdata;
wire                         load_quan_param_tready;
wire                         load_quan_fm1_tvalid;
wire [511:0]                 load_quan_fm1_tdata;
wire                         load_quan_fm1_tready;
wire                         load_quan_fm2_tvalid;
wire [511:0]                 load_quan_fm2_tdata;
wire                         load_quan_fm2_tready;
wire                         load_act_tvalid;
wire [511:0]                 load_act_tdata;
wire                         load_act_tready;

wire                         cal_act_tvalid;
wire [511:0]                 cal_act_tdata;
wire                         cal_act_tready;
wire                         cal_pooling_tvalid;
wire [511:0]                 cal_pooling_tdata;
wire                         cal_pooling_tready;
wire                         cal_res_tvalid;
wire [511:0]                 cal_res_tdata;
wire                         cal_res_tready;

wire                         store_circular_tvalid;
wire [511:0]                 store_circular_tdata;
wire                         store_circular_tready;
wire                         store_cpad_wrcmd_tvalid;
wire [103:0]                 store_cpad_wrcmd_tdata;
wire                         store_cpad_wrcmd_tready;

wire                         store_done_req;
wire                         store_done_ack;
wire                         store_fm_done;

// dma_clk 域复位（proc_sys_reset 标准件：ext_reset_in 异步进、域内同步出，
// 取代手写 dma_rst_d1/d2 两级链；语义同为异步置位、dma_clk 同步释放。
// 20260813 复位链重构：ext_reset_in = rst_n（ui 域复位，已含 por/MMCM 锁），
// aux_reset_in = init_calib_complete（DDR3 校准完成前保持复位）——原顶层手写
// 校准闸门删除后，闸门职责回归 IP 的 aux 口；输出经端口导出供顶层传输通路共用）
rst_dma_clk u_rst_dma_clk (
    .slowest_sync_clk   (dma_clk),
    .ext_reset_in       (rst_n),
    .aux_reset_in       (init_calib_complete),
    .mb_debug_sys_rst   (1'b0),
    .dcm_locked         (1'b1),
    .mb_reset           (),
    .bus_struct_reset   (),
    .peripheral_reset   (),
    .interconnect_aresetn(),
    .peripheral_aresetn (dma_rst_n)
);

// cal_clk 域复位（proc_sys_reset 标准件，取代手写 cal_rst_d1/d2 两级链；
// ext/aux 闸门同 dma 域：ext=ui 域复位，aux=校准完成）
(* mark_debug = "true" *) wire cal_rst_n;   // 20260817 ILA v3: 计算域复位探针

rst_cal_clk u_rst_cal_clk (
    .slowest_sync_clk   (cal_clk),
    .ext_reset_in       (rst_n),
    .aux_reset_in       (init_calib_complete),
    .mb_debug_sys_rst   (1'b0),
    .dcm_locked         (1'b1),
    .mb_reset           (),
    .bus_struct_reset   (),
    .peripheral_reset   (),
    .interconnect_aresetn(),
    .peripheral_aresetn (cal_rst_n)
);

initial_top u_initial_top (
    .clk                  (dma_clk),
    .rst_n                (dma_rst_n),
    .s_axis_ps_tvalid     (s_axis_ps_tvalid),
    .s_axis_ps_tdata      (s_axis_ps_tdata),
    .s_axis_ps_tready     (s_axis_ps_tready),
    .m_axis_rdcmd_tvalid  (init_load_cmd_tvalid),
    .m_axis_rdcmd_tdata   (init_load_cmd_tdata),
    .m_axis_rdcmd_tready  (init_load_cmd_tready),
    .m_axis_wrcmd_tvalid  (init_wrcmd_tvalid),
    .m_axis_wrcmd_tdata   (init_wrcmd_tdata),
    .m_axis_wrcmd_tready  (init_wrcmd_tready),
    .m_axis_psi_tvalid    (init_psi_tvalid),
    .m_axis_psi_tdata     (init_psi_tdata),
    .m_axis_psi_tready    (init_psi_tready),
    .load_ba_rd           (load_ba_rden),
    .load_ba_ptr          (load_ba_rdaddr),
    .load_ba_data         (load_ba_rddata),
    .store_ba_rd          (store_ba_rden),
    .store_ba_ptr         (store_ba_rdaddr),
    .store_ba_data        (store_ba_rddata)
);

load_top u_load_top (
    .clk                         (dma_clk),
    .rst_n                       (dma_rst_n),
    .s_axis_load_cmd_tvalid      (init_load_cmd_tvalid),
    .s_axis_load_cmd_tdata       (init_load_cmd_tdata),
    .s_axis_load_cmd_tready      (init_load_cmd_tready),
    .m_axis_dma_cmd_tvalid       (load_dma_cmd_tvalid),
    .m_axis_dma_cmd_tdata        (load_dma_cmd_tdata),
    .m_axis_dma_cmd_tready       (load_dma_cmd_tready),
    .s_axis_rdddr_tvalid         (dma_rdddr_tvalid),
    .s_axis_rdddr_tdata          (dma_rdddr_tdata),
    .s_axis_rdddr_tready         (dma_rdddr_tready),
    .m_axis_pso_tvalid           (m_axis_pso_tvalid),
    .m_axis_pso_tdata            (m_axis_pso_tdata),
    .m_axis_pso_tlast            (m_axis_pso_tlast),
    .m_axis_pso_tready           (m_axis_pso_tready),
    .cal_clk                     (cal_clk),
    .cal_rst_n                   (cal_rst_n),
    .m_axis_fm_tvalid            (load_fm_tvalid),
    .m_axis_fm_tdata             (load_fm_tdata),
    .m_axis_fm_tready            (load_fm_tready),
    .m_axis_cpad_tvalid          (load_cpad_tvalid),
    .m_axis_cpad_tdata           (load_cpad_tdata),
    .m_axis_cpad_tready          (load_cpad_tready),
    .m_axis_we_tvalid            (load_we_tvalid),
    .m_axis_we_tdata             (load_we_tdata),
    .m_axis_we_tready            (load_we_tready),
    .m_axis_bias_tvalid          (load_bias_tvalid),
    .m_axis_bias_tdata           (load_bias_tdata),
    .m_axis_bias_tready          (load_bias_tready),
    .m_axis_quan_param_tvalid    (load_quan_param_tvalid),
    .m_axis_quan_param_tdata     (load_quan_param_tdata),
    .m_axis_quan_param_tready    (load_quan_param_tready),
    .m_axis_quan_fm1_tvalid      (load_quan_fm1_tvalid),
    .m_axis_quan_fm1_tdata       (load_quan_fm1_tdata),
    .m_axis_quan_fm1_tready      (load_quan_fm1_tready),
    .m_axis_quan_fm2_tvalid      (load_quan_fm2_tvalid),
    .m_axis_quan_fm2_tdata       (load_quan_fm2_tdata),
    .m_axis_quan_fm2_tready      (load_quan_fm2_tready),
    .m_axis_act_tvalid           (load_act_tvalid),
    .m_axis_act_tdata            (load_act_tdata),
    .m_axis_act_tready           (load_act_tready),
    .m_axis_cal_cmd_tvalid       (load_cal_cmd_tvalid),
    .m_axis_cal_cmd_tdata        (load_cal_cmd_tdata),
    .m_axis_cal_cmd_tready       (load_cal_cmd_tready),
    .load_ba_rden                (load_ba_rden),
    .load_ba_rdaddr              (load_ba_rdaddr),
    .load_ba_rddata              (load_ba_rddata),
    .store_fm_done               (store_fm_done)
);

cal_top u_cal_top (
    .clk                         (cal_clk),
    .rst_n                       (cal_rst_n),
    .s_axis_rdcmd_tvalid         (load_cal_cmd_tvalid),
    .s_axis_rdcmd_tdata          (load_cal_cmd_tdata),
    .s_axis_rdcmd_tready         (load_cal_cmd_tready),
    .store_done_req              (store_done_req),
    .store_done_ack              (store_done_ack),
    .s_axis_fm_tvalid            (load_fm_tvalid),
    .s_axis_fm_tdata             (load_fm_tdata),
    .s_axis_fm_tready            (load_fm_tready),
    .s_axis_cpad_tvalid          (load_cpad_tvalid),
    .s_axis_cpad_tdata           (load_cpad_tdata),
    .s_axis_cpad_tready          (load_cpad_tready),
    .s_axis_cpad_store_tvalid    (store_circular_tvalid),
    .s_axis_cpad_store_tdata     (store_circular_tdata),
    .s_axis_cpad_store_tready    (store_circular_tready),
    .s_axis_wrcmd_tvalid         (store_cpad_wrcmd_tvalid),
    .s_axis_wrcmd_tdata          (store_cpad_wrcmd_tdata),
    .s_axis_wrcmd_tready         (store_cpad_wrcmd_tready),
    .s_axis_we_tvalid            (load_we_tvalid),
    .s_axis_we_tdata             (load_we_tdata),
    .s_axis_we_tready            (load_we_tready),
    .s_axis_bias_tvalid          (load_bias_tvalid),
    .s_axis_bias_tdata           (load_bias_tdata),
    .s_axis_bias_tready          (load_bias_tready),
    .s_axis_quan_param_tvalid    (load_quan_param_tvalid),
    .s_axis_quan_param_tdata     (load_quan_param_tdata),
    .s_axis_quan_param_tready    (load_quan_param_tready),
    .s_axis_quan_fm1_tvalid      (load_quan_fm1_tvalid),
    .s_axis_quan_fm1_tdata       (load_quan_fm1_tdata),
    .s_axis_quan_fm1_tready      (load_quan_fm1_tready),
    .s_axis_quan_fm2_tvalid      (load_quan_fm2_tvalid),
    .s_axis_quan_fm2_tdata       (load_quan_fm2_tdata),
    .s_axis_quan_fm2_tready      (load_quan_fm2_tready),
    .s_axis_act_tvalid           (load_act_tvalid),
    .s_axis_act_tdata            (load_act_tdata),
    .s_axis_act_tready           (load_act_tready),
    .m_axis_act_tvalid           (cal_act_tvalid),
    .m_axis_act_tdata            (cal_act_tdata),
    .m_axis_act_tready           (cal_act_tready),
    .m_axis_pooling_tvalid       (cal_pooling_tvalid),
    .m_axis_pooling_tdata        (cal_pooling_tdata),
    .m_axis_pooling_tready       (cal_pooling_tready),
    .m_axis_res_tvalid           (cal_res_tvalid),
    .m_axis_res_tdata            (cal_res_tdata),
    .m_axis_res_tready           (cal_res_tready)
);

store_top u_store_top (
    .clk                         (dma_clk),
    .rst_n                       (dma_rst_n),
    .s_axis_wrcmd_tvalid         (init_wrcmd_tvalid),
    .s_axis_wrcmd_tdata          (init_wrcmd_tdata),
    .s_axis_wrcmd_tready         (init_wrcmd_tready),
    .m_axis_wrcmd2_tvalid        (store_wrcmd2_tvalid),
    .m_axis_wrcmd2_tdata         (store_wrcmd2_tdata),
    .m_axis_wrcmd2_tready        (store_wrcmd2_tready),
    .m_axis_wrddr_tvalid         (store_wrddr_tvalid),
    .m_axis_wrddr_tdata          (store_wrddr_tdata),
    .m_axis_wrddr_tready         (store_wrddr_tready),
    .m_axis_circular_tvalid      (store_circular_tvalid),
    .m_axis_circular_tdata       (store_circular_tdata),
    .m_axis_circular_tready      (store_circular_tready),
    .m_axis_wrcmd_tvalid         (store_cpad_wrcmd_tvalid),
    .m_axis_wrcmd_tdata          (store_cpad_wrcmd_tdata),
    .m_axis_wrcmd_tready         (store_cpad_wrcmd_tready),
    .s_axis_ps_tvalid            (init_psi_tvalid),
    .s_axis_ps_tdata             (init_psi_tdata),
    .s_axis_ps_tready            (init_psi_tready),
    .cal_clk                     (cal_clk),
    .s_axis_act_tvalid           (cal_act_tvalid),
    .s_axis_act_tdata            (cal_act_tdata),
    .s_axis_act_tready           (cal_act_tready),
    .s_axis_pooling_tvalid       (cal_pooling_tvalid),
    .s_axis_pooling_tdata        (cal_pooling_tdata),
    .s_axis_pooling_tready       (cal_pooling_tready),
    .s_axis_res_tvalid           (cal_res_tvalid),
    .s_axis_res_tdata            (cal_res_tdata),
    .s_axis_res_tready           (cal_res_tready),
    .store_done_req              (store_done_req),
    .store_done_ack              (store_done_ack),
    .store_fm_done               (store_fm_done),
    .store_ba_rden               (store_ba_rden),
    .store_ba_rdaddr             (store_ba_rdaddr),
    .store_ba_rddata             (store_ba_rddata)
);

dma_axi4_master #(
    .AXI_ADDR_WIDTH (AXI_ADDR_WIDTH),
    .AXI_DATA_WIDTH (AXI_DATA_WIDTH),
    .AXI_ID_WIDTH   (AXI_ID_WIDTH),
    .AXI_STRB_WIDTH (AXI_STRB_WIDTH)
) u_dma_axi4_master (
    .clk                    (clk),
    .usr_clk                (dma_clk),
    .rst_n                  (rst_n),
    .s_axis_wrcmd2_tvalid   (store_wrcmd2_tvalid),
    .s_axis_wrcmd2_tready   (store_wrcmd2_tready),
    .s_axis_wrcmd2_tdata    (store_wrcmd2_tdata),
    .s_axis_wrddr_tvalid    (store_wrddr_tvalid),
    .s_axis_wrddr_tready    (store_wrddr_tready),
    .s_axis_wrddr_tdata     (store_wrddr_tdata),
    .s_axis_rdcmd2_tvalid   (load_dma_cmd_tvalid),
    .s_axis_rdcmd2_tready   (load_dma_cmd_tready),
    .s_axis_rdcmd2_tdata    (load_dma_cmd_tdata),
    .m_axis_rdddr_tvalid    (dma_rdddr_tvalid),
    .m_axis_rdddr_tready    (dma_rdddr_tready),
    .m_axis_rdddr_tdata     (dma_rdddr_tdata),
    .m_axi_awid             (m_axi_awid),
    .m_axi_awaddr           (m_axi_awaddr),
    .m_axi_awlen            (m_axi_awlen),
    .m_axi_awsize           (m_axi_awsize),
    .m_axi_awburst          (m_axi_awburst),
    .m_axi_awlock           (m_axi_awlock),
    .m_axi_awcache          (m_axi_awcache),
    .m_axi_awprot           (m_axi_awprot),
    .m_axi_awqos            (m_axi_awqos),
    .m_axi_awvalid          (m_axi_awvalid),
    .m_axi_awready          (m_axi_awready),
    .m_axi_wdata            (m_axi_wdata),
    .m_axi_wstrb            (m_axi_wstrb),
    .m_axi_wlast            (m_axi_wlast),
    .m_axi_wvalid           (m_axi_wvalid),
    .m_axi_wready           (m_axi_wready),
    .m_axi_bid              (m_axi_bid),
    .m_axi_bresp            (m_axi_bresp),
    .m_axi_bvalid           (m_axi_bvalid),
    .m_axi_bready           (m_axi_bready),
    .m_axi_arid             (m_axi_arid),
    .m_axi_araddr           (m_axi_araddr),
    .m_axi_arlen            (m_axi_arlen),
    .m_axi_arsize           (m_axi_arsize),
    .m_axi_arburst          (m_axi_arburst),
    .m_axi_arlock           (m_axi_arlock),
    .m_axi_arcache          (m_axi_arcache),
    .m_axi_arprot           (m_axi_arprot),
    .m_axi_arqos            (m_axi_arqos),
    .m_axi_arvalid          (m_axi_arvalid),
    .m_axi_arready          (m_axi_arready),
    .m_axi_rid              (m_axi_rid),
    .m_axi_rdata            (m_axi_rdata),
    .m_axi_rresp            (m_axi_rresp),
    .m_axi_rlast            (m_axi_rlast),
    .m_axi_rvalid           (m_axi_rvalid),
    .m_axi_rready           (m_axi_rready)
);

endmodule

////////////////////////////////////////////////////////////////////////////////
// RTL 自检清单(rtl_rules 规则 35, 20260813 复位链重构轮):
//   [PASS] 纯 Verilog, 无 SV 关键字; 端口显式方向+wire+位宽
//   [PASS] 组合/时序分离, 赋值符合规(本轮无新增组合/时序逻辑)
//   [PASS] 复位链: 每域单一异步复位源(rst_dma_clk/rst_cal_clk 各管一域),
//          释放经 IP 内同步(异步置位/同步释放), 链上无手写组合逻辑
//   [PASS] 端口增删(init_calib_complete/dma_rst_n)为用户拍板的架构决策(方案B)
//   [PASS] 无 initial/#delay/系统任务/for/while/repeat/function//%
//   [PASS] 无隐式 latch/组合环路; 状态机 localparam+default(未改动)
//   [PASS] RAM 均为独立模块例化, 未改动
//   [PASS] 仿真排空指示(drain 相关)未改动
//   [NOTE] init_calib_complete 为 ui_clk 域信号, 作 aux 异步闸门进 dma/cal 域 IP,
//          由 IP 内部 C_AUX_RST_WIDTH 滤波+本域同步, 属 aux 口设计用途(规则 18)
////////////////////////////////////////////////////////////////////////////////
