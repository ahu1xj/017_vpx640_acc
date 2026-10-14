////////////////////////////////////////////////////////////////////////////////
// VPX610 集成副本 —— srio2axis(探针模块, 第三方上板验证代码)
//
// 来源: E:\zcodework\025_690T_env\work_prob\huangluxuan\VPX610\VPX610\
//       FPGA\FPGA.srcs\sources_1\srio2axis.v
// 该模块在原探针工程中已完成上板验证(SRIO Gen2 x4 3.125G HELLO,
// treq/tresp 64b 用户口 + 两个 axis_data_fifo_0 完成 log_clk<->用户钟跨域)。
// 集成本副本时仅做以下手术, 其余字节与原文件保持一致:
//
//   手术A: 新增 parameter [0:0] SIM_TRAIN_EN(默认 1'b0), 接到 srio_inst 的
//          .sim_train_en(原来绑 1'b0)。仿真时置 1 走快速链路训练。
//   手术B: NWRITE 路由改为配置写抽头, 不再进数据流:
//          - req_route2m_axis_fifo 由 (NWRITE||SWRITE) 改为仅 FTYPE_SWRITE;
//          - 输出 cfg_wr_en/cfg_wr_addr[7:0]/cfg_wr_data[31:0](log_clk 域):
//            NWRITE 包(ft==NWRITE, 含 NWRITE_R)第一个数据拍握手时锁存
//            头地址低 8 位与本拍低 32 位(小端), cfg_wr_en 随后拉高 4 个
//            log_clk 周期(展宽), addr/data 在展宽及其后保持窗口内稳定;
//          - 展宽+保持共 8 拍内对后续 NWRITE 数据拍反压(treq_tready 加
//            cfg_busy 项), 保证同一时刻只有一笔配置写在窗内;
//          (v5: 配置寄存器就在 log_clk 域消费, 展宽保留以维持最小改动;
//           消费端对 commit 做沿检测, 对寄存器写同值重复无害)
//   手术C: ILA 例化(ila_0 x2 / ila_1 x2)整段移除——008 不需要在线调试核。
//   手术D: s_axis_ireq_*/m_axis_iresp_* 由绑 0/悬空改为引出为模块端口,
//          顶层接 srio_tx_ctrl(SWRITE 分包 + 门铃), FPGA 侧唯一主动发包。
//   手术E: log_clk/log_rst 引出为 log_clk_out/log_rst_out。
//   手术F: clk_lock_out 引出(顶层 SRIO 上电复位计数器的异步清零)。
//   手术G(v5 通路重构, 20260813):
//          - 删除: 512b 拼装逻辑、m_axis_fifo、s_axis_fifo(两个 512b BRAM
//            异步 FIFO)、NREAD 响应引擎、nwrite_r 响应链、check_nread;
//            相应 clk/rst(用户钟)、m_axis_*(512b)、s_axis_*(512b)端口移除。
//          - 收方向改为输出 64b 路由净荷 rx64_*(log_clk 域): 仅 FTYPE_SWRITE
//            净荷拍直通(拍内字节已倒序), 下游接 srio_rx_path 窄带跨钟;
//          - tresp 通道绑死(v5 退役 NREAD; 核侧无响应需求);
//          - NREAD/NWRITE_R/DOORBELL 及其他非 SWRITE/NWRITE 包: 照收丢弃,
//            记 dbg_err_cnt(协议 v5 §8)。
//
// 注意: 本文件为第三方代码集成副本, 保留原作者代码风格(端口不标 wire 等),
// 不套用项目 RTL 规则全文; 新增逻辑尽量贴合原风格。
////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps

module srio2axis #(
    parameter   [0:0]   SIM_TRAIN_EN = 1'b0     // 手术A: 1=仿真快速链路训练
)(
    input           srio_rst,
    // Serial RapidIO
    input           srio_clkp,
    input           srio_clkn,
    output          srio_clk_out,
    input           srio_rxn0,
    input           srio_rxp0,
    input           srio_rxn1,
    input           srio_rxp1,
    input           srio_rxn2,
    input           srio_rxp2,
    input           srio_rxn3,
    input           srio_rxp3,
    output          srio_txn0,
    output          srio_txp0,
    output          srio_txn1,
    output          srio_txp1,
    output          srio_txn2,
    output          srio_txp2,
    output          srio_txn3,
    output          srio_txp3,

    // 手术B: NWRITE 配置写抽头(log_clk 域, 展宽输出)
    output          cfg_wr_en,
    output  [7:0]   cfg_wr_addr,
    output  [31:0]  cfg_wr_data,

    // 手术G: 收方向 64b 路由净荷(log_clk 域, 仅 SWRITE 净荷, 字节已倒序)
    output  [63:0]  rx64_tdata,
    output          rx64_tvalid,
    input           rx64_tready,

    // 手术D: Initiator 请求/响应通道引出(原绑 0/悬空), 接 srio_tx_ctrl
    input           s_axis_ireq_tvalid,
    output          s_axis_ireq_tready,
    input           s_axis_ireq_tlast,
    input   [63:0]  s_axis_ireq_tdata,
    input   [7:0]   s_axis_ireq_tkeep,
    input   [31:0]  s_axis_ireq_tuser,
    output          m_axis_iresp_tvalid,
    input           m_axis_iresp_tready,
    output          m_axis_iresp_tlast,
    output  [63:0]  m_axis_iresp_tdata,
    output  [7:0]   m_axis_iresp_tkeep,
    output  [31:0]  m_axis_iresp_tuser,

    // 手术E: log_clk/log_rst 引出(核内产生)
    output          log_clk_out,
    output          log_rst_out,
    // 手术F: clk_lock_out 引出(顶层 SRIO 上电复位计数器的异步清零)
    output          clk_lock_out,
    // 手术G: 链路就绪(port_initialized & link_initialized, 分包器起包条件)
    output          link_up_out,
    // 手术G: 非契约包丢弃计数(协议 v5 §8 调试观察)
    output  [31:0]  dbg_err_cnt
);
    // RapidIO FTYPE encoding
    localparam  [3:0]   FTYPE_NREAD = 4'd2;
    localparam  [3:0]   FTYPE_NWRITE = 4'd5;
    localparam  [3:0]   FTYPE_SWRITE = 4'd6;
    localparam  [3:0]   FTYPE_RESP = 4'd13;

    // PHY
    wire        log_clk;
    wire        log_rst;
    // 手术E: 域信号引出
    assign      log_clk_out = log_clk;
    assign      log_rst_out = log_rst;
    wire        treq_tvalid;
    wire        treq_tready;
    wire        treq_tlast;
    wire [63:0] treq_tdata;
    wire [7:0]  treq_tkeep;
    wire [31:0] treq_tuser;
    // 20260816 上板调试: ILA 标记(不影响逻辑)
    wire        dbg_treq_tvalid;
    wire        dbg_treq_tready;
    wire        dbg_treq_tlast;
    wire [63:0] dbg_treq_tdata;
    wire [31:0] dbg_treq_tuser;
    wire        dbg_treq_theader;
    reg         treq_theader;   // 声明前移(20260819: VCS 禁止先用后声明)
    assign      dbg_treq_tvalid  = treq_tvalid;
    assign      dbg_treq_tready  = treq_tready;
    assign      dbg_treq_tlast   = treq_tlast;
    assign      dbg_treq_tdata   = treq_tdata;
    assign      dbg_treq_tuser   = treq_tuser;
    assign      dbg_treq_theader = treq_theader;
    wire        port_initialized;
    wire        link_initialized;
    // 手术G: 链路就绪 = port_initialized & link_initialized
    assign      link_up_out = port_initialized && link_initialized;
    srio_gen2_0 srio_inst (
        .sys_clkp                       (srio_clkp),
        .sys_clkn                       (srio_clkn),
        .sys_rst                        (srio_rst),
        .log_clk_out                    (log_clk),
        .phy_clk_out                    (),
        .gt_clk_out                     (),
        .gt_pcs_clk_out                 (),
        .drpclk_out                     (),
        .refclk_out                     (srio_clk_out),
        .clk_lock_out                   (clk_lock_out),   // 手术F: 原悬空
        .cfg_rst_out                    (),
        .log_rst_out                    (log_rst),
        .buf_rst_out                    (),
        .phy_rst_out                    (),
        .gt_pcs_rst_out                 (),
        .gt0_qpll_lock_out              (),
        .gt0_qpll_clk_out               (),
        .gt0_qpll_out_refclk_out        (),
        .srio_rxn0                      (srio_rxn0),
        .srio_rxp0                      (srio_rxp0),
        .srio_rxn1                      (srio_rxn1),
        .srio_rxp1                      (srio_rxp1),
        .srio_rxn2                      (srio_rxn2),
        .srio_rxp2                      (srio_rxp2),
        .srio_rxn3                      (srio_rxn3),
        .srio_rxp3                      (srio_rxp3),
        .srio_txn0                      (srio_txn0),
        .srio_txp0                      (srio_txp0),
        .srio_txn1                      (srio_txn1),
        .srio_txp1                      (srio_txp1),
        .srio_txn2                      (srio_txn2),
        .srio_txp2                      (srio_txp2),
        .srio_txn3                      (srio_txn3),
        .srio_txp3                      (srio_txp3),
        .s_axis_ireq_tvalid             (s_axis_ireq_tvalid), // 手术D: 引出, 接 srio_tx_ctrl
        .s_axis_ireq_tready             (s_axis_ireq_tready),
        .s_axis_ireq_tlast              (s_axis_ireq_tlast),
        .s_axis_ireq_tdata              (s_axis_ireq_tdata),
        .s_axis_ireq_tkeep              (s_axis_ireq_tkeep),
        .s_axis_ireq_tuser              (s_axis_ireq_tuser),
        .m_axis_iresp_tvalid            (m_axis_iresp_tvalid),
        .m_axis_iresp_tready            (m_axis_iresp_tready),
        .m_axis_iresp_tlast             (m_axis_iresp_tlast),
        .m_axis_iresp_tdata             (m_axis_iresp_tdata),
        .m_axis_iresp_tkeep             (m_axis_iresp_tkeep),
        .m_axis_iresp_tuser             (m_axis_iresp_tuser),
        .m_axis_treq_tvalid             (treq_tvalid),      // only use slave mode
        .m_axis_treq_tready             (treq_tready),
        .m_axis_treq_tlast              (treq_tlast),
        .m_axis_treq_tdata              (treq_tdata),
        .m_axis_treq_tkeep              (treq_tkeep),
        .m_axis_treq_tuser              (treq_tuser),
        .s_axis_tresp_tvalid            (1'b0),             // 手术G: tresp 绑死(NREAD 退役)
        .s_axis_tresp_tready            (),
        .s_axis_tresp_tlast             (1'b0),
        .s_axis_tresp_tdata             (64'd0),
        .s_axis_tresp_tkeep             (8'd0),
        .s_axis_tresp_tuser             (32'd0),
        .s_axi_maintr_rst               (1'b0),             // maintaining register never used
        .s_axi_maintr_awvalid           (1'b0),
        .s_axi_maintr_awready           (),
        .s_axi_maintr_awaddr            (32'd0),
        .s_axi_maintr_wvalid            (1'b0),
        .s_axi_maintr_wready            (),
        .s_axi_maintr_wdata             (32'd0),
        .s_axi_maintr_bvalid            (),
        .s_axi_maintr_bready            (1'b0),
        .s_axi_maintr_bresp             (),
        .s_axi_maintr_arvalid           (1'b0),
        .s_axi_maintr_arready           (),
        .s_axi_maintr_araddr            (32'd0),
        .s_axi_maintr_rvalid            (),
        .s_axi_maintr_rready            (1'b0),
        .s_axi_maintr_rdata             (),
        .s_axi_maintr_rresp             (),
        .sim_train_en                   (SIM_TRAIN_EN),     // 手术A: 原绑 1'b0
        .force_reinit                   (1'b0),
        .phy_link_reset                 (1'b0),
        .phy_mce                        (1'b0),
        .phy_rcvd_mce                   (),
        .phy_rcvd_link_reset            (),
        .phy_debug                      (),
        .gtrx_disperr_or                (),
        .gtrx_notintable_or             (),
        .port_error                     (),
        .port_timeout                   (),
        .srio_host                      (),
        .port_decode_error              (),
        .deviceid                       (),
        .idle2_selected                 (),
        .phy_lcl_master_enable_out      (),
        .buf_lcl_response_only_out      (),
        .buf_lcl_tx_flow_control_out    (),
        .buf_lcl_phy_buf_stat_out       (),
        .phy_lcl_phy_next_fm_out        (),
        .phy_lcl_phy_last_ack_out       (),
        .phy_lcl_phy_rewind_out         (),
        .phy_lcl_phy_rcvd_buf_stat_out  (),
        .phy_lcl_maint_only_out         (),
        .port_initialized               (port_initialized),     // 手术G: 原悬空, link_up_out 用
        .link_initialized               (link_initialized),     // 手术G: 原悬空, link_up_out 用
        .idle_selected                  (),
        .mode_1x                        ()
    );
    // REQ
    always @(posedge log_clk) begin
        if (log_rst)
            treq_theader <= 1'b1;
        else if (treq_tvalid && treq_tready) begin
            if (treq_tlast)
                treq_theader <= 1'b1;
            else
                treq_theader <= 1'b0;
        end
    end
    reg [15:0]  req_src;
    reg [15:0]  req_dst;
    reg [63:0]  req_header;
    always @(posedge log_clk) begin
        if (treq_tvalid && treq_tready && treq_theader) begin
            req_src <= treq_tuser[31:16];
            req_dst <= treq_tuser[15:0];
            req_header <= treq_tdata;
        end
    end
    wire [7:0]  req_tid = req_header[63:56];
    wire [3:0]  req_ftype = req_header[55:52];
    wire [3:0]  req_ttype = req_header[51:48];
    wire [1:0]  req_prio = req_header[46:45];
    wire        req_crf = req_header[44];
    wire [7:0]  req_size = req_header[43:36];
    wire [33:0] req_addr = req_header[33:0];
    wire        req_route2rx64 = req_ftype == FTYPE_SWRITE;   // 手术B/G: NWRITE 走配置抽头, 仅 SWRITE 进数据流
    genvar b;
    wire [63:0] treq_tdata_le;
    wire [7:0]  treq_tkeep_le;
    generate
    for (b = 0; b < 8; b = b + 1) begin : swap_treq
        assign treq_tdata_le[b*8 +: 8] = treq_tdata[(7-b)*8 +: 8];
        assign treq_tkeep_le[b]        = treq_tkeep[7-b];
    end
    endgenerate
    // ==================================================================
    // 手术B: NWRITE 配置写抽头(log_clk 域)
    //   NWRITE/NWRITE_R 净荷首拍握手时锁存地址低 8 位与小端低 32 位数据,
    //   cfg_wr_en 随后拉高 4 个 log_clk 周期(展宽); cfg_hold 计满 8 拍,
    //   其中后 4 拍为数据保持窗口: 展宽窗口内经 treq_tready 的 cfg_busy
    //   项反压后续 NWRITE 数据拍, 同一时刻只有一笔配置写在窗。
    // ==================================================================
    reg         cfg_first;      // 高 = 下一净荷拍为 NWRITE 首拍
    reg  [7:0]  cfg_addr_r;
    reg  [31:0] cfg_data_r;
    reg  [3:0]  cfg_hold;       // 展宽+保持计数, 非 0 即 cfg_busy
    wire        cfg_pload = treq_tvalid && treq_tready && !treq_theader
                            && (req_ftype == FTYPE_NWRITE);
    wire        cfg_busy  = (cfg_hold != 4'd0);
    always @(posedge log_clk) begin
        if (log_rst) begin
            cfg_first <= 1'b0;
            cfg_hold  <= 4'd0;
        end else begin
            if (treq_tvalid && treq_tready && treq_theader)
                cfg_first <= (treq_tdata[55:52] == FTYPE_NWRITE) && !treq_tlast;
            else if (cfg_pload)
                cfg_first <= 1'b0;
            if (cfg_pload && cfg_first) begin
                cfg_addr_r <= req_addr[7:0];
                cfg_data_r <= treq_tdata_le[31:0];
                cfg_hold   <= 4'd8;
            end else if (cfg_busy)
                cfg_hold   <= cfg_hold - 4'd1;
        end
    end
    assign  cfg_wr_en   = (cfg_hold >= 4'd5);   // 展宽段: 4 拍
    assign  cfg_wr_addr = cfg_addr_r;
    assign  cfg_wr_data = cfg_data_r;

    // ==================================================================
    // 手术G: 收方向 64b 路由净荷(仅 SWRITE, 直通下游 srio_rx_path)
    // ==================================================================
    assign  rx64_tdata  = treq_tdata_le;
    assign  rx64_tvalid = !treq_theader && req_route2rx64 && treq_tvalid;

    // tready: 包头照收; SWRITE 净荷反压直通 rx64; NWRITE 保持窗内拦后续拍;
    // 其他(NREAD/NWRITE_R/DOORBELL 等)照收丢弃
    assign  treq_tready = treq_theader ? 1'b1 :
                          req_route2rx64 ? rx64_tready :
                          !cfg_busy;

    // ==================================================================
    // 手术G: 非契约包丢弃计数(协议 v5 §8; NREAD/NWRITE_R/DOORBELL/其他)
    // ==================================================================
    reg  [31:0] err_drop_cnt;
    wire        drop_pkt = treq_tvalid && treq_tready && treq_theader && !treq_tlast
                           && (treq_tdata[55:52] != FTYPE_SWRITE)
                           && (treq_tdata[55:52] != FTYPE_NWRITE);
    always @(posedge log_clk) begin
        if (log_rst)
            err_drop_cnt <= 32'd0;
        else if (drop_pkt)
            err_drop_cnt <= err_drop_cnt + 32'd1;
    end
    assign  dbg_err_cnt = err_drop_cnt;

endmodule
