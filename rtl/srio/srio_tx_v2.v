////////////////////////////////////////////////////////////////////////////////
// Module Name:  srio_tx_v2
// Project:      VPX610 CNN ACC (XC7VX690T)
// Description:
//   SRIO 发方向控制 v3.1(20260819 重构, FIXLIST 首轮 #1~#4 + 评审轮 #1~#5):
//     cnn_acc_top.m_axis_pso(512b+tlast, dma_clk)
//       -> axis_data_fifo_512x16_cdc(生成 IP, 仅跨钟; 计数端口不生成)
//       -> log_clk 域 512->64 拆拍(移位寄存器: 整字弹入, 每拍低端出片
//          右移 64; 字节倒序装载时纯走线; 无可变索引 mux)
//       -> axis_skid_64(寄存化 ready, 切断 IP tready 组合回灌)
//       -> axis_data_fifo_64x32_sync(生成 IP, last tag 随数据)
//       -> SWRITE 分包(满 256B=32 拍 / pso tlast 收尾包)
//       -> axis_skid_64(ireq 出口全寄存器化送 SRIO 核) + 每导出层一门铃
//   分包与控制(去闸门版):
//     - SWRITE 为 posted write 无响应超时, 包中气泡合法; 线上包长 =
//       实际发出拍数(包头无 size 字段), 故整包准入闸门删除;
//     - 发包计数器计满 32 拍收满包; 同步 FIFO 弹出带 last tag 的拍
//       (=某导出层最后 8B)无视计数器直接收尾包并发完成门铃;
//     - wr_addr 按实际发出拍数累加(收尾时刻定包长);
//     - cfg_layers(0x20): 门铃确认数够 cfg_layers 判轮末, wr_addr 自动回
//       基址保持 armed——同模型连续推理 DSP 零配置流量;
//     - commit 全量语义: 任意状态立即 cfg_base/wr_addr 同步切换 +
//       计数器清零 + FSM 回 IDLE。
//     - 20260819: 诊断门铃(WB_DBGDB, 0816 一次性调试资产)已删除;
//       init_calib_complete 输入口随之移除。
//     - 20260823 鲁棒门铃(doorbell-robust-redesign R1, acc v3.1 门铃丢失定案):
//       * info = 层索引({9'd0,2'b00,1'b0,layer[3:0]}, DOORBELL0 bit=层号;
//         保留位恒 0, C6678 对保留位置位回 ERROR——禁止夹带标志);
//       * WB_DB_WAIT 确认制: 门铃发出后等 iresp; DONE(FTYPE=D,E=0,tid 回显
//         匹配)才推进 layer_cnt/轮末回绕(从"发出"改到"确认"时刻);
//       * E=1 或 2^16 log_clk(约 420us)无响应 -> 回 WB_DB 重发(tid 递增,
//         单 outstanding, DSP 按位幂等记账); 重发 8 次耗尽 -> sticky db_fail
//         原地停机待 commit(轮卡死但可判责, 大声失败不静默);
//       * tid 匹配防"重发前的迟到 DONE"被错认为在途门铃的确认;
//         tid 不匹配的响应收下丢弃。
//   配置(log_clk 域 cfg 抽头写入, 协议 §3):
//     0x00 结果缓冲基址低 32b / 0x08 基址高 2b / 0x18 commit / 0x20 导出层数
//     (0x10 总量在 tx 侧已退役; 顶层仍为图案源锁存, 与 tx 无关)
//   输入契约(上游/DSP 必须保证, RTL 不做兜底):
//     - pso tlast 只出现在某导出层最后一个 512b 字上; 每层至少 1 字;
//     - cfg_layers >= 1, 且与本轮实际 tlast 次数一致;
//     - 配置写只发生在两轮之间或 commit 语义覆盖的强制同步点。
//   字节序: 数据拍做 64b 拍内 8 字节倒序(SRIO 线上大端), 包头/门铃不倒序。
// 自检清单(规则 35):
//   [PASS] 纯 Verilog; 端口全显式 wire; 组合=/时序<=分离
//   [PASS] 跨钟仅在生成 FIFO IP 边界; 本模块无第二钟域采样逻辑
//   [PASS] 控制信号(FSM/cfg/计数/拆拍/skid valid)复位, 数据通路(移位
//          寄存/skid 数据/FIFO 内容)不复位
//   [PASS] 状态机三段式前两段(状态寄存+次态组合), 输出控制分布于各自
//          时序块; localparam 编码 + default(规则 26/27)
//   [PASS] 无 latch/无组合环/无三态/#delay/initial/系统任务/for(仅 generate
//          字节倒序)/function / / 与 %; 无可变索引 mux
//   [PASS] FIFO 均为 Vivado 生成 axis_data_fifo IP(按 veo 例化), 无内联
//          存储, 无手写 XPM; 计数端口未生成
//   [PASS] 提供仿真排空指示 drain_done(规则 44)
////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps

module srio_tx_v2 #(
    parameter [15:0] DSP_ID  = 16'h00AA,
    parameter [15:0] FPGA_ID = 16'h00BB
)(
    input  wire         dma_clk,
    input  wire         dma_rst_n,
    input  wire         log_clk,
    input  wire         log_rst,

    // 加速器结果流(512b, dma_clk); tlast = 某导出层最后一字
    input  wire [511:0] s_axis_pso_tdata,
    input  wire         s_axis_pso_tvalid,
    input  wire         s_axis_pso_tlast,
    output wire         s_axis_pso_tready,

    // 配置抽头(log_clk, 来自 srio2axis; 写使能已展宽, 同值重复写无害)
    input  wire         cfg_wr_en,
    input  wire [7:0]   cfg_wr_addr,
    input  wire [31:0]  cfg_wr_data,

    // 链路状态(log_clk, 来自 SRIO 核)
    input  wire         link_up,

    // Initiator 请求口(log_clk, 驱动 srio2axis.s_axis_ireq)
    output wire [63:0]  ireq_tdata,
    output wire [7:0]   ireq_tkeep,
    output wire         ireq_tlast,
    output wire         ireq_tvalid,
    output wire [31:0]  ireq_tuser,
    input  wire         ireq_tready,

    // Initiator 响应口(log_clk, 来自 srio2axis.m_axis_iresp):
    //   ireq 通道只有 SWRITE(posted, 无响应)与门铃两类包, 故 iresp 上
    //   只会出现门铃响应; 仅在 WB_DB_WAIT 态拉 tready(20260823 确认制)
    input  wire         iresp_tvalid,
    output wire         iresp_tready,
    input  wire [63:0]  iresp_tdata,
    input  wire         iresp_tlast,

    // 仿真排空指示(规则 44)
    output wire         drain_done
);

    // ==================================================================
    // 512b x16 异步 AXIS FIFO(dma_clk -> log_clk): Vivado 生成
    //   axis_data_fifo IP, 例化端口严格按 axis_data_fifo_512x16_cdc.veo。
    //   仅做跨钟; 计数端口未生成(异步 FIFO 自带计数不可做控制, 铁律)。
    //   resetn 用 dma 域复位: IP 内部完成双域复位同步(同 v2 用法)。
    // ==================================================================
    (* mark_debug = "true" *) wire         af_tvalid;
    (* mark_debug = "true" *) wire         af_tready;
    (* mark_debug = "true" *) wire         af_tlast;
    wire [511:0] af_tdata;

    axis_data_fifo_512x16_cdc u_afifo (
        .s_axis_aclk    (dma_clk),
        .s_axis_aresetn (dma_rst_n),
        .s_axis_tvalid  (s_axis_pso_tvalid),
        .s_axis_tdata   (s_axis_pso_tdata),
        .s_axis_tlast   (s_axis_pso_tlast),
        .s_axis_tready  (s_axis_pso_tready),
        .m_axis_aclk    (log_clk),
        .m_axis_tvalid  (af_tvalid),
        .m_axis_tdata   (af_tdata),
        .m_axis_tlast   (af_tlast),
        .m_axis_tready  (af_tready)
    );

    // ==================================================================
    // 512->64 拆拍(移位寄存器形态, 评审 #1):
    //   上游字空档时一次弹入移位寄存(字节倒序装载纯走线, 零逻辑级),
    //   之后每拍低端出 64b 片、整体右移 64; 字级 tlast 锁存, 末片带出。
    //   字间留 1 拍装载空档(速率由 SRIO 线决定, 弹性 FIFO 遮盖, 无影响)。
    // ==================================================================
    (* mark_debug = "true" *) reg         sh_valid;
    (* mark_debug = "true" *) reg  [2:0]  slice_cnt;
    reg         sh_tag;
    reg  [511:0] sh_reg;

    // 装载: 各 64b lane 内 8 字节倒序(lane 顺序不变), 纯走线
    wire [511:0] sh_load;
    genvar      b;
    generate
        for (b = 0; b < 8; b = b + 1) begin : gen_swap
            assign sh_load[b*64 +  0 +: 8] = af_tdata[b*64 + 56 +: 8];
            assign sh_load[b*64 +  8 +: 8] = af_tdata[b*64 + 48 +: 8];
            assign sh_load[b*64 + 16 +: 8] = af_tdata[b*64 + 40 +: 8];
            assign sh_load[b*64 + 24 +: 8] = af_tdata[b*64 + 32 +: 8];
            assign sh_load[b*64 + 32 +: 8] = af_tdata[b*64 + 24 +: 8];
            assign sh_load[b*64 + 40 +: 8] = af_tdata[b*64 + 16 +: 8];
            assign sh_load[b*64 + 48 +: 8] = af_tdata[b*64 +  8 +: 8];
            assign sh_load[b*64 + 56 +: 8] = af_tdata[b*64 +  0 +: 8];
        end
    endgenerate

    // 弹字条件: 移位寄存空档且上游有字
    assign af_tready = !sh_valid;

    // 拆拍数据通路(规则 10: 数据通路寄存器不接复位)
    wire        sl_tready;
    wire        sl_fire = sh_valid && sl_tready;

    always @(posedge log_clk) begin
        if (!sh_valid) begin
            if (af_tvalid)
                sh_reg <= sh_load;
        end else if (sl_fire) begin
            sh_reg <= {64'd0, sh_reg[511:64]};
        end
    end

    // 拆拍控制通路
    always @(posedge log_clk or posedge log_rst) begin
        if (log_rst) begin
            sh_valid  <= 1'b0;
            slice_cnt <= 3'd0;
            sh_tag    <= 1'b0;
        end else begin
            if (!sh_valid) begin
                if (af_tvalid) begin
                    sh_valid  <= 1'b1;
                    slice_cnt <= 3'd0;
                    sh_tag    <= af_tlast;
                end
            end else if (sl_fire) begin
                slice_cnt <= slice_cnt + 3'd1;   // 3bit 自然回卷 7->0
                if (slice_cnt == 3'd7)
                    sh_valid <= 1'b0;
            end
        end
    end

    (* mark_debug = "true" *) wire        sl_tvalid = sh_valid;
    (* mark_debug = "true" *) wire        sl_tlast  = sh_tag && (slice_cnt == 3'd7);
    wire [63:0] sl_tdata  = sh_reg[63:0];

    // ==================================================================
    // skid(评审 #2): 拆拍器 -> 同步 FIFO; IP 的 s_axis_tready 只进 skid,
    //   拆拍器只看 skid 的寄存化 ready
    // ==================================================================
    wire [63:0] sf_s_tdata;
    wire        sf_s_tlast;
    wire        sf_s_tvalid;
    wire        sf_s_tready;
    wire        skid_sf_busy;

    axis_skid_64 u_skid_sf (
        .clk      (log_clk),
        .rst_n    (~log_rst),
        .s_tdata  (sl_tdata),
        .s_tlast  (sl_tlast),
        .s_tvalid (sl_tvalid),
        .s_tready (sl_tready),
        .m_tdata  (sf_s_tdata),
        .m_tlast  (sf_s_tlast),
        .m_tvalid (sf_s_tvalid),
        .m_tready (sf_s_tready),
        .busy     (skid_sf_busy)
    );

    // ==================================================================
    // 64b x32 同步 AXIS FIFO(log_clk): 纯弹性缓冲, last tag 随数据;
    //   计数端口未生成(去闸门后无需占用计数)
    // ==================================================================
    (* mark_debug = "true" *) wire        sf_tvalid;
    (* mark_debug = "true" *) wire        sf_tlast;
    (* mark_debug = "true" *) wire        sf_rd_en;
    wire [63:0] sf_tdata;

    axis_data_fifo_64x32_sync u_sfifo (
        .s_axis_aclk    (log_clk),
        .s_axis_aresetn (~log_rst),
        .s_axis_tvalid  (sf_s_tvalid),
        .s_axis_tdata   (sf_s_tdata),
        .s_axis_tlast   (sf_s_tlast),
        .s_axis_tready  (sf_s_tready),
        .m_axis_tvalid  (sf_tvalid),
        .m_axis_tdata   (sf_tdata),
        .m_axis_tlast   (sf_tlast),
        .m_axis_tready  (sf_rd_en)
    );

    // ==================================================================
    // 配置寄存器(log_clk): 0x00 基址低 / 0x08 基址高 / 0x18 commit / 0x20 层数
    //   case 译码(评审 #3); commit 取展宽写使能的首拍(沿检测), 独立赋值
    // ==================================================================
    reg  [31:0] cfg_base_lo;
    reg  [1:0]  cfg_base_hi;
    reg  [15:0] cfg_layers_m1;   // cfg_layers - 1(写入时预减, 省运行期减法)
    reg         cfg_commit_r;

    always @(posedge log_clk or posedge log_rst) begin
        if (log_rst) begin
            cfg_base_lo   <= 32'd0;
            cfg_base_hi   <= 2'd0;
            cfg_layers_m1 <= 16'd0;
            cfg_commit_r  <= 1'b0;
        end else begin
            if (cfg_wr_en) begin
                case (cfg_wr_addr)
                    8'h00:   cfg_base_lo   <= cfg_wr_data;
                    8'h08:   cfg_base_hi   <= cfg_wr_data[1:0];
                    8'h20:   cfg_layers_m1 <= cfg_wr_data[15:0] - 16'd1;
                    default: begin end
                endcase
            end
            cfg_commit_r <= cfg_wr_en && (cfg_wr_addr == 8'h18);
        end
    end

    wire commit_pulse = cfg_wr_en && (cfg_wr_addr == 8'h18) && !cfg_commit_r;

    // ==================================================================
    // 分包 FSM(三段式前两段, 评审 #4; 无闸门):
    //   IDLE: armed 且链路在且同步 FIFO 有数(>=1 拍)即起包头;
    //         包中允许气泡(SWRITE posted write 无超时), 数据随来随发。
    //   DATA: 计数到 32 拍收满包; 弹出拍带 last tag(层尾)直接收尾包。
    //   DB:   每收尾包发一个完成门铃(info=层索引), 发出后进 DB_WAIT。
    //   DB_WAIT(20260823 确认制): 等 iresp; DONE(tid 匹配)确认本层,
    //         确认数够 cfg_layers 判轮末, wr_addr 回基址(免 commit);
    //         E=1/超时(2^16 log_clk≈420us)回 DB 重发, 8 次耗尽 db_fail
    //         原地停机(仅 commit 可解, 大声失败)。
    //   (20260819: 诊断门铃/WB_DBGDB 为一次性调试资产, 已按用户要求删除;
    //    commit 直接回 IDLE 重新武装)
    // ==================================================================
    localparam [2:0] WB_IDLE   = 3'd0;
    localparam [2:0] WB_HDR    = 3'd1;
    localparam [2:0] WB_DATA   = 3'd2;
    localparam [2:0] WB_DB     = 3'd3;
    localparam [2:0] WB_DB_WAIT = 3'd4;

    localparam [3:0] FTYPE_SWRITE = 4'h6;
    localparam [3:0] FTYPE_DOORB  = 4'hA;

    (* mark_debug = "true" *) reg  [2:0]  wb_state;
    reg  [2:0]  wb_state_n;

    (* mark_debug = "true" *) reg         armed;
    (* mark_debug = "true" *) reg  [31:0] wr_addr;
    reg  [1:0]  wr_hi;
    (* mark_debug = "true" *) reg  [5:0]  beat_cnt;
    (* mark_debug = "true" *) reg  [15:0] layer_cnt;
    reg  [7:0]  tid_cnt;
    // 门铃确认制控制(20260823): 等待计时/重发计数/失败停机
    reg  [15:0] db_timer;
    (* mark_debug = "true" *) reg  [3:0]  db_retry_cnt;
    (* mark_debug = "true" *) reg         db_fail;

    // FSM 出端(进 ireq skid 内口)握手
    wire        oq_tready;
    wire        oq_tvalid;
    wire        oq_accept    = oq_tvalid && oq_tready;
    wire        pkt_full_end = (beat_cnt == 6'd31);
    wire        pkt_tag_end  = sf_tlast;
    wire        pkt_end      = pkt_full_end || pkt_tag_end;
    // 本包实发字节数: (beat_cnt+1)*8 —— 包长收尾才定, 地址按实发累加
    wire [8:0]  pkt_bytes    = {beat_cnt + 6'd1, 3'b000};

    // 数据拍取出: WB_DATA 态被 ireq skid 接收即取
    assign sf_rd_en = (wb_state == WB_DATA) && oq_accept;

    // ==================================================================
    // iresp 门铃响应译码(20260823 确认制; PG007 HELLO RESP 格式):
    //   [63:56]=target_info(回显请求 tid) [55:52]=FTYPE=0xD
    //   [51:48]=TTYPE(门铃响应=NoData=0)   [35]=E(0=DONE, 其余状态=1)
    //   只在 WB_DB_WAIT 拉 tready; tid 不匹配=重发前的迟到响应, 收下丢弃
    // ==================================================================
    assign iresp_tready = (wb_state == WB_DB_WAIT);
    wire        iresp_accept = iresp_tvalid && iresp_tready;
    wire [7:0]  db_tid_out   = tid_cnt - 8'd1;   // 在途门铃 tid(最近一次发出)
    wire        iresp_is_db  = (iresp_tdata[55:52] == 4'hD)
                               && (iresp_tdata[63:56] == db_tid_out);
    wire        iresp_done   = iresp_accept && iresp_is_db && !iresp_tdata[35];
    wire        iresp_err    = iresp_accept && iresp_is_db &&  iresp_tdata[35];
    wire        db_timeout   = (wb_state == WB_DB_WAIT) && (db_timer == 16'hFFFF);
    wire        db_giveup    = (iresp_err || db_timeout) && (db_retry_cnt == 4'd8);

    // ---- 第一段: 状态寄存器(commit 优先级最高) ----
    always @(posedge log_clk or posedge log_rst) begin
        if (log_rst)
            wb_state <= WB_IDLE;
        else if (commit_pulse)
            wb_state <= WB_IDLE;
        else
            wb_state <= wb_state_n;
    end

    // ---- 第二段: 次态组合(全路径赋值 + default) ----
    always @(*) begin
        case (wb_state)
            WB_IDLE: begin
                if (armed && link_up && sf_tvalid)
                    wb_state_n = WB_HDR;
                else
                    wb_state_n = WB_IDLE;
            end
            WB_HDR: begin
                if (oq_accept)
                    wb_state_n = WB_DATA;
                else
                    wb_state_n = WB_HDR;
            end
            WB_DATA: begin
                if (oq_accept && pkt_end)
                    wb_state_n = pkt_tag_end ? WB_DB : WB_IDLE;
                else
                    wb_state_n = WB_DATA;
            end
            WB_DB: begin
                if (oq_accept)
                    wb_state_n = WB_DB_WAIT;
                else
                    wb_state_n = WB_DB;
            end
            WB_DB_WAIT: begin
                if (db_fail)
                    wb_state_n = WB_DB_WAIT;      // 重发耗尽停机, 仅 commit 可解
                else if (iresp_done)
                    wb_state_n = WB_IDLE;         // DONE: 确认本层
                else if (db_giveup)
                    wb_state_n = WB_DB_WAIT;      // 第 8 次重发也失败: 置 db_fail 原地停
                else if (iresp_err || db_timeout)
                    wb_state_n = WB_DB;           // 重发(tid 递增, 单 outstanding)
                else
                    wb_state_n = WB_DB_WAIT;
            end
            default: begin
                wb_state_n = WB_IDLE;
            end
        endcase
    end

    // ---- 输出控制分布: 各自时序块, commit 全量语义逐块体现 ----

    // 发包拍计数(实发拍数; 收包清零)
    always @(posedge log_clk or posedge log_rst) begin
        if (log_rst)
            beat_cnt <= 6'd0;
        else if (commit_pulse)
            beat_cnt <= 6'd0;
        else if ((wb_state == WB_DATA) && oq_accept)
            beat_cnt <= pkt_end ? 6'd0 : beat_cnt + 6'd1;
    end

    // 工作地址: commit/轮末回基址, 收包按实发字节数累加
    always @(posedge log_clk or posedge log_rst) begin
        if (log_rst) begin
            wr_addr <= 32'd0;
            wr_hi   <= 2'd0;
        end else if (commit_pulse) begin
            wr_addr <= cfg_base_lo;
            wr_hi   <= cfg_base_hi;
        end else if ((wb_state == WB_DATA) && oq_accept && pkt_end) begin
            wr_addr <= wr_addr + pkt_bytes;
        end else if ((wb_state == WB_DB_WAIT) && iresp_done
                         && (layer_cnt == cfg_layers_m1)) begin
            // 末层门铃确认(DONE)才回基址——20260823: 从"发出"改到"确认"时刻
            wr_addr <= cfg_base_lo;
            wr_hi   <= cfg_base_hi;
        end
    end

    // 层计数(门铃确认数): 够 cfg_layers 判轮末并清零
    always @(posedge log_clk or posedge log_rst) begin
        if (log_rst)
            layer_cnt <= 16'd0;
        else if (commit_pulse)
            layer_cnt <= 16'd0;
        else if ((wb_state == WB_DB_WAIT) && iresp_done)
            layer_cnt <= (layer_cnt == cfg_layers_m1) ? 16'd0
                                                      : layer_cnt + 16'd1;
    end

    // 武装位(commit 置位, 保持到下次复位)
    always @(posedge log_clk or posedge log_rst) begin
        if (log_rst)
            armed <= 1'b0;
        else if (commit_pulse)
            armed <= 1'b1;
    end

    // 门铃 tid: 逐门铃递增(含重发, 保持单 outstanding, PG007 TID 规则),
    // commit 不清(与 v2 一致)
    always @(posedge log_clk or posedge log_rst) begin
        if (log_rst)
            tid_cnt <= 8'd0;
        else if ((wb_state == WB_DB) && oq_accept)
            tid_cnt <= tid_cnt + 8'd1;
    end

    // 门铃响应等待计时: WB_DB_WAIT 内逐拍计, 2^16 log_clk(156.25M 下
    // ≈420us, 响应往返约 1us 量级)无响应判超时; 离态清零
    always @(posedge log_clk or posedge log_rst) begin
        if (log_rst)
            db_timer <= 16'd0;
        else if ((wb_state == WB_DB_WAIT) && !db_fail)
            db_timer <= db_timer + 16'd1;
        else
            db_timer <= 16'd0;
    end

    // 重发计数与失败停机: 确认(或 commit)清零; E=1/超时累加,
    // 第 8 次重发仍失败置 sticky db_fail(轮卡死但可判责, 大声失败)
    always @(posedge log_clk or posedge log_rst) begin
        if (log_rst) begin
            db_retry_cnt <= 4'd0;
            db_fail      <= 1'b0;
        end else if (commit_pulse) begin
            db_retry_cnt <= 4'd0;
            db_fail      <= 1'b0;
        end else if ((wb_state == WB_DB_WAIT) && iresp_done) begin
            db_retry_cnt <= 4'd0;
        end else if ((wb_state == WB_DB_WAIT) && !db_fail
                         && (iresp_err || db_timeout)) begin
            if (db_retry_cnt == 4'd8)
                db_fail <= 1'b1;
            else
                db_retry_cnt <= db_retry_cnt + 4'd1;
        end
    end

    wire [63:0] swrite_hdr = {8'h00,              // tid(SWRITE 无响应, 保留)
                              FTYPE_SWRITE,
                              4'b0000,            // R
                              1'b0,               // R
                              2'b00,              // prio
                              1'b0,               // crf
                              8'h00,              // size 字段对 SWRITE 保留
                              1'b0,               // R
                              1'b0,               // R
                              wr_hi,              // 34bit 地址高 2 位
                              wr_addr};           // 34bit 地址低 32 位

    wire [63:0] doorbell_pkt = {tid_cnt,
                                FTYPE_DOORB,
                                4'b0000,
                                1'b0,
                                2'b00,
                                1'b0,
                                8'h00,
                                1'b0,
                                1'b0,
                                2'b00,
                                // info(20260823): DOORBELL0 层索引位图,
                                //   info[6:5]=寄存器号 0, info[3:0]=层号;
                                //   保留位恒 0(C6678 对保留位置位回 ERROR)
                                {9'd0, 2'b00, 1'b0, layer_cnt[3:0]},
                                16'h0000};

    // ==================================================================
    // ireq 出口 skid(评审 #5): FSM/FIFO 出端经 skid 全寄存器化送 SRIO 核
    // ==================================================================
    wire [63:0] oq_tdata;
    wire        oq_tlast;

    assign oq_tdata  = (wb_state == WB_HDR) ? swrite_hdr   :
                       (wb_state == WB_DB)  ? doorbell_pkt :
                                                sf_tdata;
    assign oq_tvalid = (wb_state == WB_HDR) || (wb_state == WB_DB)
                           ? 1'b1
                           : (wb_state == WB_DATA) && sf_tvalid;
    assign oq_tlast  = (wb_state == WB_DB)  ? 1'b1 :
                       (wb_state == WB_DATA) ? pkt_end :
                                               1'b0;

    wire        skid_ireq_busy;

    axis_skid_64 u_skid_ireq (
        .clk      (log_clk),
        .rst_n    (~log_rst),
        .s_tdata  (oq_tdata),
        .s_tlast  (oq_tlast),
        .s_tvalid (oq_tvalid),
        .s_tready (oq_tready),
        .m_tdata  (ireq_tdata),
        .m_tlast  (ireq_tlast),
        .m_tvalid (ireq_tvalid),
        .m_tready (ireq_tready),
        .busy     (skid_ireq_busy)
    );

    // tkeep/tuser 为常量(无逻辑路径), 直接驱动
    assign ireq_tkeep  = 8'hFF;
    assign ireq_tuser  = {FPGA_ID, DSP_ID};

    // ==================================================================
    // 仿真排空: FSM 空闲且 FIFO/skid 均无数据在途
    // ==================================================================
    assign drain_done = (wb_state == WB_IDLE) && !af_tvalid && !sf_tvalid
                        && !skid_sf_busy && !skid_ireq_busy;

endmodule

////////////////////////////////////////////////////////////////////////////////
// RTL SELF-CHECK (20260823 robust doorbell / WB_DB_WAIT round, loop 版):
//   [PASS] 与 acc 版(rtl_690/srio/srio_tx_v2.v)同轮修改逐段一致, 仅保留本线
//          "无 dbg 端口"形态; FSM/门铃确认制/info 层索引逻辑完全相同。
//   [PASS] Pure Verilog; 组合=时序<=分离; FSM 三段式前两段 + localparam +
//          default; 新控制寄存器(db_timer/db_retry_cnt/db_fail)异步复位。
//   [PASS] 无 latch/组合环/三态/#delay/initial/系统任务/for(genvar 字节倒序
//          除外)/function/'/'/'%'; 稳态吞吐不变(确认等待在数据通路之外)。
//   [PASS] iresp_tready 仅 WB_DB_WAIT 拉高; tid 不匹配响应收下丢弃。
//   [WARN] iresp_tkeep/tuser 不使用(译码只需 tdata), 顶层保持悬空。
//   [FAIL] none
////////////////////////////////////////////////////////////////////////////////
