////////////////////////////////////////////////////////////////////////////////
// Module Name:  srio_rx_path
// Project:      VPX610 CNN ACC (XC7VX690T)
// Description:
//   SRIO 收方向数据通路(协议 v5 通路重构, 替代探针 512b BRAM FIFO + log_clk
//   域拼装):
//     srio2axis 路由出的 64b 净荷(log_clk, 字节已倒序)
//       -> skid 打拍(log_clk) -> 64x16 异步 FIFO(Vivado 生成 IP, 跨钟)
//       -> skid 打拍(dma_clk) -> 64->512 拼装(dma_clk)
//       -> cnn_acc_top.s_axis_ps(512b, 无 tkeep/tlast, 整拍有效)
//   契约: 输入流为 64B 整数倍(协议 §2/§4), 故拼装无尾片冲刷需求;
//   边界由加速器 FSM 按 len 自定界。
//   读延迟契约: skid 各 1 拍; FIFO 写->读可见 ~2 读钟; 拼装满 8 拍出字。
// 自检清单(规则 35):
//   [PASS] 纯 Verilog; 端口全显式 wire; 组合=/时序<=分离
//   [PASS] 跨钟仅在生成 FIFO IP(fifo_64x16_cdc)边界(规则 17/18/21);
//          本模块无第二钟域采样逻辑(仅例化单域叶模块与 CDC 模块)
//   [PASS] 控制信号(asm_cnt/asm_valid)复位, 数据通路(asm_data)不复位
//   [PASS] 无 latch/无组合环/无三态/#delay/initial/系统任务/function/for
//   [PASS] FIFO 为 Vivado 生成 IP 独立模块(按 veo 例化), 无内联存储
//   [PASS] 提供仿真排空指示 drain_done(规则 44)
////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps

module srio_rx_path (
    input  wire         log_clk,
    input  wire         log_rst,        // 高有效(SRIO 核域)
    input  wire         dma_clk,
    input  wire         dma_rst_n,      // 低有效(加速器域)

    // 来自 srio2axis 的 64b 净荷流(log_clk, 已按 ftype 路由)
    input  wire [63:0]  rx64_tdata,
    input  wire         rx64_tvalid,
    output wire         rx64_tready,

    // 512b 输出到加速器(dma_clk)
    output wire [511:0] m_axis_ps_tdata,
    output wire         m_axis_ps_tvalid,
    input  wire         m_axis_ps_tready,

    // 仿真排空指示(规则 44)
    output wire         drain_done
);

    // ==================================================================
    // skid 打拍 1(log_clk): 切断核侧布线
    // ==================================================================
    wire [63:0] sk1_m_tdata;
    wire        sk1_m_tvalid;
    wire        sk1_m_tready;

    axis_skid #(.WIDTH(64)) u_skid_log (
        .clk     (log_clk),
        .rst_n   (~log_rst),
        .s_tdata (rx64_tdata),
        .s_tvalid(rx64_tvalid),
        .s_tready(rx64_tready),
        .m_tdata (sk1_m_tdata),
        .m_tvalid(sk1_m_tvalid),
        .m_tready(sk1_m_tready)
    );

    // ==================================================================
    // 64x16 异步 FIFO(log_clk -> dma_clk): Vivado 生成 IP(fifo_generator,
    // 分布式 RAM), 例化端口严格按 fifo_64x16_cdc.veo(20260818 铁律);
    // 仅跨钟弹性, 不深缓冲(256->16, BRAM 布局布线时序教训)
    // ==================================================================
    wire        af_full;
    wire [63:0] af_dout;
    wire        af_empty;

    // 注: 本 IP 无 wr_rst_busy/rd_rst_busy 端口; rst 释放后首批数据到达
    // 远在数百拍之后(协议保证), 无需忙等门控。
    assign sk1_m_tready = !af_full;

    fifo_64x16_cdc u_afifo (
        .rst           (log_rst),
        .wr_clk        (log_clk),
        .din           (sk1_m_tdata),
        .wr_en         (sk1_m_tvalid && sk1_m_tready),
        .full          (af_full),
        .wr_data_count (),
        .rd_clk        (dma_clk),
        .rd_en         (af_rd_en),
        .dout          (af_dout),
        .empty         (af_empty),
        .rd_data_count ()
    );

    // ==================================================================
    // skid 打拍 2(dma_clk): FIFO 出口不直接进拼装逻辑
    // ==================================================================
    wire [63:0] sk2_m_tdata;
    wire        sk2_m_tvalid;
    wire        sk2_m_tready;
    wire        sk2_s_tready_w;
    wire        af_rd_en;

    assign af_rd_en = !af_empty && sk2_s_tready_w;

    axis_skid #(.WIDTH(64)) u_skid_dma (
        .clk     (dma_clk),
        .rst_n   (dma_rst_n),
        .s_tdata (af_dout),
        .s_tvalid(!af_empty),
        .s_tready(sk2_s_tready_w),
        .m_tdata (sk2_m_tdata),
        .m_tvalid(sk2_m_tvalid),
        .m_tready(sk2_m_tready)
    );

    // ==================================================================
    // 64 -> 512 拼装(dma_clk): 集满 8 拍出一个 512b 字
    //   输入流为 64B 整数倍(契约), 不存在尾片
    // ==================================================================
    reg  [2:0]   asm_cnt;
    reg  [511:0] asm_data;
    reg          asm_valid;

    wire         asm_in_fire = sk2_m_tvalid && sk2_m_tready;
    wire         asm_out_fire = m_axis_ps_tvalid && m_axis_ps_tready;

    assign sk2_m_tready = !asm_valid;

    always @(posedge dma_clk or negedge dma_rst_n) begin
        if (!dma_rst_n) begin
            asm_cnt   <= 3'd0;
            asm_valid <= 1'b0;
        end else if (asm_in_fire) begin
            if (asm_cnt == 3'd7) begin
                asm_cnt   <= 3'd0;
                asm_valid <= 1'b1;
            end else begin
                asm_cnt   <= asm_cnt + 3'd1;
            end
        end else if (asm_out_fire) begin
            asm_valid <= 1'b0;
        end
    end

    always @(posedge dma_clk) begin
        if (asm_in_fire)
            asm_data[{asm_cnt, 6'd0} +: 64] <= sk2_m_tdata;
    end

    assign m_axis_ps_tdata  = asm_data;
    assign m_axis_ps_tvalid = asm_valid;

    // ==================================================================
    // 仿真排空: 各段均无数据在途
    // ==================================================================
    assign drain_done = !sk1_m_tvalid && af_empty && !sk2_m_tvalid && !asm_valid;

endmodule
