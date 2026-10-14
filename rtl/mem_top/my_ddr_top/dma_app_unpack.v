`timescale 1ns / 1ps
`include "all_defines.vh"

// =============================================================================
// dma_app_unpack —— app 拍（512b）→ N × AXI 拍（写方向）
//
// 位置: wrddr_fifo 与 beat_fifo 之间
//
//   wrddr_fifo(512, 异步) ──512──► [dma_app_unpack] ──AXI──► beat_fifo(AXI, 同步) ──► m_axi_w
//                                    ★ 插在这里
//
// 为什么插在这个位置（不是别处）:
//   ① beat_fifo 因此恒为 AXI 宽 → 「1 个字 = 1 个 AXI 拍」
//      → write_driver 的 credit 记账公式换位宽时零改动
//   ② unpack 输出的 N 拍有地方缓冲（beat_fifo）
//
// 行为:
//   锁存一个 app 拍 → 按 unpack_idx 逐拍吐出 BEATS_PER_APP 个 AXI 拍 → 吐完再收下一个
//   s_app_tready 只在「手上没有未吐完的拍」时拉高
//
// 首拍直通优化（沿用原版思路）:
//   !unpack_active_r 时 m_beat_tdata 直接取 s_app_tdata 的低位切片，
//   不必等锁存 → 省一拍延迟。锁存与 idx=1 在同一拍完成。
//
// 512 位宽下 BEATS_PER_APP = 1，走 generate 直通分支（无 mux、无寄存器）。
//
// 纯 Verilog-2001。
// =============================================================================
module dma_app_unpack #(
    parameter AXI_DATA_WIDTH = `DMA_AXI_SELECTED_DATA_WIDTH,
    parameter APP_DATA_WIDTH = `DMA_APP_DATA_WIDTH        // 512，加速器内部顶死
) (
    input  wire                       clk,
    input  wire                       rst_n,

    // ---------------- 来自 wrddr_fifo（app 侧，恒 512b）----------------
    input  wire [APP_DATA_WIDTH-1:0]  s_app_tdata,
    input  wire                       s_app_tvalid,
    output wire                       s_app_tready,

    // ---------------- 去往 beat_fifo（AXI 侧）----------------
    output wire [AXI_DATA_WIDTH-1:0]  m_beat_tdata,
    output wire                       m_beat_tvalid,
    input  wire                       m_beat_tready
);

//------------------------------------------------------------------
// 参数派生
//------------------------------------------------------------------
localparam BEATS_PER_APP = APP_DATA_WIDTH / AXI_DATA_WIDTH;   // 1/2/4/8
localparam IDX_W         = (BEATS_PER_APP <= 2) ? 1 :
                           (BEATS_PER_APP <= 4) ? 2 : 3;

// 位宽一致性检查：参数与宏不符 → 编译期报错
generate
    if (AXI_DATA_WIDTH != `DMA_AXI_SELECTED_DATA_WIDTH) begin : g_width_mismatch
        dma_app_unpack_width_must_match_mode_macro u_mismatch();
    end
endgenerate

//------------------------------------------------------------------
// 512 位宽：直通（app 拍 = AXI 拍）
//------------------------------------------------------------------
generate
    if (BEATS_PER_APP == 1) begin : g_passthrough

        assign m_beat_tdata  = s_app_tdata;
        assign m_beat_tvalid = s_app_tvalid;
        assign s_app_tready  = m_beat_tready;

    end
    else begin : g_unpack

        //--------------------------------------------------------------
        // 寄存器
        //--------------------------------------------------------------
        reg [APP_DATA_WIDTH-1:0] unpack_data_r;    // 锁存的 app 拍
        reg [IDX_W-1:0]          unpack_idx_r;     // 当前吐到第几拍
        reg                      unpack_active_r;  // 手上有没有未吐完的拍

        //--------------------------------------------------------------
        // 握手
        //--------------------------------------------------------------
        wire app_take  = s_app_tvalid && s_app_tready;
        wire beat_take = m_beat_tvalid && m_beat_tready;
        wire last_beat = (unpack_idx_r == (BEATS_PER_APP - 1));

        // ★ 首拍直通：没在吐的时候，输出直接取输入的低位切片
        assign m_beat_tdata  = unpack_active_r
                             ? unpack_data_r[unpack_idx_r * AXI_DATA_WIDTH +: AXI_DATA_WIDTH]
                             : s_app_tdata[AXI_DATA_WIDTH-1:0];
        assign m_beat_tvalid = unpack_active_r || s_app_tvalid;

        // ★ 只在「手上没有未吐完的拍」且下游能收时，才接受新 app 拍
        assign s_app_tready  = (!unpack_active_r) && m_beat_tready;

        //--------------------------------------------------------------
        // 锁存 app 拍
        //--------------------------------------------------------------
        always @(posedge clk or negedge rst_n) begin
            if (!rst_n) unpack_data_r <= {APP_DATA_WIDTH{1'b0}};
            else if (app_take) unpack_data_r <= s_app_tdata;
        end

        //--------------------------------------------------------------
        // active：首拍接走时置起，末拍吐完时落下
        //--------------------------------------------------------------
        always @(posedge clk or negedge rst_n) begin
            if (!rst_n) unpack_active_r <= 1'b0;
            else if (unpack_active_r && beat_take && last_beat)
                unpack_active_r <= 1'b0;
            else if ((!unpack_active_r) && app_take)
                unpack_active_r <= 1'b1;
        end

        //--------------------------------------------------------------
        // idx：首拍接走时置 1（首拍已直通），之后每吐一拍 +1
        //--------------------------------------------------------------
        always @(posedge clk or negedge rst_n) begin
            if (!rst_n) unpack_idx_r <= {IDX_W{1'b0}};
            else if (unpack_active_r && beat_take && last_beat)
                unpack_idx_r <= {IDX_W{1'b0}};
            else if (unpack_active_r && beat_take)
                unpack_idx_r <= unpack_idx_r + 1'b1;
            else if ((!unpack_active_r) && app_take)
                unpack_idx_r <= {{(IDX_W-1){1'b0}}, 1'b1};
        end

    end
endgenerate

endmodule
