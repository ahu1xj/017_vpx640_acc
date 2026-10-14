`timescale 1ns / 1ps
`include "all_defines.vh"

// =============================================================================
// dma_app_pack —— N × AXI 拍 → app 拍（512b）（读方向，dma_app_unpack 的镜像）
//
// 位置: read_driver 与 rdddr_fifo 之间
//
//   m_axi_r ──► read_driver ──AXI──► [dma_app_pack] ──512──► rdddr_fifo(异步 CDC)
//                                      ★ 在这里
//
// 行为: 收满 BEATS_PER_APP 个 AXI 拍 → 拼成一个 512b app 拍 → 置 valid
//       输出被占住时反压上游（s_beat_tready = !valid || 被接走）
//
// 512 位宽下 BEATS_PER_APP = 1，走 generate 直通分支。
//
// 纯 Verilog-2001。
// =============================================================================
module dma_app_pack #(
    parameter AXI_DATA_WIDTH = `DMA_AXI_SELECTED_DATA_WIDTH,
    parameter APP_DATA_WIDTH = `DMA_APP_DATA_WIDTH        // 512，加速器内部顶死
) (
    input  wire                       clk,
    input  wire                       rst_n,

    // ---------------- 来自 read_driver（AXI 侧）----------------
    input  wire [AXI_DATA_WIDTH-1:0]  s_beat_tdata,
    input  wire                       s_beat_tvalid,
    output wire                       s_beat_tready,

    // ---------------- 去往 rdddr_fifo（app 侧，恒 512b）----------------
    output wire [APP_DATA_WIDTH-1:0]  m_app_tdata,
    output wire                       m_app_tvalid,
    input  wire                       m_app_tready
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
        dma_app_pack_width_must_match_mode_macro u_mismatch();
    end
endgenerate

//------------------------------------------------------------------
// 512 位宽：直通（AXI 拍 = app 拍）
//------------------------------------------------------------------
generate
    if (BEATS_PER_APP == 1) begin : g_passthrough

        assign m_app_tdata   = s_beat_tdata;
        assign m_app_tvalid  = s_beat_tvalid;
        assign s_beat_tready = m_app_tready;

    end
    else begin : g_pack

        //--------------------------------------------------------------
        // 寄存器
        //--------------------------------------------------------------
        reg [APP_DATA_WIDTH-1:0] pack_data_r;    // 组装中的 app 拍
        reg [IDX_W-1:0]          pack_idx_r;     // 当前收到第几拍
        reg                      pack_valid_r;   // 组装完成、待下游接走
        reg [APP_DATA_WIDTH-1:0] pack_next_data;

        //--------------------------------------------------------------
        // 握手
        //--------------------------------------------------------------
        wire beat_take = s_beat_tvalid && s_beat_tready;
        wire app_take  = m_app_tvalid && m_app_tready;
        wire last_beat = (pack_idx_r == (BEATS_PER_APP - 1));

        //--------------------------------------------------------------
        // 组装: 把当前 AXI 拍塞进对应字节槽
        //--------------------------------------------------------------
        always @(*) begin
            pack_next_data = pack_data_r;
            pack_next_data[pack_idx_r * AXI_DATA_WIDTH +: AXI_DATA_WIDTH] = s_beat_tdata;
        end

        always @(posedge clk or negedge rst_n) begin
            if (!rst_n) pack_data_r <= {APP_DATA_WIDTH{1'b0}};
            else if (beat_take) pack_data_r <= pack_next_data;
        end

        //--------------------------------------------------------------
        // idx：收满一圈回 0
        //--------------------------------------------------------------
        always @(posedge clk or negedge rst_n) begin
            if (!rst_n) pack_idx_r <= {IDX_W{1'b0}};
            else if (beat_take && last_beat) pack_idx_r <= {IDX_W{1'b0}};
            else if (beat_take)              pack_idx_r <= pack_idx_r + 1'b1;
        end

        //--------------------------------------------------------------
        // valid：末拍收满时置起，被下游接走时落下
        //--------------------------------------------------------------
        always @(posedge clk or negedge rst_n) begin
            if (!rst_n) pack_valid_r <= 1'b0;
            else if (beat_take && last_beat) pack_valid_r <= 1'b1;
            else if (app_take)               pack_valid_r <= 1'b0;
        end

        assign m_app_tdata   = pack_data_r;
        assign m_app_tvalid  = pack_valid_r;
        // ★ 输出被占住就反压上游（组装期间始终能收）
        assign s_beat_tready = (!pack_valid_r) || app_take;

    end
endgenerate

endmodule
