////////////////////////////////////////////////////////////////////////////////
// Module Name:  axis_skid_64
// Project:      VPX610 CNN ACC (XC7VX690T)
// Description:
//   AXI4-Stream 打拍(双寄存 skid buffer, 两拍深度), 64b 数据 + tlast 版,
//   结构与 rtl_690/srio/axis_skid.v 同款:
//     - 前向路径(tdata/tlast/tvalid)与反向路径(tready)均不形成纯组合直通,
//       用于切断 FIFO IP 边界/SRIO 核边界的长组合路径(时序处理专用);
//     - 双寄存结构保证全吞吐: 任意 tready 行为下不降级(每拍可过 1 拍)。
//   busy 供上层仿真排空判断(drain_done), 不参与功能逻辑。
// 读延迟契约: 输入到输出固定 1 拍(主寄存); skid 寄存仅在主寄存停满时
//   承接溢出, 不改变首拍延迟。
// 自检清单(规则 35):
//   [PASS] 纯 Verilog, 无 SV 关键字; 端口全显式 wire
//   [PASS] 组合=/时序<=分离; 无 #delay/initial/系统任务/function/for
//   [PASS] 控制信号(r_valid/skid_valid)复位, 数据通路(r_data/skid_data)不复位
//   [PASS] 无 latch/无组合环/无三态; 单一时钟域
//   [PASS] 无 RAM/ROM 内联存储; 无 / 与 % 运算
////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps

module axis_skid_64 (
    input  wire        clk,
    input  wire        rst_n,

    input  wire [63:0] s_tdata,
    input  wire        s_tlast,
    input  wire        s_tvalid,
    output wire        s_tready,

    output wire [63:0] m_tdata,
    output wire        m_tlast,
    output wire        m_tvalid,
    input  wire        m_tready,

    // 占用指示(r_valid || skid_valid): 供上层 drain_done, 功能逻辑不用
    output wire        busy
);

    // 主寄存(输出级)与溢出寄存: m 侧停等且主寄存满时, 输入进溢出寄存
    reg  [63:0] r_data;
    reg         r_last;
    reg         r_valid;
    reg  [63:0] skid_data;
    reg         skid_last;
    reg         skid_valid;

    // 溢出寄存空即可收; 主寄存满且 m 侧停等时, 新到数据只能进溢出寄存
    assign s_tready = !skid_valid;

    assign m_tdata  = r_data;
    assign m_tlast  = r_last;
    assign m_tvalid = r_valid;
    assign busy     = r_valid || skid_valid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_valid    <= 1'b0;
            skid_valid <= 1'b0;
        end else begin
            // 主寄存: m 侧可收或本就空时, 从溢出寄存(优先)或输入装填
            if (m_tready || !r_valid) begin
                r_valid    <= skid_valid ? 1'b1 : s_tvalid;
                skid_valid <= 1'b0;
            end else if (s_tvalid && !skid_valid) begin
                // 主寄存满且 m 侧停等: 输入入溢出寄存(随后 s_tready 拉低)
                skid_valid <= 1'b1;
            end
        end
    end

    always @(posedge clk) begin
        if (m_tready || !r_valid) begin
            r_data <= skid_valid ? skid_data : s_tdata;
            r_last <= skid_valid ? skid_last : s_tlast;
        end else if (s_tvalid && !skid_valid) begin
            skid_data <= s_tdata;
            skid_last <= s_tlast;
        end
    end

endmodule
