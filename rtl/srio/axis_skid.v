////////////////////////////////////////////////////////////////////////////////
// Module Name:  axis_skid
// Project:      VPX610 CNN ACC (XC7VX690T)
// Description:
//   AXI4-Stream 打拍(双寄存 skid buffer, 两拍深度):
//     - 前向路径(tdata/tvalid)与反向路径(tready)均不形成纯组合直通,
//       用于切断 IP 边界/跨模块长布线(时序处理专用, 协议 v5 通路重构)。
//     - 双寄存结构保证全吞吐: 任意 tready 行为下不降级(每拍可过 1 拍)。
//   仅搬运 tdata/tvalid/tready; 不带 tkeep/tlast(本链路契约: 整拍有效,
//   边界由计数自定界)。
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

module axis_skid #(
    parameter WIDTH = 64
)(
    input  wire             clk,
    input  wire             rst_n,

    input  wire [WIDTH-1:0] s_tdata,
    input  wire             s_tvalid,
    output wire             s_tready,

    output wire [WIDTH-1:0] m_tdata,
    output wire             m_tvalid,
    input  wire             m_tready,
    // 占用数(0..2): 供下游发包准入闸门统计在途拍数; 不需要的例化可悬空
    output wire [1:0]       occ
);

    // 主寄存(输出级)与溢出寄存: m 侧停等且主寄存满时, 输入进溢出寄存
    reg  [WIDTH-1:0] r_data;
    reg              r_valid;
    reg  [WIDTH-1:0] skid_data;
    reg              skid_valid;

    // 溢出寄存空即可收; 主寄存满且 m 侧停等时, 新到数据只能进溢出寄存
    assign s_tready = !skid_valid;

    assign m_tdata  = r_data;
    assign m_tvalid = r_valid;
    assign occ      = {1'b0, r_valid} + {1'b0, skid_valid};

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
                // 主寄存满且 m 侧停等: 输入入溢出寄存(仅一空位, 随后 s_tready 拉低)
                skid_valid <= 1'b1;
            end
        end
    end

    always @(posedge clk) begin
        if (m_tready || !r_valid)
            r_data <= skid_valid ? skid_data : s_tdata;
        else if (s_tvalid && !skid_valid)
            skid_data <= s_tdata;
    end

endmodule
