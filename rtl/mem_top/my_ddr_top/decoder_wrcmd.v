`timescale 1ns / 1ps
`include "all_defines.vh"

module decoder_wrcmd #(
    parameter AXI_DATA_WIDTH  = `DMA_AXI_SELECTED_DATA_WIDTH,  // dma 位宽
    parameter MAX_BURST_BEATS = `DMA_AXI_MAX_BURST_BEATS,      // 256: awlen 8 位 → 最多 256 拍
    parameter MEM_BYTES       = 32'h4000_0000                  // DDR 最大容量
) (
    input  wire        clk,
    input  wire        rst_n,

    input  wire [63:0] cmd_data,       // {addr[31:0], bytes[31:0]}
    input  wire        cmd_valid,      // ← FIFO
    input  wire        cmd_tready,     // ← driver 反压

    output wire        s_axis_tready,  // → FIFO
    output reg  [31:0] cmd_address,
    output reg  [7:0]  burst_len,      // = awlen（拍数-1）
    output reg         cmd_tvalid,
    output reg  [3:0]  cmd_err
);

//------------------------------------------------------------------
// 参数派生（除 4096 / 8 位外，一个都别硬编码）
//------------------------------------------------------------------
localparam AXI_BYTES       = AXI_DATA_WIDTH / 8;
localparam SHIFT           = (AXI_DATA_WIDTH == 512) ? 6 :
                             (AXI_DATA_WIDTH == 256) ? 5 :
                             (AXI_DATA_WIDTH == 128) ? 4 : 3;
localparam MAX_BURST_BYTES = MAX_BURST_BEATS * AXI_BYTES;      // 最大突发字节数

// 位宽一致性检查：参数与宏不符 → 编译期报错
generate
    if (AXI_DATA_WIDTH != `DMA_AXI_SELECTED_DATA_WIDTH) begin : g_width_mismatch
        decoder_width_must_match_mode_macro u_mismatch();
    end
endgenerate

//------------------------------------------------------------------
// 命令解析
//------------------------------------------------------------------
wire [31:0] cmd_data_addr_ns0 = cmd_data[63:32];
wire [31:0] cmd_data_len_ns0  = cmd_data[31:0];

// 命令解析寄存器
reg [31:0] cur_addr;        // 下一刀的起始地址
reg [31:0] remaining_len;   // 剩余字节
reg        busy;            // 一条命令在处理中

//------------------------------------------------------------------
// 本刀算术（组合，只依赖寄存器 → 路径短）
//------------------------------------------------------------------
wire [12:0] to_4kb     = 13'd4096 - {1'b0, cur_addr[11:0]};   // ★ 13 位：页对齐时是 4096
wire [31:0] cut_reg0   = (to_4kb > remaining_len) ? remaining_len : {19'd0, to_4kb};
wire [31:0] cut_reg1   = (cut_reg0 > MAX_BURST_BYTES) ? MAX_BURST_BYTES : cut_reg0;
wire [7:0]  burst_len_ns0 = (cut_reg1 >> SHIFT) - 8'd1;        // ★ 拍数-1 = awlen

//------------------------------------------------------------------
// 握手
//------------------------------------------------------------------
wire burst_fire = cmd_tvalid & cmd_tready;   // 一条突发被 driver 接走
wire cmd_accept = cmd_valid & s_axis_tready; // 收下一条新命令

assign s_axis_tready = ~busy;                

//------------------------------------------------------------------
// 错误检测（组合，只算不存）—— 从 cmd_data 直接算
//------------------------------------------------------------------
reg [3:0] err_w;

always @(*) begin
    err_w = 4'd0;
    if (cmd_data_len_ns0 == 32'd0)
        err_w[0] = 1'b1;                                  // 空命令
    if (cmd_data_len_ns0[SHIFT-1:0] != 0)
        err_w[1] = 1'b1;                                  // 长度非 AXI_BYTES 整倍数
    if (cmd_data_addr_ns0[SHIFT-1:0] != 0)
        err_w[2] = 1'b1;                                  // 地址未对齐
    if ((cmd_data_addr_ns0 >= MEM_BYTES) ||
        (cmd_data_len_ns0 > (MEM_BYTES - cmd_data_addr_ns0)))
        err_w[3] = 1'b1;                                  // 越界（减法形式，避免 32 位溢出）
end

//------------------------------------------------------------------
// 命令级：收命令 / 收尾推进
//------------------------------------------------------------------
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        cur_addr      <= 32'd0;
        remaining_len <= 32'd0;
        busy          <= 1'b0;
    end
    else if (cmd_accept) begin
        cur_addr      <= cmd_data_addr_ns0;
        remaining_len <= cmd_data_len_ns0;
        busy          <= (err_w == 4'd0);      // 有错不进 busy → 不产生任何突发
    end
    else if (burst_fire) begin
        if (cut_reg1 == remaining_len) begin  
            cur_addr      <= 32'd0;
            remaining_len <= 32'd0;
            busy          <= 1'b0;
        end
        else begin
            cur_addr      <= cur_addr + cut_reg1;
            remaining_len <= remaining_len - cut_reg1;
        end
    end
end

//------------------------------------------------------------------
// 错误输出（粘滞：只在收命令时更新 → ILA 抓得到）
//------------------------------------------------------------------
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) cmd_err <= 4'd0;
    else if (cmd_accept) cmd_err <= err_w;
end

//------------------------------------------------------------------
// 突发级：逐条吐 {cmd_address, burst_len, cmd_tvalid}
//------------------------------------------------------------------
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        cmd_address <= 32'd0;
        burst_len   <= 8'd0;
        cmd_tvalid  <= 1'b0;
    end
    else if (!cmd_tvalid) begin
        if (busy && (cmd_err == 4'd0)) begin  
            cmd_address <= cur_addr;
            burst_len   <= burst_len_ns0;
            cmd_tvalid  <= 1'b1;
        end
    end
    else if (cmd_tready) begin
        cmd_tvalid <= 1'b0;              
    end
end

endmodule
