`timescale 1ns / 1ps

// 手写 AXIS 同步 FIFO（axis_fifo_64x16_np 的原装替代）：64bit x 16。
// 动机：xpm_fifo_axis 通用结构（格雷码同步 + 多级 doutb_pipe）在 300M 下
// 内部路径超时（ui 域 WNS -0.633/2792 端点，均为该 IP 内部流水级）。
// v2：输出由 FWFT 组合读出改为寄存器切片（mem -> mux -> 输出寄存器），
// 切断 show-ahead 组合长链（v1 实测 300M 下 14 级 5.16ns 违例）；
// 读侧语义为标准 AXIS（valid/ready），输出寄存器带保持，吞吐不受影响。
module sync_fifo_axis #(
    parameter WIDTH = 64,
    parameter DEPTH = 16,
    parameter PTR_W = 4,
    parameter CNT_W = 5,
    parameter RAM_STYLE = "auto"
) (
    input  wire             clk,
    input  wire             rst_n,

    input  wire             s_axis_tvalid,
    output wire             s_axis_tready,
    input  wire [WIDTH-1:0] s_axis_tdata,

    output wire             m_axis_tvalid,
    input  wire             m_axis_tready,
    output wire [WIDTH-1:0] m_axis_tdata,

    output wire [CNT_W-1:0] data_count
);

localparam [CNT_W-1:0] DEPTH_C  = DEPTH[CNT_W-1:0];
localparam [PTR_W-1:0] LAST_PTR = (DEPTH - 1);

(* ram_style = RAM_STYLE *) reg [WIDTH-1:0] mem [0:DEPTH-1];
reg [PTR_W-1:0] wr_ptr_reg;
reg [PTR_W-1:0] rd_ptr_reg;
reg [CNT_W-1:0] cnt_reg;
reg [WIDTH-1:0] m_data_r;
reg             m_valid_r;

wire               wr_fire;
wire               rd_fire;
wire               full;
wire               empty;
wire [PTR_W-1:0]   wr_ptr_next;
wire [PTR_W-1:0]   rd_ptr_next;
wire               dout_en;
wire [PTR_W-1:0]   show_ptr;
wire [CNT_W-1:0]   cnt_after;
wire               empty_next;

assign wr_fire = s_axis_tvalid && s_axis_tready;
assign rd_fire = m_axis_tvalid && m_axis_tready;

assign full  = (cnt_reg == DEPTH_C);
assign empty = (cnt_reg == {CNT_W{1'b0}});

// Source ready only depends on local occupancy, so downstream ready does not
// propagate combinationally back into the producer. After a full FIFO starts
// draining, source ready recovers on the following cycle.
assign s_axis_tready = !full;
assign m_axis_tvalid = m_valid_r;
assign m_axis_tdata  = m_data_r;
assign data_count     = cnt_reg;

// 输出寄存器切片：输出被消费时预取下一项地址，否则保持当前队头
assign dout_en   = m_axis_tready || !m_valid_r;
assign show_ptr  = (m_valid_r && m_axis_tready) ? rd_ptr_next : rd_ptr_reg;
assign cnt_after = cnt_reg + wr_fire - rd_fire;
assign empty_next = (cnt_after == {CNT_W{1'b0}});

assign wr_ptr_next = (wr_ptr_reg == LAST_PTR) ? {PTR_W{1'b0}} : (wr_ptr_reg + 1'b1);
assign rd_ptr_next = (rd_ptr_reg == LAST_PTR) ? {PTR_W{1'b0}} : (rd_ptr_reg + 1'b1);

always @(posedge clk) begin
    if (wr_fire)
        mem[wr_ptr_reg] <= s_axis_tdata;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        wr_ptr_reg <= {PTR_W{1'b0}};
    else if (wr_fire)
        wr_ptr_reg <= wr_ptr_next;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        rd_ptr_reg <= {PTR_W{1'b0}};
    else if (rd_fire)
        rd_ptr_reg <= rd_ptr_next;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        cnt_reg <= {CNT_W{1'b0}};
    else begin
        case ({wr_fire, rd_fire})
            2'b10:   cnt_reg <= cnt_reg + 1'b1;
            2'b01:   cnt_reg <= cnt_reg - 1'b1;
            default: cnt_reg <= cnt_reg;
        endcase
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        m_valid_r <= 1'b0;
    else if (dout_en)
        m_valid_r <= !empty_next;
end

always @(posedge clk) begin
    if (dout_en)
        m_data_r <= (wr_fire && (show_ptr == wr_ptr_reg)) ? s_axis_tdata : mem[show_ptr];
end

endmodule

// ============================================================
// RTL Self-Check Results
// ============================================================
// [PASS] Rule #1:  All ports have explicit width declarations
// [PASS] Rule #2:  Ports use Verilog wire/reg conventions
// [PASS] Rule #6:  No debug/error-attribution logic or diagnostic-only sideband in RTL
// [PASS] Rule #8:  No logic/always_ff/always_comb used
// [PASS] Rule #12: cnt update case has default branch
// [PASS] Rule #16: Non-blocking '<=' for all sequential logic
// [PASS] Rule #24: No multipliers anywhere in the module
// [PASS] One register (or one memory array) per always block
// [PASS] Declaration-before-use for all regs referenced by wire assigns
// [PASS] RAM style is selectable per instance; small FIFOs remain auto by default
// ============================================================
