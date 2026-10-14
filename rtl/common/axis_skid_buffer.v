`timescale 1ns / 1ps

// =============================================================================
// axis_skid_buffer
// Two-entry, fully-registered AXI-Stream elastic buffer.
//
// Properties:
// - Source ready depends only on local occupancy; downstream ready never
//   propagates combinationally into the producer.
// - All M_AXIS signals are driven by registers.  There is no fall-through data
//   path from S_AXIS to M_AXIS, which protects timing at module boundaries.
// - TVALID and TDATA remain stable while M_AXIS_TREADY is low.
// - Sustains one transfer per cycle when occupancy is one.  A full buffer
//   conservatively releases source ready one cycle after a drain.
// - Pack TDATA/TLAST/TUSER together into DATA_WIDTH when sidebands are needed.
// =============================================================================
module axis_skid_buffer #(
    parameter DATA_WIDTH = 8
)(
    input  wire                  clk,
    input  wire                  rst_n,

    input  wire                  s_axis_tvalid,
    output wire                  s_axis_tready,
    input  wire [DATA_WIDTH-1:0] s_axis_tdata,

    output wire                  m_axis_tvalid,
    input  wire                  m_axis_tready,
    output wire [DATA_WIDTH-1:0] m_axis_tdata
);

reg [1:0]            count_reg;
reg [DATA_WIDTH-1:0] data0_reg;
reg [DATA_WIDTH-1:0] data1_reg;

wire input_fire;
wire output_fire;

assign s_axis_tready = (count_reg != 2'd2);
assign m_axis_tvalid = (count_reg != 2'd0);
assign m_axis_tdata  = data0_reg;

assign input_fire  = s_axis_tvalid && s_axis_tready;
assign output_fire = m_axis_tvalid && m_axis_tready;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        count_reg <= 2'd0;
    end
    else begin
        case ({input_fire, output_fire})
            2'b10: count_reg <= count_reg + 2'd1;
            2'b01: count_reg <= count_reg - 2'd1;
            default: count_reg <= count_reg;
        endcase
    end
end

always @(posedge clk) begin
    if (count_reg == 2'd0) begin
        if (input_fire)
            data0_reg <= s_axis_tdata;
    end
    else if (count_reg == 2'd1) begin
        if (input_fire && output_fire)
            data0_reg <= s_axis_tdata;
    end
    else if (output_fire) begin
        data0_reg <= data1_reg;
    end
end

always @(posedge clk) begin
    if ((count_reg == 2'd1) && input_fire && !output_fire)
        data1_reg <= s_axis_tdata;
end

endmodule

// ============================================================
// RTL Self-Check Results
// ============================================================
// [PASS] Rule #1:  All ports have explicit direction and width
// [PASS] Rule #2:  Ports use wire; sequential state uses reg
// [PASS] Rule #6:  No debug/error-attribution logic in RTL
// [PASS] Rule #8:  Pure Verilog; no SystemVerilog keywords
// [PASS] Rule #12: Sequential hold behavior is explicit
// [PASS] Rule #16: Non-blocking assignments used in sequential logic
// [PASS] Rule #24: No multiplier or variable-width arithmetic
// [PASS] FIFO rule: M_AXIS valid/data are fully registered
// [PASS] Timing rule: downstream ready has no combinational source path
// ============================================================
