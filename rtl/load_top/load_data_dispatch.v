`timescale 1ns / 1ps
`include "load_tag_define.vh"

// =============================================================================
// load_data_dispatch
// =============================================================================
module load_data_dispatch (
    input  wire         clk,
    input  wire         rst_n,

    input  wire         s_axis_load_data_tvalid,
    input  wire [`LOAD_DDR_DATA_WIDTH-1:0] s_axis_load_data_tdata,
    input  wire [`LOAD_TAG_WIDTH-1:0] s_axis_load_data_tuser,
    output wire         s_axis_load_data_tready,

    output wire         m_axis_fm_tvalid,
    output wire [`LOAD_DDR_DATA_WIDTH-1:0] m_axis_fm_tdata,
    input  wire         m_axis_fm_tready,

    output wire         m_axis_cpad_tvalid,
    output wire [`LOAD_DDR_DATA_WIDTH-1:0] m_axis_cpad_tdata,
    input  wire         m_axis_cpad_tready,

    output wire         m_axis_we_tvalid,
    output wire [`LOAD_DDR_DATA_WIDTH-1:0] m_axis_we_tdata,
    input  wire         m_axis_we_tready,

    output wire         m_axis_bias_tvalid,
    output wire [`LOAD_DDR_DATA_WIDTH-1:0] m_axis_bias_tdata,
    input  wire         m_axis_bias_tready,

    output wire         m_axis_quan_param_tvalid,
    output wire [`LOAD_DDR_DATA_WIDTH-1:0] m_axis_quan_param_tdata,
    input  wire         m_axis_quan_param_tready,

    output wire         m_axis_quan_fm1_tvalid,
    output wire [`LOAD_DDR_DATA_WIDTH-1:0] m_axis_quan_fm1_tdata,
    input  wire         m_axis_quan_fm1_tready,

    output wire         m_axis_quan_fm2_tvalid,
    output wire [`LOAD_DDR_DATA_WIDTH-1:0] m_axis_quan_fm2_tdata,
    input  wire         m_axis_quan_fm2_tready,

    output wire         m_axis_act_tvalid,
    output wire [`LOAD_DDR_DATA_WIDTH-1:0] m_axis_act_tdata,
    input  wire         m_axis_act_tready
);

reg         out_valid;
reg [`LOAD_DDR_DATA_WIDTH-1:0] out_data;
reg [`LOAD_TAG_WIDTH-1:0]      out_user;
reg         skid_valid;
reg [`LOAD_DDR_DATA_WIDTH-1:0] skid_data;
reg [`LOAD_TAG_WIDTH-1:0]      skid_user;
reg         selected_ready;

wire        in_fire;
wire        out_fire;
wire        out_load_from_skid;
wire        out_load_from_input;
wire        skid_load_from_input;
wire        skid_clear;

assign s_axis_load_data_tready = !skid_valid;
assign in_fire                 = s_axis_load_data_tvalid && s_axis_load_data_tready;
assign out_fire                = out_valid && selected_ready;
assign out_load_from_skid      = out_fire && skid_valid;
assign out_load_from_input     = (!skid_valid) && in_fire && ((!out_valid) || out_fire);
assign skid_load_from_input    = out_valid && (!out_fire) && in_fire;
assign skid_clear              = out_fire && skid_valid;

always @(*) begin
    selected_ready = 1'b0;
    case (out_user)
        `LOAD_TAG_FM:         selected_ready = m_axis_fm_tready;
        `LOAD_TAG_CPAD:       selected_ready = m_axis_cpad_tready;
        `LOAD_TAG_WEIGHT:     selected_ready = m_axis_we_tready;
        `LOAD_TAG_BIAS:       selected_ready = m_axis_bias_tready;
        `LOAD_TAG_QUAN_PARAM: selected_ready = m_axis_quan_param_tready;
        `LOAD_TAG_ACT:        selected_ready = m_axis_act_tready;
        `LOAD_TAG_QUAN_FM1:   selected_ready = m_axis_quan_fm1_tready;
        `LOAD_TAG_QUAN_FM2:   selected_ready = m_axis_quan_fm2_tready;
        default:              selected_ready = 1'b0;
    endcase
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        out_valid <= 1'b0;
    else if (out_load_from_skid || out_load_from_input)
        out_valid <= 1'b1;
    else if (out_fire)
        out_valid <= 1'b0;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        out_data <= {`LOAD_DDR_DATA_WIDTH{1'b0}};
    else if (out_load_from_skid)
        out_data <= skid_data;
    else if (out_load_from_input)
        out_data <= s_axis_load_data_tdata;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        out_user <= {`LOAD_TAG_WIDTH{1'b0}};
    else if (out_load_from_skid)
        out_user <= skid_user;
    else if (out_load_from_input)
        out_user <= s_axis_load_data_tuser;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        skid_valid <= 1'b0;
    else if (skid_load_from_input)
        skid_valid <= 1'b1;
    else if (skid_clear)
        skid_valid <= 1'b0;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        skid_data <= {`LOAD_DDR_DATA_WIDTH{1'b0}};
    else if (skid_load_from_input)
        skid_data <= s_axis_load_data_tdata;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        skid_user <= {`LOAD_TAG_WIDTH{1'b0}};
    else if (skid_load_from_input)
        skid_user <= s_axis_load_data_tuser;
end

assign m_axis_fm_tdata         = out_data;
assign m_axis_cpad_tdata       = out_data;
assign m_axis_we_tdata         = out_data;
assign m_axis_bias_tdata       = out_data;
assign m_axis_quan_param_tdata = out_data;
assign m_axis_quan_fm1_tdata   = out_data;
assign m_axis_quan_fm2_tdata   = out_data;
assign m_axis_act_tdata        = out_data;

assign m_axis_fm_tvalid =
    out_valid && (out_user == `LOAD_TAG_FM);

assign m_axis_cpad_tvalid =
    out_valid && (out_user == `LOAD_TAG_CPAD);

assign m_axis_we_tvalid =
    out_valid && (out_user == `LOAD_TAG_WEIGHT);

assign m_axis_bias_tvalid =
    out_valid && (out_user == `LOAD_TAG_BIAS);

assign m_axis_quan_param_tvalid =
    out_valid && (out_user == `LOAD_TAG_QUAN_PARAM);

assign m_axis_act_tvalid =
    out_valid && (out_user == `LOAD_TAG_ACT);

assign m_axis_quan_fm1_tvalid =
    out_valid && (out_user == `LOAD_TAG_QUAN_FM1);

assign m_axis_quan_fm2_tvalid =
    out_valid && (out_user == `LOAD_TAG_QUAN_FM2);

endmodule

// ============================================================
// RTL Self-Check Results
// ============================================================
// [PASS] Rule #1:  All ports have explicit width declarations
// [PASS] Rule #2:  Ports use wire, sequential storage uses reg
// [PASS] Rule #6:  No debug/error-attribution logic or diagnostic-only sideband
// [PASS] Rule #8:  No logic/always_ff/always_comb/SystemVerilog syntax
// [PASS] Rule #12: All combinational branches have defaults
// [PASS] Rule #16: Blocking '=' for comb, non-blocking '<=' for seq
// [PASS] One-register-per-always-block used for all state regs
// [PASS] Input data/user are registered before driving destination outputs
// [PASS] Upstream ready depends only on local skid_valid, not downstream ready
// ============================================================
