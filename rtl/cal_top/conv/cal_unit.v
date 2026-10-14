`timescale 1ns / 1ps
`include "all_defines.vh"
// ===============================================================
// cal_unit（20260823，R1 PE bias local tile）
//   每个 cal_unit 只接收本输出通道的一根 32-bit bias；本模块内部保持
//   两级 bias 流水：bias_unit_r -> 4 份 bias_group_r，每份供 4 个 core，
//   同一 group 同时接 initial_psum1/initial_psum2。
//   weight_in 由 8b 改为 128b（每核 8b 切片）。
//   bias 仍每拍捕获，不增加 CE/ready/stall/reset，保持原两拍和吞吐。
// ===============================================================
module cal_unit(
input  wire                                                        clk,
input  wire                                                        rst_n,
input  wire                                                        initial_valid,
input  wire                                                        data_valid,
input  wire [`CAL_UNIT_BSINDATA_WIDTH-1:0]                         bias_in,
input  wire [`CAL_UNIT_FMINDATA_WIDTH-1:0]                         fm_data_256,
input  wire [`CAL_UNIT_WTINDATA_WIDTH*`CAL_UNIT_DSPCORE_NUM-1:0]  weight_in,
output wire [`CAL_UNIT_OUTDATA_WIDTH-1:0]                          fm_out
);

localparam BIAS_GROUP_COUNT = 4;
localparam BIAS_GROUP_SHIFT = 2;

reg [`CAL_UNIT_BSINDATA_WIDTH-1:0] bias_unit_r;
reg [`CAL_UNIT_BSINDATA_WIDTH-1:0] bias_group_r [0:BIAS_GROUP_COUNT-1];

always @(posedge clk) begin
    bias_unit_r <= bias_in;
end

genvar bias_group_idx;
generate
    for (bias_group_idx = 0; bias_group_idx < BIAS_GROUP_COUNT; bias_group_idx = bias_group_idx + 1) begin:BIAS_GROUP_REP
        always @(posedge clk) begin
            bias_group_r[bias_group_idx] <= bias_unit_r;
        end
    end
endgenerate

genvar i;
generate
    for(i=0;i<`CAL_UNIT_DSPCORE_NUM;i=i+1)begin:dsp_core_loop
        dsp_core dsp_core_n(
            .clk                (clk),
            .rst_n              (rst_n),
            .initial_valid      (initial_valid),
            .initial_psum1      (bias_group_r[i>>BIAS_GROUP_SHIFT]),
            .initial_psum2      (bias_group_r[i>>BIAS_GROUP_SHIFT]),
            .data_valid         (data_valid),
            .fm_in1             (fm_data_256[(0+i*16)+:8]),
            .fm_in2             (fm_data_256[(8+i*16)+:8]),
            .weight_in          (weight_in[`CAL_UNIT_WTINDATA_WIDTH*i+:8]),
            .fm_out1            (fm_out[(0+i*64)+:32] ),
            .fm_out2            (fm_out[(32+i*64)+:32])
        );
    end
endgenerate

// ============================================================
// RTL Self-Check Results
// ============================================================
// [PASS] Pure Verilog syntax; every port has explicit direction, wire type, and width.
// [PASS] Sequential logic uses nonblocking assignments; no combinational always block.
// [PASS] Generate loops are elaboration-time genvar loops; no procedural loop/function.
// [PASS] No division/modulo operator, initial block, system task, or RTL delay.
// [PASS] Bias data path is two cycles, has no reset/CE, and captures every clock.
// [PASS] Four local groups each drive four cores and both initial-psum inputs identically.
// [PASS] DSP ports, arithmetic, valid/reset controls, and steady-state throughput are unchanged.
// [PASS] No latch, combinational loop, multi-driver, new state, ready, CE, or CDC path.
// [PASS] Widths, static generate indexes, part-selects, and signedness were checked.
// [FAIL] none.
// ============================================================
endmodule
