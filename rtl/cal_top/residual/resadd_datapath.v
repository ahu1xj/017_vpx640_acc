// resadd_datapath：残差加 datapath（原结构规范化版，2026-07-24 定稿）。
// 数据为逐点同位置相加的纯缩减流（int16->int8 且两路合一，字节率天然 4:1，
// 无积压）：每拍可收一组 2x512b（2 路各 1ch x 32col int16），32 车道组合
// 加法+饱和当拍出 256b（1ch x 32col int8），256b 阶段施加尾列掩码
// （逐位比较 i < valid_col，valid_col=0 全有效约定，禁止 (1<<n)-1），
// 两组 256b 拼一拍 512b 输出（pack，低组在前，沿用 {本组, 上一组} 顺序）。
// 流水：pipe 收一组 -> 下一拍出和并消耗（phase0 存 pack_low，phase1 拼输出），
//   输出拍需 out_space；dp_free = pipe 空或将空，供 input ctrl 连续接收。
module resadd_datapath(
    input  wire          clk,
    input  wire          rst_n,

    input  wire          cfg_valid,

    input  wire          pair_load,
    input  wire [4:0]    pair_valid_col,
    input  wire [511:0]  pair_fm1_data,
    input  wire [511:0]  pair_fm2_data,

    input  wire          out_space,

    output wire          dp_free,
    output wire          dp_busy,
    output wire          dp_out_valid,
    output wire [511:0]  dp_out_data
);

// ---- pipe：一组寄存 ----
reg          pipe_valid_q;
reg [511:0]  pipe_fm1_q;
reg [511:0]  pipe_fm2_q;
reg [4:0]    pipe_valid_col_q;

// ---- pack 级 ----
reg          pack_phase_q;
reg [255:0]  pack_low_q;

wire          pipe_drain;
wire [255:0]  sum256;
wire [255:0]  masked256;
genvar        gl;

// phase0 无条件消耗（存入 pack_low）；phase1 拼输出拍，需 out_space
assign pipe_drain = pipe_valid_q & ((~pack_phase_q) | out_space);

assign dp_free = (~pipe_valid_q) | pipe_drain;
assign dp_busy = pipe_valid_q | pack_phase_q;

// 32 车道 int16 加法 + int8 饱和（当拍出和）
resadd_sat_add_i16_512_to_256 u_sat_add (
    .data_a (pipe_fm1_q),
    .data_b (pipe_fm2_q),
    .data_o (sum256)
);

// 256b 阶段尾列掩码：逐位比较 i < valid_col（valid_col=0 全有效）
generate
    for (gl = 0; gl < 32; gl = gl + 1) begin: gen_res_mask
        assign masked256[(8*gl) +: 8] =
            ((pipe_valid_col_q == 5'd0) || ({1'b0, pipe_valid_col_q} > gl[5:0]))
            ? sum256[(8*gl) +: 8] : 8'd0;
    end
endgenerate

// 输出拍：phase1 时 {本组 256b, 上一组 256b}（低组在前）
assign dp_out_valid = pipe_valid_q & pack_phase_q;
assign dp_out_data  = {masked256, pack_low_q};

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        pipe_valid_q <= 1'b0;
    else if (cfg_valid)
        pipe_valid_q <= 1'b0;
    else if (pair_load)
        pipe_valid_q <= 1'b1;
    else if (pipe_drain)
        pipe_valid_q <= 1'b0;
end

always @(posedge clk) begin
    if (pair_load)
        pipe_fm1_q <= pair_fm1_data;
end

always @(posedge clk) begin
    if (pair_load)
        pipe_fm2_q <= pair_fm2_data;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        pipe_valid_col_q <= 5'd0;
    else if (pair_load)
        pipe_valid_col_q <= pair_valid_col;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        pack_phase_q <= 1'b0;
    else if (cfg_valid)
        pack_phase_q <= 1'b0;
    else if (pipe_drain)
        pack_phase_q <= ~pack_phase_q;
end

always @(posedge clk) begin
    if (pipe_drain & (~pack_phase_q))
        pack_low_q <= masked256;
end

endmodule

// resadd_sat_add_i16_512_to_256：32 车道 int16 加法 + int8 饱和。
// 与原实现逐位等价（旧 residual.v 内同名模块的规范化搬迁）。
module resadd_sat_add_i16_512_to_256 (
    input  wire [511:0] data_a,
    input  wire [511:0] data_b,
    output wire [255:0] data_o
);

genvar i;
generate
    for (i = 0; i < 32; i = i + 1) begin : gen_sat_add_lane
        wire signed [15:0] lane_a;
        wire signed [15:0] lane_b;
        wire signed [16:0] lane_sum;
        wire [7:0]         lane_sat;

        assign lane_a   = data_a[(i*16)+15:(i*16)];
        assign lane_b   = data_b[(i*16)+15:(i*16)];
        assign lane_sum = lane_a + lane_b;
        assign lane_sat = (lane_sum > 17'sd127)  ? 8'h7f :
                          (lane_sum < -17'sd128) ? 8'h80 :
                          lane_sum[7:0];
        assign data_o[(i*8)+7:(i*8)] = lane_sat;
    end
endgenerate

endmodule

// ============================================================
// RTL Self-Check Results
// ============================================================
// [PASS] Rule #1:  All ports have explicit width declarations
// [PASS] Rule #2:  Ports use Verilog wire/reg conventions
// [PASS] Rule #6:  No debug/error-attribution logic or diagnostic-only sideband in RTL
// [PASS] Rule #8:  No logic/always_ff/always_comb used
// [PASS] Rule #12: All conditional assigns have both branches (no latch)
// [PASS] Rule #16: Non-blocking '<=' for all sequential logic
// [PASS] One register per always block
// [PASS] 32-lane adder, original single-cycle sum structure restored
// [PASS] Mask is per-bit compare (no (1<<n)-1 barrel shifter), applied at 256b stage
// [PASS] Pack order preserved: {current 256b, previous 256b} (low group first)
// [PASS] Reduction flow (int16->int8, 2-in-1-out): no unbounded accumulation
// ============================================================
