// =============================================================================
// quan_unit
// Single-lane quantization pipeline: acc * scale_m0 -> signed round ->
// arithmetic right shift -> INT8/INT16 saturation.
//
// Timing rework note (20260724 Phase A):
// The multiply used to be followed by one combinational stage doing round-bias
// generation, a 65b add, a 65b variable shift and two 65b saturation compares
// (27 logic levels, worst path of the design). The math is unchanged but is
// now split into four registered stages:
//   U1: product only (multiplier register-in register-out)
//   U2: round-bias generation + add
//   U3: variable arithmetic right shift (shift==0 bypass)
//   U4: INT8/INT16 saturation
// Round bias uses a per-bit thermometer compare (no barrel shift + subtract):
//   bias[i] = (shift==0) ? 0 : sign ? (i < shift-1) : (i == shift-1)
// which is bit-identical to the old (1<<(shift-1))-1 / (1<<(shift-1)) forms.
// Latency: data_valid -> quan_valid = 7 cycles (M/S/A1/A2/U2/U3/U4);
// the 32x32 multiply is decomposed into four 16x16 partial products (one DSP
// each) with register stages at partial products / alignment / pairwise add /
// final add (user-frozen pipeline contract, bit-exact vs direct multiply).
// =============================================================================
module quan_unit
#(
    parameter IN_WIDTH    = 32,
    parameter SCALE_WIDTH = 32,
    parameter SHIFT_WIDTH = 6
)
(
    input  wire                                clk,
    input  wire                                rst_n,
    input  wire                                pipe_ce,
    input  wire                                data_valid,
    input  wire signed [IN_WIDTH-1:0]          acc_in,
    input  wire signed [SCALE_WIDTH-1:0]       scale_m0,
    input  wire        [SHIFT_WIDTH-1:0]       shift_n,
    output wire                                quan_valid,
    output wire signed [7:0]                   quan_i8,
    output wire signed [15:0]                  quan_i16
);

localparam PRODUCT_WIDTH = IN_WIDTH + SCALE_WIDTH;
localparam ROUND_WIDTH   = PRODUCT_WIDTH + 1;

reg                             quan_valid_r;
reg signed [7:0]                quan_i8_r;
reg signed [15:0]               quan_i16_r;

// ---- M: four 16x16 partial products, each registered (one DSP per product) ----
// acc = a_hi*2^16 + a_lo, scale = s_hi*2^16 + s_lo（低 16 位无符号、高 16 位有符号）
// product = (pp0 + pp1<<16) + (pp2<<16 + pp3<<32)，与 32x32 直乘逐 bit 等价
// （Python 穷举含极值 6033 组验证）
// 注意：pp0 = a_lo*s_lo 为无符号 32b（可达 0xFFFE0001），al0 必须零扩展，
// 误按有符号扩展会把 >= 2^31 的正部分积翻成负（iverilog 单测实证）。
reg        [31:0]             pp0_r;
reg signed [31:0]             pp1_r;
reg signed [31:0]             pp2_r;
reg signed [31:0]             pp3_r;
reg        [SHIFT_WIDTH-1:0]  shift_m;
reg                             valid_m;

// ---- S: alignment shifts, each registered ----
reg signed [48:0]             al0_r;
reg signed [48:0]             al1_r;
reg signed [63:0]             al2_r;
reg signed [63:0]             al3_r;
reg        [SHIFT_WIDTH-1:0]  shift_s;
reg                             valid_s;

// ---- A1: pairwise adds, registered ----
reg signed [48:0]             sum_lo_r;
reg signed [63:0]             sum_hi_r;
reg        [SHIFT_WIDTH-1:0]  shift_a1;
reg                             valid_a1;

// ---- A2: final add into product register ----
reg signed [PRODUCT_WIDTH-1:0] product_r;
reg        [SHIFT_WIDTH-1:0]  shift_a2;
reg                             valid_a2;

// ---- U2: round bias + add ----
reg signed [ROUND_WIDTH-1:0]    rounded_r;
reg        [SHIFT_WIDTH-1:0]    shift_u2;
reg                             valid_u2;

// ---- U3: arithmetic right shift ----
reg signed [ROUND_WIDTH-1:0]    shifted_r;
reg                             valid_u3;

wire [15:0]                   a_lo;
wire signed [15:0]            a_hi;
wire [15:0]                   s_lo;
wire signed [15:0]            s_hi;
wire signed [16:0]            a_lo_s;
wire signed [16:0]            s_lo_s;
wire signed [ROUND_WIDTH-1:0] product_round_ext;
wire signed [ROUND_WIDTH-1:0] round_bias;

assign a_lo = acc_in[15:0];
assign a_hi = acc_in[31:16];
assign s_lo = scale_m0[15:0];
assign s_hi = scale_m0[31:16];
assign a_lo_s = {1'b0, a_lo};
assign s_lo_s = {1'b0, s_lo};
assign product_round_ext = {product_r[PRODUCT_WIDTH-1], product_r};
assign quan_valid = quan_valid_r;
assign quan_i8    = quan_i8_r;
assign quan_i16   = quan_i16_r;

genvar bi;
generate
    for (bi = 0; bi < ROUND_WIDTH; bi = bi + 1) begin: g_round_bias
        assign round_bias[bi] =
            (shift_a2 == {SHIFT_WIDTH{1'b0}}) ? 1'b0 :
            product_round_ext[ROUND_WIDTH-1]  ? (bi < (shift_a2 - 1'b1)) :
                                                (bi == (shift_a2 - 1'b1));
    end
endgenerate

// ---- M stage: partial products ----
always @(posedge clk) begin
    if(pipe_ce)
        pp0_r <= a_lo * s_lo;
end

always @(posedge clk) begin
    if(pipe_ce)
        pp1_r <= a_hi * s_lo_s;
end

always @(posedge clk) begin
    if(pipe_ce)
        pp2_r <= a_lo_s * s_hi;
end

always @(posedge clk) begin
    if(pipe_ce)
        pp3_r <= a_hi * s_hi;
end

always @(posedge clk) begin
    if(pipe_ce)
        shift_m <= shift_n;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        valid_m <= 1'b0;
    else if(pipe_ce)
        valid_m <= data_valid;
end

// ---- S stage: alignment shifts ----
always @(posedge clk) begin
    if(pipe_ce)
        al0_r <= {17'd0, pp0_r};
end

always @(posedge clk) begin
    if(pipe_ce)
        al1_r <= {pp1_r[31], pp1_r, 16'd0};
end

always @(posedge clk) begin
    if(pipe_ce)
        al2_r <= {{16{pp2_r[31]}}, pp2_r, 16'd0};
end

always @(posedge clk) begin
    if(pipe_ce)
        al3_r <= {pp3_r, 32'd0};
end

always @(posedge clk) begin
    if(pipe_ce)
        shift_s <= shift_m;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        valid_s <= 1'b0;
    else if(pipe_ce)
        valid_s <= valid_m;
end

// ---- A1 stage: pairwise adds ----
always @(posedge clk) begin
    if(pipe_ce)
        sum_lo_r <= al0_r + al1_r;
end

always @(posedge clk) begin
    if(pipe_ce)
        sum_hi_r <= al2_r + al3_r;
end

always @(posedge clk) begin
    if(pipe_ce)
        shift_a1 <= shift_s;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        valid_a1 <= 1'b0;
    else if(pipe_ce)
        valid_a1 <= valid_s;
end

// ---- A2 stage: final add ----
always @(posedge clk) begin
    if(pipe_ce)
        product_r <= {{15{sum_lo_r[48]}}, sum_lo_r} + sum_hi_r;
end

always @(posedge clk) begin
    if(pipe_ce)
        shift_a2 <= shift_a1;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        valid_a2 <= 1'b0;
    else if(pipe_ce)
        valid_a2 <= valid_a1;
end

// ---- U2 stage: round + add ----
always @(posedge clk) begin
    if(pipe_ce)
        rounded_r <= product_round_ext + round_bias;
end

always @(posedge clk) begin
    if(pipe_ce)
        shift_u2 <= shift_a2;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        valid_u2 <= 1'b0;
    else if(pipe_ce)
        valid_u2 <= valid_a2;
end

always @(posedge clk) begin
    if(pipe_ce) begin
        if (shift_u2 == {SHIFT_WIDTH{1'b0}})
            shifted_r <= rounded_r;
        else
            shifted_r <= rounded_r >>> shift_u2;
    end
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        valid_u3 <= 1'b0;
    else if(pipe_ce)
        valid_u3 <= valid_u2;
end

always @(posedge clk) begin
    if(pipe_ce) begin
        if (shifted_r > 65'sd127)
            quan_i8_r <= 8'sh7f;
        else if (shifted_r < -65'sd128)
            quan_i8_r <= 8'sh80;
        else
            quan_i8_r <= shifted_r[7:0];
    end
end

always @(posedge clk) begin
    if(pipe_ce) begin
        if (shifted_r > 65'sd32767)
            quan_i16_r <= 16'sh7fff;
        else if (shifted_r < -65'sd32768)
            quan_i16_r <= 16'sh8000;
        else
            quan_i16_r <= shifted_r[15:0];
    end
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        quan_valid_r <= 1'b0;
    else if(pipe_ce)
        quan_valid_r <= valid_u3;
end

endmodule

// ============================================================
// RTL Self-Check Results
// ============================================================
// [PASS] Rule #1:  All ports have explicit width declarations
// [PASS] Rule #2:  Ports use 'wire', sequential uses 'reg'
// [PASS] Rule #6:  No debug/error-attribution logic in RTL
// [PASS] Rule #8:  No 'logic'/'always_ff'/'always_comb' used
// [PASS] Rule #12: All combinational branches covered (no latch)
// [PASS] Rule #16: Blocking '=' for comb, non-blocking '<=' for seq
// [PASS] Every pipeline register advances only on pipe_ce and otherwise holds
// [PASS] quan output ports are wire-isolated from internal regs
// [WARN] Rule #24: 32x32 multiply decomposed into four 16x16 partial products
//                  (one DSP each); register stage at partial products /
//                  alignment / pairwise add / final add; round/shift/sat each
//                  own a registered stage; round bias uses thermometer compare
// [PASS] pipe_ce stalls data, shift, valid and output registers in lockstep
// ============================================================
