// 取窗输出级（v2：零气泡串行器，无状态机）。
// 合约：fifo_win 为 show-ahead（rd_data 组合给出队头），rd_en 当拍弹出。
// - kw_reg 记录当前 out_data_reg 中是本窗第几拍；每拍推进 kw 并预取下一拍数据。
// - 末拍且 fifo_win 非空时，同拍弹出下一窗并直通其第 0 拍——窗口间无断拍。
// - 末拍且 fifo_win 空时才断供：这是上游产能限制（如 stride=2/kw=3 每窗
//   4 读对 3 拍的因果限制）或真反压传导，不是内部状态断供。
// - lane 偏移 = byte_start + kw + (stride_c==2 ? lane<<1 : lane)，配置期二选一，
//   无运行期乘法。输出 tvalid/tdata 全部寄存器化；m_axis_ffmtem_tready 按契约视为常 1。
module prefetch_fm_conv_window_out(
    input           clk,
    input           rst_n,

    input   [2:0]   cfg_kernel_size_c,
    input   [2:0]   cfg_byte_start,
    input   [2:0]   cfg_stride_c_real,

    input           win_empty,
    input   [559:0] win_rd_data,
    output          win_rd_en,

    output          m_axis_ffmtem_tvalid,
    output  [255:0] m_axis_ffmtem_tdata,
    input           m_axis_ffmtem_tready,

    output          win_busy
);

reg [559:0]        win_reg;
reg [2:0]          kw_reg;
reg                out_valid_reg;
reg [255:0]        out_data_reg;

wire               kw_last;
wire               load_fire;
wire               output_space;
wire               advance_fire;
wire [2:0]         kw_next;
wire [559:0]       win_src;
wire [4:0]         gather_base;
wire [255:0]       out_data_comb;
genvar             gl;

assign kw_last      = (kw_reg == (cfg_kernel_size_c - 3'd1));
assign output_space = !out_valid_reg || m_axis_ffmtem_tready;
assign advance_fire = out_valid_reg && m_axis_ffmtem_tready && !kw_last;
assign load_fire    = output_space && !win_empty && (!out_valid_reg || kw_last);
assign kw_next      = load_fire ? 3'd0 : (kw_reg + 3'd1);
assign win_src   = load_fire ? win_rd_data : win_reg;

assign win_rd_en            = load_fire;
assign m_axis_ffmtem_tvalid = out_valid_reg;
assign m_axis_ffmtem_tdata  = out_data_reg;
assign win_busy             = out_valid_reg;

// 字节抽取显式拼接（FRAM-008 口径：不放行组合 procedural for）。
// 输出拍第 gl 字节 = win_src 第 (byte_start + kw_next + (stride==2 ? 2gl : gl)) 字节；
// stride 仅 1/2，gl+gl 代替乘法，<<3 代替 ×8，逐车道 generate 展开。
assign gather_base = {2'd0, cfg_byte_start} + {2'd0, kw_next};

generate
    for (gl = 0; gl < 32; gl = gl + 1) begin: out_byte_gather
        assign out_data_comb[(8*gl) +: 8] =
            win_src[(cfg_stride_c_real[1] ?
                     (({1'b0, gather_base} + gl + gl) << 3) :
                     (({1'b0, gather_base} + gl) << 3)) +: 8];
    end
endgenerate

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        out_valid_reg <= 1'b0;
    else if (output_space)
        out_valid_reg <= load_fire || (out_valid_reg && !kw_last);
end

always @(posedge clk) begin
    if (load_fire)
        win_reg <= win_rd_data;
end

always @(posedge clk) begin
    if (load_fire || advance_fire)
        out_data_reg <= out_data_comb;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        kw_reg <= 3'd0;
    else if (load_fire || advance_fire)
        kw_reg <= kw_next;
end

endmodule

// ============================================================
// RTL Self-Check Results
// ============================================================
// [PASS] Rule #1:  All ports have explicit width declarations
// [PASS] Rule #2:  Ports use Verilog wire/reg conventions
// [PASS] Rule #6:  No debug/error-attribution logic or diagnostic-only sideband in RTL
// [PASS] Rule #8:  No logic/always_ff/always_comb used
// [PASS] Rule #12: FSM-free; byte gather is explicit generate wiring (no comb procedural for)
// [PASS] Rule #16: Blocking '=' in comb block, non-blocking '<=' in sequential
// [PASS] Rule #24: No multipliers; lane offset is shift/add (stride 1 or 2)
// [PASS] tvalid/tdata both fully registered (no comb/registered desync)
// [PASS] Zero-bubble window switch: load next window on last beat, beat0 passthrough
// ============================================================
