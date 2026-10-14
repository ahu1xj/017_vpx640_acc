// res_output_ctrl：residual 模块族输出控制。
// 三源静态选择（一个事务内只有一条 datapath 产出，非仲裁器）：
//   resadd：datapath 已 pack 成 512b（256b 阶段已掩码），直通寄存；
//   bypass：直通 512b，在此施加尾列掩码（掩码计数来自 input ctrl sideband，
//           尾块用 n、非尾块全有效，逐位比较 i < valid_col）；
//   upsample：datapath 内已掩码的 512b，直通寄存。
// out_space = ~out_valid | m_axis_tready（沿用原语义）。
// out_cnt 统计 resadd/bypass 输出拍数供 normal_out_done。
module res_output_ctrl(
    input  wire          clk,
    input  wire          rst_n,

    input  wire          cfg_valid,
    input  wire [2:0]    cfg_mode,
    input  wire [31:0]   cfg_out_total,

    input  wire          resadd_out_valid,
    input  wire [511:0]  resadd_out_data,

    input  wire          bypass_accept,
    input  wire [4:0]    bypass_valid_col,
    input  wire [511:0]  bypass_data,

    input  wire          up_out_valid,
    input  wire [511:0]  up_out_data,

    input  wire          m_axis_res_tready,
    output wire          m_axis_res_tvalid,
    output wire [511:0]  m_axis_res_tdata,

    output wire          out_space,
    output wire          out_valid,
    output wire          normal_out_done
);

localparam [2:0] MODE_RESADD = 3'd1;
localparam [2:0] MODE_BYPASS = 3'd3;

reg          out_valid_q;
reg [511:0]  out_data_q;
reg [31:0]   out_cnt_q;

wire          mode_is_resadd;
wire          mode_is_bypass;
wire          resadd_fire;
wire          up_fire;
wire          any_fire;
wire          out_done;
wire [511:0]  bypass_mask_data;
genvar        gb;

assign mode_is_resadd = (cfg_mode == MODE_RESADD);
assign mode_is_bypass = (cfg_mode == MODE_BYPASS);

assign out_space    = (~out_valid_q) | m_axis_res_tready;
assign resadd_fire  = resadd_out_valid & out_space;
assign up_fire      = up_out_valid & out_space;
assign any_fire     = resadd_fire | bypass_accept | up_fire;

assign out_done        = (out_cnt_q >= cfg_out_total);
assign normal_out_done = out_done;

assign m_axis_res_tvalid = out_valid_q;
assign m_axis_res_tdata  = out_data_q;
assign out_valid         = out_valid_q;

// bypass 尾列掩码：逐位比较 i < valid_col（valid_col=0 全有效），两通道半同图
generate
    for (gb = 0; gb < 32; gb = gb + 1) begin: gen_bypass_mask
        assign bypass_mask_data[(8*gb) +: 8] =
            ((bypass_valid_col == 5'd0) || ({1'b0, bypass_valid_col} > gb[5:0]))
            ? bypass_data[(8*gb) +: 8] : 8'd0;
        assign bypass_mask_data[256+(8*gb) +: 8] =
            ((bypass_valid_col == 5'd0) || ({1'b0, bypass_valid_col} > gb[5:0]))
            ? bypass_data[256+(8*gb) +: 8] : 8'd0;
    end
endgenerate

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        out_valid_q <= 1'b0;
    else if (cfg_valid)
        out_valid_q <= 1'b0;
    else if (out_valid_q && m_axis_res_tready && (!any_fire))
        out_valid_q <= 1'b0;
    else if (any_fire)
        out_valid_q <= 1'b1;
end

always @(posedge clk) begin
    if (resadd_fire)
        out_data_q <= resadd_out_data;
    else if (bypass_accept)
        out_data_q <= bypass_mask_data;
    else if (up_fire)
        out_data_q <= up_out_data;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        out_cnt_q <= 32'd0;
    else if (cfg_valid)
        out_cnt_q <= 32'd0;
    else if (out_valid_q && m_axis_res_tready) begin
        if ((mode_is_resadd || mode_is_bypass) && (!out_done))
            out_cnt_q <= out_cnt_q + 32'd1;
    end
end

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
// [PASS] Declaration-before-use for all regs referenced by wire assigns
// [PASS] Static select (single active datapath per transaction), source priority
//        resadd > bypass > upsample identical to legacy
// [PASS] Bypass mask is per-bit compare (no (1<<n)-1); resadd/upsample pre-masked
// ============================================================
