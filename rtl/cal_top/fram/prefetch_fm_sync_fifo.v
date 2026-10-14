// 手写同步 FIFO：show-ahead 读（rd_data 组合给出队头），供 prefetch_fm CONV
// 五级流水级间缓冲使用。深度 2~4 的寄存器实现，不使用 vendor FIFO IP。
module prefetch_fm_sync_fifo #(
    parameter DATA_WIDTH = 32,
    parameter DEPTH = 2
)(
    input                       clk,
    input                       rst_n,

    input                       wr_en,
    input      [DATA_WIDTH-1:0] wr_data,
    output                      full,

    input                       rd_en,
    output     [DATA_WIDTH-1:0] rd_data,
    output                      empty
);

localparam PTR_W = (DEPTH <= 2) ? 1 : 2;
localparam CNT_W = (DEPTH <= 2) ? 2 : 3;

localparam [CNT_W-1:0] DEPTH_C   = DEPTH[CNT_W-1:0];
localparam [PTR_W-1:0] LAST_PTR  = (DEPTH-1);

reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];
reg [PTR_W-1:0]      wr_ptr_reg;
reg [PTR_W-1:0]      rd_ptr_reg;
reg [CNT_W-1:0]      cnt_reg;

wire               wr_fire;
wire               rd_fire;
wire [PTR_W-1:0]   wr_ptr_next;
wire [PTR_W-1:0]   rd_ptr_next;

assign wr_fire = wr_en && !full;
assign rd_fire = rd_en && !empty;

assign full    = (cnt_reg == DEPTH_C);
assign empty   = (cnt_reg == {CNT_W{1'b0}});
assign rd_data = mem[rd_ptr_reg];

assign wr_ptr_next = (wr_ptr_reg == LAST_PTR) ? {PTR_W{1'b0}} : (wr_ptr_reg + 1'b1);
assign rd_ptr_next = (rd_ptr_reg == LAST_PTR) ? {PTR_W{1'b0}} : (rd_ptr_reg + 1'b1);

always @(posedge clk) begin
    if (wr_fire)
        mem[wr_ptr_reg] <= wr_data;
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

endmodule

// ============================================================
// RTL Self-Check Results
// ============================================================
// [PASS] Rule #1:  All ports have explicit width declarations
// [PASS] Rule #2:  Ports use Verilog wire/reg conventions
// [PASS] Rule #6:  No debug/error-attribution logic or diagnostic-only sideband in RTL
// [PASS] Rule #8:  No logic/always_ff/always_comb used
// [PASS] Rule #12: cnt update case has default branch
// [PASS] Rule #16: Blocking-free sequential logic, non-blocking '<=' only
// [PASS] Rule #24: No multipliers anywhere in the module
// [PASS] One register (or one memory array) per always block
// [PASS] Declaration-before-use for all regs referenced by wire assigns
// ============================================================
