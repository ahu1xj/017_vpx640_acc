`include "all_defines.vh"

module dma_read_driver #(
    parameter AXI_ADDR_WIDTH  = 32,
    parameter AXI_DATA_WIDTH  = `DMA_AXI_SELECTED_DATA_WIDTH,
    parameter AXI_ID_WIDTH    = 4,
    parameter RDDDR_FIFO_DEPTH = 512
) (
    input  wire                      clk,
    input  wire                      rst_n,

    input  wire                      desc_valid,
    output wire                      desc_ready,
    input  wire [63:0]               desc_data,

    input  wire                      write_busy,

    output wire                      rdddr_valid,
    input  wire                      rdddr_ready,
    output wire [511:0]              rdddr_data,
    input  wire [10:0]               rdddr_rd_count,

    output wire [AXI_ID_WIDTH-1:0]   m_axi_arid,
    output wire [AXI_ADDR_WIDTH-1:0] m_axi_araddr,
    output wire [7:0]                m_axi_arlen,
    output wire [2:0]                m_axi_arsize,
    output wire [1:0]                m_axi_arburst,
    output wire                      m_axi_arlock,
    output wire [3:0]                m_axi_arcache,
    output wire [2:0]                m_axi_arprot,
    output wire [3:0]                m_axi_arqos,
    output wire                      m_axi_arvalid,
    input  wire                      m_axi_arready,

    input  wire [AXI_ID_WIDTH-1:0]   m_axi_rid,
    input  wire [AXI_DATA_WIDTH-1:0] m_axi_rdata,
    input  wire [1:0]                m_axi_rresp,
    input  wire                      m_axi_rlast,
    input  wire                      m_axi_rvalid,
    output wire                      m_axi_rready
);

localparam integer BEATS_PER_APP = 512 / AXI_DATA_WIDTH;
localparam integer BEATS_PER_APP_SHIFT =
    (AXI_DATA_WIDTH == 64)  ? 3 :
    (AXI_DATA_WIDTH == 128) ? 2 :
    (AXI_DATA_WIDTH == 256) ? 1 :
                              0;
localparam integer RDDDR_COMMIT_TH = (RDDDR_FIFO_DEPTH * 3) / 4;
localparam [10:0] RDDDR_COMMIT_TH_W = RDDDR_COMMIT_TH;
localparam [2:0] AXI_SIZE_CONST =
    (AXI_DATA_WIDTH == 64)  ? 3'b011 :
    (AXI_DATA_WIDTH == 128) ? 3'b100 :
    (AXI_DATA_WIDTH == 256) ? 3'b101 :
                              3'b110;

reg                         ar_valid_r;
reg [AXI_ADDR_WIDTH-1:0]    ar_addr_r;
reg [7:0]                   ar_len_r;
reg [15:0]                  ar_app_units_r;
reg [10:0]                  reserved_app_units_r;

reg [511:0]                 pack_data_r;
reg [511:0]                 pack_next_data;
reg [2:0]                   pack_idx_r;
reg                         rdddr_valid_r;
reg [511:0]                 rdddr_data_r;

wire [8:0]                  desc_axi_beats;
wire [15:0]                 desc_app_units;
wire                        ar_hs;
wire                        ar_slot_ready;
wire                        issue_fire;
wire                        r_hs;
wire                        pack_last_beat;
wire                        rdddr_take;

assign desc_axi_beats = {1'b0, desc_data[31:24]} + 9'd1;
assign desc_app_units = desc_axi_beats >> BEATS_PER_APP_SHIFT;
assign ar_hs = m_axi_arvalid && m_axi_arready;
assign ar_slot_ready = (!ar_valid_r) || ar_hs;
assign rdddr_take = rdddr_valid && rdddr_ready;
// issue 门三拍化（20260729 ui 时序）：原"三个加法+门限比较同拍"是 ui 域
// 最长链（-0.068）。拆成 加/加/比 三拍，挂寄存器：
//   occ_r   —— 与描述符无关的占用部分，逐拍跟踪；
//   total_r —— + 当前描述符需求；
//   issue_ok_r —— 纯比较；issue_fire 放行时清 0，保证下一条描述符必须
//   重新走完两级评估，不会被旧门的残值误放行；desc_valid=0 时强制清 0，
//   空 FIFO 数据为 X，比较结果不得锁存（tc1 实证 X 锁存后经 fire 扩散）。
// 描述符是保持型握手，分拍只增加等待延迟；陈旧占用的过冲方向由
// rdddr FIFO 的 R 通道反压兜底（stall 不丢数），功能语义不变。
reg [10:0] occ_r;
reg [10:0] total_r;
reg        issue_ok_r;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        occ_r <= 11'd0;
    else
        occ_r <= reserved_app_units_r + rdddr_rd_count
               + (ar_hs ? ar_app_units_r[10:0] : 11'd0);
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        total_r <= 11'd0;
    else
        total_r <= occ_r + desc_app_units[10:0];
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        issue_ok_r <= 1'b0;
    else if (!desc_valid)
        issue_ok_r <= 1'b0;
    else if (issue_fire)
        issue_ok_r <= 1'b0;
    else
        issue_ok_r <= (total_r <= RDDDR_COMMIT_TH_W);
end

assign issue_fire = desc_valid &&
                    ar_slot_ready &&
                    (!write_busy) &&
                    issue_ok_r;
assign desc_ready = issue_fire;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        ar_valid_r <= 1'b0;
    else if (issue_fire)
        ar_valid_r <= 1'b1;
    else if (ar_hs)
        ar_valid_r <= 1'b0;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        ar_addr_r <= {AXI_ADDR_WIDTH{1'b0}};
    else if (issue_fire)
        ar_addr_r <= desc_data[63:32];
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        ar_len_r <= 8'd0;
    else if (issue_fire)
        ar_len_r <= desc_data[31:24];
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        ar_app_units_r <= 16'd0;
    else if (issue_fire)
        ar_app_units_r <= desc_app_units;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        reserved_app_units_r <= 11'd0;
    else if (ar_hs && (!rdddr_take))
        reserved_app_units_r <= reserved_app_units_r + ar_app_units_r[10:0];
    else if ((!ar_hs) && rdddr_take)
        reserved_app_units_r <= reserved_app_units_r - 11'd1;
    else if (ar_hs && rdddr_take)
        reserved_app_units_r <= reserved_app_units_r + ar_app_units_r[10:0] - 11'd1;
end

always @(*) begin
    pack_next_data = pack_data_r;
    pack_next_data[(pack_idx_r * AXI_DATA_WIDTH) +: AXI_DATA_WIDTH] = m_axi_rdata;
end

assign r_hs = m_axi_rvalid && m_axi_rready;
assign pack_last_beat = (pack_idx_r == (BEATS_PER_APP - 1));
assign m_axi_rready = (!rdddr_valid_r) || rdddr_ready;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        pack_data_r <= 512'd0;
    else if (r_hs)
        pack_data_r <= pack_next_data;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        pack_idx_r <= 3'd0;
    else if (r_hs && pack_last_beat)
        pack_idx_r <= 3'd0;
    else if (r_hs)
        pack_idx_r <= pack_idx_r + 3'd1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        rdddr_valid_r <= 1'b0;
    else if (r_hs && pack_last_beat)
        rdddr_valid_r <= 1'b1;
    else if (rdddr_take)
        rdddr_valid_r <= 1'b0;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        rdddr_data_r <= 512'd0;
    else if (r_hs && pack_last_beat)
        rdddr_data_r <= pack_next_data;
end

assign rdddr_valid = rdddr_valid_r;
assign rdddr_data  = rdddr_data_r;

assign m_axi_arid    = {AXI_ID_WIDTH{1'b0}};
assign m_axi_araddr  = ar_addr_r;
assign m_axi_arlen   = ar_len_r;
assign m_axi_arsize  = AXI_SIZE_CONST;
assign m_axi_arburst = 2'b01;
assign m_axi_arlock  = 1'b0;
assign m_axi_arcache = 4'b0011;
assign m_axi_arprot  = 3'b000;
assign m_axi_arqos   = 4'b0000;
assign m_axi_arvalid = ar_valid_r;

generate
    if (AXI_DATA_WIDTH != `DMA_AXI_SELECTED_DATA_WIDTH) begin : g_axi_width_macro_mismatch
        dma_axi_width_parameter_must_match_mode_macro u_axi_width_macro_mismatch();
    end
    if ((AXI_DATA_WIDTH != 64) &&
        (AXI_DATA_WIDTH != 128) &&
        (AXI_DATA_WIDTH != 256) &&
        (AXI_DATA_WIDTH != 512)) begin : g_bad_axi_width
        dma_axi_width_must_be_64_128_256_or_512 u_bad_axi_width();
    end
endgenerate

endmodule
