`include "all_defines.vh"

module dma_axi4_master #(
    parameter AXI_ADDR_WIDTH      = 32,
    parameter AXI_DATA_WIDTH      = `DMA_AXI_SELECTED_DATA_WIDTH,
    parameter AXI_ID_WIDTH        = 4,
    parameter AXI_STRB_WIDTH      = `DMA_AXI_SELECTED_STRB_WIDTH,
    parameter APP_DATA_WIDTH      = 512,
    parameter CMD2_WIDTH          = 64,
    parameter TXN_DESC_WIDTH      = 64,
    parameter WRCMD2_FIFO_DEPTH   = 16,
    parameter RDCMD2_FIFO_DEPTH   = 16,
    parameter WRDDR_FIFO_DEPTH    = 256,
    parameter RDDDR_FIFO_DEPTH    = 256,
    parameter WR_TXN_FIFO_DEPTH   = 16,
    parameter RD_TXN_FIFO_DEPTH   = 16,
    parameter WR_BEAT_FIFO_DEPTH  = 512
) (
    input  wire                       clk,
    input  wire                       usr_clk,
    input  wire                       rst_n,

    input  wire                       s_axis_wrcmd2_tvalid,
    output wire                       s_axis_wrcmd2_tready,
    input  wire [63:0]                s_axis_wrcmd2_tdata,

    input  wire                       s_axis_wrddr_tvalid,
    output wire                       s_axis_wrddr_tready,
    input  wire [511:0]               s_axis_wrddr_tdata,

    input  wire                       s_axis_rdcmd2_tvalid,
    output wire                       s_axis_rdcmd2_tready,
    input  wire [63:0]                s_axis_rdcmd2_tdata,

    output wire                       m_axis_rdddr_tvalid,
    input  wire                       m_axis_rdddr_tready,
    output wire [511:0]               m_axis_rdddr_tdata,

    output wire [AXI_ID_WIDTH-1:0]    m_axi_awid,
    output wire [AXI_ADDR_WIDTH-1:0]  m_axi_awaddr,
    output wire [7:0]                 m_axi_awlen,
    output wire [2:0]                 m_axi_awsize,
    output wire [1:0]                 m_axi_awburst,
    output wire                       m_axi_awlock,
    output wire [3:0]                 m_axi_awcache,
    output wire [2:0]                 m_axi_awprot,
    output wire [3:0]                 m_axi_awqos,
    output wire                       m_axi_awvalid,
    input  wire                       m_axi_awready,

    output wire [AXI_DATA_WIDTH-1:0]  m_axi_wdata,
    output wire [AXI_STRB_WIDTH-1:0]  m_axi_wstrb,
    output wire                       m_axi_wlast,
    output wire                       m_axi_wvalid,
    input  wire                       m_axi_wready,

    input  wire [AXI_ID_WIDTH-1:0]    m_axi_bid,
    input  wire [1:0]                 m_axi_bresp,
    input  wire                       m_axi_bvalid,
    output wire                       m_axi_bready,

    output wire [AXI_ID_WIDTH-1:0]    m_axi_arid,
    output wire [AXI_ADDR_WIDTH-1:0]  m_axi_araddr,
    output wire [7:0]                 m_axi_arlen,
    output wire [2:0]                 m_axi_arsize,
    output wire [1:0]                 m_axi_arburst,
    output wire                       m_axi_arlock,
    output wire [3:0]                 m_axi_arcache,
    output wire [2:0]                 m_axi_arprot,
    output wire [3:0]                 m_axi_arqos,
    output wire                       m_axi_arvalid,
    input  wire                       m_axi_arready,

    input  wire [AXI_ID_WIDTH-1:0]    m_axi_rid,
    input  wire [AXI_DATA_WIDTH-1:0]  m_axi_rdata,
    input  wire [1:0]                 m_axi_rresp,
    input  wire                       m_axi_rlast,
    input  wire                       m_axi_rvalid,
    output wire                       m_axi_rready
);

wire        wrcmd2_fifo_m_valid;
wire        wrcmd2_fifo_m_ready;
wire [63:0] wrcmd2_fifo_m_data;
wire [31:0] wrcmd2_fifo_wr_count;
wire [31:0] wrcmd2_fifo_rd_count;

wire        wrddr_fifo_m_valid;
wire        wrddr_fifo_m_ready;
wire [511:0] wrddr_fifo_m_data;
wire [31:0] wrddr_fifo_wr_count;
wire [31:0] wrddr_fifo_rd_count;

wire        rdcmd2_fifo_m_valid;
wire        rdcmd2_fifo_m_ready;
wire [63:0] rdcmd2_fifo_m_data;
wire [31:0] rdcmd2_fifo_wr_count;
wire [31:0] rdcmd2_fifo_rd_count;

wire        wr_split_desc_valid;
wire        wr_split_desc_ready;
wire [63:0] wr_split_desc_data;
wire        wr_txn_fifo_m_valid;
wire        wr_txn_fifo_m_ready;
wire [63:0] wr_txn_fifo_m_data;

wire        rd_split_desc_valid;
wire        rd_split_desc_ready;
wire [63:0] rd_split_desc_data;
wire        rd_txn_fifo_m_valid;
wire        rd_txn_fifo_m_ready;
wire [63:0] rd_txn_fifo_m_data;
wire        rd_desc_ready_driver;

wire        rdddr_fifo_s_valid;
wire        rdddr_fifo_s_ready;
wire [511:0] rdddr_fifo_s_data;
wire [31:0]  rdddr_fifo_wr_count;
wire [31:0]  rdddr_fifo_rd_count;

wire        write_busy;

axis_fifo_64x16 u_wrcmd2_fifo (
    .s_axis_aresetn     (rst_n),
    .s_axis_aclk        (usr_clk),
    .m_axis_aclk        (clk),
    .s_axis_tvalid      (s_axis_wrcmd2_tvalid),
    .s_axis_tready      (s_axis_wrcmd2_tready),
    .s_axis_tdata       (s_axis_wrcmd2_tdata),
    .m_axis_tvalid      (wrcmd2_fifo_m_valid),
    .m_axis_tready      (wrcmd2_fifo_m_ready),
    .m_axis_tdata       (wrcmd2_fifo_m_data),
    .axis_wr_data_count (wrcmd2_fifo_wr_count),
    .axis_rd_data_count (wrcmd2_fifo_rd_count)
);

axis_fifo_512x256 u_wrddr_fifo (
    .s_axis_aresetn     (rst_n),
    .s_axis_aclk        (usr_clk),
    .m_axis_aclk        (clk),
    .s_axis_tvalid      (s_axis_wrddr_tvalid),
    .s_axis_tready      (s_axis_wrddr_tready),
    .s_axis_tdata       (s_axis_wrddr_tdata),
    .m_axis_tvalid      (wrddr_fifo_m_valid),
    .m_axis_tready      (wrddr_fifo_m_ready),
    .m_axis_tdata       (wrddr_fifo_m_data),
    .axis_wr_data_count (wrddr_fifo_wr_count),
    .axis_rd_data_count (wrddr_fifo_rd_count)
);

axis_fifo_64x16 u_rdcmd2_fifo (
    .s_axis_aresetn     (rst_n),
    .s_axis_aclk        (usr_clk),
    .m_axis_aclk        (clk),
    .s_axis_tvalid      (s_axis_rdcmd2_tvalid),
    .s_axis_tready      (s_axis_rdcmd2_tready),
    .s_axis_tdata       (s_axis_rdcmd2_tdata),
    .m_axis_tvalid      (rdcmd2_fifo_m_valid),
    .m_axis_tready      (rdcmd2_fifo_m_ready),
    .m_axis_tdata       (rdcmd2_fifo_m_data),
    .axis_wr_data_count (rdcmd2_fifo_wr_count),
    .axis_rd_data_count (rdcmd2_fifo_rd_count)
);

dma_cmd_splitter #(
    .AXI_DATA_WIDTH (AXI_DATA_WIDTH)
) u_write_cmd_splitter (
    .clk        (clk),
    .rst_n      (rst_n),
    .cmd_valid  (wrcmd2_fifo_m_valid),
    .cmd_ready  (wrcmd2_fifo_m_ready),
    .cmd_data   (wrcmd2_fifo_m_data),
    .desc_valid (wr_split_desc_valid),
    .desc_ready (wr_split_desc_ready),
    .desc_data  (wr_split_desc_data)
);

sync_fifo_axis #(.WIDTH(64), .DEPTH(16)) u_wr_txn_fifo (
    .clk             (clk),
    .rst_n           (rst_n),
    .s_axis_tvalid   (wr_split_desc_valid),
    .s_axis_tready   (wr_split_desc_ready),
    .s_axis_tdata    (wr_split_desc_data),
    .m_axis_tvalid   (wr_txn_fifo_m_valid),
    .m_axis_tready   (wr_txn_fifo_m_ready),
    .m_axis_tdata    (wr_txn_fifo_m_data)
);

dma_write_driver #(
    .AXI_ADDR_WIDTH     (AXI_ADDR_WIDTH),
    .AXI_DATA_WIDTH     (AXI_DATA_WIDTH),
    .AXI_ID_WIDTH       (AXI_ID_WIDTH),
    .AXI_STRB_WIDTH     (AXI_STRB_WIDTH),
    .WR_BEAT_FIFO_DEPTH (WR_BEAT_FIFO_DEPTH)
) u_write_driver (
    .clk           (clk),
    .rst_n         (rst_n),
    .desc_valid    (wr_txn_fifo_m_valid),
    .desc_ready    (wr_txn_fifo_m_ready),
    .desc_data     (wr_txn_fifo_m_data),
    .wrddr_valid   (wrddr_fifo_m_valid),
    .wrddr_ready   (wrddr_fifo_m_ready),
    .wrddr_data    (wrddr_fifo_m_data),
    .write_busy    (write_busy),
    .m_axi_awid    (m_axi_awid),
    .m_axi_awaddr  (m_axi_awaddr),
    .m_axi_awlen   (m_axi_awlen),
    .m_axi_awsize  (m_axi_awsize),
    .m_axi_awburst (m_axi_awburst),
    .m_axi_awlock  (m_axi_awlock),
    .m_axi_awcache (m_axi_awcache),
    .m_axi_awprot  (m_axi_awprot),
    .m_axi_awqos   (m_axi_awqos),
    .m_axi_awvalid (m_axi_awvalid),
    .m_axi_awready (m_axi_awready),
    .m_axi_wdata   (m_axi_wdata),
    .m_axi_wstrb   (m_axi_wstrb),
    .m_axi_wlast   (m_axi_wlast),
    .m_axi_wvalid  (m_axi_wvalid),
    .m_axi_wready  (m_axi_wready),
    .m_axi_bid     (m_axi_bid),
    .m_axi_bresp   (m_axi_bresp),
    .m_axi_bvalid  (m_axi_bvalid),
    .m_axi_bready  (m_axi_bready)
);

dma_cmd_splitter #(
    .AXI_DATA_WIDTH (AXI_DATA_WIDTH)
) u_read_cmd_splitter (
    .clk        (clk),
    .rst_n      (rst_n),
    .cmd_valid  (rdcmd2_fifo_m_valid),
    .cmd_ready  (rdcmd2_fifo_m_ready),
    .cmd_data   (rdcmd2_fifo_m_data),
    .desc_valid (rd_split_desc_valid),
    .desc_ready (rd_split_desc_ready),
    .desc_data  (rd_split_desc_data)
);

sync_fifo_axis #(.WIDTH(64), .DEPTH(16)) u_rd_txn_fifo (
    .clk             (clk),
    .rst_n           (rst_n),
    .s_axis_tvalid   (rd_split_desc_valid),
    .s_axis_tready   (rd_split_desc_ready),
    .s_axis_tdata    (rd_split_desc_data),
    .m_axis_tvalid   (rd_txn_fifo_m_valid),
    .m_axis_tready   (rd_txn_fifo_m_ready),
    .m_axis_tdata    (rd_txn_fifo_m_data)
);

// rd_txn_fifo -> read_driver 描述符滑移寄存（20260728 ui 时序）：
// fifo 数据输出的 len 字段原先当拍穿过 read_driver 的 occupancy 计算与
// issue/握手链直达 cmd_fifo 读使能（10 级，-0.233ns），打一拍后
// m_data->skid 为纯数据跳，控制链全部改从 skid 寄存器出发
reg        rd_desc_skid_valid;
reg [63:0] rd_desc_skid_data;
wire       rd_desc_consume;
wire       rd_desc_load;

assign rd_desc_consume     = rd_desc_skid_valid && rd_desc_ready_driver;
assign rd_txn_fifo_m_ready = (!rd_desc_skid_valid) || rd_desc_consume;
assign rd_desc_load        = rd_txn_fifo_m_valid && rd_txn_fifo_m_ready;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        rd_desc_skid_valid <= 1'b0;
    else if (rd_desc_load)
        rd_desc_skid_valid <= 1'b1;
    else if (rd_desc_consume)
        rd_desc_skid_valid <= 1'b0;
end

always @(posedge clk) begin
    if (rd_desc_load)
        rd_desc_skid_data <= rd_txn_fifo_m_data;
end

dma_read_driver #(
    .AXI_ADDR_WIDTH   (AXI_ADDR_WIDTH),
    .AXI_DATA_WIDTH   (AXI_DATA_WIDTH),
    .AXI_ID_WIDTH     (AXI_ID_WIDTH),
    .RDDDR_FIFO_DEPTH (RDDDR_FIFO_DEPTH)
) u_read_driver (
    .clk           (clk),
    .rst_n         (rst_n),
    .desc_valid    (rd_desc_skid_valid),
    .desc_ready    (rd_desc_ready_driver),
    .desc_data     (rd_desc_skid_data),
    .write_busy    (write_busy),
    .rdddr_valid   (rdddr_fifo_s_valid),
    .rdddr_ready   (rdddr_fifo_s_ready),
    .rdddr_data    (rdddr_fifo_s_data),
    // FIFO wrapper exposes a 32-bit count; legacy read-driver contract is 11 bits.
    .rdddr_rd_count(rdddr_fifo_wr_count[10:0]),
    .m_axi_arid    (m_axi_arid),
    .m_axi_araddr  (m_axi_araddr),
    .m_axi_arlen   (m_axi_arlen),
    .m_axi_arsize  (m_axi_arsize),
    .m_axi_arburst (m_axi_arburst),
    .m_axi_arlock  (m_axi_arlock),
    .m_axi_arcache (m_axi_arcache),
    .m_axi_arprot  (m_axi_arprot),
    .m_axi_arqos   (m_axi_arqos),
    .m_axi_arvalid (m_axi_arvalid),
    .m_axi_arready (m_axi_arready),
    .m_axi_rid     (m_axi_rid),
    .m_axi_rdata   (m_axi_rdata),
    .m_axi_rresp   (m_axi_rresp),
    .m_axi_rlast   (m_axi_rlast),
    .m_axi_rvalid  (m_axi_rvalid),
    .m_axi_rready  (m_axi_rready)
);

axis_fifo_512x256_np u_rdddr_fifo (
    .s_axis_aresetn     (rst_n),
    .s_axis_aclk        (clk),
    .m_axis_aclk        (usr_clk),
    .s_axis_tvalid      (rdddr_fifo_s_valid),
    .s_axis_tready      (rdddr_fifo_s_ready),
    .s_axis_tdata       (rdddr_fifo_s_data),
    .m_axis_tvalid      (m_axis_rdddr_tvalid),
    .m_axis_tready      (m_axis_rdddr_tready),
    .m_axis_tdata       (m_axis_rdddr_tdata),
    .axis_wr_data_count (rdddr_fifo_wr_count),
    .axis_rd_data_count (rdddr_fifo_rd_count)
);

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
    if (APP_DATA_WIDTH != 512) begin : g_bad_app_width
        dma_app_data_width_must_be_512 u_bad_app_width();
    end
    if ((CMD2_WIDTH != 64) || (TXN_DESC_WIDTH != 64)) begin : g_bad_cmd_desc_width
        dma_cmd_and_desc_width_must_be_64 u_bad_cmd_desc_width();
    end
endgenerate

endmodule
