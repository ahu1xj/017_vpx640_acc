`include "all_defines.vh"

module dma_write_driver #(
    parameter AXI_ADDR_WIDTH     = 32,
    parameter AXI_DATA_WIDTH     = `DMA_AXI_SELECTED_DATA_WIDTH,
    parameter AXI_ID_WIDTH       = 4,
    parameter AXI_STRB_WIDTH     = `DMA_AXI_SELECTED_STRB_WIDTH,
    parameter WR_BEAT_FIFO_DEPTH = 512
) (
    input  wire                       clk,
    input  wire                       rst_n,

    input  wire                       desc_valid,
    output wire                       desc_ready,
    input  wire [63:0]                desc_data,

    input  wire                       wrddr_valid,
    output wire                       wrddr_ready,
    input  wire [511:0]               wrddr_data,

    output wire                       write_busy,

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
    output wire                       m_axi_bready
);

localparam integer BEATS_PER_APP = 512 / AXI_DATA_WIDTH;
localparam [2:0] AXI_SIZE_CONST =
    (AXI_DATA_WIDTH == 64)  ? 3'b011 :
    (AXI_DATA_WIDTH == 128) ? 3'b100 :
    (AXI_DATA_WIDTH == 256) ? 3'b101 :
                              3'b110;

reg [511:0] unpack_data_r;
reg [2:0]   unpack_idx_r;
reg         unpack_active_r;
reg [AXI_DATA_WIDTH-1:0] wr_beat_s_tdata_from_reg;

wire                        wr_beat_s_tvalid;
wire                        wr_beat_s_tready;
wire [AXI_DATA_WIDTH-1:0]   wr_beat_s_tdata;
wire                        wr_beat_m_tvalid;
wire                        wr_beat_m_tready;
wire [AXI_DATA_WIDTH-1:0]   wr_beat_m_tdata;
wire [31:0]                 wr_beat_wr_count;
wire [31:0]                 wr_beat_rd_count;
wire                        wr_beat_room_ok;
wire                        wrddr_take;
wire                        wr_beat_take;
wire                        unpack_last_beat;

reg                         aw_valid_r;
reg [AXI_ADDR_WIDTH-1:0]    aw_addr_r;
reg [7:0]                   aw_len_r;
reg                         w_active_r;
reg [8:0]                   w_beats_left_r;
reg [15:0]                  wr_rsp_pending_count_r;

wire [8:0]                  desc_axi_beats;
wire                        aw_hs;
wire                        b_hs;
wire                        w_hs;
wire                        w_last_hs;
wire                        aw_slot_ready;
wire                        w_slot_ready;
wire                        commit_fire;

assign wr_beat_room_ok = (wr_beat_wr_count <= (WR_BEAT_FIFO_DEPTH - BEATS_PER_APP));
assign wr_beat_s_tvalid = unpack_active_r || ((!unpack_active_r) && wrddr_valid && wr_beat_room_ok);
assign wr_beat_s_tdata  = unpack_active_r ? wr_beat_s_tdata_from_reg
                                           : wrddr_data[AXI_DATA_WIDTH-1:0];
assign wrddr_ready = (!unpack_active_r) && wr_beat_room_ok && wr_beat_s_tready;
assign wrddr_take = wrddr_valid && wrddr_ready;
assign wr_beat_take = wr_beat_s_tvalid && wr_beat_s_tready;
assign unpack_last_beat = (unpack_idx_r == (BEATS_PER_APP - 1));

always @(*) begin
    wr_beat_s_tdata_from_reg = unpack_data_r[(unpack_idx_r * AXI_DATA_WIDTH) +: AXI_DATA_WIDTH];
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        unpack_active_r <= 1'b0;
    else if (unpack_active_r && wr_beat_take && unpack_last_beat)
        unpack_active_r <= 1'b0;
    else if ((!unpack_active_r) && wrddr_take && (BEATS_PER_APP != 1))
        unpack_active_r <= 1'b1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        unpack_data_r <= 512'd0;
    else if (wrddr_take)
        unpack_data_r <= wrddr_data;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        unpack_idx_r <= 3'd0;
    else if (unpack_active_r && wr_beat_take && unpack_last_beat)
        unpack_idx_r <= 3'd0;
    else if (unpack_active_r && wr_beat_take)
        unpack_idx_r <= unpack_idx_r + 3'd1;
    else if ((!unpack_active_r) && wrddr_take && (BEATS_PER_APP != 1))
        unpack_idx_r <= 3'd1;
end

`ifdef DMA_AXI_DATA_WIDTH_64
axis_fifo_64x512 u_wr_beat_fifo (
    .s_axis_aresetn     (rst_n),
    .s_axis_aclk        (clk),
    .s_axis_tvalid      (wr_beat_s_tvalid),
    .s_axis_tready      (wr_beat_s_tready),
    .s_axis_tdata       (wr_beat_s_tdata),
    .m_axis_tvalid      (wr_beat_m_tvalid),
    .m_axis_tready      (wr_beat_m_tready),
    .m_axis_tdata       (wr_beat_m_tdata),
    .axis_wr_data_count (wr_beat_wr_count),
    .axis_rd_data_count (wr_beat_rd_count)
);
`elsif DMA_AXI_DATA_WIDTH_128
axis_fifo_128x512 u_wr_beat_fifo (
    .s_axis_aresetn     (rst_n),
    .s_axis_aclk        (clk),
    .s_axis_tvalid      (wr_beat_s_tvalid),
    .s_axis_tready      (wr_beat_s_tready),
    .s_axis_tdata       (wr_beat_s_tdata),
    .m_axis_tvalid      (wr_beat_m_tvalid),
    .m_axis_tready      (wr_beat_m_tready),
    .m_axis_tdata       (wr_beat_m_tdata),
    .axis_wr_data_count (wr_beat_wr_count),
    .axis_rd_data_count (wr_beat_rd_count)
);
`elsif DMA_AXI_DATA_WIDTH_256
axis_fifo_256x512 u_wr_beat_fifo (
    .s_axis_aresetn     (rst_n),
    .s_axis_aclk        (clk),
    .s_axis_tvalid      (wr_beat_s_tvalid),
    .s_axis_tready      (wr_beat_s_tready),
    .s_axis_tdata       (wr_beat_s_tdata),
    .m_axis_tvalid      (wr_beat_m_tvalid),
    .m_axis_tready      (wr_beat_m_tready),
    .m_axis_tdata       (wr_beat_m_tdata),
    .axis_wr_data_count (wr_beat_wr_count),
    .axis_rd_data_count (wr_beat_rd_count)
);
`elsif DMA_AXI_DATA_WIDTH_512
axis_fifo_512x512 u_wr_beat_fifo (
    .s_axis_aresetn     (rst_n),
    .s_axis_aclk        (clk),
    .s_axis_tvalid      (wr_beat_s_tvalid),
    .s_axis_tready      (wr_beat_s_tready),
    .s_axis_tdata       (wr_beat_s_tdata),
    .m_axis_tvalid      (wr_beat_m_tvalid),
    .m_axis_tready      (wr_beat_m_tready),
    .m_axis_tdata       (wr_beat_m_tdata),
    .axis_wr_data_count (wr_beat_wr_count),
    .axis_rd_data_count (wr_beat_rd_count)
);
`else
dma_axi_data_width_mode_macro_missing u_wr_beat_fifo_mode_macro_missing();
`endif

assign desc_axi_beats = {1'b0, desc_data[31:24]} + 9'd1;
assign aw_hs = m_axi_awvalid && m_axi_awready;
assign b_hs = m_axi_bvalid && m_axi_bready;
assign w_hs = m_axi_wvalid && m_axi_wready;
assign w_last_hs = w_hs && (w_beats_left_r == 9'd1);
assign aw_slot_ready = (!aw_valid_r) || aw_hs;
assign w_slot_ready = (!w_active_r) || w_last_hs;
// commit 数据充分性的精确记账：
// wr_beat_rd_count（FIFO IP 读侧计数）在打流期间存在多拍延迟——已两次实证幻影
// commit（tc13 的 w_last 同拍、tc14 的在途尾拍），"IP 计数 + 在途修正"盖不住
// 延迟幽灵。改为自持记账，与 IP 计数延迟完全无关：
//   credit_cnt_r —— "FIFO 中未被任何 desc 预订的拍数"，增量维护：
//   wr_beat_take（拍实际进入 FIFO）+1，commit_fire 时 -desc_axi_beats，
//   与 push_cnt_r - commit_beats_r 逐拍恒等。值域被 FIFO 深度（512）夹住，
//   11bit 足够。比较直接挂寄存器输出，commit 仅在余量装得下新 desc 全部拍
//   时放行，杜绝一拍两用。
reg [10:0] credit_cnt_r;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        credit_cnt_r <= 11'd0;
    else
        credit_cnt_r <= credit_cnt_r
                      + (wr_beat_take ? 11'd1 : 11'd0)
                      - (commit_fire ? {2'd0, desc_axi_beats} : 11'd0);
end

// commit_ok_r：信用比较挂寄存器（20260729 ui 时序）。原当拍
// "credit 比较 -> commit -> credit 更新"把两级 CARRY 压进同拍；
// 打拍后 commit 只多等 1 拍（描述符保持型握手，语义不变）。
// commit_fire 放行时清 0，下一条描述符必须重新评估，杜绝旧门残值误放行。
// desc_valid=0 时强制清 0：空 FIFO 数据为 X，比较结果不得锁存
// （tc1 实证 X 沿 commit_ok_r→commit_fire→credit_cnt_r 扩散致死）。
reg        commit_ok_r;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        commit_ok_r <= 1'b0;
    else if (!desc_valid)
        commit_ok_r <= 1'b0;
    else if (commit_fire)
        commit_ok_r <= 1'b0;
    else
        commit_ok_r <= (credit_cnt_r >= {2'd0, desc_axi_beats});
end

assign commit_fire = desc_valid &&
                     aw_slot_ready &&
                     w_slot_ready &&
                     commit_ok_r;
assign desc_ready = commit_fire;

// write_busy 打一拍再喂读驱动：仅承担"写活跃期停读、防读独占总线死锁"
// 的粗粒度门控（时序契约：停住的拍点不作要求），打拍后停/放各晚 1 拍，
// 行为为原门的保守超集
reg        write_busy_r;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        write_busy_r <= 1'b0;
    else
        write_busy_r <= w_active_r || commit_fire;
end

assign write_busy = write_busy_r;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        aw_valid_r <= 1'b0;
    else if (commit_fire)
        aw_valid_r <= 1'b1;
    else if (aw_hs)
        aw_valid_r <= 1'b0;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        aw_addr_r <= {AXI_ADDR_WIDTH{1'b0}};
    else if (commit_fire)
        aw_addr_r <= desc_data[63:32];
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        aw_len_r <= 8'd0;
    else if (commit_fire)
        aw_len_r <= desc_data[31:24];
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        w_active_r <= 1'b0;
    else if (commit_fire)
        w_active_r <= 1'b1;
    else if (w_last_hs)
        w_active_r <= 1'b0;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        w_beats_left_r <= 9'd0;
    else if (commit_fire)
        w_beats_left_r <= desc_axi_beats;
    else if (w_hs && (w_beats_left_r != 9'd0))
        w_beats_left_r <= w_beats_left_r - 9'd1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        wr_rsp_pending_count_r <= 16'd0;
    else if (aw_hs && (!b_hs))
        wr_rsp_pending_count_r <= wr_rsp_pending_count_r + 16'd1;
    else if ((!aw_hs) && b_hs)
        wr_rsp_pending_count_r <= wr_rsp_pending_count_r - 16'd1;
end

assign wr_beat_m_tready = w_active_r && m_axi_wready;

assign m_axi_awid    = {AXI_ID_WIDTH{1'b0}};
assign m_axi_awaddr  = aw_addr_r;
assign m_axi_awlen   = aw_len_r;
assign m_axi_awsize  = AXI_SIZE_CONST;
assign m_axi_awburst = 2'b01;
assign m_axi_awlock  = 1'b0;
assign m_axi_awcache = 4'b0011;
assign m_axi_awprot  = 3'b000;
assign m_axi_awqos   = 4'b0000;
assign m_axi_awvalid = aw_valid_r;

assign m_axi_wdata  = wr_beat_m_tdata;
assign m_axi_wstrb  = {AXI_STRB_WIDTH{1'b1}};
assign m_axi_wlast  = (w_beats_left_r == 9'd1);
assign m_axi_wvalid = w_active_r && wr_beat_m_tvalid;

assign m_axi_bready = 1'b1;

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
