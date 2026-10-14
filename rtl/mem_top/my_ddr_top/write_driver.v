`timescale 1ns / 1ps
`include "all_defines.vh"

module write_driver #(
    parameter AXI_DATA_WIDTH     = `DMA_AXI_SELECTED_DATA_WIDTH,
    parameter AXI_ADDR_WIDTH     = 32,
    parameter AXI_ID_WIDTH       = 4,
    parameter AXI_STRB_WIDTH     = `DMA_AXI_SELECTED_STRB_WIDTH,
    parameter WR_BEAT_FIFO_DEPTH = 512
) (
    input  wire                      clk,          // ui_clk
    input  wire                      rst_n,

    // ---------------- 来自 decoder_wrcmd ----------------
    input  wire [AXI_ADDR_WIDTH-1:0] burst_addr,   // 本条突发起始地址
    input  wire [7:0]                burst_len,    // = awlen（拍数-1）
    input  wire                      cmd_valid,    // 本条突发有效
    output wire                      cmd_tready,   // 收下本条突发

    // ---------------- 数据输入（来自 wrddr_fifo / unpack）----------------
    input  wire [AXI_DATA_WIDTH-1:0] s_beat_tdata, // AXI 宽度的数据拍
    input  wire                      s_beat_tvalid,
    output wire                      s_beat_tready,

    // ---------------- AXI4 写通道 ----------------
    output wire [AXI_ID_WIDTH-1:0]   m_axi_awid,
    output wire [AXI_ADDR_WIDTH-1:0] m_axi_awaddr,
    output wire [7:0]                m_axi_awlen,
    output wire [2:0]                m_axi_awsize,
    output wire [1:0]                m_axi_awburst,
    output wire                      m_axi_awlock,
    output wire [3:0]                m_axi_awcache,
    output wire [2:0]                m_axi_awprot,
    output wire [3:0]                m_axi_awqos,
    output wire                      m_axi_awvalid,
    input  wire                      m_axi_awready,

    output wire [AXI_DATA_WIDTH-1:0] m_axi_wdata,
    output wire [AXI_STRB_WIDTH-1:0] m_axi_wstrb,
    output wire                      m_axi_wlast,
    output wire                      m_axi_wvalid,
    input  wire                      m_axi_wready,

    input  wire [AXI_ID_WIDTH-1:0]   m_axi_bid,    // 不用（awid 恒 0，无 ID 重排）
    input  wire [1:0]                m_axi_bresp,
    input  wire                      m_axi_bvalid,
    output wire                      m_axi_bready,

    // ---------------- 给读通路 ----------------
    output wire                      write_busy,   // 防死锁门（AW/W 期间有效）
    output wire                      wr_resp_err,  // 粘滞：bresp != OKAY
    output wire                      wr_starve_err // 粘滞：金丝雀，W 期间断粮
);


//输出加一层reg驱动
reg [AXI_ADDR_WIDTH-1:0] aw_addr_r;
reg [7:0]                aw_len_r;
reg                      aw_valid_r;

reg [AXI_DATA_WIDTH-1:0] w_data_r;
reg                      w_data_valid_r;

reg                      b_readty_r;

// AW 参数锁存：cmd_tready 一拉高 decoder 就换下一条突发，而 AW 还要等 awready
reg [AXI_ADDR_WIDTH-1:0] aw_addr_lat;
reg [7:0]                aw_len_lat;

// 粘滞错误（只置不清）
reg                      wr_resp_err_r;
reg                      wr_starve_err_r;


//axi位宽
localparam [2:0] AXI_SIZE_CONST= (AXI_DATA_WIDTH == 512) ? 3'd6 :
                        (AXI_DATA_WIDTH == 256) ? 3'd5 :
                        (AXI_DATA_WIDTH == 128) ? 3'd4 : 3'd3;

//先比原版加入了b通道，额外加入一个通道
localparam [1:0] IDLE=2'b00, AW=2'b01, W=2'b10, B=2'b11;
reg [1:0] cur_state, ns_state;


reg [8:0]                last_cnt;  // 记录当前突发的剩余拍数


//beat fifo，位宽转换后进行缓存
wire                      beat_s_tready;
wire                      beat_m_tvalid;
wire                      beat_m_tready;
wire [AXI_DATA_WIDTH-1:0] beat_m_tdata;
wire [9:0]                beat_count;


sync_fifo_axis #(
    .WIDTH(AXI_DATA_WIDTH), .DEPTH(WR_BEAT_FIFO_DEPTH),
    .PTR_W(9), .CNT_W(10), .RAM_STYLE("block")
) u_beat_fifo (
    .clk(clk), .rst_n(rst_n),
    .s_axis_tvalid(s_beat_tvalid), .s_axis_tready(beat_s_tready), .s_axis_tdata(s_beat_tdata),
    .m_axis_tvalid(beat_m_tvalid), .m_axis_tready(beat_m_tready), .m_axis_tdata(beat_m_tdata),
    .data_count(beat_count)
);
assign s_beat_tready = beat_s_tready;

/*计数器逻辑
last_cnt表示这次写还剩多少写完，当last_cnt==1并且从机w_tready有效时刻代表全部写完
credit_cnt表示fifo中城下的数据量，只有credit_cnt>=burst_len+1时才可以提交w信号

credit_cnt ≡ beat_count 的等价性前提：
  串行 FSM（IDLE→AW→W→B→IDLE）下，上一突发已消费完才回 IDLE，
  故「未被预订的拍数」==「FIFO 实际占用」。
  ★ 若改成流水/多 outstanding，此等价失效 → 必须改回自持记账。
*/
wire [10:0] credit_cnt = beat_count;   // ★ 必须 wire（reg 不能用 assign 驱动）

//本突发拍数：9 位（burst_len 最大 255，+1 = 256，8 位会变 0）
wire [8:0] burst_beats = {1'b0, burst_len} + 9'd1;

//数据够不够 + 收下本条突发
wire credit_ok   = (credit_cnt >= {2'd0, burst_beats});
wire commit_fire = (cur_state == IDLE) && cmd_valid && credit_ok;// 可以传数据了

assign cmd_tready = commit_fire;

//FIFO 只在 W 态且从机收数时弹出（★ 不加 beat_m_tvalid，会形成组合环）
assign beat_m_tready = (cur_state == W) && m_axi_wready;

always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        last_cnt <= 0;
    end else if(commit_fire) begin
        last_cnt <= burst_beats;
    end else if(m_axi_wvalid && m_axi_wready) begin
        last_cnt <= last_cnt - 1;
    end
end

//AW 参数锁存（commit 那一拍）
always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        aw_addr_lat <= 0;
        aw_len_lat  <= 0;
    end else if(commit_fire) begin
        aw_addr_lat <= burst_addr;
        aw_len_lat  <= burst_len;
    end
end

//状态机
always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        cur_state <= IDLE;
    end else begin
        cur_state <= ns_state;
    end
end

always@(*) begin
    ns_state = cur_state;
    case(cur_state)
        IDLE: begin
            if (commit_fire) ns_state = AW;
        end
        AW: begin
            if(m_axi_awready) ns_state = W;
        end
        W: begin
            if(last_cnt == 1 && m_axi_wvalid && m_axi_wready) ns_state = B;
        end
        B: begin
            if(m_axi_bvalid) ns_state = IDLE;
        end
        default: ns_state = IDLE;
    endcase
end

always@(*) begin
    // 默认值（防 latch：每个 reg 都要在所有分支有赋值）
    aw_valid_r     = 1'b0;
    w_data_r       = {AXI_DATA_WIDTH{1'b0}};
    w_data_valid_r = 1'b0;
    b_readty_r     = 1'b1;      // bready 恒 1，不反压

    if(!rst_n) begin
        aw_addr_r = 0;
        aw_len_r = 0;
    end else
    begin
        case(cur_state)
            IDLE: begin
                aw_addr_r = 0;
                aw_len_r = 0;
            end
            AW: begin
                aw_addr_r = aw_addr_lat;   // ★ 用锁存值，不用 burst_addr
                aw_len_r = aw_len_lat;
                aw_valid_r = 1;
            end
            W: begin
                aw_addr_r = 0;
                aw_len_r = 0;
                w_data_r = beat_m_tdata;
                w_data_valid_r = beat_m_tvalid;
            end
            B: begin
                aw_addr_r = 0;
                aw_len_r = 0;
            end
        endcase
    end
    end

//------------------------------------------------------------------
// 输出端口赋值
//------------------------------------------------------------------
assign m_axi_awaddr  = aw_addr_r;
assign m_axi_awlen   = aw_len_r;
assign m_axi_awvalid = aw_valid_r;
assign m_axi_awsize  = AXI_SIZE_CONST;
assign m_axi_awburst = 2'b01;                    // INCR
assign m_axi_awid    = {AXI_ID_WIDTH{1'b0}};
assign m_axi_awlock  = 1'b0;
assign m_axi_awcache = 4'b0011;
assign m_axi_awprot  = 3'b000;
assign m_axi_awqos   = 4'b0000;

assign m_axi_wdata   = w_data_r;
assign m_axi_wvalid  = w_data_valid_r;
assign m_axi_wstrb   = {AXI_STRB_WIDTH{1'b1}};   // 恒全 1
assign m_axi_wlast   = (last_cnt == 9'd1);       // 最后一拍

assign m_axi_bready  = b_readty_r;

assign write_busy    = (cur_state == AW) || (cur_state == W);   // ★ 不含 B

//------------------------------------------------------------------
// err（粘滞：只置不清 → ILA 抓得到）
//   wr_resp_err   —— 从机回了非 OKAY 的写响应
//   wr_starve_err —— 金丝雀：W 期间 FIFO 断粮
//                    credit 判断错 / beat fifo 深度或 CNT_W 配错 的直接症状
//                    正常永远不置位
//------------------------------------------------------------------
wire beat_starved = (cur_state == W) && (!beat_m_tvalid) && (last_cnt != 9'd0);

always@(posedge clk or negedge rst_n) begin
    if(!rst_n) wr_resp_err_r <= 1'b0;
    else if(m_axi_bvalid && (m_axi_bresp !== 2'b00)) wr_resp_err_r <= 1'b1;
end

always@(posedge clk or negedge rst_n) begin
    if(!rst_n) wr_starve_err_r <= 1'b0;
    else if(beat_starved) wr_starve_err_r <= 1'b1;
end

assign wr_resp_err   = wr_resp_err_r;
assign wr_starve_err = wr_starve_err_r;

endmodule
