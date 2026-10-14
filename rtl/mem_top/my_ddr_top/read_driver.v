`timescale 1ns / 1ps
`include "all_defines.vh"

module read_driver #(
    parameter AXI_DATA_WIDTH   = `DMA_AXI_SELECTED_DATA_WIDTH,
    parameter AXI_ADDR_WIDTH   = 32,
    parameter AXI_ID_WIDTH     = 4,
    parameter RDDDR_FIFO_DEPTH = 256
) (
    input  wire                      clk,          // ui_clk
    input  wire                      rst_n,

    // ---------------- 来自 decoder_rdcmd ----------------
    input  wire [AXI_ADDR_WIDTH-1:0] burst_addr,   // 本条突发起始地址
    input  wire [7:0]                burst_len,    // = arlen（拍数-1）
    input  wire                      cmd_valid,    // 本条突发有效
    output wire                      cmd_tready,   // 收下本条突发

    // ---------------- 去 dma_app_pack（AXI 宽度的拍流）----------------
    output wire [AXI_DATA_WIDTH-1:0] m_beat_tdata,
    output wire                      m_beat_tvalid,
    input  wire                      m_beat_tready,

    // ---------------- 下游 FIFO 占用（clk 域，单位 = 512b 字）----------------
    input  wire [10:0]               rdddr_count,

    // ---------------- 来自 write_driver ----------------
    input  wire                      write_busy,   // ★ 写活跃期让路

    // ---------------- AXI4 读通道 ----------------
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

    input  wire [AXI_ID_WIDTH-1:0]   m_axi_rid,    // 不用（arid 恒 0）
    input  wire [AXI_DATA_WIDTH-1:0] m_axi_rdata,
    input  wire [1:0]                m_axi_rresp,
    input  wire                      m_axi_rlast,
    input  wire                      m_axi_rvalid,
    output wire                      m_axi_rready,

    // ---------------- 诊断 ----------------
    output wire                      rd_resp_err   // 粘滞：rresp != OKAY
);


//输出加一层reg驱动
reg [AXI_ADDR_WIDTH-1:0] ar_addr_r;
reg [7:0]                ar_len_r;
reg                      ar_valid_r;

reg [AXI_DATA_WIDTH-1:0] r_data_r;
reg                      r_data_valid_r;

reg                      r_readty_r;

// AR 参数锁存：cmd_tready 一拉高 decoder 就换下一条突发，而 AR 还要等 arready
reg [AXI_ADDR_WIDTH-1:0] ar_addr_lat;
reg [7:0]                ar_len_lat;

// 粘滞错误（只置不清）
reg                      rd_resp_err_r;


//axi位宽
localparam [2:0] AXI_SIZE_CONST= (AXI_DATA_WIDTH == 512) ? 3'd6 :
                        (AXI_DATA_WIDTH == 256) ? 3'd5 :
                        (AXI_DATA_WIDTH == 128) ? 3'd4 : 3'd3;

//先比原版加入了b通道，读侧没有 b 段，三态
localparam [1:0] IDLE=2'b00, AR=2'b01, R=2'b10;
reg [1:0] cur_state, ns_state;


reg [8:0]                r_last_cnt;   // 本次突发还剩几拍


/*计数器逻辑
r_last_cnt表示这次读还剩多少读完，当r_last_cnt==1并且从机r_valid有效时刻代表全部读完
空间判据：读侧是「有地方才要」，和写侧的「有货才发」反着来

rdddr_count 是下游 fifo 的占用（512b 字为单位），本突发要占 burst_apps 个字，
两者之和不能超阈值，否则 fifo 溢出丢数据。

★ reserved_r（在途预留）在本结构下是多余的：
  串行 FSM 只在 R 末拍收完后才回 IDLE，故决策点「在途拍数」恒为 0，
  等价于「fifo 实际占用」。
  ★ 若改成多 outstanding（AR 不等 R 就发下一条），此等价失效 → 必须加。
*/

//本突发拍数：9 位（burst_len 最大 255，+1 = 256，8 位会变 0）
wire [8:0] burst_beats = {1'b0, burst_len} + 9'd1;

//本突发占几个 512b 字（fifo 计数单位）
localparam BEATS_PER_APP = `DMA_APP_DATA_WIDTH / AXI_DATA_WIDTH;   // 1/2/4/8
localparam APPS_SHIFT    = (BEATS_PER_APP == 1) ? 0 :
                           (BEATS_PER_APP == 2) ? 1 :
                           (BEATS_PER_APP == 4) ? 2 : 3;
wire [10:0] burst_apps = {2'd0, burst_beats} >> APPS_SHIFT;

//阈值：留 1/4 余量，避免刚好卡满
localparam [10:0] RDDDR_COMMIT_TH = (RDDDR_FIFO_DEPTH * 3) / 4;

//空间够不够
wire [11:0] apps_sum = {1'b0, rdddr_count} + {1'b0, burst_apps};
wire        room_ok  = (apps_sum <= {1'b0, RDDDR_COMMIT_TH});

//★ 必须让写：读流独占总线 → 写的 B 响应回不来 → 死锁
wire issue_fire = (cur_state == IDLE) && cmd_valid && room_ok && (!write_busy);

assign cmd_tready = issue_fire;

//★ rready 必须能被下游反压（不能恒 1，否则 fifo 会溢出丢数据）
assign m_axi_rready = r_readty_r;

//握手
wire ar_hs     = m_axi_arvalid && m_axi_arready;
wire r_hs      = m_axi_rvalid  && m_axi_rready;
wire r_last_hs = r_hs && (r_last_cnt == 9'd1);


always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        r_last_cnt <= 0;
    end else if(issue_fire) begin
        r_last_cnt <= burst_beats;
    end else if(m_axi_rvalid && m_axi_rready) begin
        r_last_cnt <= r_last_cnt - 1;
    end
end

//AR 参数锁存（issue 那一拍）
always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        ar_addr_lat <= 0;
        ar_len_lat  <= 0;
    end else if(issue_fire) begin
        ar_addr_lat <= burst_addr;
        ar_len_lat  <= burst_len;
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
            if (issue_fire) ns_state = AR;
        end
        AR: begin
            if(m_axi_arready) ns_state = R;
        end
        R: begin
            if(r_last_cnt == 1 && m_axi_rvalid && m_axi_rready) ns_state = IDLE;
        end
        default: ns_state = IDLE;
    endcase
end

always@(*) begin
    // 默认值（防 latch：每个 reg 都要在所有分支有赋值）
    ar_valid_r     = 1'b0;
    r_data_r       = {AXI_DATA_WIDTH{1'b0}};
    r_data_valid_r = 1'b0;
    r_readty_r     = 1'b0;      // rready 只在 R 态且下游能收时拉高

    if(!rst_n) begin
        ar_addr_r = 0;
        ar_len_r = 0;
    end else
    begin
        case(cur_state)
            IDLE: begin
                ar_addr_r = 0;
                ar_len_r = 0;
            end
            AR: begin
                ar_addr_r = ar_addr_lat;   // ★ 用锁存值，不用 burst_addr
                ar_len_r = ar_len_lat;
                ar_valid_r = 1;
            end
            R: begin
                ar_addr_r = 0;
                ar_len_r = 0;
                r_data_r = m_axi_rdata;
                r_data_valid_r = m_axi_rvalid;
                r_readty_r = m_beat_tready;
            end
            default: begin
                ar_addr_r = 0;
                ar_len_r = 0;
            end
        endcase
    end
    end

//------------------------------------------------------------------
// 输出端口赋值
//------------------------------------------------------------------
assign m_axi_araddr  = ar_addr_r;
assign m_axi_arlen   = ar_len_r;
assign m_axi_arvalid = ar_valid_r;
assign m_axi_arsize  = AXI_SIZE_CONST;
assign m_axi_arburst = 2'b01;                    // INCR
assign m_axi_arid    = {AXI_ID_WIDTH{1'b0}};
assign m_axi_arlock  = 1'b0;
assign m_axi_arcache = 4'b0011;
assign m_axi_arprot  = 3'b000;
assign m_axi_arqos   = 4'b0000;

assign m_beat_tdata  = r_data_r;
assign m_beat_tvalid = r_data_valid_r;

//------------------------------------------------------------------
// err（粘滞：只置不清 → ILA 抓得到）
//   rd_resp_err —— 从机回了非 OKAY 的读响应
//------------------------------------------------------------------
always@(posedge clk or negedge rst_n) begin
    if(!rst_n) rd_resp_err_r <= 1'b0;
    else if(m_axi_rvalid && m_axi_rready && (m_axi_rresp !== 2'b00)) rd_resp_err_r <= 1'b1;
end

assign rd_resp_err = rd_resp_err_r;

endmodule
