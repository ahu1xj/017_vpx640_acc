`timescale 1ns / 1ps
`include "all_defines.vh"

module ins_load (
    // ==================== 系统端口 ====================
    input  wire                         clk,           // 时钟（ui_clk）
    input  wire                         rst_n,         // 复位（低电平有效）

    // ==================== 与初始化模块交互端口（取指接口）====================
    output wire                         fetch_req,         // 取指请求
    input  wire                         fetch_ack,         // 取指应答
    output wire                         fetch_ren,          // 取指读使能
    output wire  [9:0]                  fetch_addr,        // 取指读地址
    input  wire [`RD_SM_CMD_WIDTH-1:0]  fetch_ins,         // 取指指令（160位指令字）

    input  wire                         sync_ptr_reset,    // 同步强制指针复位
    input  wire [9:0]                   fm_entry_addr,     // 特征图载入重入指针
    input  wire [9:0]                   fm_last_addr,      // 最后一条指令对应的指针

    // ==================== 写指令分发（AXI-Stream Master）====================
    output wire                         m_axis_wrcmd_tvalid,   // 写指令有效
    output wire  [`WR_SM_CMD_WIDTH-1:0] m_axis_wrcmd_tdata,     // 写指令数据（160位）
    input  wire                         m_axis_wrcmd_tready,    // 写指令就绪

    // ==================== 读指令分发（AXI-Stream Master）====================
    output wire                         m_axis_rdcmd_tvalid,    // 读指令有效
    output wire  [`RD_SM_CMD_WIDTH-1:0] m_axis_rdcmd_tdata,     // 读指令数据（160位）
    input  wire                         m_axis_rdcmd_tready     // 读指令就绪
);

wire                            work_n1;
wire                            fetch_req_n1;
wire                            fetch_issue_allow;
wire                            command_retire;
wire                            wrcmd_output_fire;
wire                            rdcmd_output_fire;
wire                            fetch_done;
wire                            fetch_ren_n1;
wire [9:0]                      fetch_addr_n1;
wire [`WR_RW_FLAG_WIDTH-1:0]    rw_flag;


reg                 work_r;
reg                 fetch_req_r;
reg                 fetch_ren_r;
reg [9:0]           fetch_addr_r;
reg                 ins_valid_d1;
reg                 ins_valid_r;
reg [2:0]           command_credit_count;

wire                raw_wrcmd_tvalid;
wire                raw_wrcmd_tready;
wire                raw_rdcmd_tvalid;
wire                raw_rdcmd_tready;

// ===============================================================
// Combinational Logic
// ===============================================================

assign work_n1              = work_r ? (!fetch_done) : (fetch_req & fetch_ack);
assign fetch_req_n1         = fetch_req_r ? (!(fetch_req & fetch_ack)) : fetch_done;
assign fetch_req            = fetch_req_r;
// Four credits cover the two-cycle instruction RAM latency plus queue storage.
// Credits are reserved when a RAM read is issued and retired only when the
// corresponding command leaves its local elastic FIFO.  Source ready therefore
// depends only on local registered occupancy, never on a downstream FIFO ready.
assign fetch_issue_allow    = (command_credit_count < 3'd4);
assign fetch_ren_n1         = fetch_ren_r ? (!fetch_done) : (work_r & (fetch_addr <= fm_last_addr));
assign fetch_ren            = fetch_ren_r && fetch_issue_allow;
assign fetch_addr_n1        = sync_ptr_reset ? 10'd0 : (fetch_ren ? ((fetch_addr_r == fm_last_addr) ? fm_entry_addr : (fetch_addr_r + 1)) : fetch_addr_r);
assign fetch_addr           = fetch_addr_r;
assign rw_flag              = fetch_ins[`WR_RW_FLAG_START+:`WR_RW_FLAG_WIDTH];
assign raw_wrcmd_tvalid     = rw_flag && ins_valid_r;
assign raw_rdcmd_tvalid     = (!rw_flag) && ins_valid_r;
assign fetch_done           = fetch_ren & (fetch_addr == fm_last_addr);
assign wrcmd_output_fire     = m_axis_wrcmd_tvalid && m_axis_wrcmd_tready;
assign rdcmd_output_fire     = m_axis_rdcmd_tvalid && m_axis_rdcmd_tready;
assign command_retire        = wrcmd_output_fire || rdcmd_output_fire;

// ===============================================================
// Sequential Logic
// ===============================================================

always@(posedge clk,negedge rst_n)begin
    if(!rst_n) work_r <= 0;
    else work_r <= work_n1;
end

always@(posedge clk,negedge rst_n)begin
    if(!rst_n) fetch_req_r <= 1;
    else fetch_req_r <= fetch_req_n1;
end

always@(posedge clk,negedge rst_n)begin
    if(!rst_n) fetch_ren_r <= 0;
    else fetch_ren_r <= fetch_ren_n1;
end

always@(posedge clk,negedge rst_n)begin
    if(!rst_n) fetch_addr_r <= 0;
    else fetch_addr_r <= fetch_addr_n1;
end

always@(posedge clk,negedge rst_n)begin
    if(!rst_n) command_credit_count <= 3'd0;
    else begin
        case ({fetch_ren, command_retire})
            2'b10: command_credit_count <= command_credit_count + 3'd1;
            2'b01: command_credit_count <= command_credit_count - 3'd1;
            default: command_credit_count <= command_credit_count;
        endcase
    end
end

// ins_ram is a two-cycle synchronous read.  The credit counter reserves one
// of four queue slots before each request enters that fixed-latency pipeline.
always@(posedge clk,negedge rst_n)begin
    if(!rst_n) ins_valid_d1 <= 0;
    else ins_valid_d1 <= fetch_ren;
end

always@(posedge clk,negedge rst_n)begin
    if(!rst_n) ins_valid_r <= 0;
    else ins_valid_r <= ins_valid_d1;
end

sync_fifo_axis #(
    .WIDTH (`WR_SM_CMD_WIDTH),
    .DEPTH (4),
    .PTR_W (2),
    .CNT_W (3)
) u_wrcmd_elastic_fifo (
    .clk           (clk),
    .rst_n         (rst_n),
    .s_axis_tvalid (raw_wrcmd_tvalid),
    .s_axis_tready (raw_wrcmd_tready),
    .s_axis_tdata  (fetch_ins),
    .m_axis_tvalid (m_axis_wrcmd_tvalid),
    .m_axis_tready (m_axis_wrcmd_tready),
    .m_axis_tdata  (m_axis_wrcmd_tdata)
);

sync_fifo_axis #(
    .WIDTH (`RD_SM_CMD_WIDTH),
    .DEPTH (4),
    .PTR_W (2),
    .CNT_W (3)
) u_rdcmd_elastic_fifo (
    .clk           (clk),
    .rst_n         (rst_n),
    .s_axis_tvalid (raw_rdcmd_tvalid),
    .s_axis_tready (raw_rdcmd_tready),
    .s_axis_tdata  (fetch_ins),
    .m_axis_tvalid (m_axis_rdcmd_tvalid),
    .m_axis_tready (m_axis_rdcmd_tready),
    .m_axis_tdata  (m_axis_rdcmd_tdata)
);

endmodule
