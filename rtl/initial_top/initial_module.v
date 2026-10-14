`timescale 1ns / 1ps
`include "all_defines.vh"

// =============================================================================
// initial_module（006 版：512bit PS 输入直取）
// 与 003 版差异：PS 输入由 32bit 改为 512bit，取消 32→512 移位拼接。
// 新数据组织（每逻辑 beat 一个 512bit 字，低字节有效、高位补 0）：
//   包头拍：低 32bit 为包头（type/len 字段同前），高 480bit 全 0；
//   INS 拍：低 160bit 为一条指令（原 5x32bit 顺序不变），直接写 ins_ram；
//   BA  拍：低 32bit 为基地址，直接写 ba_ram；
//   PARAM/FM 拍：512bit 全有效，直接进 psi skid。
// len 字段语义：有效负载 beat 数（INS=指令条数、BA=地址个数、数据=512b 字数）。
// 状态机、取指握手、fm_entry/fm_last 指针、psi 输出契约与 003 版一致。
// =============================================================================
module initial_module (
    // ==================== 系统端口 ====================
    input  wire                         clk,           // 时钟（dma_clk）
    input  wire                         rst_n,         // 复位（低电平有效）

    // ==================== 数据输入端口（AXI-Stream 从机，512bit）====================
    input  wire                         s_axis_ps_tvalid,  // PS端数据输入有效
    input  wire [511:0]                 s_axis_ps_tdata,   // PS端数据输入
    output wire                         s_axis_ps_tready,  // PS端数据就绪

    // ==================== 与后级模块交互端口（取指控制）====================
    input  wire                         fetch_req,         // 取指请求
    output reg                          fetch_ack,         // 取指应答
    input  wire                         fetch_ren,         // 取指读使能
    input  wire [9:0]                   fetch_addr,       // 取指读地址（10位寻址空间）
    output wire [`RD_SM_CMD_WIDTH-1:0]  fetch_ins,        // 取指指令（160位指令字）

    output wire                         sync_ptr_reset,    // 同步强制指针复位
    output reg  [9:0]                   fm_entry_addr,     // 特征图载入重入指针
    output reg  [9:0]                   fm_last_addr,      // 最后一条指令对应的指针

    // ==================== 加载模块基地址 RAM 访存端口 ====================
    input  wire                         load_ba_rd,         // 基地址读使能
    input  wire [10:0]                  load_ba_ptr,        // 基地址读指针（11位寻址）
    output wire [31:0]                  load_ba_data,       // 基地址读数据

    // ==================== 存储模块基地址 RAM 访存端口 ====================
    input  wire                         store_ba_rd,         // 基地址读使能
    input  wire [10:0]                  store_ba_ptr,        // 基地址读指针（11位寻址）
    output wire [31:0]                  store_ba_data,       // 基地址读数据

    // ==================== PS 数据整合后输出端口（AXI-Stream 主机）====================
    output wire                         m_axis_psi_tvalid,  // 整合后数据输出有效（权重、偏置、特征图等）
    output wire [511:0]                 m_axis_psi_tdata,    // 整合后数据输出（512位）
    input  wire                         m_axis_psi_tready    // AXI-Stream output ready
);
// ===============================================================
// 信号声明
// ===============================================================
wire                                fm_en;
wire                                param_en;
wire                                ba_en;
wire                                ins_en;
wire                                handshake;
wire                                psi_raw_tvalid;
wire                                psi_raw_tready;
wire [511:0]                        psi_raw_tdata;
wire [`INIT_DATA_TYPE_WIDTH-1:0]    data_type_decode;
wire [`INIT_DATA_LEN_WIDTH-1:0]     data_len_decode;
wire                                fetch_ack_n1;
wire                                transfer_done;
wire                                is_transfer_state;
wire [`RD_SM_CMD_WIDTH-1:0]         ins;
wire                                ins_wr_en;
wire [31:0]                         base_address;
wire                                ba_wr_en;
wire [`WR_DATA_ORIGIN_WIDTH-1:0]    data_origin;
wire [`WR_PS_DATA_TYPE_WIDTH-1:0]   ps_data_type;
wire [10:0]                         ba_addra;


(* mark_debug = "true" *) reg [2:0] curr_state;   // 20260817 ILA v3: 初始模块 FSM 状态探针
reg [2:0] next_state;
reg [`INIT_DATA_LEN_WIDTH-1:0]      transfer_cnt;
reg [`INIT_DATA_TYPE_WIDTH-1:0]     data_type;
reg [`INIT_DATA_LEN_WIDTH-1:0]      data_len;
reg  [9:0]                          ins_wr_addr;
reg  [10:0]                         ba_wr_addr;

localparam [2:0] INIT_IDLE       = 3'd0;
localparam [2:0] INIT_RECV_INS   = 3'd1;
localparam [2:0] INIT_RECV_BA    = 3'd2;
localparam [2:0] INIT_RECV_PARAM = 3'd3;
localparam [2:0] INIT_RECV_FM    = 3'd4;
localparam [2:0] INIT_WAIT_TRANS = 3'd5;
localparam [2:0] INIT_WAIT_INFER = 3'd6;

// ===============================================================
// 状态机跳转
// ===============================================================


always@(posedge clk,negedge rst_n)begin
    if(!rst_n) curr_state <= INIT_IDLE;
    else curr_state <= next_state;
end

always@(*)begin
    case(curr_state)
        INIT_IDLE:begin
            case({fetch_req & fm_en, fetch_req & param_en, fetch_req & ba_en, fetch_req & ins_en})
                4'b0001:     next_state = INIT_RECV_INS;
                4'b0010:     next_state = INIT_RECV_BA;
                4'b0100:     next_state = INIT_RECV_PARAM;
                4'b1000:     next_state = INIT_RECV_FM;
                default:    next_state = curr_state;
            endcase
        end

        INIT_RECV_INS:begin
            if(transfer_done) next_state = INIT_IDLE;
            else next_state = curr_state;
        end

        INIT_RECV_BA:begin
            if(transfer_done) next_state = INIT_IDLE;
            else next_state = curr_state;
        end

        INIT_RECV_PARAM:begin
            if(transfer_done) next_state = INIT_WAIT_TRANS;
            else next_state = curr_state;
        end

        INIT_WAIT_TRANS:begin
            if(param_en) next_state = INIT_RECV_PARAM;
            else if(fm_en) next_state = INIT_RECV_FM;
            else next_state = curr_state;
        end

        INIT_RECV_FM:begin
            if(transfer_done) next_state = INIT_WAIT_INFER;
            else next_state = curr_state;
        end

        INIT_WAIT_INFER:begin
            if(fetch_req) next_state = INIT_IDLE;
            else next_state = curr_state;
        end

        default:next_state = INIT_IDLE;
    endcase
end

// ===============================================================
// 输入握手
// ===============================================================

assign handshake                = s_axis_ps_tvalid & s_axis_ps_tready;


// ===============================================================
// 包头译码（包头拍仅低 32bit 有效，高位全 0）
// ===============================================================



assign data_type_decode         = s_axis_ps_tdata[`INIT_DATA_TYPE_START+:`INIT_DATA_TYPE_WIDTH] ;
assign data_len_decode          = s_axis_ps_tdata[`INIT_DATA_LEN_START+:`INIT_DATA_LEN_WIDTH] ;

always@(posedge clk,negedge rst_n)begin
    if(!rst_n) begin
        data_type <= 0;
        data_len <= 0;
    end
    else if((!is_transfer_state) & handshake) begin
        data_type <= data_type_decode;
        data_len <= data_len_decode;
    end
end


// ===============================================================
// 跳转使能
// ===============================================================

assign ins_en                   = (data_type_decode == `INIT_DATA_TYPE_INS) & handshake;
assign ba_en                    = (data_type_decode == `INIT_DATA_TYPE_BA) & handshake;
assign param_en                 = (data_type_decode == `INIT_DATA_TYPE_PARAM) & handshake;
assign fm_en                    = (data_type_decode == `INIT_DATA_TYPE_FM) & handshake;

// ===============================================================
// 数据接收控制
// ===============================================================
assign s_axis_ps_tready         = (curr_state == INIT_IDLE) ? fetch_req :
                                    (curr_state == INIT_WAIT_INFER) ? 1'b0 :
                                    ((curr_state == INIT_RECV_PARAM) || (curr_state == INIT_RECV_FM)) ?
                                    psi_raw_tready : 1'b1;

// ===============================================================
// 数据FIFO反压控制
// ===============================================================

// ===============================================================
// 允许取指握手逻辑
// ===============================================================
assign fetch_ack_n1 = (curr_state == INIT_IDLE) & fetch_req & (param_en | fm_en);
always@(posedge clk,negedge rst_n)begin
    if(!rst_n) fetch_ack <= 1'b0;
    else fetch_ack <= fetch_ack_n1;
end


// ===============================================================
// 数据处理
// ===============================================================

// 数据传输计数器

assign is_transfer_state = (curr_state == INIT_RECV_INS) | (curr_state == INIT_RECV_BA) | (curr_state == INIT_RECV_PARAM) | (curr_state == INIT_RECV_FM);
assign transfer_done = is_transfer_state & (transfer_cnt == data_len - 1) & handshake;
always@(posedge clk,negedge rst_n)begin
    if(!rst_n) transfer_cnt <= 0;
    else if(is_transfer_state & handshake) transfer_cnt <= (transfer_done) ? 0 : transfer_cnt + 1;
end

// ===============================================================
// 指令写入（每 beat 低 160bit 一条指令，当拍直写）
// ===============================================================
assign ins = s_axis_ps_tdata[`RD_SM_CMD_WIDTH-1:0];
assign ins_wr_en = (curr_state == INIT_RECV_INS) & handshake;

always@(posedge clk,negedge rst_n)begin
    if(!rst_n) ins_wr_addr <= 0;
    else if(curr_state == INIT_RECV_INS) begin
        if(ins_wr_en) ins_wr_addr <= ins_wr_addr + 1;
    end
    else ins_wr_addr <= 0;
end

// ===============================================================
// 指针记录
// ===============================================================
// 指令译码
assign data_origin  = ins[`WR_DATA_ORIGIN_START+:`WR_DATA_ORIGIN_WIDTH];
assign ps_data_type = ins[`WR_PS_DATA_TYPE_START+:`WR_PS_DATA_TYPE_WIDTH];

// 模型重入指针
// 2026-08-05 修复：补读写标志位——此前不卡 rw，读指令的字段混叠
// （CONV 的 model_sel=0000 撞 origin=PS、channel_in 低位撞 type=FM）会把
// fm_entry 误覆盖成读指令地址，FM-only 重入跳过 FM 上传（cin2 实证）。
always@(posedge clk,negedge rst_n)begin
    if(!rst_n) fm_entry_addr <= 0;
    else if(ins_wr_en & ins[`WR_RW_FLAG_START] & (data_origin == `DATA_ORIGIN_PS) & (ps_data_type == `PS_DATA_TYPE_FM)) fm_entry_addr <= ins_wr_addr;
end

// 模型结束指针：最后一条指令写入当拍锁存其写地址
// （等价 003 旧实现中"末拍组装完成写 RAM 时"的 ins_wr_addr 值）
always@(posedge clk,negedge rst_n)begin
    if(!rst_n) fm_last_addr <= 0;
    else if(ins_wr_en & (transfer_cnt == data_len - 1)) fm_last_addr <= ins_wr_addr;
end

// 同步强制指针复位信号
assign sync_ptr_reset = (curr_state == INIT_RECV_INS);

// ===============================================================
// 基地址写入（每 beat 低 32bit 一个地址，当拍直写）
// ===============================================================
assign base_address = s_axis_ps_tdata[31:0];
assign ba_wr_en = (curr_state == INIT_RECV_BA) & handshake;
assign ba_addra = ba_wr_en ? ba_wr_addr : load_ba_ptr;

always@(posedge clk,negedge rst_n)begin
    if(!rst_n) ba_wr_addr <= 0;
    else if(curr_state == INIT_RECV_BA) begin
        if(ba_wr_en) ba_wr_addr <= ba_wr_addr + 1;
    end
    else ba_wr_addr <= 0;
end

// ===============================================================
// 其他类型数据传输（PARAM/FM 每 beat 整字直通 psi skid）
// ===============================================================
assign psi_raw_tdata  = s_axis_ps_tdata;
assign psi_raw_tvalid = ((curr_state == INIT_RECV_PARAM) || (curr_state == INIT_RECV_FM)) & s_axis_ps_tvalid;


// ===============================================================
// ram instance（BMG IP，DO_REG=1：读出 2 拍延迟）
// ===============================================================

ins_ram_160x1024 ins_ram (
    .clka  (clk),
    .ena   (1'b1),
    .wea   (ins_wr_en),
    .addra (ins_wr_addr),
    .dina  (ins),
    .clkb  (clk),
    .enb   (1'b1),
    .addrb (fetch_addr),
    .doutb (fetch_ins)
);

// A 口：写 + load 侧读（复用）；B 口：store 侧只读
ba_ram_32x2048 base_address_ram (
    .clka  (clk),
    .ena   (1'b1),
    .wea   (ba_wr_en),
    .addra (ba_addra),
    .dina  (base_address),
    .douta (load_ba_data),
    .clkb  (clk),
    .enb   (1'b1),
    .web   (1'b0),
    .addrb (store_ba_ptr),
    .dinb  (32'd0),
    .doutb (store_ba_data)
);


axis_skid_buffer #(
    .DATA_WIDTH (512)
) u_psi_skid (
    .clk           (clk),
    .rst_n         (rst_n),
    .s_axis_tvalid (psi_raw_tvalid),
    .s_axis_tready (psi_raw_tready),
    .s_axis_tdata  (psi_raw_tdata),
    .m_axis_tvalid (m_axis_psi_tvalid),
    .m_axis_tready (m_axis_psi_tready),
    .m_axis_tdata  (m_axis_psi_tdata)
);

endmodule
