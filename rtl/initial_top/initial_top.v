`timescale 1ns / 1ps
`include "all_defines.vh"

module initial_top (
    // ==================== 系统接口 ====================
    input  wire        clk,          // 时钟（接 ui_clk）
    input  wire        rst_n,        // 复位（低电平有效）

    // ==================== PS 数据输入 ====================
    input  wire        s_axis_ps_tvalid,  // PS 数据输入有效
    input  wire [511:0] s_axis_ps_tdata,  // PS 数据输入 (512位，006 直取版)
    output wire        s_axis_ps_tready,  // PS 数据就绪

    // ==================== 读指令分发 ====================
    output wire        m_axis_rdcmd_tvalid,   // 读指令分发有效
    output wire [`RD_SM_CMD_WIDTH-1:0]m_axis_rdcmd_tdata,      // 读指令分发数据 (160位)
    input  wire        m_axis_rdcmd_tready,   // 读指令分发就绪

    // ==================== 写指令分发 ====================
    output wire        m_axis_wrcmd_tvalid,   // 写指令分发有效
    output wire [`WR_SM_CMD_WIDTH-1:0]m_axis_wrcmd_tdata,      // 写指令分发数据 (160位)
    input  wire        m_axis_wrcmd_tready,   // 写指令分发就绪

    // ==================== PS 数据整合后输出端口 ====================
    output wire        m_axis_psi_tvalid,   // 整合后数据输出有效
    output wire [511:0]m_axis_psi_tdata,    // 整合后数据输出 (512位)，如权重、偏置等模型参数或输入特征图等数据
    input  wire        m_axis_psi_tready,   // 整合后数据输出就绪

    // ==================== 加载模块基地址 RAM 访存端口 ====================
    input  wire                         load_ba_rd,         // 基地址读使能
    input  wire [10:0]                  load_ba_ptr,        // 基地址读指针（11位寻址）
    output wire [31:0]                  load_ba_data,       // 基地址读数据

    // ==================== 存储模块基地址 RAM 访存端口 ====================
    input  wire                         store_ba_rd,         // 基地址读使能
    input  wire [10:0]                  store_ba_ptr,        // 基地址读指针（11位寻址）
    output wire [31:0]                  store_ba_data       // 基地址读数据
);
// ====================================
// 和ins_load的连接信号
// ====================================
wire fetch_req;
wire fetch_ack;
wire fetch_ren;
wire [9:0] fetch_addr;
wire [`RD_SM_CMD_WIDTH-1:0] fetch_ins;
wire sync_ptr_reset;
wire [9:0] fm_entry_addr;
wire [9:0] fm_last_addr;


initial_module u_initial_module (
    // 系统
    .clk                (clk),
    .rst_n              (rst_n),
    
    // PS数据输入
    .s_axis_ps_tvalid   (s_axis_ps_tvalid),
    .s_axis_ps_tdata    (s_axis_ps_tdata),
    .s_axis_ps_tready   (s_axis_ps_tready),
    
    // 取指接口 (连接ins_load)
    .fetch_req          (fetch_req),
    .fetch_ack          (fetch_ack),
    .fetch_ren          (fetch_ren),
    .fetch_addr         (fetch_addr),
    .fetch_ins          (fetch_ins),
    .sync_ptr_reset     (sync_ptr_reset),
    .fm_entry_addr      (fm_entry_addr),
    .fm_last_addr       (fm_last_addr),
    
    // 基地址RAM
    .load_ba_rd         (load_ba_rd),
    .load_ba_ptr        (load_ba_ptr),
    .load_ba_data       (load_ba_data),

    .store_ba_rd        (store_ba_rd),
    .store_ba_ptr       (store_ba_ptr),
    .store_ba_data      (store_ba_data),
    
    // PS数据整合输出
    .m_axis_psi_tvalid  (m_axis_psi_tvalid),
    .m_axis_psi_tdata   (m_axis_psi_tdata),
    .m_axis_psi_tready  (m_axis_psi_tready)
);


ins_load u_ins_load (
    // 系统
    .clk                    (clk),
    .rst_n                  (rst_n),
    
    // 取指接口 (连接initial_module)
    .fetch_req              (fetch_req),
    .fetch_ack              (fetch_ack),
    .fetch_ren              (fetch_ren),
    .fetch_addr             (fetch_addr),
    .fetch_ins              (fetch_ins),
    .sync_ptr_reset         (sync_ptr_reset),
    .fm_entry_addr          (fm_entry_addr),
    .fm_last_addr           (fm_last_addr),
    
    // 写指令分发
    .m_axis_wrcmd_tvalid    (m_axis_wrcmd_tvalid),
    .m_axis_wrcmd_tdata     (m_axis_wrcmd_tdata),
    .m_axis_wrcmd_tready    (m_axis_wrcmd_tready),
    
    // 读指令分发
    .m_axis_rdcmd_tvalid    (m_axis_rdcmd_tvalid),
    .m_axis_rdcmd_tdata     (m_axis_rdcmd_tdata),
    .m_axis_rdcmd_tready    (m_axis_rdcmd_tready)
);


endmodule
