`timescale 1ns / 1ps
`include "all_defines.vh"

module store_top (
    // ==================== 系统接口 ====================
    input  wire                         clk,                // 时钟（ui_clk）
    input  wire                         rst_n,              // 复位

    // ==================== 写指令输入端口 ====================
    input  wire                         s_axis_wrcmd_tvalid,    // 写指令获取端口，每执行完一条写指令再拉高ready获取下一条指令
    input  wire [`WR_SM_CMD_WIDTH-1:0]  s_axis_wrcmd_tdata,     // 160位写指令数据
    output wire                         s_axis_wrcmd_tready,    // 写指令就绪

    // ==================== DMA二级写命令输出端口 ====================
    output wire                         m_axis_wrcmd2_tvalid,   // 给DMA的2级写命令，由写指令衍生，只携带存储所需要的信息和指针
    output wire [`CMD2_CMD_WIDTH-1:0]   m_axis_wrcmd2_tdata,    // 80位二级写命令数据
    input  wire                         m_axis_wrcmd2_tready,   // 二级写命令就绪

    // ==================== 向DDR写数据输出端口 ====================
    output wire                         m_axis_wrddr_tvalid,    // 要写往DDR的数据
    output wire [511:0]                 m_axis_wrddr_tdata,     // 512位DDR写数据
    input  wire                         m_axis_wrddr_tready,    // DDR写数据就绪

    // ==================== 循环padding数据缓存输出端口 ====================
    output wire                         m_axis_circular_tvalid,     // 写往循环padding缓存的数据，供下一层使用
    output wire [511:0]                 m_axis_circular_tdata,           // 512位padding数据
    input  wire                         m_axis_circular_tready, 

    // ==================== 循环padding写指令控制字段输出端口 ====================
    output wire                         m_axis_wrcmd_tvalid,
    output wire [103:0]                 m_axis_wrcmd_tdata,
    input  wire                         m_axis_wrcmd_tready,

    // ==================== 来自PS的数据写入端口 ====================
    input  wire                         s_axis_ps_tvalid,       // 经过整合后的PS数据，可能是权重等模型参数，也可能是原始输入特征图
    input  wire [511:0]                 s_axis_ps_tdata,        // 512位PS数据
    output wire                         s_axis_ps_tready,       // PS数据就绪

    // ==================== 计算时钟域时钟 ====================
    input  wire                         cal_clk,                // 以下端口异步FIFO的写时钟

    // ==================== 来自激活的数据写入端口 ====================
    input  wire                         s_axis_act_tvalid,      // 激活算子的写回数据输入,可能是卷积、split等操作层的输出
    input  wire [511:0]                 s_axis_act_tdata,       // 512位激活数据
    output wire                         s_axis_act_tready,      // 激活数据就绪

    // ==================== 来自池化的数据写入端口 ====================
    input  wire                         s_axis_pooling_tvalid,  // 池化算子的写回数据输入
    input  wire [511:0]                 s_axis_pooling_tdata,   // 512位池化数据
    output wire                         s_axis_pooling_tready,  // 池化数据就绪

    // ==================== 来自残差加的数据输入端口 ====================
    input  wire                         s_axis_res_tvalid,      // 残差模块的写回数据输入，可能对应残差加、上采样、通道拼接等操作层的计算结果
    input  wire [511:0]                 s_axis_res_tdata,       // 512位残差数据
    output wire                         s_axis_res_tready,      // 残差数据就绪

    // ==================== 特征图全部写回指示（单比特跨时钟域交互） ====================
    output wire                         store_done_req,         // 特征图存储完成请求
    input  wire                         store_done_ack,         // 特征图存储完成响应
    output wire                         store_fm_done,

    // ==================== 写特征图基地址端口 ====================
    output wire                         store_ba_rden,          // 基地址ram读使能
    output wire [10:0]                  store_ba_rdaddr,        // 基地址ram读指针
    input  wire [31:0]                  store_ba_rddata         // 基地址ram读回的基
);
// 写指令经过同步FIFO后的输出信号
wire                                    wrcmd_fifo_tvalid;
wire [`WR_SM_CMD_WIDTH-1:0]             wrcmd_fifo_tdata;
wire                                    wrcmd_fifo_tready;

// 激活数据FIFO master端信号（读时钟域）
wire                                    act_fifo_tvalid;
wire [511:0]                            act_fifo_tdata;
wire                                    act_fifo_tready;

// 池化数据FIFO master端信号（读时钟域）
wire                                    pooling_fifo_tvalid;
wire [511:0]                            pooling_fifo_tdata;
wire                                    pooling_fifo_tready;

// 残差数据FIFO master端信号（读时钟域）
wire                                    res_fifo_tvalid;
wire [511:0]                            res_fifo_tdata;
wire                                    res_fifo_tready;

// PS数据FIFO master端信号（读时钟域）
wire                                    ps_fifo_tvalid;
wire [511:0]                            ps_fifo_tdata;
wire                                    ps_fifo_tready;

// 循环padding FIFO slave端信号 (写时钟域)
wire                                    circular_fifo_tvalid;
wire [511:0]                            circular_fifo_tdata;
wire                                    circular_fifo_tready;
// 循环padding写指令控制字段 FIFO slave/master 信号
wire                                    wrcmd_ctrl_fifo_tvalid;
wire [103:0]                            wrcmd_ctrl_fifo_tdata;
wire [103:0]                            wrcmd_ctrl_fifo_s_tdata;
wire [103:0]                            wrcmd_ctrl_fifo_m_tdata;
wire                                    wrcmd_ctrl_fifo_tready;

axis_data_fifo_160_sync u_store_sm_command_fifo (
    .s_axis_aclk    (clk),
    .s_axis_aresetn (rst_n),
    .s_axis_tvalid  (s_axis_wrcmd_tvalid),
    .s_axis_tdata   (s_axis_wrcmd_tdata),
    .s_axis_tready  (s_axis_wrcmd_tready),
    .m_axis_tvalid  (wrcmd_fifo_tvalid),
    .m_axis_tdata   (wrcmd_fifo_tdata),
    .m_axis_tready  (wrcmd_fifo_tready)
);

// -----------------------------------------------------------
// 激活数据异步FIFO
// -----------------------------------------------------------
axis_data_fifo_512_async_pf127 u_act_fifo (
    // 写时钟域（来自cal_clk域的激活数据）
    .s_axis_aclk    (cal_clk),
    .s_axis_aresetn (rst_n),
    .s_axis_tvalid  (s_axis_act_tvalid),
    .s_axis_tdata   (s_axis_act_tdata),
    .s_axis_tready  (s_axis_act_tready),
    // 读时钟域（同步到clk域）
    .m_axis_aclk    (clk),
    .m_axis_tvalid  (act_fifo_tvalid),
    .m_axis_tdata   (act_fifo_tdata),
    .m_axis_tready  (act_fifo_tready)
);

// -----------------------------------------------------------
// 池化数据异步FIFO
// -----------------------------------------------------------
axis_data_fifo_512_async u_pooling_fifo (
    // 写时钟域（来自cal_clk域的池化数据）
    .s_axis_aclk    (cal_clk),
    .s_axis_aresetn (rst_n),
    .s_axis_tvalid  (s_axis_pooling_tvalid),
    .s_axis_tdata   (s_axis_pooling_tdata),
    .s_axis_tready  (s_axis_pooling_tready),
    // 读时钟域（同步到clk域）
    .m_axis_aclk    (clk),
    .m_axis_tvalid  (pooling_fifo_tvalid),
    .m_axis_tdata   (pooling_fifo_tdata),
    .m_axis_tready  (pooling_fifo_tready)
);

// -----------------------------------------------------------
// 残差数据异步FIFO
// -----------------------------------------------------------
axis_data_fifo_512_async u_res_fifo (
    // 写时钟域（来自cal_clk域的残差数据）
    .s_axis_aclk    (cal_clk),
    .s_axis_aresetn (rst_n),
    .s_axis_tvalid  (s_axis_res_tvalid),
    .s_axis_tdata   (s_axis_res_tdata),
    .s_axis_tready  (s_axis_res_tready),
    // 读时钟域（同步到clk域）
    .m_axis_aclk    (clk),
    .m_axis_tvalid  (res_fifo_tvalid),
    .m_axis_tdata   (res_fifo_tdata),
    .m_axis_tready  (res_fifo_tready)
);

// -----------------------------------------------------------
// PS数据同步FIFO
// -----------------------------------------------------------
axis_data_fifo_512_sync u_ps_fifo (
    .s_axis_aclk    (clk),
    .s_axis_aresetn (rst_n),
    .s_axis_tvalid  (s_axis_ps_tvalid),
    .s_axis_tdata   (s_axis_ps_tdata),
    .s_axis_tready  (s_axis_ps_tready),
    .m_axis_tvalid  (ps_fifo_tvalid),
    .m_axis_tdata   (ps_fifo_tdata),
    .m_axis_tready  (ps_fifo_tready)
);


store u_store (
    // 系统接口
    .clk                  (clk),
    .rst_n                (rst_n),

    // 写指令输入端口
    .s_axis_wrcmd_tvalid  (wrcmd_fifo_tvalid),
    .s_axis_wrcmd_tdata   (wrcmd_fifo_tdata),
    .s_axis_wrcmd_tready  (wrcmd_fifo_tready),

    // DMA二级写命令输出端口
    .m_axis_wrcmd2_tvalid (m_axis_wrcmd2_tvalid),
    .m_axis_wrcmd2_tdata  (m_axis_wrcmd2_tdata),
    .m_axis_wrcmd2_tready (m_axis_wrcmd2_tready),
    // 向DDR写数据输出端口
    .m_axis_wrddr_tvalid  (m_axis_wrddr_tvalid),
    .m_axis_wrddr_tdata   (m_axis_wrddr_tdata),
    .m_axis_wrddr_tready  (m_axis_wrddr_tready),
    // 循环padding数据缓存输出端口
    .circular_fifo_tvalid (circular_fifo_tvalid),
    .circular_fifo_tdata  (circular_fifo_tdata),
    .circular_fifo_tready (circular_fifo_tready),
    // 循环padding写指令控制字段输出端口
    .m_axis_wrcmd_tvalid (wrcmd_ctrl_fifo_tvalid),
    .m_axis_wrcmd_tdata  (wrcmd_ctrl_fifo_tdata),
    .m_axis_wrcmd_tready (wrcmd_ctrl_fifo_tready),
    // 来自PS的数据写入端口
    .ps_fifo_tvalid       (ps_fifo_tvalid),
    .ps_fifo_tdata        (ps_fifo_tdata),
    .ps_fifo_tready       (ps_fifo_tready),

    // 来自激活FIFO的数据
    .act_fifo_tvalid      (act_fifo_tvalid),
    .act_fifo_tdata       (act_fifo_tdata),
    .act_fifo_tready      (act_fifo_tready),

    // 来自池化FIFO的数据
    .pooling_fifo_tvalid  (pooling_fifo_tvalid),
    .pooling_fifo_tdata   (pooling_fifo_tdata),
    .pooling_fifo_tready  (pooling_fifo_tready),

    // 来自残差FIFO的数据
    .res_fifo_tvalid      (res_fifo_tvalid),
    .res_fifo_tdata       (res_fifo_tdata),
    .res_fifo_tready      (res_fifo_tready),

    // 特征图全部写回指示
    .store_done_req       (store_done_req),
    .store_done_ack       (store_done_ack),
    .store_fm_done        (store_fm_done),


    // 写特征图基地址
    .store_ba_rden        (store_ba_rden),
    .store_ba_rdaddr      (store_ba_rdaddr),
    .store_ba_rddata      (store_ba_rddata)
);

axis_data_fifo_512_async u_circular_fifo (
    // 写时钟域（clk/ui_clk域，来自store模块的循环padding写回数据）
    .s_axis_aclk    (clk),
    .s_axis_aresetn (rst_n),
    .s_axis_tvalid  (circular_fifo_tvalid),
    .s_axis_tdata   (circular_fifo_tdata),
    .s_axis_tready  (circular_fifo_tready),
    // 读时钟域（同步到clk域）
    .m_axis_aclk    (cal_clk),
    .m_axis_tvalid  (m_axis_circular_tvalid),
    .m_axis_tdata   (m_axis_circular_tdata),
    .m_axis_tready  (m_axis_circular_tready)
);

assign wrcmd_ctrl_fifo_s_tdata = wrcmd_ctrl_fifo_tdata;
assign m_axis_wrcmd_tdata = wrcmd_ctrl_fifo_m_tdata;

axis_data_fifo_104_async u_wrcmd_ctrl_fifo (
    .s_axis_aclk    (clk),
    .s_axis_aresetn (rst_n),
    .s_axis_tvalid  (wrcmd_ctrl_fifo_tvalid),
    .s_axis_tdata   (wrcmd_ctrl_fifo_s_tdata),
    .s_axis_tready  (wrcmd_ctrl_fifo_tready),
    .m_axis_aclk    (cal_clk),
    .m_axis_tvalid  (m_axis_wrcmd_tvalid),
    .m_axis_tdata   (wrcmd_ctrl_fifo_m_tdata),
    .m_axis_tready  (m_axis_wrcmd_tready)
);
endmodule
