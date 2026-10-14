//=============================================================================
// 文件名: wr_path_tb.v
// 描述: 写通路单元测试平台（decoder_wrcmd → write_driver → 假 DDR）
//
//   put_cmd ──CMD2(64b)──> ┌──────────────┐ 突发 ┌────────────┐  AXI  ┌───────────────┐
//   put_data ──数据拍────> │decoder_wrcmd │─────>│write_driver│──────>│ axi4_slave_mem│
//                          └──────────────┘      └────────────┘       │ (假 DDR+断言) │
//                                                                     └───────────────┘
//                                                                      写完回读 mem[]
//
// 检查:
//   [1] 假 DDR 协议断言 err_cnt == 0（4KB/突发长度/size/burst/last/范围）
//   [2] DDR 内容逐字节 == exp_byte(地址)      ← 数据搬对了
//   [3] 突发条数与期望一致
//
// 数据模式: 字节值 = f(绝对地址) = a[7:0]^a[15:8]^a[23:16]^a[31:24]^8'h5A
//
// 编译运行（工程根目录）:
//   iverilog -g2005 -I rtl/define -o sim/wr_path_tb.vvp -s wr_path_tb \
//     rtl/mem_top/my_ddr_top/decoder_wrcmd.v \
//     rtl/mem_top/my_ddr_top/write_driver.v \
//     rtl/common/sync_fifo_axis.v \
//     vivado_tb/mem_top/axi4_slave_mem.v \
//     rtl/mem_top/my_ddr_top/wr_path_tb.v
//   vvp sim/wr_path_tb.vvp
//=============================================================================
`timescale 1ns / 1ps
`include "all_defines.vh"

module wr_path_tb;

    //------------------------------------------------------------------
    // 参数
    //------------------------------------------------------------------
    localparam CLK_PERIOD = 5;                       // ui_clk 200MHz
    localparam AXIW       = `DMA_AXI_SELECTED_DATA_WIDTH;
    localparam AXI_BYTES  = AXIW / 8;                // 512 位宽 = 64
    localparam APP_BYTES  = 64;                      // 一拍 64 字节

    localparam AXI_ADDR_WIDTH = 32;
    localparam AXI_ID_WIDTH   = 4;
    localparam AXI_STRB_WIDTH = AXIW / 8;

    //------------------------------------------------------------------
    // 时钟与复位
    //------------------------------------------------------------------
    reg clk = 1'b0;
    always #(CLK_PERIOD/2) clk = ~clk;
    reg rst_n;

    //------------------------------------------------------------------
    // 波形
    //------------------------------------------------------------------
    initial begin
        $dumpfile("sim/wr_path_tb.vcd");
        $dumpvars(0, wr_path_tb);
    end

    //------------------------------------------------------------------
    // decoder 侧
    //------------------------------------------------------------------
    reg  [63:0] cmd_data;
    reg         cmd_valid;
    wire        cmd_s_tready;        // decoder → 上游
    wire [31:0] dec_addr;
    wire [7:0]  dec_len;
    wire        dec_valid;
    wire        dec_tready;
    wire [3:0]  dec_err;

    //------------------------------------------------------------------
    // write_driver 侧
    //------------------------------------------------------------------
    reg  [AXIW-1:0] s_beat_tdata;
    reg             s_beat_tvalid;
    wire            s_beat_tready;
    wire            wr_busy, wr_resp_err, wr_starve_err;

    // AXI
    wire [AXI_ID_WIDTH-1:0]   m_axi_awid;
    wire [AXI_ADDR_WIDTH-1:0] m_axi_awaddr;
    wire [7:0]                m_axi_awlen;
    wire [2:0]                m_axi_awsize;
    wire [1:0]                m_axi_awburst;
    wire                      m_axi_awlock;
    wire [3:0]                m_axi_awcache;
    wire [2:0]                m_axi_awprot;
    wire [3:0]                m_axi_awqos;
    wire                      m_axi_awvalid;
    wire                      m_axi_awready;
    wire [AXIW-1:0]           m_axi_wdata;
    wire [AXI_STRB_WIDTH-1:0] m_axi_wstrb;
    wire                      m_axi_wlast;
    wire                      m_axi_wvalid;
    wire                      m_axi_wready;
    wire [AXI_ID_WIDTH-1:0]   m_axi_bid;
    wire [1:0]                m_axi_bresp;
    wire                      m_axi_bvalid;
    wire                      m_axi_bready;

    wire [31:0] slv_err_cnt, slv_wr_bursts, slv_rd_bursts;

    //------------------------------------------------------------------
    // decoder
    //------------------------------------------------------------------
    decoder_wrcmd #(
        .AXI_DATA_WIDTH  (AXIW),
        .MAX_BURST_BEATS (`DMA_AXI_MAX_BURST_BEATS),
        .MEM_BYTES       (32'h0020_0000)
    ) u_dec (
        .clk          (clk),
        .rst_n        (rst_n),
        .cmd_data     (cmd_data),
        .cmd_valid    (cmd_valid),
        .cmd_tready   (dec_tready),
        .s_axis_tready(cmd_s_tready),
        .cmd_address  (dec_addr),
        .burst_len    (dec_len),
        .cmd_tvalid   (dec_valid),
        .cmd_err      (dec_err)
    );

    //------------------------------------------------------------------
    // write_driver
    //------------------------------------------------------------------
    write_driver #(
        .AXI_DATA_WIDTH     (AXIW),
        .AXI_ADDR_WIDTH     (AXI_ADDR_WIDTH),
        .AXI_ID_WIDTH       (AXI_ID_WIDTH),
        .AXI_STRB_WIDTH     (AXI_STRB_WIDTH),
        .WR_BEAT_FIFO_DEPTH (512)
    ) u_wr (
        .clk           (clk),
        .rst_n         (rst_n),
        .burst_addr    (dec_addr),
        .burst_len     (dec_len),
        .cmd_valid     (dec_valid),
        .cmd_tready    (dec_tready),
        .s_beat_tdata  (s_beat_tdata),
        .s_beat_tvalid (s_beat_tvalid),
        .s_beat_tready (s_beat_tready),
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
        .m_axi_bready  (m_axi_bready),
        .write_busy    (wr_busy),
        .wr_resp_err   (wr_resp_err),
        .wr_starve_err (wr_starve_err)
    );

    //------------------------------------------------------------------
    // 假 DDR + 协议断言器
    //------------------------------------------------------------------
    axi4_slave_mem #(
        .AXI_ADDR_WIDTH  (AXI_ADDR_WIDTH),
        .AXI_DATA_WIDTH  (AXIW),
        .AXI_ID_WIDTH    (AXI_ID_WIDTH),
        .AXI_STRB_WIDTH  (AXI_STRB_WIDTH),
        .MEM_BYTES       (2*1024*1024),
        .MAX_BURST_BEATS (256)
    ) u_ddr (
        .clk          (clk),
        .rst_n        (rst_n),
        .awid         (m_axi_awid),   .awaddr  (m_axi_awaddr),
        .awlen        (m_axi_awlen),  .awsize  (m_axi_awsize),
        .awburst      (m_axi_awburst),.awlock  (m_axi_awlock),
        .awcache      (m_axi_awcache),.awprot  (m_axi_awprot),
        .awqos        (m_axi_awqos),  .awvalid (m_axi_awvalid),
        .awready      (m_axi_awready),
        .wdata        (m_axi_wdata),  .wstrb   (m_axi_wstrb),
        .wlast        (m_axi_wlast),  .wvalid  (m_axi_wvalid),
        .wready       (m_axi_wready),
        .bid          (m_axi_bid),    .bresp   (m_axi_bresp),
        .bvalid       (m_axi_bvalid), .bready  (m_axi_bready),
        .arid         (4'd0),         .araddr  (32'd0),
        .arlen        (8'd0),         .arsize  (3'd0),
        .arburst      (2'd0),         .arlock  (1'b0),
        .arcache      (4'd0),         .arprot  (3'd0),
        .arqos        (4'd0),         .arvalid (1'b0),
        .arready      (),
        .rid          (),             .rdata   (),
        .rresp        (),             .rlast   (),
        .rvalid       (),             .rready  (1'b0),
        .rdy_delay    (8'd0),
        .err_cnt      (slv_err_cnt),
        .wr_burst_cnt (slv_wr_bursts),
        .rd_burst_cnt (slv_rd_bursts)
    );

    //------------------------------------------------------------------
    // 数据模式
    //------------------------------------------------------------------
    function [7:0] exp_byte;
        input [31:0] a;
        begin
            exp_byte = a[7:0] ^ a[15:8] ^ a[23:16] ^ a[31:24] ^ 8'h5A;
        end
    endfunction

    function [AXIW-1:0] gen_beat;
        input [31:0] base;
        integer k;
        reg [AXIW-1:0] w;
        begin
            w = {AXIW{1'b0}};
            for (k = 0; k < AXI_BYTES; k = k + 1)
                w[k*8 +: 8] = exp_byte(base + k);
            gen_beat = w;
        end
    endfunction

    //------------------------------------------------------------------
    // 计数
    //------------------------------------------------------------------
    integer err_cnt;
    integer tot_bursts;

    //------------------------------------------------------------------
    // 驱动: 发一条命令
    //------------------------------------------------------------------
    task put_cmd;
        input [31:0] addr;
        input [31:0] bytes;
        begin
            @(posedge clk); #1;
            cmd_valid = 1'b1;
            cmd_data  = {addr, bytes};
            @(posedge clk);
            while (cmd_s_tready !== 1'b1) @(posedge clk);
            #1;
            cmd_valid = 1'b0;
            cmd_data  = 64'd0;
        end
    endtask

    //------------------------------------------------------------------
    // 驱动: 推一块数据（按地址生成图案）
    //------------------------------------------------------------------
    task put_data;
        input [31:0] base;
        input [31:0] bytes;
        integer k;
        reg [31:0] a;
        begin
            a = base;
            for (k = 0; k < (bytes / APP_BYTES); k = k + 1) begin
                @(posedge clk); #1;
                s_beat_tvalid = 1'b1;
                s_beat_tdata  = gen_beat(a);
                @(posedge clk);
                while (s_beat_tready !== 1'b1) @(posedge clk);
                #1;
                s_beat_tvalid = 1'b0;
                s_beat_tdata  = {AXIW{1'b0}};
                a = a + APP_BYTES;
            end
        end
    endtask

    //------------------------------------------------------------------
    // 等写通路空下来（wr_busy 落下 + FIFO 排空）
    //------------------------------------------------------------------
    task wait_idle;
        integer g;
        begin
            for (g = 0; (g < 200000) && (wr_busy === 1'b1); g = g + 1)
                @(posedge clk);
            repeat (20) @(posedge clk);
        end
    endtask

    //------------------------------------------------------------------
    // 回读 DDR 逐字节校验
    //------------------------------------------------------------------
    task check_ddr;
        input [31:0] base;
        input [31:0] bytes;
        input [127:0] tag;
        integer i;
        reg [7:0] expb;
        begin
            for (i = 0; i < bytes; i = i + 1) begin
                expb = exp_byte(base + i);
                if (u_ddr.mem[base + i] !== expb) begin
                    if (i < 4) begin
                        $display("[FAIL] %0s DDR[0x%08x] 期望 %02x 实得 %02x",
                                 tag, base + i, expb, u_ddr.mem[base + i]);
                        err_cnt = err_cnt + 1;
                    end
                end
            end
            $display("  [OK]  %0s 0x%08x + %0d 字节 回读一致", tag, base, bytes);
        end
    endtask

    //------------------------------------------------------------------
    // 一条完整命令: 清账 → 发命令 → 推数据 → 等 → 校验
    //------------------------------------------------------------------
    task run_cmd;
        input [31:0] addr;
        input [31:0] bytes;
        input [127:0] tag;
        integer b0;
        begin
            b0 = slv_wr_bursts;
            $display("[%0t] >>> %0s addr=0x%08x bytes=%0d", $time, tag, addr, bytes);

            put_cmd(addr, bytes);
            put_data(addr, bytes);
            wait_idle;

            tot_bursts = tot_bursts + (slv_wr_bursts - b0);
            check_ddr(addr, bytes, tag);

            if (dec_err !== 4'd0) begin
                $display("[FAIL] %0s cmd_err=%b 非 0", tag, dec_err);
                err_cnt = err_cnt + 1;
            end
        end
    endtask

    //------------------------------------------------------------------
    // 主激励
    //------------------------------------------------------------------
    initial begin
        rst_n        = 1'b0;
        cmd_valid    = 1'b0;
        cmd_data     = 64'd0;
        s_beat_tvalid= 1'b0;
        s_beat_tdata = {AXIW{1'b0}};
        err_cnt      = 0;
        tot_bursts   = 0;

        repeat (8) @(posedge clk);
        rst_n = 1'b1;
        repeat (4) @(posedge clk);

        $display("========================================================");
        $display(" wr_path_tb   位宽=%0d  AXI_BYTES=%0d", AXIW, AXI_BYTES);
        $display("========================================================");

        run_cmd(32'h0010_0000, 32'd512,   "C1   ");   // 1 条突发
        run_cmd(32'h0013_0F00, 32'd512,   "C3   ");   // ★ 跨 4KB → 2 条
        run_cmd(32'h0012_2000, 32'd4096,  "C2c  ");   // 正好 4KB → 1 条
        run_cmd(32'h0012_4000, 32'd8192,  "C2d  ");   // 8192 → 2 条
        run_cmd(32'h0012_0000, 32'd64,    "C2a  ");   // 最小 1 拍

        //--------------------------------------------------------------
        // 错误注入: 强制从机回 SLVERR，验 wr_resp_err 粘滞
        //   ★ 必须放最后 —— 粘滞标志置起后不会自清
        //--------------------------------------------------------------
        $display("");
        $display("########## 错误注入: bresp = SLVERR ##########");
        if (wr_resp_err !== 1'b0) begin
            $display("[FAIL] 注入前 wr_resp_err 已经是 1");
            err_cnt = err_cnt + 1;
        end
        force u_wr.m_axi_bresp = 2'b10;
        run_cmd(32'h0010_1000, 32'd512, "E-RESP");
        release u_wr.m_axi_bresp;

        if (wr_resp_err !== 1'b1) begin
            $display("[FAIL] 注入后 wr_resp_err 应为 1，实得 %0d", wr_resp_err);
            err_cnt = err_cnt + 1;
        end
        else begin
            $display("  [OK]  wr_resp_err 已粘滞置起");
        end

        $display("");
        $display("========================================================");
        $display(" TB 检查错误数    : %0d", err_cnt);
        $display(" 假 DDR 断言错误数: %0d", slv_err_cnt);
        $display(" 写突发总数       : %0d", slv_wr_bursts);
        $display(" 金丝雀(断粮)     : %0d", wr_starve_err);
        $display(" 写响应错误(注入) : %0d", wr_resp_err);
        if ((err_cnt == 0) && (slv_err_cnt == 0) && (wr_starve_err == 1'b0))
            $display("  ***** WR_PATH_TB_PASS *****");
        else
            $display("  ***** WR_PATH_TB_FAIL *****");
        $display("========================================================");
        $finish;
    end

    //------------------------------------------------------------------
    // 全局超时
    //------------------------------------------------------------------
    initial begin
        #500000;
        $display("[FAIL] 全局超时（疑似死锁）");
        $display("  ***** WR_PATH_TB_FAIL *****");
        $finish;
    end

endmodule
