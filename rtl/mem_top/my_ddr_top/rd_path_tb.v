//=============================================================================
// 文件名: rd_path_tb.v
// 描述: 读通路单元测试平台（decoder_rdcmd → read_driver → pack → 假 rdddr_fifo → 假 DDR）
//
//   put_cmd ──CMD2(64b)──> ┌──────────────┐ 突发 ┌────────────┐  AXI  ┌───────────────┐
//                          │decoder_wrcmd │─────>│read_driver │──────>│ axi4_slave_mem│
//                          └──────────────┘      └─────┬──────┘       │  (假 DDR)     │
//                                                       │ 拍流         └───────────────┘
//                                                  dma_app_pack
//                                                        │ 512b
//                                                  sync_fifo_axis  ← 假 rdddr_fifo
//                                                        │
//                                              tb 排空并逐字校验
//                                                        │ data_count
//                                                        └──► read_driver.rdddr_count
//
// 检查:
//   [1] 假 DDR 协议断言 err_cnt == 0
//   [2] 读回数据逐字节 == exp_byte(地址)
//   [3] rd_resp_err == 0
//
// 数据模式: 字节值 = f(绝对地址) = a[7:0]^a[15:8]^a[23:16]^a[31:24]^8'h5A
//
// 编译运行（工程根目录）:
//   iverilog -g2005 -I rtl/define -o sim/rd_path_tb.vvp -s rd_path_tb \
//     rtl/mem_top/my_ddr_top/decoder_wrcmd.v \
//     rtl/mem_top/my_ddr_top/read_driver.v \
//     rtl/mem_top/my_ddr_top/dma_app_pack.v \
//     rtl/common/sync_fifo_axis.v \
//     vivado_tb/mem_top/axi4_slave_mem.v \
//     rtl/mem_top/my_ddr_top/rd_path_tb.v
//   vvp sim/rd_path_tb.vvp
//=============================================================================
`timescale 1ns / 1ps
`include "all_defines.vh"

module rd_path_tb;

    localparam CLK_PERIOD = 5;
    localparam AXIW       = `DMA_AXI_SELECTED_DATA_WIDTH;
    localparam AXI_BYTES  = AXIW / 8;
    localparam APP_BYTES  = 64;

    localparam AXI_ADDR_WIDTH = 32;
    localparam AXI_ID_WIDTH   = 4;
    localparam AXI_STRB_WIDTH = AXIW / 8;
    localparam RDDDR_DEPTH    = 256;

    reg clk = 1'b0;
    always #(CLK_PERIOD/2) clk = ~clk;
    reg rst_n;

    initial begin
        $dumpfile("sim/rd_path_tb.vcd");
        $dumpvars(0, rd_path_tb);
    end

    //------------------------------------------------------------------
    // decoder 侧
    //------------------------------------------------------------------
    reg  [63:0] cmd_data;
    reg         cmd_valid;
    wire        cmd_s_tready;
    wire [31:0] dec_addr;
    wire [7:0]  dec_len;
    wire        dec_valid;
    wire        dec_tready;
    wire [3:0]  dec_err;

    //------------------------------------------------------------------
    // read_driver 侧
    //------------------------------------------------------------------
    wire [AXIW-1:0] rdr_beat_tdata;
    wire            rdr_beat_tvalid;
    wire            rdr_beat_tready;
    wire            rd_resp_err;

    reg             write_busy;

    // AXI
    wire [AXI_ID_WIDTH-1:0]   m_axi_arid;
    wire [AXI_ADDR_WIDTH-1:0] m_axi_araddr;
    wire [7:0]                m_axi_arlen;
    wire [2:0]                m_axi_arsize;
    wire [1:0]                m_axi_arburst;
    wire                      m_axi_arlock;
    wire [3:0]                m_axi_arcache;
    wire [2:0]                m_axi_arprot;
    wire [3:0]                m_axi_arqos;
    wire                      m_axi_arvalid;
    wire                      m_axi_arready;
    wire [AXI_ID_WIDTH-1:0]   m_axi_rid;
    wire [AXIW-1:0]           m_axi_rdata;
    wire [1:0]                m_axi_rresp;
    wire                      m_axi_rlast;
    wire                      m_axi_rvalid;
    wire                      m_axi_rready;

    wire [31:0] slv_err_cnt, slv_wr_bursts, slv_rd_bursts;

    //------------------------------------------------------------------
    // pack 输出 → 假 rdddr_fifo
    //------------------------------------------------------------------
    wire [511:0] pk_app_tdata;
    wire         pk_app_tvalid;
    wire         pk_app_tready;

    wire [8:0]   fake_fifo_count;      // depth 256 → CNT_W=9

    wire         fifo_m_tvalid;
    reg          fifo_m_tready;
    wire [511:0] fifo_m_tdata;

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
    // read_driver
    //------------------------------------------------------------------
    read_driver #(
        .AXI_DATA_WIDTH   (AXIW),
        .AXI_ADDR_WIDTH   (AXI_ADDR_WIDTH),
        .AXI_ID_WIDTH     (AXI_ID_WIDTH),
        .RDDDR_FIFO_DEPTH (RDDDR_DEPTH)
    ) u_rd (
        .clk           (clk),
        .rst_n         (rst_n),
        .burst_addr    (dec_addr),
        .burst_len     (dec_len),
        .cmd_valid     (dec_valid),
        .cmd_tready    (dec_tready),
        .m_beat_tdata  (rdr_beat_tdata),
        .m_beat_tvalid (rdr_beat_tvalid),
        .m_beat_tready (rdr_beat_tready),
        .rdddr_count   ({2'd0, fake_fifo_count}),
        .write_busy    (write_busy),
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
        .m_axi_rready  (m_axi_rready),
        .rd_resp_err   (rd_resp_err)
    );

    //------------------------------------------------------------------
    // AXI 拍 → app 拍
    //------------------------------------------------------------------
    dma_app_pack #(
        .AXI_DATA_WIDTH (AXIW),
        .APP_DATA_WIDTH (`DMA_APP_DATA_WIDTH)
    ) u_pack (
        .clk          (clk),
        .rst_n        (rst_n),
        .s_beat_tdata (rdr_beat_tdata),
        .s_beat_tvalid(rdr_beat_tvalid),
        .s_beat_tready(rdr_beat_tready),
        .m_app_tdata  (pk_app_tdata),
        .m_app_tvalid (pk_app_tvalid),
        .m_app_tready (pk_app_tready)
    );

    //------------------------------------------------------------------
    // 假 rdddr_fifo（真工程里是 axis_fifo_512x256_np 异步；这里用同步的顶上）
    //------------------------------------------------------------------
    sync_fifo_axis #(
        .WIDTH(512), .DEPTH(RDDDR_DEPTH),
        .PTR_W(8), .CNT_W(9), .RAM_STYLE("block")
    ) u_fake_rdddr_fifo (
        .clk(clk), .rst_n(rst_n),
        .s_axis_tvalid(pk_app_tvalid), .s_axis_tready(pk_app_tready), .s_axis_tdata(pk_app_tdata),
        .m_axis_tvalid(fifo_m_tvalid), .m_axis_tready(fifo_m_tready), .m_axis_tdata(fifo_m_tdata),
        .data_count(fake_fifo_count)
    );


    //------------------------------------------------------------------
    // 假 DDR
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
        .awid         (4'd0),         .awaddr  (32'd0),
        .awlen        (8'd0),         .awsize  (3'd0),
        .awburst      (2'd0),         .awlock  (1'b0),
        .awcache      (4'd0),         .awprot  (3'd0),
        .awqos        (4'd0),         .awvalid (1'b0),
        .awready      (),
        .wdata        ({AXIW{1'b0}}), .wstrb   ({AXI_STRB_WIDTH{1'b0}}),
        .wlast        (1'b0),         .wvalid  (1'b0),
        .wready       (),
        .bid          (),             .bresp   (),
        .bvalid       (),             .bready  (1'b0),
        .arid         (m_axi_arid),   .araddr  (m_axi_araddr),
        .arlen        (m_axi_arlen),  .arsize  (m_axi_arsize),
        .arburst      (m_axi_arburst),.arlock  (m_axi_arlock),
        .arcache      (m_axi_arcache),.arprot  (m_axi_arprot),
        .arqos        (m_axi_arqos),  .arvalid (m_axi_arvalid),
        .arready      (m_axi_arready),
        .rid          (m_axi_rid),    .rdata   (m_axi_rdata),
        .rresp        (m_axi_rresp),  .rlast   (m_axi_rlast),
        .rvalid       (m_axi_rvalid), .rready  (m_axi_rready),
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

    function [511:0] gen_word;
        input [31:0] base;
        integer k;
        reg [511:0] w;
        begin
            w = 512'd0;
            for (k = 0; k < APP_BYTES; k = k + 1)
                w[k*8 +: 8] = exp_byte(base + k);
            gen_word = w;
        end
    endfunction

    //------------------------------------------------------------------
    // 计数与校验
    //------------------------------------------------------------------
    integer err_cnt;
    reg [31:0] chk_addr;
    reg [31:0] chk_words_left;
    integer    chk_beats;
    integer    rd_words;

    // 排空假 fifo 并逐字校验
    reg [511:0] exp_word;

    always @(posedge clk) begin
        exp_word = gen_word(chk_addr);
        if (fifo_m_tvalid && fifo_m_tready) begin
            rd_words = rd_words + 1;
            if (chk_words_left != 0) begin
                if (fifo_m_tdata !== exp_word) begin
                    $display("[FAIL] 读回不符 addr=0x%08x got=%h exp=%h",
                             chk_addr, fifo_m_tdata[31:0], exp_word[31:0]);
                    err_cnt = err_cnt + 1;
                end
                chk_addr       = chk_addr + APP_BYTES;
                chk_words_left = chk_words_left - 1;
            end
            else begin
                $display("[FAIL] 多收到一拍读数据（预期已排空）");
                err_cnt = err_cnt + 1;
            end
        end
    end

    //------------------------------------------------------------------
    // 驱动
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

    task run_cmd;
        input [31:0] addr;
        input [31:0] bytes;
        input [127:0] tag;
        integer g;
        begin
            $display("[%0t] >>> %0s addr=0x%08x bytes=%0d", $time, tag, addr, bytes);
            chk_addr       = addr;
            chk_words_left = bytes / APP_BYTES;

            put_cmd(addr, bytes);

            // 等读回排空
            for (g = 0; (g < 200000) && (chk_words_left != 0); g = g + 1)
                @(posedge clk);
            repeat (10) @(posedge clk);

            if (chk_words_left != 0) begin
                $display("[FAIL] %0s 读回未排空，还剩 %0d 字", tag, chk_words_left);
                err_cnt = err_cnt + 1;
            end
            else begin
                $display("  [OK]  %0s 0x%08x + %0d 字节 读回一致", tag, addr, bytes);
            end
        end
    endtask

    //------------------------------------------------------------------
    // 预填假 DDR
    //------------------------------------------------------------------
    integer i;
    task preload_ddr;
        input [31:0] base;
        input [31:0] bytes;
        begin
            for (i = 0; i < bytes; i = i + 1)
                u_ddr.mem[base + i] = exp_byte(base + i);
        end
    endtask

    //------------------------------------------------------------------
    // 主激励
    //------------------------------------------------------------------
    initial begin
        rst_n        = 1'b0;
        cmd_valid    = 1'b0;
        cmd_data     = 64'd0;
        write_busy   = 1'b0;
        fifo_m_tready= 1'b1;
        err_cnt      = 0;
        rd_words     = 0;
        chk_addr     = 0;
        chk_words_left = 0;

        repeat (8) @(posedge clk);
        rst_n = 1'b1;
        repeat (4) @(posedge clk);

        $display("========================================================");
        $display(" rd_path_tb   位宽=%0d  AXI_BYTES=%0d", AXIW, AXI_BYTES);
        $display("========================================================");

        // 预填测试区（假 DDR 是逐字节存储，直接按地址写图案）
        preload_ddr(32'h0010_0000, 32'd1024);     // C1 / WBUSY
        preload_ddr(32'h0013_0F00, 32'd1024);     // C3（跨 4KB，到 0x131100）
        preload_ddr(32'h0012_2000, 32'd4096);     // C2c
        preload_ddr(32'h0012_4000, 32'd8192);     // C2d

        run_cmd(32'h0010_0000, 32'd512,   "C1   ");   // 1 条突发
        run_cmd(32'h0013_0F00, 32'd512,   "C3   ");   // ★ 跨 4KB → 2 条
        run_cmd(32'h0012_2000, 32'd4096,  "C2c  ");   // 正好 4KB → 1 条
        run_cmd(32'h0012_4000, 32'd8192,  "C2d  ");   // 8192 → 2 条

        // write_busy 拉高期间不应发起读（只验不死锁、数据仍对）
        $display("");
        $display("########## write_busy 拉高期间 ##########");
        write_busy = 1'b1;
        repeat (50) @(posedge clk);
        write_busy = 1'b0;
        run_cmd(32'h0010_0000, 32'd256,   "WBUSY");

        $display("");
        $display("========================================================");
        $display(" TB 检查错误数    : %0d", err_cnt);
        $display(" 假 DDR 断言错误数: %0d", slv_err_cnt);
        $display(" 读突发总数       : %0d", slv_rd_bursts);
        $display(" 读响应错误       : %0d", rd_resp_err);
        if ((err_cnt == 0) && (slv_err_cnt == 0) && (rd_resp_err == 1'b0))
            $display("  ***** RD_PATH_TB_PASS *****");
        else
            $display("  ***** RD_PATH_TB_FAIL *****");
        $display("========================================================");
        $finish;
    end

    //------------------------------------------------------------------
    // 全局超时
    //------------------------------------------------------------------
    initial begin
        #500000;
        $display("[FAIL] 全局超时（疑似死锁）");
        $display("  ***** RD_PATH_TB_FAIL *****");
        $finish;
    end

endmodule
