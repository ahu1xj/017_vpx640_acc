//=============================================================================
// 文件名: decoder_wrcmd_tb.v
// 描述: decoder_wrcmd 单元测试平台（纯 RTL，无 IP，iverilog 可直接跑）
//
// 原理:
//   喂一条 CMD2 → 收集所有 {cmd_address, burst_len} → 对账
//
// 五条检查:
//   [1] Σ (burst_len+1) × AXI_BYTES  ==  命令的 bytes      ← 不丢不重（最关键）
//   [2] 突发条数 == 期望值                                  ← 切分策略
//   [3] 每条 [addr, addr + 本刀字节) 不跨 4KB 边界
//   [4] 相邻突发地址连续: addr[i+1] == addr[i] + 本刀字节[i]
//   [5] burst_len ≤ MAX_BURST_BEATS-1
//   另: 四条错误命令必须「不产生突发」且 cmd_err 对应位拉高
//
// 用例: 抄 mem_top_tb 的 CASE 1~6（写 16 条 / 读 9 条 = 25 条命令）
//
// 编译运行（工程根目录）:
//   iverilog -g2005 -I rtl/define -o sim/decoder_wrcmd_tb.vvp \
//     -s decoder_wrcmd_tb \
//     rtl/mem_top/my_ddr_top/decoder_wrcmd.v \
//     rtl/mem_top/my_ddr_top/decoder_wrcmd_tb.v
//   vvp sim/decoder_wrcmd_tb.vvp
//
// 换位宽（验 64 位那档）: 加 -DDMA_AXI_DATA_WIDTH_64，期望值见 REWRITE_GUIDE §0
//=============================================================================
`timescale 1ns / 1ps
`include "all_defines.vh"

module decoder_wrcmd_tb;

    //------------------------------------------------------------------
    // 参数
    //------------------------------------------------------------------
    localparam CLK_PERIOD = 5;                    // ui_clk 200MHz
    localparam AXI_BYTES  = `DMA_AXI_SELECTED_DATA_WIDTH / 8;
    localparam SHIFT      = (AXI_BYTES == 64) ? 6 :
                            (AXI_BYTES == 32) ? 5 :
                            (AXI_BYTES == 16) ? 4 : 3;

    // 错误向量：低 3 位非 0 → 对 SHIFT=6/5/4/3 四档都不对齐，与位宽无关
    localparam [31:0] E_LEN_BYTES = 32'd4100;        // 4100 & {63,31,15,7} = 4 ≠ 0
    localparam [31:0] E_ALN_ADDR  = 32'h0010_0004;   // 4    & {63,31,15,7} = 4 ≠ 0

    //------------------------------------------------------------------
    // 期望突发数（REWRITE_GUIDE §0）
    //   512/256/128: 4KB 边界先触发 → 写 81 / 读 74
    //   64:          拍数上限先触发 → 写 149 / 读 143   ★ 这档才验得到上限
    //------------------------------------------------------------------
`ifdef DMA_AXI_DATA_WIDTH_64
    localparam E_C1  = 1;      // 512B
    localparam E_C2a = 1;      // 64B
    localparam E_C2b = 1;      // 256B
    localparam E_C2c = 2;      // 4096B  ← 2048 上限，切两刀
    localparam E_C2d = 4;      // 8192B
    localparam E_C3  = 2;      // 0x130F00 + 512
    localparam E_C5  = 128;    // 256KB
    localparam E_C6  = 2;      // 4096B
    localparam EXP_WR_BURSTS = 149;
    localparam EXP_RD_BURSTS = 143;
`else
    localparam E_C1  = 1;
    localparam E_C2a = 1;
    localparam E_C2b = 1;
    localparam E_C2c = 1;      // 4096B  ← 正好 4KB，一刀
    localparam E_C2d = 2;      // 8192B
    localparam E_C3  = 2;
    localparam E_C5  = 64;
    localparam E_C6  = 1;
    localparam EXP_WR_BURSTS = 81;
    localparam EXP_RD_BURSTS = 74;
`endif

    //------------------------------------------------------------------
    // 时钟与复位
    //------------------------------------------------------------------
    reg clk = 1'b0;
    always #(CLK_PERIOD/2) clk = ~clk;

    reg rst_n;

    //------------------------------------------------------------------
    // DUT 端口
    //------------------------------------------------------------------
    reg  [63:0] cmd_data;
    reg         cmd_valid;
    reg         cmd_tready;

    wire        s_axis_tready;
    wire [31:0] cmd_address;
    wire [7:0]  burst_len;
    wire        cmd_tvalid;
    wire [3:0]  cmd_err;

    decoder_wrcmd #(
        .AXI_DATA_WIDTH  (`DMA_AXI_SELECTED_DATA_WIDTH),
        .MAX_BURST_BEATS (`DMA_AXI_MAX_BURST_BEATS),
        .MEM_BYTES       (32'h4000_0000)
    ) u_dut (
        .clk          (clk),
        .rst_n        (rst_n),
        .cmd_data     (cmd_data),
        .cmd_valid    (cmd_valid),
        .cmd_tready   (cmd_tready),
        .s_axis_tready(s_axis_tready),
        .cmd_address  (cmd_address),
        .burst_len    (burst_len),
        .cmd_tvalid   (cmd_tvalid),
        .cmd_err      (cmd_err)
    );

    //------------------------------------------------------------------
    // 波形
    //------------------------------------------------------------------
    initial begin
        $dumpfile("decoder_wrcmd_tb.vcd");
        $dumpvars(0, decoder_wrcmd_tb);
    end

    //------------------------------------------------------------------
    // 观测: 收集突发
    //------------------------------------------------------------------
    localparam MAX_BURST = 256;
    reg [31:0] seen_addr [0:MAX_BURST-1];
    reg [7:0]  seen_len  [0:MAX_BURST-1];
    integer    seen_cnt;

    always @(posedge clk) begin
        if (cmd_tvalid && cmd_tready) begin
            if (seen_cnt < MAX_BURST) begin
                seen_addr[seen_cnt] = cmd_address;
                seen_len [seen_cnt] = burst_len;
            end
            seen_cnt = seen_cnt + 1;
        end
    end

    //------------------------------------------------------------------
    // 计数
    //------------------------------------------------------------------
    integer err_cnt;
    integer tot_wr_bursts;
    integer tot_rd_bursts;

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
            while (s_axis_tready !== 1'b1) @(posedge clk);
            #1;
            cmd_valid = 1'b0;
            cmd_data  = 64'd0;
        end
    endtask

    //------------------------------------------------------------------
    // 校验一条命令的结果
    //------------------------------------------------------------------
    task check_cmd;
        input [31:0] addr;
        input [31:0] bytes;
        input integer exp_bursts;
        input [127:0] tag;
        integer i;
        reg [31:0] sum_bytes;
        reg [31:0] seg_bytes;
        reg [31:0] exp_next;
        begin
            // [2] 突发条数
            if (seen_cnt !== exp_bursts) begin
                $display("[FAIL] %0s addr=0x%08x bytes=%0d 突发条数: 期望 %0d, 实得 %0d",
                         tag, addr, bytes, exp_bursts, seen_cnt);
                err_cnt = err_cnt + 1;
            end

            sum_bytes = 32'd0;
            exp_next  = addr;
            for (i = 0; i < seen_cnt; i = i + 1) begin
                seg_bytes = ({24'd0, seen_len[i]} + 32'd1) << SHIFT;

                // [5] burst_len 不超上限
                if (seen_len[i] > (`DMA_AXI_MAX_BURST_BEATS - 1)) begin
                    $display("[FAIL] %0s 突发[%0d] burst_len=%0d 超上限",
                             tag, i, seen_len[i]);
                    err_cnt = err_cnt + 1;
                end

                // [3] 不跨 4KB 边界
                if (({20'd0, seen_addr[i][11:0]} + seg_bytes) > 32'd4096) begin
                    $display("[FAIL] %0s 突发[%0d] 跨 4KB: addr=0x%08x 字节=%0d",
                             tag, i, seen_addr[i], seg_bytes);
                    err_cnt = err_cnt + 1;
                end

                // [4] 地址连续
                if (seen_addr[i] !== exp_next) begin
                    $display("[FAIL] %0s 突发[%0d] 地址不连续: 期望 0x%08x, 实得 0x%08x",
                             tag, i, exp_next, seen_addr[i]);
                    err_cnt = err_cnt + 1;
                end
                exp_next  = seen_addr[i] + seg_bytes;
                sum_bytes = sum_bytes + seg_bytes;
            end

            // [1] 总字节数（不丢不重）
            if (sum_bytes !== bytes) begin
                $display("[FAIL] %0s addr=0x%08x 总字节: 期望 %0d, 实得 %0d（丢了 %0d 或多了）",
                         tag, addr, bytes, sum_bytes,
                         (sum_bytes > bytes) ? (sum_bytes - bytes) : (bytes - sum_bytes));
                err_cnt = err_cnt + 1;
            end
        end
    endtask

    //------------------------------------------------------------------
    // 等 DUT 把当前命令吐完: 先等 busy 拉起, 再等 busy 落下
    //   不能等固定拍数 —— 一条 256KB 命令有 64 条突发, 每条占 2 拍
    //------------------------------------------------------------------
    task wait_idle;
        integer g;
        begin
            // 等 busy 拉起（最多 8 拍；错误命令不会拉起，直接跳过）
            for (g = 0; (g < 8) && (s_axis_tready === 1'b1); g = g + 1)
                @(posedge clk);
            // 等 busy 落下
            for (g = 0; (g < 100000) && (s_axis_tready !== 1'b1); g = g + 1)
                @(posedge clk);
            repeat (2) @(posedge clk);
        end
    endtask

    //------------------------------------------------------------------
    // 跑一条命令: 清账 → 发 → 等 → 查
    //------------------------------------------------------------------
    task run_cmd;
        input [31:0] addr;
        input [31:0] bytes;
        input integer exp_bursts;
        input [127:0] tag;
        begin
            seen_cnt = 0;
            $display("[%0t] >>> %0s addr=0x%08x bytes=%0d 期望 %0d 条突发",
                     $time, tag, addr, bytes, exp_bursts);
            put_cmd(addr, bytes);
            wait_idle;
            check_cmd(addr, bytes, exp_bursts, tag);
        end
    endtask

    //------------------------------------------------------------------
    // 错误命令: 必须不产生突发，且 cmd_err 对应位拉高
    //------------------------------------------------------------------
    task run_err_cmd;
        input [31:0] addr;
        input [31:0] bytes;
        input [3:0]  exp_err;
        input [127:0] tag;
        begin
            seen_cnt = 0;
            put_cmd(addr, bytes);
            wait_idle;

            if (seen_cnt !== 0) begin
                $display("[FAIL] %0s 错误命令却产生了 %0d 条突发", tag, seen_cnt);
                err_cnt = err_cnt + 1;
            end
            if (cmd_err !== exp_err) begin
                $display("[FAIL] %0s cmd_err: 期望 4'b%04b, 实得 4'b%04b",
                         tag, exp_err, cmd_err);
                err_cnt = err_cnt + 1;
            end
            else begin
                $display("  [OK]  %0s cmd_err=4'b%04b, 未产生突发", tag, cmd_err);
            end
        end
    endtask

    //------------------------------------------------------------------
    // 主激励
    //------------------------------------------------------------------
    integer n;

    initial begin
        // 初值
        rst_n      = 1'b0;
        cmd_valid  = 1'b0;
        cmd_data   = 64'd0;
        cmd_tready = 1'b1;          // 恒 ready，先不测反压
        seen_cnt   = 0;
        err_cnt    = 0;
        tot_wr_bursts = 0;
        tot_rd_bursts = 0;

        repeat (8) @(posedge clk);
        rst_n = 1'b1;
        repeat (4) @(posedge clk);

        $display("========================================================");
        $display(" decoder_wrcmd_tb   位宽=%0d  AXI_BYTES=%0d", `DMA_AXI_SELECTED_DATA_WIDTH, AXI_BYTES);
        $display("========================================================");

        //--------------------------------------------------------------
        $display("");
        $display("########## 写命令集（16 条，期望 %0d 条突发）##########", EXP_WR_BURSTS);
        //--------------------------------------------------------------
        run_cmd(32'h0010_0000, 32'd512,     E_C1,  "W-C1   ");
        run_cmd(32'h0012_0000, 32'd64,      E_C2a, "W-C2a  ");
        run_cmd(32'h0012_1000, 32'd256,     E_C2b, "W-C2b  ");
        run_cmd(32'h0012_2000, 32'd4096,    E_C2c, "W-C2c  ");
        run_cmd(32'h0012_4000, 32'd8192,    E_C2d, "W-C2d  ");
        run_cmd(32'h0013_0F00, 32'd512,     E_C3,  "W-C3   ");   // ★ 跨 4KB，首刀 256
        for (n = 0; n < 8; n = n + 1)
            run_cmd(32'h0014_0000 + n * 32'd512, 32'd512, 1, "W-C4   ");
        run_cmd(32'h0016_0000, 32'd262144, E_C5,  "W-C5   ");
        run_cmd(32'h001C_0000, 32'd4096,   E_C6,  "W-C6   ");

        //--------------------------------------------------------------
        $display("");
        $display("########## 读命令集（9 条，期望 %0d 条突发）##########", EXP_RD_BURSTS);
        //--------------------------------------------------------------
        run_cmd(32'h0010_0000, 32'd512,     E_C1,  "R-C1   ");
        run_cmd(32'h0012_0000, 32'd64,      E_C2a, "R-C2a  ");
        run_cmd(32'h0012_1000, 32'd256,     E_C2b, "R-C2b  ");
        run_cmd(32'h0012_2000, 32'd4096,    E_C2c, "R-C2c  ");
        run_cmd(32'h0012_4000, 32'd8192,    E_C2d, "R-C2d  ");
        run_cmd(32'h0013_0F00, 32'd512,     E_C3,  "R-C3   ");
        run_cmd(32'h0014_0000, 32'd4096,    E_C2c, "R-C4   ");
        run_cmd(32'h0016_0000, 32'd262144, E_C5,  "R-C5   ");
        run_cmd(32'h001C_0000, 32'd4096,   E_C6,  "R-C6   ");

        //--------------------------------------------------------------
        $display("");
        $display("########## 错误命令（必须不产生突发）##########");
        //--------------------------------------------------------------
        run_err_cmd(32'h0010_0000, 32'd0,      4'b0001, "E-null ");   // 空命令
        run_err_cmd(32'h0010_0000, E_LEN_BYTES, 4'b0010, "E-len  ");  // 长度非 AXI_BYTES 整倍数
        run_err_cmd(E_ALN_ADDR,    32'd512,    4'b0100, "E-align");  // 地址未对齐
        run_err_cmd(32'h5000_0000, 32'd512,    4'b1000, "E-range");   // 越界

        //--------------------------------------------------------------
        $display("");
        $display("========================================================");
        $display(" 检查错误数: %0d", err_cnt);
        $display(" 期望突发数: 写 %0d / 读 %0d", EXP_WR_BURSTS, EXP_RD_BURSTS);
        if (err_cnt == 0)
            $display("  ***** DECODER_TB_PASS *****");
        else
            $display("  ***** DECODER_TB_FAIL *****");
        $display("========================================================");
        $finish;
    end

    //------------------------------------------------------------------
    // 全局超时
    //------------------------------------------------------------------
    initial begin
        #2000000;
        $display("[FAIL] 全局超时（疑似卡死）");
        $display("  ***** DECODER_TB_FAIL *****");
        $finish;
    end

endmodule
