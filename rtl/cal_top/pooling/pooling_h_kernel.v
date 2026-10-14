// ============================================================
// pooling_h_kernel — 水平方向池化 (单通道)
// ============================================================
//
// 【模块定位】
//   单通道 Int8 输入，在单行内维护滑动窗口 (宽度 1~5)，按水平步长间隔
//   采样输出窗口内最大值。完全无状态机 — 纯数据通路 + 内部计数器。
//   数据全部视为有符号数: -128(0x80) 最小, +127(0x7F) 最大。
//
// 【实例数】
//   2 个实例 (ch0 / ch1 各一)，完全独立，各处理一个通道。
//
// 【数据流】
//   din(signed 8bit) → [4 级级联寄存器 h_reg0~3] → [MUX 按 kernel_size_c 选通]
//                    → h_out(signed 8bit) → pooling_v_kernel
//
//   采样点由行内拍索引 col_cnt 直接派生 (win_fire)，打两拍与 h_win_data
//   流水对齐。计数与采样全部由 din_valid 数据拍驱动：行间/组间气泡、
//   下游反压造成的停顿不改变采样相位，对外只暴露单一 h_out_valid。
//
// 【时序概要】
//   拍 N:   din_valid=1, h_reg 级联更新，窗口完成拍 win_fire=1。
//   拍 N+1: h_reg_sel 锁存进 h_win_data, win_fire_d1。
//   拍 N+2: h_win_data 稳定, win_fire_d2 = h_sample, h_out 锁存。
//   拍 N+3: h_out 稳定，h_out_valid=1。
//
// ============================================================

module pooling_h_kernel (
    // ---- 系统信号 ----
    input  wire               clk,
    input  wire               rst_n,

    // ---- 数据输入 (← pooling_input) ----
    input  wire signed [7:0]  din,
    input  wire               din_valid,

    // ---- 控制参数 (← pooling_ctrl, 整个 pooling 操作期间不变) ----
    input  wire [2:0]         kernel_size_c,          // 核宽度 (1~5)
    input  wire [1:0]         stride_c,               // 水平步长编码 (实际步长 = 编码值+1, 范围 1~2)
    input  wire [12:0]        total_cols_per_row,      // 每行总列数 (含左右 padding)
    input  wire [12:0]        output_cols_per_row,     // 每行输出列数 (ctrl 预计算)

    // ---- 水平输出 (→ pooling_v_kernel) ----
    output wire signed [7:0]  h_out,
    output wire               h_out_valid
);

    // ============================================================
    // 内部寄存器
    // ============================================================

    // 5 级级联寄存器: 存放最近 5 个输入列的值/中间 max 结果
    //   h_reg0: 当前拍输入的 din (打拍)
    //   h_reg1: max(h_reg0, 上一拍 h_reg0) = 最近 2 个元素的 max
    //   h_reg2: max(h_reg0, 上一拍 h_reg1) = 最近 3 个元素的 max
    //   h_reg3: max(h_reg0, 上一拍 h_reg2) = 最近 4 个元素的 max
    //   h_reg4: max(h_reg0, 上一拍 h_reg3) = 最近 5 个元素的 max
    reg signed [7:0] h_reg0, h_reg1, h_reg2, h_reg3, h_reg4;

    // ---- 组合逻辑: 5 个窗口大小的 max 结果 ----
    wire signed [7:0] h_max0, h_max1, h_max2, h_max3, h_max4;

    // ---- 第一级中间结果 (按核宽选通, 打拍) ----
    reg  signed [7:0] h_win_data;            // 选通的窗口 max 寄存器值

    // ---- 第二级: col_cnt 采样点 + 输出 ----
    reg  [12:0]       output_col_cnt;        // 当前行已输出的列数
    reg  signed [7:0] h_out_reg;             // 最终输出数据
    reg               h_out_valid_reg;       // 最终输出有效

    // ---- 内部列计数器 ----
    reg  [12:0] col_cnt;                     // 当前行内列号 (0 ~ total_cols_per_row-1)

    // ---- 行边界 ----
    wire        row_end;                     // 组合: 当前拍是本行最后一列

    // ============================================================
    // 水平 max 组合逻辑
    // ============================================================
    //
    // h_max0 = din                           — 窗口=1
    // h_max1 = max(din, h_reg0)              — 窗口=2
    // h_max2 = max(din, h_reg1)              — 窗口=3
    // h_max3 = max(din, h_reg2)              — 窗口=4
    // h_max4 = max(din, h_reg3)              — 窗口=5
    //
    // h_reg* 是上一拍寄存的旧值, 组合逻辑用旧值+当前 din 算出窗口 max。
    //
    assign h_max0 = din;
    assign h_max1 = (din > h_reg0) ? din : h_reg0;
    assign h_max2 = (din > h_reg1) ? din : h_reg1;
    assign h_max3 = (din > h_reg2) ? din : h_reg2;
    assign h_max4 = (din > h_reg3) ? din : h_reg3;

    // ============================================================
    // 级联寄存器更新 (时序逻辑, 每寄存器一个 always 块)
    // ============================================================
    always @(posedge clk) if (din_valid) h_reg0 <= h_max0;
    always @(posedge clk) if (din_valid) h_reg1 <= h_max1;
    always @(posedge clk) if (din_valid) h_reg2 <= h_max2;
    always @(posedge clk) if (din_valid) h_reg3 <= h_max3;
    always @(posedge clk) if (din_valid) h_reg4 <= h_max4;

    // ============================================================
    // 列计数器 + 行边界检测 (时序逻辑)
    // ============================================================
    //
    // col_cnt: 0 ~ total_cols_per_row-1, din_valid=1 时推进。
    // row_end:  当前拍是本行最后一列 (组合), 复位 col_cnt。
    //
    assign row_end = (col_cnt == total_cols_per_row - 13'd1) && din_valid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            col_cnt   <= 13'd0;
        end else begin
            if (din_valid) begin
                col_cnt   <= row_end ? 13'd0 : (col_cnt + 13'd1);
            end
        end
    end

    // ============================================================
    // 第一级输出: 按 kernel_size_c 选通 (组合 → 寄存器)
    // ============================================================
    //
    // 从 h_reg* 中按核宽选通。
    // h_regN 已在同一时钟边沿对齐，避免当前拍组合 h_max*
    // 与寄存器值混选导致水平窗口采样错位。
    //
    wire signed [7:0] h_reg_sel;

    assign h_reg_sel   = (kernel_size_c == 3'd1) ? h_reg0 :
                         (kernel_size_c == 3'd2) ? h_reg1 :
                         (kernel_size_c == 3'd3) ? h_reg2 :
                         (kernel_size_c == 3'd4) ? h_reg3 :
                                                   h_reg4;

    always @(posedge clk) begin
        h_win_data <= h_reg_sel;
    end

    // ============================================================
    // 第二级输出: 采样点由 col_cnt 派生 → h_out / h_out_valid
    // ============================================================
    //
    // win_fire: 当前数据拍恰好完成一个应输出的窗口。
    //   核宽 k、步长 s 时，窗口在第 (k-1), (k-1)+s, (k-1)+2s ... 个
    //   数据拍 (col_cnt) 完成。打两拍与 h_win_data 流水对齐:
    //     拍 N   din_valid 数据拍, 窗口在 h_reg* 级联中完成, win_fire=1
    //     拍 N+1 h_reg_sel 锁存进 h_win_data, win_fire_d1
    //     拍 N+2 h_win_data 稳定, win_fire_d2 = h_sample, h_out 锁存
    //
    //   计数与采样全部由 din_valid 数据拍驱动: 行间/组间气泡、下游反压
    //   停顿期间 win_fire 保持 0, 采样相位与墙钟节拍数无关, 不错位。
    //
    // output_col_cnt: 采样计数, 计满 output_cols_per_row 自清零。
    //   不在行边界复位: 本行末窗的 h_sample 可能落在下一行前几个墙钟拍,
    //   由自清零保证跨行/跨组计数不断链。
    //
    wire [12:0] k_minus1;
    wire [12:0] win_idx;
    wire        win_fire;
    reg         win_fire_d1, win_fire_d2;
    wire        h_sample;
    wire        output_done;

    assign k_minus1    = {10'd0, kernel_size_c} - 13'd1;
    assign win_idx     = col_cnt - k_minus1;
    assign win_fire    = din_valid && (col_cnt >= k_minus1) &&
                         (!stride_c[0] || (win_idx[0] == 1'b0));

    assign output_done = (output_col_cnt >= output_cols_per_row);
    assign h_sample    = win_fire_d2 && !output_done;

    // 采样点打拍链
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            win_fire_d1 <= 1'b0;
            win_fire_d2 <= 1'b0;
        end else begin
            win_fire_d1 <= win_fire;
            win_fire_d2 <= win_fire_d1;
        end
    end

    // 输出列计数器: 计满自清零
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            output_col_cnt <= 13'd0;
        else if (h_sample)
            output_col_cnt <= (output_col_cnt == output_cols_per_row - 13'd1)
                              ? 13'd0 : (output_col_cnt + 13'd1);
    end

    // 最终输出寄存器
    always @(posedge clk) begin
        h_out_reg <= h_win_data;
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)          h_out_valid_reg <= 1'b0;
        else                 h_out_valid_reg <= h_sample;
    end

    assign h_out       = h_out_reg;
    assign h_out_valid = h_out_valid_reg;

endmodule
