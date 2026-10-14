// ============================================================
// pooling_v_kernel — 垂直方向池化 (单通道)
// ============================================================

module pooling_v_kernel (
    input  wire               clk,
    input  wire               rst_n,

    // 数据输入
    input  wire signed [7:0]  h_in,
    input  wire               h_in_valid,

    // 控制参数
    input  wire [2:0]         kernel_size_r,
    input  wire [1:0]         stride_r,
    input  wire [12:0]        output_cols_per_row,
    input  wire [12:0]        total_rows_per_group,
    input  wire [10:0]        ch_group_total,
    input  wire [1:0]         padding_top,
    input  wire               circular_padding,      // 1=顶/底 pad 行为 ppool 供的真实环绕行

    // 垂直输出
    output wire signed [7:0]  v_out,
    output wire               v_out_valid
);

    // ============================================================
    // RAM 信号
    // ============================================================
    wire [31:0]                ram_rd_data;
    wire [31:0]                ram_wr_data;

    wire                       rd_en;
    wire                       wr_en;
    reg  [12:0]                col_addr;
    reg  [12:0]                wr_col_addr;
    reg  [12:0]                in_row_cnt;
    reg  [10:0]                ch_group_cnt;
    reg  [12:0]                output_cols_m1_reg;

    reg  signed [7:0]          h_in_0;
    reg  signed [7:0]          h_in_1;

    // pipeline 级数有效信号
    reg                        valid0;
    reg                        valid1;
    reg                        valid2;

    // row_start 流水线
    wire                       row_start;
    reg                        row_start_0;
    reg                        row_start_1;
    reg                        row_start_2;

    // row_end 流水线
    wire                       row_end;
    reg                        row_end_0;
    reg                        row_end_1;
    reg                        row_end_2;

    // ch_group_end 流水线
    wire                       ch_group_end;
    reg                        ch_group_end_0;
    reg                        ch_group_end_1;
    reg                        ch_group_end_2;

    // ============================================================
    // RAM 读出锁存
    // ============================================================
    reg  signed [7:0]          v_d0, v_d1, v_d2, v_d3;

    // ============================================================
    // 级联寄存器
    // ============================================================
    reg  signed [7:0]          v_r0, v_r1, v_r2, v_r3, v_r4;

    // ============================================================
    // VR 有效信号
    // ============================================================
    reg                        v_r_valid0;
    reg                        v_r_valid1;
    reg                        v_r_valid2;
    reg                        v_r_valid3;
    reg                        v_r_valid4;
    wire signed [7:0]          v_r_mux;
    wire                       v_r_mux_valid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            v_r_valid0 <= 1'b0;
        else if (ch_group_end_2)
            v_r_valid0 <= 1'b0;
        else if (valid1 && row_start_1)
            v_r_valid0 <= 1'b1;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            v_r_valid1 <= 1'b0;
        else if (ch_group_end_2)
            v_r_valid1 <= 1'b0;
        else if (valid1 && row_start_1)
            v_r_valid1 <= v_r_valid0;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            v_r_valid2 <= 1'b0;
        else if (ch_group_end_2)
            v_r_valid2 <= 1'b0;
        else if (valid1 && row_start_1)
            v_r_valid2 <= v_r_valid1;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            v_r_valid3 <= 1'b0;
        else if (ch_group_end_2)
            v_r_valid3 <= 1'b0;
        else if (valid1 && row_start_1)
            v_r_valid3 <= v_r_valid2;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            v_r_valid4 <= 1'b0;
        else if (ch_group_end_2)
            v_r_valid4 <= 1'b0;
        else if (valid1 && row_start_1)
            v_r_valid4 <= v_r_valid3;
    end

    // 输出级号计算
    reg  [2:0]                 v_out_stage;

    // 输出 stride 控制
    reg                        out_stride_cnt;

    // 输出行号
    reg  [12:0]                out_index;
    reg                        row_has_output;

    // ============================================================
    // unified 1R1W RAM instance
    // ============================================================
    assign ram_wr_data = {v_r3, v_r2, v_r1, v_r0};

    sync_1r1w_ram #(
        .DATA_WIDTH (32),
        .ADDR_WIDTH (11),
        .DEPTH      (2048)
    ) u_pool_v_storage (
        .clk      (clk),
        .wr_en    (wr_en),
        .wr_addr  (wr_col_addr[10:0]),
        .wr_data  (ram_wr_data),
        .rd_en    (rd_en),
        .rd_addr  (col_addr[10:0]),
        .rd_data  (ram_rd_data)
    );

    // ============================================================
    // 读使能 / 写使能
    // ============================================================
    assign rd_en = h_in_valid;
    assign wr_en = valid2;

    // 配置在首个 h_in_valid 前已经稳定。无复位本地落脚切断 ctrl 配置网
    // 到行边界计数器 CE 的跨区组合路径；仅增加配置生效固定 1 拍，
    // 不改变任意有效数据拍的吞吐或算法流水延迟。
    always @(posedge clk) begin
        output_cols_m1_reg <= output_cols_per_row - 13'd1;
    end

    // ============================================================
    // 读地址计数器
    // ============================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            col_addr <= 13'd0;
        else if (rd_en)
            col_addr <= (col_addr == output_cols_m1_reg) ? 13'd0 : (col_addr + 13'd1);
    end

    // ============================================================
    // 行计数器 (列末推进)
    // ============================================================
    assign row_end = (col_addr == output_cols_m1_reg) && rd_en;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            in_row_cnt <= 13'd0;
        else if (row_end)
            in_row_cnt <= (in_row_cnt == total_rows_per_group - 13'd1) ? 13'd0 : (in_row_cnt + 13'd1);
    end

    // ============================================================
    // 通道组计数器 (行计数器归零时推进)
    // ============================================================
    assign ch_group_end = row_end && (in_row_cnt == total_rows_per_group - 13'd1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            ch_group_cnt <= 11'd0;
        else if (ch_group_end)
            ch_group_cnt <= (ch_group_cnt == ch_group_total - 11'd1) ? 11'd0 : (ch_group_cnt + 11'd1);
    end

    // ============================================================
    // 写地址计数器
    // ============================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            wr_col_addr <= 13'd0;
        else if (wr_en)
            wr_col_addr <= (wr_col_addr == output_cols_m1_reg) ? 13'd0 : (wr_col_addr + 13'd1);
    end

    // ============================================================
    // row_start (组合逻辑: 读地址为0且读使能有效)
    // ============================================================
    assign row_start = (col_addr == 13'd0) && rd_en;

    // ============================================================
    // row_start_0 (row_start 打一拍, 与 valid0 同级)
    // ============================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            row_start_0 <= 1'b0;
        else
            row_start_0 <= row_start;
    end

    // ============================================================
    // row_start_1 (row_start_0 打一拍, 与 valid1 同级)
    // ============================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            row_start_1 <= 1'b0;
        else
            row_start_1 <= row_start_0;
    end

    // ============================================================
    // row_start_2 (row_start_1 打一拍, 与 valid2 同级)
    // ============================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            row_start_2 <= 1'b0;
        else
            row_start_2 <= row_start_1;
    end

    // ============================================================
    // row_end_0 (row_end 打一拍, 与 valid0 同级)
    // ============================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            row_end_0 <= 1'b0;
        else
            row_end_0 <= row_end;
    end

    // ============================================================
    // row_end_1 (row_end_0 打一拍, 与 valid1 同级)
    // ============================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            row_end_1 <= 1'b0;
        else
            row_end_1 <= row_end_0;
    end

    // ============================================================
    // row_end_2 (row_end_1 打一拍, 与 valid2 同级)
    // ============================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            row_end_2 <= 1'b0;
        else
            row_end_2 <= row_end_1;
    end

    // ============================================================
    // ch_group_end_0 (ch_group_end 打一拍, 与 valid0 同级)
    // ============================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            ch_group_end_0 <= 1'b0;
        else
            ch_group_end_0 <= ch_group_end;
    end

    // ============================================================
    // ch_group_end_1 (ch_group_end_0 打一拍, 与 valid1 同级)
    // ============================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            ch_group_end_1 <= 1'b0;
        else
            ch_group_end_1 <= ch_group_end_0;
    end

    // ============================================================
    // ch_group_end_2 (ch_group_end_1 打一拍, 与 valid2 同级)
    // ============================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            ch_group_end_2 <= 1'b0;
        else
            ch_group_end_2 <= ch_group_end_1;
    end

    // ============================================================
    // v_out_stage (valid1 级: kr - 1 - max(0, pt - out_index*stride))
    // ============================================================
    // 输出行 i 窗口顶部缺行数 = max(0, padding_top - i*实际步长)。
    // circular_padding=1 时顶/底 pad 行是 ppool 供的真实环绕行,
    // 窗口不缺行, excess 恒 0 (不能用非 cpad 的浅窗逻辑)。
    // padding_top<=3, 故 i*stride>=4 时缺行恒 0: out_index 先饱和到 4 再移位,
    // 不能用定长切片 (tc31 实证 [3:0] 在 out_index=16 绕回 0, 浅窗在第 16
    // 输出行复发; 早前 [1:0] 在行 4/8/12 复发同属此类)。
    wire [12:0] out_index_clamp;
    wire [4:0]  out_index_scaled;
    wire [1:0]  excess;
    assign out_index_clamp  = (out_index > 13'd4) ? 13'd4 : out_index;
    assign out_index_scaled = out_index_clamp[4:0] << stride_r;
    assign excess = circular_padding ? 2'd0 :
                    ((padding_top > out_index_scaled) ? (padding_top - out_index_scaled[1:0]) : 2'd0);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            v_out_stage <= 3'd0;
        else if ({1'b0, excess} >= kernel_size_r)
            v_out_stage <= 3'd0;
        else
            v_out_stage <= kernel_size_r - {1'b0, excess} - 3'd1;
    end

    // ============================================================
    // row_has_output (valid2 级: 本行是否有过有效输出)
    // ============================================================
    // 组末清零不看 valid2: ch_group_end_2 是单拍脉冲, 若清零拍恰好落在
    // 组边界气泡上 (valid2=0), 清零丢失会导致下一组行 0 用陈旧 v_r_valid
    // 发出整行假拍 (tc19 实证: 每组边界多 32 拍)。
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            row_has_output <= 1'b0;
        else if ((valid2 && row_start_2) || ch_group_end_2)
            row_has_output <= 1'b0;
        else if (valid2 && v_r_mux_valid)
            row_has_output <= 1'b1;
    end

    // ============================================================
    // out_index (valid2 级: 输出行末推进)
    // ============================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            out_index <= 13'd0;
        else if (ch_group_end_2)
            out_index <= 13'd0;
        else if (valid2 && row_end_2 && row_has_output)
            out_index <= out_index + 13'd1;
    end

    // ============================================================
    // out_stride_cnt (valid2 级: 行末推进, 按 stride_r 回绕)
    // ============================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            out_stride_cnt <= 1'b0;
        else if (ch_group_end_2)
            out_stride_cnt <= 1'b0;
        else if (valid2 && row_end_2)
            out_stride_cnt <= (out_stride_cnt == stride_r) ? 1'b0 : (out_stride_cnt + 1'b1);
    end

    // ============================================================
    // valid0 (rd_en 打一拍)
    // ============================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            valid0 <= 1'b0;
        else
            valid0 <= rd_en;
    end

    // ============================================================
    // h_in_0 (h_in 打一拍)
    // ============================================================
    always @(posedge clk) begin
        if (rd_en)
            h_in_0 <= h_in;
    end

    // ============================================================
    // valid1 (valid0 打一拍)
    // ============================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            valid1 <= 1'b0;
        else
            valid1 <= valid0;
    end

    // ============================================================
    // h_in_1 (h_in_0 打一拍)
    // ============================================================
    always @(posedge clk) begin
        h_in_1 <= h_in_0;
    end

    // ============================================================
    // v_d0~v_d3 (ram_rd_data 打一拍)
    // ============================================================
    always @(posedge clk) begin
        v_d0 <= ram_rd_data[7:0];
    end

    always @(posedge clk) begin
        v_d1 <= ram_rd_data[15:8];
    end

    always @(posedge clk) begin
        v_d2 <= ram_rd_data[23:16];
    end

    always @(posedge clk) begin
        v_d3 <= ram_rd_data[31:24];
    end

    // ============================================================
    // valid2 (valid1 打一拍)
    // ============================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            valid2 <= 1'b0;
        else
            valid2 <= valid1;
    end

    // ============================================================
    // 级联计算 v_r0~v_r4
    // ============================================================
    always @(posedge clk) begin
        if (valid1)
            v_r0 <= h_in_1;
    end

    always @(posedge clk) begin
        if (valid1)
            v_r1 <= (h_in_1 > v_d0) ? h_in_1 : v_d0;
    end

    always @(posedge clk) begin
        if (valid1)
            v_r2 <= (h_in_1 > v_d1) ? h_in_1 : v_d1;
    end

    always @(posedge clk) begin
        if (valid1)
            v_r3 <= (h_in_1 > v_d2) ? h_in_1 : v_d2;
    end

    always @(posedge clk) begin
        if (valid1)
            v_r4 <= (h_in_1 > v_d3) ? h_in_1 : v_d3;
    end

    // ============================================================
    // 输出选择
    // ============================================================
    assign v_r_mux = (v_out_stage == 3'd0) ? v_r0 :
                     (v_out_stage == 3'd1) ? v_r1 :
                     (v_out_stage == 3'd2) ? v_r2 :
                     (v_out_stage == 3'd3) ? v_r3 :
                                              v_r4;

    assign v_r_mux_valid = (v_out_stage == 3'd0) ? v_r_valid0 :
                           (v_out_stage == 3'd1) ? v_r_valid1 :
                           (v_out_stage == 3'd2) ? v_r_valid2 :
                           (v_out_stage == 3'd3) ? v_r_valid3 :
                                                    v_r_valid4;

    reg  signed [7:0] v_out_r;
    reg               v_out_valid_r;

    // 输出行采样相位: 输出行 i 的窗口 (kernel_size_r 行) 完成于输入行
    // i*stride + (kernel_size_r - 1) - padding_top, 该行的
    // out_stride_cnt = (kernel_size_r - 1 - padding_top) mod (stride_r + 1)。
    // circular_padding=1 时顶行真实进流, 相位不含 padding_top 项
    // (等价 pt=0: (kernel_size_r - 1) mod (stride_r + 1))。
    // stride_r=1 (步长2): 相位 = ~kernel_size_r[0] ^ (非 cpad 时 padding_top[0]);
    // stride_r=0 (步长1): 相位恒 0 (每行都输出)。
    wire v_sample_phase;
    assign v_sample_phase = stride_r[0] ? (~kernel_size_r[0] ^
                            (padding_top[0] & ~circular_padding)) : 1'b0;

    always @(posedge clk) begin
        if (valid2 && v_r_mux_valid && (out_stride_cnt == v_sample_phase))
            v_out_r <= v_r_mux;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            v_out_valid_r <= 1'b0;
        else
            v_out_valid_r <= valid2 && v_r_mux_valid && (out_stride_cnt == v_sample_phase);
    end

    assign v_out       = v_out_r;
    assign v_out_valid = v_out_valid_r;

endmodule

/*
 * RTL SELF-CHECK:
 * PASS: pure Verilog syntax; all ports explicitly use wire
 * PASS: combinational logic uses continuous assignments only
 * PASS: sequential logic uses nonblocking assignments only
 * PASS: no process loop, division, modulo, function, latch, or combinational loop
 * PASS: payload/RAM arrays remain unreset; control valid/address reset behavior is unchanged
 * PASS: output_cols_per_row_minus_one is landed locally before any valid input arrives
 * PASS: data, valid, row sideband, and RAM pipeline alignment are unchanged
 * PASS: steady-state throughput remains one accepted valid beat per cycle
 * WARN: local-placement timing benefit requires post-implementation confirmation
 * FAIL: none
 */
