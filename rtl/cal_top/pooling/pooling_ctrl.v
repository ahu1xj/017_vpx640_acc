// ============================================================
// pooling_ctrl — 主控制器
// ============================================================

`include "all_defines.vh"

module pooling_ctrl (
    input  wire               clk,
    input  wire               rst_n,

    // 指令输入
    input  wire               rdcmd_trans_valid,
    input  wire [111:0]       rdcmd_trans,

    // ---- pooling_input ----
    output wire               start,
    output wire               circular_padding,
    output wire [1:0]         padding_top,
    output wire [1:0]         padding_bottom,
    output wire [11:0]        fm_row_in,
    output wire [4:0]         last_fm_out_column_valid,
    output wire [12:0]        total_cols_per_row,
    output wire [12:0]        fm_total_rows,
    output wire [10:0]        ch_group_total,
    output wire [12:0]        last_group_start,

    // ---- pooling_h_kernel (×2) ----
    output wire [2:0]         kernel_size_c,
    output wire [1:0]         stride_c,
    output wire [12:0]        output_cols_per_row,

    // ---- pooling_v_kernel (×2) ----
    output wire [2:0]         kernel_size_r,
    output wire [1:0]         stride_r,
    output wire [12:0]        total_rows_per_group,

    // ---- pooling_packer ----
    output wire [12:0]        total_output_groups,
    output wire [5:0]         last_group_valid,
    output wire               is_odd_input
);

    // ============================================================
    // 指令锁存
    // ============================================================
    reg  [111:0]              cmd_latch;

    // ============================================================
    // 译码寄存器
    // ============================================================
    reg  [1:0]                stride_c_decoded;
    reg  [1:0]                stride_r_decoded;

    // ============================================================
    // 预计算寄存器 — stage 1
    // ============================================================
    reg  [12:0]               _total_cols_per_row;
    reg  [12:0]               _fm_total_rows;
    reg  [10:0]               _ch_group_total;
    reg  [12:0]               _last_group_start;

    // ============================================================
    // 预计算寄存器 — stage 2
    // ============================================================
    reg  [12:0]               _output_cols_per_row;
    reg  [12:0]               _total_rows_per_group;
    reg  [12:0]               _total_output_groups;
    reg  [5:0]                _last_group_valid;
    reg                       _is_odd_input;

    // ============================================================
    // FSM
    // ============================================================
    reg                       start_r;
    reg  [1:0]                state;
    reg  [1:0]                next_state;

    localparam IDLE     = 2'd0;
    localparam DECODE   = 2'd1;
    localparam COMPUTE1 = 2'd2;
    localparam COMPUTE2 = 2'd3;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            state <= IDLE;
        else
            state <= next_state;
    end

    always @(*) begin
        next_state = state;
        case (state)
            IDLE:     if (rdcmd_trans_valid) next_state = DECODE;
            DECODE:   next_state = (cmd_latch[`RD_MODEL_SEL_START +: `RD_MODEL_SEL_WIDTH] == `MODEL_SEL_POOLING)
                                   ? COMPUTE1 : IDLE;
            COMPUTE1: next_state = COMPUTE2;
            COMPUTE2: next_state = IDLE;
            default:  next_state = IDLE;
        endcase
    end

    // ============================================================
    // DECODE: 锁存指令 + 步长译码
    // ============================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cmd_latch        <= 112'd0;
            stride_c_decoded <= 2'd1;
            stride_r_decoded <= 2'd1;
        end else if (state == IDLE && rdcmd_trans_valid) begin
            cmd_latch        <= rdcmd_trans;
            stride_c_decoded <= rdcmd_trans[`RD_POOL_STRIDE_C_START +: `RD_POOL_STRIDE_C_WIDTH] + 2'd1;
            stride_r_decoded <= rdcmd_trans[`RD_POOL_STRIDE_R_START +: `RD_POOL_STRIDE_R_WIDTH] + 2'd1;
        end
    end

    // ============================================================
    // COMPUTE1: 依赖原指令字段的计算
    // ============================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            _total_cols_per_row <= 13'd0;
            _fm_total_rows      <= 13'd0;
            _ch_group_total     <= 11'd0;
            _last_group_start   <= 13'd0;
        end else if (state == COMPUTE1) begin
            _total_cols_per_row <= {1'b0, cmd_latch[`RD_FM_COLUMN_IN_START +: `RD_FM_COLUMN_IN_WIDTH], 5'd0}
                                 + 13'd32
                                 + {11'd0, cmd_latch[`RD_POOL_PADDING_LEFT_START  +: `RD_POOL_PADDING_LEFT_WIDTH]}
                                 + {11'd0, cmd_latch[`RD_POOL_PADDING_RIGHT_START +: `RD_POOL_PADDING_RIGHT_WIDTH]};

            _fm_total_rows <= cmd_latch[`RD_POOL_CIRCULAR_PADDING_START]
                            ? cmd_latch[`RD_FM_ROW_IN_START +: `RD_FM_ROW_IN_WIDTH]
                            : (cmd_latch[`RD_FM_ROW_IN_START +: `RD_FM_ROW_IN_WIDTH]
                               + {10'd0, cmd_latch[`RD_POOL_PADDING_BOTTOM_START +: `RD_POOL_PADDING_BOTTOM_WIDTH]});

            _ch_group_total <= cmd_latch[`RD_FM_CHANNEL_IN_START +: `RD_FM_CHANNEL_IN_WIDTH] >> 1;

            _last_group_start <= {11'd0, cmd_latch[`RD_POOL_PADDING_LEFT_START +: `RD_POOL_PADDING_LEFT_WIDTH]}
                               + {1'b0, cmd_latch[`RD_FM_COLUMN_IN_START +: `RD_FM_COLUMN_IN_WIDTH], 5'd0};
        end
    end

    // ============================================================
    // COMPUTE2: 依赖 stage 1 结果的计算
    // ============================================================
    wire [13:0] output_cols_numer;
    wire [13:0] output_cols_calc;
    wire [12:0] rows_calc;

    assign output_cols_numer = {1'b0, _total_cols_per_row}
                             - {11'd0, cmd_latch[`RD_POOL_KERNEL_SIZE_C_START +: `RD_POOL_KERNEL_SIZE_C_WIDTH]};

    // 合法 stride 编码仅为 0/1，对应实际步长 1/2。用 mux + 固定右移
    // 替代变量除法，保持原 floor(numer/stride)+1 的逐位结果。
    assign output_cols_calc = stride_c_decoded[1]
                            ? ({1'b0, output_cols_numer[13:1]} + 14'd1)
                            : (output_cols_numer + 14'd1);

    // 语义输出列数 = ceil(真实输入列数 / stride_c)。
    // 硬件遍历窗口按"32 列块补齐行 + 左右 padding"计（尾部窗全 -128），
    // 语义输出（与软件参考一致，pool SAME 恒等于 ceil(in/stride)）只取前 N 窗；
    // packer 据此把末组 N 之后的列掩 0，与参考 PSO 的 0 填充对齐。
    // 真实输入列数 = fm_column_in*32 + last_fm_out_column_valid + 1。
    wire [13:0] real_cols_calc;
    wire [13:0] semantic_out_cols_calc;

    assign real_cols_calc = {1'b0, cmd_latch[`RD_FM_COLUMN_IN_START +: `RD_FM_COLUMN_IN_WIDTH], 5'd0}
                          + {8'd0, cmd_latch[`RD_LAST_FM_OUT_COLUMN_VALID_START +: `RD_LAST_FM_OUT_COLUMN_VALID_WIDTH]}
                          + 14'd1;
    wire [13:0] semantic_stride2_sum;
    assign semantic_stride2_sum = real_cols_calc + 14'd1;
    // ceil(real_cols/2) = (real_cols+1)>>1；stride=1 时直接旁路。
    assign semantic_out_cols_calc = stride_c_decoded[1]
                                  ? {1'b0, semantic_stride2_sum[13:1]}
                                  : real_cols_calc;

    assign rows_calc = cmd_latch[`RD_POOL_CIRCULAR_PADDING_START]
                     ? ({11'd0, cmd_latch[`RD_POOL_PADDING_TOP_START    +: `RD_POOL_PADDING_TOP_WIDTH]}
                        + cmd_latch[`RD_FM_ROW_IN_START +: `RD_FM_ROW_IN_WIDTH]
                        + {10'd0, cmd_latch[`RD_POOL_PADDING_BOTTOM_START +: `RD_POOL_PADDING_BOTTOM_WIDTH]})
                     : (cmd_latch[`RD_FM_ROW_IN_START +: `RD_FM_ROW_IN_WIDTH]
                        + {10'd0, cmd_latch[`RD_POOL_PADDING_BOTTOM_START +: `RD_POOL_PADDING_BOTTOM_WIDTH]});

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            _output_cols_per_row  <= 13'd0;
            _total_rows_per_group <= 13'd0;
            _total_output_groups  <= 13'd0;
            _last_group_valid     <= 6'd0;
            _is_odd_input         <= 1'b0;
        end else if (state == COMPUTE2) begin
            _output_cols_per_row  <= output_cols_calc[12:0];
            _total_rows_per_group <= rows_calc;
            // is_odd_input: 语义末组有效列 <=16 时末组按 16 车道 (奇数) 布局,
            // >16 (或整 32) 时按 32 车道满组布局。旧式 ~fm_column_in[0] 按输入块数
            // 奇偶近似, 与语义末组大小无关, s1 末组 >16 列时错切 16 车道丢列
            // (tc28 实证: 语义 26 列 pack 208/208 全错)。
            _is_odd_input         <= (semantic_out_cols_calc[4:0] != 5'd0) &&
                                     (semantic_out_cols_calc[4:0] <= 5'd16);

            _total_output_groups <= (semantic_out_cols_calc[12:0] + 13'd31) >> 5;
            _last_group_valid    <= (semantic_out_cols_calc[4:0] == 5'd0)
                                   ? 6'd32
                                   : {1'b0, semantic_out_cols_calc[4:0]};
        end
    end

    // ============================================================
    // start 脉冲
    // ============================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            start_r <= 1'b0;
        else
            start_r <= (state == COMPUTE2);
    end

    // ============================================================
    // 输出端口连接
    // ============================================================
    assign start                = start_r;
    assign circular_padding     = cmd_latch[`RD_POOL_CIRCULAR_PADDING_START];
    assign padding_top          = cmd_latch[`RD_POOL_PADDING_TOP_START    +: `RD_POOL_PADDING_TOP_WIDTH];
    assign padding_bottom       = cmd_latch[`RD_POOL_PADDING_BOTTOM_START +: `RD_POOL_PADDING_BOTTOM_WIDTH];
    assign fm_row_in            = cmd_latch[`RD_FM_ROW_IN_START           +: `RD_FM_ROW_IN_WIDTH];
    assign last_fm_out_column_valid = cmd_latch[`RD_LAST_FM_OUT_COLUMN_VALID_START +: `RD_LAST_FM_OUT_COLUMN_VALID_WIDTH];
    assign total_cols_per_row   = _total_cols_per_row;
    assign fm_total_rows        = _fm_total_rows;
    assign ch_group_total       = _ch_group_total;
    assign last_group_start     = _last_group_start;
    assign kernel_size_c        = cmd_latch[`RD_POOL_KERNEL_SIZE_C_START +: `RD_POOL_KERNEL_SIZE_C_WIDTH];
    assign stride_c             = cmd_latch[`RD_POOL_STRIDE_C_START      +: `RD_POOL_STRIDE_C_WIDTH];
    assign output_cols_per_row  = _output_cols_per_row;
    assign kernel_size_r        = cmd_latch[`RD_POOL_KERNEL_SIZE_R_START +: `RD_POOL_KERNEL_SIZE_R_WIDTH];
    assign stride_r             = cmd_latch[`RD_POOL_STRIDE_R_START      +: `RD_POOL_STRIDE_R_WIDTH];
    assign total_rows_per_group = _total_rows_per_group;
    assign total_output_groups  = _total_output_groups;
    assign last_group_valid     = _last_group_valid;
    assign is_odd_input         = _is_odd_input;

endmodule

/*
 * RTL SELF-CHECK:
 * PASS: pure Verilog syntax; all ports explicitly use wire
 * PASS: combinational logic uses continuous/blocking assignments only
 * PASS: sequential logic uses nonblocking assignments only
 * PASS: no process loop, division, modulo, function, latch, or combinational loop
 * PASS: stride 1/2 arithmetic uses explicit mux and fixed right shifts
 * PASS: width and unsigned arithmetic match the previous legal-input results
 * PASS: FSM, start pulse, output latency, and reset behavior are unchanged
 * WARN: timing improvement still requires post-synthesis/implementation confirmation
 * FAIL: none
 */
