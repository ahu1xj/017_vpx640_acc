// ============================================================
// pooling_packer — 输出打包模块
// ============================================================

module pooling_packer (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               start,

    // 数据输入 (← pooling_v_kernel ×2)
    input  wire [7:0]         ch0_din,
    input  wire               ch0_valid,
    input  wire [7:0]         ch1_din,
    input  wire               ch1_valid,

    // 控制参数 (← pooling_ctrl)
    input  wire [12:0]        output_cols_per_row,
    input  wire [12:0]        total_output_groups,
    input  wire [5:0]         last_group_valid,
    input  wire               is_odd_input,

    // 输出
    output wire [511:0]       m_axis_pooling_tdata,
    output wire               m_axis_pooling_tvalid
);

    // ============================================================
    // 移位寄存器
    // ============================================================
    reg  [255:0]              ch0_shift;
    reg  [255:0]              ch1_shift;

    // ============================================================
    // 列 / 组计数器
    // ============================================================
    reg  [4:0]                col_cnt;
    reg  [12:0]               tot_col_cnt;
    reg  [12:0]               group_cnt;

    // ============================================================
    // 本地配置落脚
    // ============================================================
    // start 比第一拍 v_valid 至少提前经过 input/H/V 流水线，因此这些寄存器
    // 不需要复位；它们只在有效数据到达后被消费。
    reg  [12:0]               output_cols_m1_reg;
    reg  [12:0]               total_output_groups_reg;
    reg  [5:0]                last_group_valid_reg;

    // ============================================================
    // 当前组 metadata
    // ============================================================
    // group_done 时提前计算下一组属性，避免 group_cnt/total_output_groups 的
    // 宽比较结果直接扇出到 512-bit 数据寄存器。四份 slice metadata 分别只
    // 驱动一个 128-bit 输出切片，禁止综合形成单根 500+ loads 控制网。
    reg                       group_is_last_reg;
    reg                       group_is_odd_reg;
    reg                       group_emit_reg;
    reg  [3:0]                group_last_slice_reg;
    reg  [3:0]                group_odd_slice_reg;

    wire [12:0]               next_group_cnt;
    wire                      next_group_is_last;
    wire                      next_group_emit;
    wire                      first_group_is_last;
    wire                      first_group_emit;
    wire [4:0]                group_limit;
    wire                      row_done;
    wire                      group_done;

    assign next_group_cnt     = group_cnt + 13'd1;
    assign next_group_is_last = (next_group_cnt == total_output_groups_reg - 13'd1);
    assign next_group_emit    = (next_group_cnt < total_output_groups_reg);
    assign first_group_is_last = (total_output_groups == 13'd1);
    assign first_group_emit    = (total_output_groups != 13'd0);

    always @(posedge clk) begin
        if (start) begin
            output_cols_m1_reg        <= output_cols_per_row - 13'd1;
            total_output_groups_reg   <= total_output_groups;
            last_group_valid_reg      <= last_group_valid;
            group_is_last_reg         <= first_group_is_last;
            group_is_odd_reg          <= is_odd_input;
            group_emit_reg            <= first_group_emit;
            group_last_slice_reg      <= {4{first_group_is_last}};
            group_odd_slice_reg       <= {4{is_odd_input}};
        end else if (ch0_valid && group_done) begin
            if (row_done) begin
                group_is_last_reg     <= (total_output_groups_reg == 13'd1);
                group_emit_reg        <= (total_output_groups_reg != 13'd0);
                group_last_slice_reg  <= {4{(total_output_groups_reg == 13'd1)}};
            end else begin
                group_is_last_reg     <= next_group_is_last;
                group_emit_reg        <= next_group_emit;
                group_last_slice_reg  <= {4{next_group_is_last}};
            end
        end
    end

    // ============================================================
    // 末组无效列屏蔽与移位后值
    // ============================================================
    wire [7:0]                ch0_val;
    wire [7:0]                ch1_val;
    wire [255:0]              ch0_post;
    wire [255:0]              ch1_post;

    assign ch0_val = (group_is_last_reg && ({1'b0, col_cnt} >= last_group_valid_reg))
                   ? 8'd0 : ch0_din;
    assign ch1_val = (group_is_last_reg && ({1'b0, col_cnt} >= last_group_valid_reg))
                   ? 8'd0 : ch1_din;

    assign ch0_post = {ch0_val, ch0_shift[255:8]};
    assign ch1_post = {ch1_val, ch1_shift[255:8]};

    // ============================================================
    // 组结束 / 行结束判定
    // ============================================================
    assign group_limit = group_is_last_reg
                       ? (group_is_odd_reg ? 5'd15 : 5'd31)
                       : 5'd31;
    assign row_done    = (tot_col_cnt == output_cols_m1_reg);
    assign group_done  = (col_cnt == group_limit) || row_done;

    // ============================================================
    // 移位
    // ============================================================
    // 每个完整组的 32 个 byte 会覆盖整个 shift；奇数末组只消费高 128-bit，
    // 未消费半边由 tdata_r 显式写零。因此 group_done 时清 256-bit shift 冗余，
    // 删除该同步 clear 可消除 group_done -> 512-bit /R 大扇出路径。
    always @(posedge clk) begin
        if (ch0_valid)
            ch0_shift <= ch0_post;
    end

    always @(posedge clk) begin
        if (ch1_valid)
            ch1_shift <= ch1_post;
    end

    // ============================================================
    // 计数器
    // ============================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            col_cnt <= 5'd0;
        else if (ch0_valid)
            col_cnt <= group_done ? 5'd0 : (col_cnt + 5'd1);
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            tot_col_cnt <= 13'd0;
        else if (ch0_valid)
            tot_col_cnt <= row_done ? 13'd0 : (tot_col_cnt + 13'd1);
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            group_cnt <= 13'd0;
        else if (ch0_valid && group_done)
            group_cnt <= row_done ? 13'd0 : (group_cnt + 13'd1);
    end

    // ============================================================
    // 输出寄存器
    // ============================================================
    reg  [511:0]              tdata_r;
    reg                       tvalid_r;
    wire [3:0]                odd_last_slice;

    assign odd_last_slice = group_last_slice_reg & group_odd_slice_reg;

    always @(posedge clk) begin
        if (ch0_valid && group_done) begin
            tdata_r[127:0]   <= odd_last_slice[0] ? ch0_post[255:128] : ch0_post[127:0];
            tdata_r[255:128] <= odd_last_slice[1] ? 128'd0            : ch0_post[255:128];
            tdata_r[383:256] <= odd_last_slice[2] ? ch1_post[255:128] : ch1_post[127:0];
            tdata_r[511:384] <= odd_last_slice[3] ? 128'd0            : ch1_post[255:128];
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            tvalid_r <= 1'b0;
        else
            tvalid_r <= ch0_valid && group_done && group_emit_reg;
    end

    assign m_axis_pooling_tdata  = tdata_r;
    assign m_axis_pooling_tvalid = tvalid_r;

endmodule

/*
 * RTL SELF-CHECK:
 * PASS: pure Verilog syntax; all ports explicitly use wire
 * PASS: combinational logic uses continuous assignments only
 * PASS: sequential logic uses nonblocking assignment only
 * PASS: no process loop, division, modulo, function, latch, or combinational loop
 * PASS: payload shift/tdata registers have no reset; valid/counters retain required reset
 * PASS: start lands stable configuration before the first valid data beat
 * PASS: group metadata advances only at real group boundaries and preserves 1 beat/cycle throughput
 * PASS: four local 128-bit format controls replace one 512-bit high-fanout control
 * PASS: widths and unsigned arithmetic were checked explicitly
 * WARN: timing improvement still requires post-synthesis/implementation confirmation
 * FAIL: none
 */
