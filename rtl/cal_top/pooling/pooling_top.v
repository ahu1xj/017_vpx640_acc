// ============================================================
// pooling_top — 池化模块顶层
// ============================================================

module pooling_top (
    input  wire               clk,
    input  wire               rst_n,

    // 指令
    input  wire               rdcmd_trans_valid,
    input  wire [111:0]       rdcmd_trans,

    // Padding 输入
    input  wire               s_axis_ppooltem_tvalid,
    input  wire [15:0]        s_axis_ppooltem_tdata,
    output wire               s_axis_ppooltem_tready,

    // 特征图输入
    input  wire               s_axis_fpooltem_tvalid,
    input  wire [15:0]        s_axis_fpooltem_tdata,
    output wire               s_axis_fpooltem_tready,

    // 池化输出
    output wire [511:0]       m_axis_pooling_tdata,
    output wire               m_axis_pooling_tvalid,
    input  wire               m_axis_pooling_tready
);

    // ============================================================
    // ctrl → 各模块 信号
    // ============================================================
    wire                      start;
    wire                      circular_padding;
    wire [1:0]                padding_top;
    wire [1:0]                padding_bottom;
    wire [11:0]               fm_row_in;
    wire [4:0]                last_fm_out_column_valid;
    wire [12:0]               total_cols_per_row;
    wire [12:0]               fm_total_rows;
    wire [10:0]               ch_group_total;
    wire [12:0]               last_group_start;
    wire [2:0]                kernel_size_c;
    wire [1:0]                stride_c;
    wire [12:0]               output_cols_per_row;
    wire [2:0]                kernel_size_r;
    wire [1:0]                stride_r;
    wire [12:0]               total_rows_per_group;
    wire [12:0]               total_output_groups;
    wire [5:0]                last_group_valid;
    wire                      is_odd_input;

    // ============================================================
    // input → h_kernel 信号
    // ============================================================
    wire [7:0]                input_ch0;
    wire [7:0]                input_ch1;
    wire                      input_valid;

    // ============================================================
    // h_kernel → v_kernel 信号
    // ============================================================
    wire signed [7:0]         h_out0, h_out1;
    wire                      h_valid0, h_valid1;

    // ============================================================
    // v_kernel → packer 信号
    // ============================================================
    wire signed [7:0]         v_out0, v_out1;
    wire                      v_valid0, v_valid1;

    // Packer output is a fixed-latency internal stream.  A local FIFO absorbs
    // the bounded in-flight tail when external output stalls.
    wire [511:0]              packer_tdata;
    wire                      packer_tvalid;
    wire                      packer_fifo_ready;
    reg  [6:0]                pooling_fifo_count_reg;
    wire                      packer_fifo_write;
    wire                      pooling_output_fire;
    reg                       input_credit_ready_reg;

    // ============================================================
    // pooling_ctrl
    // ============================================================
    pooling_ctrl u_ctrl (
        .clk                     (clk),
        .rst_n                   (rst_n),
        .rdcmd_trans_valid       (rdcmd_trans_valid),
        .rdcmd_trans             (rdcmd_trans),
        .start                   (start),
        .circular_padding        (circular_padding),
        .padding_top             (padding_top),
        .padding_bottom          (padding_bottom),
        .fm_row_in               (fm_row_in),
        .last_fm_out_column_valid(last_fm_out_column_valid),
        .total_cols_per_row      (total_cols_per_row),
        .fm_total_rows           (fm_total_rows),
        .ch_group_total          (ch_group_total),
        .last_group_start        (last_group_start),
        .kernel_size_c           (kernel_size_c),
        .stride_c                (stride_c),
        .output_cols_per_row     (output_cols_per_row),
        .kernel_size_r           (kernel_size_r),
        .stride_r                (stride_r),
        .total_rows_per_group    (total_rows_per_group),
        .total_output_groups     (total_output_groups),
        .last_group_valid        (last_group_valid),
        .is_odd_input            (is_odd_input)
    );

    // ============================================================
    // pooling_input
    // ============================================================
    pooling_input u_input (
        .clk                     (clk),
        .rst_n                   (rst_n),
        .s_axis_ppooltem_tvalid  (s_axis_ppooltem_tvalid),
        .s_axis_ppooltem_tdata   (s_axis_ppooltem_tdata),
        .s_axis_ppooltem_tready  (s_axis_ppooltem_tready),
        .s_axis_fpooltem_tvalid  (s_axis_fpooltem_tvalid),
        .s_axis_fpooltem_tdata   (s_axis_fpooltem_tdata),
        .s_axis_fpooltem_tready  (s_axis_fpooltem_tready),
        .input_credit_ready      (input_credit_ready_reg),
        .start                   (start),
        .circular_padding        (circular_padding),
        .padding_top             (padding_top),
        .padding_bottom          (padding_bottom),
        .fm_row_in               (fm_row_in),
        .last_fm_out_column_valid(last_fm_out_column_valid),
        .total_cols_per_row      (total_cols_per_row),
        .fm_total_rows           (fm_total_rows),
        .ch_group_total          (ch_group_total),
        .last_group_start        (last_group_start),
        .input_done              (),
        .ch0_din                 (input_ch0),
        .ch1_din                 (input_ch1),
        .din_valid               (input_valid)
    );

    // ============================================================
    // pooling_h_kernel ×2
    // ============================================================
    pooling_h_kernel u_h0 (
        .clk                 (clk),
        .rst_n               (rst_n),
        .din                 (input_ch0),
        .din_valid           (input_valid),
        .kernel_size_c       (kernel_size_c),
        .stride_c            (stride_c),
        .total_cols_per_row  (total_cols_per_row),
        .output_cols_per_row (output_cols_per_row),
        .h_out               (h_out0),
        .h_out_valid         (h_valid0)
    );

    pooling_h_kernel u_h1 (
        .clk                 (clk),
        .rst_n               (rst_n),
        .din                 (input_ch1),
        .din_valid           (input_valid),
        .kernel_size_c       (kernel_size_c),
        .stride_c            (stride_c),
        .total_cols_per_row  (total_cols_per_row),
        .output_cols_per_row (output_cols_per_row),
        .h_out               (h_out1),
        .h_out_valid         (h_valid1)
    );

    // ============================================================
    // pooling_v_kernel ×2
    // ============================================================
    pooling_v_kernel u_v0 (
        .clk                 (clk),
        .rst_n               (rst_n),
        .h_in                (h_out0),
        .h_in_valid          (h_valid0),
        .kernel_size_r       (kernel_size_r),
        .stride_r            (stride_r),
        .output_cols_per_row (output_cols_per_row),
        .total_rows_per_group(total_rows_per_group),
        .ch_group_total      (ch_group_total),
        .padding_top         (padding_top),
        .circular_padding    (circular_padding),
        .v_out               (v_out0),
        .v_out_valid         (v_valid0)
    );

    pooling_v_kernel u_v1 (
        .clk                 (clk),
        .rst_n               (rst_n),
        .h_in                (h_out1),
        .h_in_valid          (h_valid1),
        .kernel_size_r       (kernel_size_r),
        .stride_r            (stride_r),
        .output_cols_per_row (output_cols_per_row),
        .total_rows_per_group(total_rows_per_group),
        .ch_group_total      (ch_group_total),
        .padding_top         (padding_top),
        .circular_padding    (circular_padding),
        .v_out               (v_out1),
        .v_out_valid         (v_valid1)
    );

    // ============================================================
    // pooling_packer
    // ============================================================
    pooling_packer u_packer (
        .clk                 (clk),
        .rst_n               (rst_n),
        .start               (start),
        .ch0_din             (v_out0),
        .ch0_valid           (v_valid0),
        .ch1_din             (v_out1),
        .ch1_valid           (v_valid1),
        .output_cols_per_row (output_cols_per_row),
        .total_output_groups (total_output_groups),
        .last_group_valid    (last_group_valid),
        .is_odd_input        (is_odd_input),
        .m_axis_pooling_tdata(packer_tdata),
        .m_axis_pooling_tvalid(packer_tvalid)
    );


    // The h/v/packer chain has no ready path.  Stop accepting new 16-bit input
    // beats while at least 32 FIFO entries remain free.  The registered gate
    // keeps the occupancy comparator off the upstream ready timing path.
    // After input stops, only the fixed h/v pipeline tail and one partial pack
    // can produce output; the 32-entry reserve is intentionally conservative.
    assign packer_fifo_write  = packer_tvalid && packer_fifo_ready;
    assign pooling_output_fire = m_axis_pooling_tvalid && m_axis_pooling_tready;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            pooling_fifo_count_reg <= 7'd0;
        else begin
            case ({packer_fifo_write, pooling_output_fire})
                2'b10:   pooling_fifo_count_reg <= pooling_fifo_count_reg + 7'd1;
                2'b01:   pooling_fifo_count_reg <= pooling_fifo_count_reg - 7'd1;
                default: pooling_fifo_count_reg <= pooling_fifo_count_reg;
            endcase
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            input_credit_ready_reg <= 1'b1;
        else
            input_credit_ready_reg <= (pooling_fifo_count_reg < 7'd32);
    end

    // Dedicated 64x512 common-clock AXIS FIFO.  Occupancy is mirrored locally
    // so the early-credit comparator remains small and registered.
    axis_data_fifo_512x64_sync u_pooling_output_fifo (
        .s_axis_aresetn(rst_n),
        .s_axis_aclk   (clk),
        .s_axis_tvalid (packer_tvalid),
        .s_axis_tdata  (packer_tdata),
        .s_axis_tready (packer_fifo_ready),
        .m_axis_tvalid (m_axis_pooling_tvalid),
        .m_axis_tdata  (m_axis_pooling_tdata),
        .m_axis_tready (m_axis_pooling_tready)
    );
endmodule

// ============================================================
// RTL Self-Check Results
// ============================================================
// [PASS] Rule #1:  All ports have explicit direction and width
// [PASS] Rule #2:  Ports use wire; credit state uses reg
// [PASS] Rule #6:  No debug/error-attribution logic in RTL
// [PASS] Rule #8:  Pure Verilog; no SystemVerilog keywords
// [PASS] Rule #12: No combinational always block or inferred latch
// [PASS] Rule #16: Non-blocking assignment used for credit register
// [PASS] Rule #24: No new multiplier or wide arithmetic on control paths
// [PASS] FIFO rule: external M_AXIS is fully registered
// [PASS] Timing rule: external ready terminates at the local FIFO
// [PASS] RAM rule: 64x512 output queue uses the dedicated vendor block-RAM AXIS FIFO
// [PASS] Packer rule: start provides a registered per-operation metadata initialization boundary
// ============================================================
