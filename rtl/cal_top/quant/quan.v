`include "all_defines.vh"

// =============================================================================
// quan
// Quantization control top: command/param latch, conv/FM input issue,
// QUAN_UNITNUM-lane quan_unit array, RESADD/SPLIT/CONCAT/UPSAMPLE output
// routing with one registered 2-entry skid per output direction.
//
// Timing rework note (20260724 Phase A):
// - The issue stage remains registered. CONV receives four registered
//   1024-bit PE candidates, applies quarter-local pairwise 2:1 selection,
//   crosses a locked five-skid ingress boundary, then applies a final 2:1.
//   The only 65536-bit snapshot is PE-local; quan no longer owns a copy.
// - quan_unit latency is 4 cycles; issue register adds 1, so issue accept ->
//   token out = 5 cycles (was 2+1=3 externally visible). All consumers are
//   level-based handshakes, so the extra latency is contract-safe.
// - src/high_half/conv/int16 metadata travel on an equal-length sideband
//   pipeline; the final stage keeps the legacy names unit_out_*. Token valid
//   is produced by the same sideband (no more lane-0 valid dependency).
// - UPSAMPLE bypass is registered one cycle (upsamp_data_r/upsamp_pending_r)
//   instead of a 512b combinational feed-through into the skid.
// - ceil_pair_count function removed; pair counts are explicit expressions.
// - Residual outputs terminate in registered 2-entry skids; the external
//   cal_top Vivado FIFOs remain the only deep buffering at that boundary.
// =============================================================================
module quan(
    input  wire                                      clk,
    input  wire                                      rst_n,
    input  wire                                      rdcmd_trans_valid,
    input  wire [`QUAN_INCMD-1:0]                   rdcmd_trans,
    input  wire                                      s_axis_quan_param_tvalid,
    input  wire [`QUAN_PARINDATA_WIDTH-1:0]         s_axis_quan_param_tdata,
    output wire                                      s_axis_quan_param_tready,
    input  wire                                      s_axis_quan_fm1_tvalid,
    input  wire [`QUAN_FMINDATA_WIDTH-1:0]          s_axis_quan_fm1_tdata,
    output wire                                      s_axis_quan_fm1_tready,
    input  wire                                      s_axis_quan_fm2_tvalid,
    input  wire [`QUAN_FMINDATA_WIDTH-1:0]          s_axis_quan_fm2_tdata,
    output wire                                      s_axis_quan_fm2_tready,
    input  wire                                      conv_valid,
    output wire                                      conv_candidate_start,
    output wire                                      conv_candidate_advance,
    output wire                                      conv_candidate_done,
    input  wire                                      conv_candidate_valid,
    input  wire [4095:0]                            conv_candidate_data,
    output wire                                      conv_ready,
    output wire                                      m_axis_res_fm1_tvalid,
    output wire [`QUAN_RESOUTDATA_WIDTH-1:0]        m_axis_res_fm1_tdata,
    input  wire                                      m_axis_res_fm1_tready,
    output wire                                      m_axis_res_fm2_tvalid,
    output wire [`QUAN_RESOUTDATA_WIDTH-1:0]        m_axis_res_fm2_tdata,
    input  wire                                      m_axis_res_fm2_tready,
    output wire                                      m_axis_quan_act_tvalid,
    output wire [`QUAN_ACTOUTDATA_WIDTH-1:0]        m_axis_quan_act_tdata,
    input  wire                                      m_axis_quan_act_tready
);

localparam MODEL_IDLE       = 3'd0;
localparam MODEL_CONV       = 3'd1;
localparam MODEL_RESADD     = 3'd2;
localparam MODEL_UPSAMPLE   = 3'd3;
localparam MODEL_CONCAT     = 3'd4;
localparam MODEL_SPLIT      = 3'd5;

localparam PAR_NUM          = 64;
localparam PAR_BEAT_NUM     = 8;
localparam PAR_IDX_WIDTH    = 6;
localparam PAR_BEAT_WIDTH   = 3;
localparam PARAM_WIDTH      = 64;
localparam FM_CNT_WIDTH     = `RD_FM_CHANNEL_IN_WIDTH;

localparam SCALE_WIDTH      = 32;
localparam SHIFT_WIDTH      = 6;
localparam ACC_WIDTH        = 32;
localparam QOUT_WIDTH       = 8;
localparam QOUT16_WIDTH     = 16;
localparam QUAN_NUM         = `QUAN_UNITNUM;
localparam CONV_DATA_SKID_WIDTH = 513;
localparam CONV_CTRL_SKID_WIDTH = 72;
localparam SRC_NONE         = 2'd0;
localparam SRC_FM1          = 2'd1;
localparam SRC_FM2          = 2'd2;

// =============================================================================
// Register declarations
// =============================================================================
reg [2:0]                                  model_r;
reg                                        param_loading;
reg [PAR_BEAT_WIDTH-1:0]                   param_beat_cnt;
reg [PARAM_WIDTH-1:0]                      param_mem [0:PAR_NUM-1];

reg                                        conv_busy;
reg                                        conv_capture_pending_r;
reg [PAR_IDX_WIDTH-1:0]                    conv_candidate_idx_r;
reg [3:0]                                  conv_param_load_idx_r;
reg [PAR_IDX_WIDTH-1:0]                    conv_issue_idx_r;
reg [PARAM_WIDTH-1:0]                      conv_param_candidate_g0_r;
reg [PARAM_WIDTH-1:0]                      conv_param_candidate_g1_r;
reg [PARAM_WIDTH-1:0]                      conv_param_candidate_g2_r;
reg [PARAM_WIDTH-1:0]                      conv_param_candidate_g3_r;
reg                                        conv_out_type_r;
reg [2:0]                                  concat_input_num_r;
reg [PAR_IDX_WIDTH-1:0]                    concat_param_idx_f;
reg [FM_CNT_WIDTH-1:0]                     concat_pair_cnt_f;
reg [FM_CNT_WIDTH-1:0]                     concat_pair_num [0:3];
reg                                        resadd_port_sel;
reg [FM_CNT_WIDTH-1:0]                     fm_pair_num_r;
reg [FM_CNT_WIDTH-1:0]                     fm_channel_pair_cnt;
reg [FM_CNT_WIDTH-1:0]                     res_p0_pair_cnt;
reg [FM_CNT_WIDTH-1:0]                     res_p1_pair_cnt;

reg                                        fm_busy;
reg [1:0]                                  fm_src_r;
reg [`QUAN_FMINDATA_WIDTH-1:0]             fm_data_r;
reg [PARAM_WIDTH-1:0]                      fm_param_r;
reg                                        fm_second_issue;
reg [255:0]                                split_low_q;

// UPSAMPLE registered elastic stage before the FM1 skid
reg [`QUAN_RESOUTDATA_WIDTH-1:0]           upsamp_data_r;
reg                                        upsamp_pending_r;

// Local pipeline advance. The activation skid ready is derived only from its
// registered occupancy, so no downstream ready path reaches the quan pipeline.
// Vivado may replicate this local CE to the 32 quan lanes as needed.
(* max_fanout = 32 *) wire [QUAN_NUM-1:0]  pipe_ce_lane;
(* max_fanout = 32 *) wire                 pipe_ce_ctrl;

// Issue stage (registered; replaces the old combinational issue network)
reg                                        issue_valid_r;
reg signed [ACC_WIDTH-1:0]                 issue_acc_r [0:QUAN_NUM-1];
reg signed [SCALE_WIDTH-1:0]               issue_scale_r;
reg [SHIFT_WIDTH-1:0]                      issue_shift_r;
reg [1:0]                                  issue_src_r;
reg                                        issue_high_half_r;
reg                                        issue_conv_r;
reg                                        issue_int16_r;

// Sideband pipeline stage 1..6 (final stage keeps the legacy unit_out_* names)
reg                                        tk_valid_s1;
reg                                        tk_valid_s2;
reg                                        tk_valid_s3;
reg                                        tk_valid_s4;
reg                                        tk_valid_s5;
reg                                        tk_valid_s6;
reg                                        tk_valid_s7;
reg [1:0]                                  src_s1;
reg [1:0]                                  src_s2;
reg [1:0]                                  src_s3;
reg [1:0]                                  src_s4;
reg [1:0]                                  src_s5;
reg [1:0]                                  src_s6;
reg                                        high_s1;
reg                                        high_s2;
reg                                        high_s3;
reg                                        high_s4;
reg                                        high_s5;
reg                                        high_s6;
reg                                        conv_s1;
reg                                        conv_s2;
reg                                        conv_s3;
reg                                        conv_s4;
reg                                        conv_s5;
reg                                        conv_s6;
reg                                        int16_s1;
reg                                        int16_s2;
reg                                        int16_s3;
reg                                        int16_s4;
reg                                        int16_s5;
reg                                        int16_s6;

// Sideband final stage: legacy names, consumed by output/routing logic
reg [1:0]                                  unit_out_src;
reg                                        unit_out_high_half;
reg                                        unit_out_conv;
reg                                        unit_out_int16;


// =============================================================================
// Command fields and explicit pair-count expressions (no function)
// =============================================================================
wire [`RD_MODEL_SEL_WIDTH-1:0]             cmd_model_sel;
wire                                       cmd_supported;
wire                                       param_model;
wire [`RD_FM_CHANNEL_IN_WIDTH-1:0]         cmd_fm_channel_in;
wire [`RD_CONCAT_FM2_IN_CHANNEL_WIDTH-1:0] cmd_concat_fm2_channel;
wire [`RD_CONCAT_FM3_IN_CHANNEL_WIDTH-1:0] cmd_concat_fm3_channel;
wire [`RD_CONCAT_FM4_IN_CHANNEL_WIDTH-1:0] cmd_concat_fm4_channel;
wire [`RD_CONCAT_TYPE_WIDTH-1:0]            cmd_concat_type;
wire                                       cmd_conv_out_type;
wire [FM_CNT_WIDTH-1:0]                    cmd_fm_pair_num;
wire [FM_CNT_WIDTH-1:0]                    cmd_fm2_pair_num;
wire [FM_CNT_WIDTH-1:0]                    cmd_fm3_pair_num;
wire [FM_CNT_WIDTH-1:0]                    cmd_fm4_pair_num;
wire [FM_CNT_WIDTH-1:0]                    concat_current_pair_num_f;
wire [PAR_IDX_WIDTH-1:0]                   concat_last_param_idx;
wire [2:0]                                 cmd_model_next;

assign cmd_model_sel = rdcmd_trans[`RD_MODEL_SEL_START+:`RD_MODEL_SEL_WIDTH];
assign cmd_supported = (cmd_model_sel == `MODEL_SEL_CONV)    |
                       (cmd_model_sel == `MODEL_SEL_RESADD)  |
                       (cmd_model_sel == `MODEL_SEL_UPSAMPLE) |
                       (cmd_model_sel == `MODEL_SEL_CONCAT)   |
                       (cmd_model_sel == `MODEL_SEL_SPLIT);
assign param_model   = (cmd_model_sel == `MODEL_SEL_CONV)   |
                       (cmd_model_sel == `MODEL_SEL_RESADD) |
                       (cmd_model_sel == `MODEL_SEL_CONCAT) |
                       (cmd_model_sel == `MODEL_SEL_SPLIT);
assign cmd_fm_channel_in = rdcmd_trans[`RD_FM_CHANNEL_IN_START+:`RD_FM_CHANNEL_IN_WIDTH];
assign cmd_concat_fm2_channel = rdcmd_trans[`RD_CONCAT_FM2_IN_CHANNEL_START+:`RD_CONCAT_FM2_IN_CHANNEL_WIDTH];
assign cmd_concat_fm3_channel = rdcmd_trans[`RD_CONCAT_FM3_IN_CHANNEL_START+:`RD_CONCAT_FM3_IN_CHANNEL_WIDTH];
assign cmd_concat_fm4_channel = rdcmd_trans[`RD_CONCAT_FM4_IN_CHANNEL_START+:`RD_CONCAT_FM4_IN_CHANNEL_WIDTH];
assign cmd_concat_type = rdcmd_trans[`RD_CONCAT_TYPE_START+:`RD_CONCAT_TYPE_WIDTH];
assign cmd_conv_out_type = rdcmd_trans[`RD_CONV_OUT_TYPE_START+:`RD_CONV_OUT_TYPE_WIDTH];
assign cmd_fm_pair_num  = (cmd_fm_channel_in == {FM_CNT_WIDTH{1'b0}}) ?
                          {{(FM_CNT_WIDTH-1){1'b0}},1'b1} :
                          ((cmd_fm_channel_in >> 1) + {{(FM_CNT_WIDTH-1){1'b0}}, cmd_fm_channel_in[0]});
assign cmd_fm2_pair_num = (cmd_concat_fm2_channel == {FM_CNT_WIDTH{1'b0}}) ?
                          {{(FM_CNT_WIDTH-1){1'b0}},1'b1} :
                          ((cmd_concat_fm2_channel >> 1) + {{(FM_CNT_WIDTH-1){1'b0}}, cmd_concat_fm2_channel[0]});
assign cmd_fm3_pair_num = (cmd_concat_fm3_channel == {FM_CNT_WIDTH{1'b0}}) ?
                          {{(FM_CNT_WIDTH-1){1'b0}},1'b1} :
                          ((cmd_concat_fm3_channel >> 1) + {{(FM_CNT_WIDTH-1){1'b0}}, cmd_concat_fm3_channel[0]});
assign cmd_fm4_pair_num = (cmd_concat_fm4_channel == {FM_CNT_WIDTH{1'b0}}) ?
                          {{(FM_CNT_WIDTH-1){1'b0}},1'b1} :
                          ((cmd_concat_fm4_channel >> 1) + {{(FM_CNT_WIDTH-1){1'b0}}, cmd_concat_fm4_channel[0]});
assign cmd_model_next = (cmd_model_sel == `MODEL_SEL_CONV)     ? MODEL_CONV :
                        (cmd_model_sel == `MODEL_SEL_RESADD)   ? MODEL_RESADD :
                        (cmd_model_sel == `MODEL_SEL_UPSAMPLE) ? MODEL_UPSAMPLE :
                        (cmd_model_sel == `MODEL_SEL_CONCAT)   ? MODEL_CONCAT :
                        (cmd_model_sel == `MODEL_SEL_SPLIT)    ? MODEL_SPLIT :
                                                                  MODEL_IDLE;

// =============================================================================
// Input ready / fire and control wires
// =============================================================================
wire                                       issue_slot_free;
wire                                       fm1_fire;
wire                                       fm2_fire;
wire                                       conv_issue;
wire                                       quan_out_valid;
wire                                       pipe_ce_req;
wire                                       quan_act_skid_s_ready;
wire                                       quan_act_skid_s_valid;
wire [`QUAN_ACTOUTDATA_WIDTH-1:0]          quan_act_skid_s_data;
wire                                       quan_res_fm1_skid_s_ready;
wire                                       quan_res_fm1_pipe_valid;
wire                                       quan_res_fm1_skid_s_valid;
wire [`QUAN_RESOUTDATA_WIDTH-1:0]          quan_res_fm1_skid_s_data;
wire                                       quan_res_fm1_pipe_fire;
wire                                       quan_res_fm2_skid_s_ready;
wire                                       quan_res_fm2_pipe_valid;
wire                                       quan_res_fm2_skid_s_valid;
wire [`QUAN_RESOUTDATA_WIDTH-1:0]          quan_res_fm2_skid_s_data;
wire                                       quan_res_fm2_pipe_fire;
wire                                       upsamp_drain;
wire [`QUAN_ACTOUTDATA_WIDTH-1:0]          quan_out_pack;
wire [`QUAN_ACTOUTDATA_WIDTH-1:0]          quan_out_pack16;
wire signed [QOUT_WIDTH-1:0]               quan_unit_out_i8 [0:QUAN_NUM-1];
wire signed [QOUT16_WIDTH-1:0]             quan_unit_out_i16 [0:QUAN_NUM-1];
wire [QUAN_NUM-1:0]                        quan_unit_valid_unused;

wire [3:0]                                 conv_param_select_low;
wire                                       conv_param_load_en;
wire [1023:0]                              conv_candidate_selected;
wire [PARAM_WIDTH-1:0]                     conv_param_group0_selected;
wire [PARAM_WIDTH-1:0]                     conv_param_group1_selected;
wire [PARAM_WIDTH-1:0]                     conv_param_group2_selected;
wire [PARAM_WIDTH-1:0]                     conv_param_group3_selected;
wire [PARAM_WIDTH-1:0]                     conv_candidate_source_param;
wire [PARAM_WIDTH-1:0]                     conv_candidate_param;
wire                                       conv_candidate_source_valid;
wire                                       conv_candidate_push;
wire                                       conv_ingress_all_s_ready;
wire                                       conv_ingress_all_m_valid;
wire                                       conv_ingress_pop;
wire                                       conv_ingress_issue_done;
wire                                       conv_ingress_last;
wire                                       conv_ingress_out_type;
wire [PAR_IDX_WIDTH-1:0]                   conv_ingress_idx;
wire [CONV_CTRL_SKID_WIDTH-1:0]            conv_ctrl_skid_s_data;
wire [CONV_CTRL_SKID_WIDTH-1:0]            conv_ctrl_skid_m_data;
wire                                       conv_ctrl_skid_s_ready;
wire                                       conv_ctrl_skid_m_valid;
wire                                       conv_q0_skid_s_ready;
wire                                       conv_q1_skid_s_ready;
wire                                       conv_q2_skid_s_ready;
wire                                       conv_q3_skid_s_ready;
wire                                       conv_q0_skid_m_valid;
wire                                       conv_q1_skid_m_valid;
wire                                       conv_q2_skid_m_valid;
wire                                       conv_q3_skid_m_valid;
wire [255:0]                               conv_q0_skid_m_data;
wire [255:0]                               conv_q1_skid_m_data;
wire [255:0]                               conv_q2_skid_m_data;
wire [255:0]                               conv_q3_skid_m_data;
wire [PARAM_WIDTH-1:0]                     resadd_fm1_param;
wire [PARAM_WIDTH-1:0]                     resadd_fm2_param;
wire [PARAM_WIDTH-1:0]                     concat_param;
wire [PARAM_WIDTH-1:0]                     split_param;
wire [PARAM_WIDTH-1:0]                     fm1_next_param;
wire [PARAM_WIDTH-1:0]                     fm2_next_param;

// RESADD 端口切换粒度: 每 32 通道对 (64 通道) 切换一次 fm1/fm2 端口,
// 与 load_core 的 64 通道组粒度供流一致 (load 发 [FM1 组 i][FM2 组 i])。
// 两端口各自记账当前 tile 内已处理对数: 当前组长度 = min(32, 本 FM 本 tile
// 剩余对数); 本 FM 满 fm_pair_num_r 对后其计数归零 (下一 tile 重新开始)。
wire [FM_CNT_WIDTH-1:0]                    res_cur_pair_cnt;
wire [FM_CNT_WIDTH-1:0]                    res_grp_pair_left;
wire [5:0]                                 res_grp_pair_limit;
assign res_cur_pair_cnt  = resadd_port_sel ? res_p1_pair_cnt : res_p0_pair_cnt;
assign res_grp_pair_left = fm_pair_num_r - res_cur_pair_cnt;
assign res_grp_pair_limit = (res_grp_pair_left >= {{(FM_CNT_WIDTH-7){1'b0}}, 7'd32})
                            ? 6'd32 : res_grp_pair_left[5:0];

assign s_axis_quan_param_tready = param_loading;

// FM issue accepts a new beat only when the shared quan pipeline advances.
// During the second half issue, the current beat may retire while the next beat
// is captured, preserving the original one input beat per two cycles cadence.
assign issue_slot_free = ~fm_busy | (fm_busy & fm_second_issue);

assign s_axis_quan_fm1_tready = (model_r == MODEL_UPSAMPLE) ?
                                    (~param_loading & ((~upsamp_pending_r) | quan_res_fm1_skid_s_ready)) :
                                ((model_r == MODEL_CONCAT) | (model_r == MODEL_SPLIT)) ?
                                    (issue_slot_free & pipe_ce_ctrl & ~param_loading) :
                                (model_r == MODEL_RESADD) ?
                                    (issue_slot_free & pipe_ce_ctrl & ~param_loading & (resadd_port_sel == 1'b0)) :
                                    1'b0;

assign s_axis_quan_fm2_tready = (model_r == MODEL_RESADD) ?
                                    (issue_slot_free & pipe_ce_ctrl & ~param_loading & (resadd_port_sel == 1'b1)) :
                                    1'b0;

assign fm1_fire = s_axis_quan_fm1_tvalid & s_axis_quan_fm1_tready;
assign fm2_fire = s_axis_quan_fm2_tvalid & s_axis_quan_fm2_tready;

assign conv_ready = (model_r == MODEL_CONV) & ~param_loading & ~conv_busy;
assign conv_candidate_source_valid = conv_busy & conv_candidate_valid & ~param_loading;
assign conv_ingress_all_s_ready = conv_q0_skid_s_ready & conv_q1_skid_s_ready &
                                  conv_q2_skid_s_ready & conv_q3_skid_s_ready &
                                  conv_ctrl_skid_s_ready;
assign conv_candidate_push = conv_candidate_source_valid & conv_ingress_all_s_ready;
assign conv_ingress_all_m_valid = conv_q0_skid_m_valid & conv_q1_skid_m_valid &
                                  conv_q2_skid_m_valid & conv_q3_skid_m_valid &
                                  conv_ctrl_skid_m_valid;
assign conv_ingress_pop = conv_ingress_all_m_valid & pipe_ce_ctrl;
assign conv_issue = conv_ingress_pop;
assign conv_ingress_issue_done = conv_ingress_pop & conv_ingress_last;
assign conv_candidate_start = conv_capture_pending_r;
assign conv_candidate_advance = conv_candidate_push & (conv_candidate_idx_r != (PAR_NUM - 1));
assign conv_candidate_done = conv_candidate_push & (conv_candidate_idx_r == (PAR_NUM - 1));
assign conv_param_select_low = conv_param_load_idx_r;
assign conv_param_load_en = conv_candidate_start | conv_candidate_advance;

// The arithmetic pipeline advances unless its current tail token targets a
// full local skid. All ready inputs below come only from registered local
// occupancy; no cal_top FIFO ready propagates combinationally into the lanes.
assign pipe_ce_req = ((~quan_act_skid_s_valid) | quan_act_skid_s_ready) &
                     ((~quan_res_fm1_pipe_valid) | quan_res_fm1_skid_s_ready) &
                     ((~quan_res_fm2_pipe_valid) | quan_res_fm2_skid_s_ready);
assign pipe_ce_ctrl = pipe_ce_req;
assign pipe_ce_lane = {QUAN_NUM{pipe_ce_req}};

// Four quarter data skids plus one narrow control skid form one aggregate
// two-entry registered ingress boundary. Push/pop are common, so legal reset
// operation keeps all five occupancies and valid states in lockstep.
(* keep_hierarchy = "yes" *) quan_conv_ingress_quarter u_conv_ingress_q0(
    .clk                (clk),
    .rst_n              (rst_n),
    .s_axis_tvalid      (conv_candidate_push),
    .s_axis_tready      (conv_q0_skid_s_ready),
    .source_group_low   (conv_candidate_idx_r[4]),
    .source_group_high  (conv_candidate_idx_r[5]),
    .group0_data        (conv_candidate_data[0+:256]),
    .group1_data        (conv_candidate_data[1024+:256]),
    .group2_data        (conv_candidate_data[2048+:256]),
    .group3_data        (conv_candidate_data[3072+:256]),
    .m_axis_tvalid      (conv_q0_skid_m_valid),
    .m_axis_tready      (conv_ingress_pop),
    .m_axis_tdata       (conv_q0_skid_m_data)
);

(* keep_hierarchy = "yes" *) quan_conv_ingress_quarter u_conv_ingress_q1(
    .clk                (clk),
    .rst_n              (rst_n),
    .s_axis_tvalid      (conv_candidate_push),
    .s_axis_tready      (conv_q1_skid_s_ready),
    .source_group_low   (conv_candidate_idx_r[4]),
    .source_group_high  (conv_candidate_idx_r[5]),
    .group0_data        (conv_candidate_data[256+:256]),
    .group1_data        (conv_candidate_data[1280+:256]),
    .group2_data        (conv_candidate_data[2304+:256]),
    .group3_data        (conv_candidate_data[3328+:256]),
    .m_axis_tvalid      (conv_q1_skid_m_valid),
    .m_axis_tready      (conv_ingress_pop),
    .m_axis_tdata       (conv_q1_skid_m_data)
);

(* keep_hierarchy = "yes" *) quan_conv_ingress_quarter u_conv_ingress_q2(
    .clk                (clk),
    .rst_n              (rst_n),
    .s_axis_tvalid      (conv_candidate_push),
    .s_axis_tready      (conv_q2_skid_s_ready),
    .source_group_low   (conv_candidate_idx_r[4]),
    .source_group_high  (conv_candidate_idx_r[5]),
    .group0_data        (conv_candidate_data[512+:256]),
    .group1_data        (conv_candidate_data[1536+:256]),
    .group2_data        (conv_candidate_data[2560+:256]),
    .group3_data        (conv_candidate_data[3584+:256]),
    .m_axis_tvalid      (conv_q2_skid_m_valid),
    .m_axis_tready      (conv_ingress_pop),
    .m_axis_tdata       (conv_q2_skid_m_data)
);

(* keep_hierarchy = "yes" *) quan_conv_ingress_quarter u_conv_ingress_q3(
    .clk                (clk),
    .rst_n              (rst_n),
    .s_axis_tvalid      (conv_candidate_push),
    .s_axis_tready      (conv_q3_skid_s_ready),
    .source_group_low   (conv_candidate_idx_r[4]),
    .source_group_high  (conv_candidate_idx_r[5]),
    .group0_data        (conv_candidate_data[768+:256]),
    .group1_data        (conv_candidate_data[1792+:256]),
    .group2_data        (conv_candidate_data[2816+:256]),
    .group3_data        (conv_candidate_data[3840+:256]),
    .m_axis_tvalid      (conv_q3_skid_m_valid),
    .m_axis_tready      (conv_ingress_pop),
    .m_axis_tdata       (conv_q3_skid_m_data)
);

assign conv_ctrl_skid_s_data = {(conv_candidate_idx_r == (PAR_NUM - 1)),
                                conv_out_type_r,
                                conv_candidate_source_param,
                                conv_candidate_idx_r};

axis_skid_buffer #(
    .DATA_WIDTH (CONV_CTRL_SKID_WIDTH)
) u_conv_ingress_ctrl_skid (
    .clk           (clk),
    .rst_n         (rst_n),
    .s_axis_tvalid (conv_candidate_push),
    .s_axis_tready (conv_ctrl_skid_s_ready),
    .s_axis_tdata  (conv_ctrl_skid_s_data),
    .m_axis_tvalid (conv_ctrl_skid_m_valid),
    .m_axis_tready (conv_ingress_pop),
    .m_axis_tdata  (conv_ctrl_skid_m_data)
);

assign conv_ingress_idx = conv_ctrl_skid_m_data[0+:PAR_IDX_WIDTH];
assign conv_candidate_param = conv_ctrl_skid_m_data[PAR_IDX_WIDTH+:PARAM_WIDTH];
assign conv_ingress_out_type = conv_ctrl_skid_m_data[70];
assign conv_ingress_last = conv_ctrl_skid_m_data[71];
assign conv_candidate_selected[0+:256] = conv_q0_skid_m_data;
assign conv_candidate_selected[256+:256] = conv_q1_skid_m_data;
assign conv_candidate_selected[512+:256] = conv_q2_skid_m_data;
assign conv_candidate_selected[768+:256] = conv_q3_skid_m_data;

assign conv_param_group0_selected = (conv_param_select_low == 4'd0)  ? param_mem[0] :
                               (conv_param_select_low == 4'd1) ? param_mem[1] :
                               (conv_param_select_low == 4'd2) ? param_mem[2] :
                               (conv_param_select_low == 4'd3) ? param_mem[3] :
                               (conv_param_select_low == 4'd4) ? param_mem[4] :
                               (conv_param_select_low == 4'd5) ? param_mem[5] :
                               (conv_param_select_low == 4'd6) ? param_mem[6] :
                               (conv_param_select_low == 4'd7) ? param_mem[7] :
                               (conv_param_select_low == 4'd8) ? param_mem[8] :
                               (conv_param_select_low == 4'd9) ? param_mem[9] :
                               (conv_param_select_low == 4'd10) ? param_mem[10] :
                               (conv_param_select_low == 4'd11) ? param_mem[11] :
                               (conv_param_select_low == 4'd12) ? param_mem[12] :
                               (conv_param_select_low == 4'd13) ? param_mem[13] :
                               (conv_param_select_low == 4'd14) ? param_mem[14] :
                                                                            param_mem[15];

assign conv_param_group1_selected = (conv_param_select_low == 4'd0)  ? param_mem[16] :
                               (conv_param_select_low == 4'd1) ? param_mem[17] :
                               (conv_param_select_low == 4'd2) ? param_mem[18] :
                               (conv_param_select_low == 4'd3) ? param_mem[19] :
                               (conv_param_select_low == 4'd4) ? param_mem[20] :
                               (conv_param_select_low == 4'd5) ? param_mem[21] :
                               (conv_param_select_low == 4'd6) ? param_mem[22] :
                               (conv_param_select_low == 4'd7) ? param_mem[23] :
                               (conv_param_select_low == 4'd8) ? param_mem[24] :
                               (conv_param_select_low == 4'd9) ? param_mem[25] :
                               (conv_param_select_low == 4'd10) ? param_mem[26] :
                               (conv_param_select_low == 4'd11) ? param_mem[27] :
                               (conv_param_select_low == 4'd12) ? param_mem[28] :
                               (conv_param_select_low == 4'd13) ? param_mem[29] :
                               (conv_param_select_low == 4'd14) ? param_mem[30] :
                                                                            param_mem[31];

assign conv_param_group2_selected = (conv_param_select_low == 4'd0)  ? param_mem[32] :
                               (conv_param_select_low == 4'd1) ? param_mem[33] :
                               (conv_param_select_low == 4'd2) ? param_mem[34] :
                               (conv_param_select_low == 4'd3) ? param_mem[35] :
                               (conv_param_select_low == 4'd4) ? param_mem[36] :
                               (conv_param_select_low == 4'd5) ? param_mem[37] :
                               (conv_param_select_low == 4'd6) ? param_mem[38] :
                               (conv_param_select_low == 4'd7) ? param_mem[39] :
                               (conv_param_select_low == 4'd8) ? param_mem[40] :
                               (conv_param_select_low == 4'd9) ? param_mem[41] :
                               (conv_param_select_low == 4'd10) ? param_mem[42] :
                               (conv_param_select_low == 4'd11) ? param_mem[43] :
                               (conv_param_select_low == 4'd12) ? param_mem[44] :
                               (conv_param_select_low == 4'd13) ? param_mem[45] :
                               (conv_param_select_low == 4'd14) ? param_mem[46] :
                                                                            param_mem[47];

assign conv_param_group3_selected = (conv_param_select_low == 4'd0)  ? param_mem[48] :
                               (conv_param_select_low == 4'd1) ? param_mem[49] :
                               (conv_param_select_low == 4'd2) ? param_mem[50] :
                               (conv_param_select_low == 4'd3) ? param_mem[51] :
                               (conv_param_select_low == 4'd4) ? param_mem[52] :
                               (conv_param_select_low == 4'd5) ? param_mem[53] :
                               (conv_param_select_low == 4'd6) ? param_mem[54] :
                               (conv_param_select_low == 4'd7) ? param_mem[55] :
                               (conv_param_select_low == 4'd8) ? param_mem[56] :
                               (conv_param_select_low == 4'd9) ? param_mem[57] :
                               (conv_param_select_low == 4'd10) ? param_mem[58] :
                               (conv_param_select_low == 4'd11) ? param_mem[59] :
                               (conv_param_select_low == 4'd12) ? param_mem[60] :
                               (conv_param_select_low == 4'd13) ? param_mem[61] :
                               (conv_param_select_low == 4'd14) ? param_mem[62] :
                                                                            param_mem[63];

assign conv_candidate_source_param = (conv_candidate_idx_r[5:4] == 2'd0) ? conv_param_candidate_g0_r :
                                     (conv_candidate_idx_r[5:4] == 2'd1) ? conv_param_candidate_g1_r :
                                     (conv_candidate_idx_r[5:4] == 2'd2) ? conv_param_candidate_g2_r :
                                                                                 conv_param_candidate_g3_r;

assign resadd_fm1_param = param_mem[0];
assign resadd_fm2_param = param_mem[1];
// concat 参数索引改为 fire 侧跟踪（20260725）：快速放行时结果侧 idx 滞后，
// fire 时按各流 pair 数镜像推进，与结果侧在慢速时逐拍等价。
assign concat_param     = param_mem[concat_param_idx_f];
assign split_param      = param_mem[0];
assign concat_current_pair_num_f = concat_pair_num[concat_param_idx_f[1:0]];
assign concat_last_param_idx = {{(PAR_IDX_WIDTH-3){1'b0}}, concat_input_num_r} - 1'b1;
assign fm1_next_param   = (model_r == MODEL_RESADD) ? resadd_fm1_param :
                          (model_r == MODEL_CONCAT) ? concat_param :
                                                       split_param;
assign fm2_next_param   = resadd_fm2_param;

// =============================================================================
// Issue stage combinational payload (captured into issue registers)
// =============================================================================
wire                                       issue_valid_d;
wire signed [SCALE_WIDTH-1:0]              issue_scale_d;
wire [SHIFT_WIDTH-1:0]                     issue_shift_d;
wire [1:0]                                 issue_src_d;
wire                                       issue_high_half_d;
wire                                       issue_conv_d;
wire                                       issue_int16_d;
wire signed [ACC_WIDTH-1:0]                issue_acc_d [0:QUAN_NUM-1];

assign issue_valid_d     = conv_issue | fm_busy;
assign issue_scale_d     = conv_issue ? conv_candidate_param[32+:SCALE_WIDTH] : fm_param_r[32+:SCALE_WIDTH];
assign issue_shift_d     = conv_issue ? conv_candidate_param[0+:SHIFT_WIDTH]  : fm_param_r[0+:SHIFT_WIDTH];
assign issue_src_d       = conv_issue ? SRC_NONE : fm_src_r;
assign issue_high_half_d = conv_issue ? 1'b0 : fm_second_issue;
assign issue_conv_d      = conv_issue;
assign issue_int16_d     = conv_issue ? conv_ingress_out_type : 1'b0;

genvar gi;
genvar pb;
genvar pl;
genvar ia;
generate
    for (gi = 0; gi < 8; gi = gi + 1) begin: g_issue_acc_q0
        assign issue_acc_d[gi] = conv_issue ?
            conv_candidate_selected[(gi*ACC_WIDTH)+:ACC_WIDTH] :
            fm_second_issue ?
                {{24{fm_data_r[(256+(gi*QOUT_WIDTH)+QOUT_WIDTH-1)]}},
                 fm_data_r[(256+(gi*QOUT_WIDTH))+:QOUT_WIDTH]} :
                {{24{fm_data_r[(gi*QOUT_WIDTH)+QOUT_WIDTH-1]}},
                 fm_data_r[(gi*QOUT_WIDTH)+:QOUT_WIDTH]};
    end
    for (gi = 8; gi < 16; gi = gi + 1) begin: g_issue_acc_q1
        assign issue_acc_d[gi] = conv_issue ?
            conv_candidate_selected[(gi*ACC_WIDTH)+:ACC_WIDTH] :
            fm_second_issue ?
                {{24{fm_data_r[(256+(gi*QOUT_WIDTH)+QOUT_WIDTH-1)]}},
                 fm_data_r[(256+(gi*QOUT_WIDTH))+:QOUT_WIDTH]} :
                {{24{fm_data_r[(gi*QOUT_WIDTH)+QOUT_WIDTH-1]}},
                 fm_data_r[(gi*QOUT_WIDTH)+:QOUT_WIDTH]};
    end
    for (gi = 16; gi < 24; gi = gi + 1) begin: g_issue_acc_q2
        assign issue_acc_d[gi] = conv_issue ?
            conv_candidate_selected[(gi*ACC_WIDTH)+:ACC_WIDTH] :
            fm_second_issue ?
                {{24{fm_data_r[(256+(gi*QOUT_WIDTH)+QOUT_WIDTH-1)]}},
                 fm_data_r[(256+(gi*QOUT_WIDTH))+:QOUT_WIDTH]} :
                {{24{fm_data_r[(gi*QOUT_WIDTH)+QOUT_WIDTH-1]}},
                 fm_data_r[(gi*QOUT_WIDTH)+:QOUT_WIDTH]};
    end
    for (gi = 24; gi < QUAN_NUM; gi = gi + 1) begin: g_issue_acc_q3
        assign issue_acc_d[gi] = conv_issue ?
            conv_candidate_selected[(gi*ACC_WIDTH)+:ACC_WIDTH] :
            fm_second_issue ?
                {{24{fm_data_r[(256+(gi*QOUT_WIDTH)+QOUT_WIDTH-1)]}},
                 fm_data_r[(256+(gi*QOUT_WIDTH))+:QOUT_WIDTH]} :
                {{24{fm_data_r[(gi*QOUT_WIDTH)+QOUT_WIDTH-1]}},
                 fm_data_r[(gi*QOUT_WIDTH)+:QOUT_WIDTH]};
    end
endgenerate

// Token valid: sideband pipeline output (lane valids are identical by construction)
assign quan_out_valid = tk_valid_s7;

// =============================================================================
// quan_unit array and output packing
// =============================================================================
generate
    for(gi = 0; gi < QUAN_NUM; gi = gi + 1) begin: quan_unit_loop
        quan_unit quan_unit_n(
            .clk        (clk),
            .rst_n      (rst_n),
            .pipe_ce    (pipe_ce_lane[gi]),
            .data_valid (issue_valid_r),
            .acc_in     (issue_acc_r[gi]),
            .scale_m0   (issue_scale_r),
            .shift_n    (issue_shift_r),
            .quan_valid (quan_unit_valid_unused[gi]),
            .quan_i8    (quan_unit_out_i8[gi]),
            .quan_i16   (quan_unit_out_i16[gi])
        );
    end
endgenerate

generate
    for(gi = 0; gi < QUAN_NUM; gi = gi + 1) begin: quan_out_pack_loop
        assign quan_out_pack[(gi*QOUT_WIDTH)+:QOUT_WIDTH] = quan_unit_out_i8[gi];
        assign quan_out_pack16[(gi*QOUT16_WIDTH)+:QOUT16_WIDTH] = quan_unit_out_i16[gi];
    end
endgenerate

assign quan_out_pack[`QUAN_ACTOUTDATA_WIDTH-1:256] = {(`QUAN_ACTOUTDATA_WIDTH-256){1'b0}};

// =============================================================================
// Output routing into registered 2-entry skids
// =============================================================================
assign quan_res_fm1_pipe_valid = quan_out_valid & (unit_out_src == SRC_FM1) &
                                 (((model_r != MODEL_SPLIT) & (model_r != MODEL_CONCAT)) |
                                  unit_out_high_half);
assign quan_res_fm2_pipe_valid = quan_out_valid & (unit_out_src == SRC_FM2);
assign quan_res_fm1_skid_s_valid = upsamp_pending_r | quan_res_fm1_pipe_valid;
assign quan_res_fm2_skid_s_valid = quan_res_fm2_pipe_valid;

// SPLIT/CONCAT keeps the first 256-bit half locally and emits one 512-bit beat
// on the second half.  RESADD emits the packed INT16 result on its high half.
assign quan_res_fm1_skid_s_data = upsamp_pending_r ? upsamp_data_r :
                                  ((model_r == MODEL_SPLIT) | (model_r == MODEL_CONCAT)) ?
                                  {quan_out_pack[255:0], split_low_q} : quan_out_pack16;
assign quan_res_fm2_skid_s_data = quan_out_pack16;
assign quan_res_fm1_pipe_fire = quan_res_fm1_pipe_valid & quan_res_fm1_skid_s_ready;
assign quan_res_fm2_pipe_fire = quan_res_fm2_pipe_valid & quan_res_fm2_skid_s_ready;
assign upsamp_drain = upsamp_pending_r & quan_res_fm1_skid_s_ready;

axis_skid_buffer #(
    .DATA_WIDTH (`QUAN_RESOUTDATA_WIDTH)
) u_quan_res_fm1_skid (
    .clk           (clk),
    .rst_n         (rst_n),
    .s_axis_tvalid (quan_res_fm1_skid_s_valid),
    .s_axis_tready (quan_res_fm1_skid_s_ready),
    .s_axis_tdata  (quan_res_fm1_skid_s_data),
    .m_axis_tvalid (m_axis_res_fm1_tvalid),
    .m_axis_tready (m_axis_res_fm1_tready),
    .m_axis_tdata  (m_axis_res_fm1_tdata)
);

axis_skid_buffer #(
    .DATA_WIDTH (`QUAN_RESOUTDATA_WIDTH)
) u_quan_res_fm2_skid (
    .clk           (clk),
    .rst_n         (rst_n),
    .s_axis_tvalid (quan_res_fm2_skid_s_valid),
    .s_axis_tready (quan_res_fm2_skid_s_ready),
    .s_axis_tdata  (quan_res_fm2_skid_s_data),
    .m_axis_tvalid (m_axis_res_fm2_tvalid),
    .m_axis_tready (m_axis_res_fm2_tready),
    .m_axis_tdata  (m_axis_res_fm2_tdata)
);

// CONV output terminates in a two-entry registered skid.  Its source ready
// depends only on local occupancy; when full, the complete quan arithmetic and
// sideband pipeline holds before the current tail token can be overwritten.
assign quan_act_skid_s_valid = quan_out_valid & unit_out_conv;
assign quan_act_skid_s_data  = unit_out_int16 ? quan_out_pack16 : quan_out_pack;

axis_skid_buffer #(
    .DATA_WIDTH (`QUAN_ACTOUTDATA_WIDTH)
) u_quan_act_skid (
    .clk           (clk),
    .rst_n         (rst_n),
    .s_axis_tvalid (quan_act_skid_s_valid),
    .s_axis_tready (quan_act_skid_s_ready),
    .s_axis_tdata  (quan_act_skid_s_data),
    .m_axis_tvalid (m_axis_quan_act_tvalid),
    .m_axis_tready (m_axis_quan_act_tready),
    .m_axis_tdata  (m_axis_quan_act_tdata)
);

// =============================================================================
// Sequential logic (one register per always block)
// =============================================================================
// ---- command / model latch ----
always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        model_r <= MODEL_IDLE;
    else if(rdcmd_trans_valid)
        model_r <= cmd_model_next;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        fm_pair_num_r <= {FM_CNT_WIDTH{1'b0}};
    else if(rdcmd_trans_valid & cmd_supported)
        fm_pair_num_r <= cmd_fm_pair_num;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        concat_pair_num[0] <= {FM_CNT_WIDTH{1'b0}};
    else if(rdcmd_trans_valid & cmd_supported)
        concat_pair_num[0] <= cmd_fm_pair_num;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        concat_pair_num[1] <= {FM_CNT_WIDTH{1'b0}};
    else if(rdcmd_trans_valid & cmd_supported)
        concat_pair_num[1] <= cmd_fm2_pair_num;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        concat_pair_num[2] <= {FM_CNT_WIDTH{1'b0}};
    else if(rdcmd_trans_valid & cmd_supported)
        concat_pair_num[2] <= cmd_fm3_pair_num;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        concat_pair_num[3] <= {FM_CNT_WIDTH{1'b0}};
    else if(rdcmd_trans_valid & cmd_supported)
        concat_pair_num[3] <= cmd_fm4_pair_num;
end

// ---- parameter load ----
always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        param_loading <= 1'b0;
    else if(rdcmd_trans_valid & param_model)
        param_loading <= 1'b1;
    else if(s_axis_quan_param_tvalid & s_axis_quan_param_tready &
            (param_beat_cnt == (PAR_BEAT_NUM - 1)))
        param_loading <= 1'b0;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        param_beat_cnt <= {PAR_BEAT_WIDTH{1'b0}};
    else if(rdcmd_trans_valid & param_model)
        param_beat_cnt <= {PAR_BEAT_WIDTH{1'b0}};
    else if(s_axis_quan_param_tvalid & s_axis_quan_param_tready)
        param_beat_cnt <= param_beat_cnt + 1'b1;
end

generate
    for (pb = 0; pb < PAR_BEAT_NUM; pb = pb + 1) begin: g_param_beat
        for (pl = 0; pl < 8; pl = pl + 1) begin: g_param_lane
            always @(posedge clk) begin
                if(s_axis_quan_param_tvalid & s_axis_quan_param_tready &
                        (param_beat_cnt == pb))
                    param_mem[(pb*8)+pl] <= s_axis_quan_param_tdata[(pl*PARAM_WIDTH)+:PARAM_WIDTH];
            end
        end
    end
endgenerate

// ---- CONV block control, source candidate phase and buffered issue phase ----
// PE captures its only 64K snapshot during conv_valid. One cycle later this
// pending pulse starts channel 0. PE data and parameter candidates advance on
// each aggregate ingress push. Actual issue uses the five lockstep skid heads;
// source completion and last-channel issue completion are separate events.
always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        conv_capture_pending_r <= 1'b0;
    else if(rdcmd_trans_valid & ((cmd_model_sel == `MODEL_SEL_CONV) | ~cmd_supported))
        conv_capture_pending_r <= 1'b0;
    else
        conv_capture_pending_r <= conv_valid;
end

always @(posedge clk) begin
    if(conv_param_load_en)
        conv_param_candidate_g0_r <= conv_param_group0_selected;
end

always @(posedge clk) begin
    if(conv_param_load_en)
        conv_param_candidate_g1_r <= conv_param_group1_selected;
end

always @(posedge clk) begin
    if(conv_param_load_en)
        conv_param_candidate_g2_r <= conv_param_group2_selected;
end

always @(posedge clk) begin
    if(conv_param_load_en)
        conv_param_candidate_g3_r <= conv_param_group3_selected;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        conv_busy <= 1'b0;
    else if(rdcmd_trans_valid & ((cmd_model_sel == `MODEL_SEL_CONV) | ~cmd_supported))
        conv_busy <= 1'b0;
    else if(conv_valid)
        conv_busy <= 1'b1;
    else if(conv_ingress_issue_done)
        conv_busy <= 1'b0;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        conv_candidate_idx_r <= {PAR_IDX_WIDTH{1'b0}};
    else if(rdcmd_trans_valid & ((cmd_model_sel == `MODEL_SEL_CONV) | ~cmd_supported))
        conv_candidate_idx_r <= {PAR_IDX_WIDTH{1'b0}};
    else if(conv_candidate_start)
        conv_candidate_idx_r <= {PAR_IDX_WIDTH{1'b0}};
    else if(conv_candidate_advance)
        conv_candidate_idx_r <= conv_candidate_idx_r + 1'b1;
    else if(conv_candidate_done)
        conv_candidate_idx_r <= {PAR_IDX_WIDTH{1'b0}};
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        conv_param_load_idx_r <= 4'd0;
    else if(rdcmd_trans_valid & ((cmd_model_sel == `MODEL_SEL_CONV) | ~cmd_supported))
        conv_param_load_idx_r <= 4'd0;
    else if(conv_candidate_start)
        conv_param_load_idx_r <= 4'd1;
    else if(conv_candidate_advance)
        conv_param_load_idx_r <= conv_param_load_idx_r + 1'b1;
    else if(conv_candidate_done)
        conv_param_load_idx_r <= 4'd0;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        conv_issue_idx_r <= {PAR_IDX_WIDTH{1'b0}};
    else if(rdcmd_trans_valid & ((cmd_model_sel == `MODEL_SEL_CONV) | ~cmd_supported))
        conv_issue_idx_r <= {PAR_IDX_WIDTH{1'b0}};
    else if(conv_issue)
        conv_issue_idx_r <= conv_ingress_idx;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        conv_out_type_r <= 1'b0;
    else if(rdcmd_trans_valid & (cmd_model_sel == `MODEL_SEL_CONV))
        conv_out_type_r <= cmd_conv_out_type;
end

// ---- concat control (fire 侧参数索引跟踪) ----
always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        concat_input_num_r <= 3'd2;
    else if(rdcmd_trans_valid & (cmd_model_sel == `MODEL_SEL_CONCAT))
        concat_input_num_r <= {1'b0, cmd_concat_type} + 3'd2;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        concat_param_idx_f <= {PAR_IDX_WIDTH{1'b0}};
    else if(rdcmd_trans_valid & ~cmd_supported)
        concat_param_idx_f <= {PAR_IDX_WIDTH{1'b0}};
    else if(rdcmd_trans_valid & (cmd_model_sel == `MODEL_SEL_CONCAT))
        concat_param_idx_f <= {PAR_IDX_WIDTH{1'b0}};
    else if((model_r == MODEL_CONCAT) & fm1_fire &
            (concat_pair_cnt_f == (concat_current_pair_num_f - 1'b1)))
        concat_param_idx_f <= (concat_param_idx_f == concat_last_param_idx) ?
                            {PAR_IDX_WIDTH{1'b0}} : concat_param_idx_f + 1'b1;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        concat_pair_cnt_f <= {FM_CNT_WIDTH{1'b0}};
    else if(rdcmd_trans_valid & ~cmd_supported)
        concat_pair_cnt_f <= {FM_CNT_WIDTH{1'b0}};
    else if(rdcmd_trans_valid & (cmd_model_sel == `MODEL_SEL_CONCAT))
        concat_pair_cnt_f <= {FM_CNT_WIDTH{1'b0}};
    else if((model_r == MODEL_CONCAT) & fm1_fire)
        concat_pair_cnt_f <= (concat_pair_cnt_f == (concat_current_pair_num_f - 1'b1)) ?
                           {FM_CNT_WIDTH{1'b0}} : concat_pair_cnt_f + 1'b1;
end

// ---- resadd port/group accounting ----
always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        resadd_port_sel <= 1'b0;
    else if(rdcmd_trans_valid & ~cmd_supported)
        resadd_port_sel <= 1'b0;
    else if(rdcmd_trans_valid & (cmd_model_sel == `MODEL_SEL_RESADD))
        resadd_port_sel <= 1'b0;
    else if((model_r == MODEL_RESADD) &
            ((quan_res_fm1_pipe_fire & unit_out_high_half) |
             (quan_res_fm2_pipe_fire & unit_out_high_half)) &
            (fm_channel_pair_cnt == ({26'd0, res_grp_pair_limit} - 32'd1)))
        resadd_port_sel <= ~resadd_port_sel;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        fm_channel_pair_cnt <= {PAR_IDX_WIDTH{1'b0}};
    else if(rdcmd_trans_valid & ~cmd_supported)
        fm_channel_pair_cnt <= {PAR_IDX_WIDTH{1'b0}};
    else if(rdcmd_trans_valid & (cmd_model_sel == `MODEL_SEL_RESADD))
        fm_channel_pair_cnt <= {PAR_IDX_WIDTH{1'b0}};
    else if((model_r == MODEL_RESADD) &
            ((quan_res_fm1_pipe_fire & unit_out_high_half) |
             (quan_res_fm2_pipe_fire & unit_out_high_half)))
        fm_channel_pair_cnt <= (fm_channel_pair_cnt == ({26'd0, res_grp_pair_limit} - 32'd1)) ?
                               {PAR_IDX_WIDTH{1'b0}} : fm_channel_pair_cnt + 1'b1;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        res_p0_pair_cnt <= {FM_CNT_WIDTH{1'b0}};
    else if(rdcmd_trans_valid & ~cmd_supported)
        res_p0_pair_cnt <= {FM_CNT_WIDTH{1'b0}};
    else if(rdcmd_trans_valid & (cmd_model_sel == `MODEL_SEL_RESADD))
        res_p0_pair_cnt <= {FM_CNT_WIDTH{1'b0}};
    else if((model_r == MODEL_RESADD) & quan_res_fm1_pipe_fire & unit_out_high_half &
            (fm_channel_pair_cnt == ({26'd0, res_grp_pair_limit} - 32'd1))) begin
        // 本 FM 本 tile 满 fm_pair_num_r 对 → 计数归零 (下一 tile 重开);
        // 否则累加本组对数, 等下一次切回本端口继续。
        if (res_p0_pair_cnt + {26'd0, res_grp_pair_limit} >= {20'd0, fm_pair_num_r})
            res_p0_pair_cnt <= {FM_CNT_WIDTH{1'b0}};
        else
            res_p0_pair_cnt <= res_p0_pair_cnt + {26'd0, res_grp_pair_limit};
    end
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        res_p1_pair_cnt <= {FM_CNT_WIDTH{1'b0}};
    else if(rdcmd_trans_valid & ~cmd_supported)
        res_p1_pair_cnt <= {FM_CNT_WIDTH{1'b0}};
    else if(rdcmd_trans_valid & (cmd_model_sel == `MODEL_SEL_RESADD))
        res_p1_pair_cnt <= {FM_CNT_WIDTH{1'b0}};
    else if((model_r == MODEL_RESADD) & quan_res_fm2_pipe_fire & unit_out_high_half &
            (fm_channel_pair_cnt == ({26'd0, res_grp_pair_limit} - 32'd1))) begin
        if (res_p1_pair_cnt + {26'd0, res_grp_pair_limit} >= {20'd0, fm_pair_num_r})
            res_p1_pair_cnt <= {FM_CNT_WIDTH{1'b0}};
        else
            res_p1_pair_cnt <= res_p1_pair_cnt + {26'd0, res_grp_pair_limit};
    end
end

// ---- FM two-half issue state; advances atomically with pipe_ce_ctrl ----
always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        fm_busy <= 1'b0;
    else if(rdcmd_trans_valid & ~cmd_supported)
        fm_busy <= 1'b0;
    else if(pipe_ce_ctrl) begin
        if(fm1_fire & (model_r != MODEL_UPSAMPLE))
            fm_busy <= 1'b1;
        else if(fm2_fire)
            fm_busy <= 1'b1;
        else if(fm_busy & fm_second_issue)
            fm_busy <= 1'b0;
    end
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        fm_src_r <= SRC_NONE;
    else if(rdcmd_trans_valid & ~cmd_supported)
        fm_src_r <= SRC_NONE;
    else if(pipe_ce_ctrl) begin
        if(fm1_fire & (model_r != MODEL_UPSAMPLE))
            fm_src_r <= SRC_FM1;
        else if(fm2_fire)
            fm_src_r <= SRC_FM2;
        else if(fm_busy & fm_second_issue)
            fm_src_r <= SRC_NONE;
    end
end

always @(posedge clk) begin
    if(rdcmd_trans_valid & ~cmd_supported)
        fm_data_r <= {`QUAN_FMINDATA_WIDTH{1'b0}};
    else if(fm1_fire & (model_r != MODEL_UPSAMPLE))
        fm_data_r <= s_axis_quan_fm1_tdata;
    else if(fm2_fire)
        fm_data_r <= s_axis_quan_fm2_tdata;
end

always @(posedge clk) begin
    if(rdcmd_trans_valid & ~cmd_supported)
        fm_param_r <= {PARAM_WIDTH{1'b0}};
    else if(fm1_fire & (model_r != MODEL_UPSAMPLE))
        fm_param_r <= fm1_next_param;
    else if(fm2_fire)
        fm_param_r <= fm2_next_param;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        fm_second_issue <= 1'b0;
    else if(rdcmd_trans_valid & ~cmd_supported)
        fm_second_issue <= 1'b0;
    else if(pipe_ce_ctrl) begin
        if(fm1_fire & (model_r != MODEL_UPSAMPLE))
            fm_second_issue <= 1'b0;
        else if(fm2_fire)
            fm_second_issue <= 1'b0;
        else if(fm_busy & ~fm_second_issue)
            fm_second_issue <= 1'b1;
        else if(fm_busy & fm_second_issue)
            fm_second_issue <= 1'b0;
    end
end

// ---- UPSAMPLE registered elastic stage ----
always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        upsamp_pending_r <= 1'b0;
    else if(rdcmd_trans_valid & ~cmd_supported)
        upsamp_pending_r <= 1'b0;
    else begin
        case ({(fm1_fire & (model_r == MODEL_UPSAMPLE)), upsamp_drain})
            2'b10:   upsamp_pending_r <= 1'b1;
            2'b01:   upsamp_pending_r <= 1'b0;
            2'b11:   upsamp_pending_r <= 1'b1;
            default: upsamp_pending_r <= upsamp_pending_r;
        endcase
    end
end

always @(posedge clk) begin
    if(fm1_fire & (model_r == MODEL_UPSAMPLE))
        upsamp_data_r <= s_axis_quan_fm1_tdata;
end

// ---- split/concat low-half latch ----
always @(posedge clk) begin
    if(rdcmd_trans_valid & ~cmd_supported)
        split_low_q <= 256'b0;
    else if(pipe_ce_ctrl & quan_out_valid & (unit_out_src == SRC_FM1) &
            ((model_r == MODEL_SPLIT) | (model_r == MODEL_CONCAT)) & ~unit_out_high_half)
        split_low_q <= quan_out_pack[255:0];
end

// ---- issue/pipeline registers advance from the selected local skid occupancy ----
// ---- issue stage registers ----
always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        issue_valid_r <= 1'b0;
    else if(pipe_ce_ctrl)
        issue_valid_r <= issue_valid_d;
end

generate
    for (ia = 0; ia < QUAN_NUM; ia = ia + 1) begin: g_issue_acc_reg
        always @(posedge clk) begin
            if(pipe_ce_lane[ia])
                issue_acc_r[ia] <= issue_acc_d[ia];
        end
    end
endgenerate

always @(posedge clk) begin
    if(pipe_ce_ctrl)
        issue_scale_r <= issue_scale_d;
end

always @(posedge clk) begin
    if(pipe_ce_ctrl)
        issue_shift_r <= issue_shift_d;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        issue_src_r <= SRC_NONE;
    else if(pipe_ce_ctrl)
        issue_src_r <= issue_src_d;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        issue_high_half_r <= 1'b0;
    else if(pipe_ce_ctrl)
        issue_high_half_r <= issue_high_half_d;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        issue_conv_r <= 1'b0;
    else if(pipe_ce_ctrl)
        issue_conv_r <= issue_conv_d;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        issue_int16_r <= 1'b0;
    else if(pipe_ce_ctrl)
        issue_int16_r <= issue_int16_d;
end

// ---- sideband pipeline (matches issue reg + quan_unit U1..U4 latency) ----
always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        tk_valid_s1 <= 1'b0;
    else if(pipe_ce_ctrl)
        tk_valid_s1 <= issue_valid_r;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        tk_valid_s2 <= 1'b0;
    else if(pipe_ce_ctrl)
        tk_valid_s2 <= tk_valid_s1;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        tk_valid_s3 <= 1'b0;
    else if(pipe_ce_ctrl)
        tk_valid_s3 <= tk_valid_s2;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        tk_valid_s4 <= 1'b0;
    else if(pipe_ce_ctrl)
        tk_valid_s4 <= tk_valid_s3;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        tk_valid_s5 <= 1'b0;
    else if(pipe_ce_ctrl)
        tk_valid_s5 <= tk_valid_s4;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        tk_valid_s6 <= 1'b0;
    else if(pipe_ce_ctrl)
        tk_valid_s6 <= tk_valid_s5;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        tk_valid_s7 <= 1'b0;
    else if(pipe_ce_ctrl)
        tk_valid_s7 <= tk_valid_s6;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        src_s1 <= SRC_NONE;
    else if(pipe_ce_ctrl)
        src_s1 <= issue_src_r;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        src_s2 <= SRC_NONE;
    else if(pipe_ce_ctrl)
        src_s2 <= src_s1;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        src_s3 <= SRC_NONE;
    else if(pipe_ce_ctrl)
        src_s3 <= src_s2;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        src_s4 <= SRC_NONE;
    else if(pipe_ce_ctrl)
        src_s4 <= src_s3;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        src_s5 <= SRC_NONE;
    else if(pipe_ce_ctrl)
        src_s5 <= src_s4;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        src_s6 <= SRC_NONE;
    else if(pipe_ce_ctrl)
        src_s6 <= src_s5;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        unit_out_src <= SRC_NONE;
    else if(pipe_ce_ctrl)
        unit_out_src <= src_s6;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        high_s1 <= 1'b0;
    else if(pipe_ce_ctrl)
        high_s1 <= issue_high_half_r;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        high_s2 <= 1'b0;
    else if(pipe_ce_ctrl)
        high_s2 <= high_s1;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        high_s3 <= 1'b0;
    else if(pipe_ce_ctrl)
        high_s3 <= high_s2;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        high_s4 <= 1'b0;
    else if(pipe_ce_ctrl)
        high_s4 <= high_s3;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        high_s5 <= 1'b0;
    else if(pipe_ce_ctrl)
        high_s5 <= high_s4;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        high_s6 <= 1'b0;
    else if(pipe_ce_ctrl)
        high_s6 <= high_s5;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        unit_out_high_half <= 1'b0;
    else if(pipe_ce_ctrl)
        unit_out_high_half <= high_s6;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        conv_s1 <= 1'b0;
    else if(pipe_ce_ctrl)
        conv_s1 <= issue_conv_r;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        conv_s2 <= 1'b0;
    else if(pipe_ce_ctrl)
        conv_s2 <= conv_s1;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        conv_s3 <= 1'b0;
    else if(pipe_ce_ctrl)
        conv_s3 <= conv_s2;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        conv_s4 <= 1'b0;
    else if(pipe_ce_ctrl)
        conv_s4 <= conv_s3;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        conv_s5 <= 1'b0;
    else if(pipe_ce_ctrl)
        conv_s5 <= conv_s4;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        conv_s6 <= 1'b0;
    else if(pipe_ce_ctrl)
        conv_s6 <= conv_s5;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        unit_out_conv <= 1'b0;
    else if(pipe_ce_ctrl)
        unit_out_conv <= conv_s6;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        int16_s1 <= 1'b0;
    else if(pipe_ce_ctrl)
        int16_s1 <= issue_int16_r;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        int16_s2 <= 1'b0;
    else if(pipe_ce_ctrl)
        int16_s2 <= int16_s1;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        int16_s3 <= 1'b0;
    else if(pipe_ce_ctrl)
        int16_s3 <= int16_s2;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        int16_s4 <= 1'b0;
    else if(pipe_ce_ctrl)
        int16_s4 <= int16_s3;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        int16_s5 <= 1'b0;
    else if(pipe_ce_ctrl)
        int16_s5 <= int16_s4;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        int16_s6 <= 1'b0;
    else if(pipe_ce_ctrl)
        int16_s6 <= int16_s5;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        unit_out_int16 <= 1'b0;
    else if(pipe_ce_ctrl)
        unit_out_int16 <= int16_s6;
end

endmodule

// ============================================================
// RTL Self-Check Results
// ============================================================
// [PASS] Rule #1:  All ports have explicit width declarations
// [PASS] Rule #2:  Ports use 'wire', sequential uses 'reg'
// [PASS] Rule #6:  No debug/error-attribution logic in RTL
// [PASS] Rule #8:  No 'logic'/'always_ff'/'always_comb' used
// [PASS] Rule #12: All combinational branches covered (no latch)
// [PASS] Rule #16: Blocking '=' for comb, non-blocking '<=' for seq
// [PASS] No Verilog function/task (ceil_pair_count removed, explicit wires)
// [PASS] One-register-per-always-block: every reg has its own always block
//        (param_mem/concat_pair_num/issue_acc_r treated as memory arrays)
// [PASS] Declaration-before-use: all regs declared before wires/always
// [PASS] Multiplier inputs and all data/valid/sideband stages hold on local pipe CE
// [PASS] CONV consumes four PE-local registered candidates; no 64K quan snapshot
// [PASS] Pairwise 2:1 + registered ingress + final 2:1 removes the old final 4:1
// [PASS] Four data skids and one control skid use common aggregate push/pop
// [PASS] Source data/parameter/index/valid hold or advance in lockstep on full
// [PASS] Three registered output skids cut downstream ready and sustain one beat/cycle
// [PASS] FM issue state, sideband, split half and RESADD accounting hold with pipeline stall
// [PASS] UPSAMPLE elastic register supports hold and same-cycle drain/replace
// ============================================================


// =============================================================================
// One CONV ingress quarter. The first pairwise 2:1 selection is captured in a
// two-entry registered skid; the buffered high group bit drives only the final
// local 2:1. Four instances operate in lockstep with a narrow control skid.
// =============================================================================
module quan_conv_ingress_quarter(
    input  wire         clk,
    input  wire         rst_n,
    input  wire         s_axis_tvalid,
    output wire         s_axis_tready,
    input  wire         source_group_low,
    input  wire         source_group_high,
    input  wire [255:0] group0_data,
    input  wire [255:0] group1_data,
    input  wire [255:0] group2_data,
    input  wire [255:0] group3_data,
    output wire         m_axis_tvalid,
    input  wire         m_axis_tready,
    output wire [255:0] m_axis_tdata
);

wire [255:0] pair01_data;
wire [255:0] pair23_data;
wire [512:0] skid_s_data;
wire [512:0] skid_m_data;
(* max_fanout = 512 *) wire source_group_low_local;

assign source_group_low_local = source_group_low;
assign pair01_data = source_group_low_local ? group1_data : group0_data;
assign pair23_data = source_group_low_local ? group3_data : group2_data;
assign skid_s_data = {source_group_high, pair23_data, pair01_data};
assign m_axis_tdata = skid_m_data[512] ? skid_m_data[256+:256] :
                                             skid_m_data[0+:256];

axis_skid_buffer #(
    .DATA_WIDTH (513)
) u_skid (
    .clk           (clk),
    .rst_n         (rst_n),
    .s_axis_tvalid (s_axis_tvalid),
    .s_axis_tready (s_axis_tready),
    .s_axis_tdata  (skid_s_data),
    .m_axis_tvalid (m_axis_tvalid),
    .m_axis_tready (m_axis_tready),
    .m_axis_tdata  (skid_m_data)
);

/*
 * RTL SELF-CHECK:
 * PASS: first-stage pair selection and final selection are both local 2:1 muxes
 * PASS: registered skid cuts candidate-to-issue routing and downstream ready
 * PASS: payload has no reset; only the reused skid occupancy control is reset
 * PASS: no process loop, variable array index, function, division or modulo
 * FAIL: none
 */
endmodule
