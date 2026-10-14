`timescale 1ns / 1ps
`include "all_defines.vh"
// ===============================================================
// PE array (v8: PE-local banked snapshot and quarter-local candidates)
//
// v6 keeps the two-stage local bias tree inside each cal_unit.
// v8 keeps the only 65536-bit convolution-result snapshot inside PE:
//   64 channels x 32 columns x INT32
//   -> four PE-local groups (channels 0..15/16..31/32..47/48..63)
//   -> group x quarter x bank physical ownership
//   -> quarter-local registered 16:1 candidate with local CE
//   -> 4 x 1024-bit candidate boundary to quan.
// The snapshot is not duplicated. Snapshot and candidate payload registers
// have no reset. Downstream backpressure reaches only quarter-local candidate
// CE pins; registered load indices are the sole selectors of the local16 muxes.
// ===============================================================
module pe(
input  wire                                     clk,
input  wire                                     rst_n,
input  wire                                     initial_valid,
input  wire                                     data_valid,
input  wire                                     end_data_valid,
input  wire [`PE_ARR_FMINDATA_WIDTH-1:0]        fm_data_256,
input  wire [`PE_ARR_WTINDATA_WIDTH-1:0]        weight_in,
input  wire [`PE_ARR_BIASDATA_WIDTH-1:0]        bias_in,
input  wire                                     conv_candidate_start,
input  wire                                     conv_candidate_advance,
input  wire                                     conv_candidate_done,
output wire                                     conv_valid,
output wire                                     conv_candidate_valid,
output wire [4095:0]                            conv_candidate_data
);

localparam CALNUM           = `PE_ARR_CALNUM;
localparam CORES            = `CAL_UNIT_DSPCORE_NUM;
localparam CORE_INDEX_SHIFT = 4;

(* dont_touch = "yes" *) reg [`PE_ARR_FMINDATA_WIDTH-1:0]    fm_mid [0:15];
(* dont_touch = "yes" *) reg [`PE_ARR_FMINDATA_WIDTH-1:0]    fm_rep2 [0:CALNUM-1];
(* dont_touch = "yes" *) reg [`CAL_UNIT_WTINDATA_WIDTH-1:0] w_rep [0:CALNUM-1];
(* dont_touch = "yes" *) reg [`CAL_UNIT_WTINDATA_WIDTH-1:0] w_s2 [0:CALNUM*CORES-1];
(* dont_touch = "yes" *) reg                               init_valid_rep [0:CALNUM-1];
(* dont_touch = "yes" *) reg                               data_valid_rep [0:CALNUM-1];
(* dont_touch = "yes" *) reg                               init_valid_rep2 [0:CALNUM-1];
(* dont_touch = "yes" *) reg                               data_valid_rep2 [0:CALNUM-1];

reg end_data_valid_r1;
reg end_data_valid_r2;
reg end_data_valid_r3;
reg end_data_valid_r4;
reg conv_valid_r;

wire [`PE_ARR_OUTDATA_WIDTH-1:0] cal_result_w;
wire [`CAL_UNIT_WTINDATA_WIDTH*CORES-1:0] w_bus [0:CALNUM-1];

assign conv_valid = conv_valid_r;

// ===============================================================
// FM first stage: 16 intermediate replicas.
// ===============================================================
genvar m;
generate
    for (m = 0; m < 16; m = m + 1) begin: PE_FM_MID
        always @(posedge clk) begin
            fm_mid[m] <= fm_data_256;
        end
    end
endgenerate

// ===============================================================
// Unit-level weight/valid replicas and FM second stage.
// ===============================================================
genvar i;
generate
    for (i = 0; i < CALNUM; i = i + 1) begin: PE_IN_REP
        always @(posedge clk) begin
            fm_rep2[i] <= fm_mid[i>>2];
        end
        always @(posedge clk) begin
            w_rep[i] <= weight_in[`CAL_UNIT_WTINDATA_WIDTH*i+:8];
        end
        always @(posedge clk or negedge rst_n) begin
            if(!rst_n)
                init_valid_rep[i] <= 1'b0;
            else
                init_valid_rep[i] <= initial_valid;
        end
        always @(posedge clk or negedge rst_n) begin
            if(!rst_n)
                data_valid_rep[i] <= 1'b0;
            else
                data_valid_rep[i] <= data_valid;
        end
        always @(posedge clk or negedge rst_n) begin
            if(!rst_n)
                init_valid_rep2[i] <= 1'b0;
            else
                init_valid_rep2[i] <= init_valid_rep[i];
        end
        always @(posedge clk or negedge rst_n) begin
            if(!rst_n)
                data_valid_rep2[i] <= 1'b0;
            else
                data_valid_rep2[i] <= data_valid_rep[i];
        end
    end
endgenerate

// ===============================================================
// Core-level weight second stage.
// ===============================================================
genvar g;
generate
    for (g = 0; g < CALNUM*CORES; g = g + 1) begin: PE_CORE_REP
        always @(posedge clk) begin
            w_s2[g] <= w_rep[g>>CORE_INDEX_SHIFT];
        end
    end
endgenerate

// ===============================================================
// Pack each cal_unit weight bus.
// ===============================================================
genvar uj;
genvar uc;
generate
    for (uj = 0; uj < CALNUM; uj = uj + 1) begin: PE_PACK
        for (uc = 0; uc < CORES; uc = uc + 1) begin: PE_PACK_CORE
            assign w_bus[uj][`CAL_UNIT_WTINDATA_WIDTH*uc+:`CAL_UNIT_WTINDATA_WIDTH] =
                   w_s2[uj*CORES+uc];
        end
    end
endgenerate

// ===============================================================
// 64 cal_unit instances. cal_result_w is internal to PE; it is captured
// once by the PE-local snapshot hierarchy and is no longer exported to quan.
// ===============================================================
genvar j;
generate
    for (j = 0; j < CALNUM; j = j + 1) begin: CAL_UNIT_LOOP
        cal_unit cal_unit_n(
            .clk           (clk),
            .rst_n         (rst_n),
            .initial_valid (init_valid_rep2[j]),
            .data_valid    (data_valid_rep2[j]),
            .bias_in       (bias_in[`CAL_UNIT_BSINDATA_WIDTH*j+:`CAL_UNIT_BSINDATA_WIDTH]),
            .fm_data_256   (fm_rep2[j]),
            .weight_in     (w_bus[j]),
            .fm_out        (cal_result_w[`CAL_UNIT_OUTDATA_WIDTH*j+:`CAL_UNIT_OUTDATA_WIDTH])
        );
    end
endgenerate

pe_result_snapshot_4x16 u_result_snapshot_4x16(
    .clk               (clk),
    .rst_n             (rst_n),
    .capture_pre_valid (end_data_valid_r4),
    .cal_result        (cal_result_w),
    .candidate_start   (conv_candidate_start),
    .candidate_advance (conv_candidate_advance),
    .candidate_done    (conv_candidate_done),
    .candidate_valid   (conv_candidate_valid),
    .candidate_data    (conv_candidate_data)
);

// Data path has four registered input-replication stages. conv_valid is the
// cycle in which the helper snapshot captures the aligned cal_result_w data.
always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        end_data_valid_r1 <= 1'b0;
    else
        end_data_valid_r1 <= end_data_valid;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        end_data_valid_r2 <= 1'b0;
    else
        end_data_valid_r2 <= end_data_valid_r1;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        end_data_valid_r3 <= 1'b0;
    else
        end_data_valid_r3 <= end_data_valid_r2;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        end_data_valid_r4 <= 1'b0;
    else
        end_data_valid_r4 <= end_data_valid_r3;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        conv_valid_r <= 1'b0;
    else
        conv_valid_r <= end_data_valid_r4;
end

/*
 * RTL SELF-CHECK:
 * PASS: pure Verilog syntax; ports are explicit wires
 * PASS: sequential logic uses nonblocking assignment only
 * PASS: only generate/genvar loops are used
 * PASS: no function, division, modulo, delay, initial block, or system task
 * PASS: the 65536-bit snapshot has one owner inside PE
 * PASS: candidate payload has no reset; update CE is quarter-local
 * PASS: four 16-channel groups preserve 32-column INT32 ordering
 * PASS: control reset is explicit; payload reset was not introduced
 * PASS: DSP math, input replication latency, conv_valid phase, and throughput are unchanged
 * FAIL: none
 */
endmodule

// =============================================================================
// Four-group PE-local convolution result snapshot.
// This helper is also the focused-test boundary. It owns candidate validity;
// each group is split into four quarter helpers and each quarter owns sixteen
// 256-bit snapshot banks, one registered load index and one 256-bit candidate.
// =============================================================================
module pe_result_snapshot_4x16(
    input  wire           clk,
    input  wire           rst_n,
    input  wire           capture_pre_valid,
    input  wire [65535:0] cal_result,
    input  wire           candidate_start,
    input  wire           candidate_advance,
    input  wire           candidate_done,
    output wire           candidate_valid,
    output wire [4095:0]  candidate_data
);

reg candidate_valid_r;

assign candidate_valid = candidate_valid_r;

pe_result_snapshot_group_16 u_group0(
    .clk               (clk),
    .rst_n             (rst_n),
    .capture_pre_valid (capture_pre_valid),
    .cal_result_group  (cal_result[0+:16384]),
    .candidate_start   (candidate_start),
    .candidate_advance (candidate_advance),
    .candidate_done    (candidate_done),
    .candidate_data    (candidate_data[0+:1024])
);

pe_result_snapshot_group_16 u_group1(
    .clk               (clk),
    .rst_n             (rst_n),
    .capture_pre_valid (capture_pre_valid),
    .cal_result_group  (cal_result[16384+:16384]),
    .candidate_start   (candidate_start),
    .candidate_advance (candidate_advance),
    .candidate_done    (candidate_done),
    .candidate_data    (candidate_data[1024+:1024])
);

pe_result_snapshot_group_16 u_group2(
    .clk               (clk),
    .rst_n             (rst_n),
    .capture_pre_valid (capture_pre_valid),
    .cal_result_group  (cal_result[32768+:16384]),
    .candidate_start   (candidate_start),
    .candidate_advance (candidate_advance),
    .candidate_done    (candidate_done),
    .candidate_data    (candidate_data[2048+:1024])
);

pe_result_snapshot_group_16 u_group3(
    .clk               (clk),
    .rst_n             (rst_n),
    .capture_pre_valid (capture_pre_valid),
    .cal_result_group  (cal_result[49152+:16384]),
    .candidate_start   (candidate_start),
    .candidate_advance (candidate_advance),
    .candidate_done    (candidate_done),
    .candidate_data    (candidate_data[3072+:1024])
);

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        candidate_valid_r <= 1'b0;
    else if(candidate_start)
        candidate_valid_r <= 1'b1;
    else if(candidate_done)
        candidate_valid_r <= 1'b0;
end

endmodule

// =============================================================================
// One physical 16-channel group.
// Snapshot layout: 16 channels x 4 quarters x 256 bits.
// Candidate layout: 4 quarters x 256 bits = one channel x 32 INT32 columns.
// =============================================================================
module pe_result_snapshot_group_16(
    input  wire           clk,
    input  wire           rst_n,
    input  wire           capture_pre_valid,
    input  wire [16383:0] cal_result_group,
    input  wire           candidate_start,
    input  wire           candidate_advance,
    input  wire           candidate_done,
    output wire [1023:0]  candidate_data
);

wire [4095:0] cal_result_q0;
wire [4095:0] cal_result_q1;
wire [4095:0] cal_result_q2;
wire [4095:0] cal_result_q3;
wire [255:0] candidate_q0;
wire [255:0] candidate_q1;
wire [255:0] candidate_q2;
wire [255:0] candidate_q3;

assign candidate_data[0+:256]   = candidate_q0;
assign candidate_data[256+:256] = candidate_q1;
assign candidate_data[512+:256] = candidate_q2;
assign candidate_data[768+:256] = candidate_q3;

genvar cq;
generate
    for (cq = 0; cq < 16; cq = cq + 1) begin: G_GROUP_QUARTER_PACK
        assign cal_result_q0[(cq*256)+:256] = cal_result_group[(cq*1024)+:256];
        assign cal_result_q1[(cq*256)+:256] = cal_result_group[(cq*1024)+256+:256];
        assign cal_result_q2[(cq*256)+:256] = cal_result_group[(cq*1024)+512+:256];
        assign cal_result_q3[(cq*256)+:256] = cal_result_group[(cq*1024)+768+:256];
    end
endgenerate

(* keep_hierarchy = "yes" *) pe_result_snapshot_quarter_16 u_quarter0(
    .clk                 (clk),
    .rst_n               (rst_n),
    .capture_pre_valid   (capture_pre_valid),
    .cal_result_quarter  (cal_result_q0),
    .candidate_start     (candidate_start),
    .candidate_advance   (candidate_advance),
    .candidate_done      (candidate_done),
    .candidate_data      (candidate_q0)
);

(* keep_hierarchy = "yes" *) pe_result_snapshot_quarter_16 u_quarter1(
    .clk                 (clk),
    .rst_n               (rst_n),
    .capture_pre_valid   (capture_pre_valid),
    .cal_result_quarter  (cal_result_q1),
    .candidate_start     (candidate_start),
    .candidate_advance   (candidate_advance),
    .candidate_done      (candidate_done),
    .candidate_data      (candidate_q1)
);

(* keep_hierarchy = "yes" *) pe_result_snapshot_quarter_16 u_quarter2(
    .clk                 (clk),
    .rst_n               (rst_n),
    .capture_pre_valid   (capture_pre_valid),
    .cal_result_quarter  (cal_result_q2),
    .candidate_start     (candidate_start),
    .candidate_advance   (candidate_advance),
    .candidate_done      (candidate_done),
    .candidate_data      (candidate_q2)
);

(* keep_hierarchy = "yes" *) pe_result_snapshot_quarter_16 u_quarter3(
    .clk                 (clk),
    .rst_n               (rst_n),
    .capture_pre_valid   (capture_pre_valid),
    .cal_result_quarter  (cal_result_q3),
    .candidate_start     (candidate_start),
    .candidate_advance   (candidate_advance),
    .candidate_done      (candidate_done),
    .candidate_data      (candidate_q3)
);

endmodule

// =============================================================================
// One 8-column quarter across sixteen channels.
// The selector is driven only by the registered local load index. The local
// load enable drives only this quarter's 256 candidate payload FFs.
// =============================================================================
module pe_result_snapshot_quarter_16(
    input  wire          clk,
    input  wire          rst_n,
    input  wire          capture_pre_valid,
    input  wire [4095:0] cal_result_quarter,
    input  wire          candidate_start,
    input  wire          candidate_advance,
    input  wire          candidate_done,
    output wire [255:0]  candidate_data
);

(* keep = "true", dont_touch = "yes" *) reg [3:0] candidate_load_idx_r;
reg [255:0] candidate_data_r;
wire [255:0] snapshot_bank_w [0:15];
wire [255:0] snapshot_selected;
(* keep = "true", dont_touch = "yes", max_fanout = 256 *) wire candidate_load_en_local;

assign candidate_data = candidate_data_r;
assign candidate_load_en_local = candidate_start | candidate_advance;

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        candidate_load_idx_r <= 4'd0;
    else if(candidate_start)
        candidate_load_idx_r <= 4'd1;
    else if(candidate_advance)
        candidate_load_idx_r <= candidate_load_idx_r + 1'b1;
    else if(candidate_done)
        candidate_load_idx_r <= 4'd0;
end

genvar sb;
generate
    for (sb = 0; sb < 16; sb = sb + 1) begin: G_QUARTER_SNAPSHOT_BANK
        pe_result_snapshot_bank_256 u_bank(
            .clk               (clk),
            .rst_n             (rst_n),
            .capture_pre_valid (capture_pre_valid),
            .capture_data      (cal_result_quarter[(sb*256)+:256]),
            .snapshot_data     (snapshot_bank_w[sb])
        );
    end
endgenerate

assign snapshot_selected = (candidate_load_idx_r == 4'd0)  ? snapshot_bank_w[0]  :
                           (candidate_load_idx_r == 4'd1)  ? snapshot_bank_w[1]  :
                           (candidate_load_idx_r == 4'd2)  ? snapshot_bank_w[2]  :
                           (candidate_load_idx_r == 4'd3)  ? snapshot_bank_w[3]  :
                           (candidate_load_idx_r == 4'd4)  ? snapshot_bank_w[4]  :
                           (candidate_load_idx_r == 4'd5)  ? snapshot_bank_w[5]  :
                           (candidate_load_idx_r == 4'd6)  ? snapshot_bank_w[6]  :
                           (candidate_load_idx_r == 4'd7)  ? snapshot_bank_w[7]  :
                           (candidate_load_idx_r == 4'd8)  ? snapshot_bank_w[8]  :
                           (candidate_load_idx_r == 4'd9)  ? snapshot_bank_w[9]  :
                           (candidate_load_idx_r == 4'd10) ? snapshot_bank_w[10] :
                           (candidate_load_idx_r == 4'd11) ? snapshot_bank_w[11] :
                           (candidate_load_idx_r == 4'd12) ? snapshot_bank_w[12] :
                           (candidate_load_idx_r == 4'd13) ? snapshot_bank_w[13] :
                           (candidate_load_idx_r == 4'd14) ? snapshot_bank_w[14] :
                                                            snapshot_bank_w[15];

always @(posedge clk) begin
    if(candidate_load_en_local)
        candidate_data_r <= snapshot_selected;
end

/*
 * RTL SELF-CHECK:
 * PASS: selector input is one registered local index; no advance-dependent data select
 * PASS: only generate/genvar loops and explicit constant-bank selection are used
 * PASS: candidate payload has no reset and uses one quarter-local CE
 * PASS: local index is reset control only and is protected from cross-quarter merging
 * FAIL: none
 */
endmodule

// =============================================================================
// One 256-bit snapshot bank. The local capture pulse register is intentionally
// protected so synthesis cannot merge 256 intended bank-local CE drivers.
// =============================================================================
module pe_result_snapshot_bank_256(
    input  wire         clk,
    input  wire         rst_n,
    input  wire         capture_pre_valid,
    input  wire [255:0] capture_data,
    output wire [255:0] snapshot_data
);

(* keep = "true", dont_touch = "yes" *) reg capture_en_r;
reg [255:0] snapshot_data_r;

assign snapshot_data = snapshot_data_r;

always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        capture_en_r <= 1'b0;
    else
        capture_en_r <= capture_pre_valid;
end

always @(posedge clk) begin
    if(capture_en_r)
        snapshot_data_r <= capture_data;
end

/*
 * RTL SELF-CHECK:
 * PASS: the 256-bit payload is not reset
 * PASS: the only reset state is the protected 1-bit local capture control
 * PASS: local capture control drives only one 256-bit bank CE
 * FAIL: none
 */
endmodule
