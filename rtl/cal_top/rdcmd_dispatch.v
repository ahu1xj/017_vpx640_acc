`include "all_defines.vh"

module rdcmd_dispatch(
    input  wire         clk,
    input  wire         rst_n,

    input  wire         s_axis_rdcmd_tvalid,
    input  wire [111:0] s_axis_rdcmd_tdata,
    output wire         s_axis_rdcmd_tready,

    output wire         rdcmd_trans_valid,
    output wire [111:0] rdcmd_trans,

    input  wire         store_done_req,
    output wire         store_done_ack
);

localparam [2:0] ST_IDLE           = 3'd0;
localparam [2:0] ST_PULSE          = 3'd1;
localparam [2:0] ST_WAIT_WRITEBACK = 3'd2;
localparam [2:0] ST_ACK            = 3'd3;

reg [2:0]   state;
reg [2:0]   next_state;
(* DONT_TOUCH = "TRUE", SHREG_EXTRACT = "NO" *) reg store_done_ack_reg;
reg [111:0] cmd_latch;
reg         cmd_is_conv;
reg [6:0]   conv_group_total;
reg [6:0]   pulse_count;
(* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg store_done_req_d1;
(* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg store_done_req_d2;

wire        cmd_fire;
wire [3:0]  cmd_model_sel_in;
wire [5:0]  cmd_fm_channel_out_in;
wire [6:0]  cmd_conv_group_total_in;
wire        conv_more_group;

// 契约（2026-08-05 语义重构，详 docs/handoff/solution-20260805-v5s-multiexport-dispatch-handshake.md）：
// store_done 握手只服务卷积：一次握手 = 一组卷积写回完成。非卷积指令（POOL/RESADD/
// CONCAT/UPSAMPLE/EXPORT 等）不消费握手——其层间顺序由 load 侧 fm_done_latch 保证
// （store 写完整张特征图后 load 才接收并转发下一条读指令）。
// 卷积指令（G 组）：收下后立即广播第 1 组，之后每等到一次握手、组计数加 1、
// 以同一信息再广播；最后一组的握手握完不再广播，清除本层状态回 IDLE。
// 供需自闭合：store 仅对卷积组写回完成（is_type1&act_done）产 req，每层 G 产 G 消。
// EXPORT 不由 load_core_tagged 转发（cal 域无消费者）。
assign cmd_fire                = s_axis_rdcmd_tvalid & s_axis_rdcmd_tready;
assign cmd_model_sel_in        = s_axis_rdcmd_tdata[`RD_MODEL_SEL_START +: `RD_MODEL_SEL_WIDTH];
assign cmd_fm_channel_out_in   = s_axis_rdcmd_tdata[`RD_FM_CHANNEL_OUT_START +: `RD_FM_CHANNEL_OUT_WIDTH];
assign cmd_conv_group_total_in = {1'b0, cmd_fm_channel_out_in} + 7'd1;
// pulse_count = 正在广播/等待的组索引（0 起）：收下一条时归 0，每完成一次握手
// 且仍有后续组时 +1。conv_more_group = 当前组不是末组。
assign conv_more_group         = cmd_is_conv & (pulse_count < (conv_group_total - 7'd1));

assign s_axis_rdcmd_tready = (state == ST_IDLE);
assign rdcmd_trans_valid   = (state == ST_PULSE);
assign rdcmd_trans         = cmd_latch;
assign store_done_ack      = store_done_ack_reg;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        store_done_req_d1 <= 1'b0;
    else
        store_done_req_d1 <= store_done_req;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        store_done_req_d2 <= 1'b0;
    else
        store_done_req_d2 <= store_done_req_d1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        state <= ST_IDLE;
    else
        state <= next_state;
end

// Register the source side of the cross-clock acknowledgment.  Driving the
// synchronizer from a flip-flop Q avoids decode glitches while preserving the
// original Moore timing: ACK is high exactly while state is ST_ACK.
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        store_done_ack_reg <= 1'b0;
    else
        store_done_ack_reg <= (next_state == ST_ACK);
end

always @(*) begin
    next_state = state;
    case (state)
        ST_IDLE: begin
            if (cmd_fire)
                next_state = ST_PULSE;
        end
        ST_PULSE: begin
            // 广播一拍；卷积等本组写回握手，非卷积直接完工
            if (cmd_is_conv)
                next_state = ST_WAIT_WRITEBACK;
            else
                next_state = ST_IDLE;
        end
        ST_WAIT_WRITEBACK: begin
            if (store_done_req_d2)
                next_state = ST_ACK;
        end
        ST_ACK: begin
            if (!store_done_req_d2)
                next_state = conv_more_group ? ST_PULSE : ST_IDLE;
        end
        default: begin
            next_state = ST_IDLE;
        end
    endcase
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        cmd_latch <= 112'd0;
    else if (cmd_fire)
        cmd_latch <= s_axis_rdcmd_tdata;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        cmd_is_conv <= 1'b0;
    else if (cmd_fire)
        cmd_is_conv <= (cmd_model_sel_in == `MODEL_SEL_CONV);
    else if ((state != ST_IDLE) && (next_state == ST_IDLE))
        cmd_is_conv <= 1'b0;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        conv_group_total <= 7'd0;
    else if (cmd_fire)
        conv_group_total <= (cmd_model_sel_in == `MODEL_SEL_CONV) ? cmd_conv_group_total_in : 7'd1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        pulse_count <= 7'd0;
    else if (cmd_fire)
        pulse_count <= 7'd0;
    else if ((state == ST_ACK) && !store_done_req_d2)
        pulse_count <= conv_more_group ? (pulse_count + 7'd1) : 7'd0;
end

// ============================================================
// RTL Self-Check Results
// ============================================================
// [PASS] Ports have explicit direction and width declarations.
// [PASS] Pure Verilog RTL only; no SystemVerilog keywords used.
// [PASS] Clocked logic uses nonblocking assignments.
// [PASS] Combinational next-state logic has defaults and a default case.
// [PASS] store_done_req uses a two-flop synchronizer before FSM use.
// [PASS] store_done_ack crosses clocks from a dedicated source register.
// [PASS] No debug-only ports, system tasks, initial blocks, or delays.
// ============================================================

endmodule
