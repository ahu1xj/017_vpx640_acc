// 取数级 + 70B 拼接级（v3：固定 2 拍延迟流水，无状态机）。
// 合约：frdvalid 结构保证 = frden 打 2 拍（prefetch_fm_read_mux owner 标签打 2 拍，
// RAM 为 BMG IP（enb 常使能，DO_REG 每拍推进）固定 2 拍读延迟，无仲裁/变延迟），
// 按序返回、最多 2 个在途，因此发射 valid 打 2 拍即为捕获 valid，边带随延迟链下行，
// 无需请求/响应状态。
// - 发射段：有 token 且无反压即发（每拍 1 件）；col_valid=0 不发读，zero 标记
//   随延迟链下行，捕获拍数据选 0。
// - 拼接段：按位置计数器落 cpos；末 chunk 到达当拍直通推入 fifo_win；
//   该拍 win_full 时整窗快照压入 hold 栈（栈深 2），期间停发，
//   win_full 解除后弹栈推窗。弹栈只读快照不读 cpos：发射虽停，
//   d_valid_r 存货仍会流至捕获级覆写 cpos，cpos 内容不可复用。
// - ???????? FIFO??????? elastic buffer ?????
// - fm_sub_sig 时序：补零 last token 在发射拍、真读 last 在捕获拍（较 v1 晚 1 拍，
//   行释放只晚不早，对写侧覆写更保守，无 RAW 风险）。
// 输入 token: { addr[31:0], hilow, col_valid, last } = 35bit
module prefetch_fm_conv_read_asm(
    input           clk,
    input           rst_n,

    input   [2:0]   cfg_slot_num,
    input   [1:0]   cfg_slot_start,
    input   [1:0]   cfg_slot_last,

    input           t4_empty,
    input   [34:0]  t4_rd_data,
    output          t4_rd_en,

    output          frden,
    output  [13:0]  frdaddr,
    input           frdvalid,
    input   [511:0] frddata,

    input           win_full,
    output          win_wr_en,
    output  [559:0] win_wr_data,

    output          fm_sub_sig,
    output          asm_busy
);

// --------------------------------------------------------
// 发射段：位置计数 + 每拍一发
// --------------------------------------------------------
reg [1:0]          asm_cnt_r;
reg [559:0]        hold0_reg;
reg [559:0]        hold1_reg;
reg [1:0]          hold_cnt_r;

reg                d_valid_r;
reg                d_valid_r2;
reg                d_col_valid_r;
reg                d_col_valid_r2;
reg                d_hilow_r;
reg                d_hilow_r2;
reg                d_last_r;
reg                d_last_r2;
reg                d_last_chunk_r;
reg                d_last_chunk_r2;
reg [2:0]          d_pos_r;
reg [2:0]          d_pos_r2;

reg [255:0]        cpos0_reg;
reg [255:0]        cpos1_reg;
reg [255:0]        cpos2_reg;
reg [255:0]        cpos3_reg;
reg                fm_sub_reg;

wire               t4_last;
wire               t4_col_valid;
wire               t4_hilow;
wire [31:0]        t4_addr;
wire               last_chunk_issue;
wire               last_chunk_arrive;
wire               issue_block;
wire               issue_fire;
wire               push_now_fire;
wire               hold_push;
wire               hold_pop;
wire               capture_fire;
wire [255:0]       chunk_new;
wire [2:0]         asm_position;
wire [255:0]       cpos0_sel;
wire [255:0]       cpos1_sel;
wire [255:0]       cpos2_sel;
wire [255:0]       cpos3_sel;
wire [559:0]       win_wr_now;

assign t4_last      = t4_rd_data[0];
assign t4_col_valid = t4_rd_data[1];
assign t4_hilow     = t4_rd_data[2];
assign t4_addr      = t4_rd_data[34:3];

// slot_num ∈ {1,2,3,4}：cfg_slot_num[1:0]-2'd1 对 4 回绕为 3，恰为末位索引。
assign last_chunk_issue  = (asm_cnt_r == (cfg_slot_num[1:0] - 2'd1));
assign last_chunk_arrive = d_valid_r2 && d_last_chunk_r2;

// 反压汇总：fifo_win 满且末 chunk 到达（本拍压快照）或快照未空，停发新 token；
assign issue_block = (hold_cnt_r != 2'd0) |
                     (last_chunk_arrive && win_full);
assign issue_fire  = !t4_empty && !issue_block;

assign t4_rd_en = issue_fire;
assign frden    = issue_fire && t4_col_valid;
assign frdaddr  = t4_addr[13:0];

assign asm_position = {1'b0, cfg_slot_start} + {1'b0, asm_cnt_r};

// --------------------------------------------------------
// 2 拍延迟链：发射 valid 打 2 拍 = 捕获 valid（BMG 2 拍读延迟，结构保证）
// --------------------------------------------------------
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        d_valid_r <= 1'b0;
    else
        d_valid_r <= issue_fire;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        d_valid_r2 <= 1'b0;
    else
        d_valid_r2 <= d_valid_r;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        d_col_valid_r  <= 1'b0;
        d_hilow_r      <= 1'b0;
        d_last_r       <= 1'b0;
        d_last_chunk_r <= 1'b0;
        d_pos_r        <= 3'd0;
    end
    else if (issue_fire) begin
        d_col_valid_r  <= t4_col_valid;
        d_hilow_r      <= t4_hilow;
        d_last_r       <= t4_last;
        d_last_chunk_r <= last_chunk_issue;
        d_pos_r        <= asm_position;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        d_col_valid_r2  <= 1'b0;
        d_hilow_r2      <= 1'b0;
        d_last_r2       <= 1'b0;
        d_last_chunk_r2 <= 1'b0;
        d_pos_r2        <= 3'd0;
    end
    else begin
        d_col_valid_r2  <= d_col_valid_r;
        d_hilow_r2      <= d_hilow_r;
        d_last_r2       <= d_last_r;
        d_last_chunk_r2 <= d_last_chunk_r;
        d_pos_r2        <= d_pos_r;
    end
end

assign capture_fire = d_valid_r2;
assign chunk_new    = d_col_valid_r2 ? (d_hilow_r2 ? frddata[511:256] : frddata[255:0])
                                     : 256'd0;

// --------------------------------------------------------
// 位置计数：窗口末 chunk 发射后归 0
// --------------------------------------------------------
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        asm_cnt_r <= 2'd0;
    else if (issue_fire)
        asm_cnt_r <= last_chunk_issue ? 2'd0 : (asm_cnt_r + 2'd1);
end

// --------------------------------------------------------
// 拼接寄存器：认捕获 valid 落位
// --------------------------------------------------------
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        cpos0_reg <= 256'd0;
    else if (capture_fire && (d_pos_r2 == 3'd0))
        cpos0_reg <= chunk_new;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        cpos1_reg <= 256'd0;
    else if (capture_fire && (d_pos_r2 == 3'd1))
        cpos1_reg <= chunk_new;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        cpos2_reg <= 256'd0;
    else if (capture_fire && (d_pos_r2 == 3'd2))
        cpos2_reg <= chunk_new;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        cpos3_reg <= 256'd0;
    else if (capture_fire && (d_pos_r2 == 3'd3))
        cpos3_reg <= chunk_new;
end

// --------------------------------------------------------
// 推窗：末 chunk 到达当拍直通（NBA 下 cpos 同拍写入不影响直通路径）；
// 该拍 win_full 则整窗快照压栈（当拍 cpos 组合输出完整正确），
// 栈未空期间停发，win_full 解除后弹栈推窗。
// 快照栈深 2：cpos 组装级 + d_valid_r 单拍存货各可能贡献一个待推窗，
// 保序契约（20260811 修）：hold 栈非空时禁止直通推窗，新窗一律压栈；
// 同拍 hold_push+hold_pop（仅 cnt==1 可能）出最旧快照、新窗进 hold0、cnt 不变，
// 杜绝新窗超车旧快照导致相邻窗乱序（188MHz 板上实证为相邻 cin 整 tile 互换）。
// 停发后延迟链不再有新 token，栈不会溢出。
// --------------------------------------------------------
assign push_now_fire  = last_chunk_arrive && !win_full && (hold_cnt_r == 2'd0);
assign hold_push      = last_chunk_arrive && (win_full || (hold_cnt_r != 2'd0));
assign hold_pop       = (hold_cnt_r != 2'd0) && !win_full;
assign win_wr_en      = push_now_fire || hold_pop;

// 末 chunk 到达拍（直通推窗或快照压栈）末位置 chunk 走直通，其余位置用 cpos；
// NBA 下 cpos 同拍写入不影响直通路径。快照压栈拍同为末 chunk 到达拍，直通必须生效，
// 否则快照末位置会装上一窗残留（cpos 寄存器当拍尚未写入）。
assign cpos0_sel = (last_chunk_arrive && (d_pos_r2 == 3'd0)) ? chunk_new : cpos0_reg;
assign cpos1_sel = (last_chunk_arrive && (d_pos_r2 == 3'd1)) ? chunk_new : cpos1_reg;
assign cpos2_sel = (last_chunk_arrive && (d_pos_r2 == 3'd2)) ? chunk_new : cpos2_reg;
assign cpos3_sel = (last_chunk_arrive && (d_pos_r2 == 3'd3)) ? chunk_new : cpos3_reg;

// --------------------------------------------------------
// 快照压栈：末 chunk 到达且 win_full 拍，cpos 组合输出是完整窗口。
// 压栈后 cpos 寄存器允许被后续 token 捕获覆写（发射虽停，d_valid_r
// 存货仍会流至捕获级），弹栈只读快照，不读 cpos。
// --------------------------------------------------------
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        hold0_reg <= 560'd0;
    else if (hold_push && ((hold_cnt_r == 2'd0) || hold_pop))
        hold0_reg <= win_wr_now;
    else if (hold_pop && (hold_cnt_r == 2'd2))
        hold0_reg <= hold1_reg;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        hold1_reg <= 560'd0;
    else if (hold_push && !hold_pop && (hold_cnt_r != 2'd0))
        hold1_reg <= win_wr_now;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        hold_cnt_r <= 2'd0;
    else begin
        case ({hold_push, hold_pop})
            2'b10:   hold_cnt_r <= hold_cnt_r + 2'd1;
            2'b01:   hold_cnt_r <= hold_cnt_r - 2'd1;
            default: hold_cnt_r <= hold_cnt_r;
        endcase
    end
end

assign win_wr_now[23:0]      = (cfg_slot_start == 2'd0) ? cpos0_sel[255:232] : 24'd0;
assign win_wr_now[279:24]    = cpos1_sel;
assign win_wr_now[535:280]   = (cfg_slot_last >= 2'd2) ? cpos2_sel : 256'd0;
assign win_wr_now[559:536]   = (cfg_slot_last == 2'd3) ? cpos3_sel[23:0] : 24'd0;

// 弹栈推窗只读快照；直通推窗用当拍组合（cpos 未被覆写，同拍完整）。
assign win_wr_data = push_now_fire ? win_wr_now : hold0_reg;

// --------------------------------------------------------
// fm_sub_sig / asm_busy：时序与 v1 逐拍一致
// --------------------------------------------------------
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        fm_sub_reg <= 1'b0;
    else
        fm_sub_reg <= (issue_fire && !t4_col_valid && t4_last) ||
                      (capture_fire && d_col_valid_r2 && d_last_r2);
end

assign fm_sub_sig = fm_sub_reg;
assign asm_busy   = frden | d_valid_r | d_valid_r2 | (hold_cnt_r != 2'd0);

endmodule

// ============================================================
// RTL Self-Check Results
// ============================================================
// [PASS] Rule #1:  All ports have explicit width declarations
// [PASS] Rule #2:  Ports use Verilog wire/reg conventions
// [PASS] Rule #6:  No debug/error-attribution logic or diagnostic-only sideband in RTL
// [PASS] Rule #8:  No logic/always_ff/always_comb used
// [PASS] Rule #12: All conditional assigns have both branches
// [PASS] Rule #16: Non-blocking '<=' for all sequential logic
// [PASS] Rule #24: No multipliers; window compose is static slice select
// [PASS] No FSM states: issue/delay/capture/push is a pure pipeline
// [PASS] frdvalid treated as fixed issue+2 (structural contract of read_mux+BMG 2-cycle RAM, enb const-enabled)
// [PASS] hold-stack depth 2 bound: issue stops while stack non-empty, so at most the
//        cpos-level window plus the single d_valid_r in-flight token can pend; no overflow
// [WARN] frdvalid port kept for interface compatibility; capture keys off the delay line
// ============================================================
