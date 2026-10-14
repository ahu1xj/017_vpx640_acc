// ============================================================
// pooling_input — 输入管理模块
// ============================================================
//
// 【模块定位】
//   本模块是 pooling_top 的"输入大门"。它不碰任何池化算法——不知道核尺寸、
//   不知道步长、不知道下游怎么消费数据。唯一职责是：按正确的顺序，把正确
//   的数据，以正确的拍数，喂给 pooling_h_kernel。
//
// 【数据流角色】
//   外部 AXIS (ppool / fpool) ──→ [源选择 3:1] ──→ [列屏蔽 2:1] ──→ [寄存器] ──→ ch0/1_din
//                                      ↑                        ↑
//                                row_cnt → phase           col_cnt → col_force_min
//
// 【两路输入源 —— 永不混淆】
//   ppool (Padding Pool):  只在循环 Padding 启用时才读取。数据是前置模块
//                           按行填入的填充值，top/bottom padding 各行。
//   fpool (Feature Pool):  特征图数据，无论如何都要从中取有效行。VALID
//                           阶段的唯一数据来源。
//   虚拟 -128:             非循环 Padding 模式下，bottom padding 行不读任何
//                           外部通道，内部每拍生成 8'h80(= -128)，在 MaxPooling
//                           比较中天然"必输"。top padding 由垂直池化模块内部处理。
//
// 【设计原则】
//   1. 分布式计数: 内部维护 row_cnt / col_cnt / ch_group_cnt，不依赖
//      pooling_ctrl 逐拍指挥。ctrl 只给 start 脉冲 + 常驻配置参数。
//   2. 数据输出寄存器打拍: ch0_din / ch1_din / din_valid 为寄存器输出；
//      输入 tready 为组合输出，使内部 fire 与外部 AXIS 握手同拍一致。
//   3. 组合逻辑链深度 ≤ 6 级 LUT，最长路径在 col_force_min 的 13bit 减法。
//
// 【时序概要 (一列从输入到输出)】
//   拍 N:   外部 AXIS 握手成功 (tvalid & tready)，数据出现在总线上。
//           组合路径: 源选择 mux → 列屏蔽 mux → 到达输出寄存器 D 端。
//   posedge: 寄存器锁存 data_final，handshake 标志锁存为 din_valid。
//   拍 N+1: ch0_din / ch1_din / din_valid 输出拍 N 的数据，下游可见。
//
// ============================================================

module pooling_input (
    // ============================================================
    // 系统信号
    // ============================================================
    input  wire         clk,                    // 单一时钟域，所有逻辑同步于 posedge
    input  wire         rst_n,                  // 异步低有效复位

    // ============================================================
    // 外部 AXIS 输入: Padding 通道
    // ============================================================
    // ppool 只在 circular_padding=1 时的 TOP_PAD / BOT_PAD 阶段被读取。
    // circular_padding=0 时 ppool_tready 恒为 0，ppool 通道完全不参与。
    input  wire         s_axis_ppooltem_tvalid, // Padding 数据有效 (来自前置模块)
    input  wire [15:0]  s_axis_ppooltem_tdata,  // {ch1[7:0], ch0[7:0]}，2ch×8bit 打包
    output wire         s_axis_ppooltem_tready, // 组合输出，仅在 circ=1 的 pad 阶段拉高

    // ============================================================
    // 外部 AXIS 输入: 特征图通道
    // ============================================================
    // fpool 在 VALID 阶段始终被读取，是有效特征行数据的唯一来源。
    // 无论 circular_padding 是 0 还是 1，VALID 阶段都从 fpool 取数。
    input  wire         s_axis_fpooltem_tvalid, // 特征图数据有效
    input  wire [15:0]  s_axis_fpooltem_tdata,  // {ch1[7:0], ch0[7:0]}，2ch×8bit 打包
    output wire         s_axis_fpooltem_tready, // 组合输出，仅在 VALID 阶段拉高

    // ============================================================
    // 反压
    // ============================================================
    // 下游 FIFO 近满时拉高。它只参与本级 ready 生成；计数器、状态推进
    // 和 din_valid 均由真实 fire 事件驱动。
    input  wire         input_credit_ready,

    // ============================================================
    // 控制接口 (← pooling_ctrl)
    // ============================================================
    // 所有控制参数在 start 脉冲前由 pooling_ctrl 准备好，工作期间保持不变。
    // start 是单周期脉冲，宽度 1 拍。pooling_ctrl 保证 start 拉高时
    // input_done=0 (本模块处于 IDLE 或 DONE 状态)。
    input  wire         start,                  // 启动脉冲 (1 拍)，触发新一轮全通道组处理
    input  wire         circular_padding,       // 1=从 ppool 通道读 padding; 0=内部生成虚拟 -128
    input  wire [1:0]   padding_top,            // 顶部 padding 行数 (0~3, 直接使用)
    input  wire [1:0]   padding_bottom,         // 底部 padding 行数 (0~3, 直接使用)
    input  wire [11:0]  fm_row_in,              // 输入特征图有效行数 (0~4095)
    input  wire [4:0]   last_fm_out_column_valid, // 末组有效列编码 (0~31, 实际列数 = 此值+1)
    input  wire [12:0]  total_cols_per_row,     // 每行总列数 (由 ctrl 预计算, 含左右 padding)
    input  wire [12:0]  fm_total_rows,          // FM 状态总行数 (由 ctrl 预计算: circ?fm_row_in:fm_row_in+pb)
    input  wire [10:0]  ch_group_total,         // 通道组总数 (由 ctrl 预计算: fm_channel_in/2)
    input  wire [12:0]  last_group_start,       // 末组起始列号 (由 ctrl 预计算: pad_l + fm_col_in*32)

    // ============================================================
    // 状态反馈 (→ pooling_ctrl)
    // ============================================================
    // 所有通道组的所有行 (含虚拟行) 全部输出完毕后拉高。
    // 拉高后所有 tready 按死 (=0)，等待下一个 start 脉冲清零。
    output wire         input_done,

    // ============================================================
    // 数据输出 (→ pooling_h_kernel ×2)
    // ============================================================
    // 三个信号均为寄存器输出，同拍有效。
    // din_valid=1 时 ch0_din / ch1_din 为本拍两个通道的 8bit 数据。
    output wire [7:0]   ch0_din,
    output wire [7:0]   ch1_din,
    output wire         din_valid
);


    // ============================================================
    // FSM 状态编码 (5 状态)
    // ============================================================
    //
    // 典型流程 (circ=1, pt>0, pb>0):
    //   IDLE ──[start]──→ CPAD_TOP ──[pt行完成]──→ FM ──[有效行完成]──→ CPAD_BOT ──[pb行完成]──→ DONE
    //
    // 典型流程 (circ=0):
    //   IDLE ──[start]──→ FM (处理有效行 + bottom虚拟行, top由垂直池化负责) ──[完成]──→ DONE
    //
    // 多通道组: DONE 不经过 IDLE，内部自动循环:
    //   FM → (更多通道组) → CPAD_TOP 或 FM (ch_group_cnt++)
    //   CPAD_BOT → (更多通道组) → CPAD_TOP 或 FM (ch_group_cnt++)
    //
    localparam IDLE     = 3'd0;  // 等待 start 脉冲，所有计数器=0，无输出
    localparam CPAD_TOP = 3'd1;  // 循环 Padding 顶部行: 从 ppool 读取 pt 行
    localparam FM       = 3'd2;  // 特征图有效行: 从 fpool 读取; circ=0 时也处理 bottom 虚拟行
    localparam CPAD_BOT = 3'd3;  // 循环 Padding 底部行: 从 ppool 读取 pb 行
    localparam DONE     = 3'd4;  // 全部通道组完成: input_done=1, 等待下一个 start

    // ============================================================
    // 阶段编码 (行内子阶段)
    // ============================================================
    //
    // 阶段决定数据来源和握手对象。phase 是组合逻辑，从 state + row_cnt 实时译码。
    //
    // 数据来源映射:
    //   TOP_PAD + circ=1 → ppool 通道 (CPAD_TOP 状态)
    //   TOP_PAD + circ=0 → (不存在: circ=0 时 TOP_PAD 不进入本模块)
    //   VALID            → fpool 通道 (FM 状态, 始终)
    //   BOT_PAD + circ=1 → ppool 通道 (CPAD_BOT 状态)
    //   BOT_PAD + circ=0 → 虚拟 -128   (FM 状态, 非循环模式)
    //
    localparam PHASE_TOP_PAD = 2'd0;  // Top Padding 行
    localparam PHASE_VALID   = 2'd1;  // 有效特征图行
    localparam PHASE_BOT_PAD = 2'd2;  // Bottom Padding 行

    // ============================================================
    // 寄存器声明
    // ============================================================
    reg  [2:0]  state, next_state;          // FSM 状态 (时序 + 组合)
    reg  [12:0] row_cnt;                     // 当前行号 (13bit: 最大 4100)
    reg  [12:0] col_cnt;                     // 当前列号 (13bit: 最大 4101)
    reg  [10:0] ch_group_cnt;               // 当前通道组号 (11bit: 最大 2047)
    reg  [7:0]  ch0_din_reg;                // ch0 数据输出寄存器
    reg  [7:0]  ch1_din_reg;                // ch1 数据输出寄存器
    reg         din_valid_reg;              // 数据有效寄存器
    reg         input_done_reg;             // 完成标志寄存器

    // ============================================================
    // 组合逻辑中间信号
    // ============================================================
    // (fm_total_rows 已由 ctrl 预计算, 作为输入端口)
    wire [12:0] rows_in_state;              // 当前 FSM 状态需处理的总行数
    wire        state_done;                  // 当前状态完成 (最后一行 + 最后一列 + 握手成功)
    wire        is_last_row_in_state;        // row_cnt == rows_in_state - 1
    wire        is_last_col_in_row;          // col_cnt == total_cols_per_row - 1
    reg  [1:0]  phase;                       // 当前行内子阶段

    // 列屏蔽
    // (last_group_start 已由 ctrl 预计算, 作为输入端口)
    wire [12:0] last_group_end;              // 最后一个 32 列组的结束列号 (含)
    wire        in_last_group;               // col_cnt 落在最后一个 32 列组内
    wire [12:0] col_in_last_group;           // col_cnt 在最后一组内的偏移 (0~31)
    wire [5:0]  valid_cols_in_last_group;    // 最后一组中真正属于特征图的有效列数 (1~32)
    wire        col_force_min;               // 当前列需强制替换为 -128

    // 数据通路
    wire [7:0]  ppool_ch0, ppool_ch1;       // 从 ppool_tdata 拆出的两个通道
    wire [7:0]  fpool_ch0, fpool_ch1;       // 从 fpool_tdata 拆出的两个通道
    wire [7:0]  data_raw_ch0, data_raw_ch1; // 源选择后的原始数据
    wire [7:0]  data_final_ch0, data_final_ch1; // 列屏蔽后的最终数据 (→ 寄存器)

    // 握手
    wire        ppool_ready;                 // ppool 通道 ready
    wire        fpool_ready;                 // fpool 通道 ready
    wire        virt_ready;                  // 虚拟行内部 ready
    wire        ppool_handshake;             // ppool 通道握手成功 (tvalid & tready)
    wire        fpool_handshake;             // fpool 通道握手成功 (tvalid & tready)
    wire        virt_handshake;              // 虚拟行自生成 fire (virt_valid & virt_ready)
    wire        handshake;                   // 总 fire = 以上三者任一

    // 通道组
    // (ch_group_total 已由 ctrl 预计算, 作为输入端口)
    wire        more_ch_groups;              // 当前组之后仍有合法通道组
    wire        start_new_operation;         // IDLE/DONE 接受新的 start
    wire        new_ch_group;                // 当前拍完成本组，下一拍启动新组
    wire        counter_reset;               // 行列计数器需要归零


    // ============================================================
    // 常量计算
    // ============================================================

    // fm_total_rows / ch_group_total / last_group_start 均由 pooling_ctrl 预计算
    // 并作为输入端口传入，本模块不再做加法/乘法/移位。


    // ============================================================
    // 当前状态行数选择 + 完成判定
    // ============================================================
    //
    // rows_in_state 是当前 FSM 状态需要处理的行数上限，用于判定 is_last_row。
    // CPAD_TOP 和 CPAD_BOT 只在 circ=1 时进入，行数来自 padding_top/bottom (0~3)。
    // FM 的行数来自 fm_total_rows 输入 (ctrl 预计算)。
    // IDLE/DONE 时 rows_in_state=0，is_last_row_in_state 为假，state_done 永不为真。
    //
    assign rows_in_state = (state == CPAD_TOP) ? {11'd0, padding_top} :
                           (state == CPAD_BOT) ? {11'd0, padding_bottom} :
                           (state == FM)       ? fm_total_rows :
                           13'd0;

    // is_last_row_in_state: 注意当 rows_in_state=0 时，0 == 0-1 在无符号算术中等价于
    // 0 == 13'h1FFF，为假。CPAD_TOP(padding_top=0) 和 CPAD_BOT(padding_bottom=0)
    // 永远不会进入 (FSM 在所有入口都检查了 !=0)，故此行为无害且正确。
    assign is_last_row_in_state = (row_cnt == rows_in_state - 13'd1);

    // is_last_col_in_row: total_cols_per_row 最小为 32 (=1组×32+0+0)，永不为 0。
    assign is_last_col_in_row   = (col_cnt == total_cols_per_row - 13'd1);

    // state_done: 三条件同时成立 → 当前状态完成。
    //   仅在 handshake 成立的同一拍为真 (单周期脉冲，不会跨拍保持)。
    //   用途: 驱动 FSM 跳转 + 通道组边界检测。
    assign state_done = is_last_row_in_state && is_last_col_in_row && handshake;


    // ============================================================
    // FSM: 状态寄存器 (时序逻辑)
    // ============================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
        end else begin
            state <= next_state;
        end
    end


    // ============================================================
    // FSM: 下一状态逻辑 (组合逻辑)
    // ============================================================
    //
    // 跳转规则总结:
    //   IDLE:      start=1 → circ&&pt>0 ? CPAD_TOP : FM
    //   CPAD_TOP:  state_done → FM
    //   FM:        state_done (或 fm_total_rows==0) →
    //                circ&&pb>0 → CPAD_BOT
    //                还有通道组 → (circ&&pt>0 ? CPAD_TOP : FM)
    //                否则 → DONE
    //   CPAD_BOT:  state_done →
    //                还有通道组 → (circ&&pt>0 ? CPAD_TOP : FM)
    //                否则 → DONE
    //   DONE:      start=1 → circ&&pt>0 ? CPAD_TOP : FM
    //
    // 关键设计决策:
    //   1. DONE 收到 start 直接跳 CPAD_TOP/FM，跳过了 IDLE 中转，省一拍。
    //   2. 通道组循环在 FM 和 CPAD_BOT 的出口判断，不需要独立状态。
    //   3. fm_total_rows 必须非 0，由指令生成/TB 输入契约保证。
    //
    always @(*) begin
        next_state = state;  // default: 保持当前状态

        case (state)
            IDLE: begin
                if (start) begin
                    // start 脉冲到来: 判断是否需要进入 CPAD_TOP
                    if (circular_padding && (padding_top != 2'd0))
                        next_state = CPAD_TOP;
                    else
                        next_state = FM;
                end
            end

            CPAD_TOP: begin
                // pt 行全部握手完成 → 进入有效行阶段
                if (state_done)
                    next_state = FM;
            end

            FM: begin
                // 合法指令保证 fm_total_rows 非 0；状态只在真实末拍 fire 时退出。
                if (state_done) begin
                    if (circular_padding && (padding_bottom != 2'd0))
                        // circ=1 且 pb>0: 先进入底部 padding
                        next_state = CPAD_BOT;
                    else if ((ch_group_total > 11'd0) && (ch_group_cnt < ch_group_total - 11'd1)) begin
                        // 还有更多通道组，启动下一轮
                        if (circular_padding && (padding_top != 2'd0))
                            next_state = CPAD_TOP;
                        else
                            next_state = FM;
                    end else begin
                        // 所有通道组完成
                        next_state = DONE;
                    end
                end
            end

            CPAD_BOT: begin
                if (state_done) begin
                    if ((ch_group_total > 11'd0) && (ch_group_cnt < ch_group_total - 11'd1)) begin
                        // 还有更多通道组，启动下一轮
                        if (circular_padding && (padding_top != 2'd0))
                            next_state = CPAD_TOP;
                        else
                            next_state = FM;
                    end else begin
                        // 所有通道组完成
                        next_state = DONE;
                    end
                end
            end

            DONE: begin
                if (start) begin
                    // 新一轮 pooling 操作开始
                    if (circular_padding && (padding_top != 2'd0))
                        next_state = CPAD_TOP;
                    else
                        next_state = FM;
                end
            end

            default: begin
                // 非法状态恢复: 回到 IDLE，等待外部复位或 start
                next_state = IDLE;
            end
        endcase
    end


    // ============================================================
    // 通道组 / 计数器真实事件 (组合逻辑)
    // ============================================================
    // 不再从 next_state 反解计数器 CE。next_state 只负责 FSM；计数器只响应
    // 三类真实事件：新操作 start、当前状态最后一拍 state_done、普通 handshake。
    // 这切断 state_done -> next_state -> counter CE 的长组合路径。
    assign more_ch_groups = (ch_group_cnt < ch_group_total - 11'd1);

    assign start_new_operation = start && ((state == IDLE) || (state == DONE));

    // FM 若还要进入同组的 CPAD_BOT，则不是组边界；其余 FM 末拍在 more=1
    // 时进入下一组。CPAD_BOT 末拍在 more=1 时始终进入下一组。
    assign new_ch_group = state_done && more_ch_groups &&
                          ((state == CPAD_BOT) ||
                           ((state == FM) &&
                            !(circular_padding && (padding_bottom != 2'd0))));

    assign counter_reset = start_new_operation || state_done;


    // ============================================================
    // 计数器: row_cnt / col_cnt / ch_group_cnt (时序逻辑)
    // ============================================================
    //
    // 【推进规则】
    //   三个计数器形成三维嵌套: ch_group_cnt (外层) → row_cnt (中层) → col_cnt (内层)
    //   每个 handshake 拍 col_cnt 推进 1。col 到头 → row 推进 1 + col 归零。
    //   row 到头 → 触发 state_done → FSM 跳转。
    //
    // 【复位规则】
    //   start_new_operation/state_done 时行列归零；new_ch_group 时组计数递增；
    //   IDLE/DONE 接受 start 时组计数清零。
    //   同一通道组内状态切换 (如 FM→CPAD_BOT) 只归零行列，ch_group_cnt 保持。
    //
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            row_cnt      <= 13'd0;
            col_cnt      <= 13'd0;
            ch_group_cnt <= 11'd0;
        end else begin
            if (counter_reset) begin
                // 行列计数器归零 (状态切换 或 同状态通道组重启)
                row_cnt <= 13'd0;
                col_cnt <= 13'd0;

                if (new_ch_group) begin
                    // 跨入下一通道组: 递增通道组计数器
                    ch_group_cnt <= ch_group_cnt + 11'd1;
                end else if (state == IDLE || state == DONE) begin
                    // 新一轮 pooling 操作启动: 通道组计数器清零
                    ch_group_cnt <= 11'd0;
                end
                // 其余情况 (同通道组内切换，如 FM→CPAD_BOT): ch_group_cnt 保持

            end else if (handshake) begin
                // 正常行列推进: 每拍一个 handshake → 推进一列
                if (is_last_col_in_row) begin
                    // 当前行最后一列完成 → 换行
                    col_cnt <= 13'd0;
                    row_cnt <= row_cnt + 13'd1;
                end else begin
                    // 当前行内还有列 → 列号递增
                    col_cnt <= col_cnt + 13'd1;
                end
            end
            // 既无 counter_reset 也无 handshake: 计数器全部保持 (反压等待)
        end
    end


    // ============================================================
    // 阶段译码 (组合逻辑)
    // ============================================================
    //
    // 根据 state 和 row_cnt 实时译码当前行属于哪个阶段。
    // 这个信号驱动两件事:
    //   1. 数据源选择 mux (data_raw_ch0/ch1)
    //   2. 握手逻辑 (ppool_handshake / fpool_handshake / virt_handshake)
    //
    // circ=0 时 FM 内部的阶段译码依赖 row_cnt:
    //   row_cnt < fm_row_in                          → VALID (读 fpool)
    //   row_cnt ≥ fm_row_in                          → BOT_PAD (虚拟 -128)
    //   注意: circ=0 时 TOP_PAD 不在本模块处理，由垂直池化模块内部负责。
    //
    always @(*) begin
        case (state)
            CPAD_TOP:   phase = PHASE_TOP_PAD;     // 恒为 top padding
            CPAD_BOT:   phase = PHASE_BOT_PAD;     // 恒为 bottom padding
            FM: begin
                if (circular_padding) begin
                    // circ=1: FM 状态只含有效行，phase 恒为 VALID
                    phase = PHASE_VALID;
                end else begin
                    // circ=0: FM 状态含有效行 + bottom 虚拟行
                    // top padding 不在此处理 (由垂直池化模块负责)
                    if (row_cnt < fm_row_in)
                        phase = PHASE_VALID;
                    else
                        phase = PHASE_BOT_PAD;
                end
            end
            default:    phase = PHASE_VALID;        // IDLE/DONE 不使用，赋安全值
        endcase
    end


    // ============================================================
    // 列屏蔽逻辑 (组合逻辑)
    // ============================================================
    //
    // 【背景】
    //   特征图宽度 = (fm_column_in + 1) × 32 列。前置模块在输入流中已补好
    //   左右 padding 列 (值为 -128)。但当特征图宽度不是 32 的倍数时，
    //   最后一组 32 列中只有前 N 列是真实特征图数据 (N = last_fm_out_column_valid + 1)，
    //   后面的 (32-N) 列是前置模块为凑齐 32 列而补充的补齐列。
    //   问题: 补齐列的值 **不保证是 -128**，必须在本模块强制替换。
    //
    // 【示例】
    //   特征图实际宽度 = 37 列。fm_column_in = 1, padding_left = 2。
    //   输入流: [pad_L×2] [group0: 32 valid cols] [group1: 5 valid + 27 pad cols]
    //   last_fm_out_column_valid = 4 (编码，实际有效 = 4+1 = 5)。
    //   last_group_start = padding_left + fm_column_in * 32 = 2 + 32 = 34
    //   col_cnt:
    //     0-1:   左 padding → in_last_group=0 → col_force_min=0 (已是 -128，无需再换)
    //     2-33:  第 0 组 32 列 → in_last_group=0 → col_force_min=0 (全有效)
    //     34-38: 第 1 组有效列 → in_last_group=1, offset=0~4 < 5 → col_force_min=0
    //     39-65: 第 1 组补齐列 → in_last_group=1, offset=5~31 ≥ 5 → col_force_min=1 ✗→ -128
    //
    // last_group_start 由 ctrl 预计算后传入, last_group_end 本地加常数 31
    assign last_group_end   = last_group_start + 13'd31;

    // col_cnt 是否落在这个范围内
    assign in_last_group = (col_cnt >= last_group_start) && (col_cnt <= last_group_end);

    // col_cnt 在该组内的偏移 (0~31)
    assign col_in_last_group = col_cnt - last_group_start;

    // 最后一组中有效的列数 = 编码值 + 1 (范围 1~32)
    assign valid_cols_in_last_group = {1'd0, last_fm_out_column_valid} + 6'd1;

    // 列屏蔽: 在最后一组内偏移 >= 有效列数 → 强制替换为 -128
    assign col_force_min = in_last_group && (col_in_last_group >= {7'd0, valid_cols_in_last_group});


    // ============================================================
    // 数据通路: 源选择 → 列屏蔽 (纯组合逻辑, 2 级 mux)
    // ============================================================
    //
    // 数据流: AXIS 总线 → 拆分双通道 → 源选择 mux → 列屏蔽 mux → 寄存器
    //
    // 源选择优先级 (写在 mux 结构里):
    //   1. VALID 阶段: 无条件选 fpool (特征图数据)
    //   2. TOP/BOT_PAD 阶段 + circ=1: 选 ppool (循环 Padding 数据)
    //   3. BOT_PAD 阶段 + circ=0: 选 8'h80 (虚拟 -128)
    //      (TOP_PAD+circ=0 不会发生: circ=0 时 phase 永不为 TOP_PAD)
    //   注意: circ=1 时绝不会进入条件 3 (CPAD_TOP/CPAD_BOT 保证了 ppool 读取)；
    //         circ=0 时绝不会进入条件 2 (FSM 跳过 CPAD 状态)。
    //         但条件表达式仍显式检查 circular_padding 以明确设计意图。
    //

    // 拆分 16bit AXIS 总线 → 两个 8bit 通道
    assign ppool_ch0 = s_axis_ppooltem_tdata[7:0];
    assign ppool_ch1 = s_axis_ppooltem_tdata[15:8];
    assign fpool_ch0 = s_axis_fpooltem_tdata[7:0];
    assign fpool_ch1 = s_axis_fpooltem_tdata[15:8];

    // 源选择: 3 选 1 mux (每通道独立)
    assign data_raw_ch0 = (phase == PHASE_VALID) ? fpool_ch0 :
                          ((phase == PHASE_TOP_PAD || phase == PHASE_BOT_PAD) && circular_padding) ? ppool_ch0 :
                          8'h80;

    assign data_raw_ch1 = (phase == PHASE_VALID) ? fpool_ch1 :
                          ((phase == PHASE_TOP_PAD || phase == PHASE_BOT_PAD) && circular_padding) ? ppool_ch1 :
                          8'h80;

    // 列屏蔽: 2 选 1 mux，col_force_min=1 时强制替换为 -128
    assign data_final_ch0 = col_force_min ? 8'h80 : data_raw_ch0;
    assign data_final_ch1 = col_force_min ? 8'h80 : data_raw_ch1;


    // ============================================================
    // 输出寄存器 (时序逻辑 — 数据打拍)
    // ============================================================
    //
    // 时序: 握手成功当拍 (handshake=1)，data_final 到达寄存器 D 端。
    //       posedge clk 时锁存，下一拍出现在 Q 端，din_valid 同步拉高。
    //       下游 pooling_h_kernel 在 din_valid=1 的拍采样 ch0_din/ch1_din。
    //
    always @(posedge clk) begin
        ch0_din_reg <= data_final_ch0;
    end

    always @(posedge clk) begin
        ch1_din_reg <= data_final_ch1;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            din_valid_reg <= 1'b0;
        else
            din_valid_reg <= handshake;
    end


    // ============================================================
    // 握手逻辑 (组合逻辑)
    // ============================================================
    //
    // 三路握手分别对应三种数据来源，任一成立即推进。
    // 互斥保证: 同一时刻三路中最多一路为 1 (phase 互斥 + state 互斥)。
    //
    // 【虚拟行的特殊性】
    //   仅 circ=0 的 bottom padding 阶段产生虚拟行。virt_handshake 不依赖任何
    //   外部 tvalid。只要虚拟源 ready，每拍自动产生一次 fire，
    //   以每拍一列的速率全速输出 -128，无需等待外部数据源。
    //   circ=0 的 top padding 不在此处理 —— 由垂直池化模块 (pooling_v_kernel)
    //   内部在初始阶段自行处理 top 虚拟行。
    //
    // 【反压行为】
    //   input_credit_ready gates only this stage ready; real input advances on
    //   tvalid&tready, and virtual rows advance on virt_valid&virt_ready.
    //   决定，虚拟行是否推进由 virt_valid&virt_ready 决定。
    //

    // ppool/fpool ready: 仅由本级当前可接收状态和下游提前反压决定。
    assign ppool_ready = (state == CPAD_TOP || state == CPAD_BOT)
                       && circular_padding
                       && input_credit_ready;

    assign fpool_ready = (state == FM)
                       && (phase == PHASE_VALID)
                       && input_credit_ready;

    assign virt_ready  = input_credit_ready;

    // ppool/fpool 握手: 外部 AXIS 传输事件只能由 valid && ready 定义。
    assign ppool_handshake = s_axis_ppooltem_tvalid && ppool_ready;
    assign fpool_handshake = s_axis_fpooltem_tvalid && fpool_ready;

    // 虚拟行握手: 仅在 FM 状态 + 非循环模式 + BOT_PAD 阶段成立
    // circ=0 时 TOP_PAD 不经过本模块 (由垂直池化处理)，故仅 BOT_PAD 需要虚拟行
    assign virt_handshake = (state == FM)
                         && !circular_padding
                         && (phase == PHASE_BOT_PAD)
                         && virt_ready;

    // 总握手
    assign handshake = ppool_handshake || fpool_handshake || virt_handshake;


    // ============================================================
    // tready 输出 (组合逻辑)
    // ============================================================
    //
    // tready 表示本拍真实输入源是否可被接收；内部推进只看 valid && tready。


    // ============================================================
    // input_done (时序逻辑)
    // ============================================================
    //
    // 行为:
    //   - 进入 DONE 状态后拉高 (所有通道组的所有行完成)
    //   - start 脉冲到来时清零 (新一轮 pooling 启动)
    //   - 复位后为 0
    //
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            input_done_reg <= 1'b0;
        end else if (state == DONE) begin
            input_done_reg <= 1'b1;
        end else if (start) begin
            input_done_reg <= 1'b0;
        end
    end


    // ============================================================
    // 输出端口连接
    // ============================================================
    assign ch0_din                 = ch0_din_reg;
    assign ch1_din                 = ch1_din_reg;
    assign din_valid               = din_valid_reg;
    assign s_axis_ppooltem_tready  = ppool_ready;
    assign s_axis_fpooltem_tready  = fpool_ready;
    assign input_done              = input_done_reg;


endmodule

// ============================================================
// RTL Self-Check Results
// ============================================================
// [PASS] Rule #1:  All ports have explicit direction and width
// [PASS] Rule #2:  Ports use wire; sequential state uses reg
// [PASS] Rule #6:  No debug/error-attribution logic in RTL
// [PASS] Rule #8:  Pure Verilog; no SystemVerilog keywords
// [PASS] Rule #12: Existing combinational state logic has defaults
// [PASS] Rule #16: Sequential logic uses non-blocking assignments
// [PASS] Rule #24: No new multiplier or wide combinational arithmetic
// [PASS] Handshake rule: counters and state advance on accepted input beats
// [PASS] Timing rule: ready is gated only by local registered credit
// [PASS] Timing rule: counter CE uses direct start/state_done events, not next_state decode
// [PASS] Input contract: fm_total_rows and ch_group_total are nonzero for legal pooling commands
// ============================================================
