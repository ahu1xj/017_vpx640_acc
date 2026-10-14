`include "all_defines.vh"

module store (
    // ==================== 系统接口 ====================
    input  wire                         clk,
    input  wire                         rst_n,

// Standard valid/ready stream boundary; backpressure terminates in local elastic storage.
    input  wire                         s_axis_wrcmd_tvalid,
    input  wire [`WR_SM_CMD_WIDTH-1:0]  s_axis_wrcmd_tdata,
    output wire                         s_axis_wrcmd_tready,

    // ==================== DMA二级写命令输出端口 ====================
    output wire                         m_axis_wrcmd2_tvalid,
    output wire [`CMD2_CMD_WIDTH-1:0]   m_axis_wrcmd2_tdata,
    input  wire                         m_axis_wrcmd2_tready,

    // ==================== 向DDR写数据输出端口 ====================
    output wire                         m_axis_wrddr_tvalid,
    output wire [511:0]                 m_axis_wrddr_tdata,
    input  wire                         m_axis_wrddr_tready,

    // ==================== 循环padding数据缓存输出端口 ====================
    output wire                         circular_fifo_tvalid,
    output wire [511:0]                 circular_fifo_tdata,
    input  wire                         circular_fifo_tready,

    // ==================== 循环padding写指令控制字段输出端口 ====================
    output wire                         m_axis_wrcmd_tvalid,
    output wire [103:0]                 m_axis_wrcmd_tdata,
    input  wire                         m_axis_wrcmd_tready,

    // ==================== 来自PS的数据写入端口 ====================
    input  wire                         ps_fifo_tvalid,
    input  wire [511:0]                 ps_fifo_tdata,
    output wire                         ps_fifo_tready,

// Standard valid/ready stream boundary; backpressure terminates in local elastic storage.
    input  wire                         act_fifo_tvalid,
    input  wire [511:0]                 act_fifo_tdata,
    output wire                         act_fifo_tready,

    // ==================== 来自池化FIFO的数据 ====================
    input  wire                         pooling_fifo_tvalid,
    input  wire [511:0]                 pooling_fifo_tdata,
    output wire                         pooling_fifo_tready,

    // ==================== 来自残差FIFO的数据 ====================
    input  wire                         res_fifo_tvalid,
    input  wire [511:0]                 res_fifo_tdata,
    output wire                         res_fifo_tready,

    // ==================== 特征图全部写回指示 ====================
    output reg                          store_done_req,
    input  wire                         store_done_ack,
    output reg                          store_fm_done,

    // ==================== 写特征图基地址端口 ====================
    output wire                         store_ba_rden,          
    output wire [10:0]                  store_ba_rdaddr,       
    input  wire [31:0]                  store_ba_rddata        
);

// =========================================================
// 内部信号定义
// =========================================================
wire wrcmd_handshake;
wire ps_en;
wire act_en;
wire res_en;
wire pooling_en;

wire transfer_done;
wire ps_handshake;
wire act_handshake;
wire res_handshake;
wire pooling_handshake;
wire data_in_handshake;
wire row_wrddr_done;
wire col_wrddr_done;
wire ch_wrddr_done;
wire is_start3row;
wire is_end3row;
wire act_done;
wire act_ch_done;
wire store_is_write_fm;
wire cmd2_work_state;
wire cmd2_issue_en;
wire wrcmd_forward_in;
wire wrcmd_forward_raw_valid;
wire wrcmd_forward_raw_ready;
wire [103:0] wrcmd_forward_raw_data;
wire wrddr_raw_valid;
wire wrddr_raw_ready;
wire [511:0] wrddr_raw_data;
wire circular_raw_valid;
wire circular_raw_ready;
wire [511:0] circular_raw_data;
wire selected_input_valid;
wire [511:0] selected_input_data;
wire write_data_needed;
wire circular_write_needed;
wire output_paths_ready;
wire write_data_fire;
wire cmd2_output_fire;
wire cmd2_credit_available;

wire cmd2_ch_done;
wire cmd2_col_done;
wire cmd2_row_done;
wire [`WR_FM_CHANNEL_END_WIDTH-1:0] ch_num_type1;
wire [`WR_FM_CHANNEL_END_WIDTH-1:0] ch_num;
wire [1:0] ch_add;

reg [`WR_SM_CMD_WIDTH-1:0] wr_sm_command;
reg [2:0] curr_state;
reg [2:0] next_state;
reg [`WR_FM_ROW_END_WIDTH-1:0] row_cnt_wrddr;
reg [`WR_FM_COLUMN_END_WIDTH-1:0] col_cnt_wrddr;
reg [`WR_FM_CHANNEL_END_WIDTH-1:0] ch_cnt_wrddr;
reg [`WR_FM_CHANNEL_END_WIDTH-1:0] act_ch_cnt_in;
reg is_ps_write_fm;
reg is_ps_write_other;
reg is_type1;
reg is_type2;
reg is_type3;
reg [`WR_WR_LEN_WIDTH - 1 : 0] transfer_cnt;
reg [`WR_FM_ROW_END_WIDTH-1:0] row_end;
reg [`WR_FM_ROW_END_WIDTH-1:0] row_cnt_wrcmd2;
reg [`WR_FM_COLUMN_END_WIDTH-1:0] col_cnt_wrcmd2;
reg [`WR_FM_CHANNEL_END_WIDTH-1:0] ch_cnt_wrcmd2;
reg [`WR_FM_ROW_END_WIDTH-1:0] H;
reg [`WR_FM_COLUMN_END_WIDTH-1:0] W;
reg [`WR_FM_CHANNEL_END_WIDTH-1:0] C;
reg [`WR_FM_CHANNEL_END_WIDTH-1:0] c_end;
reg [`WR_FM_CHANNEL_END_WIDTH-1:0] c_start;
reg [`WR_FM_CHANNEL_END_WIDTH-1:0] c_num;
reg [`WR_FM_OUT_TYPE_WIDTH-1:0]    cmd2_out_type;
reg [`WR_FM_OUT_TYPE_WIDTH-1:0]    cmd2_out_type1;
reg [`WR_FM_OUT_TYPE_WIDTH-1:0]    cmd2_out_type2;
reg [`WR_FM_OUT_TYPE_WIDTH-1:0]    cmd2_out_type3;
reg [31:0] row_offset_unit;
reg [31:0] col_offset_unit;
reg [31:0] row_offset;
reg [31:0] col_offset;
reg [31:0] ch_offset;
reg [31:0] base_address;
reg [2:0] cmd2_curr_state;
reg [2:0] cmd2_next_state;
reg cmd2_valid1;
reg cmd2_valid2;
reg cmd2_valid3;
reg [31:0] addr_item1;
reg [31:0] addr_item2;
reg [31:0] data_len1;
reg [31:0] data_len2;
reg [3:0] cmd2_credit_count;
(* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg store_done_ack_d1;
(* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg store_done_ack_d2;
reg  wr_command_gen_done;

reg cmd2type1_d1;
reg cmd2type2_d1;
reg cmd2type3_d1;
reg cmd2typeother_d1;

reg cmd2type1_d2;
reg cmd2type2_d2;
reg cmd2type3_d2;
reg cmd2typeother_d2;

reg wait_cut;

localparam [2:0] STORE_IDLE             = 3'd0;
localparam [2:0] STORE_INIT             = 3'd1;
localparam [2:0] STORE_PS               = 3'd2;
localparam [2:0] STORE_ACT              = 3'd3;
localparam [2:0] STORE_POOL             = 3'd4;
localparam [2:0] STORE_RES              = 3'd5;

localparam [2:0] STORE_CMD2_IDLE        = 3'd0;
localparam [2:0] STORE_CMD2_GET_BASE_ADDR = 3'd1;
localparam [2:0] STORE_CMD2_BA_WAIT     = 3'd7;
localparam [2:0] STORE_CMD2_INIT        = 3'd2;
localparam [2:0] STORE_CMD2_WORK_TYPE1  = 3'd3;
localparam [2:0] STORE_CMD2_WORK_TYPE2  = 3'd4;
localparam [2:0] STORE_CMD2_WORK_TYPE3  = 3'd5;
localparam [2:0] STORE_CMD2_WORK_OTHER  = 3'd6;

// =========================================================
// 写指令译码信号
// =========================================================

wire [`WR_RW_FLAG_WIDTH - 1 : 0]            rw_flag;
wire [`WR_DATA_ORIGIN_WIDTH - 1 : 0]        data_origin;
wire [`WR_PS_DATA_TYPE_WIDTH - 1 : 0]       ps_data_type;
wire [`WR_WR_LEN_WIDTH - 1 : 0]             wr_len;
wire [`WR_BASE_ADDR_WIDTH - 1 : 0]          base_addr_ptr;
wire [`WR_FM_CHANNEL_END_WIDTH - 1 : 0]     fm_channel_end;
wire [`WR_FM_COLUMN_END_WIDTH - 1 : 0]      fm_column_end;
wire [`WR_FM_ROW_END_WIDTH - 1 : 0]         fm_row_end;
wire [`WR_FM_CHANNEL_START_WIDTH - 1 : 0]   fm_channel_start;
wire [`WR_FM_CHANNEL_TOTAL_WIDTH - 1 : 0]   fm_channel_total;
wire [`WR_FM_END_FLAG_WIDTH - 1 : 0]        fm_end_flag;
wire [`WR_FM_OUT_TYPE_WIDTH - 1 : 0]        fm_out_type;
wire [`WR_OTHER_INIT_DONE_WIDTH - 1 : 0]    other_init_done;
wire [`WR_KERNEL_C_IN_END_WIDTH - 1 : 0]    kernel_c_in_end;
wire [`WR_KERNEL_R_SIZE_WIDTH - 1 : 0]      kernel_size_r;
wire [`WR_KERNEL_C_SIZE_WIDTH - 1 : 0]      kernel_size_c;
wire [`WR_KERNEL_C_OUT_END_WIDTH - 1 : 0]   kernel_c_out_end;
wire [`WR_DATA_ORIGIN_WIDTH - 1 : 0]        s_axis_data_origin;
wire [`WR_PS_DATA_TYPE_WIDTH - 1 : 0]       s_axis_ps_data_type;
wire [`WR_FM_OUT_TYPE_WIDTH - 1 : 0]        s_axis_fm_out_type;

// =========================================================
// 写二级命令信息
// =========================================================

reg [`CMD2_ADDR_START_WIDTH-1:0] cmd2_addr_start;
reg [`CMD2_TRANS_LEN_WIDTH -1:0] cmd2_trans_len;

// =========================================================
// 写指令译码
// =========================================================

assign rw_flag         = wr_sm_command[`WR_RW_FLAG_START         +:`WR_RW_FLAG_WIDTH];
assign data_origin     = wr_sm_command[`WR_DATA_ORIGIN_START     +:`WR_DATA_ORIGIN_WIDTH];
assign ps_data_type    = wr_sm_command[`WR_PS_DATA_TYPE_START    +:`WR_PS_DATA_TYPE_WIDTH];
assign wr_len          = wr_sm_command[`WR_WR_LEN_START          +:`WR_WR_LEN_WIDTH];
assign base_addr_ptr   = wr_sm_command[`WR_BASE_ADDR_START       +:`WR_BASE_ADDR_WIDTH];

assign fm_channel_end  = wr_sm_command[`WR_FM_CHANNEL_END_START  +:`WR_FM_CHANNEL_END_WIDTH];
assign fm_column_end   = wr_sm_command[`WR_FM_COLUMN_END_START   +:`WR_FM_COLUMN_END_WIDTH];
assign fm_row_end      = wr_sm_command[`WR_FM_ROW_END_START      +:`WR_FM_ROW_END_WIDTH];
assign fm_channel_start= wr_sm_command[`WR_FM_CHANNEL_START_START+:`WR_FM_CHANNEL_START_WIDTH];
assign fm_channel_total= wr_sm_command[`WR_FM_CHANNEL_TOTAL_START+:`WR_FM_CHANNEL_TOTAL_WIDTH];
assign fm_out_type     = wr_sm_command[`WR_FM_OUT_TYPE_START     +:`WR_FM_OUT_TYPE_WIDTH];
assign fm_end_flag     = wr_sm_command[`WR_FM_END_FLAG_START     +:`WR_FM_END_FLAG_WIDTH];

assign other_init_done = wr_sm_command[`WR_OTHER_INIT_DONE_START +:`WR_OTHER_INIT_DONE_WIDTH];
assign kernel_c_in_end = wr_sm_command[`WR_KERNEL_C_IN_END_START +:`WR_KERNEL_C_IN_END_WIDTH];
assign kernel_size_r   = wr_sm_command[`WR_KERNEL_R_SIZE_START   +:`WR_KERNEL_R_SIZE_WIDTH];
assign kernel_size_c   = wr_sm_command[`WR_KERNEL_C_SIZE_START   +:`WR_KERNEL_C_SIZE_WIDTH];
assign kernel_c_out_end= wr_sm_command[`WR_KERNEL_C_OUT_END_START+:`WR_KERNEL_C_OUT_END_WIDTH];

assign s_axis_data_origin  = s_axis_wrcmd_tdata[`WR_DATA_ORIGIN_START +:`WR_DATA_ORIGIN_WIDTH];
assign s_axis_ps_data_type = s_axis_wrcmd_tdata[`WR_PS_DATA_TYPE_START+:`WR_PS_DATA_TYPE_WIDTH];
assign s_axis_fm_out_type  = s_axis_wrcmd_tdata[`WR_FM_OUT_TYPE_START  +:`WR_FM_OUT_TYPE_WIDTH];
// 转发给循环padding缓存链路的写指令：所有特征图写都缓存，
// 只排除搬模型参数（PS 非 FM）和 int16 输出（不参与下一层计算）
assign wrcmd_forward_in    = ((s_axis_data_origin == `DATA_ORIGIN_ACTIVE) |
                             (s_axis_data_origin == `DATA_ORIGIN_RES) |
                             (s_axis_data_origin == `DATA_ORIGIN_POOLING) |
                             ((s_axis_data_origin == `DATA_ORIGIN_PS) &
                              (s_axis_ps_data_type == `PS_DATA_TYPE_FM))) &
                             (s_axis_fm_out_type == 1'b0);

// =========================================================
// 获取指令
// =========================================================
assign wrcmd_handshake          = s_axis_wrcmd_tvalid & s_axis_wrcmd_tready;
assign wrcmd_forward_raw_valid = wrcmd_handshake & wrcmd_forward_in;
assign wrcmd_forward_raw_data  = {5'd0, s_axis_wrcmd_tdata[98:0]};

always@(posedge clk,negedge rst_n)begin
    if(!rst_n) wr_sm_command <= 0;
    else begin
        if(wrcmd_handshake) wr_sm_command <= s_axis_wrcmd_tdata;
    end
end

// =========================================================
// 状态机跳转使能
// =========================================================
assign ps_en = (data_origin == `DATA_ORIGIN_PS);
assign act_en = (data_origin == `DATA_ORIGIN_ACTIVE);
assign res_en = (data_origin == `DATA_ORIGIN_RES);
assign pooling_en = (data_origin == `DATA_ORIGIN_POOLING);

// =========================================================
// 状态机跳转
// =========================================================

always@(posedge clk,negedge rst_n)begin
    if(!rst_n) curr_state <= STORE_IDLE;
    else curr_state <= next_state;
end

always@(*)begin
    case(curr_state)
        STORE_IDLE:begin
            if(wrcmd_handshake) next_state = STORE_INIT;
            else next_state = curr_state;
        end

        STORE_INIT:begin
            case({pooling_en,res_en,act_en,ps_en})
                4'b0001: next_state = STORE_PS;
                4'b0010: next_state = STORE_ACT;
                4'b0100: next_state = STORE_RES;
                4'b1000: next_state = STORE_POOL;
                default: next_state = curr_state;
            endcase
        end

        STORE_PS:begin
            if(transfer_done) next_state = STORE_IDLE;
            else next_state = curr_state;
        end

        STORE_ACT:begin
            if(act_done) next_state = STORE_IDLE;
            else next_state = curr_state;
        end

        STORE_RES:begin
            if(transfer_done) next_state = STORE_IDLE;
            else next_state = curr_state;
        end

        STORE_POOL:begin
            if(transfer_done) next_state = STORE_IDLE;
            else next_state = curr_state;
        end

        default: next_state = curr_state;
    endcase
end

// =========================================================
// 传输计数器
// =========================================================
assign ps_handshake         = (ps_fifo_tvalid & ps_fifo_tready);
assign act_handshake        = (act_fifo_tvalid & act_fifo_tready);
assign res_handshake        = (res_fifo_tvalid & res_fifo_tready);
assign pooling_handshake    = (pooling_fifo_tvalid & pooling_fifo_tready);
assign data_in_handshake    = (ps_handshake | act_handshake | res_handshake | pooling_handshake);
assign transfer_done        = (transfer_cnt == wr_len - 1) & data_in_handshake;


always@(posedge clk,negedge rst_n)begin
    if(!rst_n) transfer_cnt <= 0;
    else begin
        if(is_type1)begin
            if(data_in_handshake & act_ch_cnt_in <= fm_channel_end[5:0]) transfer_cnt <= transfer_done ? 0 : (transfer_cnt + 1);
        end
        else begin
            if(data_in_handshake) transfer_cnt <= transfer_done ? 0 : (transfer_cnt + 1);
        end
    end
end

// =========================================================
// 卷积激活的通道单独的标志
// =========================================================

assign act_done             = (fm_channel_end[5:0] == 63) ? transfer_done : (wait_cut & act_ch_done);
// act_ch_done 必须带输入握手限定: 计数器只在握手拍推进 (line 365),
// 裸比计数器会让"到值"状态跨命令/跨组保持 (stale=1), partial 组的
// act_done = wait_cut & act_ch_done 会在最后尾拍 (填充槽位) 尚未到达时
// 提前退出 STORE_ACT, 滞留拍被下一条写指令当成首拍写出 (tc55 实证:
// act_done 于 act_fifo_tvalid=0 时拉高, 下一拍尾拍才到)。
assign act_ch_done = (fm_out_type ? (act_ch_cnt_in == 63) : (act_ch_cnt_in == 62)) & act_handshake;

always@(posedge clk,negedge rst_n)begin
    if(!rst_n) wait_cut <= 0;
    else begin
        if(wait_cut) begin
            if(act_ch_done) wait_cut <= 0;
        end
        else begin
            if(is_type1 & (fm_channel_end[5:0] != 63) & transfer_done) wait_cut <= 1;
        end
    end
end

always@(posedge clk,negedge rst_n)begin
    if(!rst_n) act_ch_cnt_in <= 0;
    else begin
        if(act_fifo_tvalid & act_fifo_tready) act_ch_cnt_in <= act_ch_done ? 0 : (act_ch_cnt_in + ch_add);
    end
end


// =========================================================
// 输入控制
// =========================================================
assign selected_input_valid = (curr_state == STORE_PS)   ? ps_fifo_tvalid :
                              (curr_state == STORE_ACT)  ? act_fifo_tvalid :
                              (curr_state == STORE_RES)  ? res_fifo_tvalid :
                              (curr_state == STORE_POOL) ? pooling_fifo_tvalid : 1'b0;
assign selected_input_data = (curr_state == STORE_PS)   ? ps_fifo_tdata :
                             (curr_state == STORE_ACT)  ? act_fifo_tdata :
                             (curr_state == STORE_RES)  ? res_fifo_tdata :
                             (curr_state == STORE_POOL) ? pooling_fifo_tdata : 512'd0;
assign write_data_needed = (!is_type1) || (act_ch_cnt_in <= fm_channel_end[5:0]);
assign circular_write_needed = write_data_needed && (is_start3row || is_end3row) &&
                               (!is_ps_write_other) && (!fm_out_type);
assign output_paths_ready = (!write_data_needed) ||
                            (wrddr_raw_ready && (!circular_write_needed || circular_raw_ready));

assign ps_fifo_tready      = (curr_state == STORE_PS)   && output_paths_ready;
assign act_fifo_tready     = (curr_state == STORE_ACT)  && output_paths_ready;
assign res_fifo_tready     = (curr_state == STORE_RES)  && output_paths_ready;
assign pooling_fifo_tready = (curr_state == STORE_POOL) && output_paths_ready;
assign s_axis_wrcmd_tready = (curr_state == STORE_IDLE) &&
                             (cmd2_curr_state == STORE_CMD2_IDLE) &&
                             (!s_axis_wrcmd_tvalid || !wrcmd_forward_in ||
                              wrcmd_forward_raw_ready);

assign wrddr_raw_valid    = data_in_handshake && write_data_needed;
assign wrddr_raw_data     = selected_input_data;
assign circular_raw_valid = data_in_handshake && circular_write_needed;
assign circular_raw_data  = selected_input_data;
assign write_data_fire    = wrddr_raw_valid && wrddr_raw_ready;

always@(posedge clk,negedge rst_n)begin
    if(!rst_n) is_ps_write_fm <= 0;
    else if(curr_state == STORE_INIT)begin
        is_ps_write_fm <=  ((ps_data_type == `PS_DATA_TYPE_FM) & (data_origin == `DATA_ORIGIN_PS));
    end
    else if(curr_state == STORE_IDLE)begin
        is_ps_write_fm <= 0;
    end
end

always@(posedge clk,negedge rst_n)begin
    if(!rst_n) is_ps_write_other <= 0;
    else if(curr_state == STORE_INIT)begin
        is_ps_write_other <=  ((ps_data_type != `PS_DATA_TYPE_FM) & (data_origin == `DATA_ORIGIN_PS));
    end
    else if(curr_state == STORE_IDLE)begin
        is_ps_write_other <= 0;
    end
end

always@(posedge clk,negedge rst_n)begin
    if(!rst_n) is_type1 <= 0;
    else if(curr_state == STORE_INIT)begin
        is_type1 <= (data_origin == `DATA_ORIGIN_ACTIVE);
    end
    else if(curr_state == STORE_IDLE)begin
        is_type1 <= 0;
    end
end

always@(posedge clk,negedge rst_n)begin
    if(!rst_n) is_type2 <= 0;
    else if(curr_state == STORE_INIT)begin
        is_type2 <= (data_origin == `DATA_ORIGIN_POOLING);
    end
    else if(curr_state == STORE_IDLE)begin
        is_type2 <= 0;
    end
end

always@(posedge clk,negedge rst_n)begin
    if(!rst_n) is_type3 <= 0;
    else if(curr_state == STORE_INIT)begin
        is_type3 <= (data_origin == `DATA_ORIGIN_RES);
    end
    else if(curr_state == STORE_IDLE)begin
        is_type3 <= 0;
    end
end

// =========================================================
// 写入DDR数据的行列计数器
// =========================================================
assign ch_wrddr_done  = (fm_out_type?(ch_cnt_wrddr == fm_channel_end):(ch_cnt_wrddr[1+:`WR_FM_CHANNEL_END_WIDTH-1] == fm_channel_end[1+:`WR_FM_CHANNEL_END_WIDTH-1])) & write_data_fire;
assign col_wrddr_done = (col_cnt_wrddr == fm_column_end) & write_data_fire;
assign row_wrddr_done = (row_cnt_wrddr == fm_row_end-1) & write_data_fire;
assign ch_add = fm_out_type ? 1 : 2;
assign ch_num = fm_channel_end[5:0] + 1;


always@(posedge clk,negedge rst_n)begin
    if(!rst_n) ch_cnt_wrddr <= 0;
    else begin
        if(curr_state == STORE_INIT)begin
            ch_cnt_wrddr <= {fm_channel_start,6'd0};
        end
        else begin
            if(is_type1 | is_type3 | is_ps_write_fm)begin
                if(write_data_fire) ch_cnt_wrddr <= ch_wrddr_done ? {fm_channel_start,6'd0} : (ch_cnt_wrddr + ch_add);
            end
            else if(is_type2)begin
                if(row_wrddr_done & col_wrddr_done) ch_cnt_wrddr <= ch_wrddr_done ? {fm_channel_start,6'd0} : (ch_cnt_wrddr + ch_add);
            end
            else ch_cnt_wrddr <= 0;
        end
    end
end



always@(posedge clk,negedge rst_n)begin
    if(!rst_n) col_cnt_wrddr <= 0;
    else begin
        if(is_type1 | is_type3 | is_ps_write_fm)begin
            if(ch_wrddr_done) col_cnt_wrddr <= col_wrddr_done ? 0 : (col_cnt_wrddr + 1);
        end
        else if(is_type2)begin
            if(write_data_fire) col_cnt_wrddr <= col_wrddr_done ? 0 : (col_cnt_wrddr + 1);
        end
        else col_cnt_wrddr <=0;
    end
end

always@(posedge clk,negedge rst_n)begin
    if(!rst_n) row_cnt_wrddr <= 0;
    else begin
        if(is_type1 | is_type3 | is_ps_write_fm)begin
            if(ch_wrddr_done & col_wrddr_done) row_cnt_wrddr <= row_wrddr_done ? 0 : (row_cnt_wrddr + 1);
        end
        else if(is_type2)begin
            if(col_wrddr_done) row_cnt_wrddr <= row_wrddr_done ? 0 : (row_cnt_wrddr + 1);
        end
        else row_cnt_wrddr <= 0;
    end
end


// =========================================================
// 循环padding缓存输出
// =========================================================
assign is_start3row = (row_cnt_wrddr < 3);
assign is_end3row   = (row_cnt_wrddr >= row_end - 3);
// tap 进循环padding缓存：首尾 3 行；排除搬模型参数与 int16（int16 不做下一层输入）
assign store_is_write_fm = is_type1 | is_type2 | is_type3 | is_ps_write_fm;

always@(posedge clk,negedge rst_n)begin
    if(!rst_n) row_end <= 0;
    else row_end <= fm_row_end;
end


// =========================================================
// 写二级命令状态机
// =========================================================
always@(posedge clk,negedge rst_n)begin
    if(!rst_n) cmd2_curr_state <= STORE_CMD2_IDLE;
    else cmd2_curr_state <= cmd2_next_state;
end

always@(*)begin
    case(cmd2_curr_state)
        STORE_CMD2_IDLE:begin
            if(wrcmd_handshake) cmd2_next_state = STORE_CMD2_GET_BASE_ADDR;
            else cmd2_next_state = cmd2_curr_state;
        end

        // BA RAM 换 BMG IP（DO_REG=1）后读出为 2 拍：GET_BASE_ADDR 发地址，
        // BA_WAIT 等一拍，INIT 拍数据有效并锁存。
        STORE_CMD2_GET_BASE_ADDR: cmd2_next_state = STORE_CMD2_BA_WAIT;

        STORE_CMD2_BA_WAIT: cmd2_next_state = STORE_CMD2_INIT;

        STORE_CMD2_INIT:begin
            case({is_ps_write_other,is_type3 | is_ps_write_fm,is_type2,is_type1})
                4'b0001: cmd2_next_state = STORE_CMD2_WORK_TYPE1;
                4'b0010: cmd2_next_state = STORE_CMD2_WORK_TYPE2;
                4'b0100: cmd2_next_state = STORE_CMD2_WORK_TYPE3;
                4'b1000: cmd2_next_state = STORE_CMD2_WORK_OTHER;
                default: cmd2_next_state = cmd2_curr_state;
            endcase
        end

        STORE_CMD2_WORK_TYPE1:begin
            if(wr_command_gen_done) cmd2_next_state = STORE_CMD2_IDLE;
            else cmd2_next_state = cmd2_curr_state;
        end

        STORE_CMD2_WORK_TYPE2:begin
            if(wr_command_gen_done) cmd2_next_state = STORE_CMD2_IDLE;
            else cmd2_next_state = cmd2_curr_state;
        end

        STORE_CMD2_WORK_TYPE3:begin
            if(cmd2_issue_en) cmd2_next_state = STORE_CMD2_IDLE;
            else cmd2_next_state = cmd2_curr_state;
        end

        STORE_CMD2_WORK_OTHER:begin
            if(cmd2_issue_en) cmd2_next_state = STORE_CMD2_IDLE;
            else cmd2_next_state = cmd2_curr_state;
        end

        default: cmd2_next_state = cmd2_curr_state;
    endcase
end

// =========================================================
// 根据指针取特征图基地址
// =========================================================
assign store_ba_rden   = (cmd2_curr_state == STORE_CMD2_GET_BASE_ADDR);
assign store_ba_rdaddr = base_addr_ptr;

// =========================================================
// 初始化相关参数
// =========================================================
always@(posedge clk,negedge rst_n)begin
    if(!rst_n) base_address <= 0;
    else  begin
        if(cmd2_curr_state == STORE_CMD2_INIT) base_address <= store_ba_rddata;
    end
end

always@(posedge clk,negedge rst_n)begin
    if(!rst_n) W <= 0;
    else  begin
        if(cmd2_curr_state == STORE_CMD2_INIT) W <= fm_column_end;
    end
end

always@(posedge clk,negedge rst_n)begin
    if(!rst_n) H <= 0;
    else  begin
        if(cmd2_curr_state == STORE_CMD2_INIT) H <= fm_row_end;
    end
end

always@(posedge clk,negedge rst_n)begin
    if(!rst_n) C <= 0;
    else  begin
        if(cmd2_curr_state == STORE_CMD2_INIT) C <= fm_channel_total;
    end
end

always@(posedge clk,negedge rst_n)begin
    if(!rst_n) c_end <= 0;
    else  begin
        if(cmd2_curr_state == STORE_CMD2_INIT) c_end <= fm_channel_end;
    end
end

always@(posedge clk,negedge rst_n)begin
    if(!rst_n) c_start <= 0;
    else  begin
        if(cmd2_curr_state == STORE_CMD2_INIT) c_start <= {fm_channel_start,6'b0};
    end
end

always@(posedge clk,negedge rst_n)begin
    if(!rst_n) c_num <= 0;
    else  begin
        if(cmd2_curr_state == STORE_CMD2_INIT) c_num <= ch_num;
    end
end

always@(posedge clk,negedge rst_n)begin
    if(!rst_n) cmd2_out_type <= 0;
    else  begin
        if(cmd2_curr_state == STORE_CMD2_INIT) cmd2_out_type <= fm_out_type;
    end
end



always@(posedge clk,negedge rst_n)begin
    if(!rst_n) col_offset_unit <= 0;
    else  begin
        if(cmd2_curr_state == STORE_CMD2_INIT) col_offset_unit <= fm_out_type ? {fm_channel_total,6'b0} : {fm_channel_total,5'b0};
    end
end

always@(posedge clk,negedge rst_n)begin
    if(!rst_n) row_offset_unit <= 0;
    else  begin
        // 8b x 18b 单 DSP: (col_blk<=128) x (ch_total*32/64 <= 262080)
        if(cmd2_curr_state == STORE_CMD2_INIT) row_offset_unit <= (fm_out_type ? {fm_channel_total,6'b0} : {1'b0,fm_channel_total,5'b0}) * ({1'b0,fm_column_end} + 8'd1);
    end
end



// =========================================================
// 行列计数器
// =========================================================
assign cmd2_ch_done = cmd2_out_type ? (ch_cnt_wrcmd2 == c_end) : (ch_cnt_wrcmd2[1+:`WR_FM_CHANNEL_END_WIDTH-1] == c_end[1+:`WR_FM_CHANNEL_END_WIDTH-1]);
assign cmd2_col_done = (col_cnt_wrcmd2 == W);
assign cmd2_row_done = (row_cnt_wrcmd2 == H-1);
assign ch_num_type1 = (c_end == C - 1) ? c_num : 64;
assign cmd2_work_state = (cmd2_curr_state == STORE_CMD2_WORK_TYPE1) |
                         (cmd2_curr_state == STORE_CMD2_WORK_TYPE2) |
                         (cmd2_curr_state == STORE_CMD2_WORK_TYPE3) |
                         (cmd2_curr_state == STORE_CMD2_WORK_OTHER);
assign cmd2_issue_en = cmd2_work_state && cmd2_credit_available;

always@(*)begin
    if(cmd2_curr_state == STORE_CMD2_WORK_TYPE1)
        wr_command_gen_done = cmd2_col_done & cmd2_row_done & cmd2_issue_en;
    else if(cmd2_curr_state == STORE_CMD2_WORK_TYPE2)
        wr_command_gen_done = cmd2_ch_done & cmd2_col_done & cmd2_row_done & cmd2_issue_en;
    else
        wr_command_gen_done = 1'b0;
end


always@(posedge clk,negedge rst_n)begin
    if(!rst_n) ch_cnt_wrcmd2 <= 0;
    else begin
        if(cmd2_curr_state == STORE_CMD2_INIT)begin
            if(cmd2_next_state == STORE_CMD2_WORK_TYPE1) ch_cnt_wrcmd2 <= {fm_channel_start,6'd0};
            else ch_cnt_wrcmd2 <= 0;
        end
        else if(cmd2_curr_state == STORE_CMD2_WORK_TYPE1) begin
            if(cmd2_issue_en) ch_cnt_wrcmd2 <= {fm_channel_start,6'd0};
        end
        else if(cmd2_curr_state == STORE_CMD2_WORK_TYPE2) begin
            if(cmd2_col_done & cmd2_row_done & cmd2_issue_en) ch_cnt_wrcmd2 <= cmd2_ch_done ? 0 : (ch_cnt_wrcmd2 + 2);
        end
        else ch_cnt_wrcmd2 <= 0;
    end
end

always@(posedge clk,negedge rst_n)begin
    if(!rst_n) col_cnt_wrcmd2 <= 0;
    else begin
        if((cmd2_curr_state == STORE_CMD2_WORK_TYPE1) | (cmd2_curr_state == STORE_CMD2_WORK_TYPE2)) begin
            if(cmd2_issue_en) col_cnt_wrcmd2 <= cmd2_col_done ? 0 : (col_cnt_wrcmd2 + 1);
        end
        else col_cnt_wrcmd2 <= 0;
    end
end

always@(posedge clk,negedge rst_n)begin
    if(!rst_n) row_cnt_wrcmd2 <= 0;
    else begin
        if((cmd2_curr_state == STORE_CMD2_WORK_TYPE1) | (cmd2_curr_state == STORE_CMD2_WORK_TYPE2)) begin
            if(cmd2_col_done & cmd2_issue_en) row_cnt_wrcmd2 <= cmd2_row_done ? 0 : (row_cnt_wrcmd2 + 1);
        end
        else row_cnt_wrcmd2 <= 0;
    end
end


// =========================================================
// 计算行、列、通道偏移
// =========================================================
wire [`WR_FM_COLUMN_END_WIDTH - 1 : 0] w_mid;
wire [`WR_FM_ROW_END_WIDTH - 1 : 0] w_real;
assign w_mid = W + 1;
assign w_real = {w_mid,5'd0};
always@(posedge clk,negedge rst_n)begin
    if(!rst_n) data_len1 <= 0;
    else begin
        if(cmd2_curr_state == STORE_CMD2_WORK_TYPE1) data_len1 <= cmd2_out_type ? {ch_num_type1,6'd0} : {ch_num_type1,5'd0};
        else if(cmd2_curr_state == STORE_CMD2_WORK_TYPE2) data_len1 <= 64;
        else if(cmd2_curr_state == STORE_CMD2_WORK_TYPE3) data_len1 <= w_real * H;
        else if(cmd2_curr_state == STORE_CMD2_WORK_OTHER) data_len1 <= {wr_len,6'd0};
        else data_len1 <= 0;
    end
end

always@(posedge clk,negedge rst_n)begin
    if(!rst_n) cmd2_out_type1 <= 0;
    else cmd2_out_type1 <= cmd2_out_type;
end

always@(posedge clk,negedge rst_n)begin
    if(!rst_n) cmd2type1_d1 <= 0;
    else cmd2type1_d1 <= (cmd2_curr_state == STORE_CMD2_WORK_TYPE1) & cmd2_issue_en;
end

always@(posedge clk,negedge rst_n)begin
    if(!rst_n) cmd2type2_d1 <= 0;
    else cmd2type2_d1 <= (cmd2_curr_state == STORE_CMD2_WORK_TYPE2) & cmd2_issue_en;
end

always@(posedge clk,negedge rst_n)begin
    if(!rst_n) cmd2type3_d1 <= 0;
    else cmd2type3_d1 <= (cmd2_curr_state == STORE_CMD2_WORK_TYPE3) & cmd2_issue_en;
end

always@(posedge clk,negedge rst_n)begin
    if(!rst_n) cmd2typeother_d1 <= 0;
    else cmd2typeother_d1 <= (cmd2_curr_state == STORE_CMD2_WORK_OTHER) & cmd2_issue_en;
end

always@(posedge clk,negedge rst_n)begin
    if(!rst_n) ch_offset <= 0;
    else ch_offset <= cmd2_out_type ? {ch_cnt_wrcmd2,6'd0} : {ch_cnt_wrcmd2,5'd0};
end

always@(posedge clk,negedge rst_n)begin
    if(!rst_n) col_offset <= 0;
    else col_offset <= col_offset_unit[17:0] * {11'd0, col_cnt_wrcmd2};
end

always@(posedge clk,negedge rst_n)begin
    if(!rst_n) row_offset <= 0;
    else row_offset <= row_offset_unit[24:0] * {14'd0, row_cnt_wrcmd2};
end

always@(posedge clk,negedge rst_n)begin
    if(!rst_n) cmd2_valid1 <= 0;
    else cmd2_valid1 <= cmd2_issue_en;
end



// =========================================================
// 两级加法的第一级
// =========================================================

always@(posedge clk,negedge rst_n)begin
    if(!rst_n) addr_item1 <= 0;
    else addr_item1 <= cmd2typeother_d1 ? base_address : (base_address + row_offset);
end

always@(posedge clk,negedge rst_n)begin
    if(!rst_n) addr_item2 <= 0;
    else addr_item2 <= cmd2typeother_d1 ? 0 : (col_offset + ch_offset);
end

always@(posedge clk,negedge rst_n)begin
    if(!rst_n) data_len2 <= 0;
    else begin
        if(cmd2type1_d1) data_len2 <= data_len1;
        else if(cmd2type2_d1) data_len2 <= data_len1;
        else if(cmd2type3_d1) data_len2 <= data_len1[23:0] * C;
        else if(cmd2typeother_d1) data_len2 <= data_len1;
        else data_len2 <= 0;
    end
end

always@(posedge clk,negedge rst_n)begin
    if(!rst_n) cmd2_out_type2 <= 0;
    else cmd2_out_type2 <= cmd2_out_type1;
end

always@(posedge clk,negedge rst_n)begin
    if(!rst_n) cmd2type1_d2 <= 0;
    else cmd2type1_d2 <= cmd2type1_d1;
end

always@(posedge clk,negedge rst_n)begin
    if(!rst_n) cmd2type2_d2 <= 0;
    else cmd2type2_d2 <= cmd2type2_d1;
end

always@(posedge clk,negedge rst_n)begin
    if(!rst_n) cmd2type3_d2 <= 0;
    else cmd2type3_d2 <= cmd2type3_d1;
end

always@(posedge clk,negedge rst_n)begin
    if(!rst_n) cmd2typeother_d2 <= 0;
    else cmd2typeother_d2 <= cmd2typeother_d1;
end

always@(posedge clk,negedge rst_n)begin
    if(!rst_n) cmd2_valid2 <= 0;
    else cmd2_valid2 <= cmd2_valid1;
end


// =========================================================
// 两级加法的第二级
// =========================================================

always@(posedge clk,negedge rst_n)begin
    if(!rst_n) cmd2_addr_start <= 0;
    else cmd2_addr_start <= addr_item1 + addr_item2;
end

always@(posedge clk,negedge rst_n)begin
    if(!rst_n) cmd2_trans_len <= 0;
    else cmd2_trans_len <= data_len2;
end

always@(posedge clk,negedge rst_n)begin
    if(!rst_n) cmd2_valid3 <= 0;
    else cmd2_valid3 <= cmd2_valid2;
end

wire cmd2_raw_valid = cmd2_valid3;
wire [`CMD2_CMD_WIDTH-1:0] cmd2_raw_data = {cmd2_addr_start,cmd2_trans_len};

assign cmd2_output_fire      = m_axis_wrcmd2_tvalid && m_axis_wrcmd2_tready;
assign cmd2_credit_available = (cmd2_credit_count < 4'd8);

always@(posedge clk,negedge rst_n)begin
    if(!rst_n)
        cmd2_credit_count <= 4'd0;
    else begin
        case ({cmd2_issue_en, cmd2_output_fire})
            2'b10: cmd2_credit_count <= cmd2_credit_count + 4'd1;
            2'b01: cmd2_credit_count <= cmd2_credit_count - 4'd1;
            default: cmd2_credit_count <= cmd2_credit_count;
        endcase
    end
end

axis_skid_buffer #(.DATA_WIDTH(104)) u_wrcmd_forward_skid (
    .clk(clk), .rst_n(rst_n),
    .s_axis_tvalid(wrcmd_forward_raw_valid), .s_axis_tready(wrcmd_forward_raw_ready),
    .s_axis_tdata(wrcmd_forward_raw_data),
    .m_axis_tvalid(m_axis_wrcmd_tvalid), .m_axis_tready(m_axis_wrcmd_tready),
    .m_axis_tdata(m_axis_wrcmd_tdata)
);

axis_skid_buffer #(.DATA_WIDTH(512)) u_wrddr_skid (
    .clk(clk), .rst_n(rst_n),
    .s_axis_tvalid(wrddr_raw_valid), .s_axis_tready(wrddr_raw_ready),
    .s_axis_tdata(wrddr_raw_data),
    .m_axis_tvalid(m_axis_wrddr_tvalid), .m_axis_tready(m_axis_wrddr_tready),
    .m_axis_tdata(m_axis_wrddr_tdata)
);

axis_skid_buffer #(.DATA_WIDTH(512)) u_circular_skid (
    .clk(clk), .rst_n(rst_n),
    .s_axis_tvalid(circular_raw_valid), .s_axis_tready(circular_raw_ready),
    .s_axis_tdata(circular_raw_data),
    .m_axis_tvalid(circular_fifo_tvalid), .m_axis_tready(circular_fifo_tready),
    .m_axis_tdata(circular_fifo_tdata)
);

sync_fifo_axis #(.WIDTH(`CMD2_CMD_WIDTH), .DEPTH(8), .PTR_W(3), .CNT_W(4))
u_wrcmd2_elastic_fifo (
    .clk(clk), .rst_n(rst_n),
    .s_axis_tvalid(cmd2_raw_valid), .s_axis_tready(), .s_axis_tdata(cmd2_raw_data),
    .m_axis_tvalid(m_axis_wrcmd2_tvalid), .m_axis_tready(m_axis_wrcmd2_tready),
    .m_axis_tdata(m_axis_wrcmd2_tdata)
);

always@(posedge clk,negedge rst_n)begin
    if(!rst_n) store_done_ack_d1 <= 0;
    else store_done_ack_d1 <= store_done_ack;
end

always@(posedge clk,negedge rst_n)begin
    if(!rst_n) store_done_ack_d2 <= 0;
    else store_done_ack_d2 <= store_done_ack_d1;
end

always@(posedge clk,negedge rst_n)begin
    if(!rst_n) store_done_req <= 0;
    else begin
        // 2026-08-05 语义重构：req 只表示"卷积一组写回完成"（is_type1&act_done）。
        // pool/res/上传等其它 FM 类写完成不再产 req——dispatch 只对卷积等握手，
        // 非卷积指令的层间顺序由 load 侧 fm_done_latch 保证（详见
        // docs/handoff/solution-20260805-v5s-multiexport-dispatch-handshake.md）。
        if(!store_done_req) begin
            if (is_type1 & act_done) store_done_req <= 1;
        end
        else begin
            if(store_done_ack_d2) store_done_req <= 0;
        end
    end
end

always@(posedge clk,negedge rst_n)begin
    if(!rst_n) store_fm_done <= 0;
    // store_fm_done 等“承接责任”完成：is_type1（conv/act 路径）等整层 act 输入流
    // （含 padding 通道）全部排空（act_done），而非 transfer_done（真实通道写满 wr_len）。
    // 提前放行会让 load/dispatch 在上游排空完成前下发后续指令（Export 曾因此掐断
    // quan 末段 burst）。pool/res/ps 路径无 padding 通道，transfer_done 即全承接。
    else store_fm_done <= store_is_write_fm &
                          ((is_type1 & act_done) | (!is_type1 & transfer_done)) &
                          (fm_channel_end == (fm_channel_total-1));
end

endmodule
