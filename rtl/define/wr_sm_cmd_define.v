//=============================================================================
// 文件名: wr_sm_cmd_define.v
// 描述: 卷积神经网络加速器写指令（wr_sm_cmd）字段定义
//       指令位宽: 160bit
//       定义规则: 起始位 = 上一字段起始位 + 上一字段位宽
//=============================================================================
`define WR_SM_CMD_WIDTH  160
//-----------------------------------------------------------------------------
// 1. 统一字段 (0-47位，所有写指令共享)
//-----------------------------------------------------------------------------
`define WR_RW_FLAG_WIDTH                    1
`define WR_RW_FLAG_START                    0

`define WR_DATA_ORIGIN_WIDTH                3
`define WR_DATA_ORIGIN_START                (`WR_RW_FLAG_START + `WR_RW_FLAG_WIDTH)                 // 1 [3:1]

`define WR_PS_DATA_TYPE_WIDTH               3
`define WR_PS_DATA_TYPE_START               (`WR_DATA_ORIGIN_START + `WR_DATA_ORIGIN_WIDTH)          // 4 [6:4]

`define WR_WR_LEN_WIDTH                     30
`define WR_WR_LEN_START                     (`WR_PS_DATA_TYPE_START + `WR_PS_DATA_TYPE_WIDTH)        // 7 [36:7]

`define WR_BASE_ADDR_WIDTH                  11
`define WR_BASE_ADDR_START                  (`WR_WR_LEN_START + `WR_WR_LEN_WIDTH)                    // 37 [47:37]
// 累积校验: 37 + 11 = 48

// data_origin 编码定义
`define DATA_ORIGIN_PS                      3'b000
`define DATA_ORIGIN_ACTIVE                  3'b001
`define DATA_ORIGIN_RES                     3'b010
`define DATA_ORIGIN_POOLING                 3'b011

// ps_data_type 编码定义
`define PS_DATA_TYPE_WEIGHT                 3'b000
`define PS_DATA_TYPE_BIAS                   3'b001
`define PS_DATA_TYPE_QUAN                   3'b010
`define PS_DATA_TYPE_ACT                    3'b011
`define PS_DATA_TYPE_FM                     3'b100

//-----------------------------------------------------------------------------
// 2. 分支字段区域 (48-159位，两种数据类型复用该地址空间)
//-----------------------------------------------------------------------------

//--- 2.1 特征图数据分支 (FM) 字段 [159:48] -------------------------------------
`define WR_FM_BRANCH_START                  48

`define WR_FM_CHANNEL_END_WIDTH             12
`define WR_FM_CHANNEL_END_START             (`WR_FM_BRANCH_START)                                     // 48 [59:48]

`define WR_FM_COLUMN_END_WIDTH              7
`define WR_FM_COLUMN_END_START              (`WR_FM_CHANNEL_END_START + `WR_FM_CHANNEL_END_WIDTH)  // 60 [66:60]

`define WR_FM_ROW_END_WIDTH                 12
`define WR_FM_ROW_END_START                 (`WR_FM_COLUMN_END_START + `WR_FM_COLUMN_END_WIDTH)      // 67 [78:67]

`define WR_FM_CHANNEL_START_WIDTH           6
`define WR_FM_CHANNEL_START_START           (`WR_FM_ROW_END_START + `WR_FM_ROW_END_WIDTH)            // 79 [84:79]

`define WR_FM_CHANNEL_TOTAL_WIDTH           12
`define WR_FM_CHANNEL_TOTAL_START           (`WR_FM_CHANNEL_START_START + `WR_FM_CHANNEL_START_WIDTH)                 

`define WR_FM_END_FLAG_WIDTH                1
`define WR_FM_END_FLAG_START                (`WR_FM_CHANNEL_TOTAL_START + `WR_FM_CHANNEL_TOTAL_WIDTH)   

`define WR_FM_OUT_TYPE_WIDTH                1
`define WR_FM_OUT_TYPE_START                (`WR_FM_END_FLAG_START + `WR_FM_END_FLAG_WIDTH)

`define WR_FM_RESERVE_WIDTH                 61
`define WR_FM_RESERVE_START                 (`WR_FM_OUT_TYPE_START + `WR_FM_OUT_TYPE_WIDTH)        
// 累积校验: 99 + 61 = 160

//--- 2.2 其他数据分支 (OTHER) 字段 [159:48] - 用于权重/偏置/量化参数加载 --
`define WR_OTHER_BRANCH_START               48

`define WR_OTHER_INIT_DONE_WIDTH            1
`define WR_OTHER_INIT_DONE_START            (`WR_OTHER_BRANCH_START)                                // 48 [48]

`define WR_KERNEL_C_IN_END_WIDTH            12
`define WR_KERNEL_C_IN_END_START            (`WR_OTHER_INIT_DONE_START + `WR_OTHER_INIT_DONE_WIDTH)  // 49 [60:49]

`define WR_KERNEL_R_SIZE_WIDTH                3
`define WR_KERNEL_R_SIZE_START                (`WR_KERNEL_C_IN_END_START + `WR_KERNEL_C_IN_END_WIDTH)  // 61 [63:61]

`define WR_KERNEL_C_SIZE_WIDTH                3
`define WR_KERNEL_C_SIZE_START                (`WR_KERNEL_R_SIZE_START + `WR_KERNEL_R_SIZE_WIDTH)          // 64 [66:64]

`define WR_KERNEL_C_OUT_END_WIDTH           6
`define WR_KERNEL_C_OUT_END_START           (`WR_KERNEL_C_SIZE_START + `WR_KERNEL_C_SIZE_WIDTH)          // 67 [72:67]

`define WR_OTHER_RESERVE_WIDTH              87
`define WR_OTHER_RESERVE_START              (`WR_KERNEL_C_OUT_END_START + `WR_KERNEL_C_OUT_END_WIDTH)   // 73 [159:73]
// 累积校验: 73 + 87 = 160

//-----------------------------------------------------------------------------
// 3. 辅助宏 (可选，用于字段提取)
//-----------------------------------------------------------------------------
`define WR_GET_FIELD(signal, start, width)  (((signal) >> (start)) & ((1 << (width)) - 1))