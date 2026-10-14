//=============================================================================
// 文件名: rd_sm_cmd_define.v
// 描述: 卷积神经网络加速器读指令（rd_sm_cmd）字段定义
//       指令位宽: 160bit
//       定义规则: 起始位 = 上一字段起始位 + 上一字段位宽
//=============================================================================
`define RD_SM_CMD_WIDTH  160
//-----------------------------------------------------------------------------
// 1. 通用控制字段 (0-46位，所有算子类型共享)
//-----------------------------------------------------------------------------
`define RD_RW_FLAG_WIDTH                    1
`define RD_RW_FLAG_START                    0

`define RD_MODEL_SEL_WIDTH                  4
`define RD_MODEL_SEL_START                  (`RD_RW_FLAG_START + `RD_RW_FLAG_WIDTH)                 // 1

`define RD_FM_CHANNEL_IN_WIDTH              12
`define RD_FM_CHANNEL_IN_START              (`RD_MODEL_SEL_START + `RD_MODEL_SEL_WIDTH)              // 5

`define RD_FM_ROW_IN_WIDTH                  12
`define RD_FM_ROW_IN_START                  (`RD_FM_CHANNEL_IN_START + `RD_FM_CHANNEL_IN_WIDTH)      // 17

`define RD_FM_COLUMN_IN_WIDTH               7
`define RD_FM_COLUMN_IN_START               (`RD_FM_ROW_IN_START + `RD_FM_ROW_IN_WIDTH)              // 29

`define RD_FM_CHANNEL_OUT_WIDTH             6
`define RD_FM_CHANNEL_OUT_START             (`RD_FM_COLUMN_IN_START + `RD_FM_COLUMN_IN_WIDTH)        // 36

`define RD_LAST_FM_OUT_COLUMN_VALID_WIDTH   5
`define RD_LAST_FM_OUT_COLUMN_VALID_START   (`RD_FM_CHANNEL_OUT_START + `RD_FM_CHANNEL_OUT_WIDTH)    // 42

// 算子类型编码定义 (model_sel字段)
`define MODEL_SEL_CONV                      4'b0000
`define MODEL_SEL_POOLING                   4'b0001
`define MODEL_SEL_RESADD                    4'b0010
`define MODEL_SEL_UPSAMPLE                  4'b0011
`define MODEL_SEL_CONCAT                    4'b0100
`define MODEL_SEL_SPLIT                     4'b0101
`define MODEL_SEL_EXPORT                    4'b0110

//-----------------------------------------------------------------------------
// 2. 算子专用字段 (47-104位，不同算子复用该地址空间)
//-----------------------------------------------------------------------------

//--- 2.1 卷积 (CONV) 专用字段 [104:47] ----------------------------------------
`define RD_CONV_OP_START                    47

`define RD_CONV_KERNEL_SIZE_R_WIDTH         3
`define RD_CONV_KERNEL_SIZE_R_START         (`RD_CONV_OP_START)                                     // 47 [49:47]

`define RD_CONV_KERNEL_SIZE_C_WIDTH         3
`define RD_CONV_KERNEL_SIZE_C_START         (`RD_CONV_KERNEL_SIZE_R_START + `RD_CONV_KERNEL_SIZE_R_WIDTH)   // 50 [52:50]

`define RD_CONV_STRIDE_R_WIDTH              2
`define RD_CONV_STRIDE_R_START              (`RD_CONV_KERNEL_SIZE_C_START + `RD_CONV_KERNEL_SIZE_C_WIDTH)   // 53 [54:53]

`define RD_CONV_STRIDE_C_WIDTH              2
`define RD_CONV_STRIDE_C_START              (`RD_CONV_STRIDE_R_START + `RD_CONV_STRIDE_R_WIDTH)      // 55 [56:55]

`define RD_CONV_PADDING_TOP_WIDTH           2
`define RD_CONV_PADDING_TOP_START           (`RD_CONV_STRIDE_C_START + `RD_CONV_STRIDE_C_WIDTH)      // 57 [58:57]

`define RD_CONV_PADDING_BOTTOM_WIDTH        2
`define RD_CONV_PADDING_BOTTOM_START        (`RD_CONV_PADDING_TOP_START + `RD_CONV_PADDING_TOP_WIDTH) // 59 [60:59]

`define RD_CONV_PADDING_RIGHT_WIDTH         2
`define RD_CONV_PADDING_RIGHT_START         (`RD_CONV_PADDING_BOTTOM_START + `RD_CONV_PADDING_BOTTOM_WIDTH) // 61 [62:61]

`define RD_CONV_PADDING_LEFT_WIDTH          2
`define RD_CONV_PADDING_LEFT_START          (`RD_CONV_PADDING_RIGHT_START + `RD_CONV_PADDING_RIGHT_WIDTH)   // 63 [64:63]

`define RD_CONV_FM_OUT_ROW_WIDTH            12
`define RD_CONV_FM_OUT_ROW_START            (`RD_CONV_PADDING_LEFT_START + `RD_CONV_PADDING_LEFT_WIDTH)     // 65 [76:65]

`define RD_CONV_FM_OUT_COL_WIDTH            7
`define RD_CONV_FM_OUT_COL_START            (`RD_CONV_FM_OUT_ROW_START + `RD_CONV_FM_OUT_ROW_WIDTH)  // 77 [83:77]

`define RD_CONV_CIRCULAR_PADDING_EN_WIDTH   1
`define RD_CONV_CIRCULAR_PADDING_EN_START   (`RD_CONV_FM_OUT_COL_START + `RD_CONV_FM_OUT_COL_WIDTH)  // 84 [84]

`define RD_CONV_CIRCULAR_PADDING_SEL_WIDTH  1
`define RD_CONV_CIRCULAR_PADDING_SEL_START  (`RD_CONV_CIRCULAR_PADDING_EN_START + `RD_CONV_CIRCULAR_PADDING_EN_WIDTH) // 85 [85]

`define RD_CONV_OP_TYPE_WIDTH               1
`define RD_CONV_OP_TYPE_START               (`RD_CONV_CIRCULAR_PADDING_SEL_START + `RD_CONV_CIRCULAR_PADDING_SEL_WIDTH) // 86 [86]

`define RD_CONV_OUT_TYPE_WIDTH              1
`define RD_CONV_OUT_TYPE_START              (`RD_CONV_OP_TYPE_START + `RD_CONV_OP_TYPE_WIDTH)        // 87 [87]

`define RD_CONV_RESERVE_WIDTH               17
`define RD_CONV_RESERVE_START               (`RD_CONV_OUT_TYPE_START + `RD_CONV_OUT_TYPE_WIDTH)      // 88 [104:88]
// 累积校验: 88 + 17 = 105

//--- 2.2 池化 (POOLING) 专用字段 [104:47] -------------------------------------
`define RD_POOL_OP_START                    47

`define RD_POOL_KERNEL_SIZE_R_WIDTH         3
`define RD_POOL_KERNEL_SIZE_R_START         (`RD_POOL_OP_START)                                     // 47 [49:47]

`define RD_POOL_KERNEL_SIZE_C_WIDTH         3
`define RD_POOL_KERNEL_SIZE_C_START         (`RD_POOL_KERNEL_SIZE_R_START + `RD_POOL_KERNEL_SIZE_R_WIDTH)   // 50 [52:50]

`define RD_POOL_STRIDE_R_WIDTH              2
`define RD_POOL_STRIDE_R_START              (`RD_POOL_KERNEL_SIZE_C_START + `RD_POOL_KERNEL_SIZE_C_WIDTH)   // 53 [54:53]

`define RD_POOL_STRIDE_C_WIDTH              2
`define RD_POOL_STRIDE_C_START              (`RD_POOL_STRIDE_R_START + `RD_POOL_STRIDE_R_WIDTH)      // 55 [56:55]

`define RD_POOL_PADDING_TOP_WIDTH           2
`define RD_POOL_PADDING_TOP_START           (`RD_POOL_STRIDE_C_START + `RD_POOL_STRIDE_C_WIDTH)      // 57 [58:57]

`define RD_POOL_PADDING_BOTTOM_WIDTH        2
`define RD_POOL_PADDING_BOTTOM_START        (`RD_POOL_PADDING_TOP_START + `RD_POOL_PADDING_TOP_WIDTH) // 59 [60:59]

`define RD_POOL_PADDING_RIGHT_WIDTH         2
`define RD_POOL_PADDING_RIGHT_START         (`RD_POOL_PADDING_BOTTOM_START + `RD_POOL_PADDING_BOTTOM_WIDTH) // 61 [62:61]

`define RD_POOL_PADDING_LEFT_WIDTH          2
`define RD_POOL_PADDING_LEFT_START          (`RD_POOL_PADDING_RIGHT_START + `RD_POOL_PADDING_RIGHT_WIDTH)   // 63 [64:63]

`define RD_POOL_POOLING_TYPE_WIDTH          1
`define RD_POOL_POOLING_TYPE_START          (`RD_POOL_PADDING_LEFT_START + `RD_POOL_PADDING_LEFT_WIDTH)     // 65 [65]

`define RD_POOL_CIRCULAR_PADDING_WIDTH      1
`define RD_POOL_CIRCULAR_PADDING_START      (`RD_POOL_POOLING_TYPE_START + `RD_POOL_POOLING_TYPE_WIDTH)     // 66 [66]

`define RD_POOL_CIRCULAR_PADDING_SEL_WIDTH  1
`define RD_POOL_CIRCULAR_PADDING_SEL_START  (`RD_POOL_CIRCULAR_PADDING_START + `RD_POOL_CIRCULAR_PADDING_WIDTH) // 67 [67]

`define RD_POOL_RESERVE_WIDTH               37
`define RD_POOL_RESERVE_START               (`RD_POOL_CIRCULAR_PADDING_SEL_START + `RD_POOL_CIRCULAR_PADDING_SEL_WIDTH) // 68 [104:68]
// 累积校验: 68 + 37 = 105

//--- 2.3 残差加 (RESADD) 专用字段 [104:47] ------------------------------------
`define RD_RESADD_OP_START                  47

`define RD_RESADD_RESERVE_WIDTH              58
`define RD_RESADD_RESERVE_START             (`RD_RESADD_OP_START)                                    // 47 [104:47]
// 累积校验: 47 + 58 = 105

//--- 2.4 上采样 (UPSAMPLE) 专用字段 [104:47] ---------------------------------
`define RD_UPSAMPLE_OP_START                47

`define RD_UPSAMPLE_TYPE_WIDTH              1
`define RD_UPSAMPLE_TYPE_START              (`RD_UPSAMPLE_OP_START)                                  // 47 [47]

`define RD_UPSAMPLE_RESERVE_WIDTH           57
`define RD_UPSAMPLE_RESERVE_START           (`RD_UPSAMPLE_TYPE_START + `RD_UPSAMPLE_TYPE_WIDTH)      // 48 [104:48]
// 累积校验: 48 + 57 = 105

//--- 2.5 通道拼接 (CONCAT) 专用字段 [104:47] -----------------------------------
`define RD_CONCAT_OP_START                  47

`define RD_CONCAT_TYPE_WIDTH                2
`define RD_CONCAT_TYPE_START                (`RD_CONCAT_OP_START)                                    // 47 [48:47]

`define RD_CONCAT_FM2_IN_CHANNEL_WIDTH      12
`define RD_CONCAT_FM2_IN_CHANNEL_START      (`RD_CONCAT_TYPE_START + `RD_CONCAT_TYPE_WIDTH)          // 49 [60:49]

`define RD_CONCAT_FM3_IN_CHANNEL_WIDTH      12
`define RD_CONCAT_FM3_IN_CHANNEL_START      (`RD_CONCAT_FM2_IN_CHANNEL_START + `RD_CONCAT_FM2_IN_CHANNEL_WIDTH) // 61 [72:61]

`define RD_CONCAT_FM4_IN_CHANNEL_WIDTH      12
`define RD_CONCAT_FM4_IN_CHANNEL_START      (`RD_CONCAT_FM3_IN_CHANNEL_START + `RD_CONCAT_FM3_IN_CHANNEL_WIDTH) // 73 [84:73]

`define RD_CONCAT_RESERVE_WIDTH             20
`define RD_CONCAT_RESERVE_START             (`RD_CONCAT_FM4_IN_CHANNEL_START + `RD_CONCAT_FM4_IN_CHANNEL_WIDTH) // 85 [104:85]
// 累积校验: 85 + 20 = 105

//--- 2.6 分割 (SPLIT) 专用字段 [104:47] ---------------------------------------
`define RD_SPLIT_OP_START                   47

`define RD_SPLIT_CH_START_WIDTH             12
`define RD_SPLIT_CH_START_START             (`RD_SPLIT_OP_START)                                     // 47 [58:47]

`define RD_SPLIT_CH_NUM_WIDTH               12
`define RD_SPLIT_CH_NUM_START               (`RD_SPLIT_CH_START_START + `RD_SPLIT_CH_START_WIDTH)    // 59 [70:59]

`define RD_SPLIT_RESERVE_WIDTH              34
`define RD_SPLIT_RESERVE_START              (`RD_SPLIT_CH_NUM_START + `RD_SPLIT_CH_NUM_WIDTH)        // 71 [104:71]
// 累积校验: 71 + 34 = 105

//--- 2.7 导出 (EXPORT) 专用字段 [104:47] -------------------------------------
`define RD_EXPORT_OP_START                  47

`define RD_EXPORT_LAST_WIDTH        1
`define RD_EXPORT_LAST_START        (`RD_EXPORT_OP_START)                                    // 47 [47]

`define RD_EXPORT_OUT_TYPE_WIDTH            1
`define RD_EXPORT_OUT_TYPE_START            (`RD_EXPORT_OP_START + `RD_EXPORT_LAST_WIDTH)    // 48 [48]

`define RD_EXPORT_RESERVE_WIDTH             56
`define RD_EXPORT_RESERVE_START             (`RD_EXPORT_OUT_TYPE_START + `RD_EXPORT_OUT_TYPE_WIDTH) // 49 [104:49]
// 累积校验: 49 + 56    = 105

//-----------------------------------------------------------------------------
// 3. 指针字段 (105-159位，注意不同算子类型的复用关系)
//-----------------------------------------------------------------------------

// 基址起始位置
`define RD_PTR_START                        105

//--- 3.1 通用指针 (所有算子类型) -----------------------------------------------
`define RD_BASE_ADDR_POINT_FM_WIDTH         11
`define RD_BASE_ADDR_POINT_FM_START         (`RD_PTR_START)                                           // 105 [115:105]

`define RD_BASE_ADDR_POINT_QUAN_WIDTH       11
`define RD_BASE_ADDR_POINT_QUAN_START       (`RD_BASE_ADDR_POINT_FM_START + `RD_BASE_ADDR_POINT_FM_WIDTH)   // 116 [126:116]

//--- 3.2 卷积专用指针 (WEIGHT/BIAS/ACT) ---------------------------------------
`define RD_CONV_BASE_ADDR_POINT_WEIGHT_WIDTH    11
`define RD_CONV_BASE_ADDR_POINT_WEIGHT_START    (`RD_BASE_ADDR_POINT_QUAN_START + `RD_BASE_ADDR_POINT_QUAN_WIDTH) // 127 [137:127]

`define RD_CONV_BASE_ADDR_POINT_BIAS_WIDTH      11
`define RD_CONV_BASE_ADDR_POINT_BIAS_START      (`RD_CONV_BASE_ADDR_POINT_WEIGHT_START + `RD_CONV_BASE_ADDR_POINT_WEIGHT_WIDTH) // 138 [148:138]

`define RD_CONV_BASE_ADDR_POINT_ACT_WIDTH       11
`define RD_CONV_BASE_ADDR_POINT_ACT_START       (`RD_CONV_BASE_ADDR_POINT_BIAS_START + `RD_CONV_BASE_ADDR_POINT_BIAS_WIDTH)     // 149 [159:149]
// 累积校验: 149 + 11 = 160

//--- 3.3 残差加与通道拼接共用指针 (FM2/FM3/FM4) ---------------------------
`define RD_MULTI_FM2_WIDTH              11
`define RD_MULTI_FM2_START              (`RD_BASE_ADDR_POINT_QUAN_START + `RD_BASE_ADDR_POINT_QUAN_WIDTH) // 127 [137:127]

`define RD_MULTI_FM3_WIDTH              11
`define RD_MULTI_FM3_START              (`RD_MULTI_FM2_START + `RD_MULTI_FM2_WIDTH)   // 138 [148:138]

`define RD_MULTI_FM4_WIDTH              11
`define RD_MULTI_FM4_START              (`RD_MULTI_FM3_START + `RD_MULTI_FM3_WIDTH)   // 149 [159:149]

//-----------------------------------------------------------------------------
// 4. 辅助宏 (可选，用于字段提取)
//-----------------------------------------------------------------------------
`define RD_GET_FIELD(signal, start, width)  (((signal) >> (start)) & ((1 << (width)) - 1))
