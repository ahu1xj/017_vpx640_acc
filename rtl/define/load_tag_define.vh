`ifndef LOAD_TAG_DEFINE_VH
`define LOAD_TAG_DEFINE_VH

// Width draft for local refactor discussion.
// Replace these with project official defines when migrating back.
`define LOAD_RD_CMD_WIDTH       160
`define LOAD_DMA_CMD_WIDTH      64
`define LOAD_CAL_CMD_WIDTH      112
`define LOAD_DDR_DATA_WIDTH     512
`define LOAD_BA_ADDR_WIDTH      11
`define LOAD_BA_DATA_WIDTH      32
`define LOAD_TAG_WIDTH          3

// TUSER tag values for compute-bound load data.
`define LOAD_TAG_FM             3'd0
`define LOAD_TAG_CPAD           3'd1
`define LOAD_TAG_WEIGHT         3'd2
`define LOAD_TAG_BIAS           3'd3
`define LOAD_TAG_QUAN_PARAM     3'd4
`define LOAD_TAG_ACT            3'd5
`define LOAD_TAG_QUAN_FM1       3'd6
`define LOAD_TAG_QUAN_FM2       3'd7

`endif
