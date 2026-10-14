`timescale 1ns / 1ps

// =============================================================================
// uram_ram_wrappers.v — 三大 RAM 的 URAM 版封装（端口与原 BMG IP 完全一致）
//   fram_512x11392_uram / pram_512x3712_uram / wram_512x6144_uram
//   原配置：Simple Dual Port，512bit 宽，读延迟 1 拍（doutb 寄存在 primitive 内）
//   封装用 xpm_memory_sdpram，MEMORY_PRIMITIVE=uram，READ_LATENCY_B=2 对齐延迟（原 BMG=C_READ_LATENCY_B 1 + 原语输出寄存器 1，共 2 拍，20260803 等价性 TB 实证）。
//   切换方式：cal_top.v 里把例化模块名换成 *_uram 即可（恢复 = 改回原名）。
//
//   20260812 增补：`ACC_RAM_BRAM 条件编译（690T/VPX610 用）：
//   7 系列 VX690T 无 URAM，定义该宏后同一封装改映射 blockram；
//   READ_LATENCY_B 恒为 2，端口时序契约不变（规则 46：读延迟是模块间契约）。
//   006/KU060 线不定义该宏，行为不变。
// =============================================================================

`ifdef ACC_RAM_BRAM
  `define ACC_RAM_PRIMITIVE "blockram"
`else
  `define ACC_RAM_PRIMITIVE "uram"
`endif

// fram：512bit x 11392（FM 行环）
module fram_512x11392_uram (
    input  wire         clka,
    input  wire [0:0]   wea,
    input  wire [13:0]  addra,
    input  wire [511:0] dina,
    input  wire         clkb,
    input  wire [13:0]  addrb,
    output wire [511:0] doutb
);

xpm_memory_sdpram #(
    .ADDR_WIDTH_A        (14),
    .ADDR_WIDTH_B        (14),
    .AUTO_SLEEP_TIME     (0),
    .BYTE_WRITE_WIDTH_A  (512),
    .CASCADE_HEIGHT      (0),
    .CLOCKING_MODE       ("common_clock"),
    .ECC_MODE            ("no_ecc"),
    .MEMORY_INIT_FILE    ("none"),
    .MEMORY_INIT_PARAM   ("0"),
    .MEMORY_OPTIMIZATION ("true"),
    .MEMORY_PRIMITIVE    (`ACC_RAM_PRIMITIVE),
    .MEMORY_SIZE         (512 * 11392),
    .MESSAGE_CONTROL     (0),
    .READ_DATA_WIDTH_B   (512),
    .READ_LATENCY_B      (2),
    .READ_RESET_VALUE_B  ("0"),
    .SIM_ASSERT_CHK      (0),
    .USE_MEM_INIT        (0),
    .WAKEUP_TIME         ("disable_sleep"),
    .WRITE_DATA_WIDTH_A  (512),
    .WRITE_MODE_B        ("no_change")
) u_mem (
    .clka           (clka),
    .clkb           (clkb),
    .ena            (1'b1),
    .wea            (wea),
    .addra          (addra),
    .dina           (dina),
    .enb            (1'b1),
    .addrb          (addrb),
    .doutb          (doutb),
    .rstb           (1'b0),
    .regceb         (1'b1),
    .injectsbiterra (1'b0),
    .injectdbiterra (1'b0),
    .sbiterrb       (),
    .dbiterrb       (),
    .sleep          (1'b0)
);

endmodule

// pram：512bit x 3712（cpad 单区 RAM，4 例：pram0b/0t/1b/1t）
module pram_512x3712_uram (
    input  wire         clka,
    input  wire [0:0]   wea,
    input  wire [11:0]  addra,
    input  wire [511:0] dina,
    input  wire         clkb,
    input  wire [11:0]  addrb,
    output wire [511:0] doutb
);

xpm_memory_sdpram #(
    .ADDR_WIDTH_A        (12),
    .ADDR_WIDTH_B        (12),
    .AUTO_SLEEP_TIME     (0),
    .BYTE_WRITE_WIDTH_A  (512),
    .CASCADE_HEIGHT      (0),
    .CLOCKING_MODE       ("common_clock"),
    .ECC_MODE            ("no_ecc"),
    .MEMORY_INIT_FILE    ("none"),
    .MEMORY_INIT_PARAM   ("0"),
    .MEMORY_OPTIMIZATION ("true"),
    .MEMORY_PRIMITIVE    (`ACC_RAM_PRIMITIVE),
    .MEMORY_SIZE         (512 * 3712),
    .MESSAGE_CONTROL     (0),
    .READ_DATA_WIDTH_B   (512),
    .READ_LATENCY_B      (2),
    .READ_RESET_VALUE_B  ("0"),
    .SIM_ASSERT_CHK      (0),
    .USE_MEM_INIT        (0),
    .WAKEUP_TIME         ("disable_sleep"),
    .WRITE_DATA_WIDTH_A  (512),
    .WRITE_MODE_B        ("no_change")
) u_mem (
    .clka           (clka),
    .clkb           (clkb),
    .ena            (1'b1),
    .wea            (wea),
    .addra          (addra),
    .dina           (dina),
    .enb            (1'b1),
    .addrb          (addrb),
    .doutb          (doutb),
    .rstb           (1'b0),
    .regceb         (1'b1),
    .injectsbiterra (1'b0),
    .injectdbiterra (1'b0),
    .sbiterrb       (),
    .dbiterrb       (),
    .sleep          (1'b0)
);

endmodule

// wram：512bit x 6144（权重 RAM）
module wram_512x6144_uram (
    input  wire         clka,
    input  wire [0:0]   wea,
    input  wire [12:0]  addra,
    input  wire [511:0] dina,
    input  wire         clkb,
    input  wire [12:0]  addrb,
    output wire [511:0] doutb
);

xpm_memory_sdpram #(
    .ADDR_WIDTH_A        (13),
    .ADDR_WIDTH_B        (13),
    .AUTO_SLEEP_TIME     (0),
    .BYTE_WRITE_WIDTH_A  (512),
    .CASCADE_HEIGHT      (0),
    .CLOCKING_MODE       ("common_clock"),
    .ECC_MODE            ("no_ecc"),
    .MEMORY_INIT_FILE    ("none"),
    .MEMORY_INIT_PARAM   ("0"),
    .MEMORY_OPTIMIZATION ("true"),
    .MEMORY_PRIMITIVE    (`ACC_RAM_PRIMITIVE),
    .MEMORY_SIZE         (512 * 6144),
    .MESSAGE_CONTROL     (0),
    .READ_DATA_WIDTH_B   (512),
    .READ_LATENCY_B      (2),
    .READ_RESET_VALUE_B  ("0"),
    .SIM_ASSERT_CHK      (0),
    .USE_MEM_INIT        (0),
    .WAKEUP_TIME         ("disable_sleep"),
    .WRITE_DATA_WIDTH_A  (512),
    .WRITE_MODE_B        ("no_change")
) u_mem (
    .clka           (clka),
    .clkb           (clkb),
    .ena            (1'b1),
    .wea            (wea),
    .addra          (addra),
    .dina           (dina),
    .enb            (1'b1),
    .addrb          (addrb),
    .doutb          (doutb),
    .rstb           (1'b0),
    .regceb         (1'b1),
    .injectsbiterra (1'b0),
    .injectdbiterra (1'b0),
    .sbiterrb       (),
    .dbiterrb       (),
    .sleep          (1'b0)
);

endmodule
