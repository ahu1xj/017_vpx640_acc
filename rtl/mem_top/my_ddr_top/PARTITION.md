# my_ddr_top —— DMA 复写版划分方案

> 目标：功能等效于 `rtl/mem_top/dma_axi4_master`，但**结构更清晰、参数化更彻底**。
> 验收：现有 `vivado_tb/mem_top/` 的 tb 一行不改，跑出同样的数字（512 位宽：81/74/25）。

---

## 1. 划分

```
                        dma_ddr_top.v
                              │
     ┌────────────┬───────────┴───────────┬────────────┐
     │            │                       │            │
 [写命令]      [读命令]                 [写数据]      [读数据]
 64b,usr_clk   64b,usr_clk             512b,usr_clk  512b,usr_clk
     │            │                       │            │
     ↓            ↓                       │            │
 u_wr_cmd_fifo  u_rd_cmd_fifo             │            │
     │            │                       │            │
     ↓            ↓                       │            │
┌──────────┐  ┌──────────┐                  │            │
│decoder_wr│  │decoder_rd│  ← 同一份代码，例化两次        │
│ (cmd→突发)│  │ (cmd→突发)│                 │            │
└────┬─────┘  └────┬─────┘                  │            │
     │ 突发         │ 突发                    │            │
     ↓            ↓                       ↓            │
┌───────────────────┐            ┌──────────────────┐   │
│   dma_wr_path     │            │   dma_rd_path    │   │
│  desc + 数据 → AXI │            │  desc → AXI → 数据│←──┘
│  ├ dma_app_unpack │            │  ├ dma_app_pack  │
│  │  (512b→N×AXI)  │            │  │  (N×AXI→512b) │
│  └ AW/W/B 时序     │            │  └ AR/R 时序      │
└─────────┬─────────┘            └────────┬─────────┘
          │                                │
          └────────────┬───────────────────┘
                       ↓
                 m_axi_*  (AXI4 主口, clk 域)
```

**7 个模块**（原版 4 个）：

| 文件 | 行数估 | 职责 | 谁写 |
|---|---|---|---|
| `dma_ddr_top.v` | ~180 | 端口合同 + 参数校验 + 例化 | ⬜ 你写 |
| `decoder_wrcmd.v` | ~120 | **命令 → 突发序列**（4KB / 突发上限切分）| ⬜ **你写**（规格见文末）|
| `decoder_rdcmd.v` | ~120 | 同上，与写侧**同一份代码** | ⬜ **你写** |
| `dma_app_unpack.v` | ~60 | 512b app 拍 → N × AXI 拍（写方向）| ⬜ 你写 |
| `dma_app_pack.v` | ~70 | N × AXI 拍 → 512b app 拍（读方向）| ⬜ 你写 |
| `dma_wr_path.v` | ~200 | 突发 + 数据流 → AW/W/B | ⬜ **你写** |
| `dma_rd_path.v` | ~180 | 突发 → AR/R → 数据流 | ⬜ **你写** |
| `dma_fifo.v` | —— | 跨时钟/弹性缓冲（直接用 IP）| ✅ 用 IP |

> **分工理由**：切分规则（4KB 边界、突发上限）是 AXI 协议规定的，没有设计自由度。
> 而 `wr_path`/`rd_path` 里装着全部**设计决策**（AW/W 时序、流控策略、缓冲位置、单/多 outstanding）
> → 那才是练手的地方。

> **3 个唯一模块**：`decoder_wrcmd` / `decoder_rdcmd` 逻辑相同（参数也相同），
> 写一份验证通过后复制即可。所以实际要设计的是 **decoder + wr_path + rd_path 三块**。

---

## 2. 各模块接口与行为规格

### 2.1 `decoder_wrcmd` / `decoder_rdcmd` —— 命令 → 突发序列

```
入: cmd_data = {addr[31:0], bytes[31:0]}     （长度是字节）
出: burst_addr / burst_len                     （拍数-1，不是字节）
```

**完整规格见文末「复写 DMA」一节**（端口表、切分算术、状态机、握手规则、特殊情况、不变量、独立 tb）。

一句话：收下一条 CMD2 → 按 4KB 边界与拍数上限逐刀切分 → 每拍吐一条
`{burst_addr, burst_len, cmd_tvalid}`，直到剩余字节为 0。

### 2.2 `dma_app_unpack`（写方向）

```
入: 512b app 流（s_app_valid/tready/tdata）
出: N b AXI 拍流（m_beat_valid/tready/tdata），每 app 拍产出 BEATS_PER_APP 拍
```

**行为**：锁存一个 app 拍 → 按 `BEAT_IDX` 递增逐拍吐出 AXI 拍 → 吐完再收下一个。
`s_app_ready` 只在"手上没有未吐完的 app 拍"时拉高。

### 2.3 `dma_app_pack`（读方向）

反向：收 N 个 AXI 拍 → 拼成一个 512b app 拍 → 置 `m_app_valid`。

### 2.4 `dma_wr_path`（⬜ 你写）

```
入:  burst_addr / burst_len（突发）
     cmd_tready（→ decoder 的反压）
     s_beat_*（AXI 宽度的数据拍流，来自 dma_app_unpack）
     m_axi_awready / wready / bid / bresp / bvalid
出:  m_axi_aw* / w* / bready
     s_beat_ready
```

**行为规格**：
1. 收下一个突发 → 锁存地址与拍数
2. 启动 AW（地址 + `awlen = burst_len`）与 W（逐拍数据 + 末拍 `wlast`）
3. 等 B 响应 → 该突发完成 → `cmd_tready` 拉高，接下一个
4. **AW/W 的启动顺序、是否提前预取下一个突发、FIFO 深度策略 —— 由你决定**（见 REWRITE_GUIDE §4）

⚠️ **终止条件是隐式的**：没有突发要发时 `cmd_tvalid` 自然拉低，不需要记总数。
（这正是去掉 `burst_num` 换来的简化，见文末。）

### 2.5 `dma_rd_path`（⬜ 你写）

与 `dma_wr_path` 对称：启动 AR → 收 R 拍 → 交给 `dma_app_pack`。
**读写共口策略（谁优先、是否并发）由你决定**。

⚠️ **读发起新突发前必须让写** —— 否则读流一直占总线 → 写的 B 响应回不来 →
而某个状态机正等写完成 → **死锁**。原版做法：

```verilog
assign issue_fire = desc_valid && ar_slot_ready && (!write_busy) && issue_ok_r;
```

**宁可降低读吞吐，也不能锁死。** 详见 §6 坑 8。

---

## 3. 与原版的差异清单（为什么这么分）

| # | 原版做法 | 本方案 | 理由 |
|---|---|---|---|
| 1 | 写侧切分内联在 `dma_write_driver`，读侧独立成 `dma_cmd_splitter` | **读写共用 `decoder_*cmd`** | 原版有**两套地址算术**，改一处容易漏另一处；共用后只验一份 |
| 2 | 位宽适配用 `ifdef` 四档藏在 driver 里 | **独立 `dma_app_unpack`/`pack`** | 原版这条路径**从未跑过**（一直用 512 位）；独立后可单测 |
| 3 | `dma_cmd_splitter` 地址算术硬编码 32 位（10 个 `reg [31:0]`）| 内部算术用参数化位宽 | ⚠️ **但做不到 >4GB** —— CMD2 的地址字段是 32 位（合同），`store.v` 拼命令时也是 32 位。想支持 >4GB 得改**命令格式**，那是上游的合同变更。本方案只是"内部算术位宽可参数化"，**上限仍受 CMD2 的 32 位约束** |
| 4 | `MAX_AXI_BURST_BYTES = 256 * AXI_BYTES` 硬编码 | 参数 `MAX_BURST_BEATS` | 目标板若是 AXI3 需降到 16 |
| 5 | 流控依赖 IP FIFO 的 `axis_wr_data_count` | 自带 in-flight 计数 | `data_count` 的语义（是否含输出寄存器）随 IP 配置变；少一个隐式依赖 |
| 6 | 描述符 `{addr[31:0], beats_m1[7:0], 24'd0}` = **64 位**（24 位纯填充，为和 CMD2 同宽好复用 FIFO）| `{burst_addr[31:0], burst_len[7:0]}` = **40 位** | **省 24 位** |
| 7 | 参数校验分散在 4 个文件的 `generate` 里 | **集中在 `dma_ddr_top`** | 一处看全 |
| 8 | 零长度命令静默丢弃；长度非整倍数静默截断；对齐/容量不检查 | **`cmd_err[3:0]` 显式报错**，有错不发突发 | 原版四条"不检查、静默错"，移植时无防线 |

**保留原版的优点**：单 outstanding（一次一条命令）——简单、够用，且本工程的访存模式是顺序大块，不需要乱序。

---

## 4. 落地顺序（对应 REWRITE_GUIDE §3）

```
S0    骨架：dma_ddr_top 端口 + 各模块空壳
            → 编译过（wr_path/rd_path 还没填，会缺模块，这是预期的）

S0.5  ★ decoder_wrcmd 独立 tb：只验切分算术
            → 判据：突发条数对上 §0 的表（512 位宽 81/74）
            → 纯 RTL + iverilog，一轮几秒

S1    dma_wr_path 最简版（单突发、AW→W→B 串行）→ CASE=1 过
S2    dma_rd_path 最简版（单突发、AR→R）        → CASE=1 过
S3    长传输（>4KB 多突发）→ CASE=2 数字对上（写 5 / 读 5）
S4    跨 4KB              → CASE=3（写 2 / 读 2）
S5    位宽横扫 512/256/128/64 → 数字见 REWRITE_GUIDE §0（★ 64 位是重点）
S6    背靠背 + 慢 ready   → CASE=4 / CASE=6
S7    全量                → 81 / 74 / 25 / 0 / 0 / 0
```

**为什么 S0.5 要插在 S1 之前**：

- `wr_path` 最简版就要求**整条链都通**（FIFO + decoder + unpack + AXI 时序）。
  第一次调试会同时面对四个新东西。
- 而 decoder **能脱离 AXI 拿到精确数字验收** —— 其余三个模块都做不到
  （它们的验收必须带 AXI 握手，只能靠 `mem_top_tb`）。
- **一轮几秒 vs 一轮几分钟**。把切分算术在快的地方解决掉，
  别留到"开 Vivado 等仿真"的地方调。
- decoder 的输出接口要**第一个定死** —— `wr_path`/`rd_path` 都吃它；
  接口改了，两个 path 跟着重写。

---

## 5. 现有 tb 怎么接过来

**不用改 tb**——只要你的顶层模块名/端口与原版一致，`vivado_tb/mem_top/mem_top_tb.v` 里把例化名换一下即可：

```verilog
// tb 里这一处换成你的模块
dma_ddr_top #( ... ) u_dut ( ... );     // 原本是 dma_axi4_master
```

或者更省事：**别改 tb，改你的模块名**——把自己的顶层也叫 `dma_axi4_master`（放在不同文件/不同 library），tb 一字不动直接编译。

---

## 6. 已知的坑（实现时会撞上）

见 [REWRITE_GUIDE.md §5](../REWRITE_GUIDE.md)。最容易踩的三个：

1. **B 通道寄存器没复位** → `awready` 变 `x` → 写通道死等
2. **三个长度单位**：CMD2 是**字节**、`awlen` 是**拍数-1**、app 拍是 **64 字节**
3. **4KB 判据的边界**：末字节落在 0xFFF 合法 → `起始偏移 + 总字节 > 4096`，不是 `>=`

# 复写 DMA

## decoder_wrcmd

### 端口


| 方向 | 信号 | 位宽 | 说明 |
|---|---|---|---|
| in | `clk` | 1 | `ui_clk` |
| in | `rst_n` | 1 | 复位（低有效） |
| in | `cmd_data` | `[63:0]` | `{addr[31:0], bytes[31:0]}`，**字节数不是拍数** |
| in | `cmd_valid` | 1 | 来自 FIFO 的 `m_axis_tvalid` |
| **in** | **`cmd_tready`** | 1 | **← driver：我能收下一个突发** |
| out | `s_axis_tready` | 1 | → FIFO 的 `m_axis_tready` |
| out | `cmd_address` | `[31:0]` | 本条突发的起始地址 |
| out | `burst_len` | `[7:0]` | 本条突发的**拍数 − 1**（= `awlen`，直接用） |
| out | `cmd_tvalid` | 1 | 本条突发有效 |
| out | `cmd_err` | `[3:0]` | 命令级异常，旁路 |



