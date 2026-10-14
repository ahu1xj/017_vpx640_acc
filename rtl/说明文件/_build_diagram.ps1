$ErrorActionPreference = "Stop"

$dir     = "D:\acc_690t\017_vpx640_acc\rtl\说明文件"
$outVsdx = Join-Path $dir "说明.vsdx"
$outPng  = Join-Path $dir "_preview.png"

# ---------------- palette ----------------
$black   = "RGB(0,0,0)"
$gray    = "RGB(96,96,96)"
$dkblue  = "RGB(31,78,121)"
$red     = "RGB(150,40,30)"
$boxFill = "RGB(222,235,247)"
$boxLine = "RGB(31,78,121)"
$dmaFill = "RGB(226,239,218)"
$calFill = "RGB(252,228,214)"
$uiFill  = "RGB(222,235,247)"
$rstFill = "RGB(240,240,240)"
$subLine = "RGB(140,170,200)"

# ---------------- helpers ----------------
function Txt($pg, $x1, $y1, $x2, $y2, $text, $size, $bold, $halign, $color, $fill, $line) {
    $s = $pg.DrawRectangle($x1, $y1, $x2, $y2)
    if ($text) { $s.Text = $text }
    $s.CellsU("Char.Size").FormulaU = "$size pt"
    $s.CellsU("Char.Style").FormulaU = $(if ($bold) { "1" } else { "0" })
    $s.CellsU("Para.HorzAlign").FormulaU = "$halign"
    $s.CellsU("VerticalAlign").FormulaU = "1"
    $s.CellsU("Char.Color").FormulaU = $color
    if ($fill) { $s.CellsU("FillPattern").FormulaU = "1"; $s.CellsU("FillForegnd").FormulaU = $fill }
    else       { $s.CellsU("FillPattern").FormulaU = "0" }
    if ($line) { $s.CellsU("LinePattern").FormulaU = "1"; $s.CellsU("LineColor").FormulaU = $line; $s.CellsU("LineWeight").FormulaU = "1 pt" }
    else       { $s.CellsU("LinePattern").FormulaU = "0" }
    return $s
}

function Ln($pg, $x1, $y1, $x2, $y2, $color, $w, $bA, $eA, $dash) {
    $s = $pg.DrawLine($x1, $y1, $x2, $y2)
    $s.CellsU("LineColor").FormulaU = $color
    $s.CellsU("LineWeight").FormulaU = "$w pt"
    $s.CellsU("BeginArrow").FormulaU = "$bA"
    $s.CellsU("EndArrow").FormulaU = "$eA"
    if ($dash) { $s.CellsU("LinePattern").FormulaU = "2" }
    return $s
}

# ---------------- port tables ----------------
$inGroups = @(
    @{ t = "主机包流输入 (AXIS-Stream 512b)"; p = @("s_axis_ps_tdata[511:0]", "s_axis_ps_tvalid") },
    @{ t = "结果输出握手";                     p = @("m_axis_pso_tready") },
    @{ t = "时钟";                             p = @("clk", "cal_clk", "dma_clk") },
    @{ t = "复位";                             p = @("rst_n", "init_calib_complete") },
    @{ t = "AXI4 主存 · 写侧握手";             p = @("m_axi_awready", "m_axi_wready", "m_axi_bid[3:0]", "m_axi_bresp[1:0]", "m_axi_bvalid") },
    @{ t = "AXI4 主存 · 读侧";                 p = @("m_axi_arready", "m_axi_rid[3:0]", "m_axi_rdata[511:0]", "m_axi_rresp[1:0]", "m_axi_rlast", "m_axi_rvalid") }
)

$outGroups = @(
    @{ t = "主机包流就绪";                     p = @("s_axis_ps_tready") },
    @{ t = "结果输出 (AXIS-Stream 512b)";      p = @("m_axis_pso_tdata[511:0]", "m_axis_pso_tvalid", "m_axis_pso_tlast") },
    @{ t = "复位输出";                         p = @("dma_rst_n") },
    @{ t = "AXI4 主存 · 写地址 AW";            p = @("m_axi_awid[3:0]", "m_axi_awaddr[31:0]", "m_axi_awlen[7:0]", "m_axi_awsize[2:0]", "m_axi_awburst[1:0]", "m_axi_awlock", "m_axi_awcache[3:0]", "m_axi_awprot[2:0]", "m_axi_awqos[3:0]", "m_axi_awvalid") },
    @{ t = "AXI4 主存 · 写数据 W";             p = @("m_axi_wdata[511:0]", "m_axi_wstrb[63:0]", "m_axi_wlast", "m_axi_wvalid") },
    @{ t = "AXI4 主存 · 写响应 B";             p = @("m_axi_bready") },
    @{ t = "AXI4 主存 · 读地址 AR";            p = @("m_axi_arid[3:0]", "m_axi_araddr[31:0]", "m_axi_arlen[7:0]", "m_axi_arsize[2:0]", "m_axi_arburst[1:0]", "m_axi_arlock", "m_axi_arcache[3:0]", "m_axi_arprot[2:0]", "m_axi_arqos[3:0]", "m_axi_arvalid") },
    @{ t = "AXI4 主存 · 读数据 R";             p = @("m_axi_rready") }
)

# ---------------- launch visio ----------------
$app = New-Object -ComObject Visio.Application
$app.Visible = $false
$app.AlertResponse = 7
$doc  = $app.Documents.Add("")
$page = $doc.Pages.Item(1)
$page.Name = "cnn_acc_top"

$ps = $page.PageSheet
$ps.CellsU("PageWidth").FormulaU  = "16 in"
$ps.CellsU("PageHeight").FormulaU = "11 in"

# ================= module body =================
$mod = Txt $page 4.60 0.90 11.40 9.90 "cnn_acc_top" 15 $true 1 $dkblue $boxFill $boxLine
$mod.CellsU("LineWeight").FormulaU = "2 pt"

# module name sits at top of the box -> move its text up
$mod.CellsU("VerticalAlign").FormulaU = "0"
$mod.CellsU("TopMargin").FormulaU = "0.16 in"

# title above the box
Txt $page 4.60 10.30 11.40 10.85 "cnn_acc_top 顶层框图" 18 $true 1 $dkblue "" "" | Out-Null
Txt $page 4.60 9.95 11.40 10.30 "来源：rtl/top/cnn_acc_top.v ｜ 参数：ADDR=32 / DATA=512 / ID=4 / STRB=64" 9 $false 1 $gray "" "" | Out-Null

# separator under module name
Ln $page 4.90 9.42 11.10 9.42 $subLine 0.75 0 0 $false | Out-Null

# ================= internal sub-modules =================
# row A
Txt $page 6.80 8.45 9.20 9.10 "initial_top`ndma_clk" 8 $true 1 $black $dmaFill $subLine | Out-Null
# row B
Txt $page 4.95 7.00 6.45 7.75 "load_top`ndma_clk"  8 $true 1 $black $dmaFill $subLine | Out-Null
Txt $page 7.15 7.00 8.65 7.75 "cal_top`ncal_clk"    8 $true 1 $black $calFill $subLine | Out-Null
Txt $page 9.35 7.00 10.85 7.75 "store_top`ndma_clk" 8 $true 1 $black $dmaFill $subLine | Out-Null
# row C
Txt $page 5.00 5.05 10.80 5.95 "dma_axi4_master`nui_clk + dma_clk" 8 $true 1 $black $uiFill $subLine | Out-Null
# row D
Txt $page 5.40 3.50 7.90 4.10 "rst_dma_clk`ndma_clk" 8 $true 1 $black $rstFill $subLine | Out-Null
Txt $page 8.10 3.50 10.60 4.10 "rst_cal_clk`ncal_clk" 8 $true 1 $black $rstFill $subLine | Out-Null

# internal arrows
Ln $page 7.30 8.45 5.70 7.75 $dkblue 1.0 0 4 $false | Out-Null
Ln $page 8.70 8.45 10.10 7.75 $dkblue 1.0 0 4 $false | Out-Null
Ln $page 6.45 7.375 7.15 7.375 $dkblue 1.0 0 4 $false | Out-Null
Ln $page 8.65 7.375 9.35 7.375 $dkblue 1.0 0 4 $false | Out-Null
Ln $page 5.70 7.00 5.70 5.95 $dkblue 1.0 4 4 $false | Out-Null
Ln $page 10.10 7.00 10.10 5.95 $dkblue 1.0 0 4 $false | Out-Null

# internal arrow labels
Txt $page 5.60 8.05 6.90 8.35 "读指令" 6.5 $false 1 $gray "" "" | Out-Null
Txt $page 9.05 8.05 10.55 8.35 "写指令 / psi" 6.5 $false 1 $gray "" "" | Out-Null
Txt $page 6.05 7.50 7.60 7.78 "fm·we·bias·quan·act" 6 $false 1 $gray "" "" | Out-Null
Txt $page 8.30 7.50 9.75 7.78 "act·pooling·res" 6 $false 1 $gray "" "" | Out-Null
Txt $page 5.85 6.35 7.60 6.65 "rdcmd2 · rdddr" 6.5 $false 0 $gray "" "" | Out-Null
Txt $page 8.30 6.35 9.95 6.65 "wrcmd2 · wrddr" 6.5 $false 2 $gray "" "" | Out-Null

# notes
Txt $page 5.00 4.45 10.80 4.75 "m_axi4 512b ↔ DDR3 (MIG) ｜ AW/W/B/AR/R 全通道，无位宽转换" 7 $false 1 $dkblue "" "" | Out-Null
Txt $page 5.00 2.85 10.80 3.15 "域复位由 proc_sys_reset 产生：ext=rst_n，aux=init_calib_complete（异步置位 / 同步释放）" 6.5 $false 1 $gray "" "" | Out-Null

# clock-domain legend
Txt $page 5.30 2.45 5.50 2.65 "" 6.5 $false 1 $black $dmaFill $subLine | Out-Null
Txt $page 5.55 2.42 6.55 2.68 "dma_clk 域" 6.5 $false 0 $gray "" "" | Out-Null
Txt $page 6.70 2.45 6.90 2.65 "" 6.5 $false 1 $black $calFill $subLine | Out-Null
Txt $page 6.95 2.42 7.95 2.68 "cal_clk 域" 6.5 $false 0 $gray "" "" | Out-Null
Txt $page 8.10 2.45 8.30 2.65 "" 6.5 $false 1 $black $uiFill $subLine | Out-Null
Txt $page 8.35 2.42 9.60 2.68 "ui_clk + dma_clk" 6.5 $false 0 $gray "" "" | Out-Null

# ================= left ports (inputs) =================
$y = 9.75
foreach ($g in $inGroups) {
    Txt $page 1.40 ($y - 0.20) 4.15 $y $g.t 8 $true 2 $gray "" "" | Out-Null
    $y -= 0.20
    foreach ($p in $g.p) {
        $cy = $y - 0.12
        Ln $page 4.18 $cy 4.60 $cy $dkblue 0.75 0 4 $false | Out-Null
        Txt $page 1.40 ($cy - 0.12) 4.15 ($cy + 0.12) $p 8 $false 2 $black "" "" | Out-Null
        $y -= 0.24
    }
}

# ================= right ports (outputs) =================
$y = 9.75
foreach ($g in $outGroups) {
    Txt $page 11.85 ($y - 0.20) 14.60 $y $g.t 8 $true 0 $gray "" "" | Out-Null
    $y -= 0.20
    foreach ($p in $g.p) {
        $cy = $y - 0.105
        Ln $page 11.40 $cy 11.82 $cy $red 0.75 0 4 $false | Out-Null
        Txt $page 11.85 ($cy - 0.105) 14.60 ($cy + 0.105) $p 8 $false 0 $black "" "" | Out-Null
        $y -= 0.21
    }
}

# ================= save / export =================
if (Test-Path $outVsdx) { Remove-Item $outVsdx -Force }
$doc.SaveAs($outVsdx)
if (Test-Path $outPng) { Remove-Item $outPng -Force }
$page.Export($outPng)
Write-Output ("saved: " + $outVsdx)
Write-Output ("shapes on page: " + $page.Shapes.Count)

$doc.Close()
$app.Quit()
