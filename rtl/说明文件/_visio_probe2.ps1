$ErrorActionPreference = "Stop"
$exe = "C:\Program Files (x86)\Microsoft Office\root\Office16\VISIO.EXE"
Write-Output ("exe exists: " + (Test-Path $exe))

$p = Start-Process -FilePath $exe -ArgumentList "/automation" -PassThru
Write-Output ("started pid " + $p.Id)

$app = $null
for ($i = 0; $i -lt 60; $i++) {
    Start-Sleep -Milliseconds 1000
    try {
        $app = [Runtime.InteropServices.Marshal]::GetActiveObject("Visio.Application")
        if ($app) { break }
    } catch { }
}
if (-not $app) { Write-Output "FAILED to attach"; exit 1 }

Write-Output ("attached, version = " + $app.Version)
$doc = $app.Documents.Add("")
$page = $doc.Pages.Item(1)
$shp = $page.DrawRectangle(1, 1, 3, 2)
$shp.Text = "probe"
$shp.CellsU("EndArrow").FormulaU = "0"
Write-Output ("shapes = " + $page.Shapes.Count)
$doc.Close()
$app.Quit()
Write-Output "done"
