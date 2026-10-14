$ErrorActionPreference = "Stop"
$app = New-Object -ComObject Visio.Application
$app.Visible = $false
$doc = $app.Documents.Add("")
$page = $doc.Pages.Item(1)
$shp = $page.DrawRectangle(1, 1, 3, 2)
$shp.Text = "test"
Write-Output ("OK version=" + $app.Version + " shapes=" + $page.Shapes.Count)
$doc.Close()
$app.Quit()
