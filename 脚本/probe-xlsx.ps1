# probe-xlsx.ps1 —— 不装 Excel 也能把 .xlsx 读成 TSV（嘉立创导出的 BOM 就是 xlsx）
# 用法: pwsh -File 脚本\probe-xlsx.ps1 -Path BOM.xlsx [-Sheet 1] [-Out out.tsv]
param(
  [Parameter(Mandatory = $true)][string]$Path,
  [int]$Sheet = 1,
  [string]$Out = ''
)

Add-Type -AssemblyName System.IO.Compression
$zip = [System.IO.Compression.ZipFile]::OpenRead((Resolve-Path $Path).ProviderPath)
try {
  function Read-Entry([string]$name) {
    $e = $zip.Entries | Where-Object { $_.FullName -eq $name }
    if (-not $e) { return $null }
    $sr = New-Object System.IO.StreamReader($e.Open(), [System.Text.Encoding]::UTF8)
    $t = $sr.ReadToEnd(); $sr.Dispose(); return $t
  }

  # 共享字符串表
  $shared = @()
  $ss = Read-Entry 'xl/sharedStrings.xml'
  if ($ss) {
    $xml = [xml]$ss
    $shared = @($xml.sst.si | ForEach-Object {
      if ($_.t) { [string]$_.t } else { ($_.r | ForEach-Object { $_.t }) -join '' }
    })
  }

  $sheetXml = Read-Entry "xl/worksheets/sheet$Sheet.xml"
  if (-not $sheetXml) { Write-Error "找不到 sheet$Sheet"; exit 1 }
  $sx = [xml]$sheetXml

  $lines = foreach ($row in $sx.worksheet.sheetData.row) {
    $cells = @{}
    foreach ($c in $row.c) {
      $col = ($c.r -replace '\d', '')
      $v = switch ($c.t) {
        's'      { $shared[[int]$c.v] }
        'inlineStr' { $c.is.t }
        default  { [string]$c.v }
      }
      $cells[$col] = if ($null -ne $v) { $v -replace "`t", ' ' } else { '' }
    }
    $max = ($cells.Keys | Sort-Object { [int][char]($_[0]) * 26 + [char]($_[-1]) } | Select-Object -Last 1)
    $order = $cells.Keys | Sort-Object { ([int][char]$_[0] - 65) * 26 + ($(if ($_.Length -gt 1) { [int][char]$_[1] - 65 } else { -65 })) }
    ($order | ForEach-Object { $cells[$_] }) -join "`t"
  }

  Write-Output ("行数: {0}   列: {1}" -f $lines.Count, (($lines[0] -split "`t").Count))
  if ($Out) {
    $lines -join "`r`n" | Out-File -Encoding utf8 -Path $Out
    Write-Output "已导出 TSV: $Out"
  } else {
    $lines
  }
} finally { $zip.Dispose() }
