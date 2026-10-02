# 从 .epru 里挖：图上文字标注 / 位号统计 / BOM（位号+器件+厂家料号）
# 用法:  pwsh -NoProfile -File 脚本\probe-epro2-bom.ps1 -Epru "<main.epru>" [-Top 40]
# 说明: .epru 是「一行一个图元」的 JSON 流，形如 {"type":"ATTR",...}||{...,"key":"K","value":"V",...}|
#       key / value 的先后顺序不固定，所以按图元切块后再分别取，别用一条正则硬摁顺序。
param(
  [Parameter(Mandatory = $true)][string]$Epru,
  [int]$Top = 40,
  [string]$Csv = ''
)

$txt = [System.IO.File]::ReadAllText($Epru)

# 按 {"type":"XXX" 切块，返回 [类型, 块内容]
function Split-Primitives([string]$text) {
  $re = [regex]'\{"type":"([A-Z_]+)","ticket"'
  $ms = $re.Matches($text)
  for ($i = 0; $i -lt $ms.Count; $i++) {
    $start = $ms[$i].Index
    $end = if ($i -lt $ms.Count - 1) { $ms[$i + 1].Index } else { $text.Length }
    , @($ms[$i].Groups[1].Value, $text.Substring($start, $end - $start))
  }
}

$prims = Split-Primitives $txt
Write-Output ("图元总数: " + $prims.Count)

$attrByKey = @{}
$textLabels = New-Object System.Collections.Generic.List[string]
foreach ($p in $prims) {
  if ($p[0] -eq 'ATTR') {
    $k = ([regex]::Match($p[1], '"key":"([^"]+)"')).Groups[1].Value
    $v = ([regex]::Match($p[1], '"value":"([^"]*)"')).Groups[1].Value
    if ($k) {
      if (-not $attrByKey.ContainsKey($k)) { $attrByKey[$k] = New-Object System.Collections.Generic.List[string] }
      $attrByKey[$k].Add($v)
    }
  }
  elseif ($p[0] -eq 'TEXT') {
    $v = ([regex]::Match($p[1], '"value":"([^"]+)"')).Groups[1].Value
    if ($v -match '[\u4e00-\u9fff]') { $textLabels.Add($v) }
  }
}

Write-Output "`n=== 图上中文标注（板块名 / 说明）==="
$textLabels | Group-Object | Sort-Object Count -Descending | Select-Object -First $Top |
  ForEach-Object { "{0,4}  {1}" -f $_.Count, $_.Name }

Write-Output "`n=== 位号前缀统计 ==="
$attrByKey['Designator'] | Where-Object { $_ -match '^[A-Z]+\d' } |
  ForEach-Object { $_ -replace '\d+$', '' } | Group-Object | Sort-Object Count -Descending |
  ForEach-Object { "{0,5}  {1}" -f $_.Count, $_.Name }

Write-Output "`n=== Device 清单（器件名，出现次数）==="
$attrByKey['Device'] | Where-Object { $_ } | Group-Object | Sort-Object Count -Descending |
  ForEach-Object { "{0,4}  {1}" -f $_.Count, $_.Name }

Write-Output "`n=== Manufacturer Part（厂家料号）==="
$attrByKey['Manufacturer Part'] | Where-Object { $_ } | Group-Object | Sort-Object Count -Descending |
  ForEach-Object { "{0,4}  {1}" -f $_.Count, $_.Name }

Write-Output "`n=== 网络名 NET ==="
$attrByKey['NET'] | Where-Object { $_ } | Group-Object | Sort-Object Count -Descending | Select-Object -First 60 |
  ForEach-Object { "{0,4}  {1}" -f $_.Count, $_.Name }

# ---- 按 parentId 聚合出真 BOM（位号 / 器件 / 厂家料号 / 封装），顺便落 CSV ----
$rows = @{}
foreach ($p in $prims) {
  if ($p[0] -ne 'ATTR') { continue }
  $pid_ = ([regex]::Match($p[1], '"parentId":"([0-9a-f]{8,})"')).Groups[1].Value
  if (-not $pid_) { continue }
  $k = ([regex]::Match($p[1], '"key":"([^"]+)"')).Groups[1].Value
  $v = ([regex]::Match($p[1], '"value":"([^"]*)"')).Groups[1].Value
  if (-not $k) { continue }
  if (-not $rows.ContainsKey($pid_)) { $rows[$pid_] = @{} }
  switch ($k) {
    'Designator'        { $rows[$pid_]['DES'] = $v }
    'Manufacturer Part' { $rows[$pid_]['MPN'] = $v }
    'Supplier Part'     { $rows[$pid_]['C'] = $v }
    'Footprint'         { $rows[$pid_]['Footprint'] = $v }
    'Name'              { if (-not $rows[$pid_]['Name']) { $rows[$pid_]['Name'] = $v } }
  }
}

$bom = $rows.Values | Where-Object { $_.DES -and ($_.DES -match '^[A-Z]+\d') } |
  Select-Object @{n='DES';e={$_.DES}}, @{n='Name';e={$_.Name}}, @{n='MPN';e={$_.MPN}}, @{n='LCSC';e={$_.C}}, @{n='Footprint';e={$_.Footprint}} |
  Sort-Object DES

Write-Output ("`n=== 聚合到 " + $bom.Count + " 个带位号的元件，按前缀分组 ===")
foreach ($pre in 'U', 'Q', 'D', 'L', 'SW', 'H', 'P', 'TP') {
  $g = $bom | Where-Object { $_.DES -match "^$pre\d" }
  if ($g) {
    Write-Output "`n--- $pre ($($g.Count)) ---"
    $g | ForEach-Object { "  {0,-6} {1,-26} {2,-26} {3}" -f $_.DES, $_.Name, $_.MPN, $_.Footprint }
  }
}

if ($Csv) {
  $bom | Export-Csv -NoTypeInformation -Encoding UTF8 -Path $Csv
  Write-Output "`nBOM 已导出: $Csv"
}
