# 读嘉立创EDA专业版工程（.epro2 / .epru）的体检脚本
# 用法:  pwsh -NoProfile -File 脚本\probe-epro2.ps1 -Epro2 "C:\path\to\xxx.epro2"
param(
  [Parameter(Mandatory = $true)][string]$Epro2,
  [string]$WorkDir = (Join-Path $env:TEMP ('epro2_probe_' + [guid]::NewGuid().ToString('N').Substring(0, 6)))
)

Add-Type -AssemblyName System.IO.Compression.FileSystem
New-Item -ItemType Directory -Force -Path $WorkDir | Out-Null

$zip = [System.IO.Compression.ZipFile]::OpenRead($Epro2)
Write-Output "=== 压缩包条目总数: $($zip.Entries.Count) ==="
$zip.Entries | Where-Object { $_.FullName -notlike 'IMAGE/*' } | ForEach-Object { "{0,9} B  {1}" -f $_.Length, $_.FullName }

$main = $zip.Entries | Where-Object { $_.Name -like '*.epru' } | Sort-Object Length -Descending | Select-Object -First 1
if (-not $main) { Write-Output '!! 没找到 .epru 主文件'; $zip.Dispose(); exit 1 }
$epru = Join-Path $WorkDir 'main.epru'
[System.IO.Compression.ZipFileExtensions]::ExtractToFile($main, $epru, $true)
Write-Output "`n主文件: $($main.FullName)  ($($main.Length) B)  ->  已解到 $epru"
$zip.Dispose()

$txt = [System.IO.File]::ReadAllText($epru)
Write-Output ("字符数: " + $txt.Length)

Write-Output "`n=== 文档类型统计 (docType) ==="
[regex]::Matches($txt, '"docType":"([A-Za-z_]+)"') |
  Group-Object { $_.Groups[1].Value } | Sort-Object Count -Descending |
  ForEach-Object { "{0,6}  {1}" -f $_.Count, $_.Name }

Write-Output "`n=== 图元类型统计 (type) ==="
[regex]::Matches($txt, '"type":"([A-Z_]+)"') |
  Group-Object { $_.Groups[1].Value } | Sort-Object Count -Descending | Select-Object -First 25 |
  ForEach-Object { "{0,6}  {1}" -f $_.Count, $_.Name }

Write-Output "`n=== 作者 (立创账号) ==="
[regex]::Matches($txt, '"username":"([^"]+)"') | ForEach-Object { $_.Groups[1].Value } |
  Group-Object | Sort-Object Count -Descending | ForEach-Object { "{0,5}  {1}" -f $_.Count, $_.Name }

Write-Output "`n=== 改动时间跨度 ==="
$ts = [regex]::Matches($txt, '"updateTime":(\d{13})') | ForEach-Object { [int64]$_.Groups[1].Value } | Sort-Object
if ($ts.Count) {
  Write-Output ("最早: " + ([DateTimeOffset]::FromUnixTimeMilliseconds($ts[0]).LocalDateTime))
  Write-Output ("最晚: " + ([DateTimeOffset]::FromUnixTimeMilliseconds($ts[-1]).LocalDateTime))
}

Write-Output "`n=== 元件/器件名 直方图 (PART / META title) ==="
[regex]::Matches($txt, '"type":"PART","ticket":\d+,"id":"([^"]{2,60})"') | ForEach-Object { $_.Groups[1].Value -replace '\.\d+$', '' } |
  Group-Object | Sort-Object Count -Descending | Select-Object -First 60 |
  ForEach-Object { "{0,4}  {1}" -f $_.Count, $_.Name }

Write-Output "`n=== 关键字命中 ==="
$keys = 'STC32G', 'STC8H', 'STM32', 'TC264', 'GD32', '飞檐走壁', '三合一', '负压', '风机', '8520', '8523',
'无感', 'FOC', '三相', '霍尔', 'Hall', '编码器', 'MT68', 'TLE50', 'AS56', 'DRV8701', 'DRV83', 'IR210', 'EG31',
'EG21', 'BTS7960', 'FD6288', 'UCC27', '光耦', 'MP1584', 'TPS54', 'SY8089', 'ME6211', 'AMS1117', 'SGM2', 'XT30',
'CAN', 'LIN', '电磁', '电感', '摄像头', 'OV77', 'OV26', '图传', 'OLED', '红外', '循迹', '灰度', '陀螺', 'ICM2',
'BMI', 'MPU', '蜂鸣', '按键', 'COMPO', 'CMP0', 'CMP1', 'PWM0P', 'PWM0L', '预驱', '栅极', '续流', '死区'
foreach ($k in $keys) {
  $c = ([regex]::Matches($txt, [regex]::Escape($k))).Count
  if ($c -gt 0) { "{0,6}  {1}" -f $c, $k }
}
