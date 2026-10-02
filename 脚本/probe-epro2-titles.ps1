# 从 .epru（嘉立创EDA专业版工程解出的 JSON 明文）里挖：标题栏 / 图页名 / BOM / 网络名
# 用法:  pwsh -NoProfile -File 脚本\probe-epro2-titles.ps1 -Epru "<解出来的 main.epru 路径>"
param([Parameter(Mandatory = $true)][string]$Epru)

$txt = [System.IO.File]::ReadAllText($Epru)

Write-Output "=== 标题栏属性 (key: value) —— 板名 / 图页名 都在这 ==="
[regex]::Matches($txt, '"key":"(板子|图页|原理图|绘制|创建日期|更新日期|公司|版本|标题|Title|Page|Sheet)"[^}]*?"value":"([^"]*)"') |
  ForEach-Object { "{0,-8} {1}" -f $_.Groups[1].Value, $_.Groups[2].Value } |
  Group-Object | Sort-Object Count -Descending | ForEach-Object { "{0,3} x {1}" -f $_.Count, $_.Name }

Write-Output "`n=== 所有 ATTR 的 key 排行（看有哪些字段可挖）==="
[regex]::Matches($txt, '"key":"([^"]{1,30})"') | ForEach-Object { $_.Groups[1].Value } |
  Group-Object | Sort-Object Count -Descending | Select-Object -First 40 |
  ForEach-Object { "{0,5}  {1}" -f $_.Count, $_.Name }

Write-Output "`n=== COMPONENT 原始样本 1 条（学 schema 用）==="
$s = $txt.IndexOf('"type":"COMPONENT"')
if ($s -ge 0) { $txt.Substring([Math]::Max(0, $s - 40), [Math]::Min(1600, $txt.Length - $s)) }

Write-Output "`n=== 网络名 (NET 的 id) ==="
[regex]::Matches($txt, '"type":"NET","ticket":\d+,"id":"([^"]{1,60})"') | ForEach-Object { $_.Groups[1].Value } |
  Group-Object | Sort-Object Count -Descending | Select-Object -First 60 |
  ForEach-Object { "{0,4}  {1}" -f $_.Count, $_.Name }
