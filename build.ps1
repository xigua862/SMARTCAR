param([string]$Keil = 'C:/keil5/UV4/UV4.exe')
$ErrorActionPreference = 'Stop'
$project = Join-Path $PSScriptRoot 'MDK-ARM/test16.uvprojx'
$log = Join-Path $PSScriptRoot 'MDK-ARM/build-0929-R1.log'
if (!(Test-Path -LiteralPath $Keil)) { throw "未找到 Keil：$Keil" }
# 只重编译，不烧录；隐藏构建窗口并检查真实进程退出码。
$process = Start-Process -FilePath $Keil -ArgumentList @('-r', ('"' + $project + '"'), '-j0', '-o', ('"' + $log + '"')) -WindowStyle Hidden -Wait -PassThru
Get-Content -LiteralPath $log -Tail 8
if ($process.ExitCode -ne 0) { throw "Keil 构建失败或有警告，退出码：$($process.ExitCode)" }
