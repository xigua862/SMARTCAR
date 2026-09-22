# ASCII-only helper: find tune.ps1, add UTF-8 BOM, verify parse, test Parse-IR
$ErrorActionPreference = 'Stop'

$p = (Get-ChildItem 'C:\Users\asus\Desktop\test17' -Recurse -Filter 'tune.ps1' -File |
      Where-Object { $_.DirectoryName -notlike '*_tmp*' } | Select-Object -First 1).FullName
Write-Host "target: $p"

$bytes = [System.IO.File]::ReadAllBytes($p)
$hasBom = ($bytes.Length -ge 3 -and $bytes[0] -eq 0xEF -and $bytes[1] -eq 0xBB -and $bytes[2] -eq 0xBF)
Write-Host "had BOM: $hasBom"

$text = [System.IO.File]::ReadAllText($p, [System.Text.Encoding]::UTF8)
[System.IO.File]::WriteAllText($p, $text, (New-Object System.Text.UTF8Encoding($true)))
Write-Host "rewritten with UTF-8 BOM"

# 1) parse check
$errs = $null
[void][System.Management.Automation.Language.Parser]::ParseFile($p, [ref]$null, [ref]$errs)
if ($errs.Count -eq 0) { Write-Host "PARSE OK" -ForegroundColor Green }
else { Write-Host "PARSE ERRORS: $($errs.Count)" -ForegroundColor Red; $errs | ForEach-Object { Write-Host "  $($_.Extent.StartLineNumber): $($_.Message)" } }

# 2) extract Parse-IR and run cases
$m = [regex]::Match($text, '(?s)\r?\nfunction Parse-IR.*?\r?\n\}')
if (-not $m.Success) { Write-Host "cannot extract Parse-IR" -ForegroundColor Red; exit 1 }
Invoke-Expression $m.Value

$cases = @(
  @{ line = "IR:00010000 RPM1=12 RPM2=13 KP=8.0 KD=3.0 SP=55"; want = -1 },
  @{ line = "IR:00000001 RPM1=40 RPM2=41 KP=8.0 KD=3.0 SP=55"; want = 4 },
  @{ line = "IR:01000100 RPM1=-5 RPM2=6 KP=10.0 KD=2.5 SP=40"; want = -1 },
  @{ line = "IR:00000000 RPM1=0 RPM2=0 KP=8.0 KD=3.0 SP=55";   want = 0 },
  @{ line = "IR:11111111 RPM1=0 RPM2=0 KP=8.0 KD=3.0 SP=55";   want = 0 },
  @{ line = "IR:00100 RPM1=10 RPM2=10 KP=12.0 KD=1.0 SP=35";   want = 0 },
  @{ line = "IR:10000 RPM1=10 RPM2=10 KP=12.0 KD=1.0 SP=35";   want = -2 },
  @{ line = "IR:01100 RPM1=10 RPM2=10 KP=12.0 KD=1.0 SP=35";   want = -1 }
)
$pass = 0
foreach ($c in $cases) {
  $d = Parse-IR $c.line
  if ($null -eq $d) { Write-Host "[NULL] $($c.line)" -ForegroundColor Red; continue }
  $ok = ($d.Error -eq $c.want)
  if ($ok) { $pass++ }
  $tag = if ($ok) { "ok  " } else { "FAIL" }
  Write-Host ("[{0}] IR={1,-9} E={2,3} (want {3,3})  KP={4} KD={5} SP={6} RPM={7}/{8}" -f $tag, $d.IR, $d.Error, $c.want, $d.KP, $d.KD, $d.SP, $d.RPM1, $d.RPM2)
}
Write-Host "garbage line -> " -NoNewline
$g = Parse-IR "IR:xx RPM1=1 RPM2=1 KP=1.0 KD=1.0 SP=1"
Write-Host $(if ($null -eq $g) { "ignored (ok)" } else { "NOT ignored (FAIL)" })

Write-Host "passed $pass / $($cases.Count)" -ForegroundColor $(if ($pass -eq $cases.Count) { 'Green' } else { 'Red' })
