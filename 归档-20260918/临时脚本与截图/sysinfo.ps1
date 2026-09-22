Write-Host "=== Battery info (WMI) ==="
try { Get-CimInstance -Namespace root\WMI -ClassName BatteryStaticData -ErrorAction Stop | ForEach-Object { "DesignedCapacity  = $($_.DesignedCapacity) mWh" } } catch { "BatteryStaticData unavailable" }
try { Get-CimInstance -Namespace root\WMI -ClassName BatteryFullChargedCapacity -ErrorAction Stop | ForEach-Object { "FullChargedCap    = $($_.FullChargedCapacity) mWh" } } catch { "BatteryFullChargedCapacity unavailable" }
try { Get-CimInstance -Namespace root\WMI -ClassName BatteryCycleCount -ErrorAction Stop | ForEach-Object { "CycleCount        = $($_.CycleCount)" } } catch { "CycleCount unavailable" }
try { Get-CimInstance -ClassName Win32_Battery -ErrorAction Stop | ForEach-Object { "Status/Charge     = $($_.BatteryStatus) / $($_.EstimatedChargeRemaining)%" } } catch { "Win32_Battery unavailable" }

Write-Host ""
Write-Host "=== Sleep timeout (0 = never) ==="
powercfg /q SCHEME_CURRENT 238c9fa8-0aad-41ed-83f4-97be242c8f20 29f6c1db-86da-48c5-9fdb-f2b67b1f44da |
  Select-String -Pattern "当前|Current|Index|索引" | Select-Object -First 4
Write-Host "=== Display-off timeout (0 = never) ==="
powercfg /q SCHEME_CURRENT 7516b95f-f776-4464-8c53-06167f40cc99 3c0bc021-c8a8-4e07-a973-6b14cbcb2b7e |
  Select-String -Pattern "当前|Current|Index|索引" | Select-Object -First 4
Write-Host "=== Lid close action ==="
powercfg /q SCHEME_CURRENT 4f971e89-eebd-4455-a8de-9e59040e7347 5ca83367-6e45-459f-a27b-476b1d01c936 |
  Select-String -Pattern "当前|Current|Index|索引" | Select-Object -First 4
