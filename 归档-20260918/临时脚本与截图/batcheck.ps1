Write-Host "=== Battery status ==="
Get-CimInstance Win32_Battery | ForEach-Object {
  "BatteryStatus (1=discharging, 2=on AC) = $($_.BatteryStatus)"
  "Charge remaining = $($_.EstimatedChargeRemaining) %"
  "Estimated runtime on battery = $($_.EstimatedRunTime) minutes"
}

function Show-Setting($guid, $title) {
  Write-Host "--- $title ---"
  $out = powercfg /q SCHEME_CURRENT e73a048d-bf27-4f12-9731-8b2076e8891f $guid 2>&1
  $out | Select-String -Pattern "当前交流|当前直流" | ForEach-Object { $_.Line.Trim() }
}

Show-Setting 8183ba9a-e910-48da-8769-14ae6dc1170a "LOW battery level"
Show-Setting d8742dcb-3e6a-4b3c-b3fe-374623cdcf06 "LOW action (0=none 1=sleep 2=hibernate 3=shutdown)"
Show-Setting 9a66d8d7-4ff7-4ef9-b5a2-5a326ca2a469 "CRITICAL battery level"
Show-Setting 637ea02f-bbcb-4015-8e2c-a1c7b9c0b546 "CRITICAL action (0=none 1=sleep 2=hibernate 3=shutdown)"

Write-Host ""
Write-Host "=== Available sleep states ==="
powercfg /a | Select-Object -First 14
