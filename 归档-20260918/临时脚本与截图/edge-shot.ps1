Add-Type -AssemblyName System.Drawing
Add-Type @"
using System;
using System.Runtime.InteropServices;
public class WCap {
  [DllImport("user32.dll")] public static extern bool PrintWindow(IntPtr hWnd, IntPtr hdcBlt, uint nFlags);
  [DllImport("user32.dll")] public static extern bool GetWindowRect(IntPtr hWnd, out RECT r);
  [DllImport("user32.dll")] public static extern bool IsWindowVisible(IntPtr hWnd);
  [StructLayout(LayoutKind.Sequential)] public struct RECT { public int Left; public int Top; public int Right; public int Bottom; }
}
"@

$target = $null
Get-Process msedge -ErrorAction SilentlyContinue | ForEach-Object {
  if ($_.MainWindowHandle -ne 0 -and [WCap]::IsWindowVisible($_.MainWindowHandle)) {
    if ($null -eq $target -or $_.MainWindowTitle.Length -gt $target.MainWindowTitle.Length) { $target = $_ }
  }
}
if ($null -eq $target) { Write-Output "no visible msedge window"; exit }
Write-Output ("window: " + $target.MainWindowTitle)

$r = New-Object WCap+RECT
[void][WCap]::GetWindowRect($target.MainWindowHandle, [ref]$r)
$w = $r.Right - $r.Left; $h = $r.Bottom - $r.Top
Write-Output "size: ${w}x${h}"

$bmp = New-Object System.Drawing.Bitmap($w, $h)
$g = [System.Drawing.Graphics]::FromImage($bmp)
$hdc = $g.GetHdc()
# 2 = PW_RENDERFULLCONTENT (works for Chromium windows even when occluded)
[void][WCap]::PrintWindow($target.MainWindowHandle, $hdc, 2)
$g.ReleaseHdc($hdc)
$g.Dispose()
$bmp.Save("C:\Users\asus\Desktop\test17\_tmp\edge.png", [System.Drawing.Imaging.ImageFormat]::Png)
$bmp.Dispose()
Write-Output "saved edge.png"
