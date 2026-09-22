Add-Type -AssemblyName System.Windows.Forms
Add-Type -AssemblyName System.Drawing
Add-Type @"
using System;
using System.Runtime.InteropServices;
public class Cur {
  [DllImport("user32.dll")] public static extern bool SetCursorPos(int x, int y);
}
"@
# move pointer onto the video area so the player shows its control bar (no click)
[Cur]::SetCursorPos(900, 560) | Out-Null
Start-Sleep -Milliseconds 1500
$vs = [System.Windows.Forms.SystemInformation]::VirtualScreen
$bmp = New-Object System.Drawing.Bitmap($vs.Width, $vs.Height)
$g = [System.Drawing.Graphics]::FromImage($bmp)
$g.CopyFromScreen($vs.X, $vs.Y, 0, 0, $bmp.Size)
$bmp.Save("C:\Users\asus\Desktop\test17\_tmp\screen2.png", [System.Drawing.Imaging.ImageFormat]::Png)
$g.Dispose(); $bmp.Dispose()
Write-Output "hovered + saved"
