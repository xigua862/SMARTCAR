Add-Type -AssemblyName System.Drawing
$src = $args[0]
$out = $args[1]
$x = [int]$args[2]; $y = [int]$args[3]; $w = [int]$args[4]; $h = [int]$args[5]
$img = [System.Drawing.Image]::FromFile($src)
$rect = New-Object System.Drawing.Rectangle($x, $y, $w, $h)
$bmp = New-Object System.Drawing.Bitmap($w, $h)
$g = [System.Drawing.Graphics]::FromImage($bmp)
$g.DrawImage($img, (New-Object System.Drawing.Rectangle(0, 0, $w, $h)), $rect, [System.Drawing.GraphicsUnit]::Pixel)
$bmp.Save($out, [System.Drawing.Imaging.ImageFormat]::Png)
$g.Dispose(); $bmp.Dispose(); $img.Dispose()
Write-Output "cropped -> $out"
