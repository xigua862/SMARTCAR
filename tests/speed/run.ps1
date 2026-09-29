param([string]$Compiler = '')
$ErrorActionPreference = 'Stop'

# 从脚本位置推导项目路径，允许在任意工作目录调用。
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$buildRoot = Join-Path $projectRoot '_tmp/host-tests-speed'
New-Item -ItemType Directory -Path $buildRoot -Force | Out-Null

if (-not $Compiler) {
    $gccCommand = Get-Command gcc -ErrorAction SilentlyContinue
    if ($gccCommand) {
        $Compiler = $gccCommand.Source
    } elseif (Test-Path -LiteralPath 'C:/Program Files/RedPanda-Cpp/MinGW64/bin/gcc.exe') {
        $Compiler = 'C:/Program Files/RedPanda-Cpp/MinGW64/bin/gcc.exe'
    } else {
        throw '未找到宿主 GCC，请通过 -Compiler 指定 gcc.exe 的完整路径。'
    }
}
$Compiler = (Resolve-Path -LiteralPath $Compiler).Path
$testExecutable = Join-Path $buildRoot 'test_speed_and_trim.exe'
$compilerArguments = @(
    '-std=c99', '-Wall', '-Wextra', '-Werror',
    '-I', (Join-Path $PSScriptRoot 'mocks'),
    '-I', (Join-Path $projectRoot 'Core/App'),
    '-I', (Join-Path $projectRoot 'Core/App/drivers'),
    (Join-Path $PSScriptRoot 'test_speed.c'),
    (Join-Path $PSScriptRoot 'test_steering_trim.c'),
    (Join-Path $projectRoot 'Core/App/drivers/speed.c'),
    '-o', $testExecutable
)

# MinGW 的编译子进程和测试程序需要同目录 DLL；结束后恢复调用者环境。
$originalPath = $env:PATH
try {
    $env:PATH = (Split-Path -Parent $Compiler) + [IO.Path]::PathSeparator + $env:PATH
    & $Compiler @compilerArguments
    if ($LASTEXITCODE -ne 0) { throw "测试编译失败，退出码 $LASTEXITCODE" }
    & $testExecutable
    if ($LASTEXITCODE -ne 0) { throw "测试断言失败，退出码 $LASTEXITCODE" }
} finally {
    $env:PATH = $originalPath
}
