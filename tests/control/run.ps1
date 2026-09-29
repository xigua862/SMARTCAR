$ErrorActionPreference = 'Stop'
$projectRoot = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
$buildRoot = Join-Path $projectRoot '_tmp/host-tests-control'
New-Item -ItemType Directory -Path $buildRoot -Force | Out-Null
$compilerDirectory = 'C:/Program Files/RedPanda-Cpp/MinGW64/bin'
$savedPath = $env:PATH
try {
    $env:PATH = "$compilerDirectory;$savedPath"
    foreach ($legacy in 0, 1) {
        $output = Join-Path $buildRoot "test_line_follow_$legacy.exe"
        & (Join-Path $compilerDirectory 'gcc.exe') '-std=c99' '-O0' '-Wall' '-Wextra' `
            "-DTEST_LEGACY_TRIM=$legacy" `
            '-I' (Join-Path $PSScriptRoot 'mocks') `
            '-I' (Join-Path $projectRoot 'Core/App') `
            (Join-Path $PSScriptRoot 'test_line_follow.c') `
            (Join-Path $projectRoot 'Core/App/control/filter.c') `
            (Join-Path $projectRoot 'Core/App/control/pid.c') `
            '-o' $output
        if ($LASTEXITCODE -ne 0) { throw 'Control regression compilation failed.' }
        & $output
        if ($LASTEXITCODE -ne 0) { throw 'Control regression test failed.' }
    }
} finally {
    $env:PATH = $savedPath
}
