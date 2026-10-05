# 编译并运行 MS5837 深度计主机单元测试（Windows，使用仓库现有 gcc，不安装任何工具）。
# 用法：powershell -ExecutionPolicy Bypass -File tests/run_ms5837_host_test.ps1
param(
  [string]$Gcc = "gcc"
)

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot

Push-Location $root
try {
  if (-not (Test-Path "build")) {
    New-Item -ItemType Directory "build" | Out-Null
  }
  Write-Host "==> $Gcc -std=c11 -Wall -Wextra -Werror -O0 -ICore/Inc tests/ms5837_host_test.c -o build/ms5837_host_test.exe"
  & $Gcc -std=c11 -Wall -Wextra -Werror -O0 -ICore/Inc tests/ms5837_host_test.c -o build/ms5837_host_test.exe
  if ($LASTEXITCODE -ne 0) {
    Write-Error "编译失败，退出码 $LASTEXITCODE"
    exit $LASTEXITCODE
  }
  Write-Host "==> build/ms5837_host_test.exe"
  & "./build/ms5837_host_test.exe"
  $code = $LASTEXITCODE
  if ($code -ne 0) {
    Write-Error "测试失败，退出码 $code"
  }
  exit $code
}
finally {
  Pop-Location
}
