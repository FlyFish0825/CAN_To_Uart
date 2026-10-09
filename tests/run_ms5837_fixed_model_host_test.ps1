param(
  [string]$Gcc = "gcc"
)

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$buildDir = Join-Path $root "build\test_fixed_model"
$exe = Join-Path $buildDir "ms5837_fixed_model_host_test.exe"
New-Item -ItemType Directory -Force -Path $buildDir | Out-Null

$includeArgs = @(
  "-I$(Join-Path $root 'Core\Inc')",
  "-I$(Join-Path $root 'Drivers\STM32H7xx_HAL_Driver\Inc')",
  "-I$(Join-Path $root 'Drivers\STM32H7xx_HAL_Driver\Inc\Legacy')",
  "-I$(Join-Path $root 'Drivers\CMSIS\Device\ST\STM32H7xx\Include')",
  "-I$(Join-Path $root 'Drivers\CMSIS\Include')"
)
$sources = @(
  (Join-Path $PSScriptRoot "ms5837_fixed_model_host_test.c")
)

& $Gcc -std=c11 -Wall -Wextra -Werror -Wno-int-to-pointer-cast -Wno-pointer-to-int-cast `
  -DI2C_BUS_ENABLE_NVIC=0 -DUSE_HAL_DRIVER -DSTM32H750xx @includeArgs @sources -o $exe
if ($LASTEXITCODE -ne 0) { throw "MS5837 fixed-model host test compilation failed ($LASTEXITCODE)." }

& $exe
if ($LASTEXITCODE -ne 0) { throw "MS5837 fixed-model host test failed ($LASTEXITCODE)." }
