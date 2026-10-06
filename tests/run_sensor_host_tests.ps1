param([string]$Gcc='gcc')
$ErrorActionPreference='Stop'
$root=Split-Path -Parent $PSScriptRoot
Push-Location $root
try {
    New-Item -ItemType Directory -Force build | Out-Null
    $cases=@(
        @{Name='imu_sensor';Sources=@('tests/imu_sensor_host_test.c')},
        @{Name='ms5837';Sources=@('tests/ms5837_host_test.c')},
        @{Name='sensor_protocol';Sources=@('tests/sensor_protocol_host_test.c','Core/Src/sensor_protocol.c')},
        @{Name='sensor_service';Sources=@('tests/sensor_service_host_test.c','Core/Src/sensor_service.c','Core/Src/sensor_protocol.c','Core/Src/imu_sensor.c','Core/Src/ms5837.c','Core/Src/sensor_i2c_bus.c')},
        @{Name='usb_sensor_route';Sources=@('tests/usb_sensor_route_host_test.c')},
        @{Name='firmware_flow';Sources=@('tests/firmware_flow_host_test.c')}
    )
    foreach($case in $cases) {
        $exe='build/'+$case.Name+'_host_test.exe'
        $arguments=@('-std=c11','-Wall','-Wextra','-Werror','-O0','-ICore/Inc','-IUSB_DEVICE/App')
        if($case.Name -eq 'sensor_service'){$arguments+='-DI2C_HOST_TEST=1'}
        Write-Host ('=== '+$case.Name+' ===')
        & $Gcc @arguments @($case.Sources) '-lm' '-o' $exe
        if($LASTEXITCODE -ne 0){throw ('Compile failed: '+$case.Name)}
        & (Join-Path $root $exe)
        if($LASTEXITCODE -ne 0){throw ('Test failed: '+$case.Name)}
    }
    Write-Host 'All six host test executables passed; no physical device was opened.'
} finally {Pop-Location}
