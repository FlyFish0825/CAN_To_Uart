# 只读采集 WCH-Link SERIAL (COM42) 的 IMU 原始字节。
# 约束：绝不 Write、不校准、不改模式/频率、不复位/烧录；DTR/RTS 关闭。
# 用法: powershell -NoProfile -ExecutionPolicy Bypass -File capture_com42.ps1
param(
  [string]$Port = 'COM42',
  [int]$Baud = 115200,
  [int]$Seconds = 10,
  [string]$OutFile = 'imu_com42_10s.bin'
)

$ErrorActionPreference = 'Stop'
Write-Output ("ports: " + ([System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object) -join ' ')

$sp = New-Object System.IO.Ports.SerialPort
$sp.PortName = $Port
$sp.BaudRate = $Baud
$sp.Parity = [System.IO.Ports.Parity]::None
$sp.DataBits = 8
$sp.StopBits = [System.IO.Ports.StopBits]::One
$sp.Handshake = [System.IO.Ports.Handshake]::None
$sp.ReadTimeout = 100
$sp.WriteTimeout = 100
$sp.ReadBufferSize = 65536
$sp.RtsEnable = $false
$sp.DtrEnable = $false

try {
  $sp.Open()
} catch {
  Write-Output ("OPEN-FAILED: " + $_.Exception.GetType().FullName)
  Write-Output ("OPEN-FAILED-MESSAGE: " + $_.Exception.Message)
  if ($_.Exception.InnerException) {
    Write-Output ("OPEN-FAILED-INNER: " + $_.Exception.InnerException.Message)
  }
  exit 2
}

Write-Output ("OPENED: " + $Port + " " + $Baud + " 8N1, RX-only, " + $Seconds + "s")

$sw = [System.Diagnostics.Stopwatch]::StartNew()
$ms = New-Object System.IO.MemoryStream
$buf = New-Object byte[] 4096
$nextReport = 1
$total = 0

try {
  while ($sw.Elapsed.TotalSeconds -lt $Seconds) {
    $n = $sp.BytesToRead
    if ($n -gt 0) {
      if ($n -gt $buf.Length) { $n = $buf.Length }
      $got = $sp.Read($buf, 0, $n)
      if ($got -gt 0) {
        $ms.Write($buf, 0, $got)
        $total += $got
      }
    } else {
      Start-Sleep -Milliseconds 10
    }
    if ($sw.Elapsed.TotalSeconds -ge $nextReport) {
      Write-Output ("t=" + [int]$sw.Elapsed.TotalSeconds + "s bytes=" + $total)
      $nextReport++
    }
  }
} finally {
  if ($sp.IsOpen) { $sp.Close() }
}

$data = $ms.ToArray()
[System.IO.File]::WriteAllBytes($OutFile, $data)
Write-Output ("CLOSED: " + $Port + " is-open=" + $sp.IsOpen)
Write-Output ("TOTAL-BYTES: " + $total)
Write-Output ("SAVED: " + (Resolve-Path $OutFile))
