# AA5B TARGET=1 (IMU) 上位机测试脚本——真机验证协议文档用。
# 只读流 + 脚本化命令（GET/SET_PARAMETER 0001 速率往返、边界测试）；
# 不发送 CALIBRATE（会改设备校准状态，需单独授权）。
# 用法: powershell -NoProfile -ExecutionPolicy Bypass -File tests/aa5b_imu_test.ps1
param(
  [string]$Port = 'COM11',
  [int]$Seconds = 24,
  [string]$OutFile = 'aa5b_imu_test.bin',
  [string]$LogFile = 'aa5b_imu_test.log'
)
$ErrorActionPreference = 'Stop'

function Crc16Ccitt([byte[]]$d) {
  $crc = 0xFFFF
  foreach ($b in $d) {
    $crc = $crc -bxor ([int]$b -shl 8)
    for ($i = 0; $i -lt 8; $i++) {
      if (($crc -band 0x8000) -ne 0) { $crc = ((($crc -shl 1) -band 0xFFFF) -bxor 0x1021) }
      else { $crc = ($crc -shl 1) -band 0xFFFF }
    }
  }
  return $crc
}

function MakeFrame([byte]$cmd, [byte]$flags, [byte]$target, [uint32]$seq, [byte[]]$payload) {
  $len = $payload.Length
  $f = New-Object byte[] (20 + $len)
  $f[0] = 0xAA; $f[1] = 0x5B; $f[2] = 0x01; $f[3] = $cmd; $f[4] = $flags; $f[5] = $target
  [BitConverter]::GetBytes([uint32]$seq).CopyTo($f, 6)
  [BitConverter]::GetBytes([uint16]$len).CopyTo($f, 10)
  [BitConverter]::GetBytes([uint32]0).CopyTo($f, 12)
  if ($len -gt 0) { $payload.CopyTo($f, 16) }
  $crcIn = New-Object byte[] (15 + $len)
  [Array]::Copy($f, 1, $crcIn, 0, (15 + $len))
  $c = Crc16Ccitt $crcIn
  [BitConverter]::GetBytes([uint16]$c).CopyTo($f, 16 + $len)
  $f[18 + $len] = 0x5B; $f[19 + $len] = 0xAA
  return ,$f
}

# 测试序列：时间(ms) 名称 CMD FLAGS TARGET SEQ 载荷
$schedule = @(
  @{ t = 1000;  name = 'GET_INFO(seq1)';          cmd = 0x01; seq = 1; pl = @() }
  @{ t = 3000;  name = 'GET_STATUS(seq2)';        cmd = 0x02; seq = 2; pl = @() }
  @{ t = 5000;  name = 'GET_PARAM 0001(seq3)';    cmd = 0x03; seq = 3; pl = @(0x01,0x00) }
  @{ t = 7000;  name = 'SET_PARAM 0001=50(seq4)'; cmd = 0x04; seq = 4; pl = @(0x01,0x00,0x04,0x02,0x32,0x00) }
  @{ t = 11500; name = 'GET_PARAM 0001(seq5)';    cmd = 0x03; seq = 5; pl = @(0x01,0x00) }
  @{ t = 13500; name = 'SET_PARAM 0001=25(seq6)'; cmd = 0x04; seq = 6; pl = @(0x01,0x00,0x04,0x02,0x19,0x00) }
  @{ t = 15500; name = 'GET_PARAM 0003(seq7)';    cmd = 0x03; seq = 7; pl = @(0x03,0x00) }
  @{ t = 17500; name = 'GET_INFO again(seq8)';    cmd = 0x01; seq = 8; pl = @() }
  @{ t = 19500; name = 'GET_PARAM 9999(seq9)';    cmd = 0x03; seq = 9; pl = @(0x99,0x99) }
)

$sp = New-Object System.IO.Ports.SerialPort
$sp.PortName = $Port; $sp.BaudRate = 115200
$sp.Parity = [System.IO.Ports.Parity]::None; $sp.DataBits = 8
$sp.StopBits = [System.IO.Ports.StopBits]::One
$sp.Handshake = [System.IO.Ports.Handshake]::None
$sp.ReadTimeout = 50; $sp.ReadBufferSize = 65536
$sp.RtsEnable = $false; $sp.DtrEnable = $false
try { $sp.Open() } catch {
  Write-Output ("OPEN-FAILED: " + $_.Exception.GetType().FullName + " | " + $_.Exception.Message)
  exit 2
}
Write-Output ("OPENED: " + $Port + " 115200 8N1, 测试 " + $Seconds + "s")
$sw = [System.Diagnostics.Stopwatch]::StartNew()
$ms = New-Object System.IO.MemoryStream
$buf = New-Object byte[] 8192
$log = New-Object System.Collections.Generic.List[string]
$next = 0
$total = 0
try {
  while ($sw.Elapsed.TotalMilliseconds -lt ($Seconds * 1000)) {
    $n = $sp.BytesToRead
    if ($n -gt 0) {
      if ($n -gt $buf.Length) { $n = $buf.Length }
      $got = $sp.Read($buf, 0, $n)
      if ($got -gt 0) {
        $ms.Write($buf, 0, $got); $total += $got
        $log.Add(("[{0,7} ms] rx {1} B" -f [int]$sw.ElapsedMilliseconds, $got))
      }
    } else { Start-Sleep -Milliseconds 5 }
    if (($next -lt $schedule.Count) -and ($sw.ElapsedMilliseconds -ge $schedule[$next].t)) {
      $item = $schedule[$next]
      $frame = MakeFrame $item.cmd 0x01 0x01 $item.seq ([byte[]]$item.pl)
      $sp.Write($frame, 0, $frame.Length)
      $hex = ($frame | ForEach-Object { $_.ToString('X2') }) -join ' '
      $log.Add(("[{0,7} ms] TX {1}: {2}" -f [int]$sw.ElapsedMilliseconds, $item.name, $hex))
      Write-Output ("t=" + [int]$sw.ElapsedMilliseconds + "ms TX " + $item.name)
      $next++
    }
  }
} finally {
  if ($sp.IsOpen) { $sp.Close() }
}
[System.IO.File]::WriteAllBytes($OutFile, $ms.ToArray())
[System.IO.File]::WriteAllLines($LogFile, $log)
Write-Output ("CLOSED: " + $Port + " is-open=" + $sp.IsOpen)
Write-Output ("TOTAL-RX: " + $total + " B; log=" + (Resolve-Path $LogFile))
