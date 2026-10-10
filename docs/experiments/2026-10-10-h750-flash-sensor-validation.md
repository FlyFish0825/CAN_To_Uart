# 2026-10-10 H750 烧录与传感器实机记录

## 烧录证据

用户确认 H750 的 RESET/NRST 已松开、供电稳定后，使用 WCH-Link CMSIS-DAP 经 OpenOCD 将 Debug 固件写入 STM32H750。完整记录保存在：

`build/p0_air_reference_20261010/openocd_flash_retry_20261010.log`

日志顺序为 `Programming Started`、`Programming Finished`、`Verify Started`、`Verified OK`、`Resetting Target`，OpenOCD 正常退出。烧录镜像 SHA-256：

| 镜像 | SHA-256 |
|---|---|
| `CAN_To_Uart.elf` | `72E5592FCC750AE207FCAEED9F221399AA2975ABE25D4AB5B0B5D76F728561EB` |
| `CAN_To_Uart.bin` | `0293849DAE9F97356FB347DBC382C26836984A69238557D3072FB9E26CDCCB11` |
| `CAN_To_Uart.hex` | `C6604B78EC00A548049A94D446C0FC84349E8EBEFDEF83E9069FC070F7783643` |

本次固件固定使用 MS5837-02BA，并以 `100965 Pa` 作为启动空气参考 P0。没有发送深度归零命令。

## WCH-Link 拔下后的实时遥测

WCH-Link 从控制板拔下后，通过 Nano 上正在运行的 ROS 2 后台读取真实设备数据：

| 分组 | 实测状态 | 结论 |
|---|---|---|
| IMU `0x80` | `fresh=true`、`raw_valid=true`；加速度约 `(1.106, -0.129, -9.740) m/s²` | 原始 IMU 已恢复并持续上报 |
| IMU `0x81` | `quaternion_valid=true`、`euler_valid=true` | 姿态数据有效 |
| IMU 气压 `0x85` | 四项有效；压力 `101605.68 Pa`、温度 `28.69 °C`、高度 `14.52 m`、参考压力 `101780.80 Pa` | 气压遥测有效；此高度不是水深 |
| 深度状态 `0x83` / 深度样本 `0x82` | 状态 `0x80`，序号 `0`，年龄 `-1 ms`，有效帧 `0`，聚合错误 `12170`；未收到 `0x82` | 用户随后确认该测试时深度计探头没有接上；无样本与离线状态符合现场接线状态，不能据此认定驱动故障 |

`0x80` 表示固定 P0 的 `ZERO_VALID` 位有效。它不能证明传感器采样或压力补偿有效。本次没有接入深度计探头，因此尚未验证接回后的 D1/D2、PROM CRC 与压力补偿。

## 安全边界

网关连接正常，CRC 错误为 0，`tx_enabled=false`、`motor_command_active=false`。后台启动时写出 76 字节、2 个安全停止帧；本轮没有发送任何电机运行命令，也没有执行深度归零。除上述只读检查外，没有改动现场状态。

## 深度计错误诊断固件准备（尚未烧录）

现场旧状态帧只有累计错误数，无法区分 I²C 超时、NACK/总线错误或 PROM CRC。为保留 20 字节 `0x83` 状态布局，驱动现把最近关联的 I²C 命令放入状态字 `[23:16]`，把最近 `Ms5837Result_t` 错误码放入 `[31:24]`。GUI 状态区会显示错误码、文本和命令字节；错误码 4/5/6 分别表示超时、I²C/未应答、PROM CRC。

该诊断版已在独立目录 `build/p0_air_diagnostics_20261010/` 完成 ARM Debug 构建（57 个编译/链接步骤）和镜像导出；MS5837 主机回归 **34 项**通过，GUI 完整 CTest **21/21** 通过。镜像 SHA-256：

| 镜像 | SHA-256 |
|---|---|
| `CAN_To_Uart.elf` | `E5DC68499FB952237C93496C3B6257811F5C2F5053BA4F43B1770DC349ED40E6` |
| `CAN_To_Uart.bin` | `2A314A748F614B416993ECD4E4E4FAEEABE4E522B8B3968CCCE529274B533BEB` |
| `CAN_To_Uart.hex` | `59647655300553A5C1D8F2D9EDAD8390A11B874477DB195675FEC4429E4B516D` |

这份诊断固件尚未烧录，当前活动后台和实机仍运行上一版已验证固件。下一次写入会短暂复位 H750 并断开传感器数据；读取到错误码后再按证据修复，不会自动校零或发送电机运行命令。
