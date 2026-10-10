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
| 深度状态 `0x83` / 深度样本 `0x82` | 状态 `0x80`，序号 `0`，年龄 `-1 ms`，有效帧 `0`，聚合错误 `12170`；未收到 `0x82` | 深度探头尚未提供有效 ADC/PROM/压力/温度/深度样本 |

`0x80` 表示固定 P0 的 `ZERO_VALID` 位有效。它不能证明传感器采样或压力补偿有效。状态帧只上报聚合错误计数，当前还不能据此断定具体 I2C 故障。

## 安全边界

网关连接正常，CRC 错误为 0，`tx_enabled=false`、`motor_command_active=false`。后台启动时写出 76 字节、2 个安全停止帧；本轮没有发送任何电机运行命令，也没有执行深度归零。除上述只读检查外，没有改动现场状态。
