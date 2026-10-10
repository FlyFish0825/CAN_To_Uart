# 2026-10-10 H750 烧录与传感器实机记录

**最终结果：** 深度计已修复并通过实机持续采样；WCH-Link 拔下后 IMU 原始、姿态、气压与深度同时有效。以下是各排障阶段的证据，最终修复见末节。

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

用户随后接回深度计探头。5 秒间隔的两次只读状态仍为 `device_status=0x80`、序号 `0`、有效帧 `0`、错误计数 `30185`；其间 Nano 串口接收累计增长、CRC 错误保持 0，IMU 气压继续有效。深度计依然没有上报，现有固件不含状态机/I²C 阶段字段，无法判断驱动当时卡在哪一步。

## 安全边界

网关连接正常，CRC 错误为 0，`tx_enabled=false`、`motor_command_active=false`。后台启动时写出 76 字节、2 个安全停止帧；本轮没有发送任何电机运行命令，也没有执行深度归零。除上述只读检查外，没有改动现场状态。

## 深度计阶段与错误诊断固件烧录及实测

探头接回后仍没有新样本，错误计数也停止增长，旧状态帧无法判断驱动停在哪一步。为保留 20 字节 `0x83` 状态布局，驱动把状态机阶段放入 `[12:10]`、bit13 作为扩展有效标记、I²C 阶段放入 `[15:14]`，再把最近关联的 I²C 命令放入 `[23:16]`、最近 `Ms5837Result_t` 错误码放入 `[31:24]`。GUI 状态区会显示状态机阶段、I²C 阶段、错误码、文本和命令字节；错误码 4/5/6 分别表示超时、I²C/未应答、PROM CRC。

该诊断版在独立目录 `build/p0_air_diagnostics_state_20261010/` 完成 ARM Debug 构建（57 个编译/链接步骤）和镜像导出；MS5837 主机回归 **40 项**通过，GUI 完整 CTest **21/21** 通过。镜像 SHA-256：

| 镜像 | SHA-256 |
|---|---|
| `CAN_To_Uart.elf` | `4E03231D4574FC2539131D6F612B88D88EFEAEA1BBFC848A35C076D3AFC77C30` |
| `CAN_To_Uart.bin` | `A6603736606D7DA2859242C1AC4E68B203DAEA7340A8D033CE81028DD881BBDC` |
| `CAN_To_Uart.hex` | `CD427D7EE55AF08DB88BF1313C215061673C620DCD958037CB7DF075842B3DF5` |

用户暂停现场测试并接回 WCH-Link 后，诊断 ELF 已写入 H750；OpenOCD 记录在 `build/p0_air_diagnostics_state_20261010/openocd_flash_diagnostics_20261010.log`，包含 `Programming Finished`、`Verified OK`、`Resetting Target`。Nano 后台进程保持运行并重新打开串口。

接回探头后的实时状态为 `device_status=0x05A02C80`，`sample_sequence=0`、`sample_age_ms=-1`、`good_frames=0`。解码结果：固定 P0 有效；诊断字段有效；驱动阶段 `3=IDLE`；I²C 阶段 `0=IDLE`；最近关联命令 `0xA0`（PROM 第一个字）；最近错误 `5=MS5837_ERR_IO`。错误计数从 178 增至 418，随后增至 2258；期间没有收到 `0x82`。这表示 PROM 读取事务反复以 I²C 错误返回并重试，没有卡在等待回调；HAL 错误分类仍不能区分 NACK 与其他总线错误。

板卡原理图的 `IO引出` 接口列出 PC9 与 PA8；固件将 PC9 用作 I²C3 SDA、PA8 用作 I²C3 SCL，MS5837 的 7 位地址为 `0x76`。这块板没有专用的板载 MS5837 接口；下一步应核对探头线确实接到该 I/O 引出、供电、共地及外部上拉。 WCH-Link 在本次刷写后仍接着，IMU 原始组等待 5 秒未收到新帧；需拔下 WCH-Link 后再验收 IMU。

本次没有发送电机运行命令或深度归零命令。网关报告 `tx_enabled=false`、`motor_command_active=false`；复位期间写入的是启动/重连安全停止事务。

## 最终修复：主循环提交读阶段

用户指出原理图只作参考，应按实际接在深度计上的线路排查。SWD 在不复位、不停核条件下读取现场 RAM：HAL `ErrorCode=0`，PROM 全零，驱动返回 `I2C_BUS_ERROR`；SCL/SDA 输入为高。结合源码，发现写完成回调中的 `HAL_I2C_Master_Receive_IT` 启动失败被无条件归为总线错误，未区分 `HAL_BUSY`。

修复只调整读阶段时序：写完成回调标记待读；主循环在原事务超时内重试读启动；`HAL_BUSY` 延后，其他 HAL 失败按真实类型映射；读启动成功后不再重复提交。主机 51 项回归覆盖暂忙恢复、持续忙超时、错误映射和单次读取；ARM Debug 编译通过。

修复 ELF 位于 `build/ms5837_deferred_rx_fix_20261010/CAN_To_Uart.elf`，SHA-256=`C88A8E555F98CADB5516BD88631A34A3F46E64BA342778D58C890F9AD2B5FE07`；BIN SHA-256=`AB09632BA27AA4C8C41D8715932FA4D5ACD142DC35952507414CF8E9F6AC21B2`。刷写日志 `build/ms5837_deferred_rx_fix_20261010/openocd_flash_20261010.log` 确认 `Programming Finished / Verified OK / Resetting Target`。

实际深度样本：`fresh/raw/pressure/temperature/depth/zero` 全部有效；压力 `100949 Pa`，温度 `26.41 °C`，D1=`6295611`、D2=`7928969`，原始与滤波水深约 `-0.001586 m`；固定 P0=`100965 Pa` 未改变。采样频率实测 24.994～25.005 Hz，成功样本从 938 增至 5838，设备 `errors=0`。

WCH-Link 拔下后 IMU 原始有效，四元数/欧拉角有效，气压四项有效；同一后台同时接收两只传感器。IMU 气压约 `101540.67 Pa`、温度 `31.77 °C`，与深度计压力独立。没有电机运行命令或 ZERO_DEPTH。
