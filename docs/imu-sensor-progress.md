# IMU 后端（imu_sensor）进度与交接 — 2026-10-06

负责人：ZCode（分支 `imu-uart1-debug`，原工作目录）。
依据：`SENSOR_COORDINATION_20261006.md`（含勘误与统一 AA5B 约定）。
状态：**代码与主机测试完成；全部硬件事项未验证（NOT TESTED）**。

## 1. 交付文件（仅新增 4 个，未改任何既有文件）

| 文件 | 内容 |
|---|---|
| `Core/Inc/imu_sensor.h` | 公开接口、结果码/状态位/参数 ID（编号与 AA5B 对齐） |
| `Core/Src/imu_sensor.c` | 7E23 解析、原生命令组包、快照与异步请求（纯 C、无 HAL 依赖） |
| `tests/imu_sensor_host_test.c` | 主机测试（assert 风格，直接 include 被测实现） |
| `docs/imu-sensor-progress.md` | 本文件 |

未触碰：main / USB / CMake / .ioc / RTC / imu_uart_debug / 已有的 5 个未提交改动原样保留。
未切分支、未合并、未 `git add .`、未烧录、未占用 COM11/COM42。

## 2. 复现命令与结果

主机测试（w64devkit GCC 15.2.0，Git Bash）：

```
gcc -std=c11 -Wall -Wextra -Werror -O0 -ICore/Inc \
    tests/imu_sensor_host_test.c -o build/imu_sensor_host_test.exe
./build/imu_sensor_host_test.exe
# 输出：imu_sensor_host_test: PASS
```

目标侧交叉编译检查（Arm GNU Toolchain 14.3.Rel1，仅 `-c` 语法/告警检查，不链接）：

```
C:/Users/Administered/.eide/tools/gcc_arm/bin/arm-none-eabi-gcc.exe -c \
    -std=c11 -Wall -Wextra -Werror -Og -g \
    -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard \
    -ICore/Inc Core/Src/imu_sensor.c -o build/arm_check/imu_sensor.o
# exit=0，无告警
```

两条命令均在 `-Werror` 下零告警通过。测试二进制产物在 `build/` 下（已被 .gitignore 覆盖，不入库）。

## 3. API 与 AA5B 映射

```c
void     ImuSensor_Init(const ImuSensor_Config *config);   /* pins_blocked 恒=1 */
uint8_t  ImuSensor_IsPinsBlocked(void);
void     ImuSensor_SetPinsBlocked(uint8_t blocked);        /* 仅集成层在用户确认后调用 */
int      ImuSensor_Feed(const uint8_t *data, uint16_t len, uint64_t now_us);
void     ImuSensor_Process(uint64_t now_us);               /* 主循环：解析+超时 */
int      ImuSensor_GetSample(ImuSensor_Sample *out, uint64_t now_us);
int      ImuSensor_GetParameter(uint16_t id, uint32_t *value_out);
int      ImuSensor_Request(uint8_t op, uint32_t arg, uint64_t now_us, uint32_t *host_seq_out);
int      ImuSensor_PopResult(ImuSensor_Result *out);
```

`ImuSensor_Result.result` 与 `ImuSensor_Sample.status` 的位/值编号均与 AA5B v1 的
RESULT/status 一致，AA5B 层可直抄。建议映射：

| AA5B CMD | 后端调用 | 备注 |
|---|---|---|
| 01 GET_INFO | `GetSample` 取 version / 状态位 | 型号仅凭已证实信息，MODEL_CONFIRMED 恒 0 |
| 02 GET_STATUS | `GetSample` | sample_seq=raw_seq，age=now-raw_time_us（无数据 0xFFFFFFFF） |
| 03 GET_PARAMETER | `GetParameter` | 1=有缓存→RESULT 7 UNCONFIRMED+值；0=RESULT 8 NOT_READY；-1=RESULT 1 UNSUPPORTED |
| 04 SET_PARAMETER | `Request(SET_RATE/SET_MODE)` | 结果恒 7 UNCONFIRMED（原生无回复），BAD_VALUE 同步返回 |
| 05/0A/0B SAVE/SELFTEST/REBOOT | `Request` 返回 UNSUPPORTED | 预留 |
| 09 CALIBRATE type=1 | `Request(CAL_ACCEL_GYRO_START/CLEAR)` | 原生 0x70（原文已核实：陀螺仪+加速度；未实机） |
| 09 CALIBRATE type=2 | `Request(CAL_MAG_START/CLEAR)` | 原生 0x71（原文已核实：磁力计；未实机） |
| 09 CALIBRATE type=3 | `Request(CAL_TEMP, arg=温度×100)` | 原生 0x73，8 字节帧（表格长度单元格 07 为笔误，按下标行取 8）；应答 0x81 [0x73, 状态]；未实机 |
| 06 RESTORE_DEFAULTS | `Request(RESET)` | 原生 0xA0 重置用户数据，字面量 `7E 23 07 A0 01 5F A8`，无回复→UNCONFIRMED；未实机 |
| 05/0A/0B SAVE/SELFTEST/REBOOT | `Request` 返回 UNSUPPORTED | 原生协议无对应命令（0xA0 是重置用户数据，不是复位/保存/自检） |

 Stream80/81/85 载荷字段与 `ImuSensor_Sample` 一一对应（accel_g、gyro_rad_s、
mag_units、quat_wxyz、euler_rpy_rad、baro_*），无伪造字段。

## 4. 集成接线要点（传输层归统筹方）

- **调用上下文**：`Feed/Process/GetSample/Request/PopResult` 同一上下文（主循环），
  非 ISR 安全。推荐：UART IDLE 中断只做"DMA 游标差 → 线性暂存"，主循环再 `Feed`。
- **有界接收**：模块内部固定 1024 字节环形缓冲 + 16 个分块头；放不下时**整块原子
  丢弃**并计入 `rx_overflow_chunks`/`rx_dropped_bytes`，永不阻塞、永不增长。
- **分块时间戳**：`Feed` 的 `now_us` 是该块到达时刻，块内所有完整帧共用之。集成层
  用 `HAL_GetTick()*1000` 或细时钟均可，但 Feed/Process/Request 必须同一时钟。
- **DMA 缓冲与缓存**：DMA 环形缓冲放链接脚本已有的 `.dma_buffer` 段（RAM_D2
  0x30000000），32 字节对齐（参考 imu_uart_debug.c 的既有做法）。当前 main.c **未
  调用 SCB_EnableDCache()**（已 grep 证实），此刻无一致性问题；若集成时开启
  D-Cache，必须在 CPU 读取新到数据前 `SCB_InvalidateDCache_by_Addr()` 覆盖新区段，
  或用 MPU 把该缓冲设为非缓存。
- **TX 回调**：`ImuSensor_Config.tx` 返回 0 表示已交给 UART；返回非 0 会产生
  IO_ERROR 结果且不进入待确认、不写参数缓存。
- 固件侧把 `imu_sensor.c` 加进 CMake 由统筹方在 sensor-integration 工作树完成；
  本仓库未改 CMake。

## 5. 假设与未核实事项（评审重点）

统筹方 2026-10-06 复核附件原文，以下三点由"映射假设"转为**原文已核实（仍未实机）**，
实现与之一致，无需改码：**0x70=陀螺仪+加速度校准、0x71=磁力计；0x81 status
0=失败、1=成功；校准请求带 +5F 后缀**。

| # | 事项 | 说明 |
|---|---|---|
| 1 | MAG 单位保持中性（`mag_units`，800/32767） | 表格明确未命名物理单位，未写 uT |
| 2 | `CONFIG_UNKNOWN` 恒置位 | 无原生读回，配置永远无法确认；UNCONFIRMED 的 rate/mode 下发不清除该位（统筹 2026-10-06 指示的严格解释，含测试） |
| 3 | `MODEL_CONFIRMED` 恒 0 | 无原生型号读回，禁止凭空确认 |
| 4 | 帧时间戳=分块到达时刻，不做块内字节回推 | 上位机时间戳本身 ms 级（ms*1000），回推收益 <87µs/字节；如需可后续补 |
| 5 | 默认超时（`request_timeout_us=0`）：版本查询 1s、0x70/0x71/0x73 校准 30s；显式非 0 值对全部待确认请求统一覆盖。单一在飞；在飞期间一切请求（含无回复命令）BUSY | 校准实机耗时远长于版本查询，不能共用 1s；保守策略避免校准期间混入未定义时序命令 |
| 6 | rate/mode 缓存=最后成功下发的值，GET 恒 UNCONFIRMED | 原文明确无原生读回 |
| 7 | 0x73 的 T_100 **有符号性未证实**：表格只说"温度×100"，I2C 侧寄存器标 uint16；实现按 16 位线格式原样发送，负温度行为待实机确认 | 常温参考（0~50°C）下不影响使用 |
| 8 | 0x73 帧长取 8 字节：表格"长度 07"与"下标 0..7"矛盾，按下标行（与统筹勘误一致）；若实机按 07 处理需回调 | 原文矛盾无法纸面解决，已按更自洽一侧实现并记入 NOT TESTED |

变更记录：

- `3a07363`：初始提交（当时 0x70/0x71 映射与 0x81 语义为假设；CONFIG_UNKNOWN 按 rate+mode 均下发清除）。
- `4ea6084`：0x70/0x71/0x81/+5F 转为原文已核实（文档更正，实现本就一致）；`CONFIG_UNKNOWN` 改为恒置位，UNCONFIRMED 的下发不再清除该位。
- `82c590a`：校准超时分档（版本 1s / 校准 30s，`request_timeout_us=0` 时生效），显式非 0 统一覆盖。
- 本次修正（对照通信协议.xlsx 原文逐行核实后）：**修复 SET_RATE 组包 bug**——0x60 的频率参数为单字节（帧总长 07），此前误组包为 u16 两字节（8 字节）；**实现 0x73 温度校准**（8 字节帧、0x81 应答、30s 校准超时、arg=温度×100 的 16 位线格式）；**新增 IMU_SENSOR_REQ_RESET**（原生 0xA0 重置用户数据，字面量 `7E 23 07 A0 01 5F A8`，无回复→UNCONFIRMED，对应 AA5B RESTORE_DEFAULTS 语义，是否映射由统筹定）；SAVE/SELFTEST/REBOOT 保持 UNSUPPORTED（原生无对应命令）。UART MCU→IMU 命令全集（7 条）现已全部实现。

## 6. 主机测试覆盖（对照统筹要求）

| 统筹要求 | 测试函数 |
|---|---|
| 碎包 | `Test_FragmentedFeed` |
| 粘包 | `Test_CoalescedAndNoise` |
| 噪声/重同步 | `Test_CoalescedAndNoise`（伪包头 7E 7E 00、7E 23 FF） |
| 校验和(CRC) | `Test_BadChecksum` |
| 错误长度 | `Test_BadLengthAndUnknownFunc`（0x04 短帧、未知功能字） |
| 负数 | `Test_RawScalingAndNegatives`（int16 负值→g/rad/s 缩放） |
| float | `Test_NonfiniteFloat`（NaN 整帧拒收）、四元数/欧拉/气压计用例 |
| 未对齐安全 | `Test_UnalignedOffsets`（奇数偏移逐字节 LE 访问） |
| 参数边界 | `Test_ParamAndRequests`（rate 9/10/100/101，mode 6/7/9） |
| 无 ACK | `Test_ParamAndRequests`（0x60/0x61 立即 UNCONFIRMED，绝不等待 0x81/0x01 假装成功） |
| 有 ACK | `Test_CalAck`（0x81 匹配原命令；状态 0→IO_ERROR；原命令不符→unexpected+超时） |
| 超时 | `Test_TimeoutAndBusy`（1ms 超时窗、超时后恢复） |
| 引脚占用 | `Test_ParamAndRequests` + `Test_StatusBits`（默认 blocked=1，一切请求 PIN_BLOCKED，零发送） |
| 附加 | `Test_TxError`（发送失败→IO_ERROR、缓存不写）、`Test_Unsupported`、`Test_GroupTimestampsIndependent`（组独立时间/序号）、`Test_RingOverflow`（整块丢弃与恢复）、`Test_Barometer`、`Test_EdgeInputs` |

版本请求帧与统筹文档字面量 `7E 23 07 80 01 00 29` 逐字节断言一致（校验和算法锚点）。

## 7. 硬件未验证清单（NOT TESTED，勿当已验证引用）

- 真实 IMU 的 7E23 数据帧**未上机解码**：PA9/PA10 当前被 WCH-Link 占用，按统筹要求默认 `pins_blocked=1`，不驱动引脚、不自环、不自动校准、不烧录。
- 全部原生命令（0x60/0x61/0x80/0x70/0x71/0x73/0xA0）**未发往真机**；0x81 的 status 语义已按附件原文核实，但回包行为未实测。0x73 的 8 字节帧长解释与 T_100 有符号性未实机确认。
- 波特率/时序/字节间隔未实测（模块按分块时间戳工作，不依赖波特率常量）。
- 本模块从未在 STM32 上运行；目标侧仅交叉编译检查通过。
- 解锁引脚（`ImuSensor_SetPinsBlocked(0)`）与任何实机操作，等待用户明确指示后由统筹方执行。

## 8. 实机只读接收验证（COM42，2026-10-06，经统筹授权）

**结果：COM42 链路通，但采到的 261 字节全部是已烧录调试固件在 PA9 上的
"SELFTEST-7E23" 自测横幅（13B/500ms），0 个 7E23 IMU 帧。当前接线（COM42 RX←PA9）
无法采到 IMU 数据（IMU TX→PA10）。H750 DMA/正式后端联调仍未开始、未通过。**

- 方法：PowerShell `System.IO.Ports.SerialPort`，115200 8N1、无流控、DTR/RTS 关、
  RX-only 10 秒、零写入；结束后 `Close()` 确认 `is-open=False`。注意：RX-only/零写入
  只证明未主动发送数据，**不能证明 WCH-Link 物理 TX 引脚为高阻态**；PA10 共驱风险
  须由硬件断开解决。
- 依据数据认定：横幅字符串与 500ms 周期逐字对应 `imu_uart_debug.c` 自环自测代码，
  且已烧录构建的 `IMU_DBG_SELFTEST_ENABLE=1`（早于"不自环"规则），重烧即消除；
  与统筹方在 COM11（CDC）看到 IMU 十六进制文本自洽——IMU 数据只出现在 PA10→固件→CDC 路径。
- 详细报告与原始数据：`build/hardware-test/hardware-test-20261006.md`、
  `imu_com42_10s.bin`（build/ 不入库，随工作树保留）。

复现命令：

```
powershell -NoProfile -ExecutionPolicy Bypass -File tests/imu_capture_com42.ps1
# 默认 COM42/115200/10s，仅 RX，输出 build/hardware-test/imu_com42_10s.bin
gcc -std=c11 -Wall -Wextra -Werror -O0 -ICore/Inc tests/imu_bin_analyze.c -o build/imu_bin_analyze.exe
./build/imu_bin_analyze.exe build/hardware-test/imu_com42_10s.bin
```
