# MS5837 深度计实现进度与验证证据

分支：`feature/depth-ms5837-20261006`　worktree：`F:/file/BaiduSyncdisk/Project/CAN_To_Uart/worktrees/depth-ms5837`
负责人：DeepSeek（只改本模块文件，不改 main/USB/全局 HAL/CMake/.ioc）

## 1. 本分支新增/修改的文件

| 文件 | 说明 |
| --- | --- |
| `Core/Inc/sensor_i2c_bus.h` | 传感器 I2C 事务层接口（短超时、7 位地址、结果码、主机测试替身开关 `I2C_HOST_TEST`）。**刻意不叫 i2c.h**，避免与 CubeMX 生成的 `Core/Inc/i2c.h` 冲突 |
| `Core/Src/sensor_i2c_bus.c` | 基于 `HAL_I2C_Master_Transmit/Receive` 的同步短事务实现；不定义 `MX_I2C3_Init` |
| `Core/Inc/ms5837.h` | MS5837 驱动接口、参数表、状态位、样本/统计结构 |
| `Core/Src/ms5837.c` | 非阻塞状态机、CRC4、02BA/30BA 一阶+二阶补偿、零点、滤波、参数 |
| `tests/ms5837_host_test.c` | 主机单元测试（含 I2C 从设备仿真与转换时间模型） |
| `tests/run_ms5837_host_test.ps1` | 一键编译+运行脚本 |
| `docs/ms5837.md` | 接口说明、数据手册核对记录、**集成方需要做的修改清单** |
| `docs/ms5837-progress.md` | 本文件（证据与限制） |

未触碰：`Core/Src/main.c`、`Core/Src/usb_can_gateway.c`、`Core/Src/gpio.c`、`Drivers/**`、`CMakeLists.txt`、
`cmake/**`、`CAN_To_Uart.ioc`、原目录 `CAN_To_Uart/CAN_To_Uart`。

`git status` 显示本分支只新增上述文件（`build/` 已被 `.gitignore` 忽略）。

## 2. 提供的 API（摘要，详见 docs/ms5837.md）

```c
/* 生命周期 */
Ms5837Result_t Ms5837_Init(void);                 /* 非阻塞：发复位命令 */
void           Ms5837_Process(void);              /* 主循环推进状态机 */
/* 数据 */
Ms5837Result_t Ms5837_GetSample(Ms5837Sample_t *);
uint32_t       Ms5837_GetStatus(void);
uint32_t       Ms5837_GetSampleAgeMs(void);
uint8_t        Ms5837_HasNewSample(void);
void           Ms5837_ClearNewSampleFlag(void);
Ms5837Result_t Ms5837_GetStats(Ms5837Stats_t *);
Ms5837Result_t Ms5837_GetProm(uint16_t prom[8]);
/* 零点 */
Ms5837Result_t Ms5837_Zero(void);
Ms5837Result_t Ms5837_ClearZero(void);
uint8_t        Ms5837_IsZeroValid(void);
/* 参数 */
Ms5837Result_t Ms5837_SetModel(uint8_t);   uint8_t  Ms5837_GetModel(void);
Ms5837Result_t Ms5837_SetOsr(uint16_t);    uint16_t Ms5837_GetOsr(void);
Ms5837Result_t Ms5837_SetOutputRateHz(uint16_t); uint16_t Ms5837_GetOutputRateHz(void);
Ms5837Result_t Ms5837_SetWaterDensity(float);    float    Ms5837_GetWaterDensity(void);
Ms5837Result_t Ms5837_SetSurfacePressurePa(float);
Ms5837Result_t Ms5837_GetSurfacePressurePa(float *);
Ms5837Result_t Ms5837_SetFilterK(float);   float    Ms5837_GetFilterK(void);
Ms5837Result_t Ms5837_SetParam(uint16_t id, uint8_t type, const void *value, uint8_t length);
Ms5837Result_t Ms5837_GetParam(uint16_t id, uint8_t *type, uint8_t *length, uint8_t value[4]);
Ms5837Result_t Ms5837_RestoreDefaults(void);
/* 纯函数（集成/测试） */
uint8_t  Ms5837_Crc4(const uint16_t prom[8]);
uint16_t Ms5837_MaxConversionTimeMs(uint8_t model, uint16_t osr);
uint8_t  Ms5837_Compensate(uint8_t model, const uint16_t prom[8], uint32_t d1, uint32_t d2,
                           int64_t *pressure_raw, int32_t *temperature_centi_c);
```

约定要点：型号默认 unknown（只输出 PROM/D1/D2，压力/温度/深度为 NaN）；型号必须显式设定；
零点必须 `Ms5837_Zero()` 或 `SetSurfacePressurePa()` 显式建立；D1/D2 转换等待由状态机承担；
每次 I2C 事务超时 ≤ 5 ms（上限 50 ms）。

## 3. 构建与运行（未安装任何新工具，使用仓库现有 gcc）

### 3.1 主机单元测试

```
PS> gcc -std=c11 -Wall -Wextra -Werror -O0 -ICore/Inc tests/ms5837_host_test.c -o build/ms5837_host_test.exe
(exit 0，无告警)

PS> .\build\ms5837_host_test.exe
[ RUN  ] Crc4
[  OK  ] Crc4
[ RUN  ] OfficialExample30ba
[  OK  ] OfficialExample30ba
[ RUN  ] OfficialExample02ba
[  OK  ] OfficialExample02ba
[ RUN  ] TemperatureBranches
[  OK  ] TemperatureBranches
[ RUN  ] ModelDifference
[  OK  ] ModelDifference
[ RUN  ] CommandSequence
[  OK  ] CommandSequence
[ RUN  ] UnknownModelDefault
[  OK  ] UnknownModelDefault
[ RUN  ] NoZeroMeansNoDepth
[  OK  ] NoZeroMeansNoDepth
[ RUN  ] ZeroAndDepth
[  OK  ] ZeroAndDepth
[ RUN  ] FilterK
[  OK  ] FilterK
[ RUN  ] ParameterValidation
[  OK  ] ParameterValidation
[ RUN  ] ParameterWireEncoding
[  OK  ] ParameterWireEncoding
[ RUN  ] OfflineDevice
[  OK  ] OfflineDevice
[ RUN  ] BusTimeout
[  OK  ] BusTimeout
[ RUN  ] NonBlockingConversionWait
[  OK  ] NonBlockingConversionWait
[ RUN  ] CycleTimingAt25Hz
        样本时间戳: 86 126 166 206 246 286 ms；间隔 40/40/40/40/40 ms；最小转换等待 11 ms（要求 >= 11 ms）
[  OK  ] CycleTimingAt25Hz
[ RUN  ] PromCrcFailure
[  OK  ] PromCrcFailure
[ RUN  ] PromContentSanity
[  OK  ] PromContentSanity
[ RUN  ] InvalidConversionRejected
[  OK  ] InvalidConversionRejected
[ RUN  ] CompensateInputValidation
[  OK  ] CompensateInputValidation
[ RUN  ] ModelChangeClearsZero
[  OK  ] ModelChangeClearsZero
[ RUN  ] ZeroValidatesRange
[  OK  ] ZeroValidatesRange
[ RUN  ] ConfigChangeDuringConversion
[  OK  ] ConfigChangeDuringConversion
[ RUN  ] SampleMetadataAndFreshness
[  OK  ] SampleMetadataAndFreshness
[ RUN  ] I2cBusLayer
[  OK  ] I2cBusLayer
ms5837_host_test: PASS
(exit 0)
```

共 25 组测试，全部 PASS；`-O2` 构建同样 `PASS`（`gcc -std=c11 -Wall -Wextra -Werror -O2 ...`）。
一键脚本：`powershell -NoProfile -ExecutionPolicy Bypass -File tests/run_ms5837_host_test.ps1`（结果同上）。

### 3.2 目标交叉编译（只编译，不烧录、不启动 OpenOCD、不占用串口）

```
PS> arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -std=c11 -Wall -Wextra -Werror -O2 \
      -DSTM32H750xx -DUSE_HAL_DRIVER -c Core/Src/sensor_i2c_bus.c -o bus.o
(exit 0)   text 400   data 1   bss 5
PS> ... -c Core/Src/ms5837.c -o ms5837.o
(exit 0)   text 4545  data 184 bss 0
```

`-Wall -Wextra -Werror` 下无告警，说明驱动可被现有 CubeMX/HAL 工程直接编译。
本次未调用 CMake（CMake 由协调人合入 `Core/Src/sensor_i2c_bus.c`、`Core/Src/ms5837.c` 后再跑）。

## 4. 测试覆盖内容

| 测试 | 覆盖点 |
| --- | --- |
| `Crc4` | 官方算例系数的 CRC 常量（独立 Python 复现）、全 0/全 1 向量、单比特翻转必被检出、字 0 高 4 位屏蔽、字 7 固定为 0 |
| `OfficialExample30ba` | 数据手册算例：19.81 °C / 3999.8 mbar（`pressure_raw=39998`），并交叉验证独立复现实现 |
| `OfficialExample02ba` | 数据手册算例：20.00 °C / 1100.02 mbar（`pressure_raw=110002`） |
| `TemperatureBranches` | 30BA 低温、极低温（< −15 °C，叠加项）、高温分支；02BA 低温；unknown 型号不补偿、不写输出 |
| `ModelDifference` | 同一原始数据在 02BA/30BA 下结果不同；两型号最大转换时间表逐项对照；unknown 取较慢一侧 |
| `CommandSequence` | 第一条命令是复位 0x1E；PROM **恰好 7 次**读且顺序为 0xA0/0xA2/0xA4/0xA6/0xA8/0xAA/0xAC，**整个命令日志里不允许出现 0xAE**；OSR4096/256/8192 → D1 0x48/0x40/0x4A、D2 0x58/0x50/0x5A |
| `UnknownModelDefault` | 默认 unknown：RAW_VALID 有、CONFIG_UNKNOWN 置位、压力/温度/深度全 NaN（非 0）；`Ms5837_Zero()` 返回 `ERR_MODEL_UNKNOWN`；显式设型号后下一帧出有效值 |
| `NoZeroMeansNoDepth` | 无零点时深度 NaN、DEPTH_VALID=0、`GetParam(0103)` 返回 `ERR_NO_ZERO`；显式设定后深度按 `(P−P0)/(rho·g)` 计算；清除零点后回到 NaN |
| `ZeroAndDepth` | 显式 `Ms5837_Zero()` 采零点 → 深度≈0；D1 阶跃后深度与独立参考压力计算的期望值一致（含 02BA 的 Pa/LSB 换算） |
| `FilterK` | 换零点后滤波重新起步；阶跃后滤波值落在旧值与新值之间且符合 `k·y+(1−k)·x` |
| `ParameterValidation` | OSR 非法值、采样率 0/101、50 Hz+OSR4096 被拒（预算 24 ms > 20 ms）、40 Hz+OSR4096 允许、25 Hz+OSR8192 被拒（42 ms > 40 ms）、20 Hz+OSR8192 允许、02BA 25 Hz+OSR8192 允许（40 ms）、型号 1/5/99、密度 899/1301、FILTER_K −0.01/1.0、零点压力 9999/200001、未知参数 ID、类型/长度不匹配 |
| `ParameterWireEncoding` | u16/f32/u8 小端往返、`RESTORE_DEFAULTS` 语义（型号回 unknown、参数回默认、清零点） |
| `OfflineDevice` | 从设备 NACK：`Init` 返回 `ERR_IO`、ONLINE 清、无样本、错误计数增长、不产生任何伪造数据；恢复后自动重试并产出样本 |
| `BusTimeout` | 总线卡死：`Init` 返回 `ERR_TIMEOUT`，单次事务恰好消耗 5 ms（短超时），主循环每次 `Process` 消耗 ≤ 5 ms；恢复后可重新初始化并出样本 |
| `NonBlockingConversionWait` | OSR4096 下一帧样本至少跨 22 次主循环调用（2×10 ms + 2×1 ms 余量）；`Process()` 自身不推进时间（内部无忙等）；单次调用最多 3 次 I2C 事务；仿真中“提前读 ADC”次数为 0；最小转换等待 ≥ 数据手册最大值 + 1 ms |
| `CycleTimingAt25Hz` | **时序回归（评审问题）**：每次 I2C 事务真实消耗 1 ms、逐毫秒推进主循环，连续 6 帧样本时间戳间隔实测 40/40/40/40/40 ms（真 25 Hz，修复前是“转换耗时 + 周期”≈ 65 ms ≈ 15 Hz）；最小“命令→读 ADC”等待 11 ms（≥ 9.04→10 + 1 ms）；提前读 ADC 0 次；单次 `Process` 之外的任何等待都判失败 |
| `PromContentSanity` | CRC **自洽**但 C1~C6 全 0 / 全 0xFFFF 的假 PROM：先断言该 PROM 的 CRC 确实通过，再断言驱动拒绝（不置 PROM_VALID、无样本、`crc_errors` 增长）；换回正常 PROM 后恢复上线 |
| `InvalidConversionRejected` | D1=0 / D1=0xFFFFFF / D2=0xFFFFFF：不产生新样本、序号不推进、RAW/PRESSURE/DEPTH 位清零、已发布样本测量字段变 NaN、`last_error=ERR_NOT_READY`、`good_frames` 不增长、ONLINE 保留；恢复正常后序号 +1 且数据有效 |
| `CompensateInputValidation` | 纯函数拒绝 >24 位输入与 0/0xFFFFFF 输入且不写输出；24 位边界内正常工作；输出指针可为 NULL |
| `ModelChangeClearsZero` | 同型号重复设置幂等（零点保留）；30BA→02BA 与 02BA→unknown 都必须清零点与滤波：`ZERO_VALID` 清、`GetSurfacePressurePa`/`GetParam(0103)` 返回 `NO_ZERO`、深度/滤波深度/`surface_pressure_pa` 变 NaN |
| `ZeroValidatesRange` | 压力 <10000 Pa 与 >200000 Pa 时 `Ms5837_Zero()` 返回 `ERR_PARAM` 且不建立零点（`GetParam(0103)` 仍是 `NO_ZERO`）；量程内成功且 `GetSurfacePressurePa`/`GetParam(0103)` 读回的 P0 与样本压力一致并落在 10000~200000 Pa |
| `ConfigChangeDuringConversion` | 在 `CONVERT_D1` 途中把 OSR 256→8192：旧半周期被丢弃（改动后第一条转换命令是新的 D1 `0x4A`，不会先出现被中断的 D2）、新等待 ≥ 19+1 ms、提前读 0 次；型号 30BA→02BA 途中同样丢弃半周期并重新走完整周期 |
| `PromCrcFailure` | PROM 被破坏：PROM_VALID/RAW_VALID 清、`crc_errors` 增长、无样本、`GetProm` 返回 `NOT_READY`；修复后自动恢复并正常出帧 |
| `SampleMetadataAndFreshness` | 序号递增、时间戳推进、样本年龄；总线故障后旧样本不得再报告 PRESSURE_VALID/DEPTH_VALID，但 PROM_VALID 保留 |
| `I2cBusLayer` | 未绑定句柄拒绝事务；7 位地址范围校验；空指针/0 长度/超长拒绝；超时 0→5 ms、过大→50 ms 截断；HAL BUSY/ERROR/TIMEOUT 映射；错误地址被拒 |

主机测试里的从设备仿真带**独立复制的数据手册最大转换时间表**：驱动若提前读 ADC 会被记为违规
（`early_reads`），当前为 0。
仿真设备对 PROM 命令 index>6（即 0xAE 及以后）返回 `HAL_ERROR`，因此“能上线”本身就证明驱动不依赖第 8 个字。

## 4.1 集成评审修正记录（2026-10-06）

| 评审意见 | 处理 | 证据 |
| --- | --- | --- |
| 1) 自写 `i2c.c/h` 会与 CubeMX 生成的 `Core/Src/i2c.c`、`Core/Inc/i2c.h` 冲突，需改名 | 已 `git mv` 为 `Core/Inc/sensor_i2c_bus.h`、`Core/Src/sensor_i2c_bus.c`；头文件保护宏改为 `__SENSOR_I2C_BUS_H__`；`ms5837.c`、测试、文档全部更新。**HAL I2C3 初始化（GPIO/时钟/`MX_I2C3_Init`/`MspInit`）归集成方**，本模块只 `I2c_Init(&hi2c3)` 绑定句柄，不定义任何 `MX_*` 符号。驱动/总线对外函数名保持不变（`I2c_*`、`Ms5837_*`），接口无破坏性改动 | 重新编译 + 全部测试 PASS；`arm-none-eabi-gcc -Werror` 交叉编译 `sensor_i2c_bus.c` 通过 |
| 2) 30BA 物理 PROM 只有 7 个 16 位字（0xA0..0xAC），CRC4 的第 8 字由软件置 0，不得要求读 AE | 核查结果：读取循环**本来就只读 7 个字**（`MS5837_CMD_PROM_WORD_COUNT = 7`，逐字 0xA0..0xAC，随后 `ms5837.prom[7] = 0`），从未下发 0xAE。为防回归，`CommandSequence` 测试已加显式断言：PROM 读恰好 7 次且顺序为 0xA0,0xA2,0xA4,0xA6,0xA8,0xAA,0xAC，命令日志中不得出现 0xAE；仿真设备对 index>6 返回 `HAL_ERROR`，能上线即证明不依赖第 8 个字 | `ms5837_host_test: PASS`；源码注释同步说明“不读 0xAE” |
| 3) 周期调度把转换等待累加上去（25 Hz 实际 ~16 Hz） | `read_d2()` 不再用“完成时刻 + 周期”，改为在发 D1 时记录 `cycle_start_ms`，下一帧锚定 `cycle_start_ms + 1000/f_s`；超期时从当前时刻开始下一帧，不做补偿性连发 | 新增 `CycleTimingAt25Hz`：每次事务真实耗 1 ms、逐毫秒推进，连续 6 帧间隔实测 **40/40/40/40/40 ms** |
| 4) 转换 deadline 用发命令前的 now，毫秒相位可能提前读 | 改为命令发完之后重新 `HAL_GetTick()`，再加 `ceil(最大转换时间) + MS5837_CONVERSION_MARGIN_MS(1 ms)` | 同一测试断言最小“命令→读 ADC”等待 11 ms（≥ 10 + 1）且提前读 0 次 |
| 5) `schedule_fits` 余量不足 | 改为 `2 × (最大转换时间 + 1 ms) + MS5837_SCHEDULE_TX_BUDGET_MS(2 ms) ≤ 1000/f_s`：30BA OSR4096 → 24 ms（25/40 Hz 可，50 Hz 否）、OSR8192 → 42 ms（20 Hz 可，25 Hz 否）；02BA OSR8192 → 40 ms（25 Hz 可） | `ParameterValidation` 覆盖上述全部边界；`ms5837.h` 公开两个余量常量 |
| 6) 假 PROM（CRC 碰巧通过）与无效转换会被当成有效数据 | PROM 增加内容可信度检查（C1~C6 全 0 / 全 0xFFFF → 不置 PROM_VALID）；D1/D2 = 0 或 0xFFFFFF → 不发布样本、测量字段置 NaN、序号不推进；`Ms5837_Compensate()` 拒绝 >24 位与 0/0xFFFFFF 输入 | 新增 `PromContentSanity`（先断言假 PROM 的 CRC 自洽，再断言被拒）、`InvalidConversionRejected`、`CompensateInputValidation` |
| 7) 型号切换必须清除旧水面零点与滤波（旧 P0 可能是错误型号算出来的） | `SetModel()` 在型号真正变化时调用 `ClearZero()` 并作废已发布样本补偿值；同型号重复设置保持幂等不清零点 | 新增 `ModelChangeClearsZero`：30BA→02BA、02BA→unknown 均断言 `ZERO_VALID` 清、`GET_PARAMETER(0103)` 由 OK 变 `NO_ZERO`、深度变 NaN；30BA→30BA 零点保留 |
| 8) `Ms5837_Zero()` 未做范围校验，可能产生 `GET_PARAMETER` 认为越界的 P0 | `Zero()` 直接复用 `SetSurfacePressurePa()`（有限值 + 10000~200000 Pa）；越界返回 `ERR_PARAM` 且不建立零点，已有有效零点不被破坏 | 新增 `ZeroValidatesRange`：越界（<10000 / >200000 Pa）被拒且 `0103` 仍为 `NO_ZERO`；量程内成功后 `0103` 读回的 P0 与样本压力一致 |
| 9) OSR/型号在 D1/D2 转换途中变更会沿用旧 deadline（可能提前读） | 新增 `ms5837_abort_half_cycle()`：`SetOsr()`/`SetModel()` 在 `CONVERT_D1/D2` 时丢弃半周期、清半截 D1/D2，回 IDLE 用新配置重新走完整周期 | 新增 `ConfigChangeDuringConversion`：256→8192 途中改配置后必须先出现新的 D1 `0x4A`（不能先出现被中断的 D2）、最小等待 ≥ 19+1 ms、提前读 0 次；型号切换同理 |

## 5. 未做 / 未验证（重要）

* **硬件 NOT TESTED**：本分支没有烧录、没有复位设备、没有启动/终止 OpenOCD、没有打开 COM11/COM42，
  也没有任何真实 I2C3 波形或真实 MS5837 读数。全部结论来自数据手册 + 主机仿真 + 交叉编译。
* 真实型号（02BA vs 30BA）仍未确认，固件默认 unknown，必须由人显式设定；未实现任何自动型号猜测。
* 未做：`SAVE_CONFIG` 持久化、温度校准（MS5837 无该命令，应回 UNSUPPORTED）、多实例、DMA/中断收发。
* 复位后 10 ms 延时是保守余量（数据手册未给具体数值），未在硬件上测过最短安全值。
* 集成方仍需完成：`.ioc` 使能 I2C3(PA8/PC9) 与 CubeMX 的 `MX_I2C3_Init`/MspInit、`main.c` 里
  `I2c_Init(&hi2c3)` + `Ms5837_Init()` + 主循环 `Ms5837_Process()`、`CMakeLists.txt` 增加
  `Core/Src/sensor_i2c_bus.c` 与 `Core/Src/ms5837.c` —— 清单见 `docs/ms5837.md` 第 5 节。
* 工具链限制：本机 w64devkit 未带 ASan/UBSan 运行库，无法跑 sanitizer；改用 `-O0`/`-O2` + `-Werror`
  加独立参考实现交叉验证，并以 `arm-none-eabi-gcc -Werror` 交叉编译确认无告警。

## 6. 提交

* 分支：`feature/depth-ms5837-20261006`
* 实现提交：`1130aca` “功能：新增MS5837深度计I2C3驱动与主机测试（02BA/30BA补偿、CRC4、非阻塞转换、显式零点）”
  （8 个文件、3547 行新增；总线文件名在评审后由 `i2c.c/i2c.h` 改名为 `sensor_i2c_bus.c/.h`）。
* 文档提交：`e53785f` “文档：记录MS5837深度计实现提交与验证证据”。
* 评审修正提交 1：`d753e54` “修正：传感器总线改名以避开CubeMX i2c.c/h，并固化PROM 7字读取断言”。
* 评审修正提交 2：`7d73313` “修正：周期调度不累加转换等待、命令后重取tick加余量，并拒绝假PROM/无效转换”。
* 评审修正提交 3：本文件所在提交 “修正：型号切换清零点、Zero 复用范围校验、转换途中改配置丢弃半周期”，
  只包含 `Core/Inc/ms5837.h`、`Core/Src/ms5837.c`、`tests/ms5837_host_test.c`、`docs/ms5837.md`、
  `docs/ms5837-progress.md`。
* 只提交了本模块自己的文件（未 `git add .`，未改动/回退他人文件，未 push，未合并 main）。
* 提交时 Git 提示 “LF will be replaced by CRLF”，来自本机 `core.autocrlf=true`，与仓库既有文件一致。
