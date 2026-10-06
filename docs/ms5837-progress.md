# MS5837 深度计实现进度与验证证据

分支：`feature/depth-ms5837-20261006`　worktree：`F:/file/BaiduSyncdisk/Project/CAN_To_Uart/worktrees/depth-ms5837`
负责人：DeepSeek（只改本模块文件，不改 main/USB/全局 HAL/CMake/.ioc）

## 1. 本分支新增/修改的文件

| 文件 | 说明 |
| --- | --- |
| ~~`Core/Inc/sensor_i2c_bus.h`~~ | **已并入 `Core/Inc/ms5837.h`（2026-10-06 折叠，见文末）** |
| ~~`Core/Src/sensor_i2c_bus.c`~~ | **已并入 `Core/Src/ms5837.c`** |
| `Core/Inc/ms5837.h` | MS5837 驱动接口、参数表、状态位、样本/统计结构 |
| `Core/Src/ms5837.c` | 非阻塞状态机、CRC4、02BA/30BA 一阶+二阶补偿、零点、滤波、参数 |
| `tests/ms5837_host_test.c` | **（已按用户要求删除，见文末）**
| `tests/ms5837_reference_vectors.h` | **（已按用户要求删除，见文末）**
| `tests/gen_ms5837_reference_vectors.py` | **（已按用户要求删除，见文末）**
| `tests/run_ms5837_host_test.ps1` | **（已按用户要求删除，见文末）**
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
        样本时间戳: 97 137 177 217 257 297 ms；间隔 40/40/40/40/40 ms；最小转换等待 12 ms（要求 >= 11 ms）
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
[ RUN  ] BusyDeviceNoEarlyRestart
        D1-8192to256     busy重发=0 busy读=0 提前读=0 新D1=0x40
        D2-8192to256     busy重发=0 busy读=0 提前读=0 新D1=0x40
        D1-256to8192     busy重发=0 busy读=0 提前读=0 新D1=0x4A
        D2-256to8192     busy重发=0 busy读=0 提前读=0 新D1=0x4A
        D1-30BAto02BA    busy重发=0 busy读=0 提前读=0 新D1=0x48
        D1-02BAto30BA    busy重发=0 busy读=0 提前读=0 新D1=0x48
        D1-change-x4     busy重发=0 busy读=0 提前读=0 新D1=0x40
        D1-wrap-8192to256 busy重发=0 busy读=0 提前读=0 新D1=0x40
[  OK  ] BusyDeviceNoEarlyRestart
[ RUN  ] RestoreDefaultsDuringConversion
[  OK  ] RestoreDefaultsDuringConversion
[ RUN  ] OfficialExample02baTerms
[  OK  ] OfficialExample02baTerms
[ RUN  ] TemperatureBranchBoundary
[  OK  ] TemperatureBranchBoundary
[ RUN  ] UnitsAndNegativeDepth
[  OK  ] UnitsAndNegativeDepth
[ RUN  ] RandomizedInvariants
        fuzz seed=0: 6000 步 / 5 个样本
        fuzz seed=1: 6000 步 / 5 个样本
        fuzz seed=2: 6000 步 / 4 个样本
        fuzz seed=3: 6000 步 / 4 个样本
        fuzz seed=4: 6000 步 / 5 个样本
        fuzz 合计: 5 seeds × 6000 步 / 23 个样本 / 最终 model=0 osr=1024 rate=38Hz
[  OK  ] RandomizedInvariants
[ RUN  ] ReferenceVectors
        参考向量 100 条全部一致（30BA/02BA、物理量与极端输入、分支边界）
[  OK  ] ReferenceVectors
[ RUN  ] InterruptModeAsync
[  OK  ] InterruptModeAsync
[ RUN  ] SampleMetadataAndFreshness
[  OK  ] SampleMetadataAndFreshness
[ RUN  ] I2cBusLayer
[  OK  ] I2cBusLayer
ms5837_host_test: PASS
(exit 0)
```

共 31 组测试，全部 PASS；`-O2` 构建同样 `PASS`（`gcc -std=c11 -Wall -Wextra -Werror -O2 ...`）。
修复前的失败证据（同一条命令、同一套用例）：`BusyDeviceNoEarlyRestart: sim.new_conversion_while_busy == 0U` ×6
（D1/D2 8192→256、30BA↔02BA、连续 4 次改配置、tick 回绕），修复后 8 场景全 0 违规。
一键脚本：`powershell -NoProfile -ExecutionPolicy Bypass -File tests/run_ms5837_host_test.ps1`（结果同上）。

### 3.2 目标交叉编译（只编译，不烧录、不启动 OpenOCD、不占用串口）

```
PS> arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -std=c11 -Wall -Wextra -Werror -O2 \
      -DSTM32H750xx -DUSE_HAL_DRIVER -c Core/Src/sensor_i2c_bus.c -o bus.o
(exit 0)   text 400   data 1   bss 5
PS> ... -c Core/Src/ms5837.c -o ms5837.o
(exit 0)   text 4705  data 184 bss 0
```

另：`gcc -std=c99 -Wall -Wextra -Werror -pedantic -DI2C_HOST_TEST -c Core/Src/ms5837.c` 也 exit 0。

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

### 4.2 本轮（02BA01 06/2017 用户手册核对）

| 测试 | 覆盖点 |
| --- | --- |
| `BusyDeviceNoEarlyRestart` | **手册第 11 页回归（先失败后修复）**：HAL mock 维护芯片 `busy_until`，转换未完成时收到新 D1/D2 或 ADC read 即计数。8 个场景：D1/D2 期间 8192→256、256→8192、30BA↔02BA、连续 4 次改配置、tick 从 0xFFFFFF00 回绕。修复前 6/8 场景报“busy 期间重发”，修复后全部 `busy重发=0 busy读=0 提前读=0`，且新周期首条命令等于新 OSR（0x40/0x4A/0x48） |
| `OfficialExample02baTerms` | 手册第 7 页算例**逐项**断言：`dT=68`、`TEMP=2000`、`OFF=5764707214`、`SENS=3039050829`、`P=110002`（1100.02 mbar，1 LSB = 1 Pa），并断言驱动输出与逐项复算一致 |
| `TemperatureBranchBoundary` | TEMP=1999 与 TEMP=2000 分界两侧（1999 走低温修正、2000 的二阶修正必须全为 0，且等于一阶结果）；02BA 非低温分支不得有任何修正（手册第 8 页无高温项）；冷→热→冷连续三帧都等于独立复算，证明没有残留上一帧的 Ti/OFFi/SENSi |
| `UnitsAndNegativeDepth` | 30BA `pressure_pa = raw×10`（0.1 mbar/LSB）、02BA `pressure_pa = raw`（0.01 mbar/LSB = 1 Pa）且 `raw/100 = 1100.02 mbar`；24 位 ADC 按 MSB first 组装（仿真 0x123456 原样出现在样本里）；负水深不被夹到 0 且与 `(P−P0)/(rho·g)` 一致 |
| `InterruptModeAsync` | 总线层异步核心：延迟完成模式下提交后 tick 不推进（零等待）、事务在飞拒复用、Tx 完成回调自动接 Rx、49 ms 不中止 / 50 ms 必须 Abort（1000 ms 被钳到上限）、离线由 ErrorCallback 结束、只读事务与参数校验、未绑定句柄拒绝 |
| `BusTimeout`（改） | 中断模式下改为：提交不消耗时间（elapsed==0）、deadline 到点后由 `I2c_Process` 请求 Abort、`AbortCplt` 落地超时、错误码为 `ERR_TIMEOUT`；卡死期间每次 Process 最多消耗一个超时窗口 |
| `OfflineDevice`（改） | 异步语义：`Init()` 只保证“命令已提交”，NACK 在下一次 `Process()` 落地为 `ERR_IO` + 在线位清零 |
| `BusyDeviceNoEarlyRestart`（改） | 场景等待改用 `wait_state()`（提交→分派同拍，不推进 tick）；异步下允许状态转换恰好落在芯片完成那一拍（`<= 0`），关键不变量（busy 重发/提前读/配对）仍为 0 |
| `CycleTimingAt25Hz`（改） | 异步后样本时间戳整体平移约 1 ms（97/137/177/…），**40 ms 间隔与最小等待 12 ms ≥ 11 ms 不变** |
| `RandomizedInvariants`（改） | 步数 3000→6000 且提高 Process 调用概率，覆盖异步后“每个事务需要两次 Process”的路径；5 seeds 无违规 |

| `Crc4`（扩充） | 计算 CRC **不得破坏**调用方保存的原始 PROM：字 0 高 4 位原 CRC 与第 8 个软件辅助字在调用后必须逐字节不变 |
| `RestoreDefaultsDuringConversion` | **自查发现的缺陷回归**：`RESTORE_DEFAULTS` 会改 OSR/型号，必须与 `SetOsr`/`SetModel` 一样延迟丢弃半周期（`DISCARD_WAIT`、不重发转换、D1/D2 配对不错位）；且型号回到 unknown 后不得再保留 `PRESSURE_VALID`/`TEMPERATURE_VALID` 与旧压力值（必须 NaN） |
| `RandomizedInvariants` | 确定性 LCG、5 个种子 × 3000 步，随机组合：推进主循环、改 OSR/型号/采样率、`RESTORE_DEFAULTS`、改零点/滤波、通用 `SetParam`/`GetParam`、重新 `Init`、随机离线/卡死、tick 停滞或大步跳（含超期不追赶）。每步检查全局不变量：busy 期间不得重发/读、D1/D2 必须同 OSR、型号 unknown 不得报压力有效、零点无效不得报深度有效、`ZERO` 与 `GET_PARAMETER(0103)` 范围口径必须一致、**标志有效则数值不得为 NaN**、序号单调递增 |
| `ReferenceVectors` | 跨实现校验：`tests/gen_ms5837_reference_vectors.py`（独立 Python，无共享代码）生成 100 条向量（30BA/02BA、物理量与极端输入、TEMP=1999/2000/2001/−1499/−1500/−1501 分支边界），逐条比对 `Ms5837_Compensate()` 的整数输出 |

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
| 10) **（上一轮）** 手册第 11 页：转换期间芯片一直 busy，重发转换/提前读 ADC 都会得到错误结果——“丢弃软件半周期”不能等于“立刻重发” | `ms5837_abort_half_cycle()` 改为**延迟丢弃**：只清半截 D1/D2 并进入新状态 `DISCARD_WAIT`，**保留原 deadline**（按旧配置算、不会更短）；到点后（芯片已空闲）才用新配置重新发 D1。`Ms5837_Process()` 增加 `DISCARD_WAIT` 分支 | 新增 `BusyDeviceNoEarlyRestart`（8 场景）先复现失败：修复前 6/8 报“busy 期间重发”，修复后 8/8 全 0 违规 |
| 11) **（本轮自查）** `Ms5837_RestoreDefaults()` 也改 OSR/型号，却**没有**丢弃半周期 → D1/D2 会跨配置配对 | 在 `RestoreDefaults()` 末尾调用 `ms5837_abort_half_cycle()`（与 `SetOsr`/`SetModel` 同一条延迟丢弃路径） | 新增 `RestoreDefaultsDuringConversion` + 仿真新增“D1/D2 必须同 OSR”配对不变量：修复前 `state == DISCARD_WAIT` 失败，修复后通过 |
| 12) **（本轮自查）** `RestoreDefaults()` 把型号退回 unknown 后，仍保留 `PRESSURE_VALID`/`TEMPERATURE_VALID` 与旧压力值 → “有效假数据” | 抽出 `ms5837_invalidate_compensated()`（清 3 个有效位 + 测量字段置 NaN + 清滤波），`SetModel` 与 `RestoreDefaults` 共用 | 由 `RandomizedInvariants` 的随机序列在 step=1288 命中（action=RESTORE_DEFAULTS，无 busy/配对违规），再用针对性断言定位并修复；修复后 5 seeds × 3000 步 0 违规 |
| 13) **（本轮）** 用户要求 I2C 改中断模式并“主循环一点都不用等” | 总线层新增 IT 异步核心（`I2c_Submit/GetPhase/GetResult/Process` + 4 个 HAL 回调 + `I2C3_EV/ER_IRQHandler` + `I2c_Init()` 内使能 NVIC + 超时 `HAL_I2C_Master_Abort_IT`）；驱动的 5 个事务点全部改为“提交→BUS_WAIT 分派”，`Ms5837_Process()` 内调用 `I2c_Process()`，集成方**无需改任何文件** | `InterruptModeAsync` + 全套回归；ARM 交叉编译确认中断服务与 NVIC 进入目标文件；驱动已无任何 `I2c_Write/WriteRead` 调用 |
| 14) **（本轮自查）** 异步引入的死锁：事务在飞时 `Init()/SetModel()` 使状态机离开 `BUS_WAIT`，已完成结果没人取走 → 总线层停在 `DONE`，后续每次提交被拒 | `Ms5837_Process()` 增加“弃单清理”：状态不是 `BUS_WAIT` 而总线 `DONE` 时取走丢弃；`abort_half_cycle()` 覆盖“在飞的是 ADC 读”（立即重开）与“在飞的是转换命令”（保守补足等待） | 随机不变量测试一度 0 样本即暴露该卡死；修复后 5 seeds × 6000 步共 23 个样本、0 违规 |
| 15) **（本轮自查）** 测试自身缺陷：`sim_it_defer`/`sim.stall` 在 CHECK 提前返回时泄漏，导致后续同步自旋永远等不到完成（挂死） | 断言前先恢复现场（延后到本地变量再 CHECK） | 修复后全部用例在秒级完成 |
| 16) **（本轮）** 参考实现对照：C 驱动与独立 Python 仍在 100 条向量上逐条一致（异步改造不影响补偿数学） | 无需改动 | `ReferenceVectors` 100/100 |
| 17) **（上一轮自查）** 参考向量生成器第一版把 30BA 的**高温分支**套用到了 02BA 上 | 生成器改为“只有 30BA 有高温分支”（02BA 手册第 8 页无高温项） | `ReferenceVectors` 逐条比对时先报出 `向量 50 不一致: 期望 temp=2987 实得 2989`，确认是生成器错、驱动对；修正后 100/100 一致 |

## 5. 未做 / 未验证（重要）

* **硬件 NOT TESTED**：本分支没有烧录、没有复位设备、没有启动/终止 OpenOCD、没有打开 COM11/COM42，
  也没有任何真实 I2C3 波形或真实 MS5837 读数。全部结论来自**数据手册源码核对 + 主机仿真 + 交叉编译**，
  与**实机验证**严格区分：本轮只完成前者。
* **型号**：本次实物以用户提供的 MS5837-02BA01（06/2017）手册为板级选型依据，集成时由统筹方**显式 `SetModel(2)`**；
  驱动保留 unknown 默认，**不凭 PROM 猜型号**（`C1` 阈值法不是手册定义行为）。
* **水面 P0 未采集**：驱动不会自动归零；必须由操作员确认水面位置后显式 `ZERO_DEPTH`/`SetSurfacePressurePa()`。
* 02BA 定量范围只按手册原文记录：工作范围 300..1200 mbar、扩展/ADC 线性范围 10..2000 mbar、耐压 10 bar；
  **不宣称 0..10 bar 或 0..10 m 全程精度**；第 1 页 “13 cm” 是空气高度分辨率，不是水深分辨率。
* 未做：`SAVE_CONFIG` 持久化、温度校准（MS5837 无该命令，应回 UNSUPPORTED）、多实例、DMA/中断收发、
  GPIO 脉冲式总线恢复动作（手册第 10 页提到 SDA 卡 ACK 时的处理方式，本轮不引入）。
* 复位后 10 ms 延时是保守余量（数据手册未给具体数值），未在硬件上测过最短安全值。
* 集成方仍需完成：`.ioc` 使能 I2C3(PA8/PC9) 与 CubeMX 的 `MX_I2C3_Init`/MspInit、`main.c` 里
  `I2c_Init(&hi2c3)` + `Ms5837_Init()` + 主循环 `Ms5837_Process()`、`CMakeLists.txt` 增加
  `Core/Src/sensor_i2c_bus.c` 与 `Core/Src/ms5837.c` —— 清单见 `docs/ms5837.md` 第 5 节。
* 工具链限制：本机 w64devkit 未带 ASan/UBSan 运行库，无法跑 sanitizer；改用 `-O0`/`-O2` + `-Werror`
  加独立参考实现交叉验证，并以 `arm-none-eabi-gcc -Werror` 交叉编译确认无告警。

## 5.1 按需上传（单次查询）——用户 2026-10-06 要求的接线规范

用户要求：默认**完全不主动上传**深度数据；只上传**压力 + 温度 + 深度 + 数据产生时刻时间戳**；
**收到专用指令才回一帧**。规范见 `docs/ms5837.md` 第 8 节（新命令 `CMD 0x0D` / 回复 `0x4D`，
负载 21 字节；并列出 `sensor_protocol.h/.c` 与 `sensor_service.c:189/219-222/240-251` 的确切改动点）。

**驱动侧无需改动**：`Ms5837Sample_t.timestamp_ms` 本身就是“数据产生时刻”（D2 结果读完那一刻，
`ms5837.c:686`）；驱动继续 25 Hz 采样，保证查询取到的是最新样本（≤40 ms 旧）。
8.3 节同时说明这是破坏性协议变更，GUI 必须同步从解析 0x82(32B) 改为解析 0x4D(21B)。

## 5.2 折叠为两文件（2026-10-06，用户要求）

用户要求：深度计只保留 **`Core/Inc/ms5837.h` + `Core/Src/ms5837.c` 两个文件**，测试文件全部删除，准备并入主分支。

* I2C 事务层（原 `sensor_i2c_bus.c/h`）整体并入这两个文件：对外只留
  `I2c_Init/I2c_IsReady/I2c_SetDevice/I2c_GetDevice` + `I2c_EvIrqHandler/I2c_ErIrqHandler`；
  `I2c_Submit/GetPhase/GetResult/Process` 改为文件内 static；同步包装 `I2c_Write/Read/WriteRead`
  与只被测试使用的 `I2c_IsBusy` 删除；`I2C_HOST_TEST` 替身机制随测试一起移除。
* **等价性做法（无逻辑漂移的证明）**：把 `git HEAD`（折叠前）的 4 个文件导出到临时目录，
  用同一个折叠脚本生成结果，与工作树里的折叠结果**逐字节比较 → 完全一致**
  （`ms5837.c` 62153 B、`ms5837.h` 18509 B 两侧相同）；因此折叠只改了文件组织，没有改任何逻辑。
* 折叠后 ARM 交叉编译 `arm-none-eabi-gcc -mcpu=cortex-m7 -Werror` → exit 0（text 5916 / data 193 / bss 24），
  目标文件里仍有 `I2C3_EV_IRQHandler`、`I2C3_ER_IRQHandler`、4 个 HAL 回调、`I2c_Init`、`Ms5837_Init`。
* 主机测试（31 个用例）与参考向量表、生成脚本、跑测脚本**已删除**；如需要可用
  `git show 2e51f45:tests/ms5837_host_test.c` 等从历史取回（该提交保留了折叠前的完整测试）。
* 并入主分支只需：新增 2 个文件；删掉 `Core/Src/sensor_board.c:3` 与 `tests/sensor_service_host_test.c:3`
  的 `#include "sensor_i2c_bus.h"`；删掉根 `CMakeLists.txt:56` 的 `Core/Src/sensor_i2c_bus.c`。

## 6. 提交

* 分支：`feature/depth-ms5837-20261006`
* 实现提交：`1130aca` “功能：新增MS5837深度计I2C3驱动与主机测试（02BA/30BA补偿、CRC4、非阻塞转换、显式零点）”
  （8 个文件、3547 行新增；总线文件名在评审后由 `i2c.c/i2c.h` 改名为 `sensor_i2c_bus.c/.h`）。
* 文档提交：`e53785f` “文档：记录MS5837深度计实现提交与验证证据”。
* 评审修正提交 1：`d753e54` “修正：传感器总线改名以避开CubeMX i2c.c/h，并固化PROM 7字读取断言”。
* 评审修正提交 2：`7d73313` “修正：周期调度不累加转换等待、命令后重取tick加余量，并拒绝假PROM/无效转换”。
* 评审修正提交 3：`b89c60d` “修正：型号切换清零点、Zero 复用范围校验、转换途中改配置丢弃半周期”。
* 本轮（02BA01 06/2017 手册核对）提交：本文件所在提交
  “按02BA01(06/2017)手册核对一阶/二阶与PROM，转换中改配置改为延迟丢弃”，只包含
  `Core/Src/ms5837.c`、`tests/ms5837_host_test.c`、`docs/ms5837.md`、`docs/ms5837-progress.md`
  （`Core/Inc/ms5837.h`、`sensor_i2c_bus.*` 本轮无需改动）。
* 本轮使用的任务输入 `docs/MS5837-02BA01-user-manual-task-20261006.md` 由统筹方放入工作树，
  非本人文件，**未纳入提交**（保持未跟踪，原样保留给统筹方）。
* 只提交了本模块自己的文件（未 `git add .`，未改动/回退他人文件，未 push，未合并 main）。
* 提交时 Git 提示 “LF will be replaced by CRLF”，来自本机 `core.autocrlf=true`，与仓库既有文件一致。
