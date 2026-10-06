# IMU 实机读取验证（已执行，结果见 build/hardware-test/imu-read-report-20261006.md）

**2026-10-06 结果摘要**：COM42 路线因 WCH-Link 串口物理占用 PA9/PA10 不可用
（插着烧写器 IMU 拉不了低电平，拔掉才有数据），改为 CDC 重组法：桥接固件把
PA10 原始字节以 `IMUWd:<hex>` 行发上 COM11，离线重组后用 imu_sensor 复检。
**实机通过**：raw 2050B/s（25Hz×82B）、good=1503（raw/quat/euler/baro 各 375~376，
25.07Hz）、bad_checksum=1、样本物理自洽（1g 静止、roll 61° 与加速度互洽、四元数
归一化）。控制指令仍未实机发送（需单独授权）。

以下为原始测试计划，留作复现参考。

## 0. 前提与接线（用户已确认/需确认的事实）

- WCH-Link：SWD 用于烧录；其串口桥 RX 接 PA9、TX 接 PA10（COM42 实测已证实）。
- **用户需断开 WCH TX→PA10 共驱线**（统筹 2026-10-06 提出的问题）：断开后 PA10 只剩
  IMU 一个驱动源。若未断开，固件仍会跑，但 PA10 数据可能被干扰，统计行会体现异常。
- IMU 上电即自动上报（默认 25Hz，每周期 0x04/0x16/0x26/0x32 四帧，约 2KB/s）。
- 本固件替换统筹的集成固件；测试结束、统筹要跑集成前需告知统筹重烧（滚回命令见 §4）。

## 1. 烧录（openocd，WCH-Link DAP 模式）

```
cd F:/file/BaiduSyncdisk/Project/CAN_To_Uart/worktrees/imu-uart1-debug
"C:/Program Files/openOCD/bin/openocd.exe" \
    -s "C:/Program Files/openOCD/openocd/scripts" \
    -f tests/wch_flash_openocd.cfg \
    -c "program build/Debug/CAN_To_Uart.elf verify reset exit"
```

预期输出含 `** Programming Finished **` 与 `** Verified OK **`。失败则原样记录错误，
不重试不抢设备。（2026-10-06 已按此命令成功烧录一次：Verified OK。）

## 2. COM42 只读采集（15 秒，RX-only）

```
powershell -NoProfile -ExecutionPolicy Bypass -File tests/imu_capture_com42.ps1
```

（脚本默认 COM42/115200/10s；需要 15s 时传 `-Seconds 15`。）结束后确认输出
`CLOSED ... is-open=False`。产物：`build/hardware-test/imu_com42_10s.bin`。

## 3. 离线解析（复用 imu_sensor 正式实现）

```
gcc -std=c11 -Wall -Wextra -Werror -O0 -ICore/Inc tests/imu_bin_analyze.c -o build/imu_bin_analyze.exe
./build/imu_bin_analyze.exe build/hardware-test/imu_com42_10s.bin
```

**通过判据**（对照协议表）：good_frames ≈ 采集秒数×25×4（±1）；raw/quat/euler/baro
四个计数均 ≈ 25×秒数（十轴四帧齐全）；bad_checksum=0；样本值物理合理（静止时 accel_g
≈ (0,0,±1)，euler 稳定）。结果写 build/hardware-test/ 报告并回报，**注明"已实机验证"
仅限原生协议解码，不含 AA5B/H750 正式集成路径**。

## 4. 故障诊断矩阵

| 现象（CDC 统计行/COM42） | 含义 | 处置 |
|---|---|---|
| `IMUW raw=0` 持续为 0 | PA10 无数据 | 检查 WCH TX→PA10 是否已断开、IMU 供电与 TX 接线；报告，不动硬件 |
| raw 增长但 COM42 无字节 | PA9 回显路径问题 | 检查 COM42 是否被占用（打开失败会报具体错误）；报告 |
| COM42 有字节但 bad_checksum 大量增长 | 波特率/线扰 | 停止，报告原始 .bin，不做参数猜测 |
| 只有部分帧类型计数增长 | 模块为六轴（无 0x32）或模式不对 | 如实报告，不发 0x61 改模式（本次只读） |

## 5. 结束动作

1. 确认 COM42 已关闭（脚本 finally 段保证）。
2. 保存 .bin 与报告到 build/hardware-test/。
3. 向用户与统筹回报结果；**不擅自改模式/频率、不发任何原生命令**（0x60/0x61/0x70/
   0x71/0x73/0xA0 的首次实机发送需单独授权，用统筹的集成固件走 AA5B 更合适）。
4. 统筹需要集成板时：用其 sensor-integration worktree 的固件重烧（由统筹决定）。
