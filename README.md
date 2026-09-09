# STM32H750 CAN / UART 网关使用说明

本项目通过 USART1 接收电脑命令，将 CAN 帧提交到 FDCAN1；也包含 CAN 接收转串口功能。当前采用帧头、长度、CRC、帧尾协议，以及 UART DMA 和循环缓冲区。

## 1. 快速上手

1. 编译并烧录本项目固件，按下表连接 USB 转串口模块与板卡，双方共地。
2. 串口助手设置为 **921600、8 数据位、无校验、1 停止位、无流控**，发送与接收均选择 **Hex**，发送时选择“无追加”（不加换行）。
3. 板卡复位后会通过串口输出 `UART_TX!` 启动提示。该提示不发送到 CAN 总线。
4. 将下面完整的 22 字节复制到发送框，单击发送：

```text
AA 55 10 01 00 23 01 00 00 00 08 11 22 33 44 55 66 77 88 BC 55 AA
```

这条命令发送标准经典 CAN 数据帧：ID 为 `0x123`，8 字节数据为 `11 22 33 44 55 66 77 88`，默认 CAN 速率为 **500 kbit/s**。

低速手动单次测试通常会看到 `UART_RX!` 和 `CAN_PUT!` 两条状态。它们证明串口解析、软件入队和硬件 FIFO 提交已执行，**不能证明物理总线发送成功或其他节点收到数据**。

## 2. 接线与默认配置

| 接口 | MCU 引脚 | 接法 / 用途 |
| --- | --- | --- |
| USART1 TX | PA9 | 接 USB 转串口模块 RX，3.3 V TTL 电平 |
| USART1 RX | PA10 | 接 USB 转串口模块 TX，3.3 V TTL 电平 |
| GND | GND | 板卡与串口模块共地 |
| FDCAN1 TX | PD1 | MCU 到 CAN 收发器 TXD |
| FDCAN1 RX | PD0 | CAN 收发器 RXD 到 MCU |
| CANH / CANL | 收发器总线侧 | 接 CAN 总线，不能接 TTL 串口 |

本固件使用 USART1，不提供 USB CDC 虚拟串口功能；不能把板上原生 USB 接口直接当成这里的串口。请根据实际板卡原理图确认连接器引脚。

FDCAN 内核时钟配置为 80 MHz；默认仲裁段 500 kbit/s、FD 数据段 5 Mbit/s。工作模式为 Normal，自动重发关闭。经典 CAN 帧不使用 FD 数据段速率。支持 FD 报文的软件配置不等于板卡收发器及布线已通过对应高速验证。

## 3. 普通 CAN 帧的串口协议

电脑发送与板卡上报的普通 CAN 帧使用同一格式。所有多字节整数均为**小端序**。

```text
AA 55 | BODY_LEN | SEQ(2) | CAN_ID(4) | FLAGS | LEN | DATA(N) | CRC8 | 55 AA
```

| 字节偏移（从 0 开始） | 字节数 | 含义 |
| --- | --- | --- |
| 0 | 2 | 帧头 `AA 55` |
| 2 | 1 | BODY_LEN，紧凑格式为 `8 + N` |
| 3 | 2 | SEQ，16 位序号 |
| 5 | 4 | CAN ID，标准帧 0..0x7FF，扩展帧 0..0x1FFFFFFF |
| 9 | 1 | FLAGS，见下表 |
| 10 | 1 | LEN，实际数据字节数 N，不是 CAN 硬件 DLC 编码 |
| 11 | N | DATA |
| 11 + N | 1 | CRC8 |
| 12 + N | 2 | 帧尾 `55 AA` |

紧凑格式总长度为 `14 + N`，也就是 `BODY_LEN + 6`。上例 BODY_LEN=`0x10`，LEN=`0x08`，总长 22 字节。

| FLAGS 位 | 数值 | 含义 |
| --- | --- | --- |
| bit0 | 0x01 | 扩展 ID，否则标准 ID |
| bit1 | 0x02 | CAN FD，否则经典 CAN |
| bit2 | 0x04 | BRS 数据段变速，仅 FD 可用 |
| bit3 | 0x08 | 远程帧，仅经典 CAN 可用 |

常用 FLAGS：`00` 标准经典 CAN，`01` 扩展经典 CAN，`06` 标准 FD+BRS，`07` 扩展 FD+BRS。普通 CAN 帧其余位须为 0；`0x80` 留给配置命令。

经典 CAN 的 LEN 为 0..8；FD 为 0..8、12、16、20、24、32、48、64。FD 不支持远程帧。远程帧协议仍携带 LEN 个占位字节，建议填零，它们不作为 CAN 远程帧的数据发送。

接收端还接受 BODY_LEN=`0x48` 的固定长度格式：DATA 区占满 64 字节，不足部分填零，CRC 覆盖填充字节，总长 78 字节。优先使用紧凑格式。**旧版无帧尾的 76 字节报文不兼容，不能直接发送。**

数据中允许出现 `AA 55` 或 `55 AA`，无需转义；解析器根据 BODY_LEN 定位帧尾。串口读操作可能得到半帧或多帧，上位机应累计字节后按长度拆包，不能把一次串口读取当成一帧。

普通输入帧的 SEQ 不用于去重，重复发送同一 SEQ 会再次入队。板卡普通输出使用自己的递增序号，不回显命令 SEQ；配置回复例外。

### CRC 与生成示例

CRC-8：多项式 `0x07`，初值 `0x00`，输入/输出均不反射，最终异或 `0x00`。计算范围从 BODY_LEN 到 BODY 最后一个字节，包含长度字节，共 `BODY_LEN + 1` 字节；不包含帧头、CRC 自身和帧尾。

以下 Python 3 代码无需第三方库，只生成十六进制报文，不打开串口。修改 ID、FLAGS、DATA 后必须重新计算 CRC。

```python
def crc8(data):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ (0x07 if crc & 0x80 else 0)) & 0xFF
    return crc


def wrap(body):
    checked = bytes([len(body)]) + body
    return b'\xAA\x55' + checked + bytes([crc8(checked)]) + b'\x55\xAA'


def can_packet(can_id, data, seq=1, flags=0):
    data = bytes(data)
    assert 0 <= seq <= 0xFFFF
    assert 0 <= flags <= 0x0F
    assert 0 <= can_id <= (0x1FFFFFFF if flags & 1 else 0x7FF)
    if flags & 2:
        assert not flags & 8
        assert len(data) in (*range(9), 12, 16, 20, 24, 32, 48, 64)
    else:
        assert not flags & 4
        assert len(data) <= 8
    body = (seq.to_bytes(2, 'little') + can_id.to_bytes(4, 'little')
            + bytes([flags, len(data)]) + data)
    return wrap(body)


packet = can_packet(0x123, bytes.fromhex('11 22 33 44 55 66 77 88'))
assert packet == bytes.fromhex(
    'AA 55 10 01 00 23 01 00 00 00 08 11 22 33 44 55 66 77 88 BC 55 AA')
print(packet.hex(' ').upper())
```

## 4. 状态回复怎么读

状态包只在串口输出，使用普通 CAN 帧形状，FLAGS=0，LEN=8。

| ID | DATA 的 ASCII 内容 | 含义 |
| --- | --- | --- |
| 0x7FF | UART_TX! | 启动提示，串口发送链路有输出 |
| 0x7FE | UART_RX! | 输入校验通过且已进入 CAN 软件发送队列 |
| 0x7FD | CAN_PUT! | HAL 已接受帧并写入 CAN 硬件发送 FIFO |
| 0x7FC | CRC_ERR! | CRC 不匹配 |
| 0x7FB | PKT_ERR! | 帧尾、长度、标志、ID 或命令等格式错误 |
| 0x7FA | CAN_FAIL | HAL 提交 CAN 发送 FIFO 失败 |

这些是调试事件提示，不是逐帧可靠 ACK：主循环会合并相同类型的多次事件；输出队列满时状态包也可能丢失。不能按提示条数统计 CAN 帧数，也不能靠它确认节点回复。

真实 CAN 接收帧会使用相同的串口格式转发，因此总线上 ID=0x7FA..0x7FF 且数据相同的报文可能与提示混淆。当前协议没有独立状态消息类型，上位机集成时应避免此冲突。

## 5. 修改 CAN 速率（可选）

配置命令只在当前运行期间生效，不写入 Flash，复位恢复默认值。仲裁段支持 50000、100000、125000、250000、500000、800000、1000000 bit/s；数据段支持 500000、1000000、2000000、4000000、5000000、8000000 bit/s。

请求格式为：

```text
AA 55 | 10 | SEQ(2) | 00 00 00 00 | 80 | 01 | 仲裁速率(4) | 数据速率(4) | CRC | 55 AA
```

两个速率均为小端 uint32，单位 bit/s。这里偏移 10 是命令字 `01`，不是数据长度。使用上面的 wrap 函数可生成命令：

```python
body = ((2).to_bytes(2, 'little') + bytes(4) + bytes([0x80, 0x01])
        + (500000).to_bytes(4, 'little') + (5000000).to_bytes(4, 'little'))
print(wrap(body).hex(' ').upper())
```

回复共 23 字节，格式为：

```text
AA 55 | 11 | 请求SEQ(2) | 00 00 00 00 | 80 | 81 | STATUS | 当前仲裁速率(4) | 当前数据速率(4) | CRC | 55 AA
```

STATUS：`00` 成功，`01` 不支持的速率，`02` 应用失败。应用失败后的速率字段仅反映软件最后记录值，不保证控制器仍正常工作，应复位排查。切换会停止并重新初始化 FDCAN；停止业务流量后单条发送配置命令，等回复再恢复业务，避免连续配置覆盖待处理命令或影响在途帧。此功能尚未完成板上验证。

## 6. 编译、烧录与维护

需要 CMake（项目要求至少 3.22）、Ninja 和 PATH 中可用的 Arm GNU 工具链 `arm-none-eabi-gcc`。在项目根目录执行：

```powershell
cmake --preset Debug
cmake --build --preset Debug --parallel
cmake --preset Release
cmake --build --preset Release --parallel
```

产物分别为 `build/Debug/CAN_To_Uart.elf`、`build/Release/CAN_To_Uart.elf`。通过 ST-LINK/SWD 和 STM32CubeProgrammer 或调试器加载对应 ELF，完成下载后复位。编译不会自动烧录。不要把构建目录加入版本控制。

| 文件 | 维护内容 |
| --- | --- |
| Core/Src/can_uart_gateway.c | 协议、CRC、环形队列、CAN 转发、状态与速率命令 |
| Core/Inc/can_uart_gateway.h | 网关接口与协议概述 |
| Core/Src/main.c | 网关初始化及主循环调用 |
| Core/Src/usart.c | USART1 参数及 RX/TX DMA 初始化 |
| Core/Src/stm32h7xx_it.c | USART1 和 DMA 中断入口 |
| Core/Src/fdcan.c | CAN 引脚、时钟源、默认时序和 FIFO |
| STM32H750XX_FLASH.ld | .dma_buffer 放置到 RAM_D2 |
| CAN_To_Uart.ioc | CubeMX 外设配置 |
| CMakeLists.txt | 网关源文件加入构建 |

RX 使用 DMA1 Stream0 循环模式，256 字节 DMA 缓冲，通过 IDLE、半满和全满事件搬入软件环形缓冲。软件环形缓冲分配 1024 字节、可用 1023 字节。TX 使用 DMA1 Stream1 普通模式，16 个队列槽、可用 15 个。CAN 收发软件队列各 64 槽、可用 63 帧；CAN 硬件 TX FIFO 为 3 帧。

DMA 缓冲区位于 D2 SRAM，32 字节对齐，避免落入 DMA1 不可访问的 DTCM。当前未启用 D-Cache；后续启用时必须处理 DMA 缓存一致性（非缓存区或正确的缓存维护），仅地址对齐并不足够。保持主循环持续调用 CanUartGateway_Process，避免加入长时间阻塞操作。

CubeMX 再生成代码后检查 DMA 初始化顺序、中断入口、链接脚本 .dma_buffer 和 CMake 网关源文件条目；当前再生成流程未验证，不能仅凭 .ioc 存在就认为自定义内容一定保留。

## 7. 排查与验证范围

| 现象 | 检查方向 |
| --- | --- |
| 复位没有 UART_TX! | 确认固件已下载、COM 口、921600/8N1、PA9 接模块 RX、共地及供电 |
| 有启动提示，发送没有 UART_RX! | 检查模块 TX 到 PA10，发送 Hex、无追加、完整帧长和帧尾 |
| CRC_ERR! | ID/序号/数据变化后是否重算 CRC；是否遗漏字节或混用旧协议 |
| PKT_ERR! | BODY_LEN、LEN、帧尾、ID 范围和 FLAGS 组合是否合法 |
| 有 UART_RX!，没有 CAN_PUT! | 检查 CAN FIFO 是否满、控制器状态和发送队列计数 |
| 有 CAN_PUT!，总线无波形 | 先测 PD1，再测收发器 CANH/CANL；核对供电、待机脚、连接的 CAN 通道及终端电阻 |

可以在没有其他节点回复的情况下用示波器检查发送尝试；CAN_PUT! 不代表收到 ACK。当前未实现总线错误状态上报或 bus-off 自动恢复，也未证明持续无 ACK 下的长期运行行为。

缓冲有限且没有流控，持续输入超过处理能力会丢数据。调试器可观察 `uart_rx_ring_drop_count`、`uart_tx_queue_drop_count`、`can_tx_drop_count`、`can_rx_drop_count`、`can_rx_hw_lost_count`、`uart_rx_error_count` 和 `can_tx_fail_count`。这些计数目前不通过独立查询命令输出。解析器没有半帧超时；输入残缺帧可能影响后续帧对齐，手工测试遇到此情况可复位后发送完整报文。

截至本次交接：Debug/Release 已编译通过；用户板上截图显示启动提示，且两次完整 22 字节命令均产生 UART_RX! 与 CAN_PUT!。CAN 物理波形、节点接收、CAN 转串口方向、FD 高速、速率切换、长时间连续流量以及异常恢复仍需板上验证。
