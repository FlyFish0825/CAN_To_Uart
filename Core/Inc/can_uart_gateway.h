#ifndef __CAN_UART_GATEWAY_H__
#define __CAN_UART_GATEWAY_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "can_gateway_core.h"

/*
 * USB CDC 使用的 AA55 CAN 网关报文格式（多字节字段均为小端序）。
 * 该头文件保留协议说明，便于抓包工具、上位机和后续维护人员对照代码。
 *
 * 字节布局：
 *   [0]       0xAA              固定起始字节 0
 *   [1]       0x55              固定起始字节 1
 *   [2]       BODY_LEN          从 SEQ 到 DATA 末尾的字节数（8 + LEN）
 *   [3..4]    SEQ                16 位序号，小端；发送方向独立递增
 *   [5..8]    CAN_ID             32 位 CAN 标识符，小端
 *   [9]       FLAGS              帧类型位图：bit0 扩展 ID、bit1 FD、
 *                                bit2 BRS、bit3 远程帧；bit7=1 为本地配置
 *   [10]      LEN                DATA 有效长度；经典 CAN 为 0..8，
 *                                CAN FD 可为 0..8/12/16/20/24/32/48/64
 *   [11..]    DATA               CAN 数据；未使用的固定包区域填 0
 *   [...]     CRC8-ATM           CRC 多项式 0x07，覆盖 [2] 至 DATA 末尾
 *   [...]     0x55              固定结束字节 0
 *   [...]     0xAA              固定结束字节 1
 *
 * BODY_LEN=8+LEN 是变长协议的有效体长度；解析器允许 USB 分包、半包和
 * 粘包，收到完整帧后才校验帧尾/CRC。普通 CAN 数据路径如下：
 *   电脑 -> USB RX Ring -> AA55 解析 -> CAN 软件队列 -> FDCAN1 TX FIFO
 *   FDCAN1 RX FIFO -> CAN 软件队列 -> AA55 封装 -> USB TX Queue -> 电脑
 *
 * 函数指针解耦后的新入口位于 can_gateway_core.h：应用只需注册传输操作表，
 * 不要在业务代码中直接调用 USB CDC 函数。
 */

/**
 * @brief 旧工程主循环入口的兼容包装函数。
 *
 * 新代码应调用 CanGateway_Process()。保留本声明是为了让尚未迁移的旧
 * 模块继续编译；实现只转发到新核心，不会创建第二套协议状态机。
 */
void CanUartGateway_Process(void);

/**
 * @brief 旧工程协议输入入口的兼容包装函数。
 *
 * 新传输驱动应调用 CanGateway_RxFeed()。data 可以是半帧、整帧或粘包，
 * len 是本次输入的实际字节数；实现只转发到统一 AA55 解析器。
 */
void CanUartGateway_ProtocolFeed(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __CAN_UART_GATEWAY_H__ */
