/**
 * @file can_gateway_protocol.h
 * @brief AA55 外部 CAN 网关协议的布局和数据路径说明。
 *
 * 具体状态机和队列实现位于 can_gateway_core.c；本文件只保留不会被
 * CubeMX 重新生成覆盖的协议契约，供固件、上位机和抓包工具对照。
 */
#ifndef __CAN_GATEWAY_PROTOCOL_H__
#define __CAN_GATEWAY_PROTOCOL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "can_gateway_core.h"

/*
 * 外部设备使用的 AA55 CAN 网关报文格式（多字节字段均为小端序）。
 * 该头文件保留协议说明，便于抓包工具、上位机和后续维护人员对照代码。
 *
 * 字节布局（下行：电脑 -> 板卡）：
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
 * 字节布局（上行：板卡 -> 电脑，在 DATA 之后追加时间戳）：
 *   [11..]    DATA               同上
 *   [11+N]    TIMESTAMP_US(4)    32 位微秒时间戳，小端；毫秒时基×1000，
 *                                约 71.6 分钟回绕；CAN 上报帧记录中断
 *                                捕获时刻，状态/提示帧记录封装时刻
 *   [15+N]    CRC8-ATM           覆盖 [2] 至 TIMESTAMP 末尾（BODY_LEN+1 字节）
 *   [16+N]    0x55 0xAA          固定结束字节
 *
 * BODY_LEN 下行为 8+LEN、上行为 12+LEN；两类帧总长均为 BODY_LEN+6，
 * 解析器允许分包、半包和粘包，收到完整帧后才校验帧尾/CRC。普通 CAN 数据路径如下：
 *   外部设备 -> 接收环形缓冲 -> AA55 解析 -> CAN 软件队列 -> FDCAN1 TX FIFO
 *   FDCAN1 RX FIFO -> CAN 软件队列 -> AA55 封装 -> 发送队列 -> 外部设备
 *
 * 函数指针解耦后的新入口位于 can_gateway_core.h：应用只需注册传输操作表，
 * 业务代码只需注册接口并调用统一入口，不要直接访问具体传输驱动。
 */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_GATEWAY_PROTOCOL_H__ */
