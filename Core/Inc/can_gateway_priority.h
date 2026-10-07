#ifndef __CAN_GATEWAY_PRIORITY_H__
#define __CAN_GATEWAY_PRIORITY_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief 判断一个 CAN ID 是否属于电机控制通道（仅用于下行发送方向）。
 * @param id CAN 标识符。
 * @return 命中控制通道返回 1，否则返回 0。
 *
 * 依据电机控制器（Observer_Motor）协议《04-通信与调试》§8：
 * CAN ID 0x100 为控制通道——FD/24 字节多节点控制向量与 Classic/8 字节
 * 兼容旧单节点命令共用该 ID。注意电机节点号不在 CAN ID 里（向量帧在
 * DATA Byte2 的节点位图、Classic 帧在 DATA Byte0），但优先级调度只需
 * 区分"是否控制帧"，不需要知道目标节点。
 *
 * 下行命中者走高优先级队列，先于普通命令和 AA59 固件块提交总线；
 * 上行反馈帧（0x200+node 普通反馈、0x280+node 心跳、0x300+node 调试、
 * 0x380+node HELLO）一律普通队列，不调用本函数。
 *
 * 实现为常量表上的线性扫描，无共享状态，可在主循环安全调用。
 * 范围表在 can_gateway_priority.c 中维护，需要扩展（例如把 0x000
 * ENTER_BOOT 管理帧也提前）时修改该表即可。
 */
uint8_t CanGateway_IsMotorControlId(uint32_t id);

#ifdef __cplusplus
}
#endif

#endif /* __CAN_GATEWAY_PRIORITY_H__ */
