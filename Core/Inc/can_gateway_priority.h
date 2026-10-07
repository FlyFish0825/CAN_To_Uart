#ifndef __CAN_GATEWAY_PRIORITY_H__
#define __CAN_GATEWAY_PRIORITY_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief 判断一个 CAN ID 是否属于电机优先级范围。
 * @param id CAN 标识符（标准帧/扩展帧统一按 32 位数值比较）。
 * @return 命中任一优先级范围返回 1，否则返回 0。
 *
 * 命中的帧在收发两个方向都走高优先级队列：
 * - 下行（上位机 -> CAN）：先于普通命令和 AA59 固件块提交总线；
 * - 上行（CAN -> 上位机）：先于普通 CAN 数据与 AA5B/AA58 上报。
 *
 * 实现为常量表上的线性扫描，无共享状态，可在中断上下文安全调用。
 * 范围表在 can_gateway_priority.c 中维护，适配具体机器人时只需修改该表。
 */
uint8_t CanGateway_IsMotorPriorityId(uint32_t id);

#ifdef __cplusplus
}
#endif

#endif /* __CAN_GATEWAY_PRIORITY_H__ */
