#include "can_gateway_priority.h"

/*
 * 电机控制通道 CAN ID 范围表（闭区间，含两端）。
 *
 * 依据电机控制器（Observer_Motor）协议《04-通信与调试》§8 的 ID 分配：
 * - 0x100：控制通道（FD/24 多节点控制向量 + Classic/8 兼容旧单节点命令），
 *   电机节点号在 DATA 中（向量帧 Byte2 节点位图 / Classic 帧 Byte0）；
 * - 反馈族（0x200+node / 0x280+node / 0x300+node / 0x380+node）全部为
 *   上行，走普通队列，不出现在本表。
 *
 * 需要调整时修改下表：每个元素一个 {起始 ID, 结束 ID} 范围，可按需
 * 增删行数。例如想让 0x000（ENTER_BOOT 管理帧）也优先，加一行即可。
 */
typedef struct
{
  uint32_t lo;
  uint32_t hi;
} CanGateway_IdRange_t;

static const CanGateway_IdRange_t motor_control_ranges[] =
{
    { 0x100U, 0x100U },
};

uint8_t CanGateway_IsMotorControlId(uint32_t id)
{
  uint32_t i;

  for (i = 0U;
       i < (sizeof(motor_control_ranges) / sizeof(motor_control_ranges[0]));
       i++)
  {
    if ((id >= motor_control_ranges[i].lo) &&
        (id <= motor_control_ranges[i].hi))
    {
      return 1U;
    }
  }
  return 0U;
}
