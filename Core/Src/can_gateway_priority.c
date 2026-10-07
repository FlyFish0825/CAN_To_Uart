#include "can_gateway_priority.h"

/*
 * 电机优先级 CAN ID 范围表（闭区间，含两端）。
 *
 * 适配具体机器人时只需修改下表：每个元素一个 {起始 ID, 结束 ID} 范围，
 * 标准帧与扩展帧统一按 32 位数值比较，可按需增删行数。
 *
 * 默认值覆盖 RoboMaster C620/C610/GM6020 电调的常用段：
 * - 控制帧：0x1FF、0x200；
 * - 反馈帧：0x201-0x208。
 */
typedef struct
{
  uint32_t lo;
  uint32_t hi;
} CanGateway_IdRange_t;

static const CanGateway_IdRange_t motor_priority_ranges[] =
{
    { 0x1FFU, 0x208U },
};

uint8_t CanGateway_IsMotorPriorityId(uint32_t id)
{
  uint32_t i;

  for (i = 0U;
       i < (sizeof(motor_priority_ranges) / sizeof(motor_priority_ranges[0]));
       i++)
  {
    if ((id >= motor_priority_ranges[i].lo) &&
        (id <= motor_priority_ranges[i].hi))
    {
      return 1U;
    }
  }
  return 0U;
}
