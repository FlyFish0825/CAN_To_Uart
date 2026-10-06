/**
 * @file imu_uart_debug.h
 * @brief 【imu-uart1-debug 调试分支】IMU→WCH-Link 串口桥（读数验证用）。
 *
 * 用途：在授权的硬件测试窗口里，经 WCH-Link 的串口桥（COM42）读取 IMU。
 * 数据路径：
 *   IMU TX → PA10（USART1 RX，DMA 循环接收）
 *     → 原始字节逐段回显到 PA9（USART1 TX）→ WCH-Link RX → PC COM42
 *     → 同时喂给正式后端 imu_sensor 做片内解析
 *     → 每秒一行健康统计经 USB CDC（COM11）输出
 *
 * 约束：只收不发——不对 IMU 发送任何原生命令（imu_sensor 保持
 * pins_blocked=1、tx=NULL），无自环自测，无自动校准；PA9 上的唯一输出
 * 是 PA10 收到字节的回显。历史版本（7E23 直打 + RTC/!SYNC 双时钟）已
 * 留档于 da651db，本文件是其在硬件验证阶段的替代实现。
 */
#ifndef __IMU_UART_DEBUG_H__
#define __IMU_UART_DEBUG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/**
 * @brief 初始化 USART1（PA9/PA10，115200）并启动 DMA 循环接收与桥接。
 * @return HAL_OK 启动成功；HAL_ERROR DMA 接收启动失败。
 *
 * 内部会调用 CubeMX 生成的 MX_DMA_Init() 和 MX_USART1_UART_Init()，
 * 必须在 USB 传输层初始化之后、主循环启动前调用一次。
 */
HAL_StatusTypeDef ImuUartDebug_Init(void);

/**
 * @brief 主循环轮询：DMA 游标取新字节 → PA9 回显 → imu_sensor 解析 →
 *        1Hz CDC 统计行。函数非阻塞（回显按 115200 线速短暂阻塞）。
 */
void ImuUartDebug_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* __IMU_UART_DEBUG_H__ */
