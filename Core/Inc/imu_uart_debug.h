/**
 * @file imu_uart_debug.h
 * @brief 【imu-uart1-debug 测试分支】IMU 串口读取调试接口。
 *
 * 该模块只存在于调试分支：把 USART1（PA9/PA10，115200）收到的原始字节
 * 以十六进制文本行写入 USB CDC 发送队列，供电脑端串口工具直接观察 IMU
 * 输出。模块不解析 IMU 协议，也不参与正式网关数据路径。
 */
#ifndef __IMU_UART_DEBUG_H__
#define __IMU_UART_DEBUG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/**
 * @brief 初始化 USART1 并启动 DMA 环形接收。
 * @return HAL_OK 启动成功；HAL_ERROR DMA 接收启动失败。
 *
 * 内部会调用 CubeMX 生成的 MX_DMA_Init() 和 MX_USART1_UART_Init()，
 * 必须在 USB 传输层初始化之后、主循环启动前调用一次。
 */
HAL_StatusTypeDef ImuUartDebug_Init(void);

/**
 * @brief 主循环轮询：取出 DMA 新收到的字节并按行打印到 CDC 发送队列。
 *
 * 函数非阻塞。CDC 发送队列满时已收到的数据保留在模块缓冲内，下一轮继
 * 续尝试；模块缓冲也无法容纳时丢弃新数据并计数，空闲状态行会汇报丢弃
 * 数量，便于发现上位机长时间不取数据造成的堆积。
 */
void ImuUartDebug_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* __IMU_UART_DEBUG_H__ */
