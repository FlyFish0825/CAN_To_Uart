#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * 极简 I2C 主设备事务层。
 *
 * 设计目标只有两条：
 *  1. 每次事务都带明确的毫秒超时，最坏情况下不会把主循环卡住；
 *  2. 让 MS5837 这类从设备驱动可以脱离 HAL 做主机单元测试。
 * 这里不做 DMA/中断异步收发：MS5837 的“非阻塞”体现在转换等待由状态机负责，
 * 单次寄存器读写本身只有几字节，用短超时同步事务即可。
 */
#if defined(I2C_HOST_TEST)
/* 主机测试用最小替身，避免把 Cortex-M7 寄存器头文件带入 Windows 编译器。 */
typedef int HAL_StatusTypeDef;
typedef struct
{
  uint32_t host_stub;
} I2C_HandleTypeDef;

#ifndef HAL_OK
#define HAL_OK 0
#endif
#ifndef HAL_ERROR
#define HAL_ERROR 1
#endif
#ifndef HAL_BUSY
#define HAL_BUSY 2
#endif
#ifndef HAL_TIMEOUT
#define HAL_TIMEOUT 3
#endif

/* 主机测试必须提供下面三个替身实现（见 tests/ms5837_host_test.c 的 I2C 从设备仿真）。 */
uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c,
                                          uint16_t dev_address,
                                          uint8_t *data,
                                          uint16_t size,
                                          uint32_t timeout);
HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *hi2c,
                                         uint16_t dev_address,
                                         uint8_t *data,
                                         uint16_t size,
                                         uint32_t timeout);
#else
#include "stm32h7xx_hal.h"
#endif

#define I2C_BUS_DEFAULT_TIMEOUT_MS 5U /* 未显式指定时单次事务的默认超时。 */
#define I2C_BUS_MAX_TIMEOUT_MS     50U /* 单次事务允许的最大超时上限。 */
#define I2C_BUS_MAX_TRANSFER       16U /* 一次事务允许的最大字节数。 */
#define I2C_BUS_MS5837_ADDRESS7    0x76U /* MS5837 的 7 位地址（HAL 写地址 0xEC）。 */

/* 事务结果。PARAM/NOT_READY 表示调用方式错误，其余来自 HAL 状态。 */
typedef enum
{
  I2C_BUS_OK = 0, /* 事务完成。 */
  I2C_BUS_NOT_READY, /* 尚未绑定 I2C 句柄。 */
  I2C_BUS_PARAM, /* 地址、长度或缓冲区非法。 */
  I2C_BUS_BUSY, /* 总线被占用。 */
  I2C_BUS_TIMEOUT, /* 事务在超时时间内未完成。 */
  I2C_BUS_ERROR /* NACK、仲裁丢失等总线错误。 */
} I2cBusResult_t;

/**
 * @brief 绑定 HAL I2C 句柄。传 NULL 表示卸载。
 *
 * 只保存指针，不在本函数内部访问硬件寄存器；MX_I2C3_Init 由集成方在 main.c 中调用。
 */
void I2c_Init(I2C_HandleTypeDef *handle);

/** @brief 是否已绑定句柄。 */
uint8_t I2c_IsReady(void);

/**
 * @brief 设置当前 7 位从地址，合法范围 0x08~0x77。
 */
I2cBusResult_t I2c_SetDevice(uint8_t address7);

/** @brief 读取当前 7 位从地址。 */
uint8_t I2c_GetDevice(void);

/**
 * @brief 写事务：START + 从地址(写) + data + STOP。
 *
 * timeout_ms 传 0 使用默认值，超过 I2C_BUS_MAX_TIMEOUT_MS 时被截断到上限，
 * 保证任何单次调用都不会长时间阻塞调用者。
 */
I2cBusResult_t I2c_Write(const uint8_t *data,
                         uint16_t length,
                         uint32_t timeout_ms);

/**
 * @brief 读事务：START + 从地址(读) + data + STOP。
 */
I2cBusResult_t I2c_Read(uint8_t *data,
                        uint16_t length,
                        uint32_t timeout_ms);

/**
 * @brief 先写后读（两次独立事务，各自带同一个短超时）。
 *
 * MS5837 的命令—数据读取就是这种形态：命令帧以 STOP 结束，随后单独发起读事务。
 */
I2cBusResult_t I2c_WriteRead(const uint8_t *tx,
                             uint16_t tx_length,
                             uint8_t *rx,
                             uint16_t rx_length,
                             uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H__ */
