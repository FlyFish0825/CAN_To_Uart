#ifndef __SENSOR_I2C_BUS_H__
#define __SENSOR_I2C_BUS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * 极简 I2C 主设备事务层（传感器总线包装）。
 *
 * 文件名刻意不叫 i2c.c/i2c.h，避免与 STM32CubeMX 生成的 Core/Src/i2c.c、Core/Inc/i2c.h
 * （里面有 MX_I2C3_Init / HAL_I2C_MspInit）冲突。HAL 层的 I2C3 初始化由集成方负责，
 * 本模块只要求把已经初始化好的句柄通过 I2c_Init() 传进来。
 *
 * 设计目标：
 *  1. 每次事务都带明确的毫秒超时，最坏情况下不会把主循环卡住；
 *  2. 让 MS5837 这类从设备驱动可以脱离 HAL 做主机单元测试；
 *  3. **默认走中断（IT）异步收发**：提交后立即返回，完成/出错由 HAL 回调置位，
 *     主循环用 I2c_Process() 推进超时与中止。同步 API 仍保留，但只应用于
 *     初始化等非实时路径，正式采样路径不得使用（否则又变成阻塞）。
 *
 * 集成方不需要改任何文件：I2c_Init() 会自行使能对应 I2C 的 EV/ER 中断，
 * 中断服务函数与 HAL 回调都由本文件提供（可被下面的宏关掉，见注释）。
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

/* 主机测试必须提供下面这些替身实现（见 tests/ms5837_host_test.c 的 I2C 从设备仿真）。 */
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
HAL_StatusTypeDef HAL_I2C_Master_Transmit_IT(I2C_HandleTypeDef *hi2c,
                                             uint16_t dev_address,
                                             uint8_t *data,
                                             uint16_t size);
HAL_StatusTypeDef HAL_I2C_Master_Receive_IT(I2C_HandleTypeDef *hi2c,
                                            uint16_t dev_address,
                                            uint8_t *data,
                                            uint16_t size);
HAL_StatusTypeDef HAL_I2C_Master_Abort_IT(I2C_HandleTypeDef *hi2c, uint16_t dev_address);
uint32_t HAL_I2C_GetError(I2C_HandleTypeDef *hi2c);
#else
#include "stm32h7xx_hal.h"
#endif

/*
 * 是否由本文件提供 I2C 中断服务函数（I2Cx_EV_IRQHandler / I2Cx_ER_IRQHandler）。
 * 默认 1：集成方目前没有 CubeMX 生成的 i2c.c（也没有这两个中断函数），
 * 由本文件提供即可，避免动集成方文件。
 * 如果以后 CubeMX 使能了 I2C3 并生成了这两个中断函数，请把本宏置 0，
 * 并在生成的函数里调用 I2c_EvIrqHandler() / I2c_ErIrqHandler()，否则会重复定义。
 */
#ifndef I2C_BUS_DEFINE_IRQ_HANDLERS
#define I2C_BUS_DEFINE_IRQ_HANDLERS 1
#endif

/* 由本文件使能 I2C 中断时使用的中断优先级（H750：数值越大优先级越低）。 */
#ifndef I2C_BUS_IRQ_PRIORITY
#define I2C_BUS_IRQ_PRIORITY 5U
#endif

/* 是否让 I2c_Init() 顺手使能对应 I2C 的 EV/ER 中断。 */
#ifndef I2C_BUS_ENABLE_NVIC
#define I2C_BUS_ENABLE_NVIC 1
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

/* ---------------------------------------------------------------- 中断模式 */

/* 异步事务阶段。BUSY 期间调用方不得复用 tx/rx 缓冲区。 */
typedef enum
{
  I2C_BUS_PHASE_IDLE = 0, /* 总线空闲，可以提交新事务。 */
  I2C_BUS_PHASE_BUSY, /* 事务在飞（等待中断完成）。 */
  I2C_BUS_PHASE_DONE /* 事务已结束，结果由 I2c_GetResult() 取。 */
} I2cBusPhase_t;

/**
 * @brief 提交一次中断模式事务：可选先写，再可选读，全程不阻塞。
 *
 * tx 为 NULL 或 tx_length 为 0 表示只读；rx 为 NULL 或 rx_length 为 0 表示只写；
 * 两者都给表示“先写后读”（MS5837 的命令—读取形态，两次独立帧）。
 *
 * @return I2C_BUS_OK 已成功启动；I2C_BUS_BUSY 上一笔还没结束；
 *         I2C_BUS_NOT_READY / I2C_BUS_PARAM 调用方式错误。
 */
I2cBusResult_t I2c_Submit(const uint8_t *tx,
                          uint16_t tx_length,
                          uint8_t *rx,
                          uint16_t rx_length,
                          uint32_t timeout_ms);

/** @brief 当前异步阶段。 */
I2cBusPhase_t I2c_GetPhase(void);

/** @brief 取已完成事务的结果（PHASE_DONE 时有效；取走后回到 IDLE）。 */
I2cBusResult_t I2c_GetResult(void);

/** @brief 是否有一笔事务在飞。 */
uint8_t I2c_IsBusy(void);

/**
 * @brief 中断模式的“推进”函数：检查超时并对卡死的事务发起 Abort。
 *
 * 必须由主循环周期性调用（本模块的 Ms5837_Process() 内部已经调用，
 * 集成方无需额外调用）。函数本身不做任何阻塞。
 */
void I2c_Process(void);

/**
 * @brief 中断服务转发入口（供集成方自己的中断服务函数调用）。
 *
 * 若 I2C_BUS_DEFINE_IRQ_HANDLERS=1，本文件已提供 I2Cx_EV/ER_IRQHandler，
 * 集成方不需要调用；若集成方自己生成/持有这些函数，则在自己的函数里调用它们。
 */
void I2c_EvIrqHandler(void);
void I2c_ErIrqHandler(void);

/*
 * 本模块实现的 HAL 完成/错误回调（真实 HAL 里它们是 __weak，此处由本模块提供实现；
 * 主机测试的替身也需要调用它们，所以在这里声明）。签名与 HAL 完全一致。
 */
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c);

#ifdef __cplusplus
}
#endif

#endif /* __SENSOR_I2C_BUS_H__ */
