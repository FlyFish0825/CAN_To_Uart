#ifndef __MS5837_H__
#define __MS5837_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32h7xx_hal.h" /* I2C 句柄类型与 HAL 回调声明所需 */

/*
 * TE MS5837 防水压力/深度传感器驱动（I2C3, PA8=SCL, PC9=SDA, 7 位地址 0x76）。
 *
 * 关键约定（来自 SENSOR_COORDINATION_20261006.md）：
 *  - 本机硬件固定为 MS5837-02BA，运行时没有型号确认、选择、查询参数或写入入口；
 *    上电即按 02BA 做补偿，压力单位 Pa；
 *  - 零点必须显式采集（Ms5837_Zero），不存在“默认海平面零点”；参数 0103 只读；
 *  - D1/D2 转换等待由 Ms5837_Process 的状态机承担，主循环每次调用只做几字节 I2C。
 *
 * 数据手册依据（TE MS5837-02BA01，见 docs/ms5837.md 的核对记录）：
 *  - MS5837-02BA：P=(D1*SENS/2^21-OFF)/2^15（0.01 mbar，1 LSB = 1 Pa）；
 *    二阶温补（仅低温）Ti=11*dT^2/2^35 / OFFi=31*(TEMP-2000)^2/2^3 / SENSi=63*(TEMP-2000)^2/2^5；
 *  - 官方算例：02BA → 1100.02 mbar / 20.00 °C。
 */

/* ------------------------------------------------------------------ 固定硬件型号 */
#define MS5837_MODEL_02BA    2U /* MS5837-02BA，量程 10~1200 mbar。 */
#define MS5837_MODEL_FIXED   MS5837_MODEL_02BA

/* ------------------------------------------------------------------ OSR */
#define MS5837_OSR_256  256U
#define MS5837_OSR_512  512U
#define MS5837_OSR_1024 1024U
#define MS5837_OSR_2048 2048U
#define MS5837_OSR_4096 4096U
#define MS5837_OSR_8192 8192U
#define MS5837_OSR_DEFAULT MS5837_OSR_4096

/* 采样与参数默认值。 */
#define MS5837_OUTPUT_RATE_HZ_DEFAULT 25U
#define MS5837_OUTPUT_RATE_HZ_MIN     1U
#define MS5837_OUTPUT_RATE_HZ_MAX     100U
#define MS5837_WATER_DENSITY_DEFAULT  1029.0f /* kg/m3，海水常用值，可由参数 0102 覆盖。 */
#define MS5837_WATER_DENSITY_MIN      900.0f
#define MS5837_WATER_DENSITY_MAX      1300.0f
#define MS5837_SURFACE_PRESSURE_MIN   10000.0f
#define MS5837_SURFACE_PRESSURE_MAX   200000.0f
#define MS5837_FILTER_K_DEFAULT       0.0f /* 默认不做滤波，滤波深度等于原始深度。 */
#define MS5837_FILTER_K_MAX           0.99f
#define MS5837_GRAVITY                9.80665f /* m/s^2。 */

/*
 * 采样周期预算（SetOsr / SetOutputRateHz 的共同约束；运行时型号固定为 02BA）：
 *   needed_ms = 2 × (最大转换时间 + MS5837_CONVERSION_MARGIN_MS) + MS5837_SCHEDULE_TX_BUDGET_MS
 *   必须 <= 1000 / output_rate_hz，否则返回 MS5837_ERR_PARAM。
 * 每次转换的等待都从“I2C 命令真正发完”之后重新取 HAL_GetTick() 再计算，
 * 并额外加 1 ms 余量，避免毫秒相位导致提前读取未完成的转换。
 * 参考：30BA OSR4096 → 2×(10+1)+2 = 24 ms，所以 25/40 Hz 可以，50 Hz 不行；
 *       30BA OSR8192 → 2×(19+1)+2 = 42 ms，所以 20 Hz 可以，25 Hz 不行。
 */
#define MS5837_CONVERSION_MARGIN_MS  1U /* 命令完成后额外等待的毫秒数。 */
#define MS5837_SCHEDULE_TX_BUDGET_MS 2U /* 一帧内 4 次短 I2C 事务的预算。 */

/* PROM 读取与复位时序。 */
#define MS5837_PROM_WORDS        8U /* 8 个 16 位字，字 0 含 4 位 CRC。 */
#define MS5837_RESET_DELAY_MS    10U /* 复位后等待期间不占用主循环，仅为保守余量。 */
#define MS5837_RETRY_DELAY_MS    100U /* 总线错误后的重试间隔。 */

/* ------------------------------------------------------------------ 参数表 */
/* 参数 ID 与 AA5B v1 参数表一致，便于路由层直接转发。 */
#define MS5837_PARAM_OUTPUT_RATE_HZ   0x0001U /* u16，1~100 Hz。 */
#define MS5837_PARAM_DEPTH_OSR        0x0101U /* u16，256/512/1024/2048/4096/8192。 */
#define MS5837_PARAM_WATER_DENSITY    0x0102U /* f32，kg/m3，900~1300。 */
#define MS5837_PARAM_SURFACE_PRESSURE 0x0103U /* f32，Pa，只读；仅 ZERO_DEPTH 能更新。 */
#define MS5837_PARAM_FILTER_K         0x0104U /* f32，0~0.99。 */

/* 参数值类型码，与 AA5B v1 一致。 */
#define MS5837_PARAM_TYPE_U8  2U
#define MS5837_PARAM_TYPE_U16 4U
#define MS5837_PARAM_TYPE_F32 7U

/* ------------------------------------------------------------------ 状态位 */
/* 位编号与 AA5B v1 状态字一致；本模块只会置位/清零其中一部分。 */
#define MS5837_STATUS_ONLINE            (1UL << 0)  /* 最近一次总线访问成功。 */
#define MS5837_STATUS_RAW_VALID         (1UL << 1)  /* 有可信的 PROM + D1/D2。 */
#define MS5837_STATUS_PRESSURE_VALID    (1UL << 4)  /* pressure_pa 有效（需已确认型号）。 */
#define MS5837_STATUS_TEMPERATURE_VALID (1UL << 5)  /* temperature_c 有效（需已确认型号）。 */
#define MS5837_STATUS_DEPTH_VALID       (1UL << 6)  /* 深度有效（压力有效且零点有效）。 */
#define MS5837_STATUS_ZERO_VALID        (1UL << 7)  /* 零点已显式采集或设定。 */
#define MS5837_STATUS_PROM_VALID        (1UL << 8)  /* PROM CRC4 校验通过。 */
#define MS5837_STATUS_PIN_BLOCKED       (1UL << 9)  /* 本模块不使用该位（IMU 专用）。 */

/* 缺失值统一用 NaN 表示，绝不使用 0 冒充有效测量。 */
#define MS5837_IS_NAN(value) ((value) != (value))

/* ------------------------------------------------------------------ 结果码 */
typedef enum
{
  MS5837_OK = 0, /* 成功。 */
  MS5837_ERR_PARAM, /* 参数非法（越界、非支持值、或与转换时间冲突）。 */
  MS5837_ERR_NOT_READY, /* 尚未初始化 / PROM 未就绪 / 缺缓存。 */
  MS5837_ERR_BUSY, /* 总线忙。 */
  MS5837_ERR_TIMEOUT, /* I2C 事务超时。 */
  MS5837_ERR_IO, /* NACK 等总线错误，通常等价于设备离线。 */
  MS5837_ERR_CRC, /* PROM CRC4 校验失败。 */
  MS5837_ERR_NO_ZERO, /* 零点未采集/未设定。 */
  MS5837_ERR_NO_SAMPLE, /* 还没有完成的采样。 */
  MS5837_ERR_UNSUPPORTED /* 未知参数 ID。 */
} Ms5837Result_t;

/* ------------------------------------------------------------------ 数据结构 */
typedef struct
{
  uint32_t status; /* 生成该样本时的状态字快照。 */
  uint32_t sequence; /* 采样序号，从 1 开始递增。 */
  uint32_t timestamp_ms; /* 样本完成时刻（HAL_GetTick 毫秒）。 */
  uint32_t d1; /* 原始压力 ADC 值（24 位）。 */
  uint32_t d2; /* 原始温度 ADC 值（24 位）。 */
  int64_t pressure_raw; /* 数据手册整数输出：30BA 为 0.1 mbar，02BA 为 0.01 mbar(=Pa)。 */
  int32_t temperature_centi_c; /* 二阶补偿后温度，0.01 °C。 */
  float pressure_pa; /* 压力，Pa；无效补偿时为 NaN。 */
  float temperature_c; /* 温度，°C；无效补偿时为 NaN。 */
  float depth_raw_m; /* 原始深度，m，正数向下；无零点时为 NaN。 */
  float depth_filtered_m; /* 滤波深度，m；无零点时为 NaN。 */
  float surface_pressure_pa; /* 本次计算使用的零点压力，Pa；无零点时为 NaN。 */
  uint16_t prom[MS5837_PROM_WORDS]; /* PROM 原始 8 个字（含字 0 的高 4 位 CRC）。 */
} Ms5837Sample_t;

typedef struct
{
  uint32_t sample_seq; /* 已完成采样数。 */
  uint32_t good_frames; /* 成功完成 D1+D2 的帧数。 */
  uint32_t errors; /* 累计错误次数（含 I2C 与 CRC）。 */
  uint32_t crc_errors; /* PROM CRC 失败次数。 */
  uint32_t bus_timeouts; /* I2C 超时次数。 */
  uint32_t last_error; /* 最近一次错误码（Ms5837Result_t）。 */
  uint8_t prom_valid; /* PROM CRC 是否通过。 */
  uint16_t osr; /* 当前 OSR。 */
  uint16_t output_rate_hz; /* 当前采样率。 */
} Ms5837Stats_t;

/* ------------------------------------------------------------------ 生命周期 */
/**
 * @brief 启动非阻塞初始化：软复位 → 等待 → 逐字读取 PROM → CRC4 校验。
 *
 * 需要在 I2c_Init() 之后调用。本函数只发起复位命令，不阻塞等待复位延时，
 * 后续由 Ms5837_Process() 推进；可在运行期重复调用以重新初始化。
 */
Ms5837Result_t Ms5837_Init(void);

/**
 * @brief 主循环服务函数：推进初始化状态机与 D1/D2 采样状态机。
 *
 * 每次调用最多做少量几字节 I2C 事务，转换等待靠时间戳判断，
 * 不会阻塞几十毫秒。应在 while(1) 中持续调用，不要在中断里调用。
 */
void Ms5837_Process(void);

/* ------------------------------------------------------------------ 数据读取 */
/**
 * @brief 取最近一次完成的样本。
 *
 * @return MS5837_OK；MS5837_ERR_NO_SAMPLE 表示还没有任何完成的采样。
 *         status 字段为调用时刻的实时状态字。
 */
Ms5837Result_t Ms5837_GetSample(Ms5837Sample_t *sample);

/** @brief 当前实时状态字（见 MS5837_STATUS_*）。 */
uint32_t Ms5837_GetStatus(void);

/** @brief 最近样本的年龄（毫秒）；从未采样时返回 UINT32_MAX。 */
uint32_t Ms5837_GetSampleAgeMs(void);

/** @brief 自上次清除后是否产生了新样本。 */
uint8_t Ms5837_HasNewSample(void);

/** @brief 清除新样本标志（不丢弃样本本身）。 */
void Ms5837_ClearNewSampleFlag(void);

/** @brief 读取运行统计。 */
Ms5837Result_t Ms5837_GetStats(Ms5837Stats_t *stats);

/** @brief 读取 PROM 原始 8 个字（含字 0 的高 4 位 CRC）。 */
Ms5837Result_t Ms5837_GetProm(uint16_t prom[MS5837_PROM_WORDS]);

/* ------------------------------------------------------------------ 零点 */
/**
 * @brief 用最近一次有效压力显式建立零点（对应 AA5B ZERO_DEPTH）。
 *
 * 压力值走内部 P0 写入辅助函数的校验（有限值、10000~200000 Pa）：
 * 测量压力越界时返回 MS5837_ERR_PARAM 且不建立零点，避免出现
 * “ZERO 成功但 GET_PARAMETER(0103) 认为越界”的矛盾状态。
 * 还没有样本返回 MS5837_ERR_NO_SAMPLE；有效压力尚未产生时返回 MS5837_ERR_NOT_READY。
 */
Ms5837Result_t Ms5837_Zero(void);

/** @brief 清除零点与滤波状态，之后深度回到 NaN。 */
Ms5837Result_t Ms5837_ClearZero(void);

/** @brief 零点是否有效。 */
uint8_t Ms5837_IsZeroValid(void);

/* ------------------------------------------------------------------ 参数 */
/** @brief 返回本机固定探头型号 MS5837-02BA。 */
uint8_t Ms5837_GetModel(void);

/**
 * @brief 设定 OSR（256~8192）。
 *
 * 若正好处于 D1/D2 转换中，会丢弃该半周期并按新 OSR 重新开始，绝不沿用旧等待。
 * 周期预算放不下时返回 MS5837_ERR_PARAM。
 */
Ms5837Result_t Ms5837_SetOsr(uint16_t osr); /* 256~8192，且需满足当前采样率的周期预算。 */
uint16_t Ms5837_GetOsr(void);

Ms5837Result_t Ms5837_SetOutputRateHz(uint16_t rate_hz); /* 1~100，且需容纳一次保守采样周期。 */
uint16_t Ms5837_GetOutputRateHz(void);

Ms5837Result_t Ms5837_SetWaterDensity(float kg_m3); /* 900~1300。 */
float Ms5837_GetWaterDensity(void);

Ms5837Result_t Ms5837_GetSurfacePressurePa(float *pa); /* 未设定返回 MS5837_ERR_NO_ZERO。 */

Ms5837Result_t Ms5837_SetFilterK(float k); /* 0~0.99。 */
float Ms5837_GetFilterK(void);

/**
 * @brief 通用参数写入，供 AA5B SET_PARAMETER 路由。
 *
 * @param type 必须与参数表一致（0101 u16、0102/0103/0104 f32、0001 u16）。
 * @param value 指向 LE 编码的值，长度与 type 一致。
 */
Ms5837Result_t Ms5837_SetParam(uint16_t param_id,
                               uint8_t type,
                               const void *value,
                               uint8_t length);

/**
 * @brief 通用参数读取，供 AA5B GET_PARAMETER 路由。
 *
 * 参数值都保存在 MCU 侧（无设备回读），因此属于“已确认的本地配置”。
 * 零点未设定时返回 MS5837_ERR_NO_ZERO，不会用默认值冒充。
 */
Ms5837Result_t Ms5837_GetParam(uint16_t param_id,
                               uint8_t *type,
                               uint8_t *length,
                               uint8_t value[4]);

/**
 * @brief 恢复参数默认值（对应 AA5B RESTORE_DEFAULTS）。
 *
 * 恢复 OUTPUT_RATE_HZ=25、DEPTH_OSR=4096、WATER_DENSITY=1029、FILTER_K=0；
 * 保留本机固定 02BA 型号和现有水面 P0，不改变 PROM、CRC 结论和总线连接。
 * 采样时序改变时丢弃正在进行的半周期。
 */
Ms5837Result_t Ms5837_RestoreDefaults(void);

/* ------------------------------------------------------------------ 纯函数工具（供集成与测试） */
/**
 * @brief 数据手册 CRC4（多项式 0x3000，字 0 高 4 位清零后参与计算）。
 */
uint8_t Ms5837_Crc4(const uint16_t prom[MS5837_PROM_WORDS]);

/**
 * @brief 固定 MS5837-02BA 的最大转换时间（毫秒）。
 *
 * 参数表按官方数据手册 ADC 表的最大值填写：
 * 02BA 8192/4096/2048/1024/512/256 → 17.20/8.61/4.32/2.17/1.10/0.56 ms。
 * 无效 OSR 返回 0。
 */
uint16_t Ms5837_MaxConversionTimeMs(uint16_t osr);

/**
 * @brief 一阶 + 二阶温度补偿计算（整数，与数据手册公式逐项对应）。
 *
 * @param pressure_raw 输出整数压力：02BA 单位 0.01 mbar（=Pa）。
 * @param temperature_centi_c 输出 0.01 °C。
 * @return 1 表示按固定 02BA 公式补偿；0 表示未写任何输出，原因包括：
 *         prom 为空指针、**d1/d2 超过 24 位**、或 d1/d2 等于 0 / 0xFFFFFF
 *         （无效转换码字，绝不能据此发布测量值）。
 */
uint8_t Ms5837_Compensate(const uint16_t prom[MS5837_PROM_WORDS],
                          uint32_t d1,
                          uint32_t d2,
                          int64_t *pressure_raw,
                          int32_t *temperature_centi_c);


/* ---------------------------------------------------------------- I2C 事务层接口
 *
 * 原 sensor_i2c_bus.h 已并入本文件。集成方只需要：
 *   1) 自己完成 I2C3 的 CubeMX/HAL 初始化（本模块不定义 MX_I2C3_Init）；
 *   2) 调用一次 I2c_Init(&hi2c3) 把句柄交给本模块（内部按实例使能 EV/ER 中断）；
 *   3) 主循环调用 Ms5837_Process()（内部已包含事务超时推进，无需另外调用轮询函数）。
 *
 * I2C3_EV_IRQHandler / I2C3_ER_IRQHandler 由本文件提供；若集成方用 CubeMX 生成 i2c.c
 * 并自带这两个函数，请把 I2C_BUS_DEFINE_IRQ_HANDLERS 置 0，并在生成的函数里调用
 * I2c_EvIrqHandler() / I2c_ErIrqHandler()，否则会重复定义。
 */
#define I2C_BUS_DEFAULT_TIMEOUT_MS 5U /* 未显式指定时单次事务的默认超时。 */
#define I2C_BUS_MAX_TIMEOUT_MS     50U /* 单次事务允许的最大超时上限。 */
#define I2C_BUS_MAX_TRANSFER       16U /* 一次事务允许的最大字节数。 */
#define I2C_BUS_MS5837_ADDRESS7    0x76U /* MS5837 的 7 位地址（HAL 写地址 0xEC）。 */

#ifndef I2C_BUS_DEFINE_IRQ_HANDLERS
#define I2C_BUS_DEFINE_IRQ_HANDLERS 1 /* 是否由本文件提供 I2Cx_EV/ER_IRQHandler。 */
#endif
#ifndef I2C_BUS_IRQ_PRIORITY
#define I2C_BUS_IRQ_PRIORITY 5U /* 低于 FDCAN(2)/DMA(3)/USART1(4)，可按系统策略调整。 */
#endif
#ifndef I2C_BUS_ENABLE_NVIC
#define I2C_BUS_ENABLE_NVIC 1 /* 是否由 I2c_Init() 使能对应 I2C 的 EV/ER 中断。 */
#endif

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

/** @brief 绑定 HAL I2C 句柄（传 NULL 卸载），并按实例使能 EV/ER 中断。 */
void I2c_Init(I2C_HandleTypeDef *handle);

/** @brief 是否已绑定句柄。 */
uint8_t I2c_IsReady(void);

/** @brief 设置当前 7 位从地址，合法范围 0x08~0x77。 */
I2cBusResult_t I2c_SetDevice(uint8_t address7);

/** @brief 读取当前 7 位从地址。 */
uint8_t I2c_GetDevice(void);

/** @brief 中断服务转发入口（供集成方自己的中断服务函数调用）。 */
void I2c_EvIrqHandler(void);
void I2c_ErIrqHandler(void);

/* 本模块实现的 HAL 完成/错误回调（HAL 里是 __weak，这里提供实现）。 */
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c);

#ifdef __cplusplus
}
#endif

#endif /* __MS5837_H__ */
