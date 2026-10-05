#ifndef __MS5837_H__
#define __MS5837_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * TE MS5837 防水压力/深度传感器驱动（I2C3, PA8=SCL, PC9=SDA, 7 位地址 0x76）。
 *
 * 关键约定（来自 SENSOR_COORDINATION_20261006.md）：
 *  - 02BA / 30BA 的补偿公式、二阶温补系数和最大转换时间都不同；
 *  - 实际型号未确认，上电默认 MS5837_MODEL_UNKNOWN，此时只输出原始 PROM / D1 / D2，
 *    压力、温度、深度一律为 NaN 且对应状态位清零，绝不伪造有效值；
 *  - 必须显式调用 Ms5837_SetModel 指定型号后才做补偿；
 *  - 零点必须显式采集（Ms5837_Zero）或显式设定（参数 0103），不存在“默认海平面零点”；
 *  - D1/D2 转换等待由 Ms5837_Process 的状态机承担，主循环每次调用只做几字节 I2C。
 *
 * 数据手册依据（TE 官方文档，见 docs/ms5837.md 的核对记录）：
 *  - MS5837-30BA：I2C 地址 1110110x；P=(D1*SENS/2^21-OFF)/2^13（0.1 mbar）；
 *    二阶温补 Ti=3*dT^2/2^33 / OFFi=3*(TEMP-2000)^2/2 / SENSi=5*(TEMP-2000)^2/8，
 *    低温 <-15°C 追加 OFFi+=7*(TEMP+1500)^2、SENSi+=4*(TEMP+1500)^2，
 *    高温 Ti=2*dT^2/2^37 / OFFi=(TEMP-2000)^2/2^4 / SENSi=0；
 *  - MS5837-02BA：P=(D1*SENS/2^21-OFF)/2^15（0.01 mbar，1 LSB = 1 Pa）；
 *    二阶温补（仅低温）Ti=11*dT^2/2^35 / OFFi=31*(TEMP-2000)^2/2^3 / SENSi=63*(TEMP-2000)^2/2^5；
 *  - 官方算例：30BA → 3999.8 mbar / 19.81 °C；02BA → 1100.02 mbar / 20.00 °C。
 */

/* ------------------------------------------------------------------ 型号 */
#define MS5837_MODEL_UNKNOWN 0U /* 未确认型号：只读原始数据，不补偿。 */
#define MS5837_MODEL_02BA    2U /* MS5837-02BA，量程 10~1200 mbar。 */
#define MS5837_MODEL_30BA   30U /* MS5837-30BA，量程 0~30 bar。 */

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
 * 采样周期预算（SetOsr / SetOutputRateHz / SetModel 的共同约束）：
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
#define MS5837_PARAM_SURFACE_PRESSURE 0x0103U /* f32，Pa，10000~200000。 */
#define MS5837_PARAM_FILTER_K         0x0104U /* f32，0~0.99。 */
#define MS5837_PARAM_DEPTH_MODEL      0x0105U /* u8，0/2/30。 */

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
#define MS5837_STATUS_CONFIG_UNKNOWN    (1UL << 10) /* 型号未确认。 */
#define MS5837_STATUS_MODEL_CONFIRMED   (1UL << 11) /* 型号已显式设定。 */

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
  MS5837_ERR_MODEL_UNKNOWN, /* 型号未显式设定，无法补偿。 */
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
  float pressure_pa; /* 压力，Pa；型号未确认时为 NaN。 */
  float temperature_c; /* 温度，°C；型号未确认时为 NaN。 */
  float depth_raw_m; /* 原始深度，m，正数向下；无零点时为 NaN。 */
  float depth_filtered_m; /* 滤波深度，m；无零点时为 NaN。 */
  float surface_pressure_pa; /* 本次计算使用的零点压力，Pa；无零点时为 NaN。 */
  uint8_t model; /* 生成该样本时的型号设定。 */
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
  uint8_t model; /* 当前型号设定。 */
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
 * 压力值走与 Ms5837_SetSurfacePressurePa 完全相同的校验（有限值、10000~200000 Pa）：
 * 测量压力越界时返回 MS5837_ERR_PARAM 且不建立零点，避免出现
 * “ZERO 成功但 GET_PARAMETER(0103) 认为越界”的矛盾状态。
 * 未确认型号返回 MS5837_ERR_MODEL_UNKNOWN；还没有样本返回 MS5837_ERR_NO_SAMPLE。
 */
Ms5837Result_t Ms5837_Zero(void);

/** @brief 清除零点与滤波状态，之后深度回到 NaN。 */
Ms5837Result_t Ms5837_ClearZero(void);

/** @brief 零点是否有效。 */
uint8_t Ms5837_IsZeroValid(void);

/* ------------------------------------------------------------------ 参数 */
/**
 * @brief 设定型号（0 unknown / 2 = 02BA / 30 = 30BA）。
 *
 * 型号发生变化时：清除旧零点与滤波状态（旧 P0 可能是用错误型号算出来的），
 * 作废已发布样本的补偿值，并丢弃正在进行的半周期按新型号重新采样；
 * 重复设置同一个型号是幂等的，不会清零点。
 * 当前采样率的周期预算放不下新型号时返回 MS5837_ERR_PARAM。
 */
Ms5837Result_t Ms5837_SetModel(uint8_t model); /* 0 / 2 / 30。 */
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

Ms5837Result_t Ms5837_SetSurfacePressurePa(float pa); /* 10000~200000，显式设定零点。 */
Ms5837Result_t Ms5837_GetSurfacePressurePa(float *pa); /* 未设定返回 MS5837_ERR_NO_ZERO。 */

Ms5837Result_t Ms5837_SetFilterK(float k); /* 0~0.99。 */
float Ms5837_GetFilterK(void);

/**
 * @brief 通用参数写入，供 AA5B SET_PARAMETER 路由。
 *
 * @param type 必须与参数表一致（0101 u16、0102/0103/0104 f32、0105 u8、0001 u16）。
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
 * 恢复 OUTPUT_RATE_HZ=25、DEPTH_OSR=4096、WATER_DENSITY=1029、FILTER_K=0、
 * DEPTH_MODEL=unknown，并清除零点；不改变 PROM、CRC 结论和总线连接。
 * 型号回到 unknown 后需要重新显式设定型号才会输出补偿后的压力/温度/深度。
 */
Ms5837Result_t Ms5837_RestoreDefaults(void);

/* ------------------------------------------------------------------ 纯函数工具（供集成与测试） */
/**
 * @brief 数据手册 CRC4（多项式 0x3000，字 0 高 4 位清零后参与计算）。
 */
uint8_t Ms5837_Crc4(const uint16_t prom[MS5837_PROM_WORDS]);

/**
 * @brief 数据手册给的型号/OSR 最大转换时间（毫秒）。
 *
 * 参数表按官方数据手册 ADC 表的最大值填写：
 * 30BA 8192/4096/2048/1024/512/256 → 18.08/9.04/4.54/2.28/1.17/0.60 ms；
 * 02BA 同序 → 17.20/8.61/4.32/2.17/1.10/0.56 ms。
 * 型号未确认时返回两者较大值（按 30BA 处理），保证不会提前读取 ADC。
 */
uint16_t Ms5837_MaxConversionTimeMs(uint8_t model, uint16_t osr);

/**
 * @brief 一阶 + 二阶温度补偿计算（整数，与数据手册公式逐项对应）。
 *
 * @param model  MS5837_MODEL_02BA 或 MS5837_MODEL_30BA；其它值返回 0。
 * @param pressure_raw 输出整数压力：30BA 单位 0.1 mbar，02BA 单位 0.01 mbar（=Pa）。
 * @param temperature_centi_c 输出 0.01 °C。
 * @return 1 表示已按型号补偿；0 表示未写任何输出，原因包括：
 *         型号未确认、prom 为空指针、**d1/d2 超过 24 位**、或 d1/d2 等于 0 / 0xFFFFFF
 *         （无效转换码字，绝不能据此发布测量值）。
 */
uint8_t Ms5837_Compensate(uint8_t model,
                          const uint16_t prom[MS5837_PROM_WORDS],
                          uint32_t d1,
                          uint32_t d2,
                          int64_t *pressure_raw,
                          int32_t *temperature_centi_c);

#ifdef __cplusplus
}
#endif

#endif /* __MS5837_H__ */
