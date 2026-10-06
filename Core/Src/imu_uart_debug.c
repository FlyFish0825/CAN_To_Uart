/**
 * @file imu_uart_debug.c
 * @brief 【imu-uart1-debug 测试分支】IMU 帧解析 + 精确时间戳调试实现。
 *
 * 数据路径：
 *   USART1 RX（DMA1_Stream0，循环模式）-> imu_dbg_dma_ring
 *     -> 主循环按 NDTR 游标取出字节（每字节带到达时间戳）
 *     -> 7E 23 帧解析 -> 每帧一行 "IMU F: <func> <stamp_us>" 上 CDC
 *
 * 时间体系（双层时钟）：
 *   - 细时钟：TIM5 1 MHz 自由计数，主循环轮询扩展成 64 位微秒，不回绕；
 *   - 粗时钟：RTC 日历（LSE 32.768kHz + VBAT 纽扣电池），断电持续走时；
 *   - 同步：上位机经 CDC 发送 "!SYNC <epoch_ms>\n"（经字节旁路钩子
 *     UsbCanGateway_ByteTap 进入本模块），模块写入 RTC 日历并锚定细
 *     时钟；此后每帧时间戳 = 墙钟微秒，亚秒精度由细时钟提供；
 *   - 断电重启：RTC 由纽扣电池维持，开机自动恢复墙钟锚点（误差为
 *     1/256 s 亚秒量化 + 同步时刻毫秒余数记录 + 晶振漂移），无需重新
 *     同步；纽扣电池没电或首次上电时保持未同步状态等待主机命令。
 *
 * 帧时间戳取“该帧最后一个字节的到达时刻”；同一批次内各字节按 115200
 * 波特率的字节间隔向前回推。模块自身协议时延（采样到发送约 1~2 ms）
 * 由上位机按需修正。
 */
#include "imu_uart_debug.h"

#include "dma.h"
#include "usart.h"
#include "usb_can_gateway.h"

#include <string.h>

/* USART1 DMA 环形缓冲字节数，必须为 2 的幂；115200 波特率下约缓冲 178 ms。 */
#define IMU_DBG_DMA_RING_SIZE 2048U
/* 无新数据时打印统计状态行的最小间隔。 */
#define IMU_DBG_IDLE_PERIOD_MS 1000U
/* PA9-PA10 短接自环自测的发送周期；接真实 IMU 时必须保持关闭。 */
#define IMU_DBG_SELFTEST_PERIOD_MS 500U
#define IMU_DBG_SELFTEST_ENABLE 0U
/* 115200 8N1 单字节时间 86.8us，取 87us 用于同批次字节回推。 */
#define IMU_DBG_BAUD_BYTE_TIME_US 87U
/* IMU 帧 7E 23 [LEN] ...，LEN 为整帧总长，本协议最大 23 字节，上限放宽。 */
#define IMU_DBG_FRAME_MAX 64U
/* RTC 备份寄存器魔数与毫秒余数槽位：标识“日历已按主机时间设置过”。 */
#define IMU_DBG_RTC_MAGIC 0x31494D55U /* "IMU1" */
/* 主机同步命令 "!SYNC <epoch_ms>" 的缓冲长度与合法性下界（2020-09）。 */
#define IMU_DBG_SYNC_CMD_MAX 24U
#define IMU_DBG_SYNC_MIN_MS 1600000000000ULL

/* USART1 RX DMA 句柄由 CubeMX 生成在 usart.c 中，非 static，可外部引用。 */
extern DMA_HandleTypeDef hdma_usart1_rx;

/*
 * DMA1 无法访问 DTCM（链接脚本明确注明），因此环形缓冲必须放在
 * .dma_buffer 段（RAM_D2 0x30000000）。
 */
__attribute__((section(".dma_buffer"), aligned(32)))
static uint8_t imu_dbg_dma_ring[IMU_DBG_DMA_RING_SIZE];

static uint16_t imu_dbg_rd_pos;           /* DMA 环形缓冲读取游标。 */
static uint32_t imu_dbg_total_bytes;      /* 累计收到的原始字节数。 */
static uint32_t imu_dbg_bad_frames;       /* 校验失败帧计数。 */
static uint32_t imu_dbg_last_data_tick;   /* 最后一次收到字节的时间。 */
static uint32_t imu_dbg_last_status_tick; /* 最后一次打印状态行的时间。 */
static uint32_t imu_dbg_last_clk_tick;    /* 最后一次打印时钟行的时间。 */
static uint32_t imu_dbg_last_selftest_tick; /* 最后一次自环自测发送的时间。 */
static uint32_t imu_dbg_last_err_tick;    /* 最后一次打印错误行的时间。 */
static uint8_t  imu_dbg_started;          /* 初始化完成标志。 */

static const char imu_dbg_hex[] = "0123456789ABCDEF";

/* ---- 细时钟：TIM5 1 MHz + 主循环溢出扩展成 64 位 ---- */
static uint32_t imu_dbg_tim_last; /* 上一次轮询的 TIM5 计数值。 */
static uint64_t imu_dbg_time_us;  /* 扩展后的 64 位微秒计数。 */

/* ---- 粗时钟：RTC（VBAT 纽扣电池 + LSE）---- */
static RTC_HandleTypeDef imu_dbg_hrtc;
static uint8_t  imu_dbg_rtc_ok;  /* LSE/RTC 初始化成功。 */
static uint8_t  imu_dbg_synced;  /* 墙钟锚点有效。 */
static uint64_t imu_dbg_epoch_us_anchor; /* 锚点墙钟微秒。 */
static uint64_t imu_dbg_counter_anchor;  /* 锚点时刻的细时钟微秒。 */

/* ---- 主机同步命令 "!SYNC <epoch_ms>" 的行缓冲 ---- */
static char    imu_dbg_cmd[IMU_DBG_SYNC_CMD_MAX];
static uint8_t imu_dbg_cmd_len;

/* ---- IMU 帧解析状态机 ---- */
enum
{
  IMU_DBG_ST_HEAD1 = 0, /* 等待包头 0x7E。 */
  IMU_DBG_ST_HEAD2,     /* 已收到 0x7E，等待 0x23。 */
  IMU_DBG_ST_LEN,       /* 等待长度字节。 */
  IMU_DBG_ST_BODY       /* 收集帧体直到 LEN 个字节。 */
};
static uint8_t  imu_dbg_pstate;
static uint8_t  imu_dbg_frame[IMU_DBG_FRAME_MAX];
static uint16_t imu_dbg_frame_pos;
static uint16_t imu_dbg_frame_len;

/** 把无符号 64 位数转成十进制文本，返回写入的字符数。 */
static uint16_t ImuUartDebug_U64ToDec(uint64_t value, char *out)
{
  char tmp[20];
  uint16_t count = 0U;
  uint16_t i;

  do
  {
    tmp[count++] = (char)('0' + (int)(value % 10U));
    value /= 10U;
  } while (value != 0U);
  for (i = 0U; i < count; i++)
  {
    out[i] = tmp[count - 1U - i];
  }
  return count;
}

/** 天数转年月日（Howard Hinnant civil_from_days 算法，1970-01-01 为 0）。 */
static void ImuUartDebug_CivilFromDays(int64_t z, int32_t *year,
                                       uint32_t *month, uint32_t *day)
{
  int64_t era;
  uint32_t doe;
  uint32_t yoe;
  uint32_t doy;
  uint32_t mp;
  int64_t y;
  uint32_t m;

  z += 719468;
  era = ((z >= 0) ? z : (z - 146096)) / 146097;
  doe = (uint32_t)(z - era * 146097);
  yoe = (doe - doe / 1460U + doe / 36524U - doe / 146096U) / 365U;
  y = (int64_t)yoe + era * 400;
  doy = doe - (365U * yoe + yoe / 4U - yoe / 100U);
  mp = (5U * doy + 2U) / 153U;
  *day = doy - (153U * mp + 2U) / 5U + 1U;
  m = (mp < 10U) ? (mp + 3U) : (mp - 9U);
  *month = m;
  *year = (int32_t)(y + ((m <= 2U) ? 1 : 0));
}

/** 年月日转天数（Howard Hinnant days_from_civil 算法）。 */
static int64_t ImuUartDebug_DaysFromCivil(int64_t year, uint32_t month,
                                          uint32_t day)
{
  int64_t y = year - ((month <= 2U) ? 1 : 0);
  int64_t era = ((y >= 0) ? y : (y - 399)) / 400;
  uint32_t yoe = (uint32_t)(y - era * 400);
  uint32_t mp = month + ((month > 2U) ? (-3) : 9);
  uint32_t doy = (153U * mp + 2U) / 5U + day - 1U;
  uint32_t doe = yoe * 365U + yoe / 4U - yoe / 100U + doy;

  return era * 146097 + (int64_t)doe - 719468;
}

/**
 * @brief 初始化 RTC（LSE + VBAT）。
 *
 * 若 BDCR 中 RTCEN 已置位（纽扣电池维持过备份域），则跳过重初始化，
 * 避免进入 INIT 模式清零亚秒寄存器；否则完整执行 LSE 起振、时钟源
 * 选择和日历初始化。失败时 rtc_ok=0，模块退化为纯细时钟模式。
 */
static void ImuUartDebug_RtcInit(void)
{
  RTC_HandleTypeDef *h = &imu_dbg_hrtc;
  uint8_t running;
  uint32_t wait_start;

  HAL_PWR_EnableBkUpAccess();

  running = (READ_BIT(RCC->BDCR, RCC_BDCR_RTCEN) != 0U) ? 1U : 0U;

  {
    RCC_OscInitTypeDef osc = {0};

    osc.OscillatorType = RCC_OSCILLATORTYPE_LSE;
    osc.LSEState = RCC_LSE_ON;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK)
    {
      imu_dbg_rtc_ok = 0U;
      return;
    }
  }

  if (running == 0U)
  {
    RCC_PeriphCLKInitTypeDef pc = {0};

    pc.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    pc.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    if (HAL_RCCEx_PeriphCLKConfig(&pc) != HAL_OK)
    {
      imu_dbg_rtc_ok = 0U;
      return;
    }
    __HAL_RCC_RTC_ENABLE();

    h->Instance = RTC;
    h->Init.HourFormat = RTC_HOURFORMAT_24;
    h->Init.AsynchPrediv = 127U; /* 32768 / 128 = 256 Hz。 */
    h->Init.SynchPrediv = 255U;  /* 256 / 256 = 1 Hz。 */
    h->Init.OutPut = RTC_OUTPUT_DISABLE;
    h->Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
    h->Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    h->Init.OutPutType = RTC_OUTPUT_TYPE_PUSHPULL;
    if (HAL_RTC_Init(h) != HAL_OK)
    {
      imu_dbg_rtc_ok = 0U;
      return;
    }

    /* 影子寄存器需等 RSF 置位后才有效，超时 100 ms 放弃。 */
    wait_start = HAL_GetTick();
    while (READ_BIT(RTC->ISR, RTC_ISR_RSF) == 0U)
    {
      if ((HAL_GetTick() - wait_start) > 100U)
      {
        break;
      }
    }
  }
  imu_dbg_rtc_ok = 1U;
}

/** 开机时从 RTC 日历恢复墙钟锚点（需备份魔数已写入）。 */
static void ImuUartDebug_RtcAnchorFromBoot(void)
{
  RTC_TimeTypeDef time = {0};
  RTC_DateTypeDef date = {0};
  int64_t days;
  uint64_t epoch_s;
  uint64_t frac_us;
  uint64_t frac_ms;

  HAL_RTC_GetTime(&imu_dbg_hrtc, &time, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&imu_dbg_hrtc, &date, RTC_FORMAT_BIN);

  days = ImuUartDebug_DaysFromCivil(2000 + (int64_t)date.Year,
                                    date.Month, date.Date);
  epoch_s = (uint64_t)days * 86400ULL +
            (uint64_t)time.Hours * 3600ULL +
            (uint64_t)time.Minutes * 60ULL +
            (uint64_t)time.Seconds;
  /* 亚秒寄存器为 1/256 s 递减计数。 */
  frac_us = ((uint64_t)(time.SecondFraction - time.SubSeconds) * 1000000ULL) /
            (uint64_t)time.SecondFraction;
  /* 同步时刻的毫秒余数补偿：设置日历的那一秒从“余数毫秒”开始。 */
  frac_ms = (uint64_t)HAL_RTCEx_BKUPRead(&imu_dbg_hrtc, RTC_BKP_DR1);
  if (frac_ms > 999ULL)
  {
    frac_ms = 0ULL;
  }

  imu_dbg_epoch_us_anchor = epoch_s * 1000000ULL + frac_us + frac_ms * 1000ULL;
  imu_dbg_counter_anchor = imu_dbg_time_us;
  imu_dbg_synced = 1U;
}

/** 把主机下发的 epoch 毫秒写入 RTC 日历，并备份毫秒余数供开机恢复。 */
static void ImuUartDebug_RtcSetFromEpoch(uint64_t epoch_ms)
{
  RTC_TimeTypeDef time = {0};
  RTC_DateTypeDef date = {0};
  uint64_t epoch_s = epoch_ms / 1000ULL;
  uint32_t rem = (uint32_t)(epoch_s % 86400ULL);
  int32_t year;
  uint32_t month;
  uint32_t day;

  ImuUartDebug_CivilFromDays((int64_t)(epoch_s / 86400ULL),
                             &year, &month, &day);

  time.Hours = (uint8_t)(rem / 3600U);
  time.Minutes = (uint8_t)((rem / 60U) % 60U);
  time.Seconds = (uint8_t)(rem % 60U);
  time.TimeFormat = RTC_HOURFORMAT12_PM; /* 24 小时制下忽略。 */
  time.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  time.StoreOperation = RTC_STOREOPERATION_RESET;
  HAL_RTC_SetTime(&imu_dbg_hrtc, &time, RTC_FORMAT_BIN);

  date.Year = (uint8_t)(year - 2000);
  date.Month = (uint8_t)month;
  date.Date = (uint8_t)day;
  date.WeekDay = RTC_WEEKDAY_MONDAY; /* 协议不使用星期，占位即可。 */
  HAL_RTC_SetDate(&imu_dbg_hrtc, &date, RTC_FORMAT_BIN);

  HAL_RTCEx_BKUPWrite(&imu_dbg_hrtc, RTC_BKP_DR0, IMU_DBG_RTC_MAGIC);
  HAL_RTCEx_BKUPWrite(&imu_dbg_hrtc, RTC_BKP_DR1,
                      (uint32_t)(epoch_ms % 1000ULL));
}

/** 启动 TIM5：APB1 定时器时钟 240 MHz / 240 = 1 MHz，32 位自由计数。 */
static void ImuUartDebug_TimInit(void)
{
  __HAL_RCC_TIM5_CLK_ENABLE();
  TIM5->CR1 = 0U;
  TIM5->PSC = 239U;
  TIM5->ARR = 0xFFFFFFFFU;
  TIM5->EGR = TIM_EGR_UG; /* 装载预分频并清零计数。 */
  TIM5->CR1 = TIM_CR1_CEN;
  imu_dbg_tim_last = 0U;
  imu_dbg_time_us = 0U;
}

/** 轮询 TIM5 计数并扩展成 64 位微秒；主循环周期远小于 71.6 分钟回绕。 */
static void ImuUartDebug_TimeUpdate(void)
{
  uint32_t now = TIM5->CNT;

  if (now >= imu_dbg_tim_last)
  {
    imu_dbg_time_us += (uint64_t)(now - imu_dbg_tim_last);
  }
  else
  {
    imu_dbg_time_us += (uint64_t)(0xFFFFFFFFU - imu_dbg_tim_last) + 1ULL +
                       (uint64_t)now;
  }
  imu_dbg_tim_last = now;
}

/** 发送一行文本到 CDC 队列（失败即丢弃，不阻塞主循环）。 */
static void ImuUartDebug_SendLine(const char *text, uint16_t len)
{
  (void)UsbCanGateway_TxEnqueue((const uint8_t *)text, len);
}

/** 每秒打印一次墙钟与同步状态，便于和上位机时钟对表。 */
static void ImuUartDebug_ReportClock(uint32_t now)
{
  char line[USB_CAN_PACKET_SIZE];
  uint16_t pos = 0U;
  uint64_t stamp;

  if ((now - imu_dbg_last_clk_tick) < IMU_DBG_IDLE_PERIOD_MS)
  {
    return;
  }
  imu_dbg_last_clk_tick = now;

  memcpy(&line[pos], "IMU CLK: sync=", 14U);
  pos = (uint16_t)(pos + 14U);
  line[pos++] = (char)('0' + imu_dbg_synced);
  memcpy(&line[pos], " rtc=", 5U);
  pos = (uint16_t)(pos + 5U);
  line[pos++] = (char)('0' + imu_dbg_rtc_ok);
  line[pos++] = ' ';
  if (imu_dbg_synced != 0U)
  {
    stamp = imu_dbg_epoch_us_anchor + (imu_dbg_time_us - imu_dbg_counter_anchor);
    pos += ImuUartDebug_U64ToDec(stamp / 1000ULL, &line[pos]);
  }
  else
  {
    line[pos++] = '-';
  }
  line[pos++] = '\r';
  line[pos++] = '\n';
  ImuUartDebug_SendLine(line, pos);
}

/** 长时间无数据时打印一次统计行，用于确认固件与串口接收是否存活。 */
static void ImuUartDebug_ReportIdle(uint32_t now)
{
  char line[USB_CAN_PACKET_SIZE];
  uint16_t pos = 0U;

  if (((now - imu_dbg_last_data_tick) < IMU_DBG_IDLE_PERIOD_MS) ||
      ((now - imu_dbg_last_status_tick) < IMU_DBG_IDLE_PERIOD_MS))
  {
    return;
  }
  imu_dbg_last_status_tick = now;

  memcpy(&line[pos], "IMU: idle total=", 16U);
  pos = (uint16_t)(pos + 16U);
  pos += ImuUartDebug_U64ToDec(imu_dbg_total_bytes, &line[pos]);
  line[pos++] = '\r';
  line[pos++] = '\n';
  ImuUartDebug_SendLine(line, pos);
}

/** 输出一帧：功能字 + 该帧末字节到达时刻的墙钟微秒。 */
static void ImuUartDebug_EmitFrame(uint64_t t_last_us)
{
  char line[USB_CAN_PACKET_SIZE];
  uint16_t pos = 0U;
  uint64_t stamp;

  if ((imu_dbg_synced != 0U) && (t_last_us >= imu_dbg_counter_anchor))
  {
    stamp = imu_dbg_epoch_us_anchor + (t_last_us - imu_dbg_counter_anchor);
  }
  else
  {
    stamp = t_last_us; /* 未同步：输出细时钟原始微秒。 */
  }

  memcpy(&line[pos], "IMU F: ", 7U);
  pos = (uint16_t)(pos + 7U);
  line[pos++] = imu_dbg_hex[(imu_dbg_frame[3] >> 4) & 0x0FU];
  line[pos++] = imu_dbg_hex[imu_dbg_frame[3] & 0x0FU];
  line[pos++] = ' ';
  pos += ImuUartDebug_U64ToDec(stamp, &line[pos]);
  line[pos++] = '\r';
  line[pos++] = '\n';
  ImuUartDebug_SendLine(line, pos);
}

/** 校验失败计数并限频打印错误行。 */
static void ImuUartDebug_ReportBadFrame(void)
{
  char line[USB_CAN_PACKET_SIZE];
  uint16_t pos = 0U;
  uint32_t now = HAL_GetTick();

  imu_dbg_bad_frames++;
  if ((now - imu_dbg_last_err_tick) < IMU_DBG_IDLE_PERIOD_MS)
  {
    return;
  }
  imu_dbg_last_err_tick = now;

  memcpy(&line[pos], "IMU E: bad frame ", 17U);
  pos = (uint16_t)(pos + 17U);
  pos += ImuUartDebug_U64ToDec(imu_dbg_bad_frames, &line[pos]);
  line[pos++] = '\r';
  line[pos++] = '\n';
  ImuUartDebug_SendLine(line, pos);
}

/** 喂入一个带到达时间戳的字节给帧解析状态机。 */
static void ImuUartDebug_ParseByte(uint8_t byte, uint64_t t_us)
{
  switch (imu_dbg_pstate)
  {
  case IMU_DBG_ST_HEAD1:
    if (byte == 0x7EU)
    {
      imu_dbg_frame[0] = byte;
      imu_dbg_pstate = IMU_DBG_ST_HEAD2;
    }
    break;

  case IMU_DBG_ST_HEAD2:
    if (byte == 0x7EU)
    {
      break; /* 连续帧头，保持等待 0x23。 */
    }
    if (byte == 0x23U)
    {
      imu_dbg_frame[1] = byte;
      imu_dbg_pstate = IMU_DBG_ST_LEN;
    }
    else
    {
      imu_dbg_pstate = IMU_DBG_ST_HEAD1;
    }
    break;

  case IMU_DBG_ST_LEN:
    if ((byte >= 7U) && (byte <= IMU_DBG_FRAME_MAX))
    {
      imu_dbg_frame[2] = byte;
      imu_dbg_frame_len = byte;
      imu_dbg_frame_pos = 3U;
      imu_dbg_pstate = IMU_DBG_ST_BODY;
    }
    else
    {
      imu_dbg_pstate = IMU_DBG_ST_HEAD1;
    }
    break;

  case IMU_DBG_ST_BODY:
  default:
    imu_dbg_frame[imu_dbg_frame_pos++] = byte;
    if (imu_dbg_frame_pos >= imu_dbg_frame_len)
    {
      uint8_t checksum = 0U;
      uint16_t i;

      for (i = 0U; (i + 1U) < imu_dbg_frame_len; i++)
      {
        checksum = (uint8_t)(checksum + imu_dbg_frame[i]);
      }
      if (checksum == imu_dbg_frame[imu_dbg_frame_len - 1U])
      {
        ImuUartDebug_EmitFrame(t_us);
      }
      else
      {
        ImuUartDebug_ReportBadFrame();
      }
      imu_dbg_pstate = IMU_DBG_ST_HEAD1;
    }
    break;
  }
}

/** 把本批取出的字节按波特率间隔回推各字节时间后喂给解析器。 */
static void ImuUartDebug_FeedBytes(const uint8_t *data, uint16_t len,
                                   uint64_t t_batch_us)
{
  uint16_t k;

  for (k = 0U; k < len; k++)
  {
    uint64_t t = t_batch_us -
                 (uint64_t)(len - 1U - k) * IMU_DBG_BAUD_BYTE_TIME_US;
    ImuUartDebug_ParseByte(data[k], t);
  }
}

/** 处理主机同步命令行 "!SYNC <epoch_ms>"。 */
static void ImuUartDebug_HandleCommand(void)
{
  const char *p = imu_dbg_cmd + 6;
  uint64_t epoch_ms = 0ULL;
  uint8_t digits = 0U;
  char ack[USB_CAN_PACKET_SIZE];
  uint16_t pos = 0U;

  if (strncmp(imu_dbg_cmd, "!SYNC ", 6U) != 0)
  {
    return;
  }
  while ((*p >= '0') && (*p <= '9'))
  {
    if (digits < 16U)
    {
      epoch_ms = epoch_ms * 10ULL + (uint64_t)(*p - '0');
    }
    digits++;
    p++;
  }
  if ((digits == 0U) || (digits > 16U) || (epoch_ms < IMU_DBG_SYNC_MIN_MS))
  {
    return;
  }

  imu_dbg_epoch_us_anchor = epoch_ms * 1000ULL;
  imu_dbg_counter_anchor = imu_dbg_time_us;
  imu_dbg_synced = 1U;
  if (imu_dbg_rtc_ok != 0U)
  {
    ImuUartDebug_RtcSetFromEpoch(epoch_ms);
  }

  memcpy(&ack[pos], "IMU SYNC: ", 10U);
  pos = (uint16_t)(pos + 10U);
  ack[pos++] = (imu_dbg_rtc_ok != 0U) ? 'o' : 's'; /* ok=已写RTC / soft=仅锚定 */
  ack[pos++] = (imu_dbg_rtc_ok != 0U) ? 'k' : 'f';
  ack[pos++] = ' ';
  pos += ImuUartDebug_U64ToDec(epoch_ms, &ack[pos]);
  ack[pos++] = '\r';
  ack[pos++] = '\n';
  ImuUartDebug_SendLine(ack, pos);
}

/**
 * @brief 主机下行字节旁路钩子（重写 UsbCanGateway 的弱定义）。
 *
 * 逐字节收集 "!SYNC <epoch_ms>\n" 命令行；其它字节直接忽略，不影响
 * AA55/AA59 协议路由。
 */
void UsbCanGateway_ByteTap(uint8_t byte)
{
  if ((byte == '\n') || (byte == '\r'))
  {
    if (imu_dbg_cmd_len > 0U)
    {
      imu_dbg_cmd[imu_dbg_cmd_len] = '\0';
      ImuUartDebug_HandleCommand();
    }
    imu_dbg_cmd_len = 0U;
    return;
  }
  if ((imu_dbg_cmd_len + 1U) < IMU_DBG_SYNC_CMD_MAX)
  {
    imu_dbg_cmd[imu_dbg_cmd_len++] = (char)byte;
  }
  /* 超长行不再入缓冲，末尾回车后整行解析必然失败，安全丢弃。 */
}

HAL_StatusTypeDef ImuUartDebug_Init(void)
{
  /*
   * DMA 时钟和 NVIC 由 CubeMX 生成的 MX_DMA_Init() 配置，必须先于串口
   * 初始化；USART1 的 115200 波特率与 DMA 循环模式来自 .ioc 配置。
   */
  MX_DMA_Init();
  MX_USART1_UART_Init();

  ImuUartDebug_TimInit();
  ImuUartDebug_RtcInit();

  imu_dbg_rd_pos = 0U;
  imu_dbg_total_bytes = 0U;
  imu_dbg_bad_frames = 0U;
  imu_dbg_pstate = IMU_DBG_ST_HEAD1;
  imu_dbg_frame_len = 0U;
  imu_dbg_frame_pos = 0U;
  imu_dbg_cmd_len = 0U;
  imu_dbg_synced = 0U;
  imu_dbg_epoch_us_anchor = 0ULL;
  imu_dbg_counter_anchor = 0ULL;
  imu_dbg_last_data_tick = HAL_GetTick();
  imu_dbg_last_status_tick = imu_dbg_last_data_tick;
  imu_dbg_last_clk_tick = imu_dbg_last_data_tick;
  imu_dbg_last_err_tick = imu_dbg_last_data_tick;

  if ((imu_dbg_rtc_ok != 0U) &&
      (HAL_RTCEx_BKUPRead(&imu_dbg_hrtc, RTC_BKP_DR0) == IMU_DBG_RTC_MAGIC))
  {
    ImuUartDebug_RtcAnchorFromBoot();
  }

  if (HAL_UART_Receive_DMA(&huart1, imu_dbg_dma_ring, IMU_DBG_DMA_RING_SIZE) != HAL_OK)
  {
    return HAL_ERROR;
  }
  imu_dbg_started = 1U;

  {
    static const char start_line[] =
        "IMU DBG: frames+ts ready, send !SYNC <epoch_ms>\r\n";
    ImuUartDebug_SendLine(start_line, (uint16_t)(sizeof(start_line) - 1U));
  }
  return HAL_OK;
}

void ImuUartDebug_Process(void)
{
  uint16_t write_pos;
  uint32_t now;

  if (imu_dbg_started == 0U)
  {
    return;
  }

  ImuUartDebug_TimeUpdate();

  /*
   * NDTR 是 DMA 剩余传输计数，读游标 = 缓冲大小 - 剩余量；游标不一致
   * 即有新字节。读取期间 DMA 仍在写入，这里只消费游标之间的旧数据。
   */
  write_pos = (uint16_t)(IMU_DBG_DMA_RING_SIZE -
                         __HAL_DMA_GET_COUNTER(&hdma_usart1_rx));
  if (write_pos != imu_dbg_rd_pos)
  {
    uint16_t count;
    uint16_t first;

    if (write_pos > imu_dbg_rd_pos)
    {
      count = (uint16_t)(write_pos - imu_dbg_rd_pos);
    }
    else
    {
      /* DMA 写入位置已绕回缓冲区起点。 */
      count = (uint16_t)((IMU_DBG_DMA_RING_SIZE - imu_dbg_rd_pos) + write_pos);
    }

    first = (uint16_t)(IMU_DBG_DMA_RING_SIZE - imu_dbg_rd_pos);
    if (first > count)
    {
      first = count;
    }
    ImuUartDebug_FeedBytes(&imu_dbg_dma_ring[imu_dbg_rd_pos], first,
                           imu_dbg_time_us);
    if (count > first)
    {
      ImuUartDebug_FeedBytes(&imu_dbg_dma_ring[0], (uint16_t)(count - first),
                             imu_dbg_time_us);
    }
    imu_dbg_rd_pos = write_pos;
    imu_dbg_total_bytes = (uint32_t)(imu_dbg_total_bytes + count);
    imu_dbg_last_data_tick = HAL_GetTick();
  }

  now = HAL_GetTick();
  ImuUartDebug_ReportClock(now);
  ImuUartDebug_ReportIdle(now);

#if IMU_DBG_SELFTEST_ENABLE
  /*
   * 自环自测：短接 PA9-PA10 时 COM11 应出现 SELFTEST 字样。接真实
   * IMU 时必须保持 IMU_DBG_SELFTEST_ENABLE 为 0，避免 PA9 对外发送。
   */
  if ((now - imu_dbg_last_selftest_tick) >= IMU_DBG_SELFTEST_PERIOD_MS)
  {
    static const char selftest[] = "SELFTEST-7E23";

    imu_dbg_last_selftest_tick = now;
    (void)HAL_UART_Transmit(&huart1, (const uint8_t *)selftest,
                            (uint16_t)(sizeof(selftest) - 1U), 20U);
  }
#else
  imu_dbg_last_selftest_tick = now;
#endif
}
