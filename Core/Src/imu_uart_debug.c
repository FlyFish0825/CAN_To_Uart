/**
 * @file imu_uart_debug.c
 * @brief 【imu-uart1-debug 调试分支】IMU→WCH-Link 串口桥实现。
 *
 * 数据路径：
 *   USART1 RX（DMA1_Stream0，循环模式，.dma_buffer 段）→ imu_dbg_dma_ring
 *     → 主循环按 NDTR 游标取出新字节
 *     → (1b) 原始字节十六进制行（"IMUWd:<hex>"）原样发上 USB CDC（COM11）：
 *         上位机按行重组即得完整原生帧流，可离线复检协议
 *     → (2) 同时喂给正式后端 ImuSensor_Feed/Process 做片内解析
 *     → (3) 每秒两行统计经 UsbCanGateway_TxEnqueue 上 CDC（COM11）：
 *         "IMUW raw=<n> good=<n> err=<n> drop=<n>" 与
 *         "IMUW r=<raw_seq> q=<quat_seq> e=<euler_seq> b=<baro_seq>"
 *         首次收到版本号时追加一行 "IMUW ver=<a>.<b>.<c>"。
 *
 * 安全约束：
 *   - 只收不发：imu_sensor 以 tx=NULL、pins_blocked=1 初始化，不会向
 *     IMU 发送任何原生命令；PA9 上唯一输出是 PA10 收到字节的回显。
 *   - 无自环自测、无自动校准（历史实现已留档 da651db）。
 *   - DMA1 无法访问 DTCM，环形缓冲必须放在 .dma_buffer 段（RAM_D2）。
 */
#include "imu_uart_debug.h"

#include "imu_sensor.h"
#include "dma.h"
#include "usart.h"
#include "usb_can_gateway.h"

#include <string.h>

/* USART1 DMA 环形缓冲字节数，必须为 2 的幂；115200 波特率下约缓冲 178 ms。 */
#define IMU_DBG_DMA_RING_SIZE 2048U
/* 统计行打印间隔。 */
#define IMU_DBG_STATS_PERIOD_MS 1000U
/*
 * PA9 回显开关：WCH-Link 插着时其串口占用 PA9/PA10（IMU 拉不了低电平），
 * 拔掉后 WCH 串口消失，回显没有接收方；且 IMU 可能接到 PA9，故保持 0。
 */
#define IMU_DBG_ECHO_ENABLE 0U
/*
 * CDC 原始数据开关：把 PA10 收到的字节按十六进制行原样发上 CDC
 * （"IMUWd:<hex>"，每行最多 24 字节），供上位机/离线工具重组完整
 * 原生帧流做协议验证。统计行照常 1Hz。
 */
#define IMU_DBG_CDC_DUMP_ENABLE 1U
/* 每行最多转储的字节数：24*2 hex + 前缀 + 换行 < 78B CDC 单包。 */
#define IMU_DBG_DUMP_LINE_BYTES 24U

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
static uint32_t imu_dbg_last_stats_tick;  /* 最后一次打印统计行的时间。 */
static uint8_t  imu_dbg_started;          /* 初始化完成标志。 */
static uint8_t  imu_dbg_ver_reported;     /* 版本行是否已打印。 */
static uint32_t imu_dbg_rx_restarts;      /* RX DMA 错误重启次数。 */

/*
 * USART1 IDLE 中断与主循环之间的交接（中断里只写，主循环读后清）：
 * 空闲中断在帧突发结束瞬间触发，此时快照 DMA 写游标与毫秒时间戳，
 * 主循环据此把 [读游标, 快照游标) 的字节搬去 CDC 与解析器，时间戳
 * 即"这批字节接收完成"的时刻（用户要求：接收时打时间戳）。
 */
static volatile uint8_t  imu_dbg_idle_flag; /* 1 = 有待搬运的空闲事件。 */
static volatile uint16_t imu_dbg_idle_pos;  /* 中断时刻的 DMA 写游标。 */
static volatile uint32_t imu_dbg_idle_tick; /* 中断时刻（HAL_GetTick 毫秒）。 */

static const char imu_dbg_hex[] = "0123456789ABCDEF";

static void ImuUartDebug_SendLine(const char *text, uint16_t len);

/** 把一段原始字节按十六进制行发上 CDC（IMUWd: 前缀，无损重组用）。 */
static void ImuUartDebug_HexDump(const uint8_t *data, uint16_t len)
{
  char line[USB_CAN_PACKET_SIZE];
  uint16_t sent = 0U;

  while (sent < len)
  {
    uint16_t n = (uint16_t)(len - sent);
    uint16_t pos;
    uint16_t i;

    if (n > IMU_DBG_DUMP_LINE_BYTES)
    {
      n = IMU_DBG_DUMP_LINE_BYTES;
    }
    memcpy(line, "IMUWd:", 6U);
    pos = 6U;
    for (i = 0U; i < n; i++)
    {
      uint8_t b = data[sent + i];

      line[pos++] = imu_dbg_hex[(b >> 4) & 0x0FU];
      line[pos++] = imu_dbg_hex[b & 0x0FU];
    }
    line[pos++] = '\r';
    line[pos++] = '\n';
    ImuUartDebug_SendLine(line, pos);
    sent = (uint16_t)(sent + n);
  }
}

/** 无符号 32 位转十进制，返回写入字符数（值上限 10 位）。 */
static uint16_t ImuUartDebug_U32ToDec(uint32_t value, char *out)
{
  char tmp[10];
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

/** 发送一行文本到 CDC 队列（失败即丢弃，不阻塞主循环）。 */
static void ImuUartDebug_SendLine(const char *text, uint16_t len)
{
  (void)UsbCanGateway_TxEnqueue((const uint8_t *)text, len);
}

void ImuUartDebug_Uart1IrqHook(void)
{
  if (imu_dbg_started == 0U)
  {
    return;
  }
  /* 只认 IDLE：帧突发结束（线空闲一帧时间）即为一批接收完成。其余
   * 中断源不清不碰，交回其后的 HAL_UART_IRQHandler 处理。 */
  if (((huart1.Instance->ISR & USART_ISR_IDLE) != 0U) &&
      ((huart1.Instance->CR1 & USART_CR1_IDLEIE) != 0U))
  {
    huart1.Instance->ICR = USART_ICR_IDLECF; /* 写 1 清除空闲标志。 */
    imu_dbg_idle_pos = (uint16_t)(IMU_DBG_DMA_RING_SIZE -
                                  __HAL_DMA_GET_COUNTER(&hdma_usart1_rx));
    imu_dbg_idle_tick = HAL_GetTick();
    imu_dbg_idle_flag = 1U;
  }
}

/** 每秒输出健康统计（两行）与首次版本号。 */
static void ImuUartDebug_ReportStats(uint32_t now)
{
  ImuSensor_Sample s;
  char line[USB_CAN_PACKET_SIZE];
  uint16_t pos = 0U;

  if ((now - imu_dbg_last_stats_tick) < IMU_DBG_STATS_PERIOD_MS)
  {
    return;
  }
  imu_dbg_last_stats_tick = now;

  (void)ImuSensor_GetSample(&s, (uint64_t)now * 1000ULL);

  memcpy(&line[pos], "IMUW raw=", 9U);
  pos = (uint16_t)(pos + 9U);
  pos += ImuUartDebug_U32ToDec(imu_dbg_total_bytes, &line[pos]);
  memcpy(&line[pos], " good=", 6U);
  pos = (uint16_t)(pos + 6U);
  pos += ImuUartDebug_U32ToDec(s.good_frames, &line[pos]);
  memcpy(&line[pos], " err=", 5U);
  pos = (uint16_t)(pos + 5U);
  pos += ImuUartDebug_U32ToDec(s.bad_checksum_frames + s.bad_length_frames +
                                   s.nonfinite_frames + s.unknown_func_frames,
                               &line[pos]);
  memcpy(&line[pos], " drop=", 6U);
  pos = (uint16_t)(pos + 6U);
  pos += ImuUartDebug_U32ToDec(s.rx_dropped_bytes, &line[pos]);
  memcpy(&line[pos], " rst=", 5U);
  pos = (uint16_t)(pos + 5U);
  pos += ImuUartDebug_U32ToDec(imu_dbg_rx_restarts, &line[pos]);
  line[pos++] = '\r';
  line[pos++] = '\n';
  ImuUartDebug_SendLine(line, pos);

  pos = 0U;
  memcpy(&line[pos], "IMUW r=", 7U);
  pos = (uint16_t)(pos + 7U);
  pos += ImuUartDebug_U32ToDec(s.raw_seq, &line[pos]);
  memcpy(&line[pos], " q=", 3U);
  pos = (uint16_t)(pos + 3U);
  pos += ImuUartDebug_U32ToDec(s.quat_seq, &line[pos]);
  memcpy(&line[pos], " e=", 3U);
  pos = (uint16_t)(pos + 3U);
  pos += ImuUartDebug_U32ToDec(s.euler_seq, &line[pos]);
  memcpy(&line[pos], " b=", 3U);
  pos = (uint16_t)(pos + 3U);
  pos += ImuUartDebug_U32ToDec(s.baro_seq, &line[pos]);
  line[pos++] = '\r';
  line[pos++] = '\n';
  ImuUartDebug_SendLine(line, pos);

  if ((imu_dbg_ver_reported == 0U) && (s.version_valid != 0U))
  {
    pos = 0U;
    memcpy(&line[pos], "IMUW ver=", 9U);
    pos = (uint16_t)(pos + 9U);
    line[pos++] = (char)('0' + s.version[0]);
    line[pos++] = '.';
    line[pos++] = (char)('0' + s.version[1]);
    line[pos++] = '.';
    line[pos++] = (char)('0' + s.version[2]);
    line[pos++] = '\r';
    line[pos++] = '\n';
    ImuUartDebug_SendLine(line, pos);
    imu_dbg_ver_reported = 1U;
  }
}

HAL_StatusTypeDef ImuUartDebug_Init(void)
{
  static const char start_line[] =
      "IMUW: bridge ready, raw echo PA10->PA9, rx-only\r\n";

  /*
   * DMA 时钟和 NVIC 由 CubeMX 生成的 MX_DMA_Init() 配置，必须先于串口
   * 初始化；USART1 的 115200 波特率与 DMA 循环模式来自 .ioc 配置。
   */
  MX_DMA_Init();
  MX_USART1_UART_Init();

  imu_dbg_rd_pos = 0U;
  imu_dbg_total_bytes = 0U;
  imu_dbg_last_stats_tick = HAL_GetTick();
  imu_dbg_ver_reported = 0U;
  imu_dbg_rx_restarts = 0U;
  imu_dbg_idle_flag = 0U;
  imu_dbg_idle_pos = 0U;
  imu_dbg_idle_tick = 0U;

  if (HAL_UART_Receive_DMA(&huart1, imu_dbg_dma_ring, IMU_DBG_DMA_RING_SIZE) != HAL_OK)
  {
    return HAL_ERROR;
  }
  /* 接收搬运由空闲中断驱动：帧突发结束即触发，主循环完成实际搬运。 */
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
  imu_dbg_started = 1U;

  ImuUartDebug_SendLine(start_line, (uint16_t)(sizeof(start_line) - 1U));
  return HAL_OK;
}

void ImuUartDebug_Process(void)
{
  uint16_t write_pos;
  uint16_t target_pos;
  uint32_t now;
  uint32_t stamp;
  uint16_t count;
  uint16_t first;
  uint64_t now_us;

  if (imu_dbg_started == 0U)
  {
    return;
  }
  now = HAL_GetTick();

  /*
   * 错误恢复：HAL 检出 ORE 等错误时会停掉 RX DMA，这里重启接收并重新
   * 武装 IDLE 中断；重启点之前的字节已不可信，直接放弃并计入重启数。
   */
  if (huart1.ErrorCode != HAL_UART_ERROR_NONE)
  {
    huart1.ErrorCode = HAL_UART_ERROR_NONE;
    (void)HAL_UART_AbortReceive(&huart1);
    if (HAL_UART_Receive_DMA(&huart1, imu_dbg_dma_ring,
                             IMU_DBG_DMA_RING_SIZE) != HAL_OK)
    {
      return; /* 重启失败，下一拍再试。 */
    }
    imu_dbg_rd_pos = 0U;
    imu_dbg_idle_flag = 0U;
    imu_dbg_rx_restarts++;
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
  }

  /*
   * 接收搬运由 USART1 空闲中断驱动：IDLE 在帧突发结束瞬间触发，中断里
   * 快照 DMA 写游标与毫秒时间戳并置标志；主循环在此把 [读游标, 快照
   * 游标) 之间的字节搬去 CDC 与解析器，时间戳即中断时刻（这批字节的
   * 接收完成时刻）。
   */
  if (imu_dbg_idle_flag != 0U)
  {
    imu_dbg_idle_flag = 0U;
    target_pos = imu_dbg_idle_pos;
    stamp = imu_dbg_idle_tick;
  }
  else
  {
    /*
     * 安全网：长时间无空闲事件且积压超过半环（主循环停顿、IDLE 丢失）
     * 时强制排空，时间戳退化为当前时刻；正常 25Hz 突发到不了这一步。
     */
    write_pos = (uint16_t)(IMU_DBG_DMA_RING_SIZE -
                           __HAL_DMA_GET_COUNTER(&hdma_usart1_rx));
    if ((uint16_t)(write_pos - imu_dbg_rd_pos) <
        (uint16_t)(IMU_DBG_DMA_RING_SIZE / 2U))
    {
      ImuSensor_Process((uint64_t)now * 1000ULL);
      ImuUartDebug_ReportStats(now);
      return;
    }
    target_pos = write_pos;
    stamp = now;
  }

  if (target_pos >= imu_dbg_rd_pos)
  {
    count = (uint16_t)(target_pos - imu_dbg_rd_pos);
  }
  else
  {
    /* DMA 写入位置已绕回缓冲区起点。 */
    count = (uint16_t)((IMU_DBG_DMA_RING_SIZE - imu_dbg_rd_pos) + target_pos);
  }

  if (count != 0U)
  {
    first = (uint16_t)(IMU_DBG_DMA_RING_SIZE - imu_dbg_rd_pos);
    if (first > count)
    {
      first = count;
    }
    now_us = (uint64_t)stamp * 1000ULL;

#if IMU_DBG_ECHO_ENABLE
    /* (1) 原始字节回显到 PA9：WCH-Link RX→COM42 可捕获完整原生帧流。
     *     阻塞发送按 115200 线速估算超时（约 87us/字节，留倍余量）。 */
    (void)HAL_UART_Transmit(&huart1, &imu_dbg_dma_ring[imu_dbg_rd_pos],
                            first, (uint32_t)((first / 4U) + 10U));
    if (count > first)
    {
      (void)HAL_UART_Transmit(&huart1, &imu_dbg_dma_ring[0],
                              (uint16_t)(count - first),
                              (uint32_t)(((count - first) / 4U) + 10U));
    }
#endif

#if IMU_DBG_CDC_DUMP_ENABLE
    /* (1b) 数据产生时间戳行 + 原始字节十六进制行上 CDC（WCH-Link 拔除
     *      后唯一的数据出口）。IMUWt 行可离线对齐每批字节的到达时刻。 */
    {
      char line[USB_CAN_PACKET_SIZE];
      uint16_t pos = 0U;

      memcpy(line, "IMUWt:", 6U);
      pos = 6U;
      pos += ImuUartDebug_U32ToDec(stamp, &line[pos]);
      line[pos++] = ':';
      pos += ImuUartDebug_U32ToDec(count, &line[pos]);
      line[pos++] = '\r';
      line[pos++] = '\n';
      ImuUartDebug_SendLine(line, pos);
    }
    ImuUartDebug_HexDump(&imu_dbg_dma_ring[imu_dbg_rd_pos], first);
    if (count > first)
    {
      ImuUartDebug_HexDump(&imu_dbg_dma_ring[0], (uint16_t)(count - first));
    }
#endif

    /* (2) 同一批字节喂给正式后端解析：时间戳 = IDLE 中断时刻。 */
    (void)ImuSensor_Feed(&imu_dbg_dma_ring[imu_dbg_rd_pos], first, now_us);
    if (count > first)
    {
      (void)ImuSensor_Feed(&imu_dbg_dma_ring[0], (uint16_t)(count - first),
                           now_us);
    }

    imu_dbg_rd_pos = target_pos;
    imu_dbg_total_bytes = (uint32_t)(imu_dbg_total_bytes + count);
  }

  now = HAL_GetTick();
  ImuSensor_Process((uint64_t)now * 1000ULL);
  ImuUartDebug_ReportStats(now);
}
