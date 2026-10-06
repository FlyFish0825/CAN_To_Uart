/**
 * @file imu_uart_debug.c
 * @brief 【imu-uart1-debug 调试分支】IMU→WCH-Link 串口桥实现。
 *
 * 数据路径：
 *   USART1 RX（DMA1_Stream0，循环模式，.dma_buffer 段）→ imu_dbg_dma_ring
 *     → 主循环按 NDTR 游标取出新字节
 *     → (1) 原始字节阻塞回显到 PA9（→WCH-Link RX→PC COM42，供离线解析
 *         原生帧流；115200 线速约 87us/字节，批量小、阻塞可忽略）
 *     → (2) 喂给正式后端 ImuSensor_Feed/Process 做片内解析
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

  if (HAL_UART_Receive_DMA(&huart1, imu_dbg_dma_ring, IMU_DBG_DMA_RING_SIZE) != HAL_OK)
  {
    return HAL_ERROR;
  }
  imu_dbg_started = 1U;

  ImuUartDebug_SendLine(start_line, (uint16_t)(sizeof(start_line) - 1U));
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
    uint64_t now_us;

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

    now_us = (uint64_t)HAL_GetTick() * 1000ULL;

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

    /* (2) 同一批字节喂给正式后端解析（时间戳为 HAL 毫秒 ×1000）。 */
    (void)ImuSensor_Feed(&imu_dbg_dma_ring[imu_dbg_rd_pos], first, now_us);
    if (count > first)
    {
      (void)ImuSensor_Feed(&imu_dbg_dma_ring[0], (uint16_t)(count - first),
                           now_us);
    }

    imu_dbg_rd_pos = write_pos;
    imu_dbg_total_bytes = (uint32_t)(imu_dbg_total_bytes + count);
  }

  now = HAL_GetTick();
  ImuSensor_Process((uint64_t)now * 1000ULL);
  ImuUartDebug_ReportStats(now);
}
