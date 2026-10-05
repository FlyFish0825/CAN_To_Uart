/**
 * @file imu_uart_debug.c
 * @brief 【imu-uart1-debug 测试分支】IMU 串口读取调试实现。
 *
 * 调试数据路径：
 *   USART1 RX（DMA1_Stream0，循环模式）-> imu_dbg_dma_ring
 *     -> 主循环按 NDTR 游标搬入 imu_dbg_acc
 *     -> 每 16 字节组一行 "IMU: xx xx ..."，写入 USB CDC 发送队列
 * 打印不解析协议、不改动字节内容；行前缀 IMU: 仅用于人工区分输出来源。
 */
#include "imu_uart_debug.h"

#include "dma.h"
#include "usart.h"
#include "usb_can_gateway.h"

#include <string.h>

/* USART1 DMA 环形缓冲字节数，必须为 2 的幂；921600 波特率下约缓冲 22 ms。 */
#define IMU_DBG_DMA_RING_SIZE 2048U
/* 待打印累积缓冲字节数；CDC 发送队列暂满时数据先在这里排队。 */
#define IMU_DBG_ACC_SIZE 4096U
/* 每行最多打印的原始字节数；一行文本最长 5 + 16*3 - 1 + 2 = 54 字节。 */
#define IMU_DBG_BYTES_PER_LINE 16U
/* 无新数据时打印统计状态行的最小间隔。 */
#define IMU_DBG_IDLE_PERIOD_MS 1000U
/* IMU 实际波特率；.ioc 中 USART1 的 921600 是旧主机链路遗留值。 */
#define IMU_DBG_BAUDRATE 115200U

/* USART1 RX DMA 句柄由 CubeMX 生成在 usart.c 中，非 static，可外部引用。 */
extern DMA_HandleTypeDef hdma_usart1_rx;

/*
 * DMA1 无法访问 DTCM（链接脚本明确注明），因此环形缓冲必须放在
 * .dma_buffer 段（RAM_D2 0x30000000）；累积缓冲只有 CPU 访问，可留在 .bss。
 */
__attribute__((section(".dma_buffer"), aligned(32)))
static uint8_t imu_dbg_dma_ring[IMU_DBG_DMA_RING_SIZE];
/* 待打印字节累积缓冲。 */
static uint8_t imu_dbg_acc[IMU_DBG_ACC_SIZE];

static uint16_t imu_dbg_acc_len;          /* 累积缓冲中的待打印字节数。 */
static uint16_t imu_dbg_rd_pos;           /* DMA 环形缓冲读取游标。 */
static uint32_t imu_dbg_total_bytes;      /* 累计收到的原始字节数。 */
static uint32_t imu_dbg_drop_bytes;       /* 累积缓冲溢出时丢弃的字节数。 */
static uint32_t imu_dbg_last_data_tick;   /* 最后一次收到字节的时间。 */
static uint32_t imu_dbg_last_status_tick; /* 最后一次打印状态行的时间。 */
static uint8_t imu_dbg_started;           /* 初始化完成标志。 */

static const char imu_dbg_hex[] = "0123456789ABCDEF";

/**
 * @brief 把新收到的字节追加进累积缓冲，放不下时丢弃并计数。
 * @return 实际保存的字节数。
 */
static uint16_t ImuUartDebug_AccPush(const uint8_t *data, uint16_t len)
{
  uint16_t space;
  uint16_t keep;

  if (len == 0U)
  {
    return 0U;
  }
  space = (uint16_t)(IMU_DBG_ACC_SIZE - imu_dbg_acc_len);
  keep = (len > space) ? space : len;
  if (len > space)
  {
    /* 调试缓冲溢出：丢弃放不下的部分，数量由空闲状态行汇报。 */
    imu_dbg_drop_bytes = (uint32_t)(imu_dbg_drop_bytes + (len - space));
  }
  if (keep != 0U)
  {
    memcpy(&imu_dbg_acc[imu_dbg_acc_len], data, keep);
    imu_dbg_acc_len = (uint16_t)(imu_dbg_acc_len + keep);
  }
  return keep;
}

/**
 * @brief 把累积缓冲中的字节按行格式化并写入 CDC 发送队列。
 *
 * 队列返回 BUSY 时立即停止，剩余字节留在累积缓冲中等待下一轮重试，
 * 已保存的字节不会因为队列暂满而丢失。
 */
static void ImuUartDebug_FlushLines(void)
{
  char line[USB_CAN_PACKET_SIZE];

  while (imu_dbg_acc_len != 0U)
  {
    uint16_t chunk = (imu_dbg_acc_len > IMU_DBG_BYTES_PER_LINE)
                         ? IMU_DBG_BYTES_PER_LINE
                         : imu_dbg_acc_len;
    uint16_t pos = 0U;
    uint16_t i;

    line[pos++] = 'I';
    line[pos++] = 'M';
    line[pos++] = 'U';
    line[pos++] = ':';
    line[pos++] = ' ';
    for (i = 0U; i < chunk; i++)
    {
      uint8_t byte = imu_dbg_acc[i];
      line[pos++] = imu_dbg_hex[(byte >> 4) & 0x0FU];
      line[pos++] = imu_dbg_hex[byte & 0x0FU];
      line[pos++] = ' ';
    }
    pos = (uint16_t)(pos - 1U); /* 去掉行尾多余的空格。 */
    line[pos++] = '\r';
    line[pos++] = '\n';

    if (UsbCanGateway_TxEnqueue((const uint8_t *)line, pos) != HAL_OK)
    {
      break; /* 队列满：剩余数据下一轮继续发送。 */
    }
    memmove(&imu_dbg_acc[0], &imu_dbg_acc[chunk],
            (size_t)(imu_dbg_acc_len - chunk));
    imu_dbg_acc_len = (uint16_t)(imu_dbg_acc_len - chunk);
  }
}

/** 把无符号数转成十进制文本，返回写入的字符数。 */
static uint16_t ImuUartDebug_U32ToDec(uint32_t value, char *out)
{
  char tmp[10];
  uint16_t count = 0U;
  uint16_t i;

  do
  {
    tmp[count++] = (char)('0' + (value % 10U));
    value /= 10U;
  } while (value != 0U);
  for (i = 0U; i < count; i++)
  {
    out[i] = tmp[count - 1U - i];
  }
  return count;
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

  memcpy(&line[pos], "IMU: idle total=", 16U);
  pos = (uint16_t)(pos + 16U);
  pos = (uint16_t)(pos + ImuUartDebug_U32ToDec(imu_dbg_total_bytes, &line[pos]));
  memcpy(&line[pos], " drop=", 6U);
  pos = (uint16_t)(pos + 6U);
  pos = (uint16_t)(pos + ImuUartDebug_U32ToDec(imu_dbg_drop_bytes, &line[pos]));
  line[pos++] = '\r';
  line[pos++] = '\n';

  /* 状态行是辅助信息，队列满时直接丢弃即可，不影响数据行。 */
  (void)UsbCanGateway_TxEnqueue((const uint8_t *)line, pos);
  imu_dbg_last_status_tick = now;
}

HAL_StatusTypeDef ImuUartDebug_Init(void)
{
  /*
   * DMA 时钟和 NVIC 由 CubeMX 生成的 MX_DMA_Init() 配置，必须先于串口
   * 初始化；USART1 的 921600 波特率与 DMA 循环模式来自 .ioc 配置。
   */
  MX_DMA_Init();
  MX_USART1_UART_Init();

  /*
   * .ioc 中 USART1 默认 921600 为旧主机链路遗留值，与 IMU 不符；此处
   * 直接用 HAL 按新波特率重新初始化，避免为此重新生成 CubeMX 代码。
   * 正式版本确认波特率后，应在 .ioc 中修改并重新生成、删掉本段。
   */
  huart1.Init.BaudRate = IMU_DBG_BAUDRATE;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    return HAL_ERROR;
  }

  imu_dbg_acc_len = 0U;
  imu_dbg_rd_pos = 0U;
  imu_dbg_total_bytes = 0U;
  imu_dbg_drop_bytes = 0U;
  imu_dbg_last_data_tick = HAL_GetTick();
  imu_dbg_last_status_tick = imu_dbg_last_data_tick;

  if (HAL_UART_Receive_DMA(&huart1, imu_dbg_dma_ring, IMU_DBG_DMA_RING_SIZE) != HAL_OK)
  {
    return HAL_ERROR;
  }
  imu_dbg_started = 1U;

  /* 启动提示行：电脑端一打开 COM 口即可确认固件已在采集。 */
  {
    static const char start_line[] = "IMU DBG: usart1 115200 dma rx start\r\n";
    (void)UsbCanGateway_TxEnqueue((const uint8_t *)start_line,
                                  (uint16_t)(sizeof(start_line) - 1U));
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
    (void)ImuUartDebug_AccPush(&imu_dbg_dma_ring[imu_dbg_rd_pos], first);
    if (count > first)
    {
      (void)ImuUartDebug_AccPush(&imu_dbg_dma_ring[0],
                                 (uint16_t)(count - first));
    }
    imu_dbg_rd_pos = write_pos;
    imu_dbg_total_bytes = (uint32_t)(imu_dbg_total_bytes + count);
    imu_dbg_last_data_tick = HAL_GetTick();
  }

  ImuUartDebug_FlushLines();
  now = HAL_GetTick();
  ImuUartDebug_ReportIdle(now);
}
