#include "can_uart_gateway.h"
#include "fdcan.h"
#include "usart.h"

typedef struct
{
  uint32_t id;
  uint8_t flags;
  uint8_t len;
  uint8_t data[64];
} CanFrame_t;

typedef struct
{
  uint32_t bitrate;
  uint32_t prescaler;
  uint32_t sjw;
  uint32_t seg1;
  uint32_t seg2;
} CanBitTiming_t;

typedef struct
{
  uint16_t len;
  uint8_t data[78];
} UartTxPacket_t;

#define CAN_QUEUE_SIZE          64U
#define CAN_QUEUE_MASK          (CAN_QUEUE_SIZE - 1U)
#define UART_PACKET_SIZE        78U
#define UART_PACKET_BODY_LEN    72U
#define UART_PACKET_MIN_BODY_LEN 8U
#define UART_CONFIG_RESPONSE_BODY_LEN 17U
#define UART_FRAME_START_0      0xAAU
#define UART_FRAME_START_1      0x55U
#define UART_FRAME_END_0        0x55U
#define UART_FRAME_END_1        0xAAU
#define UART_RX_DMA_SIZE        256U
#define UART_RX_RING_SIZE       1024U
#define UART_RX_RING_MASK       (UART_RX_RING_SIZE - 1U)
#define UART_TX_QUEUE_SIZE      16U
#define UART_TX_QUEUE_MASK      (UART_TX_QUEUE_SIZE - 1U)

_Static_assert((UART_RX_DMA_SIZE & (UART_RX_DMA_SIZE - 1U)) == 0U,
               "UART RX DMA size must be a power of two");
_Static_assert((UART_RX_RING_SIZE & (UART_RX_RING_SIZE - 1U)) == 0U,
               "UART RX ring size must be a power of two");
_Static_assert((UART_TX_QUEUE_SIZE & (UART_TX_QUEUE_SIZE - 1U)) == 0U,
               "UART TX queue size must be a power of two");
#define UART_BOOT_TEST_ID       0x7FFU
#define UART_RX_STATUS_ID       0x7FEU
#define CAN_PUT_STATUS_ID       0x7FDU
#define UART_CRC_ERROR_ID       0x7FCU
#define UART_PROTOCOL_ERROR_ID  0x7FBU
#define CAN_PUT_ERROR_ID        0x7FAU

#define CAN_FLAG_EXTENDED       0x01U
#define CAN_FLAG_FD             0x02U
#define CAN_FLAG_BRS            0x04U
#define CAN_FLAG_REMOTE         0x08U
#define CAN_FLAG_VALID_MASK     0x0FU

#define UART_FLAG_CONTROL       0x80U
#define UART_CMD_SET_BITRATE    0x01U
#define UART_RSP_SET_BITRATE    0x81U

#define UART_CFG_OK             0x00U
#define UART_CFG_BAD_RATE       0x01U
#define UART_CFG_APPLY_FAILED   0x02U
static CanFrame_t can_rx_queue[CAN_QUEUE_SIZE];
static volatile uint16_t can_rx_head = 0U;
static volatile uint16_t can_rx_tail = 0U;

static CanFrame_t can_tx_queue[CAN_QUEUE_SIZE];
static volatile uint16_t can_tx_head = 0U;
static volatile uint16_t can_tx_tail = 0U;

static uint8_t uart_rx_dma_buffer[UART_RX_DMA_SIZE]
  __attribute__((section(".dma_buffer"), aligned(32)));
static UartTxPacket_t uart_dma_tx_queue[UART_TX_QUEUE_SIZE]
  __attribute__((section(".dma_buffer"), aligned(32)));

static uint8_t uart_rx_ring[UART_RX_RING_SIZE];
static volatile uint16_t uart_rx_ring_head = 0U;
static volatile uint16_t uart_rx_ring_tail = 0U;
static volatile uint16_t uart_rx_dma_last_pos = 0U;

static uint8_t uart_rx_packet[UART_PACKET_SIZE];
static uint8_t uart_rx_index = 0U;
static uint8_t uart_rx_expected_size = 0U;
static volatile uint8_t uart_rx_restart_pending = 0U;

static volatile uint16_t uart_dma_tx_head = 0U;
static volatile uint16_t uart_dma_tx_tail = 0U;
static volatile uint8_t uart_dma_tx_busy = 0U;
static uint16_t uart_tx_sequence = 0U;

static volatile uint32_t can_rx_drop_count = 0U;
static volatile uint32_t can_rx_hw_lost_count = 0U;
static volatile uint32_t can_tx_drop_count = 0U;
static volatile uint32_t can_tx_fail_count = 0U;
static volatile uint32_t can_tx_submit_count = 0U;
static volatile uint32_t uart_valid_packet_count = 0U;
static volatile uint32_t uart_crc_error_count = 0U;
static volatile uint32_t uart_protocol_error_count = 0U;
static volatile uint32_t uart_rx_error_count = 0U;
static volatile uint32_t uart_tx_error_count = 0U;
static volatile uint32_t uart_rx_ring_drop_count = 0U;
static volatile uint32_t uart_tx_queue_drop_count = 0U;

static volatile uint8_t bitrate_change_pending = 0U;
static uint32_t pending_nominal_bps = 500000U;
static uint32_t pending_data_bps = 5000000U;
static uint16_t pending_config_sequence = 0U;
static uint32_t current_nominal_bps = 500000U;
static uint32_t current_data_bps = 5000000U;
static volatile uint8_t config_response_pending = 0U;
static uint8_t config_response_status = UART_CFG_OK;
static uint16_t config_response_sequence = 0U;

static const CanBitTiming_t nominal_timing_table[] =
{
  {  50000U, 100U, 3U, 12U, 3U },
  { 100000U,  50U, 3U, 12U, 3U },
  { 125000U,  40U, 3U, 12U, 3U },
  { 250000U,  20U, 3U, 12U, 3U },
  { 500000U,  10U, 3U, 12U, 3U },
  { 800000U,   5U, 4U, 15U, 4U },
  {1000000U,   5U, 3U, 12U, 3U }
};

static const CanBitTiming_t data_timing_table[] =
{
  { 500000U, 10U, 3U, 12U, 3U },
  {1000000U,  5U, 3U, 12U, 3U },
  {2000000U,  2U, 4U, 15U, 4U },
  {4000000U,  1U, 4U, 15U, 4U },
  {5000000U,  1U, 3U, 12U, 3U },
  {8000000U,  1U, 2U,  7U, 2U }
};

static void Crc8Hw_Init(void)
{
  __HAL_RCC_CRC_CLK_ENABLE();
  CRC->POL = 0x07U;
  CRC->INIT = 0x00U;
  MODIFY_REG(CRC->CR, CRC_CR_POLYSIZE | CRC_CR_REV_IN | CRC_CR_REV_OUT,
             CRC_CR_POLYSIZE_1);
  SET_BIT(CRC->CR, CRC_CR_RESET);
}

static uint8_t Crc8AtmHw(const uint8_t *data, uint16_t len)
{
  uint32_t primask = __get_PRIMASK();
  uint16_t i = 0U;
  uint16_t value16;
  __IO uint16_t *dr16;

  __disable_irq();
  SET_BIT(CRC->CR, CRC_CR_RESET);

  while ((uint16_t)(len - i) >= 4U)
  {
    CRC->DR = ((uint32_t)data[i] << 24U) |
              ((uint32_t)data[i + 1U] << 16U) |
              ((uint32_t)data[i + 2U] << 8U) |
              (uint32_t)data[i + 3U];
    i += 4U;
  }
  if ((uint16_t)(len - i) == 1U)
  {
    *(__IO uint8_t *)(__IO void *)&CRC->DR = data[i];
  }
  else if ((uint16_t)(len - i) == 2U)
  {
    value16 = ((uint16_t)data[i] << 8U) | data[i + 1U];
    dr16 = (__IO uint16_t *)(__IO void *)&CRC->DR;
    *dr16 = value16;
  }
  else if ((uint16_t)(len - i) == 3U)
  {
    value16 = ((uint16_t)data[i] << 8U) | data[i + 1U];
    dr16 = (__IO uint16_t *)(__IO void *)&CRC->DR;
    *dr16 = value16;
    *(__IO uint8_t *)(__IO void *)&CRC->DR = data[i + 2U];
  }

  value16 = (uint16_t)(CRC->DR & 0xFFU);
  __set_PRIMASK(primask);
  return (uint8_t)value16;
}

static uint8_t CanDlcToLength(uint32_t dlc)
{
  switch (dlc)
  {
    case FDCAN_DLC_BYTES_0:  return 0U;
    case FDCAN_DLC_BYTES_1:  return 1U;
    case FDCAN_DLC_BYTES_2:  return 2U;
    case FDCAN_DLC_BYTES_3:  return 3U;
    case FDCAN_DLC_BYTES_4:  return 4U;
    case FDCAN_DLC_BYTES_5:  return 5U;
    case FDCAN_DLC_BYTES_6:  return 6U;
    case FDCAN_DLC_BYTES_7:  return 7U;
    case FDCAN_DLC_BYTES_8:  return 8U;
    case FDCAN_DLC_BYTES_12: return 12U;
    case FDCAN_DLC_BYTES_16: return 16U;
    case FDCAN_DLC_BYTES_20: return 20U;
    case FDCAN_DLC_BYTES_24: return 24U;
    case FDCAN_DLC_BYTES_32: return 32U;
    case FDCAN_DLC_BYTES_48: return 48U;
    case FDCAN_DLC_BYTES_64: return 64U;
    default:                 return 0U;
  }
}

static uint8_t CanLengthToDlc(uint8_t len, uint32_t *dlc)
{
  if (dlc == NULL)
  {
    return 0U;
  }

  switch (len)
  {
    case 0U:  *dlc = FDCAN_DLC_BYTES_0; break;
    case 1U:  *dlc = FDCAN_DLC_BYTES_1; break;
    case 2U:  *dlc = FDCAN_DLC_BYTES_2; break;
    case 3U:  *dlc = FDCAN_DLC_BYTES_3; break;
    case 4U:  *dlc = FDCAN_DLC_BYTES_4; break;
    case 5U:  *dlc = FDCAN_DLC_BYTES_5; break;
    case 6U:  *dlc = FDCAN_DLC_BYTES_6; break;
    case 7U:  *dlc = FDCAN_DLC_BYTES_7; break;
    case 8U:  *dlc = FDCAN_DLC_BYTES_8; break;
    case 12U: *dlc = FDCAN_DLC_BYTES_12; break;
    case 16U: *dlc = FDCAN_DLC_BYTES_16; break;
    case 20U: *dlc = FDCAN_DLC_BYTES_20; break;
    case 24U: *dlc = FDCAN_DLC_BYTES_24; break;
    case 32U: *dlc = FDCAN_DLC_BYTES_32; break;
    case 48U: *dlc = FDCAN_DLC_BYTES_48; break;
    case 64U: *dlc = FDCAN_DLC_BYTES_64; break;
    default: return 0U;
  }
  return 1U;
}

static uint8_t CanFrame_Validate(const CanFrame_t *frame)
{
  uint32_t dummy_dlc;
  if (frame == NULL) return 0U;
  if ((frame->flags & (uint8_t)(~CAN_FLAG_VALID_MASK)) != 0U) return 0U;
  if ((frame->flags & CAN_FLAG_EXTENDED) != 0U)
  {
    if (frame->id > 0x1FFFFFFFU) return 0U;
  }
  else
  {
    if (frame->id > 0x7FFU) return 0U;
  }

  if ((frame->flags & CAN_FLAG_FD) != 0U)
  {
    if ((frame->flags & CAN_FLAG_REMOTE) != 0U) return 0U;
    if (CanLengthToDlc(frame->len, &dummy_dlc) == 0U) return 0U;
  }
  else
  {
    if ((frame->flags & CAN_FLAG_BRS) != 0U) return 0U;
    if (frame->len > 8U) return 0U;
  }

  return 1U;
}

static HAL_StatusTypeDef UartTx_Enqueue(const uint8_t *data, uint16_t len)
{
  uint16_t head;
  uint16_t next;
  uint16_t i;

  if ((data == NULL) || (len == 0U) || (len > UART_PACKET_SIZE))
  {
    return HAL_ERROR;
  }

  head = uart_dma_tx_head;
  next = (uint16_t)((head + 1U) & UART_TX_QUEUE_MASK);
  if (next == uart_dma_tx_tail)
  {
    uart_tx_queue_drop_count++;
    return HAL_BUSY;
  }

  uart_dma_tx_queue[head].len = len;
  for (i = 0U; i < len; i++)
  {
    uart_dma_tx_queue[head].data[i] = data[i];
  }
  __DMB();
  uart_dma_tx_head = next;
  return HAL_OK;
}

static void UartTx_Process(void)
{
  uint16_t tail;

  if ((uart_dma_tx_busy != 0U) ||
      (uart_dma_tx_tail == uart_dma_tx_head))
  {
    return;
  }

  tail = uart_dma_tx_tail;
  uart_dma_tx_busy = 1U;
  if (HAL_UART_Transmit_DMA(&huart1,
                            uart_dma_tx_queue[tail].data,
                            uart_dma_tx_queue[tail].len) != HAL_OK)
  {
    uart_dma_tx_busy = 0U;
    uart_tx_error_count++;
  }
}

static HAL_StatusTypeDef Uart_SendCanPacket(const CanFrame_t *frame)
{
  uint8_t packet[UART_PACKET_SIZE];
  uint8_t crc_index;
  uint16_t packet_len;
  uint16_t i;

  if (CanFrame_Validate(frame) == 0U)
  {
    return HAL_ERROR;
  }

  /*
   * 此函数只做“CAN 帧 -> 串口协议帧”的封装并写入串口 TX 队列。
   * 它用于 CAN 接收上报、启动提示和状态提示；不会向 CAN 总线发送数据。
   */
  packet[0] = UART_FRAME_START_0;
  packet[1] = UART_FRAME_START_1;
  packet[2] = (uint8_t)(UART_PACKET_MIN_BODY_LEN + frame->len);
  packet[3] = (uint8_t)(uart_tx_sequence & 0xFFU);
  packet[4] = (uint8_t)(uart_tx_sequence >> 8U);
  packet[5] = (uint8_t)(frame->id & 0xFFU);
  packet[6] = (uint8_t)((frame->id >> 8U) & 0xFFU);
  packet[7] = (uint8_t)((frame->id >> 16U) & 0xFFU);
  packet[8] = (uint8_t)((frame->id >> 24U) & 0xFFU);
  packet[9] = frame->flags;
  packet[10] = frame->len;

  for (i = 0U; i < frame->len; i++)
  {
    packet[11U + i] = frame->data[i];
  }

  crc_index = (uint8_t)(11U + frame->len);
  packet[crc_index] = Crc8AtmHw(&packet[2],
                                 (uint16_t)packet[2] + 1U);
  packet[crc_index + 1U] = UART_FRAME_END_0;
  packet[crc_index + 2U] = UART_FRAME_END_1;
  packet_len = (uint16_t)crc_index + 3U;
  if (UartTx_Enqueue(packet, packet_len) != HAL_OK)
  {
    uart_tx_error_count++;
    return HAL_ERROR;
  }

  uart_tx_sequence++;
  return HAL_OK;
}

static void Uart_SendBootTestPacket(void)
{
  CanFrame_t frame = {0};

  frame.id = UART_BOOT_TEST_ID;
  frame.len = 8U;
  frame.data[0] = 'U';
  frame.data[1] = 'A';
  frame.data[2] = 'R';
  frame.data[3] = 'T';
  frame.data[4] = '_';
  frame.data[5] = 'T';
  frame.data[6] = 'X';
  frame.data[7] = '!';
  (void)Uart_SendCanPacket(&frame);
}

static void Uart_SendStatusPacket(uint32_t id, const char text[8])
{
  CanFrame_t frame = {0};
  uint8_t i;

  frame.id = id;
  frame.len = 8U;
  for (i = 0U; i < 8U; i++)
  {
    frame.data[i] = (uint8_t)text[i];
  }
  (void)Uart_SendCanPacket(&frame);
}

static void CanRx_ProcessUart(void)
{
  /*
   * 数据路径 2（CAN -> 上位机）的主循环阶段：
   * HAL_FDCAN_RxFifo0Callback() 已在中断中把 CAN 报文存入 can_rx_queue，
   * 此处取出一帧，封装成 AA 55 ... CRC 55 AA，并交给 UART TX DMA。
   * 每轮只处理一帧，避免 CAN 突发数据长期占用主循环。
   */
  if (can_rx_tail != can_rx_head)
  {
    uint16_t tail = can_rx_tail;
    CanFrame_t frame = can_rx_queue[tail];

    (void)Uart_SendCanPacket(&frame);

    __DMB();
    can_rx_tail = (uint16_t)((tail + 1U) & CAN_QUEUE_MASK);
  }
}

static void CanTx_ProcessBus(void)
{
  FDCAN_TxHeaderTypeDef header;
  CanFrame_t frame;
  uint32_t dlc;
  uint16_t tail;

  /*
   * 数据路径 1（串口 -> CAN）的最终发送阶段：
   * QueueCanTxFromPacket() 已把校验后的串口命令放入 can_tx_queue；
   * 此处转换为 FDCAN 发送头，并写入 FDCAN1 的硬件 TX FIFO。
   * HAL_OK 仅代表写入硬件 FIFO 成功，不代表总线已得到 ACK。
   */
  if (can_tx_tail == can_tx_head) return;
  if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0U) return;

  tail = can_tx_tail;
  frame = can_tx_queue[tail];
  if ((CanFrame_Validate(&frame) == 0U) ||
      (CanLengthToDlc(frame.len, &dlc) == 0U))
  {
    can_tx_drop_count++;
    can_tx_tail = (uint16_t)((tail + 1U) & CAN_QUEUE_MASK);
    return;
  }

  header.Identifier = frame.id;
  header.IdType = ((frame.flags & CAN_FLAG_EXTENDED) != 0U) ?
                  FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
  header.TxFrameType = ((frame.flags & CAN_FLAG_REMOTE) != 0U) ?
                       FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;
  header.DataLength = dlc;
  header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  header.BitRateSwitch = ((frame.flags & CAN_FLAG_BRS) != 0U) ?
                         FDCAN_BRS_ON : FDCAN_BRS_OFF;
  header.FDFormat = ((frame.flags & CAN_FLAG_FD) != 0U) ?
                    FDCAN_FD_CAN : FDCAN_CLASSIC_CAN;
  header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  header.MessageMarker = 0U;

  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header, frame.data) != HAL_OK)
  {
    can_tx_fail_count++;
    return;
  }

  can_tx_submit_count++;
  __DMB();
  can_tx_tail = (uint16_t)((tail + 1U) & CAN_QUEUE_MASK);
}

static const CanBitTiming_t *FindTiming(const CanBitTiming_t *table,
                                        uint32_t count,
                                        uint32_t bitrate)
{
  uint32_t i;
  for (i = 0U; i < count; i++)
  {
    if (table[i].bitrate == bitrate)
    {
      return &table[i];
    }
  }
  return NULL;
}

static uint32_t ReadU32Le(const uint8_t *p)
{
  return ((uint32_t)p[0]) |
         ((uint32_t)p[1] << 8U) |
         ((uint32_t)p[2] << 16U) |
         ((uint32_t)p[3] << 24U);
}

static void WriteU32Le(uint8_t *p, uint32_t value)
{
  p[0] = (uint8_t)(value & 0xFFU);
  p[1] = (uint8_t)((value >> 8U) & 0xFFU);
  p[2] = (uint8_t)((value >> 16U) & 0xFFU);
  p[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

static HAL_StatusTypeDef ApplyCanBitrate(uint32_t nominal_bps,
                                         uint32_t data_bps)
{
  const CanBitTiming_t *nominal;
  const CanBitTiming_t *data;

  nominal = FindTiming(nominal_timing_table,
                       (uint32_t)(sizeof(nominal_timing_table) / sizeof(nominal_timing_table[0])),
                       nominal_bps);
  data = FindTiming(data_timing_table,
                    (uint32_t)(sizeof(data_timing_table) / sizeof(data_timing_table[0])),
                    data_bps);
  if ((nominal == NULL) || (data == NULL))
  {
    return HAL_ERROR;
  }

  if (HAL_FDCAN_Stop(&hfdcan1) != HAL_OK)
  {
    return HAL_ERROR;
  }

  hfdcan1.Init.NominalPrescaler = nominal->prescaler;
  hfdcan1.Init.NominalSyncJumpWidth = nominal->sjw;
  hfdcan1.Init.NominalTimeSeg1 = nominal->seg1;
  hfdcan1.Init.NominalTimeSeg2 = nominal->seg2;
  hfdcan1.Init.DataPrescaler = data->prescaler;
  hfdcan1.Init.DataSyncJumpWidth = data->sjw;
  hfdcan1.Init.DataTimeSeg1 = data->seg1;
  hfdcan1.Init.DataTimeSeg2 = data->seg2;

  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                   FDCAN_ACCEPT_IN_RX_FIFO0,
                                   FDCAN_ACCEPT_IN_RX_FIFO0,
                                   FDCAN_FILTER_REMOTE,
                                   FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan1,
                                     FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                                     FDCAN_IT_RX_FIFO0_MESSAGE_LOST,
                                     0U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    return HAL_ERROR;
  }

  current_nominal_bps = nominal_bps;
  current_data_bps = data_bps;
  return HAL_OK;
}

static void Uart_SendConfigResponse(void)
{
  uint8_t packet[UART_PACKET_SIZE] = {0};

  packet[0] = UART_FRAME_START_0;
  packet[1] = UART_FRAME_START_1;
  packet[2] = UART_CONFIG_RESPONSE_BODY_LEN;
  packet[3] = (uint8_t)(config_response_sequence & 0xFFU);
  packet[4] = (uint8_t)(config_response_sequence >> 8U);
  packet[9] = UART_FLAG_CONTROL;
  packet[10] = UART_RSP_SET_BITRATE;
  packet[11] = config_response_status;
  WriteU32Le(&packet[12], current_nominal_bps);
  WriteU32Le(&packet[16], current_data_bps);
  packet[20] = Crc8AtmHw(&packet[2], 18U);
  packet[21] = UART_FRAME_END_0;
  packet[22] = UART_FRAME_END_1;

  if (UartTx_Enqueue(packet, 23U) != HAL_OK)
  {
    uart_tx_error_count++;
  }
  config_response_pending = 0U;
}

static void QueueCanTxFromPacket(const uint8_t *packet)
{
  CanFrame_t frame;
  uint16_t head;
  uint16_t next;
  uint16_t i;

  /*
   * 数据路径 1（串口 -> CAN）的协议转换点：
   * 输入 packet 已通过帧头、帧尾和 CRC 校验；这里读取 CAN_ID、FLAGS、LEN、DATA，
   * 校验 CAN 帧属性后写入 can_tx_queue。真正访问 FDCAN 硬件在 CanTx_ProcessBus()。
   */
  frame.id = ReadU32Le(&packet[5]);
  frame.flags = packet[9];
  frame.len = packet[10];
  if (((packet[2] != UART_PACKET_BODY_LEN) &&
       (packet[2] != (uint8_t)(UART_PACKET_MIN_BODY_LEN + frame.len))) ||
      (CanFrame_Validate(&frame) == 0U))
  {
    uart_protocol_error_count++;
    return;
  }

  for (i = 0U; i < 64U; i++)
  {
    frame.data[i] = (i < frame.len) ? packet[11U + i] : 0U;
  }

  head = can_tx_head;
  next = (uint16_t)((head + 1U) & CAN_QUEUE_MASK);
  if (next == can_tx_tail)
  {
    can_tx_drop_count++;
    return;
  }

  can_tx_queue[head] = frame;
  __DMB();
  can_tx_head = next;
  uart_valid_packet_count++;
}

static void HandleControlPacket(const uint8_t *packet)
{
  uint32_t nominal_bps;
  uint32_t data_bps;

  if ((packet[2] < 16U) ||
      (packet[9] != UART_FLAG_CONTROL) ||
      (packet[10] != UART_CMD_SET_BITRATE))
  {
    uart_protocol_error_count++;
    return;
  }

  nominal_bps = ReadU32Le(&packet[11]);
  data_bps = ReadU32Le(&packet[15]);

  if ((FindTiming(nominal_timing_table,
                  (uint32_t)(sizeof(nominal_timing_table) / sizeof(nominal_timing_table[0])),
                  nominal_bps) == NULL) ||
      (FindTiming(data_timing_table,
                  (uint32_t)(sizeof(data_timing_table) / sizeof(data_timing_table[0])),
                  data_bps) == NULL))
  {
    config_response_sequence = (uint16_t)packet[3] |
                               ((uint16_t)packet[4] << 8U);
    config_response_status = UART_CFG_BAD_RATE;
    config_response_pending = 1U;
    return;
  }
  pending_nominal_bps = nominal_bps;
  pending_data_bps = data_bps;
  pending_config_sequence = (uint16_t)packet[3] |
                            ((uint16_t)packet[4] << 8U);
  __DMB();
  bitrate_change_pending = 1U;
}

static void UartParser_CommitPacket(void)
{
  uint8_t crc_index;
  uint8_t end_index;
  uint8_t expected_crc;

  /*
   * 串口收包完成后的分流点：先核对帧尾和 CRC，
   * FLAGS.bit7=1 时作为本地配置命令处理；否则进入“串口 -> CAN”路径。
   */
  crc_index = (uint8_t)(uart_rx_packet[2] + 3U);
  end_index = (uint8_t)(crc_index + 1U);
  if ((uart_rx_packet[end_index] != UART_FRAME_END_0) ||
      (uart_rx_packet[end_index + 1U] != UART_FRAME_END_1))
  {
    uart_protocol_error_count++;
    return;
  }

  expected_crc = Crc8AtmHw(&uart_rx_packet[2],
                           (uint16_t)uart_rx_packet[2] + 1U);
  if (expected_crc != uart_rx_packet[crc_index])
  {
    uart_crc_error_count++;
    return;
  }

  if ((uart_rx_packet[9] & UART_FLAG_CONTROL) != 0U)
  {
    HandleControlPacket(uart_rx_packet);
  }
  else
  {
    QueueCanTxFromPacket(uart_rx_packet);
  }
}

static void UartParser_PushByte(uint8_t byte)
{
  if (uart_rx_index == 0U)
  {
    if (byte == UART_FRAME_START_0)
    {
      uart_rx_packet[0] = byte;
      uart_rx_index = 1U;
    }
    return;
  }

  if (uart_rx_index == 1U)
  {
    if (byte == UART_FRAME_START_1)
    {
      uart_rx_packet[1] = byte;
      uart_rx_index = 2U;
    }
    else
    {
      uart_rx_index = (byte == UART_FRAME_START_0) ? 1U : 0U;
      if (uart_rx_index == 1U) uart_rx_packet[0] = UART_FRAME_START_0;
    }
    return;
  }

  if (uart_rx_index == 2U)
  {
    if ((byte < UART_PACKET_MIN_BODY_LEN) ||
        (byte > UART_PACKET_BODY_LEN))
    {
      uart_protocol_error_count++;
      uart_rx_index = (byte == UART_FRAME_START_0) ? 1U : 0U;
      if (uart_rx_index == 1U) uart_rx_packet[0] = UART_FRAME_START_0;
      return;
    }
    uart_rx_packet[2] = byte;
    uart_rx_index = 3U;
    uart_rx_expected_size = (uint8_t)(byte + 6U);
    return;
  }

  if (uart_rx_index < UART_PACKET_SIZE)
  {
    uart_rx_packet[uart_rx_index] = byte;
    uart_rx_index++;
  }

  if ((uart_rx_expected_size != 0U) &&
      (uart_rx_index >= uart_rx_expected_size))
  {
    UartParser_CommitPacket();
    uart_rx_index = 0U;
    uart_rx_expected_size = 0U;
  }
}

static void UartRxRing_Push(const uint8_t *data, uint16_t len)
{
  uint16_t i;

  for (i = 0U; i < len; i++)
  {
    uint16_t head = uart_rx_ring_head;
    uint16_t next = (uint16_t)((head + 1U) & UART_RX_RING_MASK);
    if (next == uart_rx_ring_tail)
    {
      uart_rx_ring_drop_count++;
      break;
    }
    uart_rx_ring[head] = data[i];
    __DMB();
    uart_rx_ring_head = next;
  }
}

static void UartRxDma_Consume(uint16_t pos, HAL_UART_RxEventTypeTypeDef event)
{
  uint16_t last = uart_rx_dma_last_pos;

  if (pos > UART_RX_DMA_SIZE)
  {
    return;
  }

  if (pos == UART_RX_DMA_SIZE)
  {
    if ((last != 0U) || (event == HAL_UART_RXEVENT_TC))
    {
      UartRxRing_Push(&uart_rx_dma_buffer[last],
                      (uint16_t)(UART_RX_DMA_SIZE - last));
    }
    uart_rx_dma_last_pos = 0U;
  }
  else
  {
    if (pos > last)
    {
      UartRxRing_Push(&uart_rx_dma_buffer[last], (uint16_t)(pos - last));
    }
    else if (pos < last)
    {
      UartRxRing_Push(&uart_rx_dma_buffer[last],
                      (uint16_t)(UART_RX_DMA_SIZE - last));
      if (pos > 0U)
      {
        UartRxRing_Push(uart_rx_dma_buffer, pos);
      }
    }
    uart_rx_dma_last_pos = pos;
  }
}

static HAL_StatusTypeDef UartRxDma_Start(void)
{
  uart_rx_dma_last_pos = 0U;
  if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_rx_dma_buffer,
                                   UART_RX_DMA_SIZE) != HAL_OK)
  {
    uart_rx_error_count++;
    return HAL_ERROR;
  }
  uart_rx_restart_pending = 0U;
  return HAL_OK;
}

static void UartRx_Process(void)
{
  /*
   * 串口 DMA 回调只负责把新字节放入 uart_rx_ring；
   * 主循环在这里逐字节执行帧头、长度、CRC、帧尾解析，随后调用
   * QueueCanTxFromPacket()，因此这是“串口原始字节 -> CAN 发送命令”的入口。
   */
  while (uart_rx_ring_tail != uart_rx_ring_head)
  {
    uint16_t tail = uart_rx_ring_tail;
    __DMB();
    uint8_t byte = uart_rx_ring[tail];
    uart_rx_ring_tail = (uint16_t)((tail + 1U) & UART_RX_RING_MASK);
    UartParser_PushByte(byte);
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
  if ((huart != NULL) && (huart->Instance == USART1))
  {
    /* RX DMA 的 IDLE、半满、全满事件：将 DMA 缓冲的新增字节搬入软件环形缓冲。 */
    UartRxDma_Consume(size, HAL_UARTEx_GetRxEventType(huart));
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if ((huart != NULL) && (huart->Instance == USART1))
  {
    uart_dma_tx_tail = (uint16_t)((uart_dma_tx_tail + 1U) &
                                  UART_TX_QUEUE_MASK);
    __DMB();
    uart_dma_tx_busy = 0U;
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if ((huart != NULL) && (huart->Instance == USART1))
  {
    uart_rx_error_count++;
    uart_rx_index = 0U;
    uart_rx_expected_size = 0U;
    uart_rx_restart_pending = 1U;
    if ((uart_dma_tx_busy != 0U) &&
        (huart->gState == HAL_UART_STATE_READY))
    {
      uart_dma_tx_tail = (uint16_t)((uart_dma_tx_tail + 1U) &
                                    UART_TX_QUEUE_MASK);
      uart_dma_tx_busy = 0U;
      uart_tx_error_count++;
    }
  }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs)
{
  /*
   * 数据路径 2（CAN -> 上位机）的中断入口：
   * 从 FDCAN1 RX FIFO0 读出原始 CAN 帧，转换为 CanFrame_t 并写入 can_rx_queue。
   * 中断中不直接调用串口 DMA，实际串口发送由 CanRx_ProcessUart() 在主循环完成。
   */
  if ((hfdcan == NULL) || (hfdcan->Instance != FDCAN1))
  {
    return;
  }

  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) != 0U)
  {
    can_rx_hw_lost_count++;
  }

  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)
  {
    return;
  }

  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U)
  {
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[64];
    CanFrame_t frame;
    uint16_t head;
    uint16_t next;
    uint16_t i;

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0,
                               &rx_header, rx_data) != HAL_OK)
    {
      break;
    }
    frame.id = rx_header.Identifier;
    frame.flags = 0U;
    frame.len = CanDlcToLength(rx_header.DataLength);

    if (rx_header.IdType == FDCAN_EXTENDED_ID) frame.flags |= CAN_FLAG_EXTENDED;
    if (rx_header.FDFormat == FDCAN_FD_CAN) frame.flags |= CAN_FLAG_FD;
    if (rx_header.BitRateSwitch == FDCAN_BRS_ON) frame.flags |= CAN_FLAG_BRS;
    if (rx_header.RxFrameType == FDCAN_REMOTE_FRAME) frame.flags |= CAN_FLAG_REMOTE;

    for (i = 0U; i < 64U; i++)
    {
      if ((rx_header.RxFrameType == FDCAN_DATA_FRAME) && (i < frame.len))
      {
        frame.data[i] = rx_data[i];
      }
      else
      {
        frame.data[i] = 0U;
      }
    }

    head = can_rx_head;
    next = (uint16_t)((head + 1U) & CAN_QUEUE_MASK);
    if (next == can_rx_tail)
    {
      can_rx_drop_count++;
      continue;
    }

    can_rx_queue[head] = frame;
    __DMB();
    can_rx_head = next;
  }
}

HAL_StatusTypeDef CanUartGateway_Init(void)
{
  Crc8Hw_Init();

  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                   FDCAN_ACCEPT_IN_RX_FIFO0,
                                   FDCAN_ACCEPT_IN_RX_FIFO0,
                                   FDCAN_FILTER_REMOTE,
                                   FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan1,
                                     FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                                     FDCAN_IT_RX_FIFO0_MESSAGE_LOST,
                                     0U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (UartRxDma_Start() != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* A valid protocol frame on every reset proves UART TX independently of CAN traffic. */
  Uart_SendBootTestPacket();
  return HAL_OK;
}

void CanUartGateway_Process(void)
{
  uint8_t i;
  static uint32_t reported_uart_valid_packets = 0U;
  static uint32_t reported_can_tx_submits = 0U;
  static uint32_t reported_uart_crc_errors = 0U;
  static uint32_t reported_uart_protocol_errors = 0U;
  static uint32_t reported_can_tx_failures = 0U;

  if (uart_rx_restart_pending != 0U)
  {
    (void)HAL_UART_AbortReceive(&huart1);
    (void)UartRxDma_Start();
  }

  /* 路径 1：处理电脑发来的串口字节，得到待发送 CAN 帧。 */
  UartRx_Process();

  if (bitrate_change_pending != 0U)
  {
    uint32_t nominal_bps = pending_nominal_bps;
    uint32_t data_bps = pending_data_bps;
    uint16_t sequence = pending_config_sequence;

    bitrate_change_pending = 0U;
    config_response_sequence = sequence;
    if (ApplyCanBitrate(nominal_bps, data_bps) == HAL_OK)
    {
      config_response_status = UART_CFG_OK;
    }
    else
    {
      config_response_status = UART_CFG_APPLY_FAILED;
    }
    config_response_pending = 1U;
  }

  /* 路径 1：每轮最多向 FDCAN 硬件提交 3 帧。 */
  for (i = 0U; i < 3U; i++)
  {
    CanTx_ProcessBus();
  }

  if (reported_uart_valid_packets != uart_valid_packet_count)
  {
    reported_uart_valid_packets = uart_valid_packet_count;
    Uart_SendStatusPacket(UART_RX_STATUS_ID, "UART_RX!");
  }
  if (reported_can_tx_submits != can_tx_submit_count)
  {
    reported_can_tx_submits = can_tx_submit_count;
    Uart_SendStatusPacket(CAN_PUT_STATUS_ID, "CAN_PUT!");
  }
  if (reported_uart_crc_errors != uart_crc_error_count)
  {
    reported_uart_crc_errors = uart_crc_error_count;
    Uart_SendStatusPacket(UART_CRC_ERROR_ID, "CRC_ERR!");
  }
  if (reported_uart_protocol_errors != uart_protocol_error_count)
  {
    reported_uart_protocol_errors = uart_protocol_error_count;
    Uart_SendStatusPacket(UART_PROTOCOL_ERROR_ID, "PKT_ERR!");
  }
  if (reported_can_tx_failures != can_tx_fail_count)
  {
    reported_can_tx_failures = can_tx_fail_count;
    Uart_SendStatusPacket(CAN_PUT_ERROR_ID, "CAN_FAIL");
  }

  if (config_response_pending != 0U)
  {
    Uart_SendConfigResponse();
  }

  /* 路径 2：将 CAN RX FIFO 中已接收的帧转发给电脑。 */
  CanRx_ProcessUart();
  /* 统一启动串口 TX DMA，发送路径 2 的 CAN 上报或本地状态提示。 */
  UartTx_Process();
}
