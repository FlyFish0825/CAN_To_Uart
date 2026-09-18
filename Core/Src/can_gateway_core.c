#include "can_gateway_core.h"
#include "fdcan.h"

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

/*
 * AA55 协议解析状态。使用枚举而不是用 0/1/2 等数字表示状态，便于阅读
 * 和维护；每个枚举值都对应接收流程中的一个明确阶段。
 */
typedef enum
{
  GATEWAY_PARSER_WAIT_START_0 = 0U, /* 等待第一个帧头字节 AA */
  GATEWAY_PARSER_WAIT_START_1,      /* 已收到 AA，等待第二个帧头字节 55 */
  GATEWAY_PARSER_WAIT_BODY_LEN,     /* 帧头完成，等待 BODY_LEN */
  GATEWAY_PARSER_READ_BODY          /* 已知总长度，接收剩余字段 */
} GatewayParserState_t;

#define CAN_QUEUE_SIZE          64U
#define CAN_QUEUE_MASK          (CAN_QUEUE_SIZE - 1U)
/* 保留一段余量，不能等到 63 个槽位全部占满才停止接收。 */
#define CAN_TX_QUEUE_HIGH_WATERMARK 48U
/* 成功状态只做低频诊断，不为每个数据帧生成一个 USB 回包。 */
#define GATEWAY_STATUS_REPORT_INTERVAL_MS 100U
/*
 * AA55 协议长度常量：完整固定缓冲区最大 78 字节；BODY_LEN 包含 SEQ、
 * CAN_ID、FLAGS、LEN 和 DATA，不包含帧头、CRC、帧尾；完整帧总长为
 * BODY_LEN + 6。普通数据帧的 BODY_LEN 范围为 8..72。
 */
#define UART_PACKET_SIZE        78U
#define UART_PACKET_BODY_LEN    72U
#define UART_PACKET_MIN_BODY_LEN 8U
#define UART_CONFIG_RESPONSE_BODY_LEN 17U
#define UART_FRAME_START_0      0xAAU
#define UART_FRAME_START_1      0x55U
#define UART_FRAME_END_0        0x55U
#define UART_FRAME_END_1        0xAAU
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
/* CAN 接收软件队列：中断负责写入，主循环负责取出并封装上报。 */
static CanFrame_t can_rx_queue[CAN_QUEUE_SIZE];
static volatile uint16_t can_rx_head = 0U;
static volatile uint16_t can_rx_tail = 0U;

/* CAN 发送软件队列：协议解析后写入，主循环再提交给 FDCAN 硬件 FIFO。 */
static CanFrame_t can_tx_queue[CAN_QUEUE_SIZE];
static volatile uint16_t can_tx_head = 0U;
static volatile uint16_t can_tx_tail = 0U;

/*
 * 协议接收状态：
 * - uart_rx_packet：保存当前正在接收的完整协议帧；
 * - gateway_parser_state：当前解析阶段，由 GatewayParserState_t 枚举表示；
 * - uart_rx_index：当前帧缓存的下一个待写入位置，不再承担状态含义；
 * - uart_rx_expected_size：收到 BODY_LEN 后计算出的完整帧总字节数，
 *   等于 BODY_LEN+6，用于判断何时调用提交函数；
 * - uart_tx_sequence：输出协议帧使用的 16 位序号，每发送一帧递增。
 */
static uint8_t uart_rx_packet[UART_PACKET_SIZE];
static uint8_t uart_rx_index = 0U;
static uint8_t uart_rx_expected_size = 0U;
static uint16_t uart_tx_sequence = 0U;
static CanGatewayTransportOps_t gateway_transport = {0};
static GatewayParserState_t gateway_parser_state = GATEWAY_PARSER_WAIT_START_0;

/*
 * 运行统计计数。计数只用于诊断，不参与协议状态机；声明为 volatile，
 * 便于调试器或异步上下文观察最新值。
 */
static volatile uint32_t can_rx_drop_count = 0U;
static volatile uint32_t can_rx_hw_lost_count = 0U;
static volatile uint32_t can_tx_drop_count = 0U;
static volatile uint32_t can_tx_fail_count = 0U;
static volatile uint32_t can_tx_submit_count = 0U;
static volatile uint32_t uart_valid_packet_count = 0U;
static volatile uint32_t uart_crc_error_count = 0U;
static volatile uint32_t uart_protocol_error_count = 0U;
static volatile uint32_t uart_tx_error_count = 0U;
static volatile uint32_t can_bus_off_count = 0U;
static volatile uint32_t can_error_status_count = 0U;
static volatile uint8_t can_recovery_pending = 0U;

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

static HAL_StatusTypeDef GatewayTx_Enqueue(const uint8_t *data, uint16_t len)
{
  CanGatewayIoResult_t result;

  if ((data == NULL) || (len == 0U) || (len > UART_PACKET_SIZE))
  {
    return HAL_ERROR;
  }

  if (gateway_transport.send_packet == NULL)
  {
    return HAL_ERROR;
  }

  /*
   * 协议核心只调用抽象接口，不依赖任何具体传输驱动。
   * 注意：send_packet() 的 OK 只表示“驱动已复制并接收该包”，不是物理
   * 线路发送完成；BUSY/ERROR 则表示本次调用没有接收该包。
   */
  result = gateway_transport.send_packet(gateway_transport.context,
                                         data, len);
  if (result == CAN_GATEWAY_IO_OK)
  {
    return HAL_OK;
  }
  if (result == CAN_GATEWAY_IO_BUSY)
  {
    return HAL_BUSY;
  }
  return HAL_ERROR;
}

static HAL_StatusTypeDef Gateway_SendCanPacket(const CanFrame_t *frame)
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
   * 此函数只做“CAN 帧 -> AA55 协议帧”的封装并写入外部接口发送队列。
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
  if (GatewayTx_Enqueue(packet, packet_len) != HAL_OK)
  {
    uart_tx_error_count++;
    return HAL_ERROR;
  }

  uart_tx_sequence++;
  return HAL_OK;
}

static void Gateway_SendBootTestPacket(void)
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
  (void)Gateway_SendCanPacket(&frame);
}

static HAL_StatusTypeDef Gateway_SendStatusPacket(uint32_t id,
                                                  const char text[8])
{
  CanFrame_t frame = {0};
  uint8_t i;

  frame.id = id;
  frame.len = 8U;
  for (i = 0U; i < 8U; i++)
  {
    frame.data[i] = (uint8_t)text[i];
  }
  return Gateway_SendCanPacket(&frame);
}

static void CanRx_ProcessTransport(void)
{
  /*
   * 数据路径 2（CAN -> 上位机）的主循环阶段：
   * HAL_FDCAN_RxFifo0Callback() 已在中断中把 CAN 报文存入 can_rx_queue，
   * 此处取出一帧，封装成 AA 55 ... CRC 55 AA，并交给外部接口发送队列。
   * 每轮只处理一帧，避免 CAN 突发数据长期占用主循环。
   */
  if (can_rx_tail != can_rx_head)
  {
    uint16_t tail = can_rx_tail;
    CanFrame_t frame = can_rx_queue[tail];

    /* 外部发送队列暂忙时保留当前 CAN 帧，下一轮继续尝试。 */
    if (Gateway_SendCanPacket(&frame) != HAL_OK)
    {
      return;
    }

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
   * 数据路径 1（外部接口 -> CAN）的最终发送阶段：
   * QueueCanTxFromPacket() 已把校验后的命令放入 can_tx_queue；
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
    /*
     * HAL 已明确拒绝该帧时，不能把软件队列永久卡在同一个 tail。失败帧
     * 不再重复占用队列槽位，错误计数会在主循环转换成 CAN_FAIL 状态包；
     * 后续帧仍可继续尝试，避免一次硬件错误拖死整条 USB->CAN 链路。
     */
    __DMB();
    can_tx_tail = (uint16_t)((tail + 1U) & CAN_QUEUE_MASK);
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

static HAL_StatusTypeDef CanGateway_ActivateNotifications(void)
{
  return HAL_FDCAN_ActivateNotification(
      &hfdcan1,
      FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
      FDCAN_IT_RX_FIFO0_MESSAGE_LOST |
      FDCAN_IT_ERROR_WARNING |
      FDCAN_IT_ERROR_PASSIVE |
      FDCAN_IT_BUS_OFF,
      0U);
}

/**
 * @brief 总线异常后的非阻塞恢复。
 *
 * Bus-Off 时硬件可能不再释放 TX FIFO 中的槽位，单靠软件队列水位会一直
 * 停在“暂不可发送”。在主循环里停止再启动控制器可以让硬件重新退出
 * Bus-Off；不在中断里执行 HAL_Stop，避免在中断上下文等待硬件状态。
 */
static void CanGateway_ProcessCanRecovery(void)
{
  if (can_recovery_pending == 0U)
  {
    return;
  }

  if ((HAL_FDCAN_Stop(&hfdcan1) == HAL_OK) &&
      (HAL_FDCAN_Start(&hfdcan1) == HAL_OK))
  {
    can_recovery_pending = 0U;
  }
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

  if (CanGateway_ActivateNotifications() != HAL_OK)
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

static void Gateway_SendConfigResponse(void)
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

  if (GatewayTx_Enqueue(packet, 23U) != HAL_OK)
  {
    uart_tx_error_count++;
    /* 外部发送队列暂忙时保留 pending，下一轮继续尝试，不能丢配置回复。 */
    return;
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
   * 数据路径 1（外部接口 -> CAN）的协议转换点：
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

/**
 * @brief 提交一帧已按 BODY_LEN 收齐的协议数据。
 *
 * 函数调用前，uart_rx_packet 中已经包含帧头、BODY、CRC 和帧尾，
 * uart_rx_expected_size 也已经由 BODY_LEN 计算完成。函数按以下顺序处理：
 * 1. 检查帧尾，确认长度字段没有导致越界或错位；
 * 2. 计算并比较 CRC8-ATM，拒绝内容损坏的帧；
 * 3. 根据 FLAGS.bit7 分流到配置命令或“外部接口 -> CAN”数据队列。
 *
 * 校验失败时只增加对应错误计数并返回，不会把不完整数据送入 CAN 队列。
 */
static void UartParser_CommitPacket(void)
{
  uint8_t crc_index;
  uint8_t end_index;
  uint8_t expected_crc;

  /*
   * 一帧输入完成后的分流点：先核对帧尾和 CRC，
   * FLAGS.bit7=1 时作为本地配置命令处理；否则进入“外部接口 -> CAN”路径。
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

/**
 * @brief 向 AA55 协议状态机输入一个字节。
 *
 * 这是一个可跨调用保存状态的字节级解析器，适用于输入被拆成半帧、
 * 一帧或多帧粘包的情况。解析状态由 gateway_parser_state 表示，缓存写入
 * 位置由 uart_rx_index 表示；函数每次只消费当前 byte，不等待更多数据，
 * 只有收齐 BODY_LEN+6 字节后才提交整帧。
 * 当帧头或长度非法时，立即回到寻找 AA 55 的状态，保证后续数据可以重新
 * 对齐，而不会因为一个坏字节永久卡在错误位置。
 */
static void UartParser_PushByte(uint8_t byte)
{
  switch (gateway_parser_state)
  {
    case GATEWAY_PARSER_WAIT_START_0:
      /* 还没有进入一帧，只接受 AA 作为帧头候选。 */
      if (byte == UART_FRAME_START_0)
      {
        uart_rx_packet[0] = byte;
        uart_rx_index = 1U;
        gateway_parser_state = GATEWAY_PARSER_WAIT_START_1;
      }
      break;

    case GATEWAY_PARSER_WAIT_START_1:
      /*
       * 已收到 AA，只有继续收到 55 才确认帧头。
       * 如果当前字节仍是 AA，则把它作为新的帧头候选，支持 AA AA 55
       * 这种连续输入；其他字节则回到等待 AA 状态。
       */
      if (byte == UART_FRAME_START_1)
      {
        uart_rx_packet[1] = byte;
        uart_rx_index = 2U;
        gateway_parser_state = GATEWAY_PARSER_WAIT_BODY_LEN;
      }
      else if (byte == UART_FRAME_START_0)
      {
        uart_rx_packet[0] = UART_FRAME_START_0;
        uart_rx_index = 1U;
        gateway_parser_state = GATEWAY_PARSER_WAIT_START_1;
      }
      else
      {
        uart_rx_index = 0U;
        gateway_parser_state = GATEWAY_PARSER_WAIT_START_0;
      }
      break;

    case GATEWAY_PARSER_WAIT_BODY_LEN:
      /*
       * 读取 BODY_LEN 并检查范围。长度合法后计算完整帧总长度 BODY_LEN+6，
       * 后续只按协议长度接收，不依赖底层接口的分包大小。
       */
      if ((byte < UART_PACKET_MIN_BODY_LEN) ||
          (byte > UART_PACKET_BODY_LEN))
      {
        uart_protocol_error_count++;
        if (byte == UART_FRAME_START_0)
        {
          uart_rx_packet[0] = UART_FRAME_START_0;
          uart_rx_index = 1U;
          gateway_parser_state = GATEWAY_PARSER_WAIT_START_1;
        }
        else
        {
          uart_rx_index = 0U;
          gateway_parser_state = GATEWAY_PARSER_WAIT_START_0;
        }
      }
      else
      {
        uart_rx_packet[2] = byte;
        uart_rx_index = 3U;
        uart_rx_expected_size = (uint8_t)(byte + 6U);
        gateway_parser_state = GATEWAY_PARSER_READ_BODY;
      }
      break;

    case GATEWAY_PARSER_READ_BODY:
      /* 已知完整帧长度，依次保存 BODY、CRC 和帧尾字节。 */
      if (uart_rx_index < UART_PACKET_SIZE)
      {
        uart_rx_packet[uart_rx_index] = byte;
        uart_rx_index++;
      }

      /* 收齐整帧后校验并分流，然后回到下一帧的帧头搜索状态。 */
      if ((uart_rx_expected_size != 0U) &&
          (uart_rx_index >= uart_rx_expected_size))
      {
        UartParser_CommitPacket();
        uart_rx_index = 0U;
        uart_rx_expected_size = 0U;
        gateway_parser_state = GATEWAY_PARSER_WAIT_START_0;
      }
      break;

    default:
      /* 防御性处理：状态异常时清空当前帧并重新寻找帧头。 */
      uart_rx_index = 0U;
      uart_rx_expected_size = 0U;
      gateway_parser_state = GATEWAY_PARSER_WAIT_START_0;
      break;
  }
}

/**
 * @brief 向协议状态机输入电脑侧字节流。
 *
 * 传输层可以按任意粒度调用本函数。这里不假设 data 是完整协议帧，而是
 * 逐字节推进 AA55 状态机；状态机会在多次调用之间保留当前帧的半包状态。
 */
void CanGateway_RxFeed(const uint8_t *data, uint16_t len)
{
  uint16_t i;

  if ((data == NULL) || (len == 0U))
  {
    return;
  }

  /* 接收驱动在主循环中调用此入口，复用统一 AA55 协议状态机。 */
  for (i = 0U; i < len; i++)
  {
    UartParser_PushByte(data[i]);
  }
}

uint8_t CanGateway_CanTxReady(void)
{
  uint16_t used;

  __DMB();
  used = (uint16_t)((can_tx_head - can_tx_tail) & CAN_QUEUE_MASK);
  return (used < CAN_TX_QUEUE_HIGH_WATERMARK) ? 1U : 0U;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs)
{
  /*
   * 数据路径 2（CAN -> 上位机）的中断入口：
   * 从 FDCAN1 RX FIFO0 读出原始 CAN 帧，转换为 CanFrame_t 并写入 can_rx_queue。
   * 中断中不直接调用外部接口发送，实际上报由 CanRx_ProcessTransport()
   * 在主循环完成。
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

/**
 * @brief FDCAN 错误状态通知。
 *
 * 这里只记录事件并置位恢复请求；恢复动作统一留给主循环，避免中断中
 * 进行 Stop/Start 或其他可能等待硬件的操作。错误不会被当成正常发送完成。
 */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan,
                                   uint32_t ErrorStatusITs)
{
  if ((hfdcan == NULL) || (hfdcan->Instance != FDCAN1))
  {
    return;
  }

  can_error_status_count++;
  if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != 0U)
  {
    can_bus_off_count++;
    can_recovery_pending = 1U;
  }
}

/**
 * @brief 初始化 CAN 网关核心并注册电脑侧发送接口。
 *
 * 初始化顺序必须是：底层 HAL/HAL 时钟 -> FDCAN 与外部接口初始化
 * -> 本函数。函数首先检查并复制接口操作表，然后初始化 CRC 硬件、配置
 * FDCAN 接收过滤器/通知并启动 FDCAN，最后排入一帧上电测试协议包。
 */
HAL_StatusTypeDef CanGateway_Init(const CanGatewayTransportOps_t *transport)
{
  if ((transport == NULL) || (transport->send_packet == NULL))
  {
    return HAL_ERROR;
  }

  /*
   * 复制接口内容，调用方不需要保证 transport 结构体本身长期有效；但
   * transport->context（如果非 NULL）指向的实例必须在网关运行期间有效。
   */
  gateway_transport = *transport;

  Crc8Hw_Init();

  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                   FDCAN_ACCEPT_IN_RX_FIFO0,
                                   FDCAN_ACCEPT_IN_RX_FIFO0,
                                   FDCAN_FILTER_REMOTE,
                                   FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (CanGateway_ActivateNotifications() != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* 上电发送一帧标准 AA55 测试报文，验证外部接口发送链路。 */
  Gateway_SendBootTestPacket();
  return HAL_OK;
}

/**
 * @brief 网关核心的一次主循环轮询。
 *
 * 本函数不直接读取外部接口，而是处理已经通过 CanGateway_RxFeed() 进入
 * 解析器的输入数据，并驱动“外部接口->CAN”和“CAN->外部接口”两条软件
 * 队列。建议在 while(1) 中持续调用，不能只调用一次；函数本身不阻塞，
 * 适合与传输层服务函数交替调度。
 */
void CanGateway_Process(void)
{
  uint8_t i;
  uint32_t status_now;
  uint8_t status_due;
  static uint32_t reported_uart_valid_packets = 0U;
  static uint32_t reported_can_tx_submits = 0U;
  static uint32_t reported_uart_crc_errors = 0U;
  static uint32_t reported_uart_protocol_errors = 0U;
  static uint32_t reported_can_tx_failures = 0U;
  static uint32_t reported_can_tx_drops = 0U;
  static uint32_t reported_can_bus_off = 0U;
  static uint32_t reported_can_errors = 0U;
  static uint32_t status_report_tick = 0U;

  CanGateway_ProcessCanRecovery();

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

  /*
   * 成功状态包按时间合并：连续输入时每帧回一个 USB 包会形成反馈风暴，
   * 最终由诊断包自己占满 USB TX 队列。错误仍会报告，但同样限制为每
   * 100 ms 至多一个状态包；计数器只在真正入队成功后前移。
   */
  status_now = HAL_GetTick();
  status_due = ((uint32_t)(status_now - status_report_tick) >=
                GATEWAY_STATUS_REPORT_INTERVAL_MS) ? 1U : 0U;
  if (status_due != 0U)
  {
    status_report_tick = status_now;
    if ((reported_can_bus_off != can_bus_off_count) &&
        (Gateway_SendStatusPacket(CAN_PUT_ERROR_ID, "BUS_OFF!") == HAL_OK))
    {
      reported_can_bus_off = can_bus_off_count;
    }
    else if ((reported_can_errors != can_error_status_count) &&
             (Gateway_SendStatusPacket(CAN_PUT_ERROR_ID, "CAN_ERR!") == HAL_OK))
    {
      reported_can_errors = can_error_status_count;
    }
    else if ((reported_can_tx_failures != can_tx_fail_count) &&
             (Gateway_SendStatusPacket(CAN_PUT_ERROR_ID, "CAN_FAIL") == HAL_OK))
    {
      reported_can_tx_failures = can_tx_fail_count;
    }
    else if ((reported_can_tx_drops != can_tx_drop_count) &&
             (Gateway_SendStatusPacket(CAN_PUT_ERROR_ID, "CAN_QFUL") == HAL_OK))
    {
      /* 队列满丢帧必须显式告知上位机，不能只留在调试计数里。 */
      reported_can_tx_drops = can_tx_drop_count;
    }
    else if ((reported_uart_crc_errors != uart_crc_error_count) &&
             (Gateway_SendStatusPacket(UART_CRC_ERROR_ID, "CRC_ERR!") == HAL_OK))
    {
      reported_uart_crc_errors = uart_crc_error_count;
    }
    else if ((reported_uart_protocol_errors != uart_protocol_error_count) &&
             (Gateway_SendStatusPacket(UART_PROTOCOL_ERROR_ID, "PKT_ERR!") == HAL_OK))
    {
      reported_uart_protocol_errors = uart_protocol_error_count;
    }
    else if ((reported_uart_valid_packets != uart_valid_packet_count) &&
             (Gateway_SendStatusPacket(UART_RX_STATUS_ID, "USB_RX!!") == HAL_OK))
    {
      reported_uart_valid_packets = uart_valid_packet_count;
    }
    else if ((reported_can_tx_submits != can_tx_submit_count) &&
             (Gateway_SendStatusPacket(CAN_PUT_STATUS_ID, "CAN_PUT!") == HAL_OK))
    {
      reported_can_tx_submits = can_tx_submit_count;
    }
  }

  if (config_response_pending != 0U)
  {
    Gateway_SendConfigResponse();
  }

  /* 将 CAN RX FIFO 中已接收的帧转发给外部接口。 */
  CanRx_ProcessTransport();
}
