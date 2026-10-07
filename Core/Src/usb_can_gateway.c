#include "usb_can_gateway.h"

#include "can_gateway_priority.h"
#include "firmware_flow.h"
#include "sensor_protocol.h"
#include "usbd_cdc_if.h"

typedef struct
{
  /** 当前队列槽保存的有效协议包长度。 */
  uint16_t len;
  /** 当前队列槽保存的协议包字节。 */
  uint8_t data[USB_CAN_PACKET_SIZE];
} UsbCanTxPacket_t;

#define USB_CAN_RX_RING_MASK  (USB_CAN_RX_RING_SIZE - 1U)
#define USB_CAN_TX_QUEUE_HIGH_MASK (USB_CAN_TX_QUEUE_HIGH_SIZE - 1U)
#define USB_CAN_TX_QUEUE_LOW_MASK  (USB_CAN_TX_QUEUE_LOW_SIZE - 1U)
/* 路由器半帧超过该时间没有新字节时，丢弃旧长度并重新同步。 */
#define USB_PROTOCOL_ROUTE_TIMEOUT_MS 1000U

static uint8_t usb_can_rx_ring[USB_CAN_RX_RING_SIZE]; /* USB 回调写入的 RX 字节环。 */
static volatile uint16_t usb_can_rx_head = 0U; /* RX 环写入索引。 */
static volatile uint16_t usb_can_rx_tail = 0U; /* RX 环读取索引。 */
static volatile uint8_t usb_can_rx_paused = 0U; /* 是否暂停提交 USB OUT 接收。 */

/* USB TX 双优先级队列：电机相关 AA55 包进高优先级队列，其余进普通队列。 */
static UsbCanTxPacket_t usb_can_tx_queue_high[USB_CAN_TX_QUEUE_HIGH_SIZE];
static UsbCanTxPacket_t usb_can_tx_queue_low[USB_CAN_TX_QUEUE_LOW_SIZE];
static volatile uint16_t usb_can_tx_head_high = 0U; /* 高优先级入队索引。 */
static volatile uint16_t usb_can_tx_tail_high = 0U; /* 高优先级发送槽索引。 */
static volatile uint16_t usb_can_tx_head_low = 0U; /* 普通队列入队索引。 */
static volatile uint16_t usb_can_tx_tail_low = 0U; /* 普通队列发送槽索引。 */
static volatile uint8_t usb_can_tx_busy = 0U; /* 当前槽是否等待 CDC 完成。 */
static volatile uint8_t usb_can_tx_busy_high = 0U; /* busy 期间在途包所属队列。 */
static volatile uint32_t usb_can_tx_start_tick = 0U; /* 当前传输开始 tick。 */

static volatile uint32_t usb_can_rx_drop_count = 0U; /* RX 环满丢弃的字节数。 */
static volatile uint32_t usb_can_tx_drop_count = 0U; /* TX 队列满丢弃的包数。 */
static volatile uint32_t usb_can_tx_error_count = 0U; /* CDC 提交错误次数。 */
static volatile uint32_t usb_can_tx_stall_count = 0U; /* TX busy 超时恢复次数。 */

typedef enum
{
  USB_PROTOCOL_WAIT_START = 0U,
  USB_PROTOCOL_WAIT_FAMILY,
  USB_PROTOCOL_AA55,
  USB_PROTOCOL_AA59,
  USB_PROTOCOL_AA5B
} UsbProtocolRouteState_t;

static UsbProtocolRouteState_t usb_protocol_route_state =
    USB_PROTOCOL_WAIT_START;
static uint16_t usb_protocol_route_count = 0U;
static uint16_t usb_protocol_route_expected = 0U;
static uint8_t usb_protocol_route_header[16];
static uint32_t usb_protocol_route_last_tick = 0U;
static volatile uint32_t usb_connection_epoch = 0U;

static uint16_t UsbCanGateway_RxFree(void)
{
  uint16_t head;
  uint16_t tail;
  uint16_t used;

  __DMB();
  head = usb_can_rx_head;
  tail = usb_can_rx_tail;
  used = (uint16_t)((head - tail) & USB_CAN_RX_RING_MASK);
  return (uint16_t)((USB_CAN_RX_RING_SIZE - 1U) - used);
}

static uint16_t UsbCanGateway_TxUsed(void)
{
  uint16_t used_high;
  uint16_t used_low;

  __DMB();
  used_high = (uint16_t)((usb_can_tx_head_high - usb_can_tx_tail_high) &
                         USB_CAN_TX_QUEUE_HIGH_MASK);
  used_low = (uint16_t)((usb_can_tx_head_low - usb_can_tx_tail_low) &
                        USB_CAN_TX_QUEUE_LOW_MASK);
  return (uint16_t)(used_high + used_low);
}

static void UsbCanGateway_RouteReset(void)
{
  usb_protocol_route_state = USB_PROTOCOL_WAIT_START;
  usb_protocol_route_count = 0U;
  usb_protocol_route_expected = 0U;
}

static void UsbCanGateway_RouteToParser(uint8_t byte)
{
  if (usb_protocol_route_state == USB_PROTOCOL_AA59)
  {
    FirmwareFlow_RxFeed(&byte, 1U);
  }
  else if (usb_protocol_route_state == USB_PROTOCOL_AA5B)
  {
    SensorProtocol_RxFeed(&byte, 1U, HAL_GetTick());
  }
  else
  {
    CanGateway_RxFeed(&byte, 1U);
  }
}

/**
 * @brief 按 AA55/AA59 帧族把字节流交给唯一的协议解析器。
 *
 * 两种协议都使用 AA 起始字节，不能让两个解析器同时消费同一份负载；
 * 否则 AA59 数据中偶然出现 AA55 可能被误解析成 CAN 命令。路由器只做
 * 帧族和长度识别，不解析 CRC，也不执行任何业务。
 */
static void UsbCanGateway_RouteByte(uint8_t byte)
{
  uint8_t header[2] = {0xAAU, byte};
  uint32_t now = HAL_GetTick();

  if ((usb_protocol_route_state != USB_PROTOCOL_WAIT_START) &&
      ((uint32_t)(now - usb_protocol_route_last_tick) >=
       USB_PROTOCOL_ROUTE_TIMEOUT_MS))
  {
    /* 当前字节继续进入下面的 WAIT_START 分支，允许它成为新帧头。 */
    UsbCanGateway_RouteReset();
  }
  usb_protocol_route_last_tick = now;

  switch (usb_protocol_route_state)
  {
    case USB_PROTOCOL_WAIT_START:
      if (byte == 0xAAU)
      {
        usb_protocol_route_state = USB_PROTOCOL_WAIT_FAMILY;
      }
      break;

    case USB_PROTOCOL_WAIT_FAMILY:
      if ((byte == 0x55U) || (byte == 0x59U) || (byte == 0x5BU))
      {
        usb_protocol_route_state = (byte == 0x59U) ? USB_PROTOCOL_AA59 :
                                   ((byte == 0x5BU) ? USB_PROTOCOL_AA5B : USB_PROTOCOL_AA55);
        usb_protocol_route_count = 2U;
        usb_protocol_route_expected = 0U;
        usb_protocol_route_header[0] = 0xAAU;
        usb_protocol_route_header[1] = byte;
        UsbCanGateway_RouteToParser(0xAAU);
        UsbCanGateway_RouteToParser(byte);
      }
      else
      {
        /* 噪声不丢失新的 AA 候选；无效字节本身交给两个解析器丢弃。 */
        CanGateway_RxFeed(&header[0], 1U);
        FirmwareFlow_RxFeed(&header[0], 1U);
        CanGateway_RxFeed(&byte, 1U);
        FirmwareFlow_RxFeed(&byte, 1U);
        UsbCanGateway_RouteReset();
        if (byte == 0xAAU)
        {
          usb_protocol_route_state = USB_PROTOCOL_WAIT_FAMILY;
        }
      }
      break;

    case USB_PROTOCOL_AA55:
      UsbCanGateway_RouteToParser(byte);
      usb_protocol_route_count++;
      if (usb_protocol_route_count == 3U)
      {
        usb_protocol_route_expected = (uint16_t)byte + 6U;
        if ((byte < 8U) || (byte > 72U))
        {
          UsbCanGateway_RouteReset();
        }
      }
      else if ((usb_protocol_route_expected != 0U) &&
               (usb_protocol_route_count >= usb_protocol_route_expected))
      {
        UsbCanGateway_RouteReset();
      }
      break;

    case USB_PROTOCOL_AA59:
    case USB_PROTOCOL_AA5B:
      UsbCanGateway_RouteToParser(byte);
      if (usb_protocol_route_count < sizeof(usb_protocol_route_header))
      {
        usb_protocol_route_header[usb_protocol_route_count] = byte;
      }
      usb_protocol_route_count++;
      if (usb_protocol_route_count == 16U)
      {
        uint16_t payload_length =
            (uint16_t)usb_protocol_route_header[10] |
            ((uint16_t)usb_protocol_route_header[11] << 8U);
        usb_protocol_route_expected = payload_length + 20U;
        if ((usb_protocol_route_header[2] != 1U) ||
            (payload_length > ((usb_protocol_route_state == USB_PROTOCOL_AA5B) ?
                               SENSOR_MAX_PAYLOAD : FW_FLOW_MAX_PAYLOAD)))
        {
          UsbCanGateway_RouteReset();
        }
      }
      else if ((usb_protocol_route_expected != 0U) &&
               (usb_protocol_route_count >= usb_protocol_route_expected))
      {
        UsbCanGateway_RouteReset();
      }
      break;

    default:
      UsbCanGateway_RouteReset();
      break;
  }
}

/**
 * @brief USB CDC 到协议核心的发送适配器实现。
 *
 * 协议核心只会调用 CanGatewayTransportOps_t.send_packet()，这里把该抽象
 * 调用转换成 USB TX 队列入队。USB 队列会复制 data，因此本函数返回后
 * 核心栈上的临时协议包可以安全复用；真正的 USB 发送由主循环和 CDC
 * 发送完成回调异步推进。
 */
static CanGatewayIoResult_t UsbCanGateway_SendAdapter(
    void *context,
    const uint8_t *data,
    uint16_t length)
{
  HAL_StatusTypeDef status;

  UNUSED(context);

  status = UsbCanGateway_TxEnqueue(data, length);
  if (status == HAL_OK)
  {
    return CAN_GATEWAY_IO_OK;
  }
  if (status == HAL_BUSY)
  {
    return CAN_GATEWAY_IO_BUSY;
  }
  return CAN_GATEWAY_IO_ERROR;
}

/**
 * @brief 把系统心跳的通用发送请求转入同一个外部发送队列。
 *
 * 这里只负责搬运完整字节包，不判断包属于哪一种协议，也不访问 CAN。
 */
static SystemHeartbeatIoResult_t UsbCanGateway_SendSystemAdapter(
    void *context,
    const uint8_t *data,
    uint16_t length)
{
  HAL_StatusTypeDef status;

  UNUSED(context);

  status = UsbCanGateway_TxEnqueue(data, length);
  if (status == HAL_OK)
  {
    return SYSTEM_HEARTBEAT_IO_OK;
  }
  if (status == HAL_BUSY)
  {
    return SYSTEM_HEARTBEAT_IO_BUSY;
  }
  return SYSTEM_HEARTBEAT_IO_ERROR;
}

/**
 * @brief 为系统心跳提供各层软件队列的当前占用率。
 *
 * 这里是传输适配层与诊断状态之间的连接点。心跳核心只知道通用的
 * SystemHeartbeatMetrics_t，不直接依赖 USB、CAN 或 AA59 的内部变量。
 */
static void UsbCanGateway_GetSystemMetrics(
    void *context,
    SystemHeartbeatMetrics_t *metrics)
{
  (void)context;

  if (metrics == NULL)
  {
    return;
  }

  UsbCanGateway_GetBufferUsage(&metrics->input_buffer_percent,
                               &metrics->output_buffer_percent);
  CanGateway_GetQueueUsage(&metrics->can_rx_buffer_percent,
                           &metrics->can_tx_buffer_percent);
  metrics->flow_buffer_percent = FirmwareFlow_GetQueueUsage();
}

static const CanGatewayTransportOps_t usb_can_transport_ops =
{
  UsbCanGateway_SendAdapter,
  NULL
};

static const SystemHeartbeatTransportOps_t usb_system_transport_ops =
{
  UsbCanGateway_SendSystemAdapter,
  NULL,
  UsbCanGateway_GetSystemMetrics,
  NULL
};

/**
 * @brief 返回本模块提供的 USB 传输操作表。
 *
 * 返回静态只读对象，生命周期覆盖整个程序运行期。调用者不应修改该
 * 对象内容，只需把指针传给 CanGateway_Init()。
 */
const CanGatewayTransportOps_t *UsbCanGateway_GetTransport(void)
{
  return &usb_can_transport_ops;
}

const SystemHeartbeatTransportOps_t *UsbCanGateway_GetSystemTransport(void)
{
  return &usb_system_transport_ops;
}

void UsbCanGateway_RxPush(const uint8_t *data, uint16_t len)
{
  uint16_t i;

  if ((data == NULL) || (len == 0U))
  {
    return;
  }

  /*
   * 此函数在 USB OUT 回调中运行：只搬字节，不解析协议、不等待发送。
   * head 由 USB 回调侧推进，tail 由主循环侧推进；环形缓冲用一个空槽
   * 区分“空”和“满”，因此实际可用容量为 USB_CAN_RX_RING_SIZE-1。
   */
  for (i = 0U; i < len; i++)
  {
    uint16_t head = usb_can_rx_head;
    uint16_t next = (uint16_t)((head + 1U) & USB_CAN_RX_RING_MASK);

    if (next == usb_can_rx_tail)
    {
      usb_can_rx_drop_count++;
      usb_can_rx_paused = 1U;
      break;
    }

    usb_can_rx_ring[head] = data[i];
    __DMB();
    usb_can_rx_head = next;
  }
}

/*
 * 判断一个待发送协议包是否属于电机优先级流量。
 * 只有 AA55 CAN 数据帧携带 CAN_ID（偏移 5..8 小端）：电机相关 ID 的
 * 上行回报进高优先级队列；AA5B 传感器遥测、AA58 心跳、状态帧与其余
 * AA55 数据一律进普通队列，为电机数据让路。
 */
static uint8_t UsbCanGateway_TxPacketPriority(const uint8_t *data,
                                              uint16_t len)
{
  uint32_t id;

  if ((len < 9U) || (data[0] != 0xAAU) || (data[1] != 0x55U))
  {
    return 0U;
  }

  id = ((uint32_t)data[5]) |
       (((uint32_t)data[6]) << 8U) |
       (((uint32_t)data[7]) << 16U) |
       (((uint32_t)data[8]) << 24U);
  return CanGateway_IsMotorPriorityId(id);
}

HAL_StatusTypeDef UsbCanGateway_TxEnqueue(const uint8_t *data, uint16_t len)
{
  UsbCanTxPacket_t *queue;
  volatile uint16_t *head;
  volatile uint16_t *tail;
  uint16_t mask;
  uint16_t head_index;
  uint16_t next;
  uint16_t i;
  uint8_t high;

  if ((data == NULL) || (len == 0U) || (len > USB_CAN_PACKET_SIZE))
  {
    return HAL_ERROR;
  }

  high = UsbCanGateway_TxPacketPriority(data, len);
  if (high != 0U)
  {
    queue = usb_can_tx_queue_high;
    head = &usb_can_tx_head_high;
    tail = &usb_can_tx_tail_high;
    mask = USB_CAN_TX_QUEUE_HIGH_MASK;
  }
  else
  {
    queue = usb_can_tx_queue_low;
    head = &usb_can_tx_head_low;
    tail = &usb_can_tx_tail_low;
    mask = USB_CAN_TX_QUEUE_LOW_MASK;
  }

  head_index = *head;
  next = (uint16_t)((head_index + 1U) & mask);
  if (next == (*tail))
  {
    /* 可靠队列满时明确返回 BUSY，不覆盖尚未发送的数据。 */
    usb_can_tx_drop_count++;
    return HAL_BUSY;
  }

  queue[head_index].len = len;
  for (i = 0U; i < len; i++)
  {
    queue[head_index].data[i] = data[i];
  }
  __DMB();
  *head = next;
  return HAL_OK;
}

static void UsbCanGateway_ProcessRx(void)
{
  uint16_t processed = 0U;

  /*
   * USB 是字节流：一个 CDC 回调不一定等于一帧协议包。这里逐字节取出，
   * 由 AA55 解析器负责处理半包、粘包、非法长度以及错误后的重新同步。
   * 每取出一个字节就推进 tail，保证即使解析器发现错误也不会卡住队列。
   * 每轮设置上限，保证连续高速输入时仍会回到 CAN、心跳和 USB TX 服务，
   * 不会因为“清空环形缓冲”而让主循环看起来卡死。
   */
  while ((usb_can_rx_tail != usb_can_rx_head) &&
         (processed < USB_CAN_RX_PROCESS_BUDGET) &&
         (CanGateway_CanTxReady() != 0U))
  {
    uint16_t tail = usb_can_rx_tail;
    uint8_t byte;

    __DMB();
    byte = usb_can_rx_ring[tail];
    usb_can_rx_tail = (uint16_t)((tail + 1U) & USB_CAN_RX_RING_MASK);
    UsbCanGateway_RouteByte(byte);
    processed++;
  }

  /* 空间恢复后再重新提交 OUT 接收，避免缓存满时静默丢字节。 */
  if ((usb_can_rx_paused != 0U) &&
      (UsbCanGateway_TxUsed() <= USB_CAN_TX_LOW_WATERMARK) &&
      (CanGateway_CanTxReady() != 0U) &&
      (UsbCanGateway_RxCanRearm(USB_CAN_RX_PACKET_RESERVE) != 0U))
  {
    if (CDC_ResumeReceive_FS() == USBD_OK)
    {
      usb_can_rx_paused = 0U;
    }
  }
}

uint8_t UsbCanGateway_RxCanAccept(uint16_t len)
{
  if (len >= USB_CAN_RX_RING_SIZE)
  {
    return 0U;
  }
  return (UsbCanGateway_RxFree() >= len) ? 1U : 0U;
}

uint8_t UsbCanGateway_RxCanRearm(uint16_t len)
{
  if (UsbCanGateway_RxCanAccept(len) == 0U)
  {
    return 0U;
  }
  /* 输入可能产生 USB 输出；输出积压到高水位时先暂停 OUT。 */
  if (UsbCanGateway_TxUsed() >= USB_CAN_TX_HIGH_WATERMARK)
  {
    return 0U;
  }
  return (CanGateway_CanTxReady() != 0U) ? 1U : 0U;
}

void UsbCanGateway_RxMarkPaused(void)
{
  usb_can_rx_paused = 1U;
}

static void UsbCanGateway_ProcessTx(void)
{
  UsbCanTxPacket_t *queue;
  uint16_t tail;
  uint8_t high;
  uint8_t result;

  /*
   * USB CDC 同一时刻只允许一个 IN 传输。busy 表示队列尾槽已交给 USB
   * 驱动，必须等待 CDC_TransmitCplt_FS() 后才能释放该槽位。
   */
  if (usb_can_tx_busy != 0U)
  {
    /*
     * 正常情况下由 CDC_TransmitCplt_FS() 清除 busy。若主机断开、USB
     * 重置或底层提交失败导致回调永远不回来，不能让整个发送队列永久
     * 停住。恢复时保留当前 tail，不删除当前包，后续会重新发送它。
     */
    if ((uint32_t)(HAL_GetTick() - usb_can_tx_start_tick) <
        USB_CAN_TX_STALL_TIMEOUT_MS)
    {
      return;
    }

    usb_can_tx_stall_count++;
    CDC_ResetTransmitState_FS();
    __DMB();
    usb_can_tx_busy = 0U;
  }

  /* 高优先级（电机相关）队列非空时先发送，普通队列随后。 */
  if (usb_can_tx_tail_high != usb_can_tx_head_high)
  {
    high = 1U;
    queue = usb_can_tx_queue_high;
  }
  else if (usb_can_tx_tail_low != usb_can_tx_head_low)
  {
    high = 0U;
    queue = usb_can_tx_queue_low;
  }
  else
  {
    return;
  }

  tail = (high != 0U) ? usb_can_tx_tail_high : usb_can_tx_tail_low;
  /* 记录在途包所属队列，完成回调据此推进对应的 tail。 */
  usb_can_tx_busy_high = high;
  /* 先标记 busy，再调用底层，避免极短传输完成回调抢先到达。 */
  usb_can_tx_start_tick = HAL_GetTick();
  __DMB();
  usb_can_tx_busy = 1U;
  result = CDC_Transmit_FS(queue[tail].data, queue[tail].len);
  if (result != USBD_OK)
  {
    __DMB();
    usb_can_tx_busy = 0U;
    usb_can_tx_start_tick = 0U;
    if (result != USBD_BUSY)
    {
      usb_can_tx_error_count++;
    }
  }
}

void UsbCanGateway_TxComplete(void)
{
  if (usb_can_tx_busy == 0U)
  {
    return;
  }

  if (usb_can_tx_busy_high != 0U)
  {
    usb_can_tx_tail_high = (uint16_t)((usb_can_tx_tail_high + 1U) &
                                      USB_CAN_TX_QUEUE_HIGH_MASK);
  }
  else
  {
    usb_can_tx_tail_low = (uint16_t)((usb_can_tx_tail_low + 1U) &
                                     USB_CAN_TX_QUEUE_LOW_MASK);
  }
  __DMB();
  usb_can_tx_busy = 0U;
  usb_can_tx_start_tick = 0U;
}

void UsbCanGateway_OnConfigured(void)
{
  usb_connection_epoch++;
  /* 主机重新枚举后，上一条 USB IN 传输不会再收到完成回调。 */
  usb_can_tx_busy = 0U;
  usb_can_tx_start_tick = 0U;
  usb_can_rx_paused = 0U;
  UsbCanGateway_RouteReset();
  usb_protocol_route_last_tick = HAL_GetTick();
}

void UsbCanGateway_OnDeconfigured(void)
{
  usb_connection_epoch++;
  /*
   * 断开/复位时 CDC 不一定为当前包回调完成事件。只释放本层 busy，
   * 当前队列尾不前移，重新枚举后仍会重试同一个包。
   */
  usb_can_tx_busy = 0U;
  usb_can_tx_start_tick = 0U;
  usb_can_rx_paused = 0U;
  UsbCanGateway_RouteReset();
  usb_protocol_route_last_tick = HAL_GetTick();
}

static uint8_t UsbCanGateway_UsagePercent(uint16_t used, uint16_t capacity)
{
  uint32_t percent;

  if (capacity == 0U)
  {
    return 0U;
  }
  percent = ((uint32_t)used * 100U) / capacity;
  return (percent > 100U) ? 100U : (uint8_t)percent;
}

void UsbCanGateway_GetBufferUsage(uint8_t *rx_percent,
                                  uint8_t *tx_percent)
{
  uint16_t rx_used;
  uint16_t tx_used;

  __DMB();
  rx_used = (uint16_t)((usb_can_rx_head - usb_can_rx_tail) &
                       USB_CAN_RX_RING_MASK);
  tx_used = UsbCanGateway_TxUsed();

  if (rx_percent != NULL)
  {
    *rx_percent = UsbCanGateway_UsagePercent(rx_used,
                                              USB_CAN_RX_RING_SIZE - 1U);
  }
  if (tx_percent != NULL)
  {
    *tx_percent = UsbCanGateway_UsagePercent(tx_used,
                                              USB_CAN_TX_QUEUE_USABLE);
  }
}

uint32_t UsbCanGateway_GetConnectionEpoch(void)
{
  return usb_connection_epoch;
}

void UsbCanGateway_Process(void)
{
  UsbCanGateway_ProcessRx();
  UsbCanGateway_ProcessTx();
}
