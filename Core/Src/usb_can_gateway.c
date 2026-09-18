#include "usb_can_gateway.h"

#include "usbd_cdc_if.h"

typedef struct
{
  uint16_t len;
  uint8_t data[USB_CAN_PACKET_SIZE];
} UsbCanTxPacket_t;

#define USB_CAN_RX_RING_MASK  (USB_CAN_RX_RING_SIZE - 1U)
#define USB_CAN_TX_QUEUE_MASK (USB_CAN_TX_QUEUE_SIZE - 1U)

static uint8_t usb_can_rx_ring[USB_CAN_RX_RING_SIZE];
static volatile uint16_t usb_can_rx_head = 0U;
static volatile uint16_t usb_can_rx_tail = 0U;
static volatile uint8_t usb_can_rx_paused = 0U;

static UsbCanTxPacket_t usb_can_tx_queue[USB_CAN_TX_QUEUE_SIZE];
static volatile uint16_t usb_can_tx_head = 0U;
static volatile uint16_t usb_can_tx_tail = 0U;
static volatile uint8_t usb_can_tx_busy = 0U;
static volatile uint32_t usb_can_tx_start_tick = 0U;

static volatile uint32_t usb_can_rx_drop_count = 0U;
static volatile uint32_t usb_can_tx_drop_count = 0U;
static volatile uint32_t usb_can_tx_error_count = 0U;
static volatile uint32_t usb_can_tx_stall_count = 0U;

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
  uint16_t head;
  uint16_t tail;

  __DMB();
  head = usb_can_tx_head;
  tail = usb_can_tx_tail;
  return (uint16_t)((head - tail) & USB_CAN_TX_QUEUE_MASK);
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

static const CanGatewayTransportOps_t usb_can_transport_ops =
{
  UsbCanGateway_SendAdapter,
  NULL
};

static const SystemHeartbeatTransportOps_t usb_system_transport_ops =
{
  UsbCanGateway_SendSystemAdapter,
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

HAL_StatusTypeDef UsbCanGateway_TxEnqueue(const uint8_t *data, uint16_t len)
{
  uint16_t head;
  uint16_t next;
  uint16_t i;

  if ((data == NULL) || (len == 0U) || (len > USB_CAN_PACKET_SIZE))
  {
    return HAL_ERROR;
  }

  head = usb_can_tx_head;
  next = (uint16_t)((head + 1U) & USB_CAN_TX_QUEUE_MASK);
  if (next == usb_can_tx_tail)
  {
    /* 可靠队列满时明确返回 BUSY，不覆盖尚未发送的数据。 */
    usb_can_tx_drop_count++;
    return HAL_BUSY;
  }

  usb_can_tx_queue[head].len = len;
  for (i = 0U; i < len; i++)
  {
    usb_can_tx_queue[head].data[i] = data[i];
  }
  __DMB();
  usb_can_tx_head = next;
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
    CanGateway_RxFeed(&byte, 1U);
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
  uint16_t tail;
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

  if (usb_can_tx_tail == usb_can_tx_head)
  {
    return;
  }

  tail = usb_can_tx_tail;
  /* 先标记 busy，再调用底层，避免极短传输完成回调抢先到达。 */
  usb_can_tx_start_tick = HAL_GetTick();
  __DMB();
  usb_can_tx_busy = 1U;
  result = CDC_Transmit_FS(usb_can_tx_queue[tail].data,
                           usb_can_tx_queue[tail].len);
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

  usb_can_tx_tail = (uint16_t)((usb_can_tx_tail + 1U) &
                               USB_CAN_TX_QUEUE_MASK);
  __DMB();
  usb_can_tx_busy = 0U;
  usb_can_tx_start_tick = 0U;
}

void UsbCanGateway_OnConfigured(void)
{
  /* 主机重新枚举后，上一条 USB IN 传输不会再收到完成回调。 */
  usb_can_tx_busy = 0U;
  usb_can_tx_start_tick = 0U;
  usb_can_rx_paused = 0U;
}

void UsbCanGateway_OnDeconfigured(void)
{
  /*
   * 断开/复位时 CDC 不一定为当前包回调完成事件。只释放本层 busy，
   * 当前队列尾不前移，重新枚举后仍会重试同一个包。
   */
  usb_can_tx_busy = 0U;
  usb_can_tx_start_tick = 0U;
  usb_can_rx_paused = 0U;
}

void UsbCanGateway_Process(void)
{
  UsbCanGateway_ProcessRx();
  UsbCanGateway_ProcessTx();
}
