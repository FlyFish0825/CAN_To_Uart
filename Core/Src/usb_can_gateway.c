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

static UsbCanTxPacket_t usb_can_tx_queue[USB_CAN_TX_QUEUE_SIZE];
static volatile uint16_t usb_can_tx_head = 0U;
static volatile uint16_t usb_can_tx_tail = 0U;
static volatile uint8_t usb_can_tx_busy = 0U;

static volatile uint32_t usb_can_rx_drop_count = 0U;
static volatile uint32_t usb_can_tx_drop_count = 0U;
static volatile uint32_t usb_can_tx_error_count = 0U;

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

static const CanGatewayTransportOps_t usb_can_transport_ops =
{
  UsbCanGateway_SendAdapter,
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
  /*
   * USB 是字节流：一个 CDC 回调不一定等于一帧协议包。这里逐字节取出，
   * 由 AA55 解析器负责处理半包、粘包、非法长度以及错误后的重新同步。
   * 每取出一个字节就推进 tail，保证即使解析器发现错误也不会卡住队列。
   */
  while (usb_can_rx_tail != usb_can_rx_head)
  {
    uint16_t tail = usb_can_rx_tail;
    uint8_t byte;

    __DMB();
    byte = usb_can_rx_ring[tail];
    usb_can_rx_tail = (uint16_t)((tail + 1U) & USB_CAN_RX_RING_MASK);
    CanGateway_RxFeed(&byte, 1U);
  }
}

static void UsbCanGateway_ProcessTx(void)
{
  uint16_t tail;
  uint8_t result;

  /*
   * USB CDC 同一时刻只允许一个 IN 传输。busy 表示队列尾槽已交给 USB
   * 驱动，必须等待 CDC_TransmitCplt_FS() 后才能释放该槽位。
   */
  if ((usb_can_tx_busy != 0U) ||
      (usb_can_tx_tail == usb_can_tx_head))
  {
    return;
  }

  tail = usb_can_tx_tail;
  result = CDC_Transmit_FS(usb_can_tx_queue[tail].data,
                           usb_can_tx_queue[tail].len);
  if (result == USBD_OK)
  {
    /* 当前队列槽位直到 CDC_TransmitCplt_FS() 回调前都不能复用。 */
    __DMB();
    usb_can_tx_busy = 1U;
  }
  else if (result != USBD_BUSY)
  {
    usb_can_tx_error_count++;
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
}

void UsbCanGateway_OnConfigured(void)
{
  /* 主机重新枚举后，上一条 USB IN 传输不会再收到完成回调。 */
  usb_can_tx_busy = 0U;
}

void UsbCanGateway_Process(void)
{
  UsbCanGateway_ProcessRx();
  UsbCanGateway_ProcessTx();
}
