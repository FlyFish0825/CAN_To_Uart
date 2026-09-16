#include "usb_can_gateway.h"

#include "can_uart_gateway.h"
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

void UsbCanGateway_RxPush(const uint8_t *data, uint16_t len)
{
  uint16_t i;

  if ((data == NULL) || (len == 0U))
  {
    return;
  }

  /* 此函数在 USB OUT 回调中运行：只搬字节，不解析协议、不等待发送。 */
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
  /* USB 是字节流：现有 AA55 解析器负责半包、粘包和错误后重新同步。 */
  while (usb_can_rx_tail != usb_can_rx_head)
  {
    uint16_t tail = usb_can_rx_tail;
    uint8_t byte;

    __DMB();
    byte = usb_can_rx_ring[tail];
    usb_can_rx_tail = (uint16_t)((tail + 1U) & USB_CAN_RX_RING_MASK);
    CanUartGateway_ProtocolFeed(&byte, 1U);
  }
}

static void UsbCanGateway_ProcessTx(void)
{
  uint16_t tail;
  uint8_t result;

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
