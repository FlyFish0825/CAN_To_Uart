/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @version        : v1.0_Cube
  * @brief          : Usb device for Virtual Com Port.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"

/* USER CODE BEGIN INCLUDE */
/* CDC 回调只依赖 USB 网关的队列接口，不直接解析 AA55/AA59 协议。 */
#include "usb_can_gateway.h"

/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
/* USB 网关状态由 usb_can_gateway.c 私有保存，本文件不重复维护队列。 */
/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device library.
  * @{
  */

/** @addtogroup USBD_CDC_IF
  * @{
  */

/** @defgroup USBD_CDC_IF_Private_TypesDefinitions USBD_CDC_IF_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */
/* 当前 CDC 适配层不新增私有类型。 */
/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Defines USBD_CDC_IF_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */
/* 当前 CDC 适配层不新增私有宏；缓冲区和水位常量由网关头文件提供。 */
/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Macros USBD_CDC_IF_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */
/* 当前 CDC 适配层不定义宏。 */
/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Variables USBD_CDC_IF_Private_Variables
  * @brief Private variables.
  * @{
  */
/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/** Received data over USB are stored in this buffer      */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */
/* CubeMX 提供的 UserRxBufferFS/UserTxBufferFS 仍由 CDC 模板管理。 */
/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */
/* 当前没有额外导出的 CDC 变量。 */
/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_FunctionPrototypes USBD_CDC_IF_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);
static int8_t CDC_TransmitCplt_FS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */
/* 自定义 CDC 辅助函数原型由本文件后部的用户实现区提供。 */
/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS,
  CDC_TransmitCplt_FS
};

/* Private functions ---------------------------------------------------------*/
/*
 * CDC OUT 端点接收状态跟踪：0 表示端点未提交接收（主机写入会被 NAK）。
 * 该标志与 usb_can_gateway 的 paused 标志配合：paused=1 表示主动反压，
 * armed=0 则覆盖"端点意外失去提交且 paused 已被配置事件清零"的死角，
 * 让主循环的恢复逻辑能在两种情况下都重新提交 OUT 接收。
 */
static volatile uint8_t cdc_rx_armed = 0U;

uint8_t CDC_IsRxArmed(void)
{
  return cdc_rx_armed;
}

/**
  * @brief  Initializes the CDC media low layer over the FS USB IP
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_FS(void)
{
  /* USER CODE BEGIN 3 */
  /* 把 CubeMX 缓冲区交给 USB CDC 类，并清除网关残留发送状态。 */
  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  UsbCanGateway_OnConfigured();
  /* USBD_CDC_Init 在类初始化末尾通过 PrepareReceive 提交 OUT 接收。 */
  cdc_rx_armed = 1U;
  return (USBD_OK);
  /* USER CODE END 3 */
}

/**
  * @brief  DeInitializes the CDC media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_FS(void)
{
  /* USER CODE BEGIN 4 */
  /* USB 反初始化时保留网关队列数据，只清理当前 CDC 发送状态。 */
  UsbCanGateway_OnDeconfigured();
  cdc_rx_armed = 0U;
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  Manage the CDC class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  /* USER CODE BEGIN 5 */
  /* 控制请求由 CDC 类模板处理；本网关不在控制请求中承载业务协议。 */
  switch(cmd)
  {
    case CDC_SEND_ENCAPSULATED_COMMAND:

    break;

    case CDC_GET_ENCAPSULATED_RESPONSE:

    break;

    case CDC_SET_COMM_FEATURE:

    break;

    case CDC_GET_COMM_FEATURE:

    break;

    case CDC_CLEAR_COMM_FEATURE:

    break;

  /*******************************************************************************/
  /* Line Coding Structure                                                       */
  /*-----------------------------------------------------------------------------*/
  /* Offset | Field       | Size | Value  | Description                          */
  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
  /*                                        0 - 1 Stop bit                       */
  /*                                        1 - 1.5 Stop bits                    */
  /*                                        2 - 2 Stop bits                      */
  /* 5      | bParityType |  1   | Number | Parity                               */
  /*                                        0 - None                             */
  /*                                        1 - Odd                              */
  /*                                        2 - Even                             */
  /*                                        3 - Mark                             */
  /*                                        4 - Space                            */
  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
  /*******************************************************************************/
    case CDC_SET_LINE_CODING:

    break;

    case CDC_GET_LINE_CODING:

    break;

    case CDC_SET_CONTROL_LINE_STATE:

    break;

    case CDC_SEND_BREAK:

    break;

  default:
    break;
  }

  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will issue a NAK packet on any OUT packet received on
  *         USB endpoint until exiting this function. If you exit this function
  *         before transfer is complete on CDC interface (ie. using DMA controller)
  *         it will result in receiving more data while previous ones are still
  *         not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
  /* Buf/Len 属于 USB OUT 回调上下文，必须在本次回调内完成必要的入队。 */
  /* USB OUT 回调只搬字节到 8 KB 环形缓冲，不在中断中解析 AA55 协议。 */
  cdc_rx_armed = 0U; /* 当前包已到达并消费，OUT 端点暂处于未提交状态。 */
  if ((Buf != NULL) && (Len != NULL) && (*Len <= UINT16_MAX))
  {
    /* OUT 已在上一轮通过水位检查后提交；当前包到达后只写入 RX 队列。 */
    UsbCanGateway_RxPush(Buf, (uint16_t)*Len);
  }

  /*
   * 缓冲空间不足时不重新提交 OUT 接收，USB 栈会对主机返回 NAK，形成
   * 反压而不是丢弃后续字节。空间恢复后由主循环调用 ResumeReceive。
   */
  if (UsbCanGateway_RxCanRearm(USB_CAN_RX_PACKET_RESERVE) != 0U)
  {
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, Buf);
    if (USBD_CDC_ReceivePacket(&hUsbDeviceFS) == USBD_OK)
    {
      cdc_rx_armed = 1U;
    }
  }
  else
  {
    UsbCanGateway_RxMarkPaused();
  }
  return (USBD_OK);
  /* USER CODE END 6 */
}

/**
  * @brief  CDC_Transmit_FS
  *         Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  *         @note
  *
  *
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 7 */
  /* Buf/Len 是当前待发送包；本函数只提交给 CDC，不等待传输完成。 */
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;

  /* 电脑尚未完成 USB 枚举/配置时，CDC 类实例可能还没有创建。
   * 此时直接访问 hcdc->TxState 会产生空指针异常，因此统一返回 BUSY，
   * 让上层稍后再试；本测试不会在这里阻塞等待。 */
  if ((hcdc == NULL) || (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED))
  {
    return USBD_BUSY;
  }

  if (hcdc->TxState != 0U)
  {
    return USBD_BUSY;
  }
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  /* USER CODE END 7 */
  return result;
}

/**
  * @brief  CDC_TransmitCplt_FS
  *         Data transmitted callback
  *
  *         @note
  *         This function is IN transfer complete callback used to inform user that
  *         the submitted Data is successfully sent over USB.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 13 */
  /* 回调参数由 USB 栈提供；网关只需要完成通知推进 TX 队列。 */
  UNUSED(Buf);
  UNUSED(Len);
  UNUSED(epnum);
  /* 回调保持极轻，只推进 USB 可靠发送队列。 */
  UsbCanGateway_TxComplete();
  /* USER CODE END 13 */
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @brief 清除 CDC IN 传输状态，供发送队列超时恢复使用。
  *
  * 正常路径不会调用此函数。它只在发送完成回调异常丢失超过超时时间时
  * 执行，先刷新 IN 端点，再清零 CDC 类状态，使队列可以重新提交当前包。
  */
void CDC_ResetTransmitState_FS(void)
{
  USBD_CDC_HandleTypeDef *hcdc =
      (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;

  /* 超时恢复只清除 CDC 类的 busy 状态，不删除网关中尚未发送的数据。 */
  if ((hcdc == NULL) ||
      (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED))
  {
    return;
  }

  (void)USBD_LL_FlushEP(&hUsbDeviceFS, CDC_IN_EP);
  hcdc->TxState = 0U;
}

/**
  * @brief 重新提交被反压暂停的 CDC OUT 接收。
  * @retval USBD_OK 已提交；其他值表示暂时不能提交。
  */
uint8_t CDC_ResumeReceive_FS(void)
{
  USBD_CDC_HandleTypeDef *hcdc =
      (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;

  /* 仅在设备已配置且 CDC 类实例有效时重新提交 OUT 接收。 */
  if ((hcdc == NULL) ||
      (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED))
  {
    return USBD_BUSY;
  }

  {
    uint8_t result = USBD_CDC_ReceivePacket(&hUsbDeviceFS);
    if (result == USBD_OK)
    {
      cdc_rx_armed = 1U;
    }
    return result;
  }
}

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */
