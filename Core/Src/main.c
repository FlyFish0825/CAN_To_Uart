/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "fdcan.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  uint32_t id;
  uint8_t flags;
  uint8_t len;
  uint8_t data[64];
} CanRxFrame_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAN_RX_QUEUE_SIZE 64U
#define CAN_RX_QUEUE_MASK (CAN_RX_QUEUE_SIZE - 1U)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
FDCAN_TxHeaderTypeDef tx_header;
uint8_t tx_data[64];
static uint16_t uart_sequence = 0U;
static CanRxFrame_t can_rx_queue[CAN_RX_QUEUE_SIZE];
static volatile uint16_t can_rx_head = 0U;
static volatile uint16_t can_rx_tail = 0U;
static volatile uint32_t can_rx_drop_count = 0U;
static volatile uint32_t can_tx_fail_count = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
static void Crc8Hw_Init(void);
static uint8_t Crc8AtmHw(const uint8_t *data, uint16_t len);
static uint8_t CanDlcToLength(uint32_t dlc);
static void CanRx_ProcessUart(void);
static void Uart_SendCanPacket(uint32_t can_id, uint8_t flags,
                               uint8_t dlc, const uint8_t *data);
static void Test_SendOnce(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
  uint16_t i = 0U;
  uint16_t value16;
  __IO uint16_t *dr16;

  SET_BIT(CRC->CR, CRC_CR_RESET);
  while ((uint16_t)(len - i) >= 4U)
  {
    CRC->DR = ((uint32_t)data[i] << 24U) | ((uint32_t)data[i + 1U] << 16U) |
              ((uint32_t)data[i + 2U] << 8U) | (uint32_t)data[i + 3U];
    i += 4U;
  }
  if ((uint16_t)(len - i) == 1U)
  {
    *(__IO uint8_t *)(__IO void *)&CRC->DR = data[i];
  }
  else if ((uint16_t)(len - i) == 2U)
  {
    value16 = ((uint16_t)data[i] << 8U) | (uint16_t)data[i + 1U];
    dr16 = (__IO uint16_t *)(__IO void *)&CRC->DR;
    *dr16 = value16;
  }
  else if ((uint16_t)(len - i) == 3U)
  {
    value16 = ((uint16_t)data[i] << 8U) | (uint16_t)data[i + 1U];
    dr16 = (__IO uint16_t *)(__IO void *)&CRC->DR;
    *dr16 = value16;
    *(__IO uint8_t *)(__IO void *)&CRC->DR = data[i + 2U];
  }
  return (uint8_t)(CRC->DR & 0xFFU);
}

static uint8_t CanDlcToLength(uint32_t dlc)
{
  switch (dlc)
  {
    case FDCAN_DLC_BYTES_0: return 0U;
    case FDCAN_DLC_BYTES_1: return 1U;
    case FDCAN_DLC_BYTES_2: return 2U;
    case FDCAN_DLC_BYTES_3: return 3U;
    case FDCAN_DLC_BYTES_4: return 4U;
    case FDCAN_DLC_BYTES_5: return 5U;
    case FDCAN_DLC_BYTES_6: return 6U;
    case FDCAN_DLC_BYTES_7: return 7U;
    case FDCAN_DLC_BYTES_8: return 8U;
    case FDCAN_DLC_BYTES_12: return 12U;
    case FDCAN_DLC_BYTES_16: return 16U;
    case FDCAN_DLC_BYTES_20: return 20U;
    case FDCAN_DLC_BYTES_24: return 24U;
    case FDCAN_DLC_BYTES_32: return 32U;
    case FDCAN_DLC_BYTES_48: return 48U;
    case FDCAN_DLC_BYTES_64: return 64U;
    default: return 0U;
  }
}

static void Uart_SendCanPacket(uint32_t can_id, uint8_t flags,
                               uint8_t dlc, const uint8_t *data)
{
  uint8_t packet[76];
  uint16_t i;
  packet[0] = 0xAAU;
  packet[1] = 0x55U;
  packet[2] = 72U;
  packet[3] = (uint8_t)(uart_sequence & 0xFFU);
  packet[4] = (uint8_t)(uart_sequence >> 8U);
  packet[5] = (uint8_t)(can_id & 0xFFU);
  packet[6] = (uint8_t)((can_id >> 8U) & 0xFFU);
  packet[7] = (uint8_t)((can_id >> 16U) & 0xFFU);
  packet[8] = (uint8_t)((can_id >> 24U) & 0xFFU);
  packet[9] = flags;
  packet[10] = dlc;
  for (i = 0U; i < 64U; i++) packet[11U + i] = data[i];
  packet[75] = Crc8AtmHw(&packet[2], 73U);
  (void)HAL_UART_Transmit(&huart1, packet, sizeof(packet), 100U);
  uart_sequence++;
}

static void CanRx_ProcessUart(void)
{
  while (can_rx_tail != can_rx_head)
  {
    uint16_t tail = can_rx_tail;
    CanRxFrame_t frame = can_rx_queue[tail];
    __DMB();
    can_rx_tail = (uint16_t)((tail + 1U) & CAN_RX_QUEUE_MASK);
    Uart_SendCanPacket(frame.id, frame.flags, frame.len, frame.data);
  }
}

static void Test_SendOnce(void)
{
  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, tx_data) != HAL_OK)
  {
    can_tx_fail_count++;
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FDCAN1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  Crc8Hw_Init();
  for (uint32_t i = 0U; i < 64U; i++)
  {
    tx_data[i] = (uint8_t)i;
  }
  tx_header.Identifier = 0x123U;
  tx_header.IdType = FDCAN_STANDARD_ID;
  tx_header.TxFrameType = FDCAN_DATA_FRAME;
  tx_header.DataLength = FDCAN_DLC_BYTES_64;
  tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx_header.BitRateSwitch = FDCAN_BRS_ON;
  tx_header.FDFormat = FDCAN_FD_CAN;
  tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tx_header.MessageMarker = 0U;

  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                   FDCAN_ACCEPT_IN_RX_FIFO0,
                                   FDCAN_ACCEPT_IN_RX_FIFO0,
                                   FDCAN_FILTER_REMOTE,
                                   FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_FDCAN_ActivateNotification(&hfdcan1,
                                     FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                     0U) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  Test_SendOnce();
/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    static uint32_t last_test_ms = 0U;
    CanRx_ProcessUart();
    if ((uint32_t)(HAL_GetTick() - last_test_ms) >= 1000U)
    {
      last_test_ms = HAL_GetTick();
      Test_SendOnce();
    }
/* USER CODE END 3 */
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 12;
  RCC_OscInitStruct.PLL.PLLR = 4;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if ((hfdcan == NULL) || (hfdcan->Instance != FDCAN1) ||
      ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U))
  {
    return;
  }

  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U)
  {
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[64];
    uint8_t len;
    uint8_t flags = 0U;
    uint16_t head;
    uint16_t next;
    uint16_t i;
    CanRxFrame_t *frame;

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
    {
      break;
    }
    len = CanDlcToLength(rx_header.DataLength);
    if (rx_header.IdType == FDCAN_EXTENDED_ID) flags |= 0x01U;
    if (rx_header.FDFormat == FDCAN_FD_CAN) flags |= 0x02U;
    if (rx_header.BitRateSwitch == FDCAN_BRS_ON) flags |= 0x04U;
    if (rx_header.RxFrameType == FDCAN_REMOTE_FRAME) flags |= 0x08U;
    head = can_rx_head;
    next = (uint16_t)((head + 1U) & CAN_RX_QUEUE_MASK);
    if (next == can_rx_tail)
    {
      can_rx_drop_count++;
      continue;
    }

    frame = &can_rx_queue[head];
    frame->id = rx_header.Identifier;
    frame->flags = flags;
    frame->len = len;
    for (i = 0U; i < 64U; i++)
    {
      if ((rx_header.RxFrameType == FDCAN_DATA_FRAME) && (i < len))
      {
        frame->data[i] = rx_data[i];
      }
      else
      {
        frame->data[i] = 0U;
      }
    }
    __DMB();
    can_rx_head = next;
  }
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
