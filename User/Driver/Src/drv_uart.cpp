/**
 * @file drv_uart.cpp
 * @author cjw by yssickjgd
 * @brief UART通信初始化与配置流程
 * @version 0.1
 * @date 2025-07-1 0.1 26赛季定稿
 *
 * @copyright ZLLC 2026
 * 
 */

/* Includes ------------------------------------------------------------------*/

#include "drv_uart.h"
#include "drv_rs485.h"
// #include "config.h"
#include "string.h"
#include "dvc_dwt.h"
#include <stdint.h>
/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

Struct_UART_Manage_Object UART1_Manage_Object = {0};
Struct_UART_Manage_Object UART2_Manage_Object = {0};
Struct_UART_Manage_Object UART3_Manage_Object = {0};
Struct_UART_Manage_Object UART4_Manage_Object = {0};
Struct_UART_Manage_Object UART5_Manage_Object = {0};
Struct_UART_Manage_Object UART6_Manage_Object = {0};
Struct_UART_Manage_Object UART7_Manage_Object = {0};
Struct_UART_Manage_Object UART8_Manage_Object = {0};
Struct_UART_Manage_Object UART9_Manage_Object = {0};
Struct_UART_Manage_Object UART10_Manage_Object = {0};

volatile uint32_t rs485_rx_event_count = 0;
volatile uint32_t rs485_rx_total_bytes = 0;
volatile uint32_t rs485_rx_oversize_count = 0;
volatile uint32_t rs485_rx_restart_fail_count = 0;
volatile uint32_t rs485_rx_restart_busy_count = 0;
volatile uint16_t rs485_rx_last_size = 0;

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

static void USART_RxDMA_MultiBufferStart(UART_HandleTypeDef *huart, uint32_t *SrcAddress, uint32_t *DstAddress, uint32_t *SecondMemAddress, uint32_t DataLength){


 huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;

 huart->RxEventType = HAL_UART_RXEVENT_TC;

 huart->RxXferSize    = DataLength;

 SET_BIT(huart->Instance->CR3,USART_CR3_DMAR);

 __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE); 

 HAL_DMAEx_MultiBufferStart(&hdma_uart5_rx,(uint32_t)SrcAddress,(uint32_t)DstAddress,(uint32_t)SecondMemAddress,DataLength);
	

}
uint8_t Power_receive_data[32];
float temp_power = 0;
/**
 * @brief 初始化UART
 *
 * @param huart UART编号
 * @param Callback_Function 处理回调函数
 */
void UART_Init(UART_HandleTypeDef *huart, UART_Call_Back Callback_Function, uint16_t Rx_Buffer_Length)
{
    if (huart->Instance == USART1)
    {
        UART1_Manage_Object.UART_Handler = huart;
        UART1_Manage_Object.Callback_Function = Callback_Function;
        UART1_Manage_Object.Rx_Buffer_Length = Rx_Buffer_Length;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART1_Manage_Object.Rx_Buffer, UART1_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == UART5)
    {
        UART5_Manage_Object.UART_Handler = huart;
        UART5_Manage_Object.Callback_Function = Callback_Function;
        UART5_Manage_Object.Rx_Buffer_Length = Rx_Buffer_Length;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART5_Manage_Object.Rx_Buffer, UART5_Manage_Object.Rx_Buffer_Length*2);
			  __HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);
    }
    else if (huart->Instance == UART7)
    {
        UART7_Manage_Object.UART_Handler = huart;
        UART7_Manage_Object.Callback_Function = Callback_Function;
        UART7_Manage_Object.Rx_Buffer_Length = Rx_Buffer_Length;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART7_Manage_Object.Rx_Buffer, UART7_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == UART8)
    {
        UART8_Manage_Object.UART_Handler = huart;
        UART8_Manage_Object.Callback_Function = Callback_Function;
        UART8_Manage_Object.Rx_Buffer_Length = Rx_Buffer_Length;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART8_Manage_Object.Rx_Buffer, UART8_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == UART9)
    {
        UART9_Manage_Object.UART_Handler = huart;
        UART9_Manage_Object.Callback_Function = Callback_Function;
        UART9_Manage_Object.Rx_Buffer_Length = Rx_Buffer_Length;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART9_Manage_Object.Rx_Buffer, UART9_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == USART10)
    {
        UART10_Manage_Object.UART_Handler = huart;
        UART10_Manage_Object.Callback_Function = Callback_Function;
        UART10_Manage_Object.Rx_Buffer_Length = Rx_Buffer_Length;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART10_Manage_Object.Rx_Buffer, UART10_Manage_Object.Rx_Buffer_Length);
    }
    
}

/**
 * @brief 发送数据帧
 *
 * @param huart UART编号
 * @param Data 被发送的数据指针
 * @param Length 长度
 * @return uint8_t 执行状态
 */
uint8_t UART_Send_Data(UART_HandleTypeDef *huart, uint8_t *Data, uint16_t Length)
{
    return (HAL_UART_Transmit_DMA(huart, Data, Length));
}

/**
 * @brief UART的TIM定时器中断发送回调函数
 *
 */
void TIM_UART_PeriodElapsedCallback()
{
    //上位机通讯
    UART_Send_Data(&huart8, UART8_Manage_Object.Tx_Buffer, 56);
}

/**
 * @brief HAL库UART接收DMA空闲中断
 *
 * @param huart UART编号
 * @param Size 长度
 */
//int tttt = 0;
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{    
    //停止DMA接收 保护处理过程
    //HAL_UART_DMAStop(huart);
    
    //选择回调函数
    if (huart->Instance == USART1)
    {
        UART1_Manage_Object.Rx_Length = Size;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART1_Manage_Object.Rx_Buffer, UART1_Manage_Object.Rx_Buffer_Length*2);
        if( UART1_Manage_Object.Rx_Length<=UART1_Manage_Object.Rx_Buffer_Length)
            UART1_Manage_Object.Callback_Function(UART1_Manage_Object.Rx_Buffer, Size);
        else
        memset( UART1_Manage_Object.Rx_Buffer, 0, UART1_Manage_Object.Rx_Buffer_Length);
		//tttt++;
    }
    else if (huart->Instance == USART2) 
    {
        uint16_t rx_size = Size;
        if (rx_size > RS485_RX_SIZE)
        {
            rs485_rx_oversize_count++;
            rx_size = RS485_RX_SIZE;
        }

        rs485_rx_event_count++;
        rs485_rx_last_size = rx_size;
        rs485_rx_total_bytes += rx_size;

        // H7 D-Cache 失效地址需要按 cache line(32B)对齐
        uintptr_t addr = (uintptr_t)rs485_rx_buf;
        uintptr_t aligned_addr = addr & ~((uintptr_t)31);
        uintptr_t aligned_end = (addr + rx_size + 31u) & ~((uintptr_t)31);
        int32_t aligned_size = (int32_t)(aligned_end - aligned_addr);
        SCB_InvalidateDCache_by_Addr((uint32_t *)aligned_addr, aligned_size);

        RS485_Receive_Handler(rs485_rx_buf, rx_size);

        // USART2 当前是 DMA_CIRCULAR，回调中通常无需重启接收。
        // 若强行重启，常见返回 HAL_BUSY(与 RxState=BUSY_RX 对应)，并非真正故障。
        if (huart2.hdmarx != NULL && huart2.hdmarx->Init.Mode != DMA_CIRCULAR)
        {
            HAL_StatusTypeDef restart_status = HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rs485_rx_buf, RS485_RX_SIZE);
            if (restart_status == HAL_BUSY)
            {
                rs485_rx_restart_busy_count++;
            }
            else if (restart_status != HAL_OK)
            {
                rs485_rx_restart_fail_count++;
            }
        }
        __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
    }
    else if (huart->Instance == UART5)
    {
			  
        UART5_Manage_Object.Rx_Length = Size;              
        if( UART5_Manage_Object.Rx_Length<=UART5_Manage_Object.Rx_Buffer_Length)
            UART5_Manage_Object.Callback_Function(UART5_Manage_Object.Rx_Buffer, Size);
				HAL_UARTEx_ReceiveToIdle_DMA(huart, UART5_Manage_Object.Rx_Buffer, UART5_Manage_Object.Rx_Buffer_Length*2);
				__HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);
//        else
//        memset( UART5_Manage_Object.Rx_Buffer, 0, UART5_Manage_Object.Rx_Buffer_Length);
			  
    }
    else if (huart->Instance == UART7)
    {
        UART7_Manage_Object.Rx_Length = Size;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART7_Manage_Object.Rx_Buffer, UART7_Manage_Object.Rx_Buffer_Length*2);
        // if( UART7_Manage_Object.Rx_Length<=UART7_Manage_Object.Rx_Buffer_Length)
        //     UART7_Manage_Object.Callback_Function(UART7_Manage_Object.Rx_Buffer, Size);
        // else
        // memset( UART7_Manage_Object.Rx_Buffer, 0, UART7_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == UART8)
    {
        UART8_Manage_Object.Rx_Length = Size;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART8_Manage_Object.Rx_Buffer, UART8_Manage_Object.Rx_Buffer_Length);
        if( UART8_Manage_Object.Rx_Length<=UART8_Manage_Object.Rx_Buffer_Length)
            UART8_Manage_Object.Callback_Function(UART8_Manage_Object.Rx_Buffer, Size);
        // else
        // memset( UART8_Manage_Object.Rx_Buffer, 0, UART8_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == UART9)
    {
        UART9_Manage_Object.Rx_Length = Size;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART9_Manage_Object.Rx_Buffer, UART9_Manage_Object.Rx_Buffer_Length);
        if( UART9_Manage_Object.Rx_Length<=UART9_Manage_Object.Rx_Buffer_Length)
            UART9_Manage_Object.Callback_Function(UART9_Manage_Object.Rx_Buffer, Size);
        // else
        // memset( UART9_Manage_Object.Rx_Buffer, 0, UART9_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == USART10)
    {
        UART10_Manage_Object.Rx_Length = Size;        
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART10_Manage_Object.Rx_Buffer, UART10_Manage_Object.Rx_Buffer_Length);
        if( UART10_Manage_Object.Rx_Length<=UART10_Manage_Object.Rx_Buffer_Length)
            UART10_Manage_Object.Callback_Function(UART10_Manage_Object.Rx_Buffer, Size);
        // else
        // memset( UART10_Manage_Object.Rx_Buffer, 0, UART10_Manage_Object.Rx_Buffer_Length);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
//    if (huart->Instance == UART5)
//    {
//        UART5_Manage_Object.Rx_Length = 18;
//			  HAL_UART_Receive_DMA(huart, UART5_Manage_Object.Rx_Buffer, UART5_Manage_Object.Rx_Buffer_Length*2);
//        if( UART1_Manage_Object.Rx_Length<=UART1_Manage_Object.Rx_Buffer_Length)
//            UART1_Manage_Object.Callback_Function(UART5_Manage_Object.Rx_Buffer, 28);
//        else
//        memset( UART5_Manage_Object.Rx_Buffer, 0, UART5_Manage_Object.Rx_Buffer_Length);
//        
//    }
	   if (huart->Instance == USART1)
    {
        UART1_Manage_Object.Rx_Length = 32;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART1_Manage_Object.Rx_Buffer, UART1_Manage_Object.Rx_Buffer_Length*2);
        if( UART1_Manage_Object.Rx_Length<=UART1_Manage_Object.Rx_Buffer_Length)
           UART1_Manage_Object.Callback_Function(UART1_Manage_Object.Rx_Buffer, 32);
        else
        memset( UART1_Manage_Object.Rx_Buffer, 0, UART1_Manage_Object.Rx_Buffer_Length);
		}
}



/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
