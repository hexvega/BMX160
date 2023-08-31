#include <bluetooth.h>
#include "usart.h"
#include "gpio.h"

//--------------BLE--------------//
void WLL_BLE240_Sleep(void)
{
	BLE240_RESET();
	HAL_Delay(66);
	BLE240_WORK();
	BLE240_WAKEUP();	
	HAL_Delay(66);
	BLE240_WAKEOFF();	
}

void WLL_BLE240_Init(void)
{
	BLE240_WORK();
	HAL_Delay(1);
	BLE240_WAKEOFF();	
}


void WLL_BLE240_Uart1_Tx_DMA(uint8_t *TX_BUF, uint16_t Size)
{
	uint16_t cnt = 0;
	BLE240_WAKEUP();	
	while( BLE240_INT_IS_IDLE && cnt++<0x0fff ) {;}
	if(BLE240_INT_IS_IDLE) {return;}
	HAL_UART_Transmit_DMA(&huart1, TX_BUF, Size);
}

void WLL_BLE240_Uart1_Rx(uint8_t *RX_BUF, uint16_t Size, uint32_t Timeout)
{
	uint16_t cnt = 0;
	if(BLE240_INT_IS_ACTV) {
		BLE240_WAKEUP();	
		HAL_UART_Receive(&huart1, RX_BUF, Size, Timeout);
		while( BLE240_INT_IS_ACTV && cnt++<0x0fff ) {;}//wait for INT turns HIGH, 0x0fff=0.8ms
		BLE240_WAKEOFF();
	}
}

void WLL_BLE240_Uart1_Rx_IT(uint8_t *RX_BUF, uint16_t Size)
{
	if(BLE240_INT_IS_ACTV && BLE240_WAKE_IS_IDLE) {
		BLE240_WAKEUP();	
		HAL_UART_Receive_IT(&huart1, RX_BUF, Size);
	}
	if(BLE240_INT_IS_IDLE && BLE240_WAKE_IS_ACTV) {
		BLE240_WAKEOFF();	
	}
}


void WLL_BLE240_Buffer_Reset(uint8_t *BUF, uint16_t Size)
{
	uint8_t i;
	for(i=0; i<Size; i++) {
		BUF[i] = 0;
	}
}
