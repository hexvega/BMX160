#ifndef __BLUETOOTH_H__
#define __BLUETOOTH_H__

#include <main.h>


#define BLE240_RESET()		HAL_GPIO_WritePin(bleRST_GPIO_Port, bleRST_Pin, GPIO_PIN_RESET)
#define BLE240_WORK()			HAL_GPIO_WritePin(bleRST_GPIO_Port, bleRST_Pin, GPIO_PIN_SET)
#define BLE240_WAKEUP()			HAL_GPIO_WritePin(bleWAKEUP_GPIO_Port, bleWAKEUP_Pin, GPIO_PIN_RESET)	//WAKEUP拉高省电，拉低通信。不影响蓝牙连接
#define BLE240_WAKEOFF()		HAL_GPIO_WritePin(bleWAKEUP_GPIO_Port, bleWAKEUP_Pin, GPIO_PIN_SET)		//WAKEUP拉高省电，拉低通信。不影响蓝牙连接
#define BLE240_WAKE_IS_ACTV			(HAL_GPIO_ReadPin(bleWAKEUP_GPIO_Port, bleWAKEUP_Pin)==GPIO_PIN_RESET)
#define BLE240_WAKE_IS_IDLE			(HAL_GPIO_ReadPin(bleWAKEUP_GPIO_Port, bleWAKEUP_Pin)==GPIO_PIN_SET)
#define BLE240_INT_IS_ACTV			(HAL_GPIO_ReadPin(bleINT_GPIO_Port, bleINT_Pin)==GPIO_PIN_RESET)
#define BLE240_INT_IS_IDLE			(HAL_GPIO_ReadPin(bleINT_GPIO_Port, bleINT_Pin)==GPIO_PIN_SET)
#define BLE_IS_CONNECTED	(HAL_GPIO_ReadPin(bleState_GPIO_Port, bleState_Pin)==GPIO_PIN_RESET)
#define BLE_IS_BREAK			(HAL_GPIO_ReadPin(bleState_GPIO_Port, bleState_Pin)==GPIO_PIN_SET)

#define BLE_FSM_PREP		0x50
#define BLE_FSM_WAIT1		0x51
#define BLE_FSM_UPDT		0x52
#define BLE_FSM_WAIT2		0x54


extern void WLL_BLE240_Sleep(void);
extern void WLL_BLE240_Init(void);
extern void WLL_BLE240_Uart1_Tx_DMA(uint8_t *TX_BUF, uint16_t Size);
extern void WLL_BLE240_Uart1_Rx(uint8_t *RX_BUF, uint16_t Size, uint32_t Timeout);
extern void WLL_BLE240_Uart1_Rx_IT(uint8_t *RX_BUF, uint16_t Size);
extern void WLL_BLE240_Buffer_Reset(uint8_t *BUF, uint16_t Size);
#endif
