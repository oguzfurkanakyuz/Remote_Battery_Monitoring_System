/*
 * esp8266.h
 *
 *  Created on: 11 Şub 2021
 *      Author: tolga
 */

#ifndef SRC_ESP8266_H_
#define SRC_ESP8266_H_

#include "stm32f4xx_hal.h"
#include <stdio.h>
//#include "string.h"
//#include <string.h>

#define tolga

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
//extern ADC_HandleTypeDef hadc1;

#define wifi_uart &huart3
#define pc_uart   &huart2



extern uint8_t wifi_rx_gecici_buffer;
extern uint8_t pc_rx_gecici_buffer[100];
extern char wifi_rx_buffer[500];
extern uint8_t tolga11;
extern uint8_t tolga22;
extern uint32_t potdegeri;


//void ESP8266_Init(void);
void ESP8266_Init(char *ag_adi,char *parola);
void ESP_Send_Server_Multi (char *APIkey, int numberoffileds, float value[]);

void bufclr (char *buf);

void USART_puts(const char *s, UART_HandleTypeDef *uart);
void Uart_write_kod(int c, UART_HandleTypeDef *uart);
void Clear_ESPBuffer(void);
void Clear_TC_Flag(void);





#endif /* SRC_ESP8266_H_ */
