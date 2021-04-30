/*
 * esp8266.c
 *
 *  Created on: 11 Şub 2021
 *      Author: tolga
 */


#include "esp8266.h"
//#include <string.h>
//#include "stdio.h"
#include "string.h"


#define ESP8266BUFFER_LENGHT 500
//uint8_t ESPEventCase=0;

char pc_rx_buffer[ESP8266BUFFER_LENGHT]={0};
char wifi_rx_buffer[ESP8266BUFFER_LENGHT]={0};
//char buffer_wifi[ESP8266BUFFER_LENGHT]={0};
uint8_t tx_gecici_ortak_buffer=0;
char  tx_gecici_ortak_buffer55[100]={0};


uint8_t TOLGA=0;
uint8_t tolga_dizi[500]={0};

uint8_t server_flag=0;

uint8_t k_counter=0;
uint16_t wifi_buffer_counter=0;
uint8_t i=0;
uint8_t len=0;
char data[80]={0};
char transmitdata[100];
char transmitconf[100];

uint8_t esp_run=0;
uint8_t server_case =0;

int length;





void ESP_Send_Server_Multi (char *APIkey, int numberoffileds, float value[])
{
	char local_buf[1000] = {0};
	char local_buf2[30] = {0};
	char field_buf[200] = {0};
	uint8_t Sendservercase=0;

	while(Sendservercase==0)
	{
		USART_puts("AT+CIPSTART=\"TCP\",\"184.106.153.149\",80\r\n",wifi_uart);//ip adresimi yazdım
		HAL_Delay(1000);
		USART_puts("TCP Baglanti istegi gonderildi\n\n", pc_uart);
		HAL_Delay(200);
		if (strstr(wifi_rx_buffer,"OK") != NULL)//veya connect yazılabilir
		{

			USART_puts("Site ile baglanti kuruldu\n\n", pc_uart);
			HAL_Delay(500);
			Sendservercase = 1;
//			Clear_ESPBuffer();
		}

		else if (strstr(wifi_rx_buffer,"ALREAY CONNECT") != NULL)//veya connect yazılabilir
		{

			USART_puts("Site ile baglanti kuruldu\n\n", pc_uart);
			HAL_Delay(500);
			Sendservercase = 1;
//			Clear_ESPBuffer();
		}


		else
		{
			USART_puts("Site baglantisi olmadi.Tekrar Dene.\n",pc_uart);
			HAL_Delay(500);
			Sendservercase = 0;
		}
		Clear_ESPBuffer();
	}

	sprintf (local_buf, "GET /update?api_key=%s", APIkey);//GET /update?api_key=%s in arkasına 4 veriyi
	for (int i=0; i<numberoffileds; i++)                  // ekliyoruz.
	{
		sprintf(field_buf, "&field%d=%.3f",i+1, value[i]);
		strcat (local_buf, field_buf);
	}

	strcat(local_buf, "\r\n");
	int len = strlen (local_buf);

	while(Sendservercase==1)// veri gönderiliyor
	{
		sprintf (local_buf2, "AT+CIPSEND=%d\r\n", len);// verinin boyutunu gönderiyoruz
		USART_puts(local_buf2,wifi_uart);
		HAL_Delay(1000);
		if (strstr(wifi_rx_buffer,">") != NULL)
		{

			USART_puts("Veri boyutu kabul edildi\n\n", pc_uart);
			HAL_Delay(500);
			Sendservercase = 2;
			Clear_ESPBuffer();
		}
		else
		{
			USART_puts("Veri boyutu kabul edilmedi\n\n",pc_uart);
			HAL_Delay(500);
			Sendservercase = 1;
		}
	}
	while(Sendservercase==2)//veriyi gönderiyoruz
	{
		USART_puts(local_buf,wifi_uart);
		HAL_Delay(500);
		if (strstr(wifi_rx_buffer,"SEND OK\r\n") != NULL)
		{

			USART_puts("Veri kabul edildi\n\n", pc_uart);
			HAL_Delay(500);
			Sendservercase = 3;
			//Clear_ESPBuffer();
		}
		else
		{
			USART_puts("Veri kabul edilmedi\n\n",pc_uart);
			HAL_Delay(500);
			Sendservercase = 2;
		}
	}
	while(Sendservercase==3)
	{
		if (strstr(wifi_rx_buffer,"OK") != NULL)
		{

			USART_puts("veri akisi kapandi\n\n", pc_uart);
			HAL_Delay(500);
			Sendservercase = 4;
			Clear_ESPBuffer();
		}
		else
		{
			USART_puts("Veri akisi kapanmadi\n\n",pc_uart);
			HAL_Delay(5000);
			Sendservercase = 3;
		}
	}
	bufclr(local_buf);
	bufclr(local_buf2);

}

void ESP8266_Init(char *ag_adi,char *parola)
{
	uint8_t ESPEventCase=0;
//	uint8_t say2=0;

	//		USART_puts("AT+RST\r\n", wifi_uart);
	//		HAL_Delay(1000);
	//		USART_puts("RESETTING.", pc_uart);
	//		for (int i=0; i<5; i++)
	//		{
	//			USART_puts(".", pc_uart);
	//			HAL_Delay(1000);
	//		}

	while(ESPEventCase==0)/*********  AT **********/
	{
		USART_puts("AT\r\n",wifi_uart);
		HAL_Delay(500);
		USART_puts("\nAT atildi\r\n",pc_uart);
		HAL_Delay(500);

		if (strstr(wifi_rx_buffer,"OK") != NULL)
		{

			USART_puts("Module Erisildi\n",pc_uart);
			ESPEventCase = 1;

		}
		else
		{
			USART_puts("Modul Bulunamadi, Tekrar Deneniyor\n",pc_uart);
			ESPEventCase =0 ;
		}
		Clear_ESPBuffer();
	}
	while(ESPEventCase==1) /********* AT+CWMODE=1 **********/
	{
		USART_puts("AT+CWMODE?\r\n",wifi_uart);//hangi modda diye sorduk	                                         // 1 saniye gecikme koyuyoruz.
		HAL_Delay(1000);

		if (strstr(wifi_rx_buffer,"+CWMODE:1") != NULL)
		{

			USART_puts("MODE Ayar Dogru\n",pc_uart); //1 'se application modda old. öğrendik
			ESPEventCase=2;
		}
		else
		{
			// Fabrika ayarlari olarak 2 geliyor biz onu 1 yapip reset komutu ile tamamlariz.
			USART_puts("AT+CWMODE=1\r\n",wifi_uart);
			HAL_Delay(1000);
			USART_puts("MOD Degistirildi.\n",pc_uart);
			ESPEventCase = 1;
		}
		Clear_ESPBuffer();
	}
	while(ESPEventCase==2) /********* AT+CWJAP="SSID","PASSWD" **********/
	{
		sprintf (data, "AT+CWJAP=\"%s\",\"%s\"\r\n", ag_adi, parola);
		USART_puts(data, wifi_uart);
		HAL_Delay(2000);

		if (strstr(wifi_rx_buffer,"OK") != NULL)
		{
			USART_puts("Modeme Baglanti yapildi\n",pc_uart);
			ESPEventCase = 3;
			Clear_ESPBuffer();
		}
		else
		{
			USART_puts("Modeme Baglanti Bekleniyor\n",pc_uart);
			HAL_Delay(2000);
			ESPEventCase=2;
			//	Clear_ESPBuffer();
		}

	}
	//	while(ESPEventCase==3)/********* AT+CIFSR **********/
	//	{                             // IP adresi nedir diye soruyoruz ?
	//		USART_puts("AT+CIFSR\r\n",wifi_uart);
	//		HAL_Delay(1000);
	//
	//		// IP alana kadar error bilgisi gonderir. Onu ayiririz. =)
	//		if (strstr(wifi_rx_buffer,"ERROR") == NULL)// error varsa null dönmez ife girmez elseye girer
	//			//"else"de wifi_rx_buffer temizlenir.error gelmeyene kadar   elseye gider.
	//		{
	//			USART_puts("Alinan IP = \n",pc_uart); // Gelen bilginin 11.karakterinden itibaren IP adresi yaziyor.
	//			USART_puts(&wifi_rx_buffer[11],pc_uart);
	//			ESPEventCase=4;
	//		}
	//		else
	//		{  // ERROR der ise tekrar dene
	//			HAL_Delay(1000);
	//			USART_puts("Tekrar Dene.\n",pc_uart);
	//			ESPEventCase=3;
	//		}
	//		Clear_ESPBuffer();
	//	}
	while(ESPEventCase==3)/********* AT+CIPMUX=0 **********/// 0=tekil bağ,1=çoğ bağlantı
	{
		USART_puts("AT+CIPMUX=0\r\n",wifi_uart);
		HAL_Delay(1000);
		if (strstr(wifi_rx_buffer,"OK") != NULL)
		{
			USART_puts("Tekil Baglanti yapildi\n",pc_uart);
			ESPEventCase = 4;
			Clear_ESPBuffer();
		}
		else
		{
			USART_puts("Tekil baglanti yapilamadi\n",pc_uart);
			HAL_Delay(2000);
			ESPEventCase=3;
			//	Clear_ESPBuffer();
		}
		Clear_ESPBuffer();
	}
}

void bufclr (char *buf)
{
	int len = strlen (buf);
	for (int i=0; i<len; i++) buf[i] = '\0';
}
/* veri upload diğer bir yol
void ESP_Send_Data (char *APIkey, int Field_num, uint16_t value)
{
	char local_buf[100] = {0};
	char local_buf2[30] = {0};

	Uart_sendstring("AT+CIPSTART=\"TCP\",\"184.106.153.149\",80\r\n");
	while (!(Wait_for("OK\r\n")));

	sprintf (local_buf, "GET /update?api_key=%s&field%d=%u\r\n", APIkey, Field_num, value);
	int len = strlen (local_buf);

	sprintf (local_buf2, "AT+CIPSEND=%d\r\n", len);
	Uart_sendstring(local_buf2);
	while (!(Wait_for(">")));

	Uart_sendstring (local_buf);
	while (!(Wait_for("SEND OK\r\n")));

	while (!(Wait_for("CLOSED")));

	bufclr(local_buf);
	bufclr(local_buf2);

	Ringbuf_init();

} */

void Clear_TC_Flag(void)
{
//		__HAL_UART_CLEAR_FLAG(pc_uart,UART_FLAG_TC);
//		__HAL_UART_CLEAR_FLAG(wifi_uart,UART_FLAG_TC);
}
void Clear_ESPBuffer(void)
{
	for(uint16_t clean_counter=0;clean_counter<ESP8266BUFFER_LENGHT;clean_counter++)
	{
		wifi_rx_buffer[clean_counter]=0;

	}
	wifi_buffer_counter=0;
}

void USART_puts(const char *s, UART_HandleTypeDef *uart)
{
	while(*s!='\0')
	{
		Uart_write_kod(*s++, uart);

	}
}

void Uart_write_kod(int c, UART_HandleTypeDef *uart)
{
//	if(wifi_buffer_counter != 0)
//	{
//		wifi_buffer_counter=0;
//	}
	tx_gecici_ortak_buffer=c;
	if(wifi_uart== uart)
	{
		HAL_UART_Transmit(wifi_uart,&tx_gecici_ortak_buffer,1,10);

	}
	else if(pc_uart== uart)
	{
		HAL_UART_Transmit(pc_uart,&tx_gecici_ortak_buffer,1,10);

	}
}
void Uart_isr (UART_HandleTypeDef *uart)
{

	if(wifi_uart== uart)
	{

			HAL_UART_Receive(wifi_uart,&wifi_rx_buffer[wifi_buffer_counter],1,10);
			wifi_buffer_counter++;
	}
	else if(pc_uart== uart)
	{
			HAL_UART_Receive(pc_uart,&pc_rx_buffer[k_counter],1,100);
//			pc_rx_buffer[k_counter]=USART2->DR;
			k_counter++;
	}
}

