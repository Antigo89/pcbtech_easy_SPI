#ifndef H_MAIN
#define H_MAIN 1

#include <stdio.h>
#include <stdlib.h>
#include "stm32f4xx.h"

//Settings
//#define DATA_IN_MEMORY 1

//Description connect
#define EN_RST 0x0066
#define RST 0x0099
#define WR_EN 0x0006
#define SECT_ER 0x0020
#define RD_SR1 0x0005
#define PG_PROG 0x0002
#define RD_DATA 0x0003

#define ADDR1 0x303030
#define ADDR2 0x303031
#define ADDR3 0x303032
//Constants
#define LED1 0x01
#define LED2 0x02
#define LED3 0x03
#define CLEAR 0xFF

//prototype function
uint16_t w25send_byte(uint16_t data);
void w25write(uint32_t addr, uint16_t data);
uint8_t w25read(uint32_t addr);
void w25clear_sector(uint32_t addr);

void RCC_Init(void);
void key_led_Init(void);
void SPI2_Init(void);

#endif