#ifndef __IAPRAM_H
#define __IAPRAM_H

typedef unsigned int IR_uint32_t;
typedef unsigned short IR_uint16_t;
typedef unsigned char IR_uint8_t;

/* 和读写stm32内部flash相关的宏定义 */
#define FLASH_KEY1 ((IR_uint32_t)0x45670123)
#define FLASH_KEY2 ((IR_uint32_t)0xCDEF89AB)
#define CR_PG_Set ((IR_uint32_t)0x00000001)
#define CR_PG_Reset ((IR_uint32_t)0x00001FFE)
#define CR_PER_Set ((IR_uint32_t)0x00000002)
#define CR_PER_Reset ((IR_uint32_t)0x00001FFD)
#define CR_MER_Set ((IR_uint32_t)0x00000004)
#define CR_MER_Reset ((IR_uint32_t)0x00001FFB)
#define CR_OPTPG_Set ((IR_uint32_t)0x00000010)
#define CR_OPTPG_Reset ((IR_uint32_t)0x00001FEF)
#define CR_OPTER_Set ((IR_uint32_t)0x00000020)
#define CR_OPTER_Reset ((IR_uint32_t)0x00001FDF)
#define CR_STRT_Set ((IR_uint32_t)0x00000040)
#define CR_LOCK_Set ((IR_uint32_t)0x00000080)

/* FLASH大小 单位byte */
#define STM32_FLASH_SIZE (64 * 1024)

#if defined(STM32F10X_HD) || defined(STM32F10X_HD_VL) || defined(STM32F10X_CL) || defined(STM32F10X_XL)
#define FLASH_PAGE_SIZE ((uint16_t)0x800) // 2048
#else
#define FLASH_PAGE_SIZE ((uint16_t)0x400) // 1024
#endif

#define FRAME_DATA_SIZE 255
#define FRAME_BUFFER_SIZE (4 + FRAME_DATA_SIZE + 1 + 1)
#define FRAME_BUFFER_CNT 4

#define DATA_RESLOVED 0 // 缓冲区中数据已经被处理
#define DATA_RECEIVED 1 // 缓冲区已接受数据待处理

#endif
