/**
 * @file bridge.c
 * @author Chemelon (streleizia@163.com)
 * @brief 做好从主程序跳转到bootloader中运行前的初始化工作
 * 包括重置一些外设配置 中断向量表的配置 拷贝bootloader代码
 * 到它的执行地址等等
 * @version 0.1
 * @date 2022-11-24
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "bridge.h"

/* hardfalt 用于 cmbacktrace */
extern void HardFault_Handler(void);
/* 此函数在ram中 */
extern void IR_USART1_IRQHandler(void);
void undefined_handler(void)
{
    for (;;)
    {
    }
}
/* 要拷贝到ram中的中断向量表 */
static const void *const VectorTable[] = {
    0,
    0,
    undefined_handler,
    HardFault_Handler,
    undefined_handler,
    undefined_handler,
    undefined_handler,
    0,
    0,
    0,
    0,
    undefined_handler,
    undefined_handler,
    0,
    undefined_handler,
    undefined_handler,
    undefined_handler,
    undefined_handler,
    undefined_handler,
    undefined_handler,
    undefined_handler,
    undefined_handler,
    undefined_handler,
    undefined_handler,
    undefined_handler,
    undefined_handler,
    undefined_handler,
    undefined_handler,
    undefined_handler,
    undefined_handler,
    undefined_handler,
    undefined_handler,
    undefined_handler,
    undefined_handler,
    undefined_handler,
    undefined_handler,
    undefined_handler,
    undefined_handler,
    undefined_handler,
    undefined_handler,
    undefined_handler,
    undefined_handler,
    undefined_handler,
    undefined_handler,
    undefined_handler,
    undefined_handler,
    undefined_handler,
    undefined_handler,
    undefined_handler,
    undefined_handler,
    undefined_handler,
    undefined_handler,
    undefined_handler,
    IR_USART1_IRQHandler,
    undefined_handler,
    undefined_handler,
    undefined_handler,
    undefined_handler,
    undefined_handler,
};

/**
 * @brief 在main中调用任何函数前请调用此函数以初始化ZI段
 * @note c库函数不会初始化定义了OVERLAY 属性的运行域 所以要手动拷贝ZI数据到RAM对应位置 并且ZI和STACK是连续放置的所以很容易计
 * 算得到要初始化为零的区域大小
 *
 */
void ZI_and_RW_init()
{
    __IO uint32_t pstack = __get_MSP();
    /* 储存地址 */
    extern unsigned char Load$$RW_IRAM1$$Base;
    /* 运行地址 */
    extern unsigned char Image$$RW_IRAM1$$Base;
    /* 长度(只有RW段的长度不包含ZI段长度) */
    extern unsigned char Image$$RW_IRAM1$$Length;

    unsigned char *psrc = &Load$$RW_IRAM1$$Base;
    unsigned char *pdest = &Image$$RW_IRAM1$$Base;
    unsigned int len = (unsigned int)&Image$$RW_IRAM1$$Length;
    /* 复制RW */
    for (; len > 0; len--)
    {
        *pdest++ = *psrc++;
    }
    len = pstack - (unsigned int)pdest;
    /* 初始化ZI */
    for (; len > 0; len--)
    {
        *pdest++ = 0;
    }
}

/**
 * @brief 将ram_app运行时的中断向量表拷贝到内存的对应位置
 *
 */
void copy_vector_toram(void)
{
    unsigned char *src = (unsigned char *)&VectorTable[0];
    unsigned char *dest = (unsigned char *)SRAM_BASE;
    /* 大小 */
    unsigned int count = sizeof(VectorTable) * sizeof(void *);
    for (; count > 0; count--)
    {
        *dest++ = *src++;
    }
}

/**
 * @brief 复位所有已使用的外设
 *
 */
void reset_allperipheral(void)
{
    GPIO_DeInit(GPIOA);
    USART_DeInit(USART1);
}

/**
 * @brief 将IAP程序代码拷贝到链接地址
 *
 */
void copy_iapcode_toram(void)
{
    /* 储存地址 */
    extern unsigned char Load$$ER_IROM2$$Base;
    /* 运行地址 */
    extern unsigned char Image$$ER_IROM2$$Base;
    /* 长度 */
    extern unsigned char Image$$ER_IROM2$$Length;
    unsigned char *psrc, *pdest;
    unsigned int count;
    psrc = (unsigned char *)&Load$$ER_IROM2$$Base;
    pdest = (unsigned char *)&Image$$ER_IROM2$$Base;
    count = (unsigned int)&Image$$ER_IROM2$$Length;
    /* 复制代码 */
    while (count--)
    {
        *pdest++ = *psrc++;
    }
}

/**
 * @brief 跳转到ram中执行IAP代码
 *
 */
void jump_iap_ram(void)
{
    extern void iap_ram_app(void);
    /* 跳转到boot loader */
    iap_ram_app();
    /* 正常不会运行到这里 */
    while (1)
    {
    }
}
