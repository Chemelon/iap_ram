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
extern void iap_ram_app(void);

/* hardfalt 用于 cmbacktrace */
extern void HardFault_Handler(void);
extern void USART1_IRQHandler(void);
void undefined_handler(void)
{
    for (;;)
    {
    }
}
/* 中断向量表 */
static const void* const VectorTable[] = {
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
    USART1_IRQHandler,
    undefined_handler,
    undefined_handler,
    undefined_handler,
    undefined_handler,
    undefined_handler,
};

void copy_vector_toram(void)
{
    /* 运行地址 */
    extern unsigned char Image$$ER_VECTOR$$Base;
    /* 大小 */
    unsigned char * src = (unsigned char *)&VectorTable[0];
    unsigned char * dest = &Image$$ER_VECTOR$$Base;
    unsigned int count = 59 * 4;
    for(;count > 0 ;count--)
    {
        *dest++ = *src++;
    }

}

void reset_allperipheral(void)
{
    GPIO_DeInit(GPIOA);
    USART_DeInit(USART1);
}

#if 1
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
#endif

void jump_iap_ram(void)
{
    void (*pfunc_app)(void);
    /* 跳转到boot loader */
    pfunc_app = iap_ram_app;
    (*pfunc_app)();
    /* 正常不会运行到这里 */
    while (1)
    {
    }
}
