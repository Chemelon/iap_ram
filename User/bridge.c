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
