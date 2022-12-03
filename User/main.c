/**
 * @file main.c
 * @author Chemelon (streleizia@163.com)
 * @brief 自带bootloader的工程模板 逻辑上来说 bootloader 与主程序共享库函数的地址
 * @version 0.1
 * @date 2022-11-24
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "stm32f10x.h"
#include "usart.h"
#include "bridge.h"
#include "cm_backtrace.h"
#include <stdio.h>

extern void iap_ram_app(void);
int main(void)
{
    cm_backtrace_init("ram_iap", "0.0.1", "0.0.1");

    /* 嵌套向量中断控制器组选择 */
    NVIC_SetPriorityGrouping(NVIC_PriorityGroup_4);
    USART_Config();
    
    Usart_SendString(DEBUG_USARTx, __DATE__);
    Usart_SendString(DEBUG_USARTx, "\r\n");
    Usart_SendString(DEBUG_USARTx, __TIME__);
    Usart_SendString(DEBUG_USARTx, "\r\n");
    Usart_SendString(DEBUG_USARTx, "now in the main app baud 115200\r\n");
    /* 拷贝函数到RAM */
    copy_iapcode_toram();
    /* 复位所有外设 */
    reset_allperipheral();
    /* 跳转到ram */
    iap_ram_app();
    for (;;)
    {
        Usart_SendString(DEBUG_USARTx, "err running in main\r\n");
    }
}
