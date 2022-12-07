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

//#define CM_BACKTRACE_TEST
#ifdef CM_BACKTRACE_TEST
static void fault_test_by_unalign(void)
{
    volatile int *SCB_CCR = (volatile int *)0xE000ED14; // SCB->CCR
    volatile int *p;
    volatile int value;

    *SCB_CCR |= (1 << 3); /* bit3: UNALIGN_TRP. */

    p = (int *)0x00;
    value = *p;
    printf("addr:0x%02X value:0x%08X\r\n", (int)p, value);

    p = (int *)0x04;
    value = *p;
    printf("addr:0x%02X value:0x%08X\r\n", (int)p, value);

    p = (int *)0x03;
    value = *p;
    printf("addr:0x%02X value:0x%08X\r\n", (int)p, value);
}

static void fault_test_by_div0(void)
{
    volatile int *SCB_CCR = (volatile int *)0xE000ED14; // SCB->CCR
    int x, y, z;

    *SCB_CCR |= (1 << 4); /* bit4: DIV_0_TRP. */

    x = 10;
    y = 0;
    z = x / y;
    printf("z:%d\n", z);
}
#endif

int main(void)
{
    /**
     * @brief 调用任何函数之前调用此函数初始化ZI段
     *
     */
    ZI_and_RW_init();

    /* 嵌套向量中断控制器组选择 */
    NVIC_SetPriorityGrouping(NVIC_PriorityGroup_4);
    USART_Config();
    cm_backtrace_init("ram_iap", "0.0.1", "0.0.1");

    Usart_SendString(DEBUG_USARTx, __DATE__);
    Usart_SendString(DEBUG_USARTx, "\r\n");
    Usart_SendString(DEBUG_USARTx, __TIME__);
    Usart_SendString(DEBUG_USARTx, "\r\n");
    Usart_SendString(DEBUG_USARTx, "now in the main app baud 115200\r\n");

    /**
     * @brief 按以下步骤初始化运行环境
     *
     */
    /* 复位所有外设 */
    reset_allperipheral();
    /* 设置栈顶到内存最高位置 */
#define SRAM_SIZE (20 * 1024)
    __set_MSP(SRAM_BASE + SRAM_SIZE);
    /* 拷贝中断向量表到ram */
    copy_vector_toram();
    /* 设置向量表地址 */
    NVIC_SetVectorTable(NVIC_VectTab_RAM, 0);
    /* 拷贝函数到RAM */
    copy_iapcode_toram();
    /* 跳转到ram */
    jump_iap_ram();
    for (;;)
    {
        Usart_SendString(DEBUG_USARTx, "err running in main\r\n");
    }
}
