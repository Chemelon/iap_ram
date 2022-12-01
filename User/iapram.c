/**
 * @file bootloader.c
 * @author Chemelon (streleizia@163.com)
 * @brief 在ram中执行 可以完成更新固件的工作 储存在flash的最后部分 通过分散加载文件指定运行地址
 * @version 0.1
 * @date 2022-11-24
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "stm32f10x.h"
#include "usart.h"
#define FRAME_DATA_SIZE 255
#define FRAME_BUFFER_SIZE (1 + 4 + FRAME_DATA_SIZE + 1)

#define DATA_RESLOVED 0 // 缓冲区中数据已经被处理
#define DATA_RECEIVED 1 // 缓冲区已接受数据待处理

/**
 * @brief 协议数据帧结构 参考正点原子的XCOM串口软件的协议传输功能设计
 *        详细功能参见http://www.openedv.com/posts/list/22994.htm
 */
typedef struct protocol_struct
{
    unsigned char device_addr;                 // 从机地址
    unsigned char frame_func;                  // 帧功能
    unsigned char frame_seq;                   // 帧序列 由主机决定 从机返回相同序列
    unsigned char data_len;                    // 帧数据长度
    unsigned char frame_data[FRAME_DATA_SIZE]; // 帧数据
    unsigned char verify_sum;                  // 帧校验和(求和校验)
    unsigned char frame_status;                // 帧状态(非协议必须,定义此数据为了方便程序编写)
} protocol_type;

/* 数据缓冲区 */
static volatile unsigned char frame1[FRAME_BUFFER_SIZE] = {0};
static volatile unsigned char frame2[FRAME_BUFFER_SIZE] = {0};
/* 用于管理缓冲区的指针 */
volatile unsigned char *frame_rxinuse = frame1;
volatile unsigned char *frame_rxcpld = frame2;

/**
 * @brief 求和校验
 *
 * @param verify_data 待计算数据
 * @param len 数据个数(单位:字节)
 * @return unsigned char
 */
unsigned char IR_verify_sum(unsigned char *verify_data, unsigned short len)
{
    unsigned char result = 0;
    for (; len > 0; len--)
    {
        result += *verify_data++;
    }
    result = ~result + 1;
    return result;
}

/**
 * @brief 初始化串口
 *
 * @param apbclock_Mhz 总线时钟频率
 * @param baudrate 串口波特率
 */
static void IR_usart_init(unsigned char apbclock_Mhz, unsigned int baudrate)
{
    /* 开启USART1 GPIOA 时钟 */
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_IOPAEN;
    /* PB10 IPD PB9 AFPP */
    GPIOA->CRH &= 0XFFFFF00F; // IO状态设置
    GPIOA->CRH |= 0X000004B0; // IO状态设置

    unsigned short mantissa;
    unsigned short fraction;
    float temp;
    /* USARTDIV */
    temp = (float)(apbclock_Mhz * 1000000) / (baudrate * 16);
    /* 整数部分 */
    mantissa = temp;
    /* 小数部分 */
    fraction = (temp - mantissa) * 16;
    mantissa <<= 4;
    mantissa += fraction;
    USART1->BRR = mantissa;
    /* 8-N-1 */
    USART1->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

/**
 * @brief 初始化USART的DMA接收功能
 *
 */
void IR_usart_rxdma_init(void)
{
    /* 关闭USART1 */
    USART1->CR1 &= ~USART_CR1_UE;

    /* 开启DMA时钟 */
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    /* 默认设值 P->M 8位 设置内存地址递增 PL:High */
    DMA1_Channel5->CCR |= DMA_CCR5_MINC | DMA_CCR5_PL_1;
    /* 最大传输 FRAME_BUFFER_SIZE */
    DMA1_Channel5->CNDTR = FRAME_BUFFER_SIZE;
    DMA1_Channel5->CPAR = (unsigned int)&(USART1->DR);
    DMA1_Channel5->CMAR = (unsigned int)&frame_rxinuse[0];
    /* 开启DMA */
    DMA1_Channel5->CCR |= DMA_CCR5_EN;

    /* 接收DMA请求使能 */
    USART1->CR3 |= USART_CR3_DMAR;
    /* 开启IDLE中断 */
    USART1->CR1 |= USART_CR1_IDLEIE;
    /* 开启USART */
    USART1->CR1 |= USART_CR1_UE;
    /* 清除IDLE位 */
    USART1->SR;
    USART1->DR;
}

/**
 * @brief 初始化USART的DMA发送功能
 *
 */
void IR_usart_txdma_init(void)
{
    /* 关闭USART1 */
    USART1->CR1 &= ~USART_CR1_UE;

    /* 开启DMA时钟 */
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    /* 默认设值 M->P 8位 设置内存地址递增 PL:High */
    DMA1_Channel4->CCR |= DMA_CCR4_MINC | DMA_CCR4_PL_1 | DMA_CCR4_DIR;
    /* 最大传输 FRAME_BUFFER_SIZE */
    DMA1_Channel4->CNDTR = 0;
    DMA1_Channel4->CPAR = (unsigned int)&(USART1->DR);
    DMA1_Channel4->CMAR = (unsigned int)&frame_rxcpld[0];
    /* 开启DMA */
    DMA1_Channel4->CCR |= DMA_CCR4_EN;

    /* 发送DMA请求使能 */
    USART1->CR3 |= USART_CR3_DMAT;
    /* 开启USART */
    USART1->CR1 |= USART_CR1_UE;
    /* 清除IDLE位 */
    USART1->SR;
    USART1->DR;
}

/**
 * @brief 通过dma发送一个缓冲区
 *
 * @param buffer_addr 缓冲区地址
 * @param len 待发送数据长度
 */
void IR_usart_dmatx(void *buffer_addr, unsigned short len)
{
    /* 关闭通道4 */
    DMA1_Channel4->CCR &= ~DMA_CCR4_EN;

    DMA1_Channel4->CMAR = (unsigned int)buffer_addr;
    DMA1_Channel4->CNDTR = len;

    /* 开启DMA */
    DMA1_Channel4->CCR |= DMA_CCR4_EN;
}

/**
 * @brief 初始化usart的中断配置
 *
 */
void IR_usart_nvic_init(void)
{
    /* 寄存器版暂时跑不通 待修改 */
#if 0
    uint32_t tmppriority = 0x00, tmppre = 0x00, tmpsub = 0x0F;

    /* Compute the Corresponding IRQ Priority --------------------------------*/
    tmppriority = (0x700 - ((SCB->AIRCR) & (uint32_t)0x700)) >> 0x08;
    tmppre = (0x4 - tmppriority);
    tmpsub = tmpsub >> tmppriority;

    tmppriority = 6 << tmppre;
    tmppriority |= 0 & tmpsub;
    tmppriority = tmppriority << 0x04;

    NVIC->IP[USART1_IRQn] = tmppriority;

    /* Enable the Selected IRQ Channels --------------------------------------*/
    NVIC->ISER[USART1_IRQn >> 0x05] =
        (uint32_t)0x01 << (USART1_IRQn & (uint8_t)0x1F);
#else
    NVIC_InitTypeDef NVIC_InitStructure;

    /* 配置USART为中断源 */
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    /* 抢断优先级*/
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
    /* 子优先级 */
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    /* 使能中断 */
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    /* 初始化配置NVIC */
    NVIC_Init(&NVIC_InitStructure);
#endif
}

void USART1_IRQHandler(void)
{
    /* IDLE中断 */
    // Usart_SendString(DEBUG_USARTx, "enter usart irq\r\n");
    if (USART1->SR & USART_SR_IDLE)
    {
        /* 通知更新 */
        ((protocol_type *)frame_rxinuse)->frame_status = DATA_RECEIVED;
        /* 切换frame rxcpld 指向已经接收到数据的缓冲区 */
        if (frame_rxinuse == frame1)
        {
            frame_rxinuse = frame2;
            frame_rxcpld = frame1;
        }
        else
        {
            frame_rxinuse = frame1;
            frame_rxcpld = frame2;
        }

        /* 关闭DMA */
        DMA1_Channel5->CCR &= ~DMA_CCR5_EN;
        /* 最大传输 FRAME_BUFFER_SIZE */
        DMA1_Channel5->CNDTR = FRAME_BUFFER_SIZE;
        DMA1_Channel5->CMAR = (unsigned int)&frame_rxinuse[0];
        /* 开启DMA */
        DMA1_Channel5->CCR |= DMA_CCR5_EN;
        /* 清除IDLE位 */
        USART1->SR;
        USART1->DR;
    }
}

/**
 * @brief 分析数据帧的各个字段并执行相关功能
 *
 */
static void IR_frame_analysis(void)
{
    protocol_type *tempframe = NULL;
    if (((protocol_type *)frame_rxcpld)->frame_status == DATA_RECEIVED)
    {
        Usart_SendString(DEBUG_USARTx, "frame detected\r\n");
        tempframe = (protocol_type *)frame_rxcpld;
        /* 取得校验和 */
        tempframe->verify_sum = ((unsigned char *)tempframe)[4 + tempframe->data_len];
        /* 校验和是否匹配 */
        if (IR_verify_sum((unsigned char *)tempframe, 4 + tempframe->data_len + 1))
        {
            Usart_SendString(DEBUG_USARTx, "verify error\r\n");
        }
        else
        {
            IR_usart_dmatx(tempframe, 4 + tempframe->data_len + 1);
            Usart_SendArray(DEBUG_USARTx, (unsigned char *)tempframe, 4 + tempframe->data_len + 1);
        }
        tempframe->frame_status = DATA_RESLOVED;
    }
}

/**
 * @brief 主函数
 *
 */
void iap_ram_app(void)
{
    volatile int x = 10, y = 0, z = 0;

    IR_usart_init(72, 115200);
    Usart_SendString(DEBUG_USARTx, "running in the ram app baud 115200\r\n");
    Usart_SendString(DEBUG_USARTx, NULL);

    IR_usart_nvic_init();
    IR_usart_rxdma_init();
    IR_usart_txdma_init();
    
    for (;;)
    {
        IR_frame_analysis();
    }
}

void fault_test_by_unalign(void)
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

void fault_test_by_div0(void)
{
    volatile int *SCB_CCR = (volatile int *)0xE000ED14; // SCB->CCR
    int x, y, z;

    *SCB_CCR |= (1 << 4); /* bit4: DIV_0_TRP. */

    x = 10;
    y = 0;
    z = x / y;
    printf("z:%d\n", z);
}
