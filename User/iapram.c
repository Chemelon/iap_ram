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

typedef unsigned int IR_uint32_t;
typedef unsigned short IR_uint16_t;
typedef unsigned char IR_uint8_t;

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

#define STM32_FLASH_SIZE 64

#if defined(STM32F10X_HD) || defined(STM32F10X_HD_VL) || defined(STM32F10X_CL) || defined(STM32F10X_XL)
#define FLASH_PAGE_SIZE ((uint16_t)0x800) // 2048
#else
#define FLASH_PAGE_SIZE ((uint16_t)0x400) // 1024
#endif

static IR_uint8_t flashwrite_buffer[FLASH_PAGE_SIZE] = {0};
static IR_uint8_t flashread_buffer[FLASH_PAGE_SIZE] = {0};

#define FRAME_DATA_SIZE 255
#define FRAME_BUFFER_SIZE (4 + FRAME_DATA_SIZE + 1 + 1)
#define FRAME_BUFFER_CNT 4

#define DATA_RESLOVED 0 // 缓冲区中数据已经被处理
#define DATA_RECEIVED 1 // 缓冲区已接受数据待处理

/**
 * @brief 协议数据帧结构 参考正点原子的XCOM串口软件的协议传输功能设计
 *        详细功能参见http://www.openedv.com/posts/list/22994.htm
 */
typedef struct protocol_struct
{
    IR_uint8_t device_addr;                 // 从机地址
    IR_uint8_t frame_func;                  // 帧功能
    IR_uint8_t frame_seq;                   // 帧序列 由主机决定 从机返回相同序列
    IR_uint8_t data_len;                    // 帧数据长度
    IR_uint8_t frame_data[FRAME_DATA_SIZE]; // 帧数据
    /// @brief bug(solved): 将缓冲区转换为 protocol_type 类型访问时 verify_sum 应该储存在缓冲区的倒数第二个位置,之前少一个字
    ///         节,所以现在FRAME_BUFFER_SIZE = (4 + FRAME_DATA_SIZE + 1 + 1)
    IR_uint8_t verify_sum;            // 帧校验和(求和校验)
    volatile IR_uint8_t frame_status; // 帧状态(非协议必须,定义此数据为了方便程序编写)
} protocol_type;

/* 数据缓冲区 */
static IR_uint8_t frame1[FRAME_BUFFER_SIZE] = {0};
static IR_uint8_t frame2[FRAME_BUFFER_SIZE] = {0};
static IR_uint8_t frame3[FRAME_BUFFER_SIZE] = {0};
static IR_uint8_t frame4[FRAME_BUFFER_SIZE] = {0};

/* 指向所有缓冲区的指针数组 */
static IR_uint8_t *frame_list[FRAME_BUFFER_CNT] = {
    frame1,
    frame2,
    frame3,
    frame4,
};


/* 用于管理缓冲区指针的索引值 */
static volatile IR_uint8_t frame_rxinuse = 0;
static volatile IR_uint8_t frame_rxcpld = 0;

void IR_flash_writebuffer(IR_uint32_t addr, IR_uint8_t *pdata);
void IR_memcopy(IR_uint8_t *src, IR_uint8_t *dest, IR_uint32_t len);
void *function_list[] = {
    IR_flash_writebuffer,
};

void IR_USART1_IRQHandler(void)
{
    /* IDLE中断 */
    if (USART1->SR & USART_SR_IDLE)
    {
        /* 清除IDLE位 */
        USART1->SR;
        USART1->DR;
        /* 关闭DMA */
        DMA1_Channel5->CCR &= ~DMA_CCR5_EN;

        // Usart_SendString(DEBUG_USARTx, "idle event detected\r\n");

        /* 通知更新 */
        ((protocol_type *)(frame_list[frame_rxinuse]))->frame_status = DATA_RECEIVED;
        /* 切换frame 使 rxcpld 指向已经接收到数据的缓冲区 */
        frame_rxcpld = frame_rxinuse;
        frame_rxinuse = (frame_rxinuse + 1) % FRAME_BUFFER_CNT;
        /* 最大传输 FRAME_BUFFER_SIZE */
        DMA1_Channel5->CNDTR = FRAME_BUFFER_SIZE;
        DMA1_Channel5->CMAR = (IR_uint32_t)&frame_list[frame_rxinuse][0];

        /* 开启DMA */
        // DMA1_Channel5->CCR |= DMA_CCR5_EN;
    }
}

/**
 * @brief 求和校验
 *
 * @param verify_data 待计算数据
 * @param len 数据个数(单位:字节)
 * @return IR_unit8_t
 */
IR_uint8_t IR_verify_sum(IR_uint8_t *verify_data, IR_uint16_t len)
{
    IR_uint8_t result = 0;
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
static void IR_usart_init(IR_uint8_t apbclock_Mhz, IR_uint32_t baudrate)
{
    /* 开启USART1 GPIOA 时钟 */
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_IOPAEN;
    /* PB10 IPD PB9 AFPP */
    GPIOA->CRH &= 0XFFFFF00F; // IO状态设置
    GPIOA->CRH |= 0X000004B0; // IO状态设置

    IR_uint16_t mantissa;
    IR_uint16_t fraction;
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
static void IR_usart_rxdma_init(void)
{
    /* 关闭USART1 */
    USART1->CR1 &= ~USART_CR1_UE;

    /* 开启DMA时钟 */
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    /* 默认设值 P->M 8位 设置内存地址递增 PL:High */
    DMA1_Channel5->CCR |= DMA_CCR5_MINC | DMA_CCR5_PL_1;
    /* 最大传输 FRAME_BUFFER_SIZE */
    DMA1_Channel5->CNDTR = FRAME_BUFFER_SIZE;
    DMA1_Channel5->CPAR = (IR_uint32_t) & (USART1->DR);
    DMA1_Channel5->CMAR = (IR_uint32_t)&frame_list[frame_rxinuse][0];
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
static void IR_usart_txdma_init(void)
{
    /* 关闭USART1 */
    USART1->CR1 &= ~USART_CR1_UE;

    /* 开启DMA时钟 */
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    /* 默认设值 M->P 8位 设置内存地址递增 PL:High */
    DMA1_Channel4->CCR |= DMA_CCR4_MINC | DMA_CCR4_PL_1 | DMA_CCR4_DIR;
    /* 最大传输 FRAME_BUFFER_SIZE */
    DMA1_Channel4->CNDTR = 0;
    DMA1_Channel4->CPAR = (IR_uint32_t) & (USART1->DR);
    DMA1_Channel4->CMAR = (IR_uint32_t)&frame_list[frame_rxcpld][0];
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
static void IR_usart_dmatx(void *buffer_addr, IR_uint16_t len)
{
    /* 关闭通道4 */
    DMA1_Channel4->CCR &= ~DMA_CCR4_EN;

    DMA1_Channel4->CMAR = (IR_uint32_t)buffer_addr;
    DMA1_Channel4->CNDTR = len;

    /* 开启DMA */
    DMA1_Channel4->CCR |= DMA_CCR4_EN;
}

/**
 * @brief 初始化usart的中断配置
 *
 */
static void IR_usart_nvic_init(void)
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

/**
 * @brief 分析数据帧的各个字段并执行相关功能
 *
 */
static void IR_frame_analysis(void)
{
    IR_uint32_t i = 0, write_addr = FLASH_BASE;
    protocol_type *tempframe = NULL;
    for (;;)
    {
        if (((protocol_type *)frame_list[frame_rxcpld])->frame_status == DATA_RECEIVED)
        {
            // Usart_SendByte(DEBUG_USARTx, 0x30 + frame_rxcpld);
            /* 取得缓冲区指针 */
            tempframe = (protocol_type *)frame_list[frame_rxcpld];

            // Usart_SendByte(DEBUG_USARTx, tempframe->data_len);
            /* 取得校验和 */
            tempframe->verify_sum = ((IR_uint8_t *)tempframe)[4 + tempframe->data_len];
            // Usart_SendArray(DEBUG_USARTx, tempframe->frame_data, tempframe->data_len);
            /* 校验和是否匹配 */
            if (!IR_verify_sum((IR_uint8_t *)tempframe, 4 + tempframe->data_len + 1))
            {
                IR_memcopy(tempframe->frame_data, &flashwrite_buffer[i], tempframe->data_len);

                i += tempframe->data_len;
                /* 文件结尾会发送一个长度为零的数据帧 */
                if (i == FLASH_PAGE_SIZE || tempframe->data_len == 0)
                {
                    IR_flash_writebuffer(write_addr, &flashwrite_buffer[0]);
                    // Usart_SendString(DEBUG_USARTx,"update\r\n");
                    // Usart_SendArray(DEBUG_USARTx,&flashwrite_buffer[0],i);
                    i = 0;
                    write_addr += FLASH_PAGE_SIZE;
                    if (tempframe->data_len == 0)
                    {
                        write_addr = FLASH_BASE;
                    }
                }
                /* 组织并填充待发送数据 */
                tempframe->data_len = 0;
                /* 将校验位置零 */
                ((IR_uint8_t *)tempframe)[4] = 0;
                /* 计算校验值并填充 */
                ((IR_uint8_t *)tempframe)[4] = IR_verify_sum((IR_uint8_t *)tempframe, 4 + 1);
                /* 发送回复帧 */
                IR_usart_dmatx(tempframe, 4 + 1);
                /* 清除标记 */
                tempframe->frame_status = DATA_RESLOVED;
                /* 开启接收DMA */
                DMA1_Channel5->CCR |= DMA_CCR5_EN;
            }
            else
            {
                Usart_SendString(DEBUG_USARTx, "verify error\r\n");
            }
        }
    }
}

/**
 * @brief 主函数
 *
 */
void iap_ram_app(void)
{
    IR_usart_init(72, 115200);
    Usart_SendString(DEBUG_USARTx, "running in the ram app baud 115200\r\n");

    IR_usart_nvic_init();
    IR_usart_rxdma_init();
    IR_usart_txdma_init();
    // fault_test_by_unalign();
    IR_frame_analysis();
}

void IR_flash_erase(IR_uint32_t Page_Address)
{
    IR_uint32_t Timeout;
    Timeout = 0x000B0000;
    while ((FLASH->SR & FLASH_FLAG_BANK1_BSY))
    {
        Timeout--;
        if (Timeout == 0)
        {
            goto error;
        }
    }
    /* if the previous operation is completed, proceed to erase the page */
    FLASH->CR |= CR_PER_Set;
    FLASH->AR = Page_Address;
    FLASH->CR |= CR_STRT_Set;

    /* Wait for last operation to be completed */
    Timeout = 0x000B0000;
    while ((FLASH->SR & FLASH_FLAG_BANK1_BSY))
    {
        Timeout--;
        if (Timeout == 0)
        {
            goto error;
        }
    }

    /* Disable the PER Bit */
    FLASH->CR &= CR_PER_Reset;
    return;
error:
    Usart_SendString(DEBUG_USARTx, "erase error\r\n");
}

void IR_memcopy(IR_uint8_t *src, IR_uint8_t *dest, IR_uint32_t len)
{
    for (; len > 0; len--)
    {
        *dest++ = *src++;
    }
}

void IR_flash_writehalfword(IR_uint32_t addr, IR_uint16_t data)
{
    IR_uint32_t Timeout;

    if (addr < FLASH_BASE || (addr >= (FLASH_BASE + 1024 * STM32_FLASH_SIZE)) || addr % 2)
    {
        /* 非法地址 */
        goto error;
    }

    Timeout = 0x00002000;
    while ((FLASH->SR & FLASH_FLAG_BANK1_BSY))
    {
        Timeout--;
        if (Timeout == 0)
        {
            goto error;
        }
    }
    /* if the previous operation is completed, proceed to program the new data */
    FLASH->CR |= CR_PG_Set;
    /* 写入 */
    *(__IO uint16_t *)addr = data;
    /* Wait for last operation to be completed */

    Timeout = 0x00002000;
    while ((FLASH->SR & FLASH_FLAG_BANK1_BSY))
    {
        Timeout--;
        if (Timeout == 0)
        {
            goto error;
        }
    }
    /* Disable the PG Bit */
    FLASH->CR &= CR_PG_Reset;
    return;

error:
    Usart_SendString(DEBUG_USARTx, "program error\r\n");
    return;
}

void IR_flash_writepage(IR_uint8_t page, IR_uint8_t *pdata, IR_uint16_t offset)
{
    IR_uint32_t page_addr = FLASH_BASE + (page * FLASH_PAGE_SIZE);
    IR_uint16_t write_cnt = FLASH_PAGE_SIZE - offset;
    /* 回读 */
    IR_memcopy((uint8_t *)page_addr, flashread_buffer, FLASH_PAGE_SIZE);
    for (int i = 0; i < FLASH_PAGE_SIZE; i++)
    {
        if (flashread_buffer[i] != 0xff)
        {
            /* 擦除 */
            IR_flash_erase(page_addr);
            break;
        }
    }
    /* 改写 */
    IR_memcopy(pdata, &flashread_buffer[offset], write_cnt);
    /* 写入 */
    for (int i = 0; i < write_cnt;)
    {
        IR_flash_writehalfword(page_addr, flashread_buffer[i + 1] << 8 | flashread_buffer[i]);
        page_addr += 2;
        i += 2;
    }
    /* 这里缺少一个校验环节 */
}

void IR_flash_writebuffer(IR_uint32_t addr, IR_uint8_t *pdata)
{
    IR_uint8_t page = (addr / FLASH_PAGE_SIZE) - 0x20000;
    IR_uint16_t residue = addr % FLASH_PAGE_SIZE;

    /* 解锁 */
    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;
    /* 以页为单位写入 */
    IR_flash_writepage(page, pdata, residue);
    /* 横跨两页 */
    if (residue)
    {
        page++;
        IR_flash_writepage(page, pdata + FLASH_PAGE_SIZE - residue, 0);
    }
    /* 上锁 */
    FLASH->CR |= CR_LOCK_Set;
}
