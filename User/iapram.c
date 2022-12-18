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
#include "iapram.h"
#include <stdio.h>

/* DEBUG 用 */
#define DEBUG_USARTx USART3
extern void Usart_SendString(USART_TypeDef *pUSARTx, char *str);
/* DEBUG 用 */

static IR_uint8_t flashwrite_buffer[FLASH_PAGE_SIZE] = {0};
static IR_uint8_t flashread_buffer[FLASH_PAGE_SIZE] = {0};

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

#include "dependentfunc.h"
static void IR_frame_rx(void);
void IR_memcopy(IR_uint8_t *src, IR_uint8_t *dest, IR_uint32_t len);

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
        IR_frame_rx();
        /* 最大传输 FRAME_BUFFER_SIZE */
        DMA1_Channel5->CNDTR = FRAME_BUFFER_SIZE;
        DMA1_Channel5->CMAR = (IR_uint32_t)&frame_list[frame_rxinuse][0];

        /* 开启DMA */
        // DMA1_Channel5->CCR |= DMA_CCR5_EN;
    }
}

/**
 * @brief 内存拷贝
 *
 * @param src
 * @param dest
 * @param len
 */
static void IR_memcopy(IR_uint8_t *src, IR_uint8_t *dest, IR_uint32_t len)
{
    for (; len > 0; len--)
    {
        *dest++ = *src++;
    }
}

/**
 * @brief 接收完一帧后调用此函数
 *
 */
static void IR_frame_rx(void)
{
    /* 通知更新 */
    ((protocol_type *)(frame_list[frame_rxinuse]))->frame_status = DATA_RECEIVED;
    /* 切换frame 使 rxcpld 指向已经接收到数据的缓冲区 */
    frame_rxcpld = frame_rxinuse;
    frame_rxinuse = (frame_rxinuse + 1) % FRAME_BUFFER_CNT;
}

/**
 * @brief 求和校验
 *
 * @param verify_data 待计算数据
 * @param len 数据个数(单位:字节)
 * @return IR_unit8_t 校验结果 校验成功结果应该为0
 */
IR_uint8_t static IR_verify_sum(IR_uint8_t *verify_data, IR_uint16_t len)
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
 * @brief 组织发送帧并发送数据
 *
 * @param frame_buffer 帧内存
 * @param pdata 待发送数据
 * @param len 数据长度
 * @param func 帧功能
 * @param seq 帧序列
 */
static void IR_frame_tx(protocol_type *frame_buffer, IR_uint8_t *pdata, IR_uint16_t len, IR_uint8_t func, IR_uint8_t seq)
{
    if (len != 0)
    {
        IR_memcopy(pdata, frame_buffer->frame_data, len);
    }
    frame_buffer->device_addr = DEVICE_ADDR;
    frame_buffer->frame_func = func;
    frame_buffer->frame_seq = seq;
    frame_buffer->data_len = len;
    /* 将校验位置零 */
    //((IR_uint8_t *)frame_buffer)[4 + len] = 0;
    ((IR_uint8_t *)frame_buffer)[4 + len] = IR_verify_sum((IR_uint8_t *)frame_buffer, 4 + len);
    /* 发送回复帧 */
    IR_usart_dmatx(frame_buffer, 4 + 1);
}

/**
 * @brief 分析数据帧的各个字段并执行相关功能
 *
 */
static void IR_frame_analysis(void)
{
    IR_uint32_t write_inwaiting = 0, write_addr = FLASH_BASE;
    protocol_type *tempframe = NULL;
    for (;;)
    {
        if (((protocol_type *)frame_list[frame_rxcpld])->frame_status != DATA_RECEIVED)
        {
            continue;
        }
        /* 取得缓冲区指针 */
        tempframe = (protocol_type *)frame_list[frame_rxcpld];

        /* 校验和是否匹配 */
        if (IR_verify_sum((IR_uint8_t *)tempframe, 4 + tempframe->data_len + 1))
        {
            // Usart_SendString(DEBUG_USARTx, "verify error\r\n");
            continue;
        }
        /* 清除标记 */
        tempframe->frame_status = DATA_RESLOVED;

        /* 复制数据到FLASH写缓冲区 */
        IR_memcopy(tempframe->frame_data, &flashwrite_buffer[write_inwaiting], tempframe->data_len);
        write_inwaiting += tempframe->data_len;

        /* 文件结尾会发送一个长度为零的数据帧 */
        if (write_inwaiting == FLASH_PAGE_SIZE || tempframe->data_len == 0)
        {
            IR_flash_writebuffer(write_addr, &flashwrite_buffer[0]);
            write_inwaiting = 0;
            write_addr += FLASH_PAGE_SIZE;
        }
        if (tempframe->data_len == 0)
        {
            /* 所有包已收到 重置地址 */
            write_addr = FLASH_BASE;
        }
        /* 发送回复帧 */
        IR_frame_tx(tempframe, NULL, 0, tempframe->frame_func, tempframe->frame_seq);

        /* 开启接收DMA接收下一帧 */
        DMA1_Channel5->CCR |= DMA_CCR5_EN;
    }
}

/**
 * @brief 初始化所有硬件外设
 *
 */
static void IR_bspinit(void)
{
    IR_usart_init(72, 115200);
    IR_usart_nvic_init();
    IR_usart_rxdma_init();
    IR_usart_txdma_init();
}

/**
 * @brief 主函数
 *
 */
void iap_ram_app(void)
{
    Usart_SendString(DEBUG_USARTx, "running in the ram app baud 115200\r\n");
    IR_bspinit();
    IR_frame_analysis();
}

static void IR_flash_erase(IR_uint32_t Page_Address)
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
    // Usart_SendString(DEBUG_USARTx, "erase error\r\n");
    /* Disable the PER Bit */
    FLASH->CR &= CR_PER_Reset;
    return;
}

static void IR_flash_writehalfword(IR_uint32_t addr, IR_uint16_t data)
{
    IR_uint32_t Timeout;

    if (addr < FLASH_BASE || (addr >= (FLASH_BASE + STM32_FLASH_SIZE)) || addr % 2)
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
    /* 校验 */
    if (*(__IO uint16_t *)addr != data)
    {
        goto error;
    }
    /* Disable the PG Bit */
    FLASH->CR &= CR_PG_Reset;
    return;

error:
    /* Disable the PG Bit */
    FLASH->CR &= CR_PG_Reset;
    //Usart_SendString(DEBUG_USARTx, "program error\r\n");
    return;
}

/**
 * @brief 以页位索引写入数据
 *
 * @param page 要写入的页号(eg:0,1,2,3...)
 * @param pdata 指向要写入的数据
 * @param offset 页内偏移
 */
static void IR_flash_writepage(IR_uint8_t page, IR_uint8_t *pdata, IR_uint16_t offset)
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
#ifdef FALSH_USE_FUNCTIONCALL
    for (int i = 0; i < write_cnt;)
    {
#ifdef LITTLE_ENDIAN
        IR_flash_writehalfword(page_addr, flashread_buffer[i + 1] << 8 | flashread_buffer[i]);
#else
        IR_flash_writehalfword(page_addr, flashread_buffer[i] << 8 | flashread_buffer[i + 1]);
#endif
        page_addr += 2;
        i += 2;
    }
#else
    {
    }
#endif
}

/**
 * @brief 向内部flash写入一个 `FLASH_PAGE_SIZE` 大小的数据
 *
 * @param addr 起始写入地址
 * @param pdata 指向待写数据的指针
 */
static void IR_flash_writebuffer(IR_uint32_t addr, IR_uint8_t *pdata)
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

void IR_undefined_handler(void)
{
    for (;;)
    {
    }
}
