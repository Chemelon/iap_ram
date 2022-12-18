/* 这些函数与硬件相关 移植需要重新实现 */
#ifdef __IAPRAM_H
void IR_USART1_IRQHandler(void);
static void IR_bspinit(void);
static void IR_flash_writebuffer(IR_uint32_t addr, IR_uint8_t *pdata);
static void IR_frame_tx(protocol_type *frame_buffer, IR_uint8_t *pdata, IR_uint16_t len, IR_uint8_t func, IR_uint8_t seq);
#endif

#ifdef __BRIDGE_H
void reset_allperipheral(void);

/* hardfalt 用于 cmbacktrace 目前还不能用 因为和cmb相关的函数还存放在flash中 */
extern void HardFault_Handler(void);
/* ram app 需要使用IDLE中断 */
extern void IR_USART1_IRQHandler(void);
/* 不使用的中断都指向死循环 */
extern void IR_undefined_handler(void);
/* 生成要拷贝到ram中的中断向量表 */
static void *const VectorTable[] = {
    0,
    0,
    IR_undefined_handler,
    HardFault_Handler,
    IR_undefined_handler,
    IR_undefined_handler,
    IR_undefined_handler,
    0,
    0,
    0,
    0,
    IR_undefined_handler,
    IR_undefined_handler,
    0,
    IR_undefined_handler,
    IR_undefined_handler,
    IR_undefined_handler,
    IR_undefined_handler,
    IR_undefined_handler,
    IR_undefined_handler,
    IR_undefined_handler,
    IR_undefined_handler,
    IR_undefined_handler,
    IR_undefined_handler,
    IR_undefined_handler,
    IR_undefined_handler,
    IR_undefined_handler,
    IR_undefined_handler,
    IR_undefined_handler,
    IR_undefined_handler,
    IR_undefined_handler,
    IR_undefined_handler,
    IR_undefined_handler,
    IR_undefined_handler,
    IR_undefined_handler,
    IR_undefined_handler,
    IR_undefined_handler,
    IR_undefined_handler,
    IR_undefined_handler,
    IR_undefined_handler,
    IR_undefined_handler,
    IR_undefined_handler,
    IR_undefined_handler,
    IR_undefined_handler,
    IR_undefined_handler,
    IR_undefined_handler,
    IR_undefined_handler,
    IR_undefined_handler,
    IR_undefined_handler,
    IR_undefined_handler,
    IR_undefined_handler,
    IR_undefined_handler,
    IR_undefined_handler,
    IR_USART1_IRQHandler,
    IR_undefined_handler,
    IR_undefined_handler,
    IR_undefined_handler,
    IR_undefined_handler,
    IR_undefined_handler,
};
#endif
