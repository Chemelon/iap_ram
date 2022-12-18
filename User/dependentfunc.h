/* ��Щ������Ӳ����� ��ֲ��Ҫ����ʵ�� */
#ifdef __IAPRAM_H
void IR_USART1_IRQHandler(void);
static void IR_bspinit(void);
static void IR_flash_writebuffer(IR_uint32_t addr, IR_uint8_t *pdata);
static void IR_frame_tx(protocol_type *frame_buffer, IR_uint8_t *pdata, IR_uint16_t len, IR_uint8_t func, IR_uint8_t seq);
#endif

#ifdef __BRIDGE_H
void reset_allperipheral(void);

/* hardfalt ���� cmbacktrace Ŀǰ�������� ��Ϊ��cmb��صĺ����������flash�� */
extern void HardFault_Handler(void);
/* ram app ��Ҫʹ��IDLE�ж� */
extern void IR_USART1_IRQHandler(void);
/* ��ʹ�õ��ж϶�ָ����ѭ�� */
extern void IR_undefined_handler(void);
/* ����Ҫ������ram�е��ж������� */
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
