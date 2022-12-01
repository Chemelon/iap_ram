#ifndef __BRIDGE_H
#define __BRIDGE_H
#include "stm32f10x.h"

void copy_iapcode_toram(void);
void jump_iap_ram(void);
void reset_allperipheral(void);

#endif

