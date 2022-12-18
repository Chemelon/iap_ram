#ifndef __BRIDGE_H
#define __BRIDGE_H
#include "stm32f10x.h"

void ZI_and_RW_init(void);
void copy_iapcode_toram(void);
void copy_vector_toram(void);
void jump_iap_ram(void);
#include "dependentfunc.h"

#endif

