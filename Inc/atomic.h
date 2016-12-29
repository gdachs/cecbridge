/*
 * atomic.h
 *
 *  Created on: 23.12.2016
 *      Author: gda
 */

#ifndef ATOMIC_H_
#define ATOMIC_H_

#include <stm32f0xx.h>

#define ATOMIC_BLOCK() for(uint32_t __cond = 1, __prim = __get_PRIMASK(); __cond != 0 ? __disable_irq() : 0, __cond != 0; __prim == 0 ? __enable_irq() : 0, __cond = 0)


#endif /* ATOMIC_H_ */
