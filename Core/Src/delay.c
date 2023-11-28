/*
 * delay.c
 *
 *  Created on: 24.06.2020
 *      Author: zihlmalb
 */
#include "delay.h"

// This function waits for 1/fcpu * ( N * 4 + 8 )
// Overhead: 4 cylcles for call with bl, 3 cycles
//           for return with bx, 1 cycle bne
// Loop: 1 cycle sub, 3 cylces bne
__attribute__((naked))
void delay(uint32_t us){
	asm(
			".syntax unified\n"
			"delay_loop:\n"
			"subs r0,#1\n"  	// 1 cycle
			"bne delay_loop\n" 	// 3 cycles when branch else 1
			"bx lr\n"           // 3 cylces
	);
}
