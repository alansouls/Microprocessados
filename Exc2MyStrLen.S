/*
 * Exc2MyStrLen.S
 *
 *  Created on: 3 de set de 2018
 *      Author: alan
 */

.syntax unified
.text
.macro ret
    bx lr
.endm

.globl mystrlen

mystrlen:
	subs r2,r2,r2
	subs r2,#1
	LOOP:
		ldrb r1,[r0]
		adds r0,#1
		adds r2,#1
		adds r1,#0
		BNE LOOP
subs r0,r0,r0
adds r0,r0,r2
ret










