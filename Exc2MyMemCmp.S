/*
 * Exc2MyMemCmp.S
 *
 *  Created on: 3 de set de 2018
 *      Author: alan
 */

.syntax unified
.text
.macro ret
    bx lr
.endm

.globl mymemcmp

mymemcmp:
#IF RETURNS A POSITIVE NUMBER, THE FIRST IS GREATER THAN THE SECOND.
	push {r4-r5}
	subs r5,r5,r5
	adds r2,#0
	BEQ FIM
	LOOP:
		ldrb r3,[r0]
		ldrb r4,[r1]
		adds r0,#1
		adds r1,#1
		subs r5,r3,r4
		BNE FIM
		subs r2,#1
		BNE LOOP
FIM:
subs r0,r3,r4
pop {r4-r5}
ret





