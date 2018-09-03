/*
 * Exc2MyStrCmp.S
 *
 *  Created on: 3 de set de 2018
 *      Author: alan
 */

.syntax unified
.text
.macro ret
    bx lr
.endm

.globl mystrcmp

mystrcmp:
	push {r4-r5}
	subs r5,r5,r5
	LOOP:
		ldrb r3,[r0]
		ldrb r4,[r1]
		adds r0,#1
		adds r1,#1
		subs r5,r3,r4
		BNE FIM
		adds r3,#0
		BEQ FIM
		B LOOP

FIM:
subs r0,r3,r4
pop {r4-r5}
ret
