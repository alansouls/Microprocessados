/*
 * ExcMyStrCpy.S
 *
 *  Created on: 3 de set de 2018
 *      Author: alan
 */

.syntax unified
.text
.macro ret
    bx lr
.endm

.globl mystrcpy

mystrcpy:
	LOOP:
		ldrb r3,[r1]
		strb r3,[r0]
		adds r0,#1
		adds r1,#1
		adds r3,#0
		BEQ VER
		B LOOP
FIM:
ret
VER:
ldrb r3,[r0]
adds r3,#0
BEQ FIM





