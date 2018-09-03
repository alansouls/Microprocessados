/*
 * exc2.S
 *
 *  Created on: 3 de set de 2018
 *      Author: alan
 */

.syntax unified
.text
.macro ret
    bx lr
.endm

.globl mymemset

mymemset:
	adds r2,#0
	BEQ FIM
	subs r3,r3,r3
	adds r3,r3,r0
	LOOP:
	strb r1,[r3]
	asrs r1,r1,8
	adds r3,#1
	subs r2,#1
	BNE LOOP
FIM:
ret
