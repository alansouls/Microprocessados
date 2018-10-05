/*
 * digitalRead.asm
 *
 *  Created on: 22 de set de 2018
 *      Author: alan
 */

.syntax unified
.text
.macro ret
    bx lr
.endm

.globl asmDigitalRead

asmDigitalRead:
adds r0,#16
ldr r0,[r0]
ands r0,r1
BNE RETORNA1
ret
RETORNA1:
ldr r0,=0x00000001
ret
