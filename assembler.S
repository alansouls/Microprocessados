/*
 * assembler.S
 *
 *  Created on: 23 de ago de 2018
 *      Author: alan
 */

.syntax unified
.text
.macro ret
    bx lr
.endm

.globl add
.globl fibonacci
.globl fatorial
.globl soma
.globl bublesort
.globl SetBit

# perceba a passagem de parâmetros através de r0 e r1,
# bem como o retorno através de r0
# seguindo o c calling convention
add:
  add r0, r0, r1
ret

fibonacci:
	subs r5,r0,r0
	subs r1,r0,r0
	subs r2,r0,r0
	adds r2,#1
	adds r0,#0
	BEQ RETORNA
	subs r3,r0,r2
	BEQ RETORNA
	adds r5,#1
	subs r3,r3,r3
	B LOOPFIB
	RETORNA: ret
	LOOPFIB:
		adds r3,r1,r2
		subs r1,r1,r1
		adds r1,r1,r2
		subs r2,r2,r2
		adds r2,r2,r3
		adds r5,#1
		subs r3,r5,r0
		BNE LOOPFIB
		subs r0,r0,r0
		adds r0,r0,r2
		B RETORNA

fatorial:
	adds r0,#0
	BEQ RETORNA_1
	subs r5,r5,r5
	adds r5,r5,r0
	subs r0,r0,r0
	adds r0,#1
	B LOOPFAT
	RETORNA_1:
		subs r0,r0,r0
		adds r0,#1
		B RETORNA
	LOOPFAT:
		muls r0,r5,r0
		subs r5,#1
		adds r5,#0
		BNE LOOPFAT
		B RETORNA

soma:
	adds r2,#0
	BEQ RETORNA
	LOOPSOMA:
		ldr r3,[r0]
		ldr r4,[r1]
		adds r3,r3,r4
		str r3,[r0]
		subs r2,#1
		BEQ RETORNA
		adds r0,#4
		adds r1,#4
		B LOOPSOMA

bublesort:
	adds r2,#0
	BEQ RETORNA
	subs r7,r7,r7
	LOOPCOPIARARRAY:
		ldr r3,[r0]
		str r3,[r1]
		adds r7,#1
		adds r0,#4
		adds r1,#4
		subs r3,r7,r2
		BNE LOOPCOPIARARRAY


	subs r7,r7,r7
	adds r7,#4
	muls r7,r7,r2
	subs r0,r0,r7
	subs r1,r1,r7
	LOOPBUBLE_0:
		subs r5,r5,r5
		subs r6,r2,r2
		adds r6,r2,r6
		LOOPBUBLE_1:
			ldr r3,[r1]
			ldr r4,[r1,4]
			subs r7,r7,r7
			subs r7,r3,r4
			BMI PULATROCA
			str r3,[r1,4]
			str r4,[r1]
			subs r5,r5,r5
			adds r5,#1
			B ENDLOOPBUBLE_1
			PULATROCA:
			str r3,[r1]
			str r4,[r1,4]
			ENDLOOPBUBLE_1:
			subs r6,#2
			BEQ ENDLOOPBUBLE_0
			adds r6,#1
			adds r0,#4
			adds r1,#4
			B LOOPBUBLE_1
		ENDLOOPBUBLE_0:
		subs r7,r7,r7
		adds r7,#4
		muls r7,r7,r2
		subs r7,#8
		subs r0,r0,r7
		subs r1,r1,r7
		adds r5,#0
		BNE LOOPBUBLE_0


	B RETORNA



SetBit:
	subs r2,r2,r2
	adds r2,r2,r1
	BEQ POT0
	subs r1,r1,r1
	adds r1,#1
	subs r3,r3,r3
	adds r3,#2
	LOOPPOTENCIA:
		muls r1,r1,r3
		subs r2,#1
		BNE LOOPPOTENCIA
	FIMSETBIT:
		ldr r2,[r0]
		orrs r2,r1
		str r2,[r0]
		B RETORNA
	POT0:
		subs r1,r1,r1
		adds r1,#1
		B FIMSETBIT








