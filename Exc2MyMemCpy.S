.syntax unified
.text
.macro ret
    bx lr
.endm

.globl mymemcpy


mymemcpy:
	adds r2,#0
	BEQ FIM
	LOOP:
		ldrb r3,[r1]
		strb r3,[r0]
		adds r0,#1
		adds r1,#1
		subs r2,#1
		BNE LOOP
FIM:
ret








