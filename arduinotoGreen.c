/*
 * arduinoToGreen.c
 *
 *  Created on: 22 de set de 2018
 *      Author: alan
 */
#include "arduinotoGreen.h"
#include "stm32f0xx_hal.h"


unsigned int equivalencia [20][2]={
        {GPIOA,GPIO_PIN_3},
        {GPIOA,GPIO_PIN_2},
        {GPIOF,GPIO_PIN_0},
        {GPIOF,GPIO_PIN_1},
        {GPIOA,GPIO_PIN_5},
        {GPIOA,GPIO_PIN_0},
		{GPIOA,GPIO_PIN_7},
		{GPIOA,GPIO_PIN_6},
		{GPIOA,GPIO_PIN_1},
		{GPIOA,GPIO_PIN_4},
		{GPIOB,GPIO_PIN_1},
        {0,0},
        {0,0},
		{0,0},
		{0,0},
		{0,0},
        {GPIOA,GPIO_PIN_13},
        {GPIOA,GPIO_PIN_14},
        {GPIOA,GPIO_PIN_10},
        {GPIOA,GPIO_PIN_9}
};


void delay (unsigned long ms){
	HAL_Delay(ms);
}

void digitalWrite (int pino, int valor){
	GPIO_TypeDef *p;
	p = (GPIO_TypeDef *)equivalencia[pino][0];
	if (valor != 0){
		p->BSRR = equivalencia[pino][1];
	}else{
		p->BRR = equivalencia[pino][1];
	}
}

void pinMode     (int pino, int modo){
	 GPIO_InitTypeDef GPIO_InitStruct;

	 /* GPIO Ports Clock Enable */
	 __HAL_RCC_GPIOA_CLK_ENABLE();

	 /*Configure GPIO pin Output Level */
	 if(modo != 0){
		 digitalWrite(pino,0);
	 }
	 /*Configure GPIO pin : PA0 */
	 GPIO_InitStruct.Pin = equivalencia[pino][1];
	 if(modo != 0){
		 GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	 }else {
		 GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	 }

	 GPIO_InitStruct.Pull = GPIO_PULLUP;
	 if(modo != 0){GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;}

	 HAL_GPIO_Init((GPIO_TypeDef *)equivalencia[pino][0], &GPIO_InitStruct);

}

unsigned int asmDigitalRead(unsigned int gpio_, unsigned int pino);

int    digitalRead (int pino){
	unsigned int address;
	address = equivalencia[pino][0];
	return asmDigitalRead(address,equivalencia[pino][1]);
}

