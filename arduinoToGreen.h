/*
 * arduinoToGreen.h
 *
 *  Created on: 22 de set de 2018
 *      Author: alan
 */

#ifndef ARDUINOTOGREEN_H_
#define ARDUINOTOGREEN_H_

void delay (unsigned long ms); // Implementar em C usando a HAL
void digitalWrite (int pino, int valor); // Implementar em C sem usar a HAL
void pinMode     (int pino, int modo); // Implementar em C usando a HAL
int    digitalRead (int pino); // Implementar em ASM


#endif /* ARDUINOTOGREEN_H_ */
