#ifndef ATRASO_H_
#define ATRASO_H_

#include <stdint.h>

asm (".equ fcpu, 72000000/2 \n\t");				//define a frequencia para o uso em código assembly
asm (".equ const_ms, fcpu/3000		 \n\t");	//constante para o atraso_ms
asm (".equ const_us, fcpu/3000000	 \n\t");	//constante para o atraso_us

/*	Colocar o argumento igual a ZERO resultará no máximo atraso possível para a função.
	Não existe teste contra zero, visto que é um valor ilógico para a função	*/


//Valores máximos considerando uma máxima frequencia de trabalho de 84 MHz
//Valores fora da faixa permitida gerarão valores irregulares de atrasos.

//VALOR MÁXIMO DE 153s

void atraso_us(uint32_t valor);		//valor máximo de 153.000.000
void atraso_ms(uint32_t valor);		//valor máximo de 153.000
void atraso_loops(uint32_t valor);  //valor <= 4.294.967.296  (2^32) // repetição de 3 ciclos de clock por unidade de valor

#endif /* ATRASO_H_ */
