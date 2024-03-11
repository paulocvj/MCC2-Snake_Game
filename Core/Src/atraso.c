/****************************************************************************************

  O VALOR DO ATRASO É APROXIMADO.
  
  A incialização do valor de repetição do laço é desconsiderada no cômputo do atraso,
  bem como a chamada e retorno da função.
  
  Este código foi aferido com o uso de um osciloscópio.
  
  Autor: Charles Borges de Lima
  Data: 06 abril de 2015
  V. 1.1
 
****************************************************************************************/ 

#include "atraso.h"

//-----------------------------------------------------------------------------------------
void atraso_us(uint32_t valor)									//argumento passado pelo registrador r0
{
	asm volatile (  "movw r1, #:lower16:const_us	\n\t"
					"movt r1, #:upper16:const_us	\n\t"		//r1 = fcpu/3000000 (const_us)
					"mul  r0, r0, r1				\n\t"	    //r0 = valor*(fcpu/3000000), cálculo do número de repetições do laço
	
					"r_us:	   subs r0, r0, #1		\n\t"
					"bne  r_us						\n\t");		//2 ciclos gastos quando tomado, 1 contrário
}
//-----------------------------------------------------------------------------------------
 void atraso_ms(uint32_t valor)									//argumento passado pelo registrador r0
{
	asm volatile ("movw r1, #:lower16:const_ms		\n\t"
				  "movt r1, #:upper16:const_ms		\n\t"		//r1 = fcpu/3000 (const_ms)
				  "mul  r0, r0, r1					\n\t"		//r0 = valor*(fcpu/3000), cálculo do número de repetições do laço
	
				  "r_ms:	   subs r0, r0, #1		\n\t"
				  "bne  r_ms						\n\t");
	
}
//-----------------------------------------------------------------------------------------
void atraso_loops(uint32_t valor)	//argumento passado pelo registrador r0
{
	asm volatile ("r_loops:	 subs r0, r0, #1 \n\t"
							"bne  r_loops	 \n\t");
}
//-----------------------------------------------------------------------------------------
