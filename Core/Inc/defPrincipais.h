#ifndef DEFPRINCIPAIS_H_
#define DEFPRINCIPAIS_H_

// Definição para uso de atraso no FreeRTOS
#define MS(tempo)		tempo/portTICK_RATE_MS

//Definições de macros para o trabalho com bits

#define bit(x)			1<<x

#define	set_bit(y,bit)	(y|=(1<<bit))	//coloca em 1 o bit x da variável Y
#define	clr_bit(y,bit)	(y&=~(1<<bit))	//coloca em 0 o bit x da variável Y
#define cpl_bit(y,bit) 	(y^=(1<<bit))	//troca o estado lógico do bit x da variável Y
#define tst_bit(y,bit) 	(y&(1<<bit))	//retorna 0 ou 1 conforme leitura do bit

#endif /* DEFPRINCIPAIS_H_ */
