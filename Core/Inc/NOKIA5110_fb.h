#ifndef NOKIA5110_H_
#define NOKIA5110_H_

#include "stm32f1xx_hal.h"
#include "defPrincipais.h"
#include "atraso.h"
#include <stdlib.h>

//-------------------------------------------------------------------------------------
// RST (A7)
#define SET_RST()		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET)
#define CLR_RST()		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET)

// CE (A6)
#define SET_CE()		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET)
#define CLR_CE()		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET)

// DC (A5)
#define SET_DC()		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET)
#define CLR_DC()		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET)

// DIN (A4)
#define SET_DIN()		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
#define CLR_DIN()		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)

// CLK (A3)
#define SET_CLK()		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET)
#define CLR_CLK()		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET)

//-------------------------------------------------------------------------------------
#define PULSO_CLK() atraso_us(1); SET_CLK(); atraso_us(1); CLR_CLK()

#define INVERTE_PIXELS()		cmd_LCD(0x0D)
#define NORMALIZA_PIXELS()		cmd_LCD(0x0C)
//-------------------------------------------------------------------------------------

struct pontos_t{
	uint32_t x1;
	uint32_t y1;
	uint32_t x2;
	uint32_t y2;
	uint32_t x3;
	uint32_t y3;	
};

struct figura_t
{
	unsigned char largura;
	unsigned char altura;
	unsigned char pixels[];
};


void cmd_LCD(uint32_t data);
void data_LCD(uint32_t data);
void inic_LCD(); 
void goto_XY(uint32_t x, uint32_t y);
void escreve2fb(unsigned char imagem[])	;
void imprime_LCD();
void caractere_LCD(char character);
void string_LCD(char *msg);
void limpa_LCD(); 

// Fun��es do Borges
void string_LCD_Nr(char * msg, uint32_t valor, uint32_t quant2Print);

void desenha_pixel(uint32_t x, uint32_t y, uint32_t propriedade);
void desenha_linha(struct pontos_t *coord, uint32_t prop);
void desenha_circulo(int32_t x0, int32_t y0, int32_t radius, uint32_t prop);
void desenha_retangulo(struct  pontos_t *t, uint32_t prop);
void desenha_triangulo(struct  pontos_t *p, uint32_t prop);
void desenha_fig(struct  pontos_t *p, const struct figura_t *figura);

void escreve_Nr_Peq(uint32_t x, uint32_t y, int32_t valor, uint32_t quant2Print);

//-------------------------------------------------------------------------------------
#endif /* NOKIA5110_H_ */
