#include "NOKIA5110_fb.h"

/*****************************************************************************
	C�digo desenvolvido para ser empregado com um RTOS

	Qualquer altera��o ser� sempre feita no frame buffer,
	sendo o frame buffer impresso periodicamento em uma tarefa.		

******************************************************************************/

//This table contains the hex values that represent pixels for a font that is 5 pixels wide and 8 pixels high
static const unsigned char ASCII[][5] = {
	{0x00, 0x00, 0x00, 0x00, 0x00}  // 20
	,{0x00, 0x00, 0x5f, 0x00, 0x00} // 21 !
	,{0x00, 0x07, 0x00, 0x07, 0x00} // 22 "
	,{0x14, 0x7f, 0x14, 0x7f, 0x14} // 23 #
	,{0x24, 0x2a, 0x7f, 0x2a, 0x12} // 24 $
	,{0x23, 0x13, 0x08, 0x64, 0x62} // 25 %
	,{0x36, 0x49, 0x55, 0x22, 0x50} // 26 &
	,{0x00, 0x05, 0x03, 0x00, 0x00} // 27 '
	,{0x00, 0x1c, 0x22, 0x41, 0x00} // 28 (
	,{0x00, 0x41, 0x22, 0x1c, 0x00} // 29 )
	,{0x14, 0x08, 0x3e, 0x08, 0x14} // 2a *
	,{0x08, 0x08, 0x3e, 0x08, 0x08} // 2b +
	,{0x00, 0x50, 0x30, 0x00, 0x00} // 2c ,
	,{0x08, 0x08, 0x08, 0x08, 0x08} // 2d -
	,{0x00, 0x60, 0x60, 0x00, 0x00} // 2e .
	,{0x20, 0x10, 0x08, 0x04, 0x02} // 2f /
	,{0x3e, 0x51, 0x49, 0x45, 0x3e} // 30 0
	,{0x00, 0x42, 0x7f, 0x40, 0x00} // 31 1
	,{0x42, 0x61, 0x51, 0x49, 0x46} // 32 2
	,{0x21, 0x41, 0x45, 0x4b, 0x31} // 33 3
	,{0x18, 0x14, 0x12, 0x7f, 0x10} // 34 4
	,{0x27, 0x45, 0x45, 0x45, 0x39} // 35 5
	,{0x3c, 0x4a, 0x49, 0x49, 0x30} // 36 6
	,{0x01, 0x71, 0x09, 0x05, 0x03} // 37 7
	,{0x36, 0x49, 0x49, 0x49, 0x36} // 38 8
	,{0x06, 0x49, 0x49, 0x29, 0x1e} // 39 9
	,{0x00, 0x36, 0x36, 0x00, 0x00} // 3a :
	,{0x00, 0x56, 0x36, 0x00, 0x00} // 3b ;
	,{0x08, 0x14, 0x22, 0x41, 0x00} // 3c <
	,{0x14, 0x14, 0x14, 0x14, 0x14} // 3d =
	,{0x00, 0x41, 0x22, 0x14, 0x08} // 3e >
	,{0x02, 0x01, 0x51, 0x09, 0x06} // 3f ?
	,{0x32, 0x49, 0x79, 0x41, 0x3e} // 40 @
	,{0x7e, 0x11, 0x11, 0x11, 0x7e} // 41 A
	,{0x7f, 0x49, 0x49, 0x49, 0x36} // 42 B
	,{0x3e, 0x41, 0x41, 0x41, 0x22} // 43 C
	,{0x7f, 0x41, 0x41, 0x22, 0x1c} // 44 D
	,{0x7f, 0x49, 0x49, 0x49, 0x41} // 45 E
	,{0x7f, 0x09, 0x09, 0x09, 0x01} // 46 F
	,{0x3e, 0x41, 0x49, 0x49, 0x7a} // 47 G
	,{0x7f, 0x08, 0x08, 0x08, 0x7f} // 48 H
	,{0x00, 0x41, 0x7f, 0x41, 0x00} // 49 I
	,{0x20, 0x40, 0x41, 0x3f, 0x01} // 4a J
	,{0x7f, 0x08, 0x14, 0x22, 0x41} // 4b K
	,{0x7f, 0x40, 0x40, 0x40, 0x40} // 4c L
	,{0x7f, 0x02, 0x0c, 0x02, 0x7f} // 4d M
	,{0x7f, 0x04, 0x08, 0x10, 0x7f} // 4e N
	,{0x3e, 0x41, 0x41, 0x41, 0x3e} // 4f O
	,{0x7f, 0x09, 0x09, 0x09, 0x06} // 50 P
	,{0x3e, 0x41, 0x51, 0x21, 0x5e} // 51 Q
	,{0x7f, 0x09, 0x19, 0x29, 0x46} // 52 R
	,{0x46, 0x49, 0x49, 0x49, 0x31} // 53 S
	,{0x01, 0x01, 0x7f, 0x01, 0x01} // 54 T
	,{0x3f, 0x40, 0x40, 0x40, 0x3f} // 55 U
	,{0x1f, 0x20, 0x40, 0x20, 0x1f} // 56 V
	,{0x3f, 0x40, 0x38, 0x40, 0x3f} // 57 W
	,{0x63, 0x14, 0x08, 0x14, 0x63} // 58 X
	,{0x07, 0x08, 0x70, 0x08, 0x07} // 59 Y
	,{0x61, 0x51, 0x49, 0x45, 0x43} // 5a Z
	,{0x00, 0x7f, 0x41, 0x41, 0x00} // 5b [
	,{0x02, 0x04, 0x08, 0x10, 0x20} // 5c barra para esquerda
	,{0x00, 0x41, 0x41, 0x7f, 0x00} // 5d ]
	,{0x04, 0x02, 0x01, 0x02, 0x04} // 5e ^
	,{0x40, 0x40, 0x40, 0x40, 0x40} // 5f _
	,{0x00, 0x01, 0x02, 0x04, 0x00} // 60 `
	,{0x20, 0x54, 0x54, 0x54, 0x78} // 61 a
	,{0x7f, 0x48, 0x44, 0x44, 0x38} // 62 b
	,{0x38, 0x44, 0x44, 0x44, 0x20} // 63 c
	,{0x38, 0x44, 0x44, 0x48, 0x7f} // 64 d
	,{0x38, 0x54, 0x54, 0x54, 0x18} // 65 e
	,{0x08, 0x7e, 0x09, 0x01, 0x02} // 66 f
	,{0x0c, 0x52, 0x52, 0x52, 0x3e} // 67 g
	,{0x7f, 0x08, 0x04, 0x04, 0x78} // 68 h
	,{0x00, 0x44, 0x7d, 0x40, 0x00} // 69 i
	,{0x20, 0x40, 0x44, 0x3d, 0x00} // 6a j
	,{0x7f, 0x10, 0x28, 0x44, 0x00} // 6b k
	,{0x00, 0x41, 0x7f, 0x40, 0x00} // 6c l
	,{0x7c, 0x04, 0x18, 0x04, 0x78} // 6d m
	,{0x7c, 0x08, 0x04, 0x04, 0x78} // 6e n
	,{0x38, 0x44, 0x44, 0x44, 0x38} // 6f o
	,{0x7c, 0x14, 0x14, 0x14, 0x08} // 70 p
	,{0x08, 0x14, 0x14, 0x18, 0x7c} // 71 q
	,{0x7c, 0x08, 0x04, 0x04, 0x08} // 72 r
	,{0x48, 0x54, 0x54, 0x54, 0x20} // 73 s
	,{0x04, 0x3f, 0x44, 0x40, 0x20} // 74 t
	,{0x3c, 0x40, 0x40, 0x20, 0x7c} // 75 u
	,{0x1c, 0x20, 0x40, 0x20, 0x1c} // 76 v
	,{0x3c, 0x40, 0x30, 0x40, 0x3c} // 77 w
	,{0x44, 0x28, 0x10, 0x28, 0x44} // 78 x
	,{0x0c, 0x50, 0x50, 0x50, 0x3c} // 79 y
	,{0x44, 0x64, 0x54, 0x4c, 0x44} // 7a z
	,{0x00, 0x08, 0x36, 0x41, 0x00} // 7b {
	,{0x00, 0x00, 0x7f, 0x00, 0x00} // 7c |
	,{0x00, 0x41, 0x36, 0x08, 0x00} // 7d }
	,{0x10, 0x08, 0x08, 0x10, 0x08} // 7e ~
	,{0x78, 0x46, 0x41, 0x46, 0x78} // 7f DEL
};
//----------------------------------------------------------------------------------------------
const unsigned char nr_peq [] = {
	0x00, 0x1F, 0x11, 0x1F, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x19, 0x15, 0x12, 0x00, 0x11, 0x15, 0x1F,
	0x00, 0x07, 0x04, 0x1F, 0x00, 0x17, 0x15, 0x1D, 0x00, 0x1F, 0x15, 0x1D, 0x00, 0x19, 0x05, 0x03,
	0x00, 0x1F, 0x15, 0x1F, 0x00, 0x17, 0x15, 0x1F, 0x00, 0x04, 0x04, 0x04
};  // 0 1 2 3 4 5 6 7 8 9 -
//----------------------------------------------------------------------------------------------

// Espelho da tela - FRAME BUFFER - necess�rio na escrita p/ alterar pixels sem alterar os demais dentro de um byte
static unsigned char fb[504];		// 84 colunas X 6 bancos (1 byte) =  504 bytes um frame para o LCD
static uint32_t indice_fb;			// indice para uso no fb

//----------------------------------------------------------------------------------------------
/*	There are two memory banks in the LCD, data/RAM and commands. This
	function sets the DC pin high or low depending, and then sends the data byte */
//----------------------------------------------------------------------------------------------
void cmd_LCD(uint32_t data) 
{
	uint32_t i=8;
	
	CLR_DC();					// comando para o LCD

	//Send the data
	CLR_CE();
	do								// MSB primeiro
	{	i--;
		if(tst_bit(data,i))
			SET_DIN();
		else
			CLR_DIN();
		PULSO_CLK();
		
	}while(i!=0);
	
	SET_CE();
}
//----------------------------------------------------------------------------------------------
void data_LCD(uint32_t data)
{
	uint32_t i=8;

	SET_DC();					// dado para o LCD

	//Send the data
	CLR_CE();
	do								// MSB primeiro
	{	i--;
		if(tst_bit(data,i))
			SET_DIN();
		else
			CLR_DIN();
		PULSO_CLK();

	}while(i!=0);

	SET_CE();
}
//-----------------------------------------------------------------------------------------------
void inic_LCD() 
{	
	// Reset the LCD to a known state
	CLR_RST();
	atraso_us(10);
	SET_RST();

	cmd_LCD(0x21);			//Tell LCD that extended commands follow
	cmd_LCD(0xBF);			//Set LCD Vop (Contrast): Try 0xB1(good @ 3.3V) or 0xBF if your display is too dark
	cmd_LCD(0x04);			//Set Temp coefficent
	cmd_LCD(0x14);			//LCD bias mode 1:48: Try 0x13 or 0x14

	cmd_LCD(0x20);			//We must send 0x20 before modifying the display control mode
	cmd_LCD(0x0C);			//Set display control, normal mode. 0x0D for inverse
}
//----------------------------------------------------------------------------------------------- 
void goto_XY(uint32_t x, uint32_t y)  // 0<=x<=83  0<=y<=5
{
	if(y>5)	y=5;
	if(x>84)	x=83;
  
	indice_fb =  x + (84*y);		// indice para ser empregado no fb
}

//----------------------------------------------------------------------------------------------- 
void escreve2fb(unsigned char imagem[])				// altera o frame buffer, uso para c�pia de figura
{
	uint32_t i;

	for(i=0; i<504; i++)
		fb[i]= imagem[i];
}
//-----------------------------------------------------------------------------------------------
void imprime_LCD()		// desenha em todo o LCD
{
	uint32_t i;
	
	for (i=0 ; i < 504 ; i++)
		data_LCD(fb[i]);
}
//----------------------------------------------------------------------------------------------- 
/* This function takes in a character, looks it up in the font table/array and writes it to the screen
 * each character is 8 bits tall and 5 bits wide. We pad one blank column of pixels on each side of 
 * the character for readability.																	
 * Os caracteres s� podem ser escritos na linha correspondente ao banco	(0-5)							*/
void caractere_LCD(char character) 
{
	uint32_t i;
	
	fb[indice_fb] = 0x00;		//Blank vertical line padding
	indice_fb++;

	for( i= 0 ; i < 5 ; i++)
	{
		fb[indice_fb] = ASCII[character - 0x20][i];//0x20 is the ASCII character for Space (' '). The font table starts with this character
		indice_fb++;
	}
	
	fb[indice_fb] = 0x00; //Blank vertical line padding
}
//----------------------------------------------------------------------------------------------- 
void string_LCD(char *msg)
{
	while(*msg)
		caractere_LCD(*msg++);
}
//----------------------------------------------------------------------------------------------- 
void string_LCD_Nr(char * msg, uint32_t valor,			// escreve uma mensagem com um n�mero
								uint32_t quant2Print)	// quant2Print � o n�mero de digitos para serem impressos
{
	uint32_t n=0, i;
	unsigned char digitos[10];	// m�ximo de 10 digitos

	for(i=0; i<10; i++)
		digitos[i]= ' ';

	string_LCD(msg);

	do
	{
		digitos[n] = ((valor%10) + 48);	//pega o resto da divisao por 10
		valor /=10;						//pega o inteiro da divis�o por 10
		n++;

	}while (valor!=0);

	if(quant2Print != 0)
	{
		n = quant2Print;
		do
		{
			caractere_LCD(digitos[n-1]);
			n--;
		} while (n!=0);
	}

}
//-----------------------------------------------------------------------------------------------
//Clears the LCD by writing zeros to the entire screen
void limpa_LCD() 
{
	uint32_t i;
	
	for ( i= 0 ; i < 504 ; i++)
		fb[i] = 0x00;
}
//----------------------------------------------------------------------------------------------- 
// Desenha pixel 
//----------------------------------------------------------------------------------------------- 
void desenha_pixel(uint32_t x,				/* ponto horizontal para o pixel: 0 -> 83 (esq -> dir)	*/
				   uint32_t y,				/* ponto vertical para o pixel: 0 -> 47 (cima -> baixo)	*/
				   uint32_t propr)			/* 0 =  apaga pixel, 1 = liga pixel				*/
{
	uint32_t i;
	
	if(x>83)	x=83;
	if(y>47)	y=47;
	
	i = x + (84*(y/8));		/* determina��o do indice do byte a ser alterado [0 - 503]	*/
	
	if(propr==0)
		clr_bit(fb[i],y%8);
	else
		set_bit(fb[i],y%8);
	
}
//-----------------------------------------------------------------------------------------------
// Desenha linha - ALGORITMO DE BRESENHAM - (http://www.etechplanet.com/codesnippets/computer-graphics-draw-a-line-using-bresenham-algorithm.aspx)
//-----------------------------------------------------------------------------------------------
void desenha_linha(struct  pontos_t *p,		/*  p.x1=x1, p.y1=y1, p.x2=x2, p.y2=y2, passagem dos pontos por struct	*/
						   uint32_t prop)	/* 0 =  paga pixel, 1 = liga pixel				*/
{
	uint32_t i, x, y, x1, y1, x2, y2;
	int32_t dx, dy, dx1, dy1, px, py, xe, ye;

	x1 = p->x1;
	y1 = p->y1;
	x2 = p->x2;
	y2 = p->y2;
	
	dx = x2-x1;
	dy = y2-y1;
	dx1 = abs(dx);
	dy1 = abs(dy);
	px = 2*dy1 - dx1;
	py = 2*dx1 - dy1;
		
	if(dy1 <= dx1)
	{	if(dx >= 0)
		{	x = x1;
			y = y1;
			xe = x2;
		}
		else
		{	x = x2;
			y = y2;
			xe = x1;
		}
		desenha_pixel(x,y,prop);
		for(i=0; x<xe; i++)
		{
			x++;
			if(px<0)
				px = px + 2*dy1;
			else
			{
				if((dx<0 && dy<0) || (dx>0 && dy>0))
					y++;
				else
					y--;
				px = px + 2*(dy1-dx1);
			}
			desenha_pixel(x,y,prop);
		}
	}
	else
	{
		if(dy >= 0)
		{	x=x1;
			y=y1;
			ye=y2;
		}
		else
		{	x=x2;
			y=y2;
			ye=y1;
		}
		desenha_pixel(x,y,prop);
		for(i=0;y<ye;i++)
		{
			y++;
			if(py <= 0)
				py = py + 2*dx1;
			else
			{
				if((dx<0 && dy<0) || (dx>0 && dy>0))
					x++;
				else
					x--;
				py = py + 2*(dx1-dy1);
			}
			desenha_pixel(x,y,prop);
		}
	}
}
//--------------------------------------------------------------------------------------------------------------
// Desenha Circulo - Algoritmo de Ponto M�dio http://rosettacode.org/wiki/Bitmap/Midpoint_circle_algorithm#C
//--------------------------------------------------------------------------------------------------------------
void desenha_circulo(int32_t x0, int32_t y0,int32_t radius,	// valores int se fazem necess�rio devido as compara��es
											 uint32_t prop) // 0 =  paga pixel, 1 = liga pixel				
{
	  int f, ddF_x, ddF_y, x, y;
	  
	  if(x0>83)	x0=83;
	  if(y0>47) y0=47;
	  
	  f = 1 - radius;
	  ddF_x = 0;
	  ddF_y = -2 * radius;
	  x = 0;
	  y = radius;
	  
	  // evita a impress�o fora do dom�nio de x e y (erros de impress�o)
	  if((y0 + radius) < 48)		desenha_pixel(x0, y0 + radius,prop);
	  if((y0 - radius) >= 0)		desenha_pixel(x0, y0 - radius,prop);
	  if((x0 + radius) < 84)		desenha_pixel(x0 + radius, y0,prop);	  
	  if((x0 - radius) >= 0)		desenha_pixel(x0 - radius, y0,prop);
	  
	  while(x < y)
	  {
		  if(f >= 0)
		  {
			  y--;
			  ddF_y += 2;
			  f += ddF_y;
		  }
		  x++;
		  ddF_x += 2;
		  f += ddF_x + 1;
		  
		  // evita a impress�o fora do dom�nio de x e y (erros de impress�o)
		  if(((x0+x) <84) && ((y0+y) < 48))		desenha_pixel(x0 + x, y0 + y,1);
		  if(((x0-x) >=0) && ((y0+y) < 48))		desenha_pixel(x0 - x, y0 + y,1);
		  if(((x0+x) <84) && ((y0-y) >=0))		desenha_pixel(x0 + x, y0 - y,1);
		  if(((x0-x) >=0) && ((y0-y) >=0))		desenha_pixel(x0 - x, y0 - y,1);
		  if(((x0+y) <84) && ((y0+x) < 48))		desenha_pixel(x0 + y, y0 + x,1);
		  if(((x0-y) >=0) && ((y0+x) < 48))		desenha_pixel(x0 - y, y0 + x,1);
		  if(((x0+y) <84) && ((y0-x) >=0))		desenha_pixel(x0 + y, y0 - x,1);
		  if(((x0-y) >=0) && ((y0-x) >=0))		desenha_pixel(x0 - y, y0 - x,1);
	  }
}
//--------------------------------------------------------------------------------------------------------------
/*	Funcao para desenho de um ret�ngulo
*
*		p1-----------
*		|			|
*		|	
*		------------p2
*/
void desenha_retangulo(struct  pontos_t *p,	uint32_t prop)	/*  p.x1=x1, p.y1=y1, p.x2=x2, p.y2=y2, passagem dos pontos por struct	*/
															/*  ponto superior esquerdo e ponto inferior direito					*/
															/* 0 =  apaga pixel, 1 = liga pixel, 									*/
															/* 2 = preenchimento pixel apagado, 3 = preenchimento pixel ligado		*/
{	struct pontos_t pr;
	uint32_t pxl, i;
			
	if(prop < 2)								// desenho com quatro linhas
	{
		pr.x1 = p->x1; pr.y1 = p->y1;
		pr.x2 = p->x1; pr.y2 = p->y2;
		desenha_linha(&pr,prop);
		
		pr.x2 = p->x2; pr.y2 = p->y1;
		desenha_linha(&pr,prop);
		
		pr.x1 = p->x2; pr.y1 = p->y2;
		desenha_linha(&pr,prop);
		
		pr.x2 = p->x1; pr.y2 = p->y2;
		desenha_linha(&pr,prop);
	}
	else										// preenchimento desenhando v�rias linhas
	{
		if(prop == 2)							// propriedade para ligar ou apagar os pixeis
			pxl = 0;
		else
			pxl = 1;
		
		pr.y1 = p->y1;
		pr.y2 = p->y2;
		
		for(i = p->x1; i <= p->x2; i++)			// desenha as linhas verticalmente
		{
			pr.x1 = i;
			pr.x2 = i;
			desenha_linha(&pr,pxl);
		}
	}
}
//--------------------------------------------------------------------------------------------------------------	
/*	Funcao para desenho de um tri�ngulo - N�o tem preenchimento
*
*			  p2
*			/	\		
*		  /		 \
*		p1--------p3
*/
void desenha_triangulo(struct  pontos_t *p,		/*  p.x1=x1, p.y1=y1, p.x2=x2, p.y2=y2, , p.x3=x3, p.y3=y3				*/
												/*  ponto superior esquerdo e ponto inferior direito					*/
								uint32_t prop)	/* 0 =  apaga pixel, 1 = liga pixel,									*/
{	struct pontos_t pt;
	
	pt.x1 = p->x1; pt.y1 = p->y1;
	pt.x2 = p->x2; pt.y2 = p->y2;
	desenha_linha(&pt,prop);
	
	pt.x2 = p->x3; pt.y2 = p->y3;
	desenha_linha(&pt,prop);
	
	pt.x1 = p->x2; pt.y1 = p->y2;
	desenha_linha(&pt,prop);	
	
}
//--------------------------------------------------------------------------------------------------------------
/* FUN��O PARA DESENHO DE UMA FIGURA em qualquer ponto da tela e algumas partes da figura
*
*	Usando as propriedades da figura (largura e altura) - [p.x2 = 0, p.y2 = 0, p.x3 = 0  e p.y3 = 0]
*
*	 P1 --------------
*    |xxxxxxxxxxxxxxxx|
*	 |xxxxxxxxxxxxxxxx|
*	  ----------------
*
*	Redefinindo o tamanho da figura (menor) - [ p.x2 != 0 e p.y2 !=0] - canto superior esquerdo
*		
*     ___p.x2___
*	 P1 --------------
*    |xxxxxxxxxx|	  |    | p.y2
*	 |----------      |
*	 |________________|
*
*	Imprimindo a partir de um ponto no meio da figura [ p.x2 == 0 ou p.y2 == 0 com p.x3 != 0 e p.y3 != 0]
*   Agora ser� impresso a partir do ponto P1 a figura delimitada por p.x3 e p.y3 - canto inferior direito
*
*	      ____p.x3_____
*	  P1---------------
*	  |   _____________|
*	  |	 |xxxxxxxxxxxxx|    | p.y3
*      ---------------- 
*

*		figura[nr_linhas][nr colunas], onde cada elemento � composto por 1 byte de informa��o
*									   cada bit � um pixel (0 ou 1)
*/
void desenha_fig(struct  pontos_t *p,		// [p.x1 p.y1] para o ponto P1 de incersao da figura
											// Caso se deseje alterar a largura e altura da figura
											//    p.x2  e  p.y2 devem ser diferentes de zero, determinando as novas medidas
											// Caso se deseje imprimir a partir de um ponto no meio da figura (se n�o empregados devem ser zero)
											//    p.x3 e p.y3 determinam esse ponto
				const struct figura_t *f)	// ponteiro para a figura definido pelo tipo figura_t
{
	uint32_t x, y, larg, alt, i, j, ajx, ajy;
	
	x = p->x1;
	y = p->y1;
	
	// corrige limites para P1
	if(x > 83)  x = 83;
	if(y > 47)  y = 47;
	
	larg = f->largura;
	alt = f->altura;
	
	// caso se deseje outra dimensao de impressao da figura que n�o a definida por padrao
	if((p->x2!=0) && (p->y2!=0)) 
	{
		larg = p->x2;
		alt = p->y2;
	}
	else if((p->x3!=0) && (p->y3!=0)) // correcao se valor maior que o permitido
	{
		larg = p->x3;
		alt = p->y3;
	}
	// eventual correcao para a altura e largura
	if(larg > f->largura)
		larg = f->largura;
	if(alt > f->altura)
		alt = f->altura;		
		
	//corrigir dimensoes da figura para dentro da �rea de impressao
	if((x+larg)>84)
		larg = 84 - x;
	if((y+alt)>48)
		alt = 48 - y;
	
	//------------------------------------------------------------------------------------------
	if((p->x3!=0) && (p->y3!=0))
	{
		ajx = f->largura - larg;
		ajy = f->altura - alt;
		
		for(j=0; j<alt; j++)
		{
			for(i=0; i<larg; i++) // canto inferior direito
			{
				desenha_pixel(x+i, y+j, tst_bit(f->pixels[i + ajx + (((j+ajy)/8)*f->largura)], (j+ajy)%8));
			}
		}
	}
	else// impressao de figura normal ou menor (com p.x2!=0 e p.y2!=0)
	{
		for(j=0; j<alt; j++)
		{
			for(i=0; i<larg; i++)
			{
				desenha_pixel(x+i, y+j, tst_bit(f->pixels[i + ((j/8)*f->largura)], j%8));
			}
		}
	}
	//------------------------------------------------------------------------------------------	
}
//----------------------------------------------------------------------------------------------
void escreve_Nr_Peq(uint32_t x, uint32_t y, int32_t valor, uint32_t quant2Print) // quant2Print = 0, imprime todos os digitos
{
	uint32_t n=0, i, j, px=0, neg=0;
	unsigned char digitos[11];	// m�ximo de 10 digitos com um digito de sinal

	for(i=0; i<11; i++)
		digitos[i] = ' ';
		
	if(valor<0)
	{
		neg=1;
		valor = valor*-1;
	}

	do
	{
		digitos[n] = valor%10;	//pega o resto da divisao por 10
		valor /=10;						//pega o inteiro da divis�o por 10
		n++;
		
	}while (valor!=0);
	
	if(neg!=0)
	{
		digitos[n] = '-';	// sinal de menos
		n++;
		
	}
	
	if(quant2Print != 0)
		n = quant2Print;
	
	do
	{
		for(i=0; i<4; i++)		// largura
		{
			for(j=0; j<5; j++)	// altura
			{
				if(digitos[n-1] == '-')
					desenha_pixel((x+i) + (4*px) , y + j,  tst_bit(nr_peq[i + 40], j));
				else if(digitos[n-1] != ' ')
					desenha_pixel((x+i) + (4*px) , y + j,  tst_bit(nr_peq[i + (4*digitos[n-1])], j));
				else
					desenha_pixel((x+i) + (4*px) , y + j,  0);
			}
		} 
		px++;
		n--;
	} while (n!=0);
}
//----------------------------------------------------------------------------------------------


