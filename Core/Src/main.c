/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "atraso.h"
#include "defPrincipais.h"
#include "NOKIA5110_fb.h"
#include "figuras.h"
#include "PRNG_LFSR.h"
#include "snake_utils.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

//osThreadId defaultTaskHandle;
//uint32_t defaultTaskBuffer[ 128 ];
//osStaticThreadDef_t defaultTaskControlBlock;
/* USER CODE BEGIN PV */
uint32_t ADC_buffer[2];
uint32_t ADC_value[2];

// own code
struct pontos_t fruit;
cobra_t snake_player;

xTaskHandle xTaskFruitHandle;
xTaskHandle xTaskPointsHandle;
xTaskHandle xTaskSnakeHandle;
xTaskHandle xTaskMenuHandle;
xTaskHandle xTaskGameOverHandle;
xTaskHandle xTaskLCDHandle;

diff_t set_difficulty = med;
// ------------------

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if(hadc->Instance == ADC1)
	{
		ADC_value[0]=ADC_buffer[0];
		ADC_value[1]=ADC_buffer[1];
	}
}

//---------------------------------------------------------------------------------------------------
// Tarefa para atualizar periodicamente o LCD
void vTask_LCD_Print(void *pvParameters)
{
	while(1) imprime_LCD();
}

/* Funcao pra gerar uma fruta em um local aleatorio. Utiliza o LFSR pra gerar a aleatoriedade.
 * A multiplicacao e soma em 4 serve para nao passar dos limites de altura e largura da tela.
 * A task é criada via Aumenta_Ponto, que é criada quando tem uma colisao da fruta com a cobra.
 * Após desenhar a fruta, a task é deletada.
 * */
void vTask_Create_Fruit(void *pvParameters)
{
	static uint32_t x, y;
	uint32_t flag;
	do {
		flag = 0;
		x = (prng_LFSR() % 19) * 4 + 4;
		y = (prng_LFSR() % 9) * 4 + 4;
		// gambiarra. Se puder checar o pixel é mais fácil.
		for (uint32_t i = 0; i < snake_player.size; i++) {
			if (x == snake_player.node_x[i] && y == snake_player.node_y[i])
				break;
			else flag++;
		}
	} while (flag != snake_player.size);

	fruit.x1 = x; fruit.y1 = y;
	desenha_fig(&fruit, &drawing_fruit);

	vTaskDelete(xTaskFruitHandle);
}

/* Funcao para aumentar o tamanho da cobra, e por consequencia aumentar a pontuacao do jogador.
 * A task simplesmente cria a task de gerar fruta para fazer uma nova após detectada a colisão.
 * Após gerar a nova task, ela é deletada.
 */
void vTask_Add_Score(void *pvParameters)
{
	snake_player.size++;
	xTaskCreate(vTask_Create_Fruit, "Cria fruta", 128, NULL, osPriorityAboveNormal, &xTaskFruitHandle);
	vTaskDelay(20);
	vTaskDelete(xTaskPointsHandle);
}

/* Funcao para limpar a tela e mostrar a pontuação final.
 * Como o tamanho é uma variável inteira, é necessário utilizar snprintf (ou sprintf) para jogar o valor inteiro
 * para um buffer. É feito o print do buffer via string_lcd.
 * A funcao impede que as outras aconteçam pois após gerar ela, a funcao de movimento é deletada.
 */
void vTask_Game_Over(void *pvParameters)
{
	inic_LCD();
	char buffer[3];
	snake_player.points = snake_player.size - (INITIAL_SIZE + 1);
	snprintf(buffer, 3, "%d", snake_player.points);
	limpa_LCD();

	// nao mexer nisso
	while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15)) {
		goto_XY(10, 1);
		string_LCD("Score: ");
		goto_XY(10, 3);
		string_LCD(buffer);
		imprime_LCD();
	}

	NVIC_SystemReset();
}

/* Funcao que move a cobra, define direcao via eixo do joystick e faz o calculo de colisao.
 * É gerado um x e y iniciais para a cobra e uma direção (direita). O x e y precisam ser multiplos
 * de 4 para funcionar o calculo de deslocamento. É definido entao o tamanho inicial da cobra e após isso
 * a task fica no while(1) movendo a cobra e checando colisoes.
 * As colisoes sao testadas para cada no da cobra, ou seja, se ela encostar em qualquer parte do corpo o jogo acaba.
 * O jogo também acaba se a cobra bater na parede.
 * É nessa funcao tambem que é calculada a colisao da cabeca com a fruta.
 */
void vTask_Move_Snake_Player(void *pvParameters)
{
	// array de tempos de delay para a dificuldade
	uint32_t time_delay[3] = {200, 120, 70};

	// 6 -> direita, 4 -> esquerda, 8 -> cima, 2 -> baixo
	snake_player.direction = 6;

	// x e y tem que ser multiplos de 4 para nao dar problema com a colisao
	static uint32_t x = 8, y = 16;

	static int32_t diff_x_axis, diff_y_axis;

	snake_player.snake_coordinates.x2 = 0; snake_player.snake_coordinates.y2 = 0;
	snake_player.snake_coordinates.x3 = 0; snake_player.snake_coordinates.y3 = 0;

	// tamanho inicial da cobra
	snake_player.size = INITIAL_SIZE;

	// define posicao inicial da cabeca da cobra
	snake_player.node_x[0] = x;
	snake_player.node_y[0] = y;

	while(1)
	{
		diff_x_axis = 2048 - ADC_value[0];
		diff_y_axis = 2048 - ADC_value[1];

		// esse bloco de if else vai definir a direção do movimento
		if (diff_x_axis < -200){
			if (snake_player.direction != 6){ // nao pode trocar de esquerda para direita
				snake_player.direction = 4;
			}
		}
		else if (diff_x_axis > 200){
			if (snake_player.direction != 4){
				snake_player.direction = 6;
			}
		}
		else if (diff_y_axis < -200){
			if (snake_player.direction != 8){
				snake_player.direction = 2;
			}
		}
		else if (diff_y_axis > 200){
			if (snake_player.direction != 2){
				snake_player.direction = 8;
			}
		}

		// o switch faz com que a cobra continue andando na direcao escolhida
		switch (snake_player.direction) {
		case 2:
			if (y + 4 > 40) y = 40; // situaçoes de borda, para calcular colisao depois
			else y+=4;
			break;
		case 8:
			if (y == 0) y = 0;
			else y-=4;
			break;
		case 4:
			if (x == 0) x = 0;
			else x-=4;
			break;
		case 6:
			if (x + 4 > 80) x = 80;
			else x+=4;
			break;
		}
		//update dos nos
		for (uint32_t i = MAX_SIZE - 1; i != 0; i--) {
			snake_player.node_x[i] = snake_player.node_x[i-1];
			snake_player.node_y[i] = snake_player.node_y[i-1];
		}
		//update na cabeca
		snake_player.node_x[0] = x;
		snake_player.node_y[0] = y;

		for (uint32_t i = 0; i < snake_player.size; i++) {
			// joga o vetor com base no tamanho para o buffer
			snake_player.snake_coordinates.x1 = snake_player.node_x[i];
			snake_player.snake_coordinates.y1 = snake_player.node_y[i];

			// para cada iteracao do for, desenha o corpo da cobra
			desenha_fig(&snake_player.snake_coordinates, &snake);

			// checa colisao com a propria cobra, comecando pelo nó pós cabeca
			if ((snake_player.node_x[0] == snake_player.node_x[i]) && (snake_player.node_y[0] == snake_player.node_y[i]) && i > 0)
			{
				xTaskCreate(vTask_Game_Over, "Game over", 256, NULL, osPriorityNormal, &xTaskGameOverHandle);
				vTaskDelete(xTaskLCDHandle);
				vTaskDelete(xTaskSnakeHandle);
			}

			// condiçao para apagar o rabo: tamanho - i tem que ser 1.
			// tamanho 3, i = 2, no_x[2] = tamanho 3
			if ((snake_player.size - i) == 1) {
				desenha_fig(&snake_player.snake_coordinates, &delete_snake);
			}
		}

		// checa colisão da cobra com a fruta
		if (snake_player.node_x[0] == fruit.x1 && snake_player.node_y[0] == fruit.y1)
		{
			xTaskCreate(vTask_Add_Score, "Ponto", 128, NULL, osPriorityNormal, &xTaskPointsHandle);
		}

		// velocidade do jogo (dificuldade)
		vTaskDelay(time_delay[set_difficulty]);
		}
	}

/* Funcao que inicia o programa. Tem um seletor de dificuldade que altera o tempo de delay da funcao de movimento.
 * Apos selecionar a dificuldade e pressionar o botao do meio, a tarefa inicializa as outras tarefas
 * principais para o funcionamento do jogo, e no final se deleta.
 */
void vTask_Main_Menu(void *pvParameters)
{
	uint32_t seed_PRNG=1;

	static int32_t diff_x_axis;

	vTaskDelay(200);
	while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15))// quando pressionar a tecla comeca o jogo
	{
		diff_x_axis = 2048 - ADC_value[0];

		if (diff_x_axis < -200) {
			if (set_difficulty != easy)
				set_difficulty--;
			else
				set_difficulty = easy;
		}
		else if (diff_x_axis > 200) {
			if (set_difficulty != hard)
				set_difficulty++;
			else
				set_difficulty = hard;
		}

		limpa_LCD();
		seed_PRNG++;
		goto_XY(8, 1);
		string_LCD("Dificuldade: ");
		if (set_difficulty == easy) {
			goto_XY(25, 3);
			string_LCD("Facil");
		}
		else if (set_difficulty == med) {
			goto_XY(25,3);
			string_LCD("Medio");
		}
		else if (set_difficulty == hard) {
			goto_XY(21, 3);
			string_LCD("Dificil");
		}
		imprime_LCD();
		vTaskDelay(200);
	}
	init_LFSR(seed_PRNG);	// inicializacao para geracao de numeros pseudoaleatorios
	limpa_LCD();

	xTaskCreate(vTask_LCD_Print, "LCD_print", 128, NULL, osPriorityNormal, &xTaskLCDHandle);
	xTaskCreate(vTask_Move_Snake_Player, "Snake", 256, NULL, osPriorityAboveNormal, &xTaskSnakeHandle);
	xTaskCreate(vTask_Add_Score, "Ponto", 128, NULL, osPriorityNormal, &xTaskPointsHandle);

	vTaskDelete(xTaskMenuHandle);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)ADC_buffer,2);
	HAL_ADC_Start_IT(&hadc1);

	// inicializa LCD 5110
	inic_LCD();
	limpa_LCD();

	// -----------------------------------------

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  //osThreadStaticDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128, defaultTaskBuffer, &defaultTaskControlBlock);
 // defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

	xTaskCreate(vTask_Main_Menu, "Menu principal", 256, NULL, osPriorityNormal, &xTaskMenuHandle);

  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA4 PA5 PA6
                           PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

