/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdio.h>
#include <OLED_Fonts.h>
#include <OLED_Icons.h>
#include <OLED.h>
#include <mpu6050.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_MEDIDAS 60				//define o número de medidas que serão utilizadas para calcular a média móvel. Neste caso, estamos utilizando um buffer com 20 medidas.
#define PI	3.14159265
#define RAD_TO_DEG	(180/PI)		//fator de conversao de radianos para graus
/*-------------------*/
/*-------------------*/
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
// Kalman structure

float mediamovelXAC[60] = {0x00}; //BUFFER QUE GUARDAR OS VALORES DA MEDIA MOVEL /*AO INICIAR APENAS UMA POSIÇÃO DO VETOR OS DEMAIS SERÃO INICIADOS EM 0X00 AUTOMATICAMENTE PELO COMPILADOR*/
float mediamovelYAC[60] = {0x00}; //BUFFER QUE GUARDAR OS VALORES DA MEDIA MOVEL /*AO INICIAR APENAS UMA POSIÇÃO DO VETOR OS DEMAIS SERÃO INICIADOS EM 0X00 AUTOMATICAMENTE PELO COMPILADOR*/
float mediamovelZAC[60] = {0x00}; //BUFFER QUE GUARDAR OS VALORES DA MEDIA MOVEL /*AO INICIAR APENAS UMA POSIÇÃO DO VETOR OS DEMAIS SERÃO INICIADOS EM 0X00 AUTOMATICAMENTE PELO COMPILADOR*/
float mediamovelXGY[60] = {0x00}; //BUFFER QUE GUARDAR OS VALORES DA MEDIA MOVEL /*AO INICIAR APENAS UMA POSIÇÃO DO VETOR OS DEMAIS SERÃO INICIADOS EM 0X00 AUTOMATICAMENTE PELO COMPILADOR*/
float mediamovelYGY[60] = {0x00}; //BUFFER QUE GUARDAR OS VALORES DA MEDIA MOVEL /*AO INICIAR APENAS UMA POSIÇÃO DO VETOR OS DEMAIS SERÃO INICIADOS EM 0X00 AUTOMATICAMENTE PELO COMPILADOR*/
float mediamovelZGY[60] = {0x00}; //BUFFER QUE GUARDAR OS VALORES DA MEDIA MOVEL /*AO INICIAR APENAS UMA POSIÇÃO DO VETOR OS DEMAIS SERÃO INICIADOS EM 0X00 AUTOMATICAMENTE PELO COMPILADOR*/
float 	X_DataAC,
/*----*/Y_DataAC,
/*----*/Z_DataAC,
/*----*/X_DataGY,
/*----*/Y_DataGY,
/*----*/Z_DataGY=0x00;
float 	somaXAC,
/*----*/somaYAC,
/*----*/somaZAC,
/*----*/somaXGY,
/*----*/somaYGY,
/*----*/somaZGY = 0x00;
float 	X_MedAC,
/*----*/Y_MedAC,
/*----*/Z_MedAC,
/*----*/X_MedGY,
/*----*/Y_MedGY,
/*----*/Z_MedGY=0x00;
float	X_MedFC,
/*----*/Y_MedFC,
/*----*/Z_MedFC=0x00;
float 	roll,
/*----*/pitch = 0x00;
uint16_t cont[8];
uint8_t initflag = 0x00;
int		indexbuffXAC,   //INDEXADOR PARA AUXILIAR NA ALOCAÇÃO DE UM NOVO VALOR NO BUFFER DA MEDIA MOVEL
/*----*/indexbuffYAC,
/*----*/indexbuffZAC,
/*----*/indexbuffXGY,
/*----*/indexbuffYGY,
/*----*/indexbuffZGY = 0x00;
uint8_t init = 0x00;
char eixox[18];
char eixoy[18];
char eixoR[18];
char eixoP[19];
char eixoz[18];

MPU6050_t MPU6050; //REFERENCIAMENTO DA ESTRUTURA DE DADOS DO SENSOR
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void initdisp(void);
float media_movel(float novo_valor, float *buffer, int *index, float *soma);
void captura_dados(void);
float roll_angle(float acc_x, float acc_y, float acc_z);
float pitch_angle(float acc_x, float acc_y, float acc_z);
void dados_conv(void);
double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);
float filtroComplementar(float gyro, float accel, float dt, float alpha);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef * htim){//ENDEREÇO DE HTIM COMO PARAMETRO
	if(htim->Instance == TIM3){ 	//IRA LER O MEMBRO DA ESTRUTURA APONTADO PELO PONTEIRO PASSADO COMO ARGUMENTO (*htim/nome da estrutura) DA FUNC CALL BACK
		cont[0]++;							/*FUNÇÃO DE TRATAMENTO DE INTERRUPÇÃO POR OVERFLOW TIMER3*/ /*Instance É A FLAG QUE INDICA QUAL TIMER GEROU A INTERRUPÇÃO*/
		cont[1]++;							/*DELAYS POR INTERRUPÇÃO - CONTADORES*/
		cont[2]++;
		if(cont[0]>=4 && init){
			captura_dados();
			HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
			HAL_GPIO_TogglePin(LPCB_GPIO_Port, LPCB_Pin);
			cont[0] = 0;
		}
		if(cont[1] >= 250 && init){
			/*HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
			HAL_GPIO_TogglePin(LPCB_GPIO_Port, LPCB_Pin);*/
			cont[1]=0;
		}
		if(cont[2]>=100 && init){
			dados_conv();
			cont[2] = 0;
		}
	}
}
void initdisp(void){
	OLED_Clear(0);
	OLED_DrawXBM(0 , 0, logov2);
	OLED_UpdateScreen();
	HAL_Delay(2500);
}
float media_movel(float novo_valor, float *buffer, int *index, float *soma){/*Um filtro de média móvel simples pode ser usado para suavizar os dados brutos e reduzir o ruído. Este filtro calcula a média de um número fixo de medidas recentes.*/
	//static float soma = 0;					//*O uso do static faz com que a variável mantenha seu valor entre as chamadas da função.*//

	*soma += novo_valor - buffer[*index];	//*Atualizamos a soma adicionando o novo valor medido e subtraindo o valor mais antigo do buffer (aquele no índice atual)*//
	buffer[*index] = novo_valor;			//*Atualizamos o buffer na posição atual (*index) com o novo valor medido.*//
	*index = (*index + 1) % NUM_MEDIDAS;	//*Atualizamos o índice para apontar para a próxima posição no buffer, garantindo que ele circule dentro do tamanho do buffer definido por NUM_MEDIDAS. *//
	//*Usamos o operador % (módulo) para garantir que o índice permaneça dentro dos limites do buffer.*//
	return *soma / NUM_MEDIDAS;
}
void captura_dados(void){ //*CAPTURA OS DADOS DE ACELERAÇÃO E GIROSCOPIO BRUTOS E APLICA A MEDIA MOVEL *//
	while (MPU6050_Init(&hi2c1) == 1);
	MPU6050_Read_Accel(&hi2c1, &MPU6050);
	X_DataAC = MPU6050.Ax; //----->
	Y_DataAC = MPU6050.Ay; //----->
	Z_DataAC = MPU6050.Az; //----->
	X_MedAC = media_movel(X_DataAC, mediamovelXAC, &indexbuffXAC, &somaXAC);
	Y_MedAC = media_movel(Y_DataAC, mediamovelYAC, &indexbuffYAC, &somaYAC);
	Z_MedAC = media_movel(Z_DataAC, mediamovelZAC, &indexbuffZAC, &somaZAC);
	MPU6050_Read_Gyro(&hi2c1, &MPU6050);
	X_DataGY = MPU6050.Gx; //----->
	Y_DataGY = MPU6050.Gy; //----->
	Z_DataGY = MPU6050.Gz; //----->
	X_MedGY = media_movel(X_DataGY, mediamovelXGY, &indexbuffXGY, &somaXGY);
	Y_MedGY = media_movel(Y_DataGY, mediamovelYGY, &indexbuffYGY, &somaYGY);
	Z_MedGY = media_movel(Z_DataGY, mediamovelZGY, &indexbuffZGY, &somaZGY);
}
float roll_angle(float acc_x, float acc_y, float acc_z) { //CALCULA O ANGULO COM OS VALORES FILTRADOS PELA MEDIA MOVEL
	return atan2(acc_y, acc_z) * RAD_TO_DEG;
}
float pitch_angle(float acc_x, float acc_y, float acc_z) {//CALCULA O ANGULO COM OS VALORES FILTRADOS PELA MEDIA MOVEL
	return atan2(-acc_x, sqrt(acc_y * acc_y + acc_z * acc_z)) * RAD_TO_DEG;
}
void dados_conv(void){
	roll = roll_angle(X_MedAC, Y_MedAC, Z_MedAC);
	pitch = pitch_angle(X_MedAC, Y_MedAC, Z_MedAC);
	kalman_calc(X_MedGY, Y_MedGY, roll, pitch, &MPU6050);
	sprintf(eixox,"Eixo x: %.1f*", MPU6050.KalmanAngleX);
	sprintf(eixoy,"Eixo y: %.1f*", MPU6050.KalmanAngleY);
	sprintf(eixoR,"ROLL x: %.1f°", roll);
	sprintf(eixoP,"PITCH y: %.1f°", pitch);
	OLED_Clear(0);
	FontSet(Lucida_12);
	OLED_DrawStr(eixox, 1, 1, 1);
	OLED_DrawStr(eixoy, 1, 18, 1);
	OLED_DrawStr(eixoR, 1, 35, 1);
	OLED_DrawStr(eixoP, 1, 50, 1);
	OLED_UpdateScreen();
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
	MX_I2C1_Init();
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim3); //INCIA TIMER3 COM INTERRUPÇÃO NO OVERFLO
	while (MPU6050_Init(&hi2c1) == 1);
	OLED_Init(&hi2c1); //INICIANDO DISPLAY PELA I2C
	/*------------------------------------------------------*/
	OLED_Clear(0);
	OLED_UpdateScreen();
	initdisp();
	OLED_Clear(0);
	OLED_UpdateScreen();
	init = 1;
	/* USER CODE END 2 */

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
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 287;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 249;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LPCB_GPIO_Port, LPCB_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : LPCB_Pin */
	GPIO_InitStruct.Pin = LPCB_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(LPCB_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LD1_Pin */
	GPIO_InitStruct.Pin = LD1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(LD1_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
