/* USER CODE BEGIN Header */
/**
 * This whole project is in sync with its *.ioc file, and this includes the
 * ability to regenerate code. One thing that is not fully implemented is
 * handling of the touch screen input. Further on it will be added using default
 * BSP library. On the topic o BSP -- implementation of BSP in this project is a
 * custom implementation (it is not far from the oryginal, but some minor tweaks
 * were made).
 */

/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "modbus_conf.h"
#include "modbus.h"
#include <stdio.h>
#include <string.h>
#include "stm32f746g_discovery_lcd.h"
//#include "stm32746g_discovery_lcd.h"
#include "stm32746g_discovery_ts.h"
//#include "stm32f746g_discovery_ts_remote.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SLAVE_ID 16 // TODO
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc3;

DMA2D_HandleTypeDef hdma2d;

LTDC_HandleTypeDef hltdc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;

SDRAM_HandleTypeDef hsdram1;

/* USER CODE BEGIN PV */
uint32_t ADC3_buffer[2] = {0};

#define ADC_BUFFER_LENGTH 100
uint32_t uhADCxConvertedValue[ADC_BUFFER_LENGTH] = {0};

uint32_t adc_value = 0;
int c = 0 ;
char bufor[16] = {0};

TS_StateTypeDef TS_State;


uint8_t *resp;
uint16_t resplen;
MB_RESPONSE_STATE respstate;
uint8_t fan_on[]       = {0x00, 0x00, 0x03, 0xE8};
uint8_t fan_half[]     = {0x00, 0x00, 0x01, 0xF4};
uint8_t fan_off[]      = {0x00, 0x00, 0x00, 0x00};

uint8_t heater_on[]    = {0x00, 0x04, 0x03, 0xE8};
uint8_t heater_half[]  = {0x00, 0x04, 0x01, 0xF4};
uint8_t heater_var[]   = {0x00, 0x04, 0x01, 0xF4};
uint8_t heater_off[]   = {0x00, 0x04, 0x00, 0x00};

uint8_t get_temp[]     = {0x00, 0x00, 0x00, 0x01};
uint8_t get_temp_bad[] = {0x00, 0x01, 0x00, 0x01};

const uint16_t lcd_width					= 480;
const uint16_t lcd_height					= 282;

char txt1[50] = {0};
char utxt[16] = {0};
char ytxt[16] = {0};
char yzadtxt[24] = {0};
char dupobuffer[16] = {0};

volatile uint32_t input = 0;
volatile uint32_t output = 0;

uint8_t UART_MB_rcvd = 0;
volatile uint8_t UART_MB_sending = 0;

char txt[200] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_LTDC_Init(void);
static void MX_FMC_Init(void);
static void MX_DMA2D_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */
void Communication_Mode(bool rx, bool tx){
	if(rx) HAL_UART_Receive_IT(&huart6, &UART_MB_rcvd, 1);

	if(tx && UART_MB_sending == 0) {
		UART_MB_sending = 1;
		SetCharacterReadyToTransmit();
	}
	if(!tx) UART_MB_sending = 0;
}
void Communication_Put(uint8_t ch){
	HAL_UART_Transmit_IT(&huart6, &ch, 1);
}

uint8_t Communication_Get(void){
	uint8_t tmp = UART_MB_rcvd;
	UART_MB_rcvd = 0;
	SetCharacterReceived(false);
	return tmp;
}

void Enable50usTimer(void){
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
}

void Disable50usTimer(void){
  HAL_NVIC_DisableIRQ(TIM4_IRQn);
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == TS_INT_PIN){ // TOUCH SCREEN touched -- this is the place where you have to check where the touch screen is pressed
	  BSP_TS_GetState(&TS_State); /*!*/
	}
}

void DrawPointOfTouch(TS_StateTypeDef *TSS){
  static uint16_t lastx = 0;
  static uint16_t lasty = 0;
  BSP_LCD_SelectLayer(1);
  BSP_LCD_SetTextColor(LCD_COLOR_TRANSPARENT);
  BSP_LCD_DrawCircle(lastx,lasty, 3);
  BSP_LCD_DrawCircle(lastx,lasty, 2);
  if(TSS->touchDetected > 0){
	  lastx = TSS->touchX[0];
	  lasty = TSS->touchY[0];
	  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	  BSP_LCD_DrawCircle(lastx,lasty, 3);
	  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	  BSP_LCD_DrawCircle(lastx,lasty, 2);
  }
  BSP_LCD_SelectLayer(0);
}

void DrawCalibrationCross(uint32_t x, uint32_t y){
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_DrawLine(x-1, y-1, x-1, y-10); // upper left
	BSP_LCD_DrawLine(x-1, y+1, x-1, y+10); // upper right
	BSP_LCD_DrawLine(x+1, y-1, x+1, y-10); // lower left
	BSP_LCD_DrawLine(x+1, y+1, x+1, y+10); // lower right

	BSP_LCD_DrawLine(x-1, y-1, x-10, y-1); // lefter up
	BSP_LCD_DrawLine(x+1, y-1, x+10, y-1); // righter up
	BSP_LCD_DrawLine(x-1, y+1, x-10, y+1); // lefter down
	BSP_LCD_DrawLine(x+1, y+1, x+10, y+1); // righter down
}

void DrawFalka(uint32_t x, uint32_t y, uint32_t scale) {
	BSP_LCD_DrawLine(x, y, x + 2*scale/3, y + 2* scale);
	BSP_LCD_DrawLine(x, y, x + 2 * scale, y + scale);
	uint32_t new_x = x + scale;
	uint32_t new_y = y + scale;
	BSP_LCD_DrawLine(x, y, new_x, new_y);
	x = new_x;
	y = new_y;
	new_x = x + scale /2;
	new_y = y + scale;
	BSP_LCD_DrawLine(x, y, new_x, new_y);
	x = new_x;
	y = new_y;
	new_x = x;
	new_y = y + 2 * scale;
	BSP_LCD_DrawLine(x, y, new_x, new_y);
	x = new_x;
	y = new_y;
	new_x = x - scale / 2;
	new_y = y + scale;
	BSP_LCD_DrawLine(x, y, new_x, new_y);
	x = new_x;
	y = new_y;
	new_x = x - scale;
	new_y = y + scale;
	BSP_LCD_DrawLine(x, y, new_x, new_y);
	x = new_x;
	y = new_y;
	new_x = x - scale / 2;
	new_y = y + scale;
	BSP_LCD_DrawLine(x, y, new_x, new_y);
	x = new_x;
	y = new_y;
	new_x = x;
	new_y = y + 2 * scale;
	BSP_LCD_DrawLine(x, y, new_x, new_y);
	x = new_x;
	y = new_y;
	new_x = x + scale /2;
	new_y = y + scale;
	BSP_LCD_DrawLine(x, y, new_x, new_y);
	x = new_x;
	y = new_y;
	new_x = x + scale;
	new_y = y + scale;
	BSP_LCD_DrawLine(x, y, new_x, new_y);
}

void DrawGrzala(uint32_t x, uint32_t y){
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	// Obiekt
	BSP_LCD_DrawRect(x, y, 130, 80);
	BSP_LCD_DisplayStringAt(x+17, y+31, (uint8_t*)ytxt, LEFT_MODE);
	BSP_LCD_DrawCircle(x + 93, y+33, 3);
	BSP_LCD_DrawCircle(x + 93, y+33, 2);
	// Strzała
	DrawFalka(x + 65, y + 90, 4);
	DrawFalka(x + 45, y + 90, 4);
	DrawFalka(x + 85, y + 90, 4);
	// Grzała
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_FillRect(x+30, y+145, 80, 50);
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_DrawRect(x+30, y+145, 80, 50);
	BSP_LCD_DisplayStringAt(x+50, y+162, (uint8_t*)utxt, LEFT_MODE);
}

uint32_t slider_start = 160;
uint32_t slider_end = 460;
uint32_t slider_fill = 15;
void DrawSlider(uint32_t fill_percent) {
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_DrawRect(slider_start, 246, slider_end - slider_start, 8);
	uint32_t handle_x = slider_start + fill_percent * (slider_end - slider_start) / 100;
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_FillRect(handle_x, 240, 10, 20);
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_DrawRect(handle_x, 240, 10, 20);
}
#define nsamples 102
float usamples[nsamples] = {0};
float ysamples[nsamples] = {0};
float uscale = 0.7;
float yscale = 1.3;
uint32_t xscale = 3;
void DrawPlots(uint32_t x, uint32_t y) {
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_FillRect(x-1, y-50 * uscale - 70 * yscale, nsamples * xscale + 2, 70 * yscale + 100 * uscale + 2);
	BSP_LCD_SetTextColor(LCD_COLOR_RED);
	for (int i=1; i<nsamples; ++i) {
		BSP_LCD_DrawLine(x + i * xscale, y - usamples[i-1] * uscale, x + (i+1) * xscale, y - usamples[i] * uscale);
	}
	BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
	y = y - 50 * uscale;
	for (int i=1; i<nsamples; ++i) {
		BSP_LCD_DrawLine(x + i * xscale, y - ysamples[i-1] * yscale, x + (i+1) * xscale, y - ysamples[i] * yscale);
	}
}

int manual_mode = 1;
int alarm1 = 0;
int alarm2 = 0;

void DrawButtons() {
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_FillRect(50, 228, 35, 35);
	BSP_LCD_FillRect(95, 228, 35, 35);
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_DrawRect(50, 228, 35, 35);
	BSP_LCD_DrawRect(95, 228, 35, 35);
	if (manual_mode) {
		BSP_LCD_FillRect(95, 228, 35, 35);
		BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		sprintf(dupobuffer, "A");
		BSP_LCD_DisplayStringAt(55, 233,  (uint8_t*)dupobuffer, LEFT_MODE);
		BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		sprintf(dupobuffer, "M");
		BSP_LCD_DisplayStringAt(100, 233,  (uint8_t*)dupobuffer, LEFT_MODE);
	} else {
		BSP_LCD_FillRect(50, 228, 35, 35);
		BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		sprintf(dupobuffer, "A");
		BSP_LCD_DisplayStringAt(55, 233,  (uint8_t*)dupobuffer, LEFT_MODE);
		BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		sprintf(dupobuffer, "M");
		BSP_LCD_DisplayStringAt(100, 233,  (uint8_t*)dupobuffer, LEFT_MODE);
	}
	BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
}

void DrawAlarm() {
	BSP_LCD_SetTextColor(LCD_COLOR_RED);
	BSP_LCD_SetBackColor(LCD_COLOR_RED);
	BSP_LCD_FillRect(170, 58, 270, 150);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	if (alarm1) {
		sprintf(dupobuffer, "Alarm!");
		BSP_LCD_DisplayStringAt(72, 90,  (uint8_t*)dupobuffer, CENTER_MODE);
		sprintf(dupobuffer, "Awaria czujnika");
		BSP_LCD_DisplayStringAt(67, 110,  (uint8_t*)dupobuffer, CENTER_MODE);
	}
	else if (alarm2) {
		sprintf(dupobuffer, "Alarm!");
		BSP_LCD_DisplayStringAt(72, 90,  (uint8_t*)dupobuffer, CENTER_MODE);
		sprintf(dupobuffer, "Blad modbus");
		BSP_LCD_DisplayStringAt(67, 110,  (uint8_t*)dupobuffer, CENTER_MODE);
	}
}

float y_zad = 40;
void HAL_LTDC_LineEventCallback(LTDC_HandleTypeDef *hltdc){
 // Rysowanie
  if (alarm1 || alarm2) {
	  DrawAlarm();
  } else {
	  DrawPointOfTouch(&TS_State);
	  DrawGrzala(20, 20);
	  checkSlider(&TS_State);
	  checkButtons(&TS_State);
	  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	  BSP_LCD_FillRect(slider_start - 6, 239, slider_end - slider_start + 16, 22);
	  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	  DrawSlider(slider_fill);
	  DrawPlots(170, 175);
	  DrawButtons();
	  if (!manual_mode) {
		  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		  BSP_LCD_DisplayStringAt(220, 8,  (uint8_t*)yzadtxt, LEFT_MODE);
		  BSP_LCD_DrawCircle(436, 10, 3);
		  BSP_LCD_DrawCircle(436, 10, 2);
	  } else {
		  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		  BSP_LCD_FillRect(218, 4, 300, 20);
	  }
  }
  HAL_LTDC_ProgramLineEvent(hltdc, 272);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle)
{
	static int i=0;
	static uint32_t tmpval= 0;
	for(i=0,tmpval=0;i<ADC_BUFFER_LENGTH; ++i){
		tmpval += uhADCxConvertedValue[i];
	}
	adc_value = tmpval/ADC_BUFFER_LENGTH;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART6){
		SetCharacterReceived(true);
		HAL_UART_Receive_IT(&huart6, &UART_MB_rcvd, 1);
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART6)
		UART_MB_sending = 0;
}

float y;
float u = -35;

void checkSlider(TS_StateTypeDef *TSS) {
	uint32_t x = TSS->touchX[0];
	uint32_t y = TSS->touchY[0];
	if (x > slider_start && y > 235 && x < slider_end) {
		slider_fill = 100 * (x - slider_start) / (slider_end - slider_start);
	}
}

void checkButtons(TS_StateTypeDef *TSS) {
	uint32_t x = TSS->touchX[0];
	uint32_t y = TSS->touchY[0];
	if (x > 50 && x < 50+35 && y > 228 && y < 228+35) {
		manual_mode = 0;
	} else if (x > 95 && x < 95+35 && y > 228 && y < 228+35) {
		manual_mode = 1;
	}
}

float slider_to_u() {
	return (float) slider_fill - 50;
}

float slider_to_yzad() {
	return 30.0 + 30.0 * slider_fill / 100.0;
}

// Rzeczy do dmc
#define D 200
#define N 26
#define Nu 5
#define lambda 1

float Ke = 0.391418;
float Ku[D-1] = {0.014784,0.016736,0.020261,0.022179,0.024279,0.026357,0.028359,0.030523,0.032652,0.034703,0.036919,0.037516,0.039808,0.040180,0.042547,0.043255,0.044118,0.044872,0.045249,0.045801,0.046298,0.046721,0.047042,0.047573,0.046444,0.046541,0.046859,0.045781,0.045980,0.046348,0.045130,0.045168,0.045429,0.043843,0.043571,0.043585,0.041800,0.041310,0.039539,0.039358,0.037395,0.037283,0.035107,0.034797,0.034548,0.034359,0.033742,0.033429,0.031586,0.030865,0.030451,0.030163,0.029693,0.027432,0.027059,0.026518,0.025774,0.025144,0.024832,0.025954,0.025534,0.024700,0.025783,0.025416,0.024880,0.024424,0.023530,0.022973,0.023865,0.023254,0.022531,0.021633,0.022705,0.022004,0.021417,0.022278,0.021648,0.020642,0.021590,0.020855,0.019967,0.020564,0.019716,0.020354,0.017687,0.018313,0.017495,0.018146,0.017395,0.016263,0.017096,0.016245,0.016861,0.015813,0.016487,0.015499,0.016235,0.015272,0.014465,0.015315,0.014739,0.015578,0.014759,0.015388,0.014088,0.014517,0.014899,0.015421,0.014308,0.014667,0.013392,0.013635,0.012525,0.012927,0.011433,0.011939,0.010809,0.011181,0.011520,0.011825,0.010733,0.011176,0.010181,0.008854,0.009307,0.008111,0.006852,0.007371,0.007819,0.008200,0.008738,0.009224,0.009646,0.008644,0.009155,0.009565,0.010170,0.010691,0.009285,0.009937,0.010500,0.009397,0.010017,0.010551,0.011018,0.011646,0.010623,0.011083,0.009898,0.010210,0.010471,0.010705,0.010887,0.009191,0.009304,0.009608,0.008297,0.008521,0.008973,0.007545,0.007912,0.008263,0.007201,0.007674,0.008080,0.006593,0.007118,0.007596,0.006408,0.006977,0.007478,0.006297,0.006658,0.006931,0.007451,0.007883,0.008249,0.008549,0.008812,0.009041,0.009236,0.007818,0.007944,0.008034,0.006264,0.008147,0.006599,0.006615,0.005037,0.005021,0.005019,0.005243,0.003868,0.004043,0.004425,0.003192,0.003479,0.003736,0.003994};


float dUP[D-1] = {0};
float dU = 0;

float u_k_1 = 0;
// koniec rzeczy do dmc

int first_time = 1;
int counter = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM2){
		static uint16_t raw_y = 2345;
		static uint16_t raw_u = 0;
		MB_SendRequest(SLAVE_ID, FUN_READ_INPUT_REGISTER, get_temp, 4);
		respstate = MB_GetResponse(SLAVE_ID, FUN_READ_INPUT_REGISTER, &resp, &resplen, 300);
		if(respstate != RESPONSE_OK) {
			alarm2 = 1;
		}
		else {
			alarm2 = 0;
			raw_y = resp[1]*0x100+resp[2];
			y = raw_y/100.0f;
		}

		if (y > 125 || y < -55) {
			alarm1 = 1;
		} else {
			alarm1 = 0;
		}


		if (manual_mode) {
			u = slider_to_u();
		} else {
			y_zad = slider_to_yzad();
			float e = y_zad - y;
			float Ku_dUP = 0;
			for(int i = 0; i < D - 1; i++) {
				Ku_dUP += Ku[i] * dUP[i];
			}
			dU = Ke*e - Ku_dUP;
			for(int i = D - 2; i > 0; i--){
				dUP[i] = dUP[i-1];
			}
			dUP[0] = dU;
			u = u_k_1 + dU;
		}

		/* aplikacja ograniczen na sygnal sterujacy */
		if(u >   50.0f) u =  50.0f;
		if(u <  -50.0f) u = -50.0f;
		u_k_1 = u;

		if (alarm1) u = -50.f;

		if (first_time) {
			first_time = 0;
			for (int i=0; i<nsamples; ++i) {
				usamples[i] = u;
				ysamples[i] = y;
			}
		}
		--counter;
		if (counter < 1) {
			counter = 5;
			for (int i=0; i<nsamples-1; ++i) {
					usamples[i] = usamples[i+1];
				}
			usamples[nsamples-1] = u;

			for (int i=0; i<nsamples-1; ++i) {
					ysamples[i] = ysamples[i+1];
				}
			if (alarm1) ysamples[nsamples-1] = 0.0f;
			else ysamples[nsamples-1] = y;
		}

		sprintf(ytxt, "%.2f C", y);
		sprintf(utxt, "%d%%", (int)u + 50);
		sprintf(yzadtxt, "W.Zadana: %.2f C", y_zad);
		/* skalowanie z -50..50 do 0..1000 */
		raw_u = (uint16_t)(u+50.0f)*10; // przejscie z -2048 - 2047 do 0 - 4095

		/* przygotowanie wiadomosci MODBUS */
		heater_var[2] = (raw_u&0xFF00)>>8; // pierwszy bajt
		heater_var[3] = (raw_u&0x00FF)>>0; // drugi bajt

		/* wyslanie wiadomosci */
		MB_SendRequest(SLAVE_ID, FUN_WRITE_SINGLE_REGISTER, heater_var, 4);

		/* odczyt odpowiedzi i sprawdzenie jej poprawnosci */
		respstate = MB_GetResponse(SLAVE_ID, FUN_WRITE_SINGLE_REGISTER, &resp, &resplen, 300);
		if(respstate != RESPONSE_OK) {
					alarm2 = 1;
				} else {
					alarm2 = 0;
				}

		/* komunikacja z komputerem */
		while(HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX);
		sprintf(txt,"U=%.2f;Y=%.2f;Y_zad=%.2f;\n",u,y,y_zad); // 22 znaki

		if(HAL_UART_Transmit_IT(&huart1, (uint8_t*)txt, strlen(txt))!= HAL_OK) Error_Handler();
	}
	if (htim->Instance == TIM3){ // timer odpowiedzialny za aktualizacje MB i odliczanie timeout'u
		MB();
		TimeoutTick();
	}
	if (htim->Instance == TIM4){ // timer odpowiedzialny za odliczanie kwantow 50us
		Timer50usTick();
	}
	if (htim->Instance == TIM5){ // ...
	}
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

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  HAL_Delay(100); /*! Delay so that LCD will not restart during initialisation !*/
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_LTDC_Init();
  MX_FMC_Init();
  MX_DMA2D_Init();
  MX_ADC1_Init();
  MX_ADC3_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  BSP_LCD_SetFont(&Font20); // choose size of the font: Font8, Font12, Font16, Font20, Font24
  BSP_LCD_SetBackColor(LCD_COLOR_WHITE); // each character has background!!!
  BSP_TS_Init(0,0); // initialisation of TouchScreen -- arguments are irrelevant
  BSP_TS_ITConfig(); // to cancel exti interrupts from the touch screen comment this line

  HAL_ADC_Start_DMA(&hadc3, ADC3_buffer, 2);
  BSP_LCD_SelectLayer(0);
  BSP_LCD_Clear(LCD_COLOR_WHITE);
  BSP_LCD_SelectLayer(1);
  BSP_LCD_Clear(LCD_COLOR_TRANSPARENT);
  HAL_LTDC_ProgramLineEvent(&hltdc, 272);

  if(HAL_ADC_Start_DMA(&hadc1, (uint32_t*)uhADCxConvertedValue, ADC_BUFFER_LENGTH) != HAL_OK)
	  Error_Handler();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  MB_Config(115200);

	while(HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX);
	while(HAL_UART_GetState(&huart6) == HAL_UART_STATE_BUSY_TX);

	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_Base_Start_IT(&htim5);

  HAL_Delay(100); // wait for everything to set up before the controller loop starts


  MB_SendRequest(SLAVE_ID, FUN_WRITE_SINGLE_REGISTER, fan_half, 4);
	respstate = MB_GetResponse(SLAVE_ID, FUN_WRITE_SINGLE_REGISTER, &resp, &resplen, 300);
	if(respstate != RESPONSE_OK) while(1);
	HAL_Delay(900);
  HAL_TIM_Base_Start_IT(&htim2);

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		sprintf(txt1,"Test input = %ld",input);
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);

//		BSP_TS_GetState(&TS_State);
		++c;
		sprintf((char*)bufor, "Test %d", c);
		HAL_Delay(100);

		//BSP_LCD_DisplayStringAt( 10, 10, (uint8_t*)bufor, LEFT_MODE);
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
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
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 2;
  hadc3.Init.DMAContinuousRequests = ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief DMA2D Initialization Function
  * @param None
  * @retval None
  */
static void MX_DMA2D_Init(void)
{

  /* USER CODE BEGIN DMA2D_Init 0 */

  /* USER CODE END DMA2D_Init 0 */

  /* USER CODE BEGIN DMA2D_Init 1 */

  /* USER CODE END DMA2D_Init 1 */
  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_M2M;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_ARGB8888;
  hdma2d.Init.OutputOffset = 0;
  hdma2d.LayerCfg[1].InputOffset = 0;
  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_ARGB8888;
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0;
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DMA2D_Init 2 */

  /* USER CODE END DMA2D_Init 2 */

}

/**
  * @brief LTDC Initialization Function
  * @param None
  * @retval None
  */
static void MX_LTDC_Init(void)
{

  /* USER CODE BEGIN LTDC_Init 0 */

  /* USER CODE END LTDC_Init 0 */

  LTDC_LayerCfgTypeDef pLayerCfg = {0};
  LTDC_LayerCfgTypeDef pLayerCfg1 = {0};

  /* USER CODE BEGIN LTDC_Init 1 */

  /* USER CODE END LTDC_Init 1 */
  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 40;
  hltdc.Init.VerticalSync = 9;
  hltdc.Init.AccumulatedHBP = 53;
  hltdc.Init.AccumulatedVBP = 11;
  hltdc.Init.AccumulatedActiveW = 533;
  hltdc.Init.AccumulatedActiveH = 283;
  hltdc.Init.TotalWidth = 565;
  hltdc.Init.TotalHeigh = 285;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 480;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 272;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
  pLayerCfg.Alpha = 255;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg.FBStartAdress = 0xC0000000;
  pLayerCfg.ImageWidth = 480;
  pLayerCfg.ImageHeight = 272;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg1.WindowX0 = 0;
  pLayerCfg1.WindowX1 = 480;
  pLayerCfg1.WindowY0 = 0;
  pLayerCfg1.WindowY1 = 272;
  pLayerCfg1.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
  pLayerCfg1.Alpha = 255;
  pLayerCfg1.Alpha0 = 0;
  pLayerCfg1.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
  pLayerCfg1.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
  pLayerCfg1.FBStartAdress = 0xC0000000+480*272*4;
  pLayerCfg1.ImageWidth = 480;
  pLayerCfg1.ImageHeight = 272;
  pLayerCfg1.Backcolor.Blue = 0;
  pLayerCfg1.Backcolor.Green = 0;
  pLayerCfg1.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg1, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LTDC_Init 2 */
  BSP_LCD_LayerDefaultInit(0, pLayerCfg.FBStartAdress);
  BSP_LCD_LayerDefaultInit(1, pLayerCfg1.FBStartAdress);
  /* Assert display enable LCD_DISP pin */
  HAL_GPIO_WritePin(LCD_DISP_GPIO_PORT, LCD_DISP_PIN, GPIO_PIN_SET);

  /* Assert backlight LCD_BL_CTRL pin */
  HAL_GPIO_WritePin(LCD_BL_CTRL_GPIO_PORT, LCD_BL_CTRL_PIN, GPIO_PIN_SET);
  /* USER CODE END LTDC_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 10800-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim3.Init.Prescaler = 108-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 108-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 50;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 10800-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 10000;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_9B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_EVEN;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */
  FMC_SDRAM_CommandTypeDef Command;

  /* USER CODE END FMC_Init 0 */

  FMC_SDRAM_TimingTypeDef SdramTiming = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SDRAM1 memory initialization sequence
  */
  hsdram1.Instance = FMC_SDRAM_DEVICE;
  /* hsdram1.Init */
  hsdram1.Init.SDBank = FMC_SDRAM_BANK1;
  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_2;
  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_PERIOD_2;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_ENABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_0;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 2;
  SdramTiming.ExitSelfRefreshDelay = 6;
  SdramTiming.SelfRefreshTime = 4;
  SdramTiming.RowCycleDelay = 6;
  SdramTiming.WriteRecoveryTime = 2;
  SdramTiming.RPDelay = 2;
  SdramTiming.RCDDelay = 2;

  if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */
  __IO uint32_t tmpmrd =0;
  /* Step 3:  Configure a clock configuration enable command */
  Command.CommandMode = FMC_SDRAM_CMD_CLK_ENABLE;
  Command.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;
  Command.AutoRefreshNumber = 1;
  Command.ModeRegisterDefinition = 0;

  /* Send the command */
  HAL_SDRAM_SendCommand(&hsdram1, &Command, SDRAM_TIMEOUT);

  /* Step 4: Insert 100 us minimum delay */
  /* Inserted delay is equal to 1 ms due to systick time base unit (ms) */
  HAL_Delay(1);

  /* Step 5: Configure a PALL (precharge all) command */
  Command.CommandMode = FMC_SDRAM_CMD_PALL;
  Command.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;
  Command.AutoRefreshNumber = 1;
  Command.ModeRegisterDefinition = 0;

  /* Send the command */
  HAL_SDRAM_SendCommand(&hsdram1, &Command, SDRAM_TIMEOUT);

  /* Step 6 : Configure a Auto-Refresh command */
  Command.CommandMode = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
  Command.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;
  Command.AutoRefreshNumber = 8;
  Command.ModeRegisterDefinition = 0;

  /* Send the command */
  HAL_SDRAM_SendCommand(&hsdram1, &Command, SDRAM_TIMEOUT);

  /* Step 7: Program the external memory mode register */
  tmpmrd = (uint32_t)SDRAM_MODEREG_BURST_LENGTH_1          |
                     SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL   |
                     SDRAM_MODEREG_CAS_LATENCY_2           |
                     SDRAM_MODEREG_OPERATING_MODE_STANDARD |
                     SDRAM_MODEREG_WRITEBURST_MODE_SINGLE;

  Command.CommandMode = FMC_SDRAM_CMD_LOAD_MODE;
  Command.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;
  Command.AutoRefreshNumber = 1;
  Command.ModeRegisterDefinition = tmpmrd;

  /* Send the command */
  HAL_SDRAM_SendCommand(&hsdram1, &Command, SDRAM_TIMEOUT);

  /* Step 8: Set the refresh rate counter */
  /* (15.62 us x Freq) - 20 */
  /* Set the device refresh counter */
  hsdram1.Instance->SDRTR |= ((uint32_t)((1292)<< 1));

  /* USER CODE END FMC_Init 2 */
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOK_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOI, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PK3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOK, &GPIO_InitStruct);

  /*Configure GPIO pin : PI12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : PI13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : PG6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PF7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

