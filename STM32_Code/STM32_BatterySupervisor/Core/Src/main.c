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
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>

#include <string.h>

#include "common.h"
#include "gpio.h"
#include "can_bus.h"
#include "dc_control.h"
#include "sys_mode_controller.h"
#include "BatteryManagement/bms_types.h"
#include "battery.h"


// todo
typedef struct {
    uint16_t cell_voltage_max_mV;
    uint16_t cell_voltage_max_index;
    uint16_t cell_voltage_min_mV;
    uint16_t cell_voltage_min_index;
} __attribute__((__packed__)) csc_cell_volt_min_max_t;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define uSend(x) uartSend((char *)x, sizeof x -1)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

FDCAN_HandleTypeDef hfdcan2;

HRTIM_HandleTypeDef hhrtim1;

IWDG_HandleTypeDef hiwdg;

LPTIM_HandleTypeDef hlptim1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart5_tx;
DMA_HandleTypeDef hdma_uart5_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */
char tx_buf[64];
uint8_t tx_len;

UART_HandleTypeDef* huart_rs485;
UART_HandleTypeDef* huart_debug;

uint16_t ADC1ConvertedData[1];
uint16_t ADC2ConvertedData[1];
volatile uint16_t debug_sigma_delta_re;
volatile int16_t i_dc_filt_10mA;
volatile uint16_t debug_v_dc_raw;


volatile uint16_t id;
control_ref_t ctrl_ref;
volatile bool print_request;

volatile uint32_t cnt_1Hz;
volatile uint32_t cntErr_1Hz;

volatile uint16_t debug_v_dc_FBboost_sincfilt_100mV;

extern volatile int16_t debug_i_ac_amp_10mA;


uint8_t ubKeyNumber = 0x0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_HRTIM1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_UART5_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_IWDG_Init(void);
static void MX_LPTIM1_Init(void);
/* USER CODE BEGIN PFP */
static void FDCAN_Config(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void uartSend(char* ptr, int len)
{
    HAL_UART_Transmit(huart_debug, (uint8_t *)ptr,len, 10);

}

void uSendInt(int value)
{
	itoa(value, tx_buf, 10);
	tx_len=strlen(tx_buf);
	HAL_UART_Transmit(huart_debug, (uint8_t *)tx_buf, tx_len, 10);
}

void uSend_1m(int int_value)
{
	float value = int_value/1000.0;
	tx_len = snprintf(NULL, 0, "%.03f", value);
	char *str = (char *)malloc(tx_len + 1);
	snprintf(str, tx_len + 1, "%f.03", value);
	HAL_UART_Transmit(huart_debug, (uint8_t *)str, tx_len, 10);
	free(str);
}

void uSend_10m(int int_value)
{
	float value = int_value/100.0;
	tx_len = snprintf(NULL, 0, "%.02f", value);
	char *str = (char *)malloc(tx_len + 1);
	snprintf(str, tx_len + 1, "%f.02", value);
	HAL_UART_Transmit(huart_debug, (uint8_t *)str, tx_len, 10);
	free(str);
}

void uSend_100m(int int_value)
{
	float value = int_value/10.0;
	tx_len = snprintf(NULL, 0, "%.01f", value);
	char *str = (char *)malloc(tx_len + 1);
	snprintf(str, tx_len + 1, "%f.01", value);
	HAL_UART_Transmit(huart_debug, (uint8_t *)str, tx_len, 10);
	free(str);
}


void resetWatchdog()
{
	// Watchdog runs at 32/8=4kHz -> ~1sec for 4095
    /* Refresh IWDG: reload counter */
    if(HAL_IWDG_Refresh(&hiwdg) != HAL_OK) {
      /* Refresh Error */
      Error_Handler();
    }
}


void HAL_LPTIM_AutoReloadMatchCallback(LPTIM_HandleTypeDef *hlptim)
{
	cnt_1Hz++;
	if (get_sys_errorcode() != EC_NO_ERROR) {
		cntErr_1Hz++;
	}
}


void reinitUART(UART_HandleTypeDef *huart, uint32_t BaudRate)
{
	if (huart->Init.BaudRate != BaudRate) {
		huart->Init.BaudRate = BaudRate;
		if (HAL_UARTEx_DisableFifoMode(huart) != HAL_OK) {
			Error_Handler();
		}
		if (HAL_UART_Init(huart) != HAL_OK) {
			Error_Handler();
		}
		if (HAL_UARTEx_EnableFifoMode(huart) != HAL_OK) {
			Error_Handler();
		}
	}
}


void shutdownAll()
{
	gatedriverDC(0);
	contactorBattery(0);
}


void calc_and_wait(uint32_t delay)
{
	resetWatchdog();
	calc_async_dc_control();

	for (uint32_t i=0; i<delay; i++) {
		calc_async_dc_control();
		//if (!monitoring_binary_en) {
			HAL_Delay(1);  //ms
		//}
	}
//			reinitUART(huart_rs485, 115200);
//			calc_async_dc_control();
//				send_inverterdata();
#if 0  // Modbus
				// receive commands from external energy management system
//#define RX_MAX_CMD_LEN (sizeof("p_bat_chg_max65535\n")+5)
#define RX_MAX_CMD_LEN (sizeof(modbus_param_rw_t)+8)
				uint8_t rx_buf[RX_MAX_CMD_LEN];

				for (uint16_t i = 0; i < 10; i++) {
					uint16_t rx_len = 0;
					HAL_UARTEx_ReceiveToIdle(huart_rs485, rx_buf, RX_MAX_CMD_LEN, &rx_len, 10);  // 10 x 10ms rx window

					for (uint16_t j = 0; j < rx_len; j++) {
						mbus_poll(modbus, rx_buf[j]);
					}
				}
				apply_sys_mode_cmd(&ctrl_ref);
				mbus_flush(modbus);  // reset incomplete messages
#endif  // Modbus

	resetWatchdog();
	async_battery_communication();
	resetWatchdog();
}


#define IDC_OFFSET_RAW 2636  // at ~18°C after microcontroller start -40mA instead 0mA at 21°C
#define IDC_mV_per_LSB (3300.0/4096)  // 3.3V 12bit
#define IDC_mV_per_A 35 // current sensor datasheet 35mV/A
#define IDC_RAW_TO_10mA (-IDC_mV_per_LSB * 100.0/IDC_mV_per_A)  // = 2.301897321  // sign inversion (pos means battery charging)
#define CNT_I_DC_AVG 16  // +3bit in hardware


// +-50A sensor range with 3.3V ADC:
// −62.1A to +32.1A
// analog watchdog set to
// +10Ampere -> 1000/IDC_RAW_TO_10mA + 2632 = 3066
// -10Ampere -> -1000/IDC_RAW_TO_10mA + 2632 = 2198
void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef* hadc)
{
	shutdownAll();

	i_dc_filt_10mA = (HAL_ADC_GetValue(&hadc2) - CNT_I_DC_AVG*IDC_OFFSET_RAW)/CNT_I_DC_AVG * IDC_RAW_TO_10mA;

	if (i_dc_filt_10mA > 0) {
		set_sys_errorcode(EC_BATTERY_I_CHARGE_MAX);
	} else {
		set_sys_errorcode(EC_BATTERY_I_DISCHARGE_MAX);
	}
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	checkErrors();
#if 1
	static uint32_t blankingCnt = 0;

	if (blankingCnt > 240) {  // dont check sigmadelta data during startup for 240/20kHz/3ADCs = 4ms
		errorPVBI_t limitErr = checkDCLimits();
		if (limitErr != EC_NO_ERROR) {
			shutdownAll();
			set_sys_errorcode(limitErr);
		}
	} else {
		blankingCnt++;
	}

	//static volatile uint16_t vdc_sinc_mix_100mV;
	static volatile uint16_t cnt20kHz_20ms = 0;

	// ADC1_IN8 = PC2 = Vdc measurement with resistive voltage divider
	// one channel -> called once
    if (hadc == &hadc1) {

    	// Battery Vdc measurement
    	// V4: 2x330k THT + 2x330k SMD
    	// C=4.7uF
    	// 3.3V×((2×660+10)÷10) = 438.9V
// V0:
//#define VDC_mV_per_LSB (438900/4096)  // 438.9Vmax -> 3.3V 12bit
    	//Vdc_bus 31.2   raw=293   V_multimeter=38.35V
    	//Vdc_bus 359.1  raw=3361  V_multimeter=365.1V
// V1: calib
//#define VDC_OFFSET_RAW 67
//#define VDC_mV_per_LSB (436235/4096)  // 436.235Vmax -> 3.3V 12bit
// Problem: idle 7.1V

// V2: calib with idle < 1V
#define VDC_OFFSET_RAW 8
#define VDC_mV_per_LSB (443763/4096)

//V3:  Oszi 312V Amprobe AM-550-EUR: 306.5V
//     Vdc_bus 297.1  raw=2744
//     v_dc_FBboost_filt50Hz308.1
//#define VDC_OFFSET_RAW 8
//#define VDC_mV_per_LSB (456000/4096)

//V4: Lower voltage without parallel Multimeter -> using V2 again
//    	Vdc_bus 354.5  raw=3186
//    	v_dc_FBboost_filt50Hz347.3

		int v_dc_raw_no_calib = ADC1ConvertedData[0];
		debug_v_dc_raw = v_dc_raw_no_calib;
		v_dc_bus_100mV = ((v_dc_raw_no_calib+VDC_OFFSET_RAW)*VDC_mV_per_LSB)/100;

		//uint16_t v_dc_FBboost_sincfilt_100mV = get_v_dc_FBboost_sincfilt_100mV();
		//uint16_t v_dc_FBboost_filt50Hz_100mV = get_v_dc_FBboost_filt50Hz_100mV();
    }


    if (hadc == &hadc2) {
	// called with f=40kHz when repetition counter = 0 -> for double current sense
	// called with f=20kHz when repetition counter = 1 -> for single current sense or hardware oversampling
	// called with f=20kHz when repetition counter = 3 and PWM period for 40kHz
	//
	// In interleaved DCDC booster with diodes, discontinuous current can occur -> Sampling the whole period is necessary
	// fadc = 170MHz/4 = 42.5MHz
	// cycles = 2.5+12.5 = 15
	// fsample = 42.5MHz/15 = 2.833Msamples/s
	// oversampling 128 -> 22.135kHz
	// todo: 22kHz vs 20kHz : no exact average current, but robust for MPPT and much better in discontinuous mode than double sample
		//GPIOC->BSRR = (1<<4);  // set Testpin TP201 PC4

		measVdcFBboost();

		i_dc_filt_10mA = (ADC2ConvertedData[0]-CNT_I_DC_AVG*IDC_OFFSET_RAW)/CNT_I_DC_AVG * IDC_RAW_TO_10mA;

		cnt20kHz_20ms++;

		if (cnt20kHz_20ms >= CYCLES_cnt20kHz_20ms ) {
			id++;
			cnt20kHz_20ms = 0;

			static uint16_t cnt50Hz_1s = 0;
			cnt50Hz_1s++;
			if (cnt50Hz_1s >= 50 ) {
				cnt50Hz_1s = 0;
				battery_update_request();
				print_request = true;
			}
		}

		int16_t dutyHS = dcControlStep(cnt20kHz_20ms, ctrl_ref.v_dc_100mV, i_dc_filt_10mA);

		// Test using AC-DC trafo rectifier with Vin=150V:

		//dutyHS = DEF_MPPT_DUTY_ABSMAX*0.75;  // boost to 200V
		// Iripple=0,25*25us*150V/130uH =       7.2A  6.4Ameas  Vdc_bus 188.6 Sigmadelta=207.0
		// Iripple=0,25*25us*150V/(130uH + 1mH)=0.83A 0.36Ameas Vdc_bus 197.3 Sigmadelta=214.1

		//dutyHS = DEF_MPPT_DUTY_ABSMAX*0.5;  // boost to 300V; UART shows wrong letters -> fix by GND cable wound around RX
		// Iripple=0,5*25us*150V/(130uH + 1mH)=1.66A 0.70Ameas Vdc_bus 292.1 Sigmadelta=297.2

		//dutyHS = DEF_MPPT_DUTY_ABSMAX*0.4;  // boost to 375V; UART shows wrong letters, USB reset
		// Iripple=0,6*25us*150V/(130uH + 1mH)=2.00A  0.80Ameas Vdc_bus 363.2 Sigmadelta=366.6 Multimeter=365V
		//  PacELV=12W (1,9W im Notaus bei 150Vdc) -> bei 90% Trafo Wirkungsgrad, ca. 9W Drossel + Halbleiterverluste


		int16_t dutyB1 = DEF_MPPT_DUTY_ABSMAX;  // Highside switch on
//		int16_t dutyB2 = DEF_MPPT_DUTY_ABSMAX;  // Highside switch on

		switch(dcdc_mode) {
//			case DCDC_HB1:
//				dutyB1 = dutyHS;
//				break;
//
//			case DCDC_HB2:
//				dutyB2 = dutyHS;
//				break;
//
//			case DCDC_INTERLEAVED:
//				dutyB1 = dutyHS;
//				dutyB2 = dutyHS;
//				break;

			default:
				dutyB1 = dutyHS;
				break;
		}

		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, dutyB1);  // update pwm value
		//__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, DEF_MPPT_DUTY_ABSMAX-dutyB2);

		//GPIOC->BRR = (1<<4);  // reset Testpin TP201 PC4
    }
#endif
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
  MX_ADC2_Init();
  MX_FDCAN2_Init();
  MX_HRTIM1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_UART5_Init();
  MX_USART3_UART_Init();
  MX_IWDG_Init();
  MX_LPTIM1_Init();
  /* USER CODE BEGIN 2 */

  /* Configure the FDCAN peripheral */
  FDCAN_Config();  // CAN2 is connected to Battery Cell Stack Controller CAN Bus

// Set to 1 to swap UARTs for Modbus debugging without using RS485 transceivers
#if 0
  huart_rs485 = &huart3;
  huart_debug = &huart5;
#else  // production
  huart_rs485 = &huart5;  // binary communication
  huart_debug = &huart3;  // text messages
#endif

  // DCDC sw-freq: 20kHz 170MHz/20kHz = 8500 -> Period = 4249
  // DCDC ctrl-freq: 100Hz -> Repetition Counter = 200
  // Todo check ADC clock

  // ###########################
  // ### Timer configuration ###
  // ###########################

  // start PWM for PV boost Half-bridge 1 (PWMA)
  if (   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK) {
    /* PWM Generation Error */
    Error_Handler();
  }
  // start PWM for PV boost Half-bridge 2 (PWMB)
  if (  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) {
    /* PWM Generation Error */
    Error_Handler();
  }
  // start PWM timer for PV boost Full-bridge
  if (  HAL_TIM_Base_Start_IT(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  // start timer for 1Hz system timebase
  #define LPTIM_TICKS_PER_SEC (32000/128)  // timer has 32kHz oscillator and DIV128 -> 250Hz
  if (HAL_LPTIM_Counter_Start_IT(&hlptim1, LPTIM_TICKS_PER_SEC) != HAL_OK) {
    Error_Handler();
  }

  // start Sigma-delta-ADC counter for Vdc FB_boost
  HAL_TIM_Base_Start(&htim4);


  // #########################
  // ### DMA configuration ###
  // #########################
  if ( HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC1ConvertedData, sizeof(ADC1ConvertedData)/sizeof(ADC1ConvertedData[0]))  != HAL_OK)
  {
    Error_Handler();
  }

  if ( HAL_ADC_Start_DMA(&hadc2, (uint32_t *)ADC2ConvertedData, sizeof(ADC2ConvertedData)/sizeof(ADC2ConvertedData[0]))  != HAL_OK)
  {
    Error_Handler();
  }

  //__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 2125);  //update pwm value 50% dutycycle

  uSend("Battery Supervisor  ");
  uSend(__DATE__ " ");
  uSend(__TIME__);
  uSend("\n");
  GPIOB->BSRR = (1<<0);  // disable red LED

  contactorBattery(0);

  /*## Check if the system has resumed from IWDG reset ####################*/
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != 0x00u)
  {
    uSend("Watchdog caused reset!");
    uSend("\n");
    set_sys_errorcode(EC_WATCHDOG_RESET);  // prevent ongoing inverter turnon in case of software bug
    HAL_Delay(300);  //ms
  }
  /* Clear reset flags anyway */
  __HAL_RCC_CLEAR_RESET_FLAGS();

  bool uart_output_text = true;
  bool uart_input_text = true;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	checkErrors();
	if (get_sys_mode() != OFF) GPIOB->BRR = (1<<1);  // enable green LED
	calc_and_wait(250);  //ms
	calc_and_wait(250);  //ms
	GPIOB->BSRR = (1<<1);  // disable green LED
	calc_and_wait(250);  //ms

	sys_mode_ctrl_step(&ctrl_ref);


	// UART
	if (uart_output_text) {

		calc_and_wait(250);  //ms

#if SYSTEM_HAS_BATTERY == 1
		const batteryStatus_t* battery = get_batteryStatus();
#endif //SYSTEM_HAS_BATTERY

#if 1  // debug print battery info
		/************************/
		/* print battery status */
		/************************/

		uSend("\nBatteryStatus\n");

		for (int c=0; c<(4*14); c++) {

			uSend("Temp");
			if (c<=8)
				uSend(" ");
			uSendInt(c+1);
			uSend(" ");
			uSendInt(get_battery_temperature(c));
			uSend("\n");
		}

		uint8_t nr_cells_balancing = 0;

		if (battery->voltage_100mV != 0) {
			for (int c=0; c<96; c++) {
				if (c%12 == 0) uSend(" ");
				uSend("Vc");
				if (c<=8)
					uSend(" ");
				uSendInt(c+1);
				uSend(" ");
				uSend_1m(get_battery_vCell_mV(c));

				if (get_battery_balancingState(c)) {
					uSend(" balancing");
					nr_cells_balancing++;
				}
				uSend("\n");
			}
		}
		uSend("nr_cells_balancing ");
		uSendInt(nr_cells_balancing);
		uSend("\n");

		int16_t vCellDIFF_mV = battery->maxVcell_mV - battery->minVcell_mV;
		uSend("vCellDIFF_mV ");
		uSend_1m(vCellDIFF_mV);
		uSend("\n");

		for (int csc=0; csc<4;csc++) {
			if (get_battery_csc_err(csc)) {
				uSend("Err in CSC");
				uSendInt(csc+1);
				uSend("\n");
				for (uint8_t bit=0; bit<64; bit++) {
					if ( (1<<bit) & get_battery_csc_err(csc)) {
						const char* strerr = get_battery_csc_err_str(bit);
						tx_len=strlen(strerr);
						HAL_UART_Transmit(huart_debug, (uint8_t *)strerr, tx_len, 10);
						uSend("\n");
					}
				}
			}
		}
#endif

		if (uart_output_text) {
		//if (print_request && uart_output_text) {
			print_request = false;

			if (get_sys_errorcode() != EC_NO_ERROR) {
				uSend("E ");
				itoa(get_sys_errorcode(), tx_buf, 10);
				tx_len=strlen(tx_buf);
				HAL_UART_Transmit(huart_debug, (uint8_t *)tx_buf, tx_len, 10);
				uSend(" ");
				char * strerr = strerror(get_sys_errorcode());
				tx_len=strlen(strerr);
				HAL_UART_Transmit(huart_debug, (uint8_t *)strerr, tx_len, 10);
				uSend("\n");
			}

			uSend("\nS ");
			itoa(get_sys_mode(), tx_buf, 10);
			tx_len=strlen(tx_buf);
			HAL_UART_Transmit(huart_debug, (uint8_t *)tx_buf, tx_len, 10);

			uSend("\nDC ");
			itoa(stateDC, tx_buf, 10);
			tx_len=strlen(tx_buf);
			HAL_UART_Transmit(huart_debug, (uint8_t *)tx_buf, tx_len, 10);
#if SYSTEM_HAS_BATTERY == 1
			uSend(" Bat ");
			itoa(get_stateBattery(), tx_buf, 10);
			tx_len=strlen(tx_buf);
			HAL_UART_Transmit(huart_debug, (uint8_t *)tx_buf, tx_len, 10);
#endif //SYSTEM_HAS_BATTERY
			uSend("\n");

#if SYSTEM_HAS_BATTERY == 1
			uSend("Vbat ");
			const batteryStatus_t* battery = get_batteryStatus();
			uSend_100m(battery->voltage_100mV);
			uSend("  ");
			uSend_1m(battery->minVcell_mV);
			uSend("  ");
			uSend_1m(battery->maxVcell_mV);
			uSend("\n");

			uSend("Tmin");
			uSendInt(battery->minTemp);
			uSend("  Tmax");
			uSendInt(battery->maxTemp);
			uSend("\n");
#endif //SYSTEM_HAS_BATTERY

	//		uSend("VdcFBboost_sinc ");
	//		uSend_100m(VdcFBboost_sincfilt_100mV);
	//		uSend("\n");
	//
	//		uSend("VdcFBgrid_sinc  ");
	//		uSend_100m(VdcFBgrid_sincfilt_100mV);
	//		uSend("\n");

			uSend("Vdc_bus ");
			uSend_100m(v_dc_bus_100mV);
			uSend("  raw=");
			uSendInt(debug_v_dc_raw);
			uSend("\n");

			uSend("v_dc_FBboost_filt50Hz");
			uSend_100m(get_v_dc_FBboost_filt50Hz_100mV());
			uSend("\n");

#if SYSTEM_HAS_BATTERY == 1
			uSend("Pb ");
			uSendInt(battery->power_W);
			uSend("\n");

			uSend("SoC ");
			uSendInt(battery->soc_percent);
			uSend("\n");
#endif //SYSTEM_HAS_BATTERY

			uSend("Idc ");
			uSend_10m(i_dc_filt_10mA);

	//		uSend("Idc ");
	//		uSend_10m(Idc_filt_10mA);
	//		uSend("\n");
		//	// no current 4avg: 2632-2634
		//	// ~-1A : 2592  ->LSB 23,8mA;  ~+1A 2675
		//
		//	uSend("Vg_raw_filt ");
		//	itoa(debug_Vg_raw_filt, Tx_Buffer, 10);
		//	Tx_len=strlen(Tx_Buffer);
		//	HAL_UART_Transmit(huart_debug, (uint8_t *)Tx_Buffer, Tx_len, 10);
		//	uSend("\n");
		//	// no voltage 4avg: 1971-1982
		//	// +5V + an N  - an L: 1995-2005  5V/24=0,2V per LSB
		//	// -5V: 1955-1964
		//
		//
		//	uSend("Ig_raw_filt ");
		//	itoa(debug_Ig_raw_filt, Tx_Buffer, 10);
		//	Tx_len=strlen(Tx_Buffer);
		//	HAL_UART_Transmit(huart_debug, (uint8_t *)Tx_Buffer, Tx_len, 10);
		//	uSend("\n");
		//	// no current 4avg: 2627-2630
		//	// ~+1A 2671
		}
	}

	// UART RX
	uint8_t rx_buf = 0;
	HAL_UART_Receive(huart_debug, &rx_buf, 1, 1);  // todo: watchdog is triggered if this line missing in monitor mode

	if (uart_input_text) {
		if (rx_buf == 'p') {
			uSend("Print ENABLE\n");
			uart_output_text = true;
		} else if (rx_buf == 'd') {
			uSend("Print DISABLE\n");
			uart_output_text = false;
//		} else if (rx_buf == 'm') {
//			uSend("Monitoring ENABLE\n");
//			monitoring_binary_en = true;
//			uart_output_text = false;
//			uart_input_text = false;
//		} else if (rx_buf == 'f') {
//			uSend("Fast monitor TRIG\n");
//			fast_mon_vars_trig = true;
//			uart_output_text = false;
//		} else if (rx_buf == 'c') {
//			uSend("Contactor Enable\n");
//			contactorBattery(1);
		} else if (rx_buf == 'b') {
			uSend("BALANCING ENABLE\n");
			battery_set_balancing(0b1111, 3600);  // all CSCs
		} else	if (rx_buf == 's') {
			uSend("BALANCING DISABLE\n");
			battery_set_balancing(0b0, 3600);
		}
	}

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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_8;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_3;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = ENABLE;
  hadc2.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_128;
  hadc2.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_3;
  hadc2.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc2.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analog WatchDog 1
  */
  AnalogWDGConfig.WatchdogNumber = ADC_ANALOGWATCHDOG_1;
  AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
  AnalogWDGConfig.Channel = ADC_CHANNEL_11;
  AnalogWDGConfig.ITMode = ENABLE;
  AnalogWDGConfig.HighThreshold = 3066;
  AnalogWDGConfig.LowThreshold = 2198;
  AnalogWDGConfig.FilteringConfig = ADC_AWD_FILTERING_2SAMPLES;
  if (HAL_ADC_AnalogWDGConfig(&hadc2, &AnalogWDGConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief FDCAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = DISABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 68;
  hfdcan2.Init.NominalSyncJumpWidth = 1;
  hfdcan2.Init.NominalTimeSeg1 = 2;
  hfdcan2.Init.NominalTimeSeg2 = 2;
  hfdcan2.Init.DataPrescaler = 1;
  hfdcan2.Init.DataSyncJumpWidth = 1;
  hfdcan2.Init.DataTimeSeg1 = 1;
  hfdcan2.Init.DataTimeSeg2 = 1;
  hfdcan2.Init.StdFiltersNbr = 2;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */

  /* USER CODE END FDCAN2_Init 2 */

}

/**
  * @brief HRTIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_HRTIM1_Init(void)
{

  /* USER CODE BEGIN HRTIM1_Init 0 */

  /* USER CODE END HRTIM1_Init 0 */

  HRTIM_ADCTriggerCfgTypeDef pADCTriggerCfg = {0};
  HRTIM_TimeBaseCfgTypeDef pTimeBaseCfg = {0};
  HRTIM_TimerCtlTypeDef pTimerCtl = {0};
  HRTIM_TimerCfgTypeDef pTimerCfg = {0};
  HRTIM_CompareCfgTypeDef pCompareCfg = {0};
  HRTIM_OutputCfgTypeDef pOutputCfg = {0};

  /* USER CODE BEGIN HRTIM1_Init 1 */

  /* USER CODE END HRTIM1_Init 1 */
  hhrtim1.Instance = HRTIM1;
  hhrtim1.Init.HRTIMInterruptResquests = HRTIM_IT_NONE;
  hhrtim1.Init.SyncOptions = HRTIM_SYNCOPTION_NONE;
  if (HAL_HRTIM_Init(&hhrtim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_DLLCalibrationStart(&hhrtim1, HRTIM_CALIBRATIONRATE_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_PollForDLLCalibration(&hhrtim1, 10) != HAL_OK)
  {
    Error_Handler();
  }
  pADCTriggerCfg.UpdateSource = HRTIM_ADCTRIGGERUPDATE_TIMER_A;
  pADCTriggerCfg.Trigger = HRTIM_ADCTRIGGEREVENT13_TIMERA_PERIOD;
  if (HAL_HRTIM_ADCTriggerConfig(&hhrtim1, HRTIM_ADCTRIGGER_1, &pADCTriggerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_ADCPostScalerConfig(&hhrtim1, HRTIM_ADCTRIGGER_1, 0x0) != HAL_OK)
  {
    Error_Handler();
  }
  pTimeBaseCfg.Period = 8500;
  pTimeBaseCfg.RepetitionCounter = 0x00;
  pTimeBaseCfg.PrescalerRatio = HRTIM_PRESCALERRATIO_MUL2;
  pTimeBaseCfg.Mode = HRTIM_MODE_CONTINUOUS;
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCtl.UpDownMode = HRTIM_TIMERUPDOWNMODE_UPDOWN;
  pTimerCtl.GreaterCMP1 = HRTIM_TIMERGTCMP1_EQUAL;
  pTimerCtl.DualChannelDacEnable = HRTIM_TIMER_DCDE_DISABLED;
  if (HAL_HRTIM_WaveformTimerControl(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pTimerCtl) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_RollOverModeConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_TIM_FEROM_BOTH|HRTIM_TIM_BMROM_BOTH
                              |HRTIM_TIM_ADROM_BOTH|HRTIM_TIM_OUTROM_BOTH
                              |HRTIM_TIM_ROM_BOTH) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCfg.InterruptRequests = HRTIM_TIM_IT_NONE;
  pTimerCfg.DMARequests = HRTIM_TIM_DMA_NONE;
  pTimerCfg.DMASrcAddress = 0x0000;
  pTimerCfg.DMADstAddress = 0x0000;
  pTimerCfg.DMASize = 0x1;
  pTimerCfg.HalfModeEnable = HRTIM_HALFMODE_DISABLED;
  pTimerCfg.InterleavedMode = HRTIM_INTERLEAVED_MODE_DISABLED;
  pTimerCfg.StartOnSync = HRTIM_SYNCSTART_DISABLED;
  pTimerCfg.ResetOnSync = HRTIM_SYNCRESET_DISABLED;
  pTimerCfg.DACSynchro = HRTIM_DACSYNC_NONE;
  pTimerCfg.PreloadEnable = HRTIM_PRELOAD_ENABLED;
  pTimerCfg.UpdateGating = HRTIM_UPDATEGATING_INDEPENDENT;
  pTimerCfg.BurstMode = HRTIM_TIMERBURSTMODE_MAINTAINCLOCK;
  pTimerCfg.RepetitionUpdate = HRTIM_UPDATEONREPETITION_ENABLED;
  pTimerCfg.PushPull = HRTIM_TIMPUSHPULLMODE_DISABLED;
  pTimerCfg.FaultEnable = HRTIM_TIMFAULTENABLE_NONE;
  pTimerCfg.FaultLock = HRTIM_TIMFAULTLOCK_READWRITE;
  pTimerCfg.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_DISABLED;
  pTimerCfg.DelayedProtectionMode = HRTIM_TIMER_A_B_C_DELAYEDPROTECTION_DISABLED;
  pTimerCfg.UpdateTrigger = HRTIM_TIMUPDATETRIGGER_NONE;
  pTimerCfg.ResetTrigger = HRTIM_TIMRESETTRIGGER_NONE;
  pTimerCfg.ResetUpdate = HRTIM_TIMUPDATEONRESET_DISABLED;
  pTimerCfg.ReSyncUpdate = HRTIM_TIMERESYNC_UPDATE_CONDITIONAL;
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = 4250;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pOutputCfg.Polarity = HRTIM_OUTPUTPOLARITY_LOW;
  pOutputCfg.SetSource = HRTIM_OUTPUTSET_TIMCMP1;
  pOutputCfg.ResetSource = HRTIM_OUTPUTRESET_NONE;
  pOutputCfg.IdleMode = HRTIM_OUTPUTIDLEMODE_NONE;
  pOutputCfg.IdleLevel = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
  pOutputCfg.FaultLevel = HRTIM_OUTPUTFAULTLEVEL_NONE;
  pOutputCfg.ChopperModeEnable = HRTIM_OUTPUTCHOPPERMODE_DISABLED;
  pOutputCfg.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA1, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pOutputCfg.Polarity = HRTIM_OUTPUTPOLARITY_HIGH;
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA2, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN HRTIM1_Init 2 */

  /* USER CODE END HRTIM1_Init 2 */
  HAL_HRTIM_MspPostInit(&hhrtim1);

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_8;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief LPTIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPTIM1_Init(void)
{

  /* USER CODE BEGIN LPTIM1_Init 0 */

  /* USER CODE END LPTIM1_Init 0 */

  /* USER CODE BEGIN LPTIM1_Init 1 */

  /* USER CODE END LPTIM1_Init 1 */
  hlptim1.Instance = LPTIM1;
  hlptim1.Init.Clock.Source = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
  hlptim1.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV128;
  hlptim1.Init.Trigger.Source = LPTIM_TRIGSOURCE_SOFTWARE;
  hlptim1.Init.OutputPolarity = LPTIM_OUTPUTPOLARITY_HIGH;
  hlptim1.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;
  hlptim1.Init.CounterSource = LPTIM_COUNTERSOURCE_INTERNAL;
  hlptim1.Init.Input1Source = LPTIM_INPUT1SOURCE_GPIO;
  hlptim1.Init.Input2Source = LPTIM_INPUT2SOURCE_GPIO;
  if (HAL_LPTIM_Init(&hlptim1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPTIM1_Init 2 */

  /* USER CODE END LPTIM1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 2124;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 3;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_ETRMODE2;
  sClockSourceConfig.ClockPolarity = TIM_CLOCKPOLARITY_NONINVERTED;
  sClockSourceConfig.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;
  sClockSourceConfig.ClockFilter = 0;
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
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 9600;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart5, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart5, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_EnableFifoMode(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_EnableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_0|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_14
                          |GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC0 PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_0|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB10 PB14
                           PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_14
                          |GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief  Configures the FDCAN.
  * @param  None
  * @retval None
  */
static void FDCAN_Config(void)
{
  /* Configure global filter:
     Filter all remote frames with STD and EXT ID
     Reject non matching frames with STD ID and EXT ID */
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }

  /* Start the FDCAN module */
  if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }

  // use polling for now
//  if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
//  {
//    Error_Handler();
//  }

}

/**
  * @brief  Rx FIFO 0 callback.
  * @param  hfdcan: pointer to an FDCAN_HandleTypeDef structure that contains
  *         the configuration information for the specified FDCAN.
  * @param  RxFifo0ITs: indicates which Rx FIFO 0 interrupts are signalled.
  *         This parameter can be any combination of @arg FDCAN_Rx_Fifo0_Interrupts.
  * @retval None
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
//  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
//  {
//    /* Retrieve Rx messages from RX FIFO0 */
//    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
//    {
//    Error_Handler();
//    }
//
//    if ((RxHeader.Identifier == 0x40D) && (RxHeader.IdType == FDCAN_STANDARD_ID))// && (RxHeader.DataLength == FDCAN_DLC_BYTES_8))
//    {
//    	uSend("\nRX ID: ");
//    	uSendInt(RxHeader.Identifier);
//    	uSend("\nRX DLC: ");
//    	uSendInt(RxHeader.DataLength);
//
//    	//csc_cell_volt_min_max_t *csc_cell_volt_min_max = &RxData[0];
//
//    	uSend("\ncell_voltage_max_mV: ");
//    	uSendInt(((csc_cell_volt_min_max_t *)RxData)->cell_voltage_max_mV);
//    } else {
//    	uSend("invalid CAN msg\n");
//    }
//  }
}

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
	  shutdownAll();
	  uSend("Error_Handler\n");
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
