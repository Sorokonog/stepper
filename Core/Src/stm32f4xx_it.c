/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
__fp16 f16_tester = 0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
// CAN Variables
extern uint8_t TxData[8];					// Data to be sent via CAN
extern uint64_t TxMailbox;					// CAN temporary mailbox. Required by HAL function
extern CAN_TxHeaderTypeDef TxHeader;		// Header for can message
extern CAN_FilterTypeDef canfilterconfig;	// Filter for receiving CAN messages

extern uint8_t RxData[8];					// data received from can bus
extern CAN_RxHeaderTypeDef   RxHeader;		// header received by can bus

//extern uint32_t my_device_id;

void My_CAN_Rx_Handler( CAN_RxHeaderTypeDef* RxHeader, uint8_t* RxData);
// encoder current and previous angle
extern float angle_rad;
float angle_rad_prev = 0;
extern uint8_t target_velocity;	// targeted velocity
extern float target_angle; 		//target angle
extern uint8_t target_effort;
extern uint8_t velocity;			//current velocity

extern char goal_status;		// set to 1  if the last goal has been reached.
extern char stall_status; 	// set to 1  if the stepper stall has been detected.


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
extern void CTRL_Reg_Set();

float read_angle();	// reads current encoder angle (converted to rad)

float Ticks_To_Angle(int ticks);		// convert encoder ticks to absolute angle
void Angle_To_CAN_Data(float angle, float speed, char goal_status, char stall_status ,uint8_t* TxData); // convert angle to CAN message
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern I2C_HandleTypeDef hi2c1;

extern SPI_HandleTypeDef hspi1;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim7;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles CAN1 RX0 interrupt.
  */
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */
	//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData);	// reads CAN message
	My_CAN_Rx_Handler(&RxHeader, RxData);	// My handler function
  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */
  angle_rad = read_angle();
  // 	convert and send CAN message
  Angle_To_CAN_Data(angle_rad, velocity, goal_status, stall_status, TxData);
  if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 0){
  //		flag_tim7++;
  		if ( HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK){
  			Error_Handler();
  		}
  	}

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void My_CAN_Rx_Handler( CAN_RxHeaderTypeDef* RxHeader, uint8_t* RxData){

	//*TxData = &RxData;
	 for(int i=0; i<sizeof(RxData); i++)
	  {
		TxData[i] = RxData[i];
	  }

	 // checks received ID against this device's ID

	 if(RxHeader->StdId == 1)
	 {
			float* temp_angle = RxData;

			target_angle = *temp_angle;

			char* temp = RxData + 4;

			target_velocity = *temp;

			temp = RxData + 5;

			target_effort = *temp;

			goal_status = 0; //possible not usefull

			char* temp_char = temp_angle;

			TxData[0] = *temp_char;
			TxData[1] = *++temp_char;
			TxData[2] = *++temp_char;
			TxData[3] = *++temp_char;
			TxData[4] = 1;
			TxData[5] = target_effort;
			TxData[6] = 0;
			TxData[7] = target_velocity;


			if(target_velocity < 127)
				 {
					 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
				 }
				 else
				 {
					 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
				 }
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
			for(int i=0;i<1000000;i++)
			{}
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
			for(int i=0;i<1000000;i++)
			{}
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
	 }

	if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 0){
		if ( HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK){
			Error_Handler();
		}
	}
	return;
}

float Ticks_To_Angle(int ticks){
	return  (ticks)* M_PI * 2.0  / 4096.0;
}

float read_angle(){
	HAL_StatusTypeDef ret;
	uint8_t buf[12];	// temporaty buffer for i2c message transmition
	uint8_t buf_recv[12];	// temporaty buffer for i2c message reception
	buf[0] = 0x0E;
	ret = HAL_I2C_Master_Transmit(&hi2c1, (0b0110110 << 1) , buf, 1, HAL_MAX_DELAY);
	if(ret == HAL_OK ){
//		flag ++;
	}
	ret = HAL_I2C_Master_Receive(&hi2c1, 0b0110110 << 1, buf_recv, 2, HAL_MAX_DELAY);
	if(ret == HAL_OK ){
//		flag --;
	}
	angle_rad = Ticks_To_Angle(buf_recv[1] + buf_recv[0] * 256);
	return angle_rad;
}

void Angle_To_CAN_Data(float angle, float speed, char goal_status, char stall_status ,uint8_t* TxData){
	// pack angle
		__fp16 temp_float16 = angle;
		char* temp_char = &temp_float16;
		TxData[0] = temp_char[0];
		TxData[1] = temp_char[1];
		//pack velocity
		temp_float16 = speed;
		temp_char = &temp_float16;
		TxData[2] = temp_char[2];
		TxData[3] = temp_char[3];
		//pack goal_status. 0 if goal not reached, 1 if it is reached.
		TxData[4] = goal_status;
		//pack stall_status. 0 if the motor has not stalled, 1 if it has.
		TxData[5] = stall_status;
}
/* USER CODE END 1 */
