/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "CO_app_STM32.h"
#include "CANopen.h"
#include "../CANopenNode/301/CO_SDOclient.h"
#include "../CANopenNode/301/CO_NMT_Heartbeat.h"
#include "../CANopenNode/301/CO_PDO.h"
#include "OD.h"
#include "math.h"


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
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

TIM_HandleTypeDef htim14;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
CO_SDO_abortCode_t read_SDO(CO_SDOclient_t *SDO_C, uint8_t nodeId,
                            uint16_t index, uint8_t subIndex,
                            uint8_t *buf, size_t bufSize, size_t *readSize)
{
    CO_SDO_return_t SDO_ret;

    // setup client (this can be skipped, if remote device don't change)
    SDO_ret = CO_SDOclient_setup(SDO_C,
                                 CO_CAN_ID_SDO_CLI + nodeId,
                                 CO_CAN_ID_SDO_SRV + nodeId,
                                 nodeId);
    if (SDO_ret != CO_SDO_RT_ok_communicationEnd) {
        return CO_SDO_AB_GENERAL;
    }

    // initiate upload
    SDO_ret = CO_SDOclientUploadInitiate(SDO_C, index, subIndex, 1000, false);
    if (SDO_ret != CO_SDO_RT_ok_communicationEnd) {
        return CO_SDO_AB_GENERAL;
    }

    // upload data
    do {
        uint32_t timeDifference_us = 10000;
        CO_SDO_abortCode_t abortCode = CO_SDO_AB_NONE;

        SDO_ret = CO_SDOclientUpload(SDO_C,
                                     timeDifference_us,
                                     false,
                                     &abortCode,
                                     NULL, NULL, NULL);
        if (SDO_ret < 0) {
            return abortCode;
        }

        HAL_Delay(1);
    } while(SDO_ret > 0);

    // copy data to the user buffer (for long data function must be called
    // several times inside the loop)
    *readSize = CO_SDOclientUploadBufRead(SDO_C, buf, bufSize);

    return CO_SDO_AB_NONE;
}

CO_SDO_abortCode_t write_SDO(CO_SDOclient_t *SDO_C, uint8_t nodeId,
                             uint16_t index, uint8_t subIndex,
                             uint8_t *data, size_t dataSize)
{
    CO_SDO_return_t SDO_ret;
    bool_t bufferPartial = false;

    // setup client (this can be skipped, if remote device is the same)
    SDO_ret = CO_SDOclient_setup(SDO_C,
                                 CO_CAN_ID_SDO_CLI + nodeId,
                                 CO_CAN_ID_SDO_SRV + nodeId,
                                 nodeId);
    if (SDO_ret != CO_SDO_RT_ok_communicationEnd) {
        return -1;
    }

    // initiate download
    SDO_ret = CO_SDOclientDownloadInitiate(SDO_C, index, subIndex,
                                           dataSize, 1000, false);
    if (SDO_ret != CO_SDO_RT_ok_communicationEnd) {
        return -1;
    }

    // fill data
    size_t nWritten = CO_SDOclientDownloadBufWrite(SDO_C, data, dataSize);
    if (nWritten < dataSize) {
        bufferPartial = true;
        // If SDO Fifo buffer is too small, data can be refilled in the loop.
    }

    //download data
    do {
        uint32_t timeDifference_us = 10000;
        CO_SDO_abortCode_t abortCode = CO_SDO_AB_NONE;

        SDO_ret = CO_SDOclientDownload(SDO_C,
                                       timeDifference_us,
                                       false,
                                       bufferPartial,
                                       &abortCode,
                                       NULL, NULL);
        if (SDO_ret < 0) {
            return abortCode;
        }

        HAL_Delay(1);
    } while(SDO_ret > 0);

    return CO_SDO_AB_NONE;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	  int i=0;
	  int s = 4500000;
	  int16_t next;
	  int16_t prev;
	  int32_t xi;
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
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
  CANopenNodeSTM32 canopenNodeSTM32;
    canopenNodeSTM32.CANHandle = &hcan2;
    canopenNodeSTM32.HWInitFunction = MX_CAN2_Init;
    canopenNodeSTM32.timerHandle = &htim14;
    canopenNodeSTM32.desiredNodeID = 7;
    canopenNodeSTM32.baudrate = 1000;
    canopen_app_init(&canopenNodeSTM32);


    uint16_t tmp1 = 0x80;
    uint16_t tmp2 = 0x00;
    uint16_t tmp3 = 0x06;
    uint16_t tmp4 = 0x07;
    uint16_t tmp5 = 0x0F;

    int8_t interpolation_mode = 0x07;

    int16_t intrpolation_sub_mode_select = 0;

    uint8_t interpolation_time_unit = 10;
    int8_t interpolation_time_index = -3;

//control_word for node 1(Gearbox1)...............................................................

    write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 1, 0x6040, 00, &tmp1, 2);
    HAL_Delay(10);
    write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 1, 0x6040, 00, &tmp2, 2);
    HAL_Delay(10);
    write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 1, 0x6040, 00, &tmp3, 2);
    HAL_Delay(10);
    write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 1, 0x6040, 00, &tmp4, 2);
    HAL_Delay(10);
    write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 1, 0x6040, 00, &tmp5, 2);
    HAL_Delay(10);


    write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 1, 0x6060, 00, &interpolation_mode, 1);
    HAL_Delay(10);
    write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 1, 0x60C0, 00, &intrpolation_sub_mode_select, 2);
    HAL_Delay(10);
    write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 1, 0x60C2, 0x01, &interpolation_time_unit, 1);
    HAL_Delay(10);
    write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 1, 0x60C2, 0x02, &interpolation_time_index, 1);

    //control_word for node 2(Gearbox2)...............................................................


    write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 2, 0x6040, 00, &tmp1, 2);
    HAL_Delay(10);
    write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 2, 0x6040, 00, &tmp2, 2);
    HAL_Delay(10);
    write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 2, 0x6040, 00, &tmp3, 2);
    HAL_Delay(10);
    write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 2, 0x6040, 00, &tmp4, 2);
    HAL_Delay(10);
    write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 2, 0x6040, 00, &tmp5, 2);
    HAL_Delay(10);


    write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 2, 0x6060, 00, &interpolation_mode, 1);
    HAL_Delay(10);
    write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 2, 0x60C0, 00, &intrpolation_sub_mode_select, 2);
    HAL_Delay(10);
    write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 2, 0x60C2, 0x01, &interpolation_time_unit, 1);
    HAL_Delay(10);
    write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 2, 0x60C2, 0x02, &interpolation_time_index, 1);


    //control_word for node 3(Gearbox3)...............................................................


    write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 3, 0x6040, 00, &tmp1, 2);
    HAL_Delay(10);
    write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 3, 0x6040, 00, &tmp2, 2);
    HAL_Delay(10);
    write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 3, 0x6040, 00, &tmp3, 2);
    HAL_Delay(10);
    write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 3, 0x6040, 00, &tmp4, 2);
    HAL_Delay(10);
    write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 3, 0x6040, 00, &tmp5, 2);
    HAL_Delay(10);


    write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 3, 0x6060, 00, &interpolation_mode, 1);
    HAL_Delay(10);
    write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 3, 0x60C0, 00, &intrpolation_sub_mode_select, 2);
    HAL_Delay(10);
    write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 3, 0x60C2, 0x01, &interpolation_time_unit, 1);
    HAL_Delay(10);
    write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 3, 0x60C2, 0x02, &interpolation_time_index, 1);


    //control_word for node 4(Gearbox4)...............................................................


    write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 4, 0x6040, 00, &tmp1, 2);
    HAL_Delay(10);
    write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 4, 0x6040, 00, &tmp2, 2);
    HAL_Delay(10);
    write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 4, 0x6040, 00, &tmp3, 2);
    HAL_Delay(10);
    write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 4, 0x6040, 00, &tmp4, 2);
    HAL_Delay(10);
    write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 4, 0x6040, 00, &tmp5, 2);
    HAL_Delay(10);


    write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 4, 0x6060, 00, &interpolation_mode, 1);
    HAL_Delay(10);
    write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 4, 0x60C0, 00, &intrpolation_sub_mode_select, 2);
    HAL_Delay(10);
    write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 4, 0x60C2, 0x01, &interpolation_time_unit, 1);
    HAL_Delay(10);
    write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 4, 0x60C2, 0x02, &interpolation_time_index, 1);


    //control_word for node 5(Gearbox5)...............................................................


    write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 5, 0x6040, 00, &tmp1, 2);
    HAL_Delay(10);
    write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 5, 0x6040, 00, &tmp2, 2);
    HAL_Delay(10);
    write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 5, 0x6040, 00, &tmp3, 2);
    HAL_Delay(10);
    write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 5, 0x6040, 00, &tmp4, 2);
    HAL_Delay(10);
    write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 5, 0x6040, 00, &tmp5, 2);
    HAL_Delay(10);


    write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 5, 0x6060, 00, &interpolation_mode, 1);
    HAL_Delay(10);
    write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 5, 0x60C0, 00, &intrpolation_sub_mode_select, 2);
    HAL_Delay(10);
    write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 5, 0x60C2, 0x01, &interpolation_time_unit, 1);
    HAL_Delay(10);
    write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 5, 0x60C2, 0x02, &interpolation_time_index, 1);


    //control_word for node 6(Gearbox6)...............................................................


    write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 6, 0x6040, 00, &tmp1, 2);
    HAL_Delay(10);
    write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 6, 0x6040, 00, &tmp2, 2);
    HAL_Delay(10);
    write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 6, 0x6040, 00, &tmp3, 2);
    HAL_Delay(10);
    write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 6, 0x6040, 00, &tmp4, 2);
    HAL_Delay(10);
    write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 6, 0x6040, 00, &tmp5, 2);
    HAL_Delay(10);


    write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 6, 0x6060, 00, &interpolation_mode, 1);
    HAL_Delay(10);
    write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 6, 0x60C0, 00, &intrpolation_sub_mode_select, 2);
    HAL_Delay(10);
    write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 6, 0x60C2, 0x01, &interpolation_time_unit, 1);
    HAL_Delay(10);
    write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 6, 0x60C2, 0x02, &interpolation_time_index, 1);

//node1................................................................................
    uint32_t CW1 ;
    uint8_t CW2 ;
    uint8_t CW3 ;
	uint32_t CW4 ;
	uint32_t CW5 ;
	uint8_t CW6 ;
	uint32_t CW7 ;



//interpolation_Tpdo1_Gearbox1........................................................................

			CW1 = 0x80000201;
			write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 1, 0x1400, 0x01, &CW1, 4);
			HAL_Delay(10);

			CW2 = 0xFF;
			write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 1, 0x1400, 0x02, &CW2, 1);
			HAL_Delay(10);

			CW3 = 0x00;
			write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 1, 0x1600, 0x00, &CW3, 1);
			HAL_Delay(10);

			CW4 = 0x60C10120;
			write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 1, 0x1600, 0x01, &CW4, 4);
			HAL_Delay(10);

			CW5 = 0x60C10210;
			write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 1, 0x1600, 0x02, &CW5, 4);
			HAL_Delay(10);

			CW6 = 2;
			write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 1, 0x1600, 0x00, &CW6, 1);
			HAL_Delay(10);

			CW7 = 0x201;
			write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 1, 0x1400, 0x01, &CW7, 4);
			HAL_Delay(10);


//node2...............................................................................

//interpolation_Tpdo2_Gearbox2..........................................................................

			CW1 = 0x80000202;
			write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 2, 0x1400, 0x01, &CW1, 4);
			HAL_Delay(10);

			CW2 = 0xFF;
			write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 2, 0x1400, 0x02, &CW2, 1);
			HAL_Delay(10);

			CW3 = 0x00;
			write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 2, 0x1600, 0x00, &CW3, 1);
			HAL_Delay(10);

			CW4 = 0x60C10120;
			write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 2, 0x1600, 0x01, &CW4, 4);
			HAL_Delay(10);

			CW5 = 0x60C10210;
			write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 2, 0x1600, 0x02, &CW5, 4);
			HAL_Delay(10);

			CW6 = 2;
			write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 2, 0x1600, 0x00, &CW6, 1);
			HAL_Delay(10);

			CW7 = 0x202;
			write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 2, 0x1400, 0x01, &CW7, 4);
			HAL_Delay(10);


//node3.................................................................................

//interpolation_Tpdo3_Gearbox3.............................................................

		   CW1 = 0x80000203;
		   write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 3, 0x1400, 0x01, &CW1, 4);
		   HAL_Delay(10);

		   CW2 = 0xFF;
		   write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 3, 0x1400, 0x02, &CW2, 1);
		   HAL_Delay(10);

		   CW3 = 0x00;
		   write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 3, 0x1600, 0x00, &CW3, 1);
		   HAL_Delay(10);

		   CW4 = 0x60C10120;
		   write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 3, 0x1600, 0x01, &CW4, 4);
		   HAL_Delay(10);

		   CW5 = 0x60C10210;
		   write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 3, 0x1600, 0x02, &CW5, 4);
		   HAL_Delay(10);

		   CW6 = 2;
		   write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 3, 0x1600, 0x00, &CW6, 1);
		   HAL_Delay(10);

		   CW7 = 0x203;
		   write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 3, 0x1400, 0x01, &CW7, 4);
		   HAL_Delay(10);


//node4.................................................................................

//interpolation_Tpdo3_Gearbox4.............................................................

		   CW1 = 0x80000204;
		   write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 4, 0x1400, 0x01, &CW1, 4);
		   HAL_Delay(10);

		   CW2 = 0xFF;
		   write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 4, 0x1400, 0x02, &CW2, 1);
		   HAL_Delay(10);

		   CW3 = 0x00;
		   write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 4, 0x1600, 0x00, &CW3, 1);
		   HAL_Delay(10);

		   CW4 = 0x60C10120;
		   write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 4, 0x1600, 0x01, &CW4, 4);
		   HAL_Delay(10);

		   CW5 = 0x60C10210;
		   write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 4, 0x1600, 0x02, &CW5, 4);
		   HAL_Delay(10);

		   CW6 = 2;
		   write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 4, 0x1600, 0x00, &CW6, 1);
		   HAL_Delay(10);

		   CW7 = 0x204;
		   write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 4, 0x1400, 0x01, &CW7, 4);
		   HAL_Delay(10);

//node5.................................................................................

//interpolation_Tpdo3_Gearbox5.............................................................


		   CW1 = 0x80000205;
		   write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 5, 0x1400, 0x01, &CW1, 4);
		   HAL_Delay(10);

		   CW2 = 0xFF;
		   write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 5, 0x1400, 0x02, &CW2, 1);
		   HAL_Delay(10);

		   CW3 = 0x00;
		   write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 5, 0x1600, 0x00, &CW3, 1);
		   HAL_Delay(10);

		   CW4 = 0x60C10120;
		   write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 5, 0x1600, 0x01, &CW4, 4);
		   HAL_Delay(10);

		   CW5 = 0x60C10210;
		   write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 5, 0x1600, 0x02, &CW5, 4);
		   HAL_Delay(10);

		   CW6 = 2;
		   write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 5, 0x1600, 0x00, &CW6, 1);
		   HAL_Delay(10);

		   CW7 = 0x205;
		   write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 5, 0x1400, 0x01, &CW7, 4);
		   HAL_Delay(10);
//node6.................................................................................

//interpolation_Tpdo3_Gearbox6.............................................................

		   CW1 = 0x80000206;
		   write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 6, 0x1400, 0x01, &CW1, 4);
		   HAL_Delay(10);

		   CW2 = 0xFF;
		   write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 6, 0x1400, 0x02, &CW2, 1);
		   HAL_Delay(10);

		   CW3 = 0x00;
		   write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 6, 0x1600, 0x00, &CW3, 1);
		   HAL_Delay(10);

		   CW4 = 0x60C10120;
		   write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 6, 0x1600, 0x01, &CW4, 4);
		   HAL_Delay(10);

		   CW5 = 0x60C10210;
		   write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 6, 0x1600, 0x02, &CW5, 4);
		   HAL_Delay(10);

		   CW6 = 2;
		   write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 6, 0x1600, 0x00, &CW6, 1);
		   HAL_Delay(10);

		   CW7 = 0x206;
		   write_SDO(canopenNodeSTM32.canOpenStack->SDOclient , 6, 0x1400, 0x01, &CW7, 4);
		   HAL_Delay(10);

//......................................................................................
//    targetVelocity = 950;
    CO_NMT_command_t CO_NMT_OPERATIONAL;
    CO_NMT_sendCommand(canopenNodeSTM32.canOpenStack->NMT, CO_NMT_ENTER_OPERATIONAL, 1);
    HAL_Delay(100);
    CO_NMT_sendCommand(canopenNodeSTM32.canOpenStack->NMT, CO_NMT_ENTER_OPERATIONAL, 2);
    HAL_Delay(100);
    CO_NMT_sendCommand(canopenNodeSTM32.canOpenStack->NMT, CO_NMT_ENTER_OPERATIONAL, 3);
    HAL_Delay(100);
    CO_NMT_sendCommand(canopenNodeSTM32.canOpenStack->NMT, CO_NMT_ENTER_OPERATIONAL, 4);
    HAL_Delay(100);
    CO_NMT_sendCommand(canopenNodeSTM32.canOpenStack->NMT, CO_NMT_ENTER_OPERATIONAL, 5);
    HAL_Delay(100);
    CO_NMT_sendCommand(canopenNodeSTM32.canOpenStack->NMT, CO_NMT_ENTER_OPERATIONAL, 6);
    HAL_Delay(100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {

	  canopen_app_process();
	  HAL_Delay(0.5);
// SIN or COS motion ................................................................................................................
	  next = s * sin((i+1)/1000.0);
	  prev = s * sin((i-1)/1000.0);
	  xi = s * sin(i/1000.0);
	  i = i + 1;

//interpolation for node1(Gearbox1).....................................................................................

	  OD_PERSIST_COMM.x60C1_interpolation_data_record_1.subObject_1 = xi;
	  OD_PERSIST_COMM.x60C1_interpolation_data_record_1.subObject_2 = (next - prev)/2;


//interpolation for node2(Gearbox2).....................................................................................

	  OD_PERSIST_COMM.x70C1_interpolation_data_record_2.subObject_1 = xi;
	  OD_PERSIST_COMM.x70C1_interpolation_data_record_2.subObject_2 = (next - prev)/2;


//interpolation for node3(Gearbox3).....................................................................................

	  OD_PERSIST_COMM.x80C1_interpolation_data_record_3.subObject_1 = xi;
	  OD_PERSIST_COMM.x80C1_interpolation_data_record_3.subObject_2 = (next - prev)/2;


//interpolation for node4(Gearbox4).....................................................................................

	  OD_PERSIST_COMM.x90C1_interpolation_data_record_4.subObject_1 = xi;
	  OD_PERSIST_COMM.x90C1_interpolation_data_record_4.subObject_2 = (next - prev)/2;


//interpolation for node5(Gearbox5).....................................................................................

	  OD_PERSIST_COMM.x91C1_interpolation_data_record_5.subObject_1 = xi;
	  OD_PERSIST_COMM.x91C1_interpolation_data_record_5.subObject_2 = (next - prev)/2;


//interpolation for node6(Gearbox6).....................................................................................

	  OD_PERSIST_COMM.x92C1_interpolation_data_record_6.subObject_1 = xi;
	  OD_PERSIST_COMM.x92C1_interpolation_data_record_6.subObject_2 = (next - prev)/2;







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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV6;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 1;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_14TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 1;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_14TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 15;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 999;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

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
