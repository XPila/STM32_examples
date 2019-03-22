/**
  ******************************************************************************
  * @file            : usb_host.c
  * @version         : v1.0_Cube
  * @brief           : This file implements the USB Host
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include "usb_host.h"
#include "usbh_core.h"
#include "usbh_msc.h"
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USB Host core handle declaration */
USBH_HandleTypeDef hUsbHostHS;
ApplicationTypeDef Appli_state = APPLICATION_IDLE;

/*
 * -- Insert your variables declaration here --
 */
/* USER CODE BEGIN 0 */
#include "fatfs.h"
FATFS USBDISKFatFs;           /* File system object for USB disk logical drive */
FIL MyFile;                   /* File object */
char USBDISKPath[4];          /* USB Host logical drive path */
USBH_HandleTypeDef hUSB_Host; /* USB Host handle */
/* USER CODE END 0 */

/*
 * user callback declaration
 */
static void USBH_UserProcess(USBH_HandleTypeDef *phost, uint8_t id);

/*
 * -- Insert your external function declaration here --
 */
/* USER CODE BEGIN 1 */
void USB_Error_Handler(void)
{
  /* USER CODE BEGIN USB_Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
 //HAL_GPIO_WritePin(LD5_GPIO_Port,LD5_Pin,GPIO_PIN_SET);
  while(1)
  {
  }
  /* USER CODE END USB_Error_Handler */
}

static void MSC_Application(void)
{
  FRESULT res;                                          /* FatFs function common result code */
  uint32_t byteswritten, bytesread;                     /* File write/read counts */
  uint8_t wtext[] = "This is STM32 working with FatFs"; /* File write buffer */
  uint8_t rtext[100];                                   /* File read buffer */
  uint8_t txt[4096] = "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";
  uint32_t time = 0;
  uint32_t endtime =0;
  uint32_t delay = 0;
  int i =0;


  f_mount(&USBDISKFatFs, (TCHAR const*)USBDISKPath, 0);
  f_open(&MyFile, "test.TXT", FA_CREATE_ALWAYS | FA_WRITE);
  time = HAL_GetTick();
  for(;i<1000;i++){
	  f_write(&MyFile,txt,sizeof(txt),&byteswritten);
  }

  endtime = HAL_GetTick();

  delay=endtime - time;
  sprintf(txt," %ld",delay);
  f_write(&MyFile,txt,strlen(txt),&byteswritten);
  f_close(&MyFile);
  /* Unlink the USB disk I/O driver */
  FATFS_UnLinkDriver(USBDISKPath);
}
/* USER CODE END 1 */

/**
  * Init USB host library, add supported class and start the library
  * @retval None
  */
void MX_USB_HOST_Init(void)
{
  /* USER CODE BEGIN USB_HOST_Init_PreTreatment */
  
  /* USER CODE END USB_HOST_Init_PreTreatment */
  
  /* Init host Library, add supported class and start the library. */
  USBH_Init(&hUsbHostHS, USBH_UserProcess, HOST_HS);

  USBH_RegisterClass(&hUsbHostHS, USBH_MSC_CLASS);

  USBH_Start(&hUsbHostHS);

  /* USER CODE BEGIN USB_HOST_Init_PostTreatment */
  
  /* USER CODE END USB_HOST_Init_PostTreatment */
}

/*
 * Background task
 */
void MX_USB_HOST_Process(void)
{
  /* USB Host Background task */
  USBH_Process(&hUsbHostHS);
}
/*
 * user callback definition
 */
static void USBH_UserProcess  (USBH_HandleTypeDef *phost, uint8_t id)
{
  /* USER CODE BEGIN CALL_BACK_1 */
  switch(id)
  {
  case HOST_USER_SELECT_CONFIGURATION:
  break;

  case HOST_USER_DISCONNECTION:
  Appli_state = APPLICATION_DISCONNECT;
  break;

  case HOST_USER_CLASS_ACTIVE:
  Appli_state = APPLICATION_READY;
  MSC_Application();
  break;

  case HOST_USER_CONNECTION:
  Appli_state = APPLICATION_START;
  break;

  default:
  break;
  }
  /* USER CODE END CALL_BACK_1 */
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
