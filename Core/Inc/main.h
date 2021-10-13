/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void setReset(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LWIP_DEBUG 0
#define LWIP_NUM_NETIF_CLIENT_DATA 1
#define LED_YELLOW_Pin GPIO_PIN_2
#define LED_YELLOW_GPIO_Port GPIOE
#define LED_RED_Pin GPIO_PIN_3
#define LED_RED_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */
/* Flash user area definition *************************************************/
/*
   IMPORTANT NOTE:
   ==============
   Be sure that USER_FLASH_FIRST_PAGE_ADDRESS do not overlap with IAP code.
   For example, with all option enabled the total readonly memory size using
   gcc compiler, is 76676 bytes
   Based on this result four sectors of 32 Kbytes will
   be used to store the IAP code, so the user Flash address will start from Sector4.

   In this application the define USER_FLASH_FIRST_PAGE_ADDRESS is set to 128K boundary,
   but it can be changed to any other address outside the 1st 128 Kbytes of the Flash.
   */
#define BOOTLOADER_ADDRESS            0x08000000
#define USER_FLASH_FIRST_PAGE_ADDRESS 0x08040000
#define USER_FLASH_LAST_PAGE_ADDRESS  0x080C0000
#define USER_FLASH_END_ADDRESS        0x080FFFFF
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
