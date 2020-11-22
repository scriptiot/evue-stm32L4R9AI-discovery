/**
  ******************************************************************************
  * @file    DSI/DSI_CmdMode_SingleBuffer/Inc/main.h
  * @author  MCD Application Team
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_sd.h"

#include "stm32l4r9i_discovery.h"
#include "stm32l4r9i_discovery_io.h"
#include "stm32l4r9i_discovery_lcd.h"
#include "stm32l4r9i_discovery_sd.h"


/* FatFs includes component */
#include "ff_gen_drv.h"
#include "sd_diskio.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#define VSYNC               1
#define VBP                 1
#define VFP                 1
#define VACT                390
#define HSYNC               1
#define HBP                 1
#define HFP                 1
#define HACT                390

#define LAYER_ADDRESS       GFXMMU_VIRTUAL_BUFFER0_BASE

#define BRIGHTNESS_MIN      50
#define BRIGHTNESS_NORMAL   200

void * new_malloc(uint32_t size);

void new_free(void * mem);


#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
