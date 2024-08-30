/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    mw_log_conf.h
  * @author  MCD Application Team
  * @brief   Configure (enable/disable) traces
  *******************************************************************************
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MW_LOG_CONF_H__
#define __MW_LOG_CONF_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
#define MW_LOG_ENABLED

/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Exported macro ------------------------------------------------------------*/
#ifdef MW_LOG_ENABLED
/* USER CODE BEGIN Mw_Logs_En*/
/* Map your own trace mechanism or to map UTIL_ADV_TRACE see examples from CubeFw, i.e.:
                             do{ {UTIL_ADV_TRACE_COND_FSend(VL, T_REG_OFF, TS, __VA_ARGS__);} }while(0) */
#define MW_LOG(TS,VL, ...)
/* USER CODE END Mw_Logs_En */
#else  /* MW_LOG_ENABLED */
/* USER CODE BEGIN Mw_Logs_Dis*/
#define MW_LOG(TS,VL, ...)
/* USER CODE END Mw_Logs_Dis */
#endif /* MW_LOG_ENABLED */
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /*__MW_LOG_CONF_H__ */
