/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "adc.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "math.h"
#include "string.h"
#include "stm32_dsp.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define NPT 64					//FFT采样点数
uint8_t	GREEN_STOP_TIME=10;		//绿色点顶端停顿时间，值越大时间越长
uint8_t	GREEN_SUB_SPEED=40;		//绿色点下移速度，值越大速度越慢
uint8_t	RED_SUB_SPEED=2;		//红色频柱向下缩短速度，值越大速度越慢
uint32_t ADC_DataNum=0;			//ADC采样点数
uint32_t RedTime=0;				//红色点下移时间变量

uint32_t GreenTime=0;			//绿色点下移时间变量
uint32_t GreenStopTime[32]= {0};	//绿色点顶端停顿时间数据

volatile uint8_t ADC_TimeOutFlag = 0;			//ADC定时采样时间到标志

__IO uint16_t ADCConvertedValue;		//ADC采样值

long lBUFMAG[NPT+NPT/2];					//存储求模后的数据
long lBUFOUT[NPT];//FFT输出序列
long lBUFIN[NPT];//FFT输入系列

uint8_t fftHightRedBuf[NPT/2]= {0};			//红色频柱高度数组
uint8_t fftHightGreenBuf[NPT/2]= {0};		//绿色频点高度数组

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM4)
    {
        ADC_TimeOutFlag = 1;
    }
}

void music_fft_main(uint8_t *RedNewHeight,uint8_t *GreenNewHeight)
{
    int i=0;
    static uint8_t RedOldHeight[32] = {0};
    static uint8_t GreenOldHeight[32] = {0};
    for(i=0; i<32; i++)
    {
        if(1)
        {
            uint8_t a,b,c;
            c = RedNewHeight[i]/2;
            if(c>=64)
            {
                c = 64;
            }
            if(c/8)
            {
                uint8_t val=0;
                for(val=0; val<c/8; val++)
                {
                    OLED_GRAM[i * 4 + 0][val] = 0xFF;
                    OLED_GRAM[i * 4 + 1][val] = 0xFF;
                    OLED_GRAM[i * 4 + 2][val] = 0xFF;
                }
            }
            if(c%8)
            {
                for(b=c/8*8; b<c+1; b++)
                {
                    OLED_DrawPoint(i * 4 + 0,64 - b,1);
                    OLED_DrawPoint(i * 4 + 1,64 - b,1);
                    OLED_DrawPoint(i * 4 + 2,64 - b,1);
                }
            }

            a = GreenNewHeight[i]/2 + 2;
            if(a>=64)a=64;
            OLED_DrawPoint(i * 4 + 0,64 - a,1);
            OLED_DrawPoint(i * 4 + 1,64 - a,1);
            OLED_DrawPoint(i * 4 + 2,64 - a,1);
            OLED_DrawPoint(i * 4 + 0,64 - a - 1,1);
            OLED_DrawPoint(i * 4 + 1,64 - a - 1,1);
            OLED_DrawPoint(i * 4 + 2,64 - a - 1,1);

            if(a<64)
            {
                for(b=0; b<64-a; b++)
                {
                    OLED_DrawPoint(i * 4 + 0,b,0);
                    OLED_DrawPoint(i * 4 + 1,b,0);
                    OLED_DrawPoint(i * 4 + 2,b,0);
                }
            }
        }

        //将新数据保存
        RedOldHeight[i] = RedNewHeight[i];
        GreenOldHeight[i] = GreenNewHeight[i];
    }
    OLED_Refresh_Gram();
}

void powerMag(long nfill)
{   int32_t lX,lY;
    uint32_t i;
    for (i=0; i < nfill; i++)
    {
        lX= (lBUFOUT[i]<<16)>>16; /* sine_cosine --> cos */
        lY= (lBUFOUT[i] >> 16);   /* sine_cosine --> sin */
        {
            float X=  64*((float)lX)/32768;
            float Y = 64*((float)lY)/32768;
            float Mag = sqrt(X*X+ Y*Y)/nfill;  // 先平方和,再开方
            lBUFMAG[i] = (long)(Mag*65536);
        }
    }
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* USER CODE BEGIN 1 */
//	uint8_t ble_sta = 0;//蓝牙连接标志
    RTC_DateTypeDef sdatestructure;
    RTC_TimeTypeDef stimestructure;
    char str[20];

    uint8_t i = 0;

    uint32_t ADC_Value[2];
    float battery=0;

    uint16_t temp_val[2];

    uint8_t mode = 0,key_sta = 0;

    uint8_t refresh_data=0;
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
    MX_ADC1_Init();
    MX_USART1_UART_Init();
    MX_RTC_Init();
    MX_TIM4_Init();
    MX_ADC2_Init();
    /* USER CODE BEGIN 2 */
    HAL_GPIO_WritePin(SPE_CTRL_GPIO_Port,SPE_CTRL_Pin,GPIO_PIN_SET);
    __HAL_TIM_ENABLE_IT(&htim4,TIM_IT_UPDATE);

    OLED_Init();
    OLED_Clear();
    show_str(0,0,"蓝牙音箱",16,16);

//	show_str(0,48,"等待蓝牙连接...",16,16);
    OLED_Refresh_Gram();
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    HAL_TIM_Base_Start_IT(&htim4);
    while (1)
    {
        refresh_data ++;
        if(HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin) == GPIO_PIN_SET)
        {
            HAL_Delay(10);
            if(HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin) == GPIO_PIN_SET)
            {
                if(key_sta == 0)
                {
                    key_sta = 1;
                    mode ++;
                    OLED_Clear();
                    if(mode == 5)
                    {
                        mode = 0;
                    }
                    if(mode == 1)
                    {
                        ADC_TimeOutFlag = 0;
                    }
                }
            }
        }
        else if(HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin) == GPIO_PIN_RESET)
        {
            HAL_Delay(10);
            if(HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin) == GPIO_PIN_RESET)
            {
                key_sta = 0;
            }
        }

//	  sprintf(str,"%d",mode);
//	  show_str(0,48,(const uint8_t *)str,16,16);

        if(mode == 0)
        {
            show_str(0,0,"蓝牙音箱",16,16);
            if(HAL_GPIO_ReadPin(BLE_MUTE_GPIO_Port,BLE_MUTE_Pin) == GPIO_PIN_RESET)
            {
                HAL_Delay(10);
                if(HAL_GPIO_ReadPin(BLE_MUTE_GPIO_Port,BLE_MUTE_Pin) == GPIO_PIN_RESET)
                {
                    show_str(2,16,"播放",16,16);
                    HAL_GPIO_WritePin(SPE_CTRL_GPIO_Port,SPE_CTRL_Pin,GPIO_PIN_SET);
                }
            }
            else if(HAL_GPIO_ReadPin(BLE_MUTE_GPIO_Port,BLE_MUTE_Pin) == GPIO_PIN_SET)
            {
                show_str(2,16,"静音",16,16);
                HAL_GPIO_WritePin(SPE_CTRL_GPIO_Port,SPE_CTRL_Pin,GPIO_PIN_RESET);
            }

            if(refresh_data > 50)
            {
                refresh_data = 0;
            }
            if(refresh_data == 2)
            {
                HAL_ADC_Start(&hadc1);
                HAL_ADC_PollForConversion(&hadc1, 50);
                if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC))
                {
                    ADCConvertedValue = HAL_ADC_GetValue(&hadc1);
                    temp_val[0] = (uint16_t)(ADCConvertedValue*6.63/4.096)/1000;//解决分压电阻阻值误差，理论上应该是3.3半电压6.6，实际7.897比较接近真实电压
                    temp_val[1] = (uint16_t)(ADCConvertedValue*6.63/4.096)%1000;

                    battery = ((temp_val[0] + temp_val[1] * 0.001 - 3.25) * 100 / (4.15 - 3.25));
                    if(battery >= 100)
                    {
                        battery = 100;
                    }
                    else if(battery <= 0)
                    {
                        battery = 0;
                    }
                    sprintf(str,"电压:%d.%03dV",temp_val[0],temp_val[1]);
                    show_str(36,16,(const uint8_t *)str,16,16);

                    sprintf(str,"%3d%s",(uint8_t)battery,"%");
                    show_str(78,1,(const uint8_t *)str,16,16);

                    oled_showPicture(110,1,bmp_battery[(uint8_t)battery*12/99],10,16);
                }
            }


            if(1)							//显示日历
            {
                /* Get the RTC current Time ,must get time first*/
                HAL_RTC_GetTime(&hrtc, &stimestructure, RTC_FORMAT_BIN);
                /* Get the RTC current Date */
                HAL_RTC_GetDate(&hrtc, &sdatestructure, RTC_FORMAT_BIN);
                sprintf(str,"%4d年%2d月%2d日",2000 + sdatestructure.Year, sdatestructure.Month, sdatestructure.Date);
                show_str(0,32,(const uint8_t *)str,16,16);
                sprintf(str,"%2d:%02d:%02d",stimestructure.Hours, stimestructure.Minutes, stimestructure.Seconds);
                show_str(0,48,(const uint8_t *)str,16,16);
            }
        }
        else if(mode == 2)
        {
            sprintf(str,"版本号:V1.0");
            show_str(16,16,(const uint8_t *)str,16,16);
            sprintf(str,"Lovelessing...");
            show_str(16,32,(const uint8_t *)str,16,16);
        }
        else if(mode == 3)
        {
            sprintf(str,"频谱显示");
            show_str(16,16,(const uint8_t *)str,16,16);
        }
        else if(mode == 4)
        {
            sprintf(str,"波形显示");
            show_str(16,16,(const uint8_t *)str,16,16);
        }

//	  sprintf(str,"%d",mode);
//	  show_str(0,32,(const uint8_t *)str,16,16);
//	  OLED_Refresh_Gram();





        else if(mode == 1)
        {
            if(ADC_TimeOutFlag)
            {
                GreenTime++;
                RedTime++;
                ADC_TimeOutFlag=0;
                if(ADC_DataNum<NPT)
                {
                    //采样点没有达到所要求的点
                    HAL_ADC_Start(&hadc2);
                    HAL_ADC_PollForConversion(&hadc2, 50);
                    if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc2), HAL_ADC_STATE_REG_EOC))
                    {
                        ADCConvertedValue = HAL_ADC_GetValue(&hadc2);
                    }
                    lBUFIN[ADC_DataNum]=ADCConvertedValue<<16;
                    ADC_DataNum++;
                }
                else
                {
                    HAL_TIM_Base_Stop(&htim4);
                    ADC_DataNum=0;

                    if(0)
                    {
                        //KEY handle
                    }

                    cr4_fft_64_stm32(lBUFOUT,lBUFIN,NPT);//调用STM32的DSP库作FFT变换
                    powerMag(NPT);//计算频点幅值
                    //更新红色点的高度
                    for(i=1; i<NPT/2 + 1; i++)
                    {
                        if((uint8_t)(lBUFMAG[i-1])>fftHightRedBuf[i-1])
                        {
                            fftHightRedBuf[i-1]=(lBUFMAG[i-1])/2;
                        }
                        //刷新绿色点高度
                        if(fftHightRedBuf[i-1]>=fftHightGreenBuf[i-1])
                        {
                            fftHightGreenBuf[i]=fftHightRedBuf[i-1];
                            GreenStopTime[i-1]=GREEN_STOP_TIME;//绿点停顿时间
                            if(fftHightRedBuf[i-1]>=128)
                            {
                                fftHightGreenBuf[i-1]=128;
                                fftHightRedBuf[i-1]=128;
                            }
                        }
                    }
                    //显示红色柱子
                    music_fft_main(fftHightRedBuf,fftHightGreenBuf);
                    //显示绿色点
                    //绿色点下移
                    if((GreenTime>GREEN_SUB_SPEED))
                    {
                        //绿色点下降间隔时间
                        GreenTime=0;
                        for(i=0; i<NPT/2; i++)
                        {
                            if((fftHightGreenBuf[i]!=0)&&(GreenStopTime[i]==0))
                            {
                                fftHightGreenBuf[i]--;
                            }
                        }
                    }
                    //红色下移
                    if(RedTime>RED_SUB_SPEED)
                    {
                        RedTime=0;
                        for(i=0; i<NPT/2; i++)
                        {
                            if(fftHightRedBuf[i]!=0)
                            {
                                fftHightRedBuf[i]--;
                            }
                        }
                    }
                    //绿色点停顿时间减一
                    for(i=0; i<NPT/2; i++)
                    {
                        if(GreenStopTime[i]!=0)
                        {
                            GreenStopTime[i]--;
                        }
                    }
                    HAL_TIM_Base_Start(&htim4);
                    ADC_TimeOutFlag = 0;
                }
            }
        }
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        if(mode != 1)
            OLED_Refresh_Gram();
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

    /** Initializes the CPU, AHB and APB busses clocks
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.LSIState = RCC_LSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB busses clocks
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
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
    PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
