#ifndef __OLED_H
#define __OLED_H			  	 
#include "main.h"
#include "stdlib.h"	    
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK战舰STM32开发板
//ALIENTEK OLED模块驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/9/5
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////	

//OLED模式设置
//0:4线串行模式
//1:并行8080模式
#define OLED_MODE 1

#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据

extern uint8_t OLED_GRAM[128][8];
extern const unsigned char bmp_battery[13][20];

//OLED控制用函数
void OLED_WR_Byte(uint8_t dat,uint8_t cmd);	    
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Refresh_Gram(void);		   
							   		    
void OLED_Init(void);
void OLED_Clear(void);
void OLED_DrawPoint(uint8_t x,uint8_t y,uint8_t t);
uint8_t OLED_ReadPoint(uint8_t x,uint8_t y);
void OLED_Fill(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2,uint8_t dot);
void oled_showChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t f_w,uint8_t f_h,uint8_t mode);
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t size,uint8_t mode);
void OLED_ShowNum(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size);
void OLED_ShowString(uint8_t x,uint8_t y,const uint8_t *p);	 

void show_font(uint16_t x,uint16_t y,const uint8_t *font,uint8_t f_w,uint8_t f_h,uint8_t mode);
void show_str(uint16_t x,uint16_t y,const uint8_t*str,uint8_t f_h,uint8_t mode);
void show_str_mid(uint16_t x,uint16_t y,const uint8_t*str,uint8_t f_h,uint8_t mode,uint16_t len);
void oled_showPicture(uint8_t x,uint8_t y,const uint8_t *p,uint8_t p_w,uint8_t p_h);

#endif  
