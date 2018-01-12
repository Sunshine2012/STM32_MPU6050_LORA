#ifndef __OLED_I2C_H
#define __OLED_I2C_H

#include "stm32f10x.h"

#define OLED_ADDRESS    0x7e // ����Ĭ�ϵ�ַ0x7e




#define             macOLED_RESET_GPIO_APBxClock_FUN              RCC_APB2PeriphClockCmd
#define             macOLED_RESET_GPIO_CLK                        RCC_APB2Periph_GPIOB
#define             macOLED_RESET_GPIO_PORT                       GPIOB
#define             macOLED_RESET_GPIO_PIN                        GPIO_Pin_5



/****************************** LED �����궨��***************************************/
#define             macOLED_RESET_ON()                            GPIO_ResetBits ( macOLED_RESET_GPIO_PORT, macOLED_RESET_GPIO_PIN )
#define             macOLED_RESET_OFF()                           GPIO_SetBits ( macOLED_RESET_GPIO_PORT, macOLED_RESET_GPIO_PIN )
#define             macOLED_RESET_TOGGLE()                        GPIO_ReadOutputDataBit ( macOLED_RESET_GPIO_PORT, macLED1_GPIO_PIN ) ? \
                                                                  GPIO_ResetBits ( macOLED_RESET_GPIO_PORT, macOLED_RESET_GPIO_PIN ) : GPIO_SetBits ( macOLED_RESET_GPIO_PORT, macOLED_RESET_GPIO_PIN )

/*�ȴ���ʱʱ��*/
#define IICT_FLAG_TIMEOUT         ((uint32_t)0x1000)
#define IICT_LONG_TIMEOUT         ((uint32_t)(10 * IICT_FLAG_TIMEOUT))

void I2C_Configuration(void);
void I2C_WriteByte(uint8_t addr,uint8_t data);
void WriteCmd(unsigned char I2C_Command);
void WriteDat(unsigned char I2C_Data);


static uint16_t IIC_TIMEOUT_UserCallback(uint8_t errorCode);

#endif
