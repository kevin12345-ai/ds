#ifndef __OLED_H
#define __OLED_H

#include "ti_msp_dl_config.h"

/* OLED GPIO Pins (PORT defined in ti_msp_dl_config.h) */
#define OLED_SCL            DL_GPIO_PIN_31   /* PA31 = SCL */
#define OLED_SDA            DL_GPIO_PIN_28   /* PA28 = SDA */

/* OLED IOMUX */
#define OLED_SCL_IOMUX      IOMUX_PINCM6     /* PA31 */
#define OLED_SDA_IOMUX_OUT  IOMUX_PINCM3     /* PA28 */

void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char);
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String);
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);

#endif
