#include "OLED.h"
#include "OLEDfont.h"

/*==========================================================================
 * Low-Level GPIO Functions (with I2C timing delays for MSPM0)
 *==========================================================================*/

#define I2C_DELAY   delay_cycles(160)   /* ~5us @ 32MHz, I2C standard mode */

static void OLED_W_SCL(uint8_t x)
{
    if (x) DL_GPIO_setPins(OLED_PORT, OLED_SCL);
    else   DL_GPIO_clearPins(OLED_PORT, OLED_SCL);
    I2C_DELAY;
}

static void OLED_W_SDA(uint8_t x)
{
    if (x) DL_GPIO_setPins(OLED_PORT, OLED_SDA);
    else   DL_GPIO_clearPins(OLED_PORT, OLED_SDA);
    I2C_DELAY;
}

/*==========================================================================
 * I2C Functions
 *==========================================================================*/

void OLED_I2C_Init(void)
{
    DL_GPIO_initDigitalOutput(OLED_SCL_IOMUX);
    DL_GPIO_initDigitalOutput(OLED_SDA_IOMUX_OUT);
    DL_GPIO_setPins(OLED_PORT, OLED_SCL | OLED_SDA);
    DL_GPIO_enableOutput(OLED_PORT, OLED_SCL | OLED_SDA);
}

void OLED_I2C_Start(void)
{
    OLED_W_SDA(1);
    OLED_W_SCL(1);
    OLED_W_SDA(0);
    OLED_W_SCL(0);
}

void OLED_I2C_Stop(void)
{
    OLED_W_SDA(0);
    OLED_W_SCL(1);
    OLED_W_SDA(1);
}

void OLED_I2C_SendByte(uint8_t Byte)
{
    uint8_t i;
    for (i = 0; i < 8; i++)
    {
        OLED_W_SDA(!!(Byte & (0x80 >> i)));
        OLED_W_SCL(1);
        OLED_W_SCL(0);
    }
    OLED_W_SCL(1);  /* Extra clock for ACK (not checked) */
    OLED_W_SCL(0);
}

/*==========================================================================
 * OLED Command / Data Transfer
 *==========================================================================*/

void OLED_WriteCommand(uint8_t Command)
{
    OLED_I2C_Start();
    OLED_I2C_SendByte(0x78);    /* Slave addr: 0x3C << 1 */
    OLED_I2C_SendByte(0x00);    /* Control: COMMAND */
    OLED_I2C_SendByte(Command);
    OLED_I2C_Stop();
}

void OLED_WriteData(uint8_t Data)
{
    OLED_I2C_Start();
    OLED_I2C_SendByte(0x78);    /* Slave addr: 0x3C << 1 */
    OLED_I2C_SendByte(0x40);    /* Control: DATA */
    OLED_I2C_SendByte(Data);
    OLED_I2C_Stop();
}

void OLED_SetCursor(uint8_t Y, uint8_t X)
{
    OLED_WriteCommand(0xB0 | Y);                    /* Page address */
    OLED_WriteCommand(0x10 | ((X & 0xF0) >> 4));    /* Column high nibble */
    OLED_WriteCommand(0x00 | (X & 0x0F));            /* Column low nibble */
}

/*==========================================================================
 * OLED Init & Clear
 *==========================================================================*/

void OLED_Init(void)
{
    uint32_t i, j;

    for (i = 0; i < 1000; i++)          /* Power-up delay */
        for (j = 0; j < 1000; j++);

    OLED_I2C_Init();

    OLED_WriteCommand(0xAE);    /* Display OFF */

    OLED_WriteCommand(0xD5);    /* Clock divide / oscillator freq */
    OLED_WriteCommand(0x80);

    OLED_WriteCommand(0xA8);    /* Multiplex ratio */
    OLED_WriteCommand(0x3F);

    OLED_WriteCommand(0xD3);    /* Display offset */
    OLED_WriteCommand(0x00);

    OLED_WriteCommand(0x40);    /* Start line */

    OLED_WriteCommand(0xA1);    /* Segment re-map: 0xA1 normal, 0xA0 mirrored */

    OLED_WriteCommand(0xC8);    /* COM scan: 0xC8 normal, 0xC0 flipped */

    OLED_WriteCommand(0xDA);    /* COM pins hardware config */
    OLED_WriteCommand(0x12);

    OLED_WriteCommand(0x81);    /* Contrast */
    OLED_WriteCommand(0xCF);

    OLED_WriteCommand(0xD9);    /* Pre-charge period */
    OLED_WriteCommand(0xF1);

    OLED_WriteCommand(0xDB);    /* VCOMH deselect level */
    OLED_WriteCommand(0x30);

    OLED_WriteCommand(0xA4);    /* Display follows RAM */

    OLED_WriteCommand(0xA6);    /* Normal (not inverted) */

    OLED_WriteCommand(0x8D);    /* Charge pump */
    OLED_WriteCommand(0x14);

    OLED_WriteCommand(0xAF);    /* Display ON */

    OLED_Clear();
}

void OLED_Clear(void)
{
    uint8_t i, j;
    for (j = 0; j < 8; j++)
    {
        OLED_SetCursor(j, 0);
        for (i = 0; i < 128; i++)
            OLED_WriteData(0x00);
    }
}

/*==========================================================================
 * Character & String Display
 *==========================================================================*/

/**
 * @brief  Display one ASCII character (OLED_F8x16 font: 8x16 pixels)
 * @param  Line:   line 0-3 (STM32 ref: 1-4, adapted to 0-based)
 * @param  Column: column 0-15 (STM32 ref: 1-16, adapted to 0-based)
 */
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char)
{
    uint8_t i;
    if (Char < ' ' || Char > '~') Char = ' ';

    /* Upper half: page = Line * 2 */
    OLED_SetCursor(Line * 2, Column * 8);
    for (i = 0; i < 8; i++)
        OLED_WriteData(OLED_F8x16[Char - ' '][i]);

    /* Lower half: page = Line * 2 + 1 */
    OLED_SetCursor(Line * 2 + 1, Column * 8);
    for (i = 0; i < 8; i++)
        OLED_WriteData(OLED_F8x16[Char - ' '][i + 8]);
}

void OLED_ShowString(uint8_t Line, uint8_t Column, char *String)
{
    uint8_t i;
    for (i = 0; String[i] != '\0'; i++)
    {
        OLED_ShowChar(Line, Column + i, String[i]);
    }
}

/*==========================================================================
 * Number Display
 *==========================================================================*/

uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while (Y--) Result *= X;
    return Result;
}

void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++)
        OLED_ShowChar(Line, Column + i,
                      Number / OLED_Pow(10, Length - i - 1) % 10 + '0');
}

void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length)
{
    uint8_t i;
    uint32_t Number1;
    if (Number >= 0)
    {
        OLED_ShowChar(Line, Column, '+');
        Number1 = Number;
    }
    else
    {
        OLED_ShowChar(Line, Column, '-');
        Number1 = -Number;
    }
    for (i = 0; i < Length; i++)
        OLED_ShowChar(Line, Column + i + 1,
                      Number1 / OLED_Pow(10, Length - i - 1) % 10 + '0');
}

void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i, SingleNumber;
    for (i = 0; i < Length; i++)
    {
        SingleNumber = Number / OLED_Pow(16, Length - i - 1) % 16;
        if (SingleNumber < 10)
            OLED_ShowChar(Line, Column + i, SingleNumber + '0');
        else
            OLED_ShowChar(Line, Column + i, SingleNumber - 10 + 'A');
    }
}

void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++)
        OLED_ShowChar(Line, Column + i,
                      Number / OLED_Pow(2, Length - i - 1) % 2 + '0');
}
