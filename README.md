# CH32V003 SSD1306 ULWOS2 TIMERS

## Simple test for CH32V003 Timers in encoder mode and PWM. 
SSD1306 OLED and ULWOS2 scheduler 
TIM2 in encoder mode drived by rotary encoder is used to change PWM frequency and duty cicle on TIM1.
Data displayed on OLED display. 
SSD1306 drivers supports Mikroe GLCD Font Creator.

References
1. https://github.com/fabiopjve/ULWOS2
2. https://www.mikroe.com/glcd-font-creator

Library supports C font array created by GLCD Font Creator by MikroElektronika.<br>
https://www.mikroe.com/glcd-font-creator<br>

Please refer to the following link to get more details about fonts.<br>
https://os.mbed.com/users/dreschpe/code/SPI_TFT_ILI9341/<br>

[ How to add new fonts ]
1. Run GLCD Font Creator
2. Click File-New Font-Import An Existing System Font
3. Select font, style and size from font dialog.
4. GLCD Font Cretor makes Bitmap fonts
5. Click Export for GLCD menu
6. Select mikroC tab.
7. Copy generated code to fonts.c file.
8. Modify data type from unsigned short to uint8_t
9. Add optional bytes (offset, width, height, bpl, staring char) to the array. open fonts.c for details.
10. Add extern declaration to fonts.h file.
