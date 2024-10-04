/*
To use this Font with lib you have to add 5 parameter at the beginning of the font array.
- the number of byte per char 
- the vertical size in pixel
- the horizontal size in pixel 
- the number of byte per vertical line (it is vertical size / 8 ) 
- starting ASCII char
 You also have to change the array to const uint8_t[].
 The horizontal size of each character is also stored in the font. 
 It look better if you use bigger fonts or italic. The letter M is wider than a l.
*/

#include "fonts.h"





//WARNING: This Font Require X-GLCD Lib.
//         You can not use it with MikroE GLCD Lib.

//Font Generated by MikroElektronika GLCD Font Creator 1.2.0.0
//MikroElektronika 2011 
//http://www.mikroe.com 

//GLCD FontName : Envy_Code_R11x21
//GLCD FontSize : 11 x 21

const uint8_t Envy_Code_R11x21[] = {
        // Offset, Width, Height, BPL, start char
                 34,11,21,3,65,
        0x0A, 0xF8, 0xFF, 0x07, 0xFC, 0xFF, 0x07, 0x0E, 0x06, 0x00, 0x03, 0x06, 0x00, 0x03, 0x06, 0x00, 0x03, 0x06, 0x00, 0x03, 0x06, 0x00, 0x0E, 0x06, 0x00, 0xFE, 0xFF, 0x07, 0xF8, 0xFF, 0x07, 0x00, 0x00, 0x00,  // Code for char A
        0x0A, 0xFF, 0xFF, 0x07, 0xFF, 0xFF, 0x07, 0x03, 0x06, 0x06, 0x03, 0x06, 0x06, 0x03, 0x06, 0x06, 0x03, 0x06, 0x06, 0x03, 0x06, 0x06, 0x8E, 0x0F, 0x03, 0xFE, 0xFD, 0x03, 0xF8, 0xF8, 0x00, 0x00, 0x00, 0x00,  // Code for char B
        0x0A, 0xF8, 0xFF, 0x00, 0xFE, 0xFF, 0x03, 0x06, 0x00, 0x03, 0x03, 0x00, 0x06, 0x03, 0x00, 0x06, 0x03, 0x00, 0x06, 0x03, 0x00, 0x06, 0x06, 0x00, 0x03, 0x0E, 0x80, 0x03, 0x08, 0x80, 0x00, 0x00, 0x00, 0x00,  // Code for char C
        0x0B, 0xFF, 0xFF, 0x07, 0xFF, 0xFF, 0x07, 0x03, 0x00, 0x06, 0x03, 0x00, 0x06, 0x03, 0x00, 0x06, 0x03, 0x00, 0x06, 0x03, 0x00, 0x06, 0x07, 0x00, 0x07, 0x0E, 0x80, 0x03, 0xFC, 0xFF, 0x01, 0xF8, 0xFF, 0x00,  // Code for char D
        0x0B, 0xFF, 0xFF, 0x07, 0xFF, 0xFF, 0x07, 0x03, 0x06, 0x06, 0x03, 0x06, 0x06, 0x03, 0x06, 0x06, 0x03, 0x06, 0x06, 0x03, 0x06, 0x06, 0x03, 0x06, 0x06, 0x03, 0x00, 0x06, 0x03, 0x00, 0x06, 0x03, 0x00, 0x06,  // Code for char E
        0x0B, 0xFF, 0xFF, 0x07, 0xFF, 0xFF, 0x07, 0x03, 0x06, 0x00, 0x03, 0x06, 0x00, 0x03, 0x06, 0x00, 0x03, 0x06, 0x00, 0x03, 0x06, 0x00, 0x03, 0x06, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00,  // Code for char F
        0x0A, 0xF8, 0xFF, 0x00, 0xFC, 0xFF, 0x01, 0x06, 0x00, 0x03, 0x03, 0x00, 0x06, 0x03, 0x00, 0x06, 0x03, 0x00, 0x06, 0x03, 0x06, 0x07, 0x06, 0x86, 0x03, 0x0E, 0xFE, 0x07, 0x08, 0xFE, 0x07, 0x00, 0x00, 0x00,  // Code for char G
        0x0A, 0xFF, 0xFF, 0x07, 0xFF, 0xFF, 0x07, 0x00, 0x06, 0x00, 0x00, 0x06, 0x00, 0x00, 0x06, 0x00, 0x00, 0x06, 0x00, 0x00, 0x06, 0x00, 0x00, 0x06, 0x00, 0xFF, 0xFF, 0x07, 0xFF, 0xFF, 0x07, 0x00, 0x00, 0x00,  // Code for char H
        0x0A, 0x03, 0x00, 0x06, 0x03, 0x00, 0x06, 0x03, 0x00, 0x06, 0x03, 0x00, 0x06, 0xFF, 0xFF, 0x07, 0xFF, 0xFF, 0x07, 0x03, 0x00, 0x06, 0x03, 0x00, 0x06, 0x03, 0x00, 0x06, 0x03, 0x00, 0x06, 0x00, 0x00, 0x00,  // Code for char I
        0x0B, 0x00, 0xE0, 0x00, 0x00, 0xE0, 0x01, 0x00, 0x80, 0x03, 0x00, 0x00, 0x07, 0x00, 0x00, 0x06, 0x00, 0x00, 0x06, 0x00, 0x00, 0x06, 0x00, 0x00, 0x07, 0x00, 0x80, 0x03, 0xFF, 0xFF, 0x01, 0xFF, 0xFF, 0x00,  // Code for char J
        0x0A, 0xFF, 0xFF, 0x07, 0xFF, 0xFF, 0x07, 0x00, 0x07, 0x00, 0x80, 0x0F, 0x00, 0xE0, 0x38, 0x00, 0x70, 0x70, 0x00, 0x1C, 0xC0, 0x01, 0x0E, 0x80, 0x03, 0x03, 0x00, 0x06, 0x01, 0x00, 0x04, 0x00, 0x00, 0x00,  // Code for char K
        0x0B, 0xFF, 0xFF, 0x07, 0xFF, 0xFF, 0x07, 0x00, 0x00, 0x06, 0x00, 0x00, 0x06, 0x00, 0x00, 0x06, 0x00, 0x00, 0x06, 0x00, 0x00, 0x06, 0x00, 0x00, 0x06, 0x00, 0x00, 0x06, 0x00, 0x00, 0x06, 0x00, 0x00, 0x06,  // Code for char L
        0x0A, 0xFF, 0xFF, 0x07, 0xFF, 0xFF, 0x07, 0x1E, 0x00, 0x00, 0x78, 0x00, 0x00, 0xE0, 0x03, 0x00, 0xE0, 0x01, 0x00, 0x78, 0x00, 0x00, 0x1E, 0x00, 0x00, 0xFF, 0xFF, 0x07, 0xFF, 0xFF, 0x07, 0x00, 0x00, 0x00,  // Code for char M
        0x0A, 0xFF, 0xFF, 0x07, 0xFF, 0xFF, 0x07, 0x3E, 0x00, 0x00, 0xF8, 0x01, 0x00, 0xC0, 0x07, 0x00, 0x00, 0x1F, 0x00, 0x00, 0xFC, 0x00, 0x00, 0xE0, 0x03, 0xFF, 0xFF, 0x07, 0xFF, 0xFF, 0x07, 0x00, 0x00, 0x00,  // Code for char N
        0x0A, 0xF8, 0xFF, 0x00, 0xFC, 0xFF, 0x01, 0x06, 0x00, 0x03, 0x03, 0x00, 0x06, 0x03, 0x00, 0x06, 0x03, 0x00, 0x06, 0x03, 0x00, 0x06, 0x06, 0x00, 0x03, 0xFE, 0xFF, 0x03, 0xF8, 0xFF, 0x00, 0x00, 0x00, 0x00,  // Code for char O
        0x0A, 0xFF, 0xFF, 0x07, 0xFF, 0xFF, 0x07, 0x03, 0x06, 0x00, 0x03, 0x06, 0x00, 0x03, 0x06, 0x00, 0x03, 0x06, 0x00, 0x03, 0x06, 0x00, 0x8E, 0x03, 0x00, 0xFC, 0x01, 0x00, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00,  // Code for char P
        0x0A, 0xF8, 0xFF, 0x00, 0xFC, 0xFF, 0x01, 0x06, 0x00, 0x03, 0x03, 0x00, 0x06, 0x03, 0x20, 0x06, 0x03, 0xE0, 0x06, 0x03, 0xC0, 0x07, 0x06, 0x00, 0x0F, 0xFE, 0xFF, 0x1D, 0xF8, 0xFF, 0x10, 0x00, 0x00, 0x00,  // Code for char Q
        0x0A, 0xFF, 0xFF, 0x07, 0xFF, 0xFF, 0x07, 0x03, 0x06, 0x00, 0x03, 0x06, 0x00, 0x03, 0x0E, 0x00, 0x03, 0x3E, 0x00, 0x03, 0xFA, 0x00, 0x8E, 0xC3, 0x03, 0xFE, 0x01, 0x07, 0xF8, 0x00, 0x04, 0x00, 0x00, 0x00,  // Code for char R
        0x0A, 0xF8, 0x00, 0x06, 0xFC, 0x01, 0x06, 0x8E, 0x03, 0x06, 0x03, 0x06, 0x06, 0x03, 0x06, 0x06, 0x03, 0x06, 0x06, 0x03, 0x06, 0x06, 0x03, 0x0C, 0x03, 0x03, 0xFC, 0x03, 0x03, 0xF0, 0x00, 0x00, 0x00, 0x00,  // Code for char S
        0x0A, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0xFF, 0xFF, 0x07, 0xFF, 0xFF, 0x07, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,  // Code for char T
        0x0A, 0xFF, 0xFF, 0x00, 0xFF, 0xFF, 0x01, 0x00, 0x00, 0x03, 0x00, 0x00, 0x06, 0x00, 0x00, 0x06, 0x00, 0x00, 0x06, 0x00, 0x00, 0x06, 0x00, 0x00, 0x03, 0xFF, 0xFF, 0x03, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00,  // Code for char U
        0x0A, 0x03, 0x00, 0x00, 0x7F, 0x00, 0x00, 0xFC, 0x0F, 0x00, 0x80, 0xFF, 0x01, 0x00, 0xF8, 0x07, 0x00, 0xF8, 0x07, 0x80, 0xFF, 0x01, 0xFC, 0x0F, 0x00, 0x7F, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,  // Code for char V
        0x0A, 0xFF, 0xFF, 0x07, 0xFF, 0xFF, 0x07, 0x00, 0xC0, 0x03, 0x00, 0xF0, 0x00, 0x00, 0x3C, 0x00, 0x00, 0x3E, 0x00, 0x00, 0xF8, 0x00, 0x00, 0xE0, 0x03, 0xFF, 0xFF, 0x07, 0xFF, 0xFF, 0x07, 0x00, 0x00, 0x00,  // Code for char W
        0x0A, 0x01, 0x00, 0x04, 0x07, 0x00, 0x07, 0x3E, 0xE0, 0x03, 0xF8, 0xF8, 0x00, 0xC0, 0x1F, 0x00, 0xC0, 0x1F, 0x00, 0xF8, 0xF8, 0x00, 0x3E, 0xE0, 0x03, 0x07, 0x00, 0x07, 0x01, 0x00, 0x04, 0x00, 0x00, 0x00,  // Code for char X
        0x0A, 0x03, 0x00, 0x00, 0x1F, 0x00, 0x00, 0xFC, 0x00, 0x00, 0xE0, 0x07, 0x00, 0x00, 0xFF, 0x07, 0x00, 0xFF, 0x07, 0xE0, 0x07, 0x00, 0xFC, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,  // Code for char Y
        0x0A, 0x03, 0x00, 0x07, 0x03, 0xC0, 0x07, 0x03, 0xF0, 0x06, 0x03, 0x3C, 0x06, 0x03, 0x0E, 0x06, 0x83, 0x03, 0x06, 0xE3, 0x01, 0x06, 0x7B, 0x00, 0x06, 0x1F, 0x00, 0x06, 0x07, 0x00, 0x06, 0x00, 0x00, 0x00   // Code for char Z
        };



//GLCD FontName : MicroSquare27x32
//GLCD FontSize : 27 x 32

const uint8_t MicroSquare27x32[] = {
		/* Offset, Width, Height, BPL, start char*/
		                 109,27,32,4,48,
        0x1A, 0xC0, 0xFF, 0xFF, 0x07, 0xF0, 0xFF, 0xFF, 0x1F, 0xF8, 0xFF, 0xFF, 0x3F, 0xFC, 0xFF, 0xFF, 0x3F, 0xFE, 0xFF, 0xFF, 0x7F, 0xFE, 0xFF, 0xFF, 0x7F, 0x7F, 0x00, 0x00, 0xFE, 0x3F, 0x00, 0x00, 0xFC, 0x1F, 0x00, 0x00, 0xF8, 0x1F, 0x00, 0x00, 0xF8, 0x1F, 0x00, 0x00, 0xF8, 0x1F, 0xC0, 0x03, 0xF8, 0x1F, 0xE0, 0x07, 0xF8, 0x1F, 0xE0, 0x07, 0xF8, 0x1F, 0xC0, 0x03, 0xF8, 0x1F, 0x00, 0x00, 0xF8, 0x1F, 0x00, 0x00, 0xF8, 0x1F, 0x00, 0x00, 0xF8, 0x3F, 0x00, 0x00, 0xFC, 0x7F, 0x00, 0x00, 0xFF, 0xFE, 0xFF, 0xFF, 0x7F, 0xFE, 0xFF, 0xFF, 0x7F, 0xFC, 0xFF, 0xFF, 0x3F, 0xFC, 0xFF, 0xFF, 0x1F, 0xF8, 0xFF, 0xFF, 0x0F, 0xE0, 0xFF, 0xFF, 0x03, 0x00, 0x00, 0x00, 0x00,  // Code for char 0
        0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x80, 0x0F, 0x00, 0x00, 0xC0, 0x3F, 0x00, 0x00, 0xE0, 0x1F, 0x00, 0x00, 0xE0, 0x0F, 0x00, 0x00, 0xF0, 0x07, 0x00, 0x00, 0xF8, 0x03, 0x00, 0x00, 0xFC, 0x01, 0x00, 0x00, 0xFE, 0x00, 0x00, 0x00, 0x7F, 0x00, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // Code for char 1
        0x19, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x0F, 0xE0, 0xFF, 0xF8, 0x0F, 0xF8, 0xFF, 0xFC, 0x0F, 0xFC, 0xFF, 0xFE, 0x0F, 0xFC, 0xFF, 0xFE, 0x0F, 0xFE, 0xFF, 0xFE, 0x0F, 0xFE, 0xFF, 0x3F, 0x00, 0x3E, 0xF8, 0x1F, 0x00, 0x3E, 0xF8, 0x1F, 0x00, 0x1F, 0xF8, 0x1F, 0x00, 0x1F, 0xF8, 0x1F, 0x00, 0x1F, 0xF8, 0x1F, 0x00, 0x1F, 0xF8, 0x1F, 0x00, 0x1F, 0xF8, 0x1F, 0x80, 0x0F, 0xF8, 0x1F, 0x80, 0x0F, 0xF8, 0x1F, 0x80, 0x0F, 0xF8, 0x3F, 0x80, 0x0F, 0xF8, 0x7F, 0xC0, 0x0F, 0xF8, 0xFE, 0xFF, 0x07, 0xF8, 0xFE, 0xFF, 0x07, 0xF8, 0xFE, 0xFF, 0x07, 0xF8, 0xFC, 0xFF, 0x03, 0xF8, 0xF8, 0xFF, 0x01, 0xF8, 0xE0, 0x7F, 0x00, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // Code for char 2
        0x19, 0x00, 0x00, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x1F, 0xF8, 0x07, 0xE0, 0x3F, 0xFC, 0x07, 0xE0, 0x7F, 0xFE, 0x07, 0xE0, 0x7F, 0xFE, 0x07, 0xE0, 0x7F, 0xFE, 0x07, 0x00, 0xFC, 0x3F, 0x00, 0x00, 0xFC, 0x1F, 0x00, 0x00, 0xF8, 0x1F, 0x00, 0x00, 0xF8, 0x1F, 0x00, 0x00, 0xF8, 0x1F, 0xE0, 0x03, 0xF8, 0x1F, 0xE0, 0x03, 0xF8, 0x1F, 0xE0, 0x03, 0xF8, 0x1F, 0xE0, 0x03, 0xF8, 0x1F, 0xE0, 0x03, 0xF8, 0x1F, 0xE0, 0x03, 0xF8, 0x1F, 0xE0, 0x03, 0xFC, 0x3F, 0xF0, 0x07, 0xFE, 0xFE, 0x7F, 0xFF, 0x7F, 0xFE, 0x7F, 0xFF, 0x7F, 0xFE, 0x7F, 0xFF, 0x7F, 0xFC, 0x7F, 0xFE, 0x3F, 0xF8, 0x3F, 0xFE, 0x1F, 0xE0, 0x0F, 0xF8, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // Code for char 3
        0x1B, 0x00, 0x00, 0xFE, 0x01, 0x00, 0x80, 0xFF, 0x01, 0x00, 0xC0, 0xFF, 0x01, 0x00, 0xE0, 0xFF, 0x01, 0x00, 0xF0, 0xFF, 0x01, 0x00, 0xFC, 0xF7, 0x01, 0x00, 0xFE, 0xF3, 0x01, 0x00, 0xFF, 0xF0, 0x01, 0xC0, 0x7F, 0xF0, 0x01, 0xE0, 0x3F, 0xF0, 0x01, 0xF0, 0x0F, 0xF0, 0x01, 0xFC, 0x07, 0xF0, 0x01, 0xFE, 0x03, 0xF0, 0x01, 0xFF, 0x00, 0xF0, 0x01, 0x7F, 0x00, 0xF0, 0x01, 0x3F, 0x00, 0xF0, 0x01, 0x1F, 0x00, 0xF0, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0xF0, 0x01, 0x00, 0x00, 0xF0, 0x01, 0x00, 0x00, 0xF0, 0x01, 0x00, 0x00, 0xF0, 0x01,  // Code for char 4
        0x19, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x81, 0x0F, 0xFF, 0xFF, 0x81, 0x1F, 0xFF, 0xFF, 0x81, 0x3F, 0xFF, 0xFF, 0x81, 0x7F, 0xFF, 0xFF, 0x81, 0x7F, 0xFF, 0xFF, 0x81, 0xFF, 0x1F, 0xF0, 0x00, 0xFC, 0x1F, 0x78, 0x00, 0xF8, 0x1F, 0x78, 0x00, 0xF8, 0x1F, 0x7C, 0x00, 0xF8, 0x1F, 0x7C, 0x00, 0xF8, 0x1F, 0x7C, 0x00, 0xF8, 0x1F, 0x7C, 0x00, 0xF8, 0x1F, 0x7C, 0x00, 0xF8, 0x1F, 0x7C, 0x00, 0xF8, 0x1F, 0x7C, 0x00, 0xF8, 0x1F, 0xFC, 0x00, 0xFC, 0x1F, 0xFC, 0x01, 0xFE, 0x1F, 0xF8, 0xFF, 0x7F, 0x1F, 0xF8, 0xFF, 0x7F, 0x1F, 0xF8, 0xFF, 0x3F, 0x1F, 0xF0, 0xFF, 0x3F, 0x1F, 0xE0, 0xFF, 0x1F, 0x00, 0x00, 0xFF, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // Code for char 5
        0x19, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xFF, 0xFF, 0x03, 0xF0, 0xFF, 0xFF, 0x0F, 0xFC, 0xFF, 0xFF, 0x3F, 0xFC, 0xFF, 0xFF, 0x3F, 0xFE, 0xFF, 0xFF, 0x7F, 0xFE, 0xFF, 0xFF, 0x7F, 0x7F, 0xC0, 0x03, 0xFE, 0x3F, 0xE0, 0x03, 0xFC, 0x1F, 0xE0, 0x01, 0xF8, 0x1F, 0xF0, 0x01, 0xF8, 0x1F, 0xF0, 0x01, 0xF8, 0x1F, 0xF0, 0x01, 0xF8, 0x1F, 0xF0, 0x01, 0xF8, 0x1F, 0xF0, 0x01, 0xF8, 0x1F, 0xF0, 0x01, 0xF8, 0x1F, 0xF0, 0x01, 0xF8, 0x1F, 0xF0, 0x01, 0xFC, 0x3F, 0xF0, 0x03, 0xFE, 0xFF, 0xF1, 0xFF, 0x7F, 0xFE, 0xE1, 0xFF, 0x7F, 0xFE, 0xE1, 0xFF, 0x7F, 0xFC, 0xC1, 0xFF, 0x3F, 0xF8, 0x81, 0xFF, 0x1F, 0xE0, 0x01, 0xFE, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // Code for char 6
        0x18, 0x1F, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x80, 0x1F, 0x00, 0x00, 0xC0, 0x1F, 0x00, 0x00, 0xF0, 0x1F, 0x00, 0x00, 0xFC, 0x1F, 0x00, 0x00, 0xFF, 0x1F, 0x00, 0xC0, 0xFF, 0x1F, 0x00, 0xF0, 0xFF, 0x1F, 0x00, 0xFC, 0x7F, 0x1F, 0x00, 0xFF, 0x1F, 0x1F, 0xC0, 0xFF, 0x07, 0x1F, 0xF0, 0xFF, 0x01, 0x1F, 0xFC, 0x7F, 0x00, 0x1F, 0xFF, 0x1F, 0x00, 0xDF, 0xFF, 0x07, 0x00, 0xFF, 0xFF, 0x01, 0x00, 0xFF, 0x7F, 0x00, 0x00, 0xFF, 0x1F, 0x00, 0x00, 0xFF, 0x07, 0x00, 0x00, 0xFF, 0x01, 0x00, 0x00, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // Code for char 7
        0x19, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x0F, 0xF8, 0x07, 0xF8, 0x3F, 0xFE, 0x1F, 0xFC, 0x3F, 0xFF, 0x3F, 0xFE, 0x7F, 0xFF, 0x7F, 0xFE, 0x7F, 0xFF, 0x7F, 0xFF, 0xFF, 0xFF, 0x7F, 0x3F, 0xF0, 0x07, 0xFC, 0x1F, 0xE0, 0x03, 0xF8, 0x1F, 0xE0, 0x03, 0xF8, 0x1F, 0xE0, 0x03, 0xF8, 0x1F, 0xE0, 0x03, 0xF8, 0x1F, 0xE0, 0x03, 0xF8, 0x1F, 0xE0, 0x03, 0xF8, 0x1F, 0xE0, 0x03, 0xF8, 0x1F, 0xE0, 0x03, 0xF8, 0x1F, 0xE0, 0x03, 0xF8, 0x1F, 0xE0, 0x03, 0xFC, 0x3F, 0xF0, 0x07, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x7F, 0xFF, 0x7F, 0xFE, 0x7F, 0xFF, 0x7F, 0xFC, 0x7F, 0xFF, 0x3F, 0xF8, 0x3F, 0xFE, 0x1F, 0xE0, 0x0F, 0xFC, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // Code for char 8
        0x19, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x7F, 0x80, 0x07, 0xF8, 0xFF, 0x81, 0x1F, 0xFC, 0xFF, 0x83, 0x3F, 0xFE, 0xFF, 0x87, 0x7F, 0xFE, 0xFF, 0x87, 0x7F, 0xFE, 0xFF, 0x8F, 0xFF, 0x3F, 0xC0, 0x0F, 0xFC, 0x3F, 0x80, 0x0F, 0xF8, 0x1F, 0x80, 0x0F, 0xF8, 0x1F, 0x80, 0x0F, 0xF8, 0x1F, 0x80, 0x0F, 0xF8, 0x1F, 0x80, 0x0F, 0xF8, 0x1F, 0x80, 0x0F, 0xF8, 0x1F, 0x80, 0x0F, 0xF8, 0x1F, 0x80, 0x0F, 0xF8, 0x1F, 0x80, 0x07, 0xF8, 0x3F, 0xC0, 0x07, 0xFC, 0x7F, 0xC0, 0x03, 0xFE, 0xFE, 0xFF, 0xFF, 0x7F, 0xFE, 0xFF, 0xFF, 0x7F, 0xFE, 0xFF, 0xFF, 0x7F, 0xFC, 0xFF, 0xFF, 0x3F, 0xF0, 0xFF, 0xFF, 0x0F, 0xC0, 0xFF, 0xFF, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00   // Code for char 9
        };

/*
const uint8_t Envy_Code_R8x15[] = {
    // Offset, Width, Height, BPL, start char
                         17,8,15,2,32,

        0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // Code for char  
        0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // Code for char !
        0x06, 0x00, 0x00, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x00,  // Code for char "
        0x07, 0x00, 0x00, 0xA0, 0x00, 0xF8, 0x07, 0xA0, 0x00, 0xA0, 0x00, 0xF8, 0x07, 0xA0, 0x00, 0x00, 0x00,  // Code for char #
        0x07, 0x00, 0x00, 0x30, 0x04, 0x48, 0x04, 0x48, 0x04, 0xFC, 0x0F, 0x48, 0x04, 0x88, 0x03, 0x00, 0x00,  // Code for char $
        0x07, 0x38, 0x02, 0x28, 0x01, 0xB8, 0x00, 0x40, 0x00, 0xA0, 0x03, 0x90, 0x02, 0x88, 0x03, 0x00, 0x00,  // Code for char %
        0x07, 0x80, 0x07, 0x78, 0x0C, 0x64, 0x08, 0xE4, 0x09, 0xA4, 0x0F, 0x18, 0x0E, 0x80, 0x0B, 0x00, 0x00,  // Code for char &
        0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // Code for char '
        0x05, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x03, 0x0E, 0x1C, 0x01, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // Code for char (
        0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x20, 0x0E, 0x1C, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00,  // Code for char )
        0x07, 0x00, 0x00, 0x10, 0x00, 0x48, 0x00, 0x38, 0x00, 0x3C, 0x00, 0x48, 0x00, 0x10, 0x00, 0x00, 0x00,  // Code for char *
        0x07, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x40, 0x00, 0xF0, 0x01, 0x40, 0x00, 0x40, 0x00, 0x00, 0x00,  // Code for char +
        0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2C, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // Code for char ,
        0x07, 0x00, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x00, 0x00,  // Code for char -
        0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // Code for char .
        0x07, 0x00, 0x00, 0x00, 0x10, 0x00, 0x0C, 0x80, 0x03, 0xE0, 0x00, 0x18, 0x00, 0x04, 0x00, 0x00, 0x00,  // Code for char /
        0x07, 0x00, 0x00, 0xF8, 0x07, 0x0C, 0x0D, 0x84, 0x08, 0x44, 0x08, 0x2C, 0x0C, 0xF8, 0x07, 0x00, 0x00,  // Code for char 0
        0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x08, 0xFC, 0x0F, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00,  // Code for char 1
        0x07, 0x00, 0x00, 0x08, 0x0E, 0x04, 0x09, 0x84, 0x09, 0xC4, 0x08, 0x44, 0x08, 0x38, 0x08, 0x00, 0x00,  // Code for char 2
        0x07, 0x00, 0x00, 0x04, 0x08, 0x44, 0x08, 0x44, 0x08, 0x44, 0x08, 0xE4, 0x0C, 0x38, 0x07, 0x00, 0x00,  // Code for char 3
        0x07, 0x00, 0x00, 0x00, 0x03, 0xC0, 0x02, 0x70, 0x02, 0x1C, 0x02, 0xFC, 0x0F, 0x00, 0x02, 0x00, 0x00,  // Code for char 4
        0x07, 0x00, 0x00, 0x7C, 0x08, 0x44, 0x08, 0x44, 0x08, 0x44, 0x08, 0xC4, 0x0C, 0x84, 0x07, 0x00, 0x00,  // Code for char 5
        0x07, 0x00, 0x00, 0xE0, 0x07, 0xD0, 0x0C, 0x48, 0x08, 0x44, 0x08, 0xC4, 0x0C, 0x80, 0x07, 0x00, 0x00,  // Code for char 6
        0x07, 0x00, 0x00, 0x04, 0x00, 0x04, 0x08, 0x04, 0x06, 0xC4, 0x01, 0x34, 0x00, 0x0C, 0x00, 0x00, 0x00,  // Code for char 7
        0x07, 0x00, 0x00, 0x38, 0x07, 0xCC, 0x0C, 0x44, 0x08, 0x44, 0x08, 0xCC, 0x0C, 0x38, 0x07, 0x00, 0x00,  // Code for char 8
        0x07, 0x00, 0x00, 0x38, 0x00, 0x6C, 0x08, 0x44, 0x0C, 0x44, 0x06, 0xEC, 0x03, 0xF8, 0x00, 0x00, 0x00,  // Code for char 9
        0x04, 0x00, 0x00, 0x00, 0x00, 0x60, 0x0C, 0x60, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // Code for char :
        0x04, 0x00, 0x00, 0x00, 0x00, 0x60, 0x2C, 0x60, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // Code for char ;
        0x05, 0x00, 0x00, 0xC0, 0x00, 0xE0, 0x01, 0x30, 0x03, 0x18, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // Code for char <
        0x07, 0xA0, 0x00, 0xA0, 0x00, 0xA0, 0x00, 0xA0, 0x00, 0xA0, 0x00, 0xA0, 0x00, 0xA0, 0x00, 0x00, 0x00,  // Code for char =
        0x05, 0x00, 0x00, 0x18, 0x06, 0x30, 0x03, 0xE0, 0x01, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // Code for char >
        0x07, 0x00, 0x00, 0x18, 0x00, 0x04, 0x00, 0x04, 0x00, 0x84, 0x0D, 0x44, 0x00, 0x38, 0x00, 0x00, 0x00,  // Code for char ?
        0x07, 0xF0, 0x0F, 0x08, 0x10, 0xC4, 0x23, 0x24, 0x24, 0x24, 0x24, 0x28, 0x24, 0xF0, 0x07, 0x00, 0x00,  // Code for char @
        0x07, 0x00, 0x00, 0xF8, 0x0F, 0x4C, 0x00, 0x44, 0x00, 0x44, 0x00, 0x4C, 0x00, 0xF8, 0x0F, 0x00, 0x00,  // Code for char A
        0x07, 0x00, 0x00, 0xFC, 0x0F, 0x44, 0x08, 0x44, 0x08, 0x44, 0x08, 0xEC, 0x0C, 0x38, 0x07, 0x00, 0x00,  // Code for char B
        0x07, 0x00, 0x00, 0xF0, 0x03, 0x08, 0x04, 0x04, 0x08, 0x04, 0x08, 0x04, 0x08, 0x08, 0x04, 0x00, 0x00,  // Code for char C
        0x07, 0x00, 0x00, 0xFC, 0x0F, 0x04, 0x08, 0x04, 0x08, 0x04, 0x08, 0x08, 0x04, 0xF0, 0x03, 0x00, 0x00,  // Code for char D
        0x07, 0x00, 0x00, 0xFC, 0x0F, 0x44, 0x08, 0x44, 0x08, 0x44, 0x08, 0x44, 0x08, 0x04, 0x08, 0x00, 0x00,  // Code for char E
        0x07, 0x00, 0x00, 0xFC, 0x0F, 0x44, 0x00, 0x44, 0x00, 0x44, 0x00, 0x44, 0x00, 0x04, 0x00, 0x00, 0x00,  // Code for char F
        0x07, 0x00, 0x00, 0xF8, 0x07, 0x0C, 0x0C, 0x04, 0x08, 0x04, 0x08, 0x4C, 0x0C, 0xC8, 0x0F, 0x00, 0x00,  // Code for char G
        0x07, 0x00, 0x00, 0xFC, 0x0F, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0xFC, 0x0F, 0x00, 0x00,  // Code for char H
        0x07, 0x00, 0x00, 0x00, 0x00, 0x04, 0x08, 0x04, 0x08, 0xFC, 0x0F, 0x04, 0x08, 0x04, 0x08, 0x00, 0x00,  // Code for char I
        0x07, 0x00, 0x00, 0x00, 0x06, 0x00, 0x08, 0x00, 0x08, 0x00, 0x08, 0x00, 0x08, 0xFC, 0x07, 0x00, 0x00,  // Code for char J
        0x07, 0x00, 0x00, 0xFC, 0x0F, 0xC0, 0x00, 0x20, 0x01, 0x10, 0x02, 0x08, 0x04, 0x04, 0x08, 0x00, 0x00,  // Code for char K
        0x07, 0x00, 0x00, 0xFC, 0x0F, 0x00, 0x08, 0x00, 0x08, 0x00, 0x08, 0x00, 0x08, 0x00, 0x08, 0x00, 0x00,  // Code for char L
        0x07, 0x00, 0x00, 0xFC, 0x0F, 0x1C, 0x00, 0x70, 0x00, 0x30, 0x00, 0x1C, 0x00, 0xFC, 0x0F, 0x00, 0x00,  // Code for char M
        0x07, 0x00, 0x00, 0xFC, 0x0F, 0x18, 0x00, 0x60, 0x00, 0x80, 0x01, 0x00, 0x06, 0xFC, 0x0F, 0x00, 0x00,  // Code for char N
        0x07, 0x00, 0x00, 0xF8, 0x07, 0x0C, 0x0C, 0x04, 0x08, 0x04, 0x08, 0x0C, 0x0C, 0xF8, 0x07, 0x00, 0x00,  // Code for char O
        0x07, 0x00, 0x00, 0xFC, 0x0F, 0x44, 0x00, 0x44, 0x00, 0x44, 0x00, 0x6C, 0x00, 0x38, 0x00, 0x00, 0x00,  // Code for char P
        0x07, 0x00, 0x00, 0xF8, 0x07, 0x0C, 0x0C, 0x04, 0x08, 0x04, 0x0A, 0x0C, 0x0C, 0xF8, 0x17, 0x00, 0x00,  // Code for char Q
        0x07, 0x00, 0x00, 0xFC, 0x0F, 0x44, 0x00, 0x44, 0x00, 0xC4, 0x01, 0x6C, 0x06, 0x38, 0x08, 0x00, 0x00,  // Code for char R
        0x07, 0x00, 0x00, 0x38, 0x08, 0x44, 0x08, 0x44, 0x08, 0x44, 0x08, 0x44, 0x08, 0x84, 0x07, 0x00, 0x00,  // Code for char S
        0x07, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x04, 0x00, 0xFC, 0x0F, 0x04, 0x00, 0x04, 0x00, 0x00, 0x00,  // Code for char T
        0x07, 0x00, 0x00, 0xFC, 0x07, 0x00, 0x0C, 0x00, 0x08, 0x00, 0x08, 0x00, 0x0C, 0xFC, 0x07, 0x00, 0x00,  // Code for char U
        0x07, 0x00, 0x00, 0x0C, 0x00, 0x70, 0x00, 0x80, 0x03, 0x00, 0x0C, 0xE0, 0x03, 0x1C, 0x00, 0x00, 0x00,  // Code for char V
        0x07, 0x00, 0x00, 0xFC, 0x0F, 0x00, 0x06, 0x80, 0x03, 0x00, 0x03, 0x00, 0x06, 0xFC, 0x0F, 0x00, 0x00,  // Code for char W
        0x07, 0x00, 0x00, 0x04, 0x08, 0x18, 0x06, 0xE0, 0x01, 0xE0, 0x01, 0x18, 0x06, 0x04, 0x08, 0x00, 0x00,  // Code for char X
        0x07, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x70, 0x00, 0x80, 0x0F, 0x70, 0x00, 0x0C, 0x00, 0x00, 0x00,  // Code for char Y
        0x07, 0x00, 0x00, 0x04, 0x0C, 0x04, 0x0A, 0x84, 0x09, 0x64, 0x08, 0x14, 0x08, 0x0C, 0x08, 0x00, 0x00,  // Code for char Z
        0x05, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x3F, 0x01, 0x20, 0x01, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // Code for char [
        0x07, 0x00, 0x00, 0x04, 0x00, 0x18, 0x00, 0xE0, 0x00, 0x80, 0x03, 0x00, 0x0C, 0x00, 0x10, 0x00, 0x00,  // Code for char BackSlash
        0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x20, 0x01, 0x20, 0xFF, 0x3F, 0x00, 0x00, 0x00, 0x00,  // Code for char ]
        0x07, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x38, 0x00, 0x0C, 0x00, 0x38, 0x00, 0x60, 0x00, 0x00, 0x00,  // Code for char ^
        0x08, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40, 0x00, 0x40,  // Code for char _
        0x05, 0x00, 0x00, 0x03, 0x00, 0x02, 0x00, 0x04, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // Code for char `
        0x07, 0x00, 0x00, 0x00, 0x07, 0x90, 0x08, 0x90, 0x08, 0x90, 0x08, 0xB0, 0x08, 0xE0, 0x0F, 0x00, 0x00,  // Code for char a
        0x07, 0x00, 0x00, 0xFC, 0x0F, 0x20, 0x04, 0x10, 0x08, 0x10, 0x08, 0x10, 0x08, 0xE0, 0x07, 0x00, 0x00,  // Code for char b
        0x07, 0x00, 0x00, 0xE0, 0x07, 0x10, 0x08, 0x10, 0x08, 0x10, 0x08, 0x10, 0x08, 0x20, 0x04, 0x00, 0x00,  // Code for char c
        0x07, 0x00, 0x00, 0xC0, 0x03, 0x20, 0x04, 0x10, 0x08, 0x10, 0x08, 0x20, 0x04, 0xFC, 0x0F, 0x00, 0x00,  // Code for char d
        0x07, 0x00, 0x00, 0xE0, 0x07, 0xB0, 0x0C, 0x90, 0x08, 0x90, 0x08, 0xB0, 0x08, 0xE0, 0x08, 0x00, 0x00,  // Code for char e
        0x07, 0x00, 0x00, 0x20, 0x00, 0xF8, 0x0F, 0x2C, 0x00, 0x24, 0x00, 0x24, 0x00, 0x04, 0x00, 0x00, 0x00,  // Code for char f
        0x07, 0x00, 0x00, 0xE0, 0x07, 0x30, 0x4C, 0x10, 0x48, 0x10, 0x48, 0x30, 0x6C, 0xF0, 0x3F, 0x00, 0x00,  // Code for char g
        0x07, 0x00, 0x00, 0xFC, 0x0F, 0x20, 0x00, 0x10, 0x00, 0x10, 0x00, 0x20, 0x00, 0xC0, 0x0F, 0x00, 0x00,  // Code for char h
        0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0xF4, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // Code for char i
        0x06, 0x00, 0x00, 0x00, 0x20, 0x00, 0x20, 0x00, 0x20, 0x10, 0x20, 0xF4, 0x1F, 0x00, 0x00, 0x00, 0x00,  // Code for char j
        0x07, 0x00, 0x00, 0xFC, 0x0F, 0x80, 0x00, 0x40, 0x01, 0x20, 0x02, 0x10, 0x04, 0x00, 0x08, 0x00, 0x00,  // Code for char k
        0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0xFC, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // Code for char l
        0x06, 0x00, 0x00, 0xF0, 0x0F, 0x10, 0x00, 0xE0, 0x0F, 0x10, 0x00, 0xF0, 0x0F, 0x00, 0x00, 0x00, 0x00,  // Code for char m
        0x07, 0x00, 0x00, 0xF0, 0x0F, 0x20, 0x00, 0x10, 0x00, 0x10, 0x00, 0x20, 0x00, 0xC0, 0x0F, 0x00, 0x00,  // Code for char n
        0x07, 0x00, 0x00, 0xE0, 0x07, 0x30, 0x0C, 0x10, 0x08, 0x10, 0x08, 0x30, 0x0C, 0xE0, 0x07, 0x00, 0x00,  // Code for char o
        0x07, 0x00, 0x00, 0xF0, 0x7F, 0x30, 0x0C, 0x10, 0x08, 0x10, 0x08, 0x30, 0x0C, 0xE0, 0x07, 0x00, 0x00,  // Code for char p
        0x07, 0x00, 0x00, 0xE0, 0x07, 0x30, 0x0C, 0x10, 0x08, 0x10, 0x08, 0x30, 0x0C, 0xF0, 0x7F, 0x00, 0x00,  // Code for char q
        0x07, 0x00, 0x00, 0xF0, 0x0F, 0x20, 0x00, 0x10, 0x00, 0x10, 0x00, 0x10, 0x00, 0x20, 0x00, 0x00, 0x00,  // Code for char r
        0x07, 0x00, 0x00, 0x60, 0x08, 0x90, 0x08, 0x90, 0x08, 0x90, 0x08, 0x90, 0x08, 0x10, 0x07, 0x00, 0x00,  // Code for char s
        0x07, 0x00, 0x00, 0x10, 0x00, 0xFC, 0x07, 0x10, 0x0C, 0x10, 0x08, 0x10, 0x08, 0x00, 0x08, 0x00, 0x00,  // Code for char t
        0x07, 0x00, 0x00, 0xF0, 0x07, 0x00, 0x0C, 0x00, 0x08, 0x00, 0x08, 0x00, 0x0C, 0xF0, 0x0F, 0x00, 0x00,  // Code for char u
        0x07, 0x00, 0x00, 0x30, 0x00, 0xC0, 0x01, 0x00, 0x0E, 0x00, 0x0E, 0xC0, 0x01, 0x30, 0x00, 0x00, 0x00,  // Code for char v
        0x07, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x07, 0x00, 0x08, 0x80, 0x07, 0x00, 0x08, 0xF0, 0x07, 0x00, 0x00,  // Code for char w
        0x07, 0x00, 0x00, 0x00, 0x00, 0x10, 0x08, 0x60, 0x06, 0x80, 0x01, 0x60, 0x06, 0x10, 0x08, 0x00, 0x00,  // Code for char x
        0x07, 0x00, 0x00, 0xF0, 0x07, 0x00, 0x4C, 0x00, 0x48, 0x00, 0x48, 0x00, 0x6C, 0xF0, 0x3F, 0x00, 0x00,  // Code for char y
        0x07, 0x00, 0x00, 0x10, 0x0C, 0x10, 0x0A, 0x10, 0x09, 0x90, 0x08, 0x50, 0x08, 0x30, 0x08, 0x00, 0x00,  // Code for char z
        0x07, 0x00, 0x00, 0x40, 0x00, 0x5E, 0x1E, 0xB1, 0x33, 0x01, 0x20, 0x01, 0x20, 0x01, 0x20, 0x00, 0x00,  // Code for char {
        0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // Code for char |
        0x07, 0x00, 0x00, 0x01, 0x20, 0x01, 0x20, 0x01, 0x20, 0xB1, 0x33, 0x5E, 0x1E, 0x40, 0x00, 0x00, 0x00,  // Code for char }
        0x07, 0x00, 0x00, 0xC0, 0x00, 0x20, 0x00, 0x60, 0x00, 0xC0, 0x00, 0x80, 0x00, 0x60, 0x00, 0x00, 0x00,  // Code for char ~
        0x04, 0xFC, 0x1F, 0x04, 0x10, 0x04, 0x10, 0xFC, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00   // Code for char 
        };
*/
