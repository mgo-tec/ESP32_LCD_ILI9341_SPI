/*
  ESP32_LCD_ILI9341_SPI.h - for Arduino core for the ESP32 ( Use SPI library ).
  Beta version 1.27
  
The MIT License (MIT)

Copyright (c) 2018 Mgo-tec. All rights reserved.
Blog URL ---> https://www.mgo-tec.com

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

Modify Display.cpp of M5stack library.
M5stack library - MIT License
Copyright (c) 2017 M5Stack
*/

#ifndef _ESP32_LCD_ILI9341_SPI_H_INCLUDED
#define _ESP32_LCD_ILI9341_SPI_H_INCLUDED

#include <Arduino.h>
#include <SPI.h>
#include "soc/spi_reg.h"

class ESP32_LCD_ILI9341_SPI
{
private:
  int8_t _sck;
  int8_t _miso;
  int8_t _mosi;
  int8_t _cs;
  int8_t _dc;
  int8_t _rst;
  int8_t _ledpin;

  bool _Hw_cs;
  bool mp_isIPS_lcd = false;

  uint32_t _freq;

  const uint8_t _SPI_NUM = 0x3; //VSPI=0x3, HSPI=0x2
  uint16_t _Max_Width_x = 320;
  uint16_t _Max_Width_y = 240;
  #define _Max_Num 10

  uint16_t _Max_Pix_X = _Max_Width_x - 1;
  uint16_t _Max_Pix_Y = _Max_Width_y - 1;
  uint8_t _txt_H_max = _Max_Width_x / 8;

  uint8_t _H_Size[_Max_Num];
  uint8_t _V_Size[_Max_Num];
  uint8_t _Dot_MSB[_Max_Num] = {};
  uint8_t _Dot_LSB[_Max_Num] = {};
  uint8_t _BG_Dot_MSB[_Max_Num] = {};
  uint8_t _BG_Dot_LSB[_Max_Num] = {};
  uint16_t _scl_pix_cnt[_Max_Num] = {};
  uint16_t _prev_scl_pix_cnt[_Max_Num] = {};
  uint16_t _H_pix_max[_Max_Num];
  uint16_t _tmp_width1[_Max_Num];
  uint32_t _scl_LastTime[_Max_Num];

  int8_t _scl_cnt[ _Max_Num ] = {};
  uint16_t _fnt_cnt[ _Max_Num ] = {};
  uint32_t _scl_speed[ _Max_Num ] = {};
  uint16_t _txt_length[ _Max_Num ] = {};
  uint8_t _txt_width = {};
  uint16_t _zen_or_han_cnt[ _Max_Num ] = {};

  uint16_t _X0, _Y0 = {};

  #define SWAP(type, x, y) do { type tmp = x; x = y; y = tmp; } while (0)

public:
  ESP32_LCD_ILI9341_SPI(int8_t sck, int8_t miso, int8_t mosi, int8_t cs, int8_t dc, int8_t rst, int8_t led);

  void ILI9341_Init();
  void Disp_Rotation(uint8_t rot);
  void ILI9341_Init(uint32_t clk);
  void ILI9341_Init(bool hwcs, uint32_t clk);
  void dispInversionOn();
  void dispInversionOff();
  void CommandWrite(uint8_t b);
  void DataWrite(uint8_t b);
  void DataWrite16(uint16_t b);
  void DataWrite32(uint32_t b);
  void DataWriteBytes(uint8_t *b, uint32_t b_size);

  void SPI_set_change();

  void XY_Range(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);

  void Scrolle_Font_SetUp(uint8_t num, uint8_t txt_width, uint8_t red, uint8_t green, uint8_t blue, uint8_t bg_red = 0, uint8_t bg_green = 0, uint8_t bg_blue = 0, uint16_t x0 = 0, uint16_t y0 = 0, uint8_t H_size = 1, uint8_t V_size = 1, bool xy_range_set = false );
  void Scrolle_Font_SetUp(uint8_t num, uint8_t txt_width, uint16_t txt_length, uint8_t red, uint8_t green, uint8_t blue, uint8_t bg_red = 0, uint8_t bg_green = 0, uint8_t bg_blue = 0, uint16_t x0 = 0, uint16_t y0 = 0, uint8_t H_size = 1, uint8_t V_size = 1, bool xy_range_set = false );
  void Scrolle_Font_SetUp(uint8_t num, uint8_t txt_width, uint16_t txt_length, uint8_t H_size, uint8_t V_size, uint32_t scl_speed, uint8_t red, uint8_t green, uint8_t blue, uint8_t bg_red = 0, uint8_t bg_green = 0, uint8_t bg_blue = 0, bool xy_range_set = false);

  void Scrolle_XYrange_Com_Change(uint8_t num);
  void Scrolle_XYrange_Com_Change(uint8_t num, uint16_t x0, uint16_t y0);

  void Scrolle_HVsizeUp_8x16_Font_DisplayOut(uint8_t num, uint32_t scl_speed, uint8_t H_size, uint8_t V_size, uint8_t Fnt[][16], uint8_t scl_buf[][640]);

  void Scrolle_HVsizeUp_8x16_Font_DisplayOut(uint8_t num, uint16_t x0, uint16_t y0, uint8_t Fnt[][16], uint8_t scl_buf[][640]);
  void Scrolle_HVsizeUp_8x16_Font_DisplayOut(uint8_t num, uint32_t scl_speed, uint8_t H_size, uint8_t V_size, uint16_t x0, uint16_t y0, uint8_t Fnt[][16], uint8_t scl_buf[][640]);
  void Scrolle_HVsizeUp_8x16_Font_DisplayOut(uint8_t num, uint32_t scl_speed, uint8_t H_size, uint8_t V_size, int8_t *scl_cnt, uint16_t *fnt_cnt, uint16_t txt_length, uint16_t x0, uint16_t y0, uint8_t Fnt[][16], uint8_t scl_buf[][640], bool xy_range_set = true);
  void Scrolle_HVsizeUp_8x16_Font_DisplayOut(uint8_t num, int8_t *scl_cnt, uint16_t *fnt_cnt, uint16_t x0, uint16_t y0, uint8_t Fnt[][16], uint8_t scl_buf[][640], bool xy_range_set = true);

  boolean Scrolle_Inc_HVsizeUp_8x16_Font_DisplayOut(uint8_t num, uint8_t zen_or_han, uint32_t scl_speed, uint8_t H_size, uint8_t V_size, int8_t *scl_cnt, uint16_t txt_length, uint8_t Fnt[][16], uint8_t scl_buf[][640]);
  boolean Scrolle_Inc_HVsizeUp_8x16_Font_DisplayOut(uint8_t num, uint8_t zen_or_han, uint32_t scl_speed, uint8_t H_size, uint8_t V_size, int8_t *scl_cnt, uint16_t txt_length, uint16_t x0, uint16_t y0, uint8_t Fnt[][16], uint8_t scl_buf[][640], bool xy_range_set = true);

  void HVsizeUp_8x16_Font_DisplayOut(uint8_t H_Size, uint8_t V_Size, uint16_t txt_length, uint16_t x0, uint16_t y0, uint8_t red, uint8_t green, uint8_t blue, uint8_t Fnt[][16], uint8_t bg_red = 0, uint8_t bg_green = 0, uint8_t bg_blue = 0 );
  void Font_8x16_DisplayOut(uint16_t txt_length, uint16_t x0, uint16_t y0, uint8_t red, uint8_t green, uint8_t blue, uint8_t Fnt[][16]);

  void Display_Clear();
  void Display_Clear(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
  void spiWriteBlock(uint16_t color, uint32_t repeat);
  void Block_SPI_Fast_Write(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t red, uint8_t green, uint8_t blue, uint32_t repeat);
  void Draw_Pixel_65k_DotColor_sd(uint16_t x0, uint16_t y0, uint16_t DotColor);
  void Draw_Pixel_65k_DotColor(uint16_t x0, uint16_t y0, uint16_t DotColor);
  void Draw_Pixel_65k_3Color_sd(uint16_t x0, uint16_t y0, uint8_t red, uint8_t green, uint8_t blue);
  void Draw_Pixel_65k_3Color(uint16_t x0, uint16_t y0, uint8_t red, uint8_t green, uint8_t blue);
  void Draw_Line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint8_t red, uint8_t green, uint8_t blue);
  void Draw_Horizontal_Line(int16_t x0, int16_t x1, int16_t y0, uint8_t red, uint8_t green, uint8_t blue);
  void Draw_Vertical_Line(int16_t x0, int16_t y0, int16_t y1, uint8_t red, uint8_t green, uint8_t blue);
  void Draw_Rectangle_Line(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t red, uint8_t green, uint8_t blue);
  void Draw_Rectangle_Fill(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t red, uint8_t green, uint8_t blue);
  void Draw_Circle_Line(uint16_t x0, uint16_t y0, uint16_t r, uint8_t red, uint8_t green, uint8_t blue);
  void Draw_Circle_Fill(uint16_t x0, uint16_t y0, uint16_t r, uint8_t red, uint8_t green, uint8_t blue);

  //void Idle_mode_OFF();
  //void Idle_mode_ON();
  void Brightness(uint8_t brightness);

};

#endif
