/*
  ESP32_LCD_ILI9341_SPI.cpp - for Arduino core for the ESP32 ( Use SPI library ).
  Beta version 1.1
  
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

#include "ESP32_LCD_ILI9341_SPI.h"

ESP32_LCD_ILI9341_SPI::ESP32_LCD_ILI9341_SPI(int8_t sck, int8_t miso, int8_t mosi, int8_t cs, int8_t dc, int8_t rst, int8_t led)
  :_sck(sck)
  , _miso(miso)
  , _mosi(mosi)
  , _cs(cs)
  , _dc(dc)
  , _rst(rst)
  , _ledpin(led)
{}

//************ ILI9341 初期化 ********************
void ESP32_LCD_ILI9341_SPI::ILI9341_Init(){
  ESP32_LCD_ILI9341_SPI::ILI9341_Init(true, 27000000);
}
//************ ILI9341 初期化 ********************
void ESP32_LCD_ILI9341_SPI::ILI9341_Init(uint32_t clk){
  ESP32_LCD_ILI9341_SPI::ILI9341_Init(true, clk);
}
//****** LCD ILI9341 ディスプレイ初期化 ***********
void ESP32_LCD_ILI9341_SPI::ILI9341_Init(bool hwcs, uint32_t clk){
  ESP32_LCD_ILI9341_SPI::Brightness(0);

  _Hw_cs = hwcs;
  _txt_H_max = _Max_Width_x / 8;

  pinMode(_rst, OUTPUT); //Set RESET pin
  pinMode(_dc, OUTPUT); //Set Data/Command pin

  SPI.begin(_sck, _miso, _mosi, _cs); //VSPI setting

  SPI.setBitOrder(MSBFIRST);
  //ILI9341 のSPI Clock Cycle Time (Write) Minimun 100ns=10MHz
  SPI.setFrequency(clk);
  SPI.setDataMode(SPI_MODE0);
  SPI.setHwCs(_Hw_cs); //Set Hardware CS pin

  //Hardware Reset------------
  digitalWrite(_rst, HIGH);
  delay(5);
  digitalWrite(_rst, LOW);
  delay(10);
  digitalWrite(_rst, HIGH);
  delay(121);

  if(!_Hw_cs){
    pinMode(_cs, OUTPUT);
    digitalWrite(_cs, HIGH);
    digitalWrite(_cs, LOW);
  }
  digitalWrite(_dc, HIGH);

  ESP32_LCD_ILI9341_SPI::CommandWrite(0x38); //Idle mode OFF
  ESP32_LCD_ILI9341_SPI::CommandWrite(0x3A); //COLMOD: Pixel Format Set
    ESP32_LCD_ILI9341_SPI::DataWrite(0b01010101); //RGB 16 bits / pixel, MCU 16 bits / pixel
  ESP32_LCD_ILI9341_SPI::CommandWrite(0x36); //MADCTL: Memory Access Control
    ESP32_LCD_ILI9341_SPI::DataWrite(0b00001000); //D3: BGR(RGB-BGR Order control bit )="1"
  ESP32_LCD_ILI9341_SPI::CommandWrite(0x11); //Sleep OUT
  delay(10);

  ESP32_LCD_ILI9341_SPI::CommandWrite(0x29); //Display ON

  ESP32_LCD_ILI9341_SPI::Display_Clear(0, 0, 319, 239);

  if(!_Hw_cs) digitalWrite(_cs, HIGH);
  ESP32_LCD_ILI9341_SPI::Brightness(100);
}
//********* 4wire SPI Data / Command write************
void ESP32_LCD_ILI9341_SPI::CommandWrite(uint8_t b){
  digitalWrite(_dc, LOW);
  SPI.write(b);
  digitalWrite(_dc, HIGH);
}

void ESP32_LCD_ILI9341_SPI::DataWrite(uint8_t b){
  SPI.write(b);
}

void ESP32_LCD_ILI9341_SPI::DataWrite16(uint16_t b){
  SPI.write16(b);
}

void ESP32_LCD_ILI9341_SPI::DataWrite32(uint32_t b){
  SPI.write32(b);
}

void ESP32_LCD_ILI9341_SPI::DataWriteBytes(uint8_t *b, uint32_t b_size){
  //ESP32_LCD_ILI9341_SPI::CommandWrite( 0x2C ); //RAM write
  SPI.writeBytes(b, b_size);
}
//******** Set Column and Page Address ( X Y range setting )***********
void ESP32_LCD_ILI9341_SPI::XY_Range(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1){
  uint32_t X = (uint32_t)x0<<16 | x1;
  uint32_t Y = (uint32_t)y0<<16 | y1;

  ESP32_LCD_ILI9341_SPI::CommandWrite( 0x2A ); //Set Column Address
    ESP32_LCD_ILI9341_SPI::DataWrite32(X);
  ESP32_LCD_ILI9341_SPI::CommandWrite( 0x2B ); //Set Page Address
    ESP32_LCD_ILI9341_SPI::DataWrite32(Y);
}
//**********スクロールセットアップ*******************
void ESP32_LCD_ILI9341_SPI::Scrolle_Font_SetUp(uint8_t num, uint8_t txt_width, uint16_t txt_length, uint8_t red, uint8_t green, uint8_t blue){
  ESP32_LCD_ILI9341_SPI::Scrolle_Font_SetUp(num, txt_width, txt_length, 1, 1, 10, red, green, blue);
}
//**********スクロールセットアップ*******************
void ESP32_LCD_ILI9341_SPI::Scrolle_Font_SetUp(uint8_t num, uint8_t txt_width, uint16_t txt_length, uint8_t H_size, uint8_t V_size, uint32_t scl_speed, uint8_t red, uint8_t green, uint8_t blue){
  _txt_length[num] = txt_length;
  _H_Size[num] = H_size;
  _V_Size[num] = V_size;
  _txt_length[num] = txt_length;

  _Dot_MSB[num] = (red << 3) | (green >> 3);
  _Dot_LSB[num] = (green << 5) | blue;

  if(_H_Size[num] > 15) _H_Size[num] = 15;
  if(_V_Size[num] > 15) _V_Size[num] = 15;
  if(txt_width > 40) txt_width = 40;

  _tmp_width1[num] = txt_width * ( 8 * _H_Size[num] );
  if( _tmp_width1[num] > _Max_Width_x ) _tmp_width1[num] = _Max_Width_x;

  _H_pix_max[num] = _tmp_width1[num] - 1;
}

//********* スクロール ********************
void ESP32_LCD_ILI9341_SPI::Scrolle_HVsizeUp_8x16_Font_DisplayOut(uint8_t num, uint16_t x0, uint16_t y0, uint8_t Fnt[][16], uint8_t scl_buf[][640]){
  ESP32_LCD_ILI9341_SPI::Scrolle_HVsizeUp_8x16_Font_DisplayOut(num, &_scl_cnt[num], &_fnt_cnt[num], x0, y0, Fnt, scl_buf);
}
//********* スクロール ********************
void ESP32_LCD_ILI9341_SPI::Scrolle_HVsizeUp_8x16_Font_DisplayOut(uint8_t num, uint32_t scl_speed, uint8_t H_size, uint8_t V_size, uint16_t x0, uint16_t y0, uint8_t Fnt[][16], uint8_t scl_buf[][640]){
  ESP32_LCD_ILI9341_SPI::Scrolle_HVsizeUp_8x16_Font_DisplayOut(num, scl_speed, H_size, V_size, &_scl_cnt[num], &_fnt_cnt[num], _txt_length[num], x0, y0, Fnt, scl_buf);
}
//********* スクロール ********************
void ESP32_LCD_ILI9341_SPI::Scrolle_HVsizeUp_8x16_Font_DisplayOut(uint8_t num, uint32_t scl_speed, uint8_t H_size, uint8_t V_size, int8_t *scl_cnt, uint16_t *fnt_cnt, uint16_t txt_length, uint16_t x0, uint16_t y0, uint8_t Fnt[][16], uint8_t scl_buf[][640]){
  _scl_speed[num] = scl_speed;
  _H_Size[num] = H_size;
  _V_Size[num] = V_size;
  _txt_length[num] = txt_length;

  ESP32_LCD_ILI9341_SPI::Scrolle_HVsizeUp_8x16_Font_DisplayOut(num, scl_cnt, fnt_cnt, x0, y0, Fnt, scl_buf);
}
//********* スクロール ********************
void ESP32_LCD_ILI9341_SPI::Scrolle_HVsizeUp_8x16_Font_DisplayOut(uint8_t num, int8_t *scl_cnt, uint16_t *fnt_cnt, uint16_t x0, uint16_t y0, uint8_t Fnt[][16], uint8_t scl_buf[][640]){
  //SPI.setFrequency(27000000);
  //SPI.setDataMode(SPI_MODE0);
  //spi.setHwCs(true);

  if( millis() - _scl_LastTime[num] > _scl_speed[num]){
    uint16_t array_max = (_tmp_width1[num] - x0) * 2;
    uint8_t bt1 = 0b10000000;
    int i, j;

    ESP32_LCD_ILI9341_SPI::XY_Range(x0, y0, _H_pix_max[num], y0 + 16 * _V_Size[num] - 1);
    ESP32_LCD_ILI9341_SPI::CommandWrite( 0x2C ); //RAM write

    uint16_t b2 = array_max - _scl_pix_cnt[num];
    for(i = 0; i < 16; i++){
      for(j = _V_Size[num]; j > 0; j--){
        ESP32_LCD_ILI9341_SPI::DataWriteBytes(&scl_buf[i][_prev_scl_pix_cnt[num]], b2);
        ESP32_LCD_ILI9341_SPI::DataWriteBytes(scl_buf[i], _scl_pix_cnt[num]);
      }
    }

    if( _scl_pix_cnt[num] < (array_max - _H_Size[num] * 2) ){
      _prev_scl_pix_cnt[num] = _prev_scl_pix_cnt[num] + _H_Size[num] * 2;
    }else{
      _prev_scl_pix_cnt[num] = 0; //ポインタをゼロにリセット
    }

    uint16_t count;
    for(i = 0; i < 16; i++){
      count = _scl_pix_cnt[num];
      if( Fnt[*fnt_cnt][i] & (bt1 >> *scl_cnt) ){
        for( j = _H_Size[num]; j > 0; j-- ){
          scl_buf[i][count++] = _Dot_MSB[num];
          scl_buf[i][count++] = _Dot_LSB[num];
          if(count >= array_max) break;
        }
      }else{
        for( j = _H_Size[num]; j > 0; j-- ){
          scl_buf[i][count++] = 0;
          scl_buf[i][count++] = 0;
          if(count >= array_max) break;
        }
      }
    }
    _scl_pix_cnt[num] = count;

    if( _scl_pix_cnt[num] >= array_max ) _scl_pix_cnt[num] = 0;

    if( ++(*scl_cnt) > 7 ){
      *scl_cnt = 0;
      if( ++(*fnt_cnt) >= _txt_length[num] ) {
        *fnt_cnt = 0;
      }
    }
    _scl_LastTime[num] = millis();
  }
}
//********* OLED 8x16フォント　サイズアップ　出力 ********************
void ESP32_LCD_ILI9341_SPI::HVsizeUp_8x16_Font_DisplayOut(uint8_t H_Size, uint8_t V_Size, uint16_t txt_length, uint16_t x0, uint16_t y0, uint8_t red, uint8_t green, uint8_t blue, uint8_t Fnt[][16]){
  //SPI.setFrequency(27000000);
  //SPI.setDataMode(SPI_MODE0);
  //spi.setHwCs(true);

  if(H_Size > 16) H_Size = 16;
  if(V_Size > 16) V_Size = 16;

  uint8_t X_txt_MAX = (uint8_t)ceilf( (float)_txt_H_max / (float)H_Size );
  if( txt_length > X_txt_MAX) txt_length = X_txt_MAX;

  uint8_t bt = 0b00000001;
  uint8_t dot_MSB = (red << 3) | (green >> 3);
  uint8_t dot_LSB = (green << 5) | blue;
  uint8_t disp_byte[(txt_length * (8 * H_Size) - x0) * 2] = {};
  uint16_t byte_cnt = 0;
  uint16_t X_pix_cnt = x0, Y_pix_cnt = y0;
  uint16_t Y_tmp_range = 0;

  int i, j, k, ii, jj;
  for(i = 0; i < 16; i++){
    for(j = 0; j < txt_length; j++){
      for( k = 7; k >= 0; k--){
        if( Fnt[j][i] & (bt << k) ){
          for(ii = H_Size; ii > 0; ii--){
            disp_byte[byte_cnt++] = dot_MSB;
            disp_byte[byte_cnt++] = dot_LSB;
            X_pix_cnt++;
            if(X_pix_cnt > _Max_Pix_X) goto max_pixX;
          }
        }else{
          for(ii = H_Size; ii > 0; ii--){
            disp_byte[byte_cnt++] = 0;
            disp_byte[byte_cnt++] = 0;
            X_pix_cnt++;
            if(X_pix_cnt > _Max_Pix_X) goto max_pixX;
          }
        }
      }
    }
max_pixX:
    Y_tmp_range = Y_pix_cnt + V_Size -1;
    if(Y_tmp_range > _Max_Pix_Y) Y_tmp_range = _Max_Pix_Y;
    ESP32_LCD_ILI9341_SPI::XY_Range(x0, Y_pix_cnt, X_pix_cnt - 1, Y_tmp_range);
    ESP32_LCD_ILI9341_SPI::CommandWrite( 0x2C ); //RAM write
    for(jj = 0; jj < V_Size; jj++){
      ESP32_LCD_ILI9341_SPI::DataWriteBytes(disp_byte, byte_cnt);
      Y_pix_cnt++;
      if(Y_pix_cnt > _Max_Pix_Y) return;
    }
    X_pix_cnt = x0;
    byte_cnt = 0;
  }
}
//********* LCD ILE9341 8x16 font 1line display out ********************
void ESP32_LCD_ILI9341_SPI::Font_8x16_DisplayOut(uint16_t txt_length, uint16_t x0, uint16_t y0, uint8_t red, uint8_t green, uint8_t blue, uint8_t Fnt[][16]){
  ESP32_LCD_ILI9341_SPI::HVsizeUp_8x16_Font_DisplayOut(1, 1, txt_length, x0, y0, red, green, blue, Fnt);
}
//********* Display All Black Clear ******************************
void ESP32_LCD_ILI9341_SPI::Display_Clear(){
  ESP32_LCD_ILI9341_SPI::Display_Clear(0, 0, _Max_Pix_X, _Max_Pix_Y);
}
//********* Display All Black Clear ******************************
void ESP32_LCD_ILI9341_SPI::Display_Clear(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1){
  if( x0 >= _Max_Width_x ) x0 = _Max_Pix_X;
  if( y0 >= _Max_Width_y ) y0 = _Max_Pix_Y;
  if( x1 >= _Max_Width_x ) x1 = _Max_Pix_X;
  if( y1 >= _Max_Width_y ) y1 = _Max_Pix_Y;
  uint32_t Total_Pixels = (x1 - x0 + 1) * (y1 - y0 + 1) ;
  ESP32_LCD_ILI9341_SPI::Block_SPI_Fast_Write(x0, y0, x1, y1, 0, 0, 0, Total_Pixels);
}
//*********** LCD ILE9341 Block Pixel SPI Fast Write *****************
void ESP32_LCD_ILI9341_SPI::Block_SPI_Fast_Write(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t red, uint8_t green, uint8_t blue, uint32_t repeat){
  uint16_t ColorDot = (red << 11) | (green << 5) | blue;
  ESP32_LCD_ILI9341_SPI::XY_Range(x0, y0, x1, y1);
  ESP32_LCD_ILI9341_SPI::CommandWrite( 0x2C ); //LCD RAM write
  ESP32_LCD_ILI9341_SPI::spiWriteBlock(ColorDot, repeat);
}
//********* Display Color Pixel Block Fast Write *****************
void ESP32_LCD_ILI9341_SPI::spiWriteBlock(uint16_t color, uint32_t repeat) {
  uint16_t color16 = (color >> 8) | (color << 8);
  uint32_t color32 = color16 | color16 << 16;

  if (repeat > 15) {
    SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(_SPI_NUM), SPI_USR_MOSI_DBITLEN, 255,
                      SPI_USR_MOSI_DBITLEN_S);

    while (repeat > 15) {
      while (READ_PERI_REG(SPI_CMD_REG(_SPI_NUM)) & SPI_USR)
        ;
      for (uint32_t i = 0; i < 16; i++)
        WRITE_PERI_REG((SPI_W0_REG(_SPI_NUM) + (i << 2)), color32);
      SET_PERI_REG_MASK(SPI_CMD_REG(_SPI_NUM), SPI_USR);
      repeat -= 16;
    }
    while (READ_PERI_REG(SPI_CMD_REG(_SPI_NUM)) & SPI_USR)
      ;
  }

  if (repeat) {
    repeat = (repeat << 4) - 1;
    SET_PERI_REG_BITS(SPI_MOSI_DLEN_REG(_SPI_NUM), SPI_USR_MOSI_DBITLEN, repeat,
                      SPI_USR_MOSI_DBITLEN_S);
    for (uint32_t i = 0; i < 16; i++)
      WRITE_PERI_REG((SPI_W0_REG(_SPI_NUM) + (i << 2)), color32);
    SET_PERI_REG_MASK(SPI_CMD_REG(_SPI_NUM), SPI_USR);
    while (READ_PERI_REG(SPI_CMD_REG(_SPI_NUM)) & SPI_USR)
      ;
  }
}
//*********** 65k Color Pixel (Dot) Write ****************************
void ESP32_LCD_ILI9341_SPI::Draw_Pixel_65k_DotColor(uint16_t x0, uint16_t y0, uint16_t DotColor){
  ESP32_LCD_ILI9341_SPI::XY_Range(x0, y0, x0, y0);
  ESP32_LCD_ILI9341_SPI::CommandWrite( 0x2C ); //RAM write
  ESP32_LCD_ILI9341_SPI::DataWrite16( DotColor );
}
//*********** 65k Pixel RGB color Write ****************************
void ESP32_LCD_ILI9341_SPI::Draw_Pixel_65k_3Color(uint16_t x0, uint16_t y0, uint8_t red, uint8_t green, uint8_t blue){
  //red (0-31), green (0-63), blue (0-31)
  ESP32_LCD_ILI9341_SPI::XY_Range(x0, y0, x0, y0);

  uint16_t Dot = ((uint16_t)red << 11) | ((uint16_t)green << 5) | (uint16_t)blue;

  ESP32_LCD_ILI9341_SPI::CommandWrite( 0x2C ); //RAM write
  ESP32_LCD_ILI9341_SPI::DataWrite16( Dot );
}
//***************************************
void ESP32_LCD_ILI9341_SPI::Draw_Rectangle_Line(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t red, uint8_t green, uint8_t blue){
  ESP32_LCD_ILI9341_SPI::Draw_Horizontal_Line(x0, x1, y0, red, green, blue);
  ESP32_LCD_ILI9341_SPI::Draw_Horizontal_Line(x0, x1, y1, red, green, blue);
  ESP32_LCD_ILI9341_SPI::Draw_Vertical_Line(x0, y0, y1, red, green, blue);
  ESP32_LCD_ILI9341_SPI::Draw_Vertical_Line(x1, y0, y1, red, green, blue);
}
//***************************************
void ESP32_LCD_ILI9341_SPI::Draw_Horizontal_Line(int16_t x0, int16_t x1, int16_t y0, uint8_t red, uint8_t green, uint8_t blue){
  if(x1 < x0) SWAP(int16_t, x0, x1);
  uint32_t Width_x = x1 - x0 + 1;
  ESP32_LCD_ILI9341_SPI::Block_SPI_Fast_Write(x0, y0, x1, y0, red, green, blue, Width_x);
}
//***************************************
void ESP32_LCD_ILI9341_SPI::Draw_Vertical_Line(int16_t x0, int16_t y0, int16_t y1, uint8_t red, uint8_t green, uint8_t blue){
  if(y1 < y0) SWAP(int16_t, y0, y1);
  uint16_t Width_y = y1 - y0 + 1;
  ESP32_LCD_ILI9341_SPI::Block_SPI_Fast_Write(x0, y0, x0, y1, red, green, blue, Width_y);
}
//***************************************
void ESP32_LCD_ILI9341_SPI::Draw_Line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint8_t red, uint8_t green, uint8_t blue){
  int i;
  int16_t Y = 0, X = 0;
  int16_t length_x = x1 - x0;
  int16_t length_y = y1 - y0;

  uint16_t Dot = (red << 11) | (green << 5) | blue;

  if(abs(length_x) > abs(length_y)){
    float degY = ((float)length_y) / ((float)length_x);
    if(x0 < x1){
      for(i=x0; i<(x1+1); i++){
        Y = y0 + round((i-x0) * degY);
        ESP32_LCD_ILI9341_SPI::Draw_Pixel_65k_DotColor(i, Y, Dot);
      }
    }else{
      for(i=x0; i>=x1; i--){
        Y = y0 + round((i-x0) * degY);
        ESP32_LCD_ILI9341_SPI::Draw_Pixel_65k_DotColor(i, Y, Dot);
      }
    }
  }else{
    float degX = ((float)length_x) / ((float)length_y);

    if(y0 < y1){
      for(i=y0; i<(y1+1); i++){
        X = x0 + round((i-y0) * degX);
        ESP32_LCD_ILI9341_SPI::Draw_Pixel_65k_DotColor(X, i, Dot);
      }
    }else{
      for(i=y0; i>=y1; i--){
        X = x0 + round((i-y0) * degX);
        ESP32_LCD_ILI9341_SPI::Draw_Pixel_65k_DotColor(X, i, Dot);
      }
    }
  }
}
//***************************************
void ESP32_LCD_ILI9341_SPI::Draw_Rectangle_Fill(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t red, uint8_t green, uint8_t blue){
  uint16_t Width_x = x1 - x0 + 1;
  uint16_t Width_y = y1 - y0 + 1;
  uint32_t Total = Width_x * Width_y ;
  ESP32_LCD_ILI9341_SPI::Block_SPI_Fast_Write(x0, y0, x1, y1, red, green, blue, Total);
}
//***************************************
void ESP32_LCD_ILI9341_SPI::Draw_Circle_Line(uint16_t x0, uint16_t y0, uint16_t r, uint8_t red, uint8_t green, uint8_t blue){
  uint16_t x1, y1;
  float i;
  float deg = 1.0;
  if( r > 50 ) deg = 0.5;
  if( r > 110) deg = 0.25;

  uint16_t Dot = ((uint16_t)red << 11) | ((uint16_t)green << 5) | (uint16_t)blue;
  int16_t dummy_x = -1, dummy_y = -1;

  for(i=0; i<360; i=i+deg){
    x1 = round( (float)(x0 + (r * cos(radians(i)))) );
    y1 = round( (float)(y0 + (r * sin(radians(i)))) );

    if((dummy_x != x1) || (dummy_y != y1)){
      ESP32_LCD_ILI9341_SPI::Draw_Pixel_65k_DotColor(x1, y1, Dot);
      dummy_x = x1;
      dummy_y = y1;
    }
  }
}
//***************************************
void ESP32_LCD_ILI9341_SPI::Draw_Circle_Fill(uint16_t x0, uint16_t y0, uint16_t r, uint8_t red, uint8_t green, uint8_t blue){
  //red (0-31), green (0-63), blue (0-31)
  uint16_t x1, y1;
  float i;
  float deg = 1.0;
  //半径が大きくなると、角度の刻み方を細かくしないと、完全に塗りつぶせないので注意。
  if( r > 50 ) deg = 0.5;
  if( r > 110) deg = 0.25;

  int16_t dummy_x = -1, dummy_y = -1;
  for( i = 0; i < 360; i = i + deg ){
    x1 = round( (float)(x0 + (r * cos(radians(i)))) );
    y1 = round( (float)(y0 + (r * sin(radians(i)))) );
    if((dummy_x != x1) || (dummy_y != y1)){
      ESP32_LCD_ILI9341_SPI::Draw_Vertical_Line(x1, y0, y1, red, green, blue);
      dummy_x = x1;
      dummy_y = y1;
    }
  }
}
//********* LCD Display LED Brightness **************
void ESP32_LCD_ILI9341_SPI::Brightness(uint8_t brightness){
  uint8_t ledc_ch = 0;
  uint32_t valueMax = 255;
  uint32_t duty = (8191 / valueMax) * min(brightness, valueMax);
  ledcSetup(ledc_ch, 5000, 13);
  ledcAttachPin(_ledpin, 0);
  ledcWrite(ledc_ch, duty);
}
