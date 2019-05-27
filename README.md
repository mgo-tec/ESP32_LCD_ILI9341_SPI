# ESP32_LCD_ILI9341_SPI
This is Arduino core for the ESP32 library ( SPI ).  
Beta ver 1.26  
  
Use SPI.h library  
  
# Change log
(1.26)  
- Fixed LEDC related use in Brightness function.  
  
(1.25)  
- Class argument was added so that background color of 8 x 16 or 16 x 16 bit map font display can be specified.  
- he Draw_Pixel_65k_DotColor_sd function was added because the Draw_Pixel_65k_DotColor function could not be used with the microSD card.  
- In addition, since the graphic drawing function could not be used with the SD card, it was fixed.  
  
(1.23)  
Added Disp_Rotation function.  
It corresponds to the vertical installation of the display.  
Other minor fixes.  
  
(1.2)  
Added Scrolle_Inc_HVsizeUp_8x16_Font_DisplayOut function.  
This is a function to read font character by character and scroll (to save memory).  
  
(1.1)  
Corrected the argument uint8_t txt_length to uint16_t txt_length.      
  
# 【更新履歴】(Japanese)  
(1.26)  
- Brightness関数で使っている LEDC 関連を修正しました。  
  
(1.25)  
- 8x16 又は 16x16ビットマップフォント表示の背景色を指定できるように、クラスの引数を追加しました。  
- Draw_Pixel_65k_DotColor 関数が microSD カードと併用できなかったので、Draw_Pixel_65k_DotColor_sd 関数を追加しました。  
- その他、グラフィック描画関数が SD カードと併用できなかったので、修正しました。  
  
(1.23)  
Disp_Rotation 関数を追加。  
ディスプレイの縦置きに対応しました。  
その他、軽微な修正。  
  
(1.2)  
Scrolle_Inc_HVsizeUp_8x16_Font_DisplayOut関数を追加。  
これはフォントを１文字づつ読み込んでスクロールする関数です（メモリ節約のため、）。  
  
(1.1)  
引数 uint8_t txt_length を uint16_t txt_length に修正。  
  
The MIT License (MIT)  
  
MITライセンス  
Copyright (c) 2018 Mgo-tec  
  
My Blog:  
https://www.mgo-tec.com  