/*
 * fgen.c
 *
 *  Created on: 2019. márc. 15.
 *      Author: Benjami
 */

#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "ddsmain.h"
#include "ddsdrv.h"
#include "control.h"
#include "stm32_adafruit_lcd.h"

#define FGENMENUSIZE    4
const MENUITEM FGenMenu[FGENMENUSIZE] =
{
  {3,  5, DDS_COLORSET1, 0, "AD9851"},
  {0, 54, DDS_COLORSET1, 0, "Freq"},
  {0, 93, DDS_COLORSET2, 0, "Raster"},
  {3, 25, DDS_COLORSET1, 0, "Freq generator"}
};

struct TF
{
  int32_t fgFreq_dhz;   // tized Hz-ben beállított frekvencia (pl. 1kHz -> 10000)
  int32_t fgRaster;     // 0 = tized Hz, 1 = 1Hz ... 18 = 1MHz
}tf = {10000, 9};

//============================================================================
void LcdStringFixSize(uint8_t * ts, uint32_t size)
{
  uint32_t i = strlen((char *)ts);
  if(size <= i)
    return;
  while(i < size)
  {
    ts[i] = ' ';
    i++;
  }
  ts[i] = 0;
}

//============================================================================
/* Create string from frekvency
   - input
       f_dhz : frequency [dHz] (dHz = 1/10 Hz)
   - output
       ts* : frequency in string formate
 */
void LcdPrintFreq(uint8_t * ts, uint32_t f_dhz)
{
  if(f_dhz == 0)
  {
    sprintf((char *)ts, "0 Hz");
  }
  else if((f_dhz % 10000000) == 0)
  { // kerek MHz
    sprintf((char *)ts, "%d MHz", (int)f_dhz / 10000000);
  }
  else if((f_dhz % 10000) == 0)
  { // kerek kHz
    sprintf((char *)ts, "%d kHz", (int)f_dhz / 10000);
  }
  else if((f_dhz % 10) == 0)
  { // kerek Hz
    sprintf((char *)ts, "%d Hz", (int)f_dhz / 10);
  }
  else
  {
    sprintf((char *)ts, "%d.%d Hz", (int)f_dhz / 10, (int)f_dhz % 10);
  }
}

//============================================================================
void LcdFStart(void)
{
  #define XSTART  1
  #define XSIZE   BSP_LCD_GetXSize() - 1 - (2 * XSTART)

  BSP_LCD_Clear(LCD_COLOR(16, 16, 16));

  BSP_LCD_SetTextColor(FGenMenu[1].SelectBackColor);
  BSP_LCD_FillRect(XSTART, FGenMenu[1].y - 2, XSIZE, 36);
  //BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  //BSP_LCD_DrawRect(XSTART, FGenMenu[1].y - 2, XSIZE, 36);
  BSP_LCD_SetTextColor(FGenMenu[1].SelectTextColor);
  BSP_LCD_SetBackColor(FGenMenu[1].SelectBackColor);
  BSP_LCD_DisplayStringAt(FGenMenu[1].x + 5, FGenMenu[1].y + 2, (uint8_t *)FGenMenu[1].Menutext, LEFT_MODE);

  BSP_LCD_SetTextColor(FGenMenu[2].SelectBackColor);
  BSP_LCD_FillRect(XSTART, FGenMenu[2].y - 2, XSIZE, 36);
  //BSP_LCD_SetTextColor(LCD_COLOR(64, 64, 64));
  //BSP_LCD_DrawRect(XSTART, FGenMenu[2].y - 2, XSIZE, 36);
  BSP_LCD_SetTextColor(FGenMenu[2].SelectTextColor);
  BSP_LCD_SetBackColor(FGenMenu[2].SelectBackColor);
  BSP_LCD_DisplayStringAt(FGenMenu[2].x + 5, FGenMenu[2].y + 2, (uint8_t *)FGenMenu[2].Menutext, LEFT_MODE);

  BSP_LCD_SetFont(&Font16);
  BSP_LCD_SetTextColor(LCD_COLOR(255, 178, 68));
  BSP_LCD_SetBackColor(LCD_COLOR(16, 16, 16));
  BSP_LCD_DisplayStringAt(FGenMenu[0].x, FGenMenu[0].y, (uint8_t *)FGenMenu[0].Menutext, CENTER_MODE);
  BSP_LCD_DisplayStringAt(FGenMenu[3].x, FGenMenu[3].y, (uint8_t *)FGenMenu[3].Menutext, CENTER_MODE);
}

//============================================================================
void LcdFGen(void)
{
  /* Frekvencia */
  BSP_LCD_SetTextColor(FGenMenu[1].SelectTextColor);
  BSP_LCD_SetBackColor(FGenMenu[1].SelectBackColor);
  LcdPrintFreq(&s[0], tf.fgFreq_dhz);
  LcdStringFixSize(&s[0], 13);
  BSP_LCD_DisplayStringAt(FGenMenu[1].x + 8, FGenMenu[1].y + 16, s, LEFT_MODE);

  /* Raster */
  BSP_LCD_SetTextColor(FGenMenu[2].SelectTextColor);
  BSP_LCD_SetBackColor(FGenMenu[2].SelectBackColor);
  sprintf((char *)&s, "%s", Rasters[tf.fgRaster].fRasterText);
  LcdStringFixSize(&s[0], 13);
  BSP_LCD_DisplayStringAt(FGenMenu[2].x + 8, FGenMenu[2].y + 16, s, LEFT_MODE);
}

//-----------------------------------------------------------------------------
void FGen(void)
{
  static int32_t prefgFreq_dhz = 0;    /* tized Hz-ben beállított frekvencia elözö érték */

  DdsReset();
  LcdFStart();
  BSP_LCD_SetFont(&Font16);
  LcdFGen();
  DdsSetFreq(FreqDHzToDds(tf.fgFreq_dhz));

  osEvent   ctrlevent;
  control_t ctrlmsg;   // osEvent helyett

  Status = STATUS_FGEN;
  while(Status == STATUS_FGEN)
  {
    ctrlevent = osMessageGet(controlQueueHandle, osWaitForever);
    ctrlmsg.Data = ctrlevent.value.v;

    if(ctrlmsg.Control == ENCODER_BTN_DOWN)
    {
      BSP_LCD_SetTextColor(LCD_COLOR(64, 64, 64));
      BSP_LCD_DrawRect(XSTART, FGenMenu[1].y - 2, XSIZE, 36);
      BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
      BSP_LCD_DrawRect(XSTART, FGenMenu[2].y - 2, XSIZE, 36);
    }

    else if(ctrlmsg.Control == ENCODER_BTN_UP)
    { /* */
      BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
      BSP_LCD_DrawRect(XSTART, FGenMenu[1].y - 2, XSIZE, 36);
      BSP_LCD_SetTextColor(LCD_COLOR(64, 64, 64));
      BSP_LCD_DrawRect(XSTART, FGenMenu[2].y - 2, XSIZE, 36);
    }

    else if(ctrlmsg.Control == ENCODER_ROTATE)
    { /* Encoder forgatás (frekvencia állitás) */
      tf.fgFreq_dhz = (tf.fgFreq_dhz / Rasters[tf.fgRaster].fRaster) * Rasters[tf.fgRaster].fRaster;
      tf.fgFreq_dhz += ctrlmsg.Value * Rasters[tf.fgRaster].fRaster;
      if(tf.fgFreq_dhz < 0)
        tf.fgFreq_dhz = 0;
      else if(tf.fgFreq_dhz > (MAXFREQHZ * 10))
        tf.fgFreq_dhz = (MAXFREQHZ * 10);
      LcdFGen();
    }

    else if(ctrlmsg.Control == ENCODER_BTN_ROTATE)
    { /* Benyomott encoder forgatás (raszter állitás) */
      tf.fgRaster += ctrlmsg.Value;
      if(tf.fgRaster < 0)
        tf.fgRaster = 0;
      else if(tf.fgRaster >= RASTERNUM -1)
        tf.fgRaster = RASTERNUM -1;
      LcdFGen();
    }

    #if 0
    else if(ctrlmsg.Control == ENCODER_BTN_DOUBLECLICK)
    {
      Status = STATUS_STOP;
    }
    #endif

    if(prefgFreq_dhz != tf.fgFreq_dhz)
    {
      DdsSetFreq(FreqDHzToDds(tf.fgFreq_dhz));
      prefgFreq_dhz = tf.fgFreq_dhz;
    }

  } /* while(Status == STATUS_FGEN) */
  // DdsStop();
  // DdsReset();
  // BSP_LCD_SetFont(&Font12);
}

