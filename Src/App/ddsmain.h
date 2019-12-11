/*
 * ddsmain.h
 *
 *  Created on: 2019. márc. 6.
 *      Author: Benjami
 */

#include  "control.h"

#ifndef __DDSMAIN_H
#define __DDSMAIN_H

// 0.1Hz a mértékegység!
#define  MINMODFREQ         10
#define  MAXMODFREQ     200000

#define  MAXMODDEPHT   1000000

// közös frekvencia paraméterek
#define  RASTERNUM          19
#define  RASTERTEXTLENGTH   16
typedef  struct
{
  int32_t   fRaster;
  uint8_t   fRasterText[RASTERTEXTLENGTH];
}RASTER;

extern const RASTER Rasters[RASTERNUM];

typedef struct
{
  uint16_t  x;
  uint16_t  y;
  uint16_t  TextColor;
  uint16_t  BackColor;
  uint16_t  SelectTextColor;
  uint16_t  SelectBackColor;
  uint16_t  ModifyTextColor;
  uint16_t  ModifyBackColor;
  void (*fp)();
  uint8_t   Menutext[24];
}MENUITEM;

#define DDS_COLORSET1     LCD_COLOR_GRAY, LCD_COLOR_BLACK, LCD_COLOR_YELLOW, LCD_COLOR_BLUE, LCD_COLOR_BLUE, LCD_COLOR_WHITE
#define DDS_COLORSET2     LCD_COLOR_GRAY, LCD_COLOR_BLACK, LCD_COLOR(255, 255, 128), LCD_COLOR(0, 64, 0), LCD_COLOR_BLUE, LCD_COLOR_WHITE

extern  uint8_t s[128];                 // átmeneti string sprintf használathoz

// program állapot
enum    STATUS_T {STATUS_STOP, STATUS_FGEN, STATUS_SWEEPGEN, STATUS_FMGEN};
extern  uint32_t Status;

extern  const int16_t sine1024q[];
#define getsine1024(v, fok1024) {     \
  if(fok1024 <= 256)                  \
    v = sine1024q[fok1024];           \
  else if(fok1024 <= 512)             \
    v = sine1024q[512 - fok1024];     \
  else if(fok1024 <= 768)             \
    v = 0 - sine1024q[fok1024 - 512]; \
  else                                \
    v = 0 - sine1024q[1024 - fok1024];}

extern  osMessageQId controlQueueHandle;// control üzenetek

void    StartDefaultTask(void const * argument);

#endif /* __DDSMAIN_H */
