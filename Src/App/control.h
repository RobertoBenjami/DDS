/*
 * control.h
 *
 *  Created on: 2019. m�rc. 10.
 *      Author: Benjami
 */

#ifndef __CONTROL_H_
#define __CONTROL_H_

//-----------------------------------------------------------------------------
// Lcd kijelz� frissit�se ennyi ms-�nt t�rt�njen meg
#define CONTROL_LCD_REFRESH_MS      0

// Vez�rl�s lek�rdez�se ilyen s�r�s�ggel t�rt�njen meg (ms)
#define CONTROL_SAMPLETIME_MS      50

/* vez�rl�si parancsok */
enum CONTROL_TYPE {ENCODER_ROTATE,
                   ENCODER_BTN_ROTATE,
                   ENCODER_BTN_DOWN,
                   ENCODER_BTN_UP,
                   ENCODER_BTN_SHORTCLICK,
                   ENCODER_BTN_LONGCLICK,
                   ENCODER_BTN_DOUBLECLICK,
                   LCD_REFRESH};

/* gomb kattint�s hosszak [CONTROL_SAMPLETIME_MS] */
#define  SHORTCLICK_MAX 10
#define  LONGCLICK_MIN  20

#define  ENCODER_A    B, 6
#define  ENCODER_B    B, 7
#define  ENCODER_BT   B, 8
#define  ENCODER_TIMER   4
//-----------------------------------------------------------------------------

typedef union
{
  struct
  {
    uint16_t     Control; /* vez�rl�s tipusa (CONTROL_TYPE) */
    int16_t      Value;   /* adat */
  };
  uint32_t   Data;
}control_t;

#endif /* __CONTROL_H_ */
