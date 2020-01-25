#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "cmsis_os.h"
#include "control.h"

/*
 * control.c
 *
 *  Created on: 2019. márc. 10.
 *      Author: Benjami
 */

//-----------------------------------------------------------------------------
#define BITBAND_ACCESS(a, b)  *(volatile uint32_t*)(((uint32_t)&a & 0xF0000000) + 0x2000000 + (((uint32_t)&a & 0x000FFFFF) << 5) + (b << 2))

// portláb módok (PP: push-pull, OD: open drain, FF: input floating)
#define MODE_ANALOG_INPUT     0x0
#define MODE_PP_OUT_10MHZ     0x1
#define MODE_PP_OUT_2MHZ      0x2
#define MODE_PP_OUT_50MHZ     0x3
#define MODE_FF_DIGITAL_INPUT 0x4
#define MODE_OD_OUT_10MHZ     0x5
#define MODE_OD_OUT_2MHZ      0x6
#define MODE_OD_OUT_50MHZ     0x7
#define MODE_PU_DIGITAL_INPUT 0x8
#define MODE_PP_ALTER_10MHZ   0x9
#define MODE_PP_ALTER_2MHZ    0xA
#define MODE_PP_ALTER_50MHZ   0xB
#define MODE_RESERVED         0xC
#define MODE_OD_ALTER_10MHZ   0xD
#define MODE_OD_ALTER_2MHZ    0xE
#define MODE_OD_ALTER_50MHZ   0xF

#define GPIOX_PORT_(a, b)     GPIO ## a
#define GPIOX_PORT(a)         GPIOX_PORT_(a)

#define GPIOX_PIN_(a, b)      b
#define GPIOX_PIN(a)          GPIOX_PIN_(a)

#define GPIOX_MODE_(a,b,c)    ((GPIO_TypeDef*)(((c & 8) >> 1) + GPIO ## b ## _BASE))->CRL = (((GPIO_TypeDef*)(((c & 8) >> 1) + GPIO ## b ## _BASE))->CRL & ~(0xF << ((c & 7) << 2))) | (a << ((c & 7) << 2))
#define GPIOX_MODE(a, b)      GPIOX_MODE_(a, b)

#define GPIOX_ODR_(a, b)      BITBAND_ACCESS(GPIO ## a ->ODR, b)
#define GPIOX_ODR(a)          GPIOX_ODR_(a)

#define GPIOX_IDR_(a, b)      BITBAND_ACCESS(GPIO ## a ->IDR, b)
#define GPIOX_IDR(a)          GPIOX_IDR_(a)

#define GPIOX_LINE_(a, b)     EXTI_Line ## b
#define GPIOX_LINE(a)         GPIOX_LINE_(a)

#define GPIOX_PORTSRC_(a, b)  GPIO_PortSourceGPIO ## a
#define GPIOX_PORTSRC(a)      GPIOX_PORTSRC_(a)

#define GPIOX_PINSRC_(a, b)   GPIO_PinSource ## b
#define GPIOX_PINSRC(a)       GPIOX_PINSRC_(a)

#define GPIOX_CLOCK_(a, b)    RCC_APB2ENR_IOP ## a ## EN
#define GPIOX_CLOCK(a)        GPIOX_CLOCK_(a)

#define GPIOX_PORTNUM_A       1
#define GPIOX_PORTNUM_B       2
#define GPIOX_PORTNUM_C       3
#define GPIOX_PORTNUM_D       4
#define GPIOX_PORTNUM_E       5
#define GPIOX_PORTNUM_F       6
#define GPIOX_PORTNUM_G       7
#define GPIOX_PORTNUM_H       8
#define GPIOX_PORTNUM_I       9
#define GPIOX_PORTNUM_J       10
#define GPIOX_PORTNUM_K       11
#define GPIOX_PORTNUM_(a, b)  GPIOX_PORTNUM_ ## a
#define GPIOX_PORTNUM(a)      GPIOX_PORTNUM_(a)

#define GPIOX_PORTNAME_(a, b) a
#define GPIOX_PORTNAME(a)     GPIOX_PORTNAME_(a)
//-----------------------------------------------------------------------------

#if     ENCODER_TIMER == 1
#define TIMX                  TIM1
#define TIMX_CLOCK            RCC->APB1ENR |= RCC_APB1ENR_TIM1EN
#elif   ENCODER_TIMER == 2
#define TIMX                  TIM2
#define TIMX_CLOCK            RCC->APB1ENR |= RCC_APB1ENR_TIM2EN
#elif   ENCODER_TIMER == 3
#define TIMX                  TIM3
#define TIMX_CLOCK            RCC->APB1ENR |= RCC_APB1ENR_TIM3EN
#elif   ENCODER_TIMER == 4
#define TIMX                  TIM4
#define TIMX_CLOCK            RCC->APB1ENR |= RCC_APB1ENR_TIM4EN
#endif


extern  osMessageQId controlQueueHandle;
void    StartControllTask(void const * argument);
extern  TIM_HandleTypeDef htim4;

//-----------------------------------------------------------------------------
// Encoder kezelés, gomb és tekergetés
void StartControlTask(void const * argument)
{
  control_t ctrlmsg;     // az üzenet maga
  int32_t   e = 0, ep, r;// rotate számlálok
  uint32_t  t = 0;       // LCD_REFRESH üzenet küldésének számláloja
  uint32_t  tclick = 0;  // gombnyomás hosszak
  uint8_t   e_btn = 255; // gombnyomás elözö állapota (hogy induláskor elküldje az aktuális állapotát)
  uint8_t   dblclick = 0;// duplaclick számlálás

  RCC->APB2ENR |= (GPIOX_CLOCK(ENCODER_A) | GPIOX_CLOCK(ENCODER_B) | GPIOX_CLOCK(ENCODER_BT));
  GPIOX_MODE(MODE_PU_DIGITAL_INPUT, ENCODER_A);
  GPIOX_MODE(MODE_PU_DIGITAL_INPUT, ENCODER_B);
  GPIOX_MODE(MODE_PU_DIGITAL_INPUT, ENCODER_BT);
  GPIOX_ODR(ENCODER_A) = 1;
  GPIOX_ODR(ENCODER_B) = 1;
  GPIOX_ODR(ENCODER_BT) = 1;

  TIMX_CLOCK;
  TIMX->CCMR1 = (1 << TIM_CCMR1_CC1S_Pos) | (1 << TIM_CCMR1_CC2S_Pos);
  TIMX->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
  TIMX->SMCR = (3 << TIM_SMCR_SMS_Pos);
  TIMX->CR1 |= TIM_CR1_CEN;

  while(1)
  {
    if(GPIOX_IDR(ENCODER_BT))
    { // nincs lenyomva az encoder gombja
      if(e_btn != 0)
      { // eddig le volt nyomva -> most lett felengedve
        ctrlmsg.Control = ENCODER_BTN_UP;
        osMessagePut(controlQueueHandle, ctrlmsg.Data, 0);
        e_btn = 0;
        t = 0;

        if(tclick <= SHORTCLICK_MAX)
        {
          ctrlmsg.Control = ENCODER_BTN_SHORTCLICK;
          osMessagePut(controlQueueHandle, ctrlmsg.Data, 0);
          dblclick++;
          if(dblclick >= 3)
          {
            ctrlmsg.Control = ENCODER_BTN_DOUBLECLICK;
            osMessagePut(controlQueueHandle, ctrlmsg.Data, 0);
            dblclick = 0;
          }
        }
        else if(tclick >= LONGCLICK_MIN)
        {
          ctrlmsg.Control = ENCODER_BTN_LONGCLICK;
          osMessagePut(controlQueueHandle, ctrlmsg.Data, 0);
          dblclick = 0;
        }
        tclick = 0;
      }
    }
    else
    { // le van nyomva az encoder gombja
      if(e_btn != 1)
      { // eddig nem volt lenyomva -> most lett lenyomva
        ctrlmsg.Control = ENCODER_BTN_DOWN;
        osMessagePut(controlQueueHandle, ctrlmsg.Data, 0);
        e_btn = 1;
        t = 0;

        if(tclick <= SHORTCLICK_MAX)
          dblclick++;
        else
          dblclick = 0;
        tclick = 0;
      }
    }

    ep = e;
    e = TIMX->CNT >> 2;
    r = e - ep;
    if(r > 8191)
      r -= 16384;  // számlálo elöre átfordult
    if(r < -8192)
      r += 16384;  // számlálo hátra átfordult
    if(r != 0)
    {
      if(e_btn)
        ctrlmsg.Control = ENCODER_BTN_ROTATE;
      else
        ctrlmsg.Control = ENCODER_ROTATE;
      ctrlmsg.Value   = (int16_t)r;
      osMessagePut(controlQueueHandle, ctrlmsg.Data, 0);
      t = 0;
    }

    #if CONTROL_LCD_REFRESH_MS > 0
    if(t >= CONTROL_LCD_REFRESH_MS)
    {
      ctrlmsg.Control = LCD_REFRESH;
      osMessagePut(controlQueueHandle, ctrlmsg.Data, 0);
      t -= CONTROL_LCD_REFRESH_MS;
    }
    #endif

    osDelay(CONTROL_SAMPLETIME_MS);

    t+= CONTROL_SAMPLETIME_MS;
    if(tclick <= LONGCLICK_MIN)
      tclick++;
  }
}
