/*
 * ddsdrv.c
 *
 *  Created on: 2019. m�rc. 10.
 *      Author: Benjami
 */

/*
DDS driver (AD9850, AD9851), Freertos sz�ks�ges

M�k�d�se:
  3 �zemmod lehets�ges
  - passziv �zemmod (ilyenkor a timer nem fut, a DDS ir�sa pedig a DdsSetFreq f�ggv�nnyel t�rt�nik)
  - aktiv �zemmod1 (timer megszakit�sban olvassa ki a ddsSamples cirkul�ris t�mbben tal�lhato adatokat,
                    l�p�sk�z = modul�lo frekvenci�tol f�gg, callback nem aktiv)
  - aktiv �zemmod2 (timer megszakit�sban olvassa ki a ddsSamples cirkul�ris t�mbben tal�lhato adatokat,
                    l�p�sk�z = 1, callback aktiv)

  Aktiv1 �zemmodban a timer megszakit�s int�zi a DDS ir�s�t. Pl. FM gener�tor �zemmodban a ddsSamples t�mb egy teljes hull�mot
   (pl. sinust) tartalmaz. Az hogy mennyit ugrunk DDS ir�sonk�nt el�re a ddsSamples t�bl�zatban,
   az a modul�lo frekvenci�tol f�gg. A t�bl�zat nem modosul ez�rt a callback f�ggv�ny cime = NULL.

  Aktiv2 �zemmodban a timer megszakit�s int�zi a DDS ir�s�t. A t�mb fel�n�l �s a v�g�n�l
    �zenetet k�ld a programsz�lnak (ddsSamplesSemHandle szemafor), jelezve hogy a sample t�mb �ppen nem haszn�lt
    fel�n lehet az adatokat aktualiz�lni. A t�mb fel�n�l j�rva ddsBufferHalf v�ltoz�ba 0-t tesz, ezzel jelzi,
    hogy a t�mb els� fele el lett k�ldve a DDS fel�, lehet az els� fel�t aktualiz�lni.
    A t�mb v�g�re �rve ddsBufferHalf v�ltoz�ba 1-t tesz, ezzel azt jelzi, hogy a t�mb m�sodik fel�t lehet aktualiz�lni.

  A StartSampleDdsTask f�ggv�ny a programsz�lat kell a freertos-al elind�tani, lehet�leg magas priorit�ssal.
  Indul�skor elv�gzi a DDS I/O l�bak inicializ�l�s�t �s a timer inicializ�l�s�t
  A v�gtelen ciklushoz el�rve semmi m�st nem csin�l, csak v�rakozik a timer megszak�t�s �zenet�re (ddsSamplesSemHandle szemafor)
  �s ha a szemafor szabadra v�lt megh�vja a sample memori�t felt�lt� callback_fp pointer �ltali f�ggv�nyt (csak akkor ha nem NULL)

  - A DdsSamplesFreq f�ggv�nnyel lehet a timer frekvenci�j�t be�llitani a kiv�nt mintav�teli frekvenci�ra
  - DdsSamplesOpen
*/

#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "ddsdrv.h"
#include "cmsis_os.h"

// param�terben a f�zishelyzet (0..31)
#define DDS_RUN(p)   ((p << 3) | 0b001)
#define DDS_STOP     0b100

#define DDS_IO_DELAY /*DDS_IO_Delay(10)*/ ;

//-----------------------------------------------------------------------------
#define BITBAND_ACCESS(a, b)  *(volatile uint32_t*)(((uint32_t)&a & 0xF0000000) + 0x2000000 + (((uint32_t)&a & 0x000FFFFF) << 5) + (b << 2))

// portl�b m�dok (PP: push-pull, OD: open drain, FF: input floating)
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
#define GPIOX_PORTNUM_J       9
#define GPIOX_PORTNUM_K       10
#define GPIOX_PORTNUM_L       11
#define GPIOX_PORTNUM_M       12
#define GPIOX_PORTNUM_(a, b)  GPIOX_PORTNUM_ ## a
#define GPIOX_PORTNUM(a)      GPIOX_PORTNUM_(a)

#define GPIOX_PORTNAME_(a, b) a
#define GPIOX_PORTNAME(a)     GPIOX_PORTNAME_(a)
//-----------------------------------------------------------------------------

#define DDS_RST_IMPULSE {\
  GPIOX_ODR(DDS_RST) = 1;\
  DDS_IO_Delay(128);     \
  GPIOX_ODR(DDS_RST) = 0;\
  DDS_IO_Delay(128);     }

#define DDS_WCK_IMPULSE {\
  GPIOX_ODR(DDS_WCK) = 1;\
  GPIOX_ODR(DDS_WCK) = 1;\
  GPIOX_ODR(DDS_WCK) = 0;}

#define DDS_UD_IMPULSE {\
  GPIOX_ODR(DDS_UD) = 1;\
  GPIOX_ODR(DDS_UD) = 1;\
  GPIOX_ODR(DDS_UD) = 0;}

/* DDS write */
#if  DDSMODE == 1    /* paralell mode */

/* optimalisation */
#if ((GPIOX_PORTNUM(DDS_D0) == GPIOX_PORTNUM(DDS_D1))\
  && (GPIOX_PORTNUM(DDS_D1) == GPIOX_PORTNUM(DDS_D2))\
  && (GPIOX_PORTNUM(DDS_D2) == GPIOX_PORTNUM(DDS_D3))\
  && (GPIOX_PORTNUM(DDS_D3) == GPIOX_PORTNUM(DDS_D4))\
  && (GPIOX_PORTNUM(DDS_D4) == GPIOX_PORTNUM(DDS_D5))\
  && (GPIOX_PORTNUM(DDS_D5) == GPIOX_PORTNUM(DDS_D6))\
  && (GPIOX_PORTNUM(DDS_D6) == GPIOX_PORTNUM(DDS_D7))\
  && (GPIOX_PIN(DDS_D0) + 1 == GPIOX_PIN(DDS_D1))\
  && (GPIOX_PIN(DDS_D1) + 1 == GPIOX_PIN(DDS_D2))\
  && (GPIOX_PIN(DDS_D2) + 1 == GPIOX_PIN(DDS_D3))\
  && (GPIOX_PIN(DDS_D3) + 1 == GPIOX_PIN(DDS_D4))\
  && (GPIOX_PIN(DDS_D4) + 1 == GPIOX_PIN(DDS_D5))\
  && (GPIOX_PIN(DDS_D5) + 1 == GPIOX_PIN(DDS_D6))\
  && (GPIOX_PIN(DDS_D6) + 1 == GPIOX_PIN(DDS_D7)))

#if GPIOX_PIN(DDS_D0) == 0
/* DDS datapins on 0..7 pin (exmpl. B0,B1,B2,B3,B4,B5,B6,B7) */
#define DDS_DATA_DIRWRITE  GPIOX_PORT(DDS_D0)->CRL = 0x33333333  // MODE_PP_OUT_50MHZ

#elif GPIOX_PIN(DDS_D0) == 8
/* DDS datapins on 8..15 pin (exmpl. B8,B9,B10,B11,B12,B13,B14,B15) */
#define DDS_DATA_DIRWRITE  GPIOX_PORT(DDS_D0)->CRH = 0x33333333  // MODE_PP_OUT_50MHZ

#endif

#define DDS_DATA_WRITEBYTE(d) {                                   \
  GPIOX_PORT(DDS_D0)->ODR = (GPIOX_PORT(DDS_D0)->ODR              \
                             & ~(0x00FF << GPIOX_PIN(DDS_D0)))    \
                             | (uint8_t)(d) << GPIOX_PIN(DDS_D0); \
  DDS_WCK_IMPULSE;                                                }

#endif /* D0..D7 optimalization */

#ifndef DDS_DATA_DIRWRITE
#define DDS_DATA_DIRWRITE {                                                          \
  GPIOX_MODE(MODE_PP_OUT_50MHZ, DDS_D0); GPIOX_MODE(MODE_PP_OUT_50MHZ, DDS_D1);\
  GPIOX_MODE(MODE_PP_OUT_50MHZ, DDS_D2); GPIOX_MODE(MODE_PP_OUT_50MHZ, DDS_D3);\
  GPIOX_MODE(MODE_PP_OUT_50MHZ, DDS_D4); GPIOX_MODE(MODE_PP_OUT_50MHZ, DDS_D5);\
  GPIOX_MODE(MODE_PP_OUT_50MHZ, DDS_D6); GPIOX_MODE(MODE_PP_OUT_50MHZ, DDS_D7);}
#endif

#ifndef DDS_DATA_WRITEBYTE
uint8_t  ddsdata;
#define DDS_DATA_WRITEBYTE(d) {                   \
  ddsdata = d;                                    \
  GPIOX_ODR(DDS_D0) = BITBAND_ACCESS(ddsdata, 0); \
  GPIOX_ODR(DDS_D1) = BITBAND_ACCESS(ddsdata, 1); \
  GPIOX_ODR(DDS_D2) = BITBAND_ACCESS(ddsdata, 2); \
  GPIOX_ODR(DDS_D3) = BITBAND_ACCESS(ddsdata, 3); \
  GPIOX_ODR(DDS_D4) = BITBAND_ACCESS(ddsdata, 4); \
  GPIOX_ODR(DDS_D5) = BITBAND_ACCESS(ddsdata, 5); \
  GPIOX_ODR(DDS_D6) = BITBAND_ACCESS(ddsdata, 6); \
  GPIOX_ODR(DDS_D7) = BITBAND_ACCESS(ddsdata, 7); }
#endif

/* DDS <- 40 bits (paralell mode) */
#define DDS_DATA_WRITE(d, p) {   \
  DDS_DATA_WRITEBYTE(DDS_RUN(p));\
  DDS_DATA_WRITEBYTE(d >> 24);   \
  DDS_DATA_WRITEBYTE(d >> 16);   \
  DDS_DATA_WRITEBYTE(d >> 8);    \
  DDS_DATA_WRITEBYTE(d);         }

#endif // #if  DDSMODE == 1

#if  DDSMODE == 2  /* serial mode */

uint8_t  ddsdata;
#define DDS_DATA_WRITEBYTE(d) {                   \
  ddsdata = d;                                    \
  GPIOX_ODR(DDS_D7) = BITBAND_ACCESS(ddsdata, 0); \
  DDS_WCK_IMPULSE;                                \
  GPIOX_ODR(DDS_D7) = BITBAND_ACCESS(ddsdata, 1); \
  DDS_WCK_IMPULSE;                                \
  GPIOX_ODR(DDS_D7) = BITBAND_ACCESS(ddsdata, 2); \
  DDS_WCK_IMPULSE;                                \
  GPIOX_ODR(DDS_D7) = BITBAND_ACCESS(ddsdata, 3); \
  DDS_WCK_IMPULSE;                                \
  GPIOX_ODR(DDS_D7) = BITBAND_ACCESS(ddsdata, 4); \
  DDS_WCK_IMPULSE;                                \
  GPIOX_ODR(DDS_D7) = BITBAND_ACCESS(ddsdata, 5); \
  DDS_WCK_IMPULSE;                                \
  GPIOX_ODR(DDS_D7) = BITBAND_ACCESS(ddsdata, 6); \
  DDS_WCK_IMPULSE;                                \
  GPIOX_ODR(DDS_D7) = BITBAND_ACCESS(ddsdata, 7); \
  DDS_WCK_IMPULSE;                                }

/* DDS <- 40 bits (serial mode) */
#define DDS_DATA_WRITE(d, p) {   \
  DDS_DATA_WRITEBYTE(d);         \
  DDS_DATA_WRITEBYTE(d >> 8);    \
  DDS_DATA_WRITEBYTE(d >> 16);   \
  DDS_DATA_WRITEBYTE(d >> 24);   \
  DDS_DATA_WRITEBYTE(DDS_RUN(p));}
#endif

void      StartSampleDdsTask(void const * argument);

extern    osSemaphoreId  ddsSamplesSemHandle;
extern    osThreadId     sampleTaskHandle;
extern    TIM_HandleTypeDef  htim1;

volatile  uint32_t  ddsBufferLength = 0;// mintav�teli adatok buffer hossza
volatile  uint32_t* ddsBufferP;         // mintav�teli adatok buffer cime
volatile  uint32_t  ddsBufferStep;      // ennyi mint�t ugrik el�re mintav�telenk�nt (csak ha callback_fp = NULL)
volatile  uint32_t  ddsBufferHalf;      // ha 0->ddsSamples 1.fele irhato, ha 1->2.fele irhato
void      (*callback_fp)(uint32_t bp);

uint32_t  ddsSamples[DDS_SAMPLESIZE];   // DDS mintav�teli buffer

//-----------------------------------------------------------------------------
#pragma GCC push_options
#pragma GCC optimize("O0")
void DDS_IO_Delay(volatile uint32_t c)
{
  while(c--);
}
#pragma GCC pop_options

//-----------------------------------------------------------------------------
// Mintav�telez�si adatok felt�lt�se (FM, A/D)
void StartSampleDdsTask(void const * argument)
{
  osSemaphoreWait(ddsSamplesSemHandle, 10);
  osDelay(20);

  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

  TIM1->CCMR1 = (6 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
  TIM1->CCER = TIM_CCER_CC1P | TIM_CCER_CC1E;
  TIM1->DMAR = 0x81 << TIM_DMAR_DMAB_Pos;
  TIM1->BDTR = TIM_BDTR_MOE;
  TIM1->DIER = TIM_DIER_UIE;

  RCC->APB2ENR |= GPIOX_CLOCK(DDS_UD) | GPIOX_CLOCK(DDS_WCK) | GPIOX_CLOCK(DDS_RST);
  GPIOX_ODR(DDS_UD) = 0;
  GPIOX_ODR(DDS_WCK) = 0;
  GPIOX_IDR(DDS_RST) = 0;
  GPIOX_MODE(MODE_PP_OUT_50MHZ, DDS_UD);
  GPIOX_MODE(MODE_PP_OUT_50MHZ, DDS_WCK);
  GPIOX_MODE(MODE_PP_OUT_50MHZ, DDS_RST);

  #if DDSMODE == 1 /* paralell mode */

  RCC->APB2ENR |= GPIOX_CLOCK(DDS_D0) | GPIOX_CLOCK(DDS_D1) | GPIOX_CLOCK(DDS_D2) | GPIOX_CLOCK(DDS_D3) |
                  GPIOX_CLOCK(DDS_D4) | GPIOX_CLOCK(DDS_D5) | GPIOX_CLOCK(DDS_D6) | GPIOX_CLOCK(DDS_D7);

  GPIOX_ODR(DDS_D0) = 0;      // D0 = 0
  GPIOX_ODR(DDS_D1) = 0;      // D1 = 0
  GPIOX_ODR(DDS_D2) = 0;      // D2 = 0
  DDS_DATA_DIRWRITE;

  DDS_RST_IMPULSE;
  DDS_UD_IMPULSE;
  #endif

  #if DDSMODE == 2 /* serial mode */

  #ifdef DDS_D0
  RCC->APB2ENR |= GPIOX_CLOCK(DDS_D0);
  GPIOX_ODR(DDS_D0) = 1;       // D0 = 1
  GPIOX_MODE(MODE_PP_OUT_50MHZ, DDS_D0);
  #endif
  #ifdef DDS_D1
  RCC->APB2ENR |= GPIOX_CLOCK(DDS_D1);
  GPIOX_ODR(DDS_D1) = 1;       // D1 = 1
  GPIOX_MODE(MODE_PP_OUT_50MHZ, DDS_D1);
  #endif
  #ifdef DDS_D2
  RCC->APB2ENR |= GPIOX_CLOCK(DDS_D2);
  GPIOX_ODR(DDS_D2) = 0;       // D2 = 0
  GPIOX_MODE(MODE_PP_OUT_50MHZ, DDS_D2);
  #endif

  RCC->APB2ENR |= GPIOX_CLOCK(DDS_D7);
  GPIOX_ODR(DDS_D7) = 0;       // D7 = 0
  GPIOX_MODE(MODE_PP_OUT_50MHZ, DDS_D7);

  DDS_RST_IMPULSE;

  DDS_DATAWRITEBYTE(0b101);
  DDS_UD_IMPULSE;
  DDS_DATAWRITEBYTE(0b001);
  DDS_UD_IMPULSE;
  #endif

  DdsStop();

  // GPIOX_MODE(MODE_PP_OUT_50MHZ, LED);

  while(1)
  {
    osSemaphoreWait(ddsSamplesSemHandle, osWaitForever);

    if(callback_fp != NULL)
      callback_fp(ddsBufferHalf);
  }
}

//-----------------------------------------------------------------------------
void DdsReset(void)
{
  DDS_RST_IMPULSE;
}

//-----------------------------------------------------------------------------
void DdsStop(void)
{
  TIM1->CR1 = 0;
  callback_fp = NULL;
  ddsBufferLength = 0;
  HAL_NVIC_DisableIRQ(TIM1_UP_IRQn);
  GPIOX_MODE(MODE_PP_OUT_50MHZ, DDS_UD);
  DDS_DATA_WRITE(0, DDS_RUN(0));
  DDS_UD_IMPULSE;
}

//-----------------------------------------------------------------------------
void DdsSetFreq(uint32_t ddsfreq)
{
  // printf("DdsSetFreq:%d\n", (unsigned int)ddsfreq);
  DDS_DATA_WRITE(ddsfreq, DDS_RUN(0));
  DDS_UD_IMPULSE;
}

/*-----------------------------------------------------------------------------
  Start DdsSamples (DDS sample timer enabled) */
void DdsSamplesStart(void)
{
  GPIOX_MODE(MODE_PP_ALTER_50MHZ, DDS_UD); // DDS kimenet Timer 1 ch1 out
  HAL_NVIC_SetPriority(TIM1_UP_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
  TIM1->CNT = 0;                          // sz�ml�lo = 0
  TIM1->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
}

/*-----------------------------------------------------------------------------
  Stop DdsSamples (DDS sample timer disabled) */
void DdsSamplesStop(void)
{
  TIM1->CR1 = 0;
  HAL_NVIC_DisableIRQ(TIM1_UP_IRQn);
  GPIOX_MODE(MODE_PP_OUT_50MHZ, DDS_UD);
  DDS_RST_IMPULSE;
}

//-----------------------------------------------------------------------------
void DdsSamplesOpen(uint32_t* sbuffer, uint32_t sbufferlength, uint32_t step, void (*cbfp)(uint32_t bp))
{
  ddsBufferP = sbuffer;
  ddsBufferLength = sbufferlength;
  if(sbufferlength >= step)
    ddsBufferStep = step;
  else
    ddsBufferStep = 1;
  callback_fp = cbfp;
}

//-----------------------------------------------------------------------------
void DdsSamplesClose(void)
{
  callback_fp = NULL;
  HAL_NVIC_DisableIRQ(TIM1_UP_IRQn);
  GPIOX_MODE(MODE_PP_OUT_50MHZ, DDS_UD);
  DDS_RST_IMPULSE;
}

//-----------------------------------------------------------------------------
void DdsSamplesStep(uint32_t step)
{
  if(ddsBufferLength >= step)
    ddsBufferStep = step;
  else
    ddsBufferStep = 1;
}

//-----------------------------------------------------------------------------
// DDS mintav�telez�si frekvenci�j�nak be�llit�sa (timer)
// (az itt be�llitott frekvencia szerint veszi ki a bufferb�l az �j adatokat)
void DdsSamplesFreq(uint32_t sfreq)
{
  uint32_t period;
  period = SystemCoreClock / sfreq;
  if(period >= 0x10000)
  { /* El�oszto is sz�ks�ges */
    TIM1->PSC = period >> 16;
    period = period / (TIM1->PSC + 1);
  }
  else
    TIM1->PSC = 0;            /* nincs el�oszto */
  TIM1->ARR = period;         /* autoreload */
  TIM1->CCR1 = period - 2;    /* 2 impulzus hossznyi lesz a triggerjel */
}

//-----------------------------------------------------------------------------
//
void TIM1_UP_IRQHandler(void)
{
  static volatile uint32_t bp = 0;               // buffer pointer sz�ml�lo
  uint32_t ddsSample;

  TIM1->SR = 0;

  ddsSample = *(ddsBufferP + bp);
  DDS_DATA_WRITE(ddsSample, DDS_RUN(0));

  // F�lk�sz illetve teljesen k�sz puffer eset�n lehet bele irni.
  if(callback_fp == NULL)
  { /* Nincs ujrairo f�ggv�ny, a pufferben a sinus t�bl�zat lett belerakva, abban lehet el�re l�pni */
    bp += ddsBufferStep; /* magasabb frekvencia eset�n t�bbet ugrunk a t�bl�zatban */
    if(bp >= ddsBufferLength)
    {
      bp -= ddsBufferLength;
    }
  }
  else
  {
    bp++;
    if(bp >= ddsBufferLength)
    {
      bp = 0;
      ddsBufferHalf = 1;
      osSemaphoreRelease(ddsSamplesSemHandle);
    }
    else if(bp == ddsBufferLength >> 1)
    {
      ddsBufferHalf = 0;
      osSemaphoreRelease(ddsSamplesSemHandle);
    }
  }
}
