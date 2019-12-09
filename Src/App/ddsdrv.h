/*
 * ddsdrv.h
 *
 *  Created on: 2019. márc. 10.
 *      Author: Benjami
 */

#ifndef __DDSDRV_H_
#define __DDSDRV_H_

/* ADxxxx type
   - 9850 : AD9850, 0..40MHz
   - 9851 : AD9851, 0..70MHz */
#define  ADTYPE   9851

/* DDS I/O mode
   - 1 : 8 bits paralell mode
   - 2 : serial mode */
#define  DDSMODE  1

#define  DDS_D0   A, 0
#define  DDS_D1   A, 1
#define  DDS_D2   A, 2
#define  DDS_D3   A, 3
#define  DDS_D4   A, 4
#define  DDS_D5   A, 5
#define  DDS_D6   A, 6
#define  DDS_D7   A, 7
#define  DDS_UD   A, 8
#define  DDS_WCK  B, 9
#define  DDS_RST  B, 10

// #define  LED      C, 13

#if      ADTYPE == 9850
#define  MAXFREQHZ    40000000    /* 40MHz */
#define  MAGICNUMBER  14757395259
#elif    ADTYPE == 9851
#define  MAXFREQHZ    70000000    /* 70MHz */
#define  MAGICNUMBER  10248191152
#endif

#define  DDS_SAMPLESIZE 1024
extern   uint32_t  ddsSamples[DDS_SAMPLESIZE];

/* Convert from freqvency to DDS number (from dHz to DDS 32 and DDS 64 bit number) */
#define  FreqDHzToDds(f)    (uint32_t)((uint64_t)(f) * MAGICNUMBER >> 32)
#define  FreqDHzToDds64(f)  ((uint64_t)(f) * MAGICNUMBER)

//-----------------------------------------------------------------------------
void     DdsReset(void);
void     DdsStop(void);
void     DdsSetFreq(uint32_t hz);
void     DdsSamplesStart(void);
void     DdsSamplesStop(void);
void     DdsSamplesOpen(uint32_t* sbuffer, uint32_t sbufferlength, uint32_t step, void (*cbfp)(uint32_t bp));
void     DdsSamplesClose(void);
void     DdsSamplesFreq(uint32_t sfreq);
void     DdsSamplesStep(uint32_t step);


#endif /* __DDSDRV_H_ */
