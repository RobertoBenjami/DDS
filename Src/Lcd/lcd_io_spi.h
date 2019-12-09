  /*
 * SPI LCD driver STM32F1
 * k�szit�: Roberto Benjami
 * verzio:  2019.06
*/

//=============================================================================
/* SPI kiv�laszt�sa (0, 1, 2 3)
   - 0: szoftveres SPI driver (a l�bhozz�rendel�s szabadon kiv�laszthato)
   - 1..3: hardver SPI driver (az LCD_SCK, LCD_MOSI, LCD_MISO l�bak k�t�ttek) */
#define LCD_SPI           2

/* SPI �zemmod
   - 0: csak SPI TX (csak irni lehet a kijelz�t, LCD_MISO l�bat nem sz�ks�ges megadni, nem lesz haszn�latban)
   - 1: half duplex (LCD_MOSI l�b k�t ir�nyban lesz m�k�dtetve, LCD_MISO l�b nem lesz haszn�lva)
   - 2: full duplex (SPI TX: LCD_MOSI, SPI RX: LCD_MISO) */
#define LCD_SPI_MODE      1

/* SPI sebess�ge
   - szoftver SPI: 0..
     - 0: semmi v�rakoz�s nem lesz
     - 1: GPIOX_ODR(LCD_SCK) = 0 id�nyi v�rakoz�s
     - 2..: LCD_IO_Delay(LCD_SPI_SPD - 2)
   - hardver SPI: 0..7 oszto: fPCLK/oszto, 0=/2, 1=/4, 2=/8, 3=/16, 4=/32, 5=/64, 6=/128, 7=/256 */
#define LCD_SPI_SPD       1
/* Megadhato az olvas�shoz tartozo orajel (ha azonos vagy nincs megadva akkor nem v�lt sebess�get olvas�skor) */
#define LCD_SPI_SPD_READ  3

/* Lcd vez�rl� l�bak hozz�rendel�se ((A..M, 0..15)
   - LCD_RST megad�sa nem k�telez� (X, 0)
   - LCD_MISO megad�sa csak full duplex (LCD_SPI_MODE 2) �zemmodban sz�ks�ges
   - hardver SPI eset�n az SCK, MOSI, MISO l�bak hozz�rendel�se k�t�tt */
#define LCD_RST           B, 11
#define LCD_RS            B, 14


#define LCD_CS            B, 12
#define LCD_SCK           B, 13
#define LCD_MOSI          B, 15
#define LCD_MISO          X, 0

/* H�tt�rvil�git�s vez�rl�s
   - BL: A..M, 0..15 (ha nem haszn�ljuk, akkor rendelj�k hozz� az X, 0 �rt�ket)
   - BL_ON: 0 vagy 1, a bekapcsolt �llapothoz tartoz� logikai szint */
#define LCD_BL            X, 0   // ha nem akarjuk haszn�lni X, 0 -t adjunk
#define LCD_BLON          0

/* Adatir�ny v�lt�skor (OUT->IN) van olyan kijelz�, amelyik extra �rajele(ke)t k�r az SCK l�bra
   - 0.. (ST7735: 1,  ILI9341: 0) */
#define LCD_SCK_EXTRACLK  1

/* DMA be�llit�sok
   - 0..2: 0 = nincs DMA, 1 = DMA1, 2 = DMA2 (DMA request mapping)
   - 0..7: DMA csatorna (DMA request mapping)
   - 1..3: DMA priorit�s (0=low..3=very high) */
#define LCD_DMA_TX        1, 5, 0
#define LCD_DMA_RX        1, 4, 0

/* DMA RX buffer [byte] (csak a ...24to16 f�ggv�nyek eset�ben lesz haszn�latban)
   (2 eg�sz sz�mu hatv�nya legyen: 16, 32, 64, 128, 256, 512, 1024...  32768) */
#define LCD_DMA_RX_BUFSIZE 128

/* DMA RX buffer helye
   - 0: stack
   - 1: static buffer */
#define LCD_DMA_RX_BUFMODE 0
