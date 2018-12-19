	/*!
* @file Adafruit_ST7789.cpp
*
* @mainpage Adafruit ST7789 TFT Displays
*
* @section intro_sec Introduction
*
* This is the documentation for Adafruit's ST7789 driver for the
* Arduino platform. 
*
* This library works with the Adafruit 2.8" Touch Shield V2 (SPI)
*    http://www.adafruit.com/products/1651
*
* Adafruit 2.4" TFT LCD with Touchscreen Breakout w/MicroSD Socket - ST7789
*    https://www.adafruit.com/product/2478
*
* 2.8" TFT LCD with Touchscreen Breakout Board w/MicroSD Socket - ST7789
*    https://www.adafruit.com/product/1770
*
* 2.2" 18-bit color TFT LCD display with microSD card breakout - ILI9340
*    https://www.adafruit.com/product/1770
*
* TFT FeatherWing - 2.4" 320x240 Touchscreen For All Feathers 
*    https://www.adafruit.com/product/3315
*
* These displays use SPI to communicate, 4 or 5 pins are required
* to interface (RST is optional).
*
* Adafruit invests time and resources providing this open source code,
* please support Adafruit and open-source hardware by purchasing
* products from Adafruit!
*
* @section dependencies Dependencies
*
* This library depends on <a href="https://github.com/adafruit/Adafruit_GFX">
* Adafruit_GFX</a> being present on your system. Please make sure you have
* installed the latest version before using this library.
*
* @section author Author
*
* Written by Limor "ladyada" Fried for Adafruit Industries.
*
* @section license License
*
* BSD license, all text here must be included in any redistribution.
*
*/

#include "Adafruit_ST7789.h"
#ifndef ARDUINO_STM32_FEATHER
  #include "pins_arduino.h"
  #ifndef RASPI
    #include "wiring_private.h"
  #endif
#endif
#include <limits.h>

#define MADCTL_MY  0x80     ///< Right to left
#define MADCTL_MX  0x40     ///< Bottom to top
#define MADCTL_MV  0x20     ///< Reverse Mode
#define MADCTL_ML  0x10     ///< LCD refresh Bottom to top
#define MADCTL_RGB 0x00     ///< Red-Green-Blue pixel order
#define MADCTL_BGR 0x08     ///< Blue-Green-Red pixel order
#define MADCTL_MH  0x04     ///< LCD refresh right to left

/*
 * Control Pins
 * */

#ifdef USE_FAST_PINIO
#define SPI_DC_HIGH()           *dcport |=  dcpinmask   ///< set DataCommand pin high
#define SPI_DC_LOW()            *dcport &= ~dcpinmask   ///< set DataCommand pin low
#define SPI_CS_HIGH()           *csport |= cspinmask    ///< set ChipSelect pin high
#define SPI_CS_LOW()            *csport &= ~cspinmask   ///< set ChipSelect pin high
#else
#define SPI_DC_HIGH()           digitalWrite(_dc, HIGH) ///< set DataCommand pin high
#define SPI_DC_LOW()            digitalWrite(_dc, LOW)  ///< set DataCommand pin low
#define SPI_CS_HIGH()           digitalWrite(_cs, HIGH) ///< set ChipSelect pin high
#define SPI_CS_LOW()            digitalWrite(_cs, LOW)  ///< set ChipSelect pin high
#endif

/*
 * Hardware SPI Macros
 * */

#ifndef ESP32
    #define SPI_OBJECT  SPI         ///< Default SPI hardware peripheral
#else
    #define SPI_OBJECT  _spi        ///< Default SPI hardware peripheral
#endif

#if defined (__AVR__) ||  defined(TEENSYDUINO) ||  defined(ARDUINO_ARCH_STM32F1)
    #define HSPI_SET_CLOCK() SPI_OBJECT.setClockDivider(SPI_CLOCK_DIV2);
#elif defined (__arm__)
    #define HSPI_SET_CLOCK() SPI_OBJECT.setClockDivider(11);
#elif defined(ESP8266) || defined(ESP32)
    #define HSPI_SET_CLOCK() SPI_OBJECT.setFrequency(_freq);
#elif defined(RASPI)
    #define HSPI_SET_CLOCK() SPI_OBJECT.setClock(_freq);
#elif defined(ARDUINO_ARCH_STM32F1)
    #define HSPI_SET_CLOCK() SPI_OBJECT.setClock(_freq);
#else
    #define HSPI_SET_CLOCK()   ///< Hardware SPI set clock frequency
#endif

#ifdef SPI_HAS_TRANSACTION
    #define HSPI_BEGIN_TRANSACTION() SPI_OBJECT.beginTransaction(SPISettings(_freq, MSBFIRST, SPI_MODE0))
    #define HSPI_END_TRANSACTION()   SPI_OBJECT.endTransaction()
#else
    #define HSPI_BEGIN_TRANSACTION() HSPI_SET_CLOCK(); SPI_OBJECT.setBitOrder(MSBFIRST); SPI_OBJECT.setDataMode(SPI_MODE0)        ///< Hardware SPI begin transaction
    #define HSPI_END_TRANSACTION()    ///< Hardware SPI end transaction
#endif

#ifdef ESP32
    #define SPI_HAS_WRITE_PIXELS
#endif
#if defined(ESP8266) || defined(ESP32)
    // Optimized SPI (ESP8266 and ESP32)
    #define HSPI_READ()              SPI_OBJECT.transfer(0)    ///< Hardware SPI read 8 bits
    #define HSPI_WRITE(b)            SPI_OBJECT.write(b)       ///< Hardware SPI write 8 bits
    #define HSPI_WRITE16(s)          SPI_OBJECT.write16(s)     ///< Hardware SPI write 16 bits
    #define HSPI_WRITE32(l)          SPI_OBJECT.write32(l)     ///< Hardware SPI write 32 bits
    #ifdef SPI_HAS_WRITE_PIXELS
        #define SPI_MAX_PIXELS_AT_ONCE  32
        #define HSPI_WRITE_PIXELS(c,l)   SPI_OBJECT.writePixels(c,l)
    #else
        #define HSPI_WRITE_PIXELS(c,l)   for(uint32_t i=0; i<((l)/2); i++){ SPI_WRITE16(((uint16_t*)(c))[i]); }
    #endif
#else
    // Standard Byte-by-Byte SPI

    #if defined (__AVR__) || defined(TEENSYDUINO)
static inline uint8_t _avr_spi_read(void) __attribute__((always_inline));
static inline uint8_t _avr_spi_read(void) {
    uint8_t r = 0;
    SPDR = r;
    while(!(SPSR & _BV(SPIF)));
    r = SPDR;
    return r;
}
        #define HSPI_WRITE(b)            {SPDR = (b); while(!(SPSR & _BV(SPIF)));}
        #define HSPI_READ()              _avr_spi_read()
    #else
        #define HSPI_WRITE(b)            SPI_OBJECT.transfer((uint8_t)(b))    ///< Hardware SPI write 8 bits
        #define HSPI_READ()              HSPI_WRITE(0)    ///< Hardware SPI read 8 bits
    #endif
    #define HSPI_WRITE16(s)          HSPI_WRITE((s) >> 8); HSPI_WRITE(s)  ///< Hardware SPI write 16 bits
    #define HSPI_WRITE32(l)          HSPI_WRITE((l) >> 24); HSPI_WRITE((l) >> 16); HSPI_WRITE((l) >> 8); HSPI_WRITE(l)          ///< Hardware SPI write 32 bits
    #define HSPI_WRITE_PIXELS(c,l)   for(uint32_t i=0; i<(l); i+=2){ HSPI_WRITE(((uint8_t*)(c))[i+1]); HSPI_WRITE(((uint8_t*)(c))[i]); }       ///< Hardware SPI write 'l' pixels 16-bits each
#endif

/*
 * Final SPI Macros
 * */
#if defined (ARDUINO_ARCH_ARC32) || defined (ARDUINO_MAXIM)
#define SPI_DEFAULT_FREQ         16000000
#elif defined (__AVR__) || defined(TEENSYDUINO)
#define SPI_DEFAULT_FREQ         8000000
#elif defined(ESP8266) || defined(ESP32)
#define SPI_DEFAULT_FREQ         40000000
#elif defined(RASPI)
#define SPI_DEFAULT_FREQ         80000000
#elif defined(ARDUINO_ARCH_STM32F1)
#define SPI_DEFAULT_FREQ         36000000
#else
#define SPI_DEFAULT_FREQ         24000000      ///< Default SPI data clock frequency
#endif

#if defined(ESP8266) || defined(ESP32)
#define SPI_BEGIN()             if(_sclk < 0){SPI_OBJECT.begin();}else{SPI_OBJECT.begin(_sclk,_miso,_mosi,-1);}          ///< SPI initialize
#else
#define SPI_BEGIN()             SPI_OBJECT.begin();          ///< SPI initialize
#endif
#define SPI_BEGIN_TRANSACTION() HSPI_BEGIN_TRANSACTION();    ///< SPI begin transaction
#define SPI_END_TRANSACTION()   HSPI_END_TRANSACTION();      ///< SPI end transaction
#define SPI_WRITE16(s)          HSPI_WRITE16(s);  ///< SPI write 16 bits
#define SPI_WRITE32(l)          HSPI_WRITE32(l);  ///< SPI write 32 bits
#define SPI_WRITE_PIXELS(c,l)   HSPI_WRITE_PIXELS(c,l);  ///< SPI write 'l' pixels of 16-bits each


static const uint8_t PROGMEM
  cmd_ST7789_240x240[] =  {                	// Init commands for 7789 screens
    19,                              	//  9 commands in list:
    //ST7789_SWRESET,   ST_CMD_DELAY,	 	//  1: Software reset, no args, w/delay
    //  255,                         		//    150 ms delay
    ST7789_SLPOUT ,   ST_CMD_DELAY, 	//  2: Out of sleep mode, no args, w/delay
      120,                          	//     255 = 500 ms delay
	ST7789_PWCTR1 	, 2,				//Power control
	  0xA4,0xA1,						//AVDD=6.8V,AVCL=-4.8V,VDDS=2.3V
	ST7789_GCCTL  	, 1,
	  0x35,								//VGH=13.26V,VGL=-10.43
	ST7789_VCMOSET	, 1,				//VCM control
	  0x32,								//Vcom=1.35V
	ST7789_VDVVRHEN , 1,			
	  0x01,							
	ST7789_VRHSET  	, 1,				
	  0x19,								//GVDD=4.8V
	ST7789_VDVSET  	, 1,				
	  0x20,								//VDV=0v

    ST7789_COLMOD 	, 1+ST_CMD_DELAY,	//  3: Set color mode, 1 arg + delay:
      0x55,                         	//     16-bit color
      10,                           	//     10 ms delay
    ST7789_MADCTL 	, 1,            	//  4: Mem access ctrl (directions), 1 arg:
      0x00,                        	    //     Row/col addr, bottom-top refresh
    ST7789_PORCTL  	, 5,             
      0x0C,0x0C,0x00,0x33,0x33,

	ST7789_FRMCTR2 	, 1,				//In Normal Mode	
	  0x0F,								//0x0F:60Hz
										
	ST7789_GAMMASET , 1,				//Gamma curve selected
	  0x01,								
	ST7789_GMCTRP1 	, 14,				//Set Gamma
	  0xD0,0x08,0x0E,0x09,0x09,0x05,0x31,0x33,0x48,0x17,0x14,0x15,0x31,0x34,						
	ST7789_GMCTRN1 , 14,				//Set Gamma
	  0xD0,0x08,0x0E,0x09,0x09,0x15,0x31,0x33,0x48,0x17,0x14,0x15,0x31,0x34,												
	  
    ST7789_CASET  , 4,              	//  5: Column addr set, 4 args, no delay:
      0x00,
      ST7789_240x240_XSTART,        	//     XSTART = 0
      (240+ST7789_240x240_XSTART)>>8,
      (240+ST7789_240x240_XSTART)&0xFF, //     XEND = 240
    ST7789_RASET  , 4,              	//  6: Row addr set, 4 args, no delay:
      0x00,
      ST7789_240x240_YSTART,            //     YSTART = 0
      (240+ST7789_240x240_YSTART)>>8,
      (240+ST7789_240x240_YSTART)&0xFF, //     YEND = 240

    ST7789_INVON  ,   ST_CMD_DELAY,  	//  7: hack
      10,
    ST7789_NORON  ,   ST_CMD_DELAY, 	//  8: Normal display on, no args, w/delay
      10,                           	//     10 ms delay
    ST7789_DISPON ,   ST_CMD_DELAY, 	//  9: Main screen turn on, no args, delay
	  255 								//     255 = max (500 ms) delay
  };                          	


/**************************************************************************/
/*!
    @brief  Instantiate Adafruit ST7789 driver with SPI
    @param    cs    Chip select pin #
    @param    dc    Data/Command pin #
    @param    rst   Reset pin # (optional, pass -1 if unused)
*/
/**************************************************************************/
Adafruit_ST7789::Adafruit_ST7789(int8_t cs, int8_t dc, int8_t mosi,
        int8_t sclk, int8_t rst, int8_t miso) : Adafruit_GFX(ST7789_TFTWIDTH, ST7789_TFTHEIGHT) {
    _cs   = cs;
    _dc   = dc;
    _rst  = rst;
#ifdef ESP32
    _sclk = sclk;
    _mosi  = mosi;
    _miso = miso;
#else
    _sclk  = -1;
    _mosi  = -1;
    _miso  = -1;
#endif
    _freq = 0;
#ifdef USE_FAST_PINIO
    if (_cs >= 0) {
    csport    = portOutputRegister(digitalPinToPort(_cs));
    cspinmask = digitalPinToBitMask(_cs);
	}
    dcport    = portOutputRegister(digitalPinToPort(_dc));
    dcpinmask = digitalPinToBitMask(_dc);
#endif
}

/**************************************************************************/
/*!
    @brief  Instantiate Adafruit ST7789 driver with SPI
    @param    cs    Chip select pin #
    @param    dc    Data/Command pin #
    @param    rst   Reset pin # (optional, pass -1 if unused)
*/
/**************************************************************************/
Adafruit_ST7789::Adafruit_ST7789(int8_t cs, int8_t dc, int8_t rst) : Adafruit_GFX(ST7789_TFTWIDTH, ST7789_TFTHEIGHT) {
    _cs   = cs;
    _dc   = dc;
    _rst  = rst;
    _sclk  = -1;
    _mosi  = -1;
    _miso  = -1;
    _freq = 0;
#ifdef USE_FAST_PINIO
    if (_cs >= 0) {
    csport    = portOutputRegister(digitalPinToPort(_cs));
    cspinmask = digitalPinToBitMask(_cs);
	}
    dcport    = portOutputRegister(digitalPinToPort(_dc));
    dcpinmask = digitalPinToBitMask(_dc);
#endif
}

/**************************************************************************/
/*!
    @brief  Companion code to the initiliazation tables. Reads and issues
            a series of LCD commands stored in PROGMEM byte array.
    @param  addr  Flash memory array with commands and data to send
*/
/**************************************************************************/
void Adafruit_ST7789::displayInit(const uint8_t *addr) {

  uint8_t  numCommands, numArgs;
  uint16_t ms;

  startWrite();
  numCommands = pgm_read_byte(addr++);   // Number of commands to follow
  while(numCommands--) {                 // For each command...

    writeCommand(pgm_read_byte(addr++)); // Read, issue command
    numArgs  = pgm_read_byte(addr++);    // Number of args to follow
    ms       = numArgs & ST_CMD_DELAY;   // If hibit set, delay follows args
    numArgs &= ~ST_CMD_DELAY;            // Mask out delay bit
    while(numArgs--) {                   // For each argument...
      spiWrite(pgm_read_byte(addr++));   // Read, issue argument
    }

    if(ms) {
      ms = pgm_read_byte(addr++); // Read post-command delay time (ms)
      if(ms == 255) ms = 500;     // If 255, delay for 500 ms
      delay(ms);
    }
  }
  endWrite();
}

/**************************************************************************/
/*!
    @brief   Initialize ST7789 chip
    Connects to the ST7789 over SPI and sends initialization procedure commands
    @param    freq  Desired SPI clock frequency
*/
/**************************************************************************/
#ifdef ESP32
void Adafruit_ST7789::begin(uint32_t freq, SPIClass &spi)
#else
void Adafruit_ST7789::begin(uint32_t freq)
#endif
{
#ifdef ESP32
    _spi = spi;
#endif
    if(!freq){
        freq = SPI_DEFAULT_FREQ;
    }
    _freq = freq;

    // toggle RST low to reset
    if (_rst >= 0) {
        pinMode(_rst, OUTPUT);
        digitalWrite(_rst, HIGH);
        delay(100);
        digitalWrite(_rst, LOW);
        delay(100);
        digitalWrite(_rst, HIGH);
        delay(200);
    }

    // Control Pins
    pinMode(_dc, OUTPUT);
    digitalWrite(_dc, LOW);
    if (_cs >= 0) {
	pinMode(_cs, OUTPUT);
	digitalWrite(_cs, HIGH);
	}

    // Hardware SPI
	SPI_BEGIN();
	
	displayInit(cmd_ST7789_240x240);
	setRotation(0);

    _width  = ST7789_TFTWIDTH;
    _height = ST7789_TFTHEIGHT;
}


void Adafruit_ST7789::softReset() {
	writeCommand(ST7789_SWRESET);   // 1: Software reset, no args, w/delay
    delay(150);
}


/**************************************************************************/
/*!
    @brief  Pass 8-bit (each) R,G,B, get back 16-bit packed color
            This function converts 8-8-8 RGB data to 16-bit 5-6-5
    @param    red   Red 8 bit color
    @param    green Green 8 bit color
    @param    blue  Blue 8 bit color
    @return   Unsigned 16-bit down-sampled color in 5-6-5 format
*/
/**************************************************************************/
uint16_t Adafruit_ST7789::color565(uint8_t red, uint8_t green, uint8_t blue) {
    return ((red & 0xF8) << 8) | ((green & 0xFC) << 3) | ((blue) >> 3);
}

/**************************************************************************/
/*!
    @brief   Set origin of (0,0) and orientation of TFT display
    @param   m  The index for rotation, from 0-3 inclusive
*/
/**************************************************************************/
void Adafruit_ST7789::setRotation(uint8_t m) {
    rotation = m % 4; // can't be higher than 3
    switch (rotation) {
        case 0:
            m = (0x00 | MADCTL_RGB);
//			m=0x00;
            _width  = ST7789_TFTWIDTH;
            _height = ST7789_TFTHEIGHT;
			scrollTo(0);
            break;
        case 1:
            m = (MADCTL_MX | MADCTL_MV | MADCTL_RGB);
//			m=0x60;
            _width  = ST7789_TFTHEIGHT;
            _height = ST7789_TFTWIDTH;
			scrollTo(0);
            break;
        case 2:
            m = (MADCTL_MX | MADCTL_MY | MADCTL_RGB);
//			m=0xC0;
            _width  = ST7789_TFTWIDTH;
            _height = ST7789_TFTHEIGHT;
			scrollTo(ST7789_240x240_YSTART);
            break;
        case 3:
            m = (MADCTL_MY | MADCTL_MV | MADCTL_RGB);
//			m=0xA0;
            _width  = ST7789_TFTHEIGHT;
            _height = ST7789_TFTWIDTH;
			scrollTo(ST7789_240x240_YSTART);
            break;
    }

/*	
   switch (cwj) {
      case 0x00:    //！正
        tft.scrollTo(240);
        break;
      case 0x20:    //逆时针 镜像
        tft.scrollTo(240);
        break;
      case 0x40:    //正 镜像
        tft.scrollTo(240);
        break;
      case 0x60:    //！顺时针
        tft.scrollTo(240);
        break;
      case 0x80:    //倒立 镜像
        tft.scrollTo(0);
        break;
      case 0xA0:    //！逆时针
        tft.scrollTo(0);
        break;
      case 0xC0:    //！倒立	
        tft.scrollTo(0);
        break;
      case 0xE0:    //顺时针 镜像
        tft.scrollTo(0);
        break;
    }
*/
    startWrite();
    writeCommand(ST7789_MADCTL);
    spiWrite(m);
    endWrite();
}

/**************************************************************************/
/*!
    @brief   Enable/Disable display color inversion
    @param   invert True to invert, False to have normal color
*/
/**************************************************************************/
void Adafruit_ST7789::invertDisplay(boolean invert) {
    startWrite();
    writeCommand(invert ? ST7789_INVON : ST7789_INVOFF);
    endWrite();
}

/**************************************************************************/
/*!
    @brief   Scroll display memory
    @param   y How many pixels to scroll display by
*/
/**************************************************************************/
void Adafruit_ST7789::scrollTo(uint16_t y) {
    startWrite();
    writeCommand(ST7789_VSCRSADD);
    SPI_WRITE16(y);
    endWrite();
}

/**************************************************************************/
/*!
    @brief   Set the "address window" - the rectangle we will write to RAM with the next chunk of SPI data writes. The ST7789 will automatically wrap the data as each row is filled
    @param   x  TFT memory 'x' origin
    @param   y  TFT memory 'y' origin
    @param   w  Width of rectangle
    @param   h  Height of rectangle
*/
/**************************************************************************/
void Adafruit_ST7789::setAddrWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
    uint32_t xa = ((uint32_t)x << 16) | (x+w-1);
    uint32_t ya = ((uint32_t)y << 16) | (y+h-1);
    writeCommand(ST7789_CASET); // Column addr set
    SPI_WRITE32(xa);
    writeCommand(ST7789_RASET); // Row addr set
    SPI_WRITE32(ya);
    writeCommand(ST7789_RAMWR); // write to RAM
}

/**************************************************************************/
/*!
    @brief   Blit 1 pixel of color without setting up SPI transaction
    @param   color 16-bits of 5-6-5 color data
/**************************************************************************/
void Adafruit_ST7789::pushColor(uint16_t color) {
    SPI_WRITE16(color);
}

/**************************************************************************/
/*!
    @brief   Blit 1 pixel of color without setting up SPI transaction
    @param   color 16-bits of 5-6-5 color data
*/
/**************************************************************************/
void Adafruit_ST7789::writePixel(uint16_t color){
    SPI_WRITE16(color);
}

/**************************************************************************/
/*!
    @brief   Blit 'len' pixels of color without setting up SPI transaction
    @param   colors Array of 16-bit 5-6-5 color data
    @param   len Number of 16-bit pixels in colors array
*/
/**************************************************************************/
void Adafruit_ST7789::writePixels(uint16_t * colors, uint32_t len){
    SPI_WRITE_PIXELS((uint8_t*)colors , len * 2);
}

/**************************************************************************/
/*!
    @brief   Blit 'len' pixels of a single color without setting up SPI transaction
    @param   color 16-bits of 5-6-5 color data
    @param   len Number of 16-bit pixels you want to write out with same color
*/
/**************************************************************************/
void Adafruit_ST7789::writeColor(uint16_t color, uint32_t len){
#ifdef SPI_HAS_WRITE_PIXELS
    if(_sclk >= 0){
        for (uint32_t t=0; t<len; t++){
            writePixel(color);
        }
        return;
    }
    static uint16_t temp[SPI_MAX_PIXELS_AT_ONCE];
    size_t blen = (len > SPI_MAX_PIXELS_AT_ONCE)?SPI_MAX_PIXELS_AT_ONCE:len;
    uint16_t tlen = 0;

    for (uint32_t t=0; t<blen; t++){
        temp[t] = color;
    }

    while(len){
        tlen = (len>blen)?blen:len;
        writePixels(temp, tlen);
        len -= tlen;
    }
#else
    uint8_t hi = color >> 8, lo = color;
 //AVR Optimization
    for (uint32_t t=len; t; t--){
        HSPI_WRITE(hi);
        HSPI_WRITE(lo);
    }
    return;
    for (uint32_t t=len; t; t--){
        spiWrite(hi);
        spiWrite(lo);
    }
#endif
}

/**************************************************************************/
/*!
   @brief  Draw a single pixel, DOES NOT set up SPI transaction
    @param    x  TFT X location
    @param    y  TFT Y location
    @param    color 16-bit 5-6-5 Color to draw with
*/
/**************************************************************************/
void Adafruit_ST7789::writePixel(int16_t x, int16_t y, uint16_t color) {
    if((x < 0) ||(x >= _width) || (y < 0) || (y >= _height)) return;
    setAddrWindow(x,y,1,1);
    writePixel(color);
}

/**************************************************************************/
/*!
   @brief  Fill a rectangle, DOES NOT set up SPI transaction
    @param    x  TFT X location begin
    @param    y  TFT Y location begin
    @param    w  Width of rectangle
    @param    h  Height of rectangle
    @param    color 16-bit 5-6-5 Color to fill with
*/
/**************************************************************************/
void Adafruit_ST7789::writeFillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color){
    if((x >= _width) || (y >= _height)) return;
    int16_t x2 = x + w - 1, y2 = y + h - 1;
    if((x2 < 0) || (y2 < 0)) return;

    // Clip left/top
    if(x < 0) {
        x = 0;
        w = x2 + 1;
    }
    if(y < 0) {
        y = 0;
        h = y2 + 1;
    }

    // Clip right/bottom
    if(x2 >= _width)  w = _width  - x;
    if(y2 >= _height) h = _height - y;

    int32_t len = (int32_t)w * h;
    setAddrWindow(x, y, w, h);
    writeColor(color, len);
}


/**************************************************************************/
/*!
   @brief  Draw a vertical line, DOES NOT set up SPI transaction
    @param    x  TFT X location begin
    @param    y  TFT Y location begin
    @param    l  Length of line in pixels
    @param    color 16-bit 5-6-5 Color to draw with
*/
/**************************************************************************/
void Adafruit_ST7789::writeFastVLine(int16_t x, int16_t y, int16_t l, uint16_t color){
    writeFillRect(x, y, 1, l, color);
}


/**************************************************************************/
/*!
   @brief  Draw a horizontal line, DOES NOT set up SPI transaction
    @param    x  TFT X location begin
    @param    y  TFT Y location begin
    @param    l  Length of line in pixels
    @param    color 16-bit 5-6-5 Color to draw with
*/
/**************************************************************************/
void Adafruit_ST7789::writeFastHLine(int16_t x, int16_t y, int16_t l, uint16_t color){
    writeFillRect(x, y, l, 1, color);
}

/**************************************************************************/
/*!
   @brief  Draw a single pixel, includes code for SPI transaction
    @param    x  TFT X location
    @param    y  TFT Y location
    @param    color 16-bit 5-6-5 Color to draw with
*/
/**************************************************************************/
void Adafruit_ST7789::drawPixel(int16_t x, int16_t y, uint16_t color){
    startWrite();
    writePixel(x, y, color);
    endWrite();
}

/**************************************************************************/
/*!
   @brief  Draw a vertical line, includes code for SPI transaction
    @param    x  TFT X location begin
    @param    y  TFT Y location begin
    @param    l  Length of line in pixels
    @param    color 16-bit 5-6-5 Color to draw with
*/
/**************************************************************************/
void Adafruit_ST7789::drawFastVLine(int16_t x, int16_t y,
        int16_t l, uint16_t color) {
    startWrite();
    writeFastVLine(x, y, l, color);
    endWrite();
}

/**************************************************************************/
/*!
   @brief  Draw a horizontal line, includes code for SPI transaction
    @param    x  TFT X location begin
    @param    y  TFT Y location begin
    @param    l  Length of line in pixels
    @param    color 16-bit 5-6-5 Color to draw with
*/
/**************************************************************************/
void Adafruit_ST7789::drawFastHLine(int16_t x, int16_t y,
        int16_t l, uint16_t color) {
    startWrite();
    writeFastHLine(x, y, l, color);
    endWrite();
}

/**************************************************************************/
/*!
   @brief  Fill a rectangle, includes code for SPI transaction
    @param    x  TFT X location begin
    @param    y  TFT Y location begin
    @param    w  Width of rectangle
    @param    h  Height of rectangle
    @param    color 16-bit 5-6-5 Color to fill with
*/
/**************************************************************************/
void Adafruit_ST7789::fillRect(int16_t x, int16_t y, int16_t w, int16_t h,
        uint16_t color) {
    startWrite();
    writeFillRect(x,y,w,h,color);
    endWrite();
}

/**************************************************************************/
/*!
   @brief  Draw RGB rectangle of data from RAM to a location on screen
   Adapted from https://github.com/PaulStoffregen/ST7789_t3
   by Marc MERLIN. See examples/pictureEmbed to use this.
   5/6/2017: function name and arguments have changed for compatibility
   with current GFX library and to avoid naming problems in prior
   implementation.  Formerly drawBitmap() with arguments in different order.

    @param    x  TFT X location begin
    @param    y  TFT Y location begin
    @param    pcolors Pointer to 16-bit color data
    @param    w  Width of pcolors rectangle
    @param    h  Height of pcolors rectangle
*/
/**************************************************************************/
void Adafruit_ST7789::drawRGBBitmap(int16_t x, int16_t y,
  uint16_t *pcolors, int16_t w, int16_t h) {

    int16_t x2, y2; // Lower-right coord
    if(( x             >= _width ) ||      // Off-edge right
       ( y             >= _height) ||      // " top
       ((x2 = (x+w-1)) <  0      ) ||      // " left
       ((y2 = (y+h-1)) <  0)     ) return; // " bottom

    int16_t bx1=0, by1=0, // Clipped top-left within bitmap
            saveW=w;      // Save original bitmap width value
    if(x < 0) { // Clip left
        w  +=  x;
        bx1 = -x;
        x   =  0;
    }
    if(y < 0) { // Clip top
        h  +=  y;
        by1 = -y;
        y   =  0;
    }
    if(x2 >= _width ) w = _width  - x; // Clip right
    if(y2 >= _height) h = _height - y; // Clip bottom

    pcolors += by1 * saveW + bx1; // Offset bitmap ptr to clipped top-left
    startWrite();
    setAddrWindow(x, y, w, h); // Clipped area
    while(h--) { // For each (clipped) scanline...
      writePixels(pcolors, w); // Push one (clipped) row
      pcolors += saveW; // Advance pointer by one full (unclipped) line
    }
    endWrite();
}


/**************************************************************************/
/*!
   @brief  Read 8 bits of data from ST7789 configuration memory. NOT from RAM!
           This is highly undocumented/supported, it's really a hack but kinda works?
    @param    command  The command register to read data from
    @param    index  The byte index into the command to read from
    @return   Unsigned 8-bit data read from ST7789 register
*/
/**************************************************************************/
uint8_t Adafruit_ST7789::readcommand8(uint8_t command, uint8_t index) {
    uint32_t freq = _freq;
    if(_freq > 24000000){
        _freq = 24000000;
    }
    startWrite();
    writeCommand(0xD9);  // woo sekret command?
    spiWrite(0x10 + index);
    writeCommand(command);
    uint8_t r = spiRead();
    endWrite();
    _freq = freq;
    return r;
}


/**************************************************************************/
/*!
   @brief  Begin SPI transaction, for hardware SPI
*/
/**************************************************************************/
void Adafruit_ST7789::startWrite(void){
    SPI_BEGIN_TRANSACTION();
    if (_cs >= 0) {
    SPI_CS_LOW();
	}
}

/**************************************************************************/
/*!
   @brief  End SPI transaction, for hardware SPI
*/
/**************************************************************************/
void Adafruit_ST7789::endWrite(void){
    if (_cs >= 0) {
    SPI_CS_HIGH();
	}
    SPI_END_TRANSACTION();
}

/**************************************************************************/
/*!
   @brief  Write 8-bit data to command/register (DataCommand line low).
   Does not set up SPI transaction.
   @param  cmd The command/register to transmit
*/
/**************************************************************************/
void Adafruit_ST7789::writeCommand(uint8_t cmd){
    SPI_DC_LOW();
    spiWrite(cmd);
    SPI_DC_HIGH();
}


/**************************************************************************/
/*!
   @brief  Read 8-bit data via hardware SPI. Does not set up SPI transaction.
   @returns One byte of data from SPI
*/
/**************************************************************************/
uint8_t Adafruit_ST7789::spiRead() {
    if(_miso < 0){
        return 0;
    }
    return HSPI_READ();
}

/**************************************************************************/
/*!
   @brief  Write 8-bit data via hardware SPI. Does not set up SPI transaction.
   @param  b Byte of data to write over SPI
*/
/**************************************************************************/
void Adafruit_ST7789::spiWrite(uint8_t b) {
    HSPI_WRITE(b);
    return;
}
