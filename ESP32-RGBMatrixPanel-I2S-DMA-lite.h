#ifndef _ESP32_RGB_64_32_MATRIX_PANEL_I2S_DMA
#define _ESP32_RGB_64_32_MATRIX_PANEL_I2S_DMA

/***************************************************************************************/
/* COMPILE-TIME OPTIONS - CONFIGURE AS DESIRED                                         */
/***************************************************************************************/

/* Enable serial debugging of the library, to see how memory is allocated etc. */
#define SERIAL_DEBUG 1

/* Use GFX_Root (https://github.com/mrfaptastic/GFX_Root) instead of 
 * Adafruit_GFX library. No real benefit unless you don't want Bus_IO library. 
 */
//#define USE_GFX_ROOT 1


#ifndef MATRIX_HEIGHT
	#define MATRIX_HEIGHT               32 
#endif

#ifndef MATRIX_WIDTH
	#define MATRIX_WIDTH                64
#endif

#ifndef PIXEL_COLOR_DEPTH_BITS
	#define PIXEL_COLOR_DEPTH_BITS      8   // 8bit per RGB color = 24 bit/per pixel, reduce to save RAM
#endif


/* ESP32 Default Pin definition. You can change this, but best if you keep it as is and provide custom pin mappings 
 * as part of the begin(...) function.
 */
#define R1_PIN_DEFAULT  25
#define G1_PIN_DEFAULT  26
#define B1_PIN_DEFAULT  27
#define R2_PIN_DEFAULT  14
#define G2_PIN_DEFAULT  12
#define B2_PIN_DEFAULT  13

#define A_PIN_DEFAULT   23
#define B_PIN_DEFAULT   19
#define C_PIN_DEFAULT   5
#define D_PIN_DEFAULT   17
#define E_PIN_DEFAULT   -1 // IMPORTANT: Change to a valid pin if using a 64x64px panel.
          
#define LAT_PIN_DEFAULT 4
#define OE_PIN_DEFAULT  15
#define CLK_PIN_DEFAULT 16

// Interesting Fact: We end up using a uint16_t to send data in parallel to the HUB75... but 
//                   given we only map to 14 physical output wires/bits, we waste 2 bits.

/***************************************************************************************/
/* Do not change.                                                                      */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "esp_heap_caps.h"
#include "esp32_i2s_parallel.h"

#ifdef USE_GFX_ROOT
	#include "GFX.h" // Adafruit GFX core class -> https://github.com/mrfaptastic/GFX_Root
#else	
	#include "Adafruit_GFX.h" // Adafruit class with all the other stuff
#endif	

/***************************************************************************************/
/* Do not change.                                                                      */
#define I2S_DEVICE I2S1

// Panel Upper half RGB (numbering according to order in DMA gpio_bus configuration)
#define BIT_R1  (1<<0)   
#define BIT_G1  (1<<1)   
#define BIT_B1  (1<<2)   

// Panel Lower half RGB
#define BITS_RGB2_OFFSET 3 // Start point of RGB_X2 bits
#define BIT_R2  (1<<3)   
#define BIT_G2  (1<<4)   
#define BIT_B2  (1<<5)   

// Panel GPIO Pin Addresses (A, B, C, D etc..)
#define BIT_A (1<<9)    
#define BIT_B (1<<10)    
#define BIT_C (1<<11)   
#define BIT_D (1<<12)   
#define BIT_E (1<<13)  
 // Panel Control Signals
#define BIT_LAT (1<<14) 
#define BIT_OE  (1<<15) 

// RGB Panel Constants / Calculated Values
#define COLOR_CHANNELS_PER_PIXEL 3 
#define PIXELS_PER_ROW ((MATRIX_WIDTH * MATRIX_HEIGHT) / MATRIX_HEIGHT) // = 64
#define ROWS_PER_FRAME (MATRIX_HEIGHT/MATRIX_ROWS_IN_PARALLEL) //  = 16
#define MATRIX_ROWS_IN_PARALLEL     2   // Don't change this unless you know what you're doing


/***************************************************************************************/
/* Keep this as is. Do not change.                                                     */
#define ESP32_I2S_DMA_MODE          I2S_PARALLEL_BITS_16    // Pump 16 bits out in parallel
#define ESP32_I2S_DMA_STORAGE_TYPE  uint16_t                // one uint16_t at a time.
//#define ESP32_I2S_CLOCK_SPEED     (20000000UL)            // @ 20Mhz
#define ESP32_I2S_CLOCK_SPEED       (10000000UL)  // @ 10Mhz
#define CLKS_DURING_LATCH            0   // Not used. 
/***************************************************************************************/            

/* rowBitStruct
 * Note: sizeof(data) must be multiple of 32 bits, as ESP32 DMA linked list buffer address pointer 
 *       must be word-aligned.
 */
struct rowBitStruct {
    ESP32_I2S_DMA_STORAGE_TYPE data[PIXELS_PER_ROW]; 
    // This evaluates to just data[64] really.. an array of 64 uint16_t's
};

/* rowColorDepthStruct
 * Duplicates of row bit structure, but for each color 'depth'ness. 
 */
struct rowColorDepthStruct {
    rowBitStruct rowbits[PIXEL_COLOR_DEPTH_BITS];
};

/* frameStruct
 * Note: This 'frameStruct' will contain ALL the data for a full-frame as BOTH 2x16-row frames are
 *       are contained in parallel within the one uint16_t that is sent in parallel to the HUB75. 
 */
struct frameStruct {
    rowColorDepthStruct rowdata[ROWS_PER_FRAME];
};



/***************************************************************************************/   
#ifdef USE_GFX_ROOT
class RGB64x32MatrixPanel_I2S_DMA : public GFX {
#else
class RGB64x32MatrixPanel_I2S_DMA : public Adafruit_GFX {	
#endif

  // ------- PUBLIC -------
  public:
    
    /**
     * RGB64x32MatrixPanel_I2S_DMA 
     * 
     * @param  {bool} _double_buffer : Double buffer is disabled by default. Enable only if you know what you're doing. Manual switching required with flipDMABuffer() and showDMABuffer()
     *        
     */
    RGB64x32MatrixPanel_I2S_DMA()
#ifdef USE_GFX_ROOT	
      : GFX(MATRIX_WIDTH, MATRIX_HEIGHT)  {
#else
      : Adafruit_GFX(MATRIX_WIDTH, MATRIX_HEIGHT)   {
#endif		  

    }

    /* Propagate the DMA pin configuration, or use compiler defaults */
    bool begin(int dma_r1_pin = R1_PIN_DEFAULT , int dma_g1_pin = G1_PIN_DEFAULT, int dma_b1_pin = B1_PIN_DEFAULT , int dma_r2_pin = R2_PIN_DEFAULT , int dma_g2_pin = G2_PIN_DEFAULT , int dma_b2_pin = B2_PIN_DEFAULT , int dma_a_pin  = A_PIN_DEFAULT  , int dma_b_pin = B_PIN_DEFAULT  , int dma_c_pin = C_PIN_DEFAULT , int dma_d_pin = D_PIN_DEFAULT  , int dma_e_pin = E_PIN_DEFAULT , int dma_lat_pin = LAT_PIN_DEFAULT, int dma_oe_pin = OE_PIN_DEFAULT , int dma_clk_pin = CLK_PIN_DEFAULT)
    { 

      if ( !allocateDMAmemory(dma_r1_pin, dma_g1_pin, dma_b1_pin, dma_r2_pin, dma_g2_pin, dma_b2_pin, dma_a_pin,  dma_b_pin, dma_c_pin, dma_d_pin, dma_e_pin, dma_lat_pin,  dma_oe_pin,   dma_clk_pin) ) {  return false; } // couldn't even get the basic ram required.
   
      // Flush the DMA buffers prior to configuring DMA - Avoid visual artefacts on boot.
      fillScreen(0); // Must fill the DMA buffer with the initial output bit sequence or the panel will display garbage
      flipDMABuffer(); // flip to backbuffer 1
      fillScreen(0); // Must fill the DMA buffer with the initial output bit sequence or the panel will display garbage
      flipDMABuffer(); // backbuffer 0

      return everything_OK;

    }

    // Draw pixels
    virtual void drawPixel(int16_t x, int16_t y, uint16_t color);   // overwrite adafruit implementation
    //virtual void fillScreen(uint16_t color);                        // overwrite adafruit implementation
    void clearScreen() { fillScreen(0); } 

    void drawPixel(int16_t x, int16_t y, uint8_t r, uint8_t g, uint8_t b);
   
    // Color 444 is a 4 bit scale, so 0 to 15, color 565 takes a 0-255 bit value, so scale up by 255/15 (i.e. 17)!
    uint16_t color444(uint8_t r, uint8_t g, uint8_t b) { return color565(r*17,g*17,b*17); }

    // Converts RGB888 to RGB565
    uint16_t color565(uint8_t r, uint8_t g, uint8_t b); // This is what is used by Adafruit GFX!
    

    inline void flipDMABuffer()
    {
      
        i2s_parallel_flip_to_buffer(&I2S1, back_buffer_id);

        // Flip to other buffer as the backbuffer. i.e. Graphic changes happen to this buffer (but aren't displayed until showDMABuffer())
        back_buffer_id ^= 1;    

        if (!back_buffer_id)
              fb = matrix_framebuffer_malloc_1;
        else
              fb = matrix_framebuffer_malloc_2;
            
    }
    
    
    inline void setPanelBrightness(int b)
    {
      // Change to set the brightness of the display, range of 1 to matrixWidth (i.e. 1 - 64)
        brightness = b;
    }

    inline void set_debug(bool state)
    {
      debug = state;
    }
    

    i2s_parallel_buffer_desc_t *fb_desc;   // for dma descriptors

    frameStruct *matrix_framebuffer_malloc_1; // fb 1
    frameStruct *matrix_framebuffer_malloc_2; // fb 2   
    frameStruct *fb; // back buffer

    // ESP32-RGB64x32MatrixPanel-I2S-DMA functioning
    bool everything_OK        = false;
    int  back_buffer_id       = 0;    // If using double buffer, which one is NOT active (ie. being displayed) to write too?
    int  brightness           = 48;             // If you get ghosting... reduce brightness level. 60 seems to be the limit before ghosting on a 64 pixel wide physical panel for some panels.
    int  min_refresh_rate     = 60;            // Probably best to leave as is unless you want to experiment. Framerate has an impact on brightness and also power draw - voltage ripple.
    int  lsbMsbTransitionBit  = 0;     // For possible color depth calculations
    bool debug = false;


    /* Calculate the memory available for DMA use, do some other stuff, and allocate accordingly */
    bool allocateDMAmemory(int r1_pin, int  g1_pin, int  b1_pin, int  r2_pin, int  g2_pin, int  b2_pin, int  a_pin, int   b_pin, int  c_pin, int  d_pin, int  e_pin, int  lat_pin, int   oe_pin, int clk_pin);

    /* Update a specific pixel in the DMA buffer to a colour */
    void updateMatrixDMABuffer(int16_t x, int16_t y, uint8_t red, uint8_t green, uint8_t blue);

	// Clear everything
	//void clearMatrixDMABufferRGB();
	

}; // end Class header

/***************************************************************************************/   

// For adafruit
inline void RGB64x32MatrixPanel_I2S_DMA::drawPixel(int16_t x, int16_t y, uint16_t color) 
{
  uint8_t r = ((((color >> 11) & 0x1F) * 527) + 23) >> 6;
  uint8_t g = ((((color >> 5) & 0x3F) * 259) + 33) >> 6;
  uint8_t b = (((color & 0x1F) * 527) + 23) >> 6;
  
  updateMatrixDMABuffer( x, y, r, g, b);
}

inline void RGB64x32MatrixPanel_I2S_DMA::drawPixel(int16_t x, int16_t y, uint8_t r, uint8_t g,uint8_t b) 
{
  updateMatrixDMABuffer( x, y, r, g, b);
}

// Pass 8-bit (each) R,G,B, get back 16-bit packed color
//https://github.com/squix78/ILI9341Buffer/blob/master/ILI9341_SPI.cpp
inline uint16_t RGB64x32MatrixPanel_I2S_DMA::color565(uint8_t r, uint8_t g, uint8_t b) {

  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}


#endif
