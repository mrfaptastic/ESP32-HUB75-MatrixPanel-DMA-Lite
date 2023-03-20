#include "ESP32-RGBMatrixPanel-I2S-DMA-lite.h"

/* We need to update the correct uint16_t in the rowBitStruct array, that gets sent out in parallel
 * 16 bit parallel mode - Save the calculated value to the bitplane memory in reverse order to account for I2S Tx FIFO mode1 ordering
 * Irrelevant for ESP32-S2 the way the FIFO ordering works is different - refer to page 679 of S2 technical reference manual
 */
#define ESP32_TX_FIFO_POSITION_ADJUST(x_coord) (((x_coord)&1U) ? (x_coord - 1) : (x_coord + 1))

/* This library is designed to take an 8 bit / 1 byte value (0-255) for each R G B colour sub-pixel.
 * The PIXEL_COLOR_DEPTH_BITS should always be '8' as a result.
 * However, if the library is to be used with lower colour depth (i.e. 6 bit colour), then we need to ensure the 8-bit value passed to the colour masking
 * is adjusted accordingly to ensure the LSB's are shifted left to MSB, by the difference. Otherwise the colours will be all screwed up.
 */
#define MASK_OFFSET (16 - PIXEL_COLOR_DEPTH_BITS)
#define PIXEL_COLOR_MASK_BIT(color_depth_index, mask_offset) (1 << (color_depth_index + mask_offset))



bool RGB64x32MatrixPanel_I2S_DMA::allocateDMAmemory(int r1_pin, int  g1_pin, int  b1_pin, int  r2_pin, int  g2_pin, int  b2_pin, int  a_pin, int   b_pin, int  c_pin, int  d_pin, int  e_pin, int  lat_pin, int   oe_pin, int clk_pin)
{

   /***
    * Step 1: Look at the overall DMA capable memory for the DMA FRAMEBUFFER data only (not the DMA linked list descriptors yet) 
    *         and do some pre-checks.
    */
    size_t _frame_buffer_memory_required        = sizeof(frameStruct); // * _num_frame_buffers; 

    // Can we fit the framebuffer into the single DMA capable memory block available?
    if ( heap_caps_get_largest_free_block(MALLOC_CAP_DMA) < _frame_buffer_memory_required  ) {
        Serial.println("Not enough memory to create framebuffer 1.\r\n");          
      return false;
    }
 
     // Allocate the framebuffer 1 memory, fail if we can even do this
    matrix_framebuffer_malloc_1 = (frameStruct *)heap_caps_malloc(_frame_buffer_memory_required, MALLOC_CAP_DMA);
    if ( matrix_framebuffer_malloc_1 == NULL ) {       
        Serial.println("ERROR: Couldn't malloc matrix_framebuffer_malloc_1! Critical fail.\r\n");            

        return false;
    }
    memset(matrix_framebuffer_malloc_1, 0, _frame_buffer_memory_required);

    // Can we fit the framebuffer into the single DMA capable memory block available?
    // Can we fit the framebuffer into the single DMA capable memory block available?
    if ( heap_caps_get_largest_free_block(MALLOC_CAP_DMA) < _frame_buffer_memory_required  ) { 
        Serial.println("Not enough memory to create framebuffer 2.\r\n");   
      return false;
    }

    // Allocate the framebuffer 2 memory, fail if we can even do this
    matrix_framebuffer_malloc_2 = (frameStruct *)heap_caps_malloc(_frame_buffer_memory_required, MALLOC_CAP_DMA);
    if ( matrix_framebuffer_malloc_2 == NULL ) {       
        Serial.println("ERROR: Couldn't malloc matrix_framebuffer_malloc_2! Critical fail.\r\n");            
        return false;
    }

    memset(matrix_framebuffer_malloc_2, 0, _frame_buffer_memory_required);    

    fb_desc = (i2s_parallel_buffer_desc_t*) heap_caps_malloc(sizeof(i2s_parallel_buffer_desc_t)*4, MALLOC_CAP_DMA);

    for (int i =0; i < 4; i++) {
      fb_desc[i].memory = nullptr;
      fb_desc[i].size = 0;
    }

    fb_desc[0].memory = matrix_framebuffer_malloc_1;
    fb_desc[0].size =_frame_buffer_memory_required;

    fb_desc[2].memory = matrix_framebuffer_malloc_2;
    fb_desc[2].size =_frame_buffer_memory_required;    

    i2s_parallel_config_t cfg={
        .gpio_bus={r1_pin, g1_pin, b1_pin, r2_pin, g2_pin, b2_pin, -1,-1, -1, a_pin, b_pin, c_pin, d_pin, e_pin, lat_pin, oe_pin},
        .gpio_clk=clk_pin,
        .clkspeed_hz=ESP32_I2S_CLOCK_SPEED, //ESP32_I2S_CLOCK_SPEED,  // formula used is 80000000L/(cfg->clkspeed_hz + 1), must result in >=2.  Acceptable values 26.67MHz, 20MHz, 16MHz, 13.34MHz...
        .bits=ESP32_I2S_DMA_MODE, //ESP32_I2S_DMA_MODE,
        .bufa=&fb_desc[0],
        .bufb=&fb_desc[2]
    };

    //Setup I2S
    i2s_parallel_setup_without_malloc(&I2S_DEVICE, &cfg);

    #if SERIAL_DEBUG  
      Serial.println("configureDMA(): DMA configuration completed on I2S_DEVICE.\r\n");
    #endif      


    // Just os we know
  	everything_OK = true;

    return true;

} // end initMatrixDMABuffer()

/* Update a specific co-ordinate in the DMA buffer */
void RGB64x32MatrixPanel_I2S_DMA::updateMatrixDMABuffer(int16_t x_coord, int16_t y_coord, uint8_t red, uint8_t green, uint8_t blue)
{
    if ( !everything_OK ) { 
      

      #if SERIAL_DEBUG 
              Serial.println("Cannot updateMatrixDMABuffer as setup failed!");
      #endif         
      
      return;
    }

  
  #ifdef SPLIT_MEMORY_MODE
  #ifdef SERIAL_DEBUG 
    int tmp_y_coord = y_coord;
  #endif
  #endif
    
   /* 1) Check that the co-ordinates are within range, or it'll break everything big time.
    * Valid co-ordinates are from 0 to (MATRIX_XXXX-1)
    */
	  if ( x_coord < 0 || y_coord < 0 || x_coord >= MATRIX_WIDTH || y_coord >= MATRIX_HEIGHT) {
      return;
    }

   /* 2) Convert the vertical axis / y-axis pixel co-ordinate to a matrix panel parallel co-ordinate..
    * eg. If the y co-ordinate is 23, that's actually in the second half of the panel, row 7.
    *     23 (y coord) - 16 (for 32px high panel) = 7 
    */
    bool paint_top_half = true;
    if ( y_coord >= ROWS_PER_FRAME) // co-ords start at zero, y_coord = 15 = 16 (rows per frame)
    {
        y_coord -= ROWS_PER_FRAME;  // Subtract the ROWS_PER_FRAME from the pixel co-ord to get the panel co-ord.
        paint_top_half = false;
    }
       
    for(int color_depth_idx=0; color_depth_idx<PIXEL_COLOR_DEPTH_BITS; color_depth_idx++)  // color depth - 8 iterations
    {
        uint16_t mask = (1 << color_depth_idx); // 24 bit color
        
        // The destination for the pixel bitstream 
        rowBitStruct *p;

        //Serial.printf("Drawing to buffer %d", back_buffer_id);

        if (back_buffer_id == 0) {
          p = &matrix_framebuffer_malloc_1->rowdata[y_coord].rowbits[color_depth_idx];
        } else {
          p = &matrix_framebuffer_malloc_2->rowdata[y_coord].rowbits[color_depth_idx];
        }
     
        int v=0; // the output bitstream
        
        // if there is no latch to hold address, output ADDX lines directly to GPIO and latch data at end of cycle
        int gpioRowAddress = y_coord;
        
        // normally output current rows ADDX, special case for LSB, output previous row's ADDX (as previous row is being displayed for one latch cycle)
        if(color_depth_idx == 0)
          gpioRowAddress = y_coord-1;
        
        if (gpioRowAddress & 0x01) v|=BIT_A; // 1
        if (gpioRowAddress & 0x02) v|=BIT_B; // 2
        if (gpioRowAddress & 0x04) v|=BIT_C; // 4
        if (gpioRowAddress & 0x08) v|=BIT_D; // 8
        if (gpioRowAddress & 0x10) v|=BIT_E; // 16
		
        // need to disable OE after latch to hide row transition
        if((x_coord) == 0 ) v|=BIT_OE;
        
        // drive latch while shifting out last bit of RGB data
        if((x_coord) == PIXELS_PER_ROW-1) v|=BIT_LAT;
		
        // need to turn off OE one clock before latch, otherwise can get ghosting
        if((x_coord)==PIXELS_PER_ROW-2) v|=BIT_OE;		

        // turn off OE after brightness value is reached when displaying MSBs
        // MSBs always output normal brightness
        // LSB (!color_depth_idx) outputs normal brightness as MSB from previous row is being displayed
        if((color_depth_idx > lsbMsbTransitionBit || !color_depth_idx) && ((x_coord) >= brightness)) v|=BIT_OE; // For Brightness
        
        // special case for the bits *after* LSB through (lsbMsbTransitionBit) - OE is output after data is shifted, so need to set OE to fractional brightness
        if(color_depth_idx && color_depth_idx <= lsbMsbTransitionBit) {
          // divide brightness in half for each bit below lsbMsbTransitionBit
          int lsbBrightness = brightness >> (lsbMsbTransitionBit - color_depth_idx + 1);
          if((x_coord) >= lsbBrightness) v|=BIT_OE; // For Brightness
        }
        
        /* When using the drawPixel, we are obviously only changing the value of one x,y position, 
         * however, the HUB75 is wired up such that it is always painting TWO lines at the same time
         * and this reflects the parallel in-DMA-memory data structure of uint16_t's that are getting
         * pumped out at high speed.
         * 
         * So we need to ensure we persist the bits (8 of them) of the uint16_t for the row we aren't changing.
         * 
         * The DMA buffer order has also been reversed (refer to the last code in this function)
         * so we have to check for this and check the correct position of the MATRIX_DATA_STORAGE_TYPE
         * data.
         */
        int tmp_x_coord = ESP32_TX_FIFO_POSITION_ADJUST(x_coord);

       if (debug)
        {
          Serial.print("dma x pos: "); Serial.println(tmp_x_coord, DEC);

        }

        if (paint_top_half)
        { // Need to copy what the RGB status is for the bottom pixels

           // Set the color of the pixel of interest
           if (green & mask) {  v|=BIT_G1; }
           if (blue & mask)  {  v|=BIT_B1; }
           if (red & mask)   {  v|=BIT_R1; }

           // Persist what was painted to the other half of the frame equiv. pixel
           if (p->data[tmp_x_coord] & BIT_R2)
                v|=BIT_R2;
                
           if (p->data[tmp_x_coord] & BIT_G2)
                v|=BIT_G2;

           if (p->data[tmp_x_coord] & BIT_B2)
                v|=BIT_B2;
        }
        else
        { // Do it the other way around 

          // Color to set
          if (red & mask)   { v|=BIT_R2; }
          if (green & mask) { v|=BIT_G2; }
          if (blue & mask)  { v|=BIT_B2; }
          
          // Copy
          if (p->data[tmp_x_coord] & BIT_R1)
              v|=BIT_R1;
              
          if (p->data[tmp_x_coord] & BIT_G1)
              v|=BIT_G1;
          
          if (p->data[tmp_x_coord] & BIT_B1)
              v|=BIT_B1; 
               
        } // paint
		
        // 16 bit parallel mode
        //Save the calculated value to the bitplane memory in reverse order to account for I2S Tx FIFO mode1 ordering
          p->data[tmp_x_coord] = v;
          
    } // color depth loop (8)

} // updateMatrixDMABuffer (specific co-ords change)

/*

/* Update the entire buffer with a single specific colour - quicker 
void RGB64x32MatrixPanel_I2S_DMA::clearMatrixDMABufferRGB()
{
  if ( !everything_OK ) return;

  for (unsigned int matrix_frame_parallel_row = 0; matrix_frame_parallel_row < ROWS_PER_FRAME; matrix_frame_parallel_row++) // half height - 16 iterations
  {	
    for(int color_depth_idx=0; color_depth_idx<PIXEL_COLOR_DEPTH_BITS; color_depth_idx++)  // color depth - 8 iterations
    {
        for(int x_coord=0; x_coord < MATRIX_WIDTH; x_coord++) // row pixel width 64 iterations
        { 		


  
          int v=0; // the output bitstream

          int y_coord = matrix_frame_parallel_row;
          
          // if there is no latch to hold address, output ADDX lines directly to GPIO and latch data at end of cycle
          int gpioRowAddress = y_coord;
          
          // normally output current rows ADDX, special case for LSB, output previous row's ADDX (as previous row is being displayed for one latch cycle)
          if(color_depth_idx == 0)
            gpioRowAddress = y_coord-1;
          
          if (gpioRowAddress & 0x01) v|=BIT_A; // 1
          if (gpioRowAddress & 0x02) v|=BIT_B; // 2
          if (gpioRowAddress & 0x04) v|=BIT_C; // 4
          if (gpioRowAddress & 0x08) v|=BIT_D; // 8
          if (gpioRowAddress & 0x10) v|=BIT_E; // 16
      
          // need to disable OE after latch to hide row transition
          if((x_coord) == 0 ) v|=BIT_OE;
          
          // drive latch while shifting out last bit of RGB data
          if((x_coord) == PIXELS_PER_ROW-1) v|=BIT_LAT;
      
          // need to turn off OE one clock before latch, otherwise can get ghosting
          if((x_coord)==PIXELS_PER_ROW-2) v|=BIT_OE;		

          // turn off OE after brightness value is reached when displaying MSBs
          // MSBs always output normal brightness
          // LSB (!color_depth_idx) outputs normal brightness as MSB from previous row is being displayed
          if((color_depth_idx > lsbMsbTransitionBit || !color_depth_idx) && ((x_coord) >= brightness)) v|=BIT_OE; // For Brightness
          
          // special case for the bits *after* LSB through (lsbMsbTransitionBit) - OE is output after data is shifted, so need to set OE to fractional brightness
          if(color_depth_idx && color_depth_idx <= lsbMsbTransitionBit) {
            // divide brightness in half for each bit below lsbMsbTransitionBit
            int lsbBrightness = brightness >> (lsbMsbTransitionBit - color_depth_idx + 1);
            if((x_coord) >= lsbBrightness) v|=BIT_OE; // For Brightness

          }

                      
           //fb->rowdata[matrix_frame_parallel_row].rowbits[color_depth_idx].data[x_coord] &= BITMASK_RGB12_CLEAR;
           fb->rowdata[matrix_frame_parallel_row].rowbits[color_depth_idx].data[x_coord] = v;          
           
        } // end x_coord iteration
    } // colour depth loop (8)
  } // end row iteration
} // updateMatrixDMABuffer (full frame paint)
*/