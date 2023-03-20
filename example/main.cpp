#include <ESP32-RGBMatrixPanel-I2S-DMA-lite.h>
RGB64x32MatrixPanel_I2S_DMA dma_display;

void setup() {


  Serial.begin(115200);

  Serial.println("*****************************************************");
  Serial.println(" HELLO !");
  Serial.println("*****************************************************");

  dma_display.begin();

  delay(1000);
  // fix the screen with green
  dma_display.fillRect(0, 0, dma_display.width(), dma_display.height(), dma_display.color444(0, 15, 0));  
  dma_display.drawRect(0, 0, dma_display.width(), dma_display.height(), dma_display.color444(15, 15, 0));
  dma_display.drawLine(0, 0, dma_display.width()-1, dma_display.height()-1, dma_display.color444(15, 0, 0));
  dma_display.drawLine(dma_display.width()-1, 0, 0, dma_display.height()-1, dma_display.color444(15, 0, 0));
  dma_display.drawCircle(10, 10, 10, dma_display.color444(0, 0, 15));
  dma_display.fillCircle(40, 21, 10, dma_display.color444(15, 0, 15));

  // Show everything above.
  dma_display.flipDMABuffer();
  delay(2000);


}

int column = 0;
void loop() {
  // do nothing


    dma_display.clearScreen();
    dma_display.drawLine(column, 0, column, dma_display.height()-1, dma_display.color444(15, 15, 15));
    dma_display.flipDMABuffer();
    column++;

    if (column == 64) column = 0;

    delay(10);

}
