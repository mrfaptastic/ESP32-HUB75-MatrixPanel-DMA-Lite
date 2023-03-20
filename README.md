# ESP32-HUB75-MatrixPanel-DMA-Lite-
Playground library by the author. Bare bones version of ESP32-HUB75-MatrixPanel-DMA library which only has uber basic functions. No run-time configuration. ESP32 only (no S2, S3 etc.)

Double buffering is ALWAYS enabled with this version. You must use dma_display.flipDMABuffer() to display changes.