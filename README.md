# lcd-dma-font-spi-flash-storage

## Description

This example uses an external SPI FLASH to stores a 1bpp 32x64 font.
When user presses a button, a internal counter is incremented and its value is displayed on LCD using font stored in Spi flash.
LCD and SPI flash accesses are accelerated by LDMA. 

The graphical library must run inside a FreeRtos task which means that all LCD and Spi Flash operation will be blocked (not consuming any CPU time and letting other tasks run) waiting for end of DMA operation.

