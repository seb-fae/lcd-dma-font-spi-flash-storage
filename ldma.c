/*
 * ldma_access.c
 *
 *  Created on: 11 mars 2022
 *      Author: secerdan
 */

#include "em_ldma.h"
#include "em_cmu.h"
#include "dmadrv.h"
#include <stdio.h>
#include "cmsis_os2.h"
#include "app.h"

static osEventFlagsId_t evt_id;                        // event flags id
unsigned int rx_channel, tx_channel;

void (*callback)();

bool ldma_cb()
{
  osEventFlagsSet(evt_id, 0x0001U);
  return 0;
}

void start_spi_ldma_transfer(uint32_t size, uint8_t *tx, uint8_t *rx)
{
  SL_USART_EXTFLASH_LCD->CMD = USART_CMD_CLEARRX;

  //Starting both ldma transfers on different channels
  if (rx)
  {
    DMADRV_PeripheralMemory(rx_channel, dmadrvPeripheralSignal_EUSART1_RXDATAV, rx, (void*)&(SL_USART_EXTFLASH_LCD->RXDATA), true, size, dmadrvDataSize1, (DMADRV_Callback_t)&ldma_cb, NULL); 

    DMADRV_MemoryPeripheral(tx_channel, dmadrvPeripheralSignal_EUSART1_TXBL, (void*)&(SL_USART_EXTFLASH_LCD->TXDATA), tx, true, size, dmadrvDataSize1, NULL, NULL);  
  }
  else
    DMADRV_MemoryPeripheral(tx_channel, dmadrvPeripheralSignal_EUSART1_TXBL, (void*)&(SL_USART_EXTFLASH_LCD->TXDATA), tx, true, size, dmadrvDataSize1, (DMADRV_Callback_t)&ldma_cb, NULL);

  /* Wait end of DMA transfer */ 
  osEventFlagsWait(evt_id, 0x0001U, osFlagsWaitAny, osWaitForever);
}

/**************************************************************************//**
 * @Initialize LDMA Descriptors and start transfers
 *****************************************************************************/
void initLDMA(void)
{

  evt_id = osEventFlagsNew(NULL);

  DMADRV_Init();

  // Allocate channels for transmission and reception
  uint32_t status = DMADRV_AllocateChannel(&tx_channel, NULL);
  EFM_ASSERT(status == ECODE_EMDRV_DMADRV_OK);
  status = DMADRV_AllocateChannel(&rx_channel, NULL);
  EFM_ASSERT(status == ECODE_EMDRV_DMADRV_OK);
}


