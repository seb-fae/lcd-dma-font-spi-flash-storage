/***************************************************************************//**
 * @file app.h
 * @brief Top level application functions
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#ifndef APP_H
#define APP_H

#include <stdio.h>
#include "em_common.h"

#define SL_USART_EXTFLASH_LCD   EUSART1

//#define SPI_FLASH_NEED_INITIALISATION
#define STORAGE_EXTERNAL_FLASH
#define STORAGE_DMA_ACCESS
#define LCD_LDMA

void start_spi_ldma_transfer(uint32_t size, uint8_t *tx, uint8_t *rx);
void memlcd_wait_dma();
uint8_t * storage_allocate_buffer(size_t length);
void storage_free_buffer();
int32_t storage_readRaw(uint32_t address, size_t length, uint8_t *data);

static inline uint8_t SL_RBIT8(uint8_t value)
{
  return (uint8_t)(SL_RBIT(value) >> 24);
}

/***************************************************************************//**
 * Initialize application.
 ******************************************************************************/
void app_init(void);

/***************************************************************************//**
 * App ticking function.
 ******************************************************************************/
void app_process_action(void);

#endif // APP_H
