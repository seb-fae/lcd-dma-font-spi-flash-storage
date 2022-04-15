/*
 * spi_flash_access.c
 *
 *  Created on: 24 févr. 2022
 *      Author: secerdan
 */

#include <stdio.h>
#include <sl_udelay.h>
#include "app.h"
/***************************************************************************//**
 * @file
 * @brief Spiflash-backed storage component for Silicon Labs Bootloader.
 *******************************************************************************
 * # License
 * <b>Copyright 2021 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc.  Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement.  This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#include "em_eusart.h"
#include "em_gpio.h"
#include "em_ldma.h"
#include "spi_flash_access.h"
#include "app.h"

uint8_t buf_count = 1;

bool storage_isBusy();

uint32_t cfg0, cfg2;
uint32_t cfg0_flash, cfg2_flash;

//Max font size is 32x64. One additionnal word for SPI Flash command.
#define BSIZE (64 + 1)

uint32_t rbuffer[BSIZE];
uint32_t tbuffer[BSIZE];

#ifdef STORAGE_DMA_ACCESS

void start_spi_ldma_transfer(uint32_t size, uint8_t *tx, uint8_t *rx);


#endif

void spi_init(void)
{
  CMU_ClockEnable(cmuClock_EUSART1, true);
  CMU_ClockEnable(cmuClock_GPIO, true);

  // MOSI
  GPIO_PinModeSet(SL_USART_EXTFLASH_TX_PORT,
                  SL_USART_EXTFLASH_TX_PIN,
                  gpioModePushPull,
                  0);
  // MISO
  GPIO_PinModeSet(SL_USART_EXTFLASH_RX_PORT,
                  SL_USART_EXTFLASH_RX_PIN,
                  gpioModeInputPull,
                  0);
  // CLK
  GPIO_PinModeSet(SL_USART_EXTFLASH_CLK_PORT,
                  SL_USART_EXTFLASH_CLK_PIN,
                  gpioModePushPull,
                  0);
  // CS#
  GPIO_PinModeSet(SL_USART_EXTFLASH_CS_PORT,
                  SL_USART_EXTFLASH_CS_PIN,
                  gpioModePushPull,
                  1);


  // SPI advanced configuration (part of the initializer)
  EUSART_SpiAdvancedInit_TypeDef adv = EUSART_SPI_ADVANCED_INIT_DEFAULT;

  // Default asynchronous initializer (main/master mode and 8-bit data)
  EUSART_SpiInit_TypeDef init = EUSART_SPI_MASTER_INIT_DEFAULT_HF;

  adv.msbFirst = true;        // SPI standard MSB first
  init.bitRate = SL_USART_EXTFLASH_FREQUENCY;
  init.advancedSettings = &adv;  // Advanced settings structure

  /*
   * Route EUSART1 MOSI, MISO, and SCLK to the specified pins.  CS is
   * not controlled by EUSART1 so there is no write to the corresponding
   * EUSARTROUTE register to do this.
   */
  GPIO->EUSARTROUTE[1].TXROUTE = (SL_USART_EXTFLASH_TX_PORT << _GPIO_EUSART_TXROUTE_PORT_SHIFT)
      | (SL_USART_EXTFLASH_TX_PIN << _GPIO_EUSART_TXROUTE_PIN_SHIFT);
  GPIO->EUSARTROUTE[1].RXROUTE = (SL_USART_EXTFLASH_RX_PORT << _GPIO_EUSART_RXROUTE_PORT_SHIFT)
      | (SL_USART_EXTFLASH_RX_PIN << _GPIO_EUSART_RXROUTE_PIN_SHIFT);
  GPIO->EUSARTROUTE[1].SCLKROUTE = (SL_USART_EXTFLASH_CLK_PORT << _GPIO_EUSART_SCLKROUTE_PORT_SHIFT)
      | (SL_USART_EXTFLASH_CLK_PIN << _GPIO_EUSART_SCLKROUTE_PIN_SHIFT);

  // Enable EUSART interface pins
  GPIO->EUSARTROUTE[1].ROUTEEN = GPIO_EUSART_ROUTEEN_RXPEN |    // MISO
                                 GPIO_EUSART_ROUTEEN_TXPEN |    // MOSI
                                 GPIO_EUSART_ROUTEEN_SCLKPEN;

  // Configure and enable EUSART1
  EUSART_SpiInit(SL_USART_EXTFLASH_LCD, &init);

  cfg0_flash = SL_USART_EXTFLASH_LCD->CFG0;
  cfg2_flash = SL_USART_EXTFLASH_LCD->CFG2;

#ifdef STORAGE_DMA_ACCESS
  for (uint32_t i = 0; i< BSIZE ;i++)
  {
    tbuffer[i] = 0xFF;
    rbuffer[i] = 0x00;
  }
#endif

}

void spi_setCsActive(void)
{
  GPIO_PinOutClear(SL_USART_EXTFLASH_CS_PORT, SL_USART_EXTFLASH_CS_PIN);
}

void spi_setCsInactive(void)
{
  GPIO_PinOutSet(SL_USART_EXTFLASH_CS_PORT, SL_USART_EXTFLASH_CS_PIN);
}

uint16_t spi_readHalfword(void)
{
  uint16_t retval = 0;
  retval = EUSART_Spi_TxRx(SL_USART_EXTFLASH_LCD, 0xFF) << 8;
  retval |= EUSART_Spi_TxRx(SL_USART_EXTFLASH_LCD, 0xFF);
  return retval;
}

void spi_write3Byte(uint32_t data)
{
  EUSART_Spi_TxRx(SL_USART_EXTFLASH_LCD, (data >> 16) & 0xFF);
  EUSART_Spi_TxRx(SL_USART_EXTFLASH_LCD, (data >> 8) & 0xFF);
  EUSART_Spi_TxRx(SL_USART_EXTFLASH_LCD, data & 0xFF);
}

void spi_writeByte(uint8_t data)
{
  EUSART_Spi_TxRx(SL_USART_EXTFLASH_LCD, data);
}

uint8_t spi_readByte(void)
{
  uint8_t byte = EUSART_Spi_TxRx(SL_USART_EXTFLASH_LCD, 0xFF);
  return byte;
}
// -----------------------------------------------------------------------------
// Functions

static void waitUntilNotBusy(void)
{
  while (storage_isBusy()) {
    // Do nothing
  }
}

static void setWriteEnableLatch(void)
{
  spi_setCsActive();
  spi_writeByte(CMD_WRITE_ENABLE);
  spi_setCsInactive();
}

static StorageSpiflashDevice_t getDeviceType(void)
{
  uint16_t deviceId;

  // cannot check for busy in this API since it is used by
  //  init.  Callers must verify not busy individually.
  spi_setCsActive();
  // following implementation takes smaller buffer (3) of efm into account
  spi_writeByte(CMD_JEDEC_ID);
  /* Read MfgId */
  spi_readByte();
  /* Read Device Id */
  deviceId = spi_readHalfword();
  spi_setCsInactive();

  switch (deviceId) {
    case DEVICE_ID_MACRONIX_8M_LP:
      return MACRONIX_8M_LP_DEVICE;
    default:
      return UNKNOWN_DEVICE;
  }
}

static uint32_t getDeviceSize(StorageSpiflashDevice_t *pDeviceType)
{
  StorageSpiflashDevice_t deviceType;
  waitUntilNotBusy();
  if (pDeviceType == NULL) {
    deviceType = getDeviceType();
  } else {
    deviceType = *pDeviceType;
    if (deviceType == UNKNOWN_DEVICE) {
      deviceType = getDeviceType();
      *pDeviceType = deviceType;
    }
  }
  switch (deviceType) {
    case MACRONIX_8M_DEVICE:
    case MACRONIX_8M_LP_DEVICE:
    case NUMONYX_8M_DEVICE:
      return DEVICE_SIZE_8M;
    default:
      return 0;
  }
}

static bool verifyAddressRange(uint32_t                address,
                               uint32_t                length,
                               StorageSpiflashDevice_t *pDeviceType)
{
  uint32_t deviceSize = getDeviceSize(pDeviceType);
  if ((length > deviceSize)
      || (address > deviceSize)) {
    return false;
  }

  if ((address + length) <= deviceSize) {
    return true;
  }

  // out of range
  return false;
}


static void sendCommand(uint8_t command, uint32_t address)
{
  spi_writeByte(command);
  spi_write3Byte(address);
}

static bool verifyErased(uint32_t address, uint32_t len)
{
  waitUntilNotBusy();

  spi_setCsActive();
  sendCommand(CMD_READ_DATA, address);

  while (len--) {
    if (spi_readByte() != 0xFF) {
      return false;
    }
  }

  spi_setCsInactive();
  return true;
}

static void writePage(uint32_t address, const uint8_t *data, uint32_t length)
{
  waitUntilNotBusy();
  setWriteEnableLatch();

  spi_setCsActive();
  sendCommand(CMD_PAGE_PROG, address);

  while (length--) {
    spi_writeByte(*data++);
  }
  spi_setCsInactive();
}

static void eraseCommand(uint8_t command, uint32_t address)
{
  waitUntilNotBusy();
  setWriteEnableLatch();
  spi_setCsActive();
  sendCommand(command, address);
  spi_setCsInactive();
}

static void eusart_sync(EUSART_TypeDef *eusart, uint32_t mask)
{
  // Wait for any pending previous write operation to have been completed
  // in the low-frequency domain.
  while ((eusart->SYNCBUSY & mask) != 0U) {
  }
}
void EUSART_Disable(EUSART_TypeDef *eusart)
{
    // General Programming Guideline to properly disable the module:
    // 1a. Disable TX and RX using TXDIS and RXDIS cmd
    eusart->CMD = EUSART_CMD_TXDIS | EUSART_CMD_RXDIS;
    // 1b. Poll for EUSARTn_SYNCBUSY.TXDIS and EUSARTn_SYNCBUSY.RXDIS to go low;
    eusart_sync(eusart, (EUSART_SYNCBUSY_TXDIS | EUSART_SYNCBUSY_RXDIS));
    // 1c. Wait for EUSARTn_STATUS.TXENS and EUSARTn_STATUS.RXENS to go low
    while (eusart->STATUS & (_EUSART_STATUS_TXENS_MASK | _EUSART_STATUS_RXENS_MASK)) {
    }

    eusart->EN_CLR = EUSART_EN_EN;

#if defined(_EUSART_EN_DISABLING_MASK)
    // 2. Polling for EUSARTn_EN.DISABLING = 0.
    while (eusart->EN & _EUSART_EN_DISABLING_MASK) {
    }
#endif
}


int32_t storage_init(void)
{
  spi_init();

  sl_udelay_wait(500000);

  // Release the chip from powerdown mode
  spi_setCsActive();
  spi_writeByte(CMD_POWER_UP);
  spi_setCsInactive();

  sl_udelay_wait(500000);

  StorageSpiflashDevice_t deviceType;

  deviceType = getDeviceType();
  if (deviceType == UNKNOWN_DEVICE) {
    return -1;
  }

  return 0;
}


void apply_flash_cfg()
{
  eusart_sync(SL_USART_EXTFLASH_LCD, _EUSART_SYNCBUSY_MASK);
  EUSART_Disable(SL_USART_EXTFLASH_LCD);

  /* backup configuration */
  SL_USART_EXTFLASH_LCD->CFG0 = cfg0_flash;
  SL_USART_EXTFLASH_LCD->CFG2 = cfg2_flash;

  // Enable EUSART interface pins
  GPIO->EUSARTROUTE[1].ROUTEEN = GPIO_EUSART_ROUTEEN_RXPEN |    // MISO
                                 GPIO_EUSART_ROUTEEN_TXPEN |    // MOSI
                                 GPIO_EUSART_ROUTEEN_SCLKPEN;

  // Enable EUSART IP.
  EUSART_Enable(SL_USART_EXTFLASH_LCD, eusartEnable);
  // Finally enable the Rx and/or Tx channel (as specified).
  SL_USART_EXTFLASH_LCD->CMD = EUSART_CMD_RXEN | EUSART_CMD_TXEN;
  eusart_sync(SL_USART_EXTFLASH_LCD, _EUSART_SYNCBUSY_RXEN_MASK | _EUSART_SYNCBUSY_TXEN_MASK);
  GPIO->EUSARTROUTE[1].RXROUTE = (SL_USART_EXTFLASH_RX_PORT << _GPIO_EUSART_RXROUTE_PORT_SHIFT)
      | (SL_USART_EXTFLASH_RX_PIN << _GPIO_EUSART_RXROUTE_PIN_SHIFT);

}

void cfg_backup(void)
{
  /* backup configuration */
  cfg0 = SL_USART_EXTFLASH_LCD->CFG0;
  cfg2 = SL_USART_EXTFLASH_LCD->CFG2;
}

void cfg_restore(void)
{
  eusart_sync(SL_USART_EXTFLASH_LCD, _EUSART_SYNCBUSY_MASK);
  EUSART_Disable(SL_USART_EXTFLASH_LCD);

  /* backup configuration */
  SL_USART_EXTFLASH_LCD->CFG0 = cfg0;
  SL_USART_EXTFLASH_LCD->CFG2 = cfg2;


  // Enable EUSART interface pins
  GPIO->EUSARTROUTE[1].ROUTEEN = GPIO_EUSART_ROUTEEN_TXPEN |    // MOSI
                                 GPIO_EUSART_ROUTEEN_SCLKPEN;

  // Enable EUSART IP.
  EUSART_Enable(SL_USART_EXTFLASH_LCD, eusartEnable);
  // Finally enable the Rx and/or Tx channel (as specified).
  SL_USART_EXTFLASH_LCD->CMD = EUSART_CMD_RXEN | EUSART_CMD_TXEN;
  eusart_sync(SL_USART_EXTFLASH_LCD, _EUSART_SYNCBUSY_RXEN_MASK | _EUSART_SYNCBUSY_TXEN_MASK);
}

bool storage_isBusy(void)
{
  uint8_t status;

  spi_setCsActive();
  spi_writeByte(CMD_READ_STATUS);
  status = spi_readByte();
  spi_setCsInactive();

  return (bool)(status & STATUS_BUSY_MASK);
}

uint8_t storage_start_read_dma(uint32_t address, size_t length);

int32_t storage_readRaw(uint32_t address, size_t length, uint8_t *data)
{

  cfg_backup();

  apply_flash_cfg();

  // Ensure address is is within chip
  if (!verifyAddressRange(address, length, NULL)) {
    return -2;
  }

  waitUntilNotBusy();

  spi_setCsActive();

#ifdef STORAGE_DMA_ACCESS
  EFM_ASSERT(data == (uint8_t*)(rbuffer + 1));
  storage_start_read_dma(address, length);
#else
  sendCommand(CMD_READ_DATA, address);

  while (length--) {
    *data++ = spi_readByte();
  }
#endif

  spi_setCsInactive();

  /* Restore SPI configuration */
  cfg_restore();

  return 0;
}

int32_t storage_writeRaw(uint32_t address, uint8_t *data, size_t numBytes)
{
  uint32_t nextPageAddr;
  uint32_t currentLength;

  // Ensure address is is within chip
  if (!verifyAddressRange(address, numBytes, NULL)) {
    return -2;
  }
  // Ensure space is empty
  if (!verifyErased(address, numBytes)) {
    return -3;
  }

  if (address & DEVICE_PAGE_MASK) {
    // handle unaligned first block
    nextPageAddr = (address & (~DEVICE_PAGE_MASK)) + DEVICE_PAGE_SIZE;
    if ((address + numBytes) < nextPageAddr) {
      // fits all within first block
      currentLength = numBytes;
    } else {
      currentLength = (uint16_t) (nextPageAddr - address);
    }
  } else {
    currentLength = (numBytes > DEVICE_PAGE_SIZE) ? DEVICE_PAGE_SIZE : numBytes;
  }
  while (numBytes) {
    writePage(address, data, currentLength);
    numBytes -= currentLength;
    address += currentLength;
    data += currentLength;
    currentLength = (numBytes > DEVICE_PAGE_SIZE) ? DEVICE_PAGE_SIZE : numBytes;
  }

  return 0;
}

int32_t storage_getDMAchannel(void)
{
  return -1;
}

int32_t storage_eraseRaw(uint32_t address, size_t totalLength)
{
  // Get device characteristics
  StorageSpiflashDevice_t deviceType = UNKNOWN_DEVICE;
  uint32_t sectorMask = DEVICE_SECTOR_MASK;
  uint32_t deviceSize = getDeviceSize(&deviceType);
  uint32_t deviceBlockSize = DEVICE_BLOCK_SIZE_64K;
  uint32_t deviceBlockMask = DEVICE_BLOCK_MASK_64K;
  // Numonyx/Micron parts only support block erase, not sector
  if ((deviceType >= NUMONYX_2M_DEVICE)
      && (deviceType <= NUMONYX_16M_DEVICE)) {
    sectorMask = DEVICE_BLOCK_MASK_64K;
  } else if ((deviceType >= ISSI_256K_DEVICE)
             && (deviceType <= ISSI_512K_DEVICE)) {
    deviceBlockSize = DEVICE_BLOCK_SIZE_32K;
    deviceBlockMask = DEVICE_BLOCK_MASK_32K;
  }

  // Validate that it's possible to erase the slot
  // Length must be a multiple of the sector size
  if (totalLength & sectorMask) {
    return -4;
  }
  // Address must be sector aligned
  if (address & sectorMask) {
    return -4;
  }
  // Address and length must be in range
  if (!verifyAddressRange(address, totalLength, &deviceType)) {
    return -2;
  }

  // Test for full chip erase
  if ((address == 0) && (totalLength == deviceSize)) {
    waitUntilNotBusy();
    setWriteEnableLatch();
    spi_setCsActive();
    spi_writeByte(CMD_ERASE_CHIP);
    spi_setCsInactive();
    return 0;
  }

  // first handle leading partial blocks
  while (totalLength && (address & deviceBlockMask)) {
    eraseCommand(CMD_ERASE_SECTOR, address);
    address += DEVICE_SECTOR_SIZE;
    totalLength -= DEVICE_SECTOR_SIZE;
  }
  // handle any full blocks
  while (totalLength >= deviceBlockSize) {
    eraseCommand(CMD_ERASE_BLOCK, address);
    address += deviceBlockSize;
    totalLength -= deviceBlockSize;
  }
  // finally handle any trailing partial blocks
  while (totalLength) {
    eraseCommand(CMD_ERASE_SECTOR, address);
    address += DEVICE_SECTOR_SIZE;
    totalLength -= DEVICE_SECTOR_SIZE;
  }
  return 0;
}

uint8_t * storage_allocate_buffer(size_t length)
{
  if (buf_count-- == 0)
    return NULL;

  if (length > ((BSIZE << 2) - 4))
    return NULL;

  return (uint8_t *)(rbuffer + 1);
}

void storage_free_buffer()
{
  buf_count++;
}

#ifdef STORAGE_DMA_ACCESS

uint8_t storage_start_read_dma(uint32_t address, size_t length)
{
  /* Put Read command and Address in txbuffer */
  uint8_t * p = (uint8_t *)tbuffer;
  *p++ = CMD_READ_DATA;
  *p++ = (address >> 16) & 0xFF;
  *p++ = (address >> 8) & 0xFF;
  *p++ = address & 0xFF;

  start_spi_ldma_transfer(length + 4, (uint8_t*)tbuffer, (uint8_t *)rbuffer);

  return 0;
}

#endif



