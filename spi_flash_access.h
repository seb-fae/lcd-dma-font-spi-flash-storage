/*
 * spi_flash_access.h
 *
 *  Created on: 24 févr. 2022
 *      Author: secerdan
 */

#ifndef SPI_FLASH_ACCESS_H_
#define SPI_FLASH_ACCESS_H_

#include "em_cmu.h"
#include "em_usart.h"
#include "em_gpio.h"

/// Unique identifiers for supported SPI flash parts
typedef enum {
  UNKNOWN_DEVICE,
  SPANSION_8M_DEVICE,
  WINBOND_2M_DEVICE,
  WINBOND_8M_DEVICE,
  MACRONIX_2M_DEVICE,
  MACRONIX_4M_DEVICE,
  MACRONIX_8M_DEVICE,
  MACRONIX_8M_LP_DEVICE,
  MACRONIX_16M_DEVICE,
  MACRONIX_16M_2V_DEVICE,
  MACRONIX_64M_LP_DEVICE,
  ATMEL_4M_DEVICE,
  ATMEL_8M_DEVICE,
  // N.B. If add more ATMEL_ devices, update storage_init() accordingly
  ADESTO_4M_DEVICE,
  NUMONYX_2M_DEVICE,
  NUMONYX_4M_DEVICE,
  NUMONYX_8M_DEVICE,
  NUMONYX_16M_DEVICE,
  // N.B. If add more NUMONYX_ devices, update storage_eraseRaw() accordingly
  ISSI_256K_DEVICE,
  ISSI_512K_DEVICE,
  ISSI_1M_DEVICE,
  ISSI_2M_DEVICE,
  ISSI_4M_DEVICE,
} StorageSpiflashDevice_t;

#define SL_USART_EXTFLASH_SPI_USART_CLOCK    cmuClock_EUSART1

// <o SL_USART_EXTFLASH_FREQUENCY> Frequency
// <i> Default: 6400000
#define SL_USART_EXTFLASH_FREQUENCY             6400000
// <<< sl:start pin_tool >>>
// <usart signal=TX,RX,CLK,(CS)> SL_USART_EXTFLASH
// $[USART_SL_USART_EXTFLASH]
#define SL_USART_EXTFLASH_PERIPHERAL_NO          1

// USART1 TX on PC6
#define SL_USART_EXTFLASH_TX_PORT                gpioPortC
#define SL_USART_EXTFLASH_TX_PIN                 1
#define SL_USART_EXTFLASH_TX_LOC                 11

// USART1 RX on PC7
#define SL_USART_EXTFLASH_RX_PORT                gpioPortC
#define SL_USART_EXTFLASH_RX_PIN                 2
#define SL_USART_EXTFLASH_RX_LOC                 11

// USART1 CLK on PC8
#define SL_USART_EXTFLASH_CLK_PORT               gpioPortC
#define SL_USART_EXTFLASH_CLK_PIN                3
#define SL_USART_EXTFLASH_CLK_LOC                11

// USART1 CS on PA4
#define SL_USART_EXTFLASH_CS_PORT                gpioPortC
#define SL_USART_EXTFLASH_CS_PIN                 4
#define SL_USART_EXTFLASH_CS_LOC                 1

#define BTL_DRIVER_SPI_USART_TXLOC \
  (SL_USART_EXTFLASH_TX_LOC << _USART_ROUTELOC0_TXLOC_SHIFT)
#define BTL_DRIVER_SPI_USART_RXLOC \
  (SL_USART_EXTFLASH_RX_LOC << _USART_ROUTELOC0_RXLOC_SHIFT)
#define BTL_DRIVER_SPI_USART_CLKLOC \
  (SL_USART_EXTFLASH_CLK_LOC << _USART_ROUTELOC0_CLKLOC_SHIFT)

#define DEVICE_SIZE_256K                    (32768L)
#define DEVICE_SIZE_512K                    (65536L)
#define DEVICE_SIZE_1M                      (131072L)
#define DEVICE_SIZE_2M                      (262144L)
#define DEVICE_SIZE_4M                      (524288L)
#define DEVICE_SIZE_8M                      (1048576L)
#define DEVICE_SIZE_16M                     (2097152L)
#define DEVICE_SIZE_64M                     (8388608L)

// Pages are the write buffer granularity
#define DEVICE_PAGE_SIZE                    (256)
#define DEVICE_PAGE_MASK                    (255)
// Sectors are the erase granularity
// *except* for Numonyx parts which only support BLOCK erase granularity
#define DEVICE_SECTOR_SIZE                  (4096)
#define DEVICE_SECTOR_MASK                  (4095)
// Blocks define a larger erase granularity
#define DEVICE_BLOCK_SIZE_64K               (65536L)
#define DEVICE_BLOCK_MASK_64K               (65535L)
#define DEVICE_BLOCK_SIZE_32K               (32768L)
#define DEVICE_BLOCK_MASK_32K               (32767L)
// The flash word size in bytes
#define DEVICE_WORD_SIZE                    (1)

// JEDEC Manufacturer IDs
#define MFG_ID_SPANSION                     (0x01)
#define MFG_ID_WINBOND                      (0xEF)
#define MFG_ID_MACRONIX                     (0xC2)
#define MFG_ID_ATMEL                        (0x1F)
#define MFG_ID_ADESTO                       (0x1F)
#define MFG_ID_NUMONYX                      (0x20)
#define MFG_ID_ISSI                         (0x9D)

// JEDEC Device IDs
#define DEVICE_ID_MACRONIX_2M               (0x2012)
#define DEVICE_ID_MACRONIX_4M               (0x2013)
#define DEVICE_ID_MACRONIX_8M               (0x2014)
#define DEVICE_ID_MACRONIX_8M_LP            (0x2814)
#define DEVICE_ID_MACRONIX_16M              (0x2015)
#define DEVICE_ID_MACRONIX_16M_2V           (0x2535)
#define DEVICE_ID_MACRONIX_64M_LP           (0x2817)

// Protocol commands
#define CMD_WRITE_ENABLE                    (0x06)
#define CMD_WRITE_DISABLE                   (0x04)
#define CMD_READ_STATUS                     (0x05)
#define CMD_WRITE_STATUS                    (0x01)
#define CMD_READ_DATA                       (0x03)
#define CMD_PAGE_PROG                       (0x02)
#define CMD_ERASE_SECTOR                    (0x20)
#define CMD_ERASE_BLOCK                     (0xD8)
#define CMD_ERASE_CHIP                      (0xC7)
#define CMD_POWER_DOWN                      (0xB9)
#define CMD_POWER_UP                        (0xAB)
#define CMD_JEDEC_ID                        (0x9F)
#define CMD_UNIQUE_ID                       (0x4B)

// Bitmasks for status register fields
#define STATUS_BUSY_MASK                    (0x01)
#define STATUS_WEL_MASK                     (0x02)

// These timings represent the worst case out of all chips supported by this
//  driver.  Some chips may perform faster.
// (in general Winbond is faster than Macronix is faster than Numonyx)
#define TIMING_POWERON_MAX_US               (30000)
#define TIMING_POWERDOWN_MAX_US             (10000)
#define TIMING_SLEEP_MAX_US                 (10)
#define TIMING_WAKEUP_MAX_US                (30)
#define TIMING_PROG_MAX_US                  (5000)
#define TIMING_WRITE_STATUS_MAX_US          (40000)
// (MS units are 1024Hz based)
#define TIMING_ERASE_SECTOR_MAX_MS          (410)
#define TIMING_ERASE_BLOCK_MAX_MS           (3072)

#define TIMING_ERASE_MACRONIX_2M_MAX_MS     (3892)
#define TIMING_ERASE_MACRONIX_4M_MAX_MS     (7680)
#define TIMING_ERASE_MACRONIX_8M_MAX_MS     (15360)
#define TIMING_ERASE_MACRONIX_8M_LP_MAX_MS  (30720)
#define TIMING_ERASE_MACRONIX_16M_MAX_MS    (30720)
#define TIMING_ERASE_MACRONIX_16M_2V_MAX_MS (20480)
#define TIMING_ERASE_MACRONIX_64M_LP_MAX_MS (245760)
#endif /* SPI_FLASH_ACCESS_H_ */


/// Information about the bootloader storage implementation
typedef struct {
  /// The version of this data structure
  uint16_t version;
  /// A bitmask describing the capabilities of this particular storage
  uint16_t capabilitiesMask;
  /// Maximum time it takes to erase a page. (in milliseconds)
  uint32_t pageEraseMs;
  /// Maximum time it takes to erase the entire part. (in milliseconds)
  uint32_t partEraseMs;
  /// The size of a single erasable page in bytes
  uint32_t pageSize;
  /// The total size of the storage in bytes
  uint32_t partSize;
  /// Pointer to a string describing the attached storage
  char *partDescription;
  /// The number of bytes in a word for the storage
  uint8_t wordSizeBytes;
} BootloaderStorageImplementationInformation_t;




