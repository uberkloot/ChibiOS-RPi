/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * @file    BCM2835/sdc_lld.h
 * @brief   BCM2835 SDC subsystem low level driver header.
 *
 * @addtogroup SDC
 * @{
 */

#ifndef _SDC_LLD_H_
#define _SDC_LLD_H_

#if HAL_USE_SDC || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/


/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/


/**
 * @brief   Write timeout in milliseconds.
 */
#if !defined(SDC_WRITE_TIMEOUT_MS) || defined(__DOXYGEN__)
#define SDC_WRITE_TIMEOUT_MS                250
#endif

/**
 * @brief   Read timeout in milliseconds.
 */
#if !defined(SDC_READ_TIMEOUT_MS) || defined(__DOXYGEN__)
#define SDC_READ_TIMEOUT_MS                 5
#endif

/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/


/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   Type of SDIO bus mode.
 */
typedef enum {
  SDC_MODE_1BIT = 0,
  SDC_MODE_4BIT,
  SDC_MODE_8BIT
} sdcbusmode_t;

/**
 * @brief   Type of card flags.
 */
typedef uint32_t sdcmode_t;

/**
 * @brief   SDC Driver condition flags type.
 */
typedef uint32_t sdcflags_t;

/**
 * @brief   Type of a structure representing an SDC driver.
 */
typedef struct SDCDriver SDCDriver;

/**
 * @brief   Driver configuration structure.
 * @note    It could be empty on some architectures.
 */
typedef struct {
  uint32_t dummy;
} SDCConfig;

/**
 * @brief   @p SDCDriver specific methods.
 */
#define _sdc_driver_methods                                                 \
  _mmcsd_block_device_methods

/**
 * @extends MMCSDBlockDeviceVMT
 *
 * @brief   @p SDCDriver virtual methods table.
 */
struct SDCDriverVMT {
  _sdc_driver_methods
};

/**
 * @brief   Structure representing an SDC driver.
 */
struct SDCDriver {
  /**
   * @brief Virtual Methods Table.
   */
  const struct SDCDriverVMT *vmt;
  _mmcsd_block_device_data
  /**
   * @brief Current configuration data.
   */
  const SDCConfig           *config;
  /**
   * @brief Various flags regarding the mounted card.
   */
  sdcmode_t                 cardmode;
  /**
   * @brief Errors flags.
   */
  sdcflags_t                errors;
  /**
   * @brief Card RCA.
   */
  uint32_t                  rca;
  /* End of the mandatory fields.*/
  /**
   * @brief Base frequency of the eMMC module.
   */
  uint32_t                  emmc_base_clock_rate;
  /**
   * @brief Thread waiting for I/O completion IRQ.
   */
  Thread                    *thread;
  /**
   * @brief     DMA mode bit mask.
   */
  uint32_t                  dmamode;

  /**
   * @brief     Pointer to the SDIO registers block.
   * @note      Used only for dubugging purpose.
   */
#if CH_DBG_ENABLE_ASSERTS
  SDIO_TypeDef              *sdio;
#endif
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @name    R1 response utilities
 * @{
 */
/**
 * @brief   Evaluates to @p TRUE if the R1 response contains error flags.
 *
 * @param[in] r1        the r1 response
 */
#define MMCSD_R1_ERROR(r1)              (((r1) & MMCSD_R1_ERROR_MASK) != 0)

/**
 * @brief   Returns the status field of an R1 response.
 *
 * @param[in] r1        the r1 response
 */
#define MMCSD_R1_STS(r1)                (((r1) >> 9) & 15)

/**
 * @brief   Evaluates to @p TRUE if the R1 response indicates a locked card.
 *
 * @param[in] r1        the r1 response
 */
#define MMCSD_R1_IS_CARD_LOCKED(r1)     (((r1) >> 21) & 1)
/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if !defined(__DOXYGEN__)
extern SDCDriver SDCD1;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void sdc_lld_init(void);
  void sdc_lld_start(SDCDriver *sdcp);
  void sdc_lld_stop(SDCDriver *sdcp);
  void sdc_lld_start_clk(SDCDriver *sdcp);
  void sdc_lld_set_data_clk(SDCDriver *sdcp);
  void sdc_lld_stop_clk(SDCDriver *sdcp);
  void sdc_lld_set_bus_mode(SDCDriver *sdcp, sdcbusmode_t mode);
  void sdc_lld_send_cmd_none(SDCDriver *sdcp, uint8_t cmd, uint32_t arg);
  bool_t sdc_lld_send_cmd_short(SDCDriver *sdcp, uint8_t cmd, uint32_t arg,
                                uint32_t *resp);
  bool_t sdc_lld_send_cmd_short_crc(SDCDriver *sdcp, uint8_t cmd, uint32_t arg,
                                    uint32_t *resp);
  bool_t sdc_lld_send_cmd_long_crc(SDCDriver *sdcp, uint8_t cmd, uint32_t arg,
                                   uint32_t *resp);
  bool_t sdc_lld_read(SDCDriver *sdcp, uint32_t startblk,
                      uint8_t *buf, uint32_t n);
  bool_t sdc_lld_write(SDCDriver *sdcp, uint32_t startblk,
                       const uint8_t *buf, uint32_t n);
  bool_t sdc_lld_sync(SDCDriver *sdcp);
  bool_t sdc_lld_is_card_inserted(SDCDriver *sdcp);
  bool_t sdc_lld_is_write_protected(SDCDriver *sdcp);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_SDC */

#endif /* _SDC_LLD_H_ */

/** @} */
