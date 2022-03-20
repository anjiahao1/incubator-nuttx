/****************************************************************************
 * arch/arm/src/rp2040/hardware/rp2040_adc.h
 *
 * Generated from rp2040.svd originally provided by
 *   Raspberry Pi (Trading) Ltd.
 *
 * Copyright 2020 (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_RP2040_HARDWARE_RP2040_ADC_H
#define __ARCH_ARM_SRC_RP2040_HARDWARE_RP2040_ADC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/rp2040_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/
#define RP2040_ADC_CS_OFFSET       0x00000000
#define RP2040_ADC_RESULT_OFFSET   0x00000004
#define RP2040_ADC_FCS_OFFSET      0x00000008
#define RP2040_ADC_FIFO_OFFSET     0x0000000c
#define RP2040_ADC_DIV_OFFSET      0x00000010
#define RP2040_ADC_INTR_OFFSET     0x00000014
#define RP2040_ADC_INTE_OFFSET     0x00000018
#define RP2040_ADC_INTF_OFFSET     0x0000001c
#define RP2040_ADC_INTS_OFFSET     0x00000020
/* Register definitions *****************************************************/
#define RP2040_ADC_CS       (RP2040_ADC_BASE + RP2040_ADC_CS_OFFSET)
#define RP2040_ADC_RESULT   (RP2040_ADC_BASE + RP2040_ADC_CS_OFFSET)
#define RP2040_ADC_FCS      (RP2040_ADC_BASE + RP2040_ADC_CS_OFFSET)
#define RP2040_ADC_FIFO     (RP2040_ADC_BASE + RP2040_ADC_CS_OFFSET)
#define RP2040_ADC_DIV      (RP2040_ADC_BASE + RP2040_ADC_CS_OFFSET)
#define RP2040_ADC_INTR     (RP2040_ADC_BASE + RP2040_ADC_CS_OFFSET)
#define RP2040_ADC_INTE     (RP2040_ADC_BASE + RP2040_ADC_CS_OFFSET)
#define RP2040_ADC_INTF     (RP2040_ADC_BASE + RP2040_ADC_CS_OFFSET)
#define RP2040_ADC_INTS     (RP2040_ADC_BASE + RP2040_ADC_CS_OFFSET)

/* Register bit definitions *************************************************/


#endif /* __ARCH_ARM_SRC_RP2040_HARDWARE_RP2040_SPI_H */
