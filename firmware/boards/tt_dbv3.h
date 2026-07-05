/*
 * Minimal board definition for the Tiny Tapeout demo board v3 (RP2350B).
 * The B package matters: the ui/uo/uio pins sit on GPIOs 33..40, which
 * only exist with PICO_RP2350A = 0.
 */
// pico_cmake_set PICO_PLATFORM=rp2350-arm-s

#ifndef _BOARDS_TT_DBV3_H
#define _BOARDS_TT_DBV3_H

#define PICO_RP2350A 0 /* RP2350B: 48-GPIO bank */

#define PICO_DEFAULT_LED_PIN 11

#ifndef PICO_FLASH_SPI_CLKDIV
#define PICO_FLASH_SPI_CLKDIV 2
#endif

/* Assumed 16 MB QSPI flash; this firmware never addresses flash beyond
 * its own (tiny) image, so an oversize value is harmless. */
#ifndef PICO_FLASH_SIZE_BYTES
#define PICO_FLASH_SIZE_BYTES (16 * 1024 * 1024)
#endif

#endif
