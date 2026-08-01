#pragma once

#include <stdbool.h>

/* Core 1 entry point: bit-banged SPI RAM slave backing the BF tape. */
void spi_ram_core1_entry(void);

/* Zero the tape. Only call from core 0 with the ASIC in reset. */
void spi_ram_clear(void);

/* Gate for core 1. While false, core 1 stays parked and never drives
 * MISO — required when a non-BF design owns the uio pins. */
void spi_ram_set_enabled(bool on);
