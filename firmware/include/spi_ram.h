#pragma once

/* Core 1 entry point: bit-banged SPI RAM slave backing the BF tape. */
void spi_ram_core1_entry(void);

/* Zero the tape. Only call from core 0 with the ASIC in reset. */
void spi_ram_clear(void);
