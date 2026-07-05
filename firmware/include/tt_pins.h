/*
 * GPIO map: Tiny Tapeout demo board v3 (RP2350B) <-> tt_um_brainfck_asic pins.
 *
 * Board-side numbers come from TinyTapeout/tt-micropython-firmware
 * (src/ttboard/pins/gpio_map_dbv3.py, class GPIOMapTTDBv3).
 * Define TT_DBV3_ALPHA when building for the v3 "Alpha" prototype board,
 * which uses a different set of GPIOs.
 *
 * Design-side roles come from info.yaml / docs/bf_asic.md.
 */
#pragma once

#ifdef TT_DBV3_ALPHA
#define TT_GPIO_UI_BASE 12 /* ui_in[0..7]  = GPIO 12..19 */
#define TT_GPIO_UO_BASE 30 /* uo_out[0..7] = GPIO 30..37 */
#define TT_GPIO_UIO_BASE 22 /* uio[0..7]   = GPIO 22..29 */
#define TT_PIN_PROJ_CLK 21
#define TT_PIN_PROJ_NRST 20
#else
#define TT_GPIO_UI_BASE 17 /* ui_in[0..7]  = GPIO 17..24 */
#define TT_GPIO_UO_BASE 33 /* uo_out[0..7] = GPIO 33..40 */
#define TT_GPIO_UIO_BASE 25 /* uio[0..7]   = GPIO 25..32 */
#define TT_PIN_PROJ_CLK 16
#define TT_PIN_PROJ_NRST 14
#endif

/* Project mux control (same on both board revisions). */
#define TT_PIN_CTRL_ENA 0
#define TT_PIN_CTRL_NRST 1
#define TT_PIN_CTRL_INC 2

#define TT_PIN_LED 11

/* ---- ui_in: MCU outputs, ASIC inputs ---- */
#define PIN_INSTR0 (TT_GPIO_UI_BASE + 0)
#define PIN_INSTR1 (TT_GPIO_UI_BASE + 1)
#define PIN_INSTR2 (TT_GPIO_UI_BASE + 2)
#define PIN_INSTR_VALID (TT_GPIO_UI_BASE + 3)
#define PIN_RX_CLK (TT_GPIO_UI_BASE + 4) /* MCU->ASIC serial clock */
#define PIN_RX_BIT (TT_GPIO_UI_BASE + 5) /* MCU->ASIC serial data  */
#define PIN_SEL0 (TT_GPIO_UI_BASE + 6)   /* inspect_sel[0] */
#define PIN_SEL1 (TT_GPIO_UI_BASE + 7)   /* inspect_sel[1] */

/* ---- uo_out: ASIC outputs, MCU inputs ---- */
#define PIN_TX_BIT (TT_GPIO_UO_BASE + 0) /* ASIC->MCU serial data  */
#define PIN_TX_CLK (TT_GPIO_UO_BASE + 1) /* ASIC->MCU serial clock (idle high) */
#define PIN_IRQ_IO (TT_GPIO_UO_BASE + 2)
#define PIN_IRQ_JUMP (TT_GPIO_UO_BASE + 3)
/* uo_out[7:4] = inspect_data[3:0] */

/* ---- uio: SPI RAM link (ASIC is master) + inspect high nibble ---- */
#define PIN_SPI_CS (TT_GPIO_UIO_BASE + 0)   /* input  */
#define PIN_SPI_MOSI (TT_GPIO_UIO_BASE + 1) /* input  */
#define PIN_SPI_MISO (TT_GPIO_UIO_BASE + 2) /* OUTPUT */
#define PIN_SPI_SCK (TT_GPIO_UIO_BASE + 3)  /* input  */
/* uio[7:4] = inspect_data[7:4] */

/* inspect_sel values (bf_asic inspection mux) */
#define INSPECT_DATA 0u
#define INSPECT_PTR 1u
#define INSPECT_PC 2u
#define INSPECT_BSTACK 3u
