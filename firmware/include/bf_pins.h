/*
 * tt_um_brainfck_asic pin roles on the demo board GPIO map.
 * The map itself (TT_GPIO_*_BASE and friends) comes from the kit.
 * Design-side roles come from info.yaml / docs/bf_asic.md.
 */
#pragma once

#include "tt_pins.h"

#define BF_DESIGN_ADDR 448u /* tt_um_brainfck_asic mux address on       \
                               ttsky25b; ignored by the FPGA ASIC-sim */

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
