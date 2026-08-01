#pragma once

/* Board-level pin control shared by both firmware targets. */

#define BF_DESIGN_ADDR 448u /* tt_um_brainfck_asic mux address on       \
                               ttsky25b; ignored by the FPGA ASIC-sim */

/* One-time init of mux control, project reset, and LED pins; ends in
 * the safe profile. */
void board_pins_init(void);

/* Release everything a foreign design could drive: uio pads become
 * inputs, ui pads (always ASIC inputs) are driven 0. */
void pins_safe(void);

/* BF pin roles on top of the safe profile; enables the core-1 SPI RAM. */
void pins_bf(void);

/* Walk the project mux to design n and enable it. */
void tt_select_design(unsigned n);
