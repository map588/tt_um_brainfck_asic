#pragma once

/* One-time init of the board-level pins (mux control, project reset,
 * LED) and the safe ui/uo/uio profile. */
void board_pins_init(void);

/* Read and execute protocol commands from USB CDC. Does not return. */
void command_loop(void);
