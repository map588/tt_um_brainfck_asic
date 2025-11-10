/*
 * Copyright (c) 2024 Your Name
 * SPDX-License-Identifier: Apache-2.0
 */

//`default_nettype none

module tt_um_brainfck_asic (
    input wire [7:0] ui_in,  // Dedicated inputs
    output wire [7:0] uo_out,  // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path (only uio_in[2] used as spi_miso; tie unused pins off at top-level)
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,  // IOs: Enable path (active high: 0=input, 1=output)
    input wire ena,  // always 1 when the design is powered, so you can ignore it
    input wire clk,  // clock
    input wire rst_n  // rst_n - low to reset
);
  // INSTRUCTION IN
  wire [2:0] instruction;
  assign instruction[2] = ui_in[2];
  assign instruction[1] = ui_in[1];
  assign instruction[0] = ui_in[0];
  wire run_instr = ui_in[3];

  // SERIAL IN
  wire rx_clk = ui_in[4];
  wire rx_bit = ui_in[5];

  // SERIAL OUT
  wire tx_clk;
  wire tx_bit;
  assign uo_out[1] = tx_clk;
  assign uo_out[0] = tx_bit;

  // USER OUTPUT SELECT
  wire Sel_0 = ui_in[6];
  wire Sel_1 = ui_in[7];
  wire [1:0] inspect_sel = {Sel_1, Sel_0};

  // BUSMUX OUT
  wire [7:0] inspect_data;

  // Output bus split between uo and uio
  assign uo_out[7:4] = inspect_data[3:0];
  assign uio_out[7:4] = inspect_data[7:4];

  // Wire declarations for interrupts (must be before use)
  wire interrupt_jump;  // Jump interrupt to microcontroller
  wire interrupt_io;    // I/O interrupt to microcontroller

  // Interrupt for I/O operation or instruction jump.
  assign uo_out[3] = interrupt_jump;
  assign uo_out[2] = interrupt_io;

  // Suppress unused signal warnings
  wire _unused_1;
  wire _unused_2;
  wire _unused_ena;

  assign _unused_1   = &{uio_in[7:3], 1'b0};
  assign _unused_2   = &{uio_in[1:0], 1'b0};
  assign _unused_ena = &{ena, 1'b1};

  // SPI MISO is uio_in[2], so this is unused
  assign uio_out[2]  = 1'b0;


  // uio_out[7:0] - Bidirectional output path
  wire spi_sck;  // SPI clock to external RAM
  wire spi_mosi;  // SPI MOSI to external RAM
  wire spi_miso;  // SPI MISO from external RAM
  wire spi_cs;  // SPI chip select to external RAM



  assign uio_out[0] = spi_cs;
  assign uio_out[1] = spi_mosi;
  assign spi_miso = uio_in[2];
  assign uio_out[3] = spi_sck;

  // 0 = input, 1 = output
  //                      ____ spi_miso
  //                      |
  assign uio_oe = 8'b11111011;

  // ========== INTERMEDIATE SIGNALS (bf_asic <-> spi_master) ==========

  // ==================  SPI RAM ACCESS PATTERN  =======================

  // Internal Cache is 9 bytes:     | X X X X P X X X X |   We always start in the middle
  // and end up on either side:     | X X X X X X X X X | P 
  //                              P | X X X X X X X X X | 

  // So we will transfer always:    | T T T T T X X X X | P
  //                         or:  P | X X X X T T T T T |

  // transfer:      {l_cache[3:0], base_byte} or {base_byte , r_cache[3:0]}
  // receive:         [baseptr : base_ptr+4]  or  [baseptr-4 : baseptr]

  // 5 out, 5 in, every time

  wire [2:0] spi_num_bytes = 3'd5;

  // SPI control signals: bf_asic → spi_master
  wire spi_start_read;  // bf_asic.spi_read → spi_master.start_read
  wire spi_start_write;  // bf_asic.spi_write → spi_master.start_write
  wire [15:0] spi_address;  // bf_asic.spi_base_addr → spi_master.address
  wire [7:0] spi_write_data;  // bf_asic.spi_wdata → spi_master.write_data

  // SPI status/data signals: spi_master → bf_asic
  wire [7:0] spi_read_data;  // spi_master.read_data → bf_asic.spi_rdata
  wire spi_busy_signal;  // spi_master.busy → bf_asic.spi_busy
  wire        spi_byte_done_signal;  // spi_master.byte_done → bf_asic  -- The read is terminated by stopping the SCK and raising CS.
  wire spi_xfer_done;  // spi_master.transfer_done → bf_asic.spi_transfer_done

  // ========== SERIAL SIGNALS (bf_asic <-> serial_tx/rx) ========== 

  // Transmitter for jump and output operations (drives inout pins)
  wire tx_start;
  wire tx_busy;
  wire tx_done;
  wire [9:0] tx_data;


  // Receiver for input operation (reads from input pins)
  wire rx_enable;
  wire rx_done;
  wire [9:0] rx_data;

  // ========== MODULE INSTANTIATIONS ==========

  // Brainfuck ASIC core
  bf_asic bf_core (
      // Clock and reset (external in)
      .clk  (clk),
      .rst_n(rst_n),

      // Instruction input and run (external in)
      .instruction(instruction),
      .instr_valid(run_instr),

      // Interrupts (external out)
      .interrupt_jump(interrupt_jump),
      .interrupt_io  (interrupt_io),

      // UART/Serial TX interface (internal)
      .tx_start(tx_start),
      .tx_data (tx_data),
      .tx_done (tx_done),
      .tx_busy (tx_busy),

      // UART/Serial RX interface (internal)
      .rx_data  (rx_data),
      .rx_enable(rx_enable),
      .rx_done  (rx_done),
      // SPI control interface (internal) 
      .spi_read(spi_start_read),
      .spi_write(spi_start_write),
      .spi_busy(spi_busy_signal),
      .spi_byte_done(spi_byte_done_signal),
      .spi_base_addr(spi_address),
      .spi_wdata(spi_write_data),
      .spi_rdata(spi_read_data),
      .spi_transfer_done(spi_xfer_done),

      // Output Control (external in)
      .inspect_data(inspect_data),
      .inspect_sel (inspect_sel)
  );

  // SPI Master controller
  spi_master spi (
      // Clock and reset
      .clk  (clk),
      .rst_n(rst_n),

      // SPI physical interface
      .spi_sck (spi_sck),
      .spi_mosi(spi_mosi),
      .spi_miso(spi_miso),
      .spi_cs  (spi_cs),

      // Control interface (from bf_asic)
      .start_read(spi_start_read),
      .start_write(spi_start_write),
      .address(spi_address),
      .num_bytes(spi_num_bytes),  // Always 5 bytes for cache operations
      .write_data(spi_write_data),
      .read_data(spi_read_data),
      .busy(spi_busy_signal),
      .byte_done(spi_byte_done_signal),  // Registers byte into cache
      .transfer_done(spi_xfer_done)
  );
  // ========== 2-WIRE INTERFACES ==========

  serial_tx tx (
      .clk(clk),
      .rst_n(rst_n),
      .start(tx_start),
      .busy(tx_busy),
      .done(tx_done),
      .data_in(tx_data),
      .serial_clk(tx_clk),
      .serial_data(tx_bit)
  );

  serial_rx rx (
      .clk(clk),
      .rst_n(rst_n),
      .enable(rx_enable),
      .done(rx_done),
      .data_out(rx_data),
      .serial_clk(rx_clk),
      .serial_data(rx_bit)
  );

endmodule
