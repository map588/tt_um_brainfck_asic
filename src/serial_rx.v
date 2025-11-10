// Serial 2-wire receiver for input operation Microcontroller generates clock and sends data
// ASIC receives data on clock edges
module serial_rx (
    input wire clk,
    input wire rst_n,

    // Control
    input  wire enable,  // Enable receiver
    output wire done,    // Byte received
    // Data
    output wire [9:0] data_out,  // Received byte

    // 2-wire interface (both inputs)
    input wire serial_clk,  // Clock from microcontroller
    input wire serial_data  // Data from microcontroller
);

  reg [3:0] bit_count;
  reg [9:0] shift_in;
  reg done_reg;
  reg sclk;
  reg sclk_prev;

  assign data_out = done_reg ? shift_in : 10'd0;
  assign done = done_reg;

  // Proper two-flop synchronizer - both registers clocked by system clock
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      sclk <= 1'b0;
      sclk_prev <= 1'b0;
    end else begin
      sclk <= serial_clk;      // First flop - sync to clk domain
      sclk_prev <= sclk;       // Second flop - capture previous value
    end
  end

  wire sclk_rising = sclk && !sclk_prev;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      bit_count <= 4'd0;
      shift_in  <= 10'd0;
      done_reg  <= 1'b0;
    end else begin
    if (!enable) begin
      bit_count <= 4'd0;
      done_reg  <= 1'b0;
      end else if (sclk_rising) begin
        // Sample data on rising edge of serial_clk
        shift_in  <= {shift_in[8:0], serial_data};
        bit_count <= bit_count + 4'd1;

        if (bit_count == 4'd9) begin
          // Byte complete (shift_in already has the last bit from line above)
          done_reg  <= 1'b1;
          bit_count <= 4'd0;
        end else begin
          // Clear done when starting a new byte
          if (bit_count == 4'd0 && done_reg) begin
            done_reg <= 1'b0;
          end
        end
      end
    end
  end

endmodule
