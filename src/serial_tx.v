// Simple 2-wire serial interface
// Clocks out 8 bits of data serially
module serial_tx (
    input wire clk,
    input wire rst_n,

    // Control
    input  wire start,  // Start transmission
    output reg  busy,   // Transmission in progress
    output reg  done,   // Transmission complete

    // Data
    input wire [9:0] data_in,  // Byte to transmit

    // 2-wire interface
    output wire serial_clk,  // Clock output
    output wire serial_data  // Data output
);

    reg [3:0] bit_count;
    reg [9:0] shift_reg;

    localparam IDLE = 2'd0;
    localparam TRANSMIT = 2'd1;
    localparam DONE_STATE = 2'd2;

    reg [1:0] state;

    reg sclk;
    assign serial_clk = sclk;
    reg sdata;
    assign serial_data = sdata;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            busy <= 0;
            done <= 0;
            bit_count <= 0;
            shift_reg <= 10'd0;
            sclk <= 1;
            sdata <= 0;
        end
        else begin
            case (state)
                IDLE: begin
                    done <= 0;
                    if (start) begin
                        shift_reg <= data_in;
                        bit_count <= 0;
                        busy <= 1;
                        sclk <= 0;  // drop SCLK for 1 cycle to alert impending transaction
                        state <= TRANSMIT;
                    end
                    else begin
                        busy <= 0;
                        sclk <= 1;
                    end
                end

                TRANSMIT: begin
                    // Toggle clock and shift out data
                    sclk <= ~sclk;

                    if (sclk) begin
                        // On falling edge of serial_clk, advance to next bit
                        bit_count <= bit_count + 4'd1;
                        shift_reg <= {shift_reg[8:0], 1'b0};

                        if (bit_count == 4'd9) begin
                            state <= DONE_STATE;
                        end
                    end
                    else begin
                        // On rising edge of serial_clk, output current bit
                        sdata <= shift_reg[9];
                    end
                end

                DONE_STATE: begin
                    sclk  <= 0;
                    done  <= 1;
                    busy  <= 0;
                    state <= IDLE;
                end
                default: state <= IDLE;
            endcase
        end
    end
endmodule
