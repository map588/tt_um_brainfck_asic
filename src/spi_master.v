// SPI Master Controller for 23LC1024 or similar SPI RAM
// Optimized version: supports 5-byte transfers for cache management
// SPI Mode 0 or 3 - data transfer on rising edge of SCK

module spi_master (
    input wire clk,
    input wire rst_n,

    // SPI interface (master side)
    output wire spi_sck,
    output reg  spi_mosi,
    input  wire spi_miso,
    output reg  spi_cs,

    // Control interface
    input  wire        start_read,
    input  wire        start_write,
    input  wire [15:0] address,       // Base address
    input  wire [ 2:0] num_bytes,     // Number of bytes to transfer (1-5)
    input  wire [ 7:0] write_data,    // Single byte to write
    output reg  [ 7:0] read_data,     // Single byte read
    output reg         busy,
    output reg         byte_done,     // Single byte complete (pulse)
    output reg         transfer_done  // All bytes complete (pulse)
);

    // State machine
    localparam IDLE = 3'd0;
    localparam SEND_CMD = 3'd1;
    localparam SEND_ADDR_H = 3'd2;
    localparam SEND_ADDR_L = 3'd3;
    localparam WRITE_BYTES = 3'd4;
    localparam READ_BYTES = 3'd5;
    localparam FINISH = 3'd6;

    reg  [ 2:0] state;
    reg  [ 7:0] cmd_byte;
    reg  [ 7:0] shift_out;
    reg  [ 6:0] shift_in;
    reg  [ 3:0] bit_count;
    reg  [ 2:0] byte_count;  // 0-4 for up to 5 bytes
    reg  [ 2:0] target_bytes;  // Total bytes to transfer
    reg  [15:0] addr_reg;
    reg         is_write;

    // SPI clock divider
    reg  [ 2:0] clk_div;
    wire        sck_edge = (clk_div == 3'd3);
    reg         sck;
    assign spi_sck = sck;

    // SPI clock generation
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sck <= 1'b0;
        end
        else if (sck_edge) begin
            sck <= ~sck;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            spi_cs <= 1'b1;
            spi_mosi <= 1'b0;
            busy <= 1'b0;
            byte_done <= 1'b0;
            transfer_done <= 1'b0;
            bit_count <= 4'd0;
            byte_count <= 3'd0;
            target_bytes <= 3'd0;
            clk_div <= 3'd0;
            shift_out <= 8'd0;
            shift_in <= 7'd0;
            read_data <= 8'd0;
            addr_reg <= 16'd0;
            cmd_byte <= 8'd0;
            is_write <= 1'b0;
        end
        else begin
            clk_div <= clk_div + 3'd1;

            case (state)
                IDLE: begin
                    spi_cs <= 1'b1;
                    byte_done <= 1'b0;
                    transfer_done <= 1'b0;
                    busy <= 1'b0;

                    if (start_write) begin
                        cmd_byte <= 8'h02;
                        addr_reg <= address;
                        target_bytes <= num_bytes;
                        is_write <= 1'b1;
                        busy <= 1'b1;
                        spi_cs <= 1'b0;
                        state <= SEND_CMD;
                        bit_count <= 4'd0;
                        byte_count <= 3'd0;
                        clk_div <= 3'd0;
                    end
                    else if (start_read) begin
                        cmd_byte <= 8'h03;
                        addr_reg <= address;
                        target_bytes <= num_bytes;
                        is_write <= 1'b0;
                        busy <= 1'b1;
                        spi_cs <= 1'b0;
                        state <= SEND_CMD;
                        bit_count <= 4'd0;
                        byte_count <= 3'd0;
                        clk_div <= 3'd0;
                    end
                end

                SEND_CMD: begin
                    if (sck_edge) begin
                        if (sck) begin
                            bit_count <= bit_count + 4'd1;
                            if (bit_count == 4'd7) begin
                                state <= SEND_ADDR_H;
                                bit_count <= 4'd0;
                            end
                        end
                        else begin
                            spi_mosi <= cmd_byte[7-bit_count];
                        end
                    end
                end

                SEND_ADDR_H: begin
                    if (sck_edge) begin
                        if (sck) begin
                            bit_count <= bit_count + 4'd1;
                            if (bit_count == 4'd7) begin
                                state <= SEND_ADDR_L;
                                bit_count <= 4'd0;
                            end
                        end
                        else begin
                            spi_mosi <= addr_reg[15-bit_count];
                        end
                    end
                end

                SEND_ADDR_L: begin
                    if (sck_edge) begin
                        if (sck) begin
                            bit_count <= bit_count + 4'd1;
                            if (bit_count == 4'd7) begin
                                bit_count <= 4'd0;
                                if (is_write) begin
                                    state <= WRITE_BYTES;
                                end
                                else begin
                                    state <= READ_BYTES;
                                end
                            end
                        end
                        else begin
                            spi_mosi <= addr_reg[7-bit_count];
                        end
                    end
                end

                WRITE_BYTES: begin
                    if (byte_done) begin
                        byte_done <= 1'b0;
                    end

                    if (sck_edge) begin
                        if (~sck) begin
                            if (bit_count == 4'd0) begin
                                shift_out <= write_data;
                            end
                            spi_mosi <= shift_out[7-bit_count];
                        end
                        else begin
                            bit_count <= bit_count + 4'd1;
                            if (bit_count == 4'd7) begin
                                bit_count  <= 4'd0;
                                byte_count <= byte_count + 3'd1;
                                byte_done  <= 1'b1;

                                if (byte_count + 3'd1 == target_bytes) begin
                                    state <= FINISH;
                                end
                            end
                        end
                    end
                end

                READ_BYTES: begin
                    if (byte_done) begin
                        byte_done <= 1'b0;
                    end

                    if (sck_edge) begin
                        if (sck) begin
                            shift_in  <= {shift_in[6:1], spi_miso};
                            bit_count <= bit_count + 4'd1;
                            if (bit_count == 4'd7) begin
                                bit_count  <= 4'd0;
                                byte_count <= byte_count + 3'd1;
                                read_data  <= {shift_in, spi_miso};
                                byte_done  <= 1'b1;
                                if (byte_count + 3'd1 == target_bytes) begin
                                    state <= FINISH;
                                end
                            end
                        end
                    end
                end

                FINISH: begin
                    spi_cs <= 1'b1;
                    transfer_done <= 1'b1;
                    busy <= 1'b0;
                    state <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
