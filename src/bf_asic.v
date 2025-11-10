/*
 * TRUE 1-CYCLE EXECUTION WITH INSTRUCTION VALID HANDSHAKING
 * 
 * Instead of run_pause, uses instr_valid signal from MCU
 * - MCU pulses instr_valid HIGH for one cycle when instruction is ready
 * - ASIC executes instruction on that cycle
 * - No IDLE state - always ready in EXEC
 * - Interrupts tell MCU to stop pulsing instr_valid
 * 
 * FIXED: data_current now updates automatically when ptr moves within cache
 * 
 * COMPATIBILITY: Verilog-2005 synthesizable
 *   - Uses always @(*) for combinational logic
 *   - No functions (replaced with combinational always blocks)
 *   - Compatible with all major synthesis tools
 */

`define MINUS 3'b000
`define PLUS 3'b001
`define LEFT 3'b010
`define RIGHT 3'b011
`define OPEN 3'b100
`define CLOSE 3'b101
`define IN 3'b110
`define OUT 3'b111

module bf_asic (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [ 2:0] instruction,
    output wire [ 7:0] inspect_data,
    input  wire [ 1:0] inspect_sel,
    output wire        interrupt_jump,
    output wire        interrupt_io,
    input  wire        tx_done,
    input  wire        tx_busy,
    output reg         tx_start,
    output reg  [ 9:0] tx_data,
    input  wire [ 9:0] rx_data,
    input  wire        rx_done,
    output reg         rx_enable,
    output reg         spi_read,
    output reg         spi_write,
    input  wire        spi_busy,
    input  wire        spi_byte_done,
    input  wire        spi_transfer_done,
    output reg  [15:0] spi_base_addr,
    output reg  [ 7:0] spi_wdata,
    input  wire [ 7:0] spi_rdata,
    input  wire        instr_valid         // NEW: Pulse high when instruction ready
);
    // Data current and Tape
    reg [7:0] data_current;
    reg [7:0] l_tape_cache                                    [0:3];
    reg [7:0] r_tape_cache                                    [0:3];
    reg [9:0] ptr;
    reg [9:0] pc;
    reg [2:0] bstack_ptr;
    reg [9:0] bstack                                          [0:7];

    reg [9:0] tape_base;
    reg [2:0] spi_byte_idx;
    reg       tape_move_right;

    reg       irq_io;
    reg       irq_jump;  // Held HIGH for MCU jumps
    reg       irq_cache_pulse;  // Pulsed for cache start/stop

    // Interrupts: HOLD for MCU, PULSE for cache events
    assign interrupt_jump = irq_jump | irq_cache_pulse;
    assign interrupt_io   = irq_io;

    localparam STATE_EXEC = 3'd0;
    localparam STATE_SPI_WRITE = 3'd1;
    localparam STATE_SPI_FETCH = 3'd2;
    localparam STATE_WAIT_JUMP = 3'd3;
    localparam STATE_WAIT_IO = 3'd4;

    reg  [2:0] state;
    reg  [2:0] op_saved;  // For I/O-bound instructions

    wire [2:0] active_instr;
    assign active_instr = instruction;

    wire [2:0] op;

    assign op = (state == STATE_EXEC) ? active_instr : op_saved;
    wire       is_data_op = (op[2:1] == 2'b00);
    wire       is_ptr_op = (op[2:1] == 2'b01);
    wire       is_bstack_op = (op[2:1] == 2'b10);
    wire       is_io_op = (op[2:1] == 2'b11);

    // If it's sub, left, or in, thats just !add, !right, or !out
    wire       op_plus = is_data_op && op[0];
    wire       op_move_right = is_ptr_op && op[0];
    wire       op_output = is_io_op && op[0];

    // We do need to know if we jump forwards or backwards
    wire       op_openb = is_bstack_op && !op[0];  // [ = 0b100
    wire       op_closeb = is_bstack_op && op[0];  // ] = 0b101

    // Inspection MUX
    reg  [7:0] output_data;

    always @(*) begin
        case (inspect_sel)
            2'b00:   output_data = data_current;
            2'b01:   output_data = ptr[7:0];
            2'b10:   output_data = pc[7:0];
            2'b11:   output_data = (bstack_ptr > 0) ? bstack[bstack_ptr - 3'd1][7:0] : 8'h00;
            default: output_data = 8'h00;
        endcase
    end

    assign inspect_data = output_data;

    // ========== OFFSET CALCULATION AND CACHE LOOKUP ==========

    // Pointer advancement with boundary blocking
    reg [9:0] ptr_next;

    always @(*) begin
        if (is_ptr_op) begin
            if (op_move_right) begin
                if (ptr == 10'd1023) begin
                    ptr_next = ptr;  // Block at 1023
                end
                else begin
                    ptr_next = ptr + 10'd1;
                end
            end
            else begin
                if (ptr == 10'd0) begin
                    ptr_next = ptr;  // Block at 0
                end
                else begin
                    ptr_next = ptr - 10'd1;
                end
            end
        end
        else begin
            ptr_next = ptr;
        end
    end

    wire signed [10:0] ptr_offset_current = $signed({1'b0, ptr}) - $signed({1'b0, tape_base});
    wire signed [10:0] ptr_offset_next = $signed({1'b0, ptr_next}) - $signed({1'b0, tape_base});

    wire [9:0] tape_min = (tape_base < 10'd4) ? 10'd0 : (tape_base - 10'd4);
    wire [9:0] tape_max = tape_base + 10'd4;

    wire need_data_low_next = (ptr_next < tape_min) && (tape_base >= 10'd4);
    wire need_data_high_next = (ptr_next > tape_max);
    wire need_spi_fetch_next = need_data_low_next || need_data_high_next;

    reg [7:0] data_at_next_ptr;

    wire data_zero = (data_current == 8'h0);

    always @(*) begin
        case (ptr_offset_next)
            -4: data_at_next_ptr = l_tape_cache[0];
            -3: data_at_next_ptr = l_tape_cache[1];
            -2: data_at_next_ptr = l_tape_cache[2];
            -1: data_at_next_ptr = l_tape_cache[3];
            0: data_at_next_ptr = data_current;
            1: data_at_next_ptr = r_tape_cache[0];
            2: data_at_next_ptr = r_tape_cache[1];
            3: data_at_next_ptr = r_tape_cache[2];
            4: data_at_next_ptr = r_tape_cache[3];
            default: data_at_next_ptr = 8'h00;
        endcase
    end

    // ========== END OFFSET SECTION ==========

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= STATE_EXEC;
            op_saved <= 3'd0;
            data_current <= 8'd0;
            l_tape_cache[0] <= 8'd0;
            l_tape_cache[1] <= 8'd0;
            l_tape_cache[2] <= 8'd0;
            l_tape_cache[3] <= 8'd0;
            r_tape_cache[0] <= 8'd0;
            r_tape_cache[1] <= 8'd0;
            r_tape_cache[2] <= 8'd0;
            r_tape_cache[3] <= 8'd0;
            tape_base <= 10'd4;
            ptr <= 10'd0;
            pc <= 10'd0;
            bstack_ptr <= 3'd0;
            bstack[0] <= 10'd0;
            bstack[1] <= 10'd0;
            bstack[2] <= 10'd0;
            bstack[3] <= 10'd0;
            bstack[4] <= 10'd0;
            bstack[5] <= 10'd0;
            bstack[6] <= 10'd0;
            bstack[7] <= 10'd0;
            irq_jump <= 1'b0;
            irq_io <= 1'b0;
            irq_cache_pulse <= 1'b0;
            spi_read <= 1'b0;
            spi_write <= 1'b0;
            spi_base_addr <= 16'd0;
            spi_wdata <= 8'd0;
            spi_byte_idx <= 3'd0;
            tape_move_right <= 1'b0;
            tx_start <= 1'b0;
            tx_data <= 10'd0;
            rx_enable <= 1'b0;
        end
        else begin
            // Default: clear pulse signals each cycle
            irq_cache_pulse <= 1'b0;

            case (state)
                STATE_EXEC: begin
                    // NEW: Only execute when instr_valid is HIGH
                    if (instr_valid) begin
                        if (is_data_op) begin
                            data_current <= op_plus ? (data_current + 8'd1) : (data_current - 8'd1);
                            pc <= pc + 10'd1;
                        end
                        else if (is_ptr_op) begin
                            // Save current data_current to cache before moving
                            case (ptr_offset_current)
                                -4: l_tape_cache[0] <= data_current;
                                -3: l_tape_cache[1] <= data_current;
                                -2: l_tape_cache[2] <= data_current;
                                -1: l_tape_cache[3] <= data_current;
                                0: ;  // Already in data_current
                                1: r_tape_cache[0] <= data_current;
                                2: r_tape_cache[1] <= data_current;
                                3: r_tape_cache[2] <= data_current;
                                4: r_tape_cache[3] <= data_current;
                                default: ;
                            endcase

                            ptr <= ptr_next;
                            if (need_spi_fetch_next) begin
                                op_saved <= instruction;  // Store instruction, because it will be a while
                                if (need_data_high_next) begin
                                    tape_move_right <= 1'b1;  // Moving RIGHT
                                end
                                else begin
                                    tape_move_right <= 1'b0;  // Moving LEFT
                                end
                                state <= STATE_SPI_WRITE;
                            end
                            else begin
                                // Load data_current from cache at new position
                                data_current <= data_at_next_ptr;
                                pc <= pc + 10'd1;
                            end
                        end
                        else if (op_openb) begin
                            if (data_zero) begin
                                // NEED TO JUMP > FORWARD, WE DON'T KNOW WHERE
                                op_saved <= instruction;
                                irq_jump <= 1'b1;
                                state <= STATE_WAIT_JUMP;
                            end
                            else begin
                                // PUSH BRACKET - CONTINUE
                                if (bstack_ptr < 3'd7) begin  // Only push if stack not full
                                    bstack[bstack_ptr] <= pc;
                                    bstack_ptr <= bstack_ptr + 3'd1;
                                end
                                pc <= pc + 10'd1;  // Always advance PC
                            end
                        end
                        else if (op_closeb) begin
                            if (data_zero) begin
                                // POP BRACKET - CONTINUE
                                if (bstack_ptr > 3'd0) begin
                                    bstack_ptr <= bstack_ptr - 3'd1;
                                end
                                pc <= pc + 10'd1;
                            end
                            else begin
                                // NEED TO JUMP < BACK, WE KNOW WHERE, BUT NOT WHAT
                                op_saved <= instruction;
                                irq_jump <= 1'b1;
                                state <= STATE_WAIT_JUMP;
                            end
                        end
                        else if (is_io_op) begin
                            op_saved <= instruction;
                            irq_io <= 1'b1;
                            state <= STATE_WAIT_IO;
                        end
                    end
                    // If instr_valid is LOW, just wait (no state change)
                end

                STATE_WAIT_JUMP: begin
                    if (op_saved == `CLOSE) begin // ']'
                       // If we haven't started transmitting
                        if (!tx_busy && !tx_done) begin
                            // Only pop if stack is not empty
                            if (bstack_ptr > 0) begin
                                tx_data <= bstack[bstack_ptr - 3'd1];
                                pc <= bstack[bstack_ptr - 3'd1];  // Update PC to jump target
                                bstack_ptr <= bstack_ptr - 3'd1;
                                tx_start <= 1'b1;
                            end
                            // else: stack is empty, do not pop, could set error/irq here if desired
                        end
                        else if (tx_done) begin
                            tx_start <= 1'b0;
                            irq_jump <= 1'b0;
                            state <= STATE_EXEC;
                        end
                    end
                    else begin  // '['
                        if (!rx_enable) begin
                            if (bstack_ptr < 3'd7) begin  // Only push if stack not full
                                bstack[bstack_ptr] <= pc;
                                bstack_ptr <= bstack_ptr + 3'd1;
                            end
                            rx_enable <= 1'b1;
                        end
                        else if (rx_done) begin
                            pc <= rx_data;
                            rx_enable <= 1'b0;
                            irq_jump <= 1'b0;
                            state <= STATE_EXEC;
                        end
                    end
                end

                STATE_WAIT_IO: begin
                    if (op_output) begin
                        if (!tx_busy && !tx_done) begin
                            tx_data  <= {2'b00, data_current};
                            tx_start <= 1'b1;
                        end
                        else if (tx_done) begin
                            tx_start <= 1'b0;
                            irq_io <= 1'b0;
                            pc <= pc + 10'd1;
                            state <= STATE_EXEC;
                        end
                    end
                    else begin
                        if (!rx_enable) begin
                            rx_enable <= 1'b1;
                        end
                        else if (rx_done) begin
                            data_current <= rx_data[7:0];
                            rx_enable <= 1'b0;
                            irq_io <= 1'b0;
                            pc <= pc + 10'd1;
                            state <= STATE_EXEC;
                        end
                    end
                end

                STATE_SPI_WRITE: begin
                    if (!spi_busy && !spi_write && !spi_read) begin
                        spi_byte_idx <= 3'd0;
                        spi_write <= 1'b1;
                        if (tape_move_right) begin
                            spi_base_addr <= {6'd0, tape_base} - 16'd4;
                            spi_wdata <= l_tape_cache[0];
                        end
                        else begin
                            spi_base_addr <= {6'd0, tape_base};
                            spi_wdata <= data_current;
                        end
                    end
                    else if (spi_write && spi_busy) begin
                        spi_write <= 1'b0;
                    end
                    else if (spi_byte_done && spi_busy) begin
                        spi_byte_idx <= spi_byte_idx + 3'd1;
                        if (tape_move_right) begin
                            case (spi_byte_idx + 3'd1)
                                3'd1: spi_wdata <= l_tape_cache[1];
                                3'd2: spi_wdata <= l_tape_cache[2];
                                3'd3: spi_wdata <= l_tape_cache[3];
                                3'd4: spi_wdata <= data_current;
                                default: spi_wdata <= 8'd0;
                            endcase
                        end
                        else begin
                            case (spi_byte_idx + 3'd1)
                                3'd1: spi_wdata <= r_tape_cache[0];
                                3'd2: spi_wdata <= r_tape_cache[1];
                                3'd3: spi_wdata <= r_tape_cache[2];
                                3'd4: spi_wdata <= r_tape_cache[3];
                                default: spi_wdata <= 8'd0;
                            endcase
                        end
                    end
                    else if (spi_transfer_done) begin
                        state <= STATE_SPI_FETCH;
                    end
                end

                STATE_SPI_FETCH: begin
                    if (!spi_busy && !spi_write && !spi_read) begin
                        spi_byte_idx <= 3'd0;
                        spi_read <= 1'b1;
                        if (tape_move_right) begin
                            spi_base_addr <= {6'd0, ptr};
                        end
                        else begin
                            // When moving left, read from [ptr-4, ptr-3, ptr-2, ptr-1, ptr]
                            // Clamp to prevent address underflow when ptr < 4
                            // If ptr < 4, read from address 0 instead
                            spi_base_addr <= (ptr < 10'd4) ? 16'd0 : ({6'd0, ptr} - 16'd4);
                        end
                    end
                    else if (spi_read && spi_busy) begin
                        spi_read <= 1'b0;
                    end
                    else if (spi_byte_done && spi_busy) begin
                        if (tape_move_right) begin
                            case (spi_byte_idx)
                                3'd0: data_current <= spi_rdata;
                                3'd1: r_tape_cache[0] <= spi_rdata;
                                3'd2: r_tape_cache[1] <= spi_rdata;
                                3'd3: r_tape_cache[2] <= spi_rdata;
                                3'd4: r_tape_cache[3] <= spi_rdata;
                                default: ;
                            endcase
                        end
                        else begin
                            case (spi_byte_idx)
                                3'd0: l_tape_cache[0] <= spi_rdata;
                                3'd1: l_tape_cache[1] <= spi_rdata;
                                3'd2: l_tape_cache[2] <= spi_rdata;
                                3'd3: l_tape_cache[3] <= spi_rdata;
                                3'd4: data_current <= spi_rdata;
                                default: ;
                            endcase
                        end
                        spi_byte_idx <= spi_byte_idx + 3'd1;
                    end
                    else if (spi_transfer_done) begin
                        if (tape_move_right) begin
                            l_tape_cache[0] <= r_tape_cache[0];
                            l_tape_cache[1] <= r_tape_cache[1];
                            l_tape_cache[2] <= r_tape_cache[2];
                            l_tape_cache[3] <= r_tape_cache[3];
                        end
                        else begin
                            r_tape_cache[0] <= l_tape_cache[0];
                            r_tape_cache[1] <= l_tape_cache[1];
                            r_tape_cache[2] <= l_tape_cache[2];
                            r_tape_cache[3] <= l_tape_cache[3];
                        end
                        // Update cache center to new pointer position
                        // Note: SPI read address is clamped separately to prevent underflow
                        tape_base <= ptr;
                        pc <= pc + 10'd1;
                        state <= STATE_EXEC;
                    end
                end

                default: state <= STATE_EXEC;  // NEW: Default to EXEC
            endcase
        end
    end
endmodule
