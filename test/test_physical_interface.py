"""
Test the physical TinyTapeout interface (ui_in pins) directly.

This test verifies:
1. Instructions can be fed via ui_in[2:0]
2. instr_valid handshaking via ui_in[3]
3. Inspect data readback via uo_out[7:4] with ui_in[7:6] select
4. Interrupts on uo_out[3:2]

This simulates how the actual MCU would interact with the ASIC.
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge

# Instruction encodings
INSTR_SUB    = 0b000  # -
INSTR_ADD    = 0b001  # +
INSTR_LEFT   = 0b010  # <
INSTR_RIGHT  = 0b011  # >
INSTR_OPENB  = 0b100  # [
INSTR_CLOSEB = 0b101  # ]
INSTR_IN     = 0b110  # ,
INSTR_OUT    = 0b111  # .

# Inspect select values
INSPECT_DATA  = 0b00
INSPECT_PTR   = 0b01
INSPECT_PC    = 0b10
INSPECT_STACK = 0b11


def set_ui_in(dut, instruction, instr_valid, inspect_sel):
    """Set ui_in pins directly."""
    ui_in_value = (
        (instruction & 0b111) |           # ui_in[2:0]
        ((instr_valid & 0b1) << 3) |      # ui_in[3]
        ((inspect_sel & 0b11) << 6)       # ui_in[7:6]
    )
    # ui_in[5:4] are for serial RX (not used in this test)
    dut.ui_in.value = ui_in_value


def get_inspect_data(dut):
    """Read inspect data from uo_out[7:4]."""
    uo_out = int(dut.uo_out.value)
    return (uo_out >> 4) & 0xF  # Upper nibble


def get_interrupts(dut):
    """Read interrupt flags from uo_out[3:2]."""
    uo_out = int(dut.uo_out.value)
    interrupt_jump = (uo_out >> 3) & 0x1
    interrupt_io = (uo_out >> 2) & 0x1
    return interrupt_jump, interrupt_io


async def pulse_instruction(dut, instruction):
    """Send a single instruction with proper handshaking."""
    # Set instruction and pulse instr_valid for one cycle
    set_ui_in(dut, instruction, 1, INSPECT_DATA)
    await RisingEdge(dut.clk)
    set_ui_in(dut, instruction, 0, INSPECT_DATA)
    await RisingEdge(dut.clk)


async def read_full_byte(dut, inspect_sel):
    """Read full 8-bit value by reading both nibbles."""
    # Read lower nibble
    set_ui_in(dut, 0, 0, inspect_sel)
    await ClockCycles(dut.clk, 1)
    low_nibble = get_inspect_data(dut)

    # For now, just return low nibble (full byte reading would need
    # additional logic or two inspect operations)
    return low_nibble


@cocotb.test()
async def test_physical_interface(dut):
    """Test direct ui_in pin interface."""

    # Start clock
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())

    # Reset
    dut.rst_n.value = 0
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)

    dut._log.info("========================================")
    dut._log.info("PHYSICAL INTERFACE TEST")
    dut._log.info("========================================")

    errors = 0

    # ===== TEST 1: Basic Instruction Execution via ui_in =====
    dut._log.info("\nTest 1: Execute ADD via ui_in[2:0]")

    # Read initial data (should be 0)
    set_ui_in(dut, 0, 0, INSPECT_DATA)
    await ClockCycles(dut.clk, 1)
    data_before = get_inspect_data(dut)
    dut._log.info(f"  Data before: 0x{data_before:X}")

    # Execute ADD instruction via ui_in
    await pulse_instruction(dut, INSTR_ADD)

    # Read data after
    set_ui_in(dut, 0, 0, INSPECT_DATA)
    await ClockCycles(dut.clk, 1)
    data_after = get_inspect_data(dut)
    dut._log.info(f"  Data after ADD: 0x{data_after:X}")

    if data_after == ((data_before + 1) & 0xF):
        dut._log.info("  PASS: ADD executed correctly via ui_in")
    else:
        dut._log.error(f"  FAIL: Expected 0x{(data_before + 1) & 0xF:X}, got 0x{data_after:X}")
        errors += 1

    # ===== TEST 2: Multiple Instructions =====
    dut._log.info("\nTest 2: Execute multiple instructions")

    await pulse_instruction(dut, INSTR_ADD)
    await pulse_instruction(dut, INSTR_ADD)
    await pulse_instruction(dut, INSTR_ADD)

    set_ui_in(dut, 0, 0, INSPECT_DATA)
    await ClockCycles(dut.clk, 1)
    data_after_3 = get_inspect_data(dut)
    dut._log.info(f"  Data after 3 more ADDs: 0x{data_after_3:X}")

    expected = (data_after + 3) & 0xF
    if data_after_3 == expected:
        dut._log.info(f"  PASS: Multiple instructions executed correctly")
    else:
        dut._log.error(f"  FAIL: Expected 0x{expected:X}, got 0x{data_after_3:X}")
        errors += 1

    # ===== TEST 3: SUB Instruction =====
    dut._log.info("\nTest 3: Execute SUB via ui_in")

    await pulse_instruction(dut, INSTR_SUB)

    set_ui_in(dut, 0, 0, INSPECT_DATA)
    await ClockCycles(dut.clk, 1)
    data_after_sub = get_inspect_data(dut)
    dut._log.info(f"  Data after SUB: 0x{data_after_sub:X}")

    expected_sub = (data_after_3 - 1) & 0xF
    if data_after_sub == expected_sub:
        dut._log.info("  PASS: SUB executed correctly")
    else:
        dut._log.error(f"  FAIL: Expected 0x{expected_sub:X}, got 0x{data_after_sub:X}")
        errors += 1

    # ===== TEST 4: Inspect PC via ui_in[7:6] =====
    dut._log.info("\nTest 4: Read PC via inspect_sel")

    # Select PC view
    set_ui_in(dut, 0, 0, INSPECT_PC)
    await ClockCycles(dut.clk, 1)
    pc_low = get_inspect_data(dut)
    dut._log.info(f"  PC (low nibble): 0x{pc_low:X}")

    # We've executed 5 instructions, so PC should be 5
    if pc_low == 5:
        dut._log.info("  PASS: PC incremented correctly")
    else:
        dut._log.error(f"  FAIL: Expected PC=5, got 0x{pc_low:X}")
        errors += 1

    # ===== TEST 5: No execution without instr_valid =====
    dut._log.info("\nTest 5: Verify no execution without instr_valid pulse")

    # Set instruction but don't pulse instr_valid
    set_ui_in(dut, INSTR_ADD, 0, INSPECT_DATA)
    await ClockCycles(dut.clk, 3)

    set_ui_in(dut, 0, 0, INSPECT_DATA)
    await ClockCycles(dut.clk, 1)
    data_no_exec = get_inspect_data(dut)

    if data_no_exec == data_after_sub:
        dut._log.info("  PASS: No execution without instr_valid")
    else:
        dut._log.error(f"  FAIL: Data changed without instr_valid")
        errors += 1

    # ===== TEST 6: Pointer Movement =====
    dut._log.info("\nTest 6: Pointer movement via ui_in")

    # Read initial pointer
    set_ui_in(dut, 0, 0, INSPECT_PTR)
    await ClockCycles(dut.clk, 1)
    ptr_before = get_inspect_data(dut)
    dut._log.info(f"  Pointer before: 0x{ptr_before:X}")

    # Move right
    await pulse_instruction(dut, INSTR_RIGHT)

    set_ui_in(dut, 0, 0, INSPECT_PTR)
    await ClockCycles(dut.clk, 1)
    ptr_after = get_inspect_data(dut)
    dut._log.info(f"  Pointer after >: 0x{ptr_after:X}")

    if ptr_after == ((ptr_before + 1) & 0xF):
        dut._log.info("  PASS: Pointer moved right")
    else:
        dut._log.error(f"  FAIL: Pointer movement failed")
        errors += 1

    # ===== TEST 7: Interrupt Detection =====
    dut._log.info("\nTest 7: Interrupt detection via uo_out[3:2]")

    # Set data to 0
    set_ui_in(dut, 0, 0, INSPECT_DATA)
    await ClockCycles(dut.clk, 1)
    current_data = get_inspect_data(dut)
    for _ in range(current_data):
        await pulse_instruction(dut, INSTR_SUB)

    # Execute [ with data=0, should trigger interrupt
    await pulse_instruction(dut, INSTR_OPENB)
    await ClockCycles(dut.clk, 2)

    interrupt_jump, interrupt_io = get_interrupts(dut)
    dut._log.info(f"  interrupt_jump: {interrupt_jump}, interrupt_io: {interrupt_io}")

    if interrupt_jump == 1:
        dut._log.info("  PASS: interrupt_jump asserted for [ with data=0")
    else:
        dut._log.error("  FAIL: interrupt_jump not asserted")
        errors += 1

    # ===== SUMMARY =====
    dut._log.info("\n========================================")
    dut._log.info("TEST SUMMARY")
    dut._log.info(f"Errors: {errors}")
    dut._log.info("========================================")

    if errors > 0:
        raise AssertionError(f"{errors} test(s) failed")
    else:
        dut._log.info("ALL TESTS PASSED!")
