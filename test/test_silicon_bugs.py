# SPDX-License-Identifier: Apache-2.0
"""Correctness tests for the four RTL bugs found on ttsky25b silicon
(2026-08-01). Each test asserts the CORRECT behavior, so each test
FAILS on the taped-out RTL. They are marked expect_fail=True:

- Remove expect_fail to see each test catch its bug. These are the
  tests that would have prevented the bugs before tapeout.
- If a test starts to "unexpectedly pass", the RTL was changed. The
  firmware workarounds in firmware/src/bf_run.c must then be removed,
  because they compensate for the exact buggy behavior.

The bug list and the firmware workarounds are described in
firmware/README.md ("Silicon bugs and their firmware workarounds").
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles

from test import (
    OP_ADD,
    OP_CLOSE,
    OP_OPEN,
    OP_RIGHT,
    OP_LEFT,
    OP_SUB,
    execute_instr,
    get_inspect_data,
    get_interrupt_jump,
    get_tx_clk,
    set_inspect_sel,
)

SEL_DATA = 0
SEL_PTR = 1
SEL_PC = 2
SEL_BSTACK = 3


async def setup(dut):
    clock = Clock(dut.clk, 10, units="us")
    cocotb.start_soon(clock.start())
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 10)


async def read_inspect(dut, sel):
    set_inspect_sel(dut, sel)
    await ClockCycles(dut.clk, 2)
    return get_inspect_data(dut)


@cocotb.test(expect_fail=True)
async def test_spi_refill_completes(dut):
    """Bug 1: a pointer move across the cache boundary must complete.

    The refill FSM in bf_asic.v re-issues its SPI transaction forever:
    transfer_done and !spi_busy arrive on the same cycle, and the
    issue branch has no !transfer_done guard, so the exit branch never
    wins. Correct RTL retires the instruction and advances the PC.
    """
    await setup(dut)

    await execute_instr(dut, OP_ADD)  # cell 0 = 1
    for _ in range(8):
        await execute_instr(dut, OP_RIGHT)  # ptr 8: still in the window
    await execute_instr(dut, OP_RIGHT)  # ptr 9: starts the refill

    # A write plus a read of 8 bytes each at SCK = clk/16 needs about
    # 2100 cycles. Give it a generous margin.
    set_inspect_sel(dut, SEL_PC)
    for _ in range(60):
        await ClockCycles(dut.clk, 100)
        if get_inspect_data(dut) == 10:
            break
    assert get_inspect_data(dut) == 10, (
        f"PC stuck at {get_inspect_data(dut)} after cache refill: "
        "the SPI transaction re-issues forever"
    )


@cocotb.test(expect_fail=True)
async def test_serial_tx_single_frame(dut):
    """Bug 2: a ']' jump must transmit exactly one 10-bit frame.

    bf_asic clears tx_start one cycle after serial_tx returns to IDLE,
    so serial_tx re-arms on the stale strobe and sends the same frame
    a second time. The phantom frame collides with the next handshake
    in tight loops.
    """
    await setup(dut)

    # ++ [ - ]  -> data = 1 at the ']', so the ASIC transmits a target
    await execute_instr(dut, OP_ADD)
    await execute_instr(dut, OP_ADD)
    await execute_instr(dut, OP_OPEN)
    await execute_instr(dut, OP_SUB)
    await execute_instr(dut, OP_CLOSE)

    # Count complete frames on TX_CLK for the next 200 cycles. One
    # frame is 10 bits at 2 cycles per bit.
    frames = 0
    in_frame = False
    rising = 0
    prev = get_tx_clk(dut)
    for _ in range(200):
        await ClockCycles(dut.clk, 1)
        now = get_tx_clk(dut)
        if not in_frame and prev == 1 and now == 0:
            in_frame = True
            rising = 0
        elif in_frame and prev == 0 and now == 1:
            rising += 1
            if rising == 10:
                frames += 1
                in_frame = False
        prev = now
    assert frames == 1, f"']' transmitted {frames} frames, want exactly 1"


@cocotb.test(expect_fail=True)
async def test_close_bracket_pops_once(dut):
    """Bug 3: a ']' jump must pop exactly one bracket-stack entry.

    The WAIT_JUMP start branch fires on two consecutive cycles before
    tx_busy rises. With two or more entries the ASIC pops both and its
    PC lands on the second entry.
    """
    await setup(dut)

    # + [ + [ - ]   pushes 1 then 3; the ']' at pc 5 sees data = 1
    await execute_instr(dut, OP_ADD)    # pc 0
    await execute_instr(dut, OP_OPEN)   # pc 1: push 1
    await execute_instr(dut, OP_ADD)    # pc 2
    await execute_instr(dut, OP_OPEN)   # pc 3: push 3
    await execute_instr(dut, OP_SUB)    # pc 4: data back to 1
    await execute_instr(dut, OP_CLOSE)  # pc 5: jump to 3

    # Wait for the jump handshake to finish (plus any phantom frame).
    for _ in range(100):
        await ClockCycles(dut.clk, 1)
        if not get_interrupt_jump(dut):
            break
    await ClockCycles(dut.clk, 60)

    pc = await read_inspect(dut, SEL_PC)
    top = await read_inspect(dut, SEL_BSTACK)
    assert pc == 3, f"PC after ']' jump is {pc}, want 3 (single pop)"
    assert top == 1, f"bstack top after ']' jump is {top}, want 1"


@cocotb.test(expect_fail=True)
async def test_tape_base_cell_persists(dut):
    """Bug 4: the cell at tape_base (physical cell 4) must keep data.

    The cache multiplexer returns data_current for offset 0 and the
    save path drops the value when the pointer leaves. The cell reads
    back whatever value the pointer carries in.
    """
    await setup(dut)

    for _ in range(4):
        await execute_instr(dut, OP_RIGHT)  # ptr 4 = tape_base
    for _ in range(3):
        await execute_instr(dut, OP_ADD)  # cell 4 = 3
    await execute_instr(dut, OP_RIGHT)  # leave: value must be saved
    await execute_instr(dut, OP_LEFT)   # return: value must come back

    data = await read_inspect(dut, SEL_DATA)
    assert data == 3, f"cell 4 reads {data} after leave/return, want 3"
