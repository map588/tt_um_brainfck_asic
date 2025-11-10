#!/usr/bin/env python3
"""
bf_to_pico.py

Convert a Brainfuck (.bf) program into:
  - a list of 3-bit codes (printed to stdout), and
  - packed bytes suitable for embedding in Raspberry Pi Pico firmware,
    emitted as a C header (and optionally as a raw .bin or a MicroPython .py).

Encoding:
'-' => "000"
'+' => "001"
'<' => "010"
'>' => "011"
'[' => "100"
']' => "101"
',' => "110"
'.' => "111"

Packing details:
- Codes are concatenated in program order into one big bitstream.
- Bytes are produced MSB-first within each byte.
- The final (partial) byte is padded on the right with 0-bits.
- We record the true bit length so your firmware can ignore pad bits if needed.
"""

import argparse
import textwrap
from pathlib import Path

BF_MAP = {
    "-": "000",
    "+": "001",
    "<": "010",
    ">": "011",
    "[": "100",
    "]": "101",
    ",": "110",
    ".": "111",
}


def read_bf(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="ignore")


def bf_to_3bit_list(src: str) -> list[str]:
    """Return list of 3-bit strings for valid BF ops; ignore all other chars."""
    out = []
    for ch in src:
        code = BF_MAP.get(ch)
        if code is not None:
            out.append(code)
    return out


def pack_3bit_codes_to_bytes(codes: list[str], pad_bit: str = "0") -> tuple[bytes, int]:
    """
    Concatenate 3-bit strings, pad with pad_bit to a multiple of 8, and
    return (packed_bytes, true_bit_len).
    Bytes are MSB-first: the earliest bits in the stream become the high bits of the first byte.
    """
    bitstream = "".join(codes)
    true_len = len(bitstream)
    if true_len == 0:
        return b"", 0

    pad_needed = (8 - (true_len % 8)) % 8
    bitstream_padded = bitstream + (pad_bit * pad_needed)

    data = bytearray()
    for i in range(0, len(bitstream_padded), 8):
        chunk = bitstream_padded[i : i + 8]  # MSB-first
        data.append(int(chunk, 2))
    return bytes(data), true_len


def make_c_header(name: str, data: bytes, bit_len: int) -> str:
    """
    Produce a self-contained C header.
    - 'name' will be used for symbols: <name>_data, <name>_bits, <name>_bytes
    - Mark arrays as static const so you can include the header in multiple TUs safely.
    """
    bytes_per_line = 12
    hex_lines = []
    for i in range(0, len(data), bytes_per_line):
        line = ", ".join(f"0x{b:02X}" for b in data[i : i + bytes_per_line])
        hex_lines.append(f"    {line}")

    arr_body = ",\n".join(hex_lines) if hex_lines else "    /* empty */"

    guard = f"{name.upper()}_BF_PROG_H"
    header = f"""\
#ifndef {guard}
#define {guard}

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {{
#endif

/* Packed 3-bit codes, MSB-first per byte. Only the first {bit_len} bits are meaningful. */
static const uint8_t {name}_data[] = {{
{arr_body}
}};

static const size_t {name}_bits  = {bit_len};
static const size_t {name}_bytes = sizeof({name}_data);

#ifdef __cplusplus
}} /* extern "C" */
#endif

#endif /* {guard} */
"""
    return textwrap.dedent(header)


def make_micropython_snippet(name: str, data: bytes, bit_len: int) -> str:
    hex_bytes = ", ".join(f"0x{b:02X}" for b in data) if data else ""
    return f"""\
# Auto-generated from bf_to_pico.py
# Packed MSB-first; only the first {bit_len} bits are meaningful.
{name}_data = bytes([{hex_bytes}])
{name}_bits = {bit_len}
{name}_bytes = len({name}_data)
"""


def print_to_stdout(bf_bits):
    for inst in bf_bits:
        print(f"{inst}")


def main():
    ap = argparse.ArgumentParser(
        description="Convert Brainfuck to 3-bit codes and Pico-friendly data."
    )
    ap.add_argument("input", type=Path, help="Path to .bf file")
    ap.add_argument(
        "--name",
        default="bf_prog",
        help="Base symbol name for generated artifacts (default: bf_prog)",
    )
    ap.add_argument(
        "--header",
        type=Path,
        help="Output C header path (default: <input_stem>_bf_prog.h)",
    )
    ap.add_argument(
        "--bin", type=Path, help="Optional: write raw packed bytes to this .bin file"
    )
    ap.add_argument(
        "--py", type=Path, help="Optional: write a MicroPython snippet to this .py file"
    )
    ap.add_argument(
        "--no-list",
        action="store_true",
        help="Do not print the list of 3-bit codes to stdout",
    )
    ap.add_argument(
        "--print",
        "-p",
        action="store_true",
        help="Do print the list of 3-bit codes and exit",
    )
    args = ap.parse_args()

    src = read_bf(args.input)
    codes = bf_to_3bit_list(src)

    if args.print:
        print_to_stdout(codes)
        exit(0)

    if not args.no_list:
        print(f"# 3-bit codes ({len(codes)} ops):")
        if codes:
            # print in tidy rows
            for i in range(0, len(codes), 32):
                print(" ".join(codes[i : i + 32]))
        else:
            print("(no Brainfuck ops found)")

    packed, bit_len = pack_3bit_codes_to_bytes(codes, pad_bit="0")

    # Default header path
    header_path = args.header or args.input.with_name(f"{args.input.stem}_bf_prog.h")
    header_text = make_c_header(args.name, packed, bit_len)
    header_path.write_text(header_text, encoding="utf-8")
    print(f"Wrote C header: {header_path}")

    if args.bin:
        args.bin.write_bytes(packed)
        print(
            f"Wrote raw .bin: {args.bin}  ({len(packed)} bytes; {bit_len} meaningful bits)"
        )

    if args.py:
        py_text = make_micropython_snippet(args.name, packed, bit_len)
        args.py.write_text(py_text, encoding="utf-8")
        print(f"Wrote MicroPython snippet: {args.py}")


if __name__ == "__main__":
    main()
