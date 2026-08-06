from bf_explorer.app import BF_END, BF_HALTED
from bf_explorer.widgets import fmt_hz, op_index, render_program


def test_op_index_counts_ops_only():
    text = "comment ,+.\nmore [x]"
    # cursor at start: first op is index 0
    assert op_index(text, 0, 0) == 0
    # cursor on the '+' (row 0, col 9): ',' is before it
    assert op_index(text, 0, 9) == 1
    # cursor on the '[' (row 1, col 5): , + . before it
    assert op_index(text, 1, 5) == 3
    # cursor at the very end: past the last op
    assert op_index(text, 1, 8) is None


def test_op_index_empty_program():
    assert op_index("no ops here", 0, 0) is None


def test_bf_end_matches_session_trailers():
    assert BF_END.search("B\nok done\n") is not None
    m = BF_END.search("output\nerr run-fail\n")
    assert m is not None and m.group(2) == "run-fail"
    assert BF_END.search("ok 448\n# banner\n") is None
    # mid-stream 'ok' from the banner must not end the session
    assert BF_END.search("ok bf\n# paste program, end with '!'\n") is None


def test_bf_halted_parses_instruction_count():
    m = BF_HALTED.search("# halted: 1108 instructions executed")
    assert m is not None and int(m.group(1)) == 1108
    assert BF_HALTED.search("# 3 ops") is None


def test_fmt_hz():
    assert fmt_hz(200_000) == "200 kHz"
    assert fmt_hz(2_000_000) == "2 MHz"
    assert fmt_hz(1_500_000) == "1.5 MHz"
    assert fmt_hz(440) == "440 Hz"


def test_render_program_marks_pc_and_breaks():
    # ops: 0=',' 1='+' 2='.'  comments dimmed, op indices skip them
    text = "go ,+."
    out = render_program(text, pc=1, breaks={2})
    spans = {text[s.start]: s.style for s in out.spans}
    assert out.plain == text
    assert spans["+"] == "black on green"      # the next op
    assert spans["."] == "bold red3 underline"  # a breakpoint
    assert spans["g"] == "dim"                 # comment
    # pc on a breakpoint gets the combined mark
    out = render_program(text, pc=2, breaks={2})
    spans = {text[s.start]: s.style for s in out.spans}
    assert spans["."] == "black on red3"
