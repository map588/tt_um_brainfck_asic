from bf_explorer.app import BF_END
from bf_explorer.widgets import op_index


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
