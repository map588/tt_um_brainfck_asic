from tt_explorer import protocol


def test_parse_ok_with_payload():
    r = protocol.parse_reply("ok 1000000")
    assert r.ok and r.payload == "1000000"


def test_parse_ok_bare():
    r = protocol.parse_reply("ok")
    assert r.ok and r.payload == ""


def test_parse_err():
    r = protocol.parse_reply("err not-bf-design", ["# detail"])
    assert not r.ok
    assert r.payload == "not-bf-design"
    assert r.info == ["# detail"]


def test_reply_line_detection():
    assert protocol.is_reply_line("ok 42")
    assert protocol.is_reply_line("err mode")
    assert not protocol.is_reply_line("# info")
    assert protocol.is_info_line("# info")


def test_parse_hello():
    h = protocol.parse_hello("tt-explorer 1 bf=448")
    assert h == {"version": 1, "bf": 448}


def test_parse_status():
    st = protocol.parse_status(
        "design=448 mode=run freq=1000000 ui=0a uidrv=1 uiod=f0 bf=1")
    assert st["design"] == 448
    assert st["mode"] == "run"
    assert st["freq"] == 1000000
    assert st["ui"] == 0x0A
    assert st["uidrv"] == 1
    assert st["uiod"] == 0xF0
    assert st["bf"] == 1


def test_parse_status_unselected():
    st = protocol.parse_status(
        "design=-1 mode=step freq=1000000 ui=00 uiod=00 bf=0")
    assert st["design"] == -1
    assert st["mode"] == "step"


def test_hex_byte_roundtrip():
    assert protocol.hex_byte(0) == "00"
    assert protocol.hex_byte(255) == "ff"
    assert protocol.parse_hex_byte("a5") == 0xA5


def test_seven_seg():
    from tt_explorer.widgets import seven_seg
    # 0x6d lights a..g for the digit 5: a,c,d,f,g on; b,e,dp off
    assert seven_seg(0x6D) == " _ \n|_ \n _| "
    assert seven_seg(0x00) == "   \n   \n    "
    assert seven_seg(0xFF) == " _ \n|_|\n|_|."
