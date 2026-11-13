"""协议层单元测试，不需要硬件。运行：python3 -m pytest tests/"""

from nexarm_teleop import protocol as p

# 2026-11-13 从真实从臂串口抓到的一帧主动上报（11 号指令，24 字节参数）
REAL_FOLLOWER_FRAME = bytes.fromhex(
    "ff ff ff 1a 0b d4 00 f9 ff f7 ff 11 fd 00 00 00 00 17 08 43 06 e3 05 7d 07 00 08 00 08 27"
)


def test_build_read_frame_matches_known_bytes():
    assert p.build_frame(p.CMD_READ_POS) == bytes.fromhex("ff ff ff 02 60 9e")


def test_roundtrip():
    args = p.pack_positions([2078, 2502, 1501, 1915, 2045, 1853])
    frames = p.FrameParser().feed(p.build_frame(p.CMD_READ_POS, args))
    assert len(frames) == 1
    assert frames[0].cmd == p.CMD_READ_POS
    assert p.unpack_positions(frames[0].args) == [2078, 2502, 1501, 1915, 2045, 1853]


def test_real_captured_frame_parses():
    frames = p.FrameParser().feed(REAL_FOLLOWER_FRAME)
    assert len(frames) == 1
    assert frames[0].cmd == 11 and len(frames[0].args) == 24


def test_skips_debug_text_between_frames():
    reply = p.build_frame(p.CMD_READ_POS, p.pack_positions([1, 2, 3, 4, 5, 6]))
    stream = b"[CB] id=0xFF cmd=96 len=2\n" + REAL_FOLLOWER_FRAME + b"[POS] 1 2 3\n" + reply
    frames = p.FrameParser().feed(stream)
    assert [f.cmd for f in frames] == [11, p.CMD_READ_POS]


def test_frame_split_across_reads():
    reply = p.build_frame(p.CMD_READ_POS, p.pack_positions([10, 20, 30, 40, 50, 60]))
    parser = p.FrameParser()
    got = []
    for i in range(len(reply)):
        got += parser.feed(reply[i : i + 1])
    assert len(got) == 1 and p.unpack_positions(got[0].args) == [10, 20, 30, 40, 50, 60]


def test_bad_checksum_rejected():
    frame = bytearray(p.build_frame(p.CMD_READ_POS, p.pack_positions([1, 2, 3, 4, 5, 6])))
    frame[-1] ^= 0x55
    assert p.FrameParser().feed(bytes(frame)) == []


def test_leader_short_checksum_accepted():
    # 主臂固件的校验和只覆盖 ID、LEN、CMD
    args = p.pack_positions([2078, 2502, 1501, 1915, 2045, 1853])
    body = bytes([0xFF, len(args) + 2, p.CMD_READ_POS])
    frame = b"\xff\xff" + body + args + bytes([p.checksum(body)])
    frames = p.FrameParser().feed(frame)
    assert len(frames) == 1 and frames[0].cmd == p.CMD_READ_POS


def test_fake_header_with_huge_length_does_not_block_real_frame():
    reply = p.build_frame(p.CMD_READ_POS, p.pack_positions([1, 2, 3, 4, 5, 6]))
    frames = p.FrameParser().feed(b"\xff\xff\x00\xf0" + reply)
    assert [f.cmd for f in frames] == [p.CMD_READ_POS]


def test_mapping_matches_factory_firmware():
    out = p.map_leader_to_follower([2078, 2502, 1501, 1915, 2045, 1853])
    assert out == [2078, 1594, 1501, 1915, 2045, 2053]
    # 夹爪限幅
    assert p.map_leader_to_follower([0, 0, 0, 0, 0, 4095])[5] == 2833
    assert p.map_leader_to_follower([0, 0, 0, 0, 0, 0])[5] == 1195
