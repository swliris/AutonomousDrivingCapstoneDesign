import struct
import pytest
from second.protocol import Parser, command_frame, crc16, frame, legacy_packet


def test_crc_known_reference():
    assert crc16(b"123456789") == 0x29B1


@pytest.mark.parametrize("opcode", [0, 2, 3])
def test_every_legacy_payload_agrees_with_original_arduino_decoder(opcode):
    for value in range(256):
        a, b, end = legacy_packet(value, opcode)
        data = (a << 8) | b
        assert data >> 14 == opcode
        assert ((data >> 12) & 3) ^ 3 == opcode
        assert (data >> 4) & 255 == value
        assert (data >> 2) & 3 == opcode
        assert data & 3 == (data >> 2).bit_count() % 4
        assert end == 0xCC


def test_v2_chunking_corruption_and_resync():
    parser = Parser()
    good = command_frame(65535, -130, -100)
    bad = bytearray(good); bad[7] ^= 17
    assert not parser.feed(b"noise"+bad+good[:5])
    result = parser.feed(good[5:])
    assert result == [(1, struct.pack("<HhhB", 65535, -130, -100, 1))]


def test_feedback_matches_firmware_layout():
    data = struct.pack("<HHhhB", 8, 483, -130, 40, 1)
    assert Parser().feed(frame(2, data)) == [(2, data)]


def test_out_of_range_motor_commands_rejected():
    with pytest.raises(ValueError):
        command_frame(1, 256, 0)
