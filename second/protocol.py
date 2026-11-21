"""Legacy AD-Project packets and paired v2 firmware protocol (little endian)."""
import struct
import time
from dataclasses import dataclass

MAGIC = b"\xa5\x5a"


def crc16(data: bytes):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def frame(kind: int, payload: bytes):
    body = bytes([kind, len(payload)])+payload
    return MAGIC+body+struct.pack("<H", crc16(body))


def legacy_packet(value, opcode):
    value = max(0, min(255, int(value)))
    first = (opcode << 6) | ((opcode ^ 3) << 4) | (value >> 4)
    parity = (first.bit_count()+(value & 15).bit_count()+opcode.bit_count()) % 4
    return bytes([first, ((value & 15) << 4) | (opcode << 2) | parity, 0xCC])


def command_frame(seq, pwm, steer_cmd, enabled=True):
    if not -255 <= pwm <= 255 or not -120 <= steer_cmd <= 120:
        raise ValueError("motor command outside calibrated bounds")
    return frame(1, struct.pack("<HhhB", seq & 65535, pwm, steer_cmd, bool(enabled)))


class Parser:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        result = []
        while len(self.buffer) >= 6:
            i = self.buffer.find(MAGIC)
            if i < 0:
                self.buffer[:] = self.buffer[-1:]
                break
            del self.buffer[:i]
            if len(self.buffer) < 4:
                break
            length = self.buffer[3]
            if length > 32:
                del self.buffer[0]
                continue
            size = length+6
            if len(self.buffer) < size:
                break
            body = bytes(self.buffer[2:4+length])
            if crc16(body) != struct.unpack("<H", self.buffer[4+length:size])[0]:
                del self.buffer[0]
                continue
            result.append((body[0], body[2:]))
            del self.buffer[:size]
        return result


@dataclass
class Feedback:
    stamp: float
    sequence: int
    potentiometer: int
    pwm: int
    steering_command: int
    status: int


class SerialDriver:
    def __init__(self, config):
        import serial
        self.protocol = config["protocol"]
        self.port = serial.Serial(config["port"], config["baud"], timeout=0, write_timeout=.1)
        self.parser = Parser()
        self.sequence = 0
        self.feedback = None
        # Opening a Mega USB port may reset it; no actuation during boot.
        time.sleep(config.get("boot_wait_s", 2.))
        self.send(0, 0, False)

    def send(self, pwm, steer, enabled=True):
        self.sequence = (self.sequence+1) & 65535
        pwm = int(pwm) if enabled else 0
        if self.protocol == "v2":
            data = command_frame(self.sequence, pwm, int(steer), enabled)
        elif self.protocol == "legacy":
            data = legacy_packet(128+int(steer), 2)+legacy_packet(abs(pwm), 0 if pwm >= 0 else 3)
        else:
            raise ValueError("unknown serial protocol")
        if self.port.write(data) != len(data):
            raise IOError("incomplete motor command")

    def poll(self):
        for kind, data in self.parser.feed(self.port.read(self.port.in_waiting)):
            if kind == 2 and len(data) == 9:
                seq, pot, pwm, steer, status = struct.unpack("<HHhhB", data)
                # A valid packet alone is not enough: require recent command acknowledgement.
                if (self.sequence-seq) & 65535 <= 5:
                    self.feedback = Feedback(time.monotonic(), seq, pot, pwm, steer, status)
        return self.feedback

    def close(self):
        try:
            self.send(0, 0, False)
            self.port.flush()
        finally:
            self.port.close()
