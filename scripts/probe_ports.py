#!/usr/bin/env python3
"""Read-only probe of the NexArm serial ports.

Sends only query commands (firmware version, read positions). It never writes
positions, changes torque, or switches modes. Use it to find out which port is
the leader and which is the follower:

    python3 scripts/probe_ports.py            # one snapshot of every CH340 port
    python3 scripts/probe_ports.py --watch 10 # poll for 10 s; move one arm and see which port changes

Note: opening the port may toggle the board's reset line. Keep the follower's
workspace clear the first time you run this.
"""

from __future__ import annotations

import argparse
import glob
import struct
import time

import serial

BAUD = 1_000_000
CMD_FIRMWARE_VERSION = 1
CMD_READ_POS = 96


def build_frame(cmd: int, args: bytes = b"", device_id: int = 0xFF) -> bytes:
    body = bytes([device_id, len(args) + 2, cmd]) + args
    return b"\xff\xff" + body + bytes([(~sum(body)) & 0xFF])


def find_frame(buf: bytes) -> tuple[int, bytes] | None:
    """Return (cmd, args) of the first complete frame in buf, ignoring checksum."""
    i = buf.find(b"\xff\xff")
    while i >= 0 and len(buf) >= i + 5:
        length = buf[i + 3]
        if length >= 2 and len(buf) >= i + 4 + length:
            return buf[i + 4], buf[i + 5 : i + 3 + length]
        i = buf.find(b"\xff\xff", i + 1)
    return None


def query(port: serial.Serial, cmd: int, timeout: float = 0.2) -> tuple[bytes, float]:
    port.reset_input_buffer()
    t0 = time.perf_counter()
    port.write(build_frame(cmd))
    buf = b""
    while time.perf_counter() - t0 < timeout:
        buf += port.read(port.in_waiting or 1)
        if find_frame(buf) is not None:
            break
    return buf, (time.perf_counter() - t0) * 1000.0


def read_positions(port: serial.Serial) -> tuple[list[int] | None, float, bytes]:
    raw, ms = query(port, CMD_READ_POS)
    frame = find_frame(raw)
    if frame is None or frame[0] != CMD_READ_POS or len(frame[1]) < 12:
        return None, ms, raw
    return list(struct.unpack_from("<6h", frame[1])), ms, raw


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--ports", nargs="*", help="ports to probe (default: every /dev/serial/by-path/*-usb-* entry)")
    parser.add_argument("--watch", type=float, default=0.0, help="poll positions for this many seconds")
    parser.add_argument("--boot-wait", type=float, default=2.0, help="seconds to wait after opening, in case the board resets")
    args = parser.parse_args()

    names = args.ports or sorted(glob.glob("/dev/serial/by-path/*-usb-0*"))
    if not names:
        raise SystemExit("no serial ports found")

    ports = {}
    for name in names:
        ports[name] = serial.Serial(name, BAUD, timeout=0.02, write_timeout=0.2)
    time.sleep(args.boot_wait)

    for name, port in ports.items():
        boot_text = port.read(port.in_waiting or 0)
        print(f"\n== {name}")
        if boot_text:
            print(f"   text seen after open ({len(boot_text)} bytes): {boot_text[:200]!r}")
        raw, ms = query(port, CMD_FIRMWARE_VERSION)
        print(f"   version query : {ms:6.1f} ms  raw={raw.hex(' ') or '(no reply)'}")
        pos, ms, raw = read_positions(port)
        print(f"   read positions: {ms:6.1f} ms  {pos if pos else 'raw=' + (raw.hex(' ') or '(no reply)')}")

    if args.watch > 0:
        print(f"\nPolling for {args.watch:.0f} s. Move ONE arm by hand and watch which line changes.")
        end = time.time() + args.watch
        while time.time() < end:
            line = []
            for name, port in ports.items():
                pos, _, _ = read_positions(port)
                line.append(f"{name[-12:]}: {pos if pos else '---'}")
            print("   " + "  |  ".join(line))
            time.sleep(0.25)

    for port in ports.values():
        port.close()


if __name__ == "__main__":
    main()
