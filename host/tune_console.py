#!/usr/bin/env python3
"""Remote real-time PID tuner for the STM32H0 + DAPLINK-WIRELESS link.

The script implements the protocol in ``无线实时智能调参系统设计书.md``:

* Downlink: 18 bytes, ``AA FF`` header, three little-endian floats, int16 aux,
  and an additive checksum.
* Telemetry: 20 bytes, ``AA FE`` header, three little-endian floats, revision,
  status bits, and an additive checksum.

Only the transport is platform-specific. TCP is used for the wireless bridge;
serial mode is available for a direct USB-TTL test and requires pyserial.
"""

from __future__ import annotations

import argparse
import csv
import math
import shlex
import socket
import struct
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional, Protocol, Sequence


DOWNLINK_HEADER = b"\xAA\xFF"
TELEMETRY_HEADER = b"\xAA\xFE"
DOWNLINK_SIZE = 18
TELEMETRY_SIZE = 20
VALID_LOOP_IDS = frozenset(range(1, 5))

STATUS_NAMES = {
    0: "parameter_valid",
    1: "control_enabled",
    2: "sensor_valid",
    3: "parameter_rejected",
    4: "output_saturated",
    5: "sensor_timeout",
}


class ProtocolError(ValueError):
    """Raised when a frame cannot be encoded or decoded."""


def additive_checksum(data: bytes) -> int:
    """Return the protocol checksum: sum of all frame bytes modulo 256."""

    return sum(data) & 0xFF


def _require_finite(name: str, value: float) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise ProtocolError(f"{name} must be finite, got {value!r}")
    return value


def pack_tune_frame(loop_id: int, kp: float, ki: float, kd: float, aux: int) -> bytes:
    """Build one 18-byte parameter frame."""

    loop_id = int(loop_id)
    aux = int(aux)
    if loop_id not in VALID_LOOP_IDS:
        raise ProtocolError(f"loop_id must be 1..4, got {loop_id}")
    if not -32768 <= aux <= 32767:
        raise ProtocolError(f"aux must fit int16, got {aux}")

    body = struct.pack(
        "<BBBfffh",
        DOWNLINK_HEADER[0],
        DOWNLINK_HEADER[1],
        loop_id,
        _require_finite("kp", kp),
        _require_finite("ki", ki),
        _require_finite("kd", kd),
        aux,
    )
    return body + bytes((additive_checksum(body),))


@dataclass(frozen=True)
class Telemetry:
    loop_id: int
    sequence: int
    error: float
    output: float
    measurement: float
    revision: int
    status: int
    received_at: float

    @property
    def status_names(self) -> tuple[str, ...]:
        return tuple(name for bit, name in STATUS_NAMES.items() if self.status & (1 << bit))

    @property
    def sensor_valid(self) -> bool:
        return bool(self.status & (1 << 2))


def unpack_telemetry_frame(frame: bytes, received_at: Optional[float] = None) -> Telemetry:
    """Decode and validate one 20-byte telemetry frame."""

    if len(frame) != TELEMETRY_SIZE:
        raise ProtocolError(f"telemetry frame must be {TELEMETRY_SIZE} bytes")
    if frame[:2] != TELEMETRY_HEADER:
        raise ProtocolError(f"invalid telemetry header: {frame[:2].hex(' ')}")
    if additive_checksum(frame[:-1]) != frame[-1]:
        raise ProtocolError("telemetry checksum mismatch")

    _, _, loop_id, sequence, error, output, measurement, revision, status = struct.unpack(
        "<BBBBfffHB", frame[:-1]
    )
    if loop_id not in VALID_LOOP_IDS:
        raise ProtocolError(f"invalid telemetry loop_id: {loop_id}")
    for name, value in (
        ("error", error),
        ("output", output),
        ("measurement", measurement),
    ):
        _require_finite(name, value)

    return Telemetry(
        loop_id=loop_id,
        sequence=sequence,
        error=error,
        output=output,
        measurement=measurement,
        revision=revision,
        status=status,
        received_at=time.monotonic() if received_at is None else received_at,
    )


def build_telemetry_frame(
    loop_id: int,
    sequence: int,
    error: float,
    output: float,
    measurement: float,
    revision: int,
    status: int,
) -> bytes:
    """Build a telemetry frame for offline tests and protocol simulators."""

    if loop_id not in VALID_LOOP_IDS:
        raise ProtocolError(f"loop_id must be 1..4, got {loop_id}")
    if not 0 <= sequence <= 255:
        raise ProtocolError("sequence must fit uint8")
    if not 0 <= revision <= 65535:
        raise ProtocolError("revision must fit uint16")
    if not 0 <= status <= 255:
        raise ProtocolError("status must fit uint8")

    body = struct.pack(
        "<BBBBfffHB",
        TELEMETRY_HEADER[0],
        TELEMETRY_HEADER[1],
        loop_id,
        sequence,
        _require_finite("error", error),
        _require_finite("output", output),
        _require_finite("measurement", measurement),
        revision,
        status,
    )
    return body + bytes((additive_checksum(body),))


class FrameParser:
    """Incremental fixed-length parser resilient to noise and bad frames."""

    def __init__(self, header: bytes, frame_size: int, decoder: Callable[[bytes], object]):
        if not header or frame_size <= len(header):
            raise ValueError("invalid parser configuration")
        self.header = bytes(header)
        self.frame_size = frame_size
        self.decoder = decoder
        self.buffer = bytearray()
        self.frames_seen = 0
        self.frames_rejected = 0
        self.bytes_discarded = 0

    def feed(self, data: bytes) -> list[object]:
        self.buffer.extend(data)
        decoded: list[object] = []

        while True:
            start = self.buffer.find(self.header)
            if start < 0:
                keep = len(self.header) - 1
                discarded = max(0, len(self.buffer) - keep)
                if discarded:
                    del self.buffer[:discarded]
                    self.bytes_discarded += discarded
                break
            if start:
                del self.buffer[:start]
                self.bytes_discarded += start
            if len(self.buffer) < self.frame_size:
                break

            candidate = bytes(self.buffer[:self.frame_size])
            try:
                decoded.append(self.decoder(candidate))
            except (ProtocolError, struct.error, ValueError):
                self.frames_rejected += 1
                # Drop one byte instead of the whole candidate, so an embedded
                # header can become the next synchronization point.
                del self.buffer[0]
                self.bytes_discarded += 1
                continue

            self.frames_seen += 1
            del self.buffer[:self.frame_size]

        return decoded


class ByteTransport(Protocol):
    description: str

    def read(self, size: int = 4096) -> bytes:
        ...

    def write(self, data: bytes) -> None:
        ...

    def close(self) -> None:
        ...


class TcpTransport:
    def __init__(self, host: str, port: int, timeout: float = 0.25):
        self.description = f"tcp://{host}:{port}"
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.sock.settimeout(timeout)
        self._closed = False

    def read(self, size: int = 4096) -> bytes:
        if self._closed:
            return b""
        try:
            data = self.sock.recv(size)
        except socket.timeout:
            return b""
        if not data:
            raise ConnectionError("TCP peer closed the connection")
        return data

    def write(self, data: bytes) -> None:
        if self._closed:
            raise ConnectionError("TCP transport is closed")
        self.sock.sendall(data)

    def close(self) -> None:
        if not self._closed:
            self._closed = True
            try:
                self.sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self.sock.close()


class SerialTransport:
    def __init__(self, port: str, baudrate: int, timeout: float = 0.25):
        try:
            import serial  # type: ignore
        except ImportError as exc:
            raise RuntimeError("serial mode requires pyserial: python -m pip install pyserial") from exc

        self.description = f"serial://{port}@{baudrate}"
        self.serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
            write_timeout=1.0,
        )

    def read(self, size: int = 4096) -> bytes:
        return bytes(self.serial.read(size))

    def write(self, data: bytes) -> None:
        self.serial.write(data)

    def close(self) -> None:
        self.serial.close()


@dataclass
class TuneValues:
    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0
    aux: int = 0


def adaptive_step(abs_error: float, coarse: float, medium: float, fine: float) -> float:
    """Choose the UI step from the design document's error bands."""

    abs_error = abs(float(abs_error))
    if abs_error > 20.0:
        return coarse
    if abs_error < 5.0:
        return fine
    return medium


class TelemetryReceiver(threading.Thread):
    def __init__(
        self,
        transport: ByteTransport,
        on_frame: Callable[[Telemetry], None],
        on_error: Callable[[Exception], None],
    ):
        super().__init__(name="telemetry-receiver", daemon=True)
        self.transport = transport
        self.on_frame = on_frame
        self.on_error = on_error
        self.stop_event = threading.Event()
        self.parser = FrameParser(
            TELEMETRY_HEADER,
            TELEMETRY_SIZE,
            lambda frame: unpack_telemetry_frame(frame),
        )

    def stop(self) -> None:
        self.stop_event.set()

    def run(self) -> None:
        while not self.stop_event.is_set():
            try:
                data = self.transport.read()
                if data:
                    for frame in self.parser.feed(data):
                        self.on_frame(frame)  # type: ignore[arg-type]
            except Exception as exc:  # transport errors must reach the console
                self.on_error(exc)
                return


class CsvLogger:
    def __init__(self, path: Path):
        path.parent.mkdir(parents=True, exist_ok=True)
        self.file = path.open("w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.file)
        self.writer.writerow(
            [
                "timestamp_unix",
                "loop_id",
                "sequence",
                "error",
                "output",
                "measurement",
                "revision",
                "status",
                "status_names",
            ]
        )
        self.file.flush()
        self.lock = threading.Lock()

    def write(self, telemetry: Telemetry) -> None:
        with self.lock:
            self.writer.writerow(
                [
                    time.time(),
                    telemetry.loop_id,
                    telemetry.sequence,
                    telemetry.error,
                    telemetry.output,
                    telemetry.measurement,
                    telemetry.revision,
                    f"0x{telemetry.status:02X}",
                    "|".join(telemetry.status_names),
                ]
            )
            self.file.flush()

    def close(self) -> None:
        with self.lock:
            self.file.close()


class TunerApp:
    def __init__(self, args: argparse.Namespace, transport: ByteTransport):
        self.args = args
        self.transport = transport
        self.selected_loop = args.loop_id
        self.values = {loop_id: TuneValues() for loop_id in VALID_LOOP_IDS}
        self.values[self.selected_loop] = TuneValues(args.kp, args.ki, args.kd, args.aux)
        self.latest: dict[int, Telemetry] = {}
        self.latest_lock = threading.Lock()
        self.write_lock = threading.Lock()
        self.last_display = 0.0
        self.connection_error: Optional[Exception] = None
        self.logger = CsvLogger(Path(args.log)) if args.log else None
        self.receiver = TelemetryReceiver(transport, self.on_telemetry, self.on_receiver_error)

    def on_telemetry(self, telemetry: Telemetry) -> None:
        with self.latest_lock:
            self.latest[telemetry.loop_id] = telemetry
        if self.logger:
            self.logger.write(telemetry)

        now = time.monotonic()
        if not self.args.quiet and now - self.last_display >= 1.0 / self.args.display_hz:
            self.last_display = now
            status = ",".join(telemetry.status_names) or "none"
            print(
                f"[telemetry] loop={telemetry.loop_id} seq={telemetry.sequence:03d} "
                f"err={telemetry.error:.5g} out={telemetry.output:.5g} "
                f"meas={telemetry.measurement:.5g} rev={telemetry.revision} "
                f"status={status}",
                flush=True,
            )

    def on_receiver_error(self, exc: Exception) -> None:
        self.connection_error = exc
        print(f"[transport] {exc}", file=sys.stderr, flush=True)

    def send_selected(self) -> None:
        values = self.values[self.selected_loop]
        frame = pack_tune_frame(
            self.selected_loop,
            values.kp,
            values.ki,
            values.kd,
            values.aux,
        )
        with self.write_lock:
            self.transport.write(frame)
        print(
            f"[sent] loop={self.selected_loop} kp={values.kp:g} ki={values.ki:g} "
            f"kd={values.kd:g} aux={values.aux} checksum=0x{frame[-1]:02X}",
            flush=True,
        )

    def show(self, loop_id: Optional[int] = None) -> None:
        loop_id = self.selected_loop if loop_id is None else loop_id
        values = self.values[loop_id]
        telemetry = self.latest.get(loop_id)
        print(f"[config] loop={loop_id} kp={values.kp:g} ki={values.ki:g} kd={values.kd:g} aux={values.aux}")
        if telemetry:
            status = ",".join(telemetry.status_names) or "none"
            age_ms = (time.monotonic() - telemetry.received_at) * 1000.0
            print(
                f"[latest] error={telemetry.error:g} output={telemetry.output:g} "
                f"measurement={telemetry.measurement:g} revision={telemetry.revision} "
                f"age={age_ms:.0f}ms status={status}"
            )
        else:
            print("[latest] no telemetry received")

    def set_value(self, name: str, value: str, loop_id: Optional[int] = None) -> None:
        loop_id = self.selected_loop if loop_id is None else loop_id
        if loop_id not in VALID_LOOP_IDS:
            raise ValueError("loop must be 1..4")
        if name not in {"kp", "ki", "kd", "aux"}:
            raise ValueError("field must be kp, ki, kd or aux")
        if name == "aux":
            parsed: float | int = int(value, 0)
            if not -32768 <= parsed <= 32767:
                raise ValueError("aux must fit int16")
        else:
            parsed = _require_finite(name, float(value))
        setattr(self.values[loop_id], name, parsed)
        self.selected_loop = loop_id
        print(f"[config] loop={loop_id} {name}={parsed}")

    def step_value(self, name: str, direction: str) -> None:
        if name not in {"kp", "ki", "kd"}:
            raise ValueError("step field must be kp, ki or kd")
        if direction.lower() in {"up", "+", "+1", "1"}:
            sign = 1.0
        elif direction.lower() in {"down", "-", "-1"}:
            sign = -1.0
        else:
            raise ValueError("direction must be up/down or +/-1")

        telemetry = self.latest.get(self.selected_loop)
        abs_error = abs(telemetry.error) if telemetry else float("inf")
        delta = sign * adaptive_step(
            abs_error,
            self.args.coarse_step,
            self.args.medium_step,
            self.args.fine_step,
        )
        setattr(self.values[self.selected_loop], name, getattr(self.values[self.selected_loop], name) + delta)
        print(
            f"[step] loop={self.selected_loop} {name} delta={delta:g} "
            f"new={getattr(self.values[self.selected_loop], name):g} abs_error={abs_error:g}"
        )

    def command_loop(self) -> None:
        print_help()
        while self.connection_error is None:
            try:
                line = input("tune> ").strip()
            except EOFError:
                return
            except KeyboardInterrupt:
                print()
                return
            if not line:
                continue
            try:
                if self.execute_command(line):
                    return
            except (ValueError, ProtocolError, ConnectionError) as exc:
                print(f"[error] {exc}")

    def execute_command(self, line: str) -> bool:
        tokens = shlex.split(line)
        command = tokens[0].lower()
        args = tokens[1:]
        if command in {"quit", "exit", "q"}:
            return True
        if command in {"help", "h", "?"}:
            print_help()
            return False
        if command == "select":
            if len(args) != 1:
                raise ValueError("usage: select <loop_id>")
            loop_id = int(args[0], 0)
            if loop_id not in VALID_LOOP_IDS:
                raise ValueError("loop_id must be 1..4")
            self.selected_loop = loop_id
            self.show()
            return False
        if command == "show":
            self.show(int(args[0], 0) if args else None)
            return False
        if command == "set":
            if len(args) not in {2, 3}:
                raise ValueError("usage: set <kp|ki|kd|aux> <value> [loop_id]")
            loop_id = int(args[2], 0) if len(args) == 3 else None
            self.set_value(args[0].lower(), args[1], loop_id)
            return False
        if command in {"step", "nudge"}:
            if len(args) != 2:
                raise ValueError("usage: step <kp|ki|kd> <up|down>")
            self.step_value(args[0].lower(), args[1])
            return False
        if command in {"send", "apply"}:
            self.send_selected()
            return False
        if command == "stats":
            parser = self.receiver.parser
            print(
                f"[stats] valid={parser.frames_seen} rejected={parser.frames_rejected} "
                f"discarded_bytes={parser.bytes_discarded}"
            )
            return False
        raise ValueError("unknown command; use help")

    def run(self) -> None:
        self.receiver.start()
        try:
            if self.args.send_initial:
                self.send_selected()
            if self.args.once:
                return
            self.command_loop()
        finally:
            self.receiver.stop()
            self.transport.close()
            self.receiver.join(timeout=1.0)
            if self.logger:
                self.logger.close()


def print_help() -> None:
    print(
        "Commands:\n"
        "  select <1..4>              select control loop\n"
        "  set <kp|ki|kd|aux> <value> [loop]  change a value locally\n"
        "  step <kp|ki|kd> <up|down>  adaptive nudge based on telemetry error\n"
        "  send                       transmit selected loop parameters\n"
        "  show [loop]                show config and latest telemetry\n"
        "  stats                      show parser statistics\n"
        "  help                       show this help\n"
        "  quit                       close the link\n"
    )


def build_transport(args: argparse.Namespace) -> ByteTransport:
    if args.tcp:
        host, port_text = args.tcp
        return TcpTransport(host, int(port_text), timeout=args.timeout)
    if args.serial_port:
        return SerialTransport(args.serial_port, args.baudrate, timeout=args.timeout)
    raise ValueError("choose exactly one transport: --tcp HOST PORT or --serial-port PORT")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Remote PID tuning for STM32H0 through DAPLINK-WIRELESS",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    transport = parser.add_mutually_exclusive_group(required=True)
    transport.add_argument("--tcp", nargs=2, metavar=("HOST", "PORT"), help="wireless TCP bridge")
    transport.add_argument("--serial-port", help="direct USB-TTL serial port")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=0.25, help="transport read timeout in seconds")
    parser.add_argument("--loop-id", type=int, default=1, choices=sorted(VALID_LOOP_IDS))
    parser.add_argument("--kp", type=float, default=0.0)
    parser.add_argument("--ki", type=float, default=0.0)
    parser.add_argument("--kd", type=float, default=0.0)
    parser.add_argument("--aux", type=int, default=0)
    parser.add_argument(
        "--send-initial",
        action="store_true",
        help="send the initial values immediately after connecting",
    )
    parser.add_argument("--once", action="store_true", help="connect, optionally send, then exit")
    parser.add_argument("--quiet", action="store_true", help="suppress periodic telemetry display")
    parser.add_argument("--display-hz", type=float, default=10.0)
    parser.add_argument("--log", metavar="CSV", help="write all valid telemetry frames to CSV")
    parser.add_argument("--coarse-step", type=float, default=1.0)
    parser.add_argument("--medium-step", type=float, default=0.2)
    parser.add_argument("--fine-step", type=float, default=0.05)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.display_hz <= 0.0:
        parser.error("--display-hz must be greater than zero")
    for name in ("coarse_step", "medium_step", "fine_step"):
        if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0.0:
            parser.error(f"--{name.replace('_', '-')} must be finite and greater than zero")

    try:
        transport = build_transport(args)
        print(f"Connected: {transport.description}")
        TunerApp(args, transport).run()
    except (OSError, RuntimeError, ValueError, ConnectionError) as exc:
        print(f"connection failed: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
