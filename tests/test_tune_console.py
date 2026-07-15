import struct
import unittest

from host.tune_console import (
    DOWNLINK_SIZE,
    TELEMETRY_SIZE,
    FrameParser,
    ProtocolError,
    adaptive_step,
    build_telemetry_frame,
    pack_tune_frame,
    unpack_telemetry_frame,
)


class TuneProtocolTests(unittest.TestCase):
    def test_downlink_frame_layout_and_checksum(self):
        frame = pack_tune_frame(2, 1.25, 0.0, 0.05, -12)
        self.assertEqual(len(frame), DOWNLINK_SIZE)
        self.assertEqual(frame[:3], b"\xAA\xFF\x02")
        self.assertEqual(frame[-1], sum(frame[:-1]) & 0xFF)
        values = struct.unpack_from("<fffh", frame, 3)
        self.assertAlmostEqual(values[0], 1.25)
        self.assertAlmostEqual(values[1], 0.0)
        self.assertAlmostEqual(values[2], 0.05, places=6)
        self.assertEqual(values[3], -12)

    def test_downlink_rejects_non_finite_values(self):
        with self.assertRaises(ProtocolError):
            pack_tune_frame(1, float("nan"), 0.0, 0.0, 0)

    def test_telemetry_parser_handles_noise_fragments_and_bad_frame(self):
        good = build_telemetry_frame(1, 7, -2.5, 12.0, 14.5, 3, 0b00000111)
        bad = bytearray(good)
        bad[-1] ^= 0x01
        parser = FrameParser(b"\xAA\xFE", TELEMETRY_SIZE, unpack_telemetry_frame)

        self.assertEqual(parser.feed(b"noise\x00" + good[:5]), [])
        telemetry = parser.feed(good[5:])
        self.assertEqual(len(telemetry), 1)
        telemetry = parser.feed(bytes(bad) + good)

        self.assertEqual(len(telemetry), 1)
        self.assertEqual(telemetry[0].loop_id, 1)
        self.assertEqual(telemetry[0].revision, 3)
        self.assertGreaterEqual(parser.frames_rejected, 1)

    def test_adaptive_steps(self):
        self.assertEqual(adaptive_step(21, 1.0, 0.2, 0.05), 1.0)
        self.assertEqual(adaptive_step(5, 1.0, 0.2, 0.05), 0.2)
        self.assertEqual(adaptive_step(4.99, 1.0, 0.2, 0.05), 0.05)


if __name__ == "__main__":
    unittest.main()
