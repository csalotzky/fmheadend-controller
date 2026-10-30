"""Offline bench attenuation contract; never opens a device."""
from pathlib import Path
import struct
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from bench_support import protocol as p

try:
    from bench_support.client import TxConfig
except ModuleNotFoundError as exc:
    if exc.name != 'pydantic':
        raise
    TxConfig = None  # Optional bench dependency; protocol tests need only stdlib.


def configuration(level):
    return dict(frequency_khz=98100, rf_attenuation_db=level, stereo=True,
                preemphasis_us=50, audio_mode='DIGITAL', rds_enabled=False,
                gain_mode='AUTO', analog_gain_db=0.0, digital_trim_db=0,
                soft_clipping=False)


def status_payload(level):
    return (p.GLOBAL_STATUS.pack(0, 0, 0, 0) +
            p.CHANNEL_STATUS.pack(0, level, 0, 0, 64, 0, 255) * 8)


class BenchAttenuation(unittest.TestCase):
    def test_every_request_preserves_wire_tenths(self):
        for wire in range(406):
            with self.subTest(wire=wire):
                value = wire / 10
                self.assertEqual(p.rf_attenuation_x10(value), wire)
                payload = p.tx_config(SimpleNamespace(**configuration(value)))
                self.assertEqual(len(payload), 15)
                self.assertEqual(payload[4:6], struct.pack('<H', wire))
                frame = p.request(p.Command.SET_TX_CONFIG, 0, payload)
                self.assertEqual(len(frame), 20)
                self.assertEqual(frame[:3], bytes((2, 0, 15)))
                self.assertEqual(frame[3:-2], payload)
                self.assertEqual(int.from_bytes(frame[-2:], 'little'), p.crc16(frame[:-2]))
                # Independent nearest-level oracle, not IPOW/PAGAIN logic.
                expected = min(range(0, 406, 15), key=lambda level: abs(level - wire))
                self.assertEqual(p.quantized_rf_attenuation_db(value), expected / 10)

    def test_invalid_requests(self):
        for value in (-0.1, 40.6, 43, 6553.6, 0.01, 1.55,
                      float('nan'), float('inf'), -float('inf'), True, False, '6', None):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    p.tx_config(SimpleNamespace(**configuration(value)))
                with self.assertRaises(ValueError):
                    p.quantized_rf_attenuation_db(value)
        for wire in range(406, 65536):
            payload = p.TX_CONFIG.pack(98100, wire, 1, 50, 0, 0, 0, 0, 0, 0)
            with self.assertRaises(ValueError):
                p.request(p.Command.SET_TX_CONFIG, 0, payload)

    def test_status_requires_nominal_hardware_steps(self):
        for level in range(406):
            with self.subTest(level=level):
                if level % 15:
                    with self.assertRaises(p.ProtocolError):
                        p.parse_status(status_payload(level))
                else:
                    decoded = p.parse_status(status_payload(level))
                    self.assertEqual([row['rf_attenuation_db'] for row in decoded['modulators']],
                                     [level / 10] * 8)
        for level in (-32767, -1, 406, 420, 430, 32767):
            with self.assertRaises(p.ProtocolError):
                p.parse_status(status_payload(level))
        decoded = p.parse_status(status_payload(-32768))
        self.assertTrue(all(row['rf_attenuation_db'] is None for row in decoded['modulators']))

    @unittest.skipIf(TxConfig is None, 'Install tools/requirements-bench.txt for model validation')
    def test_bench_model(self):
        for wire in range(406):
            model = TxConfig(**configuration(wire / 10))
            self.assertEqual(p.tx_config(model)[4:6], struct.pack('<H', wire))
        for value in (-0.1, 40.6, 43, 0.01, 1.55, float('nan'), float('inf'), True, '6'):
            with self.subTest(value=value), self.assertRaises(ValueError):
                TxConfig(**configuration(value))
        self.assertIsNone(TxConfig(**configuration(None)).rf_attenuation_db)


if __name__ == '__main__':
    unittest.main()
