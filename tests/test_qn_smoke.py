"""Exercise the no-RF bench command with a fake Linux client, no dependencies."""
import contextlib
import importlib.util
import io
from pathlib import Path
import sys
import types


sys.path.insert(0, str(Path(__file__).parents[1] / 'tools'))
from bench_support.protocol import Command, InvalidArgument, Status


class Config:
    def __init__(self, **values):
        self.__dict__.update(values)

    def model_copy(self, update):
        return Config(**(self.__dict__ | update))


class Client:
    calls = []
    level = 22.5
    version = "1.0.0"

    def __init__(self, device, address):
        assert device == '/dev/i2c-1' and address == 0x62

    def get_info(self):
        return {'rp_firmware_version': self.version}

    def get_status(self):
        return {'state': 'OK', 'rp_spi_audio_link_error_count': 0,
                'modulators': [{'id': n, 'tx_state': 'OFF', 'rf_attenuation_db': self.level}
                               for n in range(8)]}

    def qn_read_register(self, tx, reg):
        assert tx in range(8) and reg == 6
        return 0x11

    def set_tx_enabled(self, tx, enabled):
        assert tx == 0 and not enabled  # This command must never turn RF ON.
        self.calls.append(('off', tx))

    def set_tx_config(self, tx, config):
        assert tx == 0 and config.audio_mode == 'DIGITAL'
        if config.frequency_khz == 98101:
            raise InvalidArgument(Command.SET_TX_CONFIG, Status.INVALID_ARGUMENT)
        assert config.frequency_khz == 98100
        self.calls.append(('configured', tx))

    def close(self):
        self.calls.append(('closed',))


for name, values in {
    'bench_support.linux_client': {'Rp2354Client': Client},
    'bench_support.client': {'TxConfig': Config},
}.items():
    fake = types.ModuleType(name)
    fake.__dict__.update(values)
    sys.modules[name] = fake
spec = importlib.util.spec_from_file_location('qn_smoke', Path(__file__).parents[1] / 'tools/qn_smoke.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
for version in ('0.8.5', '1.0.0'):
    Client.version = version
    for requested, expected in [(0, 0), (0.7, 0), (0.8, 1.5), (1, 1.5),
                                (22.5, 22.5), (23.2, 22.5), (23.3, 24),
                                (30, 30), (40.4, 40.5), (40.5, 40.5)]:
        Client.calls = []
        Client.level = expected
        sys.argv = ['qn_smoke', '--address', '0x62', 'configure', '--tx', '0',
                    '--frequency-khz', '98100', '--attenuation-db', str(requested), '--mode', 'DIGITAL']
        with contextlib.redirect_stdout(io.StringIO()):
            module.main()
        assert Client.calls == [('off', 0), ('configured', 0), ('off', 0), ('closed',)]
    # A valid nominal value that is wrong for this request must still fail.
    Client.level = 39
    Client.calls = []
    with contextlib.redirect_stdout(io.StringIO()):
        try:
            module.main()
        except RuntimeError as exc:
            assert 'Attenuation readback' in str(exc)
        else:
            raise AssertionError('Incorrect quantized readback was accepted')
    assert Client.calls[-2:] == [('off', 0), ('closed',)]
print('No-RF bench configure: quantized readback, validation, never ON PASS')
