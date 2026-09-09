from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class FirmwareProtocolTests(unittest.TestCase):
    @unittest.skipUnless(shutil.which('gcc'), 'Host GCC unavailable')
    def test_compiled_parser_and_dma_framing(self):
        with tempfile.TemporaryDirectory() as folder:
            binary = str(Path(folder) / 'protocol.exe')
            subprocess.run(['gcc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                            '-I' + str(ROOT / 'tests/firmware_mocks'),
                            '-I' + str(ROOT / 'firmware/Core/Inc'),
                            str(ROOT / 'firmware/Core/Src/transmit.c'),
                            str(ROOT / 'tests/firmware_protocol_test.c'),
                            '-o', binary], check=True)
            subprocess.run([binary], check=True)
