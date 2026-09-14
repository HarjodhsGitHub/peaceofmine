"""Offline regressions for GNSS/NTRIP framing and correction delivery."""
import io
import unittest
from unittest.mock import Mock

import pynmea2
from pyubx2 import UBXMessage, UBXReader, GET

from gnss_viewer import process_ubx, GnssState, process_sentence, read_ntrip_response, stream_corrections

GGA = str(pynmea2.GGA('GN', 'GGA', (
    '120000', '5920.0000', 'N', '01800.0000', 'E', '1', '12',
    '0.8', '10.0', 'M', '0.0', 'M', '', '')))
RTCM = b'\xd3\x00\x03\x01\x02\x03\x04\x05\x06'


def caster_with(*chunks):
    caster = Mock()
    caster.recv.side_effect = chunks
    return caster


class NtripTests(unittest.TestCase):
    def test_accuracy_units_and_invalid_fix(self):
        state = GnssState('fake', 115200)
        msg = UBXMessage('NAV', 'NAV-PVT', GET, gnssFixOk=1, fixType=3, hAcc=1250)
        process_ubx(state, UBXReader.parse(msg.serialize()))
        self.assertEqual(state.snapshot()['horizontal_accuracy_m'], 1.25)
        process_ubx(state, UBXMessage('NAV', 'NAV-PVT', GET, gnssFixOk=0, fixType=3, hAcc=100))
        self.assertIsNone(state.snapshot()['horizontal_accuracy_m'])

    def test_gst_horizontal_rms(self):
        state = GnssState('fake', 115200)
        msg = pynmea2.GST('GN', 'GST', ('120000', '1', '5', '3', '0', '3', '4', '6'))
        process_sentence(state, msg)
        self.assertEqual(state.snapshot()['horizontal_accuracy_m'], 5)

    def test_unrelated_messages_do_not_refresh_position(self):
        state = GnssState('fake', 115200)
        state.update(latitude=59.0, longitude=18.0)
        state.field_updated['latitude'] -= 20
        state.update(satellites_in_view=12)
        snapshot = state.snapshot()
        self.assertGreaterEqual(snapshot['field_age_s']['latitude'], 20)
        self.assertLess(snapshot['field_age_s']['satellites_in_view'], 1)

    def test_icy_single_line_does_not_wait_for_corrections(self):
        caster = caster_with(b'ICY 200 OK\r\n')
        self.assertEqual(read_ntrip_response(caster), b'')
        self.assertEqual(caster.recv.call_count, 1)

    def test_fragmented_icy_preserves_first_corrections(self):
        caster = caster_with(b'I', b'CY 200 OK\r', b'\n' + RTCM)
        self.assertEqual(read_ntrip_response(caster), RTCM)

    def test_http_preserves_first_corrections(self):
        caster = caster_with(b'HTTP/1.1 200 OK\r\nContent-Type: gnss/data\r\n', b'\r\n' + RTCM)
        self.assertEqual(read_ntrip_response(caster), RTCM)

    def test_rejected_auth_and_sourcetable(self):
        for response in (b'HTTP/1.0 401 Unauthorized\r\n',
                         b'HTTP/1.1 403 Forbidden\r\n',
                         b'SOURCETABLE 200 OK\r\n',
                         b'HTTP/1.1 200 OK\r\nContent-Type: gnss/sourcetable\r\n\r\n'):
            with self.subTest(response=response), self.assertRaises(RuntimeError):
                read_ntrip_response(caster_with(response))

    def test_disconnect_during_headers(self):
        for response in (b'HTTP/1.0', b'HTTP/1.0 200 OK\r\n'):
            with self.subTest(response=response), self.assertRaisesRegex(RuntimeError, 'closed'):
                read_ntrip_response(caster_with(response, b''))

    def test_gga_sent_before_receiving_and_all_corrections_forwarded(self):
        state = GnssState('fake', 115200)
        state.set_gga(GGA)
        port = Mock(is_open=True)
        port.write.side_effect = lambda data: len(data)
        state.set_port(port)
        caster = caster_with(RTCM, b'')
        with self.assertRaisesRegex(RuntimeError, 'closed'):
            stream_corrections(state, caster, RTCM)
        caster.sendall.assert_called_once_with((GGA + '\r\n').encode())
        self.assertEqual([call.args[0] for call in port.write.call_args_list], [RTCM, RTCM])
        self.assertEqual(state.snapshot()['corrections_bytes'], 2 * len(RTCM))

    def test_incomplete_serial_write_not_counted(self):
        state = GnssState('fake', 115200)
        state.set_gga(GGA)
        state.set_port(Mock(is_open=True, write=Mock(return_value=1)))
        with self.assertRaisesRegex(RuntimeError, 'Incomplete'):
            stream_corrections(state, Mock(), RTCM)
        self.assertEqual(state.snapshot()['corrections_bytes'], 0)

    def test_stale_and_disconnected_gga_cleared(self):
        state = GnssState('fake', 115200)
        state.set_gga(GGA)
        state.gga_time -= 16
        self.assertIsNone(state.get_gga())
        state.set_gga(GGA)
        state.set_port(None)
        self.assertIsNone(state.get_gga())

    def test_no_fix_gga_not_sent(self):
        state = GnssState('fake', 115200)
        sentence = pynmea2.parse(GGA)
        process_sentence(state, sentence, GGA)
        self.assertEqual(state.get_gga(), GGA)
        sentence.gps_qual = '0'
        process_sentence(state, sentence, str(sentence))
        self.assertIsNone(state.get_gga())

    def test_mixed_binary_and_nmea(self):
        binary = UBXMessage('ACK', 'ACK-ACK', GET, clsID=6, msgID=1).serialize()
        reader = UBXReader(io.BytesIO(binary + (GGA + '\r\n').encode()))
        self.assertEqual(reader.read()[0], binary)
        raw, _ = reader.read()
        state = GnssState('fake', 115200)
        process_sentence(state, pynmea2.parse(raw.decode()), raw.decode())
        self.assertEqual(state.get_gga(), GGA)
        self.assertEqual(state.snapshot()['fix_label'], 'GPS FIX')


if __name__ == '__main__':
    unittest.main()
