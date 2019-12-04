from mock import MagicMock
import sys

import pytest

# pylint: disable=wrong-import-position
sys.modules['machine'] = MagicMock()
from pms7003 import PMS7003, CommunicationError, DataIntegrityError  # noqa


class Test_PMS7003:
    @pytest.fixture
    def fixture_sensor(self, mocker):
        self.mock_uart = mocker.patch('pms7003.UART')
        mocker.patch('pms7003.sleep')

        sensor = PMS7003(1)
        sensor.MAX_ITERATION_TIME = 5

        return sensor

    def assert_valid_sensor_data(self, sensor):
        assert sensor.pm1_0 == 5
        assert sensor.pm2_5 == 8
        assert sensor.pm10 == 35
        assert sensor.pm1_0_cf_1 == 5
        assert sensor.pm2_5_cf_1 == 8
        assert sensor.pm10_cf_1 == 35
        assert sensor.raw_gt_0_3 == 48
        assert sensor.raw_gt_0_5 == 12
        assert sensor.raw_gt_1_0 == 1
        assert sensor.raw_gt_2_5 == 0
        assert sensor.raw_gt_5_0 == 0
        assert sensor.raw_gt_10 == 0

        assert sensor.version == 152
        assert sensor.error == 0

    def test_sensor_initialisation(self, fixture_sensor):

        self.mock_uart.assert_called_once_with(1, rx=None, tx=None)
        self.mock_uart().init.assert_called_once_with(9600, bits=8, parity=None, stop=1)

        self.mock_uart().write.assert_called_once_with(
            PMS7003.START_SEQUENCE + PMS7003.CMD_MODE + PMS7003.MODE_ACTIVE + b'\x01\x71'
        )

    def test_get_sensor_good_read(self, fixture_sensor, mocker):
        self.mock_uart().read.side_effect = [
            bytes(PMS7003.START_SEQUENCE[0:1]),
            bytes(PMS7003.START_SEQUENCE[1:2]),
            b'\x00\x1c\x00\x05\x00\x08\x00\x23\x00\x05\x00\x08\x00\x23\x00\x30\x00\x0c\x00\x01\x00\x00\x00\x00\x00\x00\x98\x00\x01\xe0'
        ]

        fixture_sensor.get_sensor_data()

        self.mock_uart().write.call_count = 1

        self.assert_valid_sensor_data(fixture_sensor)

    def test_get_sensor_data_timeout(self, fixture_sensor, mocker):
        self.mock_uart().read.return_value = b'\x00\x80'

        with pytest.raises(CommunicationError):
            fixture_sensor.get_sensor_data()

    def test_read_all_zeros(self, fixture_sensor, mocker):
        self.mock_uart().read.side_effect = [
            bytes(PMS7003.START_SEQUENCE[0:1]),
            bytes(PMS7003.START_SEQUENCE[1:2]),
            b'\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x98\x00\x01\x27'
        ]

        with pytest.raises(DataIntegrityError):
            fixture_sensor.get_sensor_data()

    def test_start_sequence_garbage_first(self, fixture_sensor, mocker):
        """
        We should handle the case where the start sequence within the read buffer are not the first 2 bytes read
        Otherwise we will have many communication errors
        """
        MOCK_START_SEQUENCE = bytearray(b'\x01')
        MOCK_START_SEQUENCE.extend(PMS7003.START_SEQUENCE)

        self.mock_uart().read.side_effect = [
            b'\x00',
            b'\x06',
            bytes(PMS7003.START_SEQUENCE[0:1]),
            bytes(PMS7003.START_SEQUENCE[1:2]),
            b'\x00\x1c\x00\x05\x00\x08\x00\x23\x00\x05\x00\x08\x00\x23\x00\x30\x00\x0c\x00\x01\x00\x00\x00\x00\x00\x00\x98\x00\x01\xe0'
        ]

        fixture_sensor.get_sensor_data()

        self.assert_valid_sensor_data(fixture_sensor)

    def test_partial_start_sequence_continues(self, fixture_sensor, mocker):
        """
        Handle the case where we fail to read a byte, or get invalid data midway through the start sequence
        We should continue and successfully read the next value
        """
        MOCK_START_SEQUENCE = bytearray(b'\x01')
        MOCK_START_SEQUENCE.extend(PMS7003.START_SEQUENCE)

        self.mock_uart().read.side_effect = [
            bytes(PMS7003.START_SEQUENCE[0:1]),
            b'\x00',
            None,
            bytes(PMS7003.START_SEQUENCE[0:1]),
            bytes(PMS7003.START_SEQUENCE[1:2]),
            b'\x00\x1c\x00\x05\x00\x08\x00\x23\x00\x05\x00\x08\x00\x23\x00\x30\x00\x0c\x00\x01\x00\x00\x00\x00\x00\x00\x98\x00\x01\xe0'
        ]

        fixture_sensor.get_sensor_data()

        self.assert_valid_sensor_data(fixture_sensor)

    def test_no_data_returns_none(self, fixture_sensor, mocker):
        assert fixture_sensor.pm1_0 == None
        assert fixture_sensor.pm2_5 == None
        assert fixture_sensor.pm10 == None
        assert fixture_sensor.pm1_0_cf_1 == None
        assert fixture_sensor.pm2_5_cf_1 == None
        assert fixture_sensor.pm10_cf_1 == None
        assert fixture_sensor.raw_gt_0_3 == None
        assert fixture_sensor.raw_gt_0_5 == None
        assert fixture_sensor.raw_gt_1_0 == None
        assert fixture_sensor.raw_gt_2_5 == None
        assert fixture_sensor.raw_gt_5_0 == None
        assert fixture_sensor.raw_gt_10 == None

        assert fixture_sensor.version == None
        assert fixture_sensor.error == None

    def test_read_returns_nothing(self, fixture_sensor, mocker):
        """
        Read never returns any value, we should timeout with a CommunicationError
        """
        self.mock_uart().read.return_value = None

        with pytest.raises(CommunicationError):
            fixture_sensor.get_sensor_data()

    def test_get_sensor_error_code_field(self, fixture_sensor, mocker):
        self.mock_uart().read.side_effect = [
            bytes(PMS7003.START_SEQUENCE[0:1]),
            bytes(PMS7003.START_SEQUENCE[1:2]),
            b'\x00\x1c\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x30\x00\x0c\x00\x01\x00\x00\x00\x00\x00\x00\x98\x08\x01\x88'
        ]

        with pytest.raises(DataIntegrityError):
            fixture_sensor.get_sensor_data()

    @pytest.mark.parametrize('data', [
        b'\x00\x1c\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x30\x00\x0c\x00\x01\x00\x00\x00\x00\x00\x00\x98\x00\x01\x70',
        b'\x00\x1c\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x30\x00\x0c\x00\x01\x00\x00\x00\x00\x00'
    ])
    def test_get_sensor_checksum_mismatch(self, data, fixture_sensor, mocker):
        self.mock_uart().read.side_effect = [
            bytes(PMS7003.START_SEQUENCE[0:1]),
            bytes(PMS7003.START_SEQUENCE[1:2]),
            data
        ]

        with pytest.raises(DataIntegrityError):
            fixture_sensor.get_sensor_data()


class Test_PMS7003_Passive_Commands:
    @pytest.fixture
    def fixture_sensor(self, mocker):
        self.mock_uart = mocker.patch('pms7003.UART')
        self.mock_sleep = mocker.patch('pms7003.sleep')

        sensor = PMS7003(1, passive=True)
        sensor.MAX_ITERATION_TIME = 5

        return sensor

    def test_sensor_passive_initialisation(self, fixture_sensor):

        self.mock_uart.assert_called_once_with(1, rx=None, tx=None)
        self.mock_uart().init.assert_called_once_with(9600, bits=8, parity=None, stop=1)

        self.mock_uart().write.assert_called_once_with(
            PMS7003.START_SEQUENCE + PMS7003.CMD_MODE + PMS7003.MODE_PASSIVE + b'\x01\x70'
        )

    def test_read_passive_success(self, fixture_sensor, mocker):
        """
        Assert that we send the correct read commands in passive mode, and check for response
        """
        mock_read = mocker.patch('pms7003.PMS7003._read')

        expected_calls = [
            mocker.call(PMS7003.START_SEQUENCE + PMS7003.CMD_READ + b'\x00\x00\x01\x71')
        ]

        self.mock_uart().write.return_value = 7

        fixture_sensor.get_sensor_data()

        self.mock_uart().write.call_count == 2
        self.mock_uart().write.assert_has_calls(expected_calls)

        mock_read.assert_called_once()


    def test_sleep(self, fixture_sensor, mocker):
        """
        Assert that we send the correct commands to send the device to sleep
        """
        expected_calls = [
            mocker.call(PMS7003.START_SEQUENCE + PMS7003.CMD_SLEEP + PMS7003.MODE_SLEEP + b'\x01\x73')
        ]

        fixture_sensor.sleep()

        self.mock_uart().write.call_count == 2
        self.mock_uart().write.assert_has_calls(expected_calls)


    def test_wake(self, fixture_sensor, mocker):
        """
        Assert that we send the correct commands to send the device to sleep
        """
        expected_calls = [
            mocker.call(PMS7003.START_SEQUENCE + PMS7003.CMD_SLEEP + PMS7003.MODE_WAKE + b'\x01\x74')
        ]

        fixture_sensor.wake()

        self.mock_uart().write.call_count == 2
        self.mock_uart().write.assert_has_calls(expected_calls)
