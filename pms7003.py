"""
PM 2.5 Sensor PMS-7003

Data Packet is 32 bytes
Baud rate: 9600, Check Bit: None, Stop Bit: 1

01 02     uint16    Start bytes: \x42 \x4d
03 04     uint16    Frame length: 2x13+2(data+check bytes)
05 06     uint16    PM1.0 concentration unit μg/m3 (CF=1，standard particle)
07 08     uint16    PM2.5 concentration unit μg/m3 (CF=1，standard particle)
09 10     uint16    PM10 concentration unit μg/m3 (CF=1，standard particle)
11 12     uint16    PM1.0 concentration unit μg/m3 (under atmospheric environment)
13 14     uint16    PM2.5 concentration unit μg/m3 (under atmospheric environment)
15 16     uint16    PM10 concentration unit μg/m3 (under atmospheric environment)
17 18     uint16    Number of particles >0.3um in 0.1L of air
19 20     uint16    Number of particles >0.5um in 0.1L of air
21 22     uint16    Number of particles >1um in 0.1L of air
23 24     uint16    Number of particles >2.5um in 0.1L of air
25 26     uint16    Number of particles >5um in 0.1L of air
27 28     uint16    Number of particles >10um in 0.1L of air
29        uint8     Reserved: Sources suggest Version Number
30        uint8     Reserved: Sources suggest Error Code
31 32     uint16    Checksum: byte 01+02+03+ .. 30. Low 8 bits.

https://download.kamami.com/p564008-p564008-PMS7003%20series%20data%20manua_English_V2.5.pdf
"""

import struct

from time import sleep
from machine import UART  # pylint: disable=import-error


class PMS7003Error(Exception):
    pass


class CommunicationError(PMS7003Error):
    pass


class DataIntegrityError(PMS7003Error):
    pass


class PMS7003:
    START_SEQUENCE = bytearray(b"\x42\x4d")

    CMD_READ = b"\xe2"
    CMD_MODE = b"\xe1"
    CMD_SLEEP = b"\xe4"

    MODE_PASSIVE = b"\x00\x00"
    MODE_ACTIVE = b"\x00\x01"

    MODE_SLEEP = b"\x00\x00"
    MODE_WAKE = b"\x00\x01"

    # Number of bytes to read from serial, without finding start sequence before reporting an error
    MAX_ITERATION_TIME = 64

    # Number of seconds to sleep between each serial read when unable to find the start sequence
    # TODO: This isn't ideal - serial may be moving much faster than this
    # we should poll fast for a set time period instead
    ITERATION_SLEEP_INTERVAL = 0.1

    def __init__(self, uart_index, tx_pin=None, rx_pin=None, passive=False, timeout=0):
        self._sensor = UART(uart_index, tx=tx_pin, rx=rx_pin)
        self._sensor.init(9600, bits=8, parity=None, stop=1)

        if passive:
            # Put the sensor into passive mode
            self.set_passive()
        else:
            self.set_active()

        self._passive = passive
        self._timeout = timeout

        self._data = None

    def set_passive(self):
        self._write(self.CMD_MODE, self.MODE_PASSIVE)

        # Read the entire buffer, to empty out the active junk from poweron
        self._sensor.read()


    def set_active(self):
        self._write(self.CMD_MODE, self.MODE_ACTIVE)

    def sleep(self):
        self._write(self.CMD_SLEEP, self.MODE_SLEEP)

    def wake(self):
        self._write(self.CMD_SLEEP, self.MODE_WAKE)

    def get_sensor_data(self):
        """
        Retrieve readings from the sensor running in active mode

        :returns: None
        :raises CommunicationError: Unable to communicate with sensor, no start sequence found
        :raises DataIntegrityError: Data received is invalid, checksums do not match
        """
        if self._passive:
            self._write(self.CMD_READ, b"\x00\x00")
            self._read()
        else:
            self._read()

    def _write(self, command, data):
        # 01 02     uint16  Start bytes: \x42 \x4d
        # 03        uint8   Command
        # 04 05     uint16  Data
        # 06 07     uint16  Checksum: byte 01+02+03+ .. 5.
        packet = bytearray(self.START_SEQUENCE)

        packet.extend(command)  # uint8  command
        packet.extend(data)     # uint16 data

        checksum = b"\xFF\xFF"
        checksum_int = self._checksum(packet + checksum)
        checksum = struct.pack('>H', checksum_int)

        packet.extend(checksum)  # uint16 checksum

        # Write packet to sensor
        bytes_written = self._sensor.write(packet)

        # Sleep to allow time for the sensor to send response after write
        sleep(0.5)

        return bytes_written

    def _read(self):
        """
        Retrieve readings from the sensor by polling UART

        :returns: None
        :raises CommunicationError: Unable to communicate with sensor, no start sequence found
        :raises DataIntegrityError: Data received is invalid, checksums do not match
        """
        count = 0

        self._data = None

        print("UART buffer depth: {}".format(self._sensor.any()))

        _previous_read = None

        while True:
            count += 1

            # Make this time-based - 3 seconds at least, otherwise it is hard to know
            # Spec says it can take >2 seconds between reading
            # OR turn to passive mode and request data as needed (maybe better option for this use case)
            # Allow the choice in library
            if count > self.MAX_ITERATION_TIME:
                raise CommunicationError("Unable to communicate with sensor")

            # Read 1 byte from the wire
            _raw_read = self._sensor.read(1)

            # If we do not already have the first byte of the start sequence, do not process anything more
            if not _previous_read or _previous_read != self.START_SEQUENCE[0:1]:
                _previous_read = _raw_read
                continue

            # If we do not have the start sequence, continue checking in 100ms
            if not _raw_read or _raw_read != self.START_SEQUENCE[1:2]:
                sleep(0.1)
                continue

            packet = bytearray()
            packet.extend(_previous_read)
            packet.extend(_raw_read)

            # Read the next 30 bytes from the sensor, see packet struct at start for details
            if not self._passive:
                # If in active mode, sleep to allow the full 30 bytes to be buffered
                sleep(0.5)

            packet.extend(self._sensor.read(30))

            checksum_received = int(packet[-1])
            checksum_derived = self._checksum(packet) & 0xFF  # Low 8 bits of checksum

            if checksum_received != checksum_derived:
                raise DataIntegrityError(
                    "Checksum mismatch: received {}, derived {}".format(hex(checksum_received), hex(checksum_derived))
                )

            # Validate that the readings are not all zero
            # Some of the hardware sensors start with all-zero readings, this appears to be a bug - restarting fixes
            if sum(packet[2:-4]) == 0:
                raise DataIntegrityError(
                    "Sensor is returning all-zero values, which seems unlikely outside of a lab"
                )

            # We have already validated the packet checksum, so length should be good already
            # Unpack as uint16 (H) and uint8 (B) as appropriate
            self._data = struct.unpack(">HHHHHHHHHHHHHBBH", packet[2:])

            # Handle the case where the sensor returns an error code
            error_code = self.error
            if error_code:
                self._data = None
                raise DataIntegrityError("An error was reported from the sensor, code: {}".format(error_code))

            return

    def _checksum(self, packet):
        # Sum the first 30 bytes of the packet. Return int
        checksum = sum(packet[0:-2])
        return int(checksum)

    @property
    def pm1_0_cf_1(self):
        try:
            return self._data[1]
        except TypeError:
            return None

    @property
    def pm2_5_cf_1(self):
        try:
            return self._data[2]
        except TypeError:
            return None

    @property
    def pm10_cf_1(self):
        try:
            return self._data[3]
        except TypeError:
            return None

    @property
    def pm1_0(self):
        try:
            return self._data[4]
        except TypeError:
            return None

    @property
    def pm2_5(self):
        try:
            return self._data[5]
        except TypeError:
            return None

    @property
    def pm10(self):
        try:
            return self._data[6]
        except TypeError:
            return None

    @property
    def raw_gt_0_3(self):
        try:
            return self._data[7]
        except TypeError:
            return None

    @property
    def raw_gt_0_5(self):
        try:
            return self._data[8]
        except TypeError:
            return None

    @property
    def raw_gt_1_0(self):
        try:
            return self._data[9]
        except TypeError:
            return None

    @property
    def raw_gt_2_5(self):
        try:
            return self._data[10]
        except TypeError:
            return None

    @property
    def raw_gt_5_0(self):
        try:
            return self._data[11]
        except TypeError:
            return None

    @property
    def raw_gt_10(self):
        try:
            return self._data[12]
        except TypeError:
            return None

    @property
    def version(self):
        try:
            return self._data[13]
        except TypeError:
            return None

    @property
    def error(self):
        try:
            return self._data[14]
        except TypeError:
            return None
