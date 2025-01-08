# SPDX-FileCopyrightText: 2017 Tony DiCola for Adafruit Industries
#
# SPDX-License-Identifier: MIT

"""
`adafruit_tsl2591`
====================================================

CircuitPython module for the TSL2591 precision light sensor.  See
examples/simpletest.py for a demo of the usage.

* Author(s): Tony DiCola

Implementation Notes
--------------------

**Hardware:**

* Adafruit `TSL2591 High Dynamic Range Digital Light Sensor
  <https://www.adafruit.com/product/1980>`_ (Product ID: 1980)

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://circuitpython.org/downloads

 * Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
"""
from micropython import const

from adafruit_bus_device import i2c_device

try:
    from typing import Tuple
    from busio import I2C
except ImportError:
    pass

__version__ = "0.0.0+auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_TSL2591.git"

# Internal constants:
_TSL2591_ADDR = const(0x29)
_TSL2591_COMMAND_BIT = const(0xA0)
_TSL2591_SPECIAL_BIT = const(0xE0)
_TSL2591_ENABLE_POWEROFF = const(0x00)
_TSL2591_ENABLE_POWERON = const(0x01)
_TSL2591_ENABLE_AEN = const(0x02)

_TSL2591_REGISTER_ENABLE = const(0x00)
_TSL2591_REGISTER_CONTROL = const(0x01)

_TSL2591_AILTL = const(0x04)
_TSL2591_AILTH = const(0x05)
_TSL2591_AIHTL = const(0x06)
_TSL2591_AIHTH = const(0x07)
_TSL2591_NPAILTL = const(0x08)
_TSL2591_NPAILTH = const(0x09)
_TSL2591_NPAIHTL = const(0x0A)
_TSL2591_NPAIHTH = const(0x0B)
_TSL2591_PERSIST_FILTER = const(0x0C)

_TSL2591_REGISTER_DEVICE_ID = const(0x12)
_TSL2591_REGISTER_CHAN0_LOW = const(0x14)
_TSL2591_REGISTER_CHAN1_LOW = const(0x16)
_TSL2591_LUX_DF = 408.0
_TSL2591_LUX_COEFB = 1.64
_TSL2591_LUX_COEFC = 0.59
_TSL2591_LUX_COEFD = 0.86
_TSL2591_MAX_COUNT_100MS = const(36863)  # 0x8FFF
_TSL2591_MAX_COUNT = const(65535)  # 0xFFFF

# User-facing constants:
GAIN_LOW = 0x00  # low gain (1x)
"""Low gain (1x)"""
GAIN_MED = 0x10  # medium gain (25x)
"""Medium gain (25x)"""
GAIN_HIGH = 0x20  # medium gain (428x)
"""High gain (428x)"""
GAIN_MAX = 0x30  # max gain (9876x)
"""Max gain (9876x)"""
INTEGRATIONTIME_100MS = 0x00  # 100 millis
"""100 millis"""
INTEGRATIONTIME_200MS = 0x01  # 200 millis
"""200 millis"""
INTEGRATIONTIME_300MS = 0x02  # 300 millis
"""300 millis"""
INTEGRATIONTIME_400MS = 0x03  # 400 millis
"""400 millis"""
INTEGRATIONTIME_500MS = 0x04  # 500 millis
"""500 millis"""
INTEGRATIONTIME_600MS = 0x05  # 600 millis
"""600 millis"""
CLEAR_INTERRUPT = const(0x06)
"""Clears ALS interrupt"""
CLEAR_ALL_INTERRUPTS = const(0x07)
"""Clears ALS and no persist ALS interrupt"""
CLEAR_PERSIST_INTERRUPT = const(0x0A)
"""Clears no persist ALS interrupt"""
ENABLE_AIEN = const(0x10)
"""ALS Interrupt Enable. When asserted permits ALS interrupts to be generated."""
ENABLE_NPIEN = const(0x80)
"""No Persist Interrupt Enable. When asserted NP Threshold conditions will generate an interrupt."""
ENABLE_NPAIEN = const(0x10 | 0x80)
"""ALS and No Persist Interrupt Enable."""


class TSL2591:
    """TSL2591 high precision light sensor.

    :param ~busio.I2C i2c: The I2C bus the device is connected to
    :param int address: The I2C device address. Defaults to :const:`0x29`


    **Quickstart: Importing and using the device**

        Here is an example of using the :class:`TSL2591` class.
        First you will need to import the libraries to use the sensor

        .. code-block:: python

            import board
            import adafruit_tsl2591

        Once this is done you can define your `board.I2C` object and define your sensor object

        .. code-block:: python

            i2c = board.I2C()  # uses board.SCL and board.SDA
            sensor = adafruit_tsl2591.TSL2591(i2c)

        Now you have access to the :attr:`lux`, :attr:`infrared`
        :attr:`visible` and :attr:`full_spectrum` attributes

        .. code-block:: python

            lux = sensor.lux
            infrared = sensor.infrared
            visible = sensor.visible
            full_spectrum = sensor.full_spectrum

    """

    # Class-level buffer to reduce memory usage and allocations.
    # Note this is NOT thread-safe or re-entrant by design.
    _BUFFER = bytearray(2)

    def __init__(self, i2c: I2C, address: int = _TSL2591_ADDR) -> None:
        self._integration_time = 0
        self._gain = 0
        self._device = i2c_device.I2CDevice(i2c, address)
        # Verify the chip ID.
        if self._read_u8(_TSL2591_REGISTER_DEVICE_ID) != 0x50:
            raise RuntimeError("Failed to find TSL2591, check wiring!")
        # Set default gain and integration times.
        self.gain = GAIN_MED
        self.integration_time = INTEGRATIONTIME_100MS
        # Put the device in a powered on state after initialization.
        self.enable()

    def _read_u8(self, address: int) -> int:
        # Read an 8-bit unsigned value from the specified 8-bit address.
        with self._device as i2c:
            # Make sure to add command bit to read request.
            self._BUFFER[0] = (_TSL2591_COMMAND_BIT | address) & 0xFF
            i2c.write_then_readinto(self._BUFFER, self._BUFFER, out_end=1, in_end=1)
        return self._BUFFER[0]

    # Disable invalid name check since pylint isn't smart enough to know LE
    # is an abbreviation for little-endian.
    # pylint: disable=invalid-name
    def _read_u16LE(self, address: int) -> int:
        # Read a 16-bit little-endian unsigned value from the specified 8-bit
        # address.
        with self._device as i2c:
            # Make sure to add command bit to read request.
            self._BUFFER[0] = (_TSL2591_COMMAND_BIT | address) & 0xFF
            i2c.write_then_readinto(self._BUFFER, self._BUFFER, out_end=1, in_end=2)
        return (self._BUFFER[1] << 8) | self._BUFFER[0]

    # pylint: enable=invalid-name

    def _write_u8(self, address: int, val: int) -> None:
        # Write an 8-bit unsigned value to the specified 8-bit address.
        with self._device as i2c:
            # Make sure to add command bit to write request.
            self._BUFFER[0] = (_TSL2591_COMMAND_BIT | address) & 0xFF
            self._BUFFER[1] = val & 0xFF
            i2c.write(self._BUFFER, end=2)

    def enable(self) -> None:
        """Put the device in a powered, enabled mode."""
        self._write_u8(
            _TSL2591_REGISTER_ENABLE,
            _TSL2591_ENABLE_POWERON | _TSL2591_ENABLE_AEN,
        )

    def disable(self) -> None:
        """Disable the device and go into low power mode."""
        self._write_u8(_TSL2591_REGISTER_ENABLE, _TSL2591_ENABLE_POWEROFF)

    def clear_interrupt(self, operation: int) -> None:
        """Send special function command to control interrupt bits in the status register (0x13).
        Can be a value of:
        - ``CLEAR_INTERRUPT``
        - ``CLEAR_ALL_INTERRUPTS``
        - ``CLEAR_PERSIST_INTERRUPT``
        """
        assert operation in (
            CLEAR_INTERRUPT,
            CLEAR_ALL_INTERRUPTS,
            CLEAR_PERSIST_INTERRUPT,
        )
        control = (_TSL2591_SPECIAL_BIT | operation) & 0xFF
        with self._device as i2c:
            self._BUFFER[0] = control
            i2c.write(self._BUFFER, end=1)

    def enable_interrupt(self, interrupts: int) -> None:
        """Enable interrupts on device. ENABLE_NPIEN will turn on No Persist interrupts, these
        bypass the persist filter and assert immediately when values are detected above the high
        threshold or below the low threshold. Similarly, ENABLE_AIEN will assert at the respective
        ALS thresholds, but only after the values persist longer than the persist filter cycle
        duration. The device powers on with thresholds at 0, meaning enabling interrupts may
        cause an immediate assertion.
        Can be a value of:
        - ``ENABLE_NPIEN``
        - ``ENABLE_AIEN``
        - ``ENABLE_NPAIEN``
        """
        assert interrupts in (ENABLE_NPIEN, ENABLE_AIEN, (ENABLE_NPIEN | ENABLE_AIEN))
        functions = self._read_u8(_TSL2591_REGISTER_ENABLE)
        functions = (functions | interrupts) & 0xFF
        self._write_u8(_TSL2591_REGISTER_ENABLE, functions)

    def disable_interrupt(self, interrupts: int) -> None:
        """Disables the requested interrupts.
        Can be a value of:
        - ``ENABLE_NPIEN``
        - ``ENABLE_AIEN``
        - ``ENABLE_NPAIEN``
        """
        assert interrupts in (ENABLE_NPIEN, ENABLE_AIEN, (ENABLE_NPIEN | ENABLE_AIEN))
        functions = self._read_u8(_TSL2591_REGISTER_ENABLE)
        functions = (functions & ~interrupts) & 0xFF
        self._write_u8(_TSL2591_REGISTER_ENABLE, functions)

    @property
    def gain(self) -> int:
        """Get and set the gain of the sensor.  Can be a value of:

        - ``GAIN_LOW`` (1x)
        - ``GAIN_MED`` (25x)
        - ``GAIN_HIGH`` (428x)
        - ``GAIN_MAX`` (9876x)
        """
        control = self._read_u8(_TSL2591_REGISTER_CONTROL)
        return control & 0b00110000

    @gain.setter
    def gain(self, val: int) -> None:
        assert val in (GAIN_LOW, GAIN_MED, GAIN_HIGH, GAIN_MAX)
        # Set appropriate gain value.
        control = self._read_u8(_TSL2591_REGISTER_CONTROL)
        control &= 0b11001111
        control |= val
        self._write_u8(_TSL2591_REGISTER_CONTROL, control)
        # Keep track of gain for future lux calculations.
        self._gain = val

    @property
    def integration_time(self) -> int:
        """Get and set the integration time of the sensor.  Can be a value of:

        - ``INTEGRATIONTIME_100MS`` (100 millis)
        - ``INTEGRATIONTIME_200MS`` (200 millis)
        - ``INTEGRATIONTIME_300MS`` (300 millis)
        - ``INTEGRATIONTIME_400MS`` (400 millis)
        - ``INTEGRATIONTIME_500MS`` (500 millis)
        - ``INTEGRATIONTIME_600MS`` (600 millis)
        """
        control = self._read_u8(_TSL2591_REGISTER_CONTROL)
        return control & 0b00000111

    @integration_time.setter
    def integration_time(self, val: int) -> None:
        assert 0 <= val <= 5
        # Set control bits appropriately.
        control = self._read_u8(_TSL2591_REGISTER_CONTROL)
        control &= 0b11111000
        control |= val
        self._write_u8(_TSL2591_REGISTER_CONTROL, control)
        # Keep track of integration time for future reading delay times.
        self._integration_time = val

    @property
    def threshold_low(self) -> int:
        """Get and set the ALS interrupt low threshold bytes. If the detected value is below
        the low threshold for the number of persist filter cycles an interrupt will be triggered.
        Can be 16-bit value."""
        th_low = self._read_u16LE(_TSL2591_AILTL)
        return th_low

    @threshold_low.setter
    def threshold_low(self, val: int) -> None:
        lower = val & 0xFF
        upper = (val >> 8) & 0xFF
        self._write_u8(_TSL2591_AILTL, lower)
        self._write_u8(_TSL2591_AILTH, upper)

    @property
    def threshold_high(self) -> int:
        """Get and set the ALS interrupt high threshold bytes. If the detected value is above
        the high threshold for the number of persist filter cycles an interrupt will be triggered.
        Can be 16-bit value."""
        th_high = self._read_u16LE(_TSL2591_AIHTL)
        return th_high

    @threshold_high.setter
    def threshold_high(self, val: int) -> None:
        lower = val & 0xFF
        upper = (val >> 8) & 0xFF
        self._write_u8(_TSL2591_AIHTL, lower)
        self._write_u8(_TSL2591_AIHTH, upper)

    @property
    def nopersist_threshold_low(self) -> int:
        """Get and set the No Persist ALS low threshold bytes. An interrupt will be triggered
        immediately once the detected value is below the low threshold. Can be 16-bit value.
        """
        np_th_low = self._read_u16LE(_TSL2591_NPAILTL)
        return np_th_low

    @nopersist_threshold_low.setter
    def nopersist_threshold_low(self, val: int) -> None:
        lower = val & 0xFF
        upper = (val >> 8) & 0xFF
        self._write_u8(_TSL2591_NPAILTL, lower)
        self._write_u8(_TSL2591_NPAILTH, upper)

    @property
    def nopersist_threshold_high(self) -> int:
        """Get and set the No Persist ALS high threshold bytes. An interrupt will be triggered
        immediately once the detected value is above the high threshold. Can be 16-bit value.
        """
        np_th_high = self._read_u16LE(_TSL2591_NPAIHTL)
        return np_th_high

    @nopersist_threshold_high.setter
    def nopersist_threshold_high(self, val: int) -> None:
        lower = val & 0xFF
        upper = (val >> 8) & 0xFF
        self._write_u8(_TSL2591_NPAIHTL, lower)
        self._write_u8(_TSL2591_NPAIHTH, upper)

    @property
    def persist(self) -> int:
        """Get and set the interrupt persist filter - the number of consecutive out-of-range
        ALS cycles necessary to generate an interrupt. Valid persist values are 0 - 15 (inclusive),
        corresponding to a preset number of cycles. Only the 4 lower bits will be used to write
        to the device.
        Can be a value of:
        - ``0 (0000)`` - Every ALS cycle generates an interrupt.
        - ``1 (0001)`` - Any value outside of threshold range.
        - ``2 (0010)`` - 2 consecutive values out of range.
        - ``3 (0011)`` - 3 consecutive values out of range.
        - ``4 (0100)`` - 5 consecutive values out of range.
        - ``5 (0101)`` - 10 consecutive values out of range.
        - ``6 (0110)`` - 15 consecutive values out of range.
        - ``7 (0111)`` - 20 consecutive values out of range.
        - ``8 (1000)`` - 25 consecutive values out of range.
        - ``9 (1001)`` - 30 consecutive values out of range.
        - ``10 (1010)`` - 35 consecutive values out of range.
        - ``11 (1011)`` - 40 consecutive values out of range.
        - ``12 (1100)`` - 45 consecutive values out of range.
        - ``13 (1101)`` - 50 consecutive values out of range.
        - ``14 (1110)`` - 55 consecutive values out of range.
        - ``15 (1111)`` - 60 consecutive values out of range.
        """
        persist = self._read_u8(_TSL2591_PERSIST_FILTER)
        return persist & 0x0F

    @persist.setter
    def persist(self, val: int) -> None:
        assert 0 <= val <= 15
        persist = val & 0x0F
        self._write_u8(_TSL2591_PERSIST_FILTER, persist)

    @property
    def raw_luminosity(self) -> Tuple[int, int]:
        """Read the raw luminosity from the sensor (both IR + visible and IR
        only channels) and return a 2-tuple of those values.  The first value
        is IR + visible luminosity (channel 0) and the second is the IR only
        (channel 1).  Both values are 16-bit unsigned numbers (0-65535).
        """
        # Read both the luminosity channels.
        channel_0 = self._read_u16LE(_TSL2591_REGISTER_CHAN0_LOW)
        channel_1 = self._read_u16LE(_TSL2591_REGISTER_CHAN1_LOW)
        return (channel_0, channel_1)

    @property
    def full_spectrum(self) -> int:
        """Read the full spectrum (IR + visible) light and return its value
        as a 32-bit unsigned number.
        """
        channel_0, channel_1 = self.raw_luminosity
        return (channel_1 << 16) | channel_0

    @property
    def infrared(self) -> int:
        """Read the infrared light and return its value as a 16-bit unsigned number."""
        _, channel_1 = self.raw_luminosity
        return channel_1

    @property
    def visible(self) -> int:
        """Read the visible light and return its value as a 32-bit unsigned number."""
        channel_0, channel_1 = self.raw_luminosity
        full = (channel_1 << 16) | channel_0
        return full - channel_1

    @property
    def lux(self) -> float:
        """Read the sensor and calculate a lux value from both its infrared
        and visible light channels.

        .. note::
            :attr:`lux` is not calibrated!

        """
        channel_0, channel_1 = self.raw_luminosity

        # Compute the atime in milliseconds
        atime = 100.0 * self._integration_time + 100.0

        # Set the maximum sensor counts based on the integration time (atime) setting
        if self._integration_time == INTEGRATIONTIME_100MS:
            max_counts = _TSL2591_MAX_COUNT_100MS
        else:
            max_counts = _TSL2591_MAX_COUNT

        # Handle overflow.
        if channel_0 >= max_counts or channel_1 >= max_counts:
            message = (
                "Overflow reading light channels!, Try to reduce the gain of\n "
                + "the sensor using adafruit_tsl2591.GAIN_LOW"
            )
            raise RuntimeError(message)
        # Calculate lux using same equation as Arduino library:
        #  https://github.com/adafruit/Adafruit_TSL2591_Library/blob/master/Adafruit_TSL2591.cpp
        again = 1.0
        if self._gain == GAIN_MED:
            again = 25.0
        elif self._gain == GAIN_HIGH:
            again = 428.0
        elif self._gain == GAIN_MAX:
            again = 9876.0
        cpl = (atime * again) / _TSL2591_LUX_DF
        lux1 = (channel_0 - (_TSL2591_LUX_COEFB * channel_1)) / cpl
        lux2 = (
            (_TSL2591_LUX_COEFC * channel_0) - (_TSL2591_LUX_COEFD * channel_1)
        ) / cpl
        return max(lux1, lux2)
