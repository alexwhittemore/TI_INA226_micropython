# The MIT License (MIT)
#
# Copyright (c) 2017 Dean Miller for Adafruit Industries
# Copyright (c) 2020 Christian Becker
# Copyright (c) 2024 Alex Whittemore
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
"""
`ina226`
====================================================

micropython driver for the INA226 current sensor.

* Author(s): Christian Becker

"""
# taken from https://github.com/robert-hh/INA219 , modified for the INA226 devices by
# Christian Becker
# June 2020

from micropython import const
# from adafruit_bus_device.i2c_device import I2CDevice

__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/elschopi/TI_INA226_micropython.git"

# Bits
# pylint: disable=bad-whitespace
_READ = const(0x01)

# Config Register (R/W)
_REG_CONFIG = const(0x00)
_CONFIG_RESET = const(0x8000)  # Reset Bit
# Constant bits - don't change
_CONFIG_CONST_BITS = const(0x4000)
# Averaging mode
_CONFIG_AVGMODE_MASK = const(0x0e00)
_CONFIG_AVGMODE_1SAMPLES = const(0x0000)
_CONFIG_AVGMODE_4SAMPLES = const(0x0200)
_CONFIG_AVGMODE_16SAMPLES = const(0x0400)
_CONFIG_AVGMODE_64SAMPLES = const(0x0600)
_CONFIG_AVGMODE_128SAMPLES = const(0x0800)
_CONFIG_AVGMODE_256SAMPLES = const(0x0a00)
_CONFIG_AVGMODE_512SAMPLES = const(0x0c00)
_CONFIG_AVGMODE_1024SAMPLES = const(0x0e00)

# Bus voltage conversion time
_CONFIG_VBUSCT_MASK = const(0x01c0)
_CONFIG_VBUSCT_140us = const(0x0000)
_CONFIG_VBUSCT_204us = const(0x0040)
_CONFIG_VBUSCT_332us = const(0x0080)
_CONFIG_VBUSCT_588us = const(0x00c0)
_CONFIG_VBUSCT_1100us = const(0x0100)
_CONFIG_VBUSCT_21116us = const(0x0140)
_CONFIG_VBUSCT_4156us = const(0x0180)
_CONFIG_AVGMODE_8244us = const(0x01c0)

# Shunt voltage conversion time
_CONFIG_VSHUNTCT_MASK = const(0x0038)
_CONFIG_VSHUNTCT_140us = const(0x0000)
_CONFIG_VSHUNTCT_204us = const(0x0008)
_CONFIG_VSHUNTCT_332us = const(0x0010)
_CONFIG_VSHUNTCT_588us = const(0x0018)
_CONFIG_VSHUNTCT_1100us = const(0x0020)
_CONFIG_VSHUNTCT_21116us = const(0x0028)
_CONFIG_VSHUNTCT_4156us = const(0x0030)
_CONFIG_VSHUNTCT_8244us = const(0x0038)

# Operating mode
_CONFIG_MODE_MASK = const(0x0007)  # Operating Mode Mask
_CONFIG_MODE_POWERDOWN = const(0x0000)
_CONFIG_MODE_SVOLT_TRIGGERED = const(0x0001)
_CONFIG_MODE_BVOLT_TRIGGERED = const(0x0002)
_CONFIG_MODE_SANDBVOLT_TRIGGERED = const(0x0003)
_CONFIG_MODE_ADCOFF = const(0x0004)
_CONFIG_MODE_SVOLT_CONTINUOUS = const(0x0005)
_CONFIG_MODE_BVOLT_CONTINUOUS = const(0x0006)
_CONFIG_MODE_SANDBVOLT_CONTINUOUS = const(0x0007)

# SHUNT VOLTAGE REGISTER (R)
_REG_SHUNTVOLTAGE = const(0x01)
_SHUNT_V_LSB = const(0.0000025)
_BUS_V_LSB = const(0.00125)

# BUS VOLTAGE REGISTER (R)
_REG_BUSVOLTAGE = const(0x02)

# POWER REGISTER (R)
_REG_POWER = const(0x03)

# CURRENT REGISTER (R)
_REG_CURRENT = const(0x04)

# CALIBRATION REGISTER (R/W)
_REG_CALIBRATION = const(0x05)
# pylint: enable=bad-whitespace

_REG_MASK_ENABLE = const(0x06)
_REG_ALERT_LIMIT = const(0x07)
ALERT_MODE_SHUNT_OVERVOLT = const(0x8000)
ALERT_MODE_SHUNT_UNDERVOLT = const(0x4000)
ALERT_MODE_BUS_OVERVOLT = const(0x2000)
ALERT_MODE_BUS_UNDERVOLT = const(0x1000)
ALERT_MODE_OVERPOWER = const(0x800)
_ALERT_MODE_CONVERSION_READY = const(0x400)
_MASK_ENABLE_AFF_BIT = const(0x10)
_MASK_ENABLE_CONV_READY_BIT = const(0x8)
_MASK_ENABLE_MATH_OVERFLOW_BIT = const(0x4)
_MASK_ENABLE_ALERT_POLARITY_BIT = const(0x2)
_MASK_ENABLE_ALERT_LATCH_ENABLE = const(0x1)




def _to_signed(num):
    if num > 0x7FFF:
        num -= 0x10000
    return num


class INA226:
    """Driver for the INA226 current sensor"""
    def __init__(self, i2c_device, addr=0x40, shunt_resistance=0.1):
        self.i2c_device = i2c_device
        self.i2c_addr = addr
        self.shunt_resistance = shunt_resistance

        self.buf = bytearray(2)
        self._current_lsb = 0       # Multiplier in mA used to determine current from raw reading
        self._power_lsb = 0         # Multiplier in W used to determine power from raw reading

        try:
            self.set_calibration()
        except Exception as e:
            print("Exception during init:")
            print(e)


    def _write_register(self, reg, value):
        self.buf[0] = (value >> 8) & 0xFF
        self.buf[1] = value & 0xFF
        self.i2c_device.writeto_mem(self.i2c_addr, reg, self.buf)

    def _read_register(self, reg):
        self.i2c_device.readfrom_mem_into(self.i2c_addr, reg & 0xff, self.buf)
        value = (self.buf[0] << 8) | (self.buf[1])
        return value

    @property
    def shunt_voltage(self):
        """The shunt voltage (between V+ and V-) in Volts
        
        The INA226 has a fixed gain and a fixed LSB scale of 2.5uV.
        Therefore, the full-scale range is +/-0.08192 i.e. +/- 2.5uV*2^15
        """
        shunt_v = _to_signed(self._read_register(_REG_SHUNTVOLTAGE))
        return shunt_v * _SHUNT_V_LSB # Mulitply register by fixed LSB scale of 2.5uV

    @property
    def bus_voltage(self):
        """The bus voltage in Volts, not to exceed 36V

        Vbus could be GND to V-, V+, or something else depending on your wiring.
        
        The INA226 has a fixed LSB scale of 1.25mV and full-scale range of 0 to 40.96 (1.25mV*2^15), but the hardware is limited to 36V.
        """
        raw_voltage = self._read_register(_REG_BUSVOLTAGE)
        return raw_voltage * _BUS_V_LSB

    @property
    def current(self):
        """The current through the shunt resistor in milliamps, from INA226 register"""

        # Sometimes a sharp load will reset the INA219, which will
        # reset the cal register, meaning CURRENT and POWER will
        # not be available ... athis by always setting a cal
        # value even if it's an unfortunate extra step
        self._write_register(_REG_CALIBRATION, self._cal_value)

        # Now we can safely read the CURRENT register!
        raw_current = _to_signed(self._read_register(_REG_CURRENT))
        return raw_current * self._current_lsb
    
    @property
    def current_calc(self):
        """The current through the shunt resistor, calculated off-board
        
        The INA226 gives raw shunt voltage, so knowing the resistor value in Python, we can just calculate the current without any calibration step.
        """

        return self.shunt_voltage/self.shunt_resistance

    
    @property
    def power(self):
        # INA226 stores the calculated power in this register        
        raw_power = _to_signed(self._read_register(_REG_POWER))
        # Calculated power is derived by multiplying raw power value with the power LSB
        return raw_power * self._power_lsb

    def calc_calibration(self):
        """Calculate calibration values for using the current register using sensible defaults
        """
        
        # The ADC LSB for the shunt voltage measurement is fixed at 2.5uV, full scale of +/- 81.92mV,
        # so I think it maximizes resolution to set current LSB to 2.5uV/Rshunt. Since the calibration
        # register is an integer, we'll calculate that first then back-out the actual LSB scale.
        self._cal_value = int(0.00512/.0000025)
        self._current_lsb = .00512/(self._cal_value*self.shunt_resistance)
        # The power LSB is fixed as a multiple of the current LSB
        self._power_lsb = 25*self._current_lsb

    def set_calibration(self):  # pylint: disable=invalid-name
        """Configures INA226 with accurate current and power registers
        
        The current and power registers are basically for convenience and maybe processing efficiency,
        since we could just read raw shunt voltage and divide by the shunt resistor value.  
        """
        self.calc_calibration()
        self._write_register(_REG_CALIBRATION, self._cal_value)
        
        config = (_CONFIG_CONST_BITS |
                  _CONFIG_AVGMODE_512SAMPLES |
                  _CONFIG_VBUSCT_588us |
                  _CONFIG_VSHUNTCT_588us |
                  _CONFIG_MODE_SANDBVOLT_CONTINUOUS)
        
        self._write_register(_REG_CONFIG, config)
    
    def set_calibration_custom(self, calValue=512, config=0x4127):
    # Set the configuration register externally by using the hex value for the config register
    # Value can be calculated with spreadsheet
    # Calibration value needs to be calculated seperately and passed as parameter too
        self._cal_value = calValue        
        self._write_register(_REG_CALIBRATION, self._cal_value)
        self._write_register(_REG_CONFIG, config)

    def shunt_v_from_amps(self, current):
        """Get the shunt voltage for a given current, given our resistor.
        Helper for set_alert_mode() since that takes a voltage
        
        current: current in amps through the shunt.
        """
        shunt_v = current*self.shunt_resistance
        if shunt_v > 0.08192:
            raise ValueError(f"Current limit too high, must be under {.08192/self.shunt_resistance}")
        return shunt_v

    def set_alert_mode(self, alert_mode, limit_amps=None, alert_limit=None, latch=False, alert_polarity=0, conversion_ready=False):
        """Set alert mode, with options. Limitation: currently doesn't support negative values

        alert_mode:         see below. Set to None if you just want to use conversion_ready
        limit_amps:         If the mode is shunt overvolt or undervolt, calculate the voltage from this current
                            Ignores alert_limit.
        alert_limit:        (float) limit value in volts or watts
                            (int) raw value written directly to register
        latch:              if True, hold the alert pin until we manually clear it by reading this register
                            if False, the alert de-asserts as soon as the condition clears.
        alert_polarity:     0 is active-low on the alert pin, 1 is active-high (open-drain)
        conversion_ready:   assert the alert pin when a conversion is ready to read. Can be done independent 
                            of other modes. TODO: this has implications this library doesn't yet handle.

        alert_mode can be:
        ina226.ALERT_MODE_SHUNT_OVERVOLT    assert alert pin if shunt voltage register exceeds alert_limit
        ina226.ALERT_MODE_SHUNT_UNDERVOLT   ... if shunt voltage register is below alert_limit
        ina226.ALERT_MODE_BUS_OVERVOLT      ... if bus voltage register exceeds alert_limit
        ina226.ALERT_MODE_BUS_UNDERVOLT     ... if bus voltage register is below alert_limit
        ina226.ALERT_MODE_OVERPOWER         ... if power register exceeds alert_limit
        """
        
        if alert_mode and alert_limit is None and limit_amps is None:
            raise ValueError("alert_limit must be set for this alert mode")
        
        # Validate alert_mode makes sense, and set alert_limit to an int if it isn't already.
        if alert_mode == ALERT_MODE_SHUNT_OVERVOLT or alert_mode == ALERT_MODE_SHUNT_UNDERVOLT:
            if limit_amps:
                alert_limit = self.shunt_v_from_amps(limit_amps)
            if type(alert_limit) is float:
                alert_limit = int(alert_limit/_SHUNT_V_LSB)
        elif alert_mode == ALERT_MODE_BUS_OVERVOLT or alert_mode == ALERT_MODE_BUS_UNDERVOLT:
            if type(alert_limit) is float:
                alert_limit = int(alert_limit/_BUS_V_LSB)
        elif alert_mode == ALERT_MODE_OVERPOWER:
            if type(alert_limit) is float:
                alert_limit = int(alert_limit/self._power_lsb)
        elif not conversion_ready:
            raise ValueError("Unknown alert mode")

        if type(alert_limit) is int and (alert_limit > 32768 or alert_limit < 0):
            raise ValueError(f"alert_limit is out of bounds. Max shunt_v 81.92mV, bus_v 36v, power {32768*self._power_lsb}w")
        
        alert_mode_reg = 0x0
        if alert_mode:
            alert_mode_reg |= alert_mode
        if conversion_ready:
            alert_mode_reg |= _ALERT_MODE_CONVERSION_READY
        if alert_polarity:
            alert_mode_reg |= _MASK_ENABLE_ALERT_POLARITY_BIT
        if latch:
            alert_mode_reg |= _MASK_ENABLE_ALERT_LATCH_ENABLE

        self._write_register(_REG_MASK_ENABLE, alert_mode_reg)
        self._write_register(_REG_ALERT_LIMIT, alert_limit)

    def check_clear_alert(self):
        """Returns whether there was an alert to clear. There's no way to read without clearing."""
        return bool(self._read_register(_REG_MASK_ENABLE) & _MASK_ENABLE_AFF_BIT)
