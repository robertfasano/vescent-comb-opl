#!/usr/bin/env python3
"""
rubricomb.py -- driver for the Vescent RUBRIComb frequency comb
(Serial API Guide, Revision 01).

Covers the ASCII serial API exposed on the rear-panel USB interface:
  * System info          (*IDN?, #VERSION, #SAVESETTINGS, *RST)
  * System-level control (MSTRCTL, PZT_ENABLE, PZT_SLSRVEN, CURR_SLSRVEN)
  * Cavity               (temperature control, slow servo, HV amplifier)
  * Oscillator laser     (temperature, current, interlock, errors)
  * Amplifier laser      (temperature, current, interlock, errors)

Transport, parsing, and Influx plumbing live in vescent_serial.py.
Run main.py to poll this instrument alongside the SLICE-OPL.

Every setter below sends as soon as it's called. What the HTTP API will
write on a caller's behalf is controlled per field, via MONITOR_PARAMS
(Param.readonly / Param.setter).
"""

from __future__ import annotations

import logging
import time
from enum import IntEnum
from typing import Any, Dict, List, Optional, Tuple

from vescent_serial import (
    Param,
    VescentError,
    VescentSerialDevice,
    parse_bool,
    parse_float,
    parse_int,
)

log = logging.getLogger("vescent.rubricomb")

#: Kept for continuity with earlier code that caught RubriCombError.
RubriCombError = VescentError


# ---------------------------------------------------------------------------
# Enumerations / error-code decoding
# ---------------------------------------------------------------------------

class MasterMode(IntEnum):
    """MSTRCTL operating mode."""
    OFF = 0
    STANDBY = 1
    LASER_ON = 2


#: Error codes are bit positions in a uint16 whose top two bits are always set.
ERROR_NONE = 0xC000
_ERROR_MASK = 0x3FFF

_ERROR_BITS: Dict[int, str] = {
    0x0002: "TEMP_HARD_LIM_EXCEEDED",
    0x0020: "OVER_TEMP_HARDWARE",
    0x0080: "INTERLOCK_OPEN",
    0x0100: "POWER_LIMIT_EXCEEDED",
    0x0400: "LASER_TEMP_BOUNDS_EXCEEDED",
    0x0800: "SUPPLY_15V_ERROR",  # cavity channel only
}


def decode_error(code: int) -> List[str]:
    """Decode a raw *ERROR? code into a list of human-readable flag names."""
    bits = int(code) & _ERROR_MASK
    if bits == 0:
        return ["NONE"]
    names = [name for mask, name in _ERROR_BITS.items() if bits & mask]
    leftover = bits & ~sum(m for m in _ERROR_BITS if bits & m)
    if leftover:
        names.append(f"UNKNOWN_0x{leftover:04X}")
    return names


# ---------------------------------------------------------------------------
# Parameter table -- every readable field, not just the ones actively logged.
# poll=False keeps an entry out of the periodic monitoring sweep while
# leaving it in this table, documented and queryable on demand.
#
# readonly=True (the default) blocks a field from ever being written through
# anything that consults the flag. cav_temp is deliberately the only field
# with readonly=False, to test write support against before opening up more.
# ---------------------------------------------------------------------------

MONITOR_PARAMS: Tuple[Param, ...] = (
    # --- system level -------------------------------------------------------
    Param("master_mode",        "MSTRCTL?",       parse_int,   "system", "enum", poll=False),
    Param("pzt_hv_enabled",     "PZT_ENABLE?",    parse_bool,  "system", "bool"),
    Param("pzt_slow_servo_on",  "PZT_SLSRVEN?",   parse_bool,  "system", "bool"),
    Param("curr_slow_servo_on", "CURR_SLSRVEN?",  parse_bool,  "system", "bool"),

    # --- cavity -------------------------------------------------------------
    Param("cav_error_code",     "CAVERROR? 1",    parse_int,   "cavity", "code",  poll=False),
    Param("cav_temp_setpoint",  "CAVTEMPSET? 0",  parse_float, "cavity", "degC",  poll=False),
    # cav_temp is a measured reading (CAVTEMP?), not a settable register --
    # there is no "set current temperature" command. Writing here instead
    # calls set_cavity_temperature_setpoint(), the actual settable quantity
    # that drives cav_temp toward a new value via the cavity's own PID loop.
    # cav_temp's readback afterwards reflects the (slow) physical response,
    # not the setpoint itself -- that's expected, not a bug in the write path.
    Param("cav_temp",           "CAVTEMP? 0",     parse_float, "cavity", "degC",  readonly=False,
          setter=lambda dev, v: dev.set_cavity_temperature_setpoint(v, channel=0)),
    Param("cav_temp_error",     "CAVTERROR? 0",   parse_float, "cavity", "mK",    poll=False),
    Param("cav_tec_current",    "CAVCURRENT? 0",  parse_float, "cavity", "A",     poll=False),
    Param("cav_slow_servo_gain","CAVSLSRVGN? 0",  parse_float, "cavity", "dB",    poll=False),
    Param("cav_slow_servo_offs","CAVSLSRVOS? 0",  parse_float, "cavity", "V",     poll=False),
    Param("cav_dc_bias_voltage","CAVDCBIASV? 1",  parse_float, "cavity", "V",     poll=False),
    Param("cav_output_voltage", "CAVOUTVOLT? 1",  parse_float, "cavity", "V",     poll=False),

    # --- oscillator laser ---------------------------------------------------
    Param("osc_error_code",     "OSCERROR? 1",    parse_int,   "oscillator", "code", poll=False),
    Param("osc_temp",           "OSCTEMP? 0",     parse_float, "oscillator", "degC", poll=False),
    Param("osc_temp_error",     "OSCTERROR? 0",   parse_float, "oscillator", "degC", poll=False),
    Param("osc_tec_current",    "OSCTCURR? 0",    parse_float, "oscillator", "A",    poll=False),
    Param("osc_mod_current",    "OSCMODCURR?",    parse_float, "oscillator", "mA",   poll=False),
    Param("osc_current_setpt",  "OSCCCURSET? 1",  parse_float, "oscillator", "A",    poll=False),
    Param("osc_output_current", "OSCCCURR? 1",    parse_float, "oscillator", "mA"),
    Param("osc_compliance_volt","OSCCVOLTCC? 1",  parse_float, "oscillator", "V",    poll=False),
    Param("osc_interlock_ok",   "OSCINTERLK?",    parse_bool,  "oscillator", "bool", poll=False),

    # --- amplifier laser ----------------------------------------------------
    Param("amp_error_code",     "AMPERROR? 1",    parse_int,   "amplifier", "code", poll=False),
    Param("amp_temp",           "AMPTEMP? 0",     parse_float, "amplifier", "degC", poll=False),
    Param("amp_temp_error",     "AMPTERROR? 0",   parse_float, "amplifier", "degC", poll=False),
    Param("amp_tec_current",    "AMPTCURR? 0",    parse_float, "amplifier", "A",    poll=False),
    Param("amp_current_setpt",  "AMPCCURSET? 1",  parse_float, "amplifier", "A",    poll=False),
    Param("amp_output_current", "AMPCCURR? 1",    parse_float, "amplifier", "mA",   poll=False),
    Param("amp_compliance_volt","AMPCVOLTCC? 1",  parse_float, "amplifier", "V",    poll=False),
    Param("amp_interlock_ok",   "AMPINTERLK?",    parse_bool,  "amplifier", "bool", poll=False),
)


# ---------------------------------------------------------------------------
# Driver
# ---------------------------------------------------------------------------

class RubriComb(VescentSerialDevice):
    """Serial driver for the Vescent RUBRIComb frequency comb.

    Example
    -------
        with RubriComb("/dev/ttyUSB0") as comb:
            print(comb.idn())
            print(comb.cavity_temperature())
            comb.set_cavity_temperature_setpoint(24.0)
    """

    MONITOR_PARAMS = MONITOR_PARAMS

    # -- system info ---------------------------------------------------------

    def version(self, slot: Optional[int] = None) -> str:
        """Firmware version. slot: None=system controller, 1=osc, 2=amp, 3=cavity."""
        return self.query("#VERSION" if slot is None else f"#VERSION {slot}")

    def save_settings(self) -> str:
        """Persist current settings to EEPROM. CHANGES DEVICE STATE."""
        return self._transact("#SAVESETTINGS", max_lines=2, allow_echo_reply=True)

    # -- system-level control ------------------------------------------------

    def master_mode(self) -> MasterMode:
        return MasterMode(self.query("MSTRCTL?", parse_int))

    def set_master_mode(self, mode: "int | MasterMode") -> MasterMode:
        """Set operating mode. LASER_ON is only reachable from STANDBY once
        temperatures have stabilized. CHANGES DEVICE STATE."""
        self.write(f"MSTRCTL {int(mode)}")
        return self.master_mode()

    def pzt_hv_enabled(self) -> bool:
        return self.query("PZT_ENABLE?", parse_bool)

    def set_pzt_hv_enabled(self, on: bool) -> bool:
        """CHANGES DEVICE STATE."""
        self.write(f"PZT_ENABLE {int(bool(on))}")
        return self.pzt_hv_enabled()

    def pzt_slow_servo(self) -> bool:
        return self.query("PZT_SLSRVEN?", parse_bool)

    def set_pzt_slow_servo(self, on: bool) -> bool:
        """Enable/disable the PZT slow loop. Silently does nothing if the
        engage conditions are not met, so the state is read back.
        CHANGES DEVICE STATE."""
        self.write(f"PZT_SLSRVEN {int(bool(on))}")
        return self.pzt_slow_servo()

    def current_slow_servo(self) -> bool:
        return self.query("CURR_SLSRVEN?", parse_bool)

    def set_current_slow_servo(self, on: bool) -> bool:
        """Enable/disable the current slow loop; state is read back because the
        command is a no-op when the engage conditions are unmet.
        CHANGES DEVICE STATE."""
        self.write(f"CURR_SLSRVEN {int(bool(on))}")
        return self.current_slow_servo()

    # -- cavity --------------------------------------------------------------

    def cavity_error(self, channel: int = 1) -> int:
        return self.query(f"CAVERROR? {channel}", parse_int)

    def clear_cavity_error(self, code: int, channel: int = 1) -> int:
        """CHANGES DEVICE STATE."""
        return parse_int(self.write(f"CAVERROR {channel} {int(code)}"))

    def cavity_temperature(self, channel: int = 0) -> float:
        """Measured cavity temperature [degC]."""
        return self.query(f"CAVTEMP? {channel}", parse_float)

    def cavity_temperature_setpoint(self, channel: int = 0) -> float:
        return self.query(f"CAVTEMPSET? {channel}", parse_float)

    def set_cavity_temperature_setpoint(self, temperature_c: float,
                                        channel: int = 0) -> float:
        """CHANGES DEVICE STATE."""
        return parse_float(self.write(f"CAVTEMPSET {channel} {temperature_c}"))

    def cavity_temperature_error(self, channel: int = 0) -> float:
        """Cavity temperature error (Tset - Tactual) [mK]."""
        return self.query(f"CAVTERROR? {channel}", parse_float)

    def cavity_tec_current(self, channel: int = 0) -> float:
        """Cavity TEC / heater current [A]."""
        return self.query(f"CAVCURRENT? {channel}", parse_float)

    def cavity_slow_servo_gain(self, channel: int = 0) -> float:
        return self.query(f"CAVSLSRVGN? {channel}", parse_float)

    def set_cavity_slow_servo_gain(self, gain_db: float, channel: int = 0) -> float:
        """Slow-servo gain [dB], limits -100 to +100. CHANGES DEVICE STATE."""
        if not -100.0 <= gain_db <= 100.0:
            raise ValueError("slow servo gain must be within [-100, 100] dB")
        return parse_float(self.write(f"CAVSLSRVGN {channel} {gain_db}"))

    def cavity_slow_servo_offset(self, channel: int = 0) -> float:
        return self.query(f"CAVSLSRVOS? {channel}", parse_float)

    def set_cavity_slow_servo_offset(self, volts: float, channel: int = 0) -> float:
        """Slow-servo offset voltage [V], limits 0 to 60. CHANGES DEVICE STATE."""
        if not 0.0 <= volts <= 60.0:
            raise ValueError("slow servo offset must be within [0, 60] V")
        return parse_float(self.write(f"CAVSLSRVOS {channel} {volts}"))

    def cavity_dc_bias(self, channel: int = 1) -> float:
        """Cavity HV DC bias voltage [V]."""
        return self.query(f"CAVDCBIASV? {channel}", parse_float)

    def set_cavity_dc_bias(self, volts: float, channel: int = 1) -> float:
        """Set cavity HV DC bias [V], limits 0 to 60. CHANGES DEVICE STATE."""
        if not 0.0 <= volts <= 60.0:
            raise ValueError("DC bias must be within [0, 60] V")
        return parse_float(self.write(f"CAVDCBIASV {channel} {volts}"))

    def cavity_output_voltage(self, channel: int = 1) -> float:
        """Cavity HV output voltage [V]."""
        return self.query(f"CAVOUTVOLT? {channel}", parse_float)

    # -- oscillator laser ----------------------------------------------------

    def oscillator_error(self, channel: int = 1) -> int:
        return self.query(f"OSCERROR? {channel}", parse_int)

    def clear_oscillator_error(self, code: int, channel: int = 1) -> int:
        """CHANGES DEVICE STATE."""
        return parse_int(self.write(f"OSCERROR {channel} {int(code)}"))

    def oscillator_temperature(self, channel: int = 0) -> float:
        return self.query(f"OSCTEMP? {channel}", parse_float)

    def oscillator_temperature_error(self, channel: int = 0) -> float:
        """Oscillator temperature error (Tset - Tactual) [degC]."""
        return self.query(f"OSCTERROR? {channel}", parse_float)

    def oscillator_tec_current(self, channel: int = 0) -> float:
        return self.query(f"OSCTCURR? {channel}", parse_float)

    def oscillator_modulation_current(self) -> float:
        """Oscillator modulation current [mA]."""
        return self.query("OSCMODCURR?", parse_float)

    def oscillator_current_setpoint(self, channel: int = 1) -> float:
        return self.query(f"OSCCCURSET? {channel}", parse_float)

    def set_oscillator_current_setpoint(self, amps: float, channel: int = 1) -> float:
        """CHANGES DEVICE STATE."""
        return parse_float(self.write(f"OSCCCURSET {channel} {amps}"))

    def oscillator_current(self, channel: int = 1) -> float:
        """Oscillator laser output current [mA]."""
        return self.query(f"OSCCCURR? {channel}", parse_float)

    def oscillator_compliance_voltage(self, channel: int = 1) -> float:
        return self.query(f"OSCCVOLTCC? {channel}", parse_float)

    def oscillator_interlock(self) -> bool:
        """True when the oscillator interlock circuit is closed."""
        return self.query("OSCINTERLK?", parse_bool)

    # -- amplifier laser -----------------------------------------------------

    def amplifier_error(self, channel: int = 1) -> int:
        return self.query(f"AMPERROR? {channel}", parse_int)

    def clear_amplifier_error(self, code: int, channel: int = 1) -> int:
        """CHANGES DEVICE STATE."""
        return parse_int(self.write(f"AMPERROR {channel} {int(code)}"))

    def amplifier_temperature(self, channel: int = 0) -> float:
        return self.query(f"AMPTEMP? {channel}", parse_float)

    def amplifier_temperature_error(self, channel: int = 0) -> float:
        return self.query(f"AMPTERROR? {channel}", parse_float)

    def amplifier_tec_current(self, channel: int = 0) -> float:
        return self.query(f"AMPTCURR? {channel}", parse_float)

    def amplifier_current_setpoint(self, channel: int = 1) -> float:
        return self.query(f"AMPCCURSET? {channel}", parse_float)

    def set_amplifier_current_setpoint(self, amps: float, channel: int = 1) -> float:
        """CHANGES DEVICE STATE."""
        return parse_float(self.write(f"AMPCCURSET {channel} {amps}"))

    def amplifier_current(self, channel: int = 1) -> float:
        """Amplifier laser output current [mA]."""
        return self.query(f"AMPCCURR? {channel}", parse_float)

    def amplifier_compliance_voltage(self, channel: int = 1) -> float:
        return self.query(f"AMPCVOLTCC? {channel}", parse_float)

    def amplifier_interlock(self) -> bool:
        return self.query("AMPINTERLK?", parse_bool)

    # -- convenience ---------------------------------------------------------

    def errors(self) -> Dict[str, List[str]]:
        """Decoded error flags for all three subsystems."""
        return {
            "cavity": decode_error(self.cavity_error()),
            "oscillator": decode_error(self.oscillator_error()),
            "amplifier": decode_error(self.amplifier_error()),
        }

    def derived_fields(self, values: Dict[str, Any]) -> Dict[str, Any]:
        """Turn the raw error codes into per-subsystem boolean health flags."""
        derived: Dict[str, Any] = {}
        for subsystem, key in (
            ("cavity", "cav_error_code"),
            ("oscillator", "osc_error_code"),
            ("amplifier", "amp_error_code"),
        ):
            if key in values:
                flags = decode_error(values[key])
                derived[f"{subsystem}_ok"] = flags == ["NONE"]
                if flags != ["NONE"]:
                    log.warning("%s error flags: %s", subsystem, ", ".join(flags))
        return derived

    # -- sequences -----------------------------------------------------------

    def wait_for_temperature_lock(
        self,
        tolerance_mk: float = 20.0,
        timeout: float = 600.0,
        settle_time: float = 30.0,
        poll_interval: float = 2.0,
    ) -> bool:
        """Block until cavity/oscillator/amplifier temperature errors stay
        within *tolerance_mk* continuously for *settle_time* seconds.

        Read-only; safe on a read-only connection.

        Note the unit mismatch in the API: CAVTERROR? reports mK while
        OSCTERROR?/AMPTERROR? report degC. Both are converted to mK here.
        """
        deadline = time.monotonic() + timeout
        stable_since: Optional[float] = None
        while time.monotonic() < deadline:
            try:
                errs_mk = (
                    abs(self.cavity_temperature_error()),            # already mK
                    abs(self.oscillator_temperature_error()) * 1e3,  # degC -> mK
                    abs(self.amplifier_temperature_error()) * 1e3,
                )
            except VescentError as exc:
                log.warning("Temperature poll failed: %s", exc)
                stable_since = None
                time.sleep(poll_interval)
                continue
            if max(errs_mk) <= tolerance_mk:
                stable_since = stable_since or time.monotonic()
                if time.monotonic() - stable_since >= settle_time:
                    log.info("Temperatures stable (max |dT| = %.1f mK)", max(errs_mk))
                    return True
            else:
                stable_since = None
            time.sleep(poll_interval)
        log.warning("Temperatures did not stabilize within %.0f s", timeout)
        return False

    def startup(self, tolerance_mk: float = 20.0, timeout: float = 600.0) -> MasterMode:
        """Bring the comb up: OFF -> STANDBY -> (temperatures stable) -> LASER ON.

        CHANGES DEVICE STATE. The API guide is explicit that these
        system-level commands should be preferred, since they sequence the
        TECs ahead of the current supplies.
        """
        mode = self.master_mode()
        if mode == MasterMode.LASER_ON:
            return mode
        if mode == MasterMode.OFF:
            log.info("Entering STANDBY")
            self.set_master_mode(MasterMode.STANDBY)
        if not self.wait_for_temperature_lock(tolerance_mk=tolerance_mk, timeout=timeout):
            raise VescentError("temperatures did not stabilize; not enabling lasers")
        log.info("Enabling lasers")
        mode = self.set_master_mode(MasterMode.LASER_ON)
        if mode != MasterMode.LASER_ON:
            raise VescentError(f"transition to LASER ON failed (mode = {mode.name})")
        return mode

    def shutdown(self) -> MasterMode:
        """Return the comb to OFF. CHANGES DEVICE STATE."""
        return self.set_master_mode(MasterMode.OFF)
