#!/usr/bin/env python3
"""
slice_opl.py -- driver for the Vescent SLICE-OPL offset phase lock servo
(Serial API Guide, Revision 01).

Covers the ASCII serial API exposed on the rear-panel USB interface:
  * Global SLICE       (*IDN?, *RST, #SCBKLT, #SCVOL)
  * General operation  (SERVO, BNTGT, READVOLT)
  * PLL servo filter   (gain, integrator, differentiator, bias, limits, lock range)
  * AUX servo filter    (enable, polarity, target, gain, bias, limits)
  * Sweep              (RAMP, RAMPCH, RAMPNUM, RAMPFRQ, RAMPSWP, RAMPADC)
  * Beat note input    (BNMAX, N1DIV, N2DIV, READBN)
  * Reference source   (READREF, EREFBWL, PFDDDS, DDSINT, DDSREFF, M1DIV, M2DIV)
  * External I/O       (MUXO, FP1EN, FP2EN, FPMUXO, FPAINEN, FPBINEN, TRIGI, TRIGO)
  * Calibration        (offset trims, self-cal status)
  * Advanced           (HOLD, DDSFREQ, DDSRAW)

Transport, parsing, and Influx plumbing live in vescent_serial.py.
Run main.py to poll this instrument alongside the RUBRIComb.

Connections are READ-ONLY unless constructed with read_only=False; every
setter below is blocked on a read-only connection.

Two API quirks worth knowing, both handled here:
  * READVOLT is a *read* command that does not end in '?', so it is declared
    read-safe explicitly.
  * NOCP, DDSAUTO, _SELFCAL and _FACTORY *change state* and also do not end in
    '?', so they are never treated as reads.
"""

from __future__ import annotations

import logging
from enum import IntEnum
from typing import Any, Dict, Optional, Sequence, Tuple

from vescent_serial import (
    Param,
    VescentError,
    VescentSerialDevice,
    parse_bool,
    parse_float,
    parse_int,
    parse_str,
)

log = logging.getLogger("vescent.slice_opl")

SliceOPLError = VescentError


# ---------------------------------------------------------------------------
# Enumerations
# ---------------------------------------------------------------------------

class ADCChannel(IntEnum):
    """READVOLT channel selector."""
    ERROR_SIGNAL = 1
    ERROR_SIGNAL_8X = 2
    INTEGRATOR_MONITOR = 3
    PLL_OUTPUT = 4
    AUX_OUTPUT = 5
    VGA_INPUT = 6          # error out
    VGA_OUTPUT_MONITOR = 7
    GROUND = 8


class RampChannel(IntEnum):
    """RAMPCH -- which servo the sweep is applied to."""
    MAIN_SERVO = 0
    AUX_SERVO = 1


class RampADCGain(IntEnum):
    """RAMPADC -- phase error graph gain mode."""
    NORMAL_1X = 128
    HIGH_8X = 64


class MonitorMux(IntEnum):
    """MUXO -- back panel monitor mux."""
    DISABLED = 0
    ERROR_MONITOR = 1
    DIVIDED_BN_MONITOR = 2
    DDS_REF_MONITOR = 3
    DDS_OUT_MONITOR = 4


class FrontPanelMux(IntEnum):
    """FPMUXO -- front panel Output 2 mux."""
    DISABLED = 0
    PLL_OUTPUT = 1
    AUX_OUTPUT = 2
    ERROR_MONITOR = 3
    GROUND = 4


class TriggerInMode(IntEnum):
    """TRIGI -- trigger input behaviour."""
    NONE = 0
    TOGGLE_SERVO = 1
    TOGGLE_SAMPLE_HOLD = 2  # servo must be on


class TriggerOutMode(IntEnum):
    """TRIGO -- trigger output behaviour."""
    NONE = 0
    RAMP_TRIGGER = 1


#: Discrete values the firmware accepts; anything else is rounded to nearest.
INTEGRATOR_CORNERS_HZ: Tuple[float, ...] = (
    0, 50, 100, 200, 500, 1e3, 2e3, 5e3, 10e3, 20e3, 50e3, 100e3, 200e3,
    500e3, 1e6, 2e6, 5e6,
)
DIFFERENTIATOR_CORNERS_HZ: Tuple[float, ...] = (
    0, 150, 300, 600, 1500, 3000, 6000, 15e3, 30e3, 60e3, 150e3, 300e3,
    600e3, 1.5e6, 3e6,
)
BEAT_NOTE_PATHS_MHZ: Tuple[float, ...] = (1000.0, 3000.0, 12000.0)
EXT_REF_BW_LIMITS_MHZ: Tuple[float, ...] = (12.0, 40.0, 125.0, 300.0)
DDS_REF_FREQS_MHZ: Tuple[float, ...] = (7.5, 8.0, 10.0, 12.0, 12.5)
M1_DIVIDERS: Tuple[int, ...] = (1, 2, 4, 8, 16)
N1_DIVIDERS_LOW_MID: Tuple[int, ...] = (1, 2, 4, 8, 16)
N1_DIVIDERS_HIGH: Tuple[int, ...] = (4, 8, 16, 32, 64)


def _nearest(value: float, choices: Sequence[float]) -> float:
    return min(choices, key=lambda c: abs(c - value))


# ---------------------------------------------------------------------------
# Parameter table -- everything the monitoring loop polls.
# Comment out any line you do not want logged; the rest stays here for
# reference. All entries are reads, so the table is safe to sweep against a
# read-only connection.
# ---------------------------------------------------------------------------

MONITOR_PARAMS: Tuple[Param, ...] = (
    # --- general operation --------------------------------------------------
    # Param("servo_on",            "SERVO?",     parse_bool,  "general", "bool"),
    # Param("beat_note_target",    "BNTGT?",     parse_float, "general", "MHz"),

    # --- ADC monitors (READVOLT; the fast-moving diagnostics) ---------------
    # Param("adc_error_signal",    "READVOLT 1", parse_float, "adc", "V"),
    # Param("adc_error_signal_8x", "READVOLT 2", parse_float, "adc", "V"),
    # Param("adc_integrator_mon",  "READVOLT 3", parse_float, "adc", "V"),
    # Param("adc_pll_output",      "READVOLT 4", parse_float, "adc", "V"),
    # Param("adc_aux_output",      "READVOLT 5", parse_float, "adc", "V"),
    # Param("adc_vga_input",       "READVOLT 6", parse_float, "adc", "V"),
    # Param("adc_vga_output_mon",  "READVOLT 7", parse_float, "adc", "V"),
    # Param("adc_ground",          "READVOLT 8", parse_float, "adc", "V"),

    # --- PLL servo filter ---------------------------------------------------
    # Param("pll_inverted",        "PLLINVT?",   parse_bool,  "pll", "bool"),
    # Param("pll_gain",            "PLLGAIN?",   parse_float, "pll", "dB"),
    # Param("pll_int_corner",      "INT?",       parse_float, "pll", "Hz"),
    # Param("pll_diff_corner",     "DIFF?",      parse_float, "pll", "Hz"),
    # Param("pll_gain_clamped",    "GAINLIM?",   parse_bool,  "pll", "bool"),
    # Param("pll_bias",            "PLLBIAS?",   parse_float, "pll", "V"),
    # Param("pll_out_max",         "PLLMAX?",    parse_float, "pll", "V"),
    # Param("pll_out_min",         "PLLMIN?",    parse_float, "pll", "V"),
    # Param("lock_range",          "LOCKRNG?",   parse_float, "pll", "V"),

    # --- AUX servo filter ---------------------------------------------------
    # Param("aux_enabled",         "AUXEN?",     parse_bool,  "aux", "bool"),
    # Param("aux_inverted",        "AUXINVT?",   parse_bool,  "aux", "bool"),
    # Param("aux_target",          "AUXTGT?",    parse_float, "aux", "V"),
    # Param("aux_gain",            "AUXGAIN?",   parse_float, "aux", "Hz"),
    # Param("aux_bias",            "AUXBIAS?",   parse_float, "aux", "V"),
    # Param("aux_out_max",         "AUXMAX?",    parse_float, "aux", "V"),
    # Param("aux_out_min",         "AUXMIN?",    parse_float, "aux", "V"),

    # --- sweep --------------------------------------------------------------
    # Param("ramp_on",             "RAMP?",      parse_bool,  "sweep", "bool"),
    # Param("ramp_channel",        "RAMPCH?",    parse_int,   "sweep", "enum"),
    # Param("ramp_points",         "RAMPNUM?",   parse_int,   "sweep", "count"),
    # Param("ramp_frequency",      "RAMPFRQ?",   parse_float, "sweep", "Hz"),
    # Param("ramp_span",           "RAMPSWP?",   parse_float, "sweep", "V"),
    # Param("ramp_adc_gain_mode",  "RAMPADC?",   parse_int,   "sweep", "enum"),

    # --- beat note input ----------------------------------------------------
    # Param("beat_note_path_max",  "BNMAX?",     parse_float, "beatnote", "MHz"),
    # Param("n1_divider",          "N1DIV?",     parse_int,   "beatnote", "ratio"),
    # Param("n2_divider",          "N2DIV?",     parse_int,   "beatnote", "ratio"),
    Param("beat_note_divided",   "READBN?",    parse_float, "beatnote", "MHz"),

    # --- reference source ---------------------------------------------------
    # Param("ref_frequency",       "READREF?",   parse_float, "reference", "MHz"),
    # Param("ext_ref_bw_limit",    "EREFBWL?",   parse_float, "reference", "MHz"),
    # Param("dds_is_pfd_ref",      "PFDDDS?",    parse_bool,  "reference", "bool"),
    # Param("dds_internal_ref",    "DDSINT?",    parse_bool,  "reference", "bool"),
    # Param("dds_ref_frequency",   "DDSREFF?",   parse_float, "reference", "MHz"),
    # Param("m1_divider",          "M1DIV?",     parse_int,   "reference", "ratio"),
    # Param("m2_divider",          "M2DIV?",     parse_int,   "reference", "ratio"),

    # --- external I/O -------------------------------------------------------
    # Param("monitor_mux",         "MUXO?",      parse_int,   "io", "enum"),
    # Param("fp_out1_enabled",     "FP1EN?",     parse_bool,  "io", "bool"),
    # Param("fp_out2_enabled",     "FP2EN?",     parse_bool,  "io", "bool"),
    # Param("fp_out2_mux",         "FPMUXO?",    parse_int,   "io", "enum"),
    # Param("fp_mod_in_a_enabled", "FPAINEN?",   parse_bool,  "io", "bool"),
    # Param("fp_mod_in_b_enabled", "FPBINEN?",   parse_bool,  "io", "bool"),
    # Param("trigger_in_mode",     "TRIGI?",     parse_int,   "io", "enum"),
    # Param("trigger_out_mode",    "TRIGO?",     parse_int,   "io", "enum"),

    # --- advanced -----------------------------------------------------------
    # Param("integrator_hold",     "HOLD?",      parse_bool,  "advanced", "bool"),
    # Param("dds_frequency",       "DDSFREQ?",   parse_float, "advanced", "MHz"),
    # Param("dds_raw",             "DDSRAW?",    parse_int,   "advanced", "counts"),

    # --- calibration (slow-moving; comment out for routine logging) ---------
    # Param("cal_status",          "_CAL?",      parse_str,   "calibration", "status"),
    # Param("cal_opamp_offset",    "_OPOFST?",   parse_int,   "calibration", "DAC"),
    # Param("cal_vga_offset",      "_VGAOFT?",   parse_int,   "calibration", "DAC"),
    # Param("cal_vga_gain_offset", "_VGAGOF?",   parse_int,   "calibration", "DAC"),
    # Param("cal_servo_offset",    "_CALSV?",    parse_int,   "calibration", "DAC"),
    # Param("cal_aux_offset",      "_CALAX?",    parse_int,   "calibration", "DAC"),

    # --- UI settings (rarely useful to log) ---------------------------------
    # Param("screen_backlight",  "#SCBKLT?",   parse_int,   "ui", "level"),
    # Param("screen_volume",     "#SCVOL?",    parse_int,   "ui", "level"),
)


# ---------------------------------------------------------------------------
# Driver
# ---------------------------------------------------------------------------

class SliceOPL(VescentSerialDevice):
    """Serial driver for the Vescent SLICE-OPL offset phase lock servo.

    Example
    -------
        with SliceOPL("/dev/ttyUSB1") as opl:          # read-only by default
            print(opl.idn())
            print(opl.error_signal(), opl.lock_range())

        with SliceOPL("/dev/ttyUSB1", read_only=False) as opl:
            opl.set_pll_gain(-10.0)
    """

    MONITOR_PARAMS = MONITOR_PARAMS

    #: READVOLT reads an ADC channel but carries no '?'. Everything else
    #: without a '?' on this instrument changes state.
    READ_SAFE_COMMANDS = frozenset({"READVOLT"})

    #: READBN? returns the *divided* beat note. Whether the reading sits after
    #: N1 alone or after N1*N2 is not stated in Rev 01 of the API guide, so no
    #: reconstructed carrier frequency is logged by default. Set this True once
    #: you have confirmed the convention against a counter.
    ESTIMATE_BEAT_NOTE = False

    # -- global --------------------------------------------------------------

    def screen_backlight(self) -> int:
        return self.query("#SCBKLT?", parse_int)

    def set_screen_backlight(self, level: int) -> str:
        """CHANGES DEVICE STATE."""
        return self.write(f"#SCBKLT {int(level)}")

    def screen_volume(self) -> int:
        return self.query("#SCVOL?", parse_int)

    def set_screen_volume(self, level: int) -> str:
        """CHANGES DEVICE STATE."""
        return self.write(f"#SCVOL {int(level)}")

    # -- general operation ---------------------------------------------------

    def servo_enabled(self) -> bool:
        """True when the PLL servo is engaged."""
        return self.query("SERVO?", parse_bool)

    def set_servo_enabled(self, on: bool) -> bool:
        """Engage/disengage the servo. CHANGES DEVICE STATE."""
        self.write(f"SERVO {int(bool(on))}")
        return self.servo_enabled()

    def beat_note_target(self) -> float:
        """Target beat note frequency [MHz]. Only valid with DDS as REF source."""
        return self.query("BNTGT?", parse_float)

    def set_beat_note_target(self, freq_mhz: float) -> str:
        """Set target beat note [MHz], 10 to 20000.

        CHANGES DEVICE STATE, and not only this setting: per the user manual
        this recomputes DDS frequency and the divider chain.
        """
        if not 10.0 <= freq_mhz <= 20000.0:
            raise ValueError("target beat note must be within [10, 20000] MHz")
        return self.write(f"BNTGT {freq_mhz}")

    def read_voltage(self, channel: "int | ADCChannel") -> float:
        """Read an ADC monitor channel [V]. Read-only despite the missing '?'."""
        return self.query(f"READVOLT {int(channel)}", parse_float)

    def error_signal(self) -> float:
        return self.read_voltage(ADCChannel.ERROR_SIGNAL)

    def pll_output_voltage(self) -> float:
        return self.read_voltage(ADCChannel.PLL_OUTPUT)

    def aux_output_voltage(self) -> float:
        return self.read_voltage(ADCChannel.AUX_OUTPUT)

    # -- PLL servo filter ----------------------------------------------------

    def pll_inverted(self) -> bool:
        return self.query("PLLINVT?", parse_bool)

    def set_pll_inverted(self, inverted: bool) -> bool:
        """Flip the error signal sign. CHANGES DEVICE STATE."""
        self.write(f"PLLINVT {int(bool(inverted))}")
        return self.pll_inverted()

    def pll_gain(self) -> float:
        """PLL servo gain [dB]."""
        return self.query("PLLGAIN?", parse_float)

    def set_pll_gain(self, gain_db: float) -> str:
        """CHANGES DEVICE STATE. Accepted range depends on other settings."""
        return self.write(f"PLLGAIN {gain_db}")

    def integrator_corner(self) -> float:
        """Integrator pole corner frequency [Hz]; 0 is off."""
        return self.query("INT?", parse_float)

    def set_integrator_corner(self, freq_hz: float) -> str:
        """CHANGES DEVICE STATE. Snapped to the nearest supported corner."""
        snapped = _nearest(freq_hz, INTEGRATOR_CORNERS_HZ)
        if snapped != freq_hz:
            log.info("Integrator corner %.1f Hz snapped to %.1f Hz", freq_hz, snapped)
        return self.write(f"INT {snapped}")

    def differentiator_corner(self) -> float:
        """Differentiator pole [Hz]; 0 is off."""
        return self.query("DIFF?", parse_float)

    def set_differentiator_corner(self, freq_hz: float) -> str:
        """CHANGES DEVICE STATE. Snapped to the nearest supported corner."""
        snapped = _nearest(freq_hz, DIFFERENTIATOR_CORNERS_HZ)
        if snapped != freq_hz:
            log.info("Differentiator corner %.1f Hz snapped to %.1f Hz",
                     freq_hz, snapped)
        return self.write(f"DIFF {snapped}")

    def gain_clamp_enabled(self) -> bool:
        """True when the 20 dB integrator gain clamp is engaged."""
        return self.query("GAINLIM?", parse_bool)

    def set_gain_clamp_enabled(self, on: bool) -> bool:
        """CHANGES DEVICE STATE."""
        self.write(f"GAINLIM {int(bool(on))}")
        return self.gain_clamp_enabled()

    def pll_bias(self) -> float:
        """PLL servo DC offset [V]."""
        return self.query("PLLBIAS?", parse_float)

    def set_pll_bias(self, volts: float) -> str:
        """CHANGES DEVICE STATE. Limited by PLLMAX/PLLMIN."""
        return self.write(f"PLLBIAS {volts}")

    def pll_output_max(self) -> float:
        return self.query("PLLMAX?", parse_float)

    def set_pll_output_max(self, volts: float) -> str:
        """Upper output limit [V], -12 to 12. CHANGES DEVICE STATE."""
        if not -12.0 <= volts <= 12.0:
            raise ValueError("PLLMAX must be within [-12, 12] V")
        return self.write(f"PLLMAX {volts}")

    def pll_output_min(self) -> float:
        return self.query("PLLMIN?", parse_float)

    def set_pll_output_min(self, volts: float) -> str:
        """Lower output limit [V], -12 to 12. CHANGES DEVICE STATE."""
        if not -12.0 <= volts <= 12.0:
            raise ValueError("PLLMIN must be within [-12, 12] V")
        return self.write(f"PLLMIN {volts}")

    def lock_range(self) -> float:
        """Error voltage below which the GUI calls the lock good [V]."""
        return self.query("LOCKRNG?", parse_float)

    def set_lock_range(self, volts: float) -> str:
        """CHANGES DEVICE STATE."""
        return self.write(f"LOCKRNG {volts}")

    # -- AUX servo filter ----------------------------------------------------

    def aux_enabled(self) -> bool:
        return self.query("AUXEN?", parse_bool)

    def set_aux_enabled(self, on: bool) -> bool:
        """CHANGES DEVICE STATE."""
        self.write(f"AUXEN {int(bool(on))}")
        return self.aux_enabled()

    def aux_inverted(self) -> bool:
        return self.query("AUXINVT?", parse_bool)

    def set_aux_inverted(self, inverted: bool) -> bool:
        """CHANGES DEVICE STATE."""
        self.write(f"AUXINVT {int(bool(inverted))}")
        return self.aux_inverted()

    def aux_target(self) -> float:
        """PLL output voltage the AUX loop drives toward [V]."""
        return self.query("AUXTGT?", parse_float)

    def set_aux_target(self, volts: float) -> str:
        """CHANGES DEVICE STATE."""
        return self.write(f"AUXTGT {volts}")

    def aux_gain(self) -> float:
        """AUX loop gain (unity-gain time constant), 0 to 47.62."""
        return self.query("AUXGAIN?", parse_float)

    def set_aux_gain(self, gain: float) -> str:
        """CHANGES DEVICE STATE."""
        if not 0.0 <= gain <= 47.62:
            raise ValueError("AUX gain must be within [0.0, 47.62]")
        return self.write(f"AUXGAIN {gain}")

    def aux_bias(self) -> float:
        return self.query("AUXBIAS?", parse_float)

    def set_aux_bias(self, volts: float) -> str:
        """CHANGES DEVICE STATE. Limited by AUXMAX/AUXMIN."""
        return self.write(f"AUXBIAS {volts}")

    def aux_output_max(self) -> float:
        return self.query("AUXMAX?", parse_float)

    def set_aux_output_max(self, volts: float) -> str:
        """CHANGES DEVICE STATE."""
        if not -12.0 <= volts <= 12.0:
            raise ValueError("AUXMAX must be within [-12, 12] V")
        return self.write(f"AUXMAX {volts}")

    def aux_output_min(self) -> float:
        return self.query("AUXMIN?", parse_float)

    def set_aux_output_min(self, volts: float) -> str:
        """CHANGES DEVICE STATE."""
        if not -12.0 <= volts <= 12.0:
            raise ValueError("AUXMIN must be within [-12, 12] V")
        return self.write(f"AUXMIN {volts}")

    # -- sweep ---------------------------------------------------------------

    def ramp_enabled(self) -> bool:
        return self.query("RAMP?", parse_bool)

    def set_ramp_enabled(self, on: bool) -> bool:
        """CHANGES DEVICE STATE."""
        self.write(f"RAMP {int(bool(on))}")
        return self.ramp_enabled()

    def ramp_channel(self) -> RampChannel:
        return RampChannel(self.query("RAMPCH?", parse_int))

    def set_ramp_channel(self, channel: "int | RampChannel") -> RampChannel:
        """CHANGES DEVICE STATE."""
        self.write(f"RAMPCH {int(channel)}")
        return self.ramp_channel()

    def ramp_points(self) -> int:
        return self.query("RAMPNUM?", parse_int)

    def set_ramp_points(self, points: int) -> str:
        """Sweep datapoints, 1 to 1023 (100-300 recommended). CHANGES DEVICE STATE."""
        if not 1 <= int(points) <= 1023:
            raise ValueError("ramp points must be within [1, 1023]")
        return self.write(f"RAMPNUM {int(points)}")

    def ramp_frequency(self) -> float:
        return self.query("RAMPFRQ?", parse_float)

    def set_ramp_frequency(self, freq_hz: float) -> str:
        """Sweep rate [Hz]; achievable range depends on RAMPNUM.
        CHANGES DEVICE STATE."""
        return self.write(f"RAMPFRQ {freq_hz}")

    def ramp_span(self) -> float:
        """Sweep range [V]."""
        return self.query("RAMPSWP?", parse_float)

    def set_ramp_span(self, volts: float) -> str:
        """CHANGES DEVICE STATE. Clipped by the ramp channel's output limits."""
        return self.write(f"RAMPSWP {volts}")

    def ramp_adc_gain(self) -> RampADCGain:
        return RampADCGain(self.query("RAMPADC?", parse_int))

    def set_ramp_adc_gain(self, mode: "int | RampADCGain") -> RampADCGain:
        """Phase error graph 1x/8x gain. CHANGES DEVICE STATE."""
        self.write(f"RAMPADC {int(mode)}")
        return self.ramp_adc_gain()

    # -- beat note input -----------------------------------------------------

    def beat_note_path(self) -> float:
        """Max beat note frequency / signal path [MHz]: 1000, 3000 or 12000."""
        return self.query("BNMAX?", parse_float)

    def set_beat_note_path(self, freq_mhz: float) -> str:
        """CHANGES DEVICE STATE (switches the RF signal path)."""
        if freq_mhz not in BEAT_NOTE_PATHS_MHZ:
            raise ValueError(f"BNMAX must be one of {BEAT_NOTE_PATHS_MHZ} MHz")
        return self.write(f"BNMAX {freq_mhz}")

    def n1_divider(self) -> int:
        """Divider between the beat note and the PFD."""
        return self.query("N1DIV?", parse_int)

    def set_n1_divider(self, value: int) -> str:
        """CHANGES DEVICE STATE. Allowed values depend on the BNMAX path:
        1-16 in low/mid, 4-64 in high."""
        if int(value) not in set(N1_DIVIDERS_LOW_MID) | set(N1_DIVIDERS_HIGH):
            raise ValueError("N1 divider must be a power of two in 1..64")
        return self.write(f"N1DIV {int(value)}")

    def n2_divider(self) -> int:
        """PFD N (beat note) divider."""
        return self.query("N2DIV?", parse_int)

    def set_n2_divider(self, value: int) -> str:
        """CHANGES DEVICE STATE. Accepts 1 to 8191."""
        if not 1 <= int(value) <= 8191:
            raise ValueError("N2 divider must be within [1, 8191]")
        return self.write(f"N2DIV {int(value)}")

    def divided_beat_note(self) -> float:
        """Divided beat note frequency measured against the MCU clock [MHz]."""
        return self.query("READBN?", parse_float)

    # -- reference source ----------------------------------------------------

    def reference_frequency(self) -> float:
        """External reference frequency measured against the MCU clock [MHz]."""
        return self.query("READREF?", parse_float)

    def ext_ref_bandwidth_limit(self) -> float:
        """Low-pass corner on the external Ref In [MHz]."""
        return self.query("EREFBWL?", parse_float)

    def set_ext_ref_bandwidth_limit(self, freq_mhz: float) -> str:
        """CHANGES DEVICE STATE. Accepts 12, 40, 125 or 300 MHz."""
        if freq_mhz not in EXT_REF_BW_LIMITS_MHZ:
            raise ValueError(f"EREFBWL must be one of {EXT_REF_BW_LIMITS_MHZ} MHz")
        return self.write(f"EREFBWL {freq_mhz}")

    def dds_is_pfd_reference(self) -> bool:
        """True when the DDS (rather than the Ref In SMA) drives the PFD."""
        return self.query("PFDDDS?", parse_bool)

    def set_dds_is_pfd_reference(self, on: bool) -> bool:
        """CHANGES DEVICE STATE."""
        self.write(f"PFDDDS {int(bool(on))}")
        return self.dds_is_pfd_reference()

    def dds_internal_reference(self) -> bool:
        """True when the DDS runs off its internal oscillator."""
        return self.query("DDSINT?", parse_bool)

    def set_dds_internal_reference(self, on: bool) -> bool:
        """CHANGES DEVICE STATE."""
        self.write(f"DDSINT {int(bool(on))}")
        return self.dds_internal_reference()

    def dds_reference_frequency(self) -> float:
        """User-declared DDS input reference frequency [MHz]."""
        return self.query("DDSREFF?", parse_float)

    def set_dds_reference_frequency(self, freq_mhz: float) -> str:
        """CHANGES DEVICE STATE. External-reference mode only; accepts
        7.5, 8, 10, 12 or 12.5 MHz."""
        if freq_mhz not in DDS_REF_FREQS_MHZ:
            raise ValueError(f"DDSREFF must be one of {DDS_REF_FREQS_MHZ} MHz")
        return self.write(f"DDSREFF {freq_mhz}")

    def m1_divider(self) -> int:
        """Divider between the reference oscillator and the PFD."""
        return self.query("M1DIV?", parse_int)

    def set_m1_divider(self, value: int) -> str:
        """CHANGES DEVICE STATE. Accepts 1, 2, 4, 8 or 16."""
        if int(value) not in M1_DIVIDERS:
            raise ValueError(f"M1 divider must be one of {M1_DIVIDERS}")
        return self.write(f"M1DIV {int(value)}")

    def m2_divider(self) -> int:
        """PFD reference divider."""
        return self.query("M2DIV?", parse_int)

    def set_m2_divider(self, value: int) -> str:
        """CHANGES DEVICE STATE. Accepts 1 to 16383."""
        if not 1 <= int(value) <= 16383:
            raise ValueError("M2 divider must be within [1, 16383]")
        return self.write(f"M2DIV {int(value)}")

    def dds_auto_configure(self) -> str:
        """Auto-configure M1, DDS reference frequency and external reference
        bandwidth limit from the detected input.

        CHANGES DEVICE STATE (note the missing '?' -- this is not a query).
        """
        return self.write("DDSAUTO")

    # -- external I/O --------------------------------------------------------

    def monitor_mux(self) -> MonitorMux:
        return MonitorMux(self.query("MUXO?", parse_int))

    def set_monitor_mux(self, setting: "int | MonitorMux") -> MonitorMux:
        """Back panel monitor mux. CHANGES DEVICE STATE."""
        self.write(f"MUXO {int(setting)}")
        return self.monitor_mux()

    def fp_output1_enabled(self) -> bool:
        """Front panel Output 1 (ramp) routing."""
        return self.query("FP1EN?", parse_bool)

    def set_fp_output1_enabled(self, on: bool) -> bool:
        """CHANGES DEVICE STATE."""
        self.write(f"FP1EN {int(bool(on))}")
        return self.fp_output1_enabled()

    def fp_output2_enabled(self) -> bool:
        return self.query("FP2EN?", parse_bool)

    def set_fp_output2_enabled(self, on: bool) -> bool:
        """CHANGES DEVICE STATE."""
        self.write(f"FP2EN {int(bool(on))}")
        return self.fp_output2_enabled()

    def fp_output2_mux(self) -> FrontPanelMux:
        return FrontPanelMux(self.query("FPMUXO?", parse_int))

    def set_fp_output2_mux(self, setting: "int | FrontPanelMux") -> FrontPanelMux:
        """CHANGES DEVICE STATE."""
        self.write(f"FPMUXO {int(setting)}")
        return self.fp_output2_mux()

    def fp_mod_in_a_enabled(self) -> bool:
        """Front panel Input A summed into the PLL Out SMA."""
        return self.query("FPAINEN?", parse_bool)

    def set_fp_mod_in_a_enabled(self, on: bool) -> bool:
        """CHANGES DEVICE STATE."""
        self.write(f"FPAINEN {int(bool(on))}")
        return self.fp_mod_in_a_enabled()

    def fp_mod_in_b_enabled(self) -> bool:
        """Front panel Input B summed into the AUX Out SMA."""
        return self.query("FPBINEN?", parse_bool)

    def set_fp_mod_in_b_enabled(self, on: bool) -> bool:
        """CHANGES DEVICE STATE."""
        self.write(f"FPBINEN {int(bool(on))}")
        return self.fp_mod_in_b_enabled()

    def trigger_in_mode(self) -> TriggerInMode:
        return TriggerInMode(self.query("TRIGI?", parse_int))

    def set_trigger_in_mode(self, mode: "int | TriggerInMode") -> TriggerInMode:
        """CHANGES DEVICE STATE."""
        self.write(f"TRIGI {int(mode)}")
        return self.trigger_in_mode()

    def trigger_out_mode(self) -> TriggerOutMode:
        return TriggerOutMode(self.query("TRIGO?", parse_int))

    def set_trigger_out_mode(self, mode: "int | TriggerOutMode") -> TriggerOutMode:
        """CHANGES DEVICE STATE."""
        self.write(f"TRIGO {int(mode)}")
        return self.trigger_out_mode()

    # -- calibration ---------------------------------------------------------

    def calibration_status(self) -> str:
        """Status / result of the last self-calibration."""
        return self.query("_CAL?", parse_str)

    def opamp_offset_trim(self) -> int:
        return self.query("_OPOFST?", parse_int)

    def vga_offset_trim(self) -> int:
        return self.query("_VGAOFT?", parse_int)

    def vga_gain_offset_trim(self) -> int:
        return self.query("_VGAGOF?", parse_int)

    def servo_offset_trim(self) -> int:
        return self.query("_CALSV?", parse_int)

    def aux_offset_trim(self) -> int:
        return self.query("_CALAX?", parse_int)

    def start_self_calibration(self) -> str:
        """Start self-calibration. CHANGES DEVICE STATE.

        Note that '_SELFCAL 0' queries status while '_SELFCAL 1' starts a
        calibration, so the command name cannot be classified as read-safe;
        use calibration_status() (_CAL?) for a pure read.
        """
        return self.write("_SELFCAL 1")

    # Deliberately not implemented: _FACTORY, which wipes the system
    # calibration values. Issue it by hand if you truly mean to.

    # -- advanced ------------------------------------------------------------

    def integrator_hold(self) -> bool:
        return self.query("HOLD?", parse_bool)

    def set_integrator_hold(self, on: bool) -> bool:
        """Engage/disengage integrator hold. CHANGES DEVICE STATE."""
        self.write(f"HOLD {int(bool(on))}")
        return self.integrator_hold()

    def disable_charge_pump(self) -> str:
        """Disable CP output, engaging servo integrator hold.

        CHANGES DEVICE STATE (note the missing '?' -- this is not a query).
        """
        return self.write("NOCP")

    def dds_frequency(self) -> float:
        """DDS frequency [MHz]."""
        return self.query("DDSFREQ?", parse_float)

    def set_dds_frequency(self, freq_mhz: float) -> str:
        """CHANGES DEVICE STATE."""
        return self.write(f"DDSFREQ {freq_mhz}")

    def dds_raw(self) -> int:
        """DDS tuning word; frequency = 960 MHz * raw / 2**32."""
        return self.query("DDSRAW?", parse_int)

    def set_dds_raw(self, raw: int) -> str:
        """CHANGES DEVICE STATE."""
        return self.write(f"DDSRAW {int(raw)}")

    @staticmethod
    def dds_raw_to_mhz(raw: int) -> float:
        return 960.0 * int(raw) / 2 ** 32

    @staticmethod
    def dds_mhz_to_raw(freq_mhz: float) -> int:
        return int(round(freq_mhz * 2 ** 32 / 960.0))

    # -- convenience ---------------------------------------------------------

    def derived_fields(self, values: Dict[str, Any]) -> Dict[str, Any]:
        """Flag lock quality the same way the front panel does: the lock counts
        as good when |error signal| is inside the configured lock range."""
        derived: Dict[str, Any] = {}
        err = values.get("adc_error_signal")
        rng = values.get("lock_range")
        if err is not None and rng is not None:
            derived["error_within_lock_range"] = abs(err) < abs(rng)
            derived["lock_ok"] = bool(values.get("servo_on")) and abs(err) < abs(rng)
        if self.ESTIMATE_BEAT_NOTE:
            bn = values.get("beat_note_divided")
            n1 = values.get("n1_divider")
            if bn is not None and n1:
                derived["beat_note_estimate"] = bn * n1
        return derived
