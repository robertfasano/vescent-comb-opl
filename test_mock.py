"""Smoke test for the Vescent drivers against simulated instruments."""
import os, tempfile, threading, time
from vescent_serial import (MonitoredDevice, ConsoleSink, monitor, sweep_once)
from rubricomb import RubriComb
import rubricomb, slice_opl
from slice_opl import SliceOPL, ADCChannel

COMB_REPLIES = {
    "*IDN?": "Vescent,RUBRIComb,000026,S-1.241,LD-1.22,LD-1.22,Gen2-1.19",
    "MSTRCTL?": "MSTRCTL? 2", "PZT_ENABLE?": "Off", "PZT_SLSRVEN?": "On",
    "CURR_SLSRVEN?": "On", "CAVERROR? 1": "49152", "CAVTEMPSET? 0": "25.0",
    "CAVTEMP? 0": "24.21", "CAVTERROR? 0": "353.807281", "CAVCURRENT? 0": ".654",
    "CAVSLSRVGN? 0": "-58.0", "CAVSLSRVOS? 0": "5.0", "CAVDCBIASV? 1": "20.300",
    "CAVOUTVOLT? 1": "30.195938", "OSCERROR? 1": "49408", "OSCTEMP? 0": "24.21",
    "OSCTERROR? 0": ".0024", "OSCTCURR? 0": ".654", "OSCMODCURR?": "3.933144",
    "OSCCCURSET? 1": "0.370000", "OSCCCURR? 1": "1404.400146",
    "OSCCVOLTCC? 1": "1.482328", "OSCINTERLK?": "On", "AMPERROR? 1": "49152",
    "AMPTEMP? 0": "24.21", "AMPTERROR? 0": ".0024", "AMPTCURR? 0": ".654",
    "AMPCCURSET? 1": "0.370000", "AMPCCURR? 1": "1404.400146",
    "AMPCVOLTCC? 1": "1.482328", "AMPINTERLK?": "On",
    "CAVTEMPSET 0 24.0": "24.0", "#SAVESETTINGS": "#SAVESETTINGS",
    "SERVO 0": "Off",
}

OPL_REPLIES = {
    "*IDN?": "Vescent,SLICE-OPL,000026,S-V1.242,OPL-V1.27",
    "#SCBKLT?": "#SCBKLT? 5", "#SCVOL?": "#SCVOL? 5",
    "SERVO?": "On", "BNTGT?": "1000.0",
    "READVOLT 1": "0.0123", "READVOLT 2": "0.0984", "READVOLT 3": "-1.204",
    "READVOLT 4": "2.501", "READVOLT 5": "0.334", "READVOLT 6": "0.011",
    "READVOLT 7": "0.021", "READVOLT 8": "0.0001",
    "PLLINVT?": "Off", "PLLGAIN?": "-12.0", "INT?": "10000", "DIFF?": "30000",
    "GAINLIM?": "On", "PLLBIAS?": "0.5", "PLLMAX?": "10.0", "PLLMIN?": "-10.0",
    "LOCKRNG?": "0.2",
    "AUXEN?": "On", "AUXINVT?": "Off", "AUXTGT?": "0.0", "AUXGAIN?": "12.5",
    "AUXBIAS?": "0.0", "AUXMAX?": "10.0", "AUXMIN?": "-10.0",
    "RAMP?": "Off", "RAMPCH?": "0", "RAMPNUM?": "200", "RAMPFRQ?": "20.0",
    "RAMPSWP?": "5.0", "RAMPADC?": "128",
    "BNMAX?": "12000", "N1DIV?": "8", "N2DIV?": "1", "READBN?": "1225.03",
    "READREF?": "10.000001", "EREFBWL?": "125", "PFDDDS?": "On",
    "DDSINT?": "Off", "DDSREFF?": "10", "M1DIV?": "1", "M2DIV?": "1",
    "MUXO?": "1", "FP1EN?": "Off", "FP2EN?": "On", "FPMUXO?": "3",
    "FPAINEN?": "Off", "FPBINEN?": "Off", "TRIGI?": "0", "TRIGO?": "0",
    "HOLD?": "Off", "DDSFREQ?": "153.1287500", "DDSRAW?": "684919706",
    "_CAL?": "Pass", "_OPOFST?": "-120", "_VGAOFT?": "35", "_VGAGOF?": "-7",
    "_CALSV?": "12", "_CALAX?": "-3",
    "SERVO 0": "Off", "PLLGAIN -10.0": "-10.0",
}


class FakeSerial:
    is_open = True

    def __init__(self, replies, echo=True):
        self.buf = bytearray(); self.replies = replies; self.echo = echo
        self.sent = []

    def write(self, data):
        cmd = data.decode().strip()
        self.sent.append(cmd)
        if self.echo:
            self.buf += cmd.encode() + b"\r\n"
        reply = self.replies.get(cmd.upper(), self.replies.get(cmd))
        if reply is not None:
            self.buf += reply.encode() + b"\r\n"
        return len(data)

    def read(self, n=1):
        if not self.buf:
            time.sleep(0.001); return b""
        out = bytes(self.buf[:n]); del self.buf[:n]; return out

    def flush(self): pass
    def reset_input_buffer(self): self.buf.clear()
    def reset_output_buffer(self): pass
    def close(self): self.is_open = False


def make(cls, replies, echo=True, **kw):
    return cls(transport=FakeSerial(replies, echo), timeout=0.3,
               inter_command_delay=0, **kw)


# --- RUBRIComb ------------------------------------------------------------
for echo in (True, False):
    comb = make(RubriComb, COMB_REPLIES, echo)
    print(f"\n=== RUBRIComb echo={echo} ===")
    print("IDN:", comb.idn())
    print("mode:", comb.master_mode().name, "| cav T:", comb.cavity_temperature())
    print("errors:", comb.errors())
    vals, fails = comb.read_all()
    assert not fails, fails
    assert len(vals) == len(rubricomb.MONITOR_PARAMS) + 3, len(vals)
    print(f"read_all -> {len(vals)} fields; oscillator_ok={vals['oscillator_ok']}")

# --- SLICE-OPL ------------------------------------------------------------
for echo in (True, False):
    opl = make(SliceOPL, OPL_REPLIES, echo)
    print(f"\n=== SLICE-OPL echo={echo} ===")
    print("IDN:", opl.idn())
    print("servo:", opl.servo_enabled(), "| err sig:", opl.error_signal(),
          "V | lock range:", opl.lock_range(), "V")
    print("ramp gain mode:", opl.ramp_adc_gain().name,
          "| monitor mux:", opl.monitor_mux().name)
    print("DDS:", opl.dds_frequency(), "MHz | raw->MHz:",
          round(opl.dds_raw_to_mhz(opl.dds_raw()), 6))
    vals, fails = opl.read_all()
    assert not fails, fails
    print(f"read_all -> {len(vals)} fields; lock_ok={vals['lock_ok']}")
    assert vals["lock_ok"] is True
    assert vals["cal_status"] == "Pass"

assert opl.read_voltage(ADCChannel.PLL_OUTPUT) == 2.501

# --- setters send on the first call -----------------------------------
print("\n=== setters ===")
comb = make(RubriComb, COMB_REPLIES)
opl = make(SliceOPL, OPL_REPLIES)
assert comb.set_cavity_temperature_setpoint(24.0) == 24.0
assert comb.save_settings() == "#SAVESETTINGS"
assert opl.set_servo_enabled(False) is False
assert opl.set_pll_gain(-10.0) == "-10.0"
print("RubriComb/SliceOPL setters send correctly")

# --- monitor loop over both devices ---------------------------------------
print("\n=== monitor loop ===")
devices = [MonitoredDevice("rubricomb", make(RubriComb, COMB_REPLIES)),
           MonitoredDevice("slice_opl", make(SliceOPL, OPL_REPLIES))]
stop = threading.Event()
t = threading.Thread(target=monitor, args=(devices, ConsoleSink()),
                     kwargs={"interval": 0.3, "stop_event": stop})
t.start(); time.sleep(0.7); stop.set(); t.join()

# --- degraded sweep --------------------------------------------------------
class BadSerial(FakeSerial):
    def write(self, data):
        cmd = data.decode().strip()
        self.sent.append(cmd)
        if cmd.upper().startswith("READVOLT 3"):
            return len(data)                       # no reply
        if cmd.upper().startswith("PLLGAIN?"):
            self.buf += b"garbage\r\n"; return len(data)
        return super().write(cmd.encode() + b"\r")

opl = SliceOPL(transport=BadSerial(OPL_REPLIES), timeout=0.2, inter_command_delay=0)
vals, fails = opl.read_all()
assert set(fails) == {"adc_integrator_mon", "pll_gain"}, fails
print(f"\ndegraded sweep: {len(vals)} fields, failures={sorted(fails)}")

# --- main() CLI --------------------------------------------------------
import main as main_mod

rc = main_mod.main(["--once", "--config", "/nonexistent/config.yaml"])
assert rc == 2, rc
print("\nmain() with missing config file -> exit 2")

with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as f:
    f.write("influx:\n  bucket: test\n")  # no rubricomb / slice_opls
    empty_cfg = f.name
try:
    rc = main_mod.main(["--once", "--config", empty_cfg])
    assert rc == 2, rc
    print("main() with no instruments configured -> exit 2")
finally:
    os.unlink(empty_cfg)

print("ALL TESTS PASSED")
