
#PDN Load Transient Test Script


import pyvisa
import time
import csv
import os
import numpy as np
from datetime import datetime


# CONFIGURATION


# USB VISA Addresses
# NOTE: Replace these addresses with actual instrument VISA addresses
# Run list_visa_resources() to discover addresses on your system
VISA_PSU    = "USB0::0x05E6::0x2230::9102370::INSTR"   # Keithley 2230-30-1
VISA_LOAD   = "USB0::0x05E6::0x2380::4512345::INSTR"   # Keithley 2380
VISA_SCOPE  = "USB0::0x0957::0x9001::MY55310101::INSTR" # Keysight DSOX6004A
VISA_DMM    = "USB0::0x05E6::0x6500::4123456::INSTR"   # Keithley DMM6500

# Board Information
BOARD_SN    = "SN-001"          # Change per board under test
OPERATOR    = "XYZ"         # Tester name
OUTPUT_DIR  = "transient_results"  # Folder for CSV, waveforms, report

# Input Supply 
VIN_NOMINAL = 5.00 #Volts
VIN_CHANNEL = 1 # PSU channel number

# Rail Definitions 
# Format: "Rail Name": (Nominal_V, Max_I_A, Scope_Channel, DMM_Input)
RAILS = {
    "+3V6": {"vnom": 3.6, "imax": 2.5, "scope_ch": 1, "tol_pct": 5.0},
    "+3V3": {"vnom": 3.3, "imax": 3.0, "scope_ch": 2, "tol_pct": 5.0},
    "+1V8": {"vnom": 1.8, "imax": 3.0, "scope_ch": 3, "tol_pct": 5.0},
    "+2V5": {"vnom": 2.5, "imax": 1.5, "scope_ch": 4, "tol_pct": 5.0},
}

# Load Step Parameters 
LOAD_LOW_PCT    = 10            # % of Imax for low state
LOAD_HIGH_PCT   = 90            # % of Imax for high state
LOAD_SLEW       = 1.0           # A/us — slew rate for load step
STEP_DWELL_S    = 0.5           # seconds to dwell at each load level
NUM_CAPTURES    = 10            # number of transient captures per rail

# Acceptance Criteria 
# Derived from ±5% voltage tolerance
MAX_OVERSHOOT_PCT   = 5.0       # % of Vnom — max allowed positive spike
MAX_UNDERSHOOT_PCT  = 5.0       # % of Vnom — max allowed negative dip
MAX_RECOVERY_US     = 200.0     # microseconds — time to return within ±1%
MAX_SETTLING_US     = 500.0     # microseconds — time to settle within ±0.5%

# Scope Timebase & Trigger
SCOPE_TIMESCALE     = "200E-6"  # 200us/div — shows 2ms total window
SCOPE_VERT_SCALE    = "100E-3"  # 100mV/div
SCOPE_BW_LIMIT      = "20E6"    # 20MHz bandwidth limit
SCOPE_TRIG_LEVEL    = "50E-3"   # 50mV trigger threshold (AC coupled)


# List available VISA resources (run once to find addresses)


def list_visa_resources():
    rm = pyvisa.ResourceManager()
    resources = rm.list_resources()
    print("\nAvailable VISA Resources:")
    print("-" * 50)
    for r in resources:
        print(f"  {r}")
    print("-" * 50)
    return resources



# INSTRUMENT CLASS — Wraps common SCPI operations with error checking


class Instrument:
    def __init__(self, rm, visa_addr, name, timeout_ms=10000):
        self.name = name
        self.addr = visa_addr
        try:
            self.dev = rm.open_resource(visa_addr)
            self.dev.timeout = timeout_ms
            self.dev.write_termination  = '\n'
            self.dev.read_termination   = '\n'
            idn = self.query("*IDN?")
            print(f"  [OK] {name}: {idn.strip()}")
        except Exception as e:
            raise ConnectionError(f"Failed to connect to {name} at {visa_addr}: {e}")

    def write(self, cmd):
        self.dev.write(cmd)

    def query(self, cmd):
        return self.dev.query(cmd)

    def query_float(self, cmd):
        return float(self.query(cmd))

    def reset(self):
        self.write("*RST")
        time.sleep(2)

    def clear(self):
        self.write("*CLS")

    def close(self):
        self.dev.close()



# INSTRUMENT SETUP FUNCTIONS


def setup_psu(psu, voltage=VIN_NOMINAL, channel=VIN_CHANNEL):
    """Configure PSU: set output voltage, current limit, enable output."""
    print(f"\n[PSU] Configuring CH{channel} → {voltage}V, 5A limit")
    psu.reset()
    psu.write(f"INST:SEL CH{channel}")
    psu.write(f"VOLT {voltage}")
    psu.write("CURR 5.0")          # 5A input current limit
    psu.write("OUTP ON")
    time.sleep(1)
    actual_v = psu.query_float(f"MEAS:VOLT? CH{channel}")
    print(f"[PSU] Measured Vin = {actual_v:.4f}V")
    return actual_v


def setup_load(load, current=0.0):
    """Configure electronic load in CC mode at specified current."""
    load.reset()
    load.write("FUNC CURR")        # Constant Current mode
    load.write(f"CURR:STAT:L1 {current:.4f}")   # Level 1 current
    load.write("INP OFF")          # Load input off initially
    print(f"[LOAD] Configured CC mode, I = {current:.3f}A, input OFF")


def setup_scope_channel(scope, channel, vnom, scale=SCOPE_VERT_SCALE):
    """Configure one oscilloscope channel for a specific rail."""
    ch = f"CHAN{channel}"
    scope.write(f"{ch}:DISP ON")
    scope.write(f"{ch}:COUP AC")           # AC coupling to see transient
    scope.write(f"{ch}:SCAL {scale}")      # Vertical scale
    scope.write(f"{ch}:BWL {SCOPE_BW_LIMIT}")  # 20MHz bandwidth limit
    scope.write(f"{ch}:OFFS 0")            # Zero offset
    print(f"[SCOPE] CH{channel}: AC coupling, {scale}V/div, 20MHz BW limit")


def setup_scope_trigger(scope, trig_channel=1):
    """Configure scope trigger on the specified channel, rising edge."""
    scope.write(f"TRIG:SOUR CHAN{trig_channel}")
    scope.write("TRIG:SLOP POS")
    scope.write(f"TRIG:LEV {SCOPE_TRIG_LEVEL}")
    scope.write("TRIG:MODE SING")          # Single trigger — capture one event
    scope.write(f"TIM:SCAL {SCOPE_TIMESCALE}")
    scope.write("TIM:REF CENT")            # Trigger at centre of screen
    print(f"[SCOPE] Trigger: CH{trig_channel}, rising edge, {SCOPE_TRIG_LEVEL}V, single")


def setup_dmm(dmm):
    """Configure DMM for DC voltage measurement."""
    dmm.reset()
    dmm.write("SENS:FUNC 'VOLT:DC'")
    dmm.write("SENS:VOLT:DC:RANG:AUTO ON")
    dmm.write("SENS:VOLT:DC:NPLC 10")     # 10 PLC — high accuracy
    print("[DMM] Configured: DC Voltage, auto-range, 10 NPLC")



# MEASUREMENT FUNCTIONS


def measure_dc_voltage(dmm):
    """Take a single DC voltage reading from DMM."""
    return dmm.query_float("READ?")


def set_load_current(load, current):
    """Set load to a specific current level and enable input."""
    load.write(f"CURR:STAT:L1 {current:.4f}")
    load.write("INP ON")
    time.sleep(0.05)    # small settle before confirming


def capture_transient_waveform(scope, rail_name, rail_cfg, capture_num, output_dir):
    """
    Arm scope, execute load step, capture waveform.
    Returns waveform data as numpy array (time_s, voltage_V).
    """
    ch      = rail_cfg["scope_ch"]
    vnom    = rail_cfg["vnom"]
    imax    = rail_cfg["imax"]
    i_low   = imax * (LOAD_LOW_PCT  / 100.0)
    i_high  = imax * (LOAD_HIGH_PCT / 100.0)

    # Arm scope for single trigger
    scope.write("SING")
    time.sleep(0.3)     # let scope arm

    # Execute load step: low → high
    set_load_current(scope, i_low)   # already at low — this is a no-op on first call
    time.sleep(STEP_DWELL_S)
    set_load_current(scope, i_high)  # rising step — this triggers the scope

    # Wait for scope to trigger and capture
    time.sleep(float(SCOPE_TIMESCALE) * 10 * 1.5)  # wait > one full screen

    # Check trigger status
    trig_status = scope.query("TRIG:STAT?").strip()
    if trig_status not in ["STOP", "TRIG"]:
        print(f"  [WARN] Scope trigger status: {trig_status} — waveform may be incomplete")

    # Download waveform from scope
    scope.write(f"WAV:SOUR CHAN{ch}")
    scope.write("WAV:FORM ASCII")
    scope.write("WAV:POIN 1000")

    preamble    = scope.query("WAV:PRE?").split(',')
    x_increment = float(preamble[4])
    x_origin    = float(preamble[5])
    y_increment = float(preamble[7])
    y_origin    = float(preamble[8])
    y_reference = float(preamble[9])

    raw_data    = scope.query("WAV:DATA?")
    # Strip IEEE header (#NXXXXXXXX) if present
    if raw_data.startswith('#'):
        n_digits = int(raw_data[1])
        raw_data = raw_data[2 + n_digits:]

    samples     = np.array([float(v) for v in raw_data.strip().split(',')])
    voltage_V   = (samples - y_reference) * y_increment + y_origin
    time_s      = x_origin + np.arange(len(samples)) * x_increment

    # Save raw waveform CSV
    rail_safe   = rail_name.replace('+', 'p').replace('V', 'v')
    wfm_file    = os.path.join(output_dir, f"waveform_{rail_safe}_cap{capture_num:02d}.csv")
    with open(wfm_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["Time_s", "Voltage_V"])
        for t, v in zip(time_s, voltage_V):
            writer.writerow([f"{t:.9f}", f"{v:.6f}"])

    print(f"  [SCOPE] Waveform saved → {wfm_file}")
    return time_s, voltage_V


def analyse_transient(time_s, voltage_V, vnom, tol_pct=5.0):
   
    v_mv            = voltage_V * 1000.0    # convert to mV for readability
    vnom_mv         = vnom * 1000.0

    # Find overshoot and undershoot
    v_max_mv        = np.max(v_mv)
    v_min_mv        = np.min(v_mv)
    overshoot_mv    = max(0.0, v_max_mv)    # AC coupled: positive peak = overshoot
    undershoot_mv   = max(0.0, -v_min_mv)  # AC coupled: negative trough = undershoot

    # Trigger index — find the largest dV/dt (load step location)
    dv              = np.diff(voltage_V)
    trig_idx        = np.argmax(np.abs(dv))

    # Recovery time: time after trigger until |V| < 1% of Vnom (AC coupled → within ±10mV at Vnom=1V)
    recovery_band_mv    = vnom_mv * 0.01    # 1% of Vnom
    recovery_time_us    = None
    for i in range(trig_idx, len(v_mv)):
        if abs(v_mv[i]) < recovery_band_mv:
            recovery_time_us = (time_s[i] - time_s[trig_idx]) * 1e6
            break

    # Settling time: time until |V| stays within 0.5% of Vnom for rest of capture
    settling_band_mv    = vnom_mv * 0.005  # 0.5% of Vnom
    settling_time_us    = None
    for i in range(trig_idx, len(v_mv) - 10):
        window = v_mv[i:i+10]
        if np.all(np.abs(window) < settling_band_mv):
            settling_time_us = (time_s[i] - time_s[trig_idx]) * 1e6
            break

    # Handle cases where recovery/settling was not observed in capture window
    if recovery_time_us is None:
        recovery_time_us = float('inf')
        print("  [WARN] Recovery not observed within capture window — widen timebase")

    if settling_time_us is None:
        settling_time_us = float('inf')
        print("  [WARN] Settling not observed within capture window — widen timebase")

    return {
        "overshoot_mv"      : round(overshoot_mv, 2),
        "undershoot_mv"     : round(undershoot_mv, 2),
        "recovery_time_us"  : round(recovery_time_us, 2),
        "settling_time_us"  : round(settling_time_us, 2),
    }


def evaluate_pass_fail(results, vnom):
    """Compare measured results against acceptance criteria. Returns pass/fail per parameter."""
    max_over_mv     = vnom * 1000 * (MAX_OVERSHOOT_PCT  / 100.0)
    max_under_mv    = vnom * 1000 * (MAX_UNDERSHOOT_PCT / 100.0)

    pf = {
        "overshoot"     : "PASS" if results["overshoot_mv"]     <= max_over_mv      else "FAIL",
        "undershoot"    : "PASS" if results["undershoot_mv"]    <= max_under_mv     else "FAIL",
        "recovery"      : "PASS" if results["recovery_time_us"] <= MAX_RECOVERY_US  else "FAIL",
        "settling"      : "PASS" if results["settling_time_us"] <= MAX_SETTLING_US  else "FAIL",
    }
    pf["overall"] = "PASS" if all(v == "PASS" for v in pf.values()) else "FAIL"
    return pf


# REPORT GENERATION


def generate_report(board_sn, operator, results_by_rail, output_dir, timestamp):
    """Generate a plain-text test report summarising all results."""
    report_file = os.path.join(output_dir, f"report_{board_sn}_{timestamp}.txt")

    with open(report_file, 'w') as f:
        f.write("=" * 70 + "\n")
        f.write("  PDN LOAD TRANSIENT TEST REPORT\n")
        f.write("=" * 70 + "\n")
        f.write(f"  Board Serial No : {board_sn}\n")
        f.write(f"  Operator        : {operator}\n")
        f.write(f"  Timestamp       : {timestamp}\n")
        f.write(f"  Input Voltage   : {VIN_NOMINAL}V\n")
        f.write(f"  Load Step       : {LOAD_LOW_PCT}% → {LOAD_HIGH_PCT}% of Imax\n")
        f.write(f"  Captures/Rail   : {NUM_CAPTURES}\n")
        f.write("=" * 70 + "\n\n")

        overall_board = "PASS"

        for rail_name, rail_data in results_by_rail.items():
            vnom        = RAILS[rail_name]["vnom"]
            max_over    = vnom * 1000 * (MAX_OVERSHOOT_PCT  / 100.0)
            max_under   = vnom * 1000 * (MAX_UNDERSHOOT_PCT / 100.0)

            captures    = rail_data["captures"]
            n_pass      = sum(1 for c in captures if c["overall"] == "PASS")
            n_fail      = NUM_CAPTURES - n_pass
            rail_result = "PASS" if n_fail == 0 else "FAIL"

            if rail_result == "FAIL":
                overall_board = "FAIL"

            f.write(f"RAIL: {rail_name}  (Vnom = {vnom}V, Imax = {RAILS[rail_name]['imax']}A)\n")
            f.write(f"  Result : {rail_result}  ({n_pass}/{NUM_CAPTURES} captures passed)\n")
            f.write(f"  Acceptance Criteria:\n")
            f.write(f"    Overshoot   < {max_over:.1f} mV\n")
            f.write(f"    Undershoot  < {max_under:.1f} mV\n")
            f.write(f"    Recovery    < {MAX_RECOVERY_US} us\n")
            f.write(f"    Settling    < {MAX_SETTLING_US} us\n")
            f.write("\n")

            # Statistics across all captures
            over_vals   = [c["overshoot_mv"]     for c in captures]
            under_vals  = [c["undershoot_mv"]     for c in captures]
            rec_vals    = [c["recovery_time_us"]  for c in captures if c["recovery_time_us"] != float('inf')]
            set_vals    = [c["settling_time_us"]  for c in captures if c["settling_time_us"] != float('inf')]

            f.write("  Statistics (across all captures):\n")
            f.write(f"    {'Parameter':<20} {'Min':>10} {'Max':>10} {'Mean':>10} {'StdDev':>10}\n")
            f.write(f"    {'-'*62}\n")

            def stat_row(label, vals, unit):
                if vals:
                    return (f"    {label:<20} {min(vals):>9.2f}{unit} {max(vals):>9.2f}{unit} "
                            f"{np.mean(vals):>9.2f}{unit} {np.std(vals):>9.2f}{unit}\n")
                return f"    {label:<20} {'N/A':>10}\n"

            f.write(stat_row("Overshoot (mV)",    over_vals,  ""))
            f.write(stat_row("Undershoot (mV)",   under_vals, ""))
            f.write(stat_row("Recovery (us)",     rec_vals,   ""))
            f.write(stat_row("Settling (us)",     set_vals,   ""))

            # Per-capture breakdown
            f.write("\n  Per-Capture Results:\n")
            f.write(f"    {'Cap':>4} {'Over(mV)':>10} {'Under(mV)':>10} "
                    f"{'Recov(us)':>10} {'Settl(us)':>10} {'Result':>8}\n")
            f.write(f"    {'-'*56}\n")
            for i, c in enumerate(captures, 1):
                rec_str = f"{c['recovery_time_us']:>10.1f}" if c["recovery_time_us"] != float('inf') else "   >window"
                set_str = f"{c['settling_time_us']:>10.1f}" if c["settling_time_us"] != float('inf') else "   >window"
                f.write(f"    {i:>4} {c['overshoot_mv']:>10.2f} {c['undershoot_mv']:>10.2f} "
                        f"{rec_str} {set_str} {c['overall']:>8}\n")
            f.write("\n" + "-" * 70 + "\n\n")

        f.write("=" * 70 + "\n")
        f.write(f"  BOARD OVERALL RESULT: {overall_board}\n")
        f.write("=" * 70 + "\n")

    print(f"\n[REPORT] Saved → {report_file}")
    return report_file, overall_board



# CSV LOGGER


def init_csv(output_dir, board_sn, timestamp):
    """Create and return a CSV file with headers."""
    csv_file = os.path.join(output_dir, f"results_{board_sn}_{timestamp}.csv")
    with open(csv_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            "Timestamp", "Board_SN", "Rail", "Capture_No",
            "Overshoot_mV", "Undershoot_mV", "Recovery_us", "Settling_us",
            "OS_PF", "US_PF", "Rec_PF", "Set_PF", "Overall_PF"
        ])
    return csv_file


def log_csv(csv_file, board_sn, rail_name, capture_num, results, pf):
    """Append one row to the CSV log."""
    with open(csv_file, 'a', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            board_sn, rail_name, capture_num,
            results["overshoot_mv"],
            results["undershoot_mv"],
            results["recovery_time_us"] if results["recovery_time_us"] != float('inf') else "TIMEOUT",
            results["settling_time_us"] if results["settling_time_us"] != float('inf') else "TIMEOUT",
            pf["overshoot"], pf["undershoot"], pf["recovery"], pf["settling"],
            pf["overall"]
        ])


# MAIN TEST SEQUENCE


def run_transient_test():

    #  Setup output directory 
    timestamp   = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir  = os.path.join(OUTPUT_DIR, f"{BOARD_SN}_{timestamp}")
    os.makedirs(output_dir, exist_ok=True)
    print(f"\n[INFO] Output directory: {output_dir}")

    #  Connect to instruments
    print("\n[INIT] Connecting to instruments...")
    rm = pyvisa.ResourceManager()

    try:
        psu     = Instrument(rm, VISA_PSU,   "Keithley 2230 PSU")
        load    = Instrument(rm, VISA_LOAD,  "Keithley 2380 Load")
        scope   = Instrument(rm, VISA_SCOPE, "Keysight DSOX6004A")
        dmm     = Instrument(rm, VISA_DMM,   "Keithley DMM6500")
    except ConnectionError as e:
        print(f"\n[ERROR] {e}")
        print("Run list_visa_resources() to confirm VISA addresses.")
        return

    # Initialise instruments
    print("\n[INIT] Configuring instruments...")
    vin_actual = setup_psu(psu)
    setup_load(load, current=0.0)
    setup_dmm(dmm)

    # Configure all four scope channels
    for rail_name, cfg in RAILS.items():
        setup_scope_channel(scope, cfg["scope_ch"], cfg["vnom"])

    # Setup trigger on CH1 (3V6 rail — first in sequence)
    setup_scope_trigger(scope, trig_channel=1)

    # Initialise CSV
    csv_file = init_csv(output_dir, BOARD_SN, timestamp)

    # Power on PDN and verify steady state 
    print("\n[TEST] PDN powered on. Checking steady-state voltages...")
    time.sleep(3)   # allow rails to settle after PSU enable

    for rail_name, cfg in RAILS.items():
        vss = measure_dc_voltage(dmm)  # In a real setup: connect DMM to each rail
        print(f"  {rail_name}: {vss:.4f}V  (nominal {cfg['vnom']}V)")

    # Main Test Loop
    results_by_rail = {}

    for rail_name, cfg in RAILS.items():
        vnom    = cfg["vnom"]
        imax    = cfg["imax"]
        i_low   = imax * (LOAD_LOW_PCT  / 100.0)
        i_high  = imax * (LOAD_HIGH_PCT / 100.0)
        ch      = cfg["scope_ch"]

        print(f"\n{'='*60}")
        print(f"[RAIL] Testing {rail_name}  "
              f"(Vnom={vnom}V, Imax={imax}A, Step: {i_low:.2f}A→{i_high:.2f}A)")
        print(f"{'='*60}")

        # Update trigger to this rail's channel
        setup_scope_trigger(scope, trig_channel=ch)

        captures = []

        for cap_num in range(1, NUM_CAPTURES + 1):
            print(f"\n  Capture {cap_num}/{NUM_CAPTURES}:")

            # Set load to low state before capture
            set_load_current(load, i_low)
            time.sleep(STEP_DWELL_S)

            # Capture rising transient (low → high step)
            try:
                time_s, voltage_V = capture_transient_waveform(
                    scope, rail_name, cfg, cap_num, output_dir
                )
            except Exception as e:
                print(f"  [ERROR] Waveform capture failed: {e}")
                continue

            # Analyse waveform
            results = analyse_transient(time_s, voltage_V, vnom, cfg["tol_pct"])
            pf      = evaluate_pass_fail(results, vnom)

            # Store per-capture result (merge results + pf for report)
            cap_record = {**results, **pf}
            captures.append(cap_record)

            # Console output
            print(f"  Overshoot  : {results['overshoot_mv']:>8.2f} mV  → {pf['overshoot']}")
            print(f"  Undershoot : {results['undershoot_mv']:>8.2f} mV  → {pf['undershoot']}")
            print(f"  Recovery   : {results['recovery_time_us']:>8.2f} us  → {pf['recovery']}")
            print(f"  Settling   : {results['settling_time_us']:>8.2f} us  → {pf['settling']}")
            print(f"  ► Capture result: {pf['overall']}")

            # Log to CSV
            log_csv(csv_file, BOARD_SN, rail_name, cap_num, results, pf)

            # Dwell at high load before next capture
            set_load_current(load, i_high)
            time.sleep(STEP_DWELL_S)

        results_by_rail[rail_name] = {"captures": captures}
        # Turn off load between rails
        load.write("INP OFF")
        time.sleep(1)

    # Generate Report
    report_file, board_result = generate_report(
        BOARD_SN, OPERATOR, results_by_rail, output_dir, timestamp
    )

    # Shutdown 
    print("\n[SHUTDOWN] Powering down instruments...")
    load.write("INP OFF")
    psu.write("OUTP OFF")

    for inst in [psu, load, scope, dmm]:
        inst.close()
    rm.close()

    print(f"\n{'='*60}")
    print(f"  BOARD {BOARD_SN} — FINAL RESULT: {board_result}")
    print(f"  CSV    → {csv_file}")
    print(f"  Report → {report_file}")
    print(f"{'='*60}\n")


# ENTRY POINT


if __name__ == "__main__":
    # Uncomment the line below to list VISA resources before first run:
    list_visa_resources()

    run_transient_test()
