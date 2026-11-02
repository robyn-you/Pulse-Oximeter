"""Shared serial schema and validation features. Engineering metrics, not medical decisions."""
import numpy as np

FIELDS12 = ['TYPE','Time_ms','RED_raw','IR_raw','PPG_filtered','BPM','SpO2_est','RED_DC','IR_DC','RED_AC','IR_AC','Ratio_R']
FIELDS15 = FIELDS12 + ['FingerPresent','BPMReady','SpO2Ready']
LABELS = ['UNLABELED','STABLE_FINGER','MOVING_FINGER','OBJECT','NO_FINGER','POOR_CONTACT']
EXTRAS = ['Label','FingerPresent','BPMReady','SpO2Ready','Amp_5s','PI_pct','PPG_SD_5s','Roughness_5s','BPM_std_8s','BPM_valid_fraction_5s','Quality','BPM_Stability','BPM_Baseline','BPM_Deviation','BPM_ValidFraction','OpticalValidity','Ratio_R_Median5s','Ratio_R_IQR5s']
CSV_FIELDS = FIELDS12[:2] + ['Time_s'] + FIELDS12[2:] + EXTRAS
QUALITY_WIN_S = 5.0
BPM_STD_WIN_S = 8.0

def number(value):
    try:
        x = float(value)
        return x if np.isfinite(x) else np.nan
    except (ValueError,TypeError):
        return np.nan

def parse_data(line):
    parts = line.strip().split(',')
    if not parts or parts[0] != 'DATA' or len(parts) not in (12,15):
        return None
    try:
        ms = int(parts[1]); ir = float(parts[3]); red = float(parts[2])
        if ms < 0 or not np.isfinite(ir) or not np.isfinite(red): return None
    except (ValueError,TypeError):
        return None
    out = dict(zip(FIELDS12,parts[:12]))
    for key in FIELDS15[12:]:
        out[key] = parts[FIELDS15.index(key)] if len(parts)==15 else ''
    return out

def features(history, now, ir_ac, ir_dc):
    """history: iterable (time_s, ppg, bpm); time-based trailing windows."""
    recent = [(t,p,b) for t,p,b in history if now-QUALITY_WIN_S <= t <= now and np.isfinite(p)]
    x = np.asarray([p for _,p,_ in recent],dtype=float)
    amp = sd = rough = np.nan
    # Require >=2 s actual time coverage AND >=20 samples; avoid partial startup windows.
    if len(recent)>=20 and recent[-1][0]-recent[0][0]>=2.0:
        amp = float(np.ptp(x)); sd = float(np.std(x))
        rough = float(np.std(np.diff(x))/(sd+1e-9)) if len(x)>1 else np.nan
    pi = 100.0*ir_ac/ir_dc if np.isfinite(ir_ac) and np.isfinite(ir_dc) and ir_dc>0 else np.nan
    stdvals = np.asarray([b for t,_,b in history if now-BPM_STD_WIN_S<=t<=now and np.isfinite(b) and 40<=b<=180])
    bpm_std = float(np.std(stdvals)) if len(stdvals)>=25 else np.nan
    bpmwin = [b for t,_,b in history if now-QUALITY_WIN_S<=t<=now]
    bpmfrac = sum(np.isfinite(b) and 40<=b<=180 for b in bpmwin)/len(bpmwin) if bpmwin else np.nan
    return dict(Amp_5s=amp, PI_pct=pi, PPG_SD_5s=sd, Roughness_5s=rough,
                BPM_std_8s=bpm_std,BPM_valid_fraction_5s=bpmfrac)

def rule_quality(feat, active):
    if not active or not np.isfinite(feat['Amp_5s']): return 'NO SIGNAL'
    return 'GOOD' if (feat['Amp_5s']>40 and np.isfinite(feat['PI_pct']) and feat['PI_pct']>0.05
                      and np.isfinite(feat['Roughness_5s']) and feat['Roughness_5s']<0.6) else 'POOR'

def csv_value(value):
    if isinstance(value,(float,np.floating)) and not np.isfinite(value): return ''
    return value

# ---- BPM stability parameters and routine (same in both recorders) ----
BPM_RANGE = (40, 180)     # Firmware reporting capability, NOT a normal resting range.
SPO2_RANGE = (70, 100)    # Informational; does not reject raw CSV readings.
HOLD_S = 3.0              # UI only; held values are NOT substituted in CSV.
BASELINE_WINDOW_S = 20.0 # Rolling window of prior firmware BPM estimates.
BASELINE_GUARD_S = 2.0   # Exclude recent data to avoid instant baseline chasing.
BASELINE_MIN_SPAN_S = 10.0
BASELINE_MIN_FRACTION = 0.60  # Minimum coverage of valid values in baseline window.
DEVIATION_BPM = 20.0     # Provisional engineering anomaly threshold, NOT clinical.
DEVIATION_PERSIST_S = 2.0
QUALITY_WINDOW_S = 5.0



def bpm_ok(value):
    return np.isfinite(value) and BPM_RANGE[0] <= value <= BPM_RANGE[1]


def stability_assessment(history, now, quality, deviation_since):
    """Return state, note, baseline, deviation, valid_fraction, candidate_start.

    A rolling median describes stability of the *estimated BPM*, not whether a
    living person's pulse exists. Never changes the raw firmware BPM.
    """
    recent = [(t, b) for t, b in history
              if now - BASELINE_WINDOW_S <= t <= now - BASELINE_GUARD_S]
    if not recent:
        return 'INSUFFICIENT DATA', 'collecting BPM baseline', np.nan, np.nan, 0., None

    valid = [(t, b) for t, b in recent if bpm_ok(b)]
    fraction = len(valid) / len(recent)
    if (not valid or valid[-1][0] - valid[0][0] < BASELINE_MIN_SPAN_S
            or fraction < BASELINE_MIN_FRACTION):
        return 'INSUFFICIENT DATA', 'need 10 s of usable BPM history', np.nan, np.nan, fraction, None

    baseline = float(np.median([b for _, b in valid]))
    latest = history[-1][1]
    if not bpm_ok(latest):
        return 'CHECK SIGNAL', 'no current BPM estimate', baseline, np.nan, fraction, None
    deviation = abs(float(latest) - baseline)
    if quality != 'GOOD':
        return 'CHECK SIGNAL', 'PPG quality not GOOD; interpretation limited', baseline, deviation, fraction, None
    if deviation > DEVIATION_BPM:
        start = deviation_since if deviation_since is not None else now
        if now - start >= DEVIATION_PERSIST_S:
            return 'UNSTABLE', 'persistent BPM change; check signal or activity', baseline, deviation, fraction, start
        return 'CHECK CHANGE', 'large BPM deviation; observing persistence', baseline, deviation, fraction, start
    return 'STABLE', 'BPM close to recent baseline', baseline, deviation, fraction, None



# File-label aliases for your existing manually sorted folders.
LABEL_ALIASES = {
    'FINGER_ON': 'STABLE_FINGER', 'STABLE ON': 'STABLE_FINGER',
    'FINGER_WIGGLING': 'MOVING_FINGER', 'FLNGER_WIGGLING': 'MOVING_FINGER',
    'NOTHING ON': 'NO_FINGER', 'NOTHING': 'NO_FINGER',
    'OBJECT ON': 'OBJECT', 'RANDOM_OBJECT': 'OBJECT',
}
def canonical_label(name):
    raw = str(name).strip().upper()
    return LABEL_ALIASES.get(raw, raw)

# Experimental optical-ratio screening; NOT proof of a human finger.
# Wide exploratory limits based on preliminary finger/object recordings.
R_WINDOW_S = 5.0
R_MIN_SAMPLES = 40
R_MIN_SPAN_S = 3.0
R_PLAUSIBLE_MIN = 0.20
R_PLAUSIBLE_MAX = 0.90
R_MAX_IQR = 0.30

def optical_validity(r_history, now, optical_active, quality):
    """Return (status, rolling_median_R, IQR_R) without altering BPM or SpO2.

    A ratio outside this exploratory envelope is an optical warning, NOT
    a diagnosis of a non-human object. Ratios near zero with weak IR_AC
    can be numerically unstable. Use independent data before claiming accuracy.
    """
    if not optical_active:
        return 'NO SIGNAL', np.nan, np.nan
    window = [(t, r) for t, r in r_history
              if now - R_WINDOW_S <= t <= now and np.isfinite(r) and r >= 0]
    if len(window) < R_MIN_SAMPLES or window[-1][0] - window[0][0] < R_MIN_SPAN_S:
        return 'INSUFFICIENT DATA', np.nan, np.nan
    values = np.asarray([r for _, r in window], dtype=float)
    median = float(np.median(values))
    iqr = float(np.percentile(values, 75) - np.percentile(values, 25))
    if not (R_PLAUSIBLE_MIN <= median <= R_PLAUSIBLE_MAX) or iqr > R_MAX_IQR:
        return 'OPTICAL ARTIFACT SUSPECTED', median, iqr
    if quality != 'GOOD':
        return 'CHECK SIGNAL', median, iqr
    return 'R CONSISTENT (UNVERIFIED)', median, iqr
