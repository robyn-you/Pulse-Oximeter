"""
Pico (Arduino C++ firmware) --USB serial--> this script --> pyqtgraph window + CSV log
(serial format unchanged; CSV records Quality + BPM stability diagnostics)
Install:  pip install pyserial pyqtgraph numpy PyQt5
"""
import sys, csv, time
from collections import deque
from datetime import datetime
from pathlib import Path

import numpy as np
import serial
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets

PORT = "COM5"
BAUD = 115200
FS = 25.0                 # firmware prints every 4th sample of 100 Hz -> ~25 Hz
WINDOW_S = 10             # seconds shown in the plots
N = int(WINDOW_S * FS)
OLED_POINTS = 42          # ~1.7 s, similar to the OLED waveform

COLUMNS = ["TYPE", "Time_ms", "RED_raw", "IR_raw", "PPG_filtered", "BPM",
           "SpO2_est", "RED_DC", "IR_DC", "RED_AC", "IR_AC", "Ratio_R"]


from ppg_validation import (parse_data, CSV_FIELDS, number as num, features,
                            rule_quality, csv_value, stability_assessment,
                            BPM_RANGE, HOLD_S, BASELINE_WINDOW_S, QUALITY_WIN_S, bpm_ok, optical_validity, R_WINDOW_S)

# Quality algorithm identical to standalone recorder and offline analyser.
# Deliberately RULE-BASED, not an implicitly-loaded ML model.
MODEL = None

# ---------------------------------------------------------------- GUI
def make_plot(title, ylabel):
    p = pg.PlotWidget(title=title)
    p.setLabel("left", ylabel)
    p.setLabel("bottom", "time (s)")
    p.showGrid(x=True, y=True, alpha=0.3)
    return p


def big_label(text, size=34):
    lab = QtWidgets.QLabel(text)
    lab.setAlignment(QtCore.Qt.AlignCenter)
    lab.setStyleSheet(f"font-size:{size}px; font-weight:bold; border:1px solid gray;")
    return lab


def set_two_line(label, main, colour, sub):
    """Big coloured main text with a small grey note underneath."""
    label.setText(
        f"<div style='font-size:28px; font-weight:bold; color:{colour};'>{main}</div>"
        f"<div style='font-size:13px; font-weight:normal; color:#555;'>{sub}</div>")
    label.setStyleSheet("border:1px solid gray;")


class Dashboard(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Signal Success - PPG Dashboard")
        self.resize(1250, 700)

        # ---- layout: 3 columns
        g = QtWidgets.QGridLayout(self)
        self.p_dc = make_plot("DC level (baseline)", "ADC counts")
        self.p_ac = make_plot("AC amplitude (envelope)", "ADC counts")
        self.p_ppg = make_plot("PPG signal (IR, filtered)", "a.u.")
        self.p_pulse = make_plot("Pulse waveform (same as OLED)", "a.u.")
        self.lbl_bpm = big_label("BPM: --")
        self.lbl_spo2 = big_label("SpO2: -- %")
        self.lbl_quality = big_label("Quality: --", 28)
        self.lbl_class = big_label("BPM stability: --", 28)
        self.lbl_optical = QtWidgets.QLabel("Optical validity: collecting data")
        self.lbl_optical.setAlignment(QtCore.Qt.AlignCenter)
        self.lbl_optical.setStyleSheet("font-size:17px;font-weight:bold;border:1px solid gray;padding:5px;")

        # Rows 0-1: plots.  Rows 2-3: BPM / SpO2 (one row each, equal height);
        # Signal quality and Classifier each span rows 2-3, so their height
        # equals BPM + SpO2 combined (matches the sketch).
        g.addWidget(self.p_dc, 0, 0)
        g.addWidget(self.p_ac, 1, 0)
        g.addWidget(self.lbl_bpm, 2, 0)
        g.addWidget(self.lbl_spo2, 3, 0)
        g.addWidget(self.p_ppg, 0, 1, 2, 1)
        g.addWidget(self.lbl_quality, 2, 1, 2, 1)
        g.addWidget(self.p_pulse, 0, 2, 2, 1)
        g.addWidget(self.lbl_class, 2, 2, 2, 1)
        g.addWidget(self.lbl_optical, 4, 0, 1, 3)
        for c in range(3):
            g.setColumnStretch(c, 1)
        # Ignore the widgets' own size hints vertically so the stretch
        # factors below decide the heights exactly.
        for w in (self.p_dc, self.p_ac, self.p_ppg, self.p_pulse,
                  self.lbl_bpm, self.lbl_spo2, self.lbl_quality, self.lbl_class):
            w.setSizePolicy(QtWidgets.QSizePolicy.Expanding,
                            QtWidgets.QSizePolicy.Ignored)
        # Plot rows 3 : 3.  BPM and SpO2 rows 1 : 1 (each ~ the middle of the
        # old BPM and SpO2 heights).  Change BOX_ROW to make the bottom taller/shorter.
        PLOT_ROW, BOX_ROW = 3, 1.5
        g.setRowStretch(0, int(PLOT_ROW * 2))
        g.setRowStretch(1, int(PLOT_ROW * 2))
        g.setRowStretch(2, int(BOX_ROW * 2))
        g.setRowStretch(3, int(BOX_ROW * 2))

        self.c_red_dc = self.p_dc.plot(pen="r", name="RED")
        self.c_ir_dc = self.p_dc.plot(pen="c", name="IR")
        self.c_red_ac = self.p_ac.plot(pen="r")
        self.c_ir_ac = self.p_ac.plot(pen="c")
        self.c_ppg = self.p_ppg.plot(pen=pg.mkPen("y", width=2))
        self.c_pulse = self.p_pulse.plot(pen=pg.mkPen("g", width=2))

        # ---- data buffers
        keys = ["t", "ppg", "red_dc", "ir_dc", "red_ac", "ir_ac"]
        self.buf = {k: deque(maxlen=N) for k in keys}
        self.finger_present = np.nan
        self.bpm_ready = np.nan
        self.spo2_ready = np.nan
        self.bpm = np.nan
        self.spo2 = np.nan
        self.last_feature_values = None
        self.quality = "NO SIGNAL"
        self.bpm_stability = "INSUFFICIENT DATA"
        self.bpm_note = "collecting data"
        self.bpm_baseline = np.nan
        self.bpm_deviation = np.nan
        self.bpm_valid_fraction = np.nan
        self.deviation_since = None
        self.hist = deque()                   # (time_s, RAW firmware BPM or NaN)
        self.feature_hist = deque()           # (time_s, PPG, BPM)
        self.hold_bpm = None                  # (value, time_s) of the last valid BPM
        self.hold_spo2 = None                 # (value, time_s) of the last SpO2
        self.t0 = None
        self.r_history = deque()
        self.optical_validity = 'INSUFFICIENT DATA'
        self.r_median = self.r_iqr = np.nan

        # ---- CSV logging (original sensor fields + Quality and BPM stability)
        folder = Path(__file__).resolve().parent / "PPG_recordings"
        folder.mkdir(exist_ok=True)
        self.path = folder / f"PPG_DASH_{datetime.now():%Y%m%d_%H%M%S}.csv"
        self.f = open(self.path, "w", newline="", encoding="utf-8")
        self.w = csv.DictWriter(self.f, fieldnames=CSV_FIELDS)
        self.w.writeheader()
        self.f.flush()
        print("[CSV] Saving to:", self.path.resolve(), flush=True)
        self.rows = 0
        self.serial_lines = 0
        self.rejected_lines = 0
        self.first_serial_at = time.monotonic()
        self.last_notice_at = self.first_serial_at

        # ---- serial
        self.rx = b""
        self.ser = serial.Serial(PORT, BAUD, timeout=0)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.poll)
        self.timer.start(40)

    # -------------------------------------------------- serial -> buffers
    def poll(self):
        chunk = self.ser.read(4096)
        if chunk:
            self.rx += chunk
        while b"\n" in self.rx:
            line, self.rx = self.rx.split(b"\n", 1)
            self.handle(line.decode("utf-8", "replace").strip())
        self.refresh()
        # Never silently create an empty CSV without explaining why.
        if self.rows == 0 and time.monotonic() - self.last_notice_at >= 5.0:
            print(f"[Serial] No valid DATA rows yet. Lines received: {self.serial_lines}; "
                  f"rejected DATA lines: {self.rejected_lines}. "
                  "Check firmware output, COM port and sensor acquisition.", flush=True)
            self.last_notice_at = time.monotonic()

    def handle(self, line):
        self.serial_lines += 1
        if not line.startswith("DATA,"):
            # Initial firmware headers and beat-event logs are not measurement rows.
            if self.serial_lines <= 3:
                print("[Serial]", line[:160], flush=True)
            return
        data = parse_data(line)
        if data is None:
            self.rejected_lines += 1
            if self.rejected_lines <= 5:
                print("[Serial] Rejected DATA:", line[:180], flush=True)
            return
        t_ms = int(data['Time_ms'])
        p = [data[k] for k in COLUMNS]
        if self.t0 is None:
            self.t0 = t_ms
        t = (t_ms - self.t0) / 1000.0
        v = [num(x) for x in p]
        b = self.buf
        b["t"].append(t)
        b["ppg"].append(-v[4])                # same sign as the OLED (-irLP)
        b["red_dc"].append(v[7]); b["ir_dc"].append(v[8])
        b["red_ac"].append(v[9]); b["ir_ac"].append(v[10])
        self.bpm, self.spo2 = v[5], v[6]
        self.finger_present = num(data['FingerPresent'])
        self.bpm_ready = num(data['BPMReady'])
        self.spo2_ready = num(data['SpO2Ready'])
        self.feature_hist.append((t, -v[4], self.bpm))
        while self.feature_hist and t - self.feature_hist[0][0] > 8.1:
            self.feature_hist.popleft()
        # raw values go to the CSV and to the decision window; the display holds them (HOLD_S)
        self.hist.append((t, self.bpm))
        while self.hist and t - self.hist[0][0] > BASELINE_WINDOW_S + 1:
            self.hist.popleft()
        if bpm_ok(self.bpm):
            self.hold_bpm = (self.bpm, t)
        if not np.isnan(self.spo2):
            self.hold_spo2 = (self.spo2, t)
        self.update_assessments(t)
        self.r_history.append((t, num(data['Ratio_R'])))
        while self.r_history and t - self.r_history[0][0] > R_WINDOW_S + 0.1:
            self.r_history.popleft()
        active = (bool(self.finger_present) if np.isfinite(self.finger_present)
                  else np.isfinite(v[9]) and np.isfinite(v[10]))
        (self.optical_validity, self.r_median, self.r_iqr) = optical_validity(
            self.r_history, t, active, self.quality)
        feats = self.last_feature_values
        values = dict(data, Time_s=round(t, 3), Label='')
        values.update(feats)
        values.update(Quality=self.quality, BPM_Stability=self.bpm_stability,
                      BPM_Baseline=self.bpm_baseline, BPM_Deviation=self.bpm_deviation,
                      BPM_ValidFraction=self.bpm_valid_fraction,
                      OpticalValidity=self.optical_validity,
                      Ratio_R_Median5s=self.r_median, Ratio_R_IQR5s=self.r_iqr)
        self.w.writerow({k: csv_value(values.get(k, '')) for k in CSV_FIELDS})
        self.rows += 1
        if self.rows == 1:
            print("[Serial] First valid DATA received; recording raw RED/IR even without a finger.", flush=True)
        if self.rows % 25 == 0:
            self.f.flush()

    def update_assessments(self, now):
        a = self.buf
        ir_ac = a['ir_ac'][-1]
        ir_dc = a['ir_dc'][-1]
        # Prefer firmware's explicit finger decision. Old 12-field output
        # provides only optical proxy, not independent finger ground truth.
        active = (bool(self.finger_present) if np.isfinite(self.finger_present)
                  else np.isfinite(ir_ac) and np.isfinite(ir_dc))
        self.last_feature_values = features(self.feature_hist, now, ir_ac, ir_dc)
        self.quality = rule_quality(self.last_feature_values, active)
        if not active:
            self.bpm_stability, self.bpm_note = 'NO SIGNAL', 'optical data unavailable'
            self.bpm_baseline = self.bpm_deviation = self.bpm_valid_fraction = np.nan
            self.deviation_since = None
            self.hist.clear()
            self.hold_bpm = self.hold_spo2 = None
            return
        (self.bpm_stability, self.bpm_note, self.bpm_baseline,
         self.bpm_deviation, self.bpm_valid_fraction,
         self.deviation_since) = stability_assessment(
             list(self.hist), now, self.quality, self.deviation_since)

    # -------------------------------------------------- redraw
    def refresh(self):
        b = self.buf
        if not b["t"]:
            return
        t = np.array(b["t"])
        x = t - t[-1]                          # scrolling window, 0 = now
        a = {k: np.array(b[k]) for k in b if k != "t"}

        self.c_red_dc.setData(x, a["red_dc"], connect="finite")
        self.c_ir_dc.setData(x, a["ir_dc"], connect="finite")
        self.c_red_ac.setData(x, a["red_ac"], connect="finite")
        self.c_ir_ac.setData(x, a["ir_ac"], connect="finite")
        self.c_ppg.setData(x, a["ppg"], connect="finite")
        self.c_pulse.setData(x[-OLED_POINTS:], a["ppg"][-OLED_POINTS:], connect="finite")

        # displayed BPM / SpO2: hold the last value for HOLD_S so short dropouts do not blank the box
        now = t[-1]
        bpm_show = self.hold_bpm[0] if self.hold_bpm and now - self.hold_bpm[1] <= HOLD_S else np.nan
        spo2_show = self.hold_spo2[0] if self.hold_spo2 and now - self.hold_spo2[1] <= HOLD_S else np.nan
        self.lbl_bpm.setText("BPM: --" if np.isnan(bpm_show) else f"BPM: {bpm_show:.0f}")
        self.lbl_spo2.setText("SpO2: -- %" if np.isnan(spo2_show) else f"SpO2: {spo2_show:.0f} %")

        # Signal quality: same original classifier and feature definitions.
        feats = getattr(self, "last_feature_values", None)
        method = "rule-based"
        if feats is not None and np.isfinite(feats['Amp_5s']):
            sub = (f"{method}: amp={feats['Amp_5s']:.0f}, PI={feats['PI_pct']:.2f}%, "
                   f"rough={feats['Roughness_5s']:.2f}")
        else:
            sub = method
        qcol = {"GOOD": "green", "POOR": "red"}.get(self.quality, "gray")
        set_two_line(self.lbl_quality, f"Signal quality: {self.quality}", qcol, sub)

        if np.isfinite(self.bpm_baseline):
            detail = (f"baseline {self.bpm_baseline:.0f} BPM"
                      + (f", deviation {self.bpm_deviation:.0f} BPM" if np.isfinite(self.bpm_deviation) else "")
                      + " | " + self.bpm_note)
        else:
            detail = self.bpm_note
        colour = {"STABLE": "green", "UNSTABLE": "red", "CHECK CHANGE": "orange",
                  "CHECK SIGNAL": "orange"}.get(self.bpm_stability, "gray")
        set_two_line(self.lbl_class, f"BPM stability: {self.bpm_stability}", colour, detail)
        note = (f" | median R={self.r_median:.2f}, IQR={self.r_iqr:.2f}"
                if np.isfinite(self.r_median) else "")
        shade = '#b91c1c' if self.optical_validity == 'OPTICAL ARTIFACT SUSPECTED' else '#0f766e'
        self.lbl_optical.setText(f"Optical validity: {self.optical_validity}{note}")
        self.lbl_optical.setStyleSheet(
            f"font-size:17px;font-weight:bold;color:{shade};border:1px solid gray;padding:5px;")

    def closeEvent(self, e):
        self.timer.stop()
        self.f.close()
        self.ser.close()
        print("Saved:", self.path.resolve())
        e.accept()


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    win = Dashboard()
    win.show()
    sys.exit(app.exec_())


