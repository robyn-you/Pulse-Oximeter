# Unified Pico PPG tools

Place all four `.py` files in `C:\UTS master\oximeter_functions`. Install dependencies with `python -m pip install numpy pandas matplotlib openpyxl pyserial pyqtgraph PyQt5`.

**Live view and recording:** `python dashboard_ppg.py` (COM5). When finished, move each CSV into a condition folder in `PPG_recordings`, e.g. `finger_on`, `finger_wiggling`, `Nothing on`, or `object on`. No renaming needed. The dashboard preserves its graphs, BPM/SpO2 displays, signal quality and BPM stability.

**Standalone recorder (alternative; do not run while dashboard owns COM5):** `python record_ppg.py --port COM5 --label STABLE_FINGER --seconds 60`. Unlike dashboard, this recorder chooses label at launch.

**Batch report:** `python analyze_ppg_CSV.py`. Results appear in `PPG_analysis`. Filename-based conditions are NOT required; recognized folder names map to canonical labels. Unrecognized folders remain UNLABELED and are excluded from quality-threshold optimisation, although they still contribute to a per-file summary.

**Serial format:** 12-field data accepted, or **15 fields from supplied Arduino firmware**: 12 original values + `FingerPresent`, `BPMReady`, `SpO2Ready`. Missing 12-field flags remain empty. `Quality` and `BPM_Stability` are computed using the same helpers by dashboard and recorder. `BPM_Stability` is an experimental consistency indicator, not a medical alarm. The 25 Hz USB output does not provide every sensor sample.

**Important:** The status flags do not determine the actual laboratory label; folder name or explicit recorder `--label` does. Firmware reset causes and clinical SpO2 accuracy still need separate diagnosis/reference measurements. Change nothing in Arduino firmware for this Python alignment.


## Added experimental optical-validity flag (2026-10-31)
Dashboard now displays **Optical validity** in an extra bottom strip, without replacing BPM stability. Both dashboard and recorder save `OpticalValidity`, `Ratio_R_Median5s`, and `Ratio_R_IQR5s` in the CSV. Uses the last five seconds (at least 40 valid values over at least three seconds). A provisional median R outside 0.20–0.90 or IQR >0.30 flags `OPTICAL ARTIFACT SUSPECTED`. Otherwise signal may be `R CONSISTENT (UNVERIFIED)` if quality GOOD, or `CHECK SIGNAL`. These thresholds are NOT clinically established and may flag genuine low-saturation or poor-contact fingers, or miss periodic objects. Do not claim finger identification or object rejection has been validated. Firmware is unchanged; raw BPM and SpO2 are not censored. Do not compare accuracy on the same trials used to choose these thresholds.
