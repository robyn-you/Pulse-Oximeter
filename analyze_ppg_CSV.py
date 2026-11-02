"""Offline feature analysis, independent labels, quality threshold grid search.
Run after recording: python analyze_ppg_CSV.py
Requires: pip install numpy pandas matplotlib openpyxl
"""
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from ppg_validation import features,number,canonical_label

INPUT=Path('PPG_recordings'); OUTPUT=Path('PPG_analysis')
IGNORE_FIRST_SECONDS=10.0
STEP_S=5.0  # Non-overlapping 5 s windows avoid artificially inflated sample counts
GOOD={'STABLE_FINGER'}
BAD={'MOVING_FINGER','OBJECT','NO_FINGER','POOR_CONTACT'}
NUM=['Time_s','IR_raw','RED_raw','PPG_filtered','BPM','SpO2_est','IR_AC','IR_DC','FingerPresent',
     'Amp_5s','PI_pct','Roughness_5s','PPG_SD_5s','BPM_std_8s','BPM_valid_fraction_5s']

def calc_windows(df):
    if 'Time_s' not in df or 'PPG_filtered' not in df:return []
    d=df.dropna(subset=['Time_s']).sort_values('Time_s').copy()
    if d.empty:return []
    start=float(d['Time_s'].min())+IGNORE_FIRST_SECONDS
    stop=float(d['Time_s'].max())
    out=[]
    for begin in np.arange(start,stop-STEP_S+1e-6,STEP_S):
        w=d[(d.Time_s>=begin)&(d.Time_s<begin+STEP_S)]
        if len(w)<20 or w.Time_s.max()-w.Time_s.min()<2.:continue
        x=w['PPG_filtered'].dropna().to_numpy(dtype=float)
        if len(x)<20:continue
        ir_ac=w['IR_AC'].median() if 'IR_AC' in w else np.nan
        ir_dc=w['IR_DC'].median() if 'IR_DC' in w else np.nan
        history=d[(d.Time_s>=begin-3.0)&(d.Time_s<begin+STEP_S)]
        m=features([(float(r.Time_s), -number(r.PPG_filtered), number(r.BPM) if 'BPM' in history else np.nan)
                    for r in history.itertuples(index=False)],float(w.Time_s.max()),ir_ac,ir_dc)
        m['window_start_s']=round(begin,2)
        m['bpm_median']=float(w.BPM.median()) if 'BPM' in w else np.nan
        m['bpm_sd']=float(w.BPM.std()) if 'BPM' in w else np.nan
        m['spo2_median']=float(w.SpO2_est.median()) if 'SpO2_est' in w else np.nan
        m['finger_present_pct']=float(w.FingerPresent.mean()*100) if 'FingerPresent' in w else np.nan
        out.append(m)
    return out

def tune(windows):
    training=windows[windows.Label.isin(GOOD|BAD)].dropna(subset=['Amp_5s','PI_pct','Roughness_5s']).copy()
    if training.Label.isin(GOOD).sum()==0 or training.Label.isin(BAD).sum()==0:
        print('No threshold tuning: need both STABLE_FINGER and negative labels.');return None
    # Per-file training metrics are exploratory; select final thresholds using separate recordings.
    results=[]
    actual=training.Label.isin(GOOD).to_numpy()
    for a in [20,40,80,150,300,500,1000]:
        for p in [.01,.03,.05,.10,.15,.20,.30]:
            for r in [.3,.4,.5,.6,.7,.8,1.0]:
                predicted=((training.Amp_5s>a)&(training.PI_pct>p)&(training.Roughness_5s<r)).to_numpy()
                tp=int((predicted&actual).sum()); fn=int((~predicted&actual).sum())
                tn=int((~predicted&~actual).sum());fp=int((predicted&~actual).sum())
                sens=tp/(tp+fn) if tp+fn else np.nan
                spec=tn/(tn+fp) if tn+fp else np.nan
                results.append(dict(amp_threshold=a,pi_threshold=p,rough_threshold=r,
                                    sensitivity=sens,specificity=spec,balanced_accuracy=(sens+spec)/2,
                                    TP=tp,FN=fn,TN=tn,FP=fp))
    grid=pd.DataFrame(results).sort_values(['balanced_accuracy','sensitivity'],ascending=False)
    grid.to_csv(OUTPUT/'threshold_candidates.csv',index=False)
    print('Exploratory top thresholds (NOT independently validated):')
    print(grid.head(5).to_string(index=False))
    return grid

def main():
    OUTPUT.mkdir(exist_ok=True)
    files=sorted(set([*INPUT.rglob('*.csv'),*INPUT.rglob('*.xlsx')])) if INPUT.exists() else []
    summaries=[]; windows=[]
    for file in files:
        try:
            df=pd.read_csv(file) if file.suffix=='.csv' else pd.read_excel(file)
            for col in NUM:
                if col in df:df[col]=pd.to_numeric(df[col],errors='coerce')
            if 'Time_s' not in df or 'IR_raw' not in df:raise ValueError('missing Time_s/IR_raw')
            label=(df.loc[df['Label'].fillna('').astype(str).str.strip().ne(''), 'Label'].astype(str).str.strip().mode().iloc[0]
                   if 'Label' in df and df['Label'].notna().any() and df['Label'].astype(str).str.strip().ne('').any() else file.parent.name.upper())
            label=canonical_label(label)
            if label not in GOOD|BAD:label='UNLABELED'
            d=df[df.Time_s >= df.Time_s.min()+IGNORE_FIRST_SECONDS]
            summary=dict(file=str(file),label=label,rows=len(df),steady_rows=len(d),
                         duration_s=float(df.Time_s.max()-df.Time_s.min()),
                         ir_median=float(d.IR_raw.median()),
                         bpm_mean=float(d.BPM.mean()) if 'BPM' in d else np.nan,
                         bpm_std=float(d.BPM.std()) if 'BPM' in d else np.nan,
                         spo2_mean=float(d.SpO2_est.mean()) if 'SpO2_est' in d else np.nan,
                         quality_good_pct=float(d.Quality.eq('GOOD').mean()*100) if 'Quality' in d else np.nan)
            summaries.append(summary)
            # Mixed labels in a single dashboard recording must not be collapsed into one class.
            if 'Label' in df and df['Label'].fillna('').astype(str).str.strip().ne('').any():
                for lab, segment in df.groupby((df.Label!=df.Label.shift()).cumsum()):
                    actual_label=canonical_label(str(segment.Label.iloc[0]).strip())
                    for m in calc_windows(segment):windows.append(dict(file=str(file),Label=actual_label,**m))
            else:
                for m in calc_windows(df):windows.append(dict(file=str(file),Label=label,**m))
            print('Processed',file,'rows',len(df))
        except Exception as exc:print('Skipped',file,exc)
    if not summaries:print('No compatible recordings.');return
    sf=pd.DataFrame(summaries);wf=pd.DataFrame(windows)
    sf.to_csv(OUTPUT/'summary_per_file.csv',index=False)
    sf.groupby('label').mean(numeric_only=True).to_csv(OUTPUT/'summary_by_label.csv')
    if wf.empty:print('No complete windows available');return
    wf.to_csv(OUTPUT/'features_by_5s_window.csv',index=False)
    stats=wf.groupby('Label')[['Amp_5s','PI_pct','Roughness_5s','BPM_std_8s','BPM_valid_fraction_5s']].agg(['count','median','std',lambda x:x.quantile(.05),lambda x:x.quantile(.95)])
    stats.to_csv(OUTPUT/'feature_distributions.csv')
    for col in ['Amp_5s','PI_pct','Roughness_5s','BPM_std_8s']:
        plt.figure(figsize=(8,4))
        groups=[g[col].dropna().to_numpy() for _,g in wf.groupby('Label')]
        names=[key for key,_ in wf.groupby('Label')]
        keep=[(n,v) for n,v in zip(names,groups) if len(v)]
        if keep:
            plt.boxplot([v for _,v in keep],tick_labels=[n for n,_ in keep])
            plt.ylabel(col);plt.xticks(rotation=20,ha='right');plt.tight_layout()
            plt.savefig(OUTPUT/f'{col}_by_label.png',dpi=150)
        plt.close()
    tune(wf)
    print('Results saved in:',OUTPUT.resolve())
if __name__=='__main__':main()
