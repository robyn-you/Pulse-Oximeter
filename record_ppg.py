"""Standalone labelled Pico recording. Do not run alongside dashboard (same COM port).
Usage: python record_ppg.py --port COM5 --label STABLE_FINGER
        python record_ppg.py --port COM5 --label OBJECT --seconds 60
"""
import argparse,csv,time
from pathlib import Path
from datetime import datetime
from collections import deque
import numpy as np
from ppg_validation import parse_data,CSV_FIELDS,LABELS,features,rule_quality,csv_value,number,stability_assessment,BASELINE_WINDOW_S,canonical_label,optical_validity,R_WINDOW_S

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--port',default='COM5')
    ap.add_argument('--baud',type=int,default=115200)
    ap.add_argument('--label',choices=LABELS,default='UNLABELED')
    ap.add_argument('--seconds',type=float,default=60.)
    args=ap.parse_args()
    import serial
    folder=Path('PPG_recordings')/args.label
    folder.mkdir(parents=True,exist_ok=True)
    path=folder/f'PPG_{datetime.now():%Y%m%d_%H%M%S}.csv'
    history=deque(); r_history=deque(); bpm_history=deque(); deviation_since=None; baseline=None; saved=skipped=0
    try:
        with serial.Serial(args.port,args.baud,timeout=1) as port, path.open('w',newline='',encoding='utf-8') as f:
            writer=csv.DictWriter(f,fieldnames=CSV_FIELDS);writer.writeheader()
            print('Recording',args.label,'on',args.port,'to',path,'(Ctrl+C to stop)')
            until=time.monotonic()+args.seconds if args.seconds>0 else float('inf')
            while time.monotonic()<until:
                data=parse_data(port.readline().decode('utf-8','replace'))
                if data is None:
                    skipped+=1;continue
                ms=int(data['Time_ms'])
                if baseline is None:baseline=ms
                t=(ms-baseline)/1000
                ppg=-number(data['PPG_filtered']);bpm=number(data['BPM'])
                history.append((t,ppg,bpm))
                bpm_history.append((t,bpm))
                while bpm_history and t-bpm_history[0][0]>BASELINE_WINDOW_S+1:bpm_history.popleft()
                while history and t-history[0][0]>8.1:history.popleft()
                m=features(history,t,number(data['IR_AC']),number(data['IR_DC']))
                active = (data['FingerPresent']=='1' if data['FingerPresent'] in ('0','1')
                          else np.isfinite(number(data['IR_AC'])) and np.isfinite(number(data['IR_DC'])))
                quality=rule_quality(m,active)
                r_history.append((t,number(data['Ratio_R'])))
                while r_history and t-r_history[0][0]>R_WINDOW_S+0.1:r_history.popleft()
                status,rmed,riqr=optical_validity(r_history,t,active,quality)
                data.update(m);data.update(Time_s=round(t,3),Label=args.label,
                        Quality=quality,OpticalValidity=status,
                        Ratio_R_Median5s=rmed,Ratio_R_IQR5s=riqr)
                writer.writerow({k:csv_value(data.get(k,'')) for k in CSV_FIELDS})
                saved+=1
                if saved%100==0:
                    f.flush();print(f'rows={saved}, sensor-time={t:.1f}s, last-quality={data["Quality"]}',flush=True)
    except KeyboardInterrupt:print('Stopped by user')
    except Exception as exc:print('Serial/recording error:',exc)
    print('Saved:',path,'rows:',saved,'non-DATA lines:',skipped)
if __name__=='__main__':main()
