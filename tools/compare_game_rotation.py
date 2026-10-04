#!/usr/bin/env python3
"""6軸姿勢推定とGame Rotation Vectorを車体座標で比較する。
入力: 診断セッションディレクトリ
出力: attitude_game_rotation_comparison.csv、attitude_game_rotation_summary.txt
"""
from __future__ import annotations
import argparse, csv, math, struct
from pathlib import Path

HEADER_SIZE = 16
RECORD = struct.Struct('<qffffffi')
MAX_TIME_DIFF_NS = 10_000_000

# 画面上向き、端末上側が機体前方。車体から端末への回転はZ軸周り+90度。
Q_DEVICE_FROM_BODY = (math.cos(math.pi / 4.0), 0.0, 0.0, math.sin(math.pi / 4.0))

def normalize(q):
    n=math.sqrt(sum(v*v for v in q))
    if n < 1e-12: return (1.0,0.0,0.0,0.0)
    return tuple(v/n for v in q)

def multiply(a,b):
    aw,ax,ay,az=a; bw,bx,by,bz=b
    return (aw*bw-ax*bx-ay*by-az*bz,
            aw*bx+ax*bw+ay*bz-az*by,
            aw*by-ax*bz+ay*bw+az*bx,
            aw*bz+ax*by-ay*bx+az*bw)

def euler(q):
    w,x,y,z=normalize(q)
    roll=math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))
    pitch=math.asin(max(-1.0,min(1.0,2*(w*y-z*x))))
    yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
    return roll,pitch,yaw

def read_rotation(path):
    data=path.read_bytes()
    if len(data) < HEADER_SIZE: raise ValueError('Game Rotation Vectorログのヘッダーが不足しています。')
    record_size=struct.unpack_from('<i',data,8)[0]
    if record_size < RECORD.size: raise ValueError(f'不正なレコードサイズです: {record_size}')
    rows=[]
    for off in range(HEADER_SIZE,len(data)-record_size+1,record_size):
        t,x,y,z,_,_,_,_=RECORD.unpack_from(data,off)
        # Android Rotation Vectorはx,y,zと、利用可能な場合はスカラー成分をvalues[3]へ格納する。
        # 現行BinarySensorWriterは3成分だけを保存するため、単位長条件からw>=0を復元する。
        q_world_device=normalize((math.sqrt(max(0.0,1.0-x*x-y*y-z*z)),x,y,z))
        q_world_body=normalize(multiply(q_world_device,Q_DEVICE_FROM_BODY))
        rows.append((t,q_world_body,*euler(q_world_body)))
    return rows

def angle_diff(a,b): return math.atan2(math.sin(a-b),math.cos(a-b))

def metrics(values):
    deg=[math.degrees(v) for v in values]
    return sum(deg)/len(deg),math.sqrt(sum(v*v for v in deg)/len(deg)),max(abs(v) for v in deg)

def main():
    ap=argparse.ArgumentParser(description='6軸姿勢とGame Rotation Vectorを車体座標で比較します。')
    ap.add_argument('session_dir',type=Path)
    ap.add_argument('--max-time-diff-ms',type=float,default=10.0)
    args=ap.parse_args(); d=args.session_dir; tolerance=int(args.max_time_diff_ms*1_000_000)
    gr=read_rotation(d/'game_rotation_vector.bin')
    with (d/'vehicle_attitude.csv').open(newline='',encoding='utf-8') as f: att=list(csv.DictReader(f))
    out=[]; j=0
    for r in att:
        t=int(r['elapsed_realtime_ns'])
        while j+1<len(gr) and abs(gr[j+1][0]-t)<=abs(gr[j][0]-t): j+=1
        if not gr or abs(gr[j][0]-t)>tolerance: continue
        gt,gq,rr,rp,ry=gr[j]
        er=angle_diff(float(r['roll_rad']),rr); ep=angle_diff(float(r['pitch_rad']),rp)
        out.append((t,gt,gt-t,float(r['roll_rad']),rr,er,float(r['pitch_rad']),rp,ep,float(r['yaw_rad']),ry,float(r.get('attitude_confidence',0.0))))
    if not out: raise SystemExit('比較可能な同時刻データがありません。')
    with (d/'attitude_game_rotation_comparison.csv').open('w',newline='',encoding='utf-8') as f:
        w=csv.writer(f); w.writerow(['elapsed_realtime_ns','game_rotation_ns','time_diff_ns','estimated_roll_rad','game_body_roll_rad','roll_error_rad','estimated_pitch_rad','game_body_pitch_rad','pitch_error_rad','estimated_yaw_rad','game_body_yaw_rad','attitude_confidence']); w.writerows(out)
    rb,rr,rm=metrics([r[5] for r in out]); pb,pr,pm=metrics([r[8] for r in out])
    td=[abs(r[2])/1e6 for r in out]
    text=(f'比較件数: {len(out)}\n平均時刻差[ms]: {sum(td)/len(td):.6f}\n最大時刻差[ms]: {max(td):.6f}\n'
          f'ロール平均差[deg]: {rb:.6f}\nロールRMSE[deg]: {rr:.6f}\nロール最大絶対差[deg]: {rm:.6f}\n'
          f'ピッチ平均差[deg]: {pb:.6f}\nピッチRMSE[deg]: {pr:.6f}\nピッチ最大絶対差[deg]: {pm:.6f}\n')
    (d/'attitude_game_rotation_summary.txt').write_text(text,encoding='utf-8'); print(text,end='')
if __name__=='__main__': main()
