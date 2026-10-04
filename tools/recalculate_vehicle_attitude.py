#!/usr/bin/env python3
"""保存済みの未校正IMUログから6軸車体姿勢を再計算する。
入力: session.json、accel.bin、gyro.bin
出力: offline_vehicle_attitude.csv、offline_vehicle_attitude_summary.txt
"""
from __future__ import annotations
import argparse, csv, json, math, struct
from pathlib import Path

HEADER_SIZE=16
RECORD=struct.Struct('<qffffffi')
G=9.80665
MAX_G_ERR=3.0
CORRECTION_RATE=1.5
MAX_RESIDUAL=math.radians(30.0)
VALID_CONFIDENCE=0.35
MIN_DT=0.0005
MAX_DT=0.1
EPS=1e-12
WORLD_UP=(0.0,0.0,1.0)

def read_bin(path:Path):
    data=path.read_bytes()
    if len(data)<HEADER_SIZE: raise ValueError(f'{path.name}のヘッダーが不足しています')
    if data[:4]!=b'GILS': raise ValueError(f'{path.name}のマジック値が不正です')
    record_size=struct.unpack_from('<i',data,8)[0]
    if record_size<RECORD.size: raise ValueError(f'{path.name}のレコードサイズが不正です: {record_size}')
    return [RECORD.unpack_from(data,o) for o in range(HEADER_SIZE,len(data)-record_size+1,record_size)]

def norm(v): return math.sqrt(sum(x*x for x in v))
def unit(v):
    n=norm(v)
    return None if not math.isfinite(n) or n<EPS else tuple(x/n for x in v)
def dot(a,b): return sum(x*y for x,y in zip(a,b))
def cross(a,b): return (a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0])
def qnorm(q):
    n=norm(q)
    return (1.0,0.0,0.0,0.0) if not math.isfinite(n) or n<EPS else tuple(x/n for x in q)
def qmul(a,b):
    w,x,y,z=a; p,u,v,t=b
    return (w*p-x*u-y*v-z*t,w*u+x*p+y*t-z*v,w*v-x*t+y*p+z*u,w*t+x*v-y*u+z*p)
def qconj(q): return (q[0],-q[1],-q[2],-q[3])
def qrotate(q,v): return qmul(qmul(q,(0.0,*v)),qconj(q))[1:]
def qeuler(q):
    w,x,y,z=qnorm(q)
    return (math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)),math.asin(max(-1,min(1,2*(w*y-z*x)))),math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)))
def qfrom_euler(r,p,y):
    cr,sr=math.cos(r/2),math.sin(r/2); cp,sp=math.cos(p/2),math.sin(p/2); cy,sy=math.cos(y/2),math.sin(y/2)
    return qnorm((cr*cp*cy+sr*sp*sy,sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy))
def qfrom_rate(w,dt):
    m=norm(w)
    if m<EPS:return (1.0,0.0,0.0,0.0)
    h=m*dt/2; s=math.sin(h)/m
    return qnorm((math.cos(h),w[0]*s,w[1]*s,w[2]*s))
def device_to_body(v): return (v[1],-v[0],v[2])
def getv(obj,name):
    v=obj[name]; return (float(v['x']),float(v['y']),float(v['z']))

def main():
    ap=argparse.ArgumentParser(description='未校正IMUログから車体姿勢をオフライン再計算します')
    ap.add_argument('session_dir',type=Path)
    args=ap.parse_args(); d=args.session_dir
    meta=json.loads((d/'session.json').read_text(encoding='utf-8'))
    cal=meta.get('statistics',{}).get('calibration')
    if not cal: raise SystemExit('session.jsonに初期校正結果がありません')
    gravity=getv(cal,'gravity_mps2'); bias=getv(cal,'gyroscope_bias_radps')
    gb=device_to_body(gravity)
    roll=math.atan2(gb[1],gb[2]); pitch=math.atan2(-gb[0],math.sqrt(gb[1]**2+gb[2]**2))
    q=qfrom_euler(roll,pitch,0.0)
    accel=read_bin(d/'accel.bin'); gyro=read_bin(d/'gyro.bin')
    events=sorted([(r[0],0,r) for r in accel]+[(r[0],1,r) for r in gyro])
    latest=None; last_g=None; rows=[]; skipped=0
    for t,kind,r in events:
        if kind==0:
            latest=device_to_body(r[1:4]); continue
        corrected=(r[1]-bias[0],r[2]-bias[1],r[3]-bias[2]); body=device_to_body(corrected)
        if last_g is None or t<=last_g: last_g=t; continue
        dt=(t-last_g)/1e9; last_g=t
        if not MIN_DT<=dt<=MAX_DT: skipped+=1; continue
        q=qnorm(qmul(q,qfrom_rate(body,dt)))
        norm_conf=0.0; residual=None
        if latest is not None:
            norm_conf=max(0.0,min(1.0,1.0-abs(norm(latest)-G)/MAX_G_ERR))
            measured=unit(latest); predicted=unit(qrotate(qconj(q),WORLD_UP))
            if measured and predicted:
                axis=cross(measured,predicted); an=norm(axis)
                residual=math.acos(max(-1,min(1,dot(predicted,measured))))
                if norm_conf>0 and an>=EPS:
                    angle=min(residual,CORRECTION_RATE*norm_conf*dt)
                    rate=tuple(x*(angle/an/dt) for x in axis)
                    q=qnorm(qmul(q,qfrom_rate(rate,dt)))
                    predicted=unit(qrotate(qconj(q),WORLD_UP))
                    residual=math.acos(max(-1,min(1,dot(predicted,measured))))
        direction=0.0 if residual is None else max(0.0,min(1.0,1.0-residual/MAX_RESIDUAL))
        confidence=norm_conf*direction; er,ep,ey=qeuler(q)
        rows.append((t,*q,er,ep,ey,*body,residual,confidence>=VALID_CONFIDENCE,confidence))
    out=d/'offline_vehicle_attitude.csv'
    with out.open('w',newline='',encoding='utf-8') as f:
        w=csv.writer(f); w.writerow(['elapsed_realtime_ns','quaternion_w','quaternion_x','quaternion_y','quaternion_z','roll_rad','pitch_rad','yaw_rad','roll_rate_radps','pitch_rate_radps','yaw_rate_radps','gravity_residual_rad','attitude_valid','attitude_confidence']); w.writerows(rows)
    rt=d/'vehicle_attitude.csv'; matched=0; max_q=0.0; max_r=0.0; max_p=0.0
    if rt.is_file():
        with rt.open(newline='',encoding='utf-8') as f: realtime={int(r['elapsed_realtime_ns']):r for r in csv.DictReader(f)}
        for r in rows:
            x=realtime.get(r[0])
            if not x: continue
            matched+=1
            max_q=max(max_q,max(abs(r[i]-float(x[n])) for i,n in zip(range(1,5),['quaternion_w','quaternion_x','quaternion_y','quaternion_z'])))
            max_r=max(max_r,abs(r[5]-float(x['roll_rad']))); max_p=max(max_p,abs(r[6]-float(x['pitch_rad'])))
    summary=(f'加速度件数: {len(accel)}\nジャイロ件数: {len(gyro)}\n姿勢出力件数: {len(rows)}\n更新間隔除外件数: {skipped}\nリアルタイム一致件数: {matched}\nQuaternion成分最大差: {max_q:.12g}\nロール最大差[deg]: {math.degrees(max_r):.12g}\nピッチ最大差[deg]: {math.degrees(max_p):.12g}\n')
    (d/'offline_vehicle_attitude_summary.txt').write_text(summary,encoding='utf-8'); print(summary,end='')
if __name__=='__main__': main()
