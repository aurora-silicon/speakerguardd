#!/usr/bin/env python3
# Copyright (c) 2026 The Aurora Silicon Contributors
# SPDX-License-Identifier: MIT
"""Split offline per-box rig. Generate/compare on buildbox; AU host is separate.

No tuning values are embedded. Private inventory rows identify strip snapshots,
box names and graph channel widths. A refusal is never measured as a pass.
"""
import argparse
import collections
import hashlib
import json
import pathlib
import shutil
import subprocess
import time
import numpy as np

FS = 48000
FREQS = [20,25,31.5,40,50,63,80,100,125,160,200,250,315,400,500,630,800,
         1000,1250,1600,2000,2500,3150,4000,5000,6300,8000,10000,12500,16000,20000]
LEVELS = [0,-6,-12,-24,-40]
TYPES = ['cfrk','vben','axtc','vrq2','ldnm','pchr','volt']

def dump(p, value):
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(value, indent=1, allow_nan=False))

def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()

def generate(root):
    cases=[]
    def add(name, mono, ch, channel=None, measure_start=0, measure_frames=None):
        x=np.repeat(mono[:,None], ch, axis=1)
        if channel is not None:
            x[:]=0; x[:,channel]=mono
        p=root/'inputs'/f'{ch}ch-{name}.f32'; p.parent.mkdir(exist_ok=True)
        x.astype('<f4').tofile(p)
        cases.append({'name':name, 'ch':ch,'file':str(p.relative_to(root)),
                      'frames':len(mono),'sha256':sha(p),
                      'dynamic':name.startswith(('step','burst','history_step')),
                      'measure_start':measure_start,
                      'measure_frames':measure_frames if measure_frames is not None else len(mono)-measure_start})
    for ch in sorted({r['ch'] for r in json.loads((root/'inventory.json').read_text())['rows']}):
        n=FS; t=np.arange(n)/FS
        for f in FREQS:
            for db in LEVELS:
                add(f'tone_{f:g}_{db:g}',np.sin(2*np.pi*f*t)*10**(db/20),ch)
        rng=np.random.default_rng(5490); spectrum=np.fft.rfft(rng.standard_normal(n))
        k=np.arange(len(spectrum),dtype=float); k[0]=1
        pink=np.fft.irfft(spectrum/np.sqrt(k),n); pink/=max(abs(pink))
        for db in LEVELS: add(f'pink_{db:g}',pink*10**(db/20),ch)
        n=FS*4; t=np.arange(n)/FS
        for f in [40,125,1000,10000]:
            carrier=np.sin(2*np.pi*f*t)
            up=np.where(np.arange(n)<FS,0.01,1.0)
            down=np.where(np.arange(n)<FS,1.0,0.01)
            burst=np.where((np.arange(n)//4800)%10==0,1.,0.)
            for name,amp in [('step_up',up),('step_down',down),('burst',burst)]:
                add(f'{name}_{f:g}',carrier*amp,ch)
        # The short probes above start from one second of quiet history.  The
        # full-chain gate reaches VirtualBass after varied program material,
        # so retain twenty seconds of deterministic level/frequency history
        # before measuring the same -40 to 0 dBFS LF step.  Only the final
        # 1.5-second probe is scored; the prefix exists solely to advance the
        # box's follower and filter state.
        history_frames=20*FS; ht=np.arange(history_frames)/FS
        history=np.zeros(history_frames)
        hist_freqs=(31.5,63.,125.,250.,500.,1000.,2000.,4000.)
        hist_db=(-30.,-12.,-6.,-24.,0.,-18.,-9.,-36.)
        for second in range(20):
            lo=second*FS; hi=(second+1)*FS
            f=hist_freqs[second%len(hist_freqs)]; amp=10**(hist_db[second%len(hist_db)]/20)
            history[lo:hi]=amp*np.sin(2*np.pi*f*ht[lo:hi])
        probe_frames=3*FS//2; pt=np.arange(probe_frames)/FS
        step_amp=np.where(np.arange(probe_frames)<FS//2,0.01,1.0)
        for f in (40.,125.):
            probe=step_amp*np.sin(2*np.pi*f*pt)
            add(f'history_step_up_{f:g}',np.concatenate((history,probe)),ch,
                measure_start=history_frames,measure_frames=probe_frames)
        if ch>1:
            for c in range(ch):
                for f in [40,125,1000,10000,20000]:
                    add(f'only{c}_{f:g}',np.sin(2*np.pi*f*np.arange(FS)/FS)*10**(-6/20),ch,c)
    dump(root/'probes.json',{'fs':FS,'levels_dbfs':LEVELS,'cases':cases})
    print('generated',len(cases),'shared inputs',flush=True)

def metric(mac, v2, dynamic, measure_start=0, measure_frames=None):
    if mac.shape!=v2.shape or not np.isfinite(mac).all() or not np.isfinite(v2).all():
        raise ValueError('shape mismatch or nonfinite render')
    measure_end=len(mac) if measure_frames is None else min(len(mac),measure_start+measure_frames)
    mac=mac[measure_start:measure_end];v2=v2[measure_start:measure_end]
    n=len(mac); halves=[]; windows=[]; nulls=0
    # Steady ratios use a -240 dBFS numerical-silence floor. Transient
    # windows use the prior rig's -80 dBFS floor on BOTH sides, so unexpected
    # output above that floor remains a miss. No latency alignment is applied.
    def delta(a,b, floor=1e-12):
        nonlocal nulls
        ar=float(np.sqrt(np.mean(a*a))); br=float(np.sqrt(np.mean(b*b)))
        if max(ar,br)<floor:
            nulls+=1; return 0.
        return float(20*np.log10(max(br,1e-15)/max(ar,1e-15)))
    for c in range(mac.shape[1]):
        halves.append(delta(mac[n//2:,c],v2[n//2:,c]))
        if dynamic:
            w=[delta(mac[at:at+480,c],v2[at:at+480,c],1e-4) for at in range(0,n-479,480)]
            windows.append(max(w,key=abs))
    ds=halves+windows
    return {'signed_worst_db':max(ds,key=abs),'abs_db':max(map(abs,ds)),
            'steady_db':halves,'window_db':windows,'both_quiet_windows':nulls}

def render_row(root, row, tag='v2', force=False):
    result=root/'results'/f"{row['id']}-{tag}.json"
    if result.exists() and not force: return json.loads(result.read_text())
    probes=json.loads((root/'probes.json').read_text())['cases']
    probes=[p for p in probes if p['ch']==row['ch']]
    ready=root/'mac'/row['id']/'ready.json'
    if not ready.exists(): return None
    macinfo=json.loads(ready.read_text()); records=[]; retained=[]
    if row['v2_rc']:
        out={**row,'planned':len(probes),'cases':0,'over':0,'worst_db':None,
             'status':'v2-refused','au':macinfo,'records':[]}
        dump(result,out);return out
    work=root/'v2'/row['id'];work.mkdir(exist_ok=True,parents=True)
    for p in probes:
        name=p['name']; mp=root/'mac'/row['id']/f'{name}.f32'; vp=work/f'{name}.f32'
        record={'case':name,'input_sha256':p['sha256']}
        if not mp.exists():
            record.update(status='au-refused',error=macinfo.get('errors',{}).get(name,'missing output'))
        else:
            if not vp.exists() or force:
                cmd=[str(root/'bin/voicing-box-render'),'--strip',str(root/row['strip']),
                     '--only',row['box'],'--ch',str(row['ch']),str(root/p['file']),str(vp)]
                r=subprocess.run(cmd,capture_output=True,text=True,timeout=180)
                if r.returncode:
                    record.update(status='v2-refused',error=r.stderr);records.append(record);continue
            mac=np.fromfile(mp,'<f4').reshape(-1,row['ch']).astype(float)
            v2=np.fromfile(vp,'<f4').reshape(-1,row['ch']).astype(float)
            try:
                record.update(metric(mac,v2,p['dynamic'],p.get('measure_start',0),p.get('measure_frames')))
                record['status']='measured'
                record['measure_start']=p.get('measure_start',0)
                record['measure_frames']=p.get('measure_frames',p['frames'])
                record['mac_sha256']=sha(mp);record['v2_sha256']=sha(vp)
            except ValueError as e: record.update(status='invalid',error=str(e))
        records.append(record)
    measured=[r for r in records if r['status']=='measured']
    worst=max(measured,key=lambda r:r['abs_db']) if measured else None
    over=[r for r in measured if r['abs_db']>0.05]
    def signature(r):
        name=r['case']
        if name.startswith('step_down'): return 'release windows'
        if name.startswith(('step_up','history_step_up')): return 'attack windows'
        if name.startswith('burst'): return 'burst/tail windows'
        if name.startswith('only'): return 'channel isolation'
        if name.startswith('pink'): return 'broadband gain'
        f=float(name.split('_')[1])
        return 'low-frequency steady gain' if f<250 else ('midband steady gain' if f<4000 else 'high-frequency steady gain')
    sig=collections.Counter(signature(r) for r in over)
    out={**row,'planned':len(probes),'cases':len(measured),'over':len(over),
         'worst_db':worst['abs_db'] if worst else None,'worst_case':worst['case'] if worst else None,
         'worst_signed_db':worst['signed_worst_db'] if worst else None,
         'signature':sig.most_common(1)[0][0] if sig else 'within amplitude gate',
         'status':'measured' if len(measured)==len(probes) else 'incomplete','au':macinfo,'records':records}
    dump(result,out)
    # Retain the worst paired case, plus the first case, on the buildbox.
    # Checksums and metrics preserve all other evidence; reruns regenerate v2.
    keep={p['name'] for p in probes[:1]}
    if worst: keep.add(worst['case'])
    for p in work.glob('*.f32'):
        if p.stem not in keep: p.unlink()
    return out

def worker(root, seconds):
    inv=json.loads((root/'inventory.json').read_text())['rows']; end=time.monotonic()+seconds
    while time.monotonic()<end:
        left=0
        for row in inv:
            if not (root/'results'/f"{row['id']}-v2.json").exists():
                r=render_row(root,row)
                if r is None: left+=1
                else: print(row['id'],r['status'],r['cases'],r['over'],r['worst_db'],flush=True)
        if not left: return
        time.sleep(3)
    raise SystemExit('worker deadline; evidence retained, can resume')

def runbox(root, typ, tag):
    for row in json.loads((root/'inventory.json').read_text())['rows']:
        if row['type']==typ:
            r=render_row(root,row,tag,True)
            if r is None: raise SystemExit('missing AU references: '+row['id'])
    print('rerendered',typ,tag)

def main():
    p=argparse.ArgumentParser();p.add_argument('action',choices=['generate','worker','run'])
    p.add_argument('--root',type=pathlib.Path,required=True);p.add_argument('--box',choices=TYPES)
    p.add_argument('--tag',default='v2');p.add_argument('--seconds',type=int,default=10800)
    a=p.parse_args()
    if a.action=='generate':generate(a.root)
    elif a.action=='worker':worker(a.root,a.seconds)
    else:runbox(a.root,a.box,a.tag)
if __name__=='__main__':main()
