#!/usr/bin/env python3
# Copyright (c) 2026 The Aurora Silicon Contributors
# SPDX-License-Identifier: MIT
"""Thin stdlib AU renderer. Inputs and analysis belong on the buildbox.

Use an existing host-heavy slot, nice 15, max two concurrent AU processes.
No playback, usage checks, tuning edits, or local-machine overrides.
"""
import argparse
import concurrent.futures
import hashlib
import json
import pathlib
import shutil
import subprocess
import time

FACTORY={'cfrk':'AUControlFreakFactory','vben':'AUVirtualBassFactory','axtc':'AUXTCFactory',
         'vrq2':'AUVariableEQV2Factory','ldnm':'AULoudnessNormalizerFactory',
         'pchr':'AUMatrixPatcherFactory','volt':'AUVolumeTaperFactory'}

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    p=argparse.ArgumentParser();p.add_argument('--root',type=pathlib.Path,required=True)
    p.add_argument('--box-host',required=True);p.add_argument('--slot',type=pathlib.Path,required=True)
    p.add_argument('--slot-owner',required=True)
    p.add_argument('--buildbox-dest',required=True);p.add_argument('--box');p.add_argument('--id')
    p.add_argument('--minutes',type=int,default=25);a=p.parse_args()
    if not a.slot.is_dir():raise SystemExit('host-heavy resource slot required')
    owner=a.slot/'owner'
    if not owner.is_file() or not owner.read_text().startswith(a.slot_owner+' '):
        raise SystemExit('host-heavy slot owner does not match this run')
    pressure=subprocess.check_output(['memory_pressure'],text=True).splitlines()[-1]
    import re
    match=re.search(r'(\d+)%',pressure)
    if match and int(match[1])<20: raise SystemExit('host memory resource guard: below 20 percent free')
    root=a.root; inventory=json.loads((root/'inventory.json').read_text())
    probes=json.loads((root/'probes.json').read_text())['cases'];start=time.monotonic()
    rows=[r for r in inventory['rows'] if (not a.box or r['type']==a.box) and (not a.id or r['id']==a.id)]
    # Run the smaller boxes first so every rig receives early coverage.
    priority={'volt':0,'pchr':1,'axtc':2,'vrq2':3,'vben':4,'ldnm':5,'cfrk':6}
    rows.sort(key=lambda r:(priority[r['type']],r['aid'],r['id']))
    def one(row):
        d=root/'mac'/row['id'];d.mkdir(exist_ok=True,parents=True)
        done=root/'uploaded'/f"{row['id']}.json";done.parent.mkdir(exist_ok=True)
        if done.exists():return
        for disk in [pathlib.Path('/'),root]:
            if shutil.disk_usage(disk).free<20*1024**3:raise RuntimeError('disk guard: below 20 GiB')
        strip=row['sources'][0]['strip']
        selected=[c for c in probes if c['ch']==row['ch']]
        # A single AU probe still verifies installation for a v2 compile refusal;
        # no full-case comparison is possible. Count the rest as unmeasured.
        if row['v2_rc']:selected=selected[:1]
        errors={};hashes={};meta={}
        for c in selected:
            out=d/f"{c['name']}.f32";mp=d/f"{c['name']}-meta.json"
            if not out.exists():
                actual_input=root/c['file']
                preroll=96000 if row['type']=='volt' else 0
                if preroll:
                    actual_input=d/'prerolled-input.f32'
                    actual_input.write_bytes(bytes(preroll*row['ch']*4)+(root/c['file']).read_bytes())
                cmd=['nice','-n','15',a.box_host,'--strip',strip,'--stage',row['box'],
                     '--subtype',row['type'],'--factory',FACTORY[row['type']],
                     '--channels',str(row['ch']),'--in',str(actual_input),
                     '--out',str(out)+'.tmp','--meta',str(mp)]
                if row['bypass']:
                    if row['type']!='vben': raise RuntimeError('unsupported graph bypass plumbing')
                    cmd += ['--set','15=1']
                r=subprocess.run(cmd,capture_output=True,text=True,timeout=90)
                if r.returncode:
                    errors[c['name']]=r.stderr;break
                temp=pathlib.Path(str(out)+'.tmp')
                if preroll:
                    raw=temp.read_bytes();temp.write_bytes(raw[preroll*row['ch']*4:])
                    actual_input.unlink()
                temp.rename(out)
            hashes[c['name']]=sha(out)
            if mp.exists():meta[c['name']]=json.loads(mp.read_text())
        ready={'id':row['id'],'rendered':len(hashes),'errors':errors,'hashes':hashes,'meta':meta,
               'host_build':inventory['host_build'],'preroll_frames':96000 if row['type']=='volt' else 0,'strip_sha256':sha(pathlib.Path(strip)),
               'box_host_sha256':sha(pathlib.Path(a.box_host))}
        # A ready marker is copied only after all audio. The buildbox worker
        # sees complete states; host copies can then be deleted safely.
        rp=d/'ready.json';rp.write_text(json.dumps(ready,indent=1))
        dest=a.buildbox_dest.rstrip('/')+'/mac/'+row['id']+'/'
        subprocess.run(['rsync','-a','--checksum','--exclude','ready.json','-e','ssh -o ConnectTimeout=10',str(d)+'/',dest],check=True,timeout=300)
        subprocess.run(['rsync','-a','--checksum','-e','ssh -o ConnectTimeout=10',str(rp),dest],check=True,timeout=60)
        done.write_text(json.dumps(ready,indent=1))
        for f in d.glob('*.f32'):f.unlink()
        print(row['id'],'AU',len(hashes),'errors',len(errors),flush=True)
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as ex:
        pending={};it=iter(rows)
        while True:
            while len(pending)<2 and time.monotonic()-start<a.minutes*60:
                try:row=next(it)
                except StopIteration:break
                if (root/'uploaded'/f"{row['id']}.json").exists():continue
                pending[ex.submit(one,row)]=row['id']
            if not pending:break
            completed,_=concurrent.futures.wait(pending,return_when=concurrent.futures.FIRST_COMPLETED)
            for future in completed:future.result();pending.pop(future)
    print('session complete; release host-heavy slot',flush=True)
if __name__=='__main__':main()
