#!/usr/bin/env python3
# Copyright (c) 2026 The Aurora Silicon Contributors
# SPDX-License-Identifier: MIT
"""Integration controls using synthetic public-SDK presets, never Mac tunings."""
import argparse
import pathlib
import plistlib
import struct
import subprocess


def main():
    p=argparse.ArgumentParser();p.add_argument('--renderer',required=True)
    p.add_argument('--work',type=pathlib.Path,required=True);a=p.parse_args()
    a.work.mkdir(exist_ok=True,parents=True)
    def strip(kind,params,bypass=False):
        unit={k:int.from_bytes(v.encode(),'big') for k,v in [('type','aufx'),('subtype',kind),('manufacturer','appl')]}
        blob=struct.pack('>III',0,0,len(params))+b''.join(struct.pack('>If',k,v) for k,v in params)
        e={'displayname':'test','unit':unit,'bypass':bypass,'aupreset':dict(unit,version=0,data=blob)}
        q=a.work/'test.austrip';q.write_bytes(plistlib.dumps({'strips':[{'effects':[e]}]}));return q
    inp=a.work/'in.f32';out=a.work/'out.f32'
    raw=b''.join(struct.pack('<f',v) for _ in range(1001) for v in [.25,-.125])
    inp.write_bytes(raw)
    def run(q,ch=2,extra=()):
        return subprocess.run([a.renderer,'--strip',str(q),'--only','test','--ch',str(ch),*extra,str(inp),str(out)],capture_output=True,text=True)
    # Graph bypass must preserve every frame including a partial final block.
    for kind in ['cfrk','vben','axtc','vrq2','ldnm','pchr','volt']:
        q=strip(kind,[],True);r=run(q)
        assert r.returncode==0,(kind,r.stderr)
        assert out.read_bytes()==raw,kind
    # Active synthetic cross-channel mapping checks dispatch, ordering and a
    # partial final processing block independently of any machine preset.
    q=strip('pchr',[(0,0.),(1,.5),(2,-.25),(3,0.)]);r=run(q)
    assert r.returncode==0,r.stderr
    expected=b''.join(struct.pack('<f',v) for _ in range(1001) for v in [-.0625,-.0625])
    assert out.read_bytes()==expected,'active matrix/channel ordering'
    # Missing active state is refused, never silently converted into a stub.
    for kind in ['cfrk','vben','axtc','vrq2','ldnm','pchr','volt']:
        assert run(strip(kind,[(12,1.)] if kind=='cfrk' else [])).returncode!=0,kind
    q=strip('vben',[],True)
    assert run(q,0).returncode!=0
    assert run(q,9).returncode!=0
    inp.write_bytes(raw+b'\0')
    assert run(q).returncode!=0,'partial f32 frame accepted'
    inp.write_bytes(raw)
    # Unknown subtype is a hard refusal; it cannot qualify as identity.
    assert run(strip('zzzz',[])).returncode!=0
    print('renderer controls PASS: seven bypasses, seven active refusals, partial block/frame, invalid widths, unknown unit')
if __name__=='__main__':main()
