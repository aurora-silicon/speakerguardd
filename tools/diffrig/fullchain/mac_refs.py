#!/usr/bin/env python3
"""macOS side of the spp3 full-chain safety scan (host only; run under a host-heavy slot).

For each Mac (AID) and program: the Mac's own pre-protection chain (preprotect_host:
Norm..ControlFreak (+AULowFlow), full volume) gives the driver feeds; those feeds go
through the Mac's own SpeakerProtect (spp3) offline (rig_host, open loop: no V/I
telemetry, the same configuration capture.py uses: property strip + sidecar).
Writes OUT/<mac>/<prog>/{feeds.f32, run.out.f32, run.state.csv, run.meta.json, pre.json}.
Everything here is Apple-derived: OUT must be a tainted directory.

  mac_refs.py PROGS_DIR OUT_DIR [mac ...]
"""
import glob, json, os, plistlib, subprocess, sys

TUN = '/System/Library/Audio/Tunings'
PRE = glob.glob('/Volumes/AuroraS500/aurora-ctl/scratch/t-4727-*/preprotect_host')[0]
RIGBIN = '/Volumes/AuroraS500/aurora-ctl/reports/speakerguardd-v2/rig/tools'
MACS = {'m2': (13, 4), 'm1pro': (9, 6), 'j616s': (35, 6)}


def propstrip_props(aid):
    base = f'{TUN}/AID{aid}/DSP/Strips/builtin_speaker_out_general'
    strip = plistlib.load(open(base + '.austrip', 'rb'))
    sub = int.from_bytes(b'spp3', 'big')
    names = [e.get('displayname') for s in strip['strips'] for e in s['effects']
             if e.get('unit', {}).get('subtype') == sub]
    try:
        boxes = plistlib.load(open(base + '.propstrip', 'rb'))['Boxes']
    except FileNotFoundError:
        return [], []
    props, paths = [], []
    for box in boxes:
        if box.get('Name') not in names:
            continue
        for p in box.get('Properties', []):
            if 'UInt32' in p:
                props.append(f"{p['Number']}={p['UInt32']}")
            elif 'Path' in p and p['Number'] != 64003:
                paths.append(f"{p['Number']}={p['Path']}")
    return props, paths


def main():
    progs, out = sys.argv[1], sys.argv[2]
    macs = sys.argv[3:] or list(MACS)
    names = sorted(f[:-4] for f in os.listdir(progs) if f.endswith('.f32'))
    for mac in macs:
        aid, nd = MACS[mac]
        strip = f'{TUN}/AID{aid}/DSP/Strips/builtin_speaker_out_general.austrip'
        sidecar = f'{TUN}/AID{aid}/DSP/Strips/aid{aid}-aufx-spp3-appl.plist'
        props, paths = propstrip_props(aid)
        for name in names:
            d = os.path.join(out, mac, name)
            if os.path.exists(os.path.join(d, 'run.meta.json')):
                continue
            os.makedirs(d + '/logs', exist_ok=True)
            os.chmod(os.path.join(out, mac), 0o700)
            feeds = os.path.join(d, 'feeds.f32')
            subprocess.run([PRE, '--strip', strip, '--in', os.path.join(progs, name + '.f32'),
                            '--out', feeds, '--meta', os.path.join(d, 'pre.json'),
                            '--drivers', str(nd), '--q', '512'], check=True)
            args = [os.path.join(RIGBIN, 'rig_host'), '--unit', 'spp3', '--strip', strip,
                    '--sidecar', sidecar, '--in', feeds, '--ch', str(nd), '--file-ch', str(nd),
                    '--q', '512', '--out', os.path.join(d, 'run')]
            for pr in props:
                args += ['--prop', pr]
            for pr in paths:
                args += ['--plistprop', pr]
            env = dict(os.environ, DYLD_INSERT_LIBRARIES=os.path.join(RIGBIN, 'prefshim.dylib'),
                       RIGREDIR_TO=os.path.join(d, 'logs'))
            p = subprocess.run(args, env=env, cwd=d, capture_output=True, text=True)
            open(os.path.join(d, 'host.log'), 'w').write(p.stdout + p.stderr)
            if p.returncode:
                raise SystemExit(f'{mac}/{name}: rig_host rc={p.returncode}: {p.stderr[-400:]}')
            m = json.load(open(os.path.join(d, 'run.meta.json')))
            m.update(mac=mac, aid=aid, program=name, props=props, plistprops=paths)
            json.dump(m, open(os.path.join(d, 'run.meta.json'), 'w'), indent=1)
            print(mac, name, 'ok', flush=True)


if __name__ == '__main__':
    main()
