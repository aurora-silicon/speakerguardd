#!/usr/bin/env python3
"""macOS side of the spp3 full-chain safety scan (host only; run under a host-heavy slot).

For each Mac (AID) and program: the Mac's own pre-protection chain (preprotect_host:
Norm..ControlFreak (+AULowFlow), full volume, with every graph property of the property
strip, e.g. AULowFlow's 6620 sidecar) gives the driver feeds; those feeds go through the
Mac's own SpeakerProtect (spp3) offline (rig_host, open loop: no V/I telemetry, the same
configuration capture.py uses: property strip + sidecar).

The strip and property strip are the ones macOS selects for that Mac: the DSP database's
"General" speaker configuration (cdes kind 0) whose speaker ID (spid) is the Mac's, else
the vendor-neutral one (spid 65535). The speaker ID comes from the Mac's own tuning bundle
(manifest speaker_id), the same one v2 selects with; a Mac with vendor strips (AID9:
general_0 for spid 0, general_3 for spid 3) renders a different voicing under the neutral
strip.
Writes OUT/<mac>/<prog>/{feeds.f32, run.out.f32, run.state.csv, run.meta.json, pre.json}.
Everything here is Apple-derived: OUT must be a tainted directory.

  mac_refs.py [--bundles DIR] PROGS_DIR OUT_DIR [mac ...]

--bundles DIR: DIR/<mac>.tar.gz are the Macs' tuning bundles (scan.py's); without it every
Mac renders the vendor-neutral configuration.
"""
import json, os, plistlib, subprocess, sys, tarfile

TUN = '/System/Library/Audio/Tunings'
HERE = os.path.dirname(os.path.abspath(__file__))
PRE_SOURCE = os.path.join(HERE, 'preprotect_host.m')
SPID_GENERIC = 65535
RIGBIN = '/Volumes/AuroraS500/aurora-ctl/reports/speakerguardd-v2/rig/tools'
MACS = {'m2': (13, 4), 'm1pro': (9, 6), 'j616s': (35, 6)}


def selection(aid, spid):
    """(strip, propstrip) of the database's General speaker configuration for spid."""
    db = plistlib.load(open(f'{TUN}/DeviceDSPDatabase.plist', 'rb'))[f'AID{aid}']['Built-in Speaker']
    found = {}
    for name, c in db.items():
        sel = lambda key, s: next((d.get('Value') for d in c.get(key, []) if d.get('Selector') == s), None)
        kind = sel('DSPDeviceProperties', 'cdes')
        if not isinstance(kind, dict) or kind.get('kind') != 0:
            continue
        hw = {h.get('Type'): h.get('Path') for o in c.get('Output', []) for h in o.get('Hardware', [])}
        found[sel('UnderlyingDeviceProperties', 'spid')] = (name, hw.get('AUStrip'), hw.get('PropertyStrip'))
    name, strip, prop = found.get(spid, found.get(SPID_GENERIC, found.get(None)))
    return name, f'{TUN}/{strip}', f'{TUN}/{prop}' if prop else None


def bundle_spid(bundles, mac):
    if not bundles:
        return SPID_GENERIC
    t = tarfile.open(os.path.join(bundles, mac + '.tar.gz'))
    sid = json.loads(t.extractfile('audio-tuning/manifest.json').read()).get('speaker_id')
    return SPID_GENERIC if sid is None else sid


def propstrip_props(strip_path, propstrip_path):
    strip = plistlib.load(open(strip_path, 'rb'))
    sub = int.from_bytes(b'spp3', 'big')
    names = [e.get('displayname') for s in strip['strips'] for e in s['effects']
             if e.get('unit', {}).get('subtype') == sub]
    if not propstrip_path:
        return [], []
    boxes = plistlib.load(open(propstrip_path, 'rb'))['Boxes']
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
    argv = sys.argv[1:]
    bundles = None
    if argv[:1] == ['--bundles']:
        bundles, argv = argv[1], argv[2:]
    progs, out = argv[0], argv[1]
    os.makedirs(out, exist_ok=True)
    pre = os.environ.get('SGV2_PREPROTECT_HOST', os.path.join(out, 'preprotect_host'))
    if 'SGV2_PREPROTECT_HOST' not in os.environ and (
            not os.path.exists(pre) or os.path.getmtime(pre) < os.path.getmtime(PRE_SOURCE)):
        subprocess.run(['clang', '-fobjc-arc', '-framework', 'AudioToolbox', '-framework', 'Foundation',
                        PRE_SOURCE, '-o', pre], check=True)
    macs = argv[2:] or list(MACS)
    names = sorted(f[:-4] for f in os.listdir(progs) if f.endswith('.f32'))
    for mac in macs:
        aid, nd = MACS[mac]
        spid = bundle_spid(bundles, mac)
        config, strip, propstrip = selection(aid, spid)
        print(mac, f'spid {spid}: {config}: {strip} {propstrip}', flush=True)
        sidecar = f'{TUN}/AID{aid}/DSP/Strips/aid{aid}-aufx-spp3-appl.plist'
        props, paths = propstrip_props(strip, propstrip)
        for name in names:
            d = os.path.join(out, mac, name)
            if os.path.exists(os.path.join(d, 'run.meta.json')):
                continue
            os.makedirs(d + '/logs', exist_ok=True)
            os.chmod(os.path.join(out, mac), 0o700)
            feeds = os.path.join(d, 'feeds.f32')
            if not propstrip:
                raise SystemExit(f'{mac}: {config} has no property strip')
            subprocess.run([pre, '--strip', strip, '--propstrip', propstrip,
                            '--in', os.path.join(progs, name + '.f32'),
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
            m.update(mac=mac, aid=aid, program=name, props=props, plistprops=paths,
                     speaker_id=spid, configuration=config, strip=strip, propstrip=propstrip)
            json.dump(m, open(os.path.join(d, 'run.meta.json'), 'w'), indent=1)
            print(mac, name, 'ok', flush=True)


if __name__ == '__main__':
    main()
