#!/usr/bin/env python3
"""Emit a single-board test gate; never edit the shipped default in place."""
import argparse
import json
from pathlib import Path

ap = argparse.ArgumentParser(description=__doc__)
ap.add_argument('board', choices=('j616s', 'j316s', 'j413'))
ap.add_argument('output', type=Path)
a = ap.parse_args()
source = Path(__file__).resolve().parents[1] / 'session' / 'gate.json'
if a.output.resolve() == source:
    ap.error('output must not overwrite the shipped default')
gate = json.loads(source.read_text())
if gate.get('voicing_pre') != {}:
    ap.error('expected the all-off shipped default')
gate['voicing_pre'] = {a.board: True}
a.output.write_text(json.dumps(gate, indent=2) + '\n')
