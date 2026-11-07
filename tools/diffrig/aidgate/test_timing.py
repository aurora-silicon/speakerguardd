#!/usr/bin/env python3
# Copyright (c) 2026 The Aurora Silicon Contributors
# SPDX-License-Identifier: MIT
"""A transport delay must fail the gate, even when steady levels match."""
import numpy as np
from gate import compare, lag_of

rng = np.random.default_rng(5595)
x = rng.standard_normal((8192, 2)).astype(np.float32) * 0.1
x[:2048] = 0
segs = [{"name": "onset", "kind": "dynamic", "start": 0, "frames": len(x)}]
assert compare(x, x, segs)["over"] == 0
assert lag_of(x, x, segs[0]) == 0
y = np.concatenate([np.zeros((512, 2), np.float32), x[:-512]])
assert lag_of(x, y, segs[0]) == 512
assert compare(x, y, segs)["over"] > 0
print("timing control PASS: zero-delay identity passes, 512-frame transport fails")
