#!/usr/bin/env python3
# Copyright (c) 2026 The Aurora Silicon Contributors
# SPDX-License-Identifier: MIT
# perturb.py PROG.f32 OUT.f32 SEED: every nonzero sample of the gate program moved by one Float32
# ULP up or down (deterministic, seeded); zeros stay zero. For attest.py spread (README).
import sys, array, random
src, dst, seed = sys.argv[1], sys.argv[2], int(sys.argv[3])
a = array.array("f"); a.frombytes(open(src, "rb").read())
u = array.array("I", a.tobytes())
rng = random.Random(seed)
for i, v in enumerate(a):
    if v != 0.0:
        u[i] += 1 if rng.random() < 0.5 else -1   # same sign, magnitude one ULP up/down
open(dst, "wb").write(u.tobytes())
