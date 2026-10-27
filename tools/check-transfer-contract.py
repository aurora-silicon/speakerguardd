#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Exact synthetic witnesses for docs/transfer-certificate.md; no hardware I/O.

This does not accept a physical certificate or qualify a playback route.
All filter parameters are mathematical examples, unrelated to TAS/AOP facts.
"""
from fractions import Fraction as Q
import json


def trajectory(a, words, initial):
    state = initial
    output = []
    for x in words:
        output.append(x - state)
        state = a * state + (1 - a) * x
    return output, state


def energy(words):
    return sum((x*x for x in words), Q(0))


def verify_storage(a):
    assert 0 <= a < 1
    p = 2 / (1 - a*a)
    g2 = 4 / (1+a)**2
    r = (1-a)/(1+a)
    # Verify the three polynomial coefficients, so the identity holds for
    # all real (state,input), not only for a finite selection of trajectories.
    # g2*x^2 - (x-s)^2 - p*(a*s+(1-a)*x)^2 + p*s^2
    # = (s+r*x)^2 >= 0.
    assert p*(1-a*a)-1 == 1
    assert 2-2*p*a*(1-a) == 2*r
    assert g2-1-p*(1-a)**2 == r*r
    # [-1,1] is invariant for the state with full-scale-bounded input.
    assert a >= 0 and 1-a > 0 and a+(1-a) == 1
    checks = 0
    for initial in [Q(-1), Q(0), Q(1)]:
        for words in [[Q(0)]*64, [Q(1)]*64, [Q(-1)]*64,
                      [Q((-1)**n) for n in range(64)],
                      [Q((n*37 % 31)-15, 16) for n in range(64)]]:
            out, end = trajectory(a, words, initial)
            assert energy(out)+p*end*end <= g2*energy(words)+p*initial*initial
            assert -1 <= end <= 1
            checks += 1
    # Exact infinite zero-input energy, not a finite decay/noise-floor test.
    zero_input_tail = 1/(1-a*a)
    assert zero_input_tail <= p
    # Scaling output by (1+a)/2 gives a gain-one squared-norm certificate.
    normalized_p = (1+a)/(2*(1-a))
    assert normalized_p == p*(1+a)**2/4
    micros = normalized_p * Q(1000000, 48000)
    rounded_tail = -(-micros.numerator // micros.denominator)
    assert Q(rounded_tail, 1000000) >= normalized_p/48000
    return dict(a=str(a), gamma_squared=str(g2), storage=str(p),
                normalized_gain_one_storage=str(normalized_p),
                example_only_tail_us_at_48k=rounded_tail,
                exact_polynomial_identity=True, trajectories_checked=checks)


def main():
    storage = [verify_storage(a) for a in [Q(0), Q(1, 2), Q(3, 4), Q(99, 100)]]
    a = Q(3, 4)
    words = [Q((n*13 % 15)-7, 32) for n in range(64)]
    offset = Q(1, 2)
    shifted = [x+offset for x in words]
    out, end = trajectory(a, words, Q(0))
    other, other_end = trajectory(a, shifted, offset)
    assert out == other and other_end == end+offset
    assert all(-1 <= x <= 1 for x in words+shifted)
    assert energy(words) != energy(shifted)
    # A static clipping witness survives even known-zero state.
    clip = lambda x: max(Q(-1, 2), min(Q(1, 2), x))
    assert clip(Q(3, 4)) == clip(Q(1))
    assert Q(3, 4)**2 != Q(1)**2
    # Disallow the unjustified additive residual formula. Young's factors
    # with epsilon=1 repair this exact one-sample counterexample.
    new, residual = Q(1), Q(1)
    assert (new+residual)**2 > new**2+residual**2
    assert (new+residual)**2 == 2*new**2+2*residual**2
    # A positive state jump with zero input consumes fresh physical energy
    # on each reset. An epoch nonce cannot turn these jumps into one tail.
    a = Q(1, 2)
    p = 2/(1-a*a)
    assert p*Q(1)**2-p*Q(0)**2 == p > 0
    resets = 4
    repeated_minimum_output_energy = resets * Q(1)
    assert repeated_minimum_output_energy > p
    print(json.dumps(dict(scope=__doc__, physical_certificate=False,
        storage_identities=storage,
        inverse_witness=dict(samples=len(words), identical_output=True,
            input_a_squared_dose=str(energy(words)),
            input_b_squared_dose=str(energy(shifted)), offset=str(offset)),
        clipping_nonuniqueness=True, discarded_cross_term_rejected=True,
        unpaid_epoch_jump_rejected=True, checks='PASS'), indent=2))


if __name__ == '__main__':
    main()
