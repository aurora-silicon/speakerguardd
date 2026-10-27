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



def weighted_storage_bound(weights, bound):
    """Exact Abel identity and supremum for every 0 <= V[n] <= bound.

    weights run in forward time, rather than impulse-response age order.
    The extremal storage picks B precisely on positive coefficients. Choosing
    u=max(0,V[n+1]-V[n]) realizes nonnegative input and output energies, so
    the supremum is attained without a signed physical energy assumption.
    """
    assert weights and all(h >= 0 for h in weights) and bound >= 0
    coefficients = [weights[0]]
    coefficients += [b-a for a, b in zip(weights, weights[1:])]
    coefficients += [-weights[-1]]
    storage = [bound if c > 0 else Q(0) for c in coefficients]
    supplied = [max(Q(0), b-a) for a, b in zip(storage, storage[1:])]
    output = [u+a-b for u, a, b in zip(supplied, storage, storage[1:])]
    assert all(u >= 0 for u in supplied) and all(w >= 0 for w in output)
    lhs = sum((h*(w-u) for h, w, u in zip(weights, output, supplied)), Q(0))
    rhs = sum((c*v for c, v in zip(coefficients, storage)), Q(0))
    supremum = bound * sum((max(Q(0), c) for c in coefficients), Q(0))
    assert lhs == rhs == supremum
    return supremum


def verify_weighted_residual():
    # Exact discrete positive two-node systems, with all tau orderings,
    # repeated poles and a zero-retention boundary. These are mathematical
    # examples; no coefficients describe a TAS/AOP or physical thermal model.
    checked = 0
    for alpha in [Q(0), Q(1, 4), Q(1, 2), Q(3, 4), Q(99, 100)]:
        for beta in [Q(0), Q(1, 4), Q(1, 2), Q(3, 4), Q(99, 100)]:
            for a in [Q(1, 4), Q(1), Q(4)]:
                for b in [Q(1, 4), Q(1), Q(4)]:
                    coil, magnet = [a], [b]
                    for _ in range(64):
                        coil.append(alpha*coil[-1]+(1-alpha)*magnet[-1])
                        magnet.append(beta*magnet[-1])
                    for kernel, cap in [(coil, max(a, b)), (magnet, b)]:
                        # A nonnegative two-exponential impulse has no
                        # interior minimum; verify unimodality in each example.
                        falling = False
                        for left, right in zip(kernel, kernel[1:]):
                            if right < left:
                                falling = True
                            if falling:
                                assert right <= left
                        assert max(kernel) <= cap
                        bound = Q(3, 7)
                        exact = weighted_storage_bound(kernel[::-1], bound)
                        assert exact == bound*max(kernel) <= bound*cap
                        checked += 1
    windows = 0
    for total in [1, 2, 8, 65]:
        for start in range(total):
            for end in [start+1, total]:
                weights = [Q(int(start <= n < end)) for n in range(total)]
                assert weighted_storage_bound(weights, Q(3, 7)) == Q(3, 7)
                windows += 1
    # A whole-run energy bound alone has no time-local guarantee: input can
    # be stored until the input-only first-order temperature has cooled.
    retention, wait = Q(1, 2), 8
    actual_at_release, input_only = Q(1), retention**wait
    assert actual_at_release > input_only
    # Repeating unpaid releases invalidates a one-tail reserve. A common
    # bounded storage must be recharged by paid input between releases.
    bound, cycles = Q(1), 16
    storage = [bound]
    supplied, output = [], []
    for _ in range(cycles):
        supplied.extend([Q(0), bound])
        output.extend([bound, Q(0)])
        storage.extend([Q(0), bound])
    assert all(w+after-before <= u for w, u, before, after in
               zip(output, supplied, storage, storage[1:]))
    assert sum(output)-sum(supplied) <= bound
    return dict(exact_weighted_thermal_witnesses=checked,
                exact_window_witnesses=windows,
                attainable_abel_supremum=True,
                whole_run_energy_only_counterexample=True,
                paid_repeated_release_cycles=cycles,
                physical_certificate=False)



def inverse_interval(a, observed):
    """Complete exact inverse class for this example's known, noiseless HPF.

    x[n] = initial + y[n] + (1-a)*sum(y[:n]). Full-scale input and
    initial-state bounds intersect to one interval. Losses, clipping,
    coefficient/scale uncertainty and unknown epochs are NOT covered here.
    """
    assert 0 <= a < 1 and observed
    partial, base = Q(0), []
    low, high = Q(-1), Q(1)
    for y in observed:
        base.append(y+(1-a)*partial)
        partial += y
        low, high = max(low, -1-base[-1]), min(high, 1-base[-1])
    if low > high:
        return None
    vertex = -sum(base)/len(base)
    minimum_at = max(low, min(high, vertex))
    dose = lambda state: energy([state+x for x in base])
    maximum_at = max([low, high], key=dose)
    for initial in [low, high, minimum_at, maximum_at]:
        words = [initial+x for x in base]
        assert all(-1 <= x <= 1 for x in words)
        actual, _ = trajectory(a, words, initial)
        assert actual == observed
    return dict(initial_interval=[str(low), str(high)],
                minimum_squared_dose=str(dose(minimum_at)),
                maximum_squared_dose=str(dose(maximum_at)),
                minimum_at=str(minimum_at), maximum_at=str(maximum_at))


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
    inverse = inverse_interval(a, out)
    assert inverse is not None
    assert Q(inverse['minimum_squared_dose']) <= energy(words)
    assert Q(inverse['maximum_squared_dose']) >= energy(shifted)
    assert inverse_interval(a, [Q(3)]) is None
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
        weighted_residual=verify_weighted_residual(),
        inverse_witness=dict(samples=len(words), identical_output=True,
            input_a_squared_dose=str(energy(words)),
            input_b_squared_dose=str(energy(shifted)), offset=str(offset)),
        exact_noiseless_example_inverse=inverse,
        infeasible_example_inverse_rejected=True,
        clipping_nonuniqueness=True, discarded_cross_term_rejected=True,
        unpaid_epoch_jump_rejected=True, checks='PASS'), indent=2))


if __name__ == '__main__':
    main()
