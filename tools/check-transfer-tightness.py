#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Check the bounded-current impossibility witnesses and profile dose bounds.

No device access. Exact witnesses concern the declared reference contract,
not an identified physical speaker. Run from any directory; JSON goes to stdout.
"""
import configparser
from fractions import Fraction as Q
import hashlib
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MARGIN = Q(201, 200)


def verify_witnesses():
    # Normalize Vmax=Imax=fs=1. This passive, memoryless relation attains
    # the current-only bound for EVERY finite word vector, not just these.
    sign = lambda x: Q(1) if x > 0 else Q(-1) if x < 0 else Q(0)
    vectors = [[Q(0)], [Q(1, 32)] * 32,
               [Q((-1)**n, 32) for n in range(32)],
               [Q((n * 37 % 31) - 15, 16) for n in range(64)],
               [Q(1)] + [Q(0)] * 63]
    for words in vectors:
        powers = [x * sign(x) for x in words]
        assert all(p >= 0 for p in powers)
        assert sum(powers) == sum(map(abs, words))
    # Degree-p>1 supply fails on a repeated nonzero amplitude unless paid
    # background closes the entire per-frame deficit. A finite tail B cannot.
    a, background, tail = Q(1, 32), MARGIN / 100000, Q(10)
    deficit = a - MARGIN * a*a - background
    assert deficit > 0
    repeats = tail // deficit + 1
    assert repeats * a > repeats * (MARGIN*a*a + background) + tail
    # The discontinuity is unnecessary: i=v/(eps+|v|) is continuous,
    # bounded by 1 and passive. With eps=a/1000 it approaches the same bound.
    epsilon = a / 1000
    smooth_power = a*a/(epsilon+a)
    assert smooth_power == Q(1000, 1001)*a
    assert smooth_power > MARGIN*a*a+background
    # Favor a hypothetical squared contract by granting the entire retained
    # linear Ps=F*1e-5 to background, even before paying any free PCM.
    # This overgrant still does not fix the deficit above.
    assert MARGIN*Q(1, 100000) < Q(11, 1000000)
    # A tested prefix does not constrain a continuation with only |i|<=1.
    prefix = [Q(1, 32)]*8
    low = [x for x in prefix]  # resistive prefix current
    before = [x*i for x, i in zip(prefix, low)]
    assert before == [x*x for x in prefix]
    high_continuation = a * sign(a)
    assert high_continuation > MARGIN*a*a + background
    return dict(exact_vectors=len(vectors), equality_for_arbitrary_vectors=True,
                passive_continuous_approximation=True,
                squared_deficit_per_normalized_frame=str(deficit),
                example_tail=str(tail), frames_exhausting_tail=int(repeats),
                prefix_does_not_certify_future=True, physical_certificate=False)


def verify_conditional_passivity():
    # Coefficient equality proves the square identity for all real (v,i).
    # This checks algebra, not the additional physical storage premise.
    checks = 0
    for resistance in [Q(1, 10), Q(1), Q(16)]:
        assert resistance**2/resistance == resistance
        assert -2*resistance/resistance == -2
        for voltage in [Q(-2), Q(-1, 3), Q(0), Q(1, 5), Q(2)]:
            for current in [Q(-2), Q(-1, 7), Q(0), Q(1, 11), Q(2)]:
                electrical = voltage*current
                storage_derivative = electrical-resistance*current**2
                supply = voltage**2/resistance
                gap = supply-(2*electrical-resistance*current**2)
                assert gap == (voltage-resistance*current)**2/resistance >= 0
                assert max(Q(0), electrical)+storage_derivative <= supply
                assert resistance*current**2+2*storage_derivative <= supply
                checks += 1
    # R=Imax=1 cannot bound S=L*i^2/2 without an inductance bound.
    for proposed_bound in [Q(1), Q(1000), Q(10**12)]:
        inductance = 8*proposed_bound+1
        reachable_current = Q(1, 2)
        assert inductance*reachable_current**2/2 > proposed_bound
    return dict(conditional_square_identity=True, exact_checks=checks,
                uniform_storage_not_implied_by_current_cap=True,
                physical_premises_qualified=False)


def profile_rows():
    # Positive Taylor terms show exp(2.31)>10, hence ln(10)/10<0.231.
    assert sum((Q(231, 100)**i / math.factorial(i) for i in range(8)), Q(0)) > 10
    rows = []
    # The native-word lower bound favors feasibility for both S32 and S24.
    # Even the coarser S16 floor loses one further quantum before computing it.
    phases = [(18, Q(1, 10)), (10, Q(3, 100)),
              (15, Q(1, 4)), (10, Q(2, 25))]
    for name, budget in [('j316', 1), ('j413', 1), ('j616', 0)]:
        path = ROOT / 'conf' / 'apple' / (name + '.conf')
        cfg = configparser.ConfigParser()
        cfg.read(path)
        g = cfg['Globals']
        ambient = Q(g['t_ambient'])
        for section in cfg.sections():
            if not section.startswith('Speaker/'):
                continue
            s = cfg[section]
            voltage = Q(s['vs_scale'])
            # All profiles here select vs_scale; do not approximate amp gain.
            exponent = Q(g['amp_gain_dbv'])/10
            assert (voltage**2/2)**exponent.denominator > Q(10)**exponent.numerator
            full = voltage**2/min(Q(s['r_ff']), Q(s['z_nominal']))*MARGIN
            for point in [30, 60]:
                volume = Q(point, 100)**3
                native = [(dt, max(Q(0), Q(math.floor(volume*level*32768)-1, 32768)))
                          for dt, level in phases]
                active = 9*sum(dt for dt, _ in native)
                amplitude_time = 9*sum(dt*a for dt, a in native)
                dose = full*amplitude_time
                # ln(10)/10 < 231/1000. Larger slope gives a rigorously
                # smaller necessary dose and loss than the logarithmic one.
                slope_upper = Q(231, 1000)*full*max(a for _, a in native)*active
                slope = math.log(10)/10*float(full)*float(max(a for _, a in native))*active
                for model, prefix in [('ff', ''), ('vi', 'vi_')]:
                    rc, rm, tc, tm = [Q(s[prefix+k]) for k in
                        ['tr_coil', 'tr_magnet', 'tau_coil', 'tau_magnet']]
                    for ceiling, limit in [('hard', Q(s['t_limit'])),
                                           ('working', Q(s['t_limit'])-Q(s['t_headroom']))]:
                        maximum = (tc+tm+540)*(limit-ambient)/(rc+rm)
                        minimum = dose-slope_upper*budget
                        accepted_budget = Q(budget) + (Q(1, 1000000) if budget else 0)
                        accepted_minimum = dose-slope_upper*accepted_budget
                        necessary = (dose-maximum)/slope_upper
                        # The gap also survives discarding ALL ASAHI margin:
                        unmargin_minimum = minimum/MARGIN
                        exact = dict(full_power_W=full, no_loss_dose_J=dose,
                                     minimum_dose_J=minimum,
                                     maximum_model_dose_J=maximum,
                                     necessary_mean_loss_db=necessary,
                                     minimum_dose_without_margin_J=unmargin_minimum,
                                     necessary_mean_loss_without_margin_db=(dose/MARGIN-maximum)/(slope_upper/MARGIN),
                                     coefficient_ratio_ceiling=maximum/minimum if minimum > 0 else Q(0))
                        rows.append(dict(profile=name, speaker=section[8:], volume_percent=point,
                            budget_db=budget, model=model, ceiling=ceiling,
                            profile_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                            active_s=active, span_s=540,
                            **{k: float(v) for k, v in exact.items()},
                            exact={k: str(v) for k, v in exact.items()},
                            logarithmic_necessary_mean_loss_db=(float(dose)-float(maximum))/slope,
                            logarithmic_minimum_dose_J=float(dose)-slope*budget,
                            obstruction=minimum > maximum,
                            obstruction_with_test_tolerance=accepted_minimum > maximum,
                            obstruction_without_margin=unmargin_minimum > maximum))
    for name in ['j413', 'j616']:
        row = next(r for r in rows if r['profile'] == name and r['speaker'] == 'Left Tweeter'
                   and r['volume_percent'] == 60 and r['model'] == 'vi' and r['ceiling'] == 'hard')
        assert row['obstruction'] and row['obstruction_without_margin']
        assert row['obstruction_with_test_tolerance']
        assert Q(row['exact']['necessary_mean_loss_without_margin_db']) > Q(row['budget_db'])+Q(1, 1000000)
    return rows


if __name__ == '__main__':
    print(json.dumps(dict(scope='No gate-closing PCM-only universal bound from the existing bounded-current premises; '
                         'does not rule out a future independently qualified physical contract.',
                         witness_checks=verify_witnesses(),
                         conditional_passivity=verify_conditional_passivity(), rows=profile_rows()), indent=2))
