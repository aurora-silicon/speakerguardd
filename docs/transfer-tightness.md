# Why PCM statistics cannot tighten the bounded-current contract

The existing reference premises do not imply a tighter energy contract that
can close both listening assertions. This is a universal-contract
infeasibility result. It does **not** prove that the actual loudspeakers lack
a tighter physical transfer. Identifying and qualifying that narrower plant
class would require an additional premise; it cannot be inferred from PCM
statistics or successful finite replays. No production certificate is enabled.

This argument applies to the RC at `add7f9610d68b3ff898275d3e7d1cdcd8587c47c`
with the safe-tail changes `59d4668` and `4200dd4`. It leaves all runtime code,
profiles, kernel fixtures, listening budgets and safety assertions unchanged.
The [certificate requirements](transfer-certificate.md) still apply.

## The sharp bound under the current premises

For one actually billed speaker slot let `x[n]` be the delivered, normalized
native PCM word. First grant the most favorable case: no downstream state,
no background, no delay and no calibration error. The bounded-current
premises are

```
|v[n]| <= Vmax |x[n]|,   |i[n]| <= Imax,
W = Vmax Imax,          p[n] = max(0, v[n] i[n]).
```

They imply `p[n] <= W |x[n]|`. For **every** word sequence the admissible
memoryless relation

```
v[n] = Vmax x[n],   i[n] = Imax sign(x[n]),   sign(0) = 0
```

attains equality at every frame. It is causal and dissipative: `vi >= 0`.
This is a mathematical countermodel to an inference from those premises,
not an identified speaker, nor a claim about a particular TI amplifier.
Continuity does not fix the inference: the passive continuous relation
`i(v) = Imax v / (epsilon + |v|)` approaches the same energy arbitrarily
closely on any fixed finite collection of nonzero words as epsilon decreases.

Consequently any bound derived only from these premises, even one that sees
the **entire** input sequence, must cover `W/fs * sum(|x|)`. A band partition,
crest factor, block RMS or lookahead cannot lower that supremum. The same
countermodel attains it for the waveform that produced those statistics.
Cauchy-Schwarz with only `sum(i^2) <= N Imax^2` gives
`W sqrt(N sum(x^2))`, which is at least the existing linear bound. It does
not supply the missing input-dependent current norm.

Nominal resistance values in a configuration are not, by themselves, a
time-local inequality `|i[n]| <= |v[n]|/R`, nor a certified admittance norm
over every state, gain and route. Such an independently justified inequality
would exclude the countermodel and change the premises. A steady sine at
one impedance in a test harness does not establish it for arbitrary queued
words or downstream state.

## A finite tail and runtime observations do not repair the inference

Suppose a proposed squared contract uses charged full-input coefficient F,
background P0 and bounded nonnegative storage `0 <= S <= B`. Summing its
supply inequality on N repetitions of amplitude `a > 0` requires

```
N W a/fs <= N (P0 + F a^2)/fs + B.
```

Whenever `W a > P0 + F a^2`, this fails for sufficiently large N, for every
finite B. For instance, in normalized units with `W=fs=1`, `F=201/200`,
`a=1/32`, and even `P0=F*10^-5`, the deficit is positive. This P0 grants the
entire existing linear safe-power reservation to background, favoring the
proposed contract. The exact checker exhibits a finite repetition violating
a ten-joule tail. The same argument applies to any statistic-based supply
whose average is below `W mean(|x|)` on a repeatable waveform. Reserving B
again at every grant cannot turn an unbounded deficit into a uniform tail.

Past current observations can falsify a candidate premise; they cannot
certify its future validity. Two admissible trajectories can have identical
PCM and current through any tested prefix, then choose different bounded
currents on the next queued word. The protection path must cover that
continuation before it plays. Current sensing may tighten the existing
observer after verified observations, with all timing/fault rules retained;
it does not prove a smaller universal queued-energy contract. A new bound
would need an enforced physical restriction or an authenticated certificate
covering that continuation and every transition.

The official [TAS2764 datasheet, section 8.4.2.1](https://www.ti.com/lit/ds/symlink/tas2764.pdf)
and [TAS2770 datasheet, section 8.4.3.1](https://www.ti.com/lit/ds/symlink/tas2770.pdf)
describe selectable or bypassable high-pass filtering. I do not infer from
those controls a load admittance, uniform storage bound or transition proof
for the actual routes. This is the missing inference, not a new physical
certificate. The existing TAS helpers continue to return NONE.

## The attainable linear dose already excludes both gates

Here is a necessary-dose proof independent of the grant algorithm. In each
constant-amplitude 10 ms metric cell, let digital gain be `u in [0,1]`,
analogue attenuation `c >= 0`, `q=mean(u^2)`, digital loss `d=-10 log10(q)`
and `k=ln(10)/20`. For a finite-loss cell, weighted Jensen gives

```
mean(u exp(-kc)) >= mean(u^2 exp(-kc))
                 >= q exp(-k mean(u^2 c)/q)
                 >= q exp(-k mean(c)/q)
                 >= q - k mean(c)
                 >= 1 - 2k (d + mean(c)).
```

This permits arbitrary pulsing and correlation of the digital and analogue
controls. A zero-output active cell has infinite digital loss and cannot
pass either budget. If `a_j` are lower native input amplitudes, `s_j` their
durations and `A=477 s`, the necessary linear dose at mean loss D is at least

```
F sum(s_j a_j) - (ln(10)/10) F max(a_j) A D.
```

All phases and both native formats are retained. Each input amplitude is
rounded down to S16 and reduced by one additional S16 quantum, making this
lower than either native input. The entire dB budget is allocated to the
loudest cells, even if that allocation is unrealizable. Silent-cell heat,
the safe baseline, queue/residual debt and numerical reserves are all omitted.

For the independent two-node V/I thermal model, direct integration of its
ODEs gives

```
(Rc+Rm) integral(P) = tauc Delta(Tc) + taum Delta(Tm) + integral(Tc-Ta).
```

Both nodes start at least at ambient. If neither exceeds L, the maximum
dose over H seconds is `(tauc+taum+H)*(L-Ta)/(Rc+Rm)`. Grant a cold restart
after the excluded first minute, `H=540 s`, and the hard 150 C ceiling
instead of the mandatory 140 C working ceiling. These choices all favor
feasibility.

The charged coefficient is
`F=max(v_worst,vs_scale)^2/min(r_ff,z_nominal)*1.005`. The familiar
logarithmic estimates are reproduced below for the left tweeter; the
right tweeter has identical constants.

| Profile, 60% | Budget dB | Necessary dose J | Optimistic ceiling J | Necessary mean loss dB |
| --- | ---: | ---: | ---: | ---: |
| J316 | 1 | 429.636 | 646.905 | 0.404394 |
| J413 | 1 | 369.580 | 200.672 | 1.538274 |
| J616 | 0 | 794.423 | 646.905 | 0.404394 |

J413 excludes the first listening assertion and J616 excludes the second.
The J413 strict separation also survives its existing `1e-6 dB` assertion
tolerance; J616 retains its exact-zero assertion.
The J316 row alone gives no contradiction at its 1 dB budget; this loose
necessary condition does not establish a pass. At 30%, this integral test
alone gives no contradiction for any of these profiles, including the J413
case that fails the actual dynamic listening test.

For a rationally checked strict separation, replace `ln(10)/10` by the
larger `231/1000`. The positive Taylor sum for `exp(2.31)` through degree
seven already exceeds 10, proving that replacement conservative. The exact
rational lower doses are 368.569249 J on J413 and 794.422781 J on J616,
still above their ceilings. Their rational necessary losses are at least
1.533335 dB and 0.403096 dB respectively.

The sharp physical countermodel can even omit **all** of the 0.5% ASAHI
accounting margin: set `W=F/1.005`. The rational minimum doses become
366.735571 J and 790.470428 J, still impossible. Thus the conclusion does
not rely on treating a numerical safety margin as attainable plant heat.
Without the margin, necessary losses remain at least 1.530148 dB and
0.394257 dB. No margin is actually removed from the implementation.

This is a contradiction about actual admissible thermal-model inputs in
the declared bounded-current class, not merely about conservatism of a
particular tail projection. A sound accounting contract for the same class
cannot make an impossible thermal trajectory fit. An enormous residual B
would itself have to fit the unchanged thermal reserve.

## Reproduction and scope

Run `python3 tools/check-transfer-tightness.py`. It emits exact-rational
witness checks and dose bounds for every speaker, both volume points,
both active models, and both working/hard ceilings. Floating logarithmic
figures are labeled separately; the strict contradictions use rational
arithmetic. A row without an obstruction means only that this necessary
test is inconclusive. Run `python3 tools/check-transfer-contract.py` for
the existing residual, background and transition witnesses.

The full Rust suite and the two continuous campaigns remain separate
checks. Run the 30,144-case test once with `SPEAKERGUARDD_GATE_MODEL` absent
and once with it set to `1`:

```
cargo test --locked --release -j8 -- --nocapture --test-threads=4
cargo test --locked --release -j8 harness::continuous::all_30144_continuous_cases_stay_within_limits -- --ignored --exact --nocapture
```

Preserve both listening failures and all per-speaker costs. No accepted
thermal assertion, onset reserve, independent-start tolerance, overflow
regression or safe-tail test is changed by this proof. Passing simulations
would not prove an additional physical premise. Under the present premises
the infeasibility alternative is the result; a narrower qualified physical
contract remains a separate research question. Listening budgets remain
the operator's decision.

## An additional premise that would change the result

There is a useful constructive distinction. Suppose an independently
qualified load admits nonnegative physical storage S and a certified
resistance floor R such that, in continuous time,

```
dS/dt <= v i - R i^2,    R > 0.
```

For `w=max(0,vi)`, when `vi >= 0`, completing the square gives

```
w + dS/dt <= 2vi - Ri^2 <= v^2/R,
v^2/R - (2vi - Ri^2) = (v-Ri)^2/R >= 0.
```

When `vi < 0`, `w=0` and `dS/dt <= vi-Ri^2 < 0`, so the same supply
inequality holds. Actual resistive heating `w=Ri^2` instead satisfies
`w + d(2S)/dt <= v^2/R` by the same algebra. These are conditional
squared-voltage certificates. With a stateless amplifier `v=Vmax*x`,
their coefficient is `Vmax^2/R`. A qualified stateful amplifier would
need its own squared-voltage supply proof; the storage functions must
compose and their joint reachable bound and transitions must be covered.
The feed-forward model's separate voltage-squared input also remains
an independent constraint.

The missing uniform reachable-state bound cannot be inferred merely from
R and Imax. Even the passive series R-L class has storage `L i^2/2`;
without an upper bound on L, a current bound leaves that storage unbounded
over the class. Long bounded-voltage prefixes can approach a fixed nonzero
current for every finite L. Neither the resistance floor nor current cap
alone supplies a numerical B. This example concerns possible stored heat
release as well as electrical input, rather than identifying a real load.

Thus a narrower sound contract is mathematically possible under extra
physical premises. The exact checker verifies the square identity, but
does not assert those premises for TAS/AOP or the profiles. In particular,
the impossibility proof above must not be reported as proof that no future
physical certificate can exist. It establishes that replacing the existing
contract on the strength of PCM statistics alone would weaken its guarantee.
