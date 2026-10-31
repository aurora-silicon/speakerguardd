# Safe-tail projection (review draft)

This change removes an instantaneous-jump overestimate in playback grant
admission. Previously the safe tail used the invariant rectangle
`M=max(Tm,Ta+Ps*Rm)`, `C=max(Tc,M+Ps*Rc)`. When a magnet is hot and its coil
has cooled, C immediately charges the coil for the entire initial magnet
temperature plus the coil's safe-power equilibrium rise. The magnet is
actually cooling during the coil's rise.

The replacement uses the thermal model's constant-power extremum calculation,
already used by `hold()` and onset projection. The coil baseline is the maximum
of its initial temperature, safe equilibrium, and interior extremum over the
infinite safe tail. The magnet baseline remains the maximum of its initial
temperature and safe equilibrium. Equal and near-equal poles retain the
existing conservative drive bound. No infinite interval is integrated.
The old rectangle remains a second sufficient bound; taking the smaller of
the two prevents the extremum's upward rounding allowance from adding loss.

## Complete energy reserve

For `b=Rm/taum`, `k=max(Rc/tauc,b)`, nonnegative impulse response kernels are
bounded by b and k. By linear superposition, adding any nonnegative energy
measure of total dose E to the safe-tail solution increases the magnet by at
most bE and the coil by at most kE, at every future instant. Therefore the
available dose remains

```
min((working_ceiling-margin-coil_safe_peak)/k,
    (working_ceiling-margin-magnet_safe_peak)/b).
```

Every queued joule, unspent old credit, and downstream residual joule is still
subtracted from that dose. All can arrive simultaneously at any future time.
No bounded sample rate, power density, contiguous playback, fresh lease or
mandatory cooling interval is introduced. The uniform residual/storage proof
and joint safe baseline in `transfer-certificate.md` still apply. Measured
observers, integer rounding, source and delivered windows, takeover history,
qualification, state restore and fallback onset protection are unchanged.

## Validation and release

The hot-magnet witness must recover headroom compared with the old rectangle.
An independent constant-power oracle checks both nodes and continuous extrema
after full-dose arrivals at five delays over a temperature grid, equal and
near-equal poles, both pole orderings and three safe powers. Existing arbitrary
burst and infinite-tail tests remain. The full suite, unchanged listening
assertions and both continuous campaigns must be recorded on this source.

The existing linear transfer fixture's necessary-dose lower bounds still
exclude some listening budgets. Any reduction in synthetic loss is partial
progress until both strict tests pass, physical transfer is qualified and the
historical 63 replay rows have justified native inputs. This patch supplies no
physical certificate and activates no hardware route. Physical helpers remain
NONE; existing draft release/package changes remain unmerged.

Rollback is the two source files at RC commit
`221c8bfa2ac4aa43a1f9075420a95c1d12bce4b0`. No target or package changes are
made by this task. Independent safety review remains required before release.
