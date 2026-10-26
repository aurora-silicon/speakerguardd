# Continuous limit acceptance

These deterministic model inputs are retained from the paired public deployed
source and smooth-handoff hotfix audit. The hash is FNV-1a of `to_rust()`.
There are 144 long cases, 20,000 V/I seeds (7312900001..7312920000), and
10,000 legacy seeds (7313000001..7313010000). All 865 distinct cases that
failed either source are default regressions, including all 17 new crossings.
The complete acceptance campaign is `all_30144_continuous_cases_stay_within_limits`
(run it with `--ignored --exact` and its full module path). It checks the
original properties and every continuous coil/magnet maximum of each active
model. Inactive model rows are retained as diagnostics. No armed-state mask
applies to the continuous limit. Original Euler/RK4 assertions stay intact.

`historical-limits.json` retains all 74 archive entries: 72 shared rows and two
explicit packaged-Neo rows from a different source/fixture. Generated model
fixtures contain no recordings or private device identity. They use the
repository's MIT license. Raw recordings remain private.
