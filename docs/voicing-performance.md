# Measuring voicing processing

The offline renderers read and write files and never open an audio device.
Use the same compiler, target, features, input, tuning bundle, controls and
block size for each side of a performance comparison. In particular, use
`sgv2-core/lowflow-conservative` on both sides when measuring that law.

`voicing-render --profile profile.json` adds per-node timings to a normal
render. Each node's interval includes routing its input and processing the
box. The report contains only node indices, kinds and timings, without
serializing tuning parameters. Block totals include the observer's clock
reads. File I/O, compilation and chain construction are outside the timers.

`voicing-pre-render --profile profile.json` times the complete deployed LV2
callback on a compiled blob, including its input and output copies. These
clock reads surround the callback; the plugin performs no profiling work.
Use this report for the total cost, and the node report to locate hot paths.
Both reports include frame counts and nanoseconds. Convert a total to the
mean time per 512-frame quantum with `total_ns / frames * 512`. Measure on
the intended CPU; QEMU timings describe emulation, not hardware performance.

Compare rendered output bytes before interpreting a speedup. Check all scan
programs, channel arrangements, controls and supported sample rates. Retain
hashes of the inputs, binaries and outputs so the comparison is reproducible.
No tolerance is needed for the optimizations below:

* The conservative LowFlow law does not consume its detector's masking or
  velocity state. That build omits the unused masking FIRs and energy
  updates. Gain attack/release, system-volume updates, muting, sanitization
  and the delay still run in their original order. The detector law retains
  its processing. The feature is fixed when compiling, so detector history
  cannot become observable through a runtime mode switch.
* The multiband gain interpolator visits only the phase's nonzero input
  slots. The same coefficients and stored gains are multiplied and summed
  in the same order, from a positive-zero accumulator. It omits products
  with the inserted zeros without reassociating additions or introducing
  fused operations. Its phase persists across process calls and resets
  with the detector cadence. A direct zero-stuffed FIR oracle checks output
  bits at all four reduction factors and several sample rates, including
  signed zeros and subnormal inputs.

The normal `Chain::process` specializes the observation callback to an empty
closure. It does not read clocks or allocate profiling storage.
