# XTC initialization and reset

The isolated XTC AudioUnit has three filter states. ClassInfo first constructs a filter with default controls at the stream rate, restores the saved controls into a second filter, and leaves a cached construction-rate filter as the settled endpoint. A fixed settled FIR matches steady measurements but misses the first burst windows.

The runner carries two queued linear crossfades. Each uses the saved `CrossfadeDuration`, converted to frames at the stream rate. A completed fade holds its endpoint until the next render boundary; renders larger than 512 frames are subdivided. The startup filters use the same spectrum remapping, windowing and DC restoration as the settled filter. Their controls come from the saved state or the unit's construction defaults, with no measured taps or fitted correction gains.

Reset is distinct from fresh initialization: it clears history and selects the restored-controls filter. If the cached endpoint is still queued, its crossfade survives and starts from the restored filter. If it has already become active, reset holds the restored filter. Repeated resets before rendering do not consume the queue. Crossover orders remain continuous controls in the restored filter.

Deployment node tag 21 carries the settled filter, both startup filters and the fade duration. Existing fixed-filter nodes retain tag 6 and their original encoding. Allocation occurs during construction and decoding; processing and reset allocate nothing.

Synthetic tests cover fade endpoints, render boundaries, large slices, reset selection, legacy and extended encoding, parameter validation and allocation. Differential validation uses the unchanged absolute RMS gate on steady half-clips and 10 ms transient windows, without fitted latency alignment. ControlFreak's saturation reconstruction remains a separate unresolved law.

The isolated box renderer supports XTC `--q` and `--reset-at` probes so initialization and reset can be compared at the same render boundaries as the AudioUnit.
