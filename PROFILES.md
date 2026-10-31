# Public profile provenance and format

The new files are drafts for offline review and future machine validation.
They do not certify safety, acoustic performance or macOS parity. The
qualification gate, kernel interlock and existing qualified profile are
unchanged. Installing them does not opt a machine in or enable a service.
See the [README inventory and contribution procedure](README.md#speaker-profile-coverage).

## Public sources and licences

The profile data is adapted from the Asahi Linux contributors'
[speakersafetyd 2.0.2](https://github.com/AsahiLinux/speakersafetyd/tree/8c519e9594b6635d8a5e134d36dcb8ab5ea4a00f),
commit `8c519e9594b6635d8a5e134d36dcb8ab5ea4a00f`. That revision is MIT
licensed. Its complete licence notice is retained in
[conf/LICENSE.asahi](conf/LICENSE.asahi) and installed alongside the data.
Later GPL releases were not used as a source of adapted configuration.
Every new file carries Asahi and Aurora attribution and a pinned link to
the relevant source profile. Upstream aliases are expanded into regular
files so each installed draft has its own explicit qualification flag.

The following public references provide context and cross-checks:

* [Asahi device list](https://asahilinux.org/docs/hw/devices/device-list/)
  for board names and marketing identities. These are distinct from ALSA
  card names. Do not assume the filename selects every suffixed card ID.
* [Asahi per-machine speaker notes](https://github.com/AsahiLinux/speakersafetyd/blob/8c519e9594b6635d8a5e134d36dcb8ab5ea4a00f/docs/speakers.txt)
  for amplifier families and output roles. Its "Sense" column lists "no" or
  "unused?" for several machines whose Linux configs nevertheless supply
  V/I slots. The drafts follow the
  config maps and leave this source discrepancy for capture validation;
  they do not infer macOS feedback behaviour from it.
* [asahi-audio graphs](https://github.com/AsahiLinux/asahi-audio/tree/ab1b46928d056bb02bb4ad72d522dab57dc9efe1/firs)
  and [WirePlumber mapping](https://github.com/AsahiLinux/asahi-audio/blob/ab1b46928d056bb02bb4ad72d522dab57dc9efe1/conf/wireplumber.conf)
  for output roles and DSP channel order. A graph's playback order is not
  necessarily its V/I capture order. No DSP coefficients or code are
  copied from this GPL revision.
* [alsa-ucm-conf-asahi](https://github.com/AsahiLinux/alsa-ucm-conf-asahi/tree/1473329acab595dfef039f38c384ce16674d6b15/ucm2/conf.d/macaudio)
  for card/control names and playback channel counts. UCM names and
  `PlaybackChannels` do not establish capture format, full-scale voltage
  or coil impedance. No UCM implementation is copied.
* [macaudio machine data](https://github.com/aurora-silicon/linux/blob/070c8e4d43a40ac464fe8bd491c090688a92b63a/sound/soc/apple/macaudio.c),
  [TAS2764 gain control](https://github.com/aurora-silicon/linux/blob/070c8e4d43a40ac464fe8bd491c090688a92b63a/sound/soc/codecs/tas2764.c)
  and [TAS2770 gain control](https://github.com/aurora-silicon/linux/blob/070c8e4d43a40ac464fe8bd491c090688a92b63a/sound/soc/codecs/tas2770.c)
  for public amplifier cap and fallback facts. These are GPL kernel
  references by the Asahi Linux and Aurora contributors, used for numeric
  facts, with no implementation copied into this MIT project. No Gravity
  code or unlicensed fork data is incorporated.
* The existing public [J316](https://github.com/aurora-silicon/speakerguardd/blob/c020028b13515ba29767bdc325f3bd7c7cb9101a/conf/apple/j316.conf)
  and [J700](https://github.com/aurora-silicon/speakerguardd/blob/c020028b13515ba29767bdc325f3bd7c7cb9101a/conf/apple/j700.conf)
  profiles at the requested base supply policy fallbacks. No new macOS
  extraction, private recording or lab measurement is used.

Apple tech specs linked in the README and individual later-machine files
establish speaker counts only. They do not publish thermal constants,
impedance minima, sense slots or amplifier scaling. J575 covers both the
M3 Ultra and M4 Max identities; the M4 variant still needs its own check.
The older public kernel has no audio configuration for the later M4/M5/M6
boards. Their assumed routing is not evidence of kernel audio support.

J713 is not one of these drafts. It is inferred to have the J413's speakers
(shared chassis, ADT DC resistances), so its profile is
j413.conf on the J713's measured twelve-channel sense stream, with its
tweeters' own ADT DC resistances as z_min. Its kernel holds a locked
SN012776 at about -100 dB, as on the J413, so it keeps the J413's 99.5 dB
safe_reduction ([#10](https://github.com/aurora-silicon/speakerguardd/pull/10)).

## What the numbers mean

A file has `[Globals]`, `[Controls]` and one `[Speaker/<name>]` section per
independently modelled output. `src/config.rs` defines the accepted keys.
Upstream `z_shunt` and `a_t_35c` are not supported keys in this format
and are omitted. The model uses the explicit `a_t_20c` coefficient;
copying those other keys would not implement upstream calibration.

Comments are full lines beginning with `#`; inline comments are not used.
All explicit numeric assignments in a new draft have a source comment
immediately before them, or a shared comment for consecutive assignments.
The header resolves the source labels to full public URLs.

| Fields | Units and meaning | Draft source |
| --- | --- | --- |
| `channels`, `is_chan`, `vs_chan` | Total capture words per frame and zero-based I/V slots | The named Asahi profile, or explicitly declared sibling |
| `sense_pcm`, `period` | Capture device index and frames per processing period | Asahi `visense_pcm` renamed, and `period` |
| `sense_format` | Interleaved signed 16-bit I/V words | Asahi [capture format](https://github.com/AsahiLinux/speakersafetyd/blob/8c519e9594b6635d8a5e134d36dcb8ab5ea4a00f/src/helpers.rs), never a playback-slot count |
| `group` | Speakers linked to the same gain reduction | Asahi role groups |
| `tr_coil`, `tr_magnet` | Thermal resistances in K/W | Asahi measured-power model used provisionally for FF too |
| `tau_coil`, `tau_magnet` | Thermal time constants in seconds | Same Asahi model, not retuned |
| `vi_*` | Explicit thermal set driven by measured V*I power | Same Asahi model |
| `t_limit`, `t_headroom` | Hard coil limit and margin in degrees C/K | Asahi limits retained, including larger desktop margins |
| `z_nominal` | Published nominal impedance in ohms | Asahi, not a measured DC minimum |
| `r_ff`, `z_min` | FF reference resistance and pessimistic power-bound impedance in ohms | Lowest DC resistance in the public base J316, 2.7 ohm |
| `a_t_20c` | Resistance temperature coefficient at 20 degrees C, per K | Asahi; not a unit calibration |
| `is_scale`, `vs_scale` | Amps and volts at full-scale I/V words | Asahi, 3.75 A and 14 V |
| `amp_gain_dbv` | RMS output voltage in dBV for a full-scale sine | Public cap and 11 + 0.5 * index relation; later boards use the largest sibling cap |
| `amp_gain_index` | The amp gain control value `amp_gain_dbv` stands for | The same index; the daemon writes it (or the control's maximum, if lower) instead of a higher maximum the card may offer |
| `safe_reduction` | Kernel fallback below unlocked volume, dB | Public TI path and existing J316, 99.5 dB; unknown boards use J700's more pessimistic 20 dB |
| `max_reduction`, `t_reduction_max` | Kernel handoff threshold and thermal ramp range, dB | Existing J316, 7 and 20 dB |
| `horizon_s`, `release_s` | FF prediction and release time, seconds | Existing J316, 4.6 and 20 seconds |
| `t_ctrl_headroom` | Additional working-target margin, K | Existing J700, 5 K |
| `p_limit_1s`, `p_limit_60s` | Independent mean-power budgets, watts | Additional J700 sibling caps, 2.8154 W and 2.5 W; actual unit budgets are unknown |

Working thermal limit is `t_limit - t_headroom`; the unified target keeps
`t_ctrl_headroom` below that. Do not equate an upstream `t_limit` with the
base profiles' macOS-derived hard limits. The new files keep the lower
Asahi values instead of raising them to match the existing J316/J413.
The Asahi guard also retains those lower working limits.

The TI gain examples are index 10 -> 16 dBV, index 15 -> 18.5 dBV and
index 20 -> 21 dBV. Using 18.5 dBV on every desktop would understate the
public index-20 amplifier level. J575's notes list index 15, while the
public desktop kernel configuration caps it at index 20. Its draft uses
21 dBV to account for the larger of those published levels. A real card's
control range must still be checked before use.

When the kernel fallback is unknown (later boards and the M4 variant of
J575), `safe_reduction` uses J700's 20 dB, the highest-powered published
fallback. Assuming 99.5 dB there would underestimate energy during lost
sense or a restart if the real kernel fallback were only 20 dB. Known
older TI routes retain their documented 99.5 dB. Neither value proves
what an unverified kernel actually enforces; its fallback must be checked.

The drafts explicitly set `qualified = false`, `governor = unified`,
`guard = asahi` and `unified_caps = false`. Neither
`--experimental-unified` nor a governor override bypasses qualification.
`--allow-unqualified` is a risk opt-in and does not verify any assumption.
No udev or service changes enable these new machines automatically.

## Sibling fallbacks and their limits

For M1-M3, upstream links such as J414 -> J314, J516 -> J316,
J613 -> J413 and J615 -> J415 are public sharing decisions, expanded
without claiming that this implementation is qualified. The source link
in each file points to the original board entry, including its alias.

For later machines, the choices are:

| Drafts | Thermal and routing sibling | Conservative choice within that family |
| --- | --- | --- |
| J604, J614, J704, J714 | J504 | Same thermal constants as J314, with the lower 130 C woofer hard limit; later J504 slot order is explicit |
| J716 | J316 thermal / J616 routing | The Asahi 140 C hard limit, below the existing J616's 150 C; later J616 slot order is a declared assumption, not a measured M5 map |
| J813 | J413 | J413 thermal set, with older 13-inch Air J313's lower 120 C hard limit and larger 15 K margin |
| J715, J815 | J415 | The public 15-inch Air set, including left/right differences; not the less resistive 13-inch woofer model |
| J773, J775, J873 | J375 | Mono desktop set, with its 40 K margin and the highest public sibling amplifier cap, 21 dBV |

All new drafts use the lowest published sibling resistance, 2.7 ohm,
for both `r_ff` and `z_min`, even when `z_nominal` is higher. That increases
the FF estimate and the pessimistic charge for missing or suspect sense
relative to using the published nominal impedance. It is an assumption,
not proof that an unknown unit cannot have a lower resistance. Replace it
only with sourced, reviewed data. Where a unit has no published power budget, the drafts add the public
J700 caps of 2.8154 W over one second and 2.5 W over sixty seconds. These
are extra ceilings alongside the thermal models and Asahi guard, not
claims about the new speaker's rated power. These are the only published
budgets in the base profiles; they do not replace the thermal limits.

Thermal resistance and time constants cannot be ordered into one
universally conservative model merely by choosing maxima or minima.
A shorter time constant heats sooner but also cools sooner. These drafts
retain published two-node sets together and choose lower limits or a
larger margin where a family provides them. Borrowing a measured-power
set for FF, changing chassis, or changing amplifier routing still needs
validation. The qualification gate is essential; no sibling choice is
claimed to bound an unknown machine under all signals.

## Missing machines

M1 iMacs J456/J457 have public SSM3515 thermal data, but that profile lacks
sense slots, scales and controls. The public kernel explicitly excludes
SSM3515 from its sense stream and keeps its speakers disabled in the
machine configuration. The M3 J433/J434 and M4 J623/J624 iMacs likewise
lack a compatible public capture/tap mapping in the sources reviewed.
An S16 TI feedback map cannot represent those machines. They are listed
as skipped, rather than assigned fictional current sense. Support needs
a feed-forward tap and its amplifier voltage scale, plus appropriate
control and kernel interlock support. The [Apple iMac specs](https://www.apple.com/imac/specs/)
confirm a six-speaker system, but do not supply those missing facts.

J180 has public constants and a map even though the cited kernel keeps
its speakers disabled. Its profile is an unqualified draft, not an
instruction to enable them. Later boards with no audio route remain
unusable on that kernel even after `--allow-unqualified`. Public board
identities, rather than guessed future product names, define this inventory.
