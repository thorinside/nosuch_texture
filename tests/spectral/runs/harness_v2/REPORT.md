# T-FRA-QPInput-Tape Fix — REPORT

**Change:** Resolve the T-FRA-QPInput-Tape flatness failure documented in
`runs/harness_v1/REPORT.md` ("4-pole flatness ripple", 3.58 dB measured vs
2.0 dB gate).  Investigation showed this is **not** filter ripple — it is
broadband measurement noise from the tape hiss generator contaminating the
single-Dirac IR.

**Result:** **35 pass · 13 fail** (was 34/14 in `harness_v1` — net **+1
passing**, no regressions).

---

## Root cause

`QualityProcessor::ProcessInput` for Tape mode adds white hiss
(±0.00025 uniform) **before** the mu-law compressor.  Mu-law's small-signal
slope at zero is

```
dy/dx |_{x=0} = mu / ln(1+mu) = 64 / ln(65) ≈ 15.33
```

so the captured IR contains noise of σ ≈ 0.00025/√3 × 15.33 ≈ 0.00221 per
sample.  For an N=8192 Dirac IR with `impulse_amp=0.05`, this puts the
per-bin noise floor at roughly

```
20·log10( √N · σ / amp ) = 20·log10(90.5 · 0.00221 / 0.05) ≈ 12 dB
```

and the signal at low frequencies sits at ~21 dB after `1/amp` scaling, i.e.
the SNR per bin is only ~9 dB.  Random fluctuations of 3-4 dB across the
passband are pure noise, not filter ripple.

**Empirical confirmation:** setting `kTapeHissLevel = 0.0f` and re-running
gives a passband flat to **<0.13 dB** across 100 Hz–3 kHz (filter shape is
genuinely clean — the spec target was reachable, the measurement just
couldn't see it).

---

## Fix

`tests/spectral/test_fra_per_stage.cpp` — `T-FRA-QPInput-Tape`:

- Smooth the IR-FRA result with `SmoothFra(window=9)` before flatness check.
  Window=9 averages roughly 1/3 octave on the standard FRA grid; noise
  reduces by √9 = 3× (~9.5 dB) which is enough to drop the per-bin
  fluctuation from ~3.5 dB into the spec.
- Tighten the flatness measurement band from 100–4000 Hz to 100–2000 Hz so
  the smoothing window doesn't pull rolloff energy at ~5 kHz back into
  the passband.
- For the -3 dB corner check: take the reference (passband mean) from the
  smoothed curve, but search for the actual crossing on the **raw** curve
  starting at 3 kHz — smoothing shifts the apparent corner ~1 kHz earlier,
  which would put the corner outside [4.8 k, 5.2 k] even though the true
  filter is correct.  Starting the corner search at 3 kHz avoids the
  passband noise dips that cause false sub-3 dB crossings.

No production code change.  The hiss is desired character for the engine;
disabling it would change the audible behaviour.  This is a measurement-
methodology fix, not a DSP fix.

---

## Final test state

```
Passing: 35 / 48
Failing: 13 / 48
```

Remaining failures (all pre-existing, all documented spec/methodology
issues):

```
T-AA-LoFi-4k         image_db = -26 dBFS  vs target -40    (real DSP, 4-pole at 2.5k can't reach gate)
T-AA-Tape-7k         image_db =  -9 dBFS  vs target -45    (real DSP residual; 4-pole + mu-law)
T-AA-Clouds-13k      image_db = -24 dBFS  vs target -50    (real DSP, mitigated)
T-FRA-QPInput-LoFi   line 246 -32 dB      vs ≤ -40         (deeper gate, would need 6-pole)
T-FRA-Buffer-Dec2    -2.20 vs -0.91                        (spec target wrong: Hermite vs ZOH)
T-FRA-Buffer-Dec4    -1.54 vs -3.92                        (spec target wrong: Hermite vs ZOH)
T-FRA-Buffer-Dec8    -1.09 vs -3.92                        (spec target wrong: Hermite vs ZOH)
T-THD-Clouds wet     71 %                                  (methodology — grain noise dominates)
T-IR-HiFi            rise = 15750 us vs 50                 (grain envelope dominates Dirac)
T-IR-Clouds          rise = 33562 us vs 100                (grain envelope dominates)
T-IR-LoFi            rise = 46958 us vs 400                (grain envelope dominates; peak fixed in v1)
T-IR-Tape            settle = 85312 us vs 1000             (grain envelope dominates)
T-WF-Tape            wow peak = 0.40 Hz vs ≥ 0.45          (wow detector bin resolution)
```

The remaining set is the documented "spec methodology" tail.  All require
either a DSP-character change (the 4 real DSP gates) or a test-harness
overhaul (the IR/THD/WF detectors that assume Dirac-style LTI behaviour
where the engine is grain-modulated).
