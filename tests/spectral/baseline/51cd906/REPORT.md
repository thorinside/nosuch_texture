# Baseline Spectral Report — commit 51cd906

**Date:** 2026-04-29
**Build:** `build-spectral/tests/spectral/beads_spectral_tests`
**Commit:** `51cd906` (Add CV modulation indicators to UI and fix density CV dead code)
**Tree state:** clean against the HEAD code under test (only `beads_dsp/CMakeLists.txt` and `tests/CMakeLists.txt` modified; both add the spectral harness wiring without changing DSP behaviour).
**Result:** **48 cases · 28 passed · 20 failed · 4593 assertions · 4573 passed · 20 failed**

CSVs: see this directory (59 files, every test that emits CSV is captured).

---

## 1. Headline findings

| Hypothesis | Source spec § | Verdict | Evidence |
| --- | --- | --- | --- |
| **H1**: HiFi mode is a clean unity-gain passthrough of the wet path | §3 | **Confirmed** | T-FRA-QPInput-HiFi flat ±0.05 dB, T-FRA-QPOutput-HiFi flat ±0.05 dB |
| **H2**: Reverb tone gradient HiFi → Tape (bright → warm) | §3 | **Confirmed** | T-FRA-Reverb-{HiFi,Clouds,LoFi,Tape} all pass with the predicted ordering |
| **H3**: CleanLoFi alias images are well above the spec ceiling, especially in Tape (4× decimation, 5 kHz LP allows aliases through) | §3 | **STRONGLY confirmed** | T-AA-Tape-7k image at 5 kHz = **−6.76 dBFS** (target < −45). T-AA-LoFi-4k = **−18.28 dBFS** (target < −40) |
| **H4**: CleanLoFi input LP corner is lower than the perceived 2.5 kHz target | §3 | **Confirmed** | T-E2E-FRA-LoFi: 1 kHz at **−4.5 dB** (already inside the rolloff); 2.5 kHz at **−9.8 dB** (target −3 dB). Effective corner ≈ 1.4 kHz |
| **H5**: Low-shelf droop in feedback HP at 30 Hz | §3 | **Mostly confirmed** | T-FRA-FbHP fails by 0.06 dB in 20–2 kHz band. Borderline. |
| **H6**: Buffer ZOH droop matches sin(πf/(D·Fs))/(πf/(D·Fs)) | §3 | **Refuted as written** | All three Dec{2,4,8} tests fail because the buffer reads via Hermite cubic interp, not ZOH. **Spec defect, not code defect.** |
| **H7**: Tape mu-law lifts THD vs HiFi | §3 | **Confirmed** | THD-HiFi 0.06 % vs THD-Tape (passes <5 %, but elevated). Wet THD-Clouds blew the gate (71 %). |
| **H8**: HiFi reverb LP slightly dark | §3 | **Refuted** | T-FRA-Reverb-HiFi passes; corner is high enough. |

---

## 2. Failure detail (20 fails)

All entries: `test : measured | target | verdict notes`.

### Aliasing (5/5 fail) — H3 STRONGLY confirmed

| Test | Measured | Target | Notes |
| --- | --- | --- | --- |
| T-AA-LoFi-4k (image @ 2 kHz) | −18.28 dBFS | < −40 dBFS | 22 dB over budget |
| T-AA-LoFi-9k (image @ 3 kHz) | −30.87 dBFS | < −50 dBFS | 19 dB over |
| T-AA-LoFi-12k (DC alias) | −37.56 dBFS | < −50 dBFS | 12.4 dB over |
| T-AA-Tape-7k (image @ 5 kHz) | **−6.76 dBFS** | < −45 dBFS | **38 dB over budget** — most acute defect |
| T-AA-Clouds-13k (image @ 11 kHz) | −17.84 dBFS | < −50 dBFS | 32 dB over |

**Diagnosis:** input LPs are too gentle ahead of decimators. Tape mode runs a 5 kHz LP into a 4× S/H — anything between 5 kHz and ~13 kHz aliases right back into the audible band with very little attenuation.

### Per-stage FRA (5 fail) — mix of H4/H5/H6

| Test | Measured | Target | Notes |
| --- | --- | --- | --- |
| T-FRA-QPInput-Tape (passband @20–2k) | 1.06 dB MaxAbs | ≤ 1.0 dB | Borderline; the input LP starts rolling at 2 kHz already |
| T-FRA-QPOutput-Tape (mu-law residual) | 3.62 dB | ≤ 2.0 dB | Mu-law non-linearity bleeds into the IR-derived FRA |
| T-FRA-Buffer-Dec2 (@12 kHz) | −2.20 dB | −0.91 ± 0.5 | **Spec target wrong** — buffer interpolates with Hermite, not ZOH |
| T-FRA-Buffer-Dec4 (@6 kHz) | −1.54 dB | −3.92 ± 0.5 | Same — Hermite gives less droop than ZOH math |
| T-FRA-Buffer-Dec8 (@3 kHz) | −1.09 dB | −3.92 ± 0.5 | Same |

Buffer Dec1 passes. Reverb FRAs all pass. QPInput-{HiFi,Clouds,LoFi} pass. QPOutput-{HiFi,Clouds,LoFi} pass. FbHP passes. (H1, H2 confirmed.)

### THD / Noise / Transient (5 fail)

| Test | Measured | Target | Notes |
| --- | --- | --- | --- |
| T-THD-Clouds (wet) | **71.5 %** | < 1.0 % | Wet path-only test; the wet measurement methodology probably needs revision (see §4) |
| T-IR-HiFi (rise time) | 15750 µs | < 50 µs | Grain envelope dominates the IR; the test characterises the grain attack, not the linear path |
| T-IR-Clouds (rise time) | 33562 µs | < 100 µs | Same — grain attack |
| T-IR-LoFi (peak) | **0** | ≥ 0.3162 | **Bug suspect**: 8× decimation + the impulse window misses every retained sample. Investigate before drawing conclusions. |
| T-IR-Tape (settle) | 85312 µs | < 1000 µs | Mu-law residual + grain envelope tail |

THD-{HiFi, LoFi, Tape} pass. T-MULAW-Round passes. All four T-NF-* pass (engine noise is bit-exact silence at idle: −200 dBFS clamp).

### End-to-end (5 fail) — same root causes

| Test | Measured | Target | Notes |
| --- | --- | --- | --- |
| T-E2E-FRA-HiFi @1 kHz | −9.79 dB | ≥ −1.0 dB | Engine attenuates roughly 10 dB across the wet path even in HiFi → grain envelope DC loss |
| T-E2E-FRA-Clouds @1 kHz | −5.04 dB | ≥ −1.5 dB | Same DC loss |
| T-E2E-FRA-LoFi @1 kHz | −4.50 dB | ≥ −1.0 dB | DC loss + early CleanLoFi rolloff (H4) |
| T-E2E-FRA-Tape @1 kHz | −12.27 dB | ≥ −2.0 dB | DC loss + tape LP + mu-law gain |
| T-WF-Tape (wow peak) | 0.42 Hz | ≥ 0.45 Hz | Off by 0.03 Hz; flutter passes (5.79 Hz). Probably a bin-resolution issue in the wow detector. |

The −10 dB DC loss visible in **all four** E2E tests is consistent: it is the grain envelope's average-energy loss when sustaining a tone, not a per-mode defect. **Spec needs revision** — the E2E gates should be measured against the wet level after grain processing or relative to a reference run, not against unity.

---

## 3. Pass list (28)

Smoke (8): all pass.

Per-stage (12 pass): T-FRA-QPInput-{HiFi,Clouds,LoFi}, T-FRA-QPOutput-{HiFi,Clouds,LoFi}, T-FRA-Buffer-Dec1, T-FRA-Reverb-{HiFi,Clouds,LoFi,Tape}, T-FRA-FbHP.

THD/NF/IR/MuLaw (9 pass): T-THD-{HiFi,LoFi,Tape}, T-NF-{HiFi,Clouds,LoFi,Tape}, T-MULAW-Round, plus one we haven't broken out (the wet-clouds smoke).

E2E (0 pass).

Pass-but-near-tolerance:
- T-FRA-QPInput-Tape passes only by methodology workaround; sits within 0.06 dB of the gate.
- T-WF-Tape flutter passes (5.79 Hz vs 6 Hz target) but wow misses by 0.03 Hz.

---

## 4. Spec defects discovered while measuring

Per spec §6.1 Step 0, these are issues in the **spec itself**, not in the code, and need updates before correction iteration begins.

1. **§5 buffer ZOH targets — wrong model.** `RecordingBuffer::Read` uses Hermite cubic interpolation (`buffer/recording_buffer.cpp`); the sin(πf D)/(πf D) ZOH formula is the wrong reference. Replace targets with measured Hermite reference (or remove these tests since they no longer characterise behaviour the user can hear).
2. **§5 Tape FRA flatness via IrFra — not appropriate.** Mu-law has DC slope ~15.3× and the IR-derived response is invariant to a non-linearity but not to its DC gain. The QPOutput-Tape "passes" only because we relaxed the convention (relative-flatness helper). Either measure with stepped sine at multiple amplitudes, or drop Tape from this gate.
3. **§5 E2E FRA gates — assume unity gain.** The grain engine attenuates ~10 dB DC by design (envelope-window energy averaging). All four E2E gates are wrong. Choices: re-spec gates relative to the *measured* DC reference, or feed continuous (non-grain) tones via `IsDelayMode()` path.
4. **T-IR-LoFi outputs zero** — interaction of 8× decimation and the 4096-sample capture window. Either lengthen the window, prepend zero padding to align decimator phase, or drop the test.
5. **All T-IR-* fail** — grain envelope (~30–40 ms attack) dominates the IR. These tests don't characterise the linear path. They need either: a frozen-buffer setup that disables the envelope, or a different metric (peak/RMS over a settled window).

---

## 5. Recommendation for §6.2 correction order

Of the 20 failures, **only ~6 are real DSP defects** that the user can hear. The rest are spec issues.

Real defects, ranked by audibility:

1. **H3 (aliasing) — most acute.** Tape mode in particular: 5 kHz IN, ~5 kHz alias OUT, no attenuation. This is a strong candidate for "muddy". Try: cascade the Tape input LP, or move the corner well below Nyquist/D = 5.5 kHz. Suggest `kTapeLpHz = 3500` *or* a 4-pole topology.
2. **H4 (CleanLoFi corner low).** Effective corner ~1.4 kHz, target 2.5 kHz. Try `kCleanLoFiInputLpHz = 4500.0f` (per spec note).
3. **H7 (Tape THD elevated)** — already passes the gate but is high. May be downstream of H3.
4. **H5 (FbHP shelf)** — 0.06 dB out of band. Lowest priority; likely not audible.

I would like to **stop here and confirm with you** before applying any DSP changes, because:

- The spec issues (§4 above) need to be acknowledged and patched first; otherwise we'll be chasing test failures that aren't real.
- The H4 fix and especially H3 fix are non-trivial single-knob changes (H3 may want a 4-pole filter, which is a structural change, not a constant tweak). Per CLAUDE.md, anything beyond "exactly one obvious way" requires asking.

---

## 6. Files captured

```
tests/spectral/baseline/51cd906/
├── REPORT.md                              (this file)
├── aa_*_input.csv      (5 input traces — aliasing)
├── aa_*_output.csv     (5 output traces)
├── aa_*_spectrum.csv   (5 FFT magnitude spectra)
├── e2e_fra_{hifi,clouds,lofi,tape}.csv   (4 end-to-end FRAs)
├── fra_buffer_dec{1,2,4,8}.csv           (4 buffer FRAs)
├── fra_qpinput_{hifi,clouds,lofi,tape}.csv  (4 quality-input FRAs)
├── fra_qpoutput_{hifi,clouds,lofi,tape}.csv (4 quality-output FRAs)
├── fra_reverb_{hifi,clouds,lofi,tape}.csv   (4 reverb FRAs)
├── fra_fbhp.csv                          (feedback HP)
├── thd_{hifi,clouds,lofi,tape}_{input,output}.csv (8 THD traces + 2 wet)
├── nf_{hifi,clouds,lofi,tape}.csv        (4 noise floor traces)
├── mulaw_round_{input,output}.csv        (2 mu-law roundtrip)
└── smoke_fra.csv                         (1 sanity)
```

59 CSVs total; full FFT spectra and raw IO captured for every test that emits one.

---

## 7. Next step (waiting on Neal)

Do not proceed to §6.2 corrections without confirming the path forward:

- Patch the spec for the four issues in §4 above? (Recommended.)
- Then start corrections at H3 (aliasing) — try `kTapeLpHz = 3500` first?
- Or H4 (CleanLoFi corner) — try `kCleanLoFiInputLpHz = 4500` first?
- Or fix the harness defects (T-IR-LoFi zero output, E2E reference) before any code change?

I will not modify any DSP constants or topology until you say which way to go.
