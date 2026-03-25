# No Such Texture — Disting NT Plugin

A granular texture synthesizer for the Expert Sleepers Disting NT, based on the Mutable Instruments Beads manual. Records incoming audio into a circular buffer and replays it through overlapping grains with independent pitch, position, duration, and envelope controls. da6e42c (Add .scl scale quantization to pitch path)

## Controls

### Physical Controls (Custom UI)

| Control | Function |
|---------|----------|
| Pot L | Time |
| Pot C | Size (bipolar: CCW = reverse, center = short, CW = long/delay) |
| Pot R | Shape |
| Pot L press | Reset Time to center |
| Pot C press | Reset Size to center |
| Pot R press | Reset Shape to center |
| Encoder L | Density (turn) |
| Encoder L press | Toggle Freeze |
| Encoder R | Pitch in semitones (turn) |
| Encoder R press | Seed (momentary gate) |

### Parameter Pages

**Grain** — Time, Size, Shape, Pitch, Density

**Mix** — Feedback, Dry/Wet, Reverb, Macro>Fdbk, Macro>D/W, Macro>Verb

**Attenurandomizers** — Time AR, Size AR, Shape AR, Pitch AR

**Mode** — Freeze, Trigger mode, Quality, Input gain, Stereo input, MIDI channel

**Routing** — Input L/R, Output L/R + modes, all CV inputs, Macro CV

## Parameters

### Grain Controls

**Time** (0.0 - 1.0, default 0.5)
Sets the read position in the recording buffer. Fully CCW reads the most recent audio (near the write head). Fully CW reads the oldest audio in the buffer. In delay mode, Time selects the actual delay time as a multiple of the base delay time set by Density.

**Size** (-1.0 to 1.0, default 0.0)
Controls grain duration with an exponential mapping from 10ms to 4 seconds. Negative values play grains in reverse. At the maximum positive value (+1.0), the engine switches to delay mode. The center position produces the shortest grains.

**Shape** (0.0 - 1.0, default 0.5)
Controls the grain envelope. Below 0.5: blends from rectangular/trapezoidal to a smooth Hann window. Above 0.5: envelope becomes increasingly asymmetric (attack-heavy to decay-heavy). In delay mode, Shape applies a tempo-synchronized tremolo/slicer envelope on the repeats. Fully CCW for normal delay operation.

**Pitch** (-24 to +24 semitones, default 0)
Sets the grain playback rate. 0 = original pitch. Each semitone shifts the read speed by one chromatic step. Accepts 1V/oct CV.

**Density** (0.0 - 1.0, default 0.5)
Controls the grain trigger rate. In delay mode, Density sets the base delay time: at noon the delay equals the full buffer duration (~4 seconds). Turning CCW from noon shortens the delay. Turning CW from noon also shortens the delay and adds an additional, unevenly-spaced tap.

### Mix Controls

**Feedback** (0 - 100%, default 0%)
The amount of processed output signal mixed back into the input before recording to the buffer. The gain is squared internally for fine control at low settings. Each quality mode applies a different limiting scheme to the feedback path. Most noticeable in delay mode and when Time is set CCW (short delay = faster feedback buildup).

**Dry/Wet** (0 - 100%, default 50%)
Equal-power crossfade between the dry input and the wet (granular/delay) output.

**Reverb** (0 - 100%, default 0%)
Built-in reverb applied after the dry/wet mix. Higher quality modes produce brighter reverb tails.

### Attenurandomizers

**Time AR, Size AR, Shape AR, Pitch AR** (-100% to +100%, default 100%)
Each attenurandomizer modulates its corresponding grain parameter. When a CV input is connected, the AR knob attenuates (scales) the CV signal. When no CV is connected, the AR knob adds random modulation to each grain trigger, with the knob controlling the depth of randomization. Positive = unipolar random, negative = bipolar random.

### Mode / Config

**Freeze** (Off / On)
Stops recording and loops the current buffer content. In delay mode, Time selects which slice is played and Size controls the loop length. Toggle with Encoder L press.

**Trigger Mode** (Latched / Gated / Clocked / MIDI)
- **Latched**: Grains trigger continuously at the rate set by Density.
- **Gated**: Grains only trigger while the Seed gate input is high.
- **Clocked**: Each rising edge on the Seed input triggers a grain. Density subdivides the clock period. MIDI clock ticks (24 ppqn) also inject gate pulses in this mode, so an external MIDI clock can drive the grain triggers alongside or instead of CV.
- **MIDI**: MIDI Note On triggers grains. The note number sets the pitch offset (C4 / note 60 = original pitch). Velocity scales grain amplitude. While a note is held, Density controls the repeat rate (same behavior as Gated mode). Note Off stops new grain generation.

**Quality** (HiFi / Clouds / Clean LoFi / Tape)
Selects the audio processing character:
- **HiFi**: Clean pass-through, hard-clip feedback limiting.
- **Clouds**: Input LP at 14kHz, 12-bit quantization on output, soft-clip feedback.
- **Clean LoFi**: Output LP at 10kHz on wet path only, asymmetric soft-clip feedback.
- **Tape**: Input LP at 10kHz, mono sum, mu-law compression/expansion (mu=64), subtle wow/flutter pitch modulation (~0.5Hz wow, ~6Hz flutter), tape hiss.

**Input Gain** (-inf to +20dB, default -inf/auto)
At the minimum position (-inf), auto-gain is active: quiet signals are boosted up to +32dB, loud signals stay near unity. Moving away from -inf sets a fixed manual gain. The manually-set gain is applied until the parameter is returned to -inf.

**Stereo Input** (Mono / Stereo)
When set to Mono, only the left input is used (duplicated to both channels internally). When set to Stereo, both inputs are recorded independently.

**MIDI Channel** (All / 1-16, default All)
Selects which MIDI channel the plugin responds to. "All" receives on all channels (omni mode). Set to a specific channel to filter out messages on other channels.

### CV Inputs

All CV inputs expect +/-5V signals. Time, Size, Shape, and Density CVs are normalized to +/-1.0 (divided by 5V). Pitch CV follows 1V/oct (multiplied by 12 semitones/volt). Freeze and Seed gate inputs use 1V rising / 0.5V falling hysteresis thresholds.

### MIDI

The plugin responds to the following MIDI messages:

- **Note On/Off**: In MIDI trigger mode, Note On starts grain generation and Note Off stops it. Note number sets pitch (C4 = original pitch), velocity scales amplitude.
- **CC64 (Sustain Pedal)**: Value ≥ 64 enables freeze, < 64 disables. OR'd with the Freeze parameter and Freeze CV gate, so any source can activate freeze independently.
- **MIDI Clock (0xF8)**: In Clocked trigger mode, each MIDI clock tick (24 per quarter note) injects a gate pulse. Works alongside CV gate input — either source can trigger grains.

The NT host already provides generic MIDI-to-parameter mapping, so CC control of knob parameters is handled at the system level without plugin-specific mapping.

### Macro CV

A single CV input that can simultaneously modulate Feedback, Dry/Wet, and Reverb. The three Macro depth parameters (Macro>Fdbk, Macro>D/W, Macro>Verb) set how much the Macro CV affects each target. The Macro CV is clamped to 0-1 (unipolar, 0-5V range).

## Wavetable Mode

When both audio inputs remain silent for 10 seconds, Texture automatically crossfades into a built-in wavetable synthesizer. Pitch controls the oscillator frequency and Feedback selects the waveform bank. Audio input reactivates granular mode with a smooth crossfade.

## Display

The custom OLED display shows:
- **Top row**: Quality mode, trigger mode, current Density and Pitch values.
- **Middle row**: Bar graphs for the three pot-controlled parameters (Time, Size, Shape).
- **Grain activity**: A row of dots showing the number of active grains (up to 30 max).
- **Footer**: Active grain count, input level meter, and FROZEN indicator when freeze is engaged.

## Technical Details

- 4-second stereo recording buffer at 48kHz (scales with sample rate)
- Up to 30 simultaneous grains with overlap normalization
- Hermite interpolation for sub-sample buffer reads
- Grain parameters are fixed at trigger time (position, pitch, size, shape, pan)
- Feedback path: output captured before reverb, HP filtered at 20Hz (DC removal), mixed into input before buffer write
- Mode crossfade (64 samples) when switching between grain and delay engines
- Quality mode crossfade (64 samples) on input and output paths
- Soft takeover on pots to prevent parameter jumps
- All mix parameters smoothed to prevent zipper noise

## Building

Requires the Disting NT SDK. The plugin uses a unity build (`NT/beads_plugin.cpp` includes all DSP source files) for cross-translation-unit inlining on the ARM Cortex-M7 target.

### Desktop Tests

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build
./build/tests/beads_tests
```
