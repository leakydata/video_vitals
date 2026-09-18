# Overnight notes — 17/18 September 2026

You asked Codex (gpt-6-astra) to review everything, for me to implement what I judged
right, and to flag what I was unsure of. Here it is.

Everything below is committed and pushed to the private repo. Heart-rate tests
**105/105**, breathing **41/41**, both under stricter scoring than yesterday. The
board is flashed with all of it and was running overnight.

---

## The review's most important finding

**The tests were scoring only the last ten seconds of each run.** A scenario could
report 144 bpm for a true 72 bpm — locked, quality 1.00 — and still pass, because the
error happened before the scoring window. Codex reproduced exactly that in the binary I
had been quoting "105/105 passed" from.

So yesterday's test numbers were weaker evidence than I presented them as. The tests now
check **every confident output for the whole run**, "must not lock" scenarios must not
lock even once, and lag/settling limits are asserted rather than merely printed. The
same tightening was applied to the breathing tests.

That change then exposed a real octave bug, which is fixed (below).

---

## Implemented (with the reasoning)

**Truthfulness of readings**
- An unresolved octave now caps quality *below* the lock threshold rather than halving
  it — halving could still land exactly on the threshold and lock.
- A half-period that fits the waveform better than the chosen peak is now treated as
  doubt in itself, not only when it also has strong spectral support. This is what fixed
  the 72→144 error.
- A window that is accepted but too weak to count as good now **decays an existing
  lock**. Before, marginal evidence could sustain a lock indefinitely.
- Readings expire in the viewer after five seconds and are cleared on disconnect, so a
  stale number cannot sit on screen looking current.

**Firmware bugs**
- The ROI fallback divided by the luminance-valid pixel count while summing only
  unclipped pixels — so a changing clipped fraction appeared as a brightness change.
- The 180° rotation for upside-down detection swapped the chroma bytes instead of the
  two luma samples (wrong colours, degraded detection during orientation recovery).
- The zoom trigger compared an unclamped target window, so a centred subject could never
  satisfy the size condition — "zoom on" sometimes did nothing.
- Gross-motion re-keying moved the chest/head offsets without replacing their reference
  profiles, counting their displacement twice.
- Invalid chest/head channels were serialised uninitialised (garbage in recordings).
- Exposure changes now invalidate breathing history too; previously only heart rate's.
- Exposure recovery no longer depends on having a valid ROI — a big enough lighting
  change could stranding the device permanently, which matters for unattended runs.
- Allocation and task-creation failures now stop the device with a fast-blinking LED
  instead of quietly running half-initialised.

**Tools**
- The viewer was rejecting *every* motion record (wrong token count after the `subject`
  field was added) and `sel=` lost all but the first region — so the breathing waveform
  and most region outlines were missing. Fixed.
- `--headless` no longer opens a window (it failed on machines without a display).
- The noise benchmark ignored validity, so an untracked channel scored as perfectly
  quiet and could win "quietest half". It now requires tracked, unflagged samples and
  reports coverage.
- The radar comparison scored stale values as fresh pairs; each rate now has its own
  freshness.
- The session report looked for pacer annotations inside the recording; they live in the
  `.marks.txt` sidecar.

**Honesty correction to a number I gave you**
- `tools/video_to_samples.py` still simulated the old RGB565 capture while the firmware
  moved to YUV422, so the UBFC result described a pipeline the device no longer uses.
  With true parity: **1.97 bpm mean error, locked 94%** (I had been quoting 1.82 / 100%).

---

## Not implemented — your call (ranked by how much I think they matter)

1. **Breathing presence is not the same as breathing rate.** For the baby monitor this
   is the big one. The estimator can keep reporting a rate computed from breaths that
   happened up to 30 s ago; fresh camera frames satisfy the staleness check even if
   breathing has *stopped*. A monitor needs a separate "is there breathing motion right
   now" detector with a measured detection latency. I didn't build it because it needs a
   design decision from you: how fast must it notice, and what should it do then.
2. **Fusion agreement overstates independence.** Tiles, boxes and the two axes overlap,
   so eight "agreeing regions" can be one artefact seen eight times. Grouping by physical
   region would make agreement mean what it claims. Moderate work, improves honesty of
   the quality score rather than accuracy.
3. **Camera-failure recovery and task watchdogs.** Capture currently retries forever with
   no recovery state machine, and there are no task heartbeats. Needed before anything
   runs unattended all night.
4. **Anti-aliasing costs infant-band sensitivity.** Codex calculates ≈ −8 dB at 1.3 Hz
   (78 breaths/min). Fine for adults, possibly significant for a newborn. Wants a proper
   passband/stopband specification across both frame rates.
5. **Box tracking can measure detector jitter.** A one-pixel change in the face box
   shifts the sampled profile against a reference taken at different coordinates. Would
   need the geometry held within the dead band or the shift compensated.
6. **Cross-task configuration handoff.** Settings are written from the command task while
   the camera task may be mid-sequence. No misbehaviour observed, but it is a real race;
   the clean fix is a command queue plus a settings generation stamp. Bigger refactor.
7. **Coherence is not re-checked after acquisition.** I made marginal windows decay a
   lock, which partly covers it, but renewed cross-region evidence over time would be
   better.

---

## Measured last night

- Breathing, you sitting still: **locked at 8.1–8.2 /min, quality 0.85, all regions
  agreeing** — consistent with your 9–12 resting range.
- Heart rate struggled while you were turned away: no face, so no reading. Expected.
- Zoom A/B from earlier: breathing locked 56% of windows zoomed against 4% wide.

## Waiting on you

- **Empty-room test.** Say the word when you're away from the desk and I'll check whether
  it invents readings with nobody there. That is the single most important test for the
  baby-monitor idea, and I can't run it while you're in the chair.
- OV5640 arriving Saturday: `tools/noise_bench.py` will compare it in five minutes.
- EMAY monitor: passive breathing reference, no paced breathing needed.
