# Morning notes — 18 September 2026

Everything below is committed and pushed. Tests: **heart rate 107/107, breathing 53/53**,
with eleven new checks written last night. The board is flashed and has been running
unattended since you went to bed. Last night's notes are now in `docs/notes_2026-09-17.md`.

---

## The empty-room test finally ran, and it passed

This is the one I most wanted and could not run while you were in the chair.

**962 windows with nobody in the room: no heart rate reported, no breathing rate
reported, not once.** No restarts, no errors, no camera failures. The six-window lock
requirement I added the night before is holding.

---

## Your face is no longer cut off when it zooms

Two separate causes, both fixed, and this time **measured over 85 frames** rather than
eyeballed: the face box is wholly inside the frame in every frame, with 0.40 face-heights
of headroom, and the chest box stays in view.

1. It aimed from a **stale detection** — after the window moves, the last known face
   position describes the *old* view, so the aim was computed in the wrong frame of
   reference and walked the crop off you.
2. The crop was sized from face **width**, so a turned head (narrow box) tightened it at
   exactly the wrong moment. It is now sized from what has to fit, driven by face height,
   which barely changes when you turn.

A face touching the edge also re-aims after 3 s now instead of waiting out the 20 s rate
limit.

---

## All five remaining review items are done

Each one is a correctness fix, and each came with a measurement showing it mattered.

**Detector jitter was being read as breathing.** The chest box is anchored to the face
box, and the detector's idea of where your face is wobbles a pixel or two between frames.
On a **completely frozen scene**, that wobble produced **2.16 px** of apparent motion —
*more* than real 2 px movement produces. The box's displacement is known exactly, so it is
now subtracted; the same test reports 0.23 px.

**Agreement between regions was being overcounted.** The two axes of one tile, and a box
together with the tiles beneath it, all watch the same piece of you. One artefact seen
four ways scored as four agreeing witnesses. Regions that overlap now merge and cast one
vote between them.

**The anti-alias filter was leaning on the infant band.** Six one-poles at 2 Hz cost about
8 dB at 78 breaths/min, which biased every comparison toward the slower candidate. Its
response is now divided back out of the spectrum. At 70/min over five seeds: it used to
lock on three and fail completely on two; now all five lock, within 0.05 /min.

**A heart-rate lock never had to re-earn its evidence.** Coherence across the face regions
is what separates a pulse from anything else periodic, and it was checked only while
acquiring — after four good windows it stopped mattering for good. A lock could then be
carried indefinitely by any steady rhythm in one region. It may now ride out ten windows
without coherent evidence, not more.

**The command task was writing to the sensor from under a running capture.** Exposure and
gain went straight to the camera's registers while the camera task was capturing,
windowing or flipping it. Those now follow the pattern rotation already used: leave a
request, and the task that owns the hardware applies it between frames. Verified live on
the board (e600 and g12 both read back correctly).

---

## The presence detector did not work, and I could not make it work

This is the important one, and it is bad news honestly reported.

I built breathing *presence* the night before — "is anything breathing right now",
separate from the rate. I measured it last night against real recordings:

| | breathing reported |
|---|---|
| Empty, dark room | **99% of windows** |
| You in the chair | 100% of windows |

It was not detecting breathing. It was detecting that the image was moving.

The cause is that **in darkness the tile tracker does not merely get noisy, it goes
wrong**: the strongest region swings by about **24 px** over eight seconds with nobody in
the room. I tried three ways to tell that apart from breathing and measured each:

- require neighbouring regions to move together → 99% empty / 87% occupied (in the dark
  the false movement is globally correlated too)
- remove a straight-line drift first → 98% / 84% (it is not drift)
- require it to swing up and down the bounded number of times breathing does → 99% / 87%
  (it does that as well)

None of them separated the two. So presence now reports **`unknown`** below the brightness
the firmware already treats as unmeasurable, which is confirmed working on the board. For
a monitor, "I cannot tell" has to be distinguishable from "not breathing" — a false
*"breathing"* is precisely the failure that would hide a real stoppage.

What does work, measured with a new test: when the light is adequate, presence appears
**0.6 s** after breathing starts and disappears **7.1 s** after it stops, including for
0.12 px movement — an infant, or a chest under a blanket. Those numbers are the
specification for whatever alarm policy you decide on.

**I tried to run the lit empty-room test for you and could not.** I put a white
full-screen window on the monitors to use them as a lamp; the camera still read
brightness 3 of 255, because it faces the chair and away from the screens. So that test
still needs a real lamp and a hand to switch it on.

**This needs daylight data.** Everything I had to calibrate against was recorded in a dim
room. A recording of you in normal daylight, and one of the empty room with a lamp on,
would let me set the trust threshold from evidence instead of from the one constant the
firmware already had.

---

## It was reporting an infant's wriggling as its breathing rate

There is an annotated infant dataset sitting in `data/air400_infant_breathing` that we had
never actually evaluated against. I did, and it is the most important thing I found.

**On four real infant clips the device locked in 93 windows, and every single one was
wrong by more than 5 /min** — typically reporting **8.4 /min against an annotated 19**,
with quality 0.78, stability 1.00 and every region agreeing. That is the failure mode we
care about most, on exactly the subject the product is for.

The estimator was not mistaking the rate. I measured the video itself: the strongest
rhythms in these clips are at **4–8 /min — an infant wriggling — carrying about ten times
the energy of the breathing at 19 /min.** The device faithfully reported the strongest
rhythm it could see. What was wrong is that the infant band let it: the band started at
the *adult's* 6 /min and its filter passed everything above 6 /min, so a wriggle at 8 was
in band and swamped the breathing above it.

The infant band now starts at 15 /min, in both the peak search and the filter. Fifteen is
still well below anything clinically slow for an infant, so genuinely slow breathing is
reported rather than hidden.

**Confidently wrong windows: 93 before, 0 now.**

It does not yet find 19 /min on these clips either — the peak moved 8.4 → 24 and the SNR
stays negative, so it abstains. Silence is the right answer meanwhile, but finding
breathing underneath a wriggling infant is the next real problem, and I think it is a
front-end one: the replay has no face detector, so it is running on the tile grid alone
with no chest box, which is not how the device works in the room. Trying it with a chest
box is the first thing to do.

Nothing else regressed: heart rate on UBFC is byte-identical at **1.97 bpm MAE, 94%
locked**, and the breathing suite passes **79/79** over five seeds.

### Then I found why it was that bad, and it is architectural

The breathing regions are anchored to a **detected face** — the chest box is placed under
it. On the cot footage the face detector found **nothing in 201 frames**. An infant lying
down, seen from above in infrared, is not a face as far as the detector is concerned. No
face means no chest box, no head box, and nothing but the fixed tile grid — which is
exactly the configuration that was reading the wriggle.

So I gave it a chest box by hand over the infant's torso, and the same firmware pipeline
read the rate **exactly right: 19.3–19.6 /min against an annotated 19.0–19.4**, with the
chest box as the best channel and every region agreeing.

It still will not *report* it — the per-window SNR is only 2–4 dB even on the best channel,
below the confidence gate. But it is now finding the right answer instead of a wrong one,
and that is a completely different problem to solve.

**What I built from that:** you can now drag a box over the video and the device watches
there whenever it has no face (`c` clears it). Pointing the camera at the cot once is what
a parent would do anyway. Verified working on the board.

**Measured across all four clips.** The evaluator now reports the peak the estimator
*found* separately from what it was confident enough to state, because on this footage
that distinction is the whole story. Peak within 2 /min of the annotated rate:

| clip | no region | watch region |
|---|---|---|
| S01_1 | 55% | 65% |
| S01_2 | **0%** | **71%** |
| S01_3 | 23% | 53% |
| S05_1 | 6% | 23% |

About a fifth of windows to about a half, just from pointing it at the torso. Regions were
placed by eye from one frame of each clip.

**What I did not do:** lower the confidence gate so it reports these marginal readings.
That is the change that would make AIR-400 "work", and it is exactly the change that
risks bringing back false readings — and I cannot validate it safely, because the only
empty-room data I have is dark, where the tracker is unreliable for other reasons. It
needs the lit empty-room recording. I am not loosening a safety threshold on evidence I
do not have.

## Two things need you

**1. Codex never ran.** Five attempts, all rejected before it read a line of code:
`gpt-6-astra` needs a newer Codex than the plugin's bundled one (I upgraded the global CLI
to 0.155.0; it made no difference, so the plugin ships its own), and `gpt-5.3-codex-spark`
and `gpt-5-codex` are both refused for a ChatGPT-tier account. The path that *did* work
earlier — running `codex exec` directly in a shell — is now blocked for me by Claude
Code's permission classifier, which is your decision to make, not mine: a Bash permission
rule for `codex` would unblock it. The review prompt is written and ready at
`scratchpad/review3_prompt.txt`.

**2. The presence alarm policy**, once there is daylight data: how fast must it notice
breathing stopping, and what should it do then. The 7.1 s figure above is what it can
currently offer.

## Waiting on hardware

- OV5640 on Saturday — `tools/noise_bench.py` compares it in five minutes.
- EMAY monitor — a passive breathing reference, no paced breathing needed.
