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

**This needs daylight data.** Everything I had to calibrate against was recorded in a dim
room. A recording of you in normal daylight, and one of the empty room with a lamp on,
would let me set the trust threshold from evidence instead of from the one constant the
firmware already had.

---

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
