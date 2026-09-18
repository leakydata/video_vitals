# Morning notes — 18 September 2026

Everything below is committed and pushed. Last night's notes are in
`docs/notes_2026-09-17.md`.

## If you only read one thing

The device no longer states numbers it cannot support. That was true of the desk case
already; overnight it became true of the cot case too, which is the one the product is
actually for, and where it had been **wrong in every single window** it reported.

Three things need you, and only you can do them:

1. **A lit empty room.** Leave a lamp on and the chair empty for twenty minutes. Every
   empty-room hour I have is dark, and darkness is a different failure mode from the one
   that matters. This is the test that would let me finish the presence detector.
2. **A daylight recording of you.** Same reason: everything I had to calibrate against was
   shot in a dim room.
3. **A decision about Codex** — it never ran, five attempts, all account or version
   rejections. Details below.

Everything else is done, measured and written down.

## Everything, verified on one clean build

Every number here was re-run against the code that is on the board right now — rebuilt
from `fullclean`, host tools recompiled from scratch — rather than accumulated across the
night as the code changed underneath them. The last re-run was after the final review fix.

| | |
|---|---|
| Heart-rate suite | **107/107** (5 seeds) |
| Breathing suite | **80/80** (5 seeds) |
| Heart rate on UBFC | **1.97 bpm MAE, 94% locked**, 100% within 5 bpm |
| Infant clips (AIR-400) | **0 confidently wrong windows** (was 93 of 93) |
| Empty room, all night | **14,577 windows, 0 false readings of either kind** |
| Faults, restarts, camera failures | **0** |
| Frame rate | 14.80 fps |

The empty-room figure is the one I care about most. Every breathing lock recorded
overnight (36 of them, all in the first two minutes) happened while your face was still
detected — you were still in the chair. From the moment the room was empty to now, the
device has reported nothing at all, which is exactly right.

---

## The empty-room test finally ran, and it passed

This is the one I most wanted and could not run while you were in the chair.

**9,192 windows with nobody in the room, across the whole night and seven firmware
builds: no heart rate reported, no breathing rate reported, not once.** No restarts, no
errors, no camera failures. The six-window lock requirement I added the night before is
holding, and so are the watchdogs.

The caveat I owe you: the room was dark for all of it. Darkness is not the hard case for
false readings — it is a *different* case, and the lit empty room is still the test that
would settle it.

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

## I reviewed my own night's work, and found two of my own bugs

Reading the whole diff back as a reviewer rather than as the author turned up two defects
I had introduced during the night. Both are fixed, both now have regression tests, and
both tests fail on the code as it was.

**A box sliding further than the fit can measure reported movement that never happened.**
When I made the chest box tolerate sliding rather than re-keying, I widened the dead band
to an eighth of the box. But the correction is a *shift*, and the fit can only measure
about three bins of shift — past that it saturates and reports whatever is left over. On a
completely frozen scene a box sliding 12 px produced **5.75 px of false displacement**:
the exact failure the correction was written to remove, moved from one place to another.
The dead band is now tied to what the fit can actually see. Same test: 0.00 px.

**A subject against the edge of the sensor could make the zoom thrash.** A clipped face
re-aims after three seconds instead of twenty, which is right when re-aiming can help. If
you are at the edge of the *sensor*, the crop cannot move further, the face stays clipped
whatever it does, and it would have re-aimed every three seconds indefinitely — a second
of lost signal each time. It now backs off after three attempts.

A second pass over the same diff found two more, both of the kind that only bite when
something else has already gone wrong:

**The watchdog measured its deadline from boot, not from when the tasks started.** The
heartbeats begin at zero, so until each task first stamped one, "twenty seconds without a
heartbeat" meant "twenty seconds since power-on". Every task does stamp well inside that
today, so this was a latent dependency rather than a live bug — but it made a slow start
indistinguishable from a hang, in the one piece of code whose whole job is telling those
apart.

**Settings were lost whenever the board reconnected.** The viewer replays its startup
commands after a reconnect, but a watch region, band or zoom chosen *later* was not among
them. The board forgets everything when it restarts — and it now restarts itself if a task
hangs — so the region you drew over the cot would quietly stop applying while the viewer
carried on drawing it. That is the sort of thing that would have looked like "the monitor
just stopped working overnight" and been very hard to explain afterwards.

A third pass checked the tests themselves, by running each new check against code broken
in the way it is supposed to catch. **Two of the three region checks did not notice** — they
passed just as happily with the grouping removed entirely, because they asserted on the
agreement *number*, and one region whose single vote agrees still scores 1.00. What
grouping actually changes is what that agreement is worth. They now assert the quality
penalty, and against ungrouped code they fail.

The rest do discriminate, each verified against deliberately broken code: box jitter, the
over-long box slide, coherence expiry, the infant band at 70/min, region grouping, and
presence — against a detector that always claims breathing, both "notices it stopping"
checks fail as they should. The streaming rewrite of `session_report.py` also produces
byte-identical output to the version it replaced, on a 15 MB recording.

One observation while I was in there, not a bug and not new: the breathing estimator takes
about **280 ms per update** when the channels are live — roughly a third of a core, once a
second. Tonight's changes did not add to it (measured on identical input: 1.19 s before,
1.17 s after), but it is the number that will limit anything more ambitious later.

## One idea I tried, measured, and threw away

My own note from earlier in this project said *top-down views need an expansion channel,
not a shift* — and that is exactly the cot geometry. A chest seen from the side rises and
falls; a chest seen from above **expands**, and its profile barely shifts at all, which a
shift-only fit cannot see however good the rest of the pipeline is. So I wrote one: a
Lucas-Kanade fit that solves for a stretch as well as an offset.

It works, and it does not pay for itself:

| clip | shift only | + expansion | + expansion (−6 dB) |
|---|---|---|---|
| S01_1 | 65% | **77%** | 71% |
| S01_2 | 71% | **29%** | 71% |
| S01_3 | 53% | 53% | 53% |
| S05_1 | 23% | 26% | 23% |

Left unranked it wins channels it should not: S01_2 is an infant lying on its *side*, where
breathing is a shift and expansion is a distractor — it outranked the good channel and took
that clip from 71% to 29%. Penalised so it only wins when clearly better, the damage goes
away and so does the benefit: 54% against 53%, all of it from one clip.

Against that, measured on the board: **14.80 fps → 13.05 fps**, a 12% frame-rate loss. Not
a trade worth making for a gain inside the noise of four clips, so it is on the branch
`experiment/expansion-channel` rather than in main, with the numbers in its commit message.

It deserves another look with footage that is actually overhead and supine — the geometry
the idea is for, and which only one of these four clips resembles.

## The two decisions that are yours

(The two recordings I need are in the list at the top.)

**1. Codex never ran.** Five attempts, all rejected before it read a line of code:
`gpt-6-astra` needs a newer Codex than the plugin's bundled one (I upgraded the global CLI
to 0.155.0; it made no difference, so the plugin ships its own), and `gpt-5.3-codex-spark`
and `gpt-5-codex` are both refused for a ChatGPT-tier account. The path that *did* work
earlier — running `codex exec` directly in a shell — is now blocked for me by Claude
Code's permission classifier, which is your decision to make, not mine: a Bash permission
rule for `codex` would unblock it. The review prompt is written and ready, and is in the repo at
`docs/codex_review_prompt.txt` so it outlives the session.

**2. The presence alarm policy**, once there is daylight data: how fast must it notice
breathing stopping, and what should it do then. The 7.1 s figure above is what it can
currently offer.

## Starting it up again

The board is running headless right now, recording to `recordings/`, so the empty-room
evidence keeps accumulating. For the window back:

    uv run heartcam.py --single --scale 1.75 --zoom

Drag a box on the video to watch a region without a face; `c` clears it; `z` toggles the
zoom; `q` quits.

## Waiting on hardware

- OV5640 on Saturday — `tools/noise_bench.py` compares it in five minutes.
- EMAY monitor — a passive breathing reference, no paced breathing needed.
